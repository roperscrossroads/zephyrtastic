/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the cluster document (src/meshtastic_cluster_doc.c): the
 * sorted table, the LWW merge rule, the digest hash, and — the reason the
 * design exists — the anti-shadowing effective() semantics of
 * docs/CLUSTER-SYNC-M4.md §2. Pure logic, no radio, no settings.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "meshtastic_cluster_doc.h"
#include "meshtastic_cluster_key.h"

/* meshtastic_hlc.c's convenience wrappers reference the node clock; these
 * tests drive stamps explicitly and never call them, so a stub satisfies the
 * link without dragging the clock subsystem in. */
int64_t meshtastic_clock_now_epoch_ms(void)
{
	return 0;
}

#define CAP 16
static struct meshtastic_cluster_entry storage[CAP];
static struct meshtastic_cluster_doc doc;

#define SEC_DEVICE 12U /* arbitrary section tags — the doc never interprets them */
#define SEC_DISPLAY 16U
#define NODE_A 0xAAAAAAAAU
#define NODE_B 0xBBBBBBBBU

static struct meshtastic_hlc_stamp at(int64_t ms, uint32_t author)
{
	return (struct meshtastic_hlc_stamp){.physical_ms = ms, .counter = 0U, .node_id = author};
}

static struct meshtastic_cluster_key base_key(uint16_t sec)
{
	return (struct meshtastic_cluster_key){
		.layer = MESHTASTIC_CLUSTER_LAYER_BASE, .node_id = 0U, .section = sec};
}

static struct meshtastic_cluster_key node_key(uint32_t node, uint16_t sec)
{
	return (struct meshtastic_cluster_key){
		.layer = MESHTASTIC_CLUSTER_LAYER_NODE, .node_id = node, .section = sec};
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_cluster_doc_init(&doc, storage, CAP);
}

ZTEST_SUITE(cluster_doc, NULL, NULL, before, NULL, NULL);

ZTEST(cluster_doc, test_lww_newer_wins_stale_ignored)
{
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_hlc_stamp s100 = at(100, NODE_A);
	struct meshtastic_hlc_stamp s200 = at(200, NODE_B);
	const struct meshtastic_cluster_entry *e;
	uint8_t v1[] = {1};
	uint8_t v2[] = {2};

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s100, false, v1, 1), 1);
	/* Stale write bounces without touching the stored value... */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s100, false, v2, 1), 0);
	e = meshtastic_cluster_doc_find(&doc, &k);
	zassert_equal(e->payload[0], 1);
	/* ...a newer one replaces it. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s200, false, v2, 1), 1);
	e = meshtastic_cluster_doc_find(&doc, &k);
	zassert_equal(e->payload[0], 2);
	zassert_equal(doc.count, 1);
}

ZTEST(cluster_doc, test_malformed_refused)
{
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	struct meshtastic_hlc_stamp unset = {0};
	uint8_t v[] = {1};

	/* Unversioned writes can never win, so they never enter. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &unset, false, v, 1), -EINVAL);
	/* BASE never tombstones and has no owner. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, true, NULL, 0), -EINVAL);
	b.node_id = NODE_A;
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, false, v, 1), -EINVAL);
	/* NODE needs an owner; tombstones carry no payload; values carry one. */
	n.node_id = 0U;
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, v, 1), -EINVAL);
	n.node_id = NODE_A;
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, v, 1), -EINVAL);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, NULL, 0), -EINVAL);
	zassert_equal(doc.count, 0);
}

/* THE test — why base ⊕ nodes/<id> exists at all (register §7.7): a pin never
 * blocks base replication; it only wins at apply time, so unpin lands on the
 * CURRENT base, not the one frozen when the pin was made. A single-stamp
 * regression (override and base competing for one key) fails here. */
ZTEST(cluster_doc, test_pin_never_shadows_base_replication)
{
	struct meshtastic_cluster_key b = base_key(SEC_DISPLAY);
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_hlc_stamp s;
	const struct meshtastic_cluster_entry *e;
	uint8_t base_v1[] = {10};
	uint8_t pin_v[] = {77};
	uint8_t base_v2[] = {20};

	/* Fleet base arrives; effective(A) follows it. */
	s = at(100, NODE_B);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, false, base_v1, 1), 1);
	e = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_equal(e->payload[0], 10);

	/* A pins its own value; effective(A) is now the pin. */
	s = at(150, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, pin_v, 1), 1);
	e = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_equal(e->payload[0], 77);

	/* A NEWER base arrives while the pin stands. It must be STORED —
	 * replication is never blocked — while effective(A) keeps the pin. */
	s = at(200, NODE_B);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, false, base_v2, 1), 1,
		      "a pin must never block base replication");
	e = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_equal(e->payload[0], 77, "the pin still wins at apply time");
	/* Another node with no pin already follows the new base. */
	e = meshtastic_cluster_doc_effective(&doc, NODE_B, SEC_DISPLAY);
	zassert_equal(e->payload[0], 20);

	/* Unpin = tombstone. Effective(A) lands on the CURRENT base — the
	 * value that arrived DURING the pin, which a shadowing design loses. */
	s = at(250, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, NULL, 0), 1);
	e = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_not_null(e);
	zassert_equal(e->payload[0], 20, "unpin must land on the base updated during the pin");
}

/*
 * THE OTHER HALF of the test above — and the half that decides whether `unpin`
 * does anything at all on a real node.
 *
 * effective() alone answers "which value", and the test above proves that
 * answer is right. But the reconciler also has to hand the config store a
 * VERSION, and the store runs its own last-writer-wins merge: too old a version
 * and the write is declined. Score a reversion by the base entry's own stamp and
 * that is exactly what happens — the pin the store is running was minted AFTER
 * the base, so the base loses, the write bounces, and the node keeps running the
 * value it was just told to stop running. The document would be correct and the
 * radio would be wrong.
 *
 * The tombstone is the write that fixes it: it is minted after the pin, and
 * counting it in the version is what makes "unpin reverts to the current base"
 * true rather than aspirational.
 */
ZTEST(cluster_doc, test_effective_version_carries_the_reversion)
{
	struct meshtastic_cluster_key b = base_key(SEC_DISPLAY);
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_hlc_stamp s;
	struct meshtastic_hlc_stamp v;
	const struct meshtastic_cluster_entry *e;
	uint8_t base_v[] = {10};
	uint8_t pin_v[] = {77};

	/* Nothing at either layer: no version to report. */
	zassert_false(meshtastic_cluster_doc_effective_version(&doc, NODE_A, SEC_DISPLAY, &v),
		      "a section the document holds no opinion about has no version");

	/* Base alone — the version is the base's own stamp. */
	s = at(100, NODE_B);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, false, base_v, 1), 1);
	zassert_true(meshtastic_cluster_doc_effective_version(&doc, NODE_A, SEC_DISPLAY, &v));
	zassert_equal(meshtastic_hlc_compare(&v, &s), 0);

	/* A pin minted later — the version follows the value. */
	s = at(150, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, pin_v, 1), 1);
	zassert_true(meshtastic_cluster_doc_effective_version(&doc, NODE_A, SEC_DISPLAY, &v));
	zassert_equal(meshtastic_hlc_compare(&v, &s), 0, "a live pin decides, and dates, the "
		      "answer");

	/*
	 * THE CASE THAT MATTERS. Unpin with the base UNCHANGED underneath — the
	 * ordinary "I changed my mind" — so the base entry's own stamp (100) is
	 * OLDER than the pin (150) the store is running. The version must be the
	 * tombstone's (250), or the store declines the reversion.
	 */
	s = at(250, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, NULL, 0), 1);
	e = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_not_null(e);
	zassert_equal(e->payload[0], 10, "the value reverts to base");
	zassert_true(meshtastic_cluster_doc_effective_version(&doc, NODE_A, SEC_DISPLAY, &v));
	zassert_equal(meshtastic_hlc_compare(&v, &s), 0,
		      "the reversion must be dated by the TOMBSTONE, not by the older base it "
		      "falls back to — otherwise the store's own LWW merge declines the write "
		      "and the node never actually comes off the pin");

	/* And a base promoted AFTER the unpin still gets through: the version is
	 * the newest of the pair, so an old tombstone cannot freeze the section. */
	s = at(300, NODE_B);
	base_v[0] = 20;
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s, false, base_v, 1), 1);
	zassert_true(meshtastic_cluster_doc_effective_version(&doc, NODE_A, SEC_DISPLAY, &v));
	zassert_equal(meshtastic_hlc_compare(&v, &s), 0,
		      "a tombstone must not date the section forever");
}

ZTEST(cluster_doc, test_effective_fallbacks)
{
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	uint8_t v[] = {5};

	/* Nothing anywhere: NULL. */
	zassert_is_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DEVICE));

	/* Only a pin, no base: the pin serves. */
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DEVICE);

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, v, 1), 1);
	zassert_not_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DEVICE));
	/* ...but only for its owner. */
	zassert_is_null(meshtastic_cluster_doc_effective(&doc, NODE_B, SEC_DEVICE));

	/* Tombstone with no base underneath: NULL again, not the tombstone. */
	s = at(200, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, NULL, 0), 1);
	zassert_is_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DEVICE));
}

/* Two docs that hold the same rows hash identically no matter the arrival
 * order — the digest compares state, not history. */
ZTEST(cluster_doc, test_hash_is_arrival_order_independent)
{
	static struct meshtastic_cluster_entry storage2[CAP];
	struct meshtastic_cluster_doc doc2;
	struct meshtastic_cluster_key kb = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key kn = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_hlc_stamp s1 = at(100, NODE_A);
	struct meshtastic_hlc_stamp s2 = at(200, NODE_B);
	uint8_t v[] = {1};

	meshtastic_cluster_doc_init(&doc2, storage2, CAP);

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &kb, &s1, false, v, 1), 1);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &kn, &s2, false, v, 1), 1);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &kn, &s2, false, v, 1), 1);
	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &kb, &s1, false, v, 1), 1);

	zassert_equal(meshtastic_cluster_doc_hash(&doc), meshtastic_cluster_doc_hash(&doc2));
	zassert_not_equal(meshtastic_cluster_doc_hash(&doc), 0U);

	/* And any difference — here a tombstone flip — changes the hash. */
	s2 = at(300, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &kn, &s2, true, NULL, 0), 1);
	zassert_not_equal(meshtastic_cluster_doc_hash(&doc), meshtastic_cluster_doc_hash(&doc2));
}

/*
 * The anti-entropy diff (§3.3). What a node asks for after reading a peer's
 * stamp vector is exactly the rows this returns true for — and, just as
 * load-bearing, what it does NOT ask for is everything else. A row where OUR
 * copy is newer must not provoke a fetch: the walk would then pull an old value
 * over a new one, and the two nodes would trade the same key back and forth
 * forever. Nobody pushes; the node that is ahead merely lets its next digest
 * invite the other to pull.
 */
ZTEST(cluster_doc, test_wants_only_what_is_newer)
{
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_hlc_stamp s100 = at(100, NODE_A);
	struct meshtastic_hlc_stamp s200 = at(200, NODE_B);
	struct meshtastic_hlc_stamp unset = {0};
	uint8_t v[] = {1};

	/* A key we have never seen is always worth pulling. */
	zassert_true(meshtastic_cluster_doc_wants(&doc, &k, &s100));
	zassert_true(meshtastic_cluster_doc_wants(&doc, &n, &s100));

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s200, false, v, 1), 1);

	/* Older: leave it. Identical: leave it (this is the converged case, the
	 * one that must cost nothing). Newer: pull it. */
	zassert_false(meshtastic_cluster_doc_wants(&doc, &k, &s100), "an older row must not be "
								     "fetched over a newer one");
	zassert_false(meshtastic_cluster_doc_wants(&doc, &k, &s200), "an identical row is not a "
								     "difference");
	struct meshtastic_hlc_stamp s300 = at(300, NODE_A);

	zassert_true(meshtastic_cluster_doc_wants(&doc, &k, &s300));

	/* Rows accept() would refuse are not worth a round trip either: the
	 * two predicates share one validity rule so they cannot disagree. */
	struct meshtastic_cluster_key bad_base = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key bad_node = node_key(0U, SEC_DEVICE);

	bad_base.node_id = NODE_A; /* BASE has no owner */
	zassert_false(meshtastic_cluster_doc_wants(&doc, &bad_base, &s300));
	zassert_false(meshtastic_cluster_doc_wants(&doc, &bad_node, &s300));
	zassert_false(meshtastic_cluster_doc_wants(&doc, &k, &unset),
		      "an unversioned row can never win, so never fetch it");
}

ZTEST(cluster_doc, test_max_stamp_and_capacity)
{
	struct meshtastic_hlc_stamp max;
	struct meshtastic_hlc_stamp s;
	uint8_t v[] = {1};

	meshtastic_cluster_doc_max_stamp(&doc, &max);
	zassert_true(meshtastic_hlc_stamp_is_unset(&max), "empty doc has the unset max");

	for (uint16_t i = 0U; i < CAP; i++) {
		struct meshtastic_cluster_key k = base_key(i);

		s = at(100 + i, NODE_A);
		zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s, false, v, 1), 1);
	}
	meshtastic_cluster_doc_max_stamp(&doc, &max);
	zassert_equal(max.physical_ms, 100 + CAP - 1);

	/* Full: a NEW key is refused loudly; a newer stamp on an EXISTING key
	 * still lands (replacement needs no free slot). */
	struct meshtastic_cluster_key overflow = base_key(1000);

	s = at(999, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &overflow, &s, false, v, 1), -ENOSPC);
	struct meshtastic_cluster_key existing = base_key(0);

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &existing, &s, false, v, 1), 1);
}

/* ==========================================================================
 * ADVERSARIAL SET — written to BREAK the document, not to confirm it works.
 *
 * The happy-path tests above prove the merge rule. These probe the edges an
 * attacker or a buggy peer actually reaches: a full table, hostile key
 * orderings, payload boundaries, and tombstone/value races. Where one of these
 * documents a REAL weakness rather than a defended one, it says so in the
 * assertion message — a test that pins bad behaviour is only honest if it
 * admits that is what it is doing.
 * ========================================================================== */

/*
 * THE ENOSPC TRAP, closed.
 *
 * A full table refuses new keys. The diff predicate has to know that, because
 * if it does not, the walk asks for a key every round, is answered, fails to
 * store it, and asks again — forever, at one exchange per digest period, with
 * nothing ever changing. Wanting what cannot be stored is not optimism, it is
 * an unbounded request loop.
 *
 * The bound has to be here rather than in the module, because this is where
 * "would accept() take it?" is actually knowable.
 */
ZTEST(cluster_doc, test_full_table_stops_asking_for_what_it_cannot_store)
{
	struct meshtastic_hlc_stamp s;
	uint8_t v[] = {1};

	for (uint16_t i = 0U; i < CAP; i++) {
		struct meshtastic_cluster_key k = base_key(i);

		s = at(100 + i, NODE_A);
		zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s, false, v, 1), 1);
	}

	struct meshtastic_cluster_key unseen = base_key(999);

	s = at(500, NODE_B);
	zassert_false(meshtastic_cluster_doc_wants(&doc, &unseen, &s),
		      "a full table must stop asking for a key it can only refuse");
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &unseen, &s, false, v, 1), -ENOSPC,
		      "and accept() still says why, for the node that offers it anyway");

	/* But an UPDATE to a key already held needs no free slot, so a full
	 * table must still track the fleet on everything it already knows —
	 * otherwise "full" would silently mean "frozen". */
	struct meshtastic_cluster_key held = base_key(0);

	s = at(9999, NODE_B);
	zassert_true(meshtastic_cluster_doc_wants(&doc, &held, &s),
		     "capacity must not stop us updating a key we already hold");
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &held, &s, false, v, 1), 1);

	/* And a row we already have at the same or a newer stamp is still not
	 * wanted — capacity is an extra bound, not a replacement for LWW. */
	s = at(100, NODE_A);
	zassert_false(meshtastic_cluster_doc_wants(&doc, &held, &s));
}

/* Sorted insert under hostile orderings. The digest hashes rows in index order,
 * so two nodes that received the same rows in different orders MUST end up with
 * the same index order or they will never agree — and the walk resumes by
 * index, so a mis-ordered table also mis-resumes. */
ZTEST(cluster_doc, test_sort_order_survives_hostile_insert_sequences)
{
	static struct meshtastic_cluster_entry storage2[CAP];
	struct meshtastic_cluster_doc doc2;
	struct meshtastic_cluster_key keys[] = {
		node_key(NODE_B, SEC_DEVICE), base_key(SEC_DISPLAY),
		node_key(NODE_A, SEC_DISPLAY), base_key(SEC_DEVICE),
		node_key(NODE_B, SEC_DISPLAY), node_key(NODE_A, SEC_DEVICE),
	};
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	uint8_t v[] = {7};

	meshtastic_cluster_doc_init(&doc2, storage2, CAP);

	/* Forwards into one, backwards into the other. */
	for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
		zassert_equal(meshtastic_cluster_doc_accept(&doc, &keys[i], &s, false, v, 1), 1);
	}
	for (size_t i = ARRAY_SIZE(keys); i > 0; i--) {
		zassert_equal(meshtastic_cluster_doc_accept(&doc2, &keys[i - 1], &s, false, v, 1),
			      1);
	}

	zassert_equal(doc.count, doc2.count);
	for (uint16_t i = 0U; i < doc.count; i++) {
		zassert_equal(meshtastic_cluster_key_cmp(&doc.entries[i].key,
							 &doc2.entries[i].key),
			      0, "row %u differs — insert order leaked into the table", i);
		if (i > 0U) {
			zassert_true(meshtastic_cluster_key_cmp(&doc.entries[i - 1].key,
								&doc.entries[i].key) < 0,
				     "table is not strictly ascending at row %u", i);
		}
	}
	zassert_equal(meshtastic_cluster_doc_hash(&doc), meshtastic_cluster_doc_hash(&doc2));
}

/* The digest hashes (key, stamp, tombstone) — so anything that changes a key
 * must change the hash, or two different documents advertise as identical and
 * the fleet silently stops converging. Probe each key field independently. */
ZTEST(cluster_doc, test_hash_is_sensitive_to_every_key_field)
{
	static struct meshtastic_cluster_entry storage2[CAP];
	struct meshtastic_cluster_doc doc2;
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	uint8_t v[] = {1};
	struct meshtastic_cluster_key a = node_key(NODE_A, SEC_DEVICE);
	uint32_t base_hash;

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &a, &s, false, v, 1), 1);
	base_hash = meshtastic_cluster_doc_hash(&doc);

	/* Same section + node, BASE layer instead of NODE. */
	meshtastic_cluster_doc_init(&doc2, storage2, CAP);
	struct meshtastic_cluster_key layer_flipped = base_key(SEC_DEVICE);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &layer_flipped, &s, false, v, 1), 1);
	zassert_not_equal(base_hash, meshtastic_cluster_doc_hash(&doc2), "layer must hash");

	/* Same layer + section, different owner. */
	meshtastic_cluster_doc_init(&doc2, storage2, CAP);
	struct meshtastic_cluster_key owner_flipped = node_key(NODE_B, SEC_DEVICE);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &owner_flipped, &s, false, v, 1), 1);
	zassert_not_equal(base_hash, meshtastic_cluster_doc_hash(&doc2), "node_id must hash");

	/* Same key, different section. */
	meshtastic_cluster_doc_init(&doc2, storage2, CAP);
	struct meshtastic_cluster_key sec_flipped = node_key(NODE_A, SEC_DISPLAY);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &sec_flipped, &s, false, v, 1), 1);
	zassert_not_equal(base_hash, meshtastic_cluster_doc_hash(&doc2), "section must hash");

	/* Same key, stamp differing only in the author. Two nodes writing in the
	 * same millisecond with the same counter are separated by node_id alone —
	 * if that does not reach the hash, they agree while holding different
	 * values. */
	meshtastic_cluster_doc_init(&doc2, storage2, CAP);
	struct meshtastic_hlc_stamp other_author = at(100, NODE_B);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &a, &other_author, false, v, 1), 1);
	zassert_not_equal(base_hash, meshtastic_cluster_doc_hash(&doc2),
			  "stamp author must hash");

	/* Payloads deliberately do NOT hash (a byte-identical re-encode must not
	 * look like divergence) — pin that, because it is a choice, not an
	 * oversight, and it means the digest cannot detect a payload that
	 * differs under an identical stamp. */
	meshtastic_cluster_doc_init(&doc2, storage2, CAP);
	uint8_t other_v[] = {0xFF};

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &a, &s, false, other_v, 1), 1);
	zassert_equal(base_hash, meshtastic_cluster_doc_hash(&doc2),
		      "payload must NOT hash — the stamp is the version (by design)");
}

ZTEST(cluster_doc, test_payload_length_boundary)
{
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	static uint8_t big[MESHTASTIC_CLUSTER_PAYLOAD_MAX + 1];

	memset(big, 0xA5, sizeof(big));

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s, false, big,
						    MESHTASTIC_CLUSTER_PAYLOAD_MAX),
		      1, "exactly the cap must fit");
	zassert_equal(meshtastic_cluster_doc_find(&doc, &k)->payload_len,
		      MESHTASTIC_CLUSTER_PAYLOAD_MAX);

	s = at(200, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s, false, big, sizeof(big)),
		      -EINVAL, "one over the cap must be refused, not truncated");
	/* And the refusal must leave the stored entry untouched — a rejected
	 * write that half-applies is worse than one that never arrived. */
	zassert_equal(meshtastic_cluster_doc_find(&doc, &k)->stamp.physical_ms, 100);
}

/* Tombstone races. A tombstone is an ordinary versioned write, so an older
 * value must lose to it and a newer value must resurrect over it. Getting this
 * backwards would make unpin either un-undoable or useless. */
ZTEST(cluster_doc, test_tombstone_is_just_another_version)
{
	struct meshtastic_cluster_key n = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_hlc_stamp s;
	uint8_t v[] = {42};

	s = at(200, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, NULL, 0), 1);

	/* An older value arriving late must not undo the tombstone. */
	s = at(100, NODE_B);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, v, 1), 0,
		      "a stale value must not resurrect over a newer tombstone");
	zassert_true(meshtastic_cluster_doc_find(&doc, &n)->tombstone);

	/* A newer one must. */
	s = at(300, NODE_B);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, false, v, 1), 1);
	zassert_false(meshtastic_cluster_doc_find(&doc, &n)->tombstone);
	zassert_equal(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY)->payload[0], 42);

	/* And a tombstone can be re-applied over it. */
	s = at(400, NODE_A);
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &s, true, NULL, 0), 1);
	zassert_is_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY));
}

/* An equal stamp from a DIFFERENT author is not equal — the total order breaks
 * the tie on node_id — so two nodes writing in the same millisecond still pick
 * the same winner independently. If this ever returned 0-and-keep, a fleet
 * could split permanently on a simultaneous write. */
ZTEST(cluster_doc, test_simultaneous_writers_break_the_tie_the_same_way)
{
	static struct meshtastic_cluster_entry storage2[CAP];
	struct meshtastic_cluster_doc doc2;
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_hlc_stamp from_a = {.physical_ms = 100, .counter = 7, .node_id = NODE_A};
	struct meshtastic_hlc_stamp from_b = {.physical_ms = 100, .counter = 7, .node_id = NODE_B};
	uint8_t va[] = {0xAA};
	uint8_t vb[] = {0xBB};

	meshtastic_cluster_doc_init(&doc2, storage2, CAP);

	/* Node 1 sees A then B; node 2 sees B then A. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &from_a, false, va, 1), 1);
	(void)meshtastic_cluster_doc_accept(&doc, &k, &from_b, false, vb, 1);

	zassert_equal(meshtastic_cluster_doc_accept(&doc2, &k, &from_b, false, vb, 1), 1);
	(void)meshtastic_cluster_doc_accept(&doc2, &k, &from_a, false, va, 1);

	zassert_equal(meshtastic_cluster_doc_find(&doc, &k)->payload[0],
		      meshtastic_cluster_doc_find(&doc2, &k)->payload[0],
		      "a simultaneous write must resolve identically on both nodes");
	zassert_equal(meshtastic_cluster_doc_hash(&doc), meshtastic_cluster_doc_hash(&doc2));
}

/* ---- scopes (agents-xhli.10) ---------------------------------------------- */

static struct meshtastic_cluster_scope scope_of(uint8_t kind, uint32_t owner)
{
	struct meshtastic_cluster_scope s;

	meshtastic_cluster_scope_make(&s, kind, owner);
	return s;
}

static void put(const struct meshtastic_cluster_key *k, int64_t ms, uint32_t author)
{
	uint8_t v[] = {0x11};

	zassert_equal(meshtastic_cluster_doc_accept(&doc, k, &(struct meshtastic_hlc_stamp){
							     .physical_ms = ms,
							     .counter = 0U,
							     .node_id = author},
						    false, v, 1),
		      1, "fixture write must land");
}

/*
 * Membership is one function, and everything else in the design leans on it:
 * the hash, the count, the max stamp, the diff predicate, the ingest gate, the
 * vector filter and the persistence load filter all ask this and nothing else.
 * If any two of them disagreed, a node would advertise a document it does not
 * hold — the exact failure a declared scope exists to prevent.
 */
ZTEST(cluster_doc, test_scope_contains_is_the_only_membership_rule)
{
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DEVICE);
	struct meshtastic_cluster_key theirs = node_key(NODE_B, SEC_DEVICE);
	struct meshtastic_cluster_scope full = scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	struct meshtastic_cluster_scope basen = scope_of(MESHTASTIC_CLUSTER_SCOPE_BASE, 0U);

	zassert_true(meshtastic_cluster_scope_contains(&full, &b));
	zassert_true(meshtastic_cluster_scope_contains(&full, &mine));
	zassert_true(meshtastic_cluster_scope_contains(&full, &theirs));

	/* Base is inside EVERY scope. That is what makes it the one leg any two
	 * nodes can compare whatever tier the other is on. */
	zassert_true(meshtastic_cluster_scope_contains(&core, &b));
	zassert_true(meshtastic_cluster_scope_contains(&basen, &b));

	zassert_true(meshtastic_cluster_scope_contains(&core, &mine),
		     "CORE(x) must keep x's own entries — they are what effective() reads");
	zassert_false(meshtastic_cluster_scope_contains(&core, &theirs),
		      "and must NOT claim another node's entries");
	zassert_false(meshtastic_cluster_scope_contains(&basen, &mine));

	/* An owner outside CORE is meaningless and must not make two equal
	 * scopes compare unequal. */
	zassert_equal(scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, NODE_A).owner, 0U);
}

ZTEST(cluster_doc, test_scope_intersection_is_the_smaller_of_two)
{
	struct meshtastic_cluster_scope full = scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
	struct meshtastic_cluster_scope core_a = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	struct meshtastic_cluster_scope core_b = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_B);
	struct meshtastic_cluster_scope out;

	meshtastic_cluster_scope_intersect(&full, &full, &out);
	zassert_equal(out.kind, MESHTASTIC_CLUSTER_SCOPE_FULL);

	meshtastic_cluster_scope_intersect(&full, &core_a, &out);
	zassert_equal(out.kind, MESHTASTIC_CLUSTER_SCOPE_CORE);
	zassert_equal(out.owner, NODE_A,
		      "a FULL node can compare a CORE peer over that peer's whole claim");

	meshtastic_cluster_scope_intersect(&core_a, &core_a, &out);
	zassert_equal(out.kind, MESHTASTIC_CLUSTER_SCOPE_CORE);

	/* Two different constrained nodes share only base — which is why the
	 * base leg is not an optimisation but the thing that keeps an all-CORE
	 * fleet converging at all. */
	meshtastic_cluster_scope_intersect(&core_a, &core_b, &out);
	zassert_equal(out.kind, MESHTASTIC_CLUSTER_SCOPE_BASE,
		      "CORE(x) and CORE(y) have only base in common");
}

/*
 * THE BACK-COMPATIBILITY CLAIM. A fleet where nobody has narrowed its claim
 * must hash exactly as it did before scopes existed — otherwise shipping this
 * would look, to every node, like the whole fleet diverging at once.
 */
ZTEST(cluster_doc, test_scoped_hash_equals_the_whole_doc_hash_at_full_scope)
{
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_cluster_scope full = scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
	struct meshtastic_hlc_stamp max_a, max_b;

	put(&b, 100, NODE_A);
	put(&mine, 200, NODE_A);

	zassert_equal(meshtastic_cluster_doc_hash_scoped(&doc, &full),
		      meshtastic_cluster_doc_hash(&doc));
	zassert_equal(meshtastic_cluster_doc_count_scoped(&doc, &full), doc.count);
	meshtastic_cluster_doc_max_stamp_scoped(&doc, &full, &max_a);
	meshtastic_cluster_doc_max_stamp(&doc, &max_b);
	zassert_equal(meshtastic_hlc_compare(&max_a, &max_b), 0);
}

ZTEST(cluster_doc, test_scoped_hash_ignores_everything_outside_the_scope)
{
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_cluster_key theirs = node_key(NODE_B, SEC_DISPLAY);
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	uint32_t hash_before;
	uint16_t count_before;
	struct meshtastic_hlc_stamp max_before, max_after;

	put(&b, 100, NODE_A);
	put(&mine, 200, NODE_A);
	hash_before = meshtastic_cluster_doc_hash_scoped(&doc, &core);
	count_before = meshtastic_cluster_doc_count_scoped(&doc, &core);
	meshtastic_cluster_doc_max_stamp_scoped(&doc, &core, &max_before);

	/* A row this scope does not claim, and a LATER one so that a leg which
	 * wrongly included it could not possibly go unnoticed. */
	put(&theirs, 900, NODE_B);

	zassert_equal(meshtastic_cluster_doc_hash_scoped(&doc, &core), hash_before,
		      "a row outside the claim must not move the advertised hash");
	zassert_equal(meshtastic_cluster_doc_count_scoped(&doc, &core), count_before);
	meshtastic_cluster_doc_max_stamp_scoped(&doc, &core, &max_after);
	zassert_equal(meshtastic_hlc_compare(&max_before, &max_after), 0,
		      "nor the advertised max stamp — all three legs are scoped or none is");

	/* And the unscoped legs still see everything, or a FULL peer could not
	 * tell it was ahead. */
	{
		struct meshtastic_cluster_scope full =
			scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);

		zassert_equal(meshtastic_cluster_doc_count_scoped(&doc, &full), 3U);
	}
}

/*
 * The bound that makes the constrained tier safe to enter: CORE cannot outgrow
 * base + own, whatever happens. Six own tombstones is still six rows, because a
 * tombstone REPLACES the entry at its key rather than adding one — which is the
 * property that lets MAX_ENTRIES have a floor at all.
 */
ZTEST(cluster_doc, test_core_scope_cannot_exceed_twice_the_allowlist)
{
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	const uint16_t sections = 6U; /* the v1 allowlist */

	for (uint16_t i = 0U; i < sections; i++) {
		struct meshtastic_cluster_key b = base_key((uint16_t)(SEC_DEVICE + i));
		struct meshtastic_cluster_key n = node_key(NODE_A, (uint16_t)(SEC_DEVICE + i));

		put(&b, 100 + i, NODE_A);
		put(&n, 200 + i, NODE_A);
	}
	zassert_equal(meshtastic_cluster_doc_count_scoped(&doc, &core), 2U * sections);

	/* Now tombstone every one of my own: still the same number of rows. */
	for (uint16_t i = 0U; i < sections; i++) {
		struct meshtastic_cluster_key n = node_key(NODE_A, (uint16_t)(SEC_DEVICE + i));
		struct meshtastic_hlc_stamp t = at(500 + i, NODE_A);

		zassert_equal(meshtastic_cluster_doc_accept(&doc, &n, &t, true, NULL, 0), 1);
	}
	zassert_equal(meshtastic_cluster_doc_count_scoped(&doc, &core), 2U * sections,
		      "a tombstone replaces an entry; it does not add a row");
}

/*
 * The two "no"s are not the same answer. Out of scope means "never mine, do
 * nothing"; out of space means "mine, and this node has outgrown its table" —
 * the honest signal that the claim must be narrowed. Conflating them either
 * reinstates the request loop 23b47de closed, or makes an already-narrowed node
 * narrow itself again over a row it does not want.
 */
ZTEST(cluster_doc, test_want_reports_no_space_instead_of_lying)
{
	struct meshtastic_cluster_scope full = scope_of(MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	struct meshtastic_cluster_key theirs = node_key(NODE_B, SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(100, NODE_B);

	zassert_equal(meshtastic_cluster_doc_want(&doc, &full, &theirs, &s),
		      MESHTASTIC_CLUSTER_WANT_YES);
	zassert_equal(meshtastic_cluster_doc_want(&doc, &core, &theirs, &s),
		      MESHTASTIC_CLUSTER_WANT_NO,
		      "a row we never claimed is simply not wanted");

	/* Fill the table with rows CORE(NODE_A) does not claim. */
	for (uint16_t i = 0U; i < CAP; i++) {
		struct meshtastic_cluster_key k = node_key(NODE_B, (uint16_t)(SEC_DEVICE + i));

		put(&k, 100 + i, NODE_B);
	}
	zassert_equal(doc.count, CAP);

	zassert_equal(meshtastic_cluster_doc_want(&doc, &full, &mine, &s),
		      MESHTASTIC_CLUSTER_WANT_NO_SPACE,
		      "a row we DO claim and cannot store is the signal to narrow the claim");
	zassert_equal(meshtastic_cluster_doc_want(&doc, &core, &theirs, &s),
		      MESHTASTIC_CLUSTER_WANT_NO,
		      "but a full table must never report NO_SPACE for a row outside the "
		      "scope — that would make a narrowed node narrow itself again");

	/* The legacy wrapper is exactly the FULL-scope YES case, so every
	 * existing caller keeps its behaviour. */
	zassert_false(meshtastic_cluster_doc_wants(&doc, &mine, &s));
}

ZTEST(cluster_doc, test_retain_keeps_base_and_my_own_and_reports_what_it_dropped)
{
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_cluster_key theirs1 = node_key(NODE_B, SEC_DEVICE);
	struct meshtastic_cluster_key theirs2 = node_key(NODE_B, SEC_DISPLAY);
	const struct meshtastic_cluster_entry *eff_before, *eff_after;
	uint8_t before_payload;
	uint16_t dropped;

	put(&b, 100, NODE_A);
	put(&mine, 200, NODE_A);
	put(&theirs1, 300, NODE_B);
	put(&theirs2, 400, NODE_B);
	zassert_equal(doc.count, 4U);

	eff_before = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_not_null(eff_before);
	before_payload = eff_before->payload[0];

	dropped = meshtastic_cluster_doc_retain(&doc, &core, NULL, NULL);
	zassert_equal(dropped, 2U, "exactly the rows outside the claim");
	zassert_equal(doc.count, 2U);
	zassert_not_null(meshtastic_cluster_doc_find(&doc, &b));
	zassert_not_null(meshtastic_cluster_doc_find(&doc, &mine));
	zassert_is_null(meshtastic_cluster_doc_find(&doc, &theirs1));
	zassert_is_null(meshtastic_cluster_doc_find(&doc, &theirs2));

	/* Sort order must survive, or the hash stops being arrival-order
	 * independent and two converged nodes disagree. */
	for (uint16_t i = 1U; i < doc.count; i++) {
		zassert_true(meshtastic_cluster_key_cmp(&doc.entries[i - 1U].key,
							&doc.entries[i].key) < 0,
			     "retain must leave the table sorted");
	}

	/* THE PROPERTY THAT MAKES NARROWING SAFE: CORE(me) is exactly the
	 * closure of effective(me, ·), so dropping everything else cannot
	 * change what this node runs. */
	eff_after = meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY);
	zassert_not_null(eff_after);
	zassert_equal(eff_after->payload[0], before_payload);
	zassert_not_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DEVICE));
}

ZTEST(cluster_doc, test_retain_is_idempotent_and_frees_room)
{
	struct meshtastic_cluster_scope core = scope_of(MESHTASTIC_CLUSTER_SCOPE_CORE, NODE_A);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(999, NODE_A);
	uint32_t hash_after_first;
	uint8_t v[] = {0x22};

	for (uint16_t i = 0U; i < CAP; i++) {
		struct meshtastic_cluster_key k = node_key(NODE_B, (uint16_t)(SEC_DEVICE + i));

		put(&k, 100 + i, NODE_B);
	}
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &mine, &s, false, v, 1), -ENOSPC,
		      "the premise: a full table refuses a row it would otherwise take");

	zassert_equal(meshtastic_cluster_doc_retain(&doc, &core, NULL, NULL), CAP);
	zassert_equal(doc.count, 0U);
	hash_after_first = meshtastic_cluster_doc_hash_scoped(&doc, &core);

	zassert_equal(meshtastic_cluster_doc_retain(&doc, &core, NULL, NULL), 0U,
		      "a second pass has nothing left to do");
	zassert_equal(meshtastic_cluster_doc_hash_scoped(&doc, &core), hash_after_first);

	zassert_equal(meshtastic_cluster_doc_accept(&doc, &mine, &s, false, v, 1), 1,
		      "and the room it freed is real");
}

/* Collector for the clear() callback — file scope, not a nested function. */
static struct meshtastic_cluster_key cleared_keys[8];
static uint16_t cleared_n;

static void collect_cleared(const struct meshtastic_cluster_key *k, void *ctx)
{
	ARG_UNUSED(ctx);
	if (cleared_n < ARRAY_SIZE(cleared_keys)) {
		cleared_keys[cleared_n++] = *k;
	}
}

/*
 * Clearing is not expressible as a retain(): base is inside every scope by
 * construction, so there is no scope meaning "nothing". It has to be its own
 * operation, and it has to report every key it dropped — that report is what
 * lets the module forget them in flash too, which is the whole point of
 * offering a clean at all.
 */
ZTEST(cluster_doc, test_clear_empties_and_reports_every_key)
{
	struct meshtastic_cluster_key b = base_key(SEC_DEVICE);
	struct meshtastic_cluster_key mine = node_key(NODE_A, SEC_DISPLAY);
	struct meshtastic_cluster_key theirs = node_key(NODE_B, SEC_DEVICE);
	struct meshtastic_hlc_stamp s1 = at(100, NODE_A);
	uint8_t v[] = {0x33};

	put(&b, 100, NODE_A);
	put(&mine, 200, NODE_A);
	put(&theirs, 300, NODE_B);
	zassert_equal(doc.count, 3U);

	cleared_n = 0U;
	zassert_equal(meshtastic_cluster_doc_clear(&doc, collect_cleared, NULL), 3U);
	zassert_equal(cleared_n, 3U,
		      "every dropped key must be reported, or its flash record outlives the "
		      "entry and comes back at the next boot");

	zassert_equal(doc.count, 0U);
	zassert_is_null(meshtastic_cluster_doc_find(&doc, &b));
	zassert_is_null(meshtastic_cluster_doc_find(&doc, &mine));
	zassert_is_null(meshtastic_cluster_doc_find(&doc, &theirs));
	zassert_is_null(meshtastic_cluster_doc_effective(&doc, NODE_A, SEC_DISPLAY),
			"and effective() must have nothing left to answer with");

	/* Usable again immediately — a clean, not a brick. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &b, &s1, false, v, 1), 1);
	zassert_equal(doc.count, 1U);
	zassert_equal(meshtastic_cluster_doc_clear(&doc, NULL, NULL), 1U,
		      "a NULL callback is allowed");
	zassert_equal(meshtastic_cluster_doc_clear(&doc, NULL, NULL), 0U,
		      "and clearing an empty document is a no-op, not an error");
}

/* ==========================================================================
 * The settings-key parser (meshtastic_cluster_key.h).
 *
 * This replaced the firmware's only sscanf, to get picolibc's vfscanf (~2 KB) out of the
 * image. It parses keys that are ALREADY PERSISTED IN NVS on the bench, so the shape it
 * accepts is a compatibility contract, not an implementation detail -- and the version
 * that lived inside meshtastic_cluster.c was reachable from no test at all.
 * ========================================================================== */

static void parse_ok(const char *key, char exp_layer, uint32_t exp_node, uint32_t exp_sec)
{
	char layer = 0;
	uint32_t node = 0xDEADBEEF;
	uint32_t sec = 0xDEADBEEF;

	zassert_ok(meshtastic_cluster_parse_entry_key(key, &layer, &node, &sec),
		   "\"%s\" should parse", key);
	zassert_equal(layer, exp_layer, "\"%s\": layer", key);
	zassert_equal(node, exp_node, "\"%s\": node id", key);
	zassert_equal(sec, exp_sec, "\"%s\": section", key);
}

static void parse_fails(const char *key)
{
	char layer = 0;
	uint32_t node = 0;
	uint32_t sec = 0;

	zassert_true(meshtastic_cluster_parse_entry_key(key, &layer, &node, &sec) != 0,
		     "\"%s\" should be rejected", key);
}

/* The canonical shape write_key() emits, "%c%08x/%u", for both layers. */
ZTEST(cluster_doc, test_entry_key_parses_what_we_write)
{
	parse_ok("b00000000/256", 'b', 0x00000000U, 256U);
	parse_ok("n121f8bac/0", 'n', 0x121f8bacU, 0U);
	parse_ok("b075c78e8/4294967295", 'b', 0x075c78e8U, 4294967295U);
	/* upper-case hex, because %8x accepted it and an older writer may have emitted it */
	parse_ok("nAB12CD34/7", 'n', 0xAB12CD34U, 7U);
	/* fewer than 8 hex digits: %8x read up to 8, not exactly 8 */
	parse_ok("nf/1", 'n', 0xfU, 1U);
}

ZTEST(cluster_doc, test_entry_key_rejects_malformed)
{
	parse_fails("");                  /* empty */
	parse_fails("b");                 /* layer only */
	parse_fails("b00000000");         /* no section */
	parse_fails("b00000000/");        /* empty section */
	parse_fails("bzzzzzzzz/1");       /* not hex */
	parse_fails("b/1");               /* no node id */
	parse_fails("b00000000/1x");      /* trailing junk after the section */
	parse_fails("b00000000/1/2");     /* an extra component */
	parse_fails("b00000000/4294967296"); /* section overflows uint32 */
}

/* A 9th hex digit is not part of the id: "%8x" stopped at eight, so the 9th character had
 * to be the '/'. A key with nine hex digits is malformed, not a key with a truncated id --
 * getting this wrong would silently map two different nodes onto one entry. */
ZTEST(cluster_doc, test_entry_key_stops_at_eight_hex_digits)
{
	parse_fails("b123456789/1");
	parse_ok("b12345678/1", 'b', 0x12345678U, 1U);
}

/* ==========================================================================
 * Fragmentation arithmetic (agents-ooma.36). Tested here, directly, because the
 * one property that matters most -- a send cursor left past the end of an entry
 * that was replaced mid-send -- cannot be reproduced through the simulated radio:
 * injecting a frame costs more simulated time than the gap between two fragments.
 * ========================================================================== */

/* Walk a whole payload through frag_take and prove the fragments tile it:
 * contiguous, in order, none empty but a lone tombstone, ending exactly at len. */
static unsigned int walk_fragments(uint16_t len, uint16_t cap)
{
	uint16_t off = 0U;
	unsigned int n = 0U;

	for (;;) {
		uint16_t start = off;
		uint16_t take = meshtastic_cluster_frag_take(len, &off, cap);

		zassert_equal(off, start, "a cursor inside the payload must not move");
		zassert_true(take <= cap, "a fragment exceeded the cap");
		zassert_true(take > 0U || len == 0U, "an empty fragment of a non-empty entry");
		n++;
		off = (uint16_t)(off + take);
		if (off >= len) {
			zassert_equal(off, len, "fragments overran the payload");
			return n;
		}
		zassert_true(n < 1000U, "fragment walk did not terminate");
	}
}

ZTEST(cluster_doc, test_frag_take_tiles_a_payload_exactly)
{
	zassert_equal(walk_fragments(0U, 88U), 1U, "a tombstone is one empty fragment");
	zassert_equal(walk_fragments(1U, 88U), 1U, NULL);
	zassert_equal(walk_fragments(88U, 88U), 1U, "exactly one cap is one fragment");
	zassert_equal(walk_fragments(89U, 88U), 2U, "one byte over is two");
	zassert_equal(walk_fragments(128U, 88U), 2U, NULL);
	zassert_equal(walk_fragments(4U, 2U), 2U, "the 2026-09-18 bench shape");
	zassert_equal(walk_fragments(128U, 1U), 128U, NULL);
}

/*
 * THE SHRINK GUARD. A cursor part way through a 100-byte entry finds the entry
 * replaced by a 5-byte one. Unguarded, 5 - 88 wraps to 65453 and the caller
 * memcpys a cap's worth from past the end of the payload. The guard restarts
 * the entry instead.
 */
ZTEST(cluster_doc, test_frag_take_restarts_an_entry_that_shrank_under_it)
{
	uint16_t off = 88U;
	uint16_t take = meshtastic_cluster_frag_take(5U, &off, 88U);

	zassert_equal(off, 0U, "a cursor past the end must restart the entry");
	zassert_equal(take, 5U, "and then send the new, shorter entry whole");

	/* Shrunk to nothing -- replaced by a tombstone. */
	off = 40U;
	take = meshtastic_cluster_frag_take(0U, &off, 88U);
	zassert_equal(off, 0U, NULL);
	zassert_equal(take, 0U, NULL);

	/* A cursor exactly AT the end is not "past" it: that is the final,
	 * empty fragment of an entry that shrank to precisely where we were.
	 * Harmless, and resetting here would resend the whole entry. */
	off = 50U;
	take = meshtastic_cluster_frag_take(50U, &off, 88U);
	zassert_equal(off, 50U, NULL);
	zassert_equal(take, 0U, NULL);
}

ZTEST(cluster_doc, test_frag_fits_bounds_on_the_wire_widths)
{
	uint32_t total;

	/* The ordinary shapes. */
	total = 102U;
	zassert_true(meshtastic_cluster_frag_fits(&total, 0U, 88U, 128U, 88U), NULL);
	total = 102U;
	zassert_true(meshtastic_cluster_frag_fits(&total, 88U, 14U, 128U, 88U), NULL);

	/* total 0 means the fragment is the entry, and is rewritten to say so. */
	total = 0U;
	zassert_true(meshtastic_cluster_frag_fits(&total, 0U, 16U, 128U, 88U), NULL);
	zassert_equal(total, 16U, "payload_total 0 must become the fragment length");
	total = 0U;
	zassert_true(meshtastic_cluster_frag_fits(&total, 0U, 0U, 128U, 88U), "tombstone");

	/* Refusals. */
	total = 129U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 0U, 88U, 128U, 88U),
		      "more than the table can hold");
	total = 102U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 0U, 89U, 128U, 88U),
		      "a fragment larger than a frame can carry");
	total = 102U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 90U, 14U, 128U, 88U),
		      "runs off the end of the payload it claims");

	/* The narrowing bug: checked after a cast to 16 bits, these would read as
	 * offset 0 and total 10 and pass. On the wire widths they cannot. */
	total = 10U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 65536U, 10U, 128U, 88U),
		      "offset 65536 must not alias offset 0");
	total = 65546U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 0U, 10U, 128U, 88U),
		      "total 65546 must not alias total 10");
	/* And the 32-bit wrap of off + len. */
	total = 100U;
	zassert_false(meshtastic_cluster_frag_fits(&total, 0xFFFFFFF0U, 20U, 128U, 88U),
		      "off + len must not wrap");
}

/* ==========================================================================
 * Signed entries (agents-ooma.31): what the author signs, and what the table
 * does with the signature. The crypto itself is tested in tests/xeddsa against
 * the reference's own vectors; these pin the BYTES and the BOOKKEEPING.
 * ========================================================================== */

static size_t sigbuf_of(uint8_t *buf, struct meshtastic_cluster_key k,
			struct meshtastic_hlc_stamp s, bool tomb, const uint8_t *p, size_t n)
{
	return meshtastic_cluster_signing_buffer(buf, MESHTASTIC_CLUSTER_SIGBUF_MAX, &k, &s, tomb,
						 p, n);
}

ZTEST(cluster_doc, test_signing_buffer_layout_is_pinned)
{
	static const uint8_t expect[] = {
		'z', 'e', 'p', 'h', 'y', 'r', 't', 'a', 's', 't', 'i', 'c', '/', 'c', 'l',
		'u', 's', 't', 'e', 'r', '-', 'e', 'n', 't', 'r', 'y', '/', 'v', '1', 0,
		0x01,                                     /* layer NODE */
		0xAA, 0xAA, 0xAA, 0xAA,                   /* key.node_id */
		0x10, 0x00,                               /* section 16 */
		0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, /* physical_ms */
		0x03, 0x00, 0x00, 0x00,                   /* counter */
		0xBB, 0xBB, 0xBB, 0xBB,                   /* author */
		0x00,                                     /* tombstone */
		0x02, 0x00,                               /* payload_len */
		0x5A, 0xA5,                               /* payload */
	};
	struct meshtastic_hlc_stamp s = {
		.physical_ms = 0x0102030405060708LL, .counter = 3U, .node_id = NODE_B};
	uint8_t p[] = {0x5A, 0xA5};
	uint8_t buf[MESHTASTIC_CLUSTER_SIGBUF_MAX];
	size_t n = sigbuf_of(buf, node_key(NODE_A, SEC_DISPLAY), s, false, p, sizeof(p));

	/* A byte-exact expectation, not a round trip: signer and verifier share
	 * this function, so a change to it would still "verify" -- and silently
	 * invalidate every signature a node on the old layout ever made. */
	zassert_equal(n, sizeof(expect), "length %zu", n);
	zassert_mem_equal(buf, expect, sizeof(expect), "signing buffer layout changed");
}

ZTEST(cluster_doc, test_signing_buffer_binds_every_field)
{
	uint8_t p[] = {1, 2, 3};
	uint8_t q[] = {1, 2, 4};
	struct meshtastic_hlc_stamp s = at(1000, NODE_A);
	uint8_t ref[MESHTASTIC_CLUSTER_SIGBUF_MAX], alt[MESHTASTIC_CLUSTER_SIGBUF_MAX];
	size_t rn = sigbuf_of(ref, node_key(NODE_A, SEC_DEVICE), s, false, p, 3);
	struct meshtastic_hlc_stamp s2;

#define DIFFERS(expr)                                                                           \
	do {                                                                                    \
		size_t an = (expr);                                                             \
		zassert_true(an != rn || memcmp(alt, ref, rn) != 0, "not bound: " #expr);       \
	} while (0)

	DIFFERS(sigbuf_of(alt, base_key(SEC_DEVICE), s, false, p, 3));         /* layer */
	DIFFERS(sigbuf_of(alt, node_key(NODE_B, SEC_DEVICE), s, false, p, 3)); /* owner */
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DISPLAY), s, false, p, 3)); /* section */
	s2 = s; s2.physical_ms++;
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s2, false, p, 3)); /* time */
	s2 = s; s2.counter++;
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s2, false, p, 3)); /* counter */
	s2 = s; s2.node_id = NODE_B;
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s2, false, p, 3)); /* AUTHOR */
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s, true, NULL, 0)); /* tombstone */
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s, false, q, 3)); /* payload */
	DIFFERS(sigbuf_of(alt, node_key(NODE_A, SEC_DEVICE), s, false, p, 2)); /* length */
#undef DIFFERS
}

/*
 * No packet signature can be replayed as an entry signature. A packet's signed
 * bytes are from | id | portnum | payload (little-endian u32s). Ours begin with
 * the prefix, so a packet reader sees portnum = bytes 8..11 = "stic". Every real
 * portnum is below 1024 (upstream reserves up to 511), and a node signs only its
 * own real packets -- so the two byte spaces cannot meet.
 */
ZTEST(cluster_doc, test_signing_buffer_cannot_be_a_packet_buffer)
{
	uint8_t buf[MESHTASTIC_CLUSTER_SIGBUF_MAX];
	uint32_t as_portnum;

	(void)sigbuf_of(buf, base_key(SEC_DEVICE), at(1, NODE_A), false, (const uint8_t *)"x", 1);
	as_portnum = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8) | ((uint32_t)buf[10] << 16) |
		     ((uint32_t)buf[11] << 24);
	zassert_true(as_portnum >= 1024U,
		     "the word a packet verifier reads as the portnum must be outside every "
		     "real portnum (got 0x%08x)", as_portnum);
}

ZTEST(cluster_doc, test_signing_buffer_refuses_what_it_cannot_hold)
{
	uint8_t big[MESHTASTIC_CLUSTER_PAYLOAD_MAX + 1U];
	uint8_t buf[MESHTASTIC_CLUSTER_SIGBUF_MAX];
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(1, NODE_A);

	memset(big, 7, sizeof(big));
	zassert_equal(meshtastic_cluster_signing_buffer(buf, sizeof(buf), &k, &s, false, big,
							sizeof(big)), 0U, "oversize payload");
	zassert_equal(meshtastic_cluster_signing_buffer(buf, 40U, &k, &s, false, big, 4U), 0U,
		      "buffer too small");
	zassert_true(meshtastic_cluster_signing_buffer(buf, sizeof(buf), &k, &s, false, big,
						       MESHTASTIC_CLUSTER_PAYLOAD_MAX) > 0U,
		     "the largest legal payload must fit SIGBUF_MAX");
}

ZTEST(cluster_doc, test_signature_travels_with_its_version_only)
{
	struct meshtastic_cluster_key k = node_key(NODE_A, SEC_DEVICE);
	struct meshtastic_hlc_stamp s1 = at(100, NODE_A), s2 = at(200, NODE_A);
	uint8_t v[] = {9};
	uint8_t sig_a[MESHTASTIC_CLUSTER_SIG_LEN], sig_b[MESHTASTIC_CLUSTER_SIG_LEN];
	const struct meshtastic_cluster_entry *e;

	memset(sig_a, 0xA1, sizeof(sig_a));
	memset(sig_b, 0xB2, sizeof(sig_b));

	zassert_equal(meshtastic_cluster_doc_accept_signed(&doc, &k, &s1, false, v, 1, sig_a), 1);
	e = meshtastic_cluster_doc_find(&doc, &k);
	zassert_true(e->has_sig, NULL);
	zassert_mem_equal(e->sig, sig_a, sizeof(sig_a), NULL);

	/* A stale version must not replace the signature on the one we hold. */
	zassert_equal(meshtastic_cluster_doc_accept_signed(&doc, &k, &s1, false, v, 1, sig_b), 0);
	zassert_mem_equal(e->sig, sig_a, sizeof(sig_a), "equal-stamp write moved the signature");

	/* A NEWER UNSIGNED version must not inherit the old proof: relayed, it
	 * would be a signature over bytes it does not cover. */
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &k, &s2, false, v, 1), 1);
	e = meshtastic_cluster_doc_find(&doc, &k);
	zassert_false(e->has_sig, "a newer unsigned version inherited the old signature");
}

ZTEST(cluster_doc, test_signature_is_not_part_of_the_digest)
{
	struct meshtastic_cluster_key k = base_key(SEC_DEVICE);
	struct meshtastic_hlc_stamp s = at(100, NODE_A);
	uint8_t v[] = {1};
	uint8_t sig[MESHTASTIC_CLUSTER_SIG_LEN];
	uint32_t unsigned_hash;

	/* Documented, not accidental: a signed and an unsigned copy of the same
	 * version hash the same, which is why signatures spread only on a NEW
	 * version (see the migration note on meshtastic_cluster_signing_buffer).
	 * If this ever changes, the rollout procedure changes with it. */
	memset(sig, 0x33, sizeof(sig));
	(void)meshtastic_cluster_doc_accept(&doc, &k, &s, false, v, 1);
	unsigned_hash = meshtastic_cluster_doc_hash(&doc);
	meshtastic_cluster_doc_init(&doc, storage, CAP);
	(void)meshtastic_cluster_doc_accept_signed(&doc, &k, &s, false, v, 1, sig);
	zassert_equal(meshtastic_cluster_doc_hash(&doc), unsigned_hash, NULL);
}
