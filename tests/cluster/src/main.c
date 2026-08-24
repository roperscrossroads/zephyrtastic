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
 * THE ENOSPC TRAP. A full table refuses new keys, but the diff predicate does
 * not know that: wants() still says "pull this", so the walk asks for a key
 * every round, is answered, fails to store it, and asks again forever. Nothing
 * crashes and nothing corrupts — but a fleet whose document outgrows one
 * member's table burns airtime on that member indefinitely, and the walk never
 * reports why.
 *
 * That is a property of the two functions together, so it belongs here rather
 * than in the module: if a future change makes wants() capacity-aware, this
 * test is what says so out loud.
 */
ZTEST(cluster_doc, test_full_table_still_wants_what_it_cannot_store)
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
	zassert_true(meshtastic_cluster_doc_wants(&doc, &unseen, &s),
		     "a key we lack is wanted regardless of capacity");
	zassert_equal(meshtastic_cluster_doc_accept(&doc, &unseen, &s, false, v, 1), -ENOSPC);
	zassert_true(meshtastic_cluster_doc_wants(&doc, &unseen, &s),
		     "KNOWN: still wanted after ENOSPC — the walk will re-request it every "
		     "round with no way to store it. Bounded by the digest cadence, never "
		     "resolved. Capacity-aware wants() is the fix if this ever bites.");
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
