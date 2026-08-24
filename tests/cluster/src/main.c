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
