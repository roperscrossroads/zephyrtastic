/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the scheduler / QoS policy surface (meshtastic_sched).
 * Deterministic, no radio — verifies tier mapping, knob parsing, presets, and
 * stats accounting. The concurrency behavior of the priority egress queue is
 * exercised separately (hardware pass / gated-mock test).
 */

#include <errno.h>

#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_sched.h"

static void sched_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_sched_defaults();
	meshtastic_sched_stats_reset();
}

ZTEST_SUITE(sched, NULL, NULL, sched_before, NULL, NULL);

ZTEST(sched, test_compiled_defaults)
{
	const struct meshtastic_sched_config *c = meshtastic_sched_get();

	zassert_equal(c->tx_order, MT_SCHED_ORDER_PRIORITY, "default order should be priority");
	zassert_equal(c->tx_overflow, MT_SCHED_OVF_DROP_LOWEST, "default overflow should drop-lowest");
	zassert_true(c->tx_depth >= 1, "depth must be positive");
	zassert_equal(c->phone_evict, MT_SCHED_PHONE_PROTECT, "default phone evict should protect");
	zassert_true(c->airtime_max_util > 0 && c->airtime_max_util <= 100,
		     "default airtime gate should be enabled and a valid percent");
	zassert_true(c->dedup_ttl_sec > 0, "default dedup TTL should be enabled");
}

ZTEST(sched, test_tier_mapping)
{
	zassert_equal(meshtastic_sched_tier_for(MESHTASTIC_PORT_ROUTING), MT_SCHED_TIER_ACK);
	zassert_equal(meshtastic_sched_tier_for(MESHTASTIC_PORT_POSITION), MT_SCHED_TIER_BG);
	zassert_equal(meshtastic_sched_tier_for(MESHTASTIC_PORT_TELEMETRY), MT_SCHED_TIER_BG);
	zassert_equal(meshtastic_sched_tier_for(MESHTASTIC_PORT_NODEINFO), MT_SCHED_TIER_BG);
	zassert_equal(meshtastic_sched_tier_for(MESHTASTIC_PORT_TEXT_MESSAGE), MT_SCHED_TIER_NORMAL);
	/* Unknown portnum falls back to NORMAL, never dropped as background. */
	zassert_equal(meshtastic_sched_tier_for(12345U), MT_SCHED_TIER_NORMAL);
}

ZTEST(sched, test_set_valid_knobs)
{
	zassert_ok(meshtastic_sched_set("tx.order", "fifo"));
	zassert_equal(meshtastic_sched_get()->tx_order, MT_SCHED_ORDER_FIFO);
	zassert_ok(meshtastic_sched_set("tx.order", "priority"));
	zassert_equal(meshtastic_sched_get()->tx_order, MT_SCHED_ORDER_PRIORITY);

	zassert_ok(meshtastic_sched_set("tx.overflow", "drop-newest"));
	zassert_equal(meshtastic_sched_get()->tx_overflow, MT_SCHED_OVF_DROP_NEWEST);
	zassert_ok(meshtastic_sched_set("tx.overflow", "drop-lowest"));
	zassert_equal(meshtastic_sched_get()->tx_overflow, MT_SCHED_OVF_DROP_LOWEST);

	zassert_ok(meshtastic_sched_set("tx.depth", "3"));
	zassert_equal(meshtastic_sched_get()->tx_depth, 3);

	zassert_ok(meshtastic_sched_set("phone.evict", "drop-oldest"));
	zassert_equal(meshtastic_sched_get()->phone_evict, MT_SCHED_PHONE_DROP_OLDEST);
	zassert_ok(meshtastic_sched_set("phone.evict", "protect"));
	zassert_equal(meshtastic_sched_get()->phone_evict, MT_SCHED_PHONE_PROTECT);

	zassert_ok(meshtastic_sched_set("airtime.max", "0"));
	zassert_equal(meshtastic_sched_get()->airtime_max_util, 0);
	zassert_ok(meshtastic_sched_set("airtime.max", "55"));
	zassert_equal(meshtastic_sched_get()->airtime_max_util, 55);

	zassert_ok(meshtastic_sched_set("dedup.ttl", "0"));
	zassert_equal(meshtastic_sched_get()->dedup_ttl_sec, 0);
	zassert_ok(meshtastic_sched_set("dedup.ttl", "600"));
	zassert_equal(meshtastic_sched_get()->dedup_ttl_sec, 600);
}

ZTEST(sched, test_set_invalid_rejected)
{
	uint8_t depth_before = meshtastic_sched_get()->tx_depth;

	zassert_equal(meshtastic_sched_set("bogus.key", "x"), -ENOENT);
	zassert_equal(meshtastic_sched_set("tx.order", "sideways"), -EINVAL);
	zassert_equal(meshtastic_sched_set("tx.overflow", "explode"), -EINVAL);
	zassert_equal(meshtastic_sched_set("tx.depth", "0"), -EINVAL);
	zassert_equal(meshtastic_sched_set("tx.depth", "99999"), -EINVAL);
	zassert_equal(meshtastic_sched_set("tx.depth", "notanumber"), -EINVAL);
	zassert_equal(meshtastic_sched_set("phone.evict", "maybe"), -EINVAL);
	zassert_equal(meshtastic_sched_set("airtime.max", "101"), -EINVAL);
	zassert_equal(meshtastic_sched_set("airtime.max", "-1"), -EINVAL);
	zassert_equal(meshtastic_sched_set("dedup.ttl", "70000"), -EINVAL);

	/* A rejected set must not mutate state. */
	zassert_equal(meshtastic_sched_get()->tx_depth, depth_before);
}

ZTEST(sched, test_presets)
{
	zassert_ok(meshtastic_sched_apply_preset("legacy"));
	zassert_equal(meshtastic_sched_get()->tx_order, MT_SCHED_ORDER_FIFO,
		      "legacy must be FIFO for A/B comparison");
	zassert_equal(meshtastic_sched_get()->tx_overflow, MT_SCHED_OVF_DROP_NEWEST);
	zassert_equal(meshtastic_sched_get()->airtime_max_util, 0,
		      "legacy disables the airtime gate");
	zassert_equal(meshtastic_sched_get()->dedup_ttl_sec, 0,
		      "legacy uses a never-expiring dedup ring");

	zassert_ok(meshtastic_sched_apply_preset("default"));
	zassert_equal(meshtastic_sched_get()->tx_order, MT_SCHED_ORDER_PRIORITY);

	zassert_equal(meshtastic_sched_apply_preset("does-not-exist"), -ENOENT);

	/* Every advertised preset name must actually apply. */
	for (int i = 0; meshtastic_sched_preset_name(i) != NULL; i++) {
		zassert_ok(meshtastic_sched_apply_preset(meshtastic_sched_preset_name(i)),
			   "advertised preset '%s' failed to apply",
			   meshtastic_sched_preset_name(i));
	}
}

ZTEST(sched, test_stats_accounting)
{
	struct meshtastic_sched_stats st;

	meshtastic_sched_stat_enq(MT_SCHED_TIER_ACK, 3);
	meshtastic_sched_stat_enq(MT_SCHED_TIER_BG, 5);
	meshtastic_sched_stat_enq(MT_SCHED_TIER_BG, 2);
	meshtastic_sched_stat_drop(MT_SCHED_TIER_BG);

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_enq[MT_SCHED_TIER_ACK], 1);
	zassert_equal(st.tx_enq[MT_SCHED_TIER_BG], 2);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 1);
	zassert_equal(st.ob_hiwater, 5, "hi-water tracks the peak occupancy");

	meshtastic_sched_stats_reset();
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_enq[MT_SCHED_TIER_ACK], 0);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 0);
	zassert_equal(st.ob_hiwater, 0);
}

/* A snapshot must return a whole, self-consistent policy: every field belongs to
 * the same preset, and it tracks committed writes. */
ZTEST(sched, test_snapshot_coherent)
{
	struct meshtastic_sched_config snap;

	zassert_ok(meshtastic_sched_apply_preset("legacy"));
	meshtastic_sched_snapshot(&snap);
	zassert_equal(snap.tx_order, MT_SCHED_ORDER_FIFO);
	zassert_equal(snap.tx_overflow, MT_SCHED_OVF_DROP_NEWEST);
	zassert_equal(snap.phone_evict, MT_SCHED_PHONE_DROP_OLDEST);
	zassert_equal(snap.airtime_max_util, 0, "legacy bundle must be coherent");
	zassert_equal(snap.dedup_ttl_sec, 0, "legacy bundle must be coherent");

	zassert_ok(meshtastic_sched_apply_preset("default"));
	meshtastic_sched_snapshot(&snap);
	zassert_equal(snap.tx_order, MT_SCHED_ORDER_PRIORITY);
	zassert_true(snap.airtime_max_util > 0 && snap.dedup_ttl_sec > 0);

	/* A snapshot reflects an individual committed knob change. */
	zassert_ok(meshtastic_sched_set("airtime.max", "17"));
	meshtastic_sched_snapshot(&snap);
	zassert_equal(snap.airtime_max_util, 17);
	/* ...and agrees with the live pointer for that field. */
	zassert_equal(snap.airtime_max_util, meshtastic_sched_get()->airtime_max_util);
}

ZTEST(sched, test_airtime_and_dedup_stats)
{
	struct meshtastic_sched_stats st;

	meshtastic_sched_stat_airtime_drop();
	meshtastic_sched_stat_airtime_drop();
	meshtastic_sched_stat_dedup_expired();

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_airtime_drop, 2);
	zassert_equal(st.dedup_expired, 1);
}

/* Changing modes must zero the drop counters so each policy is measured from a
 * clean slate. */
ZTEST(sched, test_stats_reset_on_mode_change)
{
	struct meshtastic_sched_stats st;

	/* A committed `set` resets. */
	meshtastic_sched_stat_drop(MT_SCHED_TIER_BG);
	meshtastic_sched_stat_airtime_drop();
	zassert_ok(meshtastic_sched_set("tx.depth", "5"));
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 0, "set must reset drop stats");
	zassert_equal(st.tx_airtime_drop, 0);

	/* A rejected `set` must NOT reset. */
	meshtastic_sched_stat_drop(MT_SCHED_TIER_BG);
	zassert_equal(meshtastic_sched_set("tx.depth", "0"), -EINVAL);
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 1, "rejected set must not reset stats");

	/* Applying a preset resets. */
	zassert_ok(meshtastic_sched_apply_preset("legacy"));
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 0, "preset must reset drop stats");

	/* defaults() resets. */
	meshtastic_sched_stat_phone_drop(true);
	meshtastic_sched_defaults();
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.phone_drop, 0, "defaults must reset drop stats");
}
