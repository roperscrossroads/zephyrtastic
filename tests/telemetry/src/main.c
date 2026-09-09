/* SPDX-License-Identifier: GPL-3.0
 *
 * LocalStats telemetry tests.
 *
 * Two things are worth pinning down here, and neither is "does a struct get
 * filled in":
 *
 *   1. LocalStats reaches the PHONE and never the radio. That asymmetry is the
 *      whole reason this layer costs nothing to run (upstream's
 *      DeviceTelemetryModule::sendLocalStatsToPhone() calls sendToPhone(), never
 *      sendToMesh()), and it is the kind of property that quietly regresses the
 *      first time someone "unifies" the two send paths. So the phone hand-off is
 *      tested by decoding what actually landed in a transport queue, and the
 *      radio silence by watching the TX counter across the call.
 *
 *   2. The counter semantics the wire specifies. num_packets_rx is "good and
 *      bad" and num_packets_rx_bad is a SUBSET of it -- not a disjoint pair --
 *      and num_online_nodes can never exceed num_total_nodes. Both are easy to
 *      "fix" into being wrong.
 *
 * The stack is never initialized (no meshtastic_init()), so no threads run and
 * nothing else can move the counters underneath an assertion.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/telemetry.pb.h"

#include "meshtastic_core.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_sched.h"
#include "meshtastic_telemetry_internal.h"

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_clock.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"

/* Mirrors LOCAL_STATS_ONLINE_SEC in meshtastic_metrics.c -- upstream's
 * NUM_ONLINE_SECS. Duplicated rather than exported: the constant is a policy
 * choice, and a test that reads it from the implementation could not notice the
 * implementation changing it. */
#define LOCAL_STATS_TEST_ONLINE_SEC (2U * 60U * 60U)
#include <zephyr/meshtastic/telemetry.h>

#define Q_SIZE 4U

static struct meshtastic_phoneapi_frame q_storage[Q_SIZE];
static struct meshtastic_phoneapi api;
static meshtastic_ToRadio to_scratch;
static meshtastic_FromRadio from_scratch;

static void telemetry_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_sched_defaults();
	meshtastic_sched_stats_reset();
	meshtastic_phoneapi_init(&api, "test", q_storage, Q_SIZE, NULL, NULL, NULL, NULL,
				 &to_scratch, &from_scratch);
	meshtastic_phoneapi_register(&api);
	meshtastic_phoneapi_reset(&api);
}

ZTEST_SUITE(local_stats, NULL, NULL, telemetry_before, NULL, NULL);

/* ------------------------------------------------------------------ */
/* Collection                                                          */
/* ------------------------------------------------------------------ */

ZTEST(local_stats, test_collect_rejects_null)
{
	zassert_equal(meshtastic_collect_local_stats(NULL), -EINVAL,
		      "a NULL destination must be refused, not written through");
}

/* The wire contract: num_packets_rx counts good AND bad, so the bad count is a
 * subset of it. The router increments decode_failures and then rx_packets on the
 * same pass with no early return between them (meshtastic_router.c), which is
 * exactly what makes this hold. If someone ever makes the two counters disjoint,
 * this is the assertion that says so. */
ZTEST(local_stats, test_rx_bad_is_a_subset_of_rx)
{
	meshtastic_LocalStats stats;

	zassert_ok(meshtastic_collect_local_stats(&stats));
	zassert_true(stats.num_packets_rx_bad <= stats.num_packets_rx,
		     "rx_bad (%u) must be counted within rx (%u), not alongside it",
		     stats.num_packets_rx_bad, stats.num_packets_rx);
}

ZTEST(local_stats, test_online_never_exceeds_total)
{
	meshtastic_LocalStats stats;

	zassert_ok(meshtastic_collect_local_stats(&stats));
	zassert_true(stats.num_online_nodes <= stats.num_total_nodes,
		     "online (%u) cannot exceed total (%u)", stats.num_online_nodes,
		     stats.num_total_nodes);
	zassert_equal(stats.num_total_nodes, (uint32_t)meshtastic_nodedb_count(),
		      "total must be the NodeDB's own count");
}

/* The one rule about num_online_nodes worth getting wrong.
 *
 * An entry never heard this boot and carrying no persisted epoch has an UNKNOWN
 * age, and unknown must NOT be counted as online. That matters on this fleet
 * specifically: no bench node has a wall clock, so every entry restored from
 * NVS has exactly that shape, and rounding unknown down to "recent" would report
 * a node's entire saved NodeDB as live every time it rebooted.
 *
 * Tested against the age helper directly, with hand-built entries, so it does
 * not depend on standing the whole NodeDB up. */
ZTEST(local_stats, test_age_prefers_this_boot_uptime)
{
	struct meshtastic_nodedb_node node = {
		.num = 0x11223344U,
		.last_heard_uptime_sec = 1U,
	};

	/* Non-zero last_heard_uptime_sec means "heard this boot": differenced
	 * against uptime, which in a test has barely advanced. */
	zassert_true(meshtastic_local_stats_node_age_sec(&node) < LOCAL_STATS_TEST_ONLINE_SEC,
		     "an entry heard this boot must read as recent");
}

ZTEST(local_stats, test_unknown_age_is_not_online)
{
	struct meshtastic_nodedb_node node = {
		.num = 0x11223344U,
		.last_heard_uptime_sec = 0U, /* not heard this boot */
		.last_heard_epoch = 0U,      /* and never dated */
	};

	zassert_equal(meshtastic_local_stats_node_age_sec(&node), UINT32_MAX,
		      "no heard-time and no epoch is an UNKNOWN age, not age zero -- "
		      "reading it as zero would report every restored NVS entry as online");
	zassert_false(meshtastic_local_stats_node_age_sec(&node) < LOCAL_STATS_TEST_ONLINE_SEC,
		      "and unknown must therefore not be counted online");
}

/* The counterpart trap, from the other direction: an entry with a persisted
 * epoch is only datable if a clock has ever been seeded. Without one the epoch
 * cannot be differenced against anything, so it stays unknown. */
ZTEST(local_stats, test_epoch_without_a_clock_stays_unknown)
{
	struct meshtastic_nodedb_node node = {
		.num = 0x11223344U,
		.last_heard_uptime_sec = 0U,
		.last_heard_epoch = 1700000000U,
	};

	if (meshtastic_clock_valid()) {
		ztest_test_skip();
	}

	zassert_equal(meshtastic_local_stats_node_age_sec(&node), UINT32_MAX,
		      "a stored epoch is worthless without a clock to compare it against");
}

ZTEST(local_stats, test_heap_pair_is_consistent)
{
	meshtastic_LocalStats stats;

	zassert_ok(meshtastic_collect_local_stats(&stats));
	zassert_true(stats.heap_total_bytes > 0U,
		     "CONFIG_SYS_HEAP_RUNTIME_STATS is on, so the heap size must be readable");
	zassert_true(stats.heap_free_bytes <= stats.heap_total_bytes,
		     "free (%u) cannot exceed total (%u)", stats.heap_free_bytes,
		     stats.heap_total_bytes);
}

/* noise_floor has no source in this port (rf_measure records per-RECEPTION
 * RSSI/SNR, a different quantity). Left at zero deliberately -- asserted so that
 * a future noise-floor implementation has to come here and say so. */
ZTEST(local_stats, test_noise_floor_unreported)
{
	meshtastic_LocalStats stats;

	zassert_ok(meshtastic_collect_local_stats(&stats));
	zassert_equal(stats.noise_floor, 0,
		      "nothing measures a noise floor yet; update this test when something does");
}

/* ------------------------------------------------------------------ */
/* The phone hand-off                                                  */
/* ------------------------------------------------------------------ */

/* Pop one frame and decode it all the way down to the Telemetry variant, so the
 * assertion is about what a phone would actually parse, not about how many bytes
 * were queued. */
static bool pop_local_stats(meshtastic_LocalStats *out, uint32_t *portnum)
{
	struct meshtastic_phoneapi_frame frame;
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
	meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_zero;
	pb_istream_t stream;

	if (!meshtastic_phoneapi_pop_frame(&api, &frame)) {
		return false;
	}

	stream = pb_istream_from_buffer(frame.data, frame.len);
	if (!pb_decode(&stream, meshtastic_FromRadio_fields, &from)) {
		return false;
	}
	if (from.which_payload_variant != meshtastic_FromRadio_packet_tag ||
	    from.packet.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
		return false;
	}

	*portnum = from.packet.decoded.portnum;

	stream = pb_istream_from_buffer(from.packet.decoded.payload.bytes,
					from.packet.decoded.payload.size);
	if (!pb_decode(&stream, meshtastic_Telemetry_fields, &telemetry)) {
		return false;
	}
	if (telemetry.which_variant != meshtastic_Telemetry_local_stats_tag) {
		return false;
	}

	*out = telemetry.variant.local_stats;
	return true;
}

ZTEST(local_stats, test_push_reaches_the_phone_as_telemetry)
{
	meshtastic_LocalStats stats;
	uint32_t portnum = 0U;

	zassert_ok(meshtastic_send_local_stats_to_phone());
	zassert_equal(meshtastic_phoneapi_pending_count(&api), 1U,
		      "exactly one frame should have been enqueued");

	zassert_true(pop_local_stats(&stats, &portnum),
		     "the queued frame must decode as FromRadio.packet -> Telemetry.local_stats");
	zassert_equal(portnum, (uint32_t)meshtastic_PortNum_TELEMETRY_APP,
		      "LocalStats rides the ordinary telemetry port");
	zassert_true(stats.heap_total_bytes > 0U, "the decoded stats should carry real values");
}

/* The load-bearing property of this whole layer: pushing to the phone spends no
 * airtime. Watched via the TX counter rather than a mock-radio hook so the check
 * holds regardless of which radio backend a future config links in. */
ZTEST(local_stats, test_push_does_not_transmit)
{
	struct meshtastic_status before, after;

	zassert_ok(meshtastic_get_status(&before));
	zassert_ok(meshtastic_send_local_stats_to_phone());
	zassert_ok(meshtastic_get_status(&after));

	zassert_equal(after.tx_packets, before.tx_packets,
		      "LocalStats must never reach the radio (upstream sends it to the phone "
		      "only); a broadcast here would spend airtime on every node's private "
		      "bookkeeping");
	zassert_equal(after.tx_failures, before.tx_failures,
		      "and must not even attempt a transmission");
}

/* A phone that is not draining its queue must not be able to push real traffic
 * out of it. TELEMETRY_APP is classified droppable by the queue, so a burst of
 * LocalStats frames evicts itself rather than starving anything. */
ZTEST(local_stats, test_push_burst_stays_bounded)
{
	for (int i = 0; i < (int)Q_SIZE * 3; i++) {
		zassert_ok(meshtastic_send_local_stats_to_phone());
	}

	zassert_equal(meshtastic_phoneapi_pending_count(&api), Q_SIZE,
		      "the queue is bounded; a stalled phone cannot make it grow");
}

/* ------------------------------------------------------------------ */
/* Cadence: ModuleConfig.telemetry resolved (agents-dnr4.10)           */
/* ------------------------------------------------------------------ */

/* The stack is still never initialised here; the resolver reads the config
 * store and the channel table, and both stand on their own. The suite seeds
 * the store the way meshtastic_init() would and puts the channel table on the
 * default channel, because the default-channel coercion is one of the rules. */
static void cadence_before(void *fixture)
{
	static const struct meshtastic_config cfg = {
		.node_id = 0x0A0A0A0AU,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	};

	ARG_UNUSED(fixture);
	zassert_ok(meshtastic_channels_init_defaults(), "channel defaults");
	zassert_ok(meshtastic_config_store_seed(&cfg), "store seed");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	mt.modem.spread_factor = 11U; /* LongFast: SF11 / 250 kHz */
	mt.modem.bandwidth_hz = 250000U;
}

ZTEST_SUITE(telemetry_cadence, NULL, NULL, cadence_before, NULL, NULL);

static void store_telemetry(uint32_t device_iv, bool device_en, uint32_t env_iv, bool env_en)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	mod.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
	mod.payload_variant.telemetry.device_update_interval = device_iv;
	mod.payload_variant.telemetry.device_telemetry_enabled = device_en;
	mod.payload_variant.telemetry.environment_update_interval = env_iv;
	mod.payload_variant.telemetry.environment_measurement_enabled = env_en;
	zassert_ok(meshtastic_config_store_set_module(&mod), "store write");
}

/* Rename the primary channel: no longer "the default channel", so the
 * minimum-coercion rule must not apply. */
static void leave_the_default_channel(void)
{
	meshtastic_Channel ch = *meshtastic_channels_get(meshtastic_channels_primary_index());

	strcpy(ch.settings.name, "bench");
	zassert_ok(meshtastic_channels_set_slot(meshtastic_channels_primary_index(), &ch), "");
}

ZTEST(telemetry_cadence, test_seed_follows_the_build)
{
	struct meshtastic_telemetry_settings s;

	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_enabled, IS_ENABLED(CONFIG_MESHTASTIC_DEVICE_METRICS_AUTO_SEND),
		      "device flag seeded from AUTO_SEND");
	zassert_equal(s.device_interval_sec, CONFIG_MESHTASTIC_DEVICE_METRICS_INTERVAL_SEC,
		      "zero interval coalesces to the Kconfig default");
	zassert_false(s.environment_enabled, "no environment module in this build");
}

ZTEST(telemetry_cadence, test_zero_interval_is_role_aware)
{
	struct meshtastic_telemetry_settings s;

	store_telemetry(0U, true, 0U, false);
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 12U * 3600U, "router default is ONE_DAY / 2");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 12U * 3600U, "router_late too");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, CONFIG_MESHTASTIC_DEVICE_METRICS_INTERVAL_SEC, "");
}

ZTEST(telemetry_cadence, test_default_channel_coerces_a_short_interval)
{
	struct meshtastic_telemetry_settings s;

	/* Reference NodeDB::init: on a default channel, below the role-aware
	 * minimum -> the minimum (30 min; 12 h for a router). */
	store_telemetry(600U, true, 600U, true);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 30U * 60U, "600 s -> 1800 s on LongFast");
	zassert_equal(s.environment_interval_sec, 30U * 60U, "same rule for environment");

	store_telemetry(7200U, true, 0U, false);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 7200U, "above the minimum is kept");

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	store_telemetry(3600U, true, 0U, false);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 12U * 3600U, "router minimum is 12 h");
}

ZTEST(telemetry_cadence, test_custom_channel_keeps_a_short_interval)
{
	struct meshtastic_telemetry_settings s;

	leave_the_default_channel();
	store_telemetry(600U, true, 0U, false);
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_interval_sec, 600U, "no coercion off the default channel");
}

ZTEST(telemetry_cadence, test_scaling_matches_the_reference_coefficient)
{
	/* Reference congestionScalingCoefficient: 2^SF / (BW_kHz * 100) per online
	 * node past 40. LongFast (SF11/250k) -> 0.08192; the reference's own
	 * example table is for a 30 min base, this one for 3600 s. */
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 0U), 3600U, "");
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 40U), 3600U,
		      "no scaling up to 40 nodes");
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 41U), 3894U,
		      "41 nodes: x1.08192");
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 60U), 9498U,
		      "60 nodes: x2.6384");

	/* ShortTurbo (SF7/500k) scales far less: 0.00256 per node. */
	mt.modem.spread_factor = 7U;
	mt.modem.bandwidth_hz = 500000U;
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 60U), 3784U, "");

	/* Routers, sensors and trackers are never scaled. */
	mt.modem.spread_factor = 11U;
	mt.modem.bandwidth_hz = 250000U;
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 60U), 3600U, "router");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_TRACKER);
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(3600U, 60U), 3600U, "tracker");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);

	/* Capped at the reference's MAX_INTERVAL (INT32_MAX ms). */
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(UINT32_MAX, 0U),
		      (uint32_t)INT32_MAX / 1000U, "cap without scaling");
	zassert_equal(meshtastic_telemetry_scaled_interval_sec(2000000U, 200U),
		      (uint32_t)INT32_MAX / 1000U, "cap with scaling");
}

ZTEST(telemetry_cadence, test_flags_gate_each_sender)
{
	struct meshtastic_telemetry_settings s;

	store_telemetry(0U, false, 0U, true);
	meshtastic_telemetry_settings(&s);
	zassert_false(s.device_enabled, "device off");
	zassert_true(s.environment_enabled, "environment on");
	store_telemetry(0U, true, 0U, false);
	meshtastic_telemetry_settings(&s);
	zassert_true(s.device_enabled, "");
	zassert_false(s.environment_enabled, "");
}
