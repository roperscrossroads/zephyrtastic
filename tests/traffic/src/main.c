/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Traffic management (agents-dnr4.20), driven through the sim radio.
 *
 * In the reference's terms (TrafficManagementModule.cpp):
 *   - a peer's position broadcast on a well-known channel that fingerprints the
 *     same grid cell as its last one inside position_min_interval_secs is
 *     dropped: not delivered, not relayed; a moved one, or one after the window,
 *     passes; 0 disables it, and a private channel is never shaped;
 *   - a TRACKER may refresh a duplicate hourly whatever the window says;
 *   - more than rate_limit_max_packets from one node inside the window are
 *     dropped, and the window then reopens;
 *   - a source whose undecodable frames exceed the threshold in 5 minutes stops
 *     being relayed;
 *   - our own traffic and traffic addressed to us are never shaped;
 *   - a config enabling the NodeInfo direct response is refused.
 *
 * Delivery is observed through the public receive callback; relaying through
 * frames on the sim radio whose originator is not this node.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"
#include "meshtastic_traffic.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define PEER_A       0x0B000001U
#define PEER_B       0x0B000002U
#define PEER_T       0x0B0000CCU /* the tracker */

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* ==========================================================================
 * Delivery capture (recv_cb) and air capture (sim radio)
 * ========================================================================== */

static struct {
	uint32_t total;
	uint32_t position_from_a;
	uint32_t position_from_b;
	uint32_t position_from_t;
	uint32_t text_from_a;
	uint32_t last_from;
	uint32_t last_port;
} delivered;

static void recv_cb(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t payload_len, int16_t rssi, int8_t snr)
{
	ARG_UNUSED(to);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);

	delivered.total++;
	delivered.last_from = from;
	delivered.last_port = portnum;
	if (portnum == MESHTASTIC_PORT_POSITION) {
		delivered.position_from_a += (from == PEER_A);
		delivered.position_from_b += (from == PEER_B);
		delivered.position_from_t += (from == PEER_T);
	} else if (portnum == MESHTASTIC_PORT_TEXT_MESSAGE) {
		delivered.text_from_a += (from == PEER_A);
	}
}

/* Frames on the air that some other node originated: relays. A relay waits out an
 * SNR-derived contention window first (a loud frame is relayed LAST, so the far
 * nodes go first), so this blocks until the air has been idle for RELAY_IDLE --
 * virtual time, so the wait costs nothing. */
#define RELAY_IDLE K_SECONDS(8)

static uint32_t drain_relayed(void)
{
	struct lora_sim_frame f;
	uint32_t relayed = 0U;

	while (lora_sim_take_tx(lora_dev, &f, RELAY_IDLE) == 0) {
		const struct meshtastic_wire_header *hdr = (const struct meshtastic_wire_header *)f.data;

		if (f.len >= MESHTASTIC_HDR_LEN && sys_le32_to_cpu(hdr->src) != TEST_NODE_ID) {
			relayed++;
		}
	}
	return relayed;
}

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

static void build_frame(uint32_t from, uint32_t to, uint32_t id, uint32_t portnum,
			const uint8_t *payload, size_t len, uint8_t *wire, uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = portnum,
		.payload = payload,
		.payload_len = len,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len), "build failed");
}

static void inject_wire(const uint8_t *wire, uint32_t wire_len)
{
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, 6), "inject failed");
	k_msleep(100);
}

static void inject_position(uint32_t from, uint32_t to, uint32_t id, int32_t lat_i, int32_t lon_i)
{
	meshtastic_Position pos = meshtastic_Position_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));

	pos.has_latitude_i = true;
	pos.latitude_i = lat_i;
	pos.has_longitude_i = true;
	pos.longitude_i = lon_i;
	pos.precision_bits = 32U;
	zassert_true(pb_encode(&os, meshtastic_Position_fields, &pos), "encode failed");
	build_frame(from, to, id, MESHTASTIC_PORT_POSITION, payload, os.bytes_written, wire,
		    &wire_len);
	inject_wire(wire, wire_len);
}

static void inject_text(uint32_t from, uint32_t to, uint32_t id)
{
	static const char text[] = "hi";
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	build_frame(from, to, id, MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)text,
		    sizeof(text) - 1U, wire, &wire_len);
	inject_wire(wire, wire_len);
}

/* A frame this node cannot decode: a valid frame whose channel byte names a
 * channel we do not have. */
static void inject_undecodable(uint32_t from, uint32_t id)
{
	static const char text[] = "??";
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_wire_header *hdr = (struct meshtastic_wire_header *)wire;

	build_frame(from, MESHTASTIC_NODE_BROADCAST, id, MESHTASTIC_PORT_TEXT_MESSAGE,
		    (const uint8_t *)text, sizeof(text) - 1U, wire, &wire_len);
	hdr->channel = 0x5AU;
	inject_wire(wire, wire_len);
}

/* A peer announcing itself with a role, so the NodeDB records it. */
static void inject_nodeinfo(uint32_t from, uint32_t id, meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));

	(void)snprintk(user.id, sizeof(user.id), "!%08x", from);
	strcpy(user.long_name, "Peer");
	strcpy(user.short_name, "PR");
	user.role = role;
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "encode failed");
	build_frame(from, MESHTASTIC_NODE_BROADCAST, id, MESHTASTIC_PORT_NODEINFO, payload,
		    os.bytes_written, wire, &wire_len);
	inject_wire(wire, wire_len);
}

static void set_traffic(uint32_t dedup_secs, uint32_t window_secs, uint32_t max_packets,
			uint32_t unknown_threshold)
{
	meshtastic_ModuleConfig_TrafficManagementConfig cfg =
		meshtastic_ModuleConfig_TrafficManagementConfig_init_zero;

	cfg.position_min_interval_secs = dedup_secs;
	cfg.rate_limit_window_secs = window_secs;
	cfg.rate_limit_max_packets = max_packets;
	cfg.unknown_packet_threshold = unknown_threshold;
	zassert_ok(meshtastic_traffic_set(&cfg), "traffic set failed");
}

/* Seattle, in Meshtastic's 1e-7 degree integers. */
#define LAT_A 476062000
#define LON_A (-1223321000)
/* ~11 km north: a different grid cell even at the default channel's 13-bit
 * (~5 km) precision, which is what the dedup fingerprint uses on LongFast. */
#define LAT_MOVED (LAT_A + 1000000)

static void *traffic_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	/* A relaying node: the drop verdict is about NOT relaying, so the node
	 * must relay everything else (same as mesh_sim's setup). */
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	meshtastic_set_recv_cb(recv_cb);
	zassert_true(meshtastic_channels_is_well_known(meshtastic_channels_primary_index()),
		     "LongFast + the default key is a well-known channel");
	return NULL;
}

static uint32_t next_id = 0x40000000U;

static void traffic_before(void *fixture)
{
	ARG_UNUSED(fixture);

	set_traffic(0U, 0U, 0U, 0U);
	meshtastic_traffic_reset();
	meshtastic_nodedb_reset(false);
	memset(&delivered, 0, sizeof(delivered));
	k_msleep(200);
	lora_sim_reset(lora_dev);
	next_id += 0x1000U;
}

ZTEST_SUITE(traffic, NULL, traffic_setup, traffic_before, NULL, NULL);

/* ==========================================================================
 * Position dedup
 * ========================================================================== */

ZTEST(traffic, test_a_duplicate_position_inside_the_window_is_dropped)
{
	meshtastic_TrafficManagementStats st;

	set_traffic(60U, 0U, 0U, 0U);

	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 1U, "first position delivered");
	zassert_equal(drain_relayed(), 1U, "and relayed");

	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 1U, "same cell inside the window: not delivered");
	zassert_equal(drain_relayed(), 0U, "and not relayed");
	meshtastic_traffic_stats(&st);
	zassert_equal(st.position_dedup_drops, 1U, "counted as a dedup drop");

	/* GPS jitter inside the cell is still the same cell. */
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A + 30, LON_A - 30);
	zassert_equal(delivered.position_from_a, 1U, "jitter inside the cell: still a duplicate");

	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_MOVED, LON_A);
	zassert_equal(delivered.position_from_a, 2U, "a different cell passes");
	zassert_equal(drain_relayed(), 1U, "and is relayed");

	/* Back to the first cell: it is a new cell relative to the LAST one let
	 * through, so it passes -- the fingerprint is of the previous position. */
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 3U, "moving back is a move");

	/* A node repeating faster than the window: only what is let through
	 * re-stamps the window (reference), so the repeat at the window's edge
	 * passes. Re-stamping on every drop would mute it forever. */
	k_sleep(K_SECONDS(30));
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 3U, "30 s in: still a duplicate");
	k_sleep(K_SECONDS(31));
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 4U,
		      "61 s after the last one LET THROUGH: passes, despite the drop in between");
}

ZTEST(traffic, test_dedup_is_off_at_zero_and_never_shapes_a_private_channel)
{
	uint8_t primary = meshtastic_channels_primary_index();
	meshtastic_Channel saved = *meshtastic_channels_get(primary);
	meshtastic_Channel private = saved;

	/* 0: off (the before() default). */
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_a, 2U, "interval 0: no dedup");

	/* On, but on a private channel (custom name + custom key): not shaped. */
	set_traffic(60U, 0U, 0U, 0U);
	strcpy(private.settings.name, "Private");
	private.settings.psk.size = 16U;
	memset(private.settings.psk.bytes, 0x42, 16U);
	zassert_ok(meshtastic_channels_set_slot(primary, &private), "");
	zassert_false(meshtastic_channels_is_well_known(primary), "a custom channel is not well-known");
	inject_position(PEER_B, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	inject_position(PEER_B, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_b, 2U, "private channel: no dedup (reference)");

	zassert_ok(meshtastic_channels_set_slot(primary, &saved), "restore");
	zassert_true(meshtastic_channels_is_well_known(primary), "");
}

ZTEST(traffic, test_a_tracker_may_refresh_a_duplicate_hourly_whatever_the_window)
{
	set_traffic(7200U, 0U, 0U, 0U);

	/* T announces itself as a TRACKER; A stays a plain client. */
	inject_nodeinfo(PEER_T, next_id++, meshtastic_Config_DeviceConfig_Role_TRACKER);
	inject_position(PEER_T, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_t, 1U, "");
	zassert_equal(delivered.position_from_a, 1U, "");

	k_sleep(K_SECONDS(3601));
	inject_position(PEER_T, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	inject_position(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++, LAT_A, LON_A);
	zassert_equal(delivered.position_from_t, 2U,
		      "tracker: the 2 h window is capped at 1 h, so an hour later it passes");
	zassert_equal(delivered.position_from_a, 1U,
		      "client: still inside its 2 h window, still dropped");
}

/* ==========================================================================
 * Rate limit
 * ========================================================================== */

ZTEST(traffic, test_rate_limit_drops_beyond_the_budget_then_reopens)
{
	meshtastic_TrafficManagementStats st;

	set_traffic(0U, 60U, 3U, 0U);

	for (int i = 0; i < 3; i++) {
		inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++);
	}
	zassert_equal(delivered.text_from_a, 3U, "three inside the budget");
	zassert_equal(drain_relayed(), 3U, "all three relayed");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++);
	zassert_equal(delivered.text_from_a, 3U, "the fourth is dropped");
	zassert_equal(drain_relayed(), 0U, "and not relayed");
	meshtastic_traffic_stats(&st);
	zassert_equal(st.rate_limit_drops, 1U, "");

	/* Another node is budgeted separately. */
	inject_text(PEER_B, MESHTASTIC_NODE_BROADCAST, next_id++);
	zassert_equal(delivered.total, 4U, "B's first packet passes");

	k_sleep(K_SECONDS(61));
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, next_id++);
	zassert_equal(delivered.text_from_a, 4U, "a new window: A passes again");
}

/* ==========================================================================
 * Unknown-packet filter
 * ========================================================================== */

ZTEST(traffic, test_undecodable_frames_beyond_the_threshold_stop_being_relayed)
{
	meshtastic_TrafficManagementStats st;

	set_traffic(0U, 0U, 0U, 2U);

	inject_undecodable(PEER_B, next_id++);
	inject_undecodable(PEER_B, next_id++);
	zassert_equal(drain_relayed(), 2U,
		      "an undecodable frame is still relayed (it may be for someone else)");

	inject_undecodable(PEER_B, next_id++);
	zassert_equal(drain_relayed(), 0U, "past the threshold the source is not relayed");
	meshtastic_traffic_stats(&st);
	zassert_equal(st.unknown_packet_drops, 1U, "");

	/* A different source is unaffected. */
	inject_undecodable(PEER_A, next_id++);
	zassert_equal(drain_relayed(), 1U, "");
}

/* ==========================================================================
 * Exemptions and refusals
 * ========================================================================== */

ZTEST(traffic, test_traffic_addressed_to_us_is_never_shaped)
{
	set_traffic(60U, 60U, 1U, 0U);

	inject_position(PEER_A, TEST_NODE_ID, next_id++, LAT_A, LON_A);
	inject_position(PEER_A, TEST_NODE_ID, next_id++, LAT_A, LON_A);
	inject_text(PEER_A, TEST_NODE_ID, next_id++);
	zassert_equal(delivered.position_from_a, 2U, "a duplicate unicast to us is delivered");
	zassert_equal(delivered.text_from_a, 1U, "over the budget, still delivered: it is for us");
}

ZTEST(traffic, test_a_config_enabling_direct_response_is_refused)
{
	meshtastic_ModuleConfig_TrafficManagementConfig cfg =
		meshtastic_ModuleConfig_TrafficManagementConfig_init_zero;
	struct meshtastic_traffic_settings s;

	cfg.position_min_interval_secs = 60U;
	cfg.nodeinfo_direct_response_max_hops = 1U;
	zassert_equal(meshtastic_traffic_validate(&cfg), -ENOTSUP,
		      "spoofed NodeInfo replies need key provenance this port lacks: refused");
	zassert_equal(meshtastic_traffic_set(&cfg), -ENOTSUP, "and so the set is refused");
	meshtastic_traffic_settings(&s);
	zassert_equal(s.position_min_interval_secs, 0U, "nothing of it was stored");

	cfg.nodeinfo_direct_response_max_hops = 0U;
	zassert_ok(meshtastic_traffic_set(&cfg), "without it, accepted");
	meshtastic_traffic_settings(&s);
	zassert_equal(s.position_min_interval_secs, 60U, "");

	/* The reference caps the counters at 60 so a saturated count always trips. */
	cfg.rate_limit_max_packets = 200U;
	cfg.unknown_packet_threshold = 200U;
	zassert_ok(meshtastic_traffic_set(&cfg), "");
	meshtastic_traffic_settings(&s);
	zassert_equal(s.rate_limit_max_packets, 60U, "capped at 60 (reference)");
	zassert_equal(s.unknown_packet_threshold, 60U, "capped at 60 (reference)");
}
