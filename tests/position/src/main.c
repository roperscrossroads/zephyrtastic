/* SPDX-License-Identifier: GPL-3.0
 *
 * A position the phone hands the node, driven the way a phone drives it:
 * encoded ToRadio frames through meshtastic_phoneapi_handle_toradio(), the
 * whole stack behind it, the sim radio watched for what actually goes out
 * (agents-t2hb.13).
 *
 * The case that matters is Android's "provide phone location": every 30 s the
 * app sends a POSITION_APP packet addressed TO THE NODE ITSELF (to = my node
 * num, want_response false). Upstream Router::sendLocal() delivers anything
 * addressed to us locally and never transmits it; PositionModule then takes it
 * as the node's own position (time only while a fixed position is set) and
 * sets the clock from it at NTP quality. Before the fix this port had no local
 * delivery for it at all: the packet went to the radio as a unicast to our own
 * id, and the node never learned where it was.
 *
 * The neighbours pin the rest of upstream's rule: a phone BROADCAST is looped
 * back (adopted) and sent; a phone UNICAST to a peer is sent and not adopted.
 *
 * The second half is the beacon's cadence (agents-t2hb.2): the timer that
 * announces a position no fix callback drives (fixed, or phone-supplied) runs on
 * PositionConfig.position_broadcast_secs, falling back to the compiled default
 * (30 s in this image), re-read every cycle and re-armed by a write -- through
 * the setter and through a real admin set_config from the phone, which must also
 * not reboot the node.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_position.h"
#include "meshtastic/portnums.pb.h"

#define TEST_NODE_ID 0x0B0B0B0BU
#define PEER_ID      0x0C0C0C0CU
#define PRECISION    13U
#define SECONDARY_SLOT      1U
#define SECONDARY_PRECISION 20U

/* Well past the clock's pre-2020 floor, and distinct per test where it matters. */
#define PHONE_EPOCH 1790000000U

/* 37.7749, -122.4194 */
#define PHONE_LAT 377749000
#define PHONE_LON (-1224194000)

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

#define PHONE_Q 16U
static struct meshtastic_phoneapi_frame phone_q[PHONE_Q];
static struct meshtastic_phoneapi phone;
static meshtastic_ToRadio phone_to;
static meshtastic_FromRadio phone_from;
static uint32_t me;
static uint32_t next_id = 0x0B570001U;

static void *position_setup(void)
{
	static struct meshtastic_config cfg = {
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};
	meshtastic_Channel ch;
	uint8_t primary;

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	/* The stack derives its id from the (simulated) hardware and ignores
	 * cfg.node_id, so "to us" means this. */
	me = meshtastic_get_node_id();

	/* Share position on the primary at a known precision: a channel with no
	 * module_settings shares nothing (fail-closed), which would hide whether a
	 * broadcast was transmitted at all. */
	primary = meshtastic_channels_primary_index();
	ch = *meshtastic_channels_get(primary);
	ch.settings.has_module_settings = true;
	ch.settings.module_settings.position_precision = PRECISION;
	zassert_ok(meshtastic_config_store_set_channel(primary, &ch), "");
	zassert_ok(meshtastic_channels_set_slot(primary, &ch), "");

	/* A second, private channel at a finer precision: a request arriving on it
	 * must be answered at ITS precision (POS-3). Private key, so no public clamp. */
	{
		static const uint8_t psk[16] = {0x5a, 0x11, 0x7e, 0x02, 0x9c, 0x44, 0xd1, 0x38,
						0x0f, 0xa6, 0x71, 0xe2, 0x53, 0x8b, 0x2d, 0xc4};
		meshtastic_Channel sec = meshtastic_Channel_init_zero;

		sec.index = SECONDARY_SLOT;
		sec.role = meshtastic_Channel_Role_SECONDARY;
		sec.has_settings = true;
		strcpy(sec.settings.name, "fine");
		memcpy(sec.settings.psk.bytes, psk, sizeof(psk));
		sec.settings.psk.size = sizeof(psk);
		sec.settings.has_module_settings = true;
		sec.settings.module_settings.position_precision = SECONDARY_PRECISION;
		zassert_ok(meshtastic_config_store_set_channel(SECONDARY_SLOT, &sec), "");
		zassert_ok(meshtastic_channels_set_slot(SECONDARY_SLOT, &sec), "");
	}

	meshtastic_phoneapi_init(&phone, "phone", phone_q, PHONE_Q, NULL, NULL, NULL, NULL,
				 &phone_to, &phone_from);
	meshtastic_phoneapi_register(&phone);
	return NULL;
}

/* Wait for the sim radio to be back in receive: a frame injected while it is
 * still transmitting is refused. */
static void wait_rx_armed(void)
{
	for (int i = 0; i < 200 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(10);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "the sim radio never returned to receive");
}

static void drain_radio(void)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(50)) == 0) {
	}
	wait_rx_armed();
}

/* Read-modify-write of the smart-broadcast trio (agents-t2hb.3). */
static void set_smart(bool enabled, uint32_t distance_m, uint32_t min_interval_s)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	cfg.payload_variant.position.position_broadcast_smart_enabled = enabled;
	cfg.payload_variant.position.broadcast_smart_minimum_distance = distance_m;
	cfg.payload_variant.position.broadcast_smart_minimum_interval_secs = min_interval_s;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
	meshtastic_position_config_changed();
}

#define SEED_FLAGS                                                                                 \
	(meshtastic_Config_PositionConfig_PositionFlags_ALTITUDE |                                 \
	 meshtastic_Config_PositionConfig_PositionFlags_ALTITUDE_MSL |                             \
	 meshtastic_Config_PositionConfig_PositionFlags_SPEED |                                    \
	 meshtastic_Config_PositionConfig_PositionFlags_HEADING |                                  \
	 meshtastic_Config_PositionConfig_PositionFlags_DOP |                                      \
	 meshtastic_Config_PositionConfig_PositionFlags_SATINVIEW)

static void set_flags(uint32_t flags)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	cfg.payload_variant.position.position_flags = flags;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
}

static void set_role(meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg));
	cfg.payload_variant.device.role = role;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
}

static void position_before(void *fixture)
{
	struct meshtastic_phoneapi_frame f;

	ARG_UNUSED(fixture);
	meshtastic_position_clear_fixed();
	/* Cancel the beacon BEFORE draining, or a send already on its way lands in
	 * the next test's capture. */
	meshtastic_position_test_reset();
	(void)meshtastic_position_set_broadcast_secs(0U);
	set_smart(true, 100U, 300U); /* the seed: the reference's defaults */
	set_flags(SEED_FLAGS);
	set_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	meshtastic_position_test_reset();
	meshtastic_clock_test_reset();
	drain_radio();
	meshtastic_phoneapi_reset(&phone);
	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
	}
}

ZTEST_SUITE(position, NULL, position_setup, position_before, NULL, NULL);

/* Hand the node a ToRadio packet carrying @p pos on POSITION_APP, as the app
 * does. @p from is what the phone writes there (Android: its node num; the
 * Python CLI: 0). */
static void phone_send_position(uint32_t from, uint32_t to, const meshtastic_Position *pos)
{
	static uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	meshtastic_ToRadio to_radio = meshtastic_ToRadio_init_zero;
	meshtastic_MeshPacket *p = &to_radio.packet;
	pb_ostream_t os;

	to_radio.which_payload_variant = meshtastic_ToRadio_packet_tag;
	p->from = from;
	p->to = to;
	p->id = next_id++;
	p->channel = 0U;
	p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
	p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
	os = pb_ostream_from_buffer(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_Position_fields, pos), "Position encode");
	p->decoded.payload.size = (pb_size_t)os.bytes_written;

	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_ToRadio_fields, &to_radio), "ToRadio encode");
	meshtastic_phoneapi_handle_toradio(&phone, buf, os.bytes_written);
}

/* What Android's provide-location sends (AndroidMeshLocationManager ->
 * CommandSenderImpl.sendPosition). */
static meshtastic_Position phone_fix(void)
{
	meshtastic_Position pos = meshtastic_Position_init_zero;

	pos.has_latitude_i = true;
	pos.latitude_i = PHONE_LAT;
	pos.has_longitude_i = true;
	pos.longitude_i = PHONE_LON;
	pos.has_altitude = true;
	pos.altitude = 16;
	pos.time = PHONE_EPOCH;
	pos.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;
	return pos;
}

/* Next POSITION_APP frame on the sim radio within @p ms, decoded. */
static bool take_position_frame(uint32_t ms, struct meshtastic_packet *pkt,
				meshtastic_Position *pos)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	int64_t deadline = k_uptime_get() + ms;
	struct lora_sim_frame f;

	while (true) {
		int64_t left = deadline - k_uptime_get();
		pb_istream_t is;

		if (left <= 0 || lora_sim_take_tx(lora_dev, &f, K_MSEC(left)) != 0) {
			return false;
		}
		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, pkt, payload,
						  sizeof(payload)) != 0 ||
		    pkt->portnum != MESHTASTIC_PORT_POSITION) {
			continue;
		}
		*pos = (meshtastic_Position)meshtastic_Position_init_zero;
		is = pb_istream_from_buffer(pkt->payload, pkt->payload_len);
		zassert_true(pb_decode(&is, meshtastic_Position_fields, pos), "frame decode");
		return true;
	}
}

/* True if a POSITION_APP frame addressed to @p to reaches the sim radio within
 * @p ms. Other position frames (a fixed position's own beacon) are skipped. */
static bool position_frame_to(uint32_t to, uint32_t ms)
{
	int64_t deadline = k_uptime_get() + ms;
	struct meshtastic_packet pkt;
	meshtastic_Position got;

	while (true) {
		int64_t left = deadline - k_uptime_get();

		if (left <= 0 || !take_position_frame((uint32_t)left, &pkt, &got)) {
			return false;
		}
		if (pkt.to == to) {
			return true;
		}
	}
}

/* The QueueStatus the node answers a ToRadio packet with: its res, or 1. */
static int take_queue_status(void)
{
	struct meshtastic_phoneapi_frame f;

	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		pb_istream_t is = pb_istream_from_buffer(f.data, f.len);

		if (pb_decode(&is, meshtastic_FromRadio_fields, &from) &&
		    from.which_payload_variant == meshtastic_FromRadio_queueStatus_tag) {
			return from.queueStatus.res;
		}
	}
	return 1;
}

/* ---- to self: the provide-location stream ---------------------------------------- */

ZTEST(position, test_to_self_is_never_transmitted)
{
	meshtastic_Position pos = phone_fix();
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	phone_send_position(me, me, &pos);

	/* What does go out is the node's OWN announcement of its new position
	 * (agents-t2hb.2: the beacon owns a phone-supplied position), as a
	 * broadcast masked to the channel -- not the phone's packet. */
	zassert_true(take_position_frame(1000U, &pkt, &got),
		     "the first adopted position is announced at once");
	zassert_equal(pkt.to, MESHTASTIC_NODE_BROADCAST,
		      "a position the phone addressed to the node must stay on the node, "
		      "not go out as a unicast to our own id (got one to 0x%08x)", pkt.to);
	zassert_equal(got.precision_bits, PRECISION, "");
	zassert_false(position_frame_to(me, 500U), "and nothing addressed to ourselves");
	zassert_equal(take_queue_status(), 0, "the phone must still see its packet accepted");
}

ZTEST(position, test_to_self_becomes_the_local_position)
{
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;

	zassert_equal(meshtastic_position_get_current(&cur), -ENODATA,
		      "precondition: no position yet");
	phone_send_position(me, me, &pos);
	zassert_ok(meshtastic_position_get_current(&cur),
		   "the phone's fix must become the node's own position");
	zassert_equal(cur.latitude_i, PHONE_LAT);
	zassert_equal(cur.longitude_i, PHONE_LON);
	zassert_equal(cur.altitude, 16);
	zassert_equal(cur.location_source, meshtastic_Position_LocSource_LOC_EXTERNAL,
		      "the source the phone stated is kept");
}

/* The Python CLI's sendPosition() leaves `from` 0; the phone's own num and 0
 * both mean "from us" (upstream zeroes it in handleToRadio). */
ZTEST(position, test_to_self_with_from_zero_is_the_same)
{
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;

	phone_send_position(0U, me, &pos);
	zassert_false(position_frame_to(me, 1000U), "not transmitted");
	zassert_ok(meshtastic_position_get_current(&cur), "adopted");
	zassert_equal(cur.latitude_i, PHONE_LAT);
}

ZTEST(position, test_to_self_sets_the_clock_at_ntp_quality)
{
	meshtastic_Position pos = phone_fix();

	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NONE,
		      "precondition: clock unset");
	phone_send_position(me, me, &pos);
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NTP,
		      "phone time is NTP quality (upstream trySetRtc, isLocal)");
	zassert_within(meshtastic_clock_now_epoch(), PHONE_EPOCH, 2U);
}

/* Upstream: fixed_position => "Ignore own position update except time". The
 * fixed position keeps winning, and the phone's coordinates must not be parked
 * underneath it either: clearing the fixed position falls back to whatever the
 * node itself last knew, which the phone did not supply. */
ZTEST(position, test_to_self_while_fixed_takes_time_only)
{
	meshtastic_Position fixed = meshtastic_Position_init_zero;
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;

	fixed.has_latitude_i = true;
	fixed.latitude_i = 515000000;
	fixed.has_longitude_i = true;
	fixed.longitude_i = -1000000;
	meshtastic_position_set_fixed(&fixed); /* announces a broadcast of its own */

	phone_send_position(me, me, &pos);
	zassert_false(position_frame_to(me, 1000U), "not transmitted");
	zassert_ok(meshtastic_position_get_current(&cur), "");
	zassert_equal(cur.latitude_i, 515000000, "the fixed position still wins");
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NTP,
		      "but the phone's time is still taken");

	meshtastic_position_clear_fixed();
	zassert_equal(meshtastic_position_get_current(&cur), -ENODATA,
		      "the phone's coordinates were ignored, not stored behind the fix");
}

/* Android's empty-update filter, applied at the node: a packet with no
 * coordinates is a time-only update. It sets the clock, and must not install a
 * (0,0) position that the port would then advertise -- nor displace a real one.
 * (One clock write per test: a second NTP-quality write inside 30 min is refused
 * by the clock's re-apply gate, as upstream perhapsSetRTC refuses it.) */
ZTEST(position, test_to_self_without_coordinates_is_time_only)
{
	meshtastic_Position empty = meshtastic_Position_init_zero;
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;

	empty.time = PHONE_EPOCH;
	phone_send_position(me, me, &empty);
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NTP,
		      "the time is taken");
	zassert_equal(meshtastic_position_get_current(&cur), -ENODATA,
		      "no coordinates is not a position at (0,0)");

	phone_send_position(me, me, &pos);
	pos = (meshtastic_Position)meshtastic_Position_init_zero;
	phone_send_position(me, me, &pos);
	zassert_ok(meshtastic_position_get_current(&cur), "");
	zassert_equal(cur.latitude_i, PHONE_LAT, "a later empty packet leaves the fix alone");
}

/* ---- the neighbours: broadcast and unicast --------------------------------------- */

/* Upstream loops a phone broadcast back to the node (Router::sendLocal treats a
 * broadcast "as if we just received it ourself"), so PositionModule adopts it;
 * the copy that goes out is masked to the channel precision (POS-1). */
ZTEST(position, test_broadcast_is_adopted_and_sent_masked)
{
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	phone_send_position(me, MESHTASTIC_NODE_BROADCAST, &pos);
	zassert_true(take_position_frame(1000U, &pkt, &got), "a broadcast still goes out");
	zassert_equal(pkt.to, MESHTASTIC_NODE_BROADCAST, "");
	zassert_equal(got.precision_bits, PRECISION, "masked to the channel on the way out");
	zassert_not_equal(got.latitude_i, PHONE_LAT, "not at full precision");

	zassert_ok(meshtastic_position_get_current(&cur), "and the node adopts it");
	zassert_equal(cur.latitude_i, PHONE_LAT, "the local copy stays full precision");
}

/* Android's "request position" sends the phone's own fix to a peer with
 * want_response. That is the peer's business: it is transmitted, and it is not
 * a statement about where this node is (upstream does not loop back unicasts). */
ZTEST(position, test_unicast_to_peer_is_sent_and_not_adopted)
{
	meshtastic_Position pos = phone_fix();
	meshtastic_Position cur;

	phone_send_position(me, PEER_ID, &pos);
	zassert_true(position_frame_to(PEER_ID, 1000U), "a unicast to a peer goes out");
	zassert_equal(meshtastic_position_get_current(&cur), -ENODATA,
		      "a position sent to someone else is not our position");
}

/* ---- the beacon's cadence: position_broadcast_secs (agents-t2hb.2) --------------- */

/* Milliseconds until the next broadcast Position frame, or -1 within @p max_ms. */
static int64_t ms_to_next_broadcast(uint32_t max_ms)
{
	int64_t start = k_uptime_get();

	return position_frame_to(MESHTASTIC_NODE_BROADCAST, max_ms) ? k_uptime_get() - start
								     : -1;
}

/* The B1 cadence tests re-send one coordinate -- a stationary node -- so under
 * the stationary floor they would be pinning the floor. */
#define SKIP_UNDER_FLOOR()                                                                         \
	do {                                                                                       \
		if (CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC != 0) {                        \
			ztest_test_skip();                                                         \
		}                                                                                  \
	} while (0)

/* Frame timing on the sim radio carries airtime and a contention window, so a
 * cadence is asserted to within a few seconds, never to the millisecond. */
#define SLACK_MS 3000

static void assert_next_broadcast_in(uint32_t secs, const char *why)
{
	int64_t ms = ms_to_next_broadcast((secs * MSEC_PER_SEC) + SLACK_MS);

	zassert_true(ms >= 0, "%s: no broadcast within %u s", why, secs);
	zassert_true(ms >= ((int64_t)secs * MSEC_PER_SEC) - SLACK_MS,
		     "%s: broadcast after %lld ms, expected ~%u s", why, ms, secs);
}

ZTEST(position, test_beacon_unset_interval_is_the_compiled_default)
{
	meshtastic_Position pos = phone_fix();

	zassert_equal(meshtastic_position_broadcast_secs(),
		      CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC, "precondition");
	phone_send_position(me, me, &pos);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "announced at once");
	assert_next_broadcast_in(CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC,
				 "unset -> compiled default");
}

ZTEST(position, test_beacon_follows_the_stored_interval)
{
	meshtastic_Position pos = phone_fix();
	SKIP_UNDER_FLOOR();

	zassert_ok(meshtastic_position_set_broadcast_secs(10U));
	phone_send_position(me, me, &pos);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "announced at once");
	assert_next_broadcast_in(10U, "stored 10 s beats the compiled 30 s");
	assert_next_broadcast_in(10U, "and keeps it");
}

/* The phone refreshes every 30 s; a refresh updates what the beacon sends but
 * must not restart it, or a phone refreshing faster than the interval would
 * hold every broadcast off forever. */
ZTEST(position, test_phone_refreshes_do_not_restart_the_beacon)
{
	meshtastic_Position pos = phone_fix();
	int64_t start;
	SKIP_UNDER_FLOOR();

	zassert_ok(meshtastic_position_set_broadcast_secs(10U));
	phone_send_position(me, me, &pos);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "announced at once");
	start = k_uptime_get();
	k_msleep(4000);
	pos.latitude_i += 1000;
	phone_send_position(me, me, &pos);
	zassert_true(position_frame_to(MESHTASTIC_NODE_BROADCAST, 10000U), "");
	zassert_true(k_uptime_get() - start <= (10 * MSEC_PER_SEC) + SLACK_MS,
		     "the refresh at 4 s must not have pushed the broadcast past 10 s");
}

ZTEST(position, test_interval_write_rearms_the_beacon_live)
{
	meshtastic_Position pos = phone_fix();
	SKIP_UNDER_FLOOR();

	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	phone_send_position(me, me, &pos);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "announced at once");

	/* Two minutes out; a write to 8 s must not wait for that to expire. */
	zassert_ok(meshtastic_position_set_broadcast_secs(8U));
	assert_next_broadcast_in(8U, "the write re-arms the beacon now");
}

ZTEST(position, test_fixed_position_beacon_follows_the_interval)
{
	meshtastic_Position fixed = meshtastic_Position_init_zero;
	SKIP_UNDER_FLOOR();

	fixed.has_latitude_i = true;
	fixed.latitude_i = 515000000;
	fixed.has_longitude_i = true;
	fixed.longitude_i = -1000000;
	zassert_ok(meshtastic_position_set_broadcast_secs(10U));
	meshtastic_position_set_fixed(&fixed);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "set_fixed announces at once");
	assert_next_broadcast_in(10U, "a fixed position re-announces on the stored interval");
}

/* No position, no beacon: nothing to announce is not an empty broadcast. */
ZTEST(position, test_no_position_no_beacon)
{
	zassert_ok(meshtastic_position_set_broadcast_secs(5U));
	zassert_equal(ms_to_next_broadcast(8000U), -1, "");
}

/* The path a phone app actually takes: AdminMessage set_config(position) to
 * ourselves through the PhoneAPI. The admin module has no per-field code for the
 * section; the write must reach the store, re-arm the beacon, and -- unlike the
 * reference, which reboots on any position write -- not reboot (a reboot here
 * would end the test process). */
ZTEST(position, test_admin_set_config_position_applies_live)
{
	static uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	meshtastic_ToRadio to = meshtastic_ToRadio_init_zero;
	meshtastic_AdminMessage admin = meshtastic_AdminMessage_init_zero;
	meshtastic_Config cfg;
	meshtastic_Position pos = phone_fix();
	pb_ostream_t os;
	SKIP_UNDER_FLOOR();

	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	phone_send_position(me, me, &pos);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "announced at once");

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	cfg.payload_variant.position.position_broadcast_secs = 9U;
	admin.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	admin.payload_variant.set_config = cfg;

	to.which_payload_variant = meshtastic_ToRadio_packet_tag;
	to.packet.to = me;
	to.packet.id = next_id++;
	to.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	to.packet.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	os = pb_ostream_from_buffer(to.packet.decoded.payload.bytes,
				    sizeof(to.packet.decoded.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &admin), "");
	to.packet.decoded.payload.size = (pb_size_t)os.bytes_written;
	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_ToRadio_fields, &to), "");
	meshtastic_phoneapi_handle_toradio(&phone, buf, os.bytes_written);

	zassert_equal(meshtastic_position_broadcast_secs(), 9U, "the admin write reached the store");
	assert_next_broadcast_in(9U, "and the beacon, without a reboot");
}

/* ---- smart broadcast and the stationary floor (agents-t2hb.3) --------------------
 *
 * The reference: PositionModule::runOnce / handleNewPosition /
 * getDistanceTraveledSinceLastSend / positionUnchangedSinceLastSend. Positions
 * arrive the way a phone supplies them (to self), so each test controls exactly
 * where the node is and when it moved. PRECISION is 13 bits here -- a cell about
 * 5.8 km tall -- so "moved" means "crossed into another cell", as it does on the
 * air: both ends of the distance are snapped to that grid first. */

/* One degree of latitude at the reference's Earth radius, and the antimeridian
 * wrap: 179.9 E to 179.9 W is 0.2 deg, not 359.8. */
ZTEST(position, test_distance_matches_the_reference)
{
	float d;

	zassert_equal(meshtastic_position_distance_m(0, 0, 0, 0), 0.0f, "");
	d = meshtastic_position_distance_m(0, 0, 10000000, 0);
	zassert_within(d, 111106.9f, 2.0f, "1 deg of latitude = %f m", (double)d);
	d = meshtastic_position_distance_m(0, 1799000000, 0, -1799000000);
	zassert_within(d, 22221.4f, 5.0f, "across the antimeridian = %f m", (double)d);
	/* 60 N: a degree of longitude is about half as long. */
	d = meshtastic_position_distance_m(600000000, 0, 600000000, 10000000);
	zassert_within(d, 55553.0f, 60.0f, "1 deg of longitude at 60 N = %f m", (double)d);
}

static meshtastic_Position fix_at(int32_t lat, int32_t lon)
{
	meshtastic_Position pos = phone_fix();

	pos.latitude_i = lat;
	pos.longitude_i = lon;
	pos.time = 0U; /* the clock is not what these tests are about */
	return pos;
}

/* A cell centre at PRECISION, so small moves stay inside it. */
static void cell_centre(int32_t *lat, int32_t *lon)
{
	*lat = PHONE_LAT;
	*lon = PHONE_LON;
	meshtastic_position_truncate_latlon(lat, lon, PRECISION);
}

/* 0.2 deg of latitude: ~22 km, several cells away. */
#define FAR (2000000)

ZTEST(position, test_smart_move_broadcasts_early)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	meshtastic_Position b = fix_at(PHONE_LAT + FAR, PHONE_LON);
	int64_t ms;

	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	set_smart(true, 100U, 5U);
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "the first position goes out at once");

	k_msleep(6000);
	phone_send_position(me, me, &b);
	ms = ms_to_next_broadcast(SLACK_MS);
	zassert_true(ms >= 0, "a 22 km move past the 5 s minimum must go out now, not in 120 s");
}

/* The move happens 3 s in; the smart minimum is 20 s. The reference's 5 s tick
 * sends at the first tick past 20 s; this port's beacon sleeps until exactly then. */
ZTEST(position, test_smart_move_waits_out_the_minimum_interval)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	meshtastic_Position b = fix_at(PHONE_LAT + FAR, PHONE_LON);
	int64_t start;

	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	set_smart(true, 100U, 20U);
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");
	start = k_uptime_get();

	k_msleep(3000);
	phone_send_position(me, me, &b);
	zassert_equal(ms_to_next_broadcast(12000U), -1, "throttled until 20 s after the last send");
	zassert_true(position_frame_to(MESHTASTIC_NODE_BROADCAST, 8000U),
		     "sent when the throttle clears, without waiting for another position");
	zassert_within(k_uptime_get() - start, 20 * MSEC_PER_SEC, SLACK_MS, "");
}

ZTEST(position, test_smart_disabled_waits_for_the_interval)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	meshtastic_Position b = fix_at(PHONE_LAT + FAR, PHONE_LON);

	zassert_ok(meshtastic_position_set_broadcast_secs(30U));
	set_smart(false, 100U, 5U);
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");

	k_msleep(3000);
	phone_send_position(me, me, &b);
	zassert_equal(ms_to_next_broadcast(20000U), -1,
		      "smart off: a move waits for the periodic broadcast");
	zassert_true(ms_to_next_broadcast(10000U) >= 0, "which still comes at 30 s");
}

/* Both ends are snapped to the channel's grid before measuring, so 100 m inside
 * one 5.8 km cell is no move at all -- the air would show the same cell anyway. */
ZTEST(position, test_a_move_inside_one_cell_is_no_move)
{
	int32_t lat, lon;
	meshtastic_Position a, b;

	cell_centre(&lat, &lon);
	a = fix_at(lat, lon);
	b = fix_at(lat + 10000, lon); /* 0.001 deg, ~111 m */
	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	set_smart(true, 100U, 5U);
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");

	k_msleep(6000);
	phone_send_position(me, me, &b);
	zassert_equal(ms_to_next_broadcast(8000U), -1, "");
}

ZTEST(position, test_smart_distance_threshold_is_configurable)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	meshtastic_Position b = fix_at(PHONE_LAT + FAR, PHONE_LON);

	zassert_ok(meshtastic_position_set_broadcast_secs(120U));
	set_smart(true, 100000U, 5U); /* 100 km */
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");

	k_msleep(6000);
	phone_send_position(me, me, &b);
	zassert_equal(ms_to_next_broadcast(8000U), -1, "22 km is under a 100 km threshold");
}

/* ---- the stationary floor (scenario stationary_floor: 24 s) ---- */

#define SKIP_WITHOUT_FLOOR()                                                                       \
	do {                                                                                       \
		if (CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC == 0) {                        \
			ztest_test_skip();                                                         \
		}                                                                                  \
	} while (0)

ZTEST(position, test_stationary_holds_the_floor)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	int64_t start;

	SKIP_WITHOUT_FLOOR();
	zassert_ok(meshtastic_position_set_broadcast_secs(8U));
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");
	start = k_uptime_get();

	k_msleep(4000);
	phone_send_position(me, me, &a); /* the phone keeps saying "still here" */
	zassert_equal(ms_to_next_broadcast(12000U), -1,
		      "unchanged since the last broadcast: the 8 s interval is held to the floor");
	zassert_true(position_frame_to(MESHTASTIC_NODE_BROADCAST, 14000U), "");
	zassert_within(k_uptime_get() - start, CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC *
						       MSEC_PER_SEC, SLACK_MS, "");
}

ZTEST(position, test_leaving_the_cell_lifts_the_floor)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);
	meshtastic_Position b = fix_at(PHONE_LAT + FAR, PHONE_LON);
	int64_t start;

	SKIP_WITHOUT_FLOOR();
	zassert_ok(meshtastic_position_set_broadcast_secs(8U));
	set_smart(false, 100U, 5U); /* isolate the periodic path */
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");
	start = k_uptime_get();

	k_msleep(3000);
	phone_send_position(me, me, &b);
	zassert_true(position_frame_to(MESHTASTIC_NODE_BROADCAST, 8000U), "");
	zassert_within(k_uptime_get() - start, 8 * MSEC_PER_SEC, SLACK_MS,
		       "moved: the plain interval applies again");
}

/* The reference: fixed_position holds the floor for every role -- pinning
 * yourself forfeits the exceptions. */
ZTEST(position, test_fixed_position_holds_the_floor)
{
	meshtastic_Position fixed = fix_at(515000000, -1000000);

	SKIP_WITHOUT_FLOOR();
	zassert_ok(meshtastic_position_set_broadcast_secs(8U));
	set_role(meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND);
	meshtastic_position_set_fixed(&fixed);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "set_fixed announces at once");
	assert_next_broadcast_in(CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC,
				 "then only at the floor, even for LOST_AND_FOUND");
}

ZTEST(position, test_lost_and_found_is_exempt_from_the_floor)
{
	meshtastic_Position a = fix_at(PHONE_LAT, PHONE_LON);

	SKIP_WITHOUT_FLOOR();
	zassert_ok(meshtastic_position_set_broadcast_secs(8U));
	set_role(meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND);
	phone_send_position(me, me, &a);
	zassert_true(ms_to_next_broadcast(1000U) >= 0, "");
	assert_next_broadcast_in(8U, "a lost node keeps its interval though it has not moved");
}

/* ---- answering a peer's request (POS-3, POS-11, POS-13) --------------------------- */

static uint32_t req_id = 0x0B5E0001U;

/* A peer's POSITION_APP want_response unicast to us, on @p slot, as it arrives
 * over the air. @p pos NULL sends the Python CLI's empty Position. */
static uint32_t peer_request(uint8_t slot, const meshtastic_Position *pos)
{
	static uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_Position empty = meshtastic_Position_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));
	uint32_t wire_len;
	struct meshtastic_packet pkt;

	zassert_true(pb_encode(&os, meshtastic_Position_fields, pos ? pos : &empty), "");
	pkt = (struct meshtastic_packet){
		.from = PEER_ID,
		.to = me,
		.id = req_id++,
		.portnum = MESHTASTIC_PORT_POSITION,
		.payload = payload,
		.payload_len = os.bytes_written,
		.want_response = true,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = slot,
	};
	zassert_ok(meshtastic_build_wire_packet(&pkt, wire, &wire_len), "");
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, 8), "");
	return pkt.id;
}

/* The next frame to PEER_ID on @p port within @p ms, decoded; its payload is
 * left in @p payload. */
static bool take_frame_to_peer(uint32_t port, uint32_t ms, struct meshtastic_packet *pkt,
			       uint8_t *payload, size_t payload_size)
{
	int64_t deadline = k_uptime_get() + ms;
	struct lora_sim_frame f;

	while (true) {
		int64_t left = deadline - k_uptime_get();

		if (left <= 0 || lora_sim_take_tx(lora_dev, &f, K_MSEC(left)) != 0) {
			return false;
		}
		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, pkt, payload,
						  payload_size) == 0 &&
		    pkt->to == PEER_ID && pkt->portnum == port) {
			return true;
		}
	}
}

static bool take_position_reply(uint32_t ms, uint32_t request_id, meshtastic_Position *pos)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_packet pkt;
	pb_istream_t is;

	if (!take_frame_to_peer(MESHTASTIC_PORT_POSITION, ms, &pkt, payload, sizeof(payload))) {
		return false;
	}
	zassert_equal(pkt.request_id, request_id, "the reply must name the request");
	*pos = (meshtastic_Position)meshtastic_Position_init_zero;
	is = pb_istream_from_buffer(pkt.payload, pkt.payload_len);
	zassert_true(pb_decode(&is, meshtastic_Position_fields, pos), "");
	return true;
}

/* -1 when no ROUTING frame to the peer arrives, else its error_reason. */
static int take_routing_error(uint32_t ms, uint32_t request_id)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_packet pkt;
	meshtastic_Routing r = meshtastic_Routing_init_zero;
	pb_istream_t is;

	if (!take_frame_to_peer(MESHTASTIC_PORT_ROUTING, ms, &pkt, payload, sizeof(payload))) {
		return -1;
	}
	zassert_equal(pkt.request_id, request_id, "the NAK must name the request");
	is = pb_istream_from_buffer(pkt.payload, pkt.payload_len);
	zassert_true(pb_decode(&is, meshtastic_Routing_fields, &r), "");
	return (int)r.error_reason;
}

static void have_a_fixed_position(void)
{
	meshtastic_Position fixed = fix_at(PHONE_LAT, PHONE_LON);

	meshtastic_position_set_fixed(&fixed);
	drain_radio(); /* its own announcement */
}

/* POS-3: the reply goes back on the request's channel and must be masked for it.
 * It used to be masked for slot 0 (13 bits) and then relabelled 20 on the way out,
 * so the app drew a 20-bit accuracy circle round a 13-bit cell. */
ZTEST(position, test_reply_is_masked_for_the_request_channel)
{
	meshtastic_Position got;
	int32_t lat = PHONE_LAT, lon = PHONE_LON;
	uint32_t id;

	have_a_fixed_position();
	id = peer_request(SECONDARY_SLOT, NULL);
	zassert_true(take_position_reply(2000U, id, &got), "a request with a position is answered");
	zassert_equal(got.precision_bits, SECONDARY_PRECISION, "labelled for the request's channel");
	meshtastic_position_truncate_latlon(&lat, &lon, SECONDARY_PRECISION);
	zassert_equal(got.latitude_i, lat, "and actually masked to it, not to slot 0's 13 bits");
	zassert_equal(got.longitude_i, lon, "");
}

/* Nothing to share: NO_RESPONSE, which the Python CLI's --request-position waits
 * for (reference MeshModule: no module replied, none asked to ignore). */
ZTEST(position, test_request_without_a_position_gets_no_response)
{
	uint32_t id = peer_request(meshtastic_channels_primary_index(), NULL);

	zassert_equal(take_routing_error(2000U, id), meshtastic_Routing_Error_NO_RESPONSE, "");
}

/* POS-11: that unanswerable request must not use up the reply window. */
ZTEST(position, test_an_unanswered_request_does_not_consume_the_window)
{
	meshtastic_Position got;
	uint32_t id;

	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_equal(take_routing_error(2000U, id), meshtastic_Routing_Error_NO_RESPONSE, "");

	have_a_fixed_position();
	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_true(take_position_reply(2000U, id, &got),
		     "the next request, now answerable, is answered at once");
}

/* A second answerable request inside the window is IGNORED: no reply and no NAK
 * (the reference sets ignoreRequest). */
ZTEST(position, test_a_throttled_request_is_ignored_silently)
{
	meshtastic_Position got;
	uint32_t id;

	have_a_fixed_position();
	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_true(take_position_reply(2000U, id, &got), "");

	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_false(take_position_reply(1500U, id, &got), "throttled");
	zassert_equal(take_routing_error(500U, id), -1, "and not NAKed");
}

ZTEST(position, test_lost_and_found_answers_every_request)
{
	meshtastic_Position got;
	uint32_t id;

	set_role(meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND);
	have_a_fixed_position();
	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_true(take_position_reply(2000U, id, &got), "");
	id = peer_request(meshtastic_channels_primary_index(), NULL);
	zassert_true(take_position_reply(2000U, id, &got), "a lost node is never throttled");
}

/* POS-13: Android's request carries the requester's fix and time. A GNSS-sourced
 * one on the primary is mesh time like any other position; it used to be dropped
 * before decode because it was a request. */
ZTEST(position, test_a_request_carrying_a_position_is_also_a_position)
{
	meshtastic_Position theirs = fix_at(PHONE_LAT, PHONE_LON);

	theirs.time = PHONE_EPOCH;
	theirs.location_source = meshtastic_Position_LocSource_LOC_INTERNAL;
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NONE, "");
	(void)peer_request(meshtastic_channels_primary_index(), &theirs);
	k_msleep(500);
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NET,
		      "its time is mesh time");
}

/* `time` is the clock at send, but only a clock worth vouching for: NTP or GPS.
 * Unset, mesh-relayed (NET) or restored-from-flash time goes out as 0, as the
 * reference strips it (allocPositionPacket). */
ZTEST(position, test_position_time_needs_an_ntp_or_better_clock)
{
	meshtastic_Position fixed = fix_at(515000000, -1000000);
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	meshtastic_position_set_fixed(&fixed);
	zassert_true(take_position_frame(1000U, &pkt, &got), "");
	zassert_equal(got.time, 0U, "no clock: no time");

	meshtastic_clock_set_epoch(PHONE_EPOCH, MESHTASTIC_CLOCK_QUALITY_NET);
	meshtastic_position_set_fixed(&fixed);
	zassert_true(take_position_frame(1000U, &pkt, &got), "");
	zassert_equal(got.time, 0U, "mesh-relayed time is not ours to repeat");

	meshtastic_clock_set_epoch(PHONE_EPOCH, MESHTASTIC_CLOCK_QUALITY_NTP);
	meshtastic_position_set_fixed(&fixed);
	zassert_true(take_position_frame(1000U, &pkt, &got), "");
	zassert_within(got.time, PHONE_EPOCH, 3U, "an NTP-quality clock is sent");
}

/* ---- position_flags: which optional fields go out ------------------------------- */

/* A phone fix carrying every field the flags can select, each a distinct value. */
static meshtastic_Position rich_fix(void)
{
	meshtastic_Position pos = phone_fix();

	pos.has_altitude = true;
	pos.altitude = 101;
	pos.has_altitude_hae = true;
	pos.altitude_hae = 149;
	pos.has_altitude_geoidal_separation = true;
	pos.altitude_geoidal_separation = 48;
	pos.PDOP = 170;
	pos.HDOP = 120;
	pos.VDOP = 130;
	pos.sats_in_view = 9;
	pos.timestamp = PHONE_EPOCH - 5U;
	pos.has_ground_speed = true;
	pos.ground_speed = 7;
	pos.has_ground_track = true;
	pos.ground_track = 12340;
	pos.fix_quality = 1;
	pos.fix_type = 3;
	return pos;
}

static meshtastic_Position broadcast_with_flags(uint32_t flags)
{
	meshtastic_Position pos = rich_fix();
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	set_flags(flags);
	phone_send_position(me, me, &pos);
	zassert_true(take_position_frame(1000U, &pkt, &got), "the first position is announced");
	zassert_equal(pkt.to, MESHTASTIC_NODE_BROADCAST, "");
	zassert_true(got.has_latitude_i && got.has_longitude_i, "lat/lon always");
	zassert_equal(got.precision_bits, PRECISION, "precision always");
	zassert_equal(got.location_source, meshtastic_Position_LocSource_LOC_EXTERNAL,
		      "source always");
	zassert_equal(got.fix_quality, 0U, "never flag-selected, never sent");
	zassert_equal(got.fix_type, 0U, "");
	return got;
}

/* The reference's default: ALTITUDE|ALTITUDE_MSL|SPEED|HEADING|DOP|SATINVIEW. */
ZTEST(position, test_default_flags_select_the_reference_default_fields)
{
	meshtastic_Position got = broadcast_with_flags(SEED_FLAGS);

	zassert_true(got.has_altitude && got.altitude == 101, "MSL altitude");
	zassert_false(got.has_altitude_hae, "not HAE when MSL is asked for");
	zassert_false(got.has_altitude_geoidal_separation, "");
	zassert_equal(got.PDOP, 170U, "DOP without HVDOP is PDOP");
	zassert_equal(got.HDOP, 0U, "");
	zassert_equal(got.sats_in_view, 9U, "");
	zassert_true(got.has_ground_speed && got.ground_speed == 7, "");
	zassert_true(got.has_ground_track && got.ground_track == 12340, "");
	zassert_equal(got.timestamp, 0U, "TIMESTAMP not asked for");
	zassert_equal(got.seq_number, 0U, "SEQ_NO not asked for");
}

ZTEST(position, test_zero_flags_send_only_the_unconditional_fields)
{
	meshtastic_Position got = broadcast_with_flags(0U);

	zassert_false(got.has_altitude || got.has_altitude_hae, "");
	zassert_equal(got.PDOP + got.HDOP + got.VDOP, 0U, "");
	zassert_equal(got.sats_in_view, 0U, "");
	zassert_false(got.has_ground_speed || got.has_ground_track, "");
}

ZTEST(position, test_the_other_flag_branches)
{
	meshtastic_Position got = broadcast_with_flags(
		meshtastic_Config_PositionConfig_PositionFlags_ALTITUDE |
		meshtastic_Config_PositionConfig_PositionFlags_GEOIDAL_SEPARATION |
		meshtastic_Config_PositionConfig_PositionFlags_DOP |
		meshtastic_Config_PositionConfig_PositionFlags_HVDOP |
		meshtastic_Config_PositionConfig_PositionFlags_TIMESTAMP |
		meshtastic_Config_PositionConfig_PositionFlags_SEQ_NO);

	zassert_false(got.has_altitude, "ALTITUDE without MSL is HAE...");
	zassert_true(got.has_altitude_hae && got.altitude_hae == 149, "");
	zassert_true(got.has_altitude_geoidal_separation && got.altitude_geoidal_separation == 48,
		     "");
	zassert_equal(got.HDOP, 120U, "HVDOP: HDOP and VDOP");
	zassert_equal(got.VDOP, 130U, "");
	zassert_equal(got.PDOP, 0U, "... not PDOP");
	zassert_equal(got.timestamp, PHONE_EPOCH - 5U, "");
	zassert_true(got.seq_number != 0U, "");
	zassert_equal(got.sats_in_view, 0U, "");
}

/* A node flashed from a build that never read these fields carries them as
 * zeros. A position section nobody ever wrote (no write-stamp) follows the
 * seed after the load; a written one keeps what was written, 0 included. */
ZTEST(position, test_unstamped_section_follows_the_seed_after_a_load)
{
	uint8_t rec[256];
	int rec_len;
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	cfg.payload_variant.position.position_flags = 0U;
	cfg.payload_variant.position.position_broadcast_smart_enabled = false;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
	rec_len = meshtastic_config_store_setting_get("config/position", rec, sizeof(rec));
	zassert_true(rec_len > 0, "record encode (%d)", rec_len);

	/* The load on an upgraded node: the zeroed record lands, its stamp absent. */
	zassert_ok(meshtastic_config_store_setting_set("config/position", rec, (size_t)rec_len));
	zassert_ok(meshtastic_config_store_setting_set("hlc/config/position", rec, 0U));
	zassert_ok(meshtastic_config_store_apply_core());
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	zassert_equal(cfg.payload_variant.position.position_flags, SEED_FLAGS, "seeded");
	zassert_true(cfg.payload_variant.position.position_broadcast_smart_enabled, "seeded");

	/* Written (stamped) zeros survive the same apply. */
	set_flags(0U);
	zassert_ok(meshtastic_config_store_apply_core());
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	zassert_equal(cfg.payload_variant.position.position_flags, 0U, "a written 0 stays 0");
}
