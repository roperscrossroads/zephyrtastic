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
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_position.h"

#define TEST_NODE_ID 0x0B0B0B0BU
#define PEER_ID      0x0C0C0C0CU
#define PRECISION    13U

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

	meshtastic_phoneapi_init(&phone, "phone", phone_q, PHONE_Q, NULL, NULL, NULL, NULL,
				 &phone_to, &phone_from);
	meshtastic_phoneapi_register(&phone);
	return NULL;
}

static void drain_radio(void)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(50)) == 0) {
	}
}

static void position_before(void *fixture)
{
	struct meshtastic_phoneapi_frame f;

	ARG_UNUSED(fixture);
	meshtastic_position_clear_fixed();
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
	zassert_false(take_position_frame(500U, &pkt, &got),
		      "a position the phone addressed to the node must stay on the node, "
		      "not go out as a unicast to our own id (got one to 0x%08x)", pkt.to);
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
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	phone_send_position(0U, me, &pos);
	zassert_false(take_position_frame(500U, &pkt, &got), "not transmitted");
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
	meshtastic_Position got;
	struct meshtastic_packet pkt;

	phone_send_position(me, PEER_ID, &pos);
	zassert_true(take_position_frame(1000U, &pkt, &got), "a unicast to a peer goes out");
	zassert_equal(pkt.to, PEER_ID, "");
	zassert_equal(meshtastic_position_get_current(&cur), -ENODATA,
		      "a position sent to someone else is not our position");
}
