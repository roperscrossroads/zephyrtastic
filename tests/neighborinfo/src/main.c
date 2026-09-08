/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Neighbor info (agents-dnr4.19), driven through the sim radio.
 *
 * In the reference's terms (NeighborInfoModule.cpp):
 *   - a packet heard with hop_limit == hop_start names a direct neighbor and its
 *     SNR; a relayed one (hop_limit < hop_start) does not;
 *   - enabled, the table is broadcast a full interval after enabling and every
 *     interval after, as a NeighborInfo on port 71 from us -- on the air when
 *     transmit_over_lora is set AND the primary channel is not the default
 *     channel, otherwise not on the air at all;
 *   - a neighbor silent for twice its interval is forgotten;
 *   - a unicast want_response is answered with the table, at most once per
 *     suppression window, and never while disabled.
 *
 * The suite's primary channel is a custom name + custom key, so the LoRa gate
 * is open; one test swaps in the default channel to prove it closes.
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
#include "meshtastic_core.h"
#include "meshtastic_neighborinfo.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU

#define PEER_A 0x0B000001U
#define PEER_B 0x0B000002U
#define PEER_C 0x0B000003U
#define PEER_R 0x0B0000AAU /* the requester */

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* A key that is not the default one, so the primary channel is not "the
 * default channel" and NeighborInfo may go on the air. */
static const uint8_t test_psk[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
				     0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x02};

/* ==========================================================================
 * Frame construction / capture
 * ========================================================================== */

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

/* Inject a frame from @p from. hop_limit == hop_start is "heard directly";
 * hop_limit < hop_start is "came through a relay". */
static void inject(uint32_t from, uint32_t to, uint32_t id, uint32_t portnum,
		   const uint8_t *payload, size_t payload_len, bool want_response,
		   uint8_t hop_start, uint8_t hop_limit, int8_t snr)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
		.want_response = want_response,
		.hop_limit = hop_limit,
		.hop_start = hop_start,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len),
		   "build_wire_packet failed");
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, snr), "inject failed");
	k_msleep(100); /* let the RX thread run the module */
}

static void inject_text(uint32_t from, uint32_t id, uint8_t hop_start, uint8_t hop_limit,
			int8_t snr)
{
	static const char text[] = "hi";

	inject(from, MESHTASTIC_NODE_BROADCAST, id, MESHTASTIC_PORT_TEXT_MESSAGE,
	       (const uint8_t *)text, sizeof(text) - 1U, false, hop_start, hop_limit, snr);
}

/* A peer's NeighborInfo (its own table), or a request for ours when
 * @p want_response and @p to is us. */
static void inject_neighborinfo(uint32_t from, uint32_t to, uint32_t id, uint32_t interval_secs,
				bool want_response, int8_t snr)
{
	meshtastic_NeighborInfo ni = meshtastic_NeighborInfo_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));

	ni.node_id = from;
	ni.last_sent_by_id = from;
	ni.node_broadcast_interval_secs = interval_secs;
	ni.neighbors_count = 1U;
	ni.neighbors[0].node_id = 0x0C000001U;
	ni.neighbors[0].snr = 5.0f;
	zassert_true(pb_encode(&stream, meshtastic_NeighborInfo_fields, &ni), "encode failed");

	inject(from, to, id, MESHTASTIC_PORT_NEIGHBORINFO, payload, stream.bytes_written,
	       want_response, 3U, 3U, snr);
}

/* Only the default-table scenario observes frames; the small-table one is
 * about the table alone, and twister's -Werror refuses an unused helper. */
#if CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE != 2

struct captured {
	uint32_t count; /* NeighborInfo frames THIS node originated */
	uint32_t to;
	uint32_t request_id;
	meshtastic_NeighborInfo info; /* decoded payload of the last one */
};

static struct captured drain_tx(k_timeout_t settle)
{
	struct lora_sim_frame f;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct captured c = {0};

	k_sleep(settle);

	while (lora_sim_take_tx(lora_dev, &f, K_NO_WAIT) == 0) {
		pb_istream_t is;

		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &decoded, payload,
						  sizeof(payload)) != 0) {
			continue;
		}
		if (decoded.from != TEST_NODE_ID ||
		    decoded.portnum != MESHTASTIC_PORT_NEIGHBORINFO) {
			continue;
		}
		c.count++;
		c.to = decoded.to;
		c.request_id = decoded.request_id;
		c.info = (meshtastic_NeighborInfo)meshtastic_NeighborInfo_init_zero;
		is = pb_istream_from_buffer(decoded.payload, decoded.payload_len);
		zassert_true(pb_decode(&is, meshtastic_NeighborInfo_fields, &c.info),
			     "our own NeighborInfo must decode");
	}
	return c;
}

static bool info_lists(const meshtastic_NeighborInfo *info, uint32_t node, float *snr)
{
	for (pb_size_t i = 0; i < info->neighbors_count; i++) {
		if (info->neighbors[i].node_id == node) {
			if (snr != NULL) {
				*snr = info->neighbors[i].snr;
			}
			return true;
		}
	}
	return false;
}

#endif /* TABLE_SIZE != 2 */

static bool table_has(uint32_t node, struct meshtastic_neighborinfo_entry *out)
{
	struct meshtastic_neighborinfo_entry e;

	for (size_t i = 0; i < CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE; i++) {
		if (meshtastic_neighborinfo_at(i, &e) && e.node == node) {
			if (out != NULL) {
				*out = e;
			}
			return true;
		}
	}
	return false;
}

#define SETTLE K_MSEC(500)
#define INTERVAL_S CONFIG_MESHTASTIC_NEIGHBORINFO_INTERVAL_SEC

static void *neighborinfo_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = test_psk,
		.psk_len = sizeof(test_psk),
		.channel_name = "TestNet",
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	zassert_false(meshtastic_channels_is_default(meshtastic_channels_primary_index()),
		      "the suite's primary channel must not be the default channel");
	return NULL;
}

static void neighborinfo_before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(meshtastic_neighborinfo_set(false, 0U, true), "disable failed");
	meshtastic_neighborinfo_reset();
	k_msleep(200);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(neighborinfo, NULL, neighborinfo_setup, neighborinfo_before, NULL, NULL);

/* ==========================================================================
 * The table
 * ========================================================================== */

ZTEST(neighborinfo, test_direct_packets_make_neighbors_relayed_ones_do_not)
{
	struct meshtastic_neighborinfo_entry e;

	zassert_equal(meshtastic_neighborinfo_count(), 0U, "empty at start");

	inject_text(PEER_A, 0x1A000001U, 3U, 3U, 7);
	zassert_equal(meshtastic_neighborinfo_count(), 1U, "a direct packet adds its sender");
	zassert_true(table_has(PEER_A, &e), "A is a neighbor");
	zassert_within(e.snr, 7.0f, 0.01f, "with the SNR it was heard at");
	zassert_equal(e.interval_secs, INTERVAL_S, "assumed to broadcast as often as we do");

	inject_text(PEER_B, 0x1A000002U, 3U, 2U, 9);
	zassert_equal(meshtastic_neighborinfo_count(), 1U,
		      "a packet that came through a relay names no neighbor");
	zassert_false(table_has(PEER_B, NULL), "B is two hops away");

	inject_text(PEER_C, 0x1A000003U, 0U, 0U, 9);
	zassert_false(table_has(PEER_C, NULL),
		      "hop_start 0 is 'unknown' (old firmware), never 'adjacent'");

	inject_text(PEER_A, 0x1A000004U, 3U, 3U, 2);
	zassert_equal(meshtastic_neighborinfo_count(), 1U, "same neighbor, same slot");
	zassert_true(table_has(PEER_A, &e), "");
	zassert_within(e.snr, 2.0f, 0.01f, "the latest SNR replaces the older");
}

ZTEST(neighborinfo, test_a_neighbors_own_neighborinfo_tells_us_its_interval)
{
	struct meshtastic_neighborinfo_entry e;

	inject_neighborinfo(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x1B000001U, 3600U, false, 4);
	zassert_true(table_has(PEER_A, &e), "the sender of a NeighborInfo is a neighbor too");
	zassert_equal(e.interval_secs, 3600U, "and its stated interval is recorded");
	zassert_within(e.snr, 4.0f, 0.01f, "");
}

ZTEST(neighborinfo, test_a_silent_neighbor_is_forgotten_after_twice_its_interval)
{
	inject_text(PEER_A, 0x1C000001U, 3U, 3U, 6);
	zassert_equal(meshtastic_neighborinfo_count(), 1U, "");

	k_sleep(K_SECONDS(INTERVAL_S * 2 - 5));
	zassert_equal(meshtastic_neighborinfo_count(), 1U, "still there inside 2x its interval");

	k_sleep(K_SECONDS(10));
	zassert_equal(meshtastic_neighborinfo_count(), 0U,
		      "silent for more than twice its interval: forgotten");
}

#if CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE == 2

ZTEST(neighborinfo, test_a_full_table_replaces_the_least_recently_heard)
{
	inject_text(PEER_A, 0x1D000001U, 3U, 3U, 1);
	inject_text(PEER_B, 0x1D000002U, 3U, 3U, 2);
	inject_text(PEER_A, 0x1D000003U, 3U, 3U, 3); /* A is now the most recent */
	inject_text(PEER_C, 0x1D000004U, 3U, 3U, 4);

	zassert_equal(meshtastic_neighborinfo_count(), 2U, "the table holds two");
	zassert_true(table_has(PEER_A, NULL), "A, heard most recently, stays");
	zassert_false(table_has(PEER_B, NULL), "B, the least recently heard, is replaced");
	zassert_true(table_has(PEER_C, NULL), "by C");
}

#else /* the default table */

/* ==========================================================================
 * Broadcasting
 * ========================================================================== */

ZTEST(neighborinfo, test_disabled_or_empty_sends_nothing)
{
	struct captured c;

	inject_text(PEER_A, 0x2A000001U, 3U, 3U, 5);
	/* Disabled (the before() default) with a neighbor: nothing, ever. */
	c = drain_tx(K_SECONDS(INTERVAL_S + 5));
	zassert_equal(c.count, 0U, "disabled: no broadcast");
	zassert_equal(meshtastic_neighborinfo_send(), 0, "but an explicit send is honoured");
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 1U, "");

	/* Enabled with no neighbors: reference sends nothing ("only send
	 * neighbours if we have some"). */
	meshtastic_neighborinfo_reset();
	zassert_ok(meshtastic_neighborinfo_set(true, 0U, true), "enable failed");
	zassert_equal(meshtastic_neighborinfo_send(), -ENODATA, "empty table: nothing to say");
	c = drain_tx(K_SECONDS(INTERVAL_S + 5));
	zassert_equal(c.count, 0U, "enabled but empty: no broadcast");
}

ZTEST(neighborinfo, test_enabled_broadcasts_the_table_every_interval)
{
	struct captured c;
	float snr = 0.0f;

	inject_text(PEER_A, 0x2B000001U, 3U, 3U, 7);
	inject_text(PEER_B, 0x2B000002U, 3U, 3U, -3);
	zassert_ok(meshtastic_neighborinfo_set(true, 0U, true), "enable failed");

	/* Reference setIntervalFromNow: a full interval first, not immediately. */
	c = drain_tx(K_SECONDS(INTERVAL_S / 2));
	zassert_equal(c.count, 0U, "nothing before the first interval elapses");

	c = drain_tx(K_SECONDS((INTERVAL_S / 2) + 2));
	zassert_equal(c.count, 1U, "one broadcast at the interval");
	zassert_equal(c.to, MESHTASTIC_NODE_BROADCAST, "a broadcast");
	zassert_equal(c.info.node_id, TEST_NODE_ID, "about us");
	zassert_equal(c.info.last_sent_by_id, TEST_NODE_ID, "sent by us");
	zassert_equal(c.info.node_broadcast_interval_secs, INTERVAL_S, "carrying our interval");
	zassert_equal(c.info.neighbors_count, 2U, "both neighbors");
	zassert_true(info_lists(&c.info, PEER_A, &snr), "A listed");
	zassert_within(snr, 7.0f, 0.01f, "with its SNR");
	zassert_true(info_lists(&c.info, PEER_B, &snr), "B listed");
	zassert_within(snr, -3.0f, 0.01f, "with its SNR");
	zassert_equal(c.info.neighbors[0].last_rx_time, 0U,
		      "local-only fields are not sent (reference)");

	/* Keep them fresh (they would expire at 2x the interval) and see the
	 * next cycle. */
	inject_text(PEER_A, 0x2B000003U, 3U, 3U, 7);
	inject_text(PEER_B, 0x2B000004U, 3U, 3U, -3);
	c = drain_tx(K_SECONDS(INTERVAL_S / 2));
	zassert_equal(c.count, 0U, "nothing inside the interval");
	c = drain_tx(K_SECONDS((INTERVAL_S / 2) + 2));
	zassert_equal(c.count, 1U, "and one at the next interval");

	/* Disabling cancels the cycle. */
	zassert_ok(meshtastic_neighborinfo_set(false, 0U, true), "disable failed");
	inject_text(PEER_A, 0x2B000005U, 3U, 3U, 7);
	c = drain_tx(K_SECONDS(INTERVAL_S + 5));
	zassert_equal(c.count, 0U, "disabled: the cycle stops");
}

ZTEST(neighborinfo, test_a_stored_interval_below_the_minimum_falls_back_to_the_default)
{
	struct meshtastic_neighborinfo_settings s;

	zassert_ok(meshtastic_neighborinfo_set(true, CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC - 1,
					       true),
		   "");
	meshtastic_neighborinfo_settings(&s);
	zassert_equal(s.interval_secs, INTERVAL_S,
		      "below the minimum: the default, not the minimum (reference)");

	zassert_ok(meshtastic_neighborinfo_set(true, CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC + 7,
					       true),
		   "");
	meshtastic_neighborinfo_settings(&s);
	zassert_equal(s.interval_secs, CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC + 7,
		      "at or above the minimum: as stored");
}

/* ==========================================================================
 * The LoRa gate
 * ========================================================================== */

ZTEST(neighborinfo, test_transmit_over_lora_off_keeps_it_off_the_air)
{
	struct meshtastic_neighborinfo_settings s;
	struct captured c;

	inject_text(PEER_A, 0x2C000001U, 3U, 3U, 5);
	zassert_ok(meshtastic_neighborinfo_set(true, 0U, false), "");
	meshtastic_neighborinfo_settings(&s);
	zassert_false(s.lora_allowed, "");

	zassert_equal(meshtastic_neighborinfo_send(), 0,
		      "the send succeeds -- it went to the phone (reference NO_LORA)");
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 0U, "but nothing is on the air");
}

ZTEST(neighborinfo, test_the_default_channel_keeps_it_off_the_air)
{
	struct meshtastic_neighborinfo_settings s;
	struct captured c;
	uint8_t primary = meshtastic_channels_primary_index();
	meshtastic_Channel saved = *meshtastic_channels_get(primary);
	meshtastic_Channel dflt = saved;

	inject_text(PEER_A, 0x2D000001U, 3U, 3U, 5);
	zassert_ok(meshtastic_neighborinfo_set(true, 0U, true), "");
	meshtastic_neighborinfo_settings(&s);
	zassert_true(s.lora_allowed, "custom channel: allowed");

	/* The public channel: no name (it takes the preset's) and the 1-byte
	 * default key index. */
	dflt.settings.name[0] = '\0';
	dflt.settings.psk.size = 1U;
	dflt.settings.psk.bytes[0] = 1U;
	zassert_ok(meshtastic_channels_set_slot(primary, &dflt), "set default channel");
	zassert_true(meshtastic_channels_is_default(primary), "that IS the default channel");

	meshtastic_neighborinfo_settings(&s);
	zassert_true(s.transmit_over_lora, "the flag is still set...");
	zassert_false(s.lora_allowed, "...but the default channel closes the gate (reference)");
	zassert_equal(meshtastic_neighborinfo_send(), 0, "phone-only send");
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 0U, "nothing on the air on the public channel");

	zassert_ok(meshtastic_channels_set_slot(primary, &saved), "restore channel");
	zassert_false(meshtastic_channels_is_default(primary), "");
	meshtastic_neighborinfo_settings(&s);
	zassert_true(s.lora_allowed, "custom channel again: allowed again");
}

/* ==========================================================================
 * Answering a request
 * ========================================================================== */

ZTEST(neighborinfo, test_a_unicast_request_is_answered_once_per_window_and_never_while_disabled)
{
	struct captured c;

	/* Disabled: the request still makes the requester a neighbor, but a
	 * disabled module never answers (reference). */
	inject_neighborinfo(PEER_R, TEST_NODE_ID, 0x2E000001U, 0U, true, 5);
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 0U, "disabled: no reply");
	zassert_true(table_has(PEER_R, NULL), "the requester was heard directly, so it is a neighbor");

	zassert_ok(meshtastic_neighborinfo_set(true, 0U, true), "enable failed");
	inject_neighborinfo(PEER_R, TEST_NODE_ID, 0x2E000002U, 0U, true, 5);
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 1U, "enabled: answered");
	zassert_equal(c.to, PEER_R, "unicast back to the asker");
	zassert_equal(c.request_id, 0x2E000002U, "correlated to the request");
	zassert_true(info_lists(&c.info, PEER_R, NULL), "listing the asker itself as a neighbor");

	inject_neighborinfo(PEER_R, TEST_NODE_ID, 0x2E000003U, 0U, true, 5);
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 0U, "a second request inside the window is not answered");

	k_sleep(K_SECONDS(CONFIG_MESHTASTIC_NEIGHBORINFO_REPLY_SUPPRESS_SEC + 1));
	/* The module is enabled, so periodic broadcasts went out during that
	 * wait; discard them so only the reply is counted below. */
	(void)drain_tx(K_NO_WAIT);
	inject_neighborinfo(PEER_R, TEST_NODE_ID, 0x2E000004U, 0U, true, 5);
	c = drain_tx(SETTLE);
	zassert_equal(c.count, 1U, "answered again once the window has passed");
}

#endif /* TABLE_SIZE */
