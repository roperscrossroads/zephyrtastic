/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Node status message (agents-dnr4.26): the away-message module, driven
 * through the sim radio.
 *
 * What is asserted, in the reference's terms (StatusMessageModule.cpp):
 *   - nothing is announced while no status is set;
 *   - a status set at runtime is announced START_DELAY after the change, as a
 *     broadcast StatusMessage on port 36 from us, then repeated every INTERVAL;
 *   - clearing it cancels the repeat;
 *   - a peer's status is decoded and cached per node, the newest replacing the
 *     older, the least recently heard evicted when the cache is full.
 *
 * Every frame on the sim radio is decoded and attributed: relayed copies of an
 * injected peer frame come back out with the PEER as `from` and must not count
 * as an announce of ours.
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
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"
#include "meshtastic_statusmessage.h"

#define TEST_NODE_ID 0x0A0A0A0AU

#define PEER_A 0x0B000001U
#define PEER_B 0x0B000002U
#define PEER_C 0x0B000003U
#define PEER_D 0x0B000004U

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

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

/* A peer announcing its status, exactly the frame this module sends. */
static void inject_peer_status(uint32_t from, uint32_t id, const char *status)
{
	meshtastic_StatusMessage msg = meshtastic_StatusMessage_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t stream;
	struct meshtastic_packet packet;

	strncpy(msg.status, status, sizeof(msg.status) - 1U);
	stream = pb_ostream_from_buffer(payload, sizeof(payload));
	zassert_true(pb_encode(&stream, meshtastic_StatusMessage_fields, &msg),
		     "StatusMessage encode failed");

	packet = (struct meshtastic_packet){
		.from = from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = id,
		.portnum = MESHTASTIC_PORT_NODE_STATUS,
		.payload = payload,
		.payload_len = stream.bytes_written,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};
	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len),
		   "build_wire_packet failed");

	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	/* Let the RX thread run the module before the caller reads the cache. */
	k_msleep(100);
}

struct captured {
	uint32_t count;   /* status frames THIS node originated */
	uint32_t to;      /* destination of the last one */
	bool want_response;
	char status[80];  /* decoded payload of the last one */
};

static struct captured drain_status_tx(k_timeout_t settle)
{
	struct lora_sim_frame f;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct captured c = {0};

	k_sleep(settle);

	while (lora_sim_take_tx(lora_dev, &f, K_NO_WAIT) == 0) {
		meshtastic_StatusMessage msg = meshtastic_StatusMessage_init_zero;
		pb_istream_t is;

		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &decoded, payload,
						  sizeof(payload)) != 0) {
			continue;
		}
		if (decoded.from != TEST_NODE_ID ||
		    decoded.portnum != MESHTASTIC_PORT_NODE_STATUS) {
			continue;
		}
		c.count++;
		c.to = decoded.to;
		c.want_response = decoded.want_response;
		is = pb_istream_from_buffer(decoded.payload, decoded.payload_len);
		zassert_true(pb_decode(&is, meshtastic_StatusMessage_fields, &msg),
			     "our own StatusMessage must decode");
		strncpy(c.status, msg.status, sizeof(c.status) - 1U);
	}
	return c;
}

/* Long enough for the workqueue to run the send and the frame to reach the
 * radio through the outbound queue. */
#define SETTLE K_MSEC(500)

static void *statusmessage_setup(void)
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
	/* No random pre-TX backoff: frames are timed against the cadence. */
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void statusmessage_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Every test starts with no status of ours (which also cancels any
	 * announce a previous test armed) and an empty peer cache. */
	zassert_ok(meshtastic_statusmessage_set(""), "clear failed");
	meshtastic_statusmessage_reset();
	k_msleep(200);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(statusmessage, NULL, statusmessage_setup, statusmessage_before, NULL, NULL);

/* ==========================================================================
 * Own status
 * ========================================================================== */

ZTEST(statusmessage, test_get_reports_none_and_truncates_to_the_field_width)
{
	char buf[80];
	char longer[120];

	zassert_equal(meshtastic_statusmessage_get(buf, sizeof(buf)), 0U, "no status at start");
	zassert_equal(buf[0], '\0', "buffer is the empty string when none is set");

	memset(longer, 'x', sizeof(longer) - 1U);
	longer[sizeof(longer) - 1U] = '\0';
	zassert_ok(meshtastic_statusmessage_set(longer), "set failed");
	zassert_equal(meshtastic_statusmessage_get(buf, sizeof(buf)), 79U,
		      "a status longer than the protobuf field is truncated, not refused");
	zassert_equal(buf[79], '\0', "and NUL-terminated inside the field");
}

ZTEST(statusmessage, test_send_now_needs_a_status)
{
	struct captured c;

	zassert_equal(meshtastic_statusmessage_send(), -ENODATA,
		      "nothing to say: refused, not an empty broadcast");
	c = drain_status_tx(SETTLE);
	zassert_equal(c.count, 0U, "and nothing went on air");

	zassert_ok(meshtastic_statusmessage_set("gone fishing"), "set failed");
	zassert_ok(meshtastic_statusmessage_send(), "send failed");
	c = drain_status_tx(SETTLE);
	zassert_equal(c.count, 1U, "exactly one frame");
	zassert_equal(c.to, MESHTASTIC_NODE_BROADCAST, "a status is a broadcast");
	zassert_false(c.want_response, "and asks nothing back");
	zassert_str_equal(c.status, "gone fishing", "carrying the status verbatim");
}

#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)

ZTEST(statusmessage, test_nothing_is_announced_without_a_status)
{
	struct captured c;

	c = drain_status_tx(K_SECONDS(CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC + 1));
	zassert_equal(c.count, 0U, "no status, no announce -- past the start delay");
}

ZTEST(statusmessage, test_a_set_status_is_announced_after_the_delay_then_repeats)
{
	struct captured c;
	const int delay_s = CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC;
	const int interval_s = CONFIG_MESHTASTIC_STATUSMESSAGE_INTERVAL_SEC;

	zassert_ok(meshtastic_statusmessage_set("at the lake"), "set failed");

	/* Not immediately: the delay lets an app user finish typing. */
	c = drain_status_tx(K_MSEC(300));
	zassert_equal(c.count, 0U, "nothing before the start delay");

	c = drain_status_tx(K_SECONDS(delay_s));
	zassert_equal(c.count, 1U, "exactly one announce at the start delay");
	zassert_equal(c.to, MESHTASTIC_NODE_BROADCAST, "the announce is a broadcast");
	zassert_str_equal(c.status, "at the lake", "carrying the persisted status");

	/* Nothing more until the interval; checked at half so the assertion is
	 * about the interval, not about how long this test sleeps. */
	c = drain_status_tx(K_SECONDS(interval_s / 2));
	zassert_equal(c.count, 0U, "an announce inside the interval is unbudgeted airtime");

	c = drain_status_tx(K_SECONDS((interval_s / 2) + 2));
	zassert_equal(c.count, 1U, "and exactly one once the interval has elapsed");

	/* Clearing cancels the repeat. */
	zassert_ok(meshtastic_statusmessage_set(""), "clear failed");
	c = drain_status_tx(K_SECONDS(interval_s + 2));
	zassert_equal(c.count, 0U, "a cleared status is never announced again");
}

ZTEST(statusmessage, test_a_change_re_arms_the_delay_instead_of_waiting_for_the_interval)
{
	struct captured c;
	const int delay_s = CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC;

	zassert_ok(meshtastic_statusmessage_set("first"), "set failed");
	c = drain_status_tx(K_SECONDS(delay_s + 1));
	zassert_equal(c.count, 1U, "first announce");
	zassert_str_equal(c.status, "first", "");

	/* The reference would now hold the new text for up to 12 h (its own
	 * TODO). Here a change is announced after the start delay again. */
	zassert_ok(meshtastic_statusmessage_set("second"), "set failed");
	c = drain_status_tx(K_SECONDS(delay_s + 1));
	zassert_equal(c.count, 1U, "a changed status is announced after the delay");
	zassert_str_equal(c.status, "second", "with the new text");
}

#else /* !CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND */

ZTEST(statusmessage, test_auto_send_off_never_announces_unprompted)
{
	struct captured c;

	zassert_ok(meshtastic_statusmessage_set("quiet"), "set failed");
	/* Longer than any delay the auto-send build would use. */
	c = drain_status_tx(K_SECONDS(130));
	zassert_equal(c.count, 0U, "auto-send off: a set status stays on the node");

	zassert_ok(meshtastic_statusmessage_send(), "an explicit send still works");
	c = drain_status_tx(SETTLE);
	zassert_equal(c.count, 1U, "and goes out once");
	zassert_str_equal(c.status, "quiet", "");
}

#endif /* CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND */

/* ==========================================================================
 * Peers
 * ========================================================================== */

ZTEST(statusmessage, test_a_peer_status_is_cached_and_the_newest_wins)
{
	char buf[80];
	uint32_t node;
	int64_t age_ms;

	zassert_false(meshtastic_statusmessage_peer_get(PEER_A, buf, sizeof(buf)),
		      "nothing cached before anything is heard");

	inject_peer_status(PEER_A, 0x5A000001U, "gone fishing");
	inject_peer_status(PEER_B, 0x5A000002U, "home");

	zassert_true(meshtastic_statusmessage_peer_get(PEER_A, buf, sizeof(buf)), "A cached");
	zassert_str_equal(buf, "gone fishing", "A's status");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_B, buf, sizeof(buf)), "B cached");
	zassert_str_equal(buf, "home", "B's status");

	inject_peer_status(PEER_A, 0x5A000003U, "back");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_A, buf, sizeof(buf)), "A still cached");
	zassert_str_equal(buf, "back", "the newer status replaced the older, same slot");

	/* The walk the shell uses sees both, with a sane age. */
	{
		int seen = 0;

		for (size_t i = 0; i < CONFIG_MESHTASTIC_STATUSMESSAGE_CACHE_SIZE; i++) {
			if (meshtastic_statusmessage_peer_at(i, &node, buf, sizeof(buf),
							     &age_ms)) {
				zassert_true(node == PEER_A || node == PEER_B, "known peer");
				zassert_true(age_ms >= 0 && age_ms < 5000, "recent");
				seen++;
			}
		}
		zassert_equal(seen, 2, "exactly the two peers heard");
	}

	zassert_false(meshtastic_statusmessage_peer_get(PEER_C, buf, sizeof(buf)),
		      "a peer never heard is not in the cache");
}

ZTEST(statusmessage, test_our_own_status_is_not_a_peer)
{
	char buf[80];

	zassert_ok(meshtastic_statusmessage_set("me"), "set failed");
	zassert_ok(meshtastic_statusmessage_send(), "send failed");
	(void)drain_status_tx(SETTLE);
	zassert_false(meshtastic_statusmessage_peer_get(TEST_NODE_ID, buf, sizeof(buf)),
		      "what we sent must not land in the peer cache");
}

#if CONFIG_MESHTASTIC_STATUSMESSAGE_CACHE_SIZE == 2

ZTEST(statusmessage, test_cache_evicts_the_least_recently_heard)
{
	char buf[80];

	inject_peer_status(PEER_A, 0x5B000001U, "a");
	inject_peer_status(PEER_B, 0x5B000002U, "b");
	inject_peer_status(PEER_C, 0x5B000003U, "c");

	zassert_false(meshtastic_statusmessage_peer_get(PEER_A, buf, sizeof(buf)),
		      "A, heard first, is evicted by the third peer");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_B, buf, sizeof(buf)), "B kept");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_C, buf, sizeof(buf)), "C kept");

	/* Hearing B again makes C the oldest, so D evicts C, not B. */
	inject_peer_status(PEER_B, 0x5B000004U, "b2");
	inject_peer_status(PEER_D, 0x5B000005U, "d");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_B, buf, sizeof(buf)), "B refreshed");
	zassert_str_equal(buf, "b2", "");
	zassert_false(meshtastic_statusmessage_peer_get(PEER_C, buf, sizeof(buf)),
		      "C, now the least recently heard, is the one evicted");
	zassert_true(meshtastic_statusmessage_peer_get(PEER_D, buf, sizeof(buf)), "D cached");
}

#endif /* CACHE_SIZE == 2 */
