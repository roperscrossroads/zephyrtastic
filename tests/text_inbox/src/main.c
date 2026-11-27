/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Text inbox: the last text messages delivered to this node, driven through the
 * sim radio.
 *
 * Every message is a real wire frame from a peer, injected on the sim radio and
 * taken by the router's receive path, so what is asserted is what a console on
 * a board shows: the message is kept once, with its sender, destination,
 * channel, the radio and preset it was heard on and its text, newest first; a
 * frame on another port is not a message; a long text is cut and marked; the
 * oldest goes when the ring is full; a clear keeps the arrival count going.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"
#include "meshtastic_text_inbox.h"

#define TEST_NODE_ID 0x0A0A0A0AU

#define PEER_A 0x0B000001U
#define PEER_B 0x0B000002U

#define ENTRIES  CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES
#define TEXT_LEN CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

/* A peer's frame on the primary channel, as it would arrive over the air, two
 * hops from where it started. */
static void inject(uint32_t from, uint32_t to, uint32_t id, uint32_t portnum, const void *payload,
		   size_t len)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = portnum,
		.payload = payload,
		.payload_len = len,
		.hop_limit = 1U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
		.no_pkc = true,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len),
		   "build_wire_packet failed");
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -61, 9), "inject failed");
	/* Let the receive thread deliver it before the caller reads the inbox. */
	k_msleep(100);
}

static void inject_text(uint32_t from, uint32_t to, uint32_t id, const char *text)
{
	inject(from, to, id, MESHTASTIC_PORT_TEXT_MESSAGE, text, strlen(text));
}

static void *text_inbox_setup(void)
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
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void text_inbox_before(void *fixture)
{
	ARG_UNUSED(fixture);

	meshtastic_text_inbox_clear();
	k_msleep(200);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(text_inbox, NULL, text_inbox_setup, text_inbox_before, NULL, NULL);

ZTEST(text_inbox, test_a_cleared_inbox_is_empty)
{
	struct meshtastic_text_inbox_entry e;

	zassert_equal(meshtastic_text_inbox_count(), 0U);
	zassert_equal(meshtastic_text_inbox_get(0, &e), -ENOENT);
	zassert_equal(meshtastic_text_inbox_get(0, NULL), -EINVAL);
}

ZTEST(text_inbox, test_a_broadcast_text_is_kept_with_how_it_arrived)
{
	struct meshtastic_text_inbox_entry e;
	uint32_t before = meshtastic_text_inbox_total();

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000001U, "hello inbox");

	zassert_equal(meshtastic_text_inbox_count(), 1U);
	zassert_equal(meshtastic_text_inbox_total(), before + 1U);
	zassert_ok(meshtastic_text_inbox_get(0, &e));
	zassert_equal(e.seq, before + 1U);
	zassert_equal(e.from, PEER_A);
	zassert_equal(e.to, MESHTASTIC_NODE_BROADCAST);
	zassert_equal(e.id, 0x71000001U);
	zassert_equal(e.channel_index, meshtastic_channels_primary_index());
	zassert_equal(e.rx_radio, 0U, "heard on this board's own radio");
	zassert_equal(e.rx_heard_on, 1U + meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
		      "heard on the radio's preset, LongFast: got %u", e.rx_heard_on);
	zassert_equal(e.hops, 2U, "hop_start 3, hop_limit 1");
	zassert_equal(e.rssi, -61);
	zassert_equal(e.snr, 9);
	zassert_false(e.pki);
	zassert_false(e.truncated);
	zassert_equal(e.len, strlen("hello inbox"));
	zassert_str_equal(e.text, "hello inbox");
}

ZTEST(text_inbox, test_a_text_to_this_node_is_kept_with_its_destination)
{
	struct meshtastic_text_inbox_entry e;

	inject_text(PEER_B, TEST_NODE_ID, 0x71000002U, "just for you");

	zassert_ok(meshtastic_text_inbox_get(0, &e));
	zassert_equal(e.from, PEER_B);
	zassert_equal(e.to, TEST_NODE_ID);
	zassert_str_equal(e.text, "just for you");
}

ZTEST(text_inbox, test_newest_first)
{
	struct meshtastic_text_inbox_entry e;

	if (ENTRIES < 2) {
		ztest_test_skip();
	}
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000011U, "first");
	inject_text(PEER_B, MESHTASTIC_NODE_BROADCAST, 0x71000012U, "second");

	zassert_equal(meshtastic_text_inbox_count(), 2U);
	zassert_ok(meshtastic_text_inbox_get(0, &e));
	zassert_str_equal(e.text, "second");
	zassert_ok(meshtastic_text_inbox_get(1, &e));
	zassert_str_equal(e.text, "first");
	zassert_equal(meshtastic_text_inbox_get(2, &e), -ENOENT);
}

ZTEST(text_inbox, test_another_port_is_not_a_message)
{
	static const uint8_t not_text[] = {0x08, 0x01};

	inject(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000021U, MESHTASTIC_PORT_NODE_STATUS,
	       not_text, sizeof(not_text));
	zassert_equal(meshtastic_text_inbox_count(), 0U);
}

ZTEST(text_inbox, test_a_duplicate_frame_is_kept_once)
{
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000031U, "once");
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000031U, "once");
	zassert_equal(meshtastic_text_inbox_count(), 1U, "the router drops the duplicate");
}

ZTEST(text_inbox, test_a_long_text_is_cut_and_marked)
{
	struct meshtastic_text_inbox_entry e;
	char text[TEXT_LEN + 8];

	if (TEXT_LEN + 1 > MESHTASTIC_MAX_PAYLOAD_LEN - 8) {
		ztest_test_skip();
	}
	memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000041U, text);

	zassert_ok(meshtastic_text_inbox_get(0, &e));
	zassert_true(e.truncated);
	zassert_equal(e.len, TEXT_LEN);
	zassert_equal(strlen(e.text), TEXT_LEN);
	zassert_mem_equal(e.text, text, TEXT_LEN);
}

ZTEST(text_inbox, test_the_oldest_goes_when_the_ring_is_full)
{
	struct meshtastic_text_inbox_entry e;
	char text[16];
	uint32_t before = meshtastic_text_inbox_total();

	if (ENTRIES > 4) {
		ztest_test_skip(); /* the small_ring scenario reaches it in a few frames */
	}
	for (unsigned int i = 0; i <= ENTRIES; i++) {
		snprintk(text, sizeof(text), "m%u", i);
		inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000051U + i, text);
	}

	zassert_equal(meshtastic_text_inbox_count(), ENTRIES);
	zassert_equal(meshtastic_text_inbox_total(), before + ENTRIES + 1U);
	zassert_ok(meshtastic_text_inbox_get(0, &e));
	snprintk(text, sizeof(text), "m%u", ENTRIES);
	zassert_str_equal(e.text, text, "the newest is the last sent");
	zassert_ok(meshtastic_text_inbox_get(ENTRIES - 1U, &e));
	zassert_str_equal(e.text, "m1", "m0, the oldest, was replaced");
}

ZTEST(text_inbox, test_a_clear_empties_it_and_the_count_goes_on)
{
	struct meshtastic_text_inbox_entry e;
	uint32_t before;

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000061U, "before");
	before = meshtastic_text_inbox_total();
	meshtastic_text_inbox_clear();
	zassert_equal(meshtastic_text_inbox_count(), 0U);
	zassert_equal(meshtastic_text_inbox_total(), before, "a clear is not a reboot");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, 0x71000062U, "after");
	zassert_ok(meshtastic_text_inbox_get(0, &e));
	zassert_equal(e.seq, before + 1U);
	zassert_str_equal(e.text, "after");
}
