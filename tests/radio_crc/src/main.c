/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Frames with a failed payload CRC, end to end on the sim radio.
 *
 * On the SX126x the driver drops such a frame before the stack's receive callback and counts
 * it; the stack reports the count as `meshtastic state`'s warn.bad_crc. Here the sim radio does
 * the same (lora_sim_inject_bad_crc), so what is asserted is the whole chain: the count moves,
 * the state line shows it, and the frame is never decoded, delivered or relayed. A frame with a
 * good CRC, built the same way, is the control: it is decoded and moves nothing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"
#include "meshtastic_text_inbox.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define PEER         0x0B000001U

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

/* A peer's broadcast text on the primary channel, as it would arrive over the air, on its
 * last hop (so a good copy is decoded and not relayed either: only the CRC differs). */
static uint32_t text_frame(uint32_t id, const char *text, uint8_t *wire)
{
	uint32_t wire_len;
	struct meshtastic_packet packet = {
		.from = PEER,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = text,
		.payload_len = strlen(text),
		.hop_limit = 0U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
		.no_pkc = true,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len),
		   "build_wire_packet failed");
	return wire_len;
}

/* Run `meshtastic state` and return the value of "key": in its JSON, having checked the
 * frame's CRC-16. */
static uint32_t state_value(const char *key)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	const char *out;
	const char *start;
	const char *star;
	const char *k;
	size_t len = 0;
	char want[48];

	shell_backend_dummy_clear_output(sh);
	zassert_ok(shell_execute_cmd(sh, "meshtastic state"), "state failed");
	out = shell_backend_dummy_get_output(sh, &len);
	start = strstr(out, "~S{");
	zassert_not_null(start, "no sentinel: %s", out);
	start += 2;
	star = strrchr(start, '*');
	zassert_not_null(star, "no checksum: %s", out);
	zassert_equal(crc16_itu_t(0U, (const uint8_t *)start, (size_t)(star - start)),
		      (uint16_t)strtoul(star + 1, NULL, 16), "state line CRC");
	snprintk(want, sizeof(want), "\"%s\":", key);
	k = strstr(start, want);
	zassert_true(k != NULL && k < star, "%s missing: %s", want, start);
	return (uint32_t)strtoul(k + strlen(want), NULL, 10);
}

static void *radio_crc_setup(void)
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

static void radio_crc_before(void *fixture)
{
	ARG_UNUSED(fixture);

	meshtastic_text_inbox_clear();
	k_msleep(200);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(radio_crc, NULL, radio_crc_setup, radio_crc_before, NULL, NULL);

ZTEST(radio_crc, test_a_good_frame_is_decoded_and_counts_no_crc_error)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len = text_frame(0x7c000001U, "good crc", wire);
	struct meshtastic_status a, b;

	zassert_ok(meshtastic_get_status(&a));
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -61, 9));
	k_msleep(100);
	zassert_ok(meshtastic_get_status(&b));

	zassert_equal(b.rx_packets - a.rx_packets, 1U, "the control frame is heard");
	zassert_equal(meshtastic_text_inbox_count(), 1U, "and decoded");
	zassert_equal(meshtastic_radio_crc_error_count(), 0U);
	zassert_equal(state_value("bad_crc"), 0U);
}

ZTEST(radio_crc, test_a_bad_crc_is_counted_shown_and_never_decoded)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len = text_frame(0x7c000002U, "bad crc", wire);
	struct meshtastic_status a, b;

	zassert_ok(meshtastic_get_status(&a));
	wait_rx_armed();
	zassert_ok(lora_sim_inject_bad_crc(lora_dev, wire, (uint8_t)len), "the radio heard it");
	k_msleep(100);
	zassert_ok(meshtastic_get_status(&b));

	zassert_equal(lora_sim_crc_error_count(lora_dev), 1U, "the radio counted it");
	zassert_equal(meshtastic_radio_crc_error_count(), 1U, "the stack reads the radio's count");
	zassert_equal(state_value("bad_crc"), 1U, "and the state line shows it");

	zassert_equal(meshtastic_text_inbox_count(), 0U, "never decoded");
	zassert_equal(b.rx_packets, a.rx_packets, "never reached the stack");
	zassert_equal(b.decode_failures, a.decode_failures, "not a decode failure either");
	zassert_equal(b.duplicate_packets, a.duplicate_packets);
	zassert_equal(lora_sim_tx_pending(lora_dev), 0, "never relayed");
	zassert_true(lora_sim_rx_armed(lora_dev), "the radio is still listening");

	/* Every one counts, and the count is the stack's, not the test's. */
	wait_rx_armed();
	zassert_ok(lora_sim_inject_bad_crc(lora_dev, wire, (uint8_t)len));
	zassert_equal(state_value("bad_crc"), 2U);

	/* The same frame with its CRC intact still decodes: the bad copies left no trace (it is
	 * not a duplicate of something the node never saw). */
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -61, 9));
	k_msleep(100);
	zassert_equal(meshtastic_text_inbox_count(), 1U, "the good copy is decoded");
	zassert_equal(state_value("bad_crc"), 2U, "and is no CRC error");
}
