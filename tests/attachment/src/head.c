/* SPDX-License-Identifier: GPL-3.0 */

/*
 * The keyless radio head (ATTACHMENT-DESIGN S5), on the sim radio.
 *
 * The real stack runs as a head. What its radio hears is captured at the send
 * seam (meshtastic_attachment_head_send, overridden here) as the RX_FRAME
 * envelope the brain would receive; the brain's controls arrive through
 * meshtastic_attachment_head_on_envelope, exactly as the BLE glue delivers
 * them. No frame is ever built or decoded in this process: the build refuses
 * the ELF if that code linked (keyless-assert.cmake), so the frames here are
 * bytes, not packets.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic_attachment_codec.h"
#include "meshtastic_attachment_head.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_preset.h"
#include "meshtastic_region_presets.h"

#define TEST_NODE_ID 0x0A0A0A0AU /* the head's link identity */
#define BRAIN_NODE   0x121F0001U
#define STRANGER     0x0BAD0001U
#define PRESET_ST meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO
#define PRESET_MF meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* What the head sent to its brain. */
static struct {
	struct k_sem sem;
	uint32_t to;
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	size_t len;
	uint32_t count;
} sent;

static enum meshtastic_attach_auth test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;

static int test_bearer_send(uint32_t brain, const uint8_t *env, size_t len)
{
	if (len > sizeof(sent.env)) {
		return -EMSGSIZE;
	}
	sent.to = brain;
	memcpy(sent.env, env, len);
	sent.len = len;
	sent.count++;
	k_sem_give(&sent.sem);
	return 0;
}

static bool test_bearer_link_info(uint32_t node, struct meshtastic_attach_link_info *out)
{
	ARG_UNUSED(node);
	out->up = true;
	out->auth = test_auth;
	out->takes_envelopes = true;
	out->mtu = 20U;
	out->rtt_ms = 0U;
	return true;
}

static const struct meshtastic_attach_bearer test_bearer = {
	.name = "test",
	.send = test_bearer_send,
	.link_info = test_bearer_link_info,
};

/* A frame is any bytes with a header: the head never looks inside. */
static void some_frame(uint8_t *wire, uint8_t len, uint8_t seed)
{
	for (uint8_t i = 0U; i < len; i++) {
		wire[i] = (uint8_t)(seed + i * 7U);
	}
}

static void wait_rx_armed(void)
{
	for (int i = 0; i < 200 && !lora_sim_rx_armed(lora_dev); i++) {
		k_sleep(K_MSEC(10));
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "sim radio never armed");
}

static void *head_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_US,
	};

	k_sem_init(&sent.sem, 0, 16);
	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	zassert_ok(meshtastic_attach_bearer_register(&test_bearer), "test bearer");
	return NULL;
}

static void head_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_attachment_head_reset();
	zassert_ok(meshtastic_preset_switch(PRESET_ST, NULL), "preset");
	lora_sim_reset(lora_dev);
	wait_rx_armed();
	k_sem_reset(&sent.sem);
	memset(&sent, 0, sizeof(sent));
	k_sem_init(&sent.sem, 0, 16);
	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	/* set_brain introduces us with a STATUS; let it pass. */
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	k_sem_reset(&sent.sem);
	memset(&sent, 0, sizeof(sent));
	k_sem_init(&sent.sem, 0, 16);
}

ZTEST_SUITE(attachment_head, NULL, head_setup, head_before, NULL, NULL);

/* H1: what the radio hears goes to the brain as RX_FRAME, with the signal and
 * preset it was heard on and the wire bytes untouched. */
ZTEST(attachment_head, test_heard_frame_reaches_the_brain_untouched)
{
	uint8_t wire[48];
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_head_stats st;

	some_frame(wire, sizeof(wire), 0x10U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -77, 9), "inject");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	zassert_equal(sent.to, BRAIN_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg), "decode");
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_RX_FRAME);
	zassert_equal(msg.u.rx.preset, (uint8_t)PRESET_ST, "heard on ShortTurbo");
	zassert_equal(msg.u.rx.rssi, -77);
	zassert_equal(msg.u.rx.snr, 9);
	zassert_equal(msg.u.rx.wire_len, sizeof(wire));
	zassert_mem_equal(msg.u.rx.wire, wire, sizeof(wire), "byte for byte");

	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.heard, 1U);
	zassert_equal(st.forwarded, 1U);
	zassert_equal(st.send_failed, 0U);
}

/* H1b: a head without a brain hears and drops, and says so. */
ZTEST(attachment_head, test_no_brain_no_forwarding)
{
	uint8_t wire[32];
	struct meshtastic_attachment_head_stats st;

	meshtastic_attachment_head_set_brain(0U);
	some_frame(wire, sizeof(wire), 0x20U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -60, 4));
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "nothing sent");
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.heard, 1U);
	zassert_equal(st.no_brain, 1U);
	zassert_equal(st.forwarded, 0U);
}

/* H3: SET_PRESET retunes the radio -- from the brain only -- and the brain
 * learns the outcome from the STATUS that follows. */
ZTEST(attachment_head, test_set_preset_from_the_brain_only)
{
	uint8_t env[8];
	int len = meshtastic_attachment_encode_set_preset((uint8_t)PRESET_MF, env, sizeof(env));
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_head_stats st;
	uint32_t freq;
	uint8_t sf, bw;
	struct meshtastic_modem_params want;

	zassert_true(len > 0);

	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, STRANGER, env, (size_t)len), -EPERM,
		      "a stranger does not retune this radio");
	zassert_equal(mt.modem_preset, PRESET_ST, "still ShortTurbo");
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(200)), -EAGAIN, "and gets no STATUS");

	zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len));
	zassert_equal(mt.modem_preset, PRESET_MF, "retuned");
	zassert_ok(meshtastic_preset_to_params(PRESET_MF, false, &want));
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw));
	zassert_equal(sf, want.spread_factor, "the radio is on MediumFast's SF (%u)", sf);

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "STATUS follows the retune");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_STATUS);
	zassert_equal(msg.u.status.preset, (uint8_t)PRESET_MF);
	zassert_equal(msg.u.status.hwid, TEST_NODE_ID, "the head's own id");
	zassert_equal(msg.u.status.brain, BRAIN_NODE);
	zassert_true((msg.u.status.flags & MESHTASTIC_ATTACHMENT_ST_IS_HEAD) != 0U, "a head");
	zassert_true((msg.u.status.flags & MESHTASTIC_ATTACHMENT_ST_RX_ONLY) != 0U,
		     "rx-only until phase 3");

	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.controls, 1U);

	/* An out-of-range preset is a bad envelope, not a retune. */
	env[1] = 0xEEU;
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
		      -EBADMSG);
	zassert_equal(mt.modem_preset, PRESET_MF);
}

/* H4: what a head does not take: TX_FRAME (phase 3), and the envelopes a head
 * itself sends. */
ZTEST(attachment_head, test_controls_a_head_refuses)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	uint8_t wire[20];
	struct meshtastic_attachment_head_stats st;
	int len;

	some_frame(wire, sizeof(wire), 0x30U);
	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 1U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
			      -EPERM, "TX via a head is phase 3");
		zassert_equal(lora_sim_tx_pending(lora_dev), 0, "nothing on the air");
	}
	{
		const struct meshtastic_attachment_rx_frame rx = {
			.preset = (uint8_t)PRESET_ST, .rssi = -50, .snr = 1, .rx_ms = 1U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_rx_frame(&rx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
			      -EBADMSG, "a head sends RX_FRAME, it does not take one");
	}
	env[0] = 0x7FU;
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, 3U), -EBADMSG);
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, 0U, env, 3U), -EINVAL);

	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.rejected, 2U);
}

/* H5: a head originates nothing: the outbound queue refuses every caller. */
ZTEST(attachment_head, test_a_head_originates_nothing)
{
	uint8_t wire[40];

	some_frame(wire, sizeof(wire), 0x40U);
	zassert_equal(meshtastic_radio_send_wire(wire, sizeof(wire)), -EPERM);
	zassert_equal(meshtastic_radio_send_wire_after(wire, sizeof(wire), 0U, 10U), -EPERM);
	k_sleep(K_MSEC(200));
	zassert_equal(lora_sim_tx_pending(lora_dev), 0, "nothing on the air");
}

/* H7: STATUS on the timer (5 s in this build), and on demand. */
ZTEST(attachment_head, test_status_on_the_timer_and_on_demand)
{
	struct meshtastic_attachment_msg msg;

	zassert_ok(meshtastic_attachment_head_status_send(), "on demand");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)));
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_STATUS);
	zassert_equal(msg.u.status.preset, (uint8_t)PRESET_ST);
	zassert_equal(msg.u.status.rx_frames, 0U);

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(7)), "the timer's STATUS");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_STATUS);

	meshtastic_attachment_head_set_brain(0U);
	zassert_equal(meshtastic_attachment_head_status_send(), -EHOSTUNREACH);
}
