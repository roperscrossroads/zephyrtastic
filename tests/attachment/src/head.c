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
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#endif

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic_attachment_codec.h"
#include "meshtastic_attachment_head.h"
#include "meshtastic_packet.h"
#include "meshtastic_contention.h"
#include <zephyr/sys/byteorder.h>
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
	test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;
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
	zassert_true((msg.u.status.flags & MESHTASTIC_ATTACHMENT_ST_RX_ONLY) == 0U,
		     "a head with a transmitter says so (P3): the brain hands it frames");
	zassert_true((msg.u.status.flags & MESHTASTIC_ATTACHMENT_ST_TX_ENABLED) != 0U, "tx enabled");

	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.controls, 1U);

	/* H10 (SCOPE C4): the brain's own id over a link the bearer does not
	 * vouch for (an unbonded connection) is refused, and counted apart. */
	test_auth = MESHTASTIC_ATTACH_AUTH_NONE;
	len = meshtastic_attachment_encode_set_preset((uint8_t)PRESET_ST, env, sizeof(env));
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
		      -EACCES, "untrusted link");
	zassert_equal(mt.modem_preset, PRESET_MF, "not retuned");
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.untrusted, 1U);
	zassert_equal(st.controls, 1U);
	test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;
	len = meshtastic_attachment_encode_set_preset((uint8_t)PRESET_MF, env, sizeof(env));

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
		zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, STRANGER, env, (size_t)len),
			      -EPERM, "only the brain hands a head frames to transmit");
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

/* H7b (fix 11, review X4): the bearer's link-up to the brain sends STATUS at
 * once -- the boot-time introduction is lost before any link exists. A
 * link-up to anyone else sends nothing. */
ZTEST(attachment_head, test_status_on_link_up_to_the_brain)
{
	struct meshtastic_attachment_msg msg;

	meshtastic_attach_bearer_link_up(&test_bearer, STRANGER);
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "a stranger's link: nothing");
	meshtastic_attach_bearer_link_up(&test_bearer, BRAIN_NODE);
	zassert_ok(k_sem_take(&sent.sem, K_MSEC(500)), "the brain's link: STATUS at once");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_STATUS);
	zassert_equal(msg.u.status.brain, BRAIN_NODE);
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

#if defined(CONFIG_MESHTASTIC_SETTINGS)
/* H8 (SCOPE §7, review F4): the identity in flash is neither loaded nor
 * touched by a head. The records a node leaves are written raw (their content
 * is irrelevant: a head must never decode them), the head loads settings, and
 * afterwards no channel holds a key, the security section is empty, a save by
 * the head leaves the records byte for byte as they were, and the head's own
 * record (mtattach/brain) is the only thing it wrote. */
static const uint8_t keyed_channel[] = { 0x0A, 0x12, 0x0A, 0x10, 0xDE, 0xAD, 0xBE, 0xEF,
					 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
					 0xDE, 0xAD, 0xBE, 0xEF, 0x10, 0x01 };
static const uint8_t keyed_security[] = { 0x4A, 0x22, 0x0A, 0x20, 0x01, 0x02, 0x03, 0x04,
					  0x05, 0x06, 0x07, 0x08 };
static const uint8_t keyed_module[] = { 0x0A, 0x04, 0x08, 0x01, 0x10, 0x01 };

struct raw_read {
	uint8_t buf[64];
	size_t len;
	bool found;
};

static int raw_read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		       void *param)
{
	struct raw_read *r = param;

	ARG_UNUSED(key);
	if (len > sizeof(r->buf)) {
		return -EMSGSIZE;
	}
	r->len = (size_t)read_cb(cb_arg, r->buf, len);
	r->found = true;
	return 0;
}

static void raw_expect(const char *name, const uint8_t *want, size_t want_len)
{
	struct raw_read r = { .found = false };

	zassert_ok(settings_load_subtree_direct(name, raw_read_cb, &r), "%s", name);
	zassert_true(r.found, "%s still in flash", name);
	zassert_equal(r.len, want_len, "%s length %zu", name, r.len);
	zassert_mem_equal(r.buf, want, want_len, "%s unchanged", name);
}

ZTEST(attachment_head, test_identity_in_flash_is_never_loaded_or_touched)
{
	struct meshtastic_channel_key key;
	meshtastic_Config sec;

	/* what a node left behind */
	zassert_ok(settings_save_one("meshtastic/channel/0", keyed_channel, sizeof(keyed_channel)));
	zassert_ok(settings_save_one("meshtastic/channel/2", keyed_channel, sizeof(keyed_channel)));
	zassert_ok(settings_save_one("meshtastic/config/security", keyed_security,
				     sizeof(keyed_security)));
	zassert_ok(settings_save_one("meshtastic/module/mqtt", keyed_module, sizeof(keyed_module)));

	/* the head boots (loads) over it */
	zassert_ok(settings_load(), "settings_load");

	/* nothing reached RAM */
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		zassert_true(meshtastic_channels_get_key(i, &key) < 0, "slot %u holds a key", i);
	}
	zassert_equal(mt.psk_len, 0U, "no primary PSK in RAM");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_security_tag, &sec));
	zassert_equal(sec.payload_variant.security.private_key.size, 0U, "no private key in RAM");
	zassert_equal(meshtastic_config_store_setting_get("channel/0", (void *)key.bytes,
							  sizeof(key.bytes)),
		      -ENOENT, "a head answers no channel record");

	/* the head saves its own state: the identity is not rewritten, not wiped */
	meshtastic_attachment_head_set_brain(0x12345678U);
	zassert_ok(settings_save(), "settings_save");
	raw_expect("meshtastic/channel/0", keyed_channel, sizeof(keyed_channel));
	raw_expect("meshtastic/channel/2", keyed_channel, sizeof(keyed_channel));
	raw_expect("meshtastic/config/security", keyed_security, sizeof(keyed_security));
	raw_expect("meshtastic/module/mqtt", keyed_module, sizeof(keyed_module));
	{
		struct raw_read r = { .found = false };

		zassert_ok(settings_load_subtree_direct("mtattach/brain", raw_read_cb, &r));
		zassert_true(r.found, "the head's own record was written");
	}

	/* and it cannot be made to write one */
	{
		meshtastic_Channel ch = meshtastic_Channel_init_zero;

		ch.role = meshtastic_Channel_Role_PRIMARY;
		ch.has_settings = true;
		zassert_equal(meshtastic_config_store_set_channel(0U, &ch), -EPERM);
	}
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* P3 slice 1: a TX_FRAME from the brain over a trusted link goes on the air
 * byte for byte -- the head keys up what the brain built, through the one
 * transmit funnel -- and a TX_RESULT comes back when asked. Over a link the
 * bearer does not vouch for, it is refused (the SET_PRESET gate). */
ZTEST(attachment_head, test_tx_frame_from_the_brain_is_transmitted)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_head_stats st;
	int len;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500)); /* the introductory STATUS */
	wait_rx_armed();
	some_frame(wire, sizeof(wire), 0x50U);

	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT,
			.tx_seq = 7U, .wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
			   "accepted from the brain");
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "the frame went on the air");
	zassert_equal(f.len, sizeof(wire));
	zassert_mem_equal(f.data, wire, sizeof(wire), "byte for byte");

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "TX_RESULT sent");
	zassert_equal(sent.to, BRAIN_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_RESULT);
	zassert_equal(msg.u.result.tx_seq, 7U);
	zassert_equal(msg.u.result.rc, 0, "transmitted (%d)", msg.u.result.rc);
	zassert_equal(msg.u.result.defers, 0U);

	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.tx_sent, 1U);
	zassert_equal(st.controls, 1U);

	/* No result asked for: none sent. */
	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 8U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len));
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "on the air");
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "no TX_RESULT unasked");

	/* The twin: the brain over an untrusted link is refused. */
	test_auth = MESHTASTIC_ATTACH_AUTH_NONE;
	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 9U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
		zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len),
			      -EACCES, "untrusted link");
	}
	test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing on the air");
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.tx_sent, 2U);
	zassert_equal(st.untrusted, 1U);
}

/* P3 slice 2: an own frame of the brain's (OWN_DELAY) waits the reference's
 * own-TX contention window, drawn on THIS radio, before it keys up -- and then
 * goes out, with the result reporting the wait as no defer. */
ZTEST(attachment_head, test_own_delay_is_drawn_on_the_head)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	int64_t t0;
	int len;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	some_frame(wire, sizeof(wire), 0x60U);
	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST,
			.flags = MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT | MESHTASTIC_ATTACHMENT_TXF_OWN_DELAY,
			.tx_seq = 11U, .wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		t0 = k_uptime_get();
		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len));
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "keyed up after the window");
	zassert_true(k_uptime_get() - t0 < 1000, "within the widest own-TX window (%lld ms)",
		     (long long)(k_uptime_get() - t0));
	zassert_mem_equal(f.data, wire, sizeof(wire));
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "TX_RESULT");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_RESULT);
	zassert_equal(msg.u.result.tx_seq, 11U);
	zassert_equal(msg.u.result.rc, 0);
	zassert_equal(msg.u.result.defers, 0U, "a contention wait is not a defer");
}

/* P3 slice 3, the head's side of a relay (ATTACHMENT-DESIGN §12): the window runs
 * on THIS radio's clock of reception -- rx_ms + not_before -- and this radio's
 * own hearing decides: a copy heard again before key-up cancels (CANCEL) or
 * clamps to the end of the window (LATE); the brain's TX_CANCEL withdraws. */
static uint32_t hear_and_forward(uint8_t *wire, uint8_t len, uint8_t seed)
{
	struct meshtastic_attachment_msg msg;

	some_frame(wire, len, seed);
	/* A plausible header: src/id at the usual offsets, hop budget 3. */
	{
		struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

		h->dest = sys_cpu_to_le32(0xFFFFFFFFU);
		h->src = sys_cpu_to_le32(0x0D0D0D0DU);
		h->id = sys_cpu_to_le32(0x3B00U + seed);
		h->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
		h->relay_node = 0x0DU;
	}
	zassert_ok(lora_sim_inject(lora_dev, wire, len, -85, 4), "heard");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_RX_FRAME);
	return msg.u.rx.rx_ms;
}

static int hand_relay(const uint8_t *wire, uint8_t len, uint32_t rx_ms, uint32_t not_before,
		      uint8_t dupe, uint16_t seq)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	uint8_t relay[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)relay;
	int elen;

	memcpy(relay, wire, len);
	h->flags = (uint8_t)((h->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	h->relay_node = (uint8_t)(BRAIN_NODE & 0xFFU);
	{
		const struct meshtastic_attachment_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST,
			.flags = MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT | MESHTASTIC_ATTACHMENT_TXF_RELAY,
			.tx_seq = seq,
			.relay_src = sys_le32_to_cpu(h->src),
			.relay_id = sys_le32_to_cpu(h->id),
			.rx_ms = rx_ms,
			.not_before_ms = not_before,
			.dupe = dupe,
			.wire = relay,
			.wire_len = len,
		};
		elen = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
	}
	zassert_true(elen > 0);
	return meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)elen);
}

static void expect_result(uint16_t seq, int rc)
{
	struct meshtastic_attachment_msg msg;

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(3)), "TX_RESULT");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_RESULT, "type %u", msg.type);
	zassert_equal(msg.u.result.tx_seq, seq);
	zassert_equal(msg.u.result.rc, rc, "rc %d", msg.u.result.rc);
}

ZTEST(attachment_head, test_relay_keys_up_on_the_heads_clock)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	uint32_t rx_ms;
	int64_t t_tx;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x70U);

	/* A relay of it: not before rx_ms + 300 ms, KEEP. */
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 300U, MESHTASTIC_ATTACHMENT_DUPE_KEEP, 21U));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "keyed up");
	t_tx = k_uptime_get();
	zassert_true(t_tx - (int64_t)rx_ms >= 290, "not before the window (%lld ms after rx)",
		     (long long)(t_tx - (int64_t)rx_ms));
	zassert_true(t_tx - (int64_t)rx_ms < 1200, "and not long after (%lld ms)",
		     (long long)(t_tx - (int64_t)rx_ms));
	{
		const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)f.data;

		zassert_equal(h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 2U, "the relay as built");
		zassert_equal(h->relay_node, (uint8_t)(BRAIN_NODE & 0xFFU), "the brain's relay byte");
	}
	expect_result(21U, 0);
}

ZTEST(attachment_head, test_relay_cancels_or_clamps_on_a_copy_heard_first)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_attachment_head_stats st;
	uint32_t rx_ms;
	int64_t t0;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();

	/* CANCEL: a neighbour's copy lands inside our window -> we drop ours. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x71U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 400U, MESHTASTIC_ATTACHMENT_DUPE_CANCEL, 22U));
	k_msleep(100);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x55U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), "a peer relayed it first");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "that copy is forwarded too");
	expect_result(22U, -ECANCELED);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing keyed up");
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.tx_cancelled, 1U);

	/* LATE: the same, but a router-late relay still goes -- after the window. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x72U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 200U, MESHTASTIC_ATTACHMENT_DUPE_LATE, 23U));
	k_msleep(50);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x56U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	t0 = k_uptime_get();
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9));
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(5)), "still keyed up, late");
	zassert_true(k_uptime_get() - t0 > 150, "pushed past the copy (%lld ms)",
		     (long long)(k_uptime_get() - t0));
	expect_result(23U, 0);
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.tx_late, 1U);
}

ZTEST(attachment_head, test_tx_cancel_withdraws_a_relay)
{
	uint8_t wire[40];
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	struct lora_sim_frame f;
	uint32_t rx_ms;
	int elen;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x73U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 500U, MESHTASTIC_ATTACHMENT_DUPE_KEEP, 24U));
	k_msleep(100);
	elen = meshtastic_attachment_encode_tx_cancel(0x0D0D0D0DU, 0x3B00U + 0x73U, env, sizeof(env));
	zassert_true(elen > 0);
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, STRANGER, env, (size_t)elen), -EPERM,
		      "only the brain withdraws");
	zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)elen));
	expect_result(24U, -ECANCELED);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "withdrawn");
}

/* SET_POLICY: the brain sets its head's transmit power -- a head runs no phone
 * service, so this is the only path -- through the config store (applied,
 * persisted, region-clamped), and the head reports. A stranger is refused. */
ZTEST(attachment_head, test_set_policy_sets_tx_power_from_the_brain_only)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_head_stats st;
	const struct meshtastic_attachment_policy pol = {
		.flags = MESHTASTIC_ATTACHMENT_POL_HAS_TX_POWER, .tx_power = 2,
	};
	int len;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	len = meshtastic_attachment_encode_set_policy(&pol, env, sizeof(env));
	zassert_true(len > 0);
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, STRANGER, env, (size_t)len), -EPERM);
	zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, BRAIN_NODE, env, (size_t)len));
	zassert_equal(mt.tx_power, 2, "applied now (%d dBm)", mt.tx_power);
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "STATUS follows");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_STATUS);
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.controls, 1U);
}

/* R3's sim half (the co-site desense model, zt-ble-scan-claude's lora_sim
 * pairing): a head in one enclosure with its brain is DEAF while the brain
 * transmits. The stack's behaviour under that blanking: a neighbour's copy
 * of a frame we hold a CANCEL relay of lands during the brain's transmission,
 * so we never hear it -- and our relay must still go, because cancel-on-
 * duplicate is about what we HEARD, not what was on the air. */
static const struct device *const other_radio = DEVICE_DT_GET(DT_NODELABEL(lora_sim1));

ZTEST(attachment_head, test_relay_is_not_cancelled_by_a_copy_masked_by_the_brains_tx)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_attachment_head_stats st;
	uint32_t rx_ms;

	meshtastic_attachment_head_set_brain(BRAIN_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	zassert_ok(lora_sim_set_cosite(lora_dev, other_radio), "paired: the brain's radio beside us");

	rx_ms = hear_and_forward(wire, sizeof(wire), 0x74U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 500U, MESHTASTIC_ATTACHMENT_DUPE_CANCEL, 25U));
	k_msleep(50);

	/* The brain keys up (300 ms of air); a peer's copy lands meanwhile. */
	lora_sim_set_busy(other_radio, 300U);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x57U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	zassert_equal(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), -ECANCELED,
		      "blanked: we cannot hear while the brain transmits");
	zassert_equal(lora_sim_rx_blanked(lora_dev), 1U);
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "nothing to forward: never heard");

	/* Our relay still keys up: nothing we heard says a peer relayed it. */
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "relayed");
	expect_result(25U, 0);
	meshtastic_attachment_head_stats_get(&st);
	zassert_equal(st.tx_cancelled, 0U, "no cancel on a copy we could not hear");

	/* After the brain's air, we hear again. */
	k_msleep(50);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), "unblanked");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	lora_sim_reset(other_radio);
}
