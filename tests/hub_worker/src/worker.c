/* SPDX-License-Identifier: GPL-3.0 */

/*
 * The keyless worker (ATTACHMENT-DESIGN S5), on the sim radio.
 *
 * The real stack runs as a worker. What its radio hears is captured at the send
 * seam (meshtastic_worker_send, overridden here) as the RX_FRAME
 * envelope the hub would receive; the hub's controls arrive through
 * meshtastic_worker_on_envelope, exactly as the BLE glue delivers
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
#include "meshtastic_link_codec.h"
#include "meshtastic_worker.h"
#include "meshtastic_packet.h"
#include "meshtastic_contention.h"
#include <zephyr/sys/byteorder.h>
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_preset.h"
#include "meshtastic_region_presets.h"

#define TEST_NODE_ID 0x0A0A0A0AU /* the worker's link identity */
#define HUB_NODE   0x121F0001U
#define STRANGER     0x0BAD0001U
#define PRESET_ST meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO
#define PRESET_MF meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* What the worker sent to its hub. */
static struct {
	struct k_sem sem;
	uint32_t to;
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	size_t len;
	uint32_t count;
	uint32_t under_lock; /* sends made while another thread could not take worker_lock */
	/* Every TX_RESULT sent, in order: a result and the forward of the copy that
	 * cancelled it are sent by two threads and may reach the bearer in either
	 * order, so the last envelope alone cannot answer "was seq N reported". */
	struct {
		uint16_t seq;
		int8_t rc;
	} results[8];
	uint32_t n_results;
} sent;

static enum meshtastic_link_auth test_auth = MESHTASTIC_LINK_AUTH_ENCRYPTED;

/* agents-z7rg: a send to the hub enters the BLE peer module, whose RX thread
 * delivers the hub's envelopes into the worker under the worker's lock. A send made
 * holding worker_lock is half of an AB-BA deadlock (rzr5, 2026-11-24). The probe
 * asks another thread to take worker_lock (through a public call that locks it)
 * while the send is in progress; if it cannot within PROBE_MS, the sender holds
 * it. Sequence numbers keep a probe that finishes late from answering the next. */
#define PROBE_MS 100
static K_SEM_DEFINE(probe_go, 0, 1);
static atomic_t probe_req;
static atomic_t probe_done;

static void probe_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		atomic_val_t seq;

		k_sem_take(&probe_go, K_FOREVER);
		seq = atomic_get(&probe_req);
		(void)meshtastic_worker_get_hub(); /* takes worker_lock */
		atomic_set(&probe_done, seq);
	}
}
K_THREAD_DEFINE(worker_lock_probe, 1024, probe_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, 0);

static bool worker_lock_free_for_others(void)
{
	atomic_val_t seq = atomic_inc(&probe_req) + 1;
	int64_t end = k_uptime_get() + PROBE_MS;

	k_sem_give(&probe_go);
	while (atomic_get(&probe_done) != seq) {
		if (k_uptime_get() >= end) {
			return false;
		}
		k_msleep(1);
	}
	return true;
}

static int test_bearer_send(uint32_t hub, const uint8_t *env, size_t len)
{
	if (len > sizeof(sent.env)) {
		return -EMSGSIZE;
	}
	if (!worker_lock_free_for_others()) {
		sent.under_lock++;
	}
	{
		struct meshtastic_link_msg m;

		if (meshtastic_link_decode(env, len, &m) >= 0 &&
		    m.type == MESHTASTIC_LINK_TX_RESULT) {
			unsigned int i = sent.n_results++ % ARRAY_SIZE(sent.results);

			sent.results[i].seq = m.u.result.tx_seq;
			sent.results[i].rc = m.u.result.rc;
		}
	}
	sent.to = hub;
	memcpy(sent.env, env, len);
	sent.len = len;
	sent.count++;
	k_sem_give(&sent.sem);
	return 0;
}

static bool test_bearer_link_info(uint32_t node, struct meshtastic_link_info *out)
{
	ARG_UNUSED(node);
	out->up = true;
	out->auth = test_auth;
	out->takes_envelopes = true;
	out->mtu = 20U;
	out->rtt_ms = 0U;
	return true;
}

static const struct meshtastic_link_bearer test_bearer = {
	.name = "test",
	.send = test_bearer_send,
	.link_info = test_bearer_link_info,
};

/* A frame is any bytes with a header: the worker never looks inside. */
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

static void *worker_setup(void)
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
	zassert_ok(meshtastic_link_bearer_register(&test_bearer), "test bearer");
	return NULL;
}

static void worker_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_worker_reset();
	test_auth = MESHTASTIC_LINK_AUTH_ENCRYPTED;
	zassert_ok(meshtastic_preset_switch(PRESET_ST, NULL), "preset");
	lora_sim_reset(lora_dev);
	wait_rx_armed();
	k_sem_reset(&sent.sem);
	memset(&sent, 0, sizeof(sent));
	k_sem_init(&sent.sem, 0, 16);
	meshtastic_worker_set_hub(HUB_NODE);
	/* set_hub introduces us with a STATUS; let it pass. */
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	k_sem_reset(&sent.sem);
	memset(&sent, 0, sizeof(sent));
	k_sem_init(&sent.sem, 0, 16);
}

ZTEST_SUITE(worker, NULL, worker_setup, worker_before, NULL, NULL);

/* H1: what the radio hears goes to the hub as RX_FRAME, with the signal and
 * preset it was heard on and the wire bytes untouched. */
ZTEST(worker, test_heard_frame_reaches_the_hub_untouched)
{
	uint8_t wire[48];
	struct meshtastic_link_msg msg;
	struct meshtastic_worker_stats st;

	some_frame(wire, sizeof(wire), 0x10U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -77, 9), "inject");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	zassert_equal(sent.to, HUB_NODE);
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg), "decode");
	zassert_equal(msg.type, MESHTASTIC_LINK_RX_FRAME);
	zassert_equal(msg.u.rx.preset, (uint8_t)PRESET_ST, "heard on ShortTurbo");
	zassert_equal(msg.u.rx.rssi, -77);
	zassert_equal(msg.u.rx.snr, 9);
	zassert_equal(msg.u.rx.wire_len, sizeof(wire));
	zassert_mem_equal(msg.u.rx.wire, wire, sizeof(wire), "byte for byte");

	meshtastic_worker_stats_get(&st);
	zassert_equal(st.heard, 1U);
	zassert_equal(st.forwarded, 1U);
	zassert_equal(st.send_failed, 0U);
}

/* H1b: a worker without a hub hears and drops, and says so. */
ZTEST(worker, test_no_hub_no_forwarding)
{
	uint8_t wire[32];
	struct meshtastic_worker_stats st;

	meshtastic_worker_set_hub(0U);
	some_frame(wire, sizeof(wire), 0x20U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -60, 4));
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "nothing sent");
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.heard, 1U);
	zassert_equal(st.no_hub, 1U);
	zassert_equal(st.forwarded, 0U);
}

/* H3: SET_PRESET retunes the radio -- from the hub only -- and the hub
 * learns the outcome from the STATUS that follows. */
ZTEST(worker, test_set_preset_from_the_hub_only)
{
	uint8_t env[8];
	int len = meshtastic_link_encode_set_preset((uint8_t)PRESET_MF, env, sizeof(env));
	struct meshtastic_link_msg msg;
	struct meshtastic_worker_stats st;
	uint32_t freq;
	uint8_t sf, bw;
	struct meshtastic_modem_params want;

	zassert_true(len > 0);

	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, STRANGER, env, (size_t)len), -EPERM,
		      "a stranger does not retune this radio");
	zassert_equal(mt.modem_preset, PRESET_ST, "still ShortTurbo");
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(200)), -EAGAIN, "and gets no STATUS");

	zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	zassert_equal(mt.modem_preset, PRESET_MF, "retuned");
	zassert_ok(meshtastic_preset_to_params(PRESET_MF, false, &want));
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw));
	zassert_equal(sf, want.spread_factor, "the radio is on MediumFast's SF (%u)", sf);

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "STATUS follows the retune");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);
	zassert_equal(msg.u.status.preset, (uint8_t)PRESET_MF);
	zassert_equal(msg.u.status.hwid, TEST_NODE_ID, "the worker's own id");
	zassert_equal(msg.u.status.hub, HUB_NODE);
	zassert_true((msg.u.status.flags & MESHTASTIC_LINK_ST_IS_WORKER) != 0U, "a worker");
	zassert_true((msg.u.status.flags & MESHTASTIC_LINK_ST_RX_ONLY) == 0U,
		     "a worker with a transmitter says so (P3): the hub hands it frames");
	zassert_true((msg.u.status.flags & MESHTASTIC_LINK_ST_TX_ENABLED) != 0U, "tx enabled");

	meshtastic_worker_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.controls, 1U);

	/* H10 (SCOPE C4): the hub's own id over a link the bearer does not
	 * vouch for (an unbonded connection) is refused, and counted apart. */
	test_auth = MESHTASTIC_LINK_AUTH_NONE;
	len = meshtastic_link_encode_set_preset((uint8_t)PRESET_ST, env, sizeof(env));
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len),
		      -EACCES, "untrusted link");
	zassert_equal(mt.modem_preset, PRESET_MF, "not retuned");
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.untrusted, 1U);
	zassert_equal(st.controls, 1U);
	test_auth = MESHTASTIC_LINK_AUTH_ENCRYPTED;
	len = meshtastic_link_encode_set_preset((uint8_t)PRESET_MF, env, sizeof(env));

	/* An out-of-range preset is a bad envelope, not a retune. */
	env[1] = 0xEEU;
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len),
		      -EBADMSG);
	zassert_equal(mt.modem_preset, PRESET_MF);
}

/* H4: what a worker does not take: TX_FRAME (phase 3), and the envelopes a worker
 * itself sends. */
ZTEST(worker, test_controls_a_worker_refuses)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	uint8_t wire[20];
	struct meshtastic_worker_stats st;
	int len;

	some_frame(wire, sizeof(wire), 0x30U);
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 1U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_equal(meshtastic_link_bearer_rx(&test_bearer, STRANGER, env, (size_t)len),
			      -EPERM, "only the hub hands a worker frames to transmit");
		zassert_equal(lora_sim_tx_pending(lora_dev), 0, "nothing on the air");
	}
	{
		const struct meshtastic_link_rx_frame rx = {
			.preset = (uint8_t)PRESET_ST, .rssi = -50, .snr = 1, .rx_ms = 1U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_rx_frame(&rx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_equal(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len),
			      -EBADMSG, "a worker sends RX_FRAME, it does not take one");
	}
	env[0] = 0x7FU;
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, 3U), -EBADMSG);
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, 0U, env, 3U), -EINVAL);

	meshtastic_worker_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.rejected, 2U);
}

/* H5: a worker originates nothing: the outbound queue refuses every caller. */
ZTEST(worker, test_a_worker_originates_nothing)
{
	uint8_t wire[40];

	some_frame(wire, sizeof(wire), 0x40U);
	zassert_equal(meshtastic_radio_send_wire(wire, sizeof(wire)), -EPERM);
	zassert_equal(meshtastic_radio_send_wire_after(wire, sizeof(wire), 0U, 10U), -EPERM);
	k_sleep(K_MSEC(200));
	zassert_equal(lora_sim_tx_pending(lora_dev), 0, "nothing on the air");
}

/* H7b (fix 11, review X4): the bearer's link-up to the hub sends STATUS at
 * once -- the boot-time introduction is lost before any link exists. A
 * link-up to anyone else sends nothing. */
ZTEST(worker, test_status_on_link_up_to_the_hub)
{
	struct meshtastic_link_msg msg;

	meshtastic_link_bearer_link_up(&test_bearer, STRANGER);
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "a stranger's link: nothing");
	meshtastic_link_bearer_link_up(&test_bearer, HUB_NODE);
	zassert_ok(k_sem_take(&sent.sem, K_MSEC(500)), "the hub's link: STATUS at once");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);
	zassert_equal(msg.u.status.hub, HUB_NODE);
}

/* H7: STATUS on the timer (5 s in this build), and on demand. */
ZTEST(worker, test_status_on_the_timer_and_on_demand)
{
	struct meshtastic_link_msg msg;

	zassert_ok(meshtastic_worker_status_send(), "on demand");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)));
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);
	zassert_equal(msg.u.status.preset, (uint8_t)PRESET_ST);
	zassert_equal(msg.u.status.rx_frames, 0U);

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(7)), "the timer's STATUS");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);

	meshtastic_worker_set_hub(0U);
	zassert_equal(meshtastic_worker_status_send(), -EHOSTUNREACH);
}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
/* H8 (SCOPE §7, review F4): the identity in flash is neither loaded nor
 * touched by a worker. The records a node leaves are written raw (their content
 * is irrelevant: a worker must never decode them), the worker loads settings, and
 * afterwards no channel holds a key, the security section is empty, a save by
 * the worker leaves the records byte for byte as they were, and the worker's own
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

ZTEST(worker, test_identity_in_flash_is_never_loaded_or_touched)
{
	struct meshtastic_channel_key key;
	meshtastic_Config sec;

	/* what a node left behind */
	zassert_ok(settings_save_one("meshtastic/channel/0", keyed_channel, sizeof(keyed_channel)));
	zassert_ok(settings_save_one("meshtastic/channel/2", keyed_channel, sizeof(keyed_channel)));
	zassert_ok(settings_save_one("meshtastic/config/security", keyed_security,
				     sizeof(keyed_security)));
	zassert_ok(settings_save_one("meshtastic/module/mqtt", keyed_module, sizeof(keyed_module)));

	/* the worker boots (loads) over it */
	zassert_ok(settings_load(), "settings_load");

	/* nothing reached RAM */
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		zassert_true(meshtastic_channels_get_key(i, &key) < 0, "slot %u holds a key", i);
	}
	zassert_true(meshtastic_channels_primary_key(&key) < 0, "no primary PSK in RAM");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_security_tag, &sec));
	zassert_equal(sec.payload_variant.security.private_key.size, 0U, "no private key in RAM");
	zassert_equal(meshtastic_config_store_setting_get("channel/0", (void *)key.bytes,
							  sizeof(key.bytes)),
		      -ENOENT, "a worker answers no channel record");

	/* the worker saves its own state: the identity is not rewritten, not wiped */
	meshtastic_worker_set_hub(0x12345678U);
	zassert_ok(settings_save(), "settings_save");
	raw_expect("meshtastic/channel/0", keyed_channel, sizeof(keyed_channel));
	raw_expect("meshtastic/channel/2", keyed_channel, sizeof(keyed_channel));
	raw_expect("meshtastic/config/security", keyed_security, sizeof(keyed_security));
	raw_expect("meshtastic/module/mqtt", keyed_module, sizeof(keyed_module));
	{
		struct raw_read r = { .found = false };

		zassert_ok(settings_load_subtree_direct("mtattach/brain", raw_read_cb, &r));
		zassert_true(r.found, "the worker's own record was written");
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

/* P3 slice 1: a TX_FRAME from the hub over a trusted link goes on the air
 * byte for byte -- the worker keys up what the hub built, through the one
 * transmit funnel -- and a TX_RESULT comes back when asked. Over a link the
 * bearer does not vouch for, it is refused (the SET_PRESET gate). */
ZTEST(worker, test_tx_frame_from_the_hub_is_transmitted)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_link_msg msg;
	struct meshtastic_worker_stats st;
	int len;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500)); /* the introductory STATUS */
	wait_rx_armed();
	some_frame(wire, sizeof(wire), 0x50U);

	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = MESHTASTIC_LINK_TXF_WANT_RESULT,
			.tx_seq = 7U, .wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len),
			   "accepted from the hub");
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "the frame went on the air");
	zassert_equal(f.len, sizeof(wire));
	zassert_mem_equal(f.data, wire, sizeof(wire), "byte for byte");

	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "TX_RESULT sent");
	zassert_equal(sent.to, HUB_NODE);
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_TX_RESULT);
	zassert_equal(msg.u.result.tx_seq, 7U);
	zassert_equal(msg.u.result.rc, 0, "transmitted (%d)", msg.u.result.rc);
	zassert_equal(msg.u.result.defers, 0U);

	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_sent, 1U);
	zassert_equal(st.controls, 1U);

	/* No result asked for: none sent. */
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 8U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "on the air");
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "no TX_RESULT unasked");

	/* The twin: the hub over an untrusted link is refused. */
	test_auth = MESHTASTIC_LINK_AUTH_NONE;
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = 0U, .tx_seq = 9U,
			.wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_equal(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len),
			      -EACCES, "untrusted link");
	}
	test_auth = MESHTASTIC_LINK_AUTH_ENCRYPTED;
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing on the air");
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_sent, 2U);
	zassert_equal(st.untrusted, 1U);
}

/* P3 slice 2: an own frame of the hub's (OWN_DELAY) waits the reference's
 * own-TX contention window, drawn on THIS radio, before it keys up -- and then
 * goes out, with the result reporting the wait as no defer. */
ZTEST(worker, test_own_delay_is_drawn_on_the_worker)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_link_msg msg;
	int64_t t0;
	int len;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	some_frame(wire, sizeof(wire), 0x60U);
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST,
			.flags = MESHTASTIC_LINK_TXF_WANT_RESULT | MESHTASTIC_LINK_TXF_OWN_DELAY,
			.tx_seq = 11U, .wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		t0 = k_uptime_get();
		zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "keyed up after the window");
	zassert_true(k_uptime_get() - t0 < 1000, "within the widest own-TX window (%lld ms)",
		     (long long)(k_uptime_get() - t0));
	zassert_mem_equal(f.data, wire, sizeof(wire));
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "TX_RESULT");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_TX_RESULT);
	zassert_equal(msg.u.result.tx_seq, 11U);
	zassert_equal(msg.u.result.rc, 0);
	zassert_equal(msg.u.result.defers, 0U, "a contention wait is not a defer");
}

/* P3 slice 3, the worker's side of a relay (ATTACHMENT-DESIGN §12): the window runs
 * on THIS radio's clock of reception -- rx_ms + not_before -- and this radio's
 * own hearing decides: a copy heard again before key-up cancels (CANCEL) or
 * clamps to the end of the window (LATE); the hub's TX_CANCEL withdraws. */
static uint32_t hear_and_forward(uint8_t *wire, uint8_t len, uint8_t seed)
{
	struct meshtastic_link_msg msg;

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
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_RX_FRAME);
	return msg.u.rx.rx_ms;
}

static int hand_relay(const uint8_t *wire, uint8_t len, uint32_t rx_ms, uint32_t not_before,
		      uint8_t dupe, uint16_t seq)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	uint8_t relay[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)relay;
	int elen;

	memcpy(relay, wire, len);
	h->flags = (uint8_t)((h->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	h->relay_node = (uint8_t)(HUB_NODE & 0xFFU);
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST,
			.flags = MESHTASTIC_LINK_TXF_WANT_RESULT | MESHTASTIC_LINK_TXF_RELAY,
			.tx_seq = seq,
			.relay_src = sys_le32_to_cpu(h->src),
			.relay_id = sys_le32_to_cpu(h->id),
			.rx_ms = rx_ms,
			.not_before_ms = not_before,
			.dupe = dupe,
			.wire = relay,
			.wire_len = len,
		};
		elen = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
	}
	zassert_true(elen > 0);
	return meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)elen);
}

static void expect_result(uint16_t seq, int rc)
{
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(3)), "TX_RESULT");
	for (uint32_t k = 0U; k < MIN(sent.n_results, ARRAY_SIZE(sent.results)); k++) {
		if (sent.results[k].seq == seq) {
			zassert_equal(sent.results[k].rc, rc, "rc %d", sent.results[k].rc);
			return;
		}
	}
	zassert_unreachable("no TX_RESULT for seq %u", seq);
}

ZTEST(worker, test_relay_keys_up_on_the_workers_clock)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	uint32_t rx_ms;
	int64_t t_tx;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x70U);

	/* A relay of it: not before rx_ms + 300 ms, KEEP. */
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 300U, MESHTASTIC_LINK_DUPE_KEEP, 21U));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "keyed up");
	t_tx = k_uptime_get();
	zassert_true(t_tx - (int64_t)rx_ms >= 290, "not before the window (%lld ms after rx)",
		     (long long)(t_tx - (int64_t)rx_ms));
	zassert_true(t_tx - (int64_t)rx_ms < 1200, "and not long after (%lld ms)",
		     (long long)(t_tx - (int64_t)rx_ms));
	{
		const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)f.data;

		zassert_equal(h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 2U, "the relay as built");
		zassert_equal(h->relay_node, (uint8_t)(HUB_NODE & 0xFFU), "the hub's relay byte");
	}
	expect_result(21U, 0);
}

ZTEST(worker, test_relay_cancels_or_clamps_on_a_copy_heard_first)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_worker_stats st;
	uint32_t rx_ms;
	int64_t t0;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();

	/* CANCEL: a neighbour's copy lands inside our window -> we drop ours. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x71U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 400U, MESHTASTIC_LINK_DUPE_CANCEL, 22U));
	k_msleep(100);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x55U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), "a peer relayed it first");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "that copy is forwarded too");
	expect_result(22U, MESHTASTIC_LINK_RC_CANCELLED);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing keyed up");
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_cancelled, 1U);

	/* LATE: the same, but a router-late relay still goes -- after the window. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x72U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 200U, MESHTASTIC_LINK_DUPE_LATE, 23U));
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
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_late, 1U);
}

ZTEST(worker, test_tx_cancel_withdraws_a_relay)
{
	uint8_t wire[40];
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	struct lora_sim_frame f;
	uint32_t rx_ms;
	int elen;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x73U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 500U, MESHTASTIC_LINK_DUPE_KEEP, 24U));
	k_msleep(100);
	elen = meshtastic_link_encode_tx_cancel(0x0D0D0D0DU, 0x3B00U + 0x73U, env, sizeof(env));
	zassert_true(elen > 0);
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, STRANGER, env, (size_t)elen), -EPERM,
		      "only the hub withdraws");
	zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)elen));
	expect_result(24U, MESHTASTIC_LINK_RC_CANCELLED);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "withdrawn");
}

/* SET_POLICY: the hub sets its worker's transmit power -- a worker runs no phone
 * service, so this is the only path -- through the config store (applied,
 * persisted, region-clamped), and the worker reports. A stranger is refused. */
ZTEST(worker, test_set_policy_sets_tx_power_from_the_hub_only)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	struct meshtastic_link_msg msg;
	struct meshtastic_worker_stats st;
	const struct meshtastic_link_policy pol = {
		.flags = MESHTASTIC_LINK_POL_HAS_TX_POWER, .tx_power = 2,
	};
	int len;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	len = meshtastic_link_encode_set_policy(&pol, env, sizeof(env));
	zassert_true(len > 0);
	zassert_equal(meshtastic_link_bearer_rx(&test_bearer, STRANGER, env, (size_t)len), -EPERM);
	zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	zassert_equal(mt.tx_power, 2, "applied now (%d dBm)", mt.tx_power);
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "STATUS follows");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.refused, 1U);
	zassert_equal(st.controls, 1U);
}

/* R3's sim half (the co-site desense model, zt-ble-scan-claude's lora_sim
 * pairing): a worker in one enclosure with its hub is DEAF while the hub
 * transmits. The stack's behaviour under that blanking: a neighbour's copy
 * of a frame we hold a CANCEL relay of lands during the hub's transmission,
 * so we never hear it -- and our relay must still go, because cancel-on-
 * duplicate is about what we HEARD, not what was on the air. */
static const struct device *const other_radio = DEVICE_DT_GET(DT_NODELABEL(lora_sim1));

ZTEST(worker, test_relay_is_not_cancelled_by_a_copy_masked_by_the_hubs_tx)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_worker_stats st;
	uint32_t rx_ms;

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	zassert_ok(lora_sim_set_cosite(lora_dev, other_radio), "paired: the hub's radio beside us");

	rx_ms = hear_and_forward(wire, sizeof(wire), 0x74U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, 500U, MESHTASTIC_LINK_DUPE_CANCEL, 25U));
	k_msleep(50);

	/* The hub keys up (300 ms of air); a peer's copy lands meanwhile. */
	lora_sim_set_busy(other_radio, 300U);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x57U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	zassert_equal(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), -ECANCELED,
		      "blanked: we cannot hear while the hub transmits");
	zassert_equal(lora_sim_rx_blanked(lora_dev), 1U);
	zassert_equal(k_sem_take(&sent.sem, K_MSEC(300)), -EAGAIN, "nothing to forward: never heard");

	/* Our relay still keys up: nothing we heard says a peer relayed it. */
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "relayed");
	expect_result(25U, 0);
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_cancelled, 0U, "no cancel on a copy we could not hear");

	/* After the hub's air, we hear again. */
	k_msleep(50);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 9), "unblanked");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	lora_sim_reset(other_radio);
}

/* D3 as revised (ATTACHMENT-SCOPE §6, tooling 7d5f2a7): a CLIENT relay whose window had
 * already closed when the decision reached the worker applies its own cancel test. Three
 * cases: late with no copy heard -> relayed now; late with a copy heard -> cancelled (the
 * ordinary cancel-on-dupe); past orig_rx + 2 * worst(snr) -> stale, dropped with -ETIME and
 * its own counter. A ROUTER_LATE relay in the stale position still goes.
 * Named to sort LAST: ztest runs cases in name order, the worker's periodic STATUS fires
 * every 5 s of suite time, and test_status_on_link_up_to_the_hub asserts that a
 * stranger's link-up sends nothing within 300 ms -- a case inserted earlier in the
 * order moved that timer into its window (2026-09-29). */
/* Like expect_result, but skips envelopes that are not a TX_RESULT (the worker's periodic
 * STATUS lands inside this case's long sleeps). */
static void expect_result_skipping_status(uint16_t seq, int rc)
{
	struct meshtastic_link_msg msg;

	for (int i = 0; i < 6; i++) {
		zassert_ok(k_sem_take(&sent.sem, K_SECONDS(3)), "an envelope");
		zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
		if (msg.type != MESHTASTIC_LINK_TX_RESULT) {
			continue;
		}
		zassert_equal(msg.u.result.tx_seq, seq, "seq %u, got %u", seq, msg.u.result.tx_seq);
		zassert_equal(msg.u.result.rc,
			      rc == MESHTASTIC_LINK_RC_CANCELLED ? rc : (int8_t)CLAMP(rc, -127, 127),
			      "rc %d, got %d", rc,
			      (int)msg.u.result.rc);
		return;
	}
	zassert_true(false, "no TX_RESULT for seq %u", seq);
}

static uint32_t test_relay_worst_ms(int8_t snr)
{
	return meshtastic_contention_delay_relay_worst_ms(
		snr, meshtastic_contention_effective_slot_ms(mt.modem.spread_factor,
							     mt.modem.bandwidth_hz, false));
}

ZTEST(worker, test_zz_late_client_relay_applies_its_own_cancel_test)
{
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_worker_stats st;
	uint32_t rx_ms;
	const int8_t snr = 9;
	const uint32_t worst = test_relay_worst_ms(snr);
	const uint32_t window = MAX(worst / 2U, 40U);

	meshtastic_worker_set_hub(HUB_NODE);
	(void)k_sem_take(&sent.sem, K_MSEC(500));
	wait_rx_armed();
	zassert_true(worst >= 80U, "worst-case window %u ms too small for the timing here", worst);

	/* (a) Late, no copy heard, inside the staleness cap: relayed now. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x73U);
	k_msleep(window + 30U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, window, MESHTASTIC_LINK_DUPE_CANCEL, 24U));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "a late CLIENT relay with no copy heard goes out");
	expect_result_skipping_status(24U, 0);

	/* (b) Late, a peer's copy heard meanwhile: cancelled, never sent. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x74U);
	k_msleep(window / 2U);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x57U;
	((struct meshtastic_wire_header *)wire)->flags =
		(uint8_t)((((struct meshtastic_wire_header *)wire)->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, snr), "a peer relayed it first");
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "that copy is forwarded too");
	k_msleep(window / 2U + 30U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, window, MESHTASTIC_LINK_DUPE_CANCEL, 25U));
	expect_result_skipping_status(25U, MESHTASTIC_LINK_RC_CANCELLED);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing keyed up");

	/* (c) Past orig_rx + 2 * worst: stale, dropped. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x75U);
	k_msleep(2U * worst + 60U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, window, MESHTASTIC_LINK_DUPE_CANCEL, 26U));
	expect_result_skipping_status(26U, -ETIME);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing keyed up");
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_late_dropped, 1U, "only the stale one is counted as dropped-late");

	/* The twin: ROUTER_LATE in the stale position still relays. */
	rx_ms = hear_and_forward(wire, sizeof(wire), 0x76U);
	k_msleep(2U * worst + 60U);
	zassert_ok(hand_relay(wire, sizeof(wire), rx_ms, window, MESHTASTIC_LINK_DUPE_LATE, 27U));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "a late ROUTER relay still goes");
	expect_result_skipping_status(27U, 0);
	meshtastic_worker_stats_get(&st);
	zassert_equal(st.tx_late_dropped, 1U, "not counted for a router");
}

/* agents-z7rg: no send to the hub is made holding worker_lock -- the forward of a
 * heard frame (the BLE worker), STATUS on demand (the shell's path), STATUS after
 * a control (the BT RX thread, inside the hub's own envelope) and TX_RESULT
 * (worker_tx, the thread named in the rzr5 assert). Before the fix every one of
 * them held it. */
ZTEST(worker, test_no_send_to_the_hub_holds_the_worker_lock)
{
	uint8_t env[MESHTASTIC_LINK_ENV_MAX];
	uint8_t wire[40];
	struct lora_sim_frame f;
	struct meshtastic_link_msg msg;
	int len;

	/* Forward. */
	some_frame(wire, sizeof(wire), 0x61U);
	zassert_ok(lora_sim_inject(lora_dev, wire, sizeof(wire), -70, 5));
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(2)), "forwarded");
	zassert_equal(sent.under_lock, 0U, "RX_FRAME forwarded under worker_lock");

	/* STATUS on demand. */
	zassert_ok(meshtastic_worker_status_send());
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)));
	zassert_equal(sent.under_lock, 0U, "STATUS on demand sent under worker_lock");

	/* STATUS after a control from the hub. */
	len = meshtastic_link_encode_set_preset((uint8_t)PRESET_ST, env, sizeof(env));
	zassert_true(len > 0);
	zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "STATUS follows the control");
	zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_LINK_STATUS);
	zassert_equal(sent.under_lock, 0U, "STATUS after a control sent under worker_lock");

	/* TX_RESULT from worker_tx. */
	wait_rx_armed();
	some_frame(wire, sizeof(wire), 0x62U);
	{
		const struct meshtastic_link_tx_frame tx = {
			.preset = (uint8_t)PRESET_ST, .flags = MESHTASTIC_LINK_TXF_WANT_RESULT,
			.tx_seq = 21U, .wire = wire, .wire_len = sizeof(wire),
		};
		len = meshtastic_link_encode_tx_frame(&tx, env, sizeof(env));
		zassert_true(len > 0);
		zassert_ok(meshtastic_link_bearer_rx(&test_bearer, HUB_NODE, env, (size_t)len));
	}
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "on the air");
	/* The 5 s STATUS timer may land first: skip it. */
	for (int i = 0; i < 3; i++) {
		zassert_ok(k_sem_take(&sent.sem, K_SECONDS(1)), "TX_RESULT sent");
		zassert_ok(meshtastic_link_decode(sent.env, sent.len, &msg));
		if (msg.type == MESHTASTIC_LINK_TX_RESULT) {
			break;
		}
	}
	zassert_equal(msg.type, MESHTASTIC_LINK_TX_RESULT);
	zassert_equal(sent.under_lock, 0U, "TX_RESULT sent under worker_lock");
}
