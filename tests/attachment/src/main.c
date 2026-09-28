/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The brain's side of the attachment architecture (ATTACHMENT-DESIGN S1/S6), on
 * native_sim with the sim LoRa radio: one real stack, its radio heads scripted.
 *
 * What a head does is inject RX_FRAME envelopes -- an undecoded airframe plus the
 * signal it saw and the preset it sat on -- and the brain must treat each one as
 * a frame that crossed the air: dedup across heads, RF signal bookkeeping, NodeDB,
 * modules, and the own-voice guard; and it must NOT flood-relay a head's frame
 * onto its own radio (that is on another preset) until P3 gives relays a target.
 *
 * Phase 0 cases. The per-preset hash (a LongFast head's frame decrypting on a
 * ShortTurbo brain) is P1's and is not here yet.
 */
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic_attachment.h"
#include "meshtastic_attachment_codec.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define FAR_NODE_ID  0x0D0D0D0DU
#define HEAD1_NODE   0x00E10001U /* a head's link identity (its hw node number) */
#define HEAD2_NODE   0x00E10002U
#define PRESET_ST meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* What the stack delivered to the application. */
static struct {
	struct k_sem sem;
	uint32_t from;
	uint32_t portnum;
	int16_t rssi;
	int8_t snr;
	uint32_t count;
} rx;

static void on_recv(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t payload_len, int16_t rssi, int8_t snr)
{
	ARG_UNUSED(to);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
	rx.from = from;
	rx.portnum = portnum;
	rx.rssi = rssi;
	rx.snr = snr;
	rx.count++;
	k_sem_give(&rx.sem);
}

/* The brain's send seam, captured: what would have gone to a head. */
static struct {
	uint32_t node;
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	size_t len;
	uint32_t count;
} sent;

int meshtastic_attachment_send(uint32_t node, const uint8_t *env, size_t len)
{
	sent.node = node;
	sent.len = MIN(len, sizeof(sent.env));
	memcpy(sent.env, env, sent.len);
	sent.count++;
	return 0;
}

static void wait_rx_armed(void)
{
	for (int i = 0; i < 200 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(5);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never armed RX");
}

static void drain_radio(void)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(50)) == 0) {
	}
}

/* A frame from @p from, built with OUR channel table as it stands. */
static void build_frame(uint32_t from, uint32_t to, uint32_t id, const char *text, uint8_t *wire,
			uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)text,
		.payload_len = strlen(text),
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = 0U,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len), "build_wire_packet");
}

/* What a head does: the frame it heard, its preset, its signal -- as an envelope,
 * through the brain's ingest, exactly the path the BLE glue takes. */
static int head_hears(uint32_t head, uint8_t preset, int16_t rssi, int8_t snr,
		      const uint8_t *wire, uint32_t wire_len)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_rx_frame m = {
		.preset = preset,
		.rssi = rssi,
		.snr = snr,
		.rx_ms = (uint32_t)k_uptime_get(),
		.wire = wire,
		.wire_len = (uint16_t)wire_len,
	};
	int len = meshtastic_attachment_encode_rx_frame(&m, env, sizeof(env));

	zassert_true(len > 0, "encode rx_frame (%d)", len);
	return meshtastic_attachment_ingest(head, env, (size_t)len);
}

static void assert_not_relayed(uint32_t src, uint32_t id)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(300)) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		zassert_false(sys_le32_to_cpu(h->src) == src && sys_le32_to_cpu(h->id) == id,
			      "a head's frame was relayed onto our own air");
	}
}

/* Slot 0 holding the active preset's default channel (what stock stores). */
static void set_default_primary(void)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	const uint8_t psk[1] = { 0x01U };

	ch.index = 0;
	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	memcpy(ch.settings.psk.bytes, psk, 1U);
	zassert_ok(meshtastic_channels_set_slot(0U, &ch), "set_slot 0");
}

static void *attachment_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_US,
	};

	k_sem_init(&rx.sem, 0, 16);
	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	meshtastic_set_recv_cb(on_recv);
	return NULL;
}

static void attachment_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* The brain on the bench: ShortTurbo, on that preset's default channel. */
	set_default_primary();
	zassert_ok(meshtastic_preset_switch(PRESET_ST, NULL), "preset");
	meshtastic_sched_defaults();
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	lora_sim_reset(lora_dev);
	wait_rx_armed();
	drain_radio();
	k_sem_reset(&rx.sem);
	rx.count = 0U;
	memset(&sent, 0, sizeof(sent));
	mt.status.last_rssi = 0;
	mt.status.last_snr = 0;
	mt.status.last_rx_from = 0U;
	mt.status.self_heard = 0U;
	/* Fresh dup cache per case so the same (src,id) can be reused. */
	memset(mt.dup_cache, 0, sizeof(mt.dup_cache));
	mt.dup_head = 0U;
}

ZTEST_SUITE(attachment, NULL, attachment_setup, attachment_before, NULL, NULL);

/* T0: the table starts with the local radio and nothing else. */
ZTEST(attachment, test_0_table_starts_with_the_local_radio)
{
	struct meshtastic_attachment_info a;

	zassert_equal(meshtastic_attachment_count(), 1U);
	zassert_true(meshtastic_attachment_get(0U, &a));
	zassert_equal(a.id, 0U);
	zassert_equal(a.preset, (uint8_t)PRESET_ST, "attachment 0 reports the active preset");
	zassert_false(meshtastic_attachment_get(1U, &a), "no head yet");
}

/* T2 (+RF semantics): a head's frame is delivered AND counts as RF -- the signal
 * the head saw becomes the node's last_rssi/snr, which a BLE-peer inject never
 * does. The head is admitted into the table on first contact. */
ZTEST(attachment, test_head_frame_is_delivered_as_rf)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_info a;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1001U, "via head", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len), "ingest");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_equal(rx.from, FAR_NODE_ID);
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE);
	zassert_equal(rx.rssi, -90, "the application sees the head's signal (%d)", rx.rssi);
	zassert_equal(rx.snr, 5);

	zassert_equal(mt.status.last_rssi, -90, "the head's RSSI is the node's (%d)",
		      mt.status.last_rssi);
	zassert_equal(mt.status.last_snr, 5);
	zassert_equal(mt.status.last_rx_from, FAR_NODE_ID);

	zassert_equal(meshtastic_attachment_count(), 2U, "the head was admitted");
	zassert_equal(meshtastic_attachment_id_for_node(HEAD1_NODE), 1U);
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.node, HEAD1_NODE);
	zassert_equal(a.preset, (uint8_t)PRESET_ST);
	zassert_equal(a.rx_frames, 1U);
	zassert_equal(a.last_rssi, -90);
}

/* T2b: two heads hear the same frame -- delivered once (the dup cache is keyed
 * on (src,id), not on the radio), both heads see their count, and the FIRST
 * arrival's signal is what the node records. */
ZTEST(attachment, test_two_heads_same_frame_delivered_once)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_info a;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1002U, "twice", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -95, 2, wire, len), "head 1");
	zassert_ok(head_hears(HEAD2_NODE, PRESET_ST, -70, 9, wire, len), "head 2");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered once");
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(500)), -EAGAIN, "not delivered twice");
	zassert_equal(rx.count, 1U);
	zassert_equal(mt.status.last_rssi, -95, "the first arrival's signal");

	zassert_equal(meshtastic_attachment_count(), 3U, "both heads admitted");
	zassert_true(meshtastic_attachment_get(2U, &a));
	zassert_equal(a.node, HEAD2_NODE);
	zassert_equal(a.rx_frames, 1U, "the duplicate still counts as heard by head 2");
}

/* T3: the legacy bearer inject is untouched -- a BLE-peer frame is delivered
 * but stays non-RF: no signal, no last_rx_from. */
ZTEST(attachment, test_legacy_ble_peer_inject_stays_non_rf)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1003U, "ble", wire, &len);
	zassert_ok(meshtastic_radio_rx_inject(wire, (uint16_t)len, MESHTASTIC_BEARER_BLE_PEER));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_equal(mt.status.last_rssi, 0, "no RF signal from a wired-style bearer");
	zassert_equal(mt.status.last_rx_from, 0U);
	assert_not_relayed(FAR_NODE_ID, 0x1003U);
}

/* T4: a head's frame is never flood-relayed onto OUR radio (it is on the head's
 * preset; relays go back out the way they came in, which P3 provides). */
ZTEST(attachment, test_head_frame_is_not_relayed_on_the_local_radio)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1004U, "relay me?", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "ingest");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	assert_not_relayed(FAR_NODE_ID, 0x1004U);
}

/* T6: our own transmission heard back through a head. With OUR relay byte it
 * is our own voice (nobody rebroadcast it): counted as self_heard, never an
 * implicit ACK, never delivered. With a neighbour's relay byte it is a real
 * echo and takes the normal path (not counted as self_heard). */
ZTEST(attachment, test_own_voice_through_a_head_is_not_an_echo)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

	build_frame(TEST_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1006U, "my voice", wire, &len);
	h->relay_node = (uint8_t)(TEST_NODE_ID & 0xFFU);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 10, wire, len), "ingest");
	k_msleep(200);
	zassert_equal(mt.status.self_heard, 1U, "our own voice, counted");
	zassert_equal(rx.count, 0U, "never delivered to ourselves");

	build_frame(TEST_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1007U, "echoed", wire, &len);
	h->relay_node = 0x77U; /* a neighbour rebroadcast it */
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 10, wire, len), "ingest");
	k_msleep(200);
	zassert_equal(mt.status.self_heard, 1U, "a real echo is not self_heard");
	zassert_equal(rx.count, 0U, "an echo is never delivered either");
}

/* T7: STATUS fills the table; garbage is counted and refused; a control aimed
 * at a head (SET_PRESET) arriving at the brain is refused. */
ZTEST(attachment, test_status_and_bad_envelopes)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	struct meshtastic_attachment_info a;
	const struct meshtastic_attachment_status st = {
		.preset = (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
		.flags = MESHTASTIC_ATTACHMENT_ST_TX_ENABLED | MESHTASTIC_ATTACHMENT_ST_IS_HEAD |
			 MESHTASTIC_ATTACHMENT_ST_HAS_POS,
		.hwid = HEAD1_NODE,
		.brain = TEST_NODE_ID,
		.rx_frames = 42U,
		.uptime_s = 7U,
		.lat = 335282000,
		.lon = -821297000,
		.alt = 123,
	};
	int len;

	len = meshtastic_attachment_encode_status(&st, env, sizeof(env));
	zassert_equal(len, (int)MESHTASTIC_ATTACHMENT_STATUS_POS_LEN);
	zassert_ok(meshtastic_attachment_ingest(HEAD1_NODE, env, (size_t)len), "status ingest");
	zassert_true(meshtastic_attachment_get(meshtastic_attachment_id_for_node(HEAD1_NODE), &a));
	zassert_true(a.have_status);
	zassert_equal(a.status.hwid, HEAD1_NODE);
	zassert_equal(a.status.rx_frames, 42U);
	zassert_equal(a.status.alt, 123);
	zassert_equal(a.preset, (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
		      "STATUS updates the head's preset");

	env[0] = 0x7EU; /* not a type */
	zassert_equal(meshtastic_attachment_ingest(HEAD1_NODE, env, 5U), -EBADMSG);
	len = meshtastic_attachment_encode_set_preset(3U, env, sizeof(env));
	zassert_equal(meshtastic_attachment_ingest(HEAD1_NODE, env, (size_t)len), -EBADMSG,
		      "a control aimed at a head is refused by a brain");
	zassert_true(meshtastic_attachment_get(meshtastic_attachment_id_for_node(HEAD1_NODE), &a));
	zassert_equal(a.rejected, 2U);
}

/* T8: set_preset goes out through the send seam as a SET_PRESET envelope to the
 * head's link identity; the local radio and unknown ids are refused. */
ZTEST(attachment, test_set_preset_reaches_the_head)
{
	struct meshtastic_attachment_msg msg;
	uint8_t id;

	zassert_equal(meshtastic_attachment_set_preset(0U, 1U), -EINVAL);
	zassert_equal(meshtastic_attachment_set_preset(CONFIG_MESHTASTIC_ATTACHMENT_MAX, 1U),
		      -ENOENT, "a slot nobody occupies");
	id = meshtastic_attachment_id_for_node(HEAD1_NODE);
	zassert_true(id != 0U, "head 1 is known from the earlier cases");
	zassert_ok(meshtastic_attachment_set_preset(id, 2U));
	zassert_equal(sent.count, 1U);
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_SET_PRESET);
	zassert_equal(msg.u.preset, 2U);

	zassert_ok(meshtastic_attachment_forget(id));
	zassert_equal(meshtastic_attachment_id_for_node(HEAD1_NODE), 0U);
}
