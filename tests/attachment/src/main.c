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
#include "meshtastic_neighborinfo.h"
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include "meshtastic_packet.h"
#include "meshtastic_contention.h"
#include "meshtastic_preset.h"
#include "meshtastic_reliable.h"
#include "meshtastic_sched.h"
#include "meshtastic_outbound.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define FAR_NODE_ID  0x0D0D0D0DU
#define HEAD1_NODE   0x00E10001U /* a head's link identity (its hw node number) */
#define HEAD2_NODE   0x00E10002U
#define PRESET_ST meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO
#define PRESET_MF meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST

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

/* The test bearer (ATTACHMENT-SCOPE §4): what the brain sends is captured, and
 * what the bearer says about a link -- up, trusted -- is a knob per case. */
#define SENT_RING 6U
static struct {
	uint32_t node;
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	size_t len;
	uint32_t count;
	/* The last SENT_RING envelopes too, newest last: a test that expects two
	 * replies (an ACK and a NodeInfo request) must see both, not the last. */
	struct {
		uint32_t node;
		uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
		size_t len;
	} ring[SENT_RING];
	uint32_t ring_head;
} sent;

static uint32_t sent_ring_count(void)
{
	return MIN(sent.ring_head, SENT_RING);
}

/* The i-th newest envelope (0 = newest), decoded. */
static bool sent_ring_get(uint32_t i, struct meshtastic_attachment_msg *msg)
{
	uint32_t slot;

	if (i >= sent_ring_count()) {
		return false;
	}
	slot = (sent.ring_head - 1U - i) % SENT_RING;
	return meshtastic_attachment_decode(sent.ring[slot].env, sent.ring[slot].len, msg) == 0;
}

static enum meshtastic_attach_auth test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;

static int test_bearer_send(uint32_t node, const uint8_t *env, size_t len)
{
	if (len > sizeof(sent.env)) {
		return -EMSGSIZE;
	}
	sent.node = node;
	memcpy(sent.env, env, len);
	sent.ring[sent.ring_head % SENT_RING].node = node;
	memcpy(sent.ring[sent.ring_head % SENT_RING].env, env, len);
	sent.ring[sent.ring_head % SENT_RING].len = len;
	sent.ring_head++;
	sent.len = len;
	sent.count++;
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
static int head_hears_at(uint32_t head, uint8_t preset, int16_t rssi, int8_t snr,
			 const uint8_t *wire, uint32_t wire_len, uint32_t rx_ms);

static int head_hears(uint32_t head, uint8_t preset, int16_t rssi, int8_t snr,
		      const uint8_t *wire, uint32_t wire_len)
{
	return head_hears_at(head, preset, rssi, snr, wire, wire_len, (uint32_t)k_uptime_get());
}

static int head_hears_at(uint32_t head, uint8_t preset, int16_t rssi, int8_t snr,
			 const uint8_t *wire, uint32_t wire_len, uint32_t rx_ms)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_rx_frame m = {
		.preset = preset,
		.rssi = rssi,
		.snr = snr,
		.rx_ms = rx_ms,
		.wire = wire,
		.wire_len = (uint16_t)wire_len,
	};
	int len = meshtastic_attachment_encode_rx_frame(&m, env, sizeof(env));

	zassert_true(len > 0, "encode rx_frame (%d)", len);
	return meshtastic_attach_bearer_rx(&test_bearer, head, env, (size_t)len);
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

/* Did a frame from (src, id) leave on THIS board's radio within @p ms? Every
 * "not relayed" assertion in this suite has a "relayed" twin (review F6): the
 * suite runs with rebroadcast ALL, so a LoRa frame IS relayed, and a head's
 * frame is the one that must not be. */
static bool relayed_within(uint32_t src, uint32_t id, int ms)
{
	struct lora_sim_frame f;
	int64_t end = k_uptime_get() + ms;

	while (k_uptime_get() < end) {
		if (lora_sim_take_tx(lora_dev, &f, K_MSEC(100)) == 0) {
			const struct meshtastic_wire_header *h =
				(const struct meshtastic_wire_header *)f.data;

			if (sys_le32_to_cpu(h->src) == src && sys_le32_to_cpu(h->id) == id) {
				return true;
			}
		}
	}
	return false;
}

static void set_hop_limit(uint8_t *wire, uint8_t hops)
{
	struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

	h->flags = (uint8_t)((h->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | hops);
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
	zassert_ok(meshtastic_attach_bearer_register(&test_bearer), "test bearer");
	return NULL;
}

static void attachment_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* The brain on the bench: ShortTurbo, on that preset's default channel,
	 * relaying like a stock node -- without ALL nothing in this suite would
	 * ever relay, and "not relayed" could not fail (review F6). */
	set_default_primary();
	/* A head's preset gets a channel of its own (auto-created); none of them
	 * may leak into the next test. */
	for (uint8_t i = 1U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		meshtastic_Channel off = meshtastic_Channel_init_zero;

		off.index = (int8_t)i;
		off.role = meshtastic_Channel_Role_DISABLED;
		off.has_settings = true;
		zassert_ok(meshtastic_channels_set_slot(i, &off), "");
	}
	mt.use_preset = true;
	zassert_ok(meshtastic_preset_switch(PRESET_ST, NULL), "preset");
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
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
	/* A fresh table per case: every head is admitted by the case that uses it,
	 * over a trusted link unless the case says otherwise. */
	for (uint8_t id = 1U; id <= CONFIG_MESHTASTIC_ATTACHMENT_MAX; id++) {
		(void)meshtastic_attachment_forget(id);
	}
	test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;
	meshtastic_attachment_allow_clear();
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

/* T1 (P1): a frame heard by a head on ANOTHER preset carries that preset's
 * channel hash -- an unnamed slot is named after the preset it is used on. The
 * brain, on ShortTurbo, cannot match it against its cached hashes; it matches
 * the slot hashed under the head's preset instead. Same key, same nonce
 * (id, from): the channel byte is header-only, so the ciphertext is the one a
 * MediumFast node built. Decoded as the brain's own primary-channel content --
 * same identity, no re-origination, the point of the exercise. */
ZTEST(attachment, test_frame_from_another_preset_decodes_on_the_brain)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_info a;
	const uint8_t here = meshtastic_channels_get_hash(0U);
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);

	zassert_not_equal(here, there, "an unnamed slot hashes per preset (0x%02x)", here);
	zassert_equal(meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_ST), here,
		      "the active preset's hash is the cached one");
	zassert_equal(meshtastic_channels_hash_for_preset(0U, MESHTASTIC_PRESET_UNKNOWN), here,
		      "an unknown preset means the active one");

	/* Tagged with the wrong preset by a head: nothing in the table matches. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1101U, "from MediumFast", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -95, 3, wire, len), "ingest");
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "not decodable as ShortTurbo");

	/* Untagged (the legacy bearer inject): the active preset, so the same. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1102U, "from MediumFast", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(meshtastic_radio_rx_inject(wire, (uint16_t)len, MESHTASTIC_BEARER_BLE_PEER));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "not decodable untagged");

	/* Tagged MediumFast by the head that heard it there: decoded. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1103U, "from MediumFast", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -95, 3, wire, len), "ingest");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_equal(rx.from, FAR_NODE_ID, "the origin's identity, not a relay's");
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE);
	zassert_equal(rx.rssi, -95);
	zassert_equal(rx.count, 1U);

	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.preset, (uint8_t)PRESET_MF, "the head now reports MediumFast");
	zassert_equal(a.rx_frames, 2U, "both envelopes counted, one decoded");
}

/* agents-pcs2.11: a slot named after a head's preset is that preset's channel.
 * A brain covering MediumFast through a head shows ONE public chat by default:
 * its primary, re-hashed for MediumFast, matches first. With a slot named
 * "MediumFast" (default key) the same frame is filed under THAT slot, so the
 * phone app shows the preset as its own chat. Opt-in: without the slot, and on
 * our own radio, nothing changes. */
ZTEST(attachment, test_a_slot_named_after_the_heard_preset_gets_that_presets_traffic)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t payload[64];
	uint32_t len;
	struct meshtastic_packet pkt;
	bool decoded;
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1F01U, "mf", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;

	/* Without the slot: the primary, as before. */
	zassert_false(meshtastic_channels_named_for_preset(0U, (uint8_t)PRESET_MF));
	decoded = false;
	zassert_ok(meshtastic_try_decode_wire_packet_on(wire, (int)len, -90, 5, (uint8_t)PRESET_MF,
							&pkt, payload, sizeof(payload), &decoded,
							NULL, NULL));
	zassert_true(decoded);
	zassert_equal(pkt.channel_index, 0U, "no preset slot: filed under the primary");

	/* The operator adds a channel named after the head's preset, default key. */
	ch.index = 1;
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 1U;
	strcpy(ch.settings.name, "MediumFast");
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "");
	zassert_equal(meshtastic_channels_get_hash(1U), there,
		      "a slot named MediumFast with the default key IS MediumFast's public hash");
	zassert_true(meshtastic_channels_named_for_preset(1U, (uint8_t)PRESET_MF));
	zassert_false(meshtastic_channels_named_for_preset(1U, (uint8_t)PRESET_ST),
		      "not the channel for our own preset");

	decoded = false;
	zassert_ok(meshtastic_try_decode_wire_packet_on(wire, (int)len, -90, 5, (uint8_t)PRESET_MF,
							&pkt, payload, sizeof(payload), &decoded,
							NULL, NULL));
	zassert_true(decoded);
	zassert_equal(pkt.channel_index, 1U, "heard on MediumFast: filed under the MediumFast slot");

	/* Our own radio's default-channel traffic still belongs to the primary. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1F02U, "st", wire, &len);
	decoded = false;
	zassert_ok(meshtastic_try_decode_wire_packet_on(wire, (int)len, -90, 5, (uint8_t)PRESET_ST,
							&pkt, payload, sizeof(payload), &decoded,
							NULL, NULL));
	zassert_true(decoded);
	zassert_equal(pkt.channel_index, 0U, "our own preset's traffic stays on the primary");

	ch.role = meshtastic_Channel_Role_DISABLED;
	ch.settings.name[0] = '\0';
	ch.settings.psk.size = 0U;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "restore");
}

/* agents-pcs2.11, the send half: what is ORIGINATED on a slot named after a
 * head's preset leaves through a head on that preset, stamped with that
 * preset's hash -- typing in the "MediumFast" chat speaks on MediumFast. With no
 * ready head on that preset the send is refused, never keyed up on our own
 * radio under another preset's hash. An ordinary channel is our own radio. */
ZTEST(attachment, test_a_send_on_a_preset_channel_leaves_through_that_presets_head)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);
	struct meshtastic_packet text = {
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"on mf",
		.payload_len = 5U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = 1U,
	};
	uint32_t before;

	ch.index = 1;
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 1U;
	strcpy(ch.settings.name, "MediumFast");
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "");

	/* No head at all: refused, nothing leaves anywhere. */
	zassert_equal(meshtastic_attachment_for_channel(1U), -ENETUNREACH);
	zassert_equal(meshtastic_attachment_for_channel(0U), 0, "an ordinary channel: our radio");
	before = sent.count;
	zassert_equal(meshtastic_send_packet(&text, K_NO_WAIT), -ENETUNREACH,
		      "no head on MediumFast: refused");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "not on our radio");
	zassert_equal(sent.count, before, "and no envelope");

	/* A head on ShortTurbo does not count: wrong preset. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2A00U, "admit st", wire, &len);
	zassert_ok(head_hears(HEAD2_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300);
	drain_radio();
	zassert_equal(meshtastic_attachment_for_channel(1U), -ENETUNREACH,
		      "a head on another preset is not MediumFast's");

	/* A head on MediumFast: the send goes through it. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2A01U, "admit mf", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300); /* let the relay of the admit frame be handed over first */
	drain_radio();
	zassert_true(meshtastic_attachment_for_channel(1U) > 0, "MediumFast has a head now");

	before = sent.count;
	zassert_ok(meshtastic_send_packet(&text, K_NO_WAIT), "sent on the MediumFast channel");
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "one TX_FRAME");
	zassert_equal(sent.node, HEAD1_NODE, "to the MediumFast head, not the ShortTurbo one");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_FRAME);
	zassert_equal(msg.u.tx.preset, (uint8_t)PRESET_MF);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->channel, there,
		      "stamped with MediumFast's public hash");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing on our radio");

	/* The primary is still our own radio. */
	text.channel_index = 0U;
	zassert_ok(meshtastic_send_packet(&text, K_NO_WAIT), "");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "the primary leaves on our radio");

	ch.role = meshtastic_Channel_Role_DISABLED;
	ch.settings.name[0] = '\0';
	ch.settings.psk.size = 0U;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "restore");
}

/* Wait for the next TX_FRAME envelope after @p before, and decode it. */
static void take_tx_frame(uint32_t before, struct meshtastic_attachment_msg *msg)
{
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "one TX_FRAME (%u)", sent.count - before);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, msg));
	zassert_equal(msg->type, MESHTASTIC_ATTACHMENT_TX_FRAME);
}

/* What the node says to everyone goes out on every radio it has: its own, and
 * ONE head per other preset. Its NodeInfo beacon, so that it is a member of
 * each preset's mesh and not only a listener on it -- re-stamped with the
 * default channel's hash under that preset. What the user says on a channel
 * with a name of its own, which is one chat across presets (its hash does not
 * change). A direct message, since the peer may be on any preset. But a
 * broadcast on the primary stays on our own radio: that is OUR preset's chat,
 * and the other presets have their own. */
ZTEST(attachment, test_beacons_and_the_users_own_channels_go_out_on_every_radio)
{
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);
	const uint8_t here = meshtastic_channels_get_hash(0U);
	meshtastic_Channel fam = meshtastic_Channel_init_zero;
	meshtastic_MeshPacket text = meshtastic_MeshPacket_init_zero;
	struct meshtastic_attachment_msg msg;
	struct meshtastic_nodedb_node peer;
	struct lora_sim_frame f;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	uint32_t before;

	zassert_not_equal(there, here, "the two presets' default channels differ");

	fam.index = 3;
	fam.role = meshtastic_Channel_Role_SECONDARY;
	fam.has_settings = true;
	fam.settings.psk.size = 16U;
	memset(fam.settings.psk.bytes, 0x33, 16U);
	strcpy(fam.settings.name, "family");
	zassert_ok(meshtastic_channels_set_slot(3U, &fam), "");
	zassert_true(meshtastic_channels_custom_named(3U));
	zassert_false(meshtastic_channels_custom_named(0U), "the default channel is not");

	/* Two heads on MediumFast, one on our own preset. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2D00U, "a", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -90, 5, wire, len));
	k_msleep(50); /* so that "most recently heard" is not a tie */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2D01U, "b", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there; /* decodable there */
	zassert_ok(head_hears(HEAD2_NODE, PRESET_MF, -90, 5, wire, len));
	k_msleep(400);
	drain_radio();
	zassert_false(meshtastic_channels_custom_named(1U), "the auto-created MediumFast is not");

	/* The NodeInfo beacon: our radio, and ONE MediumFast head, re-stamped. */
	before = sent.count;
	zassert_ok(meshtastic_send_node_info(MESHTASTIC_NODE_BROADCAST));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "on our own radio");
	zassert_equal(((const struct meshtastic_wire_header *)f.data)->channel, here);
	take_tx_frame(before, &msg);
	zassert_equal(sent.node, HEAD2_NODE, "the most recently heard MediumFast head");
	zassert_equal(msg.u.tx.preset, (uint8_t)PRESET_MF);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->channel, there,
		      "under MediumFast's default-channel hash");
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->id,
		      ((const struct meshtastic_wire_header *)f.data)->id, "the same packet");
	k_msleep(200);
	zassert_equal(sent.count, before + 1U, "one head per preset, not both");

	/* The user's text on "family": both radios, the hash unchanged. */
	text.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	text.to = MESHTASTIC_NODE_BROADCAST;
	text.id = 0x2D10U;
	text.channel = 3U;
	text.hop_limit = 3U;
	text.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
	text.decoded.payload.size = 3U;
	memcpy(text.decoded.payload.bytes, "fam", 3U);
	before = sent.count;
	zassert_ok(meshtastic_send_mesh_pb(&text));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "on our own radio");
	take_tx_frame(before, &msg);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->channel,
		      meshtastic_channels_get_hash(3U), "a named channel hashes the same everywhere");

	/* The user's text on the primary: our own preset's chat, our radio only. */
	text.id = 0x2D11U;
	text.channel = 0U;
	before = sent.count;
	zassert_ok(meshtastic_send_mesh_pb(&text));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "on our own radio");
	k_msleep(300);
	zassert_equal(sent.count, before, "not through a head");

	/* A direct message to a peer not heard since boot: every radio, it may
	 * be on any preset. */
	text.id = 0x2D12U;
	text.to = 0x0D0D0D01U;
	before = sent.count;
	zassert_ok(meshtastic_send_mesh_pb(&text));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "on our own radio");
	take_tx_frame(before, &msg);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->id, 0x2D12U);

	/* To a peer last heard through the MediumFast head: that head, and not
	 * our own radio (agents-pcs2.13). */
	zassert_ok(meshtastic_nodedb_get(FAR_NODE_ID, &peer));
	zassert_equal(peer.heard_preset, (uint8_t)PRESET_MF, "heard on MediumFast");
	text.id = 0x2D13U;
	text.to = FAR_NODE_ID;
	before = sent.count;
	zassert_ok(meshtastic_send_mesh_pb(&text));
	take_tx_frame(before, &msg);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->id, 0x2D13U);
	zassert_equal(msg.u.tx.preset, (uint8_t)PRESET_MF);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "not on our radio");

	/* The same peer is then heard on our own radio: our radio, no head. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2D02U, "c", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -70, 8));
	k_msleep(400);
	drain_radio();
	zassert_ok(meshtastic_nodedb_get(FAR_NODE_ID, &peer));
	zassert_equal(peer.heard_preset, (uint8_t)PRESET_ST, "now heard on our own preset");
	text.id = 0x2D14U;
	before = sent.count;
	zassert_ok(meshtastic_send_mesh_pb(&text));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "on our own radio");
	zassert_equal(((const struct meshtastic_wire_header *)f.data)->id, 0x2D14U);
	k_msleep(300);
	zassert_equal(sent.count, before, "and through no head");
	meshtastic_reliable_reset();
}

/* How many enabled slots are named after @p preset, and the first one. */
static unsigned int preset_slots(uint8_t preset, int *first)
{
	unsigned int n = 0U;

	*first = -1;
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		if (meshtastic_channels_named_for_preset(i, preset)) {
			if (n++ == 0U) {
				*first = (int)i;
			}
		}
	}
	return n;
}

/* A head brings its preset's default channel with it: the first time a head on
 * another preset speaks, the brain creates that preset's channel -- named after
 * the preset, the well-known key in its one-byte form -- in the first free
 * slot. A head on OUR preset creates nothing (the primary is that channel), a
 * slot that already has the name is left exactly as it is, and a second head
 * on the same preset adds nothing. */
ZTEST(attachment, test_a_heads_preset_gets_its_default_channel)
{
	const uint8_t sf = (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST;
	meshtastic_Channel mine = meshtastic_Channel_init_zero;
	const meshtastic_Channel *ch;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	int slot;

	zassert_equal(preset_slots((uint8_t)PRESET_MF, &slot), 0U, "starts without one");

	/* The operator's own "ShortFast" slot, with a private key, in slot 2. */
	mine.index = 2;
	mine.role = meshtastic_Channel_Role_SECONDARY;
	mine.has_settings = true;
	mine.settings.psk.size = 16U;
	memset(mine.settings.psk.bytes, 0x5A, 16U);
	strcpy(mine.settings.name, "ShortFast");
	zassert_ok(meshtastic_channels_set_slot(2U, &mine), "");

	/* A head on our own preset: nothing is created. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2C00U, "st", wire, &len);
	zassert_ok(head_hears(HEAD2_NODE, PRESET_ST, -90, 5, wire, len));
	k_msleep(200);
	for (uint8_t i = 1U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		if (i != 2U) {
			zassert_equal(meshtastic_channels_get(i)->role,
				      meshtastic_Channel_Role_DISABLED, "slot %u untouched", i);
		}
	}

	/* A head on MediumFast: its channel appears in the first free slot. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2C01U, "mf", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -90, 5, wire, len));
	k_msleep(200);
	zassert_equal(preset_slots((uint8_t)PRESET_MF, &slot), 1U, "one MediumFast channel");
	zassert_equal(slot, 1, "in the first free slot");
	ch = meshtastic_channels_get(1U);
	zassert_equal(ch->role, meshtastic_Channel_Role_SECONDARY);
	zassert_equal(ch->settings.psk.size, 1U, "the one-byte default key");
	zassert_equal(ch->settings.psk.bytes[0], 1U);
	zassert_true(meshtastic_attachment_for_channel(1U) > 0, "and a send on it has a head");

	/* That head moves to ShortFast: the operator's slot is the ShortFast
	 * channel already, key and all. */
	zassert_ok(head_hears(HEAD1_NODE, sf, -90, 5, wire, len));
	k_msleep(200);
	zassert_equal(preset_slots(sf, &slot), 1U, "no second ShortFast channel");
	zassert_equal(slot, 2);
	zassert_equal(meshtastic_channels_get(2U)->settings.psk.size, 16U, "left as it was");

	/* A second head on MediumFast adds nothing. */
	zassert_ok(head_hears(HEAD2_NODE, PRESET_MF, -90, 5, wire, len));
	k_msleep(200);
	zassert_equal(preset_slots((uint8_t)PRESET_MF, &slot), 1U, "still one");
}

/* A want_ack send that left through a head is retransmitted through THAT head,
 * never on our own radio (where it would carry another preset's hash), and its
 * implicit ACK is the relay that head hears. This is what a phone's text on a
 * preset channel is: a want_ack broadcast. */
ZTEST(attachment, test_a_reliable_send_through_a_head_is_retransmitted_through_it)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);
	struct meshtastic_packet text = {
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = 0x2B10U,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"ack mf",
		.payload_len = 6U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_ack = true,
		.channel_index = 1U,
	};
	uint32_t before;

	meshtastic_reliable_reset();
	zassert_ok(meshtastic_sched_set("reliable.timeout", "400"));

	ch.index = 1;
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 1U;
	strcpy(ch.settings.name, "MediumFast");
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "");

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2B00U, "admit mf", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300);
	drain_radio();

	before = sent.count;
	zassert_ok(meshtastic_send_packet(&text, K_NO_WAIT), "sent on the MediumFast channel");
	for (int i = 0; i < 30 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "the original, one TX_FRAME");
	zassert_equal(meshtastic_reliable_pending(), 1U, "tracked, though it left by a head");

	/* No relay heard: the retransmit is a second TX_FRAME to the same head. */
	for (int i = 0; i < 80 && sent.count == before + 1U; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 2U, "the retransmit, through the head");
	zassert_equal(sent.node, HEAD1_NODE, "the same head");
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_FRAME);
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->id, 0x2B10U,
		      "the same packet");
	zassert_equal(((const struct meshtastic_wire_header *)msg.u.tx.wire)->channel, there,
		      "still under MediumFast's hash");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(100)), 0, "never on our radio");

	/* The head hears a neighbour relay it: resolved, no further retransmit. */
	memcpy(wire, msg.u.tx.wire, msg.u.tx.wire_len);
	((struct meshtastic_wire_header *)wire)->relay_node = 0x77U;
	set_hop_limit(wire, 2U);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -60, 10, wire, msg.u.tx.wire_len), "via head");
	k_msleep(200);
	zassert_equal(meshtastic_reliable_pending(), 0U, "the relay the head heard is the ACK");
	before = sent.count;
	k_msleep(600);
	zassert_equal(sent.count, before, "and nothing more is sent");

	meshtastic_reliable_reset(); /* the before hook restores the timeout */
	ch.role = meshtastic_Channel_Role_DISABLED;
	ch.settings.name[0] = '\0';
	ch.settings.psk.size = 0U;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "restore");
}

/* B15 (review F3, SCOPE C1): a head is admitted only over a link the bearer
 * vouches for, or from the operator's allow-list. Anything else gets nothing --
 * not a slot, not a delivery, not an RF frame. */
ZTEST(attachment, test_admission_needs_a_trusted_link_or_the_allow_list)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_stats st0, st;

	meshtastic_attachment_stats_get(&st0);
	test_auth = MESHTASTIC_ATTACH_AUTH_NONE;
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2001U, "stranger", wire, &len);
	zassert_equal(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len), -EACCES,
		      "an untrusted, unlisted link is refused");
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "nothing delivered");
	zassert_equal(meshtastic_attachment_count(), 1U, "no slot taken");
	meshtastic_attachment_stats_get(&st);
	zassert_equal(st.admission_refused, st0.admission_refused + 1U);

	/* The operator vouches for it: admitted over the same untrusted link. */
	zassert_ok(meshtastic_attachment_allow_add(HEAD1_NODE));
	zassert_true(meshtastic_attachment_is_allowed(HEAD1_NODE));
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2002U, "allowed", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len), "allow-listed");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_equal(meshtastic_attachment_count(), 2U);

	/* A second stranger stays out; a trusted link admits without the list. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2003U, "stranger2", wire, &len);
	zassert_equal(head_hears(HEAD2_NODE, PRESET_ST, -50, 9, wire, len), -EACCES);
	test_auth = MESHTASTIC_ATTACH_AUTH_PHYSICAL;
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2004U, "wired", wire, &len);
	zassert_ok(head_hears(HEAD2_NODE, PRESET_ST, -50, 9, wire, len), "a trusted wire admits");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	zassert_equal(meshtastic_attachment_count(), 3U);
}

/* B14: the table -- a malformed first contact takes no slot, a full table
 * refuses, a head whose link went down keeps its slot through the grace and
 * loses it after (EVICT_SEC=2 in this build), and speaks itself up again. */
ZTEST(attachment, test_table_admission_capacity_and_eviction)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t junk[4] = { MESHTASTIC_ATTACHMENT_STATUS, 1, 2, 3 };
	uint32_t len;
	struct meshtastic_attachment_info a;
	struct meshtastic_attachment_stats st0, st;

	meshtastic_attachment_stats_get(&st0);
	/* malformed first contact: no slot */
	zassert_equal(meshtastic_attach_bearer_rx(&test_bearer, HEAD1_NODE, junk, sizeof(junk)),
		      -EBADMSG);
	zassert_equal(meshtastic_attachment_count(), 1U, "malformed does not admit");
	meshtastic_attachment_stats_get(&st);
	zassert_equal(st.not_admitted_malformed, st0.not_admitted_malformed + 1U);

	/* fill the table (MAX=2), a third is refused */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2101U, "a", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2102U, "b", wire, &len);
	zassert_ok(head_hears(HEAD2_NODE, PRESET_ST, -50, 9, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2103U, "c", wire, &len);
	zassert_equal(head_hears(0x00E10003U, PRESET_ST, -50, 9, wire, len), -ENOSPC, "table full");

	/* link down: still there, marked; up again on its next envelope */
	meshtastic_attach_bearer_link_down(&test_bearer, HEAD1_NODE);
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_false(a.link_up, "marked down");
	zassert_equal(a.node, HEAD1_NODE, "slot kept through the grace");
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2104U, "back", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_true(a.link_up, "up again");
	zassert_equal(a.rx_frames, 2U, "counters kept");

	/* down for longer than the grace: evicted, the slot is free again */
	meshtastic_attach_bearer_link_down(&test_bearer, HEAD1_NODE);
	k_sleep(K_MSEC(3500));
	zassert_false(meshtastic_attachment_get(1U, &a), "evicted");
	zassert_equal(meshtastic_attachment_id_for_node(HEAD1_NODE), 0U);
	meshtastic_attachment_stats_get(&st);
	zassert_equal(st.evicted, st0.evicted + 1U);
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2105U, "c again", wire, &len);
	zassert_ok(head_hears(0x00E10003U, PRESET_ST, -50, 9, wire, len), "the slot is free");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
}

/* B16: hostile envelopes from an ADMITTED head -- what a compromised head
 * could say. None of it may leave on our radio or be delivered as ours. */
ZTEST(attachment, test_hostile_envelopes_from_an_admitted_head)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

	/* src = broadcast address: dropped before anything */
	build_frame(MESHTASTIC_NODE_BROADCAST, MESHTASTIC_NODE_BROADCAST, 0x2201U, "x", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "src=broadcast not delivered");
	zassert_false(relayed_within(MESHTASTIC_NODE_BROADCAST, 0x2201U, 500));
	/* src = 0: whatever the router makes of it (today it delivers, as it does
	 * from the local radio -- a pre-existing question, agents-pcs2), it must
	 * not leave on our radio */
	build_frame(0U, MESHTASTIC_NODE_BROADCAST, 0x2202U, "x", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -50, 9, wire, len));
	(void)k_sem_take(&rx.sem, K_MSEC(300));
	zassert_false(relayed_within(0U, 0x2202U, 500), "src=0 never relayed on our radio");
	/* hop limit 7, an unknown preset tag, absurd signal */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2203U, "x", wire, &len);
	set_hop_limit(wire, 7U);
	zassert_ok(head_hears(HEAD1_NODE, 0xFEU, 127, 127, wire, len));
	(void)k_sem_take(&rx.sem, K_MSEC(300));
	zassert_false(relayed_within(FAR_NODE_ID, 0x2203U, 500), "never relayed on our radio");
	ARG_UNUSED(h);
}

/* The choreography upstream's NextHopRouter demands before a route is learned
 * (meshtastic_routing_learn_next_hop): we send a unicast; we overhear a
 * neighbour relay it (our own frame back, with its relay byte); a reply to
 * that packet (request_id = our id) arrives via the same relayer, who resolves
 * to one known node. Then next_hop(from) = that relayer. @p via_head delivers
 * the reply through a head instead of our radio. */
static void learn_choreography(uint32_t far, bool via_head)
{
	struct lora_sim_frame tx;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	uint32_t our_id;
	const uint8_t relayer = (uint8_t)(far & 0xFFU);
	struct meshtastic_packet reply = {
		.from = far,
		.to = TEST_NODE_ID,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"re",
		.payload_len = 2U,
		.hop_limit = 2U,
		.hop_start = 3U,
		.channel_index = 0U,
	};

	/* 1. our unicast leaves on the air */
	zassert_ok(meshtastic_send_text(far, "q"), "send");
	zassert_ok(lora_sim_take_tx(lora_dev, &tx, K_SECONDS(3)), "our TX");
	our_id = sys_le32_to_cpu(((const struct meshtastic_wire_header *)tx.data)->id);
	/* 2. a neighbour relays it: we overhear our own frame with its relay byte */
	memcpy(wire, tx.data, tx.len);
	((struct meshtastic_wire_header *)wire)->relay_node = relayer;
	set_hop_limit(wire, 2U);
	zassert_ok(lora_sim_inject(lora_dev, wire, tx.len, -60, 8), "echo");
	k_sleep(K_MSEC(200));
	/* 3. the reply, correlated to our packet, via the same relayer */
	reply.id = our_id ^ 0x5A5A0000U;
	reply.request_id = our_id;
	zassert_ok(meshtastic_build_wire_packet(&reply, wire, &len), "build reply");
	((struct meshtastic_wire_header *)wire)->relay_node = relayer;
	if (via_head) {
		zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 8, wire, len), "via head");
	} else {
		zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -60, 8), "via our radio");
	}
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "reply delivered");
	k_sleep(K_MSEC(100));
}

/* B10 (review F5, SCOPE E1): a next hop is learned only from our own radio.
 * The relay byte on a frame a head heard names a neighbour on the head's
 * preset; a DM we later sent our radio with that next-hop byte would be
 * ignored by everyone on ours. Same choreography twice: the reply through a
 * head teaches nothing, through our radio it teaches the route. */
ZTEST(attachment, test_next_hop_is_learned_from_the_local_radio_only)
{
	const uint32_t far_a = 0x0D0D0D21U; /* low bytes distinct: unique relayers */
	const uint32_t far_b = 0x0D0D0D22U;

	learn_choreography(far_a, true);
	zassert_equal(meshtastic_nodedb_get_next_hop(far_a), 0U, "nothing learned from a head");

	learn_choreography(far_b, false);
	zassert_equal(meshtastic_nodedb_get_next_hop(far_b), (uint8_t)(far_b & 0xFFU),
		      "learned from our own radio");
}

/* B11 (SCOPE E2): NeighborInfo's table is per local radio. A head's frame
 * with hop_start == hop_limit is the HEAD's neighbour, not ours. */
ZTEST(attachment, test_neighbors_are_the_local_radios_only)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	meshtastic_neighborinfo_reset();
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2401U, "hi", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 8, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_sleep(K_MSEC(100)); /* module dispatch runs after the delivery callback */
	zassert_equal(meshtastic_neighborinfo_count(), 0U, "a head's neighbour is not ours");

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2402U, "hi", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -60, 8));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_sleep(K_MSEC(100));
	zassert_equal(meshtastic_neighborinfo_count(), 1U, "heard on our radio: a neighbour");
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
/* B1, the control: a frame on OUR radio is relayed in this suite. Without this
 * case every "not relayed" below is vacuous. */
ZTEST(attachment, test_control_local_frame_is_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1E00U, "local", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_true(relayed_within(FAR_NODE_ID, 0x1E00U, 4000), "control: a LoRa frame relays");
}

/* B3/B4 (review F1): a hop-upgraded duplicate through a head takes the second
 * relay path in the router; it must obey the same rule as the first copy --
 * in both arrival orders. */
ZTEST(attachment, test_hop_upgrade_through_a_head_is_not_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	/* head first (1 hop), head again (3 hops) */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1F00U, "upgrade me", wire, &len);
	set_hop_limit(wire, 1U);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "first copy");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_false(relayed_within(FAR_NODE_ID, 0x1F00U, 2000), "first copy not relayed");
	set_hop_limit(wire, 3U);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "upgraded copy");
	zassert_false(relayed_within(FAR_NODE_ID, 0x1F00U, 3000),
		      "a hop-upgraded head copy leaked onto the local radio");
	zassert_equal(rx.count, 1U, "never re-delivered");

	/* local first (1 hop, relayed), head upgrade (3 hops): the upgrade is the
	 * head's, so it stays off our air even though the first copy was ours */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1F01U, "upgrade me", wire, &len);
	set_hop_limit(wire, 1U);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_true(relayed_within(FAR_NODE_ID, 0x1F01U, 4000), "our copy relays");
	set_hop_limit(wire, 3U);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "head upgrade");
	zassert_false(relayed_within(FAR_NODE_ID, 0x1F01U, 3000), "the head's upgrade does not");

	/* head first (1 hop), local upgrade (3 hops): ours, relayed */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1F02U, "upgrade me", wire, &len);
	set_hop_limit(wire, 1U);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "head first");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	set_hop_limit(wire, 3U);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_true(relayed_within(FAR_NODE_ID, 0x1F02U, 4000), "our upgrade relays");
}

/* B7 (review F2): a node on a custom modem (use_preset=false) hashes an unnamed
 * slot under "Custom"; its OWN radio's frames now carry a concrete preset tag,
 * and must still decode against the cached hash. */
ZTEST(attachment, test_custom_modem_local_frame_still_decodes)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	uint8_t cached, recomputed;
	int r;

	mt.use_preset = false;
	meshtastic_channels_refresh_derived();
	cached = meshtastic_channels_get_hash(0U);
	recomputed = meshtastic_channels_hash_for_preset(0U, (uint8_t)mt.modem_preset);
	zassert_equal(recomputed, cached, "the active preset's hash is the cached one (0x%02x vs 0x%02x)",
		      recomputed, cached);
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1C00U, "custom", wire, &len);
	zassert_equal(((struct meshtastic_wire_header *)wire)->channel, cached);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	r = k_sem_take(&rx.sem, K_SECONDS(2));
	mt.use_preset = true;
	meshtastic_channels_refresh_derived();
	zassert_ok(r, "own-radio frame on an unnamed slot under a custom modem not decoded");
}

/* B9 (review F7): on the local radio a relay byte equal to our low byte is a
 * neighbour who happens to share it, relaying us -- an implicit ACK upstream,
 * never "our own voice". The guard is asked only of attachment frames. */
ZTEST(attachment, test_own_voice_guard_is_for_attachments_only)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	build_frame(TEST_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1B00U, "mine", wire, &len);
	((struct meshtastic_wire_header *)wire)->relay_node = (uint8_t)(TEST_NODE_ID & 0xFFU);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "own frames are never delivered");
	zassert_equal(mt.status.self_heard, 0U, "not our voice: our radio cannot hear itself");
	zassert_false(relayed_within(TEST_NODE_ID, 0x1B00U, 500), "own echo is never relayed");
}

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
	/* Admit head 1 the way it always is: by hearing something through it. */
	{
		uint8_t wire[MESHTASTIC_PKT_MAX];
		uint32_t len;

		build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1801U, "hello", wire, &len);
		zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -80, 6, wire, len), "ingest");
		zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	}
	id = meshtastic_attachment_id_for_node(HEAD1_NODE);
	zassert_true(id != 0U, "head 1 is known");
	memset(&sent, 0, sizeof(sent));
	zassert_ok(meshtastic_attachment_set_preset(id, 2U));
	zassert_equal(sent.count, 1U);
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_SET_PRESET);
	zassert_equal(msg.u.preset, 2U);

	zassert_ok(meshtastic_attachment_forget(id));
	zassert_equal(meshtastic_attachment_id_for_node(HEAD1_NODE), 0U);
}

/* R2's instrument (ATTACHMENT-SCOPE F5): when the brain's own radio hears a
 * frame and a head then delivers the same frame, the gap between the two
 * arrivals on the brain's clock is that head's link latency -- both radios
 * heard the air at the same instant. Measured only in that order: a head's
 * copy arriving FIRST says nothing about the link (the twin). */
ZTEST(attachment, test_link_latency_is_measured_from_a_local_first_duplicate)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_info a;

	/* Admit the head with a frame of its own, so id 1 exists. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1A00U, "admit", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.lat_n, 0U, "nothing measured yet");

	/* Local first, the head 120 ms later: one sample of ~120 ms. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1A01U, "local first", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered from the local radio");
	k_msleep(120);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "a duplicate, not delivered");
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.lat_n, 1U, "one sample (%u)", a.lat_n);
	zassert_true(a.lat_min_ms >= 120U && a.lat_min_ms < 400U, "latency ~120 ms (%u)",
		     a.lat_min_ms);
	zassert_equal(a.lat_max_ms, a.lat_min_ms);

	/* A second, slower sample: min/max/avg follow. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1A02U, "local first 2", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	(void)k_sem_take(&rx.sem, K_MSEC(200));
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.lat_n, 2U);
	zassert_true(a.lat_max_ms >= 300U, "max is the slow one (%u)", a.lat_max_ms);
	zassert_true(a.lat_min_ms < a.lat_max_ms);
	zassert_true(a.lat_sum_ms / a.lat_n >= a.lat_min_ms &&
		     a.lat_sum_ms / a.lat_n <= a.lat_max_ms);

	/* The twin: head first, local radio second -- not a measurement. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1A03U, "head first", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered from the head");
	k_msleep(100);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "a duplicate");
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.lat_n, 2U, "a head-first duplicate adds no sample (%u)", a.lat_n);

	/* The other twin (bench, 22:06Z): a neighbour REBROADCASTS the frame the
	 * local radio heard first, and the head forwards that copy -- one hop
	 * fewer, another relay byte, seconds later. Same (src, id), not the same
	 * air: no sample. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1A04U, "relayed later", wire, &len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -80, 6));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	{
		struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

		h->flags = (uint8_t)((h->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
				     ((h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK) - 1U));
		h->relay_node = 0x5AU;
	}
	k_msleep(150);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "a duplicate");
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.lat_n, 2U, "a rebroadcast through the head adds no sample (%u)", a.lat_n);
}

/* B8 (review): the positive twin of the own-voice guard, asserted on the
 * reliable module itself. A want_ack unicast leaves on our radio and is
 * pending. Heard back through a head with a NEIGHBOUR's relay byte, it is an
 * implicit ACK -- someone forwarded it -- and the tracker is resolved. Heard
 * back through a head with OUR OWN relay byte, it is our own transmission
 * reaching one of our radios: not an ACK, the tracker stays. */
ZTEST(attachment, test_implicit_ack_through_a_head_resolves_the_reliable_send)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	struct lora_sim_frame tx;
	uint32_t self_heard_before;

	meshtastic_reliable_reset();

	/* Our own voice through a head: pending stays. */
	{
		struct meshtastic_packet dm = {
			.to = FAR_NODE_ID,
			.id = 0x0B080001U,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = (const uint8_t *)"ack me",
			.payload_len = 6U,
			.hop_limit = 3U,
			.hop_start = 3U,
			.want_ack = true,
			.channel_index = 0U,
		};

		zassert_ok(meshtastic_send_packet(&dm, K_NO_WAIT), "reliable DM send");
		zassert_ok(lora_sim_take_tx(lora_dev, &tx, K_SECONDS(3)), "the DM left on our radio");
		zassert_equal(meshtastic_reliable_pending(), 1U, "awaiting an ACK");

		memcpy(wire, tx.data, tx.len);
		((struct meshtastic_wire_header *)wire)->relay_node = (uint8_t)(TEST_NODE_ID & 0xFFU);
		self_heard_before = mt.status.self_heard;
		zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 10, wire, tx.len), "via head");
		k_msleep(200);
		zassert_equal(mt.status.self_heard, self_heard_before + 1U, "our own voice");
		zassert_equal(meshtastic_reliable_pending(), 1U,
			      "our own transmission through a head is not an implicit ACK");
	}

	/* A neighbour's rebroadcast through a head: pending resolves. */
	{
		memcpy(wire, tx.data, tx.len);
		((struct meshtastic_wire_header *)wire)->relay_node = 0x77U;
		set_hop_limit(wire, 2U);
		zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -60, 10, wire, tx.len), "via head");
		k_msleep(200);
		zassert_equal(mt.status.self_heard, self_heard_before + 1U, "a real echo, not self");
		zassert_equal(meshtastic_reliable_pending(), 0U,
			      "a neighbour's rebroadcast heard through a head is an implicit ACK");
		zassert_equal(rx.count, 0U, "an echo is never delivered");
	}
	meshtastic_reliable_reset();
}

/* P3 slice 1: the pipe. A frame enqueued for attachment 1 leaves as a TX_FRAME
 * to that head -- its preset, its bytes -- and never on our own radio; a
 * TX_RESULT from the head is accounted. The twin: attachment 0 leaves on the
 * radio and nothing reaches the bearer. A head that is not ready refuses at
 * enqueue, where the caller can still be told. */
ZTEST(attachment, test_tx_through_a_head_leaves_as_tx_frame)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_info a;
	uint32_t before;

	/* Admit the head with a frame of its own (id 1, preset ShortTurbo). */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x1C00U, "admit", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	drain_radio();
	zassert_true(meshtastic_attachment_tx_ready(1U), "head 1 is ready");
	zassert_false(meshtastic_attachment_tx_ready(2U), "no head 2");

	build_frame(TEST_NODE_ID, FAR_NODE_ID, 0x1C01U, "via head", wire, &len);
	k_msleep(300); /* let any relay of the admit frame (a broadcast) go first */
	zassert_true(meshtastic_attachment_get(1U, &a));
	{
		const uint16_t seq0 = a.tx_seq;
		const uint32_t handed0 = a.tx_frames;

	before = sent.count;
	zassert_ok(meshtastic_radio_send_wire_after_on(wire, len, MT_SCHED_TIER_NORMAL, 0U, 1U),
		   "enqueue for head 1");
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "one TX_FRAME to the head");
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_FRAME);
	zassert_equal(msg.u.tx.preset, (uint8_t)PRESET_ST, "the head's preset");
	zassert_equal(msg.u.tx.wire_len, len);
	zassert_mem_equal(msg.u.tx.wire, wire, len, "the bytes as built");
	zassert_equal(msg.u.tx.tx_seq, (uint16_t)(seq0 + 1U));
	zassert_true((msg.u.tx.flags & MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT) != 0U);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing on our radio");
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_equal(a.tx_frames, handed0 + 1U);
	zassert_equal(a.tx_seq, (uint16_t)(seq0 + 1U));
	}

	/* The head reports back. */
	{
		uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
		const struct meshtastic_attachment_tx_result r = {
			.tx_seq = a.tx_seq, .rc = 0, .defers = 1U, .tx_ms = 1234U,
		};
		int elen = meshtastic_attachment_encode_tx_result(&r, env, sizeof(env));

		zassert_true(elen > 0);
		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, HEAD1_NODE, env, (size_t)elen));
	}
	zassert_true(meshtastic_attachment_get(1U, &a));
	zassert_true(a.tx_results >= 1U);
	zassert_equal(a.last_tx_rc, 0);
	zassert_equal(a.last_tx_defers, 1U);
	zassert_equal(a.tx_failed, 0U);

	/* agents-pcs2.4: a withdrawn relay and a stale one are the relay rules at
	 * work, counted apart; only a real error is a failure. */
	{
		static const int8_t rcs[] = {MESHTASTIC_ATTACHMENT_RC_CANCELLED, -ETIME, -EIO};
		uint32_t c0 = a.tx_cancelled, l0 = a.tx_late, f0 = a.tx_failed;

		for (size_t i = 0; i < ARRAY_SIZE(rcs); i++) {
			uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
			const struct meshtastic_attachment_tx_result r = {
				.tx_seq = a.tx_seq, .rc = rcs[i], .tx_ms = 1235U,
			};
			int elen = meshtastic_attachment_encode_tx_result(&r, env, sizeof(env));

			zassert_true(elen > 0);
			zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, HEAD1_NODE, env,
							       (size_t)elen));
		}
		zassert_true(meshtastic_attachment_get(1U, &a));
		zassert_equal(a.tx_cancelled, c0 + 1U, "cancelled");
		zassert_equal(a.tx_late, l0 + 1U, "late");
		zassert_equal(a.tx_failed, f0 + 1U, "only -EIO is a failure");
	}

	/* The twin: attachment 0 is our own radio. */
	build_frame(TEST_NODE_ID, FAR_NODE_ID, 0x1C02U, "local", wire, &len);
	before = sent.count;
	zassert_ok(meshtastic_radio_send_wire_after_on(wire, len, MT_SCHED_TIER_NORMAL, 0U, 0U));
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(2)), "on our radio");
	zassert_equal(f.len, len);
	k_msleep(100);
	zassert_equal(sent.count, before, "nothing to the head");

	/* Not ready: unknown, or the head says receive-only. */
	zassert_equal(meshtastic_radio_send_wire_after_on(wire, len, MT_SCHED_TIER_NORMAL, 0U, 2U),
		      -EHOSTUNREACH, "no such head");
	{
		uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
		const struct meshtastic_attachment_status st = {
			.preset = (uint8_t)PRESET_ST,
			.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD | MESHTASTIC_ATTACHMENT_ST_RX_ONLY,
			.hwid = HEAD1_NODE, .brain = TEST_NODE_ID,
		};
		int elen = meshtastic_attachment_encode_status(&st, env, sizeof(env));

		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, HEAD1_NODE, env, (size_t)elen));
	}
	zassert_false(meshtastic_attachment_tx_ready(1U), "the head says receive-only");
	zassert_equal(meshtastic_radio_send_wire_after_on(wire, len, MT_SCHED_TIER_NORMAL, 0U, 1U),
		      -EHOSTUNREACH, "refused at enqueue");
}

/* P3 slice 2, reply-on-arrival: a want_ack DM heard through a head on
 * MediumFast gets its ACK handed to THAT head, with the channel byte hashed
 * under MediumFast (decodable where the sender listens), and nothing leaves
 * our own radio. The twin: a DM heard on our own radio is ACKed on our own
 * radio, and the head sees nothing. A head that reports itself receive-only
 * gets no reply and neither does our radio: a reply on another preset is
 * noise, not a fallback. */
ZTEST(attachment, test_reply_leaves_by_the_radio_the_request_came_in_on)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	const uint8_t here = meshtastic_channels_get_hash(0U);
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);
	uint32_t before;

	/* Admit head 1 on MediumFast. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2A00U, "admit", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	drain_radio();
	before = sent.count;

	/* A want_ack DM to us, through the head. */
	build_frame(FAR_NODE_ID, TEST_NODE_ID, 0x2A01U, "ack me via head", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	((struct meshtastic_wire_header *)wire)->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "the DM is delivered");
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_true(sent.count > before, "a reply reached the head");
	k_msleep(300);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(100)), 0,
			  "nothing left on our own radio");
	/* The last envelope to the head is one of the replies (the routing ACK,
	 * or the NodeInfo request to an unknown sender): to the sender, hashed
	 * under the head's preset, an own frame the head times itself. */
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_FRAME);
	zassert_equal(msg.u.tx.preset, (uint8_t)PRESET_MF);
	zassert_true((msg.u.tx.flags & MESHTASTIC_ATTACHMENT_TXF_OWN_DELAY) != 0U, "own delay");
	{
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)msg.u.tx.wire;

		zassert_equal(sys_le32_to_cpu(h->dest), FAR_NODE_ID, "to the sender");
		zassert_equal(sys_le32_to_cpu(h->src), TEST_NODE_ID, "from us");
		zassert_equal(h->channel, there, "hashed under the HEAD's preset (0x%02x vs 0x%02x)",
			      h->channel, there);
		zassert_not_equal(h->channel, here);
	}

	/* The twin: through our own radio, the ACK leaves on our own radio. */
	before = sent.count;
	build_frame(FAR_NODE_ID, TEST_NODE_ID, 0x2A02U, "ack me locally", wire, &len);
	((struct meshtastic_wire_header *)wire)->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -70, 8));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_SECONDS(3)), "the ACK on our radio");
	{
		const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)f.data;

		zassert_equal(sys_le32_to_cpu(h->dest), FAR_NODE_ID);
		zassert_equal(h->channel, here, "hashed under OUR preset");
	}
	k_msleep(200);
	zassert_equal(sent.count, before, "nothing to the head");

	/* A receive-only head: the reply is dropped, not misrouted. */
	{
		uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
		const struct meshtastic_attachment_status st = {
			.preset = (uint8_t)PRESET_MF,
			.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD | MESHTASTIC_ATTACHMENT_ST_RX_ONLY,
			.hwid = HEAD1_NODE, .brain = TEST_NODE_ID,
		};
		int elen = meshtastic_attachment_encode_status(&st, env, sizeof(env));

		zassert_ok(meshtastic_attach_bearer_rx(&test_bearer, HEAD1_NODE, env, (size_t)elen));
	}
	drain_radio();
	before = sent.count;
	build_frame(FAR_NODE_ID, TEST_NODE_ID, 0x2A03U, "ack me, head rx-only", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	((struct meshtastic_wire_header *)wire)->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "still delivered");
	k_msleep(400);
	zassert_equal(sent.count, before, "no reply to a receive-only head");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(100)), 0,
			  "and none on our own radio either");
}

/* P3 slice 3: a flood heard through a head is relayed THROUGH that head -- never
 * on our own radio -- as a TX_FRAME{RELAY}: the relay as we built it (one hop
 * fewer, our relay byte), relay_of the frame with the head's own rx_ms echoed,
 * and a window planned with the head's modem and the SNR the head heard it at.
 * Two heads on one preset: one relay. A copy that reaches us by our own radio
 * withdraws it (CLIENT: cancel-on-duplicate) with a TX_CANCEL to the head. */
ZTEST(attachment, test_relay_goes_through_the_head_that_heard_it)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct lora_sim_frame f;
	struct meshtastic_attachment_msg msg;
	const uint8_t there = meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF);
	uint32_t relayed_before;
	uint32_t before;

	/* Admit heads 1 and 2 on MediumFast (broadcasts: relayed through them too). */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x3A00U, "admit 1", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x3A01U, "admit 2", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears(HEAD2_NODE, PRESET_MF, -90, 2, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300);
	drain_radio();
	before = sent.count;
	relayed_before = mt.status.relayed_packets;

	/* A flood from FAR, 3 hops, heard by head 1 at its uptime 5000 ms. */
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x3A02U, "relay me", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = there;
	zassert_ok(head_hears_at(HEAD1_NODE, PRESET_MF, -80, 6, wire, len, 5000U));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "one relay handed to head 1 (%u)", sent.count - before);
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_FRAME);
	zassert_true((msg.u.tx.flags & MESHTASTIC_ATTACHMENT_TXF_RELAY) != 0U, "a relay");
	zassert_true((msg.u.tx.flags & MESHTASTIC_ATTACHMENT_TXF_OWN_DELAY) == 0U, "not an own frame");
	zassert_equal(msg.u.tx.relay_src, FAR_NODE_ID);
	zassert_equal(msg.u.tx.relay_id, 0x3A02U);
	zassert_equal(msg.u.tx.rx_ms, 5000U, "the head's own clock of reception, echoed");
	zassert_equal(msg.u.tx.dupe, MESHTASTIC_ATTACHMENT_DUPE_CANCEL, "a CLIENT cancels on a dupe");
	{
		/* Planned with MediumFast's slot time and the head's SNR (6 dB). */
		uint32_t slot = meshtastic_contention_effective_slot_ms(9U, 250000U, false);
		uint32_t worst = meshtastic_contention_delay_relay_worst_ms(6, slot);
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)msg.u.tx.wire;

		zassert_true(msg.u.tx.not_before_ms <= worst, "inside the window (%u <= %u)",
			     msg.u.tx.not_before_ms, worst);
		zassert_equal(h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 2U, "one hop fewer");
		zassert_equal(h->relay_node, (uint8_t)(TEST_NODE_ID & 0xFFU), "our relay byte");
		zassert_equal(sys_le32_to_cpu(h->src), FAR_NODE_ID);
		zassert_equal(h->channel, there, "still the head's preset's hash");
	}
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "nothing on our radio");
	zassert_equal(mt.status.relayed_packets, relayed_before + 1U, "counted as relayed");

	/* Head 2 hears the same frame: a duplicate, no second relay. */
	before = sent.count;
	zassert_ok(head_hears_at(HEAD2_NODE, PRESET_MF, -90, 2, wire, len, 7000U));
	k_msleep(300);
	zassert_equal(sent.count, before, "one relayer per preset");

	/* Our own radio hears a neighbour's rebroadcast of it: withdraw the head's. */
	{
		struct meshtastic_wire_header *h = (struct meshtastic_wire_header *)wire;

		h->relay_node = 0x77U;
		set_hop_limit(wire, 2U);
	}
	before = sent.count;
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -70, 8));
	for (int i = 0; i < 100 && sent.count == before; i++) {
		k_msleep(10);
	}
	zassert_equal(sent.count, before + 1U, "a TX_CANCEL reached head 1");
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_TX_CANCEL);
	zassert_equal(msg.u.cancel.src, FAR_NODE_ID);
	zassert_equal(msg.u.cancel.id, 0x3A02U);
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(300)), 0, "still nothing on our radio");
}

/* The brain's side of SET_POLICY: one envelope to the head, and nothing to
 * a head that is not there. */
ZTEST(attachment, test_set_tx_power_reaches_the_head)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_msg msg;
	uint32_t before;

	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x4A00U, "admit", wire, &len);
	zassert_ok(head_hears(HEAD1_NODE, PRESET_ST, -90, 5, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	k_msleep(300);
	before = sent.count;
	zassert_ok(meshtastic_attachment_set_tx_power(1U, 2));
	zassert_equal(sent.count, before + 1U);
	zassert_equal(sent.node, HEAD1_NODE);
	zassert_ok(meshtastic_attachment_decode(sent.env, sent.len, &msg));
	zassert_equal(msg.type, MESHTASTIC_ATTACHMENT_SET_POLICY);
	zassert_equal(msg.u.policy.tx_power, 2);
	zassert_true((msg.u.policy.flags & MESHTASTIC_ATTACHMENT_POL_HAS_TX_POWER) != 0U);
	zassert_equal(meshtastic_attachment_set_tx_power(2U, 2), -ENOENT, "no head 2");
}

/* The other direction of R3's sim half: the BRAIN's radio is deaf while its
 * co-sited head transmits (a relay the brain itself handed over). What the
 * brain misses is bounded by the head's airtime: a frame on the brain's own
 * air during it is lost and counted; the same frame after it is delivered. */
static const struct device *const head_radio = DEVICE_DT_GET(DT_NODELABEL(lora_sim1));

ZTEST(attachment, test_brain_is_deaf_while_its_cosited_head_transmits)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;

	zassert_ok(lora_sim_set_cosite(lora_dev, head_radio), "paired: the head's radio beside us");
	build_frame(FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x5A00U, "during the head's tx", wire, &len);
	lora_sim_set_busy(head_radio, 200U);
	zassert_equal(lora_sim_inject(lora_dev, wire, (uint8_t)len, -70, 8), -ECANCELED, "blanked");
	zassert_equal(lora_sim_rx_blanked(lora_dev), 1U);
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN, "lost, not delivered");

	k_msleep(250);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)len, -70, 8), "after the head's air");
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "delivered");
	lora_sim_reset(head_radio);
	lora_sim_reset(lora_dev);
}

/* Same as the reply test above, but with the bench's channel table: a NAMED primary
 * (its hash is its name's, on every preset) and an unnamed default-PSK secondary in
 * slot 1 -- the slot a MediumFast sender's frame actually decodes on. The replies
 * (routing ACK, NodeInfo request) must carry slot 1's hash under the head's preset,
 * not the primary's name hash: a request built on index 0 left the bench's brain as
 * 0x0e ("ShortTurbo") through a MediumFast head, and the stock node there logged
 * "No channel found for decoding, hash 0xe" and never learned the brain (2026-09-29).
 * Reversal: nodeinfo's request back on index 0 (channel_index unset) fails this. */
static void build_frame_on(uint8_t slot, uint32_t from, uint32_t to, uint32_t id,
			   const char *text, uint8_t *wire, uint32_t *wire_len)
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
		.channel_index = slot,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len), "build_wire_packet");
}

ZTEST(attachment, test_replies_via_a_head_use_the_slot_the_frame_arrived_on_not_a_named_primary)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	struct meshtastic_attachment_msg msg;
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	uint8_t primary_hash;
	uint8_t slot1_there;
	uint32_t before;
	int replies_seen = 0;

	/* First, the bench's own table: the primary NAMED after the preset the node is
	 * on ("ShortTurbo", on ShortTurbo). That is the default channel under this
	 * preset by any test that reaches the air -- the same hash byte as an unnamed
	 * slot -- so under the head's preset it re-hashes like an unnamed slot. */
	ch.index = 0;
	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 1U;
	strcpy(ch.settings.name, "ShortTurbo");
	zassert_ok(meshtastic_channels_set_slot(0U, &ch), "name slot 0 after the preset");
	zassert_equal(meshtastic_channels_get_hash(0U),
		      meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_ST),
		      "on our own preset the name IS the default channel's hash");
	zassert_not_equal(meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF),
			  meshtastic_channels_get_hash(0U),
			  "under the head's preset a preset-named slot re-hashes like an unnamed one");
	/* Now a TRULY named primary ("Private"): its hash is its own on every preset. */
	strcpy(ch.settings.name, "Private");
	zassert_ok(meshtastic_channels_set_slot(0U, &ch), "name slot 0");
	primary_hash = meshtastic_channels_get_hash(0U);
	/* Slot 1: an unnamed secondary on the same PSK -- named after whatever
	 * preset it is used on, so its hash differs per preset. */
	ch = (meshtastic_Channel)meshtastic_Channel_init_zero;
	ch.index = 1;
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 1U;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "set slot 1");
	slot1_there = meshtastic_channels_hash_for_preset(1U, (uint8_t)PRESET_MF);
	zassert_not_equal(slot1_there, primary_hash, "the two hashes must differ for the test to bite");
	zassert_equal(meshtastic_channels_hash_for_preset(0U, (uint8_t)PRESET_MF), primary_hash,
		      "a named primary keeps its name's hash on every preset");

	/* Admit head 1 on MediumFast, with a frame on slot 1's MF hash. */
	build_frame_on(1U, FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0x2B00U, "admit", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = slot1_there;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)));
	drain_radio();
	before = sent.count;

	/* A want_ack DM from an UNKNOWN sender on slot 1, through the head: the
	 * brain answers with a routing ACK and asks for NodeInfo -- both must be on
	 * slot 1's hash under MediumFast, never the primary's name hash. */
	build_frame_on(1U, FAR_NODE_ID + 7U, TEST_NODE_ID, 0x2B01U, "ack me via head", wire, &len);
	((struct meshtastic_wire_header *)wire)->channel = slot1_there;
	((struct meshtastic_wire_header *)wire)->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	zassert_ok(head_hears(HEAD1_NODE, PRESET_MF, -80, 6, wire, len));
	zassert_ok(k_sem_take(&rx.sem, K_SECONDS(2)), "the DM is delivered");
	for (int i = 0; i < 300 && sent.count < before + 2U; i++) {
		k_msleep(10);
	}
	zassert_true(sent.count >= before + 2U, "two replies reached the head (ACK + NodeInfo request), got %u",
		     (unsigned int)(sent.count - before));
	k_msleep(300);
	/* Every TX_FRAME handed since: to the sender, on slot 1's MediumFast hash. */
	for (uint32_t i = 0; i < sent_ring_count(); i++) {
		const struct meshtastic_wire_header *h;

		if (!sent_ring_get(i, &msg)) {
			continue;
		}
		if (msg.type != MESHTASTIC_ATTACHMENT_TX_FRAME) {
			continue;
		}
		h = (const struct meshtastic_wire_header *)msg.u.tx.wire;
		if (sys_le32_to_cpu(h->src) != TEST_NODE_ID || sys_le32_to_cpu(h->dest) != FAR_NODE_ID + 7U) {
			continue;
		}
		replies_seen++;
		zassert_equal(h->channel, slot1_there,
			      "reply hashed under slot 1 on the head's preset (0x%02x), got 0x%02x",
			      slot1_there, h->channel);
		zassert_not_equal(h->channel, primary_hash, "never the named primary's hash");
	}
	zassert_true(replies_seen >= 2, "ACK and NodeInfo request both seen (%d)", replies_seen);
	/* Leave the table as the fixture expects: slot 1 gone, slot 0 unnamed. */
	ch = (meshtastic_Channel)meshtastic_Channel_init_zero;
	ch.index = 1;
	ch.role = meshtastic_Channel_Role_DISABLED;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch), "clear slot 1");
	set_default_primary();
}

/* DESIGN §13, the placement guard. FAST-HEAD: a head whose preset's slot time is shorter
 * than our own radio's -- the fastest preset an identity serves belongs on the brain. */
static bool attach_info_for(uint32_t node, struct meshtastic_attachment_info *out)
{
	for (uint8_t id = 1U; id <= CONFIG_MESHTASTIC_ATTACHMENT_MAX; id++) {
		if (meshtastic_attachment_get(id, out) && out->node == node) {
			return true;
		}
	}
	return false;
}

static void head_reports_preset(uint32_t node, uint8_t preset)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_status st = {
		.preset = preset,
		.flags = MESHTASTIC_ATTACHMENT_ST_TX_ENABLED | MESHTASTIC_ATTACHMENT_ST_IS_HEAD,
		.hwid = node,
		.brain = TEST_NODE_ID,
	};
	int len = meshtastic_attachment_encode_status(&st, env, sizeof(env));

	zassert_true(len > 0);
	zassert_ok(meshtastic_attachment_ingest(node, env, (size_t)len), "status ingest");
}

ZTEST(attachment, test_guard_flags_a_head_faster_than_our_own_radio)
{
	struct meshtastic_attachment_info a;

	/* We are on MediumFast; a head on ShortTurbo is faster than us. */
	zassert_ok(meshtastic_preset_switch(PRESET_MF, NULL), "preset");
	head_reports_preset(HEAD1_NODE, (uint8_t)PRESET_ST);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_true(a.warn_fast_head, "ShortTurbo head on a MediumFast brain: FAST-HEAD");
	/* The same head retuned to LongFast: slower than us, no warning. */
	head_reports_preset(HEAD1_NODE, (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_false(a.warn_fast_head, "a LongFast head on a MediumFast brain is fine");
	/* Back on ShortTurbo ourselves (the fixture's preset), a MediumFast head is fine. */
	zassert_ok(meshtastic_preset_switch(PRESET_ST, NULL), "preset");
	head_reports_preset(HEAD1_NODE, (uint8_t)PRESET_MF);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_false(a.warn_fast_head, "the bench's layout: brain ShortTurbo, head MediumFast");
}

/* LAG: the head reports, per relay, how long the decision took to reach it (its own clock);
 * the brain keeps a 16-sample ring. Warn when p90 exceeds half the head preset's ROUTER
 * window AND our role relays early; suppressed under CLIENT, where it cannot bite. */
static void head_reports_lag(uint32_t node, uint16_t seq, uint16_t lag_ms)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_tx_result r = {
		.tx_seq = seq, .rc = 0, .defers = 0, .tx_ms = 1000U, .has_lag = true, .lag_ms = lag_ms,
	};
	int len = meshtastic_attachment_encode_tx_result(&r, env, sizeof(env));

	zassert_equal(len, (int)MESHTASTIC_ATTACHMENT_TX_RESULT_LAG_LEN, "the lag rides appended");
	zassert_ok(meshtastic_attachment_ingest(node, env, (size_t)len), "result ingest");
}

ZTEST(attachment, test_guard_flags_link_lag_over_half_the_router_window_for_a_router_only)
{
	struct meshtastic_attachment_info a;
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_tx_result old_head = {
		.tx_seq = 99U, .rc = 0, .defers = 0, .tx_ms = 1000U, .has_lag = false,
	};

	/* A MediumFast head (slot 12 ms, ROUTER window 60 ms, threshold 30 ms). */
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	head_reports_preset(HEAD1_NODE, (uint8_t)PRESET_MF);
	for (uint16_t i = 0U; i < 16U; i++) {
		head_reports_lag(HEAD1_NODE, i, (i == 15U) ? 200U : 40U);
	}
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_equal(a.lag_n, 16U);
	zassert_equal(a.lag_p50_ms, 40U);
	zassert_equal(a.lag_p90_ms, 40U, "p90 of 16 is the 2nd largest, not the one outlier");
	zassert_equal(a.lag_max_ms, 200U);
	zassert_false(a.warn_lag, "under CLIENT the lag cannot bite");
	/* As a ROUTER the same samples warn. */
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	head_reports_lag(HEAD1_NODE, 16U, 40U);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_true(a.warn_lag, "40 ms p90 > 30 ms (half of MediumFast's 60 ms ROUTER window)");
	/* Back to CLIENT: the flag goes with the role, no new sample needed
	 * (agents-pcs2.5 -- it used to stay set until the next relay). */
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_false(a.warn_lag, "CLIENT again: LAG clears on read");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_true(a.warn_lag, "and returns with ROUTER");
	/* An older head's 9-byte result still decodes and adds no sample. */
	zassert_equal(meshtastic_attachment_encode_tx_result(&old_head, env, sizeof(env)),
		      (int)MESHTASTIC_ATTACHMENT_TX_RESULT_LEN);
	zassert_ok(meshtastic_attachment_ingest(HEAD1_NODE, env, MESHTASTIC_ATTACHMENT_TX_RESULT_LEN));
	zassert_true(attach_info_for(HEAD1_NODE, &a));
	zassert_equal(a.lag_n, 16U, "no sample from a result without the field");
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
}
