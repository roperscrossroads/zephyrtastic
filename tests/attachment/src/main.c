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
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#include "meshtastic_sched.h"

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
static struct {
	uint32_t node;
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	size_t len;
	uint32_t count;
} sent;

static enum meshtastic_attach_auth test_auth = MESHTASTIC_ATTACH_AUTH_ENCRYPTED;

static int test_bearer_send(uint32_t node, const uint8_t *env, size_t len)
{
	if (len > sizeof(sent.env)) {
		return -EMSGSIZE;
	}
	sent.node = node;
	memcpy(sent.env, env, len);
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
