/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Tier 1 mesh-sim smoke tests: the REAL Meshtastic stack driven against the
 * simulated LoRa driver (drivers/lora_sim) with no radio hardware. Proves the
 * injection seam described in docs/sim-lora-driver.md §2 ("no library changes
 * needed") end to end — TX capture, RX injection, and flood dedup all run under
 * twister on native_sim.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/fem.h>
#include <zephyr/meshtastic/nodedb.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#if defined(CONFIG_MESHTASTIC_CLUSTER)
#include <pb_encode.h>

#include <pb_decode.h>

#include "meshtastic/config.pb.h"
#include "zephyrtastic/cluster.pb.h"

#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_hlc.h"
#endif
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_scanner.h"
#endif
#if defined(CONFIG_MESHTASTIC_RF_HIST)
#include "meshtastic_rf_measure.h"
#endif
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
/* after meshtastic_packet.h: it declares struct meshtastic_wire_header, which
 * meshtastic_router.h uses by pointer in a prototype. */
#include "meshtastic_router.h"
#include "meshtastic_reliable.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define PEER_NODE_ID 0x0B0B0B0BU

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static struct {
	struct k_sem sem;
	uint32_t     from;
	uint32_t     portnum;
	uint8_t      payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	size_t       payload_len;
	uint32_t     count;
} rx;

static void on_recv(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t payload_len, int16_t rssi, int8_t snr)
{
	ARG_UNUSED(to);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);

	rx.from        = from;
	rx.portnum     = portnum;
	rx.payload_len = MIN(payload_len, sizeof(rx.payload));
	if (rx.payload_len > 0U) {
		memcpy(rx.payload, payload, rx.payload_len);
	}
	rx.count++;
	k_sem_give(&rx.sem);
}

/* Block until the radio is back in RX (the stack cancels RX while transmitting,
 * so a relay from a previous inject can briefly leave nothing listening). */
static void wait_rx_armed(void)
{
	for (int i = 0; i < 500 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

/* Fully quiesce the stack between tests: wait for the radio to return to RX and
 * for any relay TX triggered by the previous test's inject to finish being
 * captured, then clear the capture queue. Without this a leftover relay frame
 * (which carries the original sender, not us) is mistaken for the next test's
 * own transmission. */
static void quiesce(void)
{
	int stable = 0;

	wait_rx_armed();
	for (int i = 0; i < 200 && stable < 3; i++) {
		int n = lora_sim_tx_pending(lora_dev);

		k_msleep(5); /* virtual time: lets the mesh thread run any relay */
		if (lora_sim_rx_armed(lora_dev) && lora_sim_tx_pending(lora_dev) == n) {
			stable++;
		} else {
			stable = 0;
		}
	}
	lora_sim_reset(lora_dev);
}

/* ==========================================================================
 * Test doubles for the board FEM hooks (<zephyr/meshtastic/fem.h>).
 *
 * No board FEM source (e.g. heltec_wifi_lora32_v4_fem.c) is linked into
 * native_sim, so meshtastic_radio.c's __weak defaults would otherwise apply
 * everywhere -- which is exactly what the existing fem_lna_mode tests above
 * assert (can_control() == false). These strong overrides replace those
 * defaults for the whole suite, but every field defaults to the SAME
 * behaviour the weak versions gave (no LNA control, identity power
 * conversion, no-op TX steering), so nothing changes for a test that never
 * touches fem_spy. A test that wants the "board WITH a controllable
 * front-end" branch sets the relevant field and reads the rest back.
 */
static struct {
	bool     lna_can_control;
	bool     lna_last_set_value;
	uint32_t lna_set_count;

	int8_t tx_power_gain_db; /* subtracted from every requested radiated dBm */

	bool     tx_calls[8];
	uint32_t tx_call_count;
} fem_spy;

static void fem_spy_reset(void)
{
	memset(&fem_spy, 0, sizeof(fem_spy));
}

bool meshtastic_radio_fem_lna_can_control(void)
{
	return fem_spy.lna_can_control;
}

void meshtastic_radio_fem_lna_set(bool enable)
{
	fem_spy.lna_last_set_value = enable;
	fem_spy.lna_set_count++;
}

int8_t meshtastic_radio_fem_tx_power_conversion(int8_t radiated_dbm)
{
	return (int8_t)(radiated_dbm - fem_spy.tx_power_gain_db);
}

void meshtastic_radio_fem_set_tx(bool tx)
{
	if (fem_spy.tx_call_count < ARRAY_SIZE(fem_spy.tx_calls)) {
		fem_spy.tx_calls[fem_spy.tx_call_count] = tx;
	}
	fem_spy.tx_call_count++;
}

/* Encode a valid text-message wire frame originated by a fake neighbour. */
static void build_peer_text(uint32_t id, const char *text, uint8_t *wire, uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from          = PEER_NODE_ID,
		.to            = MESHTASTIC_NODE_BROADCAST,
		.id            = id,
		.portnum       = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload       = (const uint8_t *)text,
		.payload_len   = strlen(text),
		.hop_limit     = 3U,
		.hop_start     = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len),
		   "build_wire_packet failed");
}

static void *mesh_sim_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev     = lora_dev,
		.node_id      = TEST_NODE_ID,
		.psk          = meshtastic_default_psk,
		.psk_len      = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency    = MESHTASTIC_FREQ_EU,
	};

	k_sem_init(&rx.sem, 0, 1);
	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	meshtastic_set_recv_cb(on_recv);
	return NULL;
}

static void mesh_sim_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Let any relay TX from the previous test finish and the radio return to
	 * RX, then clear the capture queue, before we touch state. */
	quiesce();

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	memset(mt.dup_cache, 0, sizeof(mt.dup_cache));
	mt.dup_head = 0U;
	meshtastic_sched_defaults();
	/* Pin the contention window off so relay/TX timing is deterministic. */
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	meshtastic_reliable_reset();

	lora_sim_reset(lora_dev);
	memset(&rx, 0, sizeof(rx));
	k_sem_init(&rx.sem, 0, 1);
	fem_spy_reset();
}

/* TX seam: an originated text reaches the sim driver as a decodable wire frame,
 * and the frame carries a non-zero modelled airtime. */
ZTEST(mesh_sim, test_send_text_reaches_sim_driver)
{
	struct lora_sim_frame f;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	const char *msg = "hello sim";

	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, msg), "send_text failed");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)), "no TX captured");

	zassert_true(f.air_ms > 0U, "airtime not modelled (got 0)");
	zassert_ok(meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &decoded, payload,
						 sizeof(payload)),
		   "captured frame did not decode");
	zassert_equal(decoded.from, TEST_NODE_ID, "wrong source node");
	zassert_equal(decoded.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(decoded.payload_len, strlen(msg), "wrong payload length");
	zassert_mem_equal(payload, msg, strlen(msg), "wrong text");
}

/* RX seam (docs §8): a text injected from a fake neighbour surfaces to recv_cb. */
ZTEST(mesh_sim, test_injected_text_surfaces_to_recv_cb)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	const char *msg = "from a neighbour";

	build_peer_text(0x1001U, msg, wire, &wire_len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");

	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "recv_cb did not fire");
	zassert_equal(rx.from, PEER_NODE_ID, "wrong sender");
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(rx.payload_len, strlen(msg), "wrong payload length");
	zassert_mem_equal(rx.payload, msg, strlen(msg), "wrong text");
}

/* Flood dedup: the same (src,id) injected twice surfaces to the app exactly once. */
ZTEST(mesh_sim, test_duplicate_frame_delivered_once)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	const char *msg = "dup me";

	build_peer_text(0x2002U, msg, wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "first inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "first copy not delivered");

	/* The first copy is relayed (rebroadcast ALL), which cancels RX during the
	 * TX; wait for the radio to return before injecting the duplicate. */
	wait_rx_armed();

	/* An identical second copy must be deduplicated, not delivered again. */
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7),
		   "second inject failed");
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(200)), -EAGAIN,
		      "duplicate was delivered a second time");
	zassert_equal(rx.count, 1U, "expected exactly one delivery, got %u", rx.count);
}

/* The v1 link-local rule (agents-xhli.2): a frame arriving over a non-LoRa
 * bearer is delivered locally but NEVER relayed onto LoRa. The frame here is a
 * broadcast with hop budget — exactly what the LoRa path above does relay
 * (rebroadcast ALL) — so a captured TX would be the gate failing, not noise. */
ZTEST(mesh_sim, test_bearer_frame_delivered_locally_never_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct lora_sim_frame f;
	const char *msg = "via the peer bearer";

	build_peer_text(0x3003U, msg, wire, &wire_len);
	zassert_ok(meshtastic_radio_rx_inject(wire, (uint16_t)wire_len,
					      MESHTASTIC_BEARER_BLE_PEER),
		   "bearer inject failed");

	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "bearer frame not delivered");
	zassert_equal(rx.from, PEER_NODE_ID, "wrong sender");
	zassert_mem_equal(rx.payload, msg, strlen(msg), "wrong text");

	/* Delivery may legitimately trigger traffic WE originate (the NodeDB asks
	 * an unknown sender for NodeInfo); what must never appear is a relay —
	 * a TX still carrying the injected frame's (src,id). */
	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(300)) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		zassert_false(sys_le32_to_cpu(h->src) == PEER_NODE_ID &&
				      sys_le32_to_cpu(h->id) == 0x3003U,
			      "bearer frame was relayed onto LoRa");
	}
}

#if defined(CONFIG_MESHTASTIC_CLUSTER)
/* Cluster M4a: a digest describing a different document, arriving on the
 * cluster channel over the sim radio, is noticed (counted as a mismatch);
 * the same frame on the WRONG channel is refused before interpretation. */
ZTEST(mesh_sim, test_cluster_digest_divergence_noticed)
{
	zephyrtastic_ClusterMessage msg = zephyrtastic_ClusterMessage_init_zero;
	struct meshtastic_cluster_stats before_st, after_st;
	uint8_t cbuf[64];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t os = pb_ostream_from_buffer(cbuf, sizeof(cbuf));
	uint8_t ch_index;

	/* Provision the cluster channel (slot 2, the name the module binds). */
	{
		meshtastic_Channel ch = meshtastic_Channel_init_zero;

		ch.role = meshtastic_Channel_Role_SECONDARY;
		ch.has_settings = true;
		strncpy(ch.settings.name, CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME,
			sizeof(ch.settings.name) - 1U);
		ch.settings.psk.size = 16U;
		ch.settings.psk.bytes[0] = 0x42U;
		zassert_ok(meshtastic_channels_set_slot(2U, &ch), "cluster channel set failed");
	}
	zassert_true(meshtastic_cluster_channel_resolved(&ch_index), "module must bind");
	zassert_equal(ch_index, 2U);

	/* TX first: fire one digest through the real send path (workqueue →
	 * encode → channel-encrypt → radio) and capture it. This exact path
	 * reached hardware untested once — timer-only, sim cadence pinned to
	 * a day — and a sysworkq stack overflow cost a bench cycle. Never
	 * again: the seam exists so the sim drives it. */
	{
		struct lora_sim_frame f;
		const struct meshtastic_wire_header *h;

		meshtastic_cluster_digest_now();
		zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)),
			   "digest TX never reached the radio");
		h = (const struct meshtastic_wire_header *)f.data;
		zassert_equal(sys_le32_to_cpu(h->dest), MESHTASTIC_NODE_BROADCAST,
			      "digest must be a broadcast");
		zassert_equal(h->channel, meshtastic_channels_get_hash(2U),
			      "digest must ride the cluster channel's hash");
		wait_rx_armed();
	}

	/* A digest claiming one entry — our doc is empty, so this diverges. */
	msg.which_variant = zephyrtastic_ClusterMessage_digest_tag;
	msg.variant.digest.doc_hash = 0xDEADBEEFU;
	msg.variant.digest.entry_count = 1U;
	msg.variant.digest.has_max_stamp = true;
	msg.variant.digest.max_stamp.physical_ms = 12345;
	msg.variant.digest.max_stamp.node_id = PEER_NODE_ID;
	zassert_true(pb_encode(&os, zephyrtastic_ClusterMessage_fields, &msg), "encode failed");

	meshtastic_cluster_stats_get(&before_st);
	{
		struct meshtastic_packet pkt = {
			.from = PEER_NODE_ID,
			.to = MESHTASTIC_NODE_BROADCAST,
			.id = 0x5005U,
			.portnum = MESHTASTIC_PORT_PRIVATE,
			.payload = cbuf,
			.payload_len = os.bytes_written,
			.hop_limit = 3U,
			.hop_start = 3U,
			.channel_index = 2U,
		};

		zassert_ok(meshtastic_build_wire_packet(&pkt, wire, &wire_len), "encode failed");
	}
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	k_sleep(K_MSEC(200));

	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.digest_rx_mismatch, before_st.digest_rx_mismatch + 1U,
		      "divergent digest must be counted as a mismatch");
	zassert_equal(after_st.digest_rx_match, before_st.digest_rx_match,
		      "it must not be counted as a match");

	/* The broadcast digest was flood-relayed (by design — non-members carry
	 * cluster traffic too); wait out the relay TX before injecting again. */
	wait_rx_armed();

	/* The same payload on the PRIMARY channel is not cluster traffic. */
	{
		struct meshtastic_packet pkt = {
			.from = PEER_NODE_ID,
			.to = MESHTASTIC_NODE_BROADCAST,
			.id = 0x5006U,
			.portnum = MESHTASTIC_PORT_PRIVATE,
			.payload = cbuf,
			.payload_len = os.bytes_written,
			.hop_limit = 3U,
			.hop_start = 3U,
			.channel_index = meshtastic_channels_primary_index(),
		};

		zassert_ok(meshtastic_build_wire_packet(&pkt, wire, &wire_len), "encode failed");
	}
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	k_sleep(K_MSEC(200));

	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.digest_rx_mismatch, before_st.digest_rx_mismatch + 1U,
		      "wrong-channel frame must not reach digest handling");
	zassert_true(after_st.rx_wrong_channel > before_st.rx_wrong_channel,
		     "wrong-channel frame must be counted as such");

	/* Teardown: disable the channel so the module idles for other tests. */
	{
		meshtastic_Channel off = meshtastic_Channel_init_zero;

		off.role = meshtastic_Channel_Role_DISABLED;
		off.has_settings = true;
		zassert_ok(meshtastic_channels_set_slot(2U, &off), "channel teardown failed");
	}
}
#endif /* CONFIG_MESHTASTIC_CLUSTER */

/* Dedup spans bearers: the same (src,id) heard first over the peer link is a
 * duplicate when its LoRa copy arrives — one delivery, and the LoRa copy is
 * not relayed either (DUP_SEEN drops it before the relay path). */
ZTEST(mesh_sim, test_bearer_then_lora_copy_deduplicated)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct lora_sim_frame f;
	const char *msg = "one delivery only";

	build_peer_text(0x4004U, msg, wire, &wire_len);
	zassert_ok(meshtastic_radio_rx_inject(wire, (uint16_t)wire_len,
					      MESHTASTIC_BEARER_BLE_PEER),
		   "bearer inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "bearer copy not delivered");

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7),
		   "LoRa inject failed");
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(200)), -EAGAIN,
		      "LoRa copy was delivered a second time");
	zassert_equal(rx.count, 1U, "expected exactly one delivery, got %u", rx.count);
	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(200)) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		zassert_false(sys_le32_to_cpu(h->src) == PEER_NODE_ID &&
				      sys_le32_to_cpu(h->id) == 0x4004U,
			      "duplicate LoRa copy was relayed");
	}
}

/*
 * INTEROP: the on-air frame must match the wire format the stock Meshtastic
 * firmware (./firmware/) produces and expects, or a real mesh won't decode us.
 *
 * The other tests here build inject frames with the port's own encoder, so they
 * only prove self-consistency. This one pins the emitted frame's header — byte
 * for byte — to the upstream PacketHeader layout (firmware/src/mesh/, 16 bytes:
 * little-endian dest/src/id, then flags/channelHash/nextHop/relayNode), which is
 * predictable independently of the port's code. Combined with the primitive-level
 * upstream pins in the wire_vectors suite (channel-hash, nonce, crypto params) and
 * the on-air stock<->port runs in docs/parity/INTEROP-TEST-MATRIX.md, a drift away
 * from stock's format now fails in CI instead of only on the bench.
 */
ZTEST(mesh_sim, test_frame_header_matches_upstream_wire_layout)
{
	const uint32_t from = 0xDEADBEEFU; /* distinct bytes -> checks LE order */
	const uint32_t id   = 0x12345678U;
	const char *msg = "interop";
	struct meshtastic_packet packet = {
		.from          = from,
		.to            = MESHTASTIC_NODE_BROADCAST,
		.id            = id,
		.portnum       = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload       = (const uint8_t *)msg,
		.payload_len   = strlen(msg),
		.hop_limit     = 3U,
		.hop_start     = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len), "encode failed");
	zassert_true(wire_len >= 16U, "frame shorter than the 16-byte upstream header");

	/* dest = broadcast (0xFFFFFFFF) */
	zassert_equal(wire[0], 0xFFU, "dest[0]");
	zassert_equal(wire[1], 0xFFU, "dest[1]");
	zassert_equal(wire[2], 0xFFU, "dest[2]");
	zassert_equal(wire[3], 0xFFU, "dest[3]");
	/* src = from, little-endian */
	zassert_equal(wire[4], 0xEFU, "src[0] (LE)");
	zassert_equal(wire[5], 0xBEU, "src[1]");
	zassert_equal(wire[6], 0xADU, "src[2]");
	zassert_equal(wire[7], 0xDEU, "src[3]");
	/* id, little-endian */
	zassert_equal(wire[8],  0x78U, "id[0] (LE)");
	zassert_equal(wire[9],  0x56U, "id[1]");
	zassert_equal(wire[10], 0x34U, "id[2]");
	zassert_equal(wire[11], 0x12U, "id[3]");
	/* flags: hop_limit=3 (bits 0-2) | hop_start=3 (bits 5-7) = 0x63 */
	zassert_equal(wire[12], 0x63U, "flags byte (hop_limit|hop_start)");
	/* channel hash byte = the primary channel's hash, which is itself pinned to
	 * stock by wire_vectors::test_channel_hash_matches_upstream. */
	zassert_equal(wire[13], meshtastic_channels_get_hash(0), "channel hash byte");

	/* And the upstream-format frame round-trips: the port decodes it back. */
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "upstream-format frame not delivered");
	zassert_equal(rx.from, from, "wrong sender");
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(rx.payload_len, strlen(msg), "wrong payload length");
	zassert_mem_equal(rx.payload, msg, strlen(msg), "wrong text");
}

/* Point the primary channel at the default key via its 1-byte shorthand {0x01}
 * (as stock stores it), which both yields the on-air channel hash and expands to
 * the canonical default AES key — needed to match + decrypt a real stock frame. */
static void set_primary_default(const char *name)
{
	static const uint8_t psk_shorthand[1] = { 0x01 };
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	strncpy(ch.settings.name, name, sizeof(ch.settings.name) - 1U);
	ch.settings.psk.size = sizeof(psk_shorthand);
	memcpy(ch.settings.psk.bytes, psk_shorthand, sizeof(psk_shorthand));
	zassert_ok(meshtastic_channels_set_slot(0, &ch), "set_slot failed for \"%s\"", name);
}

/*
 * INTEROP (the real thing): decode an actual frame produced by stock Meshtastic.
 *
 * Captured on the bench from a Seeed T1000-E running stock Meshtastic 2.7.24,
 * broadcasting "golden-interop-1" on the default ShortTurbo channel (raw on-air
 * bytes logged by CONFIG_MESHTASTIC_PACKET_HEXDUMP; see
 * docs/parity/INTEROP-TEST-MATRIX.md). These bytes were NOT produced by the port
 * — so a decode failure here means we cannot understand stock's wire format. This
 * is the strongest interop guard in the suite: 16-byte upstream PacketHeader +
 * AES-CTR-encrypted Data, decrypted with the default channel key.
 */
ZTEST(mesh_sim, test_decodes_real_stock_frame)
{
	/* !68bfd58e -> broadcast, id 0xac747aaa, channel hash 0x0e, "golden-interop-1" */
	static const uint8_t stock_frame[] = {
		0xff, 0xff, 0xff, 0xff, 0x8e, 0xd5, 0xbf, 0x68,
		0xaa, 0x7a, 0x74, 0xac, 0xc6, 0x0e, 0x00, 0x8e,
		0xa4, 0xb5, 0xdc, 0x2b, 0xce, 0x8f, 0xbf, 0x77,
		0xdc, 0x1d, 0xd6, 0x58, 0xcc, 0x84, 0xd0, 0xb4,
		0x93, 0xfa, 0x36, 0xc1, 0xde, 0xf1,
	};
	const char *expect = "golden-interop-1";

	/* Match stock's default ShortTurbo channel so we can decrypt it. */
	set_primary_default("ShortTurbo");
	zassert_equal(meshtastic_channels_primary_hash(), 0x0eU,
		      "sim primary channel hash must equal the stock frame's 0x0e byte");

	zassert_ok(lora_sim_inject(lora_dev, stock_frame, sizeof(stock_frame), -16, 13),
		   "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)),
		   "stock frame not decoded/delivered — wire-format incompatibility");
	zassert_equal(rx.from, 0x68bfd58eU, "wrong stock sender");
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(rx.payload_len, strlen(expect), "wrong decoded text length");
	zassert_mem_equal(rx.payload, expect, strlen(expect), "decoded text mismatch");
}

/* --- preset orthogonality: the sim must model radios being deaf to each other --
 *
 * Everything in the multi-preset arc (docs/MULTI-PRESET-OPERATION.md) rests on one
 * physical fact: a preset change moves BOTH the modem settings and the frequency,
 * so a node on LongFast cannot hear a ShortTurbo node at all. Until the sim models
 * that, a test cannot tell a correct preset switch from one that retuned the modem
 * and forgot the frequency — which on hardware is silent until a second node fails
 * to hear you.
 */

/* Frequencies/params are the resolved US PROFILE_STD values; see docs/preset_math.py. */
#define LONGFAST_HZ    906875000U
#define SHORTTURBO_HZ  926750000U

ZTEST(mesh_sim, test_sim_models_preset_orthogonality)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t freq = 0U;
	uint8_t sf = 0U, bw = 0U;
	int ret;

	build_peer_text(0x3003U, "orthogonality", wire, &wire_len);

	/* Whatever the stack tuned to at init, read it back — the test asserts
	 * against the node's ACTUAL tuning rather than assuming a preset. */
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw),
		   "the radio must be configured after init");

	/* Same tuning as the receiver: delivered. */
	ret = lora_sim_inject_on(lora_dev, freq, sf, bw, wire, (uint8_t)wire_len, -50, 7);
	zassert_ok(ret, "a frame on our own tuning must be received (got %d)", ret);

	/* Right modem settings, wrong frequency — the exact failure mode of a switch
	 * that updates SF/BW but forgets meshtastic_region_freq_plan(). */
	ret = lora_sim_inject_on(lora_dev, freq + 1000000U, sf, bw, wire, (uint8_t)wire_len,
				 -50, 7);
	zassert_equal(ret, -ENOTCONN, "a frame on another frequency must NOT be heard");

	/* Right frequency, wrong spreading factor — orthogonal in the other axis. */
	ret = lora_sim_inject_on(lora_dev, freq, (uint8_t)(sf + 1U), bw, wire,
				 (uint8_t)wire_len, -50, 7);
	zassert_equal(ret, -ENOTCONN, "a frame at another SF must NOT be heard");

	/* Right frequency and SF, wrong bandwidth. */
	ret = lora_sim_inject_on(lora_dev, freq, sf, (uint8_t)(bw + 1U), wire,
				 (uint8_t)wire_len, -50, 7);
	zassert_equal(ret, -ENOTCONN, "a frame at another bandwidth must NOT be heard");

	/* And the radio-agnostic form still delivers regardless — the existing tests
	 * depend on that, and it must not have changed. */
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7),
		   "the tuning-agnostic inject must still always deliver");
}

/* LongFast and ShortTurbo are ~20 MHz and 4 spreading factors apart on a default
 * channel. Whatever this node is tuned to, a radio on the OTHER preset's settings
 * must be inaudible — and each axis (frequency, SF) must be sufficient on its own.
 *
 * Deliberately relative rather than absolute: this fixture's meshtastic_init()
 * takes an explicit .frequency, whereas a real boot derives it from the region
 * plan (meshtastic_config_store.c:713) the same way meshtastic_preset_switch()
 * does. Hardcoding a frequency here would pin a fixture artifact rather than the
 * behaviour under test.
 */
ZTEST(mesh_sim, test_sim_longfast_and_shortturbo_are_mutually_deaf)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t freq = 0U;
	uint8_t sf = 0U, bw = 0U;

	build_peer_text(0x3004U, "deaf", wire, &wire_len);
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "radio must be configured");

	/* Our own tuning: heard. */
	zassert_ok(lora_sim_inject_on(lora_dev, freq, sf, bw, wire, (uint8_t)wire_len, -50, 7),
		   "a frame on our own tuning must be received");

	/* Both LongFast and ShortTurbo canonical frequencies differ from ours in at
	 * least one axis, and each mismatch alone is disqualifying. */
	zassert_equal(lora_sim_inject_on(lora_dev, SHORTTURBO_HZ, 7U, bw, wire,
					 (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "ShortTurbo params must be inaudible");
	zassert_equal(lora_sim_inject_on(lora_dev, LONGFAST_HZ, 7U, bw, wire,
					 (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "SF7 must be inaudible whatever the frequency");

	/* Frequency axis alone: our exact SF and bandwidth, one slot away. */
	zassert_equal(lora_sim_inject_on(lora_dev, freq + 250000U, sf, bw, wire,
					 (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "one slot away must be inaudible even at our own SF");

	/* SF axis alone: our exact frequency and bandwidth, one SF away. */
	zassert_equal(lora_sim_inject_on(lora_dev, freq, (uint8_t)(sf + 1U), bw, wire,
					 (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "one SF away must be inaudible even on our own frequency");
}

/* --- the preset switch primitive -------------------------------------------
 *
 * A preset change has to move FOUR things together: modem params, every channel
 * hash, the frequency slot, and the radio itself. The silent failure mode is a
 * PARTIAL switch — new SF/BW left on the old frequency, say — which looks
 * perfectly healthy locally and is simply inaudible to everyone else.
 *
 * ⚠️ This node's primary channel carries a LITERAL name ("LongFast", see
 * mesh_sim_setup), not an empty one. That matters more than it looks:
 *
 *   empty name  -> resolves to the PRESET's display name, so the djb2 frequency
 *                  slot moves with the preset (LongFast 906.875, MediumFast
 *                  913.125, ShortTurbo 926.750).
 *   literal name-> the slot is djb2("LongFast") for every preset, so the
 *                  frequency only moves when the SLOT WIDTH changes, i.e. when
 *                  the bandwidth changes. LongFast/MediumFast/MediumSlow are all
 *                  BW250k and therefore share ONE frequency, differing only by
 *                  spreading factor. ShortTurbo (BW500k) does move.
 *
 * So these tests assert on INVARIANTS (the radio matches what the switch
 * reported; deaf to the old tuning; hearing the new) rather than on hardcoded
 * frequencies, and the named-channel consequence gets its own test below.
 */

/* The core contract: after a switch, everything moved together and the radio
 * agrees with the bookkeeping. */
ZTEST(mesh_sim, test_preset_switch_moves_everything_together)
{
	struct meshtastic_preset_result res = {0};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t old_freq = 0U, new_freq = 0U;
	uint8_t old_sf = 0U, old_bw = 0U, new_sf = 0U, new_bw = 0U;
	uint8_t old_hash;

	/* Start from a known preset rather than trusting suite order. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "baseline switch failed");
	zassert_ok(lora_sim_get_tuning(lora_dev, &old_freq, &old_sf, &old_bw), "radio unconfigured");
	old_hash = mt.ch_hash;
	zassert_equal(old_sf, 11U, "LongFast is SF11, got SF%u", old_sf);

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
					    &res), "preset switch failed");

	/* (a) the RADIO got exactly what the switch says it resolved — not just our
	 * own bookkeeping, which is the half-switch failure mode. */
	zassert_ok(lora_sim_get_tuning(lora_dev, &new_freq, &new_sf, &new_bw), "radio unconfigured");
	zassert_equal(new_freq, res.frequency_hz, "the radio must carry the resolved frequency");
	zassert_equal(new_sf, res.spread_factor, "the radio must carry the resolved SF");
	zassert_equal(res.spread_factor, 7U, "ShortTurbo is SF7, got SF%u", res.spread_factor);
	zassert_equal(res.bandwidth_hz, 500000U, "ShortTurbo is BW500k, got %u", res.bandwidth_hz);

	/* (b) the frequency moved: ShortTurbo's 500k bandwidth halves the slot count,
	 * so even a literally-named channel lands somewhere new. */
	zassert_not_equal(new_freq, old_freq,
			  "a bandwidth change must move the frequency slot (both %u)", new_freq);

	/* (c) the channel hash followed the preset. */
	zassert_equal(mt.ch_hash, res.channel_hash, "the reported hash must match reality");

	/* (d) it is real on the air: deaf to where we were, hearing where we are. */
	build_peer_text(0x4004U, "after switch", wire, &wire_len);
	zassert_equal(lora_sim_inject_on(lora_dev, old_freq, old_sf, old_bw, wire,
					 (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "must be DEAF to the preset we left");
	zassert_ok(lora_sim_inject_on(lora_dev, new_freq, new_sf, new_bw, wire,
				      (uint8_t)wire_len, -50, 7),
		   "must HEAR the preset we moved to");

	/* Restore, so suite order stays independent. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "switch back failed");
	zassert_ok(lora_sim_get_tuning(lora_dev, &new_freq, &new_sf, &new_bw), "radio unconfigured");
	zassert_equal(new_freq, old_freq, "switching back must restore the frequency");
	zassert_equal(mt.ch_hash, old_hash, "switching back must restore the channel hash");
}

/* ⚠️ The named-channel consequence, pinned because it is genuinely surprising and
 * it shapes the LongFast<->MediumFast time-slicing design: on a channel with a
 * literal name these two presets share ONE frequency and differ only by spreading
 * factor. They remain mutually deaf — SF alone is enough — but a switch between
 * them never retunes the radio. */
ZTEST(mesh_sim, test_named_channel_keeps_frequency_across_same_bandwidth_presets)
{
	struct meshtastic_preset_result lf = {0}, mf = {0};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    &lf), "LongFast switch failed");
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
					    &mf), "MediumFast switch failed");

	zassert_equal(lf.bandwidth_hz, mf.bandwidth_hz, "both presets are BW250k");
	zassert_equal(lf.frequency_hz, mf.frequency_hz,
		      "a LITERALLY-named channel keeps one frequency across equal-bandwidth "
		      "presets (LongFast %u vs MediumFast %u)", lf.frequency_hz, mf.frequency_hz);
	zassert_not_equal(lf.spread_factor, mf.spread_factor,
			  "...but the spreading factor must still differ (SF%u)", lf.spread_factor);

	/* Same frequency, different SF — still completely deaf. SF alone carries the
	 * orthogonality here, so a switch that only retuned frequency would be wrong. */
	build_peer_text(0x4005U, "same freq", wire, &wire_len);
	zassert_equal(lora_sim_inject_on(lora_dev, lf.frequency_hz, lf.spread_factor,
					 (uint8_t)0, wire, (uint8_t)wire_len, -50, 7),
		      -ENOTCONN, "LongFast must be inaudible from MediumFast despite equal freq");

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "restore failed");
}

/* A round trip must land exactly where it started — every derived value, not just
 * the frequency. Drift here would mean the derivation is order-dependent. */
ZTEST(mesh_sim, test_preset_switch_round_trip_is_exact)
{
	struct meshtastic_preset_result a = {0}, b = {0};

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, &a),
		   "baseline switch failed");
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
					    NULL), "switch away failed");
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, &b),
		   "switch back failed");

	zassert_equal(a.frequency_hz, b.frequency_hz, "frequency must round-trip exactly");
	zassert_equal(a.spread_factor, b.spread_factor, "SF must round-trip exactly");
	zassert_equal(a.bandwidth_hz, b.bandwidth_hz, "bandwidth must round-trip exactly");
	zassert_equal(a.channel_hash, b.channel_hash, "channel hash must round-trip exactly");
}

/* An unknown preset must change NOTHING.
 *
 * This one nearly shipped broken: meshtastic_preset_to_params() returns 0 for an
 * unknown preset, folding it into the LongFast default row to mirror the
 * reference's `default:` branch. Right for decoding a peer's config; very wrong
 * as a switch, where it would silently move the node to LongFast — a
 * fleet-partitioning event. The switch gates on the display name instead. */
ZTEST(mesh_sim, test_preset_switch_rejects_unknown_atomically)
{
	uint32_t freq_before = 0U, freq_after = 0U;
	uint8_t sf_before = 0U, bw_before = 0U, sf_after = 0U, bw_after = 0U;
	uint8_t hash_before;

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "baseline switch failed");
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq_before, &sf_before, &bw_before),
		   "radio unconfigured");
	hash_before = mt.ch_hash;

	zassert_equal(meshtastic_preset_switch((meshtastic_Config_LoRaConfig_ModemPreset)999, NULL),
		      -EINVAL, "an unknown preset must be refused with -EINVAL");

	zassert_ok(lora_sim_get_tuning(lora_dev, &freq_after, &sf_after, &bw_after),
		   "radio unconfigured");
	zassert_equal(freq_after, freq_before, "a refused switch must not move the frequency");
	zassert_equal(sf_after, sf_before, "a refused switch must not move the SF");
	zassert_equal(bw_after, bw_before, "a refused switch must not move the bandwidth");
	zassert_equal(mt.ch_hash, hash_before, "a refused switch must not move the channel hash");
}

/* The scanner's tuning entry point. Deliberately separate from
 * meshtastic_preset_switch(): that resolves frequency through the primary
 * channel's name, so on a NAMED channel it would keep dragging a scanner back to
 * that name's slot instead of visiting where a foreign mesh actually is
 * (docs/MULTI-PRESET-OPERATION.md §3.1a). */
ZTEST(mesh_sim, test_tune_explicit_ignores_channel_name_derivation)
{
	struct meshtastic_preset_result via_preset = {0};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t freq = 0U;
	uint8_t sf = 0U, bw = 0U;

	/* Where the preset path puts us — through this node's named channel. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    &via_preset), "baseline switch failed");

	/* Now tune somewhere the name derivation would never choose: the CANONICAL
	 * empty-name LongFast frequency, which differs from ours precisely because
	 * our channel carries a literal name. */
	zassert_ok(meshtastic_radio_tune_explicit(LONGFAST_HZ, 11U, 250000U, 5U),
		   "explicit tune failed");
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "radio unconfigured");
	zassert_equal(freq, LONGFAST_HZ, "explicit tune must land exactly where asked");
	zassert_equal(sf, 11U, "explicit tune must set the requested SF");

	/* And it really hears there — this is the scanner's whole job. */
	build_peer_text(0x5005U, "canonical", wire, &wire_len);
	zassert_ok(lora_sim_inject_on(lora_dev, LONGFAST_HZ, 11U, bw, wire,
				      (uint8_t)wire_len, -50, 7),
		   "must hear the frequency we explicitly tuned to");

	/* Reject parameters the driver cannot represent, rather than half-applying. */
	zassert_equal(meshtastic_radio_tune_explicit(LONGFAST_HZ, 11U, 999U, 5U), -EINVAL,
		      "an unrepresentable bandwidth must be refused");
	zassert_equal(meshtastic_radio_tune_explicit(LONGFAST_HZ, 11U, 250000U, 99U), -EINVAL,
		      "an unrepresentable coding rate must be refused");
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "radio unconfigured");
	zassert_equal(freq, LONGFAST_HZ, "a refused tune must not move the radio");

	/* Resuming normal operation goes back through the preset path, which
	 * re-derives everything — tuning "back" by hand would leave mt.modem_preset
	 * disagreeing with the radio. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "restore failed");
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "radio unconfigured");
	zassert_equal(freq, via_preset.frequency_hz,
		      "the preset path must restore its own derived frequency");
}

#if defined(CONFIG_MESHTASTIC_SCANNER)
/* --- participant <-> scan toggle -------------------------------------------
 *
 * The safety property the whole scanner design rests on: while sweeping, the
 * radio is parked on a frequency this node was never configured for, carrying a
 * channel hash it did not derive. A transmission there is not a local bug — it
 * is interference on someone else's channel. So TX must be refused for the WHOLE
 * time the radio is away, including the restore window.
 *
 * This is testable precisely because the gate counts refusals instead of being
 * compiled out: "nothing transmitted while scanning" is an assertion here rather
 * than an assumption.
 */
ZTEST(mesh_sim, test_scan_mode_refuses_tx_and_restores)
{
	uint32_t freq_before = 0U, freq_after = 0U;
	uint8_t sf_before = 0U, bw_before = 0U, sf_after = 0U, bw_after = 0U;
	uint8_t hash_before;
	struct lora_sim_frame f;

	/* Baseline: a normal participant, transmitting happily. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "baseline switch failed");
	zassert_false(meshtastic_scanner_active(), "must start as a participant");
	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "before"),
		   "a participant must be able to transmit");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)), "no TX captured before scanning");

	zassert_ok(lora_sim_get_tuning(lora_dev, &freq_before, &sf_before, &bw_before),
		   "radio unconfigured");
	hash_before = mt.ch_hash;
	meshtastic_scanner_reset();

	/* Enter scan mode. */
	zassert_ok(meshtastic_scanner_start(), "scan start failed");
	zassert_true(meshtastic_scanner_active(), "TX must be gated while scanning");
	zassert_equal(meshtastic_scanner_start(), -EALREADY, "starting twice must be refused");

	/* THE assertion. A send now must be refused, and counted — the count is what
	 * makes a stray transmit attempt visible instead of silent. */
	zassert_not_equal(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "during"), 0,
			  "transmitting while scanning must FAIL");
	zassert_true(meshtastic_scanner_tx_blocked() > 0U,
		     "a refused transmit must be counted, not silently dropped");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(200)), 0,
			  "nothing may reach the radio while scanning");

	/* Leave scan mode. */
	zassert_ok(meshtastic_scanner_stop(), "scan stop failed");
	zassert_false(meshtastic_scanner_active(), "TX must be re-enabled after stopping");
	zassert_equal(meshtastic_scanner_stop(), -EALREADY, "stopping twice must be refused");

	/* The node is genuinely back where it was — not merely un-gated. Restoring
	 * the gate without restoring the radio would let it transmit on a scan
	 * frequency, which is the failure this ordering exists to prevent. */
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq_after, &sf_after, &bw_after),
		   "radio unconfigured");
	zassert_equal(freq_after, freq_before, "stop must restore the operating frequency");
	zassert_equal(sf_after, sf_before, "stop must restore the operating SF");
	zassert_equal(bw_after, bw_before, "stop must restore the operating bandwidth");
	zassert_equal(mt.ch_hash, hash_before, "stop must restore the channel hash");

	/* And it participates again. */
	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "after"),
		   "must be able to transmit again after stopping");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)), "no TX captured after stopping");
}

/*
 * stop() must not return until the sweep thread has PARKED.
 *
 * The check above ("stop must restore the operating frequency") passes either
 * way, because it only samples the radio once — immediately, before a sweep
 * thread that was still running could have retuned back on top of the restore.
 * The ordering is what actually carries the safety property: stop() clears
 * tx_closed as soon as the restore succeeds, so if the sweep is still live at
 * that moment it can tune to the next scan preset with TX already re-enabled,
 * and the node transmits on a foreign mesh's frequency.
 *
 * Found 2026-08-23 by review, and it was real: scan_idle is given on every park
 * INCLUDING the one at boot, and nothing consumed that first token, so the
 * semaphore sat permanently one ahead and every stop() took the stale token and
 * returned in 0 ms without waiting for anything. Instrumented on native_sim,
 * all three stop() calls in this suite showed count=1 / waited 0 ms. start()
 * now drains it, so the only token stop() can take is the one from the park it
 * caused.
 *
 * Asserted on the park COUNT rather than on elapsed time: the invariant is
 * "the thread parked before stop() returned", which is exactly what the counter
 * records, and it does not depend on scheduler timing to be meaningful.
 */
ZTEST(mesh_sim, test_scan_stop_waits_for_the_sweep_thread_to_park)
{
	uint32_t parks_before;

	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "baseline switch failed");

	parks_before = meshtastic_scanner_parks();
	zassert_ok(meshtastic_scanner_start(), "scan start failed");

	/* Let the sweep get properly under way, so stop() is interrupting a real
	 * dwell rather than racing the thread's very first pass. */
	k_sleep(K_MSEC(200));
	zassert_true(meshtastic_scanner_sweeping(), "the sweep should be running");
	zassert_equal(meshtastic_scanner_parks(), parks_before,
		      "a running sweep must not have parked");

	zassert_ok(meshtastic_scanner_stop(), "scan stop failed");

	/* No k_sleep here, deliberately: the point is that stop() ITSELF waited. */
	zassert_equal(meshtastic_scanner_parks(), parks_before + 1U,
		      "stop() returned before the sweep thread parked — it can still "
		      "retune onto a scan preset now that TX has been re-enabled");
}

/* The survey captures headers from frames the normal stack would discard — a
 * foreign mesh's channel hash, which we hold no key for. That is the whole point:
 * the header is plaintext, so no keys and no channel setup are needed. */
ZTEST(mesh_sim, test_scan_captures_foreign_headers)
{
	struct meshtastic_scan_record rec[4];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t freq = 0U;
	uint8_t sf = 0U, bw = 0U;
	int n;

	meshtastic_scanner_reset();
	zassert_ok(meshtastic_scanner_start(), "scan start failed");

	/* Let the sweep tune somewhere, then inject on wherever it actually landed. */
	k_sleep(K_MSEC(200));
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "sweep did not tune");

	build_peer_text(0x6006U, "foreign", wire, &wire_len);
	/* Stamp a channel hash we hold no key for — the frame is undecryptable to us
	 * and the normal stack would drop it. The survey must still record it. */
	wire[13] = 0xA5U;

	zassert_ok(lora_sim_inject_on(lora_dev, freq, sf, bw, wire, (uint8_t)wire_len, -77, 3),
		   "inject on the swept tuning failed");

	zassert_true(meshtastic_scanner_total() > 0U, "an undecryptable frame must still be logged");

	n = meshtastic_scanner_records(rec, ARRAY_SIZE(rec), 0U);
	zassert_true(n > 0, "records must be readable");
	zassert_equal(rec[0].chan_hash, 0xA5U, "the SENDER's channel hash must be recorded");
	zassert_equal(rec[0].rssi, -77, "rssi must be recorded");
	zassert_equal(rec[0].snr, 3, "snr must be recorded");
	zassert_equal(rec[0].payload_len, (uint8_t)(wire_len - MESHTASTIC_HDR_LEN),
		      "payload LENGTH is recorded (contents deliberately are not)");

	zassert_ok(meshtastic_scanner_stop(), "scan stop failed");
}

/* A scanning node is an observer, not a participant: a surveyed frame must NOT
 * continue into the normal stack.
 *
 * Regression test for a defect found on rzr3 (2026-08-19). The nodedb module
 * takes ALL_PACKETS, so letting foreign frames through meant a 10-minute sweep
 * learned 43 strangers and generated NodeInfo requests for them — refused by the
 * TX gate, but they should never have been produced at all. tx_blocked is the
 * assertion that matters: it must stay ZERO across a sweep in which frames were
 * heard, because nothing should even try to respond. */
ZTEST(mesh_sim, test_scan_does_not_feed_the_participant_stack)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint32_t freq = 0U;
	uint8_t sf = 0U, bw = 0U;

	meshtastic_scanner_reset();
	k_sem_reset(&rx.sem);

	zassert_ok(meshtastic_scanner_start(), "scan start failed");
	k_sleep(K_MSEC(200));
	zassert_ok(lora_sim_get_tuning(lora_dev, &freq, &sf, &bw), "sweep did not tune");

	/* A perfectly decodable frame on OUR key — the strongest case, because a
	 * foreign undecryptable one would be dropped later anyway. If even this does
	 * not reach the app, nothing does. */
	build_peer_text(0x7007U, "not for us", wire, &wire_len);
	zassert_ok(lora_sim_inject_on(lora_dev, freq, sf, bw, wire, (uint8_t)wire_len, -50, 7),
		   "inject failed");

	/* Surveyed... */
	zassert_true(meshtastic_scanner_total() > 0U, "the survey must still record it");
	zassert_true(meshtastic_scanner_rx_dropped() > 0U, "the frame must be counted as withheld");

	/* ...but never delivered to the application. */
	zassert_not_equal(k_sem_take(&rx.sem, K_MSEC(300)), 0,
			  "a frame heard while scanning must NOT reach the app");

	/* And nothing tried to answer it. This is the assertion the rzr3 run failed. */
	zassert_equal(meshtastic_scanner_tx_blocked(), 0U,
		      "nothing should attempt to transmit in response to a surveyed frame");

	zassert_ok(meshtastic_scanner_stop(), "scan stop failed");

	/* Back to participating: the same frame now does reach the app. */
	build_peer_text(0x7008U, "for us", wire, &wire_len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)),
		   "after stopping, frames must reach the app again");
}
#endif /* CONFIG_MESHTASTIC_SCANNER */

ZTEST_SUITE(mesh_sim, NULL, mesh_sim_setup, mesh_sim_before, NULL, NULL);

/* ==========================================================================
 * Multi-hop relay (INTEROP-TEST-MATRIX 2.9)
 *
 * The on-air version of this row wants three radios with A and B out of each
 * other's range and C between them. That is not achievable on this bench and
 * cannot be faked with transmit power: ShortTurbo (SF7/BW500) decodes to about
 * -117 dBm, the measured bench link sits at -5 dBm, and the whole tx_power
 * range is ~16 dB against a ~112 dB margin. Range separation needs distance or
 * an attenuator, not firmware.
 *
 * What IS testable here — and is the part that can regress silently — is this
 * node's own behaviour at each position in the chain. The sim gives exact
 * control over what it hears and exact capture of what it says, so each role is
 * checked against the frame it must produce:
 *
 *   as C (the middle hop)  — rebroadcast with hop_limit-1, hop_start intact,
 *                            relay_node stamped as us, everything else verbatim
 *   as B (two hops away)   — deliver, and record the sender as 2 hops away
 *   at the edge of the flood — refuse to relay a spent frame
 *
 * Together those are what makes a real A->C->B delivery work; the on-air row
 * stays UNVERIFIED until someone separates two nodes physically.
 */

/* Build a frame as some ORIGIN node, with explicit hop counts and relayer, so a
 * frame that has already travelled can be handed to the node under test. */
static void build_relayed_text(uint32_t origin, uint32_t id, const char *text, uint8_t hop_limit,
			       uint8_t hop_start, uint8_t relay_node, uint8_t *wire,
			       uint32_t *wire_len)
{
	struct meshtastic_wire_header *hdr;
	struct meshtastic_packet packet = {
		.from          = origin,
		.to            = MESHTASTIC_NODE_BROADCAST,
		.id            = id,
		.portnum       = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload       = (const uint8_t *)text,
		.payload_len   = strlen(text),
		.hop_limit     = hop_limit,
		.hop_start     = hop_start,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len),
		   "build_wire_packet failed");

	/* The encoder has no relay_node input — it is stamped by whoever transmits.
	 * Patch it in so the frame looks like C's rebroadcast rather than A's
	 * original. */
	hdr = (struct meshtastic_wire_header *)wire;
	hdr->relay_node = relay_node;
}

/*
 * True if a rebroadcast of exactly this frame is transmitted within @window.
 *
 * "Did we transmit anything at all" is the wrong question: hearing an unknown
 * node legitimately makes the stack emit a NodeInfo request, so a bare
 * take_tx() would catch that and call it a relay. A relay is identified by
 * carrying the ORIGINATOR's src and the original packet id — that is precisely
 * what makes it a relay rather than a new packet of our own.
 */
static bool saw_relay_of(uint32_t origin, uint32_t id, k_timeout_t window)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, window) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		if (f.len >= MESHTASTIC_HDR_LEN && sys_le32_to_cpu(h->src) == origin &&
		    sys_le32_to_cpu(h->id) == id) {
			return true;
		}
		window = K_MSEC(100); /* keep draining anything else briefly */
	}

	return false;
}

/*
 * Position C: we are the middle hop. A's broadcast must go back out with
 * exactly one hop spent, our own stamp on it, and nothing else disturbed —
 * the id in particular, since B's duplicate suppression keys on (src, id) and
 * a rewritten id would let a flood circulate forever.
 */
ZTEST(mesh_sim, test_relay_forwards_with_one_hop_spent_and_our_stamp)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct lora_sim_frame f;
	const struct meshtastic_wire_header *out;
	const uint32_t origin = 0x0C0C0C0CU;
	const uint32_t id = 0x5150U;
	uint8_t out_hop_limit, out_hop_start;

	build_relayed_text(origin, id, "two hops", 3U, 3U, 0xEEU, wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -70, 6), "inject failed");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(2000)), "frame was not relayed");

	out = (const struct meshtastic_wire_header *)f.data;
	out_hop_limit = out->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK;
	out_hop_start = (out->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
			MESHTASTIC_FLAGS_HOP_START_SHIFT;

	zassert_equal(2U, out_hop_limit, "hop_limit must spend exactly one hop, got %u",
		      out_hop_limit);
	zassert_equal(3U, out_hop_start,
		      "hop_start must survive the relay — it is what lets the far end compute "
		      "distance (got %u)",
		      out_hop_start);
	zassert_equal((uint8_t)(TEST_NODE_ID & 0xFFU), out->relay_node,
		      "we must stamp ourselves as the relayer, got 0x%02x", out->relay_node);
	zassert_equal(origin, sys_le32_to_cpu(out->src),
		      "the originator must be preserved, not replaced by us");
	zassert_equal(id, sys_le32_to_cpu(out->id),
		      "the packet id must be preserved or downstream dedup breaks");
	zassert_equal(wire_len, f.len, "relayed frame changed length");
	zassert_mem_equal(wire + MESHTASTIC_HDR_LEN, f.data + MESHTASTIC_HDR_LEN,
			  wire_len - MESHTASTIC_HDR_LEN, "relayed payload was modified");
}

/*
 * The edge of the flood. A frame arriving with no hops left is the last copy
 * that should ever exist; relaying it is what turns a mesh into a broadcast
 * storm. Delivery to the app still has to happen — being un-relayable says
 * nothing about being un-readable.
 */
ZTEST(mesh_sim, test_spent_frame_is_delivered_but_not_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	build_relayed_text(0x0C0C0C0DU, 0x5151U, "last hop", 0U, 3U, 0xEEU, wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -70, 6), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)),
		   "a spent frame must still reach the application");
	zassert_false(saw_relay_of(0x0C0C0C0DU, 0x5151U, K_MSEC(400)),
		      "a frame with hop_limit 0 was relayed — floods would not terminate");
}

/*
 * Position B: the far end of an A->C->B chain. A started with hop_start 3 and
 * the frame arrives with 1 left, so A is two hops away. That figure is what a
 * client shows as distance and what routing decisions lean on, and it is
 * derived purely from the two counters — which is why the relay above must not
 * touch hop_start.
 */
ZTEST(mesh_sim, test_two_hop_sender_is_recorded_two_hops_away)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_nodedb_node node;
	const uint32_t origin = 0x0C0C0C0EU;

	build_relayed_text(origin, 0x5152U, "far end", 1U, 3U, 0xEEU, wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -70, 6), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "two-hop frame not delivered");
	k_sleep(K_MSEC(50)); /* module hooks run on the mesh thread, after the app callback */

	zassert_ok(meshtastic_nodedb_get(origin, &node), "two-hop sender was not learned");
	zassert_true(node.has_hops_away, "distance to a two-hop sender was not recorded");
	zassert_equal(2U, node.hops_away, "expected 2 hops away, got %u", node.hops_away);
}

/*
 * A direct neighbour must still read as zero hops away, or "2 hops" above would
 * be meaningless — this pins the other end of the same derivation.
 */
ZTEST(mesh_sim, test_direct_sender_is_recorded_zero_hops_away)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_nodedb_node node;
	const uint32_t origin = 0x0C0C0C0FU;

	build_relayed_text(origin, 0x5153U, "next door", 3U, 3U, 0x00U, wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -70, 6), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "direct frame not delivered");
	k_sleep(K_MSEC(50)); /* module hooks run on the mesh thread, after the app callback */

	zassert_ok(meshtastic_nodedb_get(origin, &node), "direct sender was not learned");
	zassert_true(node.has_hops_away, "distance to a direct sender was not recorded");
	zassert_equal(0U, node.hops_away, "expected 0 hops away, got %u", node.hops_away);
}

/* ==========================================================================
 * config.lora.tx_enabled — the "receive only" switch every config tool offers
 *
 * The reference honours this; this port stored it and transmitted anyway, so a
 * phone, web client or python CLI could set it, read it back set, and still
 * have a node on air. Enforced at the same single choke point as the scanner
 * gate, so it covers relay, NodeInfo, telemetry, retries and phone-injected
 * sends alike.
 */
ZTEST(mesh_sim, test_tx_enabled_false_makes_the_node_receive_only)
{
	meshtastic_Config cfg;
	struct lora_sim_frame f;

	/* Get-modify-set, the way a real client does it: every other field of the
	 * LoRaConfig has to survive, or this would be testing a wiped config. */
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	cfg.payload_variant.lora.tx_enabled = false;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config apply failed");

	zassert_not_equal(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "should not go"), 0,
			  "send was accepted while tx_enabled was false");
	zassert_not_equal(lora_sim_take_tx(lora_dev, &f, K_MSEC(400)), 0,
			  "a frame reached the radio while tx_enabled was false");

	/* Restore, and prove the gate is the reason rather than a broken stack. */
	cfg.payload_variant.lora.tx_enabled = true;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config restore failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config re-apply failed");

	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "should go"),
		   "send refused after tx_enabled was restored");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(2000)),
		   "nothing transmitted after tx_enabled was restored");
}

/*
 * config.lora.fem_lna_mode — normalization, not silent acceptance.
 *
 * native_sim has no front-end, so this board reports "cannot control". The
 * reference does not simply ignore the field there; it rewrites the stored
 * value to NOT_PRESENT so a tool reading its config back learns the setting
 * has no effect on this hardware, rather than seeing its own value echoed.
 */
ZTEST(mesh_sim, test_fem_lna_mode_normalizes_when_hardware_cannot_control_it)
{
	meshtastic_Config cfg;

	zassert_false(meshtastic_radio_fem_lna_can_control(),
		      "native_sim should report no controllable FEM LNA");

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	cfg.payload_variant.lora.fem_lna_mode = meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config apply failed");

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not re-read the lora config");
	zassert_equal(meshtastic_Config_LoRaConfig_FEM_LNA_Mode_NOT_PRESENT,
		      cfg.payload_variant.lora.fem_lna_mode,
		      "an unhonourable fem_lna_mode must be normalized, not echoed back");
}

/*
 * FEM_LNA_Mode_DISABLED is 0, so an unset fem_lna_mode reads as "bypass the
 * LNA". A node that has never been configured must not come up deliberately
 * deaf, which is the same trap sx126x_rx_boosted_gain documents one field
 * along. The seeded default has to describe the hardware, not the proto zero.
 */
ZTEST(mesh_sim, test_fem_lna_mode_default_is_never_the_proto_zero)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	zassert_not_equal(meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED,
			  cfg.payload_variant.lora.fem_lna_mode,
			  "a freshly seeded node must not default to bypassing its LNA");
}

/*
 * config.lora.fem_lna_mode -- the other half of the normalization test above.
 * On a board that CAN control its LNA, apply_core must actually DRIVE it per
 * the operator's setting, not merely accept the stored value. Mirrors the
 * reference (AdminModule.cpp: isLnaCanControl() -> femInterface lna
 * enable/disable).
 */
ZTEST(mesh_sim, test_fem_lna_mode_applied_when_hardware_can_control_it)
{
	meshtastic_Config cfg;

	fem_spy.lna_can_control = true;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");

	/* meshtastic_config_store_set_config() applies internally (it is what a
	 * real config write does), so each call below is one apply and must move
	 * the spy's count by exactly one -- not the explicit-apply_core() dance
	 * the tx_enabled/normalization tests above use, which would double-count
	 * here. */
	cfg.payload_variant.lora.fem_lna_mode = meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_equal(1U, fem_spy.lna_set_count, "fem_lna_set was not called for ENABLED");
	zassert_true(fem_spy.lna_last_set_value, "ENABLED must drive the LNA on");

	cfg.payload_variant.lora.fem_lna_mode = meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_equal(2U, fem_spy.lna_set_count, "fem_lna_set was not called for DISABLED");
	zassert_false(fem_spy.lna_last_set_value, "DISABLED must bypass the LNA");

	/* And unlike the cannot-control branch, a controllable LNA keeps the
	 * operator's own setting -- there is nothing to normalize away. */
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not re-read the lora config");
	zassert_equal(meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED,
		      cfg.payload_variant.lora.fem_lna_mode,
		      "a controllable LNA must keep the operator's setting, not normalize it");

	/* Restore: fem_spy resets before the next test (mesh_sim_before), but the
	 * config STORE is not test-scoped, so a later test reading fem_lna_mode
	 * (e.g. the default-is-never-DISABLED test above) must not see this
	 * test's DISABLED left behind. */
	cfg.payload_variant.lora.fem_lna_mode = meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config restore failed");
}

/* ==========================================================================
 * FEM TX/RX gating and the tx_power pipeline
 *
 * The two remaining pieces of the FEM contract that the resolution-only tests
 * in tests/protocol/src/txpower.c cannot reach: that the radio layer actually
 * calls the board hooks around a real transmit, and that the value those hooks
 * produce is what actually gets programmed into the transceiver -- not just
 * what the conversion functions return in isolation.
 */

/*
 * meshtastic_radio_fem_set_tx -- the hook a FEM-equipped board (e.g. Heltec
 * V4) uses to steer its PA/LNA mode pin. Reference: SX126xInterface's
 * setTransmitEnable(true/false), called immediately before keying the
 * transmitter and again once it has returned to receive
 * (LoRaFEMInterface::setTxModeEnable / setRxModeEnable). This proves the CALL
 * sequence around one send is exactly [true, false] -- a board with no FEM
 * sees the same calls land on its own no-op override.
 */
ZTEST(mesh_sim, test_fem_set_tx_gates_around_a_real_transmit)
{
	struct lora_sim_frame f;

	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "fem gated"),
		   "send_text failed");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)), "no TX captured");

	zassert_equal(2U, fem_spy.tx_call_count,
		      "expected exactly one TX-enable and one RX-enable call, got %u",
		      fem_spy.tx_call_count);
	zassert_true(fem_spy.tx_calls[0], "the first call must steer the front-end to TX");
	zassert_false(fem_spy.tx_calls[1], "the second call must return the front-end to RX");
}

/*
 * TX power, end to end. config.lora.tx_power is radiated dBm (reference
 * semantics); what must reach lora_config() is that figure resolved against
 * the region limit (meshtastic_tx_power_resolve) and THEN reduced by the
 * board's FEM gain (meshtastic_radio_fem_tx_power_conversion), clamped to the
 * radio's settable range. 13 dB is close to the reference's real KCT8103L
 * table at drive levels near 20 dBm (tests/protocol/src/txpower.c pins the
 * table itself); this test pins that the two stages actually compose, on the
 * live send path, rather than each independently returning the right number.
 */
ZTEST(mesh_sim, test_tx_power_pipeline_reaches_the_driver)
{
	meshtastic_Config cfg;
	struct lora_sim_frame f;
	int8_t programmed;

	fem_spy.tx_power_gain_db = 13;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	cfg.payload_variant.lora.tx_power = 20; /* radiated dBm; well under the US limit */
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config apply failed");

	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "power pipeline"),
		   "send_text failed");
	zassert_ok(lora_sim_take_tx(lora_dev, &f, K_MSEC(1000)), "no TX captured");

	zassert_ok(lora_sim_get_tx_power(lora_dev, &programmed), "no tx_power recorded");
	zassert_equal(7, programmed,
		      "20 dBm radiated through a 13 dB FEM must program drive 7, got %d",
		      programmed);

	/* Restore, so a later test in this file sees the default region-limit
	 * tx_power rather than this one's 20 dBm. */
	cfg.payload_variant.lora.tx_power = 0;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config restore failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config re-apply failed");
}

/*
 * The same pipeline, reached from the OTHER caller of apply_modem_params():
 * meshtastic_radio_retune(), which every preset switch runs. A preset switch
 * has nothing to do with tx_power -- it changes SF/BW/frequency -- but
 * meshtastic_radio.c programs tx_power on every lora_config() call, retune
 * included, so a bug that only reprogrammed it on the SEND path (as the test
 * above alone would miss) would still leave a stale drive level on the radio
 * after every plain preset change.
 */
ZTEST(mesh_sim, test_tx_power_pipeline_survives_a_retune)
{
	meshtastic_Config cfg;
	int8_t programmed;

	fem_spy.tx_power_gain_db = 13;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	cfg.payload_variant.lora.tx_power = 20; /* radiated dBm; well under the US limit */
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");

	/* Start from a known preset rather than trusting suite order, same as the
	 * preset-switch tests above. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "baseline switch failed");
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
					    NULL), "preset switch failed");

	zassert_ok(lora_sim_get_tx_power(lora_dev, &programmed), "no tx_power recorded");
	zassert_equal(7, programmed,
		      "a retune must still program the FEM-converted drive level, got %d",
		      programmed);

	/* Restore. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					    NULL), "preset restore failed");
	cfg.payload_variant.lora.tx_power = 0;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config restore failed");
}

/*
 * config.lora.sx126x_rx_boosted_gain -- the RX sensitivity boost every config
 * tool can toggle. meshtastic_radio_init() pushes mt.rx_boosted_gain to the
 * driver once at boot (G-2, see the seed-default comment in
 * meshtastic_config_store.c); this pins the half of the story that changes on
 * every config apply after that -- mt.rx_boosted_gain must track the stored
 * config rather than freeze at whatever the seed default was.
 */
ZTEST(mesh_sim, test_rx_boosted_gain_tracks_config)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "could not read the lora config");
	zassert_true(mt.rx_boosted_gain,
		     "a freshly seeded node must default to RX gain boosted (G-2)");

	cfg.payload_variant.lora.sx126x_rx_boosted_gain = false;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config apply failed");
	zassert_false(mt.rx_boosted_gain, "apply_core did not clear rx_boosted_gain");

	cfg.payload_variant.lora.sx126x_rx_boosted_gain = true;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "lora config write failed");
	zassert_ok(meshtastic_config_store_apply_core(), "config apply failed");
	zassert_true(mt.rx_boosted_gain, "apply_core did not restore rx_boosted_gain");
}

#if defined(CONFIG_MESHTASTIC_RF_HIST)
/* --- signal measurement ----------------------------------------------------
 *
 * The bin edges and the direct/relayed split are the two things here that would
 * drift silently if someone "tidied" the arithmetic, so they get exact
 * assertions rather than range checks.
 */

/* flags byte sits after dest(4) + src(4) + id(4). */
#define WIRE_FLAGS_OFFSET 12U

ZTEST(mesh_sim, test_rf_rssi_bin_edges_including_the_rail)
{
	/* The rail is the whole reason this bin exists: RssiPkt is an unsigned
	 * -dBm/2 byte, so 0 is the TOP of the scale, not a missing reading, and a
	 * bench node a metre away pins there. Folding it into the top ordinary bin
	 * would hide the one condition that invalidates a comparison outright. */
	zassert_equal(meshtastic_rf_rssi_bin(0), MESHTASTIC_RF_RSSI_RAIL_BIN,
		      "RSSI 0 is the rail, not an ordinary top-bin reading");
	zassert_equal(meshtastic_rf_rssi_bin(5), MESHTASTIC_RF_RSSI_RAIL_BIN,
		      "anything above 0 is also railed");
	zassert_not_equal(meshtastic_rf_rssi_bin(-1), MESHTASTIC_RF_RSSI_RAIL_BIN,
			  "-1 dBm is a real reading and must NOT land in the rail bin");

	zassert_equal(meshtastic_rf_rssi_bin(-131), 0, "below the floor underflows to bin 0");
	zassert_equal(meshtastic_rf_rssi_bin(-130), 1, "the floor itself is the first real bin");
	zassert_equal(meshtastic_rf_rssi_bin(-126), 1, "5 dB wide: -130..-125 share a bin");
	zassert_equal(meshtastic_rf_rssi_bin(-125), 2, "the next 5 dB step is the next bin");
}

ZTEST(mesh_sim, test_rf_snr_bin_edges)
{
	zassert_equal(meshtastic_rf_snr_bin(-23), 0, "below the floor underflows");
	zassert_equal(meshtastic_rf_snr_bin(-22), 1, "the floor is the first real bin");
	zassert_equal(meshtastic_rf_snr_bin(-21), 1, "2 dB wide");
	zassert_equal(meshtastic_rf_snr_bin(-20), 2, "the next step");
	zassert_equal(meshtastic_rf_snr_bin(15), MESHTASTIC_RF_SNR_BINS - 1, "above +14 overflows");
}

ZTEST(mesh_sim, test_rf_bins_every_received_frame)
{
	struct meshtastic_rf_window w;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	meshtastic_rf_reset();
	build_peer_text(0x7001U, "measure", wire, &wire_len);

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -87, 6),
		   "inject failed");

	meshtastic_rf_window_get(&w);
	zassert_equal(w.hist.frames, 1U, "the frame must be counted");
	zassert_equal(w.hist.rssi[meshtastic_rf_rssi_bin(-87)], 1U, "RSSI landed in its bin");
	zassert_equal(w.hist.snr[meshtastic_rf_snr_bin(6)], 1U, "SNR landed in its bin");
	zassert_equal(w.hist.rssi_min, -87, "min tracked");
	zassert_equal(w.hist.snr_max, 6, "max tracked");
}

/*
 * A frame the stack cannot decode still tells us the radio heard something.
 * Weak, undecodable frames are exactly the population an antenna or LNA change
 * moves, so measuring only what decoded would systematically miss the effect.
 */
ZTEST(mesh_sim, test_rf_counts_undecodable_frames)
{
	struct meshtastic_rf_window w;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	meshtastic_rf_reset();
	build_peer_text(0x7002U, "foreign", wire, &wire_len);
	wire[13] ^= 0xFFU; /* channel hash byte: now a channel we hold no key for */

	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -95, 2), "inject failed");

	meshtastic_rf_window_get(&w);
	zassert_equal(w.hist.frames, 1U,
		      "a frame we cannot decrypt was still demodulated and must be measured");
}

ZTEST(mesh_sim, test_rf_splits_direct_from_relayed)
{
	struct meshtastic_rf_peer peers[4];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	int n;

	meshtastic_rf_reset();

	/* One hop: hop_start still equals hop_limit, so hdr->src really is the
	 * transmitter we heard. */
	build_peer_text(0x7003U, "direct", wire, &wire_len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, 10), "inject failed");

	/* Same source, but a hop has been consumed — this measures the RELAY's
	 * link to us, and must not touch the per-peer averages. */
	build_peer_text(0x7004U, "relayed", wire, &wire_len);
	wire[WIRE_FLAGS_OFFSET] = (uint8_t)((wire[WIRE_FLAGS_OFFSET] & ~0x07U) | 0x02U);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -100, -5), "inject failed");

	n = meshtastic_rf_peers_get(peers, ARRAY_SIZE(peers));
	zassert_equal(n, 1, "both frames came from one source id");
	zassert_equal(peers[0].frames_direct, 1U, "exactly one direct frame");
	zassert_equal(peers[0].frames_relayed, 1U, "exactly one relayed frame");
	zassert_equal(peers[0].snr_sum, 10, "only the DIRECT frame's SNR may be averaged");
	zassert_equal(peers[0].rssi_sum, -60, "only the DIRECT frame's RSSI may be averaged");
}

ZTEST(mesh_sim, test_rf_reset_clears_the_window_but_not_the_lifetime)
{
	struct meshtastic_rf_window before, after;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	meshtastic_rf_reset();
	build_peer_text(0x7005U, "lifetime", wire, &wire_len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -70, 5), "inject failed");
	meshtastic_rf_window_get(&before);
	zassert_true(before.lifetime > 0U, "lifetime should have advanced");

	meshtastic_rf_reset();
	meshtastic_rf_window_get(&after);

	zassert_equal(after.hist.frames, 0U, "the window must start empty");
	zassert_equal(after.lifetime, before.lifetime,
		      "a reset must not destroy the record of how much was ever seen");
}
#endif /* CONFIG_MESHTASTIC_RF_HIST */

/* --- licensed operation is opt-in ------------------------------------------
 *
 * is_licensed lifts the regional power clamp AND skips the front-end backoff,
 * so a spuriously-true flag transmits over the legal limit. It must therefore
 * be false unless a stored owner record explicitly carries it — never inherited
 * from uninitialised memory, never implied by an absent field.
 */
ZTEST(mesh_sim, test_licensed_defaults_to_false)
{
	bool licensed = true; /* poisoned, so a no-op read cannot pass this */

	meshtastic_config_store_get_owner_flags(&licensed, NULL);
	zassert_false(licensed, "a node with no explicit licence must not be licensed");
	zassert_false(mt.licensed, "the cached transmit-path flag must agree");
}

/*
 * The proto has no presence flag for is_licensed, so a User built from zero is
 * indistinguishable from one that explicitly says "not licensed". That
 * ambiguity is resolved in the SAFE direction deliberately: writing an owner
 * without the flag clears the licence rather than silently retaining elevated
 * transmit rights.
 */
ZTEST(mesh_sim, test_setting_an_owner_without_the_flag_clears_the_licence)
{
	meshtastic_User user = meshtastic_User_init_zero;
	bool licensed = false;

	strcpy(user.long_name, "licensed node");
	user.is_licensed = true;
	zassert_ok(meshtastic_config_store_set_owner(&user), "set_owner failed");
	meshtastic_config_store_get_owner_flags(&licensed, NULL);
	zassert_true(licensed, "an explicit licence must be honoured");
	zassert_true(mt.licensed, "and must reach the cached transmit-path flag");

	/* Now a client that does not get-modify-set: same name, flag absent. */
	user = (meshtastic_User)meshtastic_User_init_zero;
	strcpy(user.long_name, "licensed node");
	zassert_ok(meshtastic_config_store_set_owner(&user), "set_owner failed");
	meshtastic_config_store_get_owner_flags(&licensed, NULL);
	zassert_false(licensed, "an owner write without the flag must clear the licence");
	zassert_false(mt.licensed, "the cached flag must not go stale and keep transmit rights");
}

/*
 * The cache must not go stale. mt.licensed is read per frame on the transmit
 * path and cannot take the config-store mutex there, so set_owner refreshes it
 * — along with the resolved tx_power, because refreshing only one would leave
 * the node skipping the front-end backoff while still clamped to the region
 * limit, a combination a reboot would never produce.
 */
ZTEST(mesh_sim, test_licence_change_refreshes_the_cached_tx_power)
{
	meshtastic_User user = meshtastic_User_init_zero;
	int8_t unlicensed_power;

	user = (meshtastic_User)meshtastic_User_init_zero;
	strcpy(user.long_name, "n");
	zassert_ok(meshtastic_config_store_set_owner(&user), "set_owner failed");
	unlicensed_power = mt.tx_power;

	user.is_licensed = true;
	zassert_ok(meshtastic_config_store_set_owner(&user), "set_owner failed");
	zassert_true(mt.licensed, "the licence must take effect without a reboot");
	zassert_true(mt.tx_power >= unlicensed_power,
		     "a licence must never LOWER the resolved transmit power (%d -> %d)",
		     unlicensed_power, mt.tx_power);

	/* Leave the fixture unlicensed for every test that follows. */
	user.is_licensed = false;
	zassert_ok(meshtastic_config_store_set_owner(&user), "set_owner failed");
	zassert_false(mt.licensed, "cleanup: the licence must clear again");
}

#if defined(CONFIG_MESHTASTIC_CLUSTER)
/* ==========================================================================
 * Cluster M4b — the anti-entropy walk, end to end over the sim radio.
 *
 * The M4a test above proved a node NOTICES divergence. These prove what it now
 * does about it: ask for the peer's stamp vector, diff it, ask for the entries
 * it is missing, merge them under LWW, and push the result through the config
 * store so the node is actually running the fleet's configuration.
 *
 * The peer is played by hand — inject its frames, capture and decode ours —
 * which is the only way to assert on what goes ON THE WIRE rather than on what
 * the module believes it sent. The M4a lesson (a TX path that reached hardware
 * having never been executed) is the reason that distinction is worth the code.
 * ========================================================================== */

#define CLUSTER_CH 2U
/* Any 32 bytes: the D4 gate is a memcmp of the NodeDB's stored key against
 * SecurityConfig.admin_key, so nothing here has to be a real X25519 point. */
static const uint8_t peer_key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN] = {
	0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA,
	0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5,
	0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
};

/*
 * Provision (or tear down) the channel the module binds to — through the CONFIG
 * STORE, not meshtastic_channels_set_slot().
 *
 * That is not a style preference. Every config-store write ends in
 * meshtastic_config_store_apply_core(), which re-applies the whole channel
 * table FROM THE STORE — so a slot set directly on the channel layer is
 * silently dropped by the next set_config(). During this walk the firmware
 * itself writes the store (that is what the reconciler does), so a directly-set
 * slot vanishes mid-test and the module stops recognising its own channel.
 * Going through the store is also what the shell and admin paths do, so this
 * matches how a real node is provisioned.
 */
static void cluster_channel(bool on)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.has_settings = true;
	if (on) {
		ch.role = meshtastic_Channel_Role_SECONDARY;
		strncpy(ch.settings.name, CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME,
			sizeof(ch.settings.name) - 1U);
		ch.settings.psk.size = 16U;
		ch.settings.psk.bytes[0] = 0x42U;
	} else {
		ch.role = meshtastic_Channel_Role_DISABLED;
	}
	zassert_ok(meshtastic_config_store_set_channel(CLUSTER_CH, &ch),
		   "cluster channel set failed");
}

/* Teach this node PEER's public key the way the air does — a NodeInfo — then
 * name that key as an admin. PEER is now a master for the D4 gate. */
static void trust_peer_as_master(bool trusted)
{
	meshtastic_User user = meshtastic_User_init_zero;
	meshtastic_Config sec = meshtastic_Config_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = PEER_NODE_ID,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	user.public_key.size = sizeof(peer_key);
	memcpy(user.public_key.bytes, peer_key, sizeof(peer_key));
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);

	sec.which_payload_variant = meshtastic_Config_security_tag;
	if (trusted) {
		sec.payload_variant.security.admin_key_count = 1U;
		sec.payload_variant.security.admin_key[0].size = (pb_size_t)sizeof(peer_key);
		memcpy(sec.payload_variant.security.admin_key[0].bytes, peer_key,
		       sizeof(peer_key));
	}
	zassert_ok(meshtastic_config_store_set_config(&sec), "security write failed");
}

/* Inject one ClusterMessage as if PEER had transmitted it. */
static void inject_cluster(const zephyrtastic_ClusterMessage *msg, uint32_t to, uint32_t id)
{
	uint8_t cbuf[zephyrtastic_ClusterMessage_size];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	pb_ostream_t os = pb_ostream_from_buffer(cbuf, sizeof(cbuf));
	uint32_t wire_len;
	struct meshtastic_packet pkt = {
		.from = PEER_NODE_ID,
		.to = to,
		.id = id,
		.portnum = MESHTASTIC_PORT_PRIVATE,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = CLUSTER_CH,
	};

	zassert_true(pb_encode(&os, zephyrtastic_ClusterMessage_fields, msg), "encode failed");
	pkt.payload = cbuf;
	pkt.payload_len = os.bytes_written;
	zassert_ok(meshtastic_build_wire_packet(&pkt, wire, &wire_len), "wire build failed");
	/* The walk is a conversation, so every inject here follows one of OUR
	 * transmissions — and the stack cancels RX while transmitting. Nothing
	 * is listening until the radio comes back. */
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
}

/*
 * Drain captured TX until one of OUR cluster frames appears. The filter matters:
 * an injected broadcast digest is flood-relayed by this node too, and that relay
 * still carries the PEER's src — so "from == us" is what separates a frame we
 * originated from one we merely repeated.
 */
static bool take_cluster_tx(zephyrtastic_ClusterMessage *out, uint32_t *to)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(2000)) == 0) {
		struct meshtastic_packet pkt;
		uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
		pb_istream_t is;

		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &pkt, payload,
						  sizeof(payload)) != 0) {
			continue;
		}
		if (pkt.from != TEST_NODE_ID || pkt.portnum != MESHTASTIC_PORT_PRIVATE) {
			continue;
		}
		*out = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
		is = pb_istream_from_buffer(pkt.payload, pkt.payload_len);
		if (!pb_decode(&is, zephyrtastic_ClusterMessage_fields, out)) {
			continue;
		}
		if (to != NULL) {
			*to = pkt.to;
		}
		return true;
	}
	return false;
}

/*
 * Wait until no anti-entropy exchange is in flight.
 *
 * Only one runs at a time (§3.3), and an earlier test may have opened one
 * against a peer that never answered — test_cluster_digest_divergence_noticed
 * does exactly that, by design. Waiting here is not a workaround: the timeout
 * that frees the slot is the mechanism guaranteeing a vanished peer can never
 * wedge the walk, so exercising it is worth a few virtual seconds.
 */
static void wait_cluster_idle(void)
{
	for (int i = 0; i < 400; i++) {
		if (strcmp(meshtastic_cluster_sync_state(NULL), "idle") == 0) {
			return;
		}
		k_msleep(50);
	}
	zassert_unreachable("an anti-entropy exchange never timed out");
}

/* A DisplayConfig section encoded exactly as the document carries it: an
 * upstream meshtastic.Config with one payload variant set (D8 — the section
 * bytes are upstream's own encoding, not a translation of it). */
static size_t encode_display(uint32_t screen_on_secs, uint8_t *buf, size_t buf_len)
{
	meshtastic_Config cfg = meshtastic_Config_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, buf_len);

	cfg.which_payload_variant = meshtastic_Config_display_tag;
	cfg.payload_variant.display.screen_on_secs = screen_on_secs;
	zassert_true(pb_encode(&os, meshtastic_Config_fields, &cfg), "display encode failed");
	return os.bytes_written;
}

static uint32_t stored_screen_on_secs(void)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_display_tag, &cfg),
		   "display read failed");
	return cfg.payload_variant.display.screen_on_secs;
}

/* THE M4b proof: hand-planted divergence converges, and the converged value is
 * applied — not merely stored in the document. */
ZTEST(mesh_sim, test_cluster_walk_converges_and_applies)
{
	zephyrtastic_ClusterMessage msg = zephyrtastic_ClusterMessage_init_zero;
	struct meshtastic_cluster_stats before_st, after_st;
	struct meshtastic_hlc_stamp base_stamp = {
		.physical_ms = 1787600000000LL, .counter = 0U, .node_id = PEER_NODE_ID};
	struct meshtastic_hlc_stamp store_stamp;
	uint32_t to = 0U;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	size_t payload_len;

	cluster_channel(true);
	trust_peer_as_master(true);
	wait_cluster_idle();
	quiesce();
	meshtastic_cluster_stats_get(&before_st);

	/* 1. PEER advertises a document we do not have. */
	msg.which_variant = zephyrtastic_ClusterMessage_digest_tag;
	msg.variant.digest.doc_hash = 0x1234ABCDU;
	msg.variant.digest.entry_count = 1U;
	msg.variant.digest.has_max_stamp = true;
	msg.variant.digest.max_stamp.physical_ms = base_stamp.physical_ms;
	msg.variant.digest.max_stamp.node_id = PEER_NODE_ID;
	inject_cluster(&msg, MESHTASTIC_NODE_BROADCAST, 0x7001U);

	/* 2. We ask for its stamp vector — a UNICAST, which is what lets a
	 * LoRa-mute node take this leg over a BLE peer link instead. */
	zassert_true(take_cluster_tx(&msg, &to), "no ClusterVectorReq was transmitted");
	zassert_equal(msg.which_variant, zephyrtastic_ClusterMessage_vector_req_tag,
		      "a digest mismatch must produce a vector request, not %u",
		      (unsigned int)msg.which_variant);
	zassert_equal(to, PEER_NODE_ID, "the request must go to the diverging peer");

	/* 3. PEER answers with one row: base/display, stamped by PEER. */
	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
	msg.which_variant = zephyrtastic_ClusterMessage_vector_tag;
	msg.variant.vector.entries_count = 1U;
	msg.variant.vector.entries[0].has_key = true;
	msg.variant.vector.entries[0].key.layer = zephyrtastic_ClusterLayer_BASE;
	msg.variant.vector.entries[0].key.section = meshtastic_Config_display_tag;
	msg.variant.vector.entries[0].has_stamp = true;
	msg.variant.vector.entries[0].stamp.physical_ms = base_stamp.physical_ms;
	msg.variant.vector.entries[0].stamp.node_id = PEER_NODE_ID;
	msg.variant.vector.offset = 0U;
	msg.variant.vector.total = 1U;
	inject_cluster(&msg, TEST_NODE_ID, 0x7002U);

	/* 4. We ask for exactly the key we lack. */
	zassert_true(take_cluster_tx(&msg, &to), "no ClusterEntryReq was transmitted");
	zassert_equal(msg.which_variant, zephyrtastic_ClusterMessage_entry_req_tag,
		      "the vector must be diffed into an entry request");
	zassert_equal(msg.variant.entry_req.keys_count, 1U, "exactly one key was missing");
	zassert_equal(msg.variant.entry_req.keys[0].section,
		      (uint32_t)meshtastic_Config_display_tag, "wrong section requested");
	zassert_equal(msg.variant.entry_req.keys[0].layer, zephyrtastic_ClusterLayer_BASE);
	zassert_equal(to, PEER_NODE_ID);

	/* 5. PEER sends the entry. */
	payload_len = encode_display(123U, payload, sizeof(payload));
	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
	msg.which_variant = zephyrtastic_ClusterMessage_entry_tag;
	msg.variant.entry.has_key = true;
	msg.variant.entry.key.layer = zephyrtastic_ClusterLayer_BASE;
	msg.variant.entry.key.section = meshtastic_Config_display_tag;
	msg.variant.entry.has_stamp = true;
	msg.variant.entry.stamp.physical_ms = base_stamp.physical_ms;
	msg.variant.entry.stamp.node_id = PEER_NODE_ID;
	msg.variant.entry.payload.size = (pb_size_t)payload_len;
	memcpy(msg.variant.entry.payload.bytes, payload, payload_len);
	inject_cluster(&msg, TEST_NODE_ID, 0x7003U);
	k_sleep(K_MSEC(300)); /* the reconciler runs on the system workqueue */

	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.entry_rx_applied, before_st.entry_rx_applied + 1U,
		      "the entry must have merged into the document");
	zassert_equal(meshtastic_cluster_entry_count(), 1U, "the document must hold it");

	/* 6. The point of all of it: the node is now RUNNING the fleet's value,
	 * not merely storing a document that describes it. */
	zassert_equal(stored_screen_on_secs(), 123U,
		      "effective(me, display) must reach the config store");
	zassert_equal(after_st.sections_applied, before_st.sections_applied + 1U);

	/* 7. And the store carries the DOCUMENT's stamp, not a fresh local one.
	 * That equality is the origin marker (D10): it is how this node knows
	 * the value came from the document and must never be lifted back into
	 * it — without which every applied entry would mint a new stamp and
	 * gossip forever. */
	zassert_ok(meshtastic_config_store_get_config_stamp(meshtastic_Config_display_tag,
							    &store_stamp),
		   "stamp read failed");
	zassert_equal(meshtastic_hlc_compare(&store_stamp, &base_stamp), 0,
		      "a doc-derived write must carry the document's own stamp");

	/* 8. Converged: PEER's digest now matches, and costs one frame to say so. */
	quiesce();
	meshtastic_cluster_stats_get(&before_st);
	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
	msg.which_variant = zephyrtastic_ClusterMessage_digest_tag;
	msg.variant.digest.doc_hash = meshtastic_cluster_doc_hash_now();
	msg.variant.digest.entry_count = 1U;
	msg.variant.digest.has_max_stamp = true;
	msg.variant.digest.max_stamp.physical_ms = base_stamp.physical_ms;
	msg.variant.digest.max_stamp.node_id = PEER_NODE_ID;
	inject_cluster(&msg, MESHTASTIC_NODE_BROADCAST, 0x7004U);
	k_sleep(K_MSEC(200));

	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.digest_rx_match, before_st.digest_rx_match + 1U,
		      "the converged document must match the peer's digest");
	zassert_equal(after_st.pull_started, before_st.pull_started,
		      "a matching digest must cost nothing beyond reading it");

	cluster_channel(false);
}

/*
 * D4, the negative half. The same entry, authored by a node that is NOT in this
 * node's admin_key[], is refused — and refused at ingest, so it never reaches
 * the document, let alone the config store.
 *
 * This gate is not proof of anything (§4 is explicit: Meshtastic PKC has no
 * signing primitive, so a channel member can claim any author and no receiver
 * can refute it). It is the difference between an ordinary member's mistake
 * rewriting fleet defaults and it not.
 */
ZTEST(mesh_sim, test_cluster_base_entry_from_untrusted_author_refused)
{
	zephyrtastic_ClusterMessage msg = zephyrtastic_ClusterMessage_init_zero;
	struct meshtastic_cluster_stats before_st, after_st;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	size_t payload_len;
	uint32_t before_display;

	cluster_channel(true);
	trust_peer_as_master(false); /* PEER's key is known, but not an admin key */
	wait_cluster_idle();
	quiesce();
	before_display = stored_screen_on_secs();
	meshtastic_cluster_stats_get(&before_st);

	payload_len = encode_display(before_display + 77U, payload, sizeof(payload));
	msg.which_variant = zephyrtastic_ClusterMessage_entry_tag;
	msg.variant.entry.has_key = true;
	msg.variant.entry.key.layer = zephyrtastic_ClusterLayer_BASE;
	msg.variant.entry.key.section = meshtastic_Config_display_tag;
	msg.variant.entry.has_stamp = true;
	/* Far in the future, so LWW would certainly have taken it. */
	msg.variant.entry.stamp.physical_ms = 1900000000000LL;
	msg.variant.entry.stamp.node_id = PEER_NODE_ID;
	msg.variant.entry.payload.size = (pb_size_t)payload_len;
	memcpy(msg.variant.entry.payload.bytes, payload, payload_len);
	inject_cluster(&msg, TEST_NODE_ID, 0x7101U);
	k_sleep(K_MSEC(300));

	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.entry_rx_refused, before_st.entry_rx_refused + 1U,
		      "a base entry from a non-master must be refused");
	zassert_equal(after_st.entry_rx_applied, before_st.entry_rx_applied,
		      "and must not reach the document");
	zassert_equal(stored_screen_on_secs(), before_display,
		      "nor, therefore, the config store");

	/* The secret boundary, ingest side (D9): security is refused even when
	 * the author IS trusted — the ban does not depend on every peer in the
	 * fleet running correct code. */
	trust_peer_as_master(true);
	meshtastic_cluster_stats_get(&before_st);
	msg.variant.entry.key.section = meshtastic_Config_security_tag;
	inject_cluster(&msg, TEST_NODE_ID, 0x7102U);
	k_sleep(K_MSEC(200));
	meshtastic_cluster_stats_get(&after_st);
	zassert_equal(after_st.entry_rx_refused, before_st.entry_rx_refused + 1U,
		      "security must be refused at ingest even from a master");

	cluster_channel(false);
}
#endif /* CONFIG_MESHTASTIC_CLUSTER */
