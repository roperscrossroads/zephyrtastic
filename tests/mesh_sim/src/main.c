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
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
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

ZTEST_SUITE(mesh_sim, NULL, mesh_sim_setup, mesh_sim_before, NULL, NULL);
