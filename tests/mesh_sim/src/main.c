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
#include "meshtastic_preset.h"
#include "meshtastic_outbound.h"
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

ZTEST_SUITE(mesh_sim, NULL, mesh_sim_setup, mesh_sim_before, NULL, NULL);
