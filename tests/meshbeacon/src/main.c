/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Mesh beacon (agents-dnr4.25), driven through the sim radio.
 *
 * In the reference's terms (MeshBeaconModule.cpp):
 *   - nothing goes out without FLAG_BROADCAST, or without text/offer content;
 *   - a beacon is a ZERO-HOP broadcast on port 37 (hop_limit 0, hop_start 0),
 *     START_DELAY after the flag is set and every interval after;
 *   - text + offer go as one packet, or under FLAG_LEGACY_SPLIT as two (offer
 *     on port 37, text on port 1), both with hop_start 1;
 *   - a target names a channel-table slot; one that needs another preset is
 *     skipped (this port does not switch the radio), and an inline channel with
 *     no matching slot is skipped;
 *   - a CLIENT_HIDDEN node never beacons;
 *   - the listener caches an offer only with FLAG_LISTEN, never applies it, and
 *     a zero-hop beacon heard is never relayed;
 *   - the write-time sanitiser caps the text, floors the interval and clears
 *     unknown region/preset/channel_index.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/mesh_beacon.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_meshbeacon.h"
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define PEER_A       0x0B000001U

#define DELAY_S      CONFIG_MESHTASTIC_MESHBEACON_START_DELAY_SEC
#define INTERVAL_S   CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static const uint8_t test_psk[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
				     0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x02};
static const uint8_t invite_psk[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
				       0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};

/* ==========================================================================
 * Capture / injection
 * ========================================================================== */

struct frame {
	uint32_t from;
	uint32_t to;
	uint32_t portnum;
	uint8_t hop_limit;
	uint8_t hop_start;
	uint8_t channel_index;
	meshtastic_MeshBeacon beacon; /* decoded when portnum == 37 */
	char text[101];               /* copied when portnum == 1 */
};

struct captured {
	uint32_t ours;    /* frames THIS node originated */
	uint32_t relayed; /* frames on the air that some other node originated */
	struct frame f[6];
};

static struct captured drain_tx(k_timeout_t settle)
{
	struct lora_sim_frame raw;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct captured c = {0};

	k_sleep(settle);

	while (lora_sim_take_tx(lora_dev, &raw, K_NO_WAIT) == 0) {
		if (meshtastic_decode_wire_packet(raw.data, raw.len, 0, 0, &decoded, payload,
						  sizeof(payload)) != 0) {
			continue;
		}
		if (decoded.from != TEST_NODE_ID) {
			c.relayed++;
			continue;
		}
		if (c.ours >= ARRAY_SIZE(c.f)) {
			continue;
		}
		struct frame *f = &c.f[c.ours++];

		*f = (struct frame){
			.from = decoded.from,
			.to = decoded.to,
			.portnum = decoded.portnum,
			.hop_limit = decoded.hop_limit,
			.hop_start = decoded.hop_start,
			.channel_index = decoded.channel_index,
			.beacon = meshtastic_MeshBeacon_init_zero,
		};
		if (decoded.portnum == MESHTASTIC_PORT_MESH_BEACON) {
			pb_istream_t is = pb_istream_from_buffer(decoded.payload,
								 decoded.payload_len);

			zassert_true(pb_decode(&is, meshtastic_MeshBeacon_fields, &f->beacon),
				     "our own MeshBeacon must decode");
		} else if (decoded.portnum == MESHTASTIC_PORT_TEXT_MESSAGE) {
			size_t n = MIN(decoded.payload_len, sizeof(f->text) - 1U);

			memcpy(f->text, decoded.payload, n);
			f->text[n] = '\0';
		}
	}
	return c;
}

static const struct frame *find_frame(const struct captured *c, uint32_t portnum)
{
	for (uint32_t i = 0; i < c->ours; i++) {
		if (c->f[i].portnum == portnum) {
			return &c->f[i];
		}
	}
	return NULL;
}

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

/* A peer's beacon, exactly as this module would send one: zero hops. */
static void inject_beacon(uint32_t from, uint32_t id, const char *text, bool with_offer)
{
	meshtastic_MeshBeacon b = meshtastic_MeshBeacon_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));

	if (text != NULL) {
		strncpy(b.message, text, sizeof(b.message) - 1U);
	}
	if (with_offer) {
		b.has_offer_channel = true;
		strcpy(b.offer_channel.name, "Invite");
		b.offer_channel.psk.size = sizeof(invite_psk);
		memcpy(b.offer_channel.psk.bytes, invite_psk, sizeof(invite_psk));
		b.has_offer_preset = true;
		b.offer_preset = meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO;
		b.offer_region = meshtastic_Config_LoRaConfig_RegionCode_US;
	}
	zassert_true(pb_encode(&os, meshtastic_MeshBeacon_fields, &b), "encode failed");

	struct meshtastic_packet packet = {
		.from = from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = id,
		.portnum = MESHTASTIC_PORT_MESH_BEACON,
		.payload = payload,
		.payload_len = os.bytes_written,
		.hop_limit = 0U,
		.hop_start = 0U,
		.channel_index = meshtastic_channels_primary_index(),
	};
	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len), "build failed");
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, 6), "inject failed");
	k_msleep(100);
}

static meshtastic_ModuleConfig_MeshBeaconConfig cfg_with(uint32_t flags, const char *text)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg =
		meshtastic_ModuleConfig_MeshBeaconConfig_init_zero;

	cfg.flags = flags;
	cfg.broadcast_interval_secs = INTERVAL_S;
	if (text != NULL) {
		strncpy(cfg.broadcast_message, text, sizeof(cfg.broadcast_message) - 1U);
	}
	return cfg;
}

static void add_offer(meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	cfg->has_broadcast_offer_channel = true;
	strcpy(cfg->broadcast_offer_channel.name, "Invite");
	cfg->broadcast_offer_channel.psk.size = sizeof(invite_psk);
	memcpy(cfg->broadcast_offer_channel.psk.bytes, invite_psk, sizeof(invite_psk));
	cfg->has_broadcast_offer_preset = true;
	cfg->broadcast_offer_preset = meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO;
	cfg->broadcast_offer_region = meshtastic_Config_LoRaConfig_RegionCode_US;
}

#define SETTLE K_MSEC(500)

static void *meshbeacon_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = test_psk,
		.psk_len = sizeof(test_psk),
		.channel_name = "TestNet",
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void meshbeacon_before(void *fixture)
{
	meshtastic_ModuleConfig_MeshBeaconConfig off = cfg_with(0U, NULL);

	ARG_UNUSED(fixture);

	zassert_ok(meshtastic_meshbeacon_set(&off), "clear failed");
	meshtastic_meshbeacon_reset();
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	k_msleep(200);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(meshbeacon, NULL, meshbeacon_setup, meshbeacon_before, NULL, NULL);

/* ==========================================================================
 * Broadcasting
 * ========================================================================== */

ZTEST(meshbeacon, test_nothing_without_the_flag_or_without_content)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	struct captured c;

	/* Flag but nothing to say (reference: "empty msg, no offer, skip"). */
	cfg = cfg_with(MESHTASTIC_MESHBEACON_FLAG_BROADCAST, NULL);
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_equal(meshtastic_meshbeacon_send(), -ENODATA, "nothing to beacon");
	c = drain_tx(K_SECONDS(DELAY_S + 1));
	zassert_equal(c.ours, 0U, "no content: nothing at the delay either");

	/* Content but no flag: only an explicit send goes out. */
	cfg = cfg_with(0U, "join us");
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	c = drain_tx(K_SECONDS(DELAY_S + 1));
	zassert_equal(c.ours, 0U, "no BROADCAST flag: nothing unprompted");
	zassert_ok(meshtastic_meshbeacon_send(), "an explicit send still works");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 1U, "");
}

ZTEST(meshbeacon, test_a_text_beacon_is_zero_hop_after_the_delay_then_every_interval)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg =
		cfg_with(MESHTASTIC_MESHBEACON_FLAG_BROADCAST, "join us");
	struct captured c;
	const struct frame *f;

	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");

	c = drain_tx(K_MSEC(300));
	zassert_equal(c.ours, 0U, "nothing before the start delay");

	c = drain_tx(K_SECONDS(DELAY_S));
	zassert_equal(c.ours, 1U, "one beacon at the start delay");
	f = &c.f[0];
	/* Reference sendBeacon: text with no offer is sent as a plain
	 * TEXT_MESSAGE_APP (sendTextOnly), so every receiver can read it; only an
	 * offer needs the MESH_BEACON_APP envelope. */
	zassert_equal(f->portnum, MESHTASTIC_PORT_TEXT_MESSAGE,
		      "text-only: a TEXT_MESSAGE_APP frame (reference)");
	zassert_equal(f->to, MESHTASTIC_NODE_BROADCAST, "a broadcast");
	zassert_equal(f->hop_limit, 0U, "ZERO hops: hop_limit 0, nobody relays it");
	zassert_equal(f->hop_start, 0U, "hop_start 0 (no legacy override)");
	zassert_equal(f->channel_index, meshtastic_channels_primary_index(), "on the primary");
	zassert_str_equal(f->text, "join us", "carrying the text");

	c = drain_tx(K_SECONDS(INTERVAL_S / 2));
	zassert_equal(c.ours, 0U, "nothing inside the interval");
	c = drain_tx(K_SECONDS((INTERVAL_S / 2) + 2));
	zassert_equal(c.ours, 1U, "and one at the interval");

	/* Clearing the flag cancels the cycle. */
	cfg.flags = 0U;
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	c = drain_tx(K_SECONDS(INTERVAL_S + 2));
	zassert_equal(c.ours, 0U, "flag off: the cycle stops");
}

ZTEST(meshbeacon, test_offer_and_text_go_combined_or_split_under_the_legacy_flag)
{
	/* Explicit sends only (no BROADCAST flag): setting the section with the
	 * flag on re-arms the start delay, and a cycle firing mid-test would add
	 * frames to the count. */
	meshtastic_ModuleConfig_MeshBeaconConfig cfg = cfg_with(0U, "join us");
	struct captured c;
	const struct frame *b;
	const struct frame *t;

	add_offer(&cfg);
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");

	zassert_ok(meshtastic_meshbeacon_send(), "");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 1U, "combined: ONE packet");
	b = find_frame(&c, MESHTASTIC_PORT_MESH_BEACON);
	zassert_not_null(b, "on MESH_BEACON_APP");
	zassert_str_equal(b->beacon.message, "join us", "with the text");
	zassert_true(b->beacon.has_offer_channel, "and the channel offer");
	zassert_str_equal(b->beacon.offer_channel.name, "Invite", "");
	zassert_equal(b->beacon.offer_channel.psk.size, 16U, "PSK included on purpose");
	zassert_true(b->beacon.has_offer_preset, "");
	zassert_equal(b->beacon.offer_preset, meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
		      "");
	zassert_equal(b->beacon.offer_region, meshtastic_Config_LoRaConfig_RegionCode_US, "");
	zassert_equal(b->hop_start, 0U, "");

	/* Legacy split: offer on port 37 WITHOUT the text, text on port 1, both
	 * hop_start 1 so pre-2.7.20 receivers accept the zero-hop frame. */
	cfg.flags |= MESHTASTIC_MESHBEACON_FLAG_LEGACY_SPLIT;
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_ok(meshtastic_meshbeacon_send(), "");
	/* Two frames back to back: the sim models airtime, so give the second
	 * one time to finish. */
	c = drain_tx(K_SECONDS(4));
	zassert_equal(c.ours, 2U, "legacy split: TWO packets");
	b = find_frame(&c, MESHTASTIC_PORT_MESH_BEACON);
	t = find_frame(&c, MESHTASTIC_PORT_TEXT_MESSAGE);
	zassert_not_null(b, "split-A on MESH_BEACON_APP");
	zassert_not_null(t, "split-B on TEXT_MESSAGE_APP");
	zassert_equal(b->beacon.message[0], '\0', "split-A carries the offer only");
	zassert_true(b->beacon.has_offer_channel, "");
	zassert_str_equal(t->text, "join us", "split-B carries the text");
	zassert_equal(b->hop_limit, 0U, "");
	zassert_equal(t->hop_limit, 0U, "both zero-hop");
	zassert_equal(b->hop_start, 1U, "legacy: hop_start 1");
	zassert_equal(t->hop_start, 1U, "legacy: hop_start 1");
}

ZTEST(meshbeacon, test_targets_resolve_to_slots_and_radio_switches_are_skipped)
{
	/* Explicit sends only -- see the previous test for why no BROADCAST flag. */
	meshtastic_ModuleConfig_MeshBeaconConfig cfg = cfg_with(0U, "join us");
	meshtastic_Channel invite = meshtastic_Channel_init_zero;
	struct meshtastic_meshbeacon_stats st;
	struct captured c;

	/* A secondary "Invite" channel in slot 1. */
	invite.index = 1;
	invite.role = meshtastic_Channel_Role_SECONDARY;
	invite.has_settings = true;
	strcpy(invite.settings.name, "Invite");
	invite.settings.psk.size = sizeof(invite_psk);
	memcpy(invite.settings.psk.bytes, invite_psk, sizeof(invite_psk));
	zassert_ok(meshtastic_channels_set_slot(1U, &invite), "set slot 1");

	/* Three targets: slot 1; another preset (needs a radio switch); slot 1
	 * again (a duplicate). */
	cfg.broadcast_targets_count = 3U;
	cfg.broadcast_targets[0].has_channel_index = true;
	cfg.broadcast_targets[0].channel_index = 1U;
	cfg.broadcast_targets[1].has_preset = true;
	cfg.broadcast_targets[1].preset =
		(mt.modem_preset == meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST)
			? meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO
			: meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	cfg.broadcast_targets[2].has_channel_index = true;
	cfg.broadcast_targets[2].channel_index = 1U;
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");

	zassert_ok(meshtastic_meshbeacon_send(), "one usable target is enough");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 1U, "one frame: slot 1 once, the switch skipped, the dup skipped");
	zassert_equal(c.f[0].channel_index, 1U, "on the Invite channel");
	meshtastic_meshbeacon_stats(&st);
	zassert_equal(st.targets_radio_switch, 1U, "the other-preset target was counted as skipped");
	zassert_equal(st.targets_no_slot, 0U, "");

	/* Single-target path: an inline channel that matches slot 1 is honoured;
	 * one that matches nothing is skipped. */
	cfg.broadcast_targets_count = 0U;
	cfg.has_broadcast_on_channel = true;
	cfg.broadcast_on_channel = invite.settings;
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_ok(meshtastic_meshbeacon_send(), "");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 1U, "");
	zassert_equal(c.f[0].channel_index, 1U, "inline channel resolved to slot 1");

	strcpy(cfg.broadcast_on_channel.name, "Nowhere");
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_equal(meshtastic_meshbeacon_send(), -ENOENT,
		      "an inline channel with no slot cannot be encrypted for: skipped");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 0U, "nothing on the air, rather than on the wrong channel");
	meshtastic_meshbeacon_stats(&st);
	zassert_equal(st.targets_no_slot, 1U, "");

	/* Only the switch is refused, not the region we are already on. */
	cfg.has_broadcast_on_channel = false;
	cfg.broadcast_on_region = meshtastic_preset_region();
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_ok(meshtastic_meshbeacon_send(), "the running region is not a switch");
	c = drain_tx(SETTLE);
	zassert_equal(c.ours, 1U, "");
}

ZTEST(meshbeacon, test_a_hidden_client_never_beacons)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg =
		cfg_with(MESHTASTIC_MESHBEACON_FLAG_BROADCAST, "join us");
	struct captured c;

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN);
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	zassert_equal(meshtastic_meshbeacon_send(), -EPERM, "reference: CLIENT_HIDDEN never beacons");
	c = drain_tx(K_SECONDS(DELAY_S + 1));
	zassert_equal(c.ours, 0U, "and not at the delay either");
}

/* ==========================================================================
 * Listening
 * ========================================================================== */

ZTEST(meshbeacon, test_the_listener_caches_an_offer_only_while_listening_and_never_relays)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg = cfg_with(0U, NULL);
	struct meshtastic_meshbeacon_offer offer;
	struct meshtastic_meshbeacon_stats st;
	struct captured c;

	/* Not listening: a beacon is just a packet. */
	inject_beacon(PEER_A, 0x3A000001U, "come join", true);
	zassert_false(meshtastic_meshbeacon_last_offer(&offer), "not listening: no offer");
	meshtastic_meshbeacon_stats(&st);
	zassert_equal(st.beacons_heard, 0U, "");

	cfg.flags = MESHTASTIC_MESHBEACON_FLAG_LISTEN;
	zassert_ok(meshtastic_meshbeacon_set(&cfg), "");
	inject_beacon(PEER_A, 0x3A000002U, "come join", true);
	zassert_true(meshtastic_meshbeacon_last_offer(&offer), "listening: offer cached");
	zassert_equal(offer.sender, PEER_A, "");
	zassert_true(offer.has_channel, "");
	zassert_str_equal(offer.channel.name, "Invite", "");
	zassert_equal(offer.channel.psk.size, 16U, "");
	zassert_true(offer.has_preset, "");
	zassert_equal(offer.preset, meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO, "");
	zassert_equal(offer.region, meshtastic_Config_LoRaConfig_RegionCode_US, "");

	/* The offer was NOT applied: the primary channel is still ours. */
	zassert_str_equal(meshtastic_channels_get_name(meshtastic_channels_primary_index()),
			  "TestNet", "an offer is cached for the app, never applied");

	/* A text-only beacon is heard but does not disturb the cached offer. */
	inject_beacon(PEER_A, 0x3A000003U, "hello again", false);
	meshtastic_meshbeacon_stats(&st);
	zassert_equal(st.beacons_heard, 2U, "");
	zassert_equal(st.offers_cached, 1U, "text-only: no new offer");
	zassert_true(meshtastic_meshbeacon_last_offer(&offer), "still cached");

	/* Zero-hop: nothing we heard went back out on the air. (Our OWN frames
	 * may: the NodeInfo module asks an unknown peer who it is.) */
	c = drain_tx(SETTLE);
	zassert_equal(c.relayed, 0U, "a hop_limit-0 beacon is never relayed");

	meshtastic_meshbeacon_clear_offer();
	zassert_false(meshtastic_meshbeacon_last_offer(&offer), "forgotten on request");
}

/* ==========================================================================
 * The write-time sanitiser (reference AdminModule)
 * ========================================================================== */

ZTEST(meshbeacon, test_sanitise_mirrors_the_reference_admin_rules)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg = cfg_with(0U, NULL);

	memset(cfg.broadcast_message, 'x', sizeof(cfg.broadcast_message) - 1U);
	cfg.broadcast_message[sizeof(cfg.broadcast_message) - 1U] = '\0';
	cfg.broadcast_interval_secs = 10U;
	cfg.has_broadcast_offer_preset = true;
	cfg.broadcast_offer_preset = (meshtastic_Config_LoRaConfig_ModemPreset)99;
	cfg.broadcast_offer_region = (meshtastic_Config_LoRaConfig_RegionCode)250;
	cfg.broadcast_targets_count = 2U;
	cfg.broadcast_targets[0].has_channel_index = true;
	cfg.broadcast_targets[0].channel_index = 99U;
	cfg.broadcast_targets[0].region = (meshtastic_Config_LoRaConfig_RegionCode)250;
	cfg.broadcast_targets[1].has_preset = true;
	cfg.broadcast_targets[1].preset = (meshtastic_Config_LoRaConfig_ModemPreset)99;
	cfg.broadcast_targets[1].has_channel_index = true;
	cfg.broadcast_targets[1].channel_index = 1U;

	meshtastic_meshbeacon_sanitise(&cfg);

	zassert_equal(strlen(cfg.broadcast_message), MESHTASTIC_MESHBEACON_MESSAGE_MAX,
		      "text hard-capped at 100");
	zassert_equal(cfg.broadcast_interval_secs, INTERVAL_S, "interval floored, not zeroed");
	zassert_false(cfg.has_broadcast_offer_preset, "unknown offer preset cleared");
	zassert_equal(cfg.broadcast_offer_region, meshtastic_Config_LoRaConfig_RegionCode_UNSET,
		      "unknown offer region cleared");
	zassert_false(cfg.broadcast_targets[0].has_channel_index, "channel_index 99 cleared");
	zassert_equal(cfg.broadcast_targets[0].region,
		      meshtastic_Config_LoRaConfig_RegionCode_UNSET, "target region cleared");
	zassert_false(cfg.broadcast_targets[1].has_preset, "unknown target preset cleared");
	zassert_false(cfg.broadcast_targets[1].has_channel_index,
		      "...and its channel_index with it (reference)");

	/* 0 stays 0 (meaning the minimum); a value above it is kept. */
	cfg.broadcast_interval_secs = 0U;
	meshtastic_meshbeacon_sanitise(&cfg);
	zassert_equal(cfg.broadcast_interval_secs, 0U, "0 = unset stays 0");
	cfg.broadcast_interval_secs = INTERVAL_S + 5U;
	meshtastic_meshbeacon_sanitise(&cfg);
	zassert_equal(cfg.broadcast_interval_secs, INTERVAL_S + 5U, "above the floor: kept");
	zassert_equal(meshtastic_meshbeacon_interval_secs(), INTERVAL_S,
		      "the effective interval with 0 stored is the minimum");
}
