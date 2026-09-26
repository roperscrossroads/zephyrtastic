/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The cross-preset relay (agents-jbrq.12): the premises it is built on.
 *
 * The relay is a PAIR: an "ear" on the source preset forwards raw frames over
 * the BLE peer link to a node on the destination preset, which decides what to
 * do with them. This suite is that receiving node, with the ear's frames
 * injected on the BLE peer bearer.
 *
 * Nothing here needs relay code. These tests pin the facts the design stands
 * on, so a change that breaks one fails here and not on the bench:
 *
 *   - a default channel's hash follows the preset (why a LongFast frame is
 *     dropped verbatim on MediumFast: hole H1);
 *   - a named channel's hash does not (why a custom channel crosses presets
 *     with no extra slot);
 *   - holding the source preset's default channel in a spare slot is enough to
 *     read the ear's frames (H29), and the link-local rule still keeps them off
 *     our air;
 *   - the existing "well known" predicate is too wide to define the relay's
 *     public class (it takes the simple2..9 keys, which are other meshes).
 *
 * The relay's own behaviour gets its tests here too, once it exists. The
 * planned cases are listed in the tooling repo's docs/RELAY-TESTBED.md.
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
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU
/* An originator on the far tier, heard by the ear. */
#define FAR_NODE_ID  0x0D0D0D0DU

#define PRESET_LF meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST
#define PRESET_MF meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST
#define PRESET_ST meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO

/* The spare slot the receiving half uses for the far tier's channel. Slot 2 is
 * the cluster channel on the bench nodes, so the design says a slot other than
 * 0 and 2. */
#define SPARE_SLOT 1U

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* A custom key, the "pugs-are-great" example from the operator. */
static const uint8_t custom_psk[16] = {
	0x70, 0x75, 0x67, 0x73, 0x2d, 0x61, 0x72, 0x65,
	0x2d, 0x67, 0x72, 0x65, 0x61, 0x74, 0x21, 0x21,
};

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

/* A slot holding `name` with `psk`. An empty name means "the default channel
 * of whatever preset is active", which is what stock stores. */
static void set_slot(uint8_t index, meshtastic_Channel_Role role, const char *name,
		     const uint8_t *psk, size_t psk_len)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.index = index;
	ch.role = role;
	ch.has_settings = true;
	strncpy(ch.settings.name, name, sizeof(ch.settings.name) - 1U);
	ch.settings.psk.size = (pb_size_t)psk_len;
	memcpy(ch.settings.psk.bytes, psk, psk_len);
	zassert_ok(meshtastic_channels_set_slot(index, &ch), "set_slot %u failed", index);
}

static void set_default_slot(uint8_t index, meshtastic_Channel_Role role, const char *name,
			     uint8_t shorthand)
{
	const uint8_t psk[1] = { shorthand };

	set_slot(index, role, name, psk, sizeof(psk));
}

static void disable_slot(uint8_t index)
{
	meshtastic_Channel off = meshtastic_Channel_init_zero;

	off.index = index;
	off.role = meshtastic_Channel_Role_DISABLED;
	zassert_ok(meshtastic_channels_set_slot(index, &off), "disable slot %u failed", index);
}

static void switch_preset(meshtastic_Config_LoRaConfig_ModemPreset preset)
{
	zassert_ok(meshtastic_preset_switch(preset, NULL), "preset switch to %d failed",
		   (int)preset);
}

/* A text broadcast from FAR_NODE_ID, encrypted and hashed with OUR channel in
 * `index` as it stands right now. Called after switching this node to the far
 * tier's preset, it produces exactly the frame a far-tier node would send:
 * the build path resolves the channel's name (and so its hash) against the
 * active preset. */
static void build_far_text(uint8_t index, uint32_t id, const char *text, uint8_t *wire,
			   uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from          = FAR_NODE_ID,
		.to            = MESHTASTIC_NODE_BROADCAST,
		.id            = id,
		.portnum       = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload       = (const uint8_t *)text,
		.payload_len   = strlen(text),
		.hop_limit     = 3U,
		.hop_start     = 3U,
		.channel_index = index,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len),
		   "build_wire_packet failed");
}

/* The ear forwards a frame over the BLE peer link. */
static void ear_forwards(const uint8_t *wire, uint32_t wire_len)
{
	zassert_ok(meshtastic_radio_rx_inject(wire, (uint16_t)wire_len,
					      MESHTASTIC_BEARER_BLE_PEER),
		   "bearer inject failed");
}

/* No TX on our air may carry the injected (src,id): the link-local rule. What
 * we originate ourselves (a NodeInfo request to an unknown sender) is allowed. */
static void assert_not_relayed(uint32_t id)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(300)) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		zassert_false(sys_le32_to_cpu(h->src) == FAR_NODE_ID &&
				      sys_le32_to_cpu(h->id) == id,
			      "the far tier's frame was relayed onto our air");
	}
}

static void *relay_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev     = lora_dev,
		.node_id      = TEST_NODE_ID,
		.psk          = meshtastic_default_psk,
		.psk_len      = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency    = MESHTASTIC_FREQ_US,
	};

	k_sem_init(&rx.sem, 0, 1);
	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	meshtastic_set_recv_cb(on_recv);
	return NULL;
}

static void relay_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* The receiving half, as on the bench: MediumFast, on that preset's
	 * default channel, nothing in the spare slot. */
	set_default_slot(0U, meshtastic_Channel_Role_PRIMARY, "", 0x01U);
	disable_slot(SPARE_SLOT);
	switch_preset(PRESET_MF);

	meshtastic_sched_defaults();
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	lora_sim_reset(lora_dev);
	memset(&rx, 0, sizeof(rx));
	k_sem_init(&rx.sem, 0, 1);
}

static void relay_after(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Leave no far-tier state behind: the next test builds its own. */
	disable_slot(SPARE_SLOT);
	lora_sim_reset(lora_dev);
}

/* H1's premise. A default channel is named after its preset, so its hash
 * changes with the preset even though the key is the same. The three values
 * are the bench's: LongFast 0x08, MediumFast 0x1f, ShortTurbo 0x0e. */
ZTEST(relay, test_default_channel_hash_follows_preset)
{
	switch_preset(PRESET_LF);
	zassert_equal(meshtastic_channels_get_hash(0U), 0x08U, "LongFast default hash");
	switch_preset(PRESET_MF);
	zassert_equal(meshtastic_channels_get_hash(0U), 0x1fU, "MediumFast default hash");
	switch_preset(PRESET_ST);
	zassert_equal(meshtastic_channels_get_hash(0U), 0x0eU, "ShortTurbo default hash");
}

/* The custom-channel premise. The hash is xor(name) ^ xor(key), and a stored
 * name does not depend on the preset, so the same channel is the same channel
 * on every tier. */
ZTEST(relay, test_named_channel_hash_ignores_preset)
{
	uint8_t at_mf;

	set_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY, "pugs-are-great", custom_psk,
		 sizeof(custom_psk));
	at_mf = meshtastic_channels_get_hash(SPARE_SLOT);

	switch_preset(PRESET_LF);
	zassert_equal(meshtastic_channels_get_hash(SPARE_SLOT), at_mf,
		      "a named channel's hash moved with the preset (LongFast)");
	switch_preset(PRESET_ST);
	zassert_equal(meshtastic_channels_get_hash(SPARE_SLOT), at_mf,
		      "a named channel's hash moved with the preset (ShortTurbo)");
}

/* H1: a LongFast default frame, forwarded verbatim to a MediumFast node that
 * holds only its own default channel, is not understood. Nothing reaches the
 * app. */
ZTEST(relay, test_far_default_frame_dropped_without_spare_slot)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	switch_preset(PRESET_LF);
	build_far_text(0U, 0x5101U, "hello from LongFast", wire, &wire_len);
	switch_preset(PRESET_MF);

	ear_forwards(wire, wire_len);
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN,
		      "a LongFast default frame was delivered on a MediumFast node with no "
		      "LongFast channel");
	assert_not_relayed(0x5101U);
}

/* H29: the same frame IS understood once the spare slot holds the LongFast
 * default channel, named explicitly so its hash does not follow our preset.
 * This is all the public-class relay needs to read the ear's frames. The
 * link-local rule still keeps the frame off our air: re-sending it is the
 * relay's job, as a new packet, not the router's. */
ZTEST(relay, test_far_default_frame_read_through_spare_slot)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	const char *msg = "hello from LongFast";

	switch_preset(PRESET_LF);
	build_far_text(0U, 0x5102U, msg, wire, &wire_len);
	switch_preset(PRESET_MF);

	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x01U);
	zassert_equal(meshtastic_channels_get_hash(SPARE_SLOT), 0x08U,
		      "the spare slot must hash as LongFast while we sit on MediumFast");

	ear_forwards(wire, wire_len);
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "the far frame was not delivered");
	zassert_equal(rx.from, FAR_NODE_ID, "wrong sender");
	zassert_equal(rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(rx.payload_len, strlen(msg), "wrong length");
	zassert_mem_equal(rx.payload, msg, strlen(msg), "wrong text");
	assert_not_relayed(0x5102U);
}

/* The custom-channel case needs no extra slot: a frame sent on
 * "pugs-are-great" by a LongFast node is read by a MediumFast node holding the
 * same channel for its own tier. */
ZTEST(relay, test_custom_channel_frame_crosses_presets)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	const char *msg = "pugs on LongFast";

	set_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY, "pugs-are-great", custom_psk,
		 sizeof(custom_psk));

	switch_preset(PRESET_LF);
	build_far_text(SPARE_SLOT, 0x5103U, msg, wire, &wire_len);
	switch_preset(PRESET_MF);

	ear_forwards(wire, wire_len);
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "the custom-channel frame was not delivered");
	zassert_equal(rx.from, FAR_NODE_ID, "wrong sender");
	zassert_mem_equal(rx.payload, msg, strlen(msg), "wrong text");
	assert_not_relayed(0x5103U);
}

/* Why neither existing predicate defines the relay's public class ("a
 * preset-named channel on exactly the default key"):
 *   - is_well_known() also takes the simple2..9 shorthands, which are
 *     different keys and so different meshes;
 *   - is_default() insists on the ACTIVE preset's name, so a LongFast channel
 *     held on a MediumFast node is not "default" to it.
 * The relay needs its own predicate; this pins the gap it fills. */
ZTEST(relay, test_existing_predicates_do_not_define_public_class)
{
	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x01U);
	zassert_true(meshtastic_channels_is_well_known(SPARE_SLOT),
		     "LongFast on the default key is well known");
	zassert_false(meshtastic_channels_is_default(SPARE_SLOT),
		      "is_default only matches the active preset's name");

	/* simple3: the default key with its last byte bumped. Another mesh. */
	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x04U);
	zassert_true(meshtastic_channels_is_well_known(SPARE_SLOT),
		     "is_well_known takes the simple keys too");
	zassert_not_equal(meshtastic_channels_get_hash(SPARE_SLOT), 0x08U,
			  "a simple key must hash apart from the default key");
}

ZTEST_SUITE(relay, NULL, relay_setup, relay_before, relay_after, NULL);
