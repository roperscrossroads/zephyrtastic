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
 * The second half of the file tests the relay itself (meshtastic_relay.c):
 * the R-cases of the tooling repo's docs/RELAY-TESTBED.md, which landed with
 * the code, not after it. The premise tests run with the relay OFF (its boot
 * state), so they still pin the router's behaviour, not the relay's.
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
#include "meshtastic_relay.h"
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

/* A packet from @p from, encrypted and hashed with OUR channel in `index` as
 * it stands right now. Called after switching this node to the far tier's
 * preset, it produces exactly the frame a far-tier node would send: the build
 * path resolves the channel's name (and so its hash) against the active
 * preset. */
static void build_far(uint8_t index, uint32_t from, uint32_t to, uint32_t portnum, uint32_t id,
		      const uint8_t *payload, size_t payload_len, uint8_t *wire, uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from          = from,
		.to            = to,
		.id            = id,
		.portnum       = portnum,
		.payload       = payload,
		.payload_len   = payload_len,
		.hop_limit     = 3U,
		.hop_start     = 3U,
		.channel_index = index,
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len),
		   "build_wire_packet failed");
}

/* A text broadcast from FAR_NODE_ID. */
static void build_far_text(uint8_t index, uint32_t id, const char *text, uint8_t *wire,
			   uint32_t *wire_len)
{
	build_far(index, FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_TEXT_MESSAGE, id,
		  (const uint8_t *)text, strlen(text), wire, wire_len);
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

	meshtastic_relay_reset();
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

/* H30 in the translating mode: the channel hash is one byte, so a stranger's
 * channel can collide with the one we read the far tier through. Here the
 * stranger holds "LongFast" with the default key's bytes 0 and 1 swapped: the
 * XOR of the key is unchanged, so the hash is 0x08 exactly like the public
 * LongFast channel, but the key is different. Its frame, forwarded by the ear,
 * must not surface as a message: decrypting with the wrong key yields bytes
 * that must fail to parse as Data. AES-CTR has no MAC, so the parse is the only
 * admission test the relay inherits from the router.
 *
 * One frame is one sample. A throwaway probe of 64 colliding frames (different
 * ids and texts) delivered none, which bounds the admission rate below ~5% at
 * 95% confidence (rule of three), not at zero. A relay that re-originates what
 * it admits multiplies any leak onto a second tier, so the relay may want its
 * own sanity check on top (e.g. valid UTF-8 text). */
ZTEST(relay, test_colliding_foreign_channel_not_admitted)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint8_t stranger_psk[16];

	memcpy(stranger_psk, meshtastic_default_psk, sizeof(stranger_psk));
	stranger_psk[0] = meshtastic_default_psk[1];
	stranger_psk[1] = meshtastic_default_psk[0];
	zassert_false(memcmp(stranger_psk, meshtastic_default_psk, sizeof(stranger_psk)) == 0,
		      "the stranger's key must differ from the default key");

	set_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY, MESHTASTIC_CHANNEL_LONGFAST,
		 stranger_psk, sizeof(stranger_psk));
	zassert_equal(meshtastic_channels_get_hash(SPARE_SLOT), 0x08U,
		      "the stranger's channel must collide with LongFast's hash");
	build_far_text(SPARE_SLOT, 0x5104U, "not your channel", wire, &wire_len);

	/* Now we hold the real public LongFast channel in that slot. */
	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x01U);
	zassert_equal(meshtastic_channels_get_hash(SPARE_SLOT), 0x08U, "same hash");

	ear_forwards(wire, wire_len);
	zassert_equal(k_sem_take(&rx.sem, K_MSEC(300)), -EAGAIN,
		      "a colliding stranger's frame surfaced as a message");
	assert_not_relayed(0x5104U);
}

/* ==========================================================================
 * The relay itself (meshtastic_relay.c). R-numbers are RELAY-TESTBED.md's.
 * ========================================================================== */

/* FAR_NODE_ID's low 16 bits, as the prefix spells them. */
#define FAR_PREFIX "[0d0d] "

struct relayed {
	struct meshtastic_packet pkt;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t hash;
	uint8_t flags;
	uint32_t id;
};

/* The next text WE put on the air, skipping what else we originate (a NodeInfo
 * request to an unknown sender). Fails the test on any TX still carrying a far
 * node's (src,id): that would be the frame itself relayed, not re-originated. */
static int take_relayed(struct relayed *out, k_timeout_t wait)
{
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, wait) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;

		zassert_not_equal(sys_le32_to_cpu(h->src), FAR_NODE_ID,
				  "a far node's frame reached our air verbatim");
		if (sys_le32_to_cpu(h->src) != TEST_NODE_ID) {
			continue;
		}
		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &out->pkt, out->payload,
						  sizeof(out->payload)) != 0 ||
		    out->pkt.portnum != MESHTASTIC_PORT_TEXT_MESSAGE) {
			continue;
		}
		out->hash = h->channel;
		out->flags = h->flags;
		out->id = sys_le32_to_cpu(h->id);
		return 0;
	}
	return -EAGAIN;
}

static void assert_nothing_relayed(void)
{
	struct relayed r = {0};

	zassert_equal(take_relayed(&r, K_MSEC(300)), -EAGAIN,
		      "unexpected relayed text \"%.*s\"", (int)r.pkt.payload_len, r.payload);
}

static void assert_relayed_text(const struct relayed *r, const char *want)
{
	zassert_equal(r->pkt.payload_len, strlen(want), "relayed length %u, want %u (\"%.*s\")",
		      (unsigned)r->pkt.payload_len, (unsigned)strlen(want),
		      (int)r->pkt.payload_len, r->payload);
	zassert_mem_equal(r->payload, want, strlen(want), "relayed text");
}

static struct meshtastic_relay_stats stats(void)
{
	struct meshtastic_relay_stats st;

	meshtastic_relay_stats_get(&st);
	return st;
}

/* The public class, as on the bench: LongFast default read through the spare
 * slot. Built on LongFast, delivered on MediumFast. */
static void public_setup_and_build(uint32_t id, const char *text, uint8_t *wire,
				   uint32_t *wire_len)
{
	switch_preset(PRESET_LF);
	build_far_text(0U, id, text, wire, wire_len);
	switch_preset(PRESET_MF);
	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x01U);
}

static void relay_inbound(void)
{
	zassert_ok(meshtastic_relay_set_direction(MESHTASTIC_RELAY_INBOUND));
}

/* R8b: the relay is off until someone turns it on. Both directions arm the
 * loop cases against themselves, so the default is a behaviour, not a doc. */
ZTEST(relay, test_r8b_default_is_off)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_equal(meshtastic_relay_get_direction(), MESHTASTIC_RELAY_OFF,
		      "the relay must boot off");
	public_setup_and_build(0x5201U, "hello", wire, &wire_len);
	ear_forwards(wire, wire_len);
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "not delivered locally");
	assert_nothing_relayed();
	zassert_equal(stats().dir_off, 1U, "dir_off not counted");
}

/* R9 (v1): only inbound exists. Outbound needs an ear that transmits. */
ZTEST(relay, test_r9_outbound_refused_in_v1)
{
	zassert_equal(meshtastic_relay_set_direction(MESHTASTIC_RELAY_OUTBOUND), -ENOTSUP);
	zassert_equal(meshtastic_relay_set_direction(MESHTASTIC_RELAY_BOTH), -ENOTSUP);
	zassert_equal(meshtastic_relay_get_direction(), MESHTASTIC_RELAY_OFF,
		      "a refused direction must not stick");
}

/* R1 + R19 + R20: LongFast public text leaves on our MediumFast public
 * channel as a NEW packet from us, prefixed with the origin, as a fresh packet
 * (hop_start = hop_limit), and the far frame itself never reaches our air. */
ZTEST(relay, test_r1_public_text_reoriginated)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;
	uint8_t hop_limit;
	uint8_t hop_start;

	public_setup_and_build(0x5202U, "Hello World!", wire, &wire_len);
	relay_inbound();
	ear_forwards(wire, wire_len);

	zassert_ok(take_relayed(&r, K_MSEC(1000)), "nothing relayed");
	assert_relayed_text(&r, FAR_PREFIX "Hello World!");
	zassert_equal(r.pkt.from, TEST_NODE_ID, "the relay must be the author");
	zassert_not_equal(r.id, 0x5202U, "a re-originated packet has its own id");
	zassert_equal(r.pkt.to, MESHTASTIC_NODE_BROADCAST, "broadcast");
	zassert_equal(r.hash, 0x1fU, "must go out on MediumFast's public channel");
	hop_limit = r.flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK;
	hop_start = (r.flags & MESHTASTIC_FLAGS_HOP_START_MASK) >> MESHTASTIC_FLAGS_HOP_START_SHIFT;
	zassert_equal(hop_start, hop_limit, "a fresh packet starts at its own hop limit");
	zassert_equal(stats().relayed, 1U);
	zassert_equal(stats().sent, 1U);
}

/* R2: a custom channel leaves on the same channel, which hashes the same here. */
ZTEST(relay, test_r2_custom_channel_same_channel)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;

	set_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY, "pugs-are-great", custom_psk,
		 sizeof(custom_psk));
	switch_preset(PRESET_LF);
	build_far_text(SPARE_SLOT, 0x5203U, "pugs", wire, &wire_len);
	switch_preset(PRESET_MF);

	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(take_relayed(&r, K_MSEC(1000)), "nothing relayed");
	assert_relayed_text(&r, FAR_PREFIX "pugs");
	zassert_equal(r.hash, meshtastic_channels_get_hash(SPARE_SLOT),
		      "must go out on the same custom channel");
}

/* R3: the public class needs a public channel on THIS preset to land on. A
 * node whose only own channel is custom has none. */
ZTEST(relay, test_r3_no_mapping_without_local_public_channel)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	public_setup_and_build(0x5204U, "nowhere to go", wire, &wire_len);
	set_slot(0U, meshtastic_Channel_Role_PRIMARY, "private-primary", custom_psk,
		 sizeof(custom_psk));
	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "not delivered locally");
	assert_nothing_relayed();
	zassert_equal(stats().no_mapping, 1U, "no_mapping not counted");
}

/* R4: LongFast on a simple key is another public mesh: not relayed. */
ZTEST(relay, test_r4_simple_key_not_public_class)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x04U);
	build_far_text(SPARE_SLOT, 0x5205U, "simple3 mesh", wire, &wire_len);
	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "not delivered locally");
	assert_nothing_relayed();
	zassert_equal(stats().no_mapping, 1U, "no_mapping not counted");
}

/* R5 + R7: only text crosses. A position in the relay's name would place the
 * relay at the origin; port 256 is the cluster's. */
ZTEST(relay, test_r5_r7_only_text_crosses)
{
	static const uint8_t junk[] = { 0x08, 0x01 };
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	public_setup_and_build(0x5206U, "unused", wire, &wire_len);
	relay_inbound();

	switch_preset(PRESET_LF);
	set_default_slot(0U, meshtastic_Channel_Role_PRIMARY, "", 0x01U);
	build_far(0U, FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_POSITION, 0x5207U,
		  junk, sizeof(junk), wire, &wire_len);
	switch_preset(PRESET_MF);
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();

	switch_preset(PRESET_LF);
	build_far(0U, FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_PRIVATE, 0x5208U,
		  junk, sizeof(junk), wire, &wire_len);
	switch_preset(PRESET_MF);
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();
	zassert_equal(stats().not_text, 2U, "not_text not counted");
}

/* R6: a DM never crosses, even one we can read. */
ZTEST(relay, test_r6_dm_never_crosses)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	const char *msg = "just for you";

	public_setup_and_build(0x5209U, "unused", wire, &wire_len);
	switch_preset(PRESET_LF);
	build_far(0U, FAR_NODE_ID, TEST_NODE_ID, MESHTASTIC_PORT_TEXT_MESSAGE, 0x520AU,
		  (const uint8_t *)msg, strlen(msg), wire, &wire_len);
	switch_preset(PRESET_MF);
	relay_inbound();
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();
	zassert_equal(stats().not_broadcast, 1U, "not_broadcast not counted");
}

/* Frames heard on OUR air are never relayed in v1 (that is outbound). */
ZTEST(relay, test_own_air_never_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	relay_inbound();
	build_far_text(0U, 0x520BU, "said on MediumFast", wire, &wire_len);
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
	zassert_ok(k_sem_take(&rx.sem, K_MSEC(1000)), "not delivered");

	/* The router may flood-relay it (it keeps FAR's src, which take_relayed
	 * would flag): drain those, then make sure no text of OURS followed. */
	struct lora_sim_frame f;

	while (lora_sim_take_tx(lora_dev, &f, K_MSEC(300)) == 0) {
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)f.data;
		struct meshtastic_packet p;
		uint8_t pl[MESHTASTIC_MAX_PAYLOAD_LEN];

		if (sys_le32_to_cpu(h->src) == TEST_NODE_ID &&
		    meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &p, pl, sizeof(pl)) == 0) {
			zassert_not_equal(p.portnum, MESHTASTIC_PORT_TEXT_MESSAGE,
					  "a text heard on our own air was re-originated");
		}
	}
	zassert_equal(stats().considered, 0U, "only ear frames are considered");
}

/* R12 + R13 (H4): the same (origin, text) under a NEW packet id is relayed
 * once inside the TTL, and again after it. */
ZTEST(relay, test_r12_r13_seen_cache_and_ttl)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;

	public_setup_and_build(0x520CU, "same words", wire, &wire_len);
	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(take_relayed(&r, K_MSEC(1000)), "first copy not relayed");

	switch_preset(PRESET_LF);
	build_far_text(0U, 0x520DU, "same words", wire, &wire_len);
	switch_preset(PRESET_MF);
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();
	zassert_equal(stats().seen, 1U, "seen not counted");

	k_sleep(K_SECONDS(CONFIG_MESHTASTIC_RELAY_SEEN_TTL_SEC + 1));
	switch_preset(PRESET_LF);
	build_far_text(0U, 0x520EU, "same words", wire, &wire_len);
	switch_preset(PRESET_MF);
	ear_forwards(wire, wire_len);
	zassert_ok(take_relayed(&r, K_MSEC(1000)), "not relayed again after the TTL");
	assert_relayed_text(&r, FAR_PREFIX "same words");
}

/* R14 (H26b): ANY relay-style prefix is refused, not only ours, so a relay
 * nobody listed cannot loop with us. */
ZTEST(relay, test_r14_any_relay_prefix_refused)
{
	static const char *const prefixed[] = {
		"[c3d4] Hello",         /* our format, someone else's relay */
		"[rly-2] Hello",        /* another implementation's */
		"[0a0a] [0d0d] Hello",  /* a double hop */
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	public_setup_and_build(0x5210U, "unused", wire, &wire_len);
	relay_inbound();
	for (size_t i = 0U; i < ARRAY_SIZE(prefixed); i++) {
		switch_preset(PRESET_LF);
		build_far_text(0U, 0x5211U + i, prefixed[i], wire, &wire_len);
		switch_preset(PRESET_MF);
		ear_forwards(wire, wire_len);
		assert_nothing_relayed();
	}
	zassert_equal(stats().prefixed, ARRAY_SIZE(prefixed), "prefixed not counted");
}

/* The prefix test itself, at the edges. */
ZTEST(relay, test_prefix_recogniser)
{
	struct {
		const char *s;
		bool prefix;
	} cases[] = {
		{ "[abcd] hi", true },   { "[a] hi", true },       { "[12345678] hi", true },
		{ "[123456789] hi", false }, { "[] hi", false },   { "[ab cd] hi", false },
		{ "[abcd]hi", false },   { "abcd] hi", false },    { "[abcd", false },
		{ "hi [abcd] ", false },
	};

	for (size_t i = 0U; i < ARRAY_SIZE(cases); i++) {
		zassert_equal(meshtastic_relay_has_prefix((const uint8_t *)cases[i].s,
							  strlen(cases[i].s)),
			      cases[i].prefix, "\"%s\"", cases[i].s);
	}
}

/* R15: a listed relay id is never translated, even unprefixed. */
ZTEST(relay, test_r15_ignore_list)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	public_setup_and_build(0x5220U, "from a known relay", wire, &wire_len);
	zassert_ok(meshtastic_relay_ignore_add(FAR_NODE_ID));
	relay_inbound();
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();
	zassert_equal(stats().ignored, 1U, "ignored not counted");
}

/* R22 (H26): convergence. We relay FAR's "Hello" once. A second relay on the
 * same tiers then carries our output back to the ear's tier, with its own
 * prefix added or with ours kept, and the ear hears it. Either way exactly one
 * copy of "Hello" ever reaches our tier.
 *
 * Limit: a foreign relay that STRIPS prefixes and re-sends under its own id
 * would pass the prefix test and the seen-cache (new origin). The ignore list
 * (R15) is the answer for a relay you know; one you don't is why the rate cap
 * exists. */
ZTEST(relay, test_r22_two_relays_converge)
{
	static const char *const echoes[] = {
		"[0a0a] " FAR_PREFIX "Hello", /* relay C added its prefix */
		FAR_PREFIX "Hello",           /* relay C copied our text verbatim */
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;

	public_setup_and_build(0x5230U, "Hello", wire, &wire_len);
	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(take_relayed(&r, K_MSEC(1000)), "first copy not relayed");

	for (size_t i = 0U; i < ARRAY_SIZE(echoes); i++) {
		switch_preset(PRESET_LF);
		build_far(0U, 0x0A0B0A0AU, MESHTASTIC_NODE_BROADCAST,
			  MESHTASTIC_PORT_TEXT_MESSAGE, 0x5231U + i, (const uint8_t *)echoes[i],
			  strlen(echoes[i]), wire, &wire_len);
		switch_preset(PRESET_MF);
		ear_forwards(wire, wire_len);
		assert_nothing_relayed();
	}
	zassert_equal(stats().relayed, 1U, "exactly one copy must cross");
}

/* R17: a text too long for the prefix goes without it, never truncated. */
ZTEST(relay, test_r17_long_text_unprefixed_not_truncated)
{
	char text[MESHTASTIC_MAX_TEXT_LEN - 3U + 1U];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;

	memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	public_setup_and_build(0x5240U, text, wire, &wire_len);
	relay_inbound();
	ear_forwards(wire, wire_len);
	zassert_ok(take_relayed(&r, K_MSEC(1000)), "long text not relayed");
	assert_relayed_text(&r, text);
	zassert_equal(stats().unprefixed, 1U, "unprefixed not counted");
}

/* R18: the per-direction cap (3 in this suite's prj.conf). */
ZTEST(relay, test_r18_rate_cap)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct relayed r;
	char text[16];
	int got = 0;

	public_setup_and_build(0x5250U, "unused", wire, &wire_len);
	relay_inbound();
	for (int i = 0; i <= CONFIG_MESHTASTIC_RELAY_RATE_MAX; i++) {
		snprintk(text, sizeof(text), "msg %d", i);
		switch_preset(PRESET_LF);
		build_far_text(0U, 0x5251U + i, text, wire, &wire_len);
		switch_preset(PRESET_MF);
		ear_forwards(wire, wire_len);
		if (take_relayed(&r, K_MSEC(1000)) == 0) {
			got++;
		}
	}
	zassert_equal(got, CONFIG_MESHTASTIC_RELAY_RATE_MAX, "cap not applied");
	zassert_equal(stats().rate_dropped, 1U, "rate_dropped not counted");
}

/* R21 (H30): a colliding stranger's frame is not re-originated, and text
 * that is not valid UTF-8 (what a wrong-key decrypt that happened to parse
 * would look like) is refused by the relay's own check. */
ZTEST(relay, test_r21_admission)
{
	static const uint8_t bad[] = { 'o', 'k', 0xC3, 0x28 }; /* broken UTF-8 */
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	uint8_t stranger_psk[16];

	memcpy(stranger_psk, meshtastic_default_psk, sizeof(stranger_psk));
	stranger_psk[0] = meshtastic_default_psk[1];
	stranger_psk[1] = meshtastic_default_psk[0];
	set_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY, MESHTASTIC_CHANNEL_LONGFAST,
		 stranger_psk, sizeof(stranger_psk));
	build_far_text(SPARE_SLOT, 0x5260U, "not your channel", wire, &wire_len);
	set_default_slot(SPARE_SLOT, meshtastic_Channel_Role_SECONDARY,
			 MESHTASTIC_CHANNEL_LONGFAST, 0x01U);
	relay_inbound();
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();

	switch_preset(PRESET_LF);
	build_far(0U, FAR_NODE_ID, MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_TEXT_MESSAGE,
		  0x5261U, bad, sizeof(bad), wire, &wire_len);
	switch_preset(PRESET_MF);
	ear_forwards(wire, wire_len);
	assert_nothing_relayed();
	zassert_equal(stats().bad_text, 1U, "bad_text not counted");
}

ZTEST_SUITE(relay, NULL, relay_setup, relay_before, relay_after, NULL);
