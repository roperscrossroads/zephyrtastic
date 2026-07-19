/* Known-answer vector tests against upstream Meshtastic firmware.
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Why this suite exists
 * ---------------------
 * Every other crypto/wire test in this tree is a self-loopback: it builds a
 * frame with the same code that parses it. That structurally cannot catch a
 * *symmetric* error -- swap a nonce field on both sides and the suite stays
 * green while the node is mute to every stock radio on the mesh.
 *
 * The vectors in meshtastic_vectors.h were produced by compiling and running
 * upstream's own functions (see tools/vectors/harvest.py). Asserting against
 * them is the only check here that can fail when we are consistently wrong.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_contention.h"
#include "meshtastic_sched.h"
#include "meshtastic_region_presets.h"
#include "vectors/meshtastic_vectors.h"

/* Vector labels whose PSK is a full-length key, i.e. one the channel layer
 * stores verbatim rather than expanding. Single-byte "short PSK" indices are
 * expanded to a full default key before hashing (both here and upstream), so
 * the raw-byte hash in the vector table is a component, not the channel hash.
 */
static const struct {
	const char *label;
	uint8_t bytes[32];
	uint8_t len;
} full_psks[] = {
	{ "aes128_zero",     { 0 }, 16 },
	{ "aes128_ff",       { [0 ... 15] = 0xFF }, 16 },
	{ "aes128_counting", { 0, 1, 2, 3, 4, 5, 6, 7,
			       8, 9, 10, 11, 12, 13, 14, 15 }, 16 },
	{ "aes256_counting", { 0, 1, 2, 3, 4, 5, 6, 7,
			       8, 9, 10, 11, 12, 13, 14, 15,
			       16, 17, 18, 19, 20, 21, 22, 23,
			       24, 25, 26, 27, 28, 29, 30, 31 }, 32 },
};

static uint8_t vec_name_hash(const char *name, bool *found)
{
	*found = false;
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_name_hashes); i++) {
		if (strcmp(mt_vec_name_hashes[i].name, name) == 0) {
			*found = true;
			return mt_vec_name_hashes[i].hash;
		}
	}
	return 0U;
}

static uint8_t vec_psk_hash(const char *label, bool *found)
{
	*found = false;
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_psk_hashes); i++) {
		if (strcmp(mt_vec_psk_hashes[i].label, label) == 0) {
			*found = true;
			return mt_vec_psk_hashes[i].hash;
		}
	}
	return 0U;
}

/* Install a channel in slot 0 with the given name and full-length PSK. */
static void set_primary(const char *name, const uint8_t *psk, uint8_t psk_len)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	strncpy(ch.settings.name, name, sizeof(ch.settings.name) - 1U);
	ch.settings.psk.size = psk_len;
	memcpy(ch.settings.psk.bytes, psk, psk_len);

	zassert_ok(meshtastic_channels_set_slot(0, &ch),
		   "set_slot failed for name=\"%s\"", name);
}

static void vectors_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_channels_init_defaults();
	/* The contention vectors assert against the reference constants, which are
	 * the *default* policy rather than fixed values. Restore it per test so a
	 * test that retunes the window (or one that fails partway through doing so)
	 * cannot silently rewrite what the next one is measuring. */
	meshtastic_sched_defaults();
}

/* The interop check that matters: our channel hash must equal
 * xorHash(name) ^ xorHash(psk) as computed by stock firmware, for every
 * channel name -- not just the default. A mismatch means stock radios drop
 * our traffic with no error anywhere.
 */
ZTEST(wire_vectors, test_channel_hash_matches_upstream)
{
	unsigned checked = 0;

	for (unsigned n = 0; n < MT_VEC_COUNT(mt_vec_name_hashes); n++) {
		const char *name = mt_vec_name_hashes[n].name;

		/* The empty name is not a real channel name -- it triggers the
		 * default-name substitution, covered separately below. */
		if (name[0] == '\0') {
			continue;
		}
		/* Names are stored in a fixed-size field; skip any vector that
		 * would truncate, since a truncated name hashes differently. */
		if (strlen(name) >= sizeof(((meshtastic_ChannelSettings *)0)->name)) {
			continue;
		}

		for (unsigned p = 0; p < ARRAY_SIZE(full_psks); p++) {
			bool have_n, have_p;
			uint8_t nh = vec_name_hash(name, &have_n);
			uint8_t ph = vec_psk_hash(full_psks[p].label, &have_p);

			zassert_true(have_n && have_p,
				     "vector table missing an entry for %s/%s",
				     name, full_psks[p].label);

			set_primary(name, full_psks[p].bytes, full_psks[p].len);

			zassert_equal(meshtastic_channels_get_hash(0),
				      (uint8_t)(nh ^ ph),
				      "channel hash diverges from stock for "
				      "name=\"%s\" psk=%s: got 0x%02x want 0x%02x",
				      name, full_psks[p].label,
				      meshtastic_channels_get_hash(0),
				      (uint8_t)(nh ^ ph));
			checked++;
		}
	}

	zassert_true(checked > 0, "no name/psk combinations were exercised");
	TC_PRINT("verified %u name/psk channel-hash combinations\n", checked);
}

/* An empty channel name must resolve to the ACTIVE preset's display name
 * (parity: crypto #1).
 *
 * The substituted string is protocol data, not a label: it is hashed for the
 * channel byte and again for the frequency slot. Pinning it to "LongFast"
 * was correct only while the modem was frozen there; now that a node can run
 * MediumFast, a pinned name produces both the wrong channel hash and the
 * wrong frequency, and stock radios simply never hear it.
 */
ZTEST(wire_vectors, test_empty_name_follows_the_active_preset)
{
	bool found;
	uint8_t psk_h = vec_psk_hash("aes128_counting", &found);
	uint8_t lf = vec_name_hash("LongFast", &found);
	uint8_t mf = vec_name_hash("MediumFast", &found);

	zassert_true(found, "vector table missing a needed name hash");

	/* Default preset: the unnamed channel hashes as "LongFast". */
	mt.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	mt.use_preset = true;
	set_primary("", full_psks[2].bytes, full_psks[2].len);
	zassert_str_equal(meshtastic_channels_get_name(0), "LongFast",
			  "unnamed channel should take the LongFast display name");
	zassert_equal(meshtastic_channels_get_hash(0), (uint8_t)(lf ^ psk_h),
		      "unnamed channel hash should match upstream's LongFast");

	/* Switch preset: the same unnamed channel must now hash as "MediumFast". */
	mt.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST;
	set_primary("", full_psks[2].bytes, full_psks[2].len);
	zassert_str_equal(meshtastic_channels_get_name(0), "MediumFast",
			  "unnamed channel must follow the preset, not stay on LongFast");
	zassert_equal(meshtastic_channels_get_hash(0), (uint8_t)(mf ^ psk_h),
		      "unnamed channel hash must follow the preset");

	/* use_preset=false is its own literal upstream. */
	mt.use_preset = false;
	set_primary("", full_psks[2].bytes, full_psks[2].len);
	zassert_str_equal(meshtastic_channels_get_name(0), "Custom",
			  "a custom modem config hashes as \"Custom\"");

	mt.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	mt.use_preset = true;
}

/* Changing the preset at runtime must rehash every channel (parity: crypto #1).
 *
 * Channel hashes are cached when a slot is stored, and an unnamed channel's
 * name now depends on the active preset -- so a cached hash could in principle
 * go stale the moment the preset changes. It does not, because every config
 * write funnels through apply_core(), which applies the preset and then
 * re-stores all channels in that order.
 *
 * This asserts that funnel actually holds, rather than trusting the ordering
 * comment in apply_core(): a stale hash would put the wrong channel byte on
 * every outgoing frame, and nothing else in the suite would notice.
 */
ZTEST(wire_vectors, test_preset_change_rehashes_channels)
{
	meshtastic_Config lora = meshtastic_Config_init_zero;
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	bool found;
	uint8_t psk_h = vec_psk_hash("aes128_counting", &found);
	uint8_t lf = vec_name_hash("LongFast", &found);
	uint8_t mf = vec_name_hash("MediumFast", &found);
	uint8_t before, after;

	zassert_true(found, "vector table missing a needed hash");

	/* Install the unnamed primary through the CONFIG STORE, not through
	 * meshtastic_channels_set_slot(). apply_core() re-stores every channel
	 * from the store, so a slot written straight to the channels module is
	 * silently reverted the next time any config changes -- which is exactly
	 * what this test needs to exercise the real admin path.
	 */
	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	ch.settings.name[0] = '\0';
	ch.settings.psk.size = full_psks[2].len;
	memcpy(ch.settings.psk.bytes, full_psks[2].bytes, full_psks[2].len);
	zassert_ok(meshtastic_config_store_set_channel(0, &ch), "set_channel failed");

	before = meshtastic_channels_get_hash(0);
	zassert_equal(before, (uint8_t)(lf ^ psk_h),
		      "unnamed channel should start on the LongFast hash: got 0x%02x "
		      "want 0x%02x", before, (uint8_t)(lf ^ psk_h));

	/* Switch preset the way an admin would, through the config store. */
	lora.which_payload_variant = meshtastic_Config_lora_tag;
	lora.payload_variant.lora.use_preset = true;
	lora.payload_variant.lora.modem_preset =
		meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST;
	lora.payload_variant.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
	lora.payload_variant.lora.hop_limit = 3;
	zassert_ok(meshtastic_config_store_set_config(&lora), "set_config(lora) failed");

	after = meshtastic_channels_get_hash(0);
	zassert_not_equal(after, before,
			  "the cached channel hash did not follow the preset change");
	zassert_equal(after, (uint8_t)(mf ^ psk_h),
		      "after switching to MediumFast the unnamed channel must hash "
		      "as \"MediumFast\": got 0x%02x want 0x%02x", after,
		      (uint8_t)(mf ^ psk_h));
}

/* The frequency must follow the channel NAME, not just the region
 * (parity: radio D3).
 *
 * This is a live bug inside the US region alone: stock firmware puts a node
 * whose primary channel is "MyMesh" on slot djb2("MyMesh") % 104, well away
 * from LongFast's slot 19. Pinning one frequency per region left a renamed
 * channel unable to hear stock radios at all.
 */
ZTEST(wire_vectors, test_frequency_follows_channel_name)
{
	struct meshtastic_freq_plan def, renamed;

	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_US,
			   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			   "LongFast", true, &def),
		   "default plan failed");
	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_US,
			   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			   "MyMesh", true, &renamed),
		   "renamed plan failed");

	zassert_equal(def.frequency_hz, MESHTASTIC_FREQ_US,
		      "the default channel keeps the established frequency");
	zassert_not_equal(renamed.frequency_hz, def.frequency_hz,
			  "a renamed channel must move frequency slot");
	zassert_equal(renamed.slot, 40,
		      "\"MyMesh\" hashes to US slot 40, got %u", renamed.slot);
	zassert_equal(renamed.frequency_hz, 912125000,
		      "\"MyMesh\" belongs on 912.125 MHz, got %u Hz",
		      renamed.frequency_hz);
}

/* Guard on the harvested contract itself.
 *
 * Upstream writes packetId as 8 bytes at offset 0, then -- when extraNonce is
 * non-zero -- writes it at offset 4, ON TOP of packetId's high half. An
 * implementation that appends extraNonce instead produces a nonce that is
 * plausible, self-consistent, and wrong on the wire.
 *
 * We assert the vectors still encode that overlap, so a future re-harvest
 * against a changed upstream fails loudly here rather than silently shifting
 * what "correct" means.
 */
ZTEST(wire_vectors, test_nonce_vectors_encode_the_overlap)
{
	const struct mt_vec_nonce *plain = NULL, *extra = NULL;

	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_nonces); i++) {
		if (strcmp(mt_vec_nonces[i].label, "psk_id1_from_deadbeef") == 0) {
			plain = &mt_vec_nonces[i];
		} else if (strcmp(mt_vec_nonces[i].label, "pkc_extra_aabbccdd") == 0) {
			extra = &mt_vec_nonces[i];
		}
	}
	zassert_not_null(plain, "missing baseline nonce vector");
	zassert_not_null(extra, "missing extra-nonce vector");

	/* Same id and from-node; the only difference is extraNonce. */
	zassert_mem_equal(plain->nonce, extra->nonce, 4,
			  "packet id must occupy bytes 0..3");
	zassert_mem_equal(plain->nonce + 8, extra->nonce + 8, 4,
			  "from-node must occupy bytes 8..11");

	/* extraNonce lands at 4..7, little-endian. */
	static const uint8_t want[4] = { 0xDD, 0xCC, 0xBB, 0xAA };

	zassert_mem_equal(extra->nonce + 4, want, sizeof(want),
			  "extra nonce must be written little-endian at offset 4");

	/* And with no extra nonce those bytes stay clear, because a real packet
	 * id is 32-bit and its high half is zero. */
	static const uint8_t zero[4] = { 0 };

	zassert_mem_equal(plain->nonce + 4, zero, sizeof(zero),
			  "bytes 4..7 must be zero when extraNonce is unused");
}

/* The port's hardcoded default channel name must equal upstream's LONG_FAST
 * display name (parity: crypto #1, radio D3).
 *
 * That string is not decorative: it is hashed to pick the frequency slot, and
 * it substitutes for an empty channel name when computing the channel hash.
 * Hardcoding it is correct only while the modem is frozen at LongFast; when
 * preset support lands, the substitution must follow the active preset. This
 * pins the current constant to the harvested literal so the two cannot drift
 * apart silently in the meantime.
 */
ZTEST(wire_vectors, test_default_channel_name_matches_upstream_longfast)
{
	const struct mt_vec_preset_name *lf = NULL;

	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_preset_names); i++) {
		if (strcmp(mt_vec_preset_names[i].preset, "LONG_FAST") == 0) {
			lf = &mt_vec_preset_names[i];
			break;
		}
	}
	zassert_not_null(lf, "vector table missing the LONG_FAST display name");

	zassert_str_equal(MESHTASTIC_CHANNEL_LONGFAST, lf->display,
			  "the port's default channel name has drifted from "
			  "upstream's LONG_FAST display name");
}

/* Guard the harvested display-name table itself.
 *
 * Every preset must yield a non-empty name, and distinct presets must not
 * collide on the djb2 hash -- a collision would put two presets on the same
 * default frequency slot. Note VERY_LONG_SLOW is deliberately absent from
 * upstream's switch and falls through to "Invalid"; that is upstream's real
 * behaviour, not a harvest error, so it is asserted rather than excluded.
 */
ZTEST(wire_vectors, test_preset_display_names_are_sane)
{
	unsigned n = MT_VEC_COUNT(mt_vec_preset_names);

	zassert_true(n >= 16, "expected the full preset set, got %u", n);

	for (unsigned i = 0; i < n; i++) {
		const struct mt_vec_preset_name *p = &mt_vec_preset_names[i];

		zassert_true(p->display != NULL && p->display[0] != '\0',
			     "preset %s has an empty display name", p->preset);
		zassert_not_equal(p->djb2, 0, "preset %s hashes to zero", p->preset);

		for (unsigned j = i + 1; j < n; j++) {
			zassert_not_equal(p->djb2, mt_vec_preset_names[j].djb2,
					  "presets %s and %s share a slot hash",
					  p->preset, mt_vec_preset_names[j].preset);
		}
	}

	/* The deprecated preset really does hash the literal "Invalid". */
	for (unsigned i = 0; i < n; i++) {
		if (strcmp(mt_vec_preset_names[i].preset, "VERY_LONG_SLOW") == 0) {
			zassert_str_equal(mt_vec_preset_names[i].display, "Invalid",
					  "VERY_LONG_SLOW should fall through to "
					  "upstream's default case");
		}
	}
}

/* Every preset must resolve to exactly the SF/BW/CR stock firmware uses
 * (parity: radio D2).
 *
 * This is the assertion the whole harness exists for. The values on the right
 * were produced by running upstream's own modemPresetToParams; the values on
 * the left come from our re-expression of it as a table. A transcription slip
 * in any of 15 rows × 2 bandwidth columns fails here rather than putting a
 * node on a modem config no stock radio can hear.
 */
ZTEST(wire_vectors, test_preset_params_match_upstream)
{
	unsigned checked = 0;

	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_presets); i++) {
		const struct mt_vec_preset *v = &mt_vec_presets[i];
		struct meshtastic_modem_params got;
		meshtastic_Config_LoRaConfig_ModemPreset preset;
		bool matched = false;

		/* Resolve the vector's preset name back to its enum via the
		 * display-name table, which carries the same enum-name keys. */
		for (unsigned p = 0; p < MT_VEC_COUNT(mt_vec_preset_names); p++) {
			if (strcmp(mt_vec_preset_names[p].preset, v->name) == 0) {
				matched = true;
				break;
			}
		}
		zassert_true(matched, "vector preset %s has no display-name entry",
			     v->name);

		preset = (meshtastic_Config_LoRaConfig_ModemPreset)v->preset_enum;

		zassert_ok(meshtastic_preset_to_params(preset, v->wide != 0, &got),
			   "preset_to_params failed for %s", v->name);

		zassert_equal(got.spread_factor, v->sf,
			      "%s%s: SF %u != upstream %u", v->name,
			      v->wide ? " (wide)" : "", got.spread_factor, v->sf);
		zassert_equal(got.bandwidth_hz, v->bw_hz,
			      "%s%s: BW %u Hz != upstream %u Hz", v->name,
			      v->wide ? " (wide)" : "", got.bandwidth_hz, v->bw_hz);
		zassert_equal(got.coding_rate, v->cr,
			      "%s%s: CR 4/%u != upstream 4/%u", v->name,
			      v->wide ? " (wide)" : "", got.coding_rate, v->cr);
		checked++;
	}

	zassert_true(checked >= 30, "expected every preset in both widths, got %u",
		     checked);
	TC_PRINT("verified %u preset/width parameter sets\n", checked);
}

/* An unknown preset must fall back to LONG_FAST's parameters, not fail
 * (parity: radio D2). The reference folds LONG_FAST and illegal presets into
 * one default branch, so a node handed a bad preset stays on the air at the
 * default config rather than going silent.
 */
ZTEST(wire_vectors, test_unknown_preset_falls_back_to_longfast)
{
	struct meshtastic_modem_params bogus, longfast;

	zassert_ok(meshtastic_preset_to_params(
			   (meshtastic_Config_LoRaConfig_ModemPreset)0x7F, false, &bogus),
		   "an illegal preset must still resolve");
	zassert_ok(meshtastic_preset_to_params(
			   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, false,
			   &longfast),
		   "LONG_FAST must resolve");

	zassert_equal(bogus.spread_factor, longfast.spread_factor, "SF differs");
	zassert_equal(bogus.bandwidth_hz, longfast.bandwidth_hz, "BW differs");
	zassert_equal(bogus.coding_rate, longfast.coding_rate, "CR differs");

	zassert_equal(meshtastic_preset_to_params(
			      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, false,
			      NULL),
		      -EINVAL, "NULL out must be rejected");
}

/* Every bandwidth and coding rate any preset can produce must convert to a
 * real LoRa driver code (parity: radio D2).
 *
 * This is the seam where units change: the reference carries coding rate as
 * 5..8 and the driver's enum is 1..4, and driver bandwidth codes are rounded
 * kHz labels (BW_62_KHZ is 62.5 kHz, BW_1600_KHZ is 1625). Both conversions
 * fail silently if wrong -- the radio configures happily at the wrong
 * settings -- so drive them from the harvested vectors rather than by hand.
 */
ZTEST(wire_vectors, test_every_preset_converts_to_driver_codes)
{
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_presets); i++) {
		const struct mt_vec_preset *v = &mt_vec_presets[i];

		zassert_true(meshtastic_radio_bw_hz_to_code(v->bw_hz) >= 0,
			     "%s%s: bandwidth %u Hz has no driver code", v->name,
			     v->wide ? " (wide)" : "", v->bw_hz);
		zassert_true(meshtastic_radio_cr_to_code(v->cr) >= 0,
			     "%s%s: coding rate 4/%u has no driver code", v->name,
			     v->wide ? " (wide)" : "", v->cr);
	}

	/* Pin the conversions themselves. CR_4_5 is 1, not 5 -- the whole point. */
	zassert_equal(meshtastic_radio_cr_to_code(5), 1, "4/5 must map to CR_4_5 (1)");
	zassert_equal(meshtastic_radio_cr_to_code(8), 4, "4/8 must map to CR_4_8 (4)");
	zassert_equal(meshtastic_radio_cr_to_code(4), -EINVAL, "4/4 is not a rate");
	zassert_equal(meshtastic_radio_cr_to_code(9), -EINVAL, "4/9 is not a rate");

	/* The rounded labels: 62.5 kHz is BW_62_KHZ, 1625 kHz is BW_1600_KHZ. */
	zassert_equal(meshtastic_radio_bw_hz_to_code(62500), 62, "62.5 kHz -> BW_62_KHZ");
	zassert_equal(meshtastic_radio_bw_hz_to_code(1625000), 1600, "1625 kHz -> BW_1600_KHZ");
	zassert_equal(meshtastic_radio_bw_hz_to_code(250000), 250, "250 kHz -> BW_250_KHZ");
	zassert_equal(meshtastic_radio_bw_hz_to_code(123456), -EINVAL,
		      "an arbitrary bandwidth must be rejected");
}

/* The djb2 slot hash must match upstream for every harvested name
 * (parity: radio D3).
 *
 * This is a different function from the xor hash that produces the wire
 * channel byte. Using one where the other belongs is silent: the node lands
 * on the wrong frequency while reporting a correct-looking configuration.
 */
ZTEST(wire_vectors, test_djb2_hash_matches_upstream)
{
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_djb2_hashes); i++) {
		zassert_equal(meshtastic_djb2_hash(mt_vec_djb2_hashes[i].name),
			      mt_vec_djb2_hashes[i].hash,
			      "djb2(\"%s\") = %u, upstream says %u",
			      mt_vec_djb2_hashes[i].name,
			      meshtastic_djb2_hash(mt_vec_djb2_hashes[i].name),
			      mt_vec_djb2_hashes[i].hash);
	}

	/* Distinctness from the channel-byte hash, on a name where they differ. */
	zassert_not_equal(meshtastic_djb2_hash("LongFast"), 10,
			  "djb2 must not be confused with the xor channel hash");
}

/* Preset display names must match upstream exactly (parity: radio D3, crypto #1). */
ZTEST(wire_vectors, test_preset_display_names_match_upstream)
{
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_preset_names); i++) {
		const struct mt_vec_preset_name *v = &mt_vec_preset_names[i];
		meshtastic_Config_LoRaConfig_ModemPreset preset =
			meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
		bool found = false;

		if (strcmp(v->preset, "_CUSTOM") == 0) {
			zassert_str_equal(
				meshtastic_preset_display_name(
					meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
					false),
				v->display, "use_preset=false must yield \"Custom\"");
			continue;
		}

		for (unsigned p = 0; p < MT_VEC_COUNT(mt_vec_presets); p++) {
			if (strcmp(mt_vec_presets[p].name, v->preset) == 0) {
				preset = (meshtastic_Config_LoRaConfig_ModemPreset)
						 mt_vec_presets[p].preset_enum;
				found = true;
				break;
			}
		}
		zassert_true(found, "no enum for preset %s", v->preset);

		zassert_str_equal(meshtastic_preset_display_name(preset, true), v->display,
				  "%s display name differs from upstream", v->preset);
	}
}

/* Every region's slot arithmetic must reproduce upstream's (parity: radio D3).
 *
 * The vectors carry both the inputs (spacing, padding, band edges) and
 * upstream's computed slot width and count, so this checks the whole
 * calculation rather than just the table it reads from.
 *
 * Note the port computes in integer Hz where upstream uses MHz floats: a
 * float32 cannot represent 906875000 exactly. The results agree because every
 * input is an exact integer number of hertz.
 */
ZTEST(wire_vectors, test_region_slot_math_matches_upstream)
{
	unsigned checked = 0;

	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_regions); i++) {
		const struct mt_vec_region *v = &mt_vec_regions[i];
		struct meshtastic_freq_plan plan;
		meshtastic_Config_LoRaConfig_ModemPreset preset =
			meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
		uint32_t want_width_hz;
		bool found = false;

		for (unsigned p = 0; p < MT_VEC_COUNT(mt_vec_presets); p++) {
			if (strcmp(mt_vec_presets[p].name, v->default_preset) == 0 &&
			    mt_vec_presets[p].wide == v->wide_lora) {
				preset = (meshtastic_Config_LoRaConfig_ModemPreset)
						 mt_vec_presets[p].preset_enum;
				found = true;
				break;
			}
		}
		zassert_true(found, "no preset entry for %s/%s", v->name,
			     v->default_preset);

		zassert_ok(meshtastic_region_freq_plan(
				   (meshtastic_Config_LoRaConfig_RegionCode)v->region_enum,
				   preset, "LongFast", true, &plan),
			   "%s: no frequency plan", v->name);

		/* Upstream's slot width in MHz, converted exactly to Hz. */
		want_width_hz = (uint32_t)((double)v->slot_width_mhz * 1e6 + 0.5);

		zassert_equal(plan.slot_width_hz, want_width_hz,
			      "%s: slot width %u Hz != upstream %u Hz", v->name,
			      plan.slot_width_hz, want_width_hz);
		zassert_equal(plan.num_slots, v->num_freq_slots,
			      "%s: %u slots != upstream %u", v->name, plan.num_slots,
			      v->num_freq_slots);

		/* The chosen frequency must land inside the band. */
		zassert_true(plan.frequency_hz >=
				     (uint32_t)((double)v->freq_start_mhz * 1e6),
			     "%s: frequency below band start", v->name);
		zassert_true(plan.frequency_hz <=
				     (uint32_t)((double)v->freq_end_mhz * 1e6),
			     "%s: frequency above band end", v->name);
		checked++;
	}

	zassert_equal(checked, MT_VEC_COUNT(mt_vec_regions),
		      "every harvested region must have a plan: %u of %u", checked,
		      MT_VEC_COUNT(mt_vec_regions));
	TC_PRINT("verified slot arithmetic for %u regions\n", checked);

	/* US and EU_868 are the anchors: both must reproduce the frequency this
	 * port already transmits on, which the on-air tests also recorded.
	 */
	struct meshtastic_freq_plan us, eu;

	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_US,
			   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			   "LongFast", true, &us),
		   "US plan failed");
	zassert_equal(us.num_slots, 104, "US should divide into 104 slots, got %u",
		      us.num_slots);
	zassert_equal(us.slot, 19, "\"LongFast\" should hash to US slot 19, got %u",
		      us.slot);
	zassert_equal(us.frequency_hz, MESHTASTIC_FREQ_US,
		      "US/LongFast must reproduce the frequency this port already "
		      "transmits on: got %u, expected %u",
		      us.frequency_hz, MESHTASTIC_FREQ_US);

	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_EU_868,
			   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			   "LongFast", true, &eu),
		   "EU_868 plan failed");
	zassert_equal(eu.frequency_hz, MESHTASTIC_FREQ_EU,
		      "EU_868/LongFast must reproduce the existing constant: got %u, "
		      "expected %u",
		      eu.frequency_hz, MESHTASTIC_FREQ_EU);

}

/* Regions with non-zero spacing or padding are where an implementation is most
 * likely to be wrong -- and where the official Android app diverges from the
 * firmware (it omits spacing and padding entirely and floors instead of
 * rounding). Assert the firmware's arithmetic explicitly, since that is what
 * governs the air.
 */
ZTEST(wire_vectors, test_padded_regions_follow_firmware_not_app)
{
	struct meshtastic_freq_plan plan;

	/* EU_866: spacing 0.4 MHz, padding 0.0375 MHz, LiteFast at 125 kHz.
	 * Firmware: slot width 600 kHz, 4 slots. The app would compute 16. */
	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_EU_866,
			   meshtastic_Config_LoRaConfig_ModemPreset_LITE_FAST,
			   "LiteFast", true, &plan),
		   "EU_866 plan failed");
	zassert_equal(plan.slot_width_hz, 600000,
		      "EU_866 slot width must include spacing and both paddings");
	zassert_equal(plan.num_slots, 4,
		      "EU_866 has 4 slots, not the 16 raw bandwidth would give");
	zassert_equal(plan.duty_cycle_pct, 2.5f, "EU_866 is duty-cycle limited");

	/* EU_N_868 uses a fixed override slot rather than a name hash. */
	zassert_ok(meshtastic_region_freq_plan(
			   meshtastic_Config_LoRaConfig_RegionCode_EU_N_868,
			   meshtastic_Config_LoRaConfig_ModemPreset_NARROW_SLOW,
			   "anything", true, &plan),
		   "EU_N_868 plan failed");
	zassert_equal(plan.slot, 0,
		      "EU_N_868 pins slot 1 (0-based 0) regardless of channel name");
}

/* An obsolete region must be refused, not guessed at (parity: radio D3).
 *
 * UA_868 is still a selectable region for the preset map, but upstream removed
 * it from the frequency table and migrates it to EU_868 at config load. With
 * no band edges there is no honest frequency to return.
 */
ZTEST(wire_vectors, test_obsolete_region_has_no_frequency_plan)
{
	struct meshtastic_freq_plan plan;

	zassert_equal(meshtastic_region_freq_plan(
			      meshtastic_Config_LoRaConfig_RegionCode_UA_868,
			      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			      "LongFast", true, &plan),
		      -ENOTSUP, "UA_868 has no frequency data and must be refused");

	zassert_equal(meshtastic_region_freq_plan(
			      meshtastic_Config_LoRaConfig_RegionCode_US,
			      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
			      NULL, true, &plan),
		      -EINVAL, "a NULL channel name must be rejected");
}

/* Helper: a minimally valid lora config for the region under test. */
static meshtastic_Config lora_cfg(meshtastic_Config_LoRaConfig_RegionCode region,
				  float override_mhz)
{
	meshtastic_Config c = meshtastic_Config_init_zero;

	c.which_payload_variant = meshtastic_Config_lora_tag;
	c.payload_variant.lora.use_preset = true;
	c.payload_variant.lora.modem_preset =
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	c.payload_variant.lora.region = region;
	c.payload_variant.lora.hop_limit = 3;
	c.payload_variant.lora.override_frequency = override_mhz;
	return c;
}

/* Region selection is open, but every region is bounded by its own allocation.
 *
 * The build previously hard-locked to US. Opening that up is only safe if the
 * band implied by the chosen region is then enforced, so these are the guards
 * that replaced the lock -- if they regress, the device can be pointed at a
 * frequency it has no right to use.
 */
ZTEST(wire_vectors, test_region_selection_is_bounded_by_its_band)
{
	meshtastic_Config c;

	/* A recognised region is accepted. */
	c = lora_cfg(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 0.0f);
	zassert_ok(meshtastic_config_store_set_config(&c), "EU_868 should be accepted");

	/* An override inside that region's allocation is accepted... */
	c = lora_cfg(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 869.5f);
	zassert_ok(meshtastic_config_store_set_config(&c),
		   "an in-band override should be accepted");

	/* ...and one outside it is not, even though it is a legal US frequency. */
	c = lora_cfg(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 915.0f);
	zassert_equal(meshtastic_config_store_set_config(&c), -EINVAL,
		      "a US frequency must be refused while the region is EU_868");

	/* An override with no region to bound it is refused outright. */
	c = lora_cfg(meshtastic_Config_LoRaConfig_RegionCode_UNSET, 915.0f);
	zassert_equal(meshtastic_config_store_set_config(&c), -EINVAL,
		      "an override needs a region to validate against");

	/* 2.4 GHz stays rejected: no wide-LoRa radio support in this port. */
	c = lora_cfg(meshtastic_Config_LoRaConfig_RegionCode_LORA_24, 0.0f);
	zassert_equal(meshtastic_config_store_set_config(&c), -EINVAL,
		      "LORA_24 must remain rejected");

	/* Restore the default so later tests are unaffected. */
	c = lora_cfg((meshtastic_Config_LoRaConfig_RegionCode)
			     CONFIG_MESHTASTIC_DEFAULT_REGION,
		     0.0f);
	zassert_ok(meshtastic_config_store_set_config(&c), "restore failed");
}

/* Amateur allocations are refused outright, not gated on the licensed flag.
 *
 * Two independent reasons, either sufficient. Regulatory: the reference
 * disables encryption in licensed mode (suppressed keygen, forced LOCAL_ONLY)
 * because amateur service forbids obscuring meaning; this port implements
 * neither, so honouring the flag would put encrypted traffic on amateur
 * spectrum. Hardware: 2 m is 144-146 MHz, below the SX1262's 150 MHz floor,
 * and the 125 cm / 70 cm bands sit far outside the boards' 863-928 MHz
 * front-end.
 *
 * This asserts the flag does NOT unlock them, which is the whole point --
 * a future change that "fixes" the licence gate would otherwise silently
 * re-open a band this firmware cannot lawfully or physically use.
 */
ZTEST(wire_vectors, test_amateur_regions_are_refused_outright)
{
	static const meshtastic_Config_LoRaConfig_RegionCode ham[] = {
		meshtastic_Config_LoRaConfig_RegionCode_ITU1_2M,
		meshtastic_Config_LoRaConfig_RegionCode_ITU2_2M,
		meshtastic_Config_LoRaConfig_RegionCode_ITU3_2M,
		meshtastic_Config_LoRaConfig_RegionCode_ITU2_125CM,
		meshtastic_Config_LoRaConfig_RegionCode_ITU1_70CM,
		meshtastic_Config_LoRaConfig_RegionCode_ITU2_70CM,
		meshtastic_Config_LoRaConfig_RegionCode_ITU3_70CM,
	};
	struct meshtastic_region_info info;

	for (unsigned i = 0; i < ARRAY_SIZE(ham); i++) {
		meshtastic_Config c = lora_cfg(ham[i], 0.0f);

		zassert_ok(meshtastic_region_info(ham[i], &info),
			   "region %d should have band data", (int)ham[i]);
		zassert_true(info.licensed_only,
			     "region %d should be flagged amateur", (int)ham[i]);
		zassert_equal(meshtastic_config_store_set_config(&c), -EINVAL,
			      "amateur region %d must be refused", (int)ham[i]);
	}

	/* The 2 m allocations are below the SX1262's tuning floor entirely. */
	zassert_ok(meshtastic_region_info(
			   meshtastic_Config_LoRaConfig_RegionCode_ITU1_2M, &info),
		   "ITU1_2M info");
	zassert_true(info.freq_end_hz < 150000000U,
		     "2 m tops out at %u Hz, under the SX1262's 150 MHz floor",
		     info.freq_end_hz);

	/* A non-amateur region is unaffected. */
	zassert_ok(meshtastic_region_info(meshtastic_Config_LoRaConfig_RegionCode_US,
					  &info),
		   "US should have band data");
	zassert_false(info.licensed_only, "US is not an amateur allocation");

	meshtastic_Config restore =
		lora_cfg((meshtastic_Config_LoRaConfig_RegionCode)
				 CONFIG_MESHTASTIC_DEFAULT_REGION,
			 0.0f);
	zassert_ok(meshtastic_config_store_set_config(&restore), "restore failed");
}

/* Region power limits must match upstream, since they now bound TX power. */
ZTEST(wire_vectors, test_region_power_limits_match_upstream)
{
	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_regions); i++) {
		const struct mt_vec_region *v = &mt_vec_regions[i];
		struct meshtastic_region_info info;

		zassert_ok(meshtastic_region_info(
				   (meshtastic_Config_LoRaConfig_RegionCode)v->region_enum,
				   &info),
			   "%s has no region info", v->name);
		zassert_equal(info.power_limit_dbm, (int8_t)v->power_limit_dbm,
			      "%s power limit %d dBm != upstream %d dBm", v->name,
			      info.power_limit_dbm, (int)v->power_limit_dbm);
		zassert_equal(info.duty_cycle_pct, v->duty_cycle_pct,
			      "%s duty cycle differs from upstream", v->name);
	}

	/* The limits genuinely differ between regions -- this is why the clamp
	 * matters once regions other than US are selectable. */
	struct meshtastic_region_info us, eu433;

	zassert_ok(meshtastic_region_info(meshtastic_Config_LoRaConfig_RegionCode_US, &us),
		   "US info");
	zassert_ok(meshtastic_region_info(meshtastic_Config_LoRaConfig_RegionCode_EU_433,
					  &eu433),
		   "EU_433 info");
	zassert_true(us.power_limit_dbm > eu433.power_limit_dbm,
		     "US should permit more power than EU_433 (%d vs %d dBm)",
		     us.power_limit_dbm, eu433.power_limit_dbm);
}

/* Reference data the port does not consume yet (parity: airtime CP-1).
 *
 * The port measures TX airtime but never gates on it, so an EU build can
 * transmit past its regulatory ceiling. These are the numbers enforcement
 * will need; assert they are present and sane so they cannot rot before the
 * feature arrives.
 */
ZTEST(wire_vectors, test_region_duty_cycle_table_present)
{
	bool saw_eu868 = false, saw_us = false;

	zassert_true(MT_VEC_COUNT(mt_vec_regions) > 0, "region table is empty");

	for (unsigned i = 0; i < MT_VEC_COUNT(mt_vec_regions); i++) {
		const struct mt_vec_region *r = &mt_vec_regions[i];

		zassert_true(r->freq_end_mhz > r->freq_start_mhz,
			     "region %s has an inverted frequency range", r->name);
		zassert_true(r->duty_cycle_pct > 0.0f && r->duty_cycle_pct <= 100.0f,
			     "region %s duty cycle %.1f%% out of range",
			     r->name, (double)r->duty_cycle_pct);

		if (strcmp(r->name, "EU_868") == 0) {
			saw_eu868 = true;
			zassert_true(r->duty_cycle_pct < 100.0f,
				     "EU_868 must carry a restrictive duty cycle");
		} else if (strcmp(r->name, "US") == 0) {
			saw_us = true;
		}
	}

	zassert_true(saw_us && saw_eu868, "expected both US and EU_868 regions");
	TC_PRINT("%u regions carry duty-cycle limits\n",
		 MT_VEC_COUNT(mt_vec_regions));
}

ZTEST_SUITE(wire_vectors, NULL, NULL, vectors_before, NULL, NULL);

/* --- Contention window timing ---------------------------------------------
 *
 * The port recomputes upstream's slot time in integer arithmetic where upstream
 * uses floats, so these vectors are the only thing standing between us and a
 * silent drift that would scale every contention delay.
 */

/* Map a vector label ("LONG_FAST", "LONG_FAST_wide") onto our preset resolver. */
static bool vec_label_to_params(const char *label, struct meshtastic_modem_params *out,
				bool *wide)
{
	static const struct {
		const char *name;
		meshtastic_Config_LoRaConfig_ModemPreset preset;
	} map[] = {
		{"LONG_FAST", meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST},
		{"LONG_SLOW", meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW},
		{"LONG_MODERATE", meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE},
		{"MEDIUM_FAST", meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST},
		{"MEDIUM_SLOW", meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW},
		{"SHORT_FAST", meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST},
		{"SHORT_SLOW", meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW},
		{"SHORT_TURBO", meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO},
	};
	size_t len = strlen(label);

	*wide = false;
	if (len > 5U && strcmp(label + len - 5, "_wide") == 0) {
		*wide = true;
		len -= 5U;
	}

	for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
		if (strlen(map[i].name) == len && strncmp(label, map[i].name, len) == 0) {
			return meshtastic_preset_to_params(map[i].preset, *wide, out) == 0;
		}
	}
	return false; /* preset we do not model (LITE_*, etc.) — skipped */
}

ZTEST(wire_vectors, test_slot_time_matches_upstream)
{
	unsigned int checked = 0U;

	for (unsigned int i = 0; i < MT_VEC_COUNT(mt_vec_slot_times); i++) {
		const struct mt_vec_slot_time *v = &mt_vec_slot_times[i];
		struct meshtastic_modem_params p;
		bool wide;
		uint32_t got;

		if (!vec_label_to_params(v->preset, &p, &wide)) {
			continue;
		}

		got = meshtastic_contention_slot_ms(p.spread_factor, p.bandwidth_hz, wide);
		zassert_equal(got, v->slot_ms,
			      "slot time for %s: got %u ms, upstream says %u ms", v->preset, got,
			      v->slot_ms);
		checked++;
	}

	zassert_true(checked >= 8U, "expected to check a useful number of presets, did %u",
		     checked);
}

/* In the mapped SNR range we must match upstream exactly. Outside it we must
 * clamp where upstream does not — the vectors record upstream returning 241 for
 * an SNR of -128 and 27 for +127, and either would become a shift count. */
ZTEST(wire_vectors, test_cw_from_snr_matches_upstream_in_range_and_clamps_outside)
{
	for (unsigned int i = 0; i < MT_VEC_COUNT(mt_vec_cw_from_snr); i++) {
		const struct mt_vec_cw_snr *v = &mt_vec_cw_from_snr[i];
		uint8_t got = meshtastic_contention_cw_from_snr((int8_t)v->snr);

		if (v->cw_unclamped >= (int32_t)MESHTASTIC_CW_MIN &&
		    v->cw_unclamped <= (int32_t)MESHTASTIC_CW_MAX) {
			zassert_equal(got, (uint8_t)v->cw_unclamped,
				      "snr %d: got CW %u, upstream says %d", v->snr, got,
				      v->cw_unclamped);
		} else {
			zassert_true(got >= MESHTASTIC_CW_MIN && got <= MESHTASTIC_CW_MAX,
				     "snr %d: upstream's unclamped %d must clamp into range, got %u",
				     v->snr, v->cw_unclamped, got);
		}
	}
}

/* A relay delay must sit inside the window its own worst-case bound describes,
 * and a non-router must always wait out the router offset first — that ordering
 * is the whole point of the weighting. */
ZTEST(wire_vectors, test_relay_delay_respects_router_priority_and_bounds)
{
	const uint32_t slot = meshtastic_contention_slot_ms(11U, 250000U, false);
	const int8_t snrs[] = {-20, -10, 0, 5, 10};

	zassert_equal(slot, 28U, "LongFast slot time should be 28 ms, got %u", slot);

	for (unsigned int i = 0; i < ARRAY_SIZE(snrs); i++) {
		uint32_t worst = meshtastic_contention_delay_relay_worst_ms(snrs[i], slot);
		uint32_t router_floor = 2U * MESHTASTIC_CW_MAX * slot;

		for (unsigned int trial = 0; trial < 32U; trial++) {
			uint32_t client = meshtastic_contention_delay_relay_ms(snrs[i], false, slot);
			uint32_t router = meshtastic_contention_delay_relay_ms(snrs[i], true, slot);

			zassert_true(client >= router_floor,
				     "a client relay must wait past the router window: %u < %u",
				     client, router_floor);
			zassert_true(client <= worst, "client delay %u exceeded worst case %u",
				     client, worst);
			zassert_true(router < router_floor,
				     "a router relay must land inside the router window: %u >= %u",
				     router, router_floor);
		}
	}
}

/* Our own traffic widens its window with channel utilisation, and never carries
 * the router offset — that offset exists only to order relays. */
ZTEST(wire_vectors, test_own_delay_scales_with_utilisation)
{
	const uint32_t slot = meshtastic_contention_slot_ms(11U, 250000U, false);

	zassert_equal(meshtastic_contention_cw_from_util(0U), MESHTASTIC_CW_MIN);
	zassert_equal(meshtastic_contention_cw_from_util(100U), MESHTASTIC_CW_MAX);
	zassert_true(meshtastic_contention_cw_from_util(200U) <= MESHTASTIC_CW_MAX,
		     "out-of-range utilisation must still clamp");

	for (unsigned int trial = 0; trial < 64U; trial++) {
		uint32_t d = meshtastic_contention_delay_own_ms(100U, slot);

		zassert_true(d < (uint32_t)BIT(MESHTASTIC_CW_MAX) * slot,
			     "own delay %u outside the widest window", d);
	}
}

/* The window is runtime policy, not a compile-time constant: the defaults
 * reproduce the reference, and "legacy" zeroes them so the port's original
 * transmit-immediately behaviour is the control arm of an on-air A/B. */
ZTEST(wire_vectors, test_contention_window_is_runtime_policy)
{
	const uint32_t slot = meshtastic_contention_slot_ms(11U, 250000U, false);
	uint8_t cw_default;

	/* Defaults must match the reference constants, or every vector assertion
	 * above is only testing whatever the policy happens to be. */
	zassert_ok(meshtastic_sched_apply_preset("default"));
	zassert_equal(meshtastic_sched_get()->cw_min, MESHTASTIC_CW_MIN);
	zassert_equal(meshtastic_sched_get()->cw_max, MESHTASTIC_CW_MAX);
	cw_default = meshtastic_contention_cw_from_snr(0);

	/* Narrowing the window narrows the exponent it can produce. */
	zassert_ok(meshtastic_sched_set("cw.max", "4"));
	zassert_true(meshtastic_contention_cw_from_snr(10) <= 4U,
		     "cw.max must bound the exponent");

	/* Zeroing it removes the delay entirely — the legacy behaviour. */
	zassert_ok(meshtastic_sched_set("cw.min", "0"));
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	zassert_ok(meshtastic_sched_set("cw.offset", "0"));
	for (unsigned int trial = 0; trial < 16U; trial++) {
		zassert_equal(meshtastic_contention_delay_relay_ms(5, false, slot), 0U,
			      "a zeroed window must not delay at all");
		zassert_equal(meshtastic_contention_delay_own_ms(100U, slot), 0U,
			      "a zeroed window must not delay our own traffic either");
	}

	/* The "legacy" policy is exactly that control arm. */
	zassert_ok(meshtastic_sched_apply_preset("legacy"));
	zassert_equal(meshtastic_sched_get()->cw_max, 0U, "legacy should disable the window");
	zassert_equal(meshtastic_contention_delay_relay_ms(5, false, slot), 0U);

	/* Slot override replaces the preset-derived value. */
	zassert_ok(meshtastic_sched_apply_preset("default"));
	zassert_equal(meshtastic_contention_effective_slot_ms(11U, 250000U, false), slot,
		      "with no override the effective slot is the derived one");
	zassert_ok(meshtastic_sched_set("cw.slot", "100"));
	zassert_equal(meshtastic_contention_effective_slot_ms(11U, 250000U, false), 100U,
		      "cw.slot must override the derived slot");

	/* A max below min is operator error; it must degrade, not invert. */
	zassert_ok(meshtastic_sched_set("cw.min", "6"));
	zassert_ok(meshtastic_sched_set("cw.max", "2"));
	zassert_equal(meshtastic_contention_cw_from_snr(0), 6U,
		      "an inverted window should collapse to cw.min");

	zassert_ok(meshtastic_sched_apply_preset("default"));
	zassert_equal(meshtastic_contention_cw_from_snr(0), cw_default,
		      "restoring the default policy must restore the default mapping");
}
