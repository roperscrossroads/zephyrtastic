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
#include "meshtastic_core.h"
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

/* Characterisation of the known gap (parity: crypto #1).
 *
 * An empty channel name resolves to the hardcoded string "LongFast" rather
 * than the *active preset's* display name. That is correct only while the
 * modem is frozen at LongFast (parity: radio D2). When preset support lands,
 * this test must change to expect the preset-derived name -- it is here so
 * that change is deliberate and visible, not silent.
 */
ZTEST(wire_vectors, test_empty_name_currently_defaults_to_longfast)
{
	bool found;
	uint8_t longfast = vec_name_hash("LongFast", &found);

	zassert_true(found, "vector table missing the LongFast name hash");

	set_primary("", full_psks[2].bytes, full_psks[2].len);

	zassert_equal(meshtastic_channels_get_hash(0),
		      (uint8_t)(longfast ^ vec_psk_hash("aes128_counting", &found)),
		      "empty-name default changed; if preset support landed, "
		      "update this test to the preset display name");
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
