/* SPDX-License-Identifier: GPL-3.0
 *
 * Region -> legal-modem-preset table, ported verbatim from the reference
 * firmware's RadioInterface.cpp (preset arrays + per-region profiles + the
 * coalescing getRegionPresetMap). Only the data the preset map needs is kept
 * here; the regulatory parameters (frequency, duty cycle, power) live with the
 * radio driver, not the phone-facing map.
 */

#include "meshtastic_region_presets.h"

#include <errno.h>

#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define PRESET(name)     meshtastic_Config_LoRaConfig_ModemPreset_##name
#define REGION(name)     meshtastic_Config_LoRaConfig_RegionCode_##name
#define MODEM_PRESET_END ((meshtastic_Config_LoRaConfig_ModemPreset)0xFF)

/* Preset lists, MODEM_PRESET_END-terminated (RadioInterface.cpp PRESETS_*). */
static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_STD[] = {
	PRESET(LONG_FAST),     PRESET(LONG_SLOW),   PRESET(MEDIUM_SLOW),
	PRESET(MEDIUM_FAST),   PRESET(SHORT_SLOW),  PRESET(SHORT_FAST),
	PRESET(LONG_MODERATE), PRESET(SHORT_TURBO), PRESET(LONG_TURBO),
	PRESET(MEDIUM_TURBO),  MODEM_PRESET_END};

static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_EU_868[] = {
	PRESET(LONG_FAST),  PRESET(LONG_SLOW),  PRESET(MEDIUM_SLOW),   PRESET(MEDIUM_FAST),
	PRESET(SHORT_SLOW), PRESET(SHORT_FAST), PRESET(LONG_MODERATE), MODEM_PRESET_END};

static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_UNDEF[] = {PRESET(LONG_FAST),
									 MODEM_PRESET_END};

static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_LITE[] = {
	PRESET(LITE_FAST), PRESET(LITE_SLOW), MODEM_PRESET_END};

static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_NARROW[] = {
	PRESET(NARROW_FAST), PRESET(NARROW_SLOW), MODEM_PRESET_END};

static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS_TINY[] = {
	PRESET(TINY_FAST), PRESET(TINY_SLOW), MODEM_PRESET_END};

/* Region profile: the preset list plus licensing. Coalescing keys on the profile
 * identity (index), not the preset array, so PROFILE_NARROW and PROFILE_HAM_100KHZ
 * stay distinct even though both use PRESETS_NARROW (they differ in licensing). */
struct region_profile {
	const meshtastic_Config_LoRaConfig_ModemPreset *presets;
	bool licensed_only;
	/* Slot geometry from the reference's RegionProfile: spacing is the gap
	 * between slots (and before the first), padding the gap at each slot's
	 * edges. Both are zero for continuous-spectrum regions, which is every
	 * classic region; only the Lite, Narrow and amateur profiles set them.
	 *
	 * Held in Hz, not MHz, so the whole frequency plan is integer maths. The
	 * reference computes in MHz floats, but a float32 cannot hold 906875000
	 * exactly -- its ULP up there is about 64 Hz -- so carrying MHz through
	 * to a Hz result would quietly corrupt the frequency. Every value is an
	 * exact integer in Hz (0.0375 MHz is 37500 Hz), so nothing is lost. */
	uint32_t spacing_hz;
	uint32_t padding_hz;
};

enum {
	PROFILE_STD = 0,
	PROFILE_EU868,
	PROFILE_UNDEF,
	PROFILE_LITE,
	PROFILE_NARROW,
	PROFILE_HAM_20KHZ,
	PROFILE_HAM_100KHZ,
};

static const struct region_profile profiles[] = {
	[PROFILE_STD] = {PRESETS_STD, false, 0U, 0U},
	[PROFILE_EU868] = {PRESETS_EU_868, false, 0U, 0U},
	[PROFILE_UNDEF] = {PRESETS_UNDEF, true, 0U, 0U},
	[PROFILE_LITE] = {PRESETS_LITE, false, 400000U, 37500U},      /* 0.4 / 0.0375 MHz */
	[PROFILE_NARROW] = {PRESETS_NARROW, false, 0U, 10400U},       /* 0.0104 MHz */
	[PROFILE_HAM_20KHZ] = {PRESETS_TINY, true, 0U, 2200U},        /* 0.0022 MHz */
	[PROFILE_HAM_100KHZ] = {PRESETS_NARROW, true, 0U, 18750U},    /* 0.01875 MHz */
};

/* One row per region the reference firmware defines (RadioInterface.cpp regions[]),
 * carrying only what the map needs: region code, profile, default preset. Regions
 * absent here (e.g. EU_874/EU_917, which the reference table also omits) carry no
 * constraint, which the proto permits. */
struct region_row {
	meshtastic_Config_LoRaConfig_RegionCode region;
	uint8_t profile;
	meshtastic_Config_LoRaConfig_ModemPreset default_preset;
};

static const struct region_row regions[] = {
	{REGION(US), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(EU_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(EU_868), PROFILE_EU868, PRESET(LONG_FAST)},
	{REGION(EU_866), PROFILE_LITE, PRESET(LITE_FAST)},
	{REGION(EU_N_868), PROFILE_NARROW, PRESET(NARROW_SLOW)},
	{REGION(CN), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(JP), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(ANZ), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(ANZ_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(RU), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(KR), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(TW), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(IN), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(NZ_865), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(TH), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(UA_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(UA_868), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(MY_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(MY_919), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(SG_923), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(PH_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(PH_868), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(PH_915), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(KZ_433), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(KZ_863), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(NP_865), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(BR_902), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(ITU1_2M), PROFILE_HAM_20KHZ, PRESET(TINY_FAST)},
	{REGION(ITU2_2M), PROFILE_HAM_20KHZ, PRESET(TINY_FAST)},
	{REGION(ITU3_2M), PROFILE_HAM_20KHZ, PRESET(TINY_FAST)},
	{REGION(ITU2_125CM), PROFILE_HAM_100KHZ, PRESET(NARROW_SLOW)},
	{REGION(ITU1_70CM), PROFILE_HAM_100KHZ, PRESET(NARROW_SLOW)},
	{REGION(ITU2_70CM), PROFILE_HAM_100KHZ, PRESET(NARROW_SLOW)},
	{REGION(ITU3_70CM), PROFILE_HAM_100KHZ, PRESET(NARROW_SLOW)},
	{REGION(LORA_24), PROFILE_STD, PRESET(LONG_FAST)},
	{REGION(UNSET), PROFILE_UNDEF, PRESET(LONG_FAST)},
};

void meshtastic_build_region_preset_map(meshtastic_LoRaRegionPresetMap *map)
{
	const size_t max_groups = ARRAY_SIZE(map->groups);
	const size_t max_regions = ARRAY_SIZE(map->region_groups);
	const size_t max_presets = ARRAY_SIZE(map->groups[0].presets);
	uint8_t group_profile[ARRAY_SIZE(map->groups)];

	*map = (meshtastic_LoRaRegionPresetMap)meshtastic_LoRaRegionPresetMap_init_zero;

	for (size_t ri = 0U; ri < ARRAY_SIZE(regions); ri++) {
		const struct region_row *r = &regions[ri];
		int gi = -1;

		/* Out of region slots: an incomplete map leaves the rest
		 * unconstrained, so surface it rather than truncate silently. */
		if (map->region_groups_count >= max_regions) {
			LOG_ERR("Region preset map full at %u regions; remainder omitted",
				(unsigned int)max_regions);
			break;
		}

		/* Reuse an existing group with the same profile + default preset. */
		for (pb_size_t g = 0U; g < map->groups_count; g++) {
			if (group_profile[g] == r->profile &&
			    map->groups[g].default_preset == r->default_preset) {
				gi = (int)g;
				break;
			}
		}

		if (gi < 0) {
			const struct region_profile *p = &profiles[r->profile];
			meshtastic_LoRaPresetGroup *grp;

			if (map->groups_count >= max_groups) {
				LOG_ERR("Region preset map out of group slots (%u); region %d omitted",
					(unsigned int)max_groups, (int)r->region);
				continue;
			}
			gi = (int)map->groups_count++;
			group_profile[gi] = r->profile;

			grp = &map->groups[gi];
			grp->default_preset = r->default_preset;
			grp->licensed_only = p->licensed_only;
			grp->presets_count = 0U;
			for (size_t i = 0U; p->presets[i] != MODEM_PRESET_END &&
					    grp->presets_count < max_presets;
			     i++) {
				grp->presets[grp->presets_count++] = p->presets[i];
			}
		}

		map->region_groups[map->region_groups_count].region = r->region;
		map->region_groups[map->region_groups_count].group_index = (uint8_t)gi;
		map->region_groups_count++;
	}
}

/* Preset -> modem parameters, ported from the reference firmware's
 * modemPresetToParams (MeshRadio.h). Expressed as a table rather than the
 * reference's switch; the wire_vectors suite asserts every row against values
 * harvested by running the reference function itself, so any transcription
 * error fails the build rather than reaching the air.
 *
 * bw_wide_hz repeats bw_narrow_hz for the presets that pin one bandwidth and
 * ignore wideLora entirely (Lite, Narrow, Tiny).
 *
 * Bandwidths are exact: the reference's 15.6/62.5/406.25/812.5/1625.0 kHz
 * become 15600/62500/406250/812500/1625000 Hz.
 */
struct preset_params_row {
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	uint32_t bw_narrow_hz;
	uint32_t bw_wide_hz;
	uint8_t spread_factor;
	uint8_t coding_rate;
};

/* LONG_FAST is deliberately absent: the reference folds it into the same
 * `default` branch as an illegal preset, and so does the lookup below. */
static const struct preset_params_row preset_params[] = {
	{PRESET(SHORT_TURBO), 500000U, 1625000U, 7U, 5U},
	{PRESET(SHORT_FAST), 250000U, 812500U, 7U, 5U},
	{PRESET(SHORT_SLOW), 250000U, 812500U, 8U, 5U},
	{PRESET(MEDIUM_FAST), 250000U, 812500U, 9U, 5U},
	{PRESET(MEDIUM_SLOW), 250000U, 812500U, 10U, 5U},
	{PRESET(MEDIUM_TURBO), 500000U, 1625000U, 9U, 5U},
	{PRESET(LONG_TURBO), 500000U, 1625000U, 11U, 8U},
	{PRESET(LONG_MODERATE), 125000U, 406250U, 11U, 8U},
	{PRESET(LONG_SLOW), 125000U, 406250U, 12U, 8U},
	{PRESET(LITE_FAST), 125000U, 125000U, 9U, 5U},
	{PRESET(LITE_SLOW), 125000U, 125000U, 10U, 5U},
	{PRESET(NARROW_FAST), 62500U, 62500U, 7U, 6U},
	{PRESET(NARROW_SLOW), 62500U, 62500U, 8U, 6U},
	{PRESET(TINY_FAST), 15600U, 15600U, 7U, 5U},
	{PRESET(TINY_SLOW), 15600U, 15600U, 8U, 6U},
};

/* The reference's `default:` branch — LONG_FAST and anything illegal. */
static const struct preset_params_row preset_params_default = {
	PRESET(LONG_FAST), 250000U, 812500U, 11U, 5U,
};

int meshtastic_preset_to_params(meshtastic_Config_LoRaConfig_ModemPreset preset,
				bool wide_lora, struct meshtastic_modem_params *out)
{
	const struct preset_params_row *row = &preset_params_default;

	if (out == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(preset_params); i++) {
		if (preset_params[i].preset == preset) {
			row = &preset_params[i];
			break;
		}
	}

	out->bandwidth_hz = wide_lora ? row->bw_wide_hz : row->bw_narrow_hz;
	out->spread_factor = row->spread_factor;
	out->coding_rate = row->coding_rate;

	return 0;
}

/* ---------------------------------------------------------------------------
 * Frequency plan (parity: radio D3)
 *
 * Ported from the reference firmware's slot arithmetic. Kept as its own table
 * rather than widening region_row, because the two sets are not identical:
 * UA_868 is a live region for the preset map but the reference retired it from
 * the frequency table and migrates it to EU_868 at config load. A region absent
 * here therefore has no frequency plan, which is the correct answer for it.
 * -------------------------------------------------------------------------*/

/* djb2. Distinct from xor_hash_bytes, which produces the wire channel byte;
 * this one only ever selects a frequency slot. */
uint32_t meshtastic_djb2_hash(const char *s)
{
	uint32_t hash = 5381U;

	if (s == NULL) {
		return hash;
	}

	while (*s != '\0') {
		/* hash * 33 + c */
		hash = ((hash << 5) + hash) + (uint32_t)(unsigned char)*s;
		s++;
	}

	return hash;
}

/* Mirrors getModemPresetDisplayName(preset, useShortName=false, usePreset).
 * These literals are protocol constants: they pick the frequency slot and
 * stand in for an empty channel name when hashing the channel. VERY_LONG_SLOW
 * is deliberately absent — the reference has no case for it (deprecated in
 * 2.5) so it falls through to "Invalid", and it appears in no region's legal
 * preset list. Note the official Android app maps it to "VLongSlow" instead;
 * the reference wins, because the reference is what transmits. */
const char *meshtastic_preset_display_name(meshtastic_Config_LoRaConfig_ModemPreset preset,
					   bool use_preset)
{
	if (!use_preset) {
		return "Custom";
	}

	switch (preset) {
	case PRESET(SHORT_TURBO):
		return "ShortTurbo";
	case PRESET(SHORT_FAST):
		return "ShortFast";
	case PRESET(SHORT_SLOW):
		return "ShortSlow";
	case PRESET(MEDIUM_FAST):
		return "MediumFast";
	case PRESET(MEDIUM_SLOW):
		return "MediumSlow";
	case PRESET(MEDIUM_TURBO):
		return "MediumTurbo";
	case PRESET(LONG_FAST):
		return "LongFast";
	case PRESET(LONG_SLOW):
		return "LongSlow";
	case PRESET(LONG_MODERATE):
		return "LongMod";
	case PRESET(LONG_TURBO):
		return "LongTurbo";
	case PRESET(LITE_FAST):
		return "LiteFast";
	case PRESET(LITE_SLOW):
		return "LiteSlow";
	case PRESET(NARROW_FAST):
		return "NarrowFast";
	case PRESET(NARROW_SLOW):
		return "NarrowSlow";
	case PRESET(TINY_FAST):
		return "TinyFast";
	case PRESET(TINY_SLOW):
		return "TinySlow";
	default:
		return "Invalid";
	}
}

/* override_slot: 0 = hash the channel name, -1 = hash the preset display name,
 * >0 = a fixed 1-based slot. */
#define OVERRIDE_SLOT_CHANNEL_HASH 0
#define OVERRIDE_SLOT_PRESET_HASH  (-1)

struct region_freq {
	meshtastic_Config_LoRaConfig_RegionCode region;
	uint32_t freq_start_hz;
	uint32_t freq_end_hz;
	float duty_cycle_pct;
	int8_t power_limit_dbm;
	int8_t override_slot;
	bool wide_lora;
};

static const struct region_freq region_freqs[] = {
	{REGION(US), 902000000U, 928000000U, 100.0f, 30, 0, 0},
	{REGION(EU_433), 433000000U, 434000000U, 10.0f, 10, 0, 0},
	{REGION(EU_868), 869400000U, 869650000U, 10.0f, 27, 0, 0},
	{REGION(EU_866), 865600000U, 867600000U, 2.5f, 27, 0, 0},
	{REGION(EU_N_868), 869400000U, 869650000U, 10.0f, 27, 1, 0},
	{REGION(CN), 470000000U, 510000000U, 100.0f, 19, 0, 0},
	{REGION(JP), 920500000U, 923500000U, 100.0f, 13, 0, 0},
	{REGION(ANZ), 915000000U, 928000000U, 100.0f, 30, 0, 0},
	{REGION(ANZ_433), 433050000U, 434790000U, 100.0f, 14, 0, 0},
	{REGION(RU), 868700000U, 869200000U, 100.0f, 20, 0, 0},
	{REGION(KR), 920000000U, 923000000U, 100.0f, 23, 0, 0},
	{REGION(TW), 920000000U, 925000000U, 100.0f, 27, 0, 0},
	{REGION(IN), 865000000U, 867000000U, 100.0f, 30, 0, 0},
	{REGION(NZ_865), 864000000U, 868000000U, 100.0f, 36, 0, 0},
	{REGION(TH), 920000000U, 925000000U, 10.0f, 27, 0, 0},
	{REGION(UA_433), 433000000U, 434700000U, 10.0f, 10, 0, 0},
	{REGION(MY_433), 433000000U, 435000000U, 100.0f, 20, 0, 0},
	{REGION(MY_919), 919000000U, 924000000U, 100.0f, 27, 0, 0},
	{REGION(SG_923), 917000000U, 925000000U, 100.0f, 20, 0, 0},
	{REGION(PH_433), 433000000U, 434700000U, 100.0f, 10, 0, 0},
	{REGION(PH_868), 868000000U, 869400000U, 100.0f, 14, 0, 0},
	{REGION(PH_915), 915000000U, 918000000U, 100.0f, 24, 0, 0},
	{REGION(KZ_433), 433075000U, 434775000U, 100.0f, 10, 0, 0},
	{REGION(KZ_863), 863000000U, 868000000U, 100.0f, 30, 0, 0},
	{REGION(NP_865), 865000000U, 868000000U, 100.0f, 30, 0, 0},
	{REGION(BR_902), 902000000U, 907500000U, 100.0f, 30, 0, 0},
	{REGION(ITU1_2M), 144000000U, 146000000U, 100.0f, 30, 26, 0},
	{REGION(ITU2_2M), 144000000U, 148000000U, 100.0f, 30, 51, 0},
	{REGION(ITU3_2M), 144000000U, 148000000U, 100.0f, 30, 33, 0},
	{REGION(ITU2_125CM), 220000000U, 225000000U, 100.0f, 30, 37, 0},
	{REGION(ITU1_70CM), 430000000U, 440000000U, 100.0f, 30, 37, 0},
	{REGION(ITU2_70CM), 420000000U, 450000000U, 100.0f, 30, 137, 0},
	{REGION(ITU3_70CM), 430000000U, 450000000U, 100.0f, 30, 37, 0},
	{REGION(LORA_24), 2400000000U, 2483500000U, 100.0f, 10, 0, 1},
	{REGION(UNSET), 902000000U, 928000000U, 100.0f, 30, 0, 0},
};

int meshtastic_region_freq_plan(meshtastic_Config_LoRaConfig_RegionCode region,
				meshtastic_Config_LoRaConfig_ModemPreset preset,
				const char *channel_name, bool use_preset,
				struct meshtastic_freq_plan *out)
{
	const struct region_freq *rf = NULL;
	const struct region_row *row = NULL;
	const struct region_profile *prof;
	struct meshtastic_modem_params modem;
	uint32_t slot_width, span, num_slots, slot;

	if (out == NULL || channel_name == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(region_freqs); i++) {
		if (region_freqs[i].region == region) {
			rf = &region_freqs[i];
			break;
		}
	}
	for (size_t i = 0U; i < ARRAY_SIZE(regions); i++) {
		if (regions[i].region == region) {
			row = &regions[i];
			break;
		}
	}
	/* No frequency data: an obsolete region (UA_868, which the reference
	 * migrates to EU_868) or one it never defined. Refuse rather than invent
	 * a frequency -- transmitting on a guess is worse than not transmitting. */
	if (rf == NULL || row == NULL) {
		return -ENOTSUP;
	}

	prof = &profiles[row->profile];
	(void)meshtastic_preset_to_params(preset, rf->wide_lora, &modem);

	slot_width = prof->spacing_hz + (2U * prof->padding_hz) + modem.bandwidth_hz;
	if (slot_width == 0U) {
		return -ENOTSUP;
	}

	/* round((end - start + spacing) / slot_width), integer half-up. */
	span = (rf->freq_end_hz - rf->freq_start_hz) + prof->spacing_hz;
	num_slots = (span + (slot_width / 2U)) / slot_width;

	/* The reference guards this too: a degenerate region can compute zero
	 * slots, and the modulo below would divide by zero. */
	if (num_slots == 0U) {
		num_slots = 1U;
	}

	if (rf->override_slot > 0) {
		/* Explicit slot, 1-based in the table, 0-based here. */
		slot = (uint32_t)(rf->override_slot - 1);
		if (slot >= num_slots) {
			slot = num_slots - 1U;
		}
	} else if (rf->override_slot == OVERRIDE_SLOT_PRESET_HASH) {
		slot = meshtastic_djb2_hash(
			       meshtastic_preset_display_name(preset, use_preset)) %
		       num_slots;
	} else {
		slot = meshtastic_djb2_hash(channel_name) % num_slots;
	}

	out->frequency_hz = rf->freq_start_hz + (modem.bandwidth_hz / 2U) +
			    prof->padding_hz + (slot * slot_width);
	out->slot_width_hz = slot_width;
	out->num_slots = (uint16_t)num_slots;
	out->slot = (uint16_t)slot;
	out->duty_cycle_pct = rf->duty_cycle_pct;

	return 0;
}

int meshtastic_region_info(meshtastic_Config_LoRaConfig_RegionCode region,
			   struct meshtastic_region_info *out)
{
	const struct region_freq *rf = NULL;
	const struct region_row *row = NULL;

	if (out == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(region_freqs); i++) {
		if (region_freqs[i].region == region) {
			rf = &region_freqs[i];
			break;
		}
	}
	for (size_t i = 0U; i < ARRAY_SIZE(regions); i++) {
		if (regions[i].region == region) {
			row = &regions[i];
			break;
		}
	}
	if (rf == NULL || row == NULL) {
		return -ENOTSUP;
	}

	out->freq_start_hz = rf->freq_start_hz;
	out->freq_end_hz = rf->freq_end_hz;
	out->duty_cycle_pct = rf->duty_cycle_pct;
	out->power_limit_dbm = rf->power_limit_dbm;
	out->licensed_only = profiles[row->profile].licensed_only;
	out->wide_lora = rf->wide_lora;

	return 0;
}
