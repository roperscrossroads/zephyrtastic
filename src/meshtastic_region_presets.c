/* SPDX-License-Identifier: GPL-3.0
 *
 * Region -> legal-modem-preset table, ported verbatim from the reference
 * firmware's RadioInterface.cpp (preset arrays + per-region profiles + the
 * coalescing getRegionPresetMap). Only the data the preset map needs is kept
 * here; the regulatory parameters (frequency, duty cycle, power) live with the
 * radio driver, not the phone-facing map.
 */

#include "meshtastic_region_presets.h"

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
	[PROFILE_STD] = {PRESETS_STD, false},
	[PROFILE_EU868] = {PRESETS_EU_868, false},
	[PROFILE_UNDEF] = {PRESETS_UNDEF, true},
	[PROFILE_LITE] = {PRESETS_LITE, false},
	[PROFILE_NARROW] = {PRESETS_NARROW, false},
	[PROFILE_HAM_20KHZ] = {PRESETS_TINY, true},
	[PROFILE_HAM_100KHZ] = {PRESETS_NARROW, true},
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
