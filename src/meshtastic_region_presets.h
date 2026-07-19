/* SPDX-License-Identifier: GPL-3.0
 *
 * Region -> legal-modem-preset map for the want_config handshake. Mirrors the
 * reference firmware's getRegionPresetMap (RadioInterface.cpp): the phone uses
 * this to prevent illegal region+preset selections.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_REGION_PRESETS_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_REGION_PRESETS_H_

#include "meshtastic/mesh.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Populate @p map with the region -> valid-preset-group table.
 *
 * Coalesces every known region into a small set of preset groups (regions that
 * share the same preset list, default preset and licensing reference one group),
 * exactly as the reference getRegionPresetMap does. Respects the proto capacities
 * (groups[8], region_groups[38]); if the table ever outgrows them, the excess is
 * dropped and logged (a client simply leaves the omitted regions unconstrained).
 */
void meshtastic_build_region_preset_map(meshtastic_LoRaRegionPresetMap *map);

/** Modem parameters a preset selects, in the units the LoRa driver wants. */
struct meshtastic_modem_params {
	uint32_t bandwidth_hz; /**< 15600 … 1625000 */
	uint8_t spread_factor; /**< 5 … 12 */
	uint8_t coding_rate;   /**< 5 … 8, meaning 4/5 … 4/8 (NOT the Zephyr enum) */
};

/**
 * @brief Resolve a modem preset to its spreading factor, bandwidth and coding rate.
 *
 * Ports the reference firmware's modemPresetToParams (MeshRadio.h). @p wide_lora
 * selects the wide-band variant used by regions whose profile allows it; the
 * reference applies the same switch with a different bandwidth column.
 *
 * @note @c coding_rate is returned in the reference's 5..8 convention, not
 *       Zephyr's @c CR_4_5..CR_4_8 enum (which is 1..4). Callers driving the LoRa
 *       API must convert. Keeping the reference convention here is deliberate:
 *       these values are asserted directly against harvested upstream vectors.
 *
 * @note An unknown or illegal preset resolves to LONG_FAST's parameters rather
 *       than failing. That mirrors the reference, whose switch folds LONG_FAST
 *       and the illegal cases into the same @c default branch — a node handed a
 *       bad preset stays on the air at the default modem config instead of going
 *       silent.
 *
 * @param preset     modem preset to resolve
 * @param wide_lora  true to use the wide-band column (ignored by the Lite,
 *                   Narrow and Tiny presets, which pin one bandwidth)
 * @param out        populated on success
 * @return 0 on success, -EINVAL if @p out is NULL.
 */
int meshtastic_preset_to_params(meshtastic_Config_LoRaConfig_ModemPreset preset,
				bool wide_lora, struct meshtastic_modem_params *out);

/**
 * @brief djb2 string hash, as the reference firmware uses for frequency slots.
 *
 * Distinct from the xor hash that produces the wire channel byte. Conflating
 * the two is silent: the node lands on the wrong frequency while still
 * believing it is correctly configured.
 */
uint32_t meshtastic_djb2_hash(const char *s);

/**
 * @brief The display name a preset hashes as.
 *
 * Not cosmetic. This string selects the frequency slot, and substitutes for an
 * empty channel name when computing the channel hash. Mirrors the reference's
 * getModemPresetDisplayName(preset, useShortName=false, usePreset).
 *
 * @param preset      preset to name
 * @param use_preset  false yields "Custom", matching the reference
 * @return a static string, never NULL.
 */
const char *meshtastic_preset_display_name(meshtastic_Config_LoRaConfig_ModemPreset preset,
					   bool use_preset);

/** A region's resolved frequency plan. */
struct meshtastic_freq_plan {
	uint32_t frequency_hz;  /**< centre of the selected slot */
	uint32_t slot_width_hz; /**< spacing + 2*padding + bandwidth */
	uint16_t num_slots;     /**< slots the band divides into */
	uint16_t slot;          /**< 0-based slot chosen */
	float duty_cycle_pct;   /**< regulatory ceiling, 100 where unrestricted */
};

/**
 * @brief Resolve a region and channel name to the frequency the radio should use.
 *
 * Ports the reference's slot arithmetic (RadioInterface::checkOrClampConfigLora
 * plus the frequency calculation in applyModemConfig):
 *
 *     slot_width  = spacing + 2*padding + bandwidth
 *     num_slots   = round((freq_end - freq_start + spacing) / slot_width)
 *     slot        = djb2(name) % num_slots
 *     frequency   = freq_start + bandwidth/2 + padding + slot*slot_width
 *
 * A region whose @c override_slot is positive uses that fixed slot instead, and
 * one set to -1 hashes the preset display name rather than the channel name.
 *
 * @note The official Android app computes a simpler variant of this — raw
 *       bandwidth, no spacing or padding, floor instead of round. The two agree
 *       for every region whose profile has zero spacing and padding (including
 *       US and EU_868) and diverge for EU_866, EU_N_868 and the ITU amateur
 *       bands. The app does not transmit, so the firmware arithmetic reproduced
 *       here is what governs the air.
 *
 * @param region        region code
 * @param preset        active modem preset
 * @param channel_name  primary channel name, already resolved (never empty)
 * @param use_preset    passed through to the preset display name
 * @param out           populated on success
 * @return 0 on success, -EINVAL on a NULL argument, -ENOTSUP if @p region has no
 *         frequency data (obsolete or unknown).
 */
int meshtastic_region_freq_plan(meshtastic_Config_LoRaConfig_RegionCode region,
				meshtastic_Config_LoRaConfig_ModemPreset preset,
				const char *channel_name, bool use_preset,
				struct meshtastic_freq_plan *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_REGION_PRESETS_H_ */
