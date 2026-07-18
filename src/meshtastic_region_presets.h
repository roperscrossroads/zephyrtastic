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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_REGION_PRESETS_H_ */
