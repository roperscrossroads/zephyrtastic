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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_REGION_PRESETS_H_ */
