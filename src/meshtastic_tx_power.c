/* SPDX-License-Identifier: GPL-3.0 */

/*
 * Transmit power: operator-facing dBm -> transceiver drive level.
 * See meshtastic_tx_power.h for why the two are not the same number.
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic_region_presets.h"
#include "meshtastic_tx_power.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

int8_t meshtastic_tx_power_resolve(int8_t configured,
				   meshtastic_Config_LoRaConfig_RegionCode region,
				   bool licensed)
{
	struct meshtastic_region_info info;
	int8_t limit = 0;

	if (region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
		region = (meshtastic_Config_LoRaConfig_RegionCode)
			CONFIG_MESHTASTIC_DEFAULT_REGION;
	}
	if (meshtastic_region_info(region, &info) == 0) {
		limit = info.power_limit_dbm;
	}

	/* 0 means "whatever the region allows", and an unlicensed operator may not
	 * exceed it. Reference: RadioInterface.cpp,
	 *   if ((power == 0) || ((power > powerLimit) && !is_licensed))
	 *           power = powerLimit;
	 */
	if ((configured == 0) || ((limit > 0) && (configured > limit) && !licensed)) {
		configured = limit;
	}

	/* Only reachable for a region with no declared limit — the reference uses
	 * the same fixed fallback there, and says so: "no region has an actual
	 * power limit of 0 dBm so we can assume regions with this variable set to
	 * 0 don't have a valid power limit". This is NOT the US default; US is 30.
	 */
	if (configured == 0) {
		configured = (int8_t)CONFIG_MESHTASTIC_TX_POWER;
	}

	return configured;
}

int8_t meshtastic_fem_gain_convert(const uint8_t *gain, size_t n, int8_t desired)
{
	if ((gain == NULL) || (n == 0U)) {
		return desired;
	}

	for (size_t i = 0U; i < n; i++) {
		int reachable = (int)i + (int)gain[i];

		/* First drive level that overshoots the request — or the last one, if
		 * even at maximum drive the FEM cannot reach it. Either way the answer
		 * is "request minus this level's gain", which the caller then clamps to
		 * what the radio can be set to. Reference: LoRaFEMInterface.cpp.
		 */
		if ((reachable > (int)desired) ||
		    ((i == (n - 1U)) && (reachable <= (int)desired))) {
			return (int8_t)((int)desired - (int)gain[i]);
		}
	}

	return desired;
}

int8_t meshtastic_tx_power_chip_drive(int8_t radiated_dbm)
{
	int8_t drive = meshtastic_radio_fem_tx_power_conversion(radiated_dbm);

	return (int8_t)CLAMP((int)drive, CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
			     CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER);
}
