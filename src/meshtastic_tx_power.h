/* SPDX-License-Identifier: GPL-3.0 */

/**
 * @file
 * @brief Transmit power: operator-facing dBm -> transceiver drive level.
 *
 * The reference firmware's `LoRaConfig.tx_power` is the power at the ANTENNA,
 * in dBm. On a board with an RF front-end module the transceiver is therefore
 * driven well below that figure, and the FEM's gain makes up the difference
 * (reference: `RadioInterface::limitPower()` ->
 * `LoRaFEMInterface::powerConversion()`).
 *
 * This port previously programmed `tx_power` straight into the SX1262 as a
 * drive level, which on a Heltec V4 meant every configured value radiated
 * ~10-13 dB hotter than the operator asked for. These helpers restore the
 * reference semantics so the number the phone, the admin API and the shell
 * show means the same thing it means on stock firmware.
 */

#ifndef MESHTASTIC_TX_POWER_H_
#define MESHTASTIC_TX_POWER_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic/config.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Resolve a configured tx_power into the desired radiated power.
 *
 * Mirrors the reference (`RadioInterface::applyModemConfig()`): 0 means "as
 * much as this region allows", an over-limit request from an unlicensed
 * operator is clamped to the region limit, and only a region with no declared
 * limit falls back to a fixed default.
 *
 * @param configured `LoRaConfig.tx_power` as stored (0 = unset)
 * @param region     region code; UNSET resolves to the build default
 * @param licensed   operator holds an amateur licence (skips the clamp)
 * @return desired radiated power in dBm, never 0
 */
int8_t meshtastic_tx_power_resolve(int8_t configured,
				   meshtastic_Config_LoRaConfig_RegionCode region,
				   bool licensed);


/**
 * @brief Desired radiated power -> the value to program into the transceiver.
 *
 * Applies the board's FEM conversion and clamps to what the radio can actually
 * be set to. The two steps are gated differently, and the difference is the
 * whole reason @p licensed is a parameter:
 *
 *  - The FEM conversion exists to keep radiated power inside a REGULATORY
 *    limit, so it is skipped for a licensed operator — mirroring the
 *    reference's `if (!is_licensed) power = powerConversion(power);`
 *    (RadioInterface::limitPower).
 *  - The clamp is the transceiver's ELECTRICAL range and applies to everyone.
 *
 * @param radiated_dbm desired power at the antenna, dBm
 * @param licensed     operator holds an amateur licence (skips the FEM backoff)
 * @return the drive level to program into the transceiver, dBm
 */
int8_t meshtastic_tx_power_chip_drive(int8_t radiated_dbm, bool licensed);

#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_TX_POWER_H_ */
