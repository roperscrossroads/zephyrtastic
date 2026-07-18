/* SPDX-License-Identifier: GPL-3.0 */

/**
 * @file
 * @brief RF front-end module (FEM) board hook.
 *
 * Some boards place an external PA/LNA front-end between the LoRa transceiver
 * and the antenna. When the transceiver's own antenna-switch line (e.g. the
 * SX126x DIO2) drives only the FEM's TX/RX *path*-select pin, a *second* FEM
 * control pin (PA-mode / LNA-enable) still has to be steered in software as the
 * radio moves between transmit and receive.
 *
 * The radio layer calls @ref meshtastic_radio_fem_set_tx around every transmit.
 * The default implementation is a weak no-op, so boards with no FEM (or whose
 * FEM is fully handled in hardware) need do nothing. A board with a FEM
 * overrides it with a strong definition that drives its mode pin.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_FEM_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_FEM_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Steer the RF front-end for the current transceiver direction.
 *
 * Called by the radio layer immediately before it keys the transmitter and
 * again after the transmission completes (before re-arming receive).
 *
 * @param tx @c true when the radio is about to transmit, @c false when it is
 *           returning to receive.
 */
void meshtastic_radio_fem_set_tx(bool tx);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_FEM_H_ */
