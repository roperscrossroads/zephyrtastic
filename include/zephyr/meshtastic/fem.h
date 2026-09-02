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
#include <stddef.h>
#include <stdint.h>

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

/**
 * @brief Convert a desired radiated power into a transceiver drive level.
 *
 * `LoRaConfig.tx_power` is the power at the ANTENNA (reference semantics). A
 * board whose FEM adds gain on transmit must therefore drive the transceiver
 * below that figure. The radio layer calls this whenever it programs the
 * radio; the default implementation is a weak identity, correct for any board
 * with no transmit gain between transceiver and antenna.
 *
 * A board overrides it with a strong definition, normally by handing its
 * per-drive-level gain table to @ref meshtastic_fem_gain_convert.
 *
 * @param radiated_dbm desired power at the antenna, dBm
 * @return the drive level to program into the transceiver, dBm (the caller
 *         clamps it to the radio's settable range)
 */
int8_t meshtastic_radio_fem_tx_power_conversion(int8_t radiated_dbm);

/**
 * @brief Map a desired radiated power onto a transceiver drive level.
 *
 * The search a board's @ref meshtastic_radio_fem_tx_power_conversion normally
 * delegates to: walk the FEM's per-drive-level gain table and take the first
 * level whose (drive + gain) exceeds the request — or the last level, if none
 * does — then subtract that level's gain from the request. Reproduces the
 * reference firmware's `LoRaFEMInterface::powerConversion()`.
 *
 * @param gain    gain in dB at drive level 0, 1, 2 ... (index == drive dBm)
 * @param n       entries in @p gain; 0 or NULL @p gain returns @p desired
 * @param desired desired radiated power, dBm
 * @return the drive level to program, dBm
 */
int8_t meshtastic_fem_gain_convert(const uint8_t *gain, size_t n, int8_t desired);

/**
 * @brief Whether this board's front-end exposes a controllable receive LNA.
 *
 * `config.lora.fem_lna_mode` is offered to every config tool (phone app, web
 * client, python CLI), but only means something on hardware whose FEM lets
 * software choose between the low-noise amplifier and a bypass path. The
 * reference normalizes the stored value to NOT_PRESENT where it cannot be
 * honoured rather than keeping a setting that does nothing, so a tool that
 * writes it and reads it back sees the truth.
 *
 * Weak default: false (no controllable LNA).
 */
bool meshtastic_radio_fem_lna_can_control(void);

/**
 * @brief Enable or bypass the front-end's receive LNA.
 *
 * Only called when @ref meshtastic_radio_fem_lna_can_control reports true.
 * Weak default: no-op.
 *
 * @param enable true for the LNA, false for the bypass path.
 */
void meshtastic_radio_fem_lna_set(bool enable);

/**
 * @brief Which receive path the board currently intends to use.
 *
 * @warning This is the board's INTENT, not a pin read, and the distinction is
 * load-bearing rather than pedantic. On the Heltec V4 the LNA select and the
 * TX/RX mode select are the SAME PIN (see that board's
 * meshtastic_radio_fem_set_tx()), so a read-back sampled during a transmit
 * would report "bypass" on a board whose LNA is perfectly well enabled. Intent
 * is the only answer that is correct at every instant, so that is what a
 * diagnostic must report — labelled as intent.
 *
 * Meaningful only where @ref meshtastic_radio_fem_lna_can_control is true.
 * Weak default: false.
 */
bool meshtastic_radio_fem_lna_get(void);

/**
 * @brief Human-readable name of the fitted front-end, or NULL if there is none.
 *
 * For diagnostics only. A board that detects its front-end at runtime (the V4
 * reads a bias line to tell a KCT8103L from a GC1109) returns what it actually
 * found.
 *
 * @note A NULL here does NOT distinguish "no front-end on this hardware" from
 * "detection failed" — they have very different consequences for transmit power
 * but the same spelling in a pointer. This doc comment used to claim a report
 * could tell them apart, which was never true of a `const char *` alone; use
 * @ref meshtastic_radio_fem_state for that.
 *
 * Weak default: NULL.
 */
const char *meshtastic_radio_fem_name(void);

/**
 * @brief How the front-end came to be what it is.
 *
 * Exists because @ref meshtastic_radio_fem_name cannot express it. A board with
 * no front-end at all and a board whose detection failed both return NULL from
 * that call, and the two are not remotely the same thing: the first is correct
 * and the weak identity power conversion is right for it, while the second
 * silently under-drives transmit for the whole run. `meshtastic rf` had to
 * print "none fitted, or detection did not complete" and let the reader guess.
 *
 * The same reasoning as @ref meshtastic_rf_row's ABSENT-vs-UNKNOWN split: a
 * missing readback must never be able to masquerade as a settled answer.
 */
enum meshtastic_fem_state {
	/** No front-end on this hardware. Correct, expected, nothing to report. */
	MESHTASTIC_FEM_STATE_NONE = 0,
	/** Detected and configured; @ref meshtastic_radio_fem_name says which. */
	MESHTASTIC_FEM_STATE_DETECTED,
	/** Detection could not be carried out. The board HAS a front-end and we
	 *  do not know which — so no gain table is in use and transmit is being
	 *  under-driven by that front-end's gain. A fault, not a configuration. */
	MESHTASTIC_FEM_STATE_FAILED,
	/** Detection ran and disagreed with what this board is known to carry.
	 *  Only possible where the build knows statically (the V4-R8 is only ever
	 *  fitted with a KCT8103L). The static fact wins — see that board's
	 *  init — but something is wrong with the hardware or the detect line and
	 *  the report must say so rather than look healthy. */
	MESHTASTIC_FEM_STATE_MISMATCH,
};

/**
 * @brief Report how the front-end was established.
 *
 * Weak default: @ref MESHTASTIC_FEM_STATE_NONE — correct for a board with no
 * front-end, which is the majority and the case the weak hooks exist for.
 */
enum meshtastic_fem_state meshtastic_radio_fem_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_FEM_H_ */
