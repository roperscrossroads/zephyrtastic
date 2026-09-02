/* SPDX-License-Identifier: GPL-3.0
 *
 * Effective gain-path readback — the data behind `meshtastic rf`.
 *
 * Deliberately a struct-filling API rather than a print-directly one, for two
 * reasons: the shell can format it without this module knowing about shells,
 * and a test can assert on the values without parsing text.
 */
#ifndef MESHTASTIC_RF_PATH_H_
#define MESHTASTIC_RF_PATH_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_core.h"

/**
 * @brief How a reported row stands.
 *
 * ABSENT and UNKNOWN are distinct on purpose. "This hardware has no
 * controllable LNA" is a fact about the board; "the driver cannot tell us what
 * the chip is doing" is a gap in our visibility. Collapsing the two would let a
 * missing readback masquerade as a settled answer.
 */
enum meshtastic_rf_row {
	MESHTASTIC_RF_ROW_OK = 0,   /**< configured and in effect */
	MESHTASTIC_RF_ROW_INEFFECTIVE, /**< configured, but NOT what the radio is doing */
	MESHTASTIC_RF_ROW_ABSENT,   /**< not present on this hardware */
	MESHTASTIC_RF_ROW_UNKNOWN,  /**< could not be determined */
};

/** @brief One snapshot of the whole chain, antenna to bits. */
struct meshtastic_rf_path {
	/* --- front end ------------------------------------------------ */
	const char *fem_name;  /**< NULL when none is fitted or detection failed —
				*   `fem_state` is what tells those two apart */
	enum meshtastic_fem_state fem_state;
	bool dio2_rf_switch;   /**< the transceiver drives the T/R switch itself */

	/* --- receive -------------------------------------------------- */
	bool rx_boost_config;  /**< what the stored LoRaConfig asks for */
	enum meshtastic_radio_tristate rx_boost_staged;
	enum meshtastic_radio_tristate rx_boost_applied;
	enum meshtastic_rf_row rx_boost_row;

	bool lna_can_control;
	bool lna_intent;       /**< board INTENT, not a pin read — see fem.h */
	meshtastic_Config_LoRaConfig_FEM_LNA_Mode lna_config;
	enum meshtastic_rf_row lna_row;

	bool rx_armed;

	/* --- transmit ------------------------------------------------- */
	bool tx_enabled;
	int8_t tx_power_stored;   /**< 0 means "whatever the region allows" */
	int8_t tx_power_radiated; /**< what that resolved to, dBm at the antenna */
	int8_t tx_drive_wanted;   /**< after FEM conversion, before clamping */
	int8_t tx_drive_clamped;  /**< what actually gets programmed */
	bool tx_drive_was_clamped;
	meshtastic_Config_LoRaConfig_RegionCode region;
	int8_t region_limit_dbm; /**< 0 when the region declares no limit */
	bool licensed;
	/* True only when a front-end is actually contributing transmit gain. The
	 * drive level differs from the requested power for TWO unrelated reasons —
	 * FEM gain and range clamping — and saying "the FEM makes up the rest" on a
	 * board with no FEM would be a confident lie. */
	bool tx_fem_gain_applied;

	/* --- what the radio was really told --------------------------- */
	struct meshtastic_radio_effective eff;

	/* --- health --------------------------------------------------- */
	uint32_t cad_clear, cad_busy, cad_timeout, cad_error;
	uint32_t agc_ok, agc_fail, agc_skipped, agc_patch_fail;
	uint32_t busy_streak;
};

/**
 * @brief Fill @p out with the current effective gain path.
 *
 * @return 0, -EINVAL for a NULL @p out, or a negative errno if the stored LoRa
 *         config could not be read (in which case @p out is left zeroed).
 */
int meshtastic_rf_path_get(struct meshtastic_rf_path *out);

/** @brief Short label for a row state: "ok", "!!", "--", "??". */
const char *meshtastic_rf_row_mark(enum meshtastic_rf_row row);

#endif /* MESHTASTIC_RF_PATH_H_ */
