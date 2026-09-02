/* SPDX-License-Identifier: GPL-3.0
 *
 * Effective gain-path readback. See meshtastic_rf_path.h and
 * main/docs/rf-measurement.md.
 *
 * The whole value of this module is the word "effective". Every number here is
 * taken from the driver, the board hooks, or the record of what was actually
 * handed to lora_config() — never from the stored configuration alone, because
 * the two disagree in ways that are otherwise silent:
 *
 *   - `sx126x_rx_boosted_gain` is seeded into the config store and pushed to the
 *     driver at every boot, overriding the devicetree default. So on any
 *     provisioned node the board's `rx-boosted` property is NOT the answer, and
 *     neither is the DT default a useful thing to print on its own.
 *
 *   - The driver only STAGES that value; it reaches the chip on the next
 *     lora_config(). A config that failed leaves the radio on the old gain while
 *     every stored value reads new — 2-3 dB of receive sensitivity gone, with
 *     nothing anywhere reporting a problem.
 *
 *   - `tx_power` is power at the ANTENNA, so on a board with a PA the number
 *     programmed into the transceiver is a different number entirely, and it can
 *     additionally be clamped to the radio's range with no log at all
 *     (meshtastic_tx_power.c).
 *
 * A row is therefore reported with a state, and a row is never omitted just
 * because it does not apply: an omitted row is indistinguishable from a
 * forgotten one.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_rf_path.h"
#include "meshtastic_tx_power.h"

/* Does the transceiver drive the antenna switch itself? A board property, so it
 * is read from devicetree rather than guessed. Absent on boards with no lora0
 * (native_sim builds a simulated radio), which reads as "not applicable". */
#if DT_NODE_EXISTS(DT_NODELABEL(lora0))
#define MT_RF_DIO2_RF_SWITCH DT_PROP_OR(DT_NODELABEL(lora0), dio2_tx_enable, 0)
#else
#define MT_RF_DIO2_RF_SWITCH 0
#endif

const char *meshtastic_rf_row_mark(enum meshtastic_rf_row row)
{
	switch (row) {
	case MESHTASTIC_RF_ROW_OK:
		return "ok";
	case MESHTASTIC_RF_ROW_INEFFECTIVE:
		return "!!";
	case MESHTASTIC_RF_ROW_ABSENT:
		return "--";
	default:
		return "??";
	}
}

/*
 * Judge the receive-gain row.
 *
 * "Applied" is the only thing that decides OK vs INEFFECTIVE, because it is the
 * only one describing the silicon. Staged-but-not-applied is precisely the
 * silent 2-3 dB loss this row exists to catch. Where the driver cannot report
 * either — any non-SX126x radio — the honest answer is UNKNOWN, never OFF.
 */
static enum meshtastic_rf_row rx_boost_row(bool want, enum meshtastic_radio_tristate staged,
					   enum meshtastic_radio_tristate applied)
{
	if (applied == MESHTASTIC_RADIO_TRI_UNKNOWN) {
		/* Staged alone still tells us the request was accepted by the
		 * driver, but not that the chip took it. Do not upgrade that to
		 * "ok" — an unverifiable claim reported as verified is worse than
		 * an admitted gap. */
		ARG_UNUSED(staged);
		return MESHTASTIC_RF_ROW_UNKNOWN;
	}

	return ((applied == MESHTASTIC_RADIO_TRI_ON) == want) ? MESHTASTIC_RF_ROW_OK
							     : MESHTASTIC_RF_ROW_INEFFECTIVE;
}

/*
 * Judge the front-end LNA row.
 *
 * On hardware with no controllable receive path the stored value means nothing,
 * and the config store already normalizes it to NOT_PRESENT with a warning. So
 * ABSENT is the truthful state there — not "off", which would imply a choice
 * was made and lost.
 */
static enum meshtastic_rf_row lna_row(bool can_control,
				      meshtastic_Config_LoRaConfig_FEM_LNA_Mode cfg, bool intent)
{
	bool want;

	if (!can_control) {
		return MESHTASTIC_RF_ROW_ABSENT;
	}

	want = (cfg != meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED);

	return (intent == want) ? MESHTASTIC_RF_ROW_OK : MESHTASTIC_RF_ROW_INEFFECTIVE;
}

int meshtastic_rf_path_get(struct meshtastic_rf_path *out)
{
	meshtastic_Config cfg;
	const meshtastic_Config_LoRaConfig *lora;
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
	if (ret < 0) {
		return ret;
	}
	lora = &cfg.payload_variant.lora;

	/* --- front end ------------------------------------------------ */
	out->fem_name = meshtastic_radio_fem_name();
	out->fem_state = meshtastic_radio_fem_state();
	out->dio2_rf_switch = (MT_RF_DIO2_RF_SWITCH != 0);

	/* --- receive -------------------------------------------------- */
	out->rx_boost_config = lora->sx126x_rx_boosted_gain;
	out->rx_boost_staged = meshtastic_radio_rx_boosted_staged();
	out->rx_boost_applied = meshtastic_radio_rx_boosted_applied();
	out->rx_boost_row =
		rx_boost_row(out->rx_boost_config, out->rx_boost_staged, out->rx_boost_applied);

	out->lna_can_control = meshtastic_radio_fem_lna_can_control();
	out->lna_intent = meshtastic_radio_fem_lna_get();
	out->lna_config = lora->fem_lna_mode;
	out->lna_row = lna_row(out->lna_can_control, out->lna_config, out->lna_intent);

	out->rx_armed = mt.radio_rx_armed;

	/* --- transmit ------------------------------------------------- */
	out->tx_enabled = mt.tx_enabled;
	out->region = lora->region;
	/* The CACHED flag, not a fresh read of the config store. This report is
	 * about what the radio is doing, and the transmit path reads mt.licensed —
	 * a store value that has been written but not yet applied would make the
	 * report disagree with the hardware it is describing. */
	out->licensed = mt.licensed;

	out->tx_power_stored = lora->tx_power;
	out->tx_power_radiated =
		meshtastic_tx_power_resolve(lora->tx_power, lora->region, out->licensed);

	/* Both sides of the clamp, because it happens silently: a request outside
	 * the radio's settable range is quietly moved with no log and no readback
	 * (meshtastic_tx_power.c). Showing wanted and clamped separately is what
	 * turns that into something an operator can see.
	 *
	 * The FEM backoff mirrors the gate in meshtastic_tx_power_chip_drive() — it
	 * is skipped for a licensed operator, who is not bound by the regulatory
	 * limit it exists to respect. Recomputing it the same way here rather than
	 * calling the conversion unconditionally keeps the report describing what
	 * the transmit path actually does; the two drifting apart would make this
	 * screen confidently wrong about the one number it exists to explain. */
	out->tx_drive_wanted =
		out->licensed ? out->tx_power_radiated
			      : meshtastic_radio_fem_tx_power_conversion(out->tx_power_radiated);
	out->tx_drive_clamped =
		meshtastic_tx_power_chip_drive(out->tx_power_radiated, out->licensed);
	out->tx_drive_was_clamped = (out->tx_drive_wanted != out->tx_drive_clamped);

	/* Separate the two reasons the drive differs from the request. Only a real
	 * front-end that moved the number counts as gain; on a bare transceiver the
	 * conversion is the weak identity, and for a licensed operator it is not
	 * applied at all — in both cases any difference is pure clamping. */
	out->tx_fem_gain_applied =
		(out->fem_name != NULL) && (out->tx_drive_wanted != out->tx_power_radiated);

	{
		struct meshtastic_region_info info;

		if (meshtastic_region_info(lora->region, &info) == 0) {
			out->region_limit_dbm = info.power_limit_dbm;
		}
	}

	/* --- what the radio was really told --------------------------- */
	(void)meshtastic_radio_effective_get(&out->eff);

	/* --- health --------------------------------------------------- */
	out->cad_clear = meshtastic_radio_cad_clear_count();
	out->cad_busy = meshtastic_radio_cad_busy_count();
	out->cad_timeout = meshtastic_radio_cad_timeout_count();
	out->cad_error = meshtastic_radio_cad_error_count();
	out->agc_ok = meshtastic_radio_agc_reset_ok_count();
	out->agc_fail = meshtastic_radio_agc_reset_fail_count();
	out->agc_skipped = meshtastic_radio_agc_reset_skipped_count();
	out->agc_patch_fail = meshtastic_radio_agc_patch_fail_count();
	out->busy_streak = meshtastic_radio_busy_timeout_streak();

	return 0;
}
