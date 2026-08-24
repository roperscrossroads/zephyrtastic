/* SPDX-License-Identifier: GPL-3.0 */

/*
 * Transmit power: operator-facing dBm -> transceiver drive level.
 * See meshtastic_tx_power.h for why the two are not the same number.
 */

#include <stdint.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic_region_presets.h"
#include "meshtastic_tx_power.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/*
 * Every figure in this file is dBm carried in an int8_t, and every step is a
 * subtraction that can leave that range. The guards below are deliberately
 * belt-and-braces: the Kconfig ranges bound the constants at build time, these
 * assertions bound their relationship, and the runtime code saturates rather
 * than wrapping. Power is the one setting where a silent wrap does not produce
 * an obviously broken node — it produces a node that transmits at the wrong
 * level and looks fine.
 */
BUILD_ASSERT(CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER <= CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER,
	     "RADIO_MIN_TX_POWER must not exceed RADIO_MAX_TX_POWER — CLAMP() with an "
	     "inverted range silently returns the low bound for every input");
BUILD_ASSERT(CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER >= INT8_MIN &&
		     CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER <= INT8_MAX,
	     "the drive-level clamp must fit the int8_t it is returned in");
BUILD_ASSERT(CONFIG_MESHTASTIC_TX_POWER >= INT8_MIN && CONFIG_MESHTASTIC_TX_POWER <= INT8_MAX,
	     "the no-region-limit fallback must fit the int8_t it is returned in");

/*
 * Sanity bounds for a RADIATED power figure, in dBm.
 *
 * Not a regulatory limit — the region table owns those — but a bound on what
 * can possibly be a real number. The stored value arrives from a config record
 * that may predate a format change, come from a client we did not write, or
 * simply be corrupt, and a nonsense figure there must not propagate into the
 * arithmetic below. -30 dBm is a microwatt; +40 dBm is 10 W, well above any
 * regional allowance including licensed operation.
 */
#define TX_POWER_SANE_MIN (-30)
#define TX_POWER_SANE_MAX (40)

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
		/* A region table entry outside the sane band means the table is
		 * wrong, and honouring it would push a real transmitter to a real
		 * wrong power. Fall back to "no declared limit", which routes to the
		 * conservative CONFIG_MESHTASTIC_TX_POWER fallback below. */
		if ((limit < TX_POWER_SANE_MIN) || (limit > TX_POWER_SANE_MAX)) {
			LOG_ERR("region %d declares an implausible power limit %d dBm; "
				"ignoring it", (int)region, limit);
			limit = 0;
		}
	}

	/* Bound whatever arrived from storage BEFORE it participates in any
	 * comparison or subtraction. A corrupt or foreign-written record is the
	 * realistic source here, and everything downstream assumes a real dBm. */
	if ((configured < TX_POWER_SANE_MIN) || (configured > TX_POWER_SANE_MAX)) {
		LOG_WRN("stored tx_power %d dBm is out of range [%d..%d]; treating it as "
			"unset", configured, TX_POWER_SANE_MIN, TX_POWER_SANE_MAX);
		configured = 0;
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

	/*
	 * The index IS a drive level in dBm — that is the table's contract — so a
	 * table longer than the dBm domain is a board bug, not a bigger table.
	 * Truncate rather than walk indices that cannot be programmed into any
	 * transceiver: entries past this point could only ever be selected by the
	 * "even at maximum drive we cannot reach it" branch, and selecting one
	 * would return a drive level the radio has no way to accept.
	 */
	if (n > (size_t)INT8_MAX) {
		LOG_ERR("FEM gain table has %zu entries; the index is a dBm drive level, "
			"so anything past %d is not programmable — truncating",
			n, INT8_MAX);
		n = (size_t)INT8_MAX;
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
			/* Saturate rather than let the cast wrap. The reference does
			 * the subtraction straight into its int8_t and would wrap, but
			 * no reachable input gets near the edge — meshtastic_tx_power_
			 * resolve() yields region limits, roughly 10..33 dBm. Saturating
			 * is therefore unobservable against the reference for every value
			 * that can actually occur, and keeps the function total for the
			 * ones that cannot: without it, desired = INT8_MIN with a 13 dB
			 * table wraps to +115 and then clamps to the radio's MAXIMUM —
			 * a request for minimum power returning maximum power. */
			int drive = (int)desired - (int)gain[i];

			return (int8_t)CLAMP(drive, INT8_MIN, INT8_MAX);
		}
	}

	return desired;
}

int8_t meshtastic_tx_power_chip_drive(int8_t radiated_dbm, bool licensed)
{
	int8_t drive = radiated_dbm;

	/* The FEM conversion is for staying inside a regulatory limit: it backs the
	 * transceiver off so that drive + front-end gain lands on the permitted
	 * radiated figure. A licensed operator is not bound by that limit, so the
	 * reference skips the conversion entirely for them and drives the
	 * transceiver with the requested number — which, with the front-end's gain
	 * on top, is the extra power the licence entitles them to.
	 *
	 * Reference: RadioInterface::limitPower(),
	 *   if (!devicestate.owner.is_licensed) {
	 *           power = loraFEMInterface.powerConversion(power);
	 *   }
	 * and the same !is_licensed guard on the non-FEM tx_gain path beside it.
	 *
	 * This port applied the conversion unconditionally until 2026-08-23, which
	 * left a licensed operator on a KCT8103L board radiating roughly 6 dB BELOW
	 * vanilla — quietly penalising exactly the operators entitled to more.
	 */
	if (!licensed) {
		drive = meshtastic_radio_fem_tx_power_conversion(radiated_dbm);
	}

	/* The clamp is NOT licence-gated: it is the transceiver's own electrical
	 * range, not a regulation. The reference clamps here too
	 * (limitPower's final `if (power > loraMaxPower)`, SX126X_MAX_POWER = 22). */
	return (int8_t)CLAMP((int)drive, CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
			     CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER);
}
