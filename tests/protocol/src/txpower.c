/* Transmit-power resolution and FEM gain conversion.
 *
 * `LoRaConfig.tx_power` is power at the ANTENNA in the reference firmware. These
 * tests pin both halves of restoring that meaning here: resolving a configured
 * value against the region's regulatory limit, and converting the result into
 * the drive level a board with a PA front-end must program.
 *
 * SPDX-License-Identifier: GPL-3.0
 */

#include <zephyr/ztest.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic_tx_power.h"

/* The reference firmware's tables (LoRaFEMInterface.cpp), indexed by drive dBm. */
static const uint8_t gain_gc1109[] = {
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 7,
};
static const uint8_t gain_kct8103l[] = {
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7,
};

/* 0 means "as much as this region allows" -- NOT the no-limit fallback. The US
 * limit is 30 dBm, and reading 17 here was the bug: 17 is what the reference
 * uses only for a region with no declared limit at all. */
ZTEST(txpower, test_zero_resolves_to_region_limit)
{
	zassert_equal(30, meshtastic_tx_power_resolve(
				  0, meshtastic_Config_LoRaConfig_RegionCode_US, false),
		      "US tx_power=0 must resolve to the 30 dBm regional limit");
	zassert_equal(27, meshtastic_tx_power_resolve(
				  0, meshtastic_Config_LoRaConfig_RegionCode_EU_868, false),
		      "EU_868 tx_power=0 must resolve to 27 dBm");
}

/* An explicit request below the limit is the operator's to make. */
ZTEST(txpower, test_explicit_value_is_kept)
{
	zassert_equal(20, meshtastic_tx_power_resolve(
				  20, meshtastic_Config_LoRaConfig_RegionCode_US, false));
}

/* Over the limit is clamped for an unlicensed operator, and honoured for a
 * licensed one (reference: `power > powerLimit && !is_licensed`). */
ZTEST(txpower, test_over_limit_clamped_unless_licensed)
{
	zassert_equal(27, meshtastic_tx_power_resolve(
				  33, meshtastic_Config_LoRaConfig_RegionCode_EU_868, false),
		      "unlicensed over-limit request must clamp to the region limit");
	zassert_equal(33, meshtastic_tx_power_resolve(
				  33, meshtastic_Config_LoRaConfig_RegionCode_EU_868, true),
		      "a licensed operator is not clamped");
}

/* UNSET must not resolve to 0 -- it takes the build's default region. */
ZTEST(txpower, test_unset_region_resolves)
{
	zassert_not_equal(0, meshtastic_tx_power_resolve(
				     0, meshtastic_Config_LoRaConfig_RegionCode_UNSET, false));
}

/* The conversion's defining property: asking for N dBm at the antenna must
 * program a drive level whose drive+gain lands back on N. This is the check
 * that would have failed on the pre-fix firmware, which programmed N directly
 * and so radiated N+gain. */
ZTEST(txpower, test_gain_convert_round_trips)
{
	for (int8_t want = 14; want <= 22; want++) {
		int8_t drive = meshtastic_fem_gain_convert(gain_kct8103l,
							   ARRAY_SIZE(gain_kct8103l), want);

		zassert_true(drive >= 0 && drive < (int8_t)ARRAY_SIZE(gain_kct8103l),
			     "drive %d out of table range for %d dBm", drive, want);
		zassert_equal(want, drive + (int8_t)gain_kct8103l[drive],
			      "asking for %d dBm gave drive %d (+%u dB) = %d", want, drive,
			      gain_kct8103l[drive], drive + gain_kct8103l[drive]);
	}
}

/* Worked example from the reference table, both front-ends: 20 dBm radiated is
 * drive 7 on the KCT8103L (7+13) and drive 9 on the GC1109 (9+11). */
ZTEST(txpower, test_gain_convert_known_points)
{
	zassert_equal(7, meshtastic_fem_gain_convert(gain_kct8103l, ARRAY_SIZE(gain_kct8103l),
						     20));
	zassert_equal(9, meshtastic_fem_gain_convert(gain_gc1109, ARRAY_SIZE(gain_gc1109), 20));
}

/* A request the FEM cannot reach even at full drive falls through to the last
 * table entry rather than running off the end. 30 dBm is exactly this case on
 * both parts, and it is the US default, so it is the common path. */
ZTEST(txpower, test_gain_convert_unreachable_request)
{
	zassert_equal(23, meshtastic_fem_gain_convert(gain_kct8103l,
						      ARRAY_SIZE(gain_kct8103l), 30),
		      "30 dBm exceeds the FEM's reach; expect request minus the top gain");
}

/* No table (detection failed, or a board with no FEM) must be the identity --
 * under-driving is the safe direction to fail in. */
ZTEST(txpower, test_gain_convert_without_table_is_identity)
{
	zassert_equal(17, meshtastic_fem_gain_convert(NULL, 0, 17));
	zassert_equal(17, meshtastic_fem_gain_convert(gain_kct8103l, 0, 17));
}

/* The default weak hook is the identity, and the clamp bounds the result to
 * what the transceiver can actually be set to. */
ZTEST(txpower, test_chip_drive_clamped_to_radio_range)
{
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER,
		      meshtastic_tx_power_chip_drive(127));
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
		      meshtastic_tx_power_chip_drive(-128));
}

ZTEST_SUITE(txpower, NULL, NULL, NULL, NULL, NULL);
