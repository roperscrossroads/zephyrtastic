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

#include <zephyr/meshtastic/nodedb.h>

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

/*
 * The identity above is right for BOTH "no front-end" and "detection failed",
 * which is exactly why those two must stay tellable apart somewhere else: the
 * power conversion cannot distinguish them and neither could the diagnostic,
 * which printed "none fitted, or detection did not complete" and left the
 * reader to guess. On a Heltec V4 the second case is silently 11-13 dB of lost
 * transmit power; on a XIAO the first case is simply correct.
 *
 * native_sim compiles no board file, so the weak default is what is under test
 * here -- and NONE is the right weak answer, because a board that detects at
 * runtime is obliged to override it (the V4 file does, and starts at FAILED so
 * an early return cannot masquerade as a bare transceiver).
 */
ZTEST(txpower, test_fem_state_weak_default_is_none_not_failed)
{
	zassert_equal(MESHTASTIC_FEM_STATE_NONE, meshtastic_radio_fem_state(),
		      "a board with no FEM hook must report NONE -- reporting FAILED would "
		      "make every bare-transceiver board look broken");
}

/* NONE must be the zero value: a memset-cleared report struct has to mean "no
 * front-end", not "detection failed", or a field nobody populated would raise
 * a fault that does not exist. */
ZTEST(txpower, test_fem_state_none_is_the_zero_value)
{
	zassert_equal(0, (int)MESHTASTIC_FEM_STATE_NONE);
	zassert_not_equal(0, (int)MESHTASTIC_FEM_STATE_FAILED);
	zassert_not_equal(0, (int)MESHTASTIC_FEM_STATE_MISMATCH);
	zassert_not_equal(0, (int)MESHTASTIC_FEM_STATE_DETECTED);
}

/*
 * The safety property behind the V4 board file's choice of starting gain table:
 * when the fitted part is UNKNOWN, converting with the highest-gain table can
 * never radiate above the request, whichever part is actually there.
 *
 * The board powers its FEM rail before it reads the detect line, so a failed
 * read leaves the front-end possibly ENABLED and unidentified. Converting with
 * the identity there (what the code did until 2026-09-02, under a comment
 * calling it the safe direction) drives the full request into ~13 dB of gain.
 * This asserts the direction rather than the mechanism, so it keeps holding if
 * the tables are ever re-sourced.
 */
ZTEST(txpower, test_unknown_fem_assumption_never_over_radiates)
{
	/* Every radiated request a region can ask for, per meshtastic_tx_power_resolve. */
	for (int8_t want = 1; want <= 33; want++) {
		int8_t drive = meshtastic_fem_gain_convert(gain_kct8103l,
							   ARRAY_SIZE(gain_kct8103l), want);

		/* Whichever part is really fitted, radiated = drive + that part's gain
		 * at that drive level. Assuming the biggest table must not let either
		 * real part exceed the request. */
		for (size_t i = 0; i < ARRAY_SIZE(gain_gc1109); i++) {
			if ((int)i != (int)drive) {
				continue;
			}
			zassert_true((int)drive + (int)gain_gc1109[i] <= (int)want,
				     "GC1109 fitted, KCT assumed: %d dBm drive + %u dB "
				     "would radiate above the %d dBm requested",
				     drive, gain_gc1109[i], want);
			zassert_true((int)drive + (int)gain_kct8103l[i] <= (int)want,
				     "KCT8103L fitted and assumed: %d dBm drive + %u dB "
				     "would radiate above the %d dBm requested",
				     drive, gain_kct8103l[i], want);
		}
	}
}

/* And the converse, which is what makes the choice above the RIGHT one rather
 * than merely a cautious one: assuming the SMALLER table while the bigger part
 * is fitted does over-radiate. Documents the trap so nobody "simplifies" the
 * board file back to a single shared table. */
ZTEST(txpower, test_assuming_the_smaller_gain_would_over_radiate)
{
	int8_t drive = meshtastic_fem_gain_convert(gain_gc1109, ARRAY_SIZE(gain_gc1109), 14);

	zassert_true((int)drive + (int)gain_kct8103l[drive] > 14,
		     "assuming GC1109 (11 dB) while a KCT8103L (13 dB) is fitted must be "
		     "the direction that radiates hot — if this ever stops being true the "
		     "tables have changed and the board file's assumption needs revisiting");
}

/*
 * Strong override of the weak board hook. native_sim compiles no board file, so
 * without this every conversion is the identity and the licence gate below
 * cannot be observed at all. Uses the real KCT8103L table so the numbers are the
 * ones a Heltec V4 rev 4.3 actually produces.
 */
int8_t meshtastic_radio_fem_tx_power_conversion(int8_t radiated_dbm)
{
	return meshtastic_fem_gain_convert(gain_kct8103l, ARRAY_SIZE(gain_kct8103l), radiated_dbm);
}

/* The default weak hook is the identity, and the clamp bounds the result to
 * what the transceiver can actually be set to.
 *
 * The clamp is NOT licence-gated — it is the transceiver's electrical range,
 * not a regulation — so both operators land on the same rails. */
ZTEST(txpower, test_chip_drive_clamped_to_radio_range)
{
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER,
		      meshtastic_tx_power_chip_drive(127, false));
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
		      meshtastic_tx_power_chip_drive(-128, false));
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER,
		      meshtastic_tx_power_chip_drive(127, true));
	zassert_equal(CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
		      meshtastic_tx_power_chip_drive(-128, true));
}

/*
 * The INT8_MIN case above is the one that found a real defect, so it gets its
 * own name rather than hiding inside a clamp test.
 *
 * meshtastic_fem_gain_convert() subtracts the front-end's gain from the request.
 * At the bottom of the range that subtraction used to overflow int8_t —
 * -128 - 13 wraps to +115 — and the clamp then pinned it to the radio's
 * MAXIMUM. A request for minimum power returning maximum power is the worst
 * possible direction for that bug to fail in, and it stayed invisible until a
 * real gain table was put behind the weak hook.
 *
 * Unreachable from a real config (tx_power_resolve yields ~10..33 dBm), which is
 * exactly why it wants a test: nothing else would ever exercise it.
 */
ZTEST(txpower, test_fem_conversion_saturates_instead_of_wrapping)
{
	zassert_equal(INT8_MIN,
		      meshtastic_fem_gain_convert(gain_kct8103l, ARRAY_SIZE(gain_kct8103l),
						  INT8_MIN),
		      "the gain subtraction must saturate at INT8_MIN, not wrap positive");
}

/*
 * The FEM backoff is skipped for a licensed operator.
 *
 * Reference: RadioInterface::limitPower() applies the conversion only inside
 * `if (!devicestate.owner.is_licensed)`. The backoff exists to keep RADIATED
 * power inside a regulatory limit, and a licensed operator is not bound by that
 * limit — so skipping it is what gives them the extra power the licence allows.
 *
 * This port applied the conversion unconditionally until 2026-08-23, which left
 * a licensed operator on a front-end board radiating roughly 6 dB BELOW vanilla.
 * Pinned here because the failure is entirely silent: nothing logs, nothing
 * errors, the node is simply quieter than the reference for the one class of
 * operator entitled to be louder.
 *
 * native_sim builds no board file, so the weak identity hook is overridden here
 * with a real gain table. Without that the test would be vacuous: with the
 * identity conversion both paths return the same number and the gate could be
 * deleted entirely without failing anything.
 */
ZTEST(txpower, test_licensed_operator_keeps_the_full_drive_level)
{
	/* 20 dBm requested, KCT8103L table: the unlicensed path finds drive 7
	 * (7 + 13 = 20 at the antenna). Both are inside the radio's range, so the
	 * clamp cannot mask the difference. */
	zassert_equal(7, meshtastic_tx_power_chip_drive(20, false),
		      "an unlicensed operator must be backed off by the front-end's gain");
	zassert_equal(20, meshtastic_tx_power_chip_drive(20, true),
		      "a licensed operator's request must reach the transceiver un-backed-off");
}

ZTEST_SUITE(txpower, NULL, NULL, NULL, NULL, NULL);

/* --- NodeInfoLite SNR persistence (Q4) -----------------------------------
 *
 * SNR used to be dropped entirely when a node record was written, so a reboot
 * blanked the link-quality column until every peer was heard again. It is now
 * persisted as the Q4 integer the reference uses; these pin that conversion,
 * which is the part that can silently lose sign or precision.
 *
 * Comparisons stay in the integer domain on purpose: Q4's defining property is
 * that a quarter-dB step IS an integer, so a float compare would be testing the
 * test rather than the encoding.
 */

ZTEST(txpower, test_snr_q4_round_trips_at_quarter_db)
{
	for (int q = -80; q <= 80; q++) {
		float db = meshtastic_snr_from_q4(q);

		zassert_equal(q, meshtastic_snr_to_q4(db),
			      "q4 %d -> dB -> q4 did not come back to %d", q, q);
	}
}

/* Negative SNR is the normal case on a real link and must not truncate toward
 * zero: a node at -7.5 dB has to persist as -30, not -29. */
ZTEST(txpower, test_snr_q4_handles_negative_values)
{
	zassert_equal(-30, meshtastic_snr_to_q4(-7.5f));
	zassert_equal(-30, meshtastic_snr_to_q4(meshtastic_snr_from_q4(-30)));
}

/* Off-grid values round to nearest in both directions, rather than toward zero
 * (which would bias every negative reading upward). */
ZTEST(txpower, test_snr_q4_rounds_to_nearest)
{
	zassert_equal(51, meshtastic_snr_to_q4(12.7f));
	zassert_equal(-51, meshtastic_snr_to_q4(-12.7f));
}

/* The range a LoRa radio actually reports (~ -20..+13 dB) maps to |q4| <= 80,
 * which zigzag-encodes in one or two bytes -- the reason to persist this rather
 * than the 5-byte float. */
ZTEST(txpower, test_snr_q4_typical_readings_stay_small)
{
	zassert_equal(52, meshtastic_snr_to_q4(13.0f));
	zassert_equal(-80, meshtastic_snr_to_q4(-20.0f));
}

/* --- sanity bounds --------------------------------------------------------
 *
 * Power is the one setting where a bad value does not produce an obviously
 * broken node — it produces a node transmitting at the wrong level that looks
 * completely healthy. So the arithmetic is pinned at its edges, not just in
 * its normal range.
 */

/*
 * A stored tx_power outside any plausible dBm band must be treated as unset
 * rather than propagated. The realistic source is not malice but a config
 * record written by a different firmware version, or one that got corrupted;
 * either way it must not reach the transceiver.
 */
ZTEST(txpower, test_resolve_rejects_implausible_stored_power)
{
	/* Both of these are "treat as unset", so they resolve to the US limit
	 * exactly as a stored 0 would. */
	zassert_equal(30, meshtastic_tx_power_resolve(
				  INT8_MAX, meshtastic_Config_LoRaConfig_RegionCode_US, false),
		      "an absurdly high stored power must fall back, not be honoured");
	zassert_equal(30, meshtastic_tx_power_resolve(
				  INT8_MIN, meshtastic_Config_LoRaConfig_RegionCode_US, false),
		      "an absurdly low stored power must fall back, not be honoured");

	/* And the licensed path must not become a way to smuggle a nonsense value
	 * through: licensing lifts the REGIONAL clamp, not the sanity bound. */
	zassert_equal(30, meshtastic_tx_power_resolve(
				  INT8_MAX, meshtastic_Config_LoRaConfig_RegionCode_US, true),
		      "a licence must not make an implausible stored power legitimate");
}

/*
 * Whatever the inputs, the value programmed into the transceiver must land
 * inside the radio's electrical range. This sweeps the entire int8_t domain
 * through both licence states, because the point is that NO input escapes.
 */
ZTEST(txpower, test_chip_drive_never_escapes_the_radio_range)
{
	for (int v = INT8_MIN; v <= INT8_MAX; v++) {
		for (int lic = 0; lic <= 1; lic++) {
			int8_t drive = meshtastic_tx_power_chip_drive((int8_t)v, lic != 0);

			zassert_true(drive >= CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER &&
					     drive <= CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER,
				     "drive %d escaped [%d..%d] for input %d (licensed=%d)",
				     drive, CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
				     CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER, v, lic);
		}
	}
}

/*
 * Same sweep for the resolver: every input, every region in the enum's range,
 * must come back inside the sane band. A region code we do not know must not
 * produce a wild number — it must fall back.
 */
ZTEST(txpower, test_resolve_always_returns_a_plausible_power)
{
	for (int v = INT8_MIN; v <= INT8_MAX; v += 7) {
		for (int r = 0; r < 32; r++) {
			int8_t p = meshtastic_tx_power_resolve(
				(int8_t)v, (meshtastic_Config_LoRaConfig_RegionCode)r, false);

			zassert_true(p >= -30 && p <= 40,
				     "resolve returned %d dBm for stored %d region %d", p, v, r);
		}
	}
}
