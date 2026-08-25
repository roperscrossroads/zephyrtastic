/* SPDX-License-Identifier: GPL-3.0
 *
 * Regulatory duty-cycle gate (src/meshtastic_duty.c).
 *
 * This is a legal ceiling, so the tests are written around the ways it could be
 * wrong in a direction that matters: failing to block when it should (a
 * regulatory violation), blocking when it should not (a bricked node), and
 * getting the EU_866 role split backwards.
 *
 * Time is driven with k_sleep(), which fast-forwards under native_sim, so the
 * one-hour TX window is exercised for real rather than mocked. Percentages come
 * from the airtime ring: tx% = ms / 36000 (see meshtastic_airtime.c), so
 * 400000 ms is ~11.1% of an hour.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "meshtastic_airtime.h"
#include "meshtastic_channels.h"
#include "meshtastic_duty.h"

#define MS_FOR_11_PERCENT 400000U
#define MS_FOR_1_PERCENT  36000U

static void duty_before(void *fixture)
{
	ARG_UNUSED(fixture);
	(void)meshtastic_airtime_init();
	meshtastic_duty_stats_reset();
	meshtastic_duty_set_override(false);
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	/* Default every test to an unrestricted region; the ones that care set
	 * their own. */
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_US, 100.0f);
}

ZTEST_SUITE(duty, NULL, NULL, duty_before, NULL, NULL);

/* ---- the ceiling ---- */

ZTEST(duty, test_unrestricted_region_never_blocks)
{
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT * 4U);

	zassert_within(meshtastic_duty_effective_pct(), 100.0f, 0.001f,
		       "US sets no duty cycle");
	zassert_false(meshtastic_duty_blocked(NULL),
		      "a region with no ceiling must never refuse a send, however busy");
}

ZTEST(duty, test_restricted_region_blocks_past_the_ceiling)
{
	uint8_t silent = 0U;

	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 10.0f);

	zassert_false(meshtastic_duty_blocked(NULL), "an idle node is under any ceiling");

	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);

	zassert_true(meshtastic_duty_blocked(&silent),
		     "11%% of the hour spent transmitting is over a 10%% ceiling");
	zassert_true(silent > 0U, "a blocked node must be told when it can send again");
}

/* Exactly at the ceiling is allowed; upstream's comparison is `>`, not `>=`,
 * and a node sitting precisely on its allocation has not exceeded it. */
ZTEST(duty, test_exactly_at_the_ceiling_is_allowed)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 10.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_1_PERCENT * 10U);

	zassert_false(meshtastic_duty_blocked(NULL),
		      "at the ceiling is not past it (%d%% used)",
		      (int)meshtastic_airtime_tx_util_percent());
}

/* A move from a restricted region to an unrestricted one must actually lift the
 * limit. The ceiling is latched at config-apply, so if that latch only ran when
 * the new region HAD a ceiling, a node moved from EU_868 to US would carry the
 * 10% limit forever. */
ZTEST(duty, test_region_change_lifts_the_limit)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 10.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);
	zassert_true(meshtastic_duty_blocked(NULL), "precondition: blocked in EU_868");

	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_US, 100.0f);

	zassert_false(meshtastic_duty_blocked(NULL),
		      "moving to a region with no ceiling must release the gate");
}

/* A zero or negative ceiling in a band table would mean "never transmit". That
 * is a data bug, and muting the radio is the wrong response to one. */
ZTEST(duty, test_implausible_ceiling_is_treated_as_unrestricted)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 0.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);

	zassert_false(meshtastic_duty_blocked(NULL),
		      "a 0%% ceiling is a bad table entry, not an order to go silent");
}

/* ---- the override ---- */

ZTEST(duty, test_override_disables_enforcement)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 10.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);
	zassert_true(meshtastic_duty_blocked(NULL), "precondition: blocked");

	meshtastic_duty_set_override(true);
	zassert_false(meshtastic_duty_blocked(NULL),
		      "config.lora.override_duty_cycle must disable the gate, as upstream");

	meshtastic_duty_set_override(false);
	zassert_true(meshtastic_duty_blocked(NULL), "and clearing it must restore the gate");
}

/* ---- EU_866's role split ---- */

ZTEST(duty, test_eu866_router_gets_the_larger_allocation)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_866, 10.0f);

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	zassert_within(meshtastic_duty_effective_pct(), 2.5f, 0.001f,
		       "a non-router in EU_866 gets 2.5%%");

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	zassert_within(meshtastic_duty_effective_pct(), 10.0f, 0.001f,
		       "a ROUTER in EU_866 gets 10%%");

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
	zassert_within(meshtastic_duty_effective_pct(), 10.0f, 0.001f,
		       "ROUTER_LATE counts as a router here too");
}

/* The split has to bite, not merely be reported: 5% of an hour is under a
 * router's 10% and over a client's 2.5%. */
ZTEST(duty, test_eu866_split_changes_the_verdict)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_866, 10.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_1_PERCENT * 5U);

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	zassert_true(meshtastic_duty_blocked(NULL), "5%% is over a client's 2.5%%");

	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER);
	zassert_false(meshtastic_duty_blocked(NULL), "but under a router's 10%%");
}

/* ---- when can I send again ---- */

ZTEST(duty, test_silent_minutes_is_zero_when_under)
{
	zassert_equal(meshtastic_airtime_silent_minutes(1.0f, 10.0f), 0U,
		      "already under the ceiling means no wait");
}

/* Airtime spent in the CURRENT minute is the last thing to age out, so burning
 * the whole allowance now costs the full hour. */
ZTEST(duty, test_silent_minutes_full_hour_for_a_fresh_burst)
{
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);

	zassert_equal(meshtastic_airtime_silent_minutes(
			      meshtastic_airtime_tx_util_percent(), 10.0f),
		      60U, "a burst in the current minute expires last");
}

/* And the wait shrinks as the burst ages: after five minutes the same sample is
 * five minutes closer to falling out of the window. */
ZTEST(duty, test_silent_minutes_shrinks_as_the_burst_ages)
{
	uint8_t before, after;

	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);
	before = meshtastic_airtime_silent_minutes(meshtastic_airtime_tx_util_percent(), 10.0f);

	k_sleep(K_SECONDS(5 * 60));

	after = meshtastic_airtime_silent_minutes(meshtastic_airtime_tx_util_percent(), 10.0f);

	zassert_equal(before, 60U, "precondition");
	zassert_equal(after, 55U, "five minutes later the wait should be five minutes shorter");
}

/* The window really does drain: an hour after the burst the node is free again.
 * This is the assertion that says the gate is temporary rather than a latch. */
ZTEST(duty, test_the_gate_releases_after_the_window_passes)
{
	meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode_EU_868, 10.0f);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, MS_FOR_11_PERCENT);
	zassert_true(meshtastic_duty_blocked(NULL), "precondition: blocked");

	k_sleep(K_SECONDS(61 * 60));

	zassert_false(meshtastic_duty_blocked(NULL),
		      "an hour on, the burst has left the window and sending resumes");
}

/* ---- counters ---- */

ZTEST(duty, test_relay_refusals_are_counted_separately)
{
	struct meshtastic_duty_stats stats;

	meshtastic_duty_note_blocked(false, 7U);
	meshtastic_duty_note_blocked(true, 7U);
	meshtastic_duty_note_blocked(true, 6U);

	meshtastic_duty_stats_get(&stats);
	zassert_equal(stats.blocked, 3U, "every refusal counts");
	zassert_equal(stats.blocked_relay, 2U,
		      "and relays are counted apart, because 'my own traffic is being "
		      "refused' and 'I am declining to relay for others' are different "
		      "operational facts");
	zassert_equal(stats.last_silent_minutes, 6U, "the most recent estimate is kept");

	meshtastic_duty_stats_reset();
	meshtastic_duty_stats_get(&stats);
	zassert_equal(stats.blocked, 0U, "reset clears");
}
