/* SPDX-License-Identifier: GPL-3.0
 *
 * meshtastic_battery_cutoff.h tests (agents-wu94.1).
 *
 * The state machine this header implements is the actual point of the
 * battery arc -- it is the port's only over-discharge protection, and two
 * bench LiPo cells swelled while this port ran without it (see
 * docs/power-and-battery/BATTERY-STATUS.md). It could not be tested at all
 * while it lived inline in meshtastic_battery.c behind
 * CONFIG_MESHTASTIC_BATTERY_CUTOFF, which depends on HAS_POWEROFF --
 * something native_sim does not have. Extracting it (same move as
 * sx126x_busy_track.h, for the same reason) makes it testable with no ADC, no
 * devicetree, and no poweroff dependency at all: this suite calls the state
 * machine directly with synthetic millivolt readings.
 *
 * What this suite does NOT cover: meshtastic_battery.c's own wiring (does it
 * actually call sys_poweroff() on TRIP, does it read the right Kconfig
 * values, does the BUILD_ASSERT on the poll/cache relationship hold). That
 * needs real hardware, a meter, and a way to recover a board that just
 * powered itself off with no wake source armed -- deferred deliberately
 * (agents-wu94.1) rather than risked on a remote board with no confirmed
 * recovery path.
 */
#include <zephyr/ztest.h>

#include "meshtastic_battery_cutoff.h"

/* Matches the Kconfig defaults (CUTOFF_MV/CUTOFF_COUNT) and the OCV-derived
 * no-battery floor, for realism -- the header itself takes these as
 * parameters, so the tests would work with any values, but using the real
 * ones documents what the shipped behaviour actually is. */
#define NO_BATTERY_MV 2600
#define CUTOFF_MV     3200
#define CUTOFF_COUNT  10

/* Comfortably below/above the two thresholds, so a test's intent survives if
 * either threshold constant above ever changes. */
#define MV_NO_BATTERY 2000 /* < NO_BATTERY_MV */
#define MV_LOW        3000 /* NO_BATTERY_MV <= mv < CUTOFF_MV */
#define MV_HEALTHY    3700 /* >= CUTOFF_MV */

ZTEST_SUITE(meshtastic_battery_cutoff, NULL, NULL, NULL, NULL, NULL);

ZTEST(meshtastic_battery_cutoff, test_no_reading_never_counts)
{
	struct meshtastic_battery_cutoff_state st;

	meshtastic_battery_cutoff_init(&st);

	/* Well past CUTOFF_COUNT iterations: an absent reading must never be
	 * able to switch the node off, no matter how many times it's polled. */
	for (int i = 0; i < CUTOFF_COUNT * 3; i++) {
		enum meshtastic_battery_cutoff_action action = meshtastic_battery_cutoff_update(
			&st, -1, NO_BATTERY_MV, CUTOFF_MV, CUTOFF_COUNT);

		zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_NONE,
			      "iteration %d: a missing reading must never count", i);
		zassert_equal(st.low_voltage_counter, 0, "counter must stay at 0");
		zassert_equal(st.critical, 0, "critical must stay false");
	}
}

ZTEST(meshtastic_battery_cutoff, test_no_battery_fitted_never_counts)
{
	struct meshtastic_battery_cutoff_state st;

	meshtastic_battery_cutoff_init(&st);

	/* A mains-powered node with an empty JST reads here. Must be
	 * indistinguishable from "no reading" to this state machine -- powering
	 * a mains node off because its (nonexistent) cell is "low" would strand
	 * it, per the header's own contract. */
	for (int i = 0; i < CUTOFF_COUNT * 3; i++) {
		enum meshtastic_battery_cutoff_action action = meshtastic_battery_cutoff_update(
			&st, MV_NO_BATTERY, NO_BATTERY_MV, CUTOFF_MV, CUTOFF_COUNT);

		zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_NONE,
			      "iteration %d: below the no-battery floor must never count", i);
		zassert_equal(st.low_voltage_counter, 0, "counter must stay at 0");
	}
}

ZTEST(meshtastic_battery_cutoff, test_healthy_reading_is_a_true_no_op)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* A healthy reading on a state that was never low is NONE, not
	 * RECOVERED -- RECOVERED means something to log; nothing happened here. */
	action = meshtastic_battery_cutoff_update(&st, MV_HEALTHY, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_NONE,
		      "a healthy reading from a fresh state has nothing to recover from");
	zassert_equal(st.critical, 0);
}

ZTEST(meshtastic_battery_cutoff, test_exact_cutoff_boundary_is_healthy)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* CUTOFF_MV itself must count as healthy ("at or above"), not low --
	 * mv == cutoff_mv is the one input that distinguishes >= from a >
	 * mistake, which a test using only comfortably-clear values either side
	 * of the threshold cannot catch. Below it by 1 mV must count as low. */
	action = meshtastic_battery_cutoff_update(&st, CUTOFF_MV, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_NONE,
		      "exactly CUTOFF_MV must be healthy, not low");

	action = meshtastic_battery_cutoff_update(&st, CUTOFF_MV - 1, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_LOW,
		      "1 mV below CUTOFF_MV must count as a low reading");
}

ZTEST(meshtastic_battery_cutoff, test_exact_no_battery_boundary_still_counts)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* NO_BATTERY_MV itself is still a real, low-but-fitted reading ("below
	 * this" is absent, not "at or below") -- the other threshold a test
	 * using only comfortably-clear values cannot distinguish from an off-by-
	 * one. 1 mV below it must be treated as absent instead. */
	action = meshtastic_battery_cutoff_update(&st, NO_BATTERY_MV, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_LOW,
		      "exactly NO_BATTERY_MV must still count as a real, low reading");

	action = meshtastic_battery_cutoff_update(&st, NO_BATTERY_MV - 1, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_NONE,
		      "1 mV below NO_BATTERY_MV must be treated as absent");
	zassert_equal(st.low_voltage_counter, 0, "an absent reading must reset the counter");
}

ZTEST(meshtastic_battery_cutoff, test_recovers_after_a_low_streak)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* A few low readings -- not enough to trip -- then a genuine recovery. */
	for (int i = 0; i < 3; i++) {
		action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
							   CUTOFF_COUNT);
		zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_LOW);
	}
	zassert_equal(st.low_voltage_counter, 3);
	zassert_equal(st.critical, 1, "critical must be set after the first low reading");

	action = meshtastic_battery_cutoff_update(&st, MV_HEALTHY, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_RECOVERED,
		      "a healthy reading after a running counter must report the recovery");
	zassert_equal(st.low_voltage_counter, 0, "the counter must reset on recovery");
	zassert_equal(st.critical, 0, "critical must clear on recovery");
}

ZTEST(meshtastic_battery_cutoff, test_trips_at_exactly_cutoff_count)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* The first CUTOFF_COUNT - 1 low readings must count but NOT trip -- the
	 * whole point of requiring several consecutive readings is to ride out
	 * a transient sag (TX burst, WiFi association) rather than acting on
	 * one. Off-by-one here in either direction is exactly the kind of bug
	 * that either bricks a healthy pack early or never protects a dying one. */
	for (uint8_t i = 1; i < CUTOFF_COUNT; i++) {
		action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
							   CUTOFF_COUNT);
		zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_LOW,
			      "reading %u/%u must not trip yet", i, CUTOFF_COUNT);
		zassert_equal(st.low_voltage_counter, i);
	}

	/* The CUTOFF_COUNT-th consecutive low reading must trip. */
	action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_TRIP,
		      "the %d-th consecutive low reading must trip", CUTOFF_COUNT);
	zassert_equal(st.low_voltage_counter, CUTOFF_COUNT);
	zassert_equal(st.critical, 1);
}

ZTEST(meshtastic_battery_cutoff, test_one_sag_does_not_trip)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	/* The exact scenario the consecutive-count requirement exists for: a
	 * single low reading from a TX-burst sag, immediately followed by
	 * recovery. Must never trip, and must report the recovery. */
	action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_LOW);

	action = meshtastic_battery_cutoff_update(&st, MV_HEALTHY, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_RECOVERED);
}

ZTEST(meshtastic_battery_cutoff, test_stays_tripped_if_polled_again)
{
	struct meshtastic_battery_cutoff_state st;
	enum meshtastic_battery_cutoff_action action;

	meshtastic_battery_cutoff_init(&st);

	for (int i = 0; i < CUTOFF_COUNT; i++) {
		action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
							   CUTOFF_COUNT);
	}
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_TRIP);

	/* Production code never calls update() again after TRIP -- it powers
	 * off, which does not return. This documents that IF something did
	 * (e.g. a caller that logs and defers rather than powering off
	 * immediately), the state stays unambiguously tripped rather than
	 * doing something surprising like un-tripping or wrapping. */
	action = meshtastic_battery_cutoff_update(&st, MV_LOW, NO_BATTERY_MV, CUTOFF_MV,
						   CUTOFF_COUNT);
	zassert_equal(action, MESHTASTIC_BATTERY_CUTOFF_TRIP,
		      "polling again while still low must stay tripped, not reset");
}
