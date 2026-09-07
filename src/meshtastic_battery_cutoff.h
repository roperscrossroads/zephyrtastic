/* SPDX-License-Identifier: GPL-3.0
 *
 * Low-voltage cutoff decision logic, split out of meshtastic_battery.c so it
 * can be unit-tested (main/tests/battery_cutoff). That is not cosmetic:
 * CONFIG_MESHTASTIC_BATTERY_CUTOFF depends on HAS_POWEROFF, which native_sim
 * does not have, so the state machine could not be exercised in sim AT ALL
 * while it lived inline behind that Kconfig -- exactly the shape of gap that
 * let carried patches 0020/0021 (sx126x_busy_track.h's own history) reach
 * hardware twice. No Zephyr dependencies on purpose: a test should be able to
 * include this and nothing else.
 *
 * This file makes exactly one decision -- consecutive-low-readings counting
 * with two escape hatches (no reading, no battery fitted) that must never be
 * allowed to trip the cutoff -- and nothing about WHEN to poll, HOW to power
 * off, or what to log. meshtastic_battery.c owns all of that; this owns only
 * "should the caller act, and on what."
 */

#ifndef MESHTASTIC_BATTERY_CUTOFF_H_
#define MESHTASTIC_BATTERY_CUTOFF_H_

#include <stdint.h>

struct meshtastic_battery_cutoff_state {
	uint8_t low_voltage_counter;
	uint8_t critical; /* bool, but keep this header's only dependency stdint.h */
};

enum meshtastic_battery_cutoff_action {
	/* No reading, or no cell fitted: counter reset, nothing happened, nothing
	 * to log. An unreadable ADC or a mains node's empty JST must never be
	 * able to switch the node off, or even nudge the counter toward it. */
	MESHTASTIC_BATTERY_CUTOFF_NONE = 0,
	/* Reading recovered at/above the cutoff voltage while the counter was
	 * running: counter reset, caller should log the recovery. */
	MESHTASTIC_BATTERY_CUTOFF_RECOVERED,
	/* Reading below the cutoff voltage, counter incremented but still under
	 * the trip threshold: caller should log "low: N/M readings". */
	MESHTASTIC_BATTERY_CUTOFF_LOW,
	/* Counter just reached the trip threshold: caller MUST power off. Only
	 * returned once per low-voltage episode -- the counter does not wrap or
	 * re-trip every poll after this, since state->critical latches and a
	 * poweroff caller does not return anyway. */
	MESHTASTIC_BATTERY_CUTOFF_TRIP,
};

static inline void meshtastic_battery_cutoff_init(struct meshtastic_battery_cutoff_state *state)
{
	state->low_voltage_counter = 0U;
	state->critical = 0U;
}

/**
 * Feed one battery reading into the cutoff state machine.
 *
 * @param state           Caller-owned state, persisted across calls.
 * @param mv              The reading in millivolts, or a negative value for
 *                        "no reading at all" (ADC not ready, every sample in
 *                        the averaging loop failed).
 * @param no_battery_mv   Below this, treat mv the same as "no reading" -- a
 *                        mains-powered node with an empty JST reads here, and
 *                        must never be penalised as if its cell were dying.
 * @param cutoff_mv       At or above this, the pack is fine; below it, the
 *                        low-voltage counter runs.
 * @param cutoff_count    Consecutive low readings required to reach TRIP.
 *
 * @return What the caller should do. state->low_voltage_counter and
 *         state->critical are updated in place; read them after the call if
 *         the caller wants the numbers for its own log line.
 */
static inline enum meshtastic_battery_cutoff_action
meshtastic_battery_cutoff_update(struct meshtastic_battery_cutoff_state *state, int mv,
				  int no_battery_mv, int cutoff_mv, uint8_t cutoff_count)
{
	if (mv < 0 || mv < no_battery_mv) {
		state->low_voltage_counter = 0U;
		state->critical = 0U;
		return MESHTASTIC_BATTERY_CUTOFF_NONE;
	}

	if (mv >= cutoff_mv) {
		uint8_t was_running = state->low_voltage_counter != 0U;

		state->low_voltage_counter = 0U;
		state->critical = 0U;
		return was_running ? MESHTASTIC_BATTERY_CUTOFF_RECOVERED
				    : MESHTASTIC_BATTERY_CUTOFF_NONE;
	}

	state->critical = 1U;
	if (state->low_voltage_counter < UINT8_MAX) {
		state->low_voltage_counter++;
	}

	if (state->low_voltage_counter >= cutoff_count) {
		return MESHTASTIC_BATTERY_CUTOFF_TRIP;
	}
	return MESHTASTIC_BATTERY_CUTOFF_LOW;
}

#endif /* MESHTASTIC_BATTERY_CUTOFF_H_ */
