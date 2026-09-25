/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BACKOFF_H_
#define MESHTASTIC_BACKOFF_H_

#include <stdint.h>

/*
 * Capped exponential retry delay, as a caller-owned value. Pure C — no kernel,
 * no clock, no RNG — so the schedule is unit-testable on native_sim and the
 * caller decides where randomness comes from (pass sys_rand32_get() in
 * firmware, a constant in a test).
 *
 * Why it exists: before this, a BLE advertise or scan start that failed was
 * logged once and never retried (the whole service silently dead until a
 * connection came or went), while a failed connect retried with no delay at
 * all (174 attempts in 50 s on the 2026-08-24 bench). Both are the same
 * missing piece: "try again, later, and later still if it keeps failing".
 *
 * Schedule: base, 2*base, 4*base, ... capped at cap. Each delay then gains a
 * jitter of up to a quarter of itself, so nodes that failed together (a whole
 * bench powered up at once) do not retry in lockstep. The result can exceed
 * cap by that quarter — cap bounds the exponential, not the jitter.
 */
struct meshtastic_backoff {
	uint32_t base_ms;
	uint32_t cap_ms;
	uint32_t attempts; /* failures since the last reset */
};

void meshtastic_backoff_init(struct meshtastic_backoff *b, uint32_t base_ms, uint32_t cap_ms);

/* The delay before the next attempt; counts one more failure. */
uint32_t meshtastic_backoff_next_ms(struct meshtastic_backoff *b, uint32_t rand);

/* Success: the next failure starts the ladder from base again. */
void meshtastic_backoff_reset(struct meshtastic_backoff *b);

#endif /* MESHTASTIC_BACKOFF_H_ */
