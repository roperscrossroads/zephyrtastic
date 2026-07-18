/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper. The port's timestamps are otherwise k_uptime-relative;
 * this establishes a Unix epoch once a real time source is available (GNSS UTC
 * or the phone's set_time_only admin message) so uptime-relative values can be
 * reported to the app as epoch seconds (e.g. NodeInfo.last_heard, Position.time).
 */
#ifndef MESHTASTIC_CLOCK_H_
#define MESHTASTIC_CLOCK_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * Seed the wall clock from a known Unix epoch (seconds). Callable from any
 * source (GNSS UTC, phone set_time_only). Values below a sanity floor are
 * ignored (unfixed GNSS / uninitialised time). Last valid seed wins.
 */
void meshtastic_clock_set_epoch(uint32_t epoch_now);

/** True once a valid epoch has been seeded. */
bool meshtastic_clock_valid(void);

/** Current wall-clock time in Unix epoch seconds, or 0 if not yet seeded. */
uint32_t meshtastic_clock_now_epoch(void);

/** Convert a k_uptime-relative second count to epoch seconds, 0 if unseeded. */
uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec);

#endif /* MESHTASTIC_CLOCK_H_ */
