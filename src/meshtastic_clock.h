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
 * @brief Trust level of a wall-clock time source; higher wins (T-A).
 *
 * Mirrors the reference RTCQuality ladder. A lower-quality source must never
 * overwrite the clock a higher-quality one already set — e.g. a phone
 * set_time_only / SNTP (NTP) cannot clobber a live GPS fix — so a spoofed or
 * low-trust time can't silently rewind every epoch-stamped field.
 */
enum meshtastic_clock_quality {
	MESHTASTIC_CLOCK_QUALITY_NONE = 0,   /**< No time set yet. */
	MESHTASTIC_CLOCK_QUALITY_DEVICE = 1, /**< Onboard peripheral / battery-backed RTC. */
	MESHTASTIC_CLOCK_QUALITY_NET = 2,    /**< Time relayed from another mesh node. */
	MESHTASTIC_CLOCK_QUALITY_NTP = 3,    /**< NTP/SNTP, or a phone set_time_only. */
	MESHTASTIC_CLOCK_QUALITY_GPS = 4,    /**< Our own GPS UTC. */
};

/**
 * Seed the wall clock from a known Unix epoch (seconds) tagged with the trust
 * level of its source. Callable from any source (GNSS UTC, SNTP, phone
 * set_time_only). Values outside the sane [2020, ~2060] window are ignored, and
 * a write is accepted only when @p quality clears the source-quality ladder:
 * strictly higher than the current source, GPS (which always re-applies), or an
 * NTP-class source past a 30-minute drift window. Otherwise the current time is
 * kept.
 */
void meshtastic_clock_set_epoch(uint32_t epoch_now, enum meshtastic_clock_quality quality);

/** Trust level of the source that last set the clock (NONE if never set). */
enum meshtastic_clock_quality meshtastic_clock_get_quality(void);

/** True once a valid epoch has been seeded. */
bool meshtastic_clock_valid(void);

/** Current wall-clock time in Unix epoch seconds, or 0 if not yet seeded. */
uint32_t meshtastic_clock_now_epoch(void);

/** Convert a k_uptime-relative second count to epoch seconds, 0 if unseeded. */
uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec);

#endif /* MESHTASTIC_CLOCK_H_ */
