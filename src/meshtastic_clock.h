/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper. The port's timestamps are otherwise k_uptime-relative;
 * this establishes a Unix epoch once a real time source is available (GNSS UTC,
 * SNTP, or the phone's set_time_only admin message) so uptime-relative values can
 * be reported to the app as epoch seconds (e.g. NodeInfo.last_heard,
 * Position.time).
 *
 * The anchor is held in MILLISECONDS. The seconds-taking entry point remains the
 * common case (GNSS and the phone only ever offer whole seconds), but a source
 * that knows better — SNTP carries a sub-second fraction — should use the _ms
 * form: anchoring on whole seconds quantises the clock by up to 1 s, and because
 * the residual is frozen at sync time it differs per node and per resync. Two
 * nodes both "NTP-synced" could then sit ~2 s apart, which is fine for a
 * last_heard display and not fine for anything that has to agree on *when*.
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
/* Sanity floor (2020-01-01 UTC): epochs below are rejected as bogus (unfixed
 * GNSS, uninitialised phone clock). In the header so a caller offering an
 * operator-supplied value (the `meshtastic time set` shell command) can
 * validate with exactly the bounds the setter enforces. */
#define MESHTASTIC_EPOCH_MIN 1577836800U

/* Sanity ceiling (T-D): ~40 years past the floor (≈2060) — a GPS week-number
 * rollover or garbage clock can present a wildly far-future time that would
 * otherwise corrupt every epoch-stamped field. Mirrors the reference's
 * BUILD_EPOCH + FORTY_YEARS bound; the port has no build epoch, so it anchors
 * on the 2020 floor. Stays within uint32 range (~2.84e9 < UINT32_MAX). */
#define MESHTASTIC_EPOCH_MAX (MESHTASTIC_EPOCH_MIN + 40ULL * 365ULL * 24ULL * 60ULL * 60ULL)

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

/**
 * Millisecond-resolution form of @ref meshtastic_clock_set_epoch, and the one the
 * seconds form is implemented on top of. Same range check (applied to the derived
 * second) and same source-quality ladder.
 *
 * Prefer this wherever the source actually knows the sub-second part; passing
 * `sec * 1000` here is exactly equivalent to the seconds form and costs nothing.
 *
 * @param epoch_ms Unix epoch in milliseconds. Negative values are ignored.
 * @param quality  Trust level of the source (see @ref meshtastic_clock_quality).
 */
void meshtastic_clock_set_epoch_ms(int64_t epoch_ms, enum meshtastic_clock_quality quality);

/** Trust level of the source that last set the clock (NONE if never set). */
enum meshtastic_clock_quality meshtastic_clock_get_quality(void);

/** True once a valid epoch has been seeded. */
bool meshtastic_clock_valid(void);

/** Current wall-clock time in Unix epoch seconds, or 0 if not yet seeded. */
uint32_t meshtastic_clock_now_epoch(void);

/**
 * Current wall-clock time in Unix epoch MILLISECONDS, or 0 if not yet seeded.
 *
 * Monotonic between seeds (it is the ms anchor plus k_uptime_get()), so it is
 * also the right source for anything that needs to schedule against absolute
 * time rather than merely display it. Note the *resolution* is milliseconds but
 * the *accuracy* is only ever as good as the source that last seeded it.
 */
int64_t meshtastic_clock_now_epoch_ms(void);

/** Convert a k_uptime-relative second count to epoch seconds, 0 if unseeded. */
uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec);

#endif /* MESHTASTIC_CLOCK_H_ */
