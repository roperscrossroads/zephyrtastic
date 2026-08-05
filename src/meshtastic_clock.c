/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper — see meshtastic_clock.h.
 */

#include "meshtastic_clock.h"

#include <zephyr/kernel.h>

/* Sanity floor (2020-01-01 UTC): reject epochs below this as bogus (unfixed
 * GNSS, uninitialised phone clock). */
#define MESHTASTIC_EPOCH_MIN 1577836800U

/* Sanity ceiling (T-D): reject epochs more than ~40 years past the floor (≈2060)
 * as bogus — a GPS week-number rollover or garbage clock can present a wildly
 * far-future time that would otherwise be accepted and corrupt every
 * epoch-stamped field. Mirrors the reference's BUILD_EPOCH + FORTY_YEARS upper
 * bound; the port has no build epoch, so it anchors on the 2020 floor. Stays
 * within uint32 range (~2.84e9 < UINT32_MAX). */
#define MESHTASTIC_EPOCH_MAX (MESHTASTIC_EPOCH_MIN + 40ULL * 365ULL * 24ULL * 60ULL * 60ULL)

/*
 * epoch at k_uptime == 0. Lock-free by design: writers (GNSS callback, admin
 * set_time) are rare and readers only use the value for display timestamps, so
 * a torn read at worst mis-stamps one value once before the next seed. boot_epoch
 * is only read once clock_valid is set.
 */
/* Reapply window (T-A) for a same-quality NTP-class source: mirrors the
 * reference's 30-minute NTP "slam" so SNTP / phone time can still re-discipline
 * the clock for drift without a higher-quality source present, while a flood of
 * low-trust writes in between is refused. */
#define MESHTASTIC_CLOCK_NTP_REAPPLY_MS (30 * 60 * 1000)

static int64_t boot_epoch;
static bool clock_valid;
static enum meshtastic_clock_quality current_quality = MESHTASTIC_CLOCK_QUALITY_NONE;
static int64_t last_set_uptime_ms;

/* Gate a clock write by source quality (T-A), mirroring the reference
 * perhapsSetRTC ladder: accept a strictly higher quality, always re-apply GPS
 * (top of the ladder), re-apply an NTP-class source only past the drift window,
 * and otherwise refuse — a lower-trust source must not rewind or advance a clock
 * a better source already set. */
static bool clock_should_set(enum meshtastic_clock_quality quality)
{
	if (quality > current_quality) {
		return true;
	}
	if (quality == MESHTASTIC_CLOCK_QUALITY_GPS) {
		return true;
	}
	if (quality == MESHTASTIC_CLOCK_QUALITY_NTP &&
	    (k_uptime_get() - last_set_uptime_ms) >= MESHTASTIC_CLOCK_NTP_REAPPLY_MS) {
		return true;
	}
	return false;
}

void meshtastic_clock_set_epoch(uint32_t epoch_now, enum meshtastic_clock_quality quality)
{
	if (epoch_now < MESHTASTIC_EPOCH_MIN || epoch_now > MESHTASTIC_EPOCH_MAX) {
		return;
	}

	if (!clock_should_set(quality)) {
		return;
	}

	current_quality = quality;
	last_set_uptime_ms = k_uptime_get();
	boot_epoch = (int64_t)epoch_now - (int64_t)k_uptime_seconds();
	clock_valid = true;
}

enum meshtastic_clock_quality meshtastic_clock_get_quality(void)
{
	return current_quality;
}

bool meshtastic_clock_valid(void)
{
	return clock_valid;
}

uint32_t meshtastic_clock_now_epoch(void)
{
	if (!clock_valid) {
		return 0U;
	}

	return (uint32_t)(boot_epoch + k_uptime_seconds());
}

uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec)
{
	if (!clock_valid) {
		return 0U;
	}

	return (uint32_t)(boot_epoch + (int64_t)uptime_sec);
}

#if defined(CONFIG_MESHTASTIC_LOG_WALLCLOCK)

#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>

/*
 * Feed the logging subsystem wall-clock time once the clock is seeded, so syslog
 * and console lines carry real time (2026-..) instead of 1970+uptime. log_output
 * renders "1970 + timestamp/freq", so returning epoch milliseconds with freq=1000
 * yields the real date to the millisecond. boot_epoch is the epoch at k_uptime==0,
 * so boot_epoch*1000 + k_uptime_get() is a monotonic ms wall clock. Before the
 * clock is valid, fall back to k_uptime (the familiar 1970+uptime) so early-boot
 * lines stay ordered; the source re-checks clock_valid on every call and switches
 * to wall time the instant SNTP/GPS/phone seeds the clock, with no re-registration.
 */
static log_timestamp_t meshtastic_log_timestamp(void)
{
	int64_t up_ms = k_uptime_get();

	if (clock_valid) {
		return (log_timestamp_t)(boot_epoch * 1000 + up_ms);
	}

	return (log_timestamp_t)up_ms;
}

static int meshtastic_log_wallclock_init(void)
{
	(void)log_set_timestamp_func(meshtastic_log_timestamp, 1000U);
	return 0;
}

/* APPLICATION level: the log subsystem is up by now; the clock may not be seeded
 * yet, which the source handles per-call (see above). */
SYS_INIT(meshtastic_log_wallclock_init, APPLICATION, 0);

#endif /* CONFIG_MESHTASTIC_LOG_WALLCLOCK */
