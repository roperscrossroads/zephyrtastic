/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper — see meshtastic_clock.h.
 */

#include "meshtastic_clock.h"

#include <zephyr/kernel.h>

/*
 * epoch MILLISECONDS at k_uptime == 0. Lock-free by design: writers (GNSS
 * callback, SNTP, admin set_time) are rare and readers only use the value for
 * display timestamps, so a torn read at worst mis-stamps one value once before
 * the next seed. boot_epoch_ms is only read once clock_valid is set.
 *
 * Held in ms rather than seconds so a source carrying a sub-second part (SNTP)
 * is not quantised on the way in. The old seconds anchor also truncated the
 * *uptime* side (k_uptime_seconds()), so the stored offset carried an error of
 * frac(epoch) - frac(uptime) in (-1 s, +1 s) — frozen at sync time, different on
 * every node, and re-randomised at each resync.
 */
/* Reapply window (T-A) for a same-quality NTP-class source: mirrors the
 * reference's 30-minute NTP "slam" so SNTP / phone time can still re-discipline
 * the clock for drift without a higher-quality source present, while a flood of
 * low-trust writes in between is refused. */
#define MESHTASTIC_CLOCK_NTP_REAPPLY_MS (30 * 60 * 1000)

static int64_t boot_epoch_ms;
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

void meshtastic_clock_set_epoch_ms(int64_t epoch_ms, enum meshtastic_clock_quality quality)
{
	int64_t epoch_sec;
	int64_t now_ms;

	if (epoch_ms < 0) {
		return;
	}

	/* Range-check on the derived second so the window stays exactly the one the
	 * seconds entry point has always enforced. */
	epoch_sec = epoch_ms / 1000;
	if (epoch_sec < (int64_t)MESHTASTIC_EPOCH_MIN || epoch_sec > (int64_t)MESHTASTIC_EPOCH_MAX) {
		return;
	}

	/* Order matters: clock_should_set() reads last_set_uptime_ms for the NTP
	 * re-apply window, so it must run before that value is overwritten. */
	if (!clock_should_set(quality)) {
		return;
	}

	/* One k_uptime_get() for both the drift window and the anchor, so the two
	 * cannot disagree about when this write happened. */
	now_ms = k_uptime_get();

	current_quality = quality;
	last_set_uptime_ms = now_ms;
	boot_epoch_ms = epoch_ms - now_ms;
	clock_valid = true;
}

void meshtastic_clock_set_epoch(uint32_t epoch_now, enum meshtastic_clock_quality quality)
{
	meshtastic_clock_set_epoch_ms((int64_t)epoch_now * 1000, quality);
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

	return (uint32_t)((boot_epoch_ms + k_uptime_get()) / 1000);
}

int64_t meshtastic_clock_now_epoch_ms(void)
{
	if (!clock_valid) {
		return 0;
	}

	return boot_epoch_ms + k_uptime_get();
}

uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec)
{
	if (!clock_valid) {
		return 0U;
	}

	return (uint32_t)((boot_epoch_ms + (int64_t)uptime_sec * 1000) / 1000);
}

#if defined(CONFIG_MESHTASTIC_LOG_WALLCLOCK)

#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>

/*
 * Feed the logging subsystem wall-clock time once the clock is seeded, so syslog
 * and console lines carry real time (2026-..) instead of 1970+uptime. log_output
 * renders "1970 + timestamp/freq", so returning epoch milliseconds with freq=1000
 * yields the real date to the millisecond. boot_epoch_ms is the epoch ms at
 * k_uptime==0, so boot_epoch_ms + k_uptime_get() is a monotonic ms wall clock —
 * note the resolution is ms but the accuracy is the seeding source's. Before the
 * clock is valid, fall back to k_uptime (the familiar 1970+uptime) so early-boot
 * lines stay ordered; the source re-checks clock_valid on every call and switches
 * to wall time the instant SNTP/GPS/phone seeds the clock, with no re-registration.
 */
static log_timestamp_t meshtastic_log_timestamp(void)
{
	int64_t up_ms = k_uptime_get();

	if (clock_valid) {
		return (log_timestamp_t)(boot_epoch_ms + up_ms);
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
