/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper — see meshtastic_clock.h.
 */

#include "meshtastic_clock.h"

#include <zephyr/kernel.h>

/* Sanity floor (2020-01-01 UTC): reject epochs below this as bogus (unfixed
 * GNSS, uninitialised phone clock). */
#define MESHTASTIC_EPOCH_MIN 1577836800U

/*
 * epoch at k_uptime == 0. Lock-free by design: writers (GNSS callback, admin
 * set_time) are rare and readers only use the value for display timestamps, so
 * a torn read at worst mis-stamps one value once before the next seed. boot_epoch
 * is only read once clock_valid is set.
 */
static int64_t boot_epoch;
static bool clock_valid;

void meshtastic_clock_set_epoch(uint32_t epoch_now)
{
	if (epoch_now < MESHTASTIC_EPOCH_MIN) {
		return;
	}

	boot_epoch = (int64_t)epoch_now - (int64_t)k_uptime_seconds();
	clock_valid = true;
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
