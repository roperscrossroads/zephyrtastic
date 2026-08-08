/* SPDX-License-Identifier: GPL-3.0 */

/**
 * @file
 * @brief Meshtastic diagnostics public API.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_DIAGNOSTICS_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_DIAGNOSTICS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot captured from ISR context the moment the hardware watchdog
 * decided to force a reset.
 *
 * @c thread_name is whatever thread happened to be running when the watchdog
 * ISR fired -- informative, but not necessarily the thread that caused the
 * hang, since the watchdog's check-in is a global liveness signal, not
 * per-thread.
 */
struct meshtastic_watchdog_crash_info {
	/** Milliseconds since the last watchdog check-in when the reset fired. */
	uint32_t since_checkin_ms;
	/** System heap free bytes at the moment of the reset. */
	uint32_t heap_free;
	/** System heap allocated bytes at the moment of the reset. */
	uint32_t heap_allocated;
	/** System heap high-water mark (bytes) at the moment of the reset. */
	uint32_t heap_max_allocated;
	/** Name of the thread the watchdog ISR interrupted, best-effort. */
	char thread_name[20];
};

/**
 * @brief Take (read and clear) the watchdog crash breadcrumb, if any.
 *
 * If the previous boot ended in a watchdog-forced reset, this populates
 * @p out with a snapshot captured at that moment and returns true. The
 * breadcrumb is stored in RTC-persistent memory -- the same mechanism used
 * for reset-cause history -- specifically because it survives the reset
 * without depending on a driver or the log subsystem, both of which are
 * unusable from the ISR context the snapshot is taken in.
 *
 * One-shot: a successful read clears the breadcrumb, so it is reported
 * exactly once and does not reappear on a later, unrelated boot. Returns
 * false (leaving @p out untouched) if there is none, including on any board
 * without a hardware watchdog.
 *
 * @param out Populated on success. Must not be NULL.
 * @retval true  A crash breadcrumb was present and @p out was populated.
 * @retval false No breadcrumb was present.
 */
bool meshtastic_watchdog_take_last_crash(struct meshtastic_watchdog_crash_info *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_DIAGNOSTICS_H_ */
