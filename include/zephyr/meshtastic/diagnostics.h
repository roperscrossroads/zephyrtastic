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
 * @brief Snapshot captured the moment a task_wdt channel's timeout fires and
 * forces a reset.
 *
 * @c thread_name is whatever thread happened to be running when the timeout
 * callback fired -- informative, but not necessarily the thread that caused
 * the hang, since a channel's feed can come from a different thread than
 * whichever one happens to be executing at the exact moment it expires.
 * @c channel identifies *which* channel timed out ("radio", "sysworkq", ...)
 * -- unlike a single-channel watchdog, this tells you which subsystem
 * specifically stopped checking in, not just that something did.
 */
struct meshtastic_watchdog_crash_info {
	/** Which task_wdt channel timed out, e.g. "radio" or "sysworkq". */
	char channel[12];
	/** System heap free bytes at the moment of the reset. */
	uint32_t heap_free;
	/** System heap allocated bytes at the moment of the reset. */
	uint32_t heap_allocated;
	/** System heap high-water mark (bytes) at the moment of the reset. */
	uint32_t heap_max_allocated;
	/** Name of the thread running when the timeout callback fired, best-effort. */
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

/**
 * @brief Snapshot captured from the hardware watchdog's own interrupt stage —
 * the last-resort path, independent of the kernel scheduler.
 *
 * task_wdt's software channel timeouts (@ref meshtastic_watchdog_crash_info)
 * depend on a k_timer expiry, which needs the scheduler/tick still running.
 * If a hang is severe enough to also stall the scheduler, that softer path
 * never fires — only the hardware watchdog peripheral's own timer, running
 * independently of any software, can still force a reset. This breadcrumb is
 * captured from that peripheral's own interrupt stage (a genuine hardware
 * ISR, one full timeout before the actual reset on this build), the only
 * diagnostic opportunity that class of hang leaves. Deliberately minimal —
 * unlike @ref meshtastic_watchdog_crash_info, no heap stats: a true hardware
 * ISR can interrupt code mid-heap-operation, where reading heap stats risks
 * deadlocking on a lock the interrupted code already holds.
 *
 * @note Stage 0 is an interrupt, not a reset — so unlike the other two
 * breadcrumbs, writing one does not imply the board went down. If the next
 * feed lands before stage 1 expires, the board recovers and keeps running
 * with the breadcrumb written. That case is deliberately NOT reported through
 * this struct (see @ref meshtastic_hw_watchdog_take_last_crash); it is logged
 * live as a warning by the sysworkq heartbeat and counted as the
 * @c hw_stage0_recovered statistic instead.
 */
struct meshtastic_hw_watchdog_crash_info {
	/** Kernel uptime (ms) when the hardware interrupt stage fired. */
	uint32_t uptime_ms;
	/** Name of the thread running when the interrupt fired, best-effort. */
	char thread_name[20];
};

/**
 * @brief Take (read and clear) the hardware-watchdog crash breadcrumb, if any.
 *
 * Same RTC-persistent, one-shot design as @ref meshtastic_watchdog_take_last_crash().
 * A breadcrumb here without a corresponding @ref meshtastic_watchdog_crash_info
 * means the scheduler itself was unresponsive — the software channel path never
 * got a chance to run at all.
 *
 * That inference only holds because the RTC slot is snapshotted into RAM at
 * boot, before the stage-0 callback is installed, so what this returns is
 * strictly "an event that survived a reset". Reading the slot directly at
 * report time would not be equivalent: stage 0 does not reset the board, and
 * a fire during WiFi association reliably beat the report site (which runs on
 * the first IPv4 lease) to it — every such breadcrumb observed on the bench
 * before 2026-08-10 was that false positive, not a real scheduler stall.
 *
 * @param out Populated on success. Must not be NULL.
 * @retval true  A crash breadcrumb was present and @p out was populated.
 * @retval false No breadcrumb was present.
 */
bool meshtastic_hw_watchdog_take_last_crash(struct meshtastic_hw_watchdog_crash_info *out);

/**
 * @brief Snapshot captured the moment a genuine software-detected fault
 * (assert, stack-canary failure, CPU exception, ...) reaches
 * k_sys_fatal_error_handler().
 *
 * Distinct from @ref meshtastic_watchdog_crash_info: this fires for a fault
 * Zephyr's own detection caught (there IS a reason code), not a pure
 * liveness timeout. @c thread_name is the thread that was actually running
 * when the fault happened -- for this path, that generally IS the culprit,
 * unlike the watchdog's best-effort snapshot.
 */
struct meshtastic_fatal_crash_info {
	/** The K_ERR_* reason code Zephyr passed to k_sys_fatal_error_handler(). */
	uint32_t reason;
	/** System heap free bytes at the moment of the fault. */
	uint32_t heap_free;
	/** System heap allocated bytes at the moment of the fault. */
	uint32_t heap_allocated;
	/** System heap high-water mark (bytes) at the moment of the fault. */
	uint32_t heap_max_allocated;
	/** Name of the thread that faulted, best-effort. */
	char thread_name[20];
};

/**
 * @brief Take (read and clear) the fatal-error crash breadcrumb, if any.
 *
 * Same RTC-persistent, one-shot design as
 * @ref meshtastic_watchdog_take_last_crash() and for the same reason: by the
 * time k_sys_fatal_error_handler() runs, LOG_PANIC() has already put every
 * log backend into panic mode, and on this project's network-only logging
 * profile that means nothing reaches netlog -- see meshtastic_fatal.c.
 *
 * @param out Populated on success. Must not be NULL.
 * @retval true  A crash breadcrumb was present and @p out was populated.
 * @retval false No breadcrumb was present.
 */
bool meshtastic_fatal_take_last_crash(struct meshtastic_fatal_crash_info *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_DIAGNOSTICS_H_ */
