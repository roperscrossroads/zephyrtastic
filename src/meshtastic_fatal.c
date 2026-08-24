/* SPDX-License-Identifier: GPL-3.0
 *
 * Fatal-error breadcrumb: overrides Zephyr's __weak k_sys_fatal_error_handler()
 * (kernel/fatal.c, called at the end of z_fatal_error(), after coredump()) to
 * capture a snapshot before the reboot the same way meshtastic_watchdog.c
 * does for a watchdog-forced reset, and for the identical reason: by the time
 * this runs, LOG_PANIC() has already put every log backend into panic mode,
 * and on this project's network-only logging profile (see
 * meshtastic_watchdog.c's file comment for the full story, confirmed by a
 * live test 2026-08-08) that means the reason/thread/heap state this
 * function has right now would otherwise reach nobody.
 *
 * This is the general case; meshtastic_watchdog.c's breadcrumb covers the
 * narrower one (a pure liveness timeout with no software fault to report).
 * Both use the same RTC-persistent, one-shot pattern as the reset-cause
 * history in samples/meshtastic/src/main.c, which is where both get read
 * back and logged.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_heap.h>

#include <zephyr/meshtastic/diagnostics.h>

#if defined(CONFIG_MESHTASTIC_DFU_ON_FATAL)
#include "meshtastic_dfu_trigger.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* No public header declares this despite being __weak/overridable -- it's
 * only ever defined, in kernel/fatal.c, which is also the only place that
 * calls it under its own steam. Only reachable here on a build with
 * CONFIG_RESET_ON_FATAL_ERROR off (not this project's default). */
FUNC_NORETURN extern void arch_system_halt(unsigned int reason);

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#define FATAL_CRASH_RTC_ATTR RTC_NOINIT_ATTR

/* Same pool meshtastic_watchdog.c's periodic logging reads -- see that
 * file's comment for why this is the heap that actually matters here. Not a
 * real symbol on non-ESP32 targets (native_sim), hence guarded alongside the
 * RTC attribute rather than separately. */
extern struct k_heap _system_heap;

static void fatal_collect_heap_stats(struct meshtastic_fatal_crash_info *info)
{
	struct sys_memory_stats stats;

	if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
		info->heap_free = (uint32_t)stats.free_bytes;
		info->heap_allocated = (uint32_t)stats.allocated_bytes;
		info->heap_max_allocated = (uint32_t)stats.max_allocated_bytes;
	}
}
#else
/* Non-ESP32 targets (native_sim, unit tests): no RTC-persistent memory to
 * put this in, so it degrades to an ordinary static -- doesn't survive a
 * real reset, but there's nothing to reset in the sense this is meant for
 * on those targets either. Keeps this file portable without conditioning
 * it out of the build entirely. */
#define FATAL_CRASH_RTC_ATTR

static void fatal_collect_heap_stats(struct meshtastic_fatal_crash_info *info)
{
	ARG_UNUSED(info);
}
#endif

#define FATAL_CRASH_MAGIC 0x46415441U /* "FATA" */

static FATAL_CRASH_RTC_ATTR uint32_t rtc_fatal_magic;
static FATAL_CRASH_RTC_ATTR struct meshtastic_fatal_crash_info rtc_fatal_info;

bool meshtastic_fatal_take_last_crash(struct meshtastic_fatal_crash_info *out)
{
	if (rtc_fatal_magic != FATAL_CRASH_MAGIC) {
		return false;
	}

	*out = rtc_fatal_info;
	rtc_fatal_magic = 0U; /* one-shot: don't re-report on a later, unrelated boot */

	return true;
}

/* Overrides the __weak default in kernel/fatal.c. Reproduces its exact
 * behavior (LOG_PANIC() + reset-or-halt per CONFIG_RESET_ON_FATAL_ERROR)
 * so nothing about the existing fatal-error path changes -- this only adds
 * the breadcrumb capture ahead of it. */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	const char *tname;

	ARG_UNUSED(esf);

	/* Plain memory stores -- no driver, no log subsystem -- same reasoning
	 * as watchdog_pre_reset_cb(), and doubly so here: this can run from a
	 * genuine CPU exception context, not just a device-driver ISR. */
	rtc_fatal_info.reason = reason;

	fatal_collect_heap_stats(&rtc_fatal_info);

	tname = k_thread_name_get(k_current_get());
	strncpy(rtc_fatal_info.thread_name, tname != NULL ? tname : "",
		sizeof(rtc_fatal_info.thread_name) - 1);
	rtc_fatal_info.thread_name[sizeof(rtc_fatal_info.thread_name) - 1] = '\0';

	/* Written last, deliberately -- same reasoning as
	 * meshtastic_watchdog.c's rtc_crash_magic. */
	rtc_fatal_magic = FATAL_CRASH_MAGIC;

	LOG_PANIC();

#if defined(CONFIG_MESHTASTIC_DFU_ON_FATAL)
	/* Bring-up aid: make the reset below land in the bootloader, ready to
	 * reflash — a crash before USB init is otherwise invisible AND
	 * unreachable without the reset pad. mark() is a plain register store,
	 * safe in this exception context. */
	meshtastic_dfu_mark(true);
#endif

#if defined(CONFIG_RESET_ON_FATAL_ERROR)
	LOG_ERR("Resetting system");
	sys_reboot(SYS_REBOOT_WARM);
#else
	LOG_ERR("Halting system");
	arch_system_halt(reason);
#endif

	CODE_UNREACHABLE;
}
