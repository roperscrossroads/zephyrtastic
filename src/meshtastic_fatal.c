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
#elif defined(CONFIG_ARCH_POSIX)
/* native_sim / unit tests: an ordinary static. Nothing survives a reset
 * there, but there is no reset to survive either -- keeps this file
 * portable without conditioning it out of the build entirely. */
#define FATAL_CRASH_RTC_ATTR
#else
/* Everywhere else -- nRF above all: __noinit. This used to be lumped in
 * with the native_sim case above, sharing its empty (plain .bss) attribute
 * by omission rather than by decision -- the same defect fixed for the log
 * ring in 5795327 and for the boot log in the file this pattern is copied
 * from: on a warm reset, .bss is zeroed by the C runtime before this file
 * ever runs, so a breadcrumb written right before the reboot that caused it
 * would already be gone by the time meshtastic_fatal_take_last_crash() is
 * called on the next boot. __noinit demonstrably survives on this target
 * (verified in the built image for the log ring); this was just not using
 * it. */
#define FATAL_CRASH_RTC_ATTR __noinit
#endif

#if defined(CONFIG_ARCH_POSIX)
static void fatal_collect_heap_stats(struct meshtastic_fatal_crash_info *info)
{
	ARG_UNUSED(info);
}
#else
/* Same pool meshtastic_watchdog.c's periodic logging reads -- see that
 * file's comment for why this is the heap that actually matters here, and
 * for confirmation the symbol is real on nRF too (kernel/mempool.c defines
 * it unconditionally, not per-SoC; meshtastic_watchdog.c already reads it
 * from this same code path on any board with a watchdog0 alias, XIAO
 * included). Only native_sim, above, has nothing meaningful to report. */
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

bool meshtastic_fatal_peek_last_crash(struct meshtastic_fatal_crash_info *out)
{
	if (rtc_fatal_magic != FATAL_CRASH_MAGIC) {
		return false;
	}

	*out = rtc_fatal_info;

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
