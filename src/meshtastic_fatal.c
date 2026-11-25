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
#include <zephyr/devicetree.h>
#if defined(CONFIG_CPU_CORTEX_M)
#include <cmsis_core.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/meshtastic/reboot_trace.h>
#include <zephyr/sys/sys_heap.h>

#include <zephyr/meshtastic/diagnostics.h>

#include "meshtastic_retained.h"

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

#define FATAL_CRASH_VERSION 2U /* 2: exception frame, fault status, nested count */

/*
 * A coredump into a partition on the nRF QSPI NOR CANNOT work, and makes every
 * fault on such a board panic twice (agents-ninc, 2026-11-24). Zephyr's
 * z_fatal_error() runs coredump() BEFORE this file's handler; the flash
 * partition backend writes through the ordinary nrf_qspi_nor driver, whose
 * every operation takes a semaphore with K_FOREVER (qspi_lock()) and then waits
 * for the QSPI completion interrupt on another one. From exception context the
 * first is the sem.c "k_sem_take with a timeout in an ISR" assert, and the
 * second could never be given. The assert re-enters the fatal path, coredump's
 * re-entry guard lets the second pass through, and this handler records only
 * the second fault: rzr5 (z7rg) and rzr7 (vnfu) on 2026-11-24, both XIAOs on
 * the sysbuild layout, whose coredump_partition is on the QSPI chip. Refuse the
 * combination at build time rather than rediscover it on a crash.
 */
#if defined(CONFIG_DEBUG_COREDUMP_BACKEND_FLASH_PARTITION) &&                                  \
	DT_NODE_EXISTS(DT_NODELABEL(coredump_partition))
BUILD_ASSERT(!DT_NODE_HAS_COMPAT(DT_GPARENT(DT_NODELABEL(coredump_partition)), nordic_qspi_nor),
	     "coredump_partition is on the nRF QSPI NOR, whose driver blocks: a coredump "
	     "from a fault asserts in it and hides the fault (agents-ninc)");
#endif

/* Set on the first entry into the handler this boot (plain .bss: zeroed by every
 * boot). A second entry is a fault inside the fatal path itself; it must not
 * overwrite the record of the fault that started the chain. */
static bool fatal_entered;

/* Sealed (meshtastic_retained.h): a magic alone also passes over another image's bytes
 * after an update (agents-8h29). */
static FATAL_CRASH_RTC_ATTR struct meshtastic_retained_seal rtc_fatal_seal;
static FATAL_CRASH_RTC_ATTR struct meshtastic_fatal_crash_info rtc_fatal_info;

static bool fatal_record_valid(void)
{
	return meshtastic_retained_valid(&rtc_fatal_seal, FATAL_CRASH_MAGIC, FATAL_CRASH_VERSION,
					 &rtc_fatal_info, sizeof(rtc_fatal_info));
}

bool meshtastic_fatal_take_last_crash(struct meshtastic_fatal_crash_info *out)
{
	if (!fatal_record_valid()) {
		return false;
	}

	*out = rtc_fatal_info;
	meshtastic_retained_void(&rtc_fatal_seal); /* one-shot: not re-reported on a later boot */
	fatal_entered = false;                     /* a fault after this is a new chain */

	return true;
}

bool meshtastic_fatal_peek_last_crash(struct meshtastic_fatal_crash_info *out)
{
	if (!fatal_record_valid()) {
		return false;
	}

	*out = rtc_fatal_info;

	return true;
}

/* Overrides the __weak default in kernel/fatal.c. Reproduces its exact
 * behavior (LOG_PANIC() + reset-or-halt per CONFIG_RESET_ON_FATAL_ERROR)
 * so nothing about the existing fatal-error path changes -- this only adds
 * the breadcrumb capture ahead of it. */
/* The exception frame and fault status, where the architecture has them. Plain
 * loads: no driver, nothing that can block. */
static void fatal_collect_frame(struct meshtastic_fatal_crash_info *info,
				const struct arch_esf *esf)
{
#if defined(CONFIG_CPU_CORTEX_M)
	if (esf != NULL) {
		info->pc = esf->basic.pc;
		info->lr = esf->basic.lr;
		info->xpsr = esf->basic.xpsr;
		/* IPSR, the exception number, in the stacked xPSR: non-zero means the
		 * fault was taken in handler mode ("Fault during interrupt handling"). */
		info->in_handler = (esf->basic.xpsr & 0x1FFU) != 0U ? 1U : 0U;
	}
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	info->cfsr = SCB->CFSR;
	info->hfsr = SCB->HFSR;
	info->mmfar = SCB->MMFAR;
	info->bfar = SCB->BFAR;
#endif
#else
	ARG_UNUSED(info);
	ARG_UNUSED(esf);
#endif
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	const char *tname;

	/* A fault inside the fatal path (LOG_PANIC's flush, the reboot): keep the
	 * first fault's record and count this one (agents-ninc). */
	if (fatal_entered) {
		if (fatal_record_valid()) {
			meshtastic_retained_void(&rtc_fatal_seal);
			if (rtc_fatal_info.nested < UINT8_MAX) {
				rtc_fatal_info.nested++;
			}
			meshtastic_retained_seal(&rtc_fatal_seal, FATAL_CRASH_MAGIC,
						 FATAL_CRASH_VERSION, &rtc_fatal_info,
						 sizeof(rtc_fatal_info));
		}
		goto finish;
	}
	fatal_entered = true;

	/* Plain memory stores -- no driver, no log subsystem -- same reasoning
	 * as watchdog_pre_reset_cb(), and doubly so here: this can run from a
	 * genuine CPU exception context, not just a device-driver ISR. Nothing on
	 * this path may block either: a semaphore taken here is the nested panic
	 * of agents-ninc. */
	meshtastic_retained_void(&rtc_fatal_seal); /* invalid while it changes */
	memset(&rtc_fatal_info, 0, sizeof(rtc_fatal_info));
	rtc_fatal_info.reason = reason;
	fatal_collect_frame(&rtc_fatal_info, esf);

	fatal_collect_heap_stats(&rtc_fatal_info);

	tname = k_thread_name_get(k_current_get());
	strncpy(rtc_fatal_info.thread_name, tname != NULL ? tname : "",
		sizeof(rtc_fatal_info.thread_name) - 1);
	rtc_fatal_info.thread_name[sizeof(rtc_fatal_info.thread_name) - 1] = '\0';

	/* Sealed last, deliberately -- same reasoning as meshtastic_watchdog.c's crash seal. */
	meshtastic_retained_seal(&rtc_fatal_seal, FATAL_CRASH_MAGIC, FATAL_CRASH_VERSION,
				 &rtc_fatal_info, sizeof(rtc_fatal_info));

finish:
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
	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_FATAL, NULL);
	sys_reboot(SYS_REBOOT_WARM);
#else
	LOG_ERR("Halting system");
	arch_system_halt(reason);
#endif

	CODE_UNREACHABLE;
}
