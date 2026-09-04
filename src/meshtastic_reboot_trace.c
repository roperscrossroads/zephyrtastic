/* SPDX-License-Identifier: GPL-3.0
 *
 * Make every reboot name itself. See <zephyr/meshtastic/reboot_trace.h> for why.
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <zephyr/meshtastic/reboot_trace.h>

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* The same three-way choice bootlog and logring make, for the same reason: the ESP32 has an
 * RTC region that outlives ordinary RAM across a reset, everything else has .noinit, and
 * native_sim has no reset to survive. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#define REBOOT_TRACE_ATTR RTC_NOINIT_ATTR
#elif defined(CONFIG_ARCH_POSIX)
#define REBOOT_TRACE_ATTR
#else
#define REBOOT_TRACE_ATTR __noinit
#endif

#define REBOOT_TRACE_MAGIC 0x5242544DU /* "RBTM" */

/* Written last, read first: the magic is what tells the next boot the rest of this struct is
 * real and not whatever the RAM happened to hold. Same discipline as the watchdog crash info. */
static REBOOT_TRACE_ATTR uint32_t rt_magic;
static REBOOT_TRACE_ATTR struct meshtastic_reboot_trace rt_record;

/* A note left by one of our own call sites, waiting for the reboot to pick it up. Kept apart
 * from rt_record so that a note left but never followed by a reboot cannot masquerade as one. */
static REBOOT_TRACE_ATTR uint32_t rt_note_magic;
static REBOOT_TRACE_ATTR uint8_t rt_note_reason;
static REBOOT_TRACE_ATTR char rt_note_detail[24];

#define REBOOT_NOTE_MAGIC 0x4E4F5445U /* "NOTE" */

static const char *const reason_names[] = {
	[MESHTASTIC_REBOOT_UNKNOWN] = "unknown",
	[MESHTASTIC_REBOOT_WATCHDOG] = "watchdog",
	[MESHTASTIC_REBOOT_FATAL] = "fatal",
	[MESHTASTIC_REBOOT_ADMIN] = "admin",
	[MESHTASTIC_REBOOT_SHUTDOWN] = "shutdown",
	[MESHTASTIC_REBOOT_DFU] = "dfu-guard",
	[MESHTASTIC_REBOOT_SHELL] = "shell",
	[MESHTASTIC_REBOOT_CONFIG] = "config-apply",
	[MESHTASTIC_REBOOT_OTA] = "ota",
};

const char *meshtastic_reboot_reason_str(uint8_t reason)
{
	if (reason < ARRAY_SIZE(reason_names) && reason_names[reason] != NULL) {
		return reason_names[reason];
	}
	return "invalid";
}

const char *meshtastic_reboot_type_str(uint8_t sys_type)
{
	return (sys_type == (uint8_t)SYS_REBOOT_COLD) ? "cold" : "warm";
}

void meshtastic_reboot_trace_note(enum meshtastic_reboot_reason reason, const char *detail)
{
	rt_note_reason = (uint8_t)reason;
	rt_note_detail[0] = '\0';
	if (detail != NULL) {
		strncpy(rt_note_detail, detail, sizeof(rt_note_detail) - 1);
		rt_note_detail[sizeof(rt_note_detail) - 1] = '\0';
	}
	rt_note_magic = REBOOT_NOTE_MAGIC; /* last */
}

bool meshtastic_reboot_trace_get(struct meshtastic_reboot_trace *out)
{
	if (rt_magic != REBOOT_TRACE_MAGIC || out == NULL) {
		return false;
	}
	*out = rt_record;
	return true;
}

void meshtastic_reboot_trace_clear(void)
{
	rt_magic = 0U;
	memset(&rt_record, 0, sizeof(rt_record));
}

/*
 * The link-time wrapper. -Wl,--wrap=sys_reboot sends EVERY caller here first -- ours, the
 * Zephyr shell's `kernel reboot`, MCUmgr's, a library's -- which is the entire point: the
 * reboot nobody could account for was, by definition, not one of the call sites we knew to
 * instrument.
 *
 * Runs in whatever context asked to reboot, including an ISR or a fatal handler with a
 * wrecked stack, so it does the least it can: no locks, no allocation, no logging, no
 * formatting. Writes to retained RAM and hands straight over to the real implementation.
 */
FUNC_NORETURN void __real_sys_reboot(int type);

/*
 * The recording half, split out from the reboot itself so it can be tested. A test cannot
 * call something that never returns, and "we record the right thing" is exactly the part
 * worth pinning -- the reboot is Zephyr's job, not ours.
 */
void meshtastic_reboot_trace_capture(int type, uint32_t caller_pc, bool have_pc)
{
	const char *tname;

	memset(&rt_record, 0, sizeof(rt_record));
	rt_record.sys_type = (uint8_t)type;
	rt_record.uptime_s = (uint32_t)(k_uptime_get() / 1000);
	rt_record.caller_pc = caller_pc;
	rt_record.have_pc = have_pc;

	/* k_thread_name_get() returns "" rather than NULL when names are on but unset. Not
	 * safe to call from an ISR on every arch, so gate on context. */
	if (!k_is_in_isr()) {
		tname = k_thread_name_get(k_current_get());
		if (tname != NULL) {
			strncpy(rt_record.thread, tname, sizeof(rt_record.thread) - 1);
		}
	} else {
		strncpy(rt_record.thread, "<isr>", sizeof(rt_record.thread) - 1);
	}

	if (rt_note_magic == REBOOT_NOTE_MAGIC) {
		rt_record.reason = rt_note_reason;
		strncpy(rt_record.detail, rt_note_detail, sizeof(rt_record.detail) - 1);
		rt_note_magic = 0U; /* consume: a note belongs to exactly one reboot */
	} else {
		rt_record.reason = MESHTASTIC_REBOOT_UNKNOWN;
	}

	rt_magic = REBOOT_TRACE_MAGIC; /* written last, deliberately */
}

FUNC_NORETURN void __wrap_sys_reboot(int type)
{
	/* __builtin_return_address(0) is our caller: whoever invoked sys_reboot(). It can be
	 * unavailable on a build without frame pointers, hence the flag rather than a zero
	 * that would read as a real address. */
	void *pc = __builtin_return_address(0);

	meshtastic_reboot_trace_capture(type, (uint32_t)(uintptr_t)pc, pc != NULL);

	__real_sys_reboot(type);
	CODE_UNREACHABLE;
}
