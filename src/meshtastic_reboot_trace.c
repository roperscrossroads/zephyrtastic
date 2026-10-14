/* SPDX-License-Identifier: GPL-3.0
 *
 * Make every reboot name itself. See <zephyr/meshtastic/reboot_trace.h> for why.
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <zephyr/meshtastic/reboot_trace.h>

#include "meshtastic_retained.h"

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

#define REBOOT_TRACE_MAGIC   0x5242544DU /* "RBTM" */
#define REBOOT_NOTE_MAGIC    0x4E4F5445U /* "NOTE" */
#define REBOOT_TRACE_VERSION 1U

/* Each record is one struct, sealed (meshtastic_retained.h): the body is written first and the
 * seal last, so the next boot believes a record only if this layout wrote it whole. The magic
 * alone used to be the test, and after an image change it passed over the other image's bytes
 * (agents-8h29). */
static REBOOT_TRACE_ATTR struct {
	struct meshtastic_retained_seal seal;
	struct meshtastic_reboot_trace record;
} rt;

/* A note left by one of our own call sites, waiting for the reboot to pick it up. Kept apart
 * from the record so that a note left but never followed by a reboot cannot masquerade as one. */
static REBOOT_TRACE_ATTR struct {
	struct meshtastic_retained_seal seal;
	struct {
		uint8_t reason;
		char detail[24];
	} body;
} rt_note;

static const char *const reason_names[] = {
	[MESHTASTIC_REBOOT_UNKNOWN] = "unknown",
	[MESHTASTIC_REBOOT_WATCHDOG] = "watchdog",
	[MESHTASTIC_REBOOT_FATAL] = "fatal",
	[MESHTASTIC_REBOOT_ADMIN] = "admin",
	[MESHTASTIC_REBOOT_SHUTDOWN] = "shutdown",
	[MESHTASTIC_REBOOT_DFU] = "dfu",
	[MESHTASTIC_REBOOT_SHELL] = "shell",
	[MESHTASTIC_REBOOT_CONFIG] = "config-apply",
	[MESHTASTIC_REBOOT_OTA] = "ota",
	[MESHTASTIC_REBOOT_LOCKDOWN] = "lockdown",
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
	meshtastic_retained_void(&rt_note.seal);
	memset(&rt_note.body, 0, sizeof(rt_note.body));
	rt_note.body.reason = (uint8_t)reason;
	if (detail != NULL) {
		strncpy(rt_note.body.detail, detail, sizeof(rt_note.body.detail) - 1);
	}
	meshtastic_retained_seal(&rt_note.seal, REBOOT_NOTE_MAGIC, REBOOT_TRACE_VERSION, &rt_note.body,
				 sizeof(rt_note.body));
}

bool meshtastic_reboot_trace_get(struct meshtastic_reboot_trace *out)
{
	if (out == NULL || !meshtastic_retained_valid(&rt.seal, REBOOT_TRACE_MAGIC,
						      REBOOT_TRACE_VERSION, &rt.record,
						      sizeof(rt.record))) {
		return false;
	}
	*out = rt.record;
	return true;
}

void meshtastic_reboot_trace_clear(void)
{
	meshtastic_retained_void(&rt.seal);
	memset(&rt.record, 0, sizeof(rt.record));
}

#if defined(CONFIG_ZTEST)
void meshtastic_reboot_trace_test_scribble(void)
{
	/* What another image leaves: the magic where this one looks, other bytes around it. */
	rt.record.uptime_s ^= 0x5A5A5A5AU;
	rt_note.body.reason ^= 0x5AU;
}
#endif

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

	meshtastic_retained_void(&rt.seal);
	memset(&rt.record, 0, sizeof(rt.record));
	rt.record.sys_type = (uint8_t)type;
	rt.record.uptime_s = (uint32_t)(k_uptime_get() / 1000);
	rt.record.caller_pc = caller_pc;
	rt.record.have_pc = have_pc;

	/* k_thread_name_get() returns "" rather than NULL when names are on but unset. Not
	 * safe to call from an ISR on every arch, so gate on context. */
	if (!k_is_in_isr()) {
		tname = k_thread_name_get(k_current_get());
		if (tname != NULL) {
			strncpy(rt.record.thread, tname, sizeof(rt.record.thread) - 1);
		}
	} else {
		strncpy(rt.record.thread, "<isr>", sizeof(rt.record.thread) - 1);
	}

	if (meshtastic_retained_valid(&rt_note.seal, REBOOT_NOTE_MAGIC, REBOOT_TRACE_VERSION,
				      &rt_note.body, sizeof(rt_note.body))) {
		rt.record.reason = rt_note.body.reason;
		strncpy(rt.record.detail, rt_note.body.detail, sizeof(rt.record.detail) - 1);
		meshtastic_retained_void(&rt_note.seal); /* consume: a note belongs to one reboot */
	} else {
		rt.record.reason = MESHTASTIC_REBOOT_UNKNOWN;
	}

	meshtastic_retained_seal(&rt.seal, REBOOT_TRACE_MAGIC, REBOOT_TRACE_VERSION, &rt.record,
				 sizeof(rt.record)); /* last, deliberately */
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
