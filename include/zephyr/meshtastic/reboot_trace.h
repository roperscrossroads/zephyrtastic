/* SPDX-License-Identifier: GPL-3.0
 *
 * Where did this reboot come from? A record written by the reboot itself.
 */
#ifndef ZEPHYR_MESHTASTIC_REBOOT_TRACE_H_
#define ZEPHYR_MESHTASTIC_REBOOT_TRACE_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * Why this exists.
 *
 * The boot log already answers "was the last reset warm, and what did hwinfo say". It cannot
 * answer "who asked for it", and on 2026-09-04 that gap cost a day: two nodes restarted within
 * a second of each other reporting cause 0x00000002 SOFTWARE -- a deliberate sys_reboot() from
 * firmware -- and NOTHING in the tree could say which one. Ruling the candidates out took
 * reading every sys_reboot() call site, checking that the two paths which record something had
 * recorded nothing, and eliminating a hardware-watchdog line that turned out to belong to a
 * boot hours earlier. The answer was still "unaccounted" (bead agents-u0h6).
 *
 * A reboot that cannot say where it came from is the defect. This makes every reboot name
 * itself.
 *
 * Two halves, deliberately:
 *
 *  - Every caller is caught, including ones we do not own. The record is written from a
 *    link-time wrapper around sys_reboot(), so the Zephyr shell's `kernel reboot`, MCUmgr, a
 *    library, or anything else that reboots this node lands in it too. Instrumenting only our
 *    own call sites would have left exactly the suspects that mattered.
 *  - Our own sites add MEANING. A caller PC identifies a site but says nothing about intent,
 *    so the paths we own call meshtastic_reboot_trace_note() first to say "admin asked",
 *    "watchdog channel starved". Unnoted reboots still get a PC, which addr2line resolves.
 */

/** Who asked. Ours are named; anything else is UNKNOWN with a caller PC to chase. */
enum meshtastic_reboot_reason {
	/** No note was left: something rebooted without going through a known path.
	 *  `caller_pc` is the only lead -- addr2line it against that build's zephyr.elf. */
	MESHTASTIC_REBOOT_UNKNOWN = 0,
	/** Task-watchdog channel timeout (a thread stopped feeding its channel). */
	MESHTASTIC_REBOOT_WATCHDOG,
	/** The kernel fatal handler. */
	MESHTASTIC_REBOOT_FATAL,
	/** An admin/PhoneAPI reboot request. */
	MESHTASTIC_REBOOT_ADMIN,
	/** An admin shutdown request. */
	MESHTASTIC_REBOOT_SHUTDOWN,
	/** The DFU boot guard (nRF). */
	MESHTASTIC_REBOOT_DFU,
	/** A shell command asked for it. */
	MESHTASTIC_REBOOT_SHELL,
	/** Applying a configuration change that needs a restart. */
	MESHTASTIC_REBOOT_CONFIG,
	/** A firmware update swapped an image in. */
	MESHTASTIC_REBOOT_OTA,
};

/** What the reboot recorded about itself. */
struct meshtastic_reboot_trace {
	/** enum meshtastic_reboot_reason */
	uint8_t reason;
	/** The `type` argument sys_reboot() was called with (WARM / COLD). */
	uint8_t sys_type;
	/** Whether a caller PC was captured. */
	bool have_pc;
	/** Return address of whoever called sys_reboot(). addr2line resolves it. */
	uint32_t caller_pc;
	/** Uptime in seconds at the moment of the reboot. */
	uint32_t uptime_s;
	/** Thread that called it -- usually more legible than the PC. */
	char thread[16];
	/** Free text from the noting site, e.g. a watchdog channel name. "" if none. */
	char detail[24];
};

/**
 * @brief Say why the reboot about to happen is happening.
 *
 * Call immediately before sys_reboot(). Safe from any context; it only writes retained RAM.
 * @param reason enum meshtastic_reboot_reason
 * @param detail short free text, or NULL
 */
void meshtastic_reboot_trace_note(enum meshtastic_reboot_reason reason, const char *detail);

/**
 * @brief Read the previous boot's record, if there is one.
 *
 * @param out filled in when true is returned
 * @return true if the previous run recorded a reboot
 */
bool meshtastic_reboot_trace_get(struct meshtastic_reboot_trace *out);

/**
 * @brief Write the record for a reboot that is about to happen.
 *
 * Called by the sys_reboot() wrapper; exposed so tests can exercise the recording without
 * rebooting. Does no logging, takes no locks and allocates nothing -- it may run in an ISR
 * or from a fatal handler with a damaged stack.
 */
void meshtastic_reboot_trace_capture(int type, uint32_t caller_pc, bool have_pc);

/** @brief Forget the record, so the next boot does not re-report this one. */
void meshtastic_reboot_trace_clear(void);

/** @brief Human-readable reason name, never NULL. */
const char *meshtastic_reboot_reason_str(uint8_t reason);

/** @brief "cold" or "warm" for a recorded sys_type. Kept here so callers do not
 *  need <zephyr/sys/reboot.h> just to render a record. */
const char *meshtastic_reboot_type_str(uint8_t sys_type);

#endif /* ZEPHYR_MESHTASTIC_REBOOT_TRACE_H_ */
