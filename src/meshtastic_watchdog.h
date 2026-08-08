/* SPDX-License-Identifier: GPL-3.0
 *
 * Hardware watchdog — a last-resort backstop for the class of hang that
 * leaves no software-fault trace (no assert, no coredump, no fatal error —
 * see Kconfig.watchdog for the observed failure mode this exists to catch).
 */

#ifndef MESHTASTIC_WATCHDOG_H_
#define MESHTASTIC_WATCHDOG_H_

/* struct meshtastic_watchdog_crash_info and meshtastic_watchdog_take_last_crash()
 * are public API — declared once in zephyr/meshtastic/diagnostics.h, defined in
 * meshtastic_watchdog.c (which includes that header directly). Nothing to
 * redeclare here; this header only covers the internal-only init/checkin calls. */

#if defined(CONFIG_MESHTASTIC_WATCHDOG)

/**
 * Arm the hardware watchdog and start the periodic feed worker. Safe to call
 * once at boot; if the board has no wdt0 alias, logs a warning and continues
 * without one rather than failing boot.
 */
void meshtastic_watchdog_init(void);

/**
 * Record "the system is alive" right now. The feed worker only feeds the
 * watchdog if a check-in landed recently enough; otherwise it lets the
 * timeout run out. Cheap enough to call from any thread context that already
 * has a natural periodic pulse — see meshtastic_powermon_set()'s call site.
 */
void meshtastic_watchdog_checkin(void);

#else /* !CONFIG_MESHTASTIC_WATCHDOG */

static inline void meshtastic_watchdog_init(void)
{
}

static inline void meshtastic_watchdog_checkin(void)
{
}

#endif /* CONFIG_MESHTASTIC_WATCHDOG */

#endif /* MESHTASTIC_WATCHDOG_H_ */
