/* SPDX-License-Identifier: GPL-3.0
 *
 * PowerMon — a lightweight power-state activity monitor (concept ported from
 * upstream Meshtastic's PowerMon). It tracks which power-relevant subsystems are
 * active as an atomic bitmask so a bench session can correlate current draw with
 * radio/wifi/sleep state, without a logic analyser on every rail.
 *
 * Zero cost when CONFIG_MESHTASTIC_POWERMON is off: the setters compile to no-ops
 * (below), so call sites in the radio/wifi/PM paths need no #ifdef guards.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_POWERMON_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_POWERMON_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Power-relevant activity bits. Kept in sync with the decode table in
 * meshtastic_powermon.c. Values are stable (logged/scoped), so append only.
 */
enum meshtastic_powermon_bit {
	MESHTASTIC_PM_CPU_LIGHT_SLEEP = (1U << 0), /* SoC in PM_STATE_STANDBY */
	MESHTASTIC_PM_LORA_RX = (1U << 1),         /* continuous RX armed */
	MESHTASTIC_PM_LORA_TX = (1U << 2),         /* transmitting */
	MESHTASTIC_PM_LORA_RX_ACTIVE = (1U << 3),  /* a frame is being handled */
	MESHTASTIC_PM_WIFI = (1U << 4),            /* wifi associated/up */
	MESHTASTIC_PM_BT = (1U << 5),              /* bluetooth active */
	MESHTASTIC_PM_SCREEN = (1U << 6),          /* display powered */
	MESHTASTIC_PM_GPS = (1U << 7),             /* GNSS powered */
	MESHTASTIC_PM_VEXT = (1U << 8),            /* Vext peripheral rail on */
};

#if defined(CONFIG_MESHTASTIC_POWERMON)

/** Set one or more activity bits; logs (INF) if the state changed. */
void meshtastic_powermon_set(uint32_t bits);

/** Clear one or more activity bits; logs (INF) if the state changed. */
void meshtastic_powermon_clear(uint32_t bits);

/** Current activity bitmask. */
uint32_t meshtastic_powermon_state(void);

/** Number of light-sleep (PM_STATE_STANDBY) entries since boot. A rising count is
 *  live proof the SoC is light-sleeping. 0 without CONFIG_PM (nothing to count). */
uint32_t meshtastic_powermon_sleep_count(void);

/** Milliseconds since the most recent light-sleep (PM_STATE_STANDBY) wake, or
 *  UINT32_MAX if the SoC has not light-slept yet. Lets a diagnostic place an event
 *  relative to the last wake without wall-clock time (agents-qnpp). */
uint32_t meshtastic_powermon_ms_since_wake(void);

#else /* !CONFIG_MESHTASTIC_POWERMON — compile the instrumentation away. */

static inline void meshtastic_powermon_set(uint32_t bits)
{
	(void)bits;
}

static inline void meshtastic_powermon_clear(uint32_t bits)
{
	(void)bits;
}

static inline uint32_t meshtastic_powermon_state(void)
{
	return 0U;
}

static inline uint32_t meshtastic_powermon_sleep_count(void)
{
	return 0U;
}

static inline uint32_t meshtastic_powermon_ms_since_wake(void)
{
	return UINT32_MAX;
}

#endif /* CONFIG_MESHTASTIC_POWERMON */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_POWERMON_H_ */
