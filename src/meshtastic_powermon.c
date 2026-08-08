/* SPDX-License-Identifier: GPL-3.0
 *
 * PowerMon — power-state activity monitor. See meshtastic_powermon.h.
 *
 * State is a single atomic word so it can be updated lock-free from any context
 * (radio RX callback on the system workqueue, the TX thread, and — for the
 * light-sleep bit — the PM notifier, which may run in ISR context / under the PM
 * spinlock). Change logging happens only from the thread-context setters; the PM
 * notifier updates the bit atomically WITHOUT logging, which keeps it safe to run
 * from that constrained context regardless of the logging mode.
 */

#include <stdio.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "meshtastic_powermon.h"
#include "meshtastic_watchdog.h"

#include <zephyr/logging/log.h>
/* Own log module so its verbosity can be raised at the bench ("log level dbg
 * meshtastic_powermon") without touching the rest of the stack. Changes log at
 * INF, so they are visible by default whenever PowerMon is enabled.
 */
LOG_MODULE_REGISTER(meshtastic_powermon, CONFIG_MESHTASTIC_LOG_LEVEL);

static atomic_t pm_state = ATOMIC_INIT(0);

/* Count of light-sleep (PM_STATE_STANDBY) entries — a monotonically increasing
 * "sleep is happening" tally. Incremented from the PM notifier (below); read by the
 * shell and the on-device UI's PM page. A count that climbs is the proof the SoC is
 * actually light-sleeping, visible even when the console is dead (the USJ console
 * dies in light sleep, so the OLED is the only channel that survives). */
static atomic_t pm_sleep_count = ATOMIC_INIT(0);

/* Uptime (ms) of the most recent PM_STATE_STANDBY exit, stamped by the PM notifier
 * on wake. Paired with pm_sleep_count so a diagnostic can place an event (e.g. a
 * radio BUSY timeout) relative to the last light-sleep wake without
 * needing wall-clock time. See meshtastic_powermon_ms_since_wake(). */
static uint32_t pm_last_wake_ms;

static const struct {
	uint32_t bit;
	const char *name;
} pm_flags[] = {
	{MESHTASTIC_PM_CPU_LIGHT_SLEEP, "SLEEP"}, {MESHTASTIC_PM_LORA_RX, "RX"},
	{MESHTASTIC_PM_LORA_TX, "TX"},            {MESHTASTIC_PM_LORA_RX_ACTIVE, "RXACT"},
	{MESHTASTIC_PM_WIFI, "WIFI"},             {MESHTASTIC_PM_BT, "BT"},
	{MESHTASTIC_PM_SCREEN, "SCREEN"},         {MESHTASTIC_PM_GPS, "GPS"},
	{MESHTASTIC_PM_VEXT, "VEXT"},
};

/* Render the active flag names into buf, space-separated. */
static void pm_decode(uint32_t s, char *buf, size_t n)
{
	size_t off = 0;

	if (n > 0) {
		buf[0] = '\0';
	}

	for (size_t i = 0; i < ARRAY_SIZE(pm_flags); i++) {
		if ((s & pm_flags[i].bit) == 0U) {
			continue;
		}

		int w = snprintf(buf + off, n - off, "%s%s", (off > 0U) ? " " : "",
				 pm_flags[i].name);
		if (w < 0 || (size_t)w >= n - off) {
			break;
		}
		off += (size_t)w;
	}
}

static void pm_log_change(uint32_t now)
{
	char buf[64];

	pm_decode(now, buf, sizeof(buf));
	LOG_INF("state=0x%03x [%s]", now, buf);
}

void meshtastic_powermon_set(uint32_t bits)
{
	atomic_val_t prev = atomic_or(&pm_state, (atomic_val_t)bits);
	atomic_val_t now = prev | (atomic_val_t)bits;

	/* Watchdog liveness signal: every LoRa TX/RX power-state transition lands
	 * here, driven both by real RF activity and unconditionally by the
	 * periodic AGC-reset timer (see Kconfig.watchdog) -- a real proof the
	 * radio subsystem is still alive, not just that some thread exists. */
	meshtastic_watchdog_checkin();

	if (now != prev) {
		pm_log_change((uint32_t)now);
	}
}

void meshtastic_powermon_clear(uint32_t bits)
{
	atomic_val_t prev = atomic_and(&pm_state, ~(atomic_val_t)bits);
	atomic_val_t now = prev & ~(atomic_val_t)bits;

	if (now != prev) {
		pm_log_change((uint32_t)now);
	}
}

uint32_t meshtastic_powermon_state(void)
{
	return (uint32_t)atomic_get(&pm_state);
}

uint32_t meshtastic_powermon_sleep_count(void)
{
	return (uint32_t)atomic_get(&pm_sleep_count);
}

uint32_t meshtastic_powermon_ms_since_wake(void)
{
	if (atomic_get(&pm_sleep_count) == 0) {
		return UINT32_MAX; /* never light-slept — no wake to measure from */
	}
	return k_uptime_get_32() - pm_last_wake_ms;
}

#if defined(CONFIG_PM)
#include <zephyr/pm/pm.h>

/*
 * Track the SoC light-sleep state. These callbacks can run in ISR context and
 * under the PM spinlock, so they do ONLY the atomic bit update — no logging, no
 * allocation, nothing that could block or re-enter PM.
 */
static void pm_note_entry(enum pm_state state)
{
	if (state == PM_STATE_STANDBY) {
		(void)atomic_or(&pm_state, (atomic_val_t)MESHTASTIC_PM_CPU_LIGHT_SLEEP);
		(void)atomic_inc(&pm_sleep_count);
	}
}

static void pm_note_exit(enum pm_state state)
{
	if (state == PM_STATE_STANDBY) {
		(void)atomic_and(&pm_state, ~(atomic_val_t)MESHTASTIC_PM_CPU_LIGHT_SLEEP);
		/* Stamp the wake. k_uptime_get_32() is a lock-minimal tick read — ISR-safe
		 * and does not re-enter PM — so it honours the no-block/no-realloc rule
		 * this callback runs under (unlike logging or allocation). */
		pm_last_wake_ms = k_uptime_get_32();
	}
}

static struct pm_notifier pm_note = {
	.state_entry = pm_note_entry,
	.state_exit = pm_note_exit,
};

static int powermon_pm_init(void)
{
	pm_notifier_register(&pm_note);
	return 0;
}

SYS_INIT(powermon_pm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /* CONFIG_PM */

#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>

static int cmd_powermon(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t s = meshtastic_powermon_state();
	char buf[64];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	pm_decode(s, buf, sizeof(buf));
	shell_print(sh, "powermon state=0x%03x [%s] sleeps=%u", s, buf,
		    (unsigned int)atomic_get(&pm_sleep_count));
	return 0;
}

SHELL_CMD_REGISTER(powermon, NULL, "Show PowerMon activity state", cmd_powermon);
#endif /* CONFIG_SHELL */
