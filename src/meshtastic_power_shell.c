/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic power` (the light-sleep power-saving policy) and `meshtastic pm` (the
 * volatile bench hold). Built with the shell and CONFIG_PM (src/CMakeLists.txt); moved
 * from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_powermon.h"
#include "meshtastic_shell_internal.h"

/*
 * power [on|off] — inspect or set the light-sleep power-saving policy.
 *
 * No argument prints the governor's live state: the stored PowerConfig fields
 * (is_power_saving / min_wake_secs / wait_bluetooth_secs), the live inhibitor mask
 * (STANDBY is blocked while any bit is set — an empty mask means the node is free to
 * sleep), and the cumulative light-sleep entry count. That last pair is the answer
 * to "why isn't it sleeping": a WiFi node shows [WIFI] while it holds a lease, and a
 * saving=off node shows [POLICY].
 *
 * `on` / `off` writes is_power_saving into the config store and applies it live via
 * the same meshtastic_power_config_apply() the admin PhoneAPI PowerConfig write
 * calls — no reboot. This is the only way to toggle saving on a BLE-only node with
 * no phone paired, and it persists across reboots.
 */
static int cmd_power_show(const struct shell *sh)
{
	meshtastic_Config config;
	bool saving = IS_ENABLED(CONFIG_MESHTASTIC_POWER_SAVE_DEFAULT);
	uint32_t min_wake = 0U;
	uint32_t wait_bt = 0U;
	bool stored = false;
	uint32_t inh;
	bool locked;
	char inhbuf[64];

	if (meshtastic_config_store_get_config(meshtastic_Config_power_tag, &config) == 0 &&
	    config.which_payload_variant == meshtastic_Config_power_tag) {
		saving = config.payload_variant.power.is_power_saving;
		min_wake = config.payload_variant.power.min_wake_secs;
		wait_bt = config.payload_variant.power.wait_bluetooth_secs;
		stored = true;
	}

	inh = meshtastic_power_inhibitors();
	meshtastic_power_inhibitors_str(inh, inhbuf, sizeof(inhbuf));
	/* Ground truth: is PM_STATE_STANDBY actually locked by ANYONE? This catches the
	 * esp-idf radio-controller locks (WiFi APB / BLE controller) that route through
	 * the HAL's pm_policy_state_all_lock_get() and are INVISIBLE to the governor
	 * mask above -- the reason a BLE node reads inhibitors 0x00 yet never sleeps. */
	locked = pm_policy_state_lock_is_active(PM_STATE_STANDBY, PM_ALL_SUBSTATES);

	shell_print(sh, "power saving:        %s%s", saving ? "on" : "off",
		    stored ? "" : " (compiled default; no PowerConfig stored)");
	shell_print(sh, "min_wake_secs:       %u%s", min_wake,
		    min_wake == 0U ? "  (activity wake window disabled)" : "");
	shell_print(sh, "wait_bluetooth_secs: %u%s", wait_bt,
		    wait_bt == 0U ? "  (boot pairing window disabled)" : "");
	shell_print(sh, "governor inhibitors: 0x%02x [%s]", inh, inhbuf);
	if (!locked) {
		shell_print(sh, "STANDBY lock:        clear -- SoC free to light-sleep");
	} else if (inh != 0U) {
		shell_print(sh, "STANDBY lock:        held (governor inhibiting; see mask above)");
	} else {
		shell_print(sh, "STANDBY lock:        held by esp-idf radio controller "
				"(WiFi/BLE), NOT the governor");
	}
	shell_print(sh, "light-sleep entries: %u%s", meshtastic_powermon_sleep_count(),
		    meshtastic_powermon_sleep_count() == 0U
			    ? "  (0 == SoC has never light-slept; expected while a radio "
			      "controller holds STANDBY)"
			    : "");
	if (meshtastic_power_manual_inhibit()) {
		shell_print(sh, "manual pm hold:      ENGAGED -- pinned awake by `meshtastic pm "
				"off` (volatile; cleared on reboot)");
	}
	return 0;
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_power_set(const struct shell *sh, bool on)
{
	meshtastic_Config config;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_power_tag, &config);
	if (ret < 0) {
		shell_error(sh, "power get failed: %d", ret);
		return ret;
	}

	/* get_config returns the slot with which_payload_variant already set to the
	 * power tag; touch only the field we own and write the whole PowerConfig back,
	 * exactly as the admin set_config path does. */
	config.which_payload_variant = meshtastic_Config_power_tag;
	config.payload_variant.power.is_power_saving = on;

	ret = meshtastic_config_store_set_config(&config);
	if (ret < 0) {
		shell_error(sh, "power set failed: %d", ret);
		return ret;
	}

	/* Apply live -- the same call the admin PowerConfig write makes, so the POLICY
	 * inhibitor flips now, no reboot. */
	meshtastic_power_config_apply();

	shell_print(sh, "power saving %s (applied live, persisted)", on ? "on" : "off");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		return cmd_power_show(sh);
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic power [on|off]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	if (strcmp(argv[1], "on") == 0) {
		return cmd_power_set(sh, true);
	}
	if (strcmp(argv[1], "off") == 0) {
		return cmd_power_set(sh, false);
	}

	shell_error(sh, "expected on or off, got %s", argv[1]);
	return -EINVAL;
#endif
}
MTSH_DEFINE_ON_WORK(cmd_power)

/*
 * pm [on|off] — a volatile debug hold that pins the node awake, distinct from
 * `power`.
 *
 *   pm off  disables light sleep now (holds the governor's MANUAL inhibitor).
 *   pm on   re-enables it (releases the hold; sleep is governed by config again).
 *   pm      reports whether the hold is engaged.
 *
 * Why this is separate from `power on|off`:
 *   - `power off` writes is_power_saving=false into the PERSISTED PowerConfig (a
 *     config write: it survives reboot, is refused on a managed node, and a phone
 *     can flip it back). It expresses the node's power-saving *preference*.
 *   - `pm off` sets a VOLATILE inhibitor only. It persists nothing, is cleared on
 *     reboot, touches no config, and is never refused — because ruling light sleep
 *     in or out for a debugging session is an observability action, not a
 *     configuration one. Use it to pin a bench node awake without disturbing its
 *     stored config or fighting an attached phone.
 *
 * Note this holds only the GOVERNOR's lock; on a radio-up node the esp-idf WiFi/BLE
 * controller already pins STANDBY independently (see `power`), so there the hold is
 * belt-and-suspenders. It bites where the governor is the deciding vote (radio down,
 * e.g. under `netpause`, or a non-radio build).
 */
static int cmd_pm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		shell_print(sh, "manual pm hold: %s",
			    meshtastic_power_manual_inhibit()
				    ? "engaged (light sleep disabled; pinned awake)"
				    : "released (light sleep governed by power config)");
		shell_print(sh, "(`meshtastic power` shows the full governor + STANDBY-lock "
				"state)");
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic pm [on|off]");
		return -EINVAL;
	}

	if (strcmp(argv[1], "off") == 0) {
		meshtastic_power_set_manual_inhibit(true);
		shell_print(sh, "light sleep DISABLED (node pinned awake; volatile, cleared on "
				"reboot)");
		return 0;
	}
	if (strcmp(argv[1], "on") == 0) {
		meshtastic_power_set_manual_inhibit(false);
		shell_print(sh, "light sleep RE-ENABLED (governed by power config)");
		return 0;
	}

	shell_error(sh, "expected on or off, got %s", argv[1]);
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_pm)

SHELL_SUBCMD_ADD((meshtastic), power, NULL,
		 SHELL_HELP("Show or set the light-sleep power-saving policy.", "[on|off]"),
		 cmd_power_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), pm, NULL,
		 SHELL_HELP("Volatile bench hold: pin the node awake (off) or release (on). "
		 "Not persisted; distinct from `power`.",
		 "[on|off]"),
		 cmd_pm_on_work, 0, 0);


