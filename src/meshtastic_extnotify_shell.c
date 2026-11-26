/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic notify`: the external notification (board LED) module. Built with the shell
 * and CONFIG_MESHTASTIC_EXTNOTIFY (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_extnotify.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic notify` (agents-dnr4.17): the LED notifier. */
static int cmd_notify_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_extnotify_settings s;
	struct meshtastic_extnotify_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_extnotify_settings(&s);
	meshtastic_extnotify_stats(&st);
	shell_print(sh, "notify: %s, led %s, on message %s, on bell %s, %u ms pulses, nag %u s, "
			"active-%s",
		    s.enabled ? "enabled" : "disabled", s.output_present ? "present" : "ABSENT",
		    s.alert_message ? "yes" : "no", s.alert_bell ? "yes" : "no", s.output_ms,
		    s.nag_timeout, s.active ? "high" : "low");
	shell_print(sh, "  now: %s, output %s; alerts %u (bells %u), muted %u",
		    meshtastic_extnotify_nagging() ? "nagging" : "idle",
		    meshtastic_extnotify_output_on() ? "on" : "off", st.alerts, st.bells, st.muted);
	shell_print(sh, "  buzzer / vibra / ringtone: not on this port (such a config is refused)");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_notify_show)

static int cmd_notify_test(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_extnotify_trigger();
	if (ret == -EPERM) {
		shell_error(sh, "notification is disabled (ModuleConfig.external_notification)");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "no notification output on this board (%d)", ret);
		return ret;
	}
	shell_print(sh, "nag cycle started");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_notify_test)

static int cmd_notify_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_extnotify_stop();
	shell_print(sh, "stopped");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_notify_stop)

static int cmd_notify_enable(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	meshtastic_ModuleConfig_ExternalNotificationConfig *cfg =
		&mod.payload_variant.external_notification;

	ARG_UNUSED(argc);
	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_external_notification_tag,
						 &mod);
	cfg->enabled = strcmp(argv[0], "enable") == 0;
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}
		ret = meshtastic_extnotify_set(cfg);
		if (ret < 0) {
			shell_error(sh, "notify set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "persisted; applied live, no reboot");
		return cmd_notify_show(sh, 1U, NULL);
	}
#endif
}
MTSH_DEFINE_ON_WORK(cmd_notify_enable)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_notify_cmds,
			       SHELL_CMD(enable, NULL, SHELL_HELP("Enable the LED notifier.", NULL),
					 cmd_notify_enable_on_work),
			       SHELL_CMD(disable, NULL, SHELL_HELP("Disable it.", NULL),
					 cmd_notify_enable_on_work),
			       SHELL_CMD(test, NULL, SHELL_HELP("Start a nag cycle now.", NULL),
					 cmd_notify_test_on_work),
			       SHELL_CMD(stop, NULL, SHELL_HELP("End the cycle, LED off.", NULL),
					 cmd_notify_stop_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), notify, &meshtastic_notify_cmds,
		 SHELL_HELP("External notification (board LED): show, enable/disable, test, "
		 "stop.",
		 NULL),
		 cmd_notify_show_on_work, 0, 0);
