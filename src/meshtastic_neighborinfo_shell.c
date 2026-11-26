/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic neighbors`: directly-heard neighbors and the NeighborInfo broadcast. Built with the shell and
 * CONFIG_MESHTASTIC_NEIGHBORINFO (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_neighborinfo.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic neighbors` (agents-dnr4.19). Reads always; writes gated behind
 * CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and the is_managed policy. */
static int cmd_neighbors_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_neighborinfo_settings s;
	struct meshtastic_neighborinfo_entry e;
	size_t shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_neighborinfo_settings(&s);
	shell_print(sh, "neighbor info: %s, every %u s, lora %s%s", s.enabled ? "enabled" : "disabled",
		    s.interval_secs, s.transmit_over_lora ? "on" : "off",
		    (s.transmit_over_lora && !s.lora_allowed)
			    ? " (held: default channel on default slot -> phone only)"
			    : "");

	for (size_t i = 0; i < CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE; i++) {
		if (!meshtastic_neighborinfo_at(i, &e)) {
			continue;
		}
		{
			/* Tenths of a dB without %f: the shell builds do not all
			 * carry CBPRINTF_FP_SUPPORT. */
			int snr10 = (int)(e.snr * 10.0f);
			int frac = snr10 % 10;

			shell_print(sh, "  0x%08x  snr %s%d.%d dB  heard %6u s ago  (its interval %u s)",
				    e.node, (snr10 < 0 && snr10 > -10) ? "-" : "", snr10 / 10,
				    frac < 0 ? -frac : frac, e.age_secs, e.interval_secs);
		}
		shown++;
	}
	shell_print(sh, "%zu direct neighbor%s", shown, shown == 1U ? "" : "s");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_show)

static int cmd_neighbors_write(const struct shell *sh, bool enabled, uint32_t interval_secs,
			       bool transmit_over_lora)
{
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	ARG_UNUSED(enabled);
	ARG_UNUSED(interval_secs);
	ARG_UNUSED(transmit_over_lora);
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	ret = meshtastic_neighborinfo_set(enabled, interval_secs, transmit_over_lora);
	if (ret < 0) {
		shell_error(sh, "neighbor info set failed: %d", ret);
		return ret;
	}
	shell_print(sh, "persisted; applied live, no reboot");
	return cmd_neighbors_show(sh, 1U, NULL);
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

/* Read the stored section (not the resolved settings) so an edit of one field
 * does not silently rewrite the others to their resolved values. */
static void neighbors_stored(bool *enabled, uint32_t *interval, bool *lora)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_neighbor_info_tag, &mod);
	*enabled = mod.payload_variant.neighbor_info.enabled;
	*interval = mod.payload_variant.neighbor_info.update_interval;
	*lora = mod.payload_variant.neighbor_info.transmit_over_lora;
}

static int cmd_neighbors_enable(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;

	ARG_UNUSED(argc);
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, strcmp(argv[0], "enable") == 0, interval, lora);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_enable)

static int cmd_neighbors_interval(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;
	unsigned long secs;
	char *end;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic neighbors interval <secs> (0 = default)");
		return -EINVAL;
	}
	secs = strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs > UINT32_MAX) {
		shell_error(sh, "invalid seconds value: %s", argv[1]);
		return -EINVAL;
	}
	if (secs != 0UL && secs < CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC) {
		shell_warn(sh, "%lu s is below the %d s minimum; the default (%d s) will be used",
			   secs, CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC,
			   CONFIG_MESHTASTIC_NEIGHBORINFO_INTERVAL_SEC);
	}
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, enabled, (uint32_t)secs, lora);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_interval)

static int cmd_neighbors_lora(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;

	if (argc != 2U || (strcmp(argv[1], "on") != 0 && strcmp(argv[1], "off") != 0)) {
		shell_error(sh, "usage: meshtastic neighbors lora on|off");
		return -EINVAL;
	}
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, enabled, interval, strcmp(argv[1], "on") == 0);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_lora)

/* Bench 2026-10-10: run inline, `neighbors send` rebooted kit1 (agents-yhdm). */
static int shell_neighbors_send_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_neighborinfo_send();
	if (ret == -ENODATA) {
		shell_error(sh,
			    "no direct neighbors heard yet; nothing to send");
	} else if (ret == 0) {
		shell_print(sh, "neighbor info sent");
	}
	return ret;
}

static int cmd_neighbors_send(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_neighbors_send_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_send)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_neighbors_cmds,
			       SHELL_CMD(enable, NULL, SHELL_HELP("Enable the periodic broadcast.", NULL),
					 cmd_neighbors_enable_on_work),
			       SHELL_CMD(disable, NULL, SHELL_HELP("Disable it.", NULL),
					 cmd_neighbors_enable_on_work),
			       SHELL_CMD_ARG(interval, NULL,
					     SHELL_HELP("Set the broadcast interval (0 = default).",
							"<secs>"),
					     cmd_neighbors_interval_on_work, 2, 0),
			       SHELL_CMD_ARG(lora, NULL,
					     SHELL_HELP("Transmit over LoRa, or phone-only.",
							"on|off"),
					     cmd_neighbors_lora_on_work, 2, 0),
			       SHELL_CMD(send, NULL, SHELL_HELP("Broadcast the table now.", NULL),
					 cmd_neighbors_send_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), neighbors, &meshtastic_neighbors_cmds,
		 SHELL_HELP("Directly-heard neighbors + the NeighborInfo broadcast: show, "
		 "enable/disable, interval, lora, send.",
		 NULL),
		 cmd_neighbors_show_on_work, 0, 0);
