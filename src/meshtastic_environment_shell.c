/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic environment`: environment telemetry. Built with the shell and
 * CONFIG_MESHTASTIC_ENVIRONMENT_METRICS (src/CMakeLists.txt); moved from
 * meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/telemetry.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

static int shell_environment_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_environment(dest, K_FOREVER);
}

static int cmd_environment_send(const struct shell *sh, size_t argc, char **argv)
{
	return mtsh_deferred_send(sh, argc, argv, shell_environment_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_environment_send)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_environment_cmds,
	SHELL_CMD(send, NULL, SHELL_HELP("Send environment telemetry.", "[dest|broadcast]"),
		  cmd_environment_send_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), environment, &meshtastic_environment_cmds,
		 SHELL_HELP("Environment telemetry commands.", NULL),
		 NULL, 0, 0);
