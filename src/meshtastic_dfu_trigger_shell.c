/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic dfu`: reboot into the bootloader. Built with the shell and
 * CONFIG_MESHTASTIC_DFU_TRIGGER (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2). It stays on the shell thread (mtsh_shell_thread_cmds[]).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_dfu_trigger.h"
#include "meshtastic_shell_internal.h"

static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	bool serial_only = (argc >= 2) && (strcmp(argv[1], "serial") == 0);

	if (argc >= 2 && !serial_only) {
		shell_error(sh, "usage: dfu [serial]  (default: UF2 mode, drive + serial DFU)");
		return -EINVAL;
	}

	shell_print(sh, "rebooting into the bootloader (%s mode) — this console goes away now",
		    serial_only ? "serial-DFU" : "UF2");
	/* Give the shell a beat to push the line out the CDC pipe. */
	k_sleep(K_MSEC(100));
	meshtastic_dfu_enter(serial_only);
	return 0; /* unreachable */
}

SHELL_SUBCMD_ADD((meshtastic), dfu, NULL,
		 SHELL_HELP("Reboot into the bootloader for reflashing.", "[serial]"),
		 cmd_dfu, 0, 0);
