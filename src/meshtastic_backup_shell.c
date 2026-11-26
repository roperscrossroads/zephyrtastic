/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic backup`: the preferences backup in flash. Built with the shell and
 * CONFIG_MESHTASTIC_SETTINGS (src/CMakeLists.txt); moved from meshtastic_shell.c
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

#include "meshtastic_backup.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* The admin backup/restore, from the console (agents-dnr4.14). A restore
 * leaves the restored config applied and a save queued; reboot to be sure
 * every consumer re-reads it, as the admin path does on its own. */

static int cmd_backup(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_backup_meta meta;
	int ret;

	if (argc < 2) {
		if (meshtastic_backup_exists(&meta)) {
			shell_print(sh, "backup present (version %u, taken at epoch %u)",
				    meta.version, meta.timestamp);
		} else {
			shell_print(sh, "no backup");
		}
		return 0;
	}
	if (strcmp(argv[1], "save") == 0) {
		ret = meshtastic_backup_save();
	} else if (strcmp(argv[1], "restore") == 0) {
		ret = meshtastic_backup_restore();
		if (ret == 0) {
			shell_warn(sh, "restored and applied; reboot to finish");
		}
	} else if (strcmp(argv[1], "remove") == 0) {
		ret = meshtastic_backup_remove();
	} else {
		shell_error(sh, "usage: backup [save|restore|remove]");
		return -EINVAL;
	}
	if (ret < 0) {
		shell_error(sh, "%s failed (%d)", argv[1], ret);
	} else {
		shell_print(sh, "%s: ok", argv[1]);
	}
	return ret;
}
MTSH_DEFINE_ON_WORK(cmd_backup)

SHELL_SUBCMD_ADD((meshtastic), backup, NULL,
		 SHELL_HELP("Preferences backup in flash: show, save, restore (then reboot), "
		 "remove.", "[save|restore|remove]"),
		 cmd_backup_on_work, 0, 0);
