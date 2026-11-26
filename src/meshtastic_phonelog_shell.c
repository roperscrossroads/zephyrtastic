/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic phonelog`: log forwarding to the phone. Built with the shell and
 * CONFIG_MESHTASTIC_PHONELOG (src/CMakeLists.txt); moved from meshtastic_shell.c
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

#include "meshtastic_core.h"
#include "meshtastic_phonelog.h"
#include "meshtastic_shell_internal.h"

static const char *const phonelog_level_names[] = { "off", "err", "wrn", "inf", "dbg" };

/* `meshtastic phonelog` -- state + counters; `meshtastic phonelog <level>` sets
 * the ceiling; `meshtastic phonelog reset` zeroes the counters.
 *
 * The counters are the point. A phone that shows nothing could mean the level is
 * too low, the queue is full because the app is not draining it, or the rate cap
 * is eating a flood -- three very different problems that look identical from
 * the phone end. */
static int cmd_phonelog(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_phonelog_stats stats;
	uint8_t level;

	if (argc >= 2) {
		if (strcmp(argv[1], "reset") == 0) {
			meshtastic_phonelog_reset_stats();
			shell_print(sh, "phonelog counters cleared");
			return 0;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(phonelog_level_names); i++) {
			if (strcmp(argv[1], phonelog_level_names[i]) == 0) {
				int ret = meshtastic_phonelog_set_level(i);

				if (ret < 0) {
					shell_error(sh, "set level failed (%d)", ret);
					return ret;
				}
				shell_print(sh, "phonelog level: %s", phonelog_level_names[i]);
				return 0;
			}
		}

		shell_error(sh, "usage: phonelog [off|err|wrn|inf|dbg|reset]");
		return -EINVAL;
	}

	level = meshtastic_phonelog_get_level();
	meshtastic_phonelog_get_stats(&stats);

	shell_print(sh, "level     : %s%s",
		    level < ARRAY_SIZE(phonelog_level_names) ? phonelog_level_names[level] : "?",
		    stats.panicked ? " (STOOD DOWN: panic)" : "");
	shell_print(sh, "forwarded : %u", stats.forwarded);
	shell_print(sh, "dropped   : %u rate, %u queue-full, %u below level, %u log-core",
		    stats.dropped_rate, stats.dropped_queue, stats.dropped_level,
		    stats.dropped_core);
	shell_print(sh, "rate cap  : %u/s", (unsigned int)CONFIG_MESHTASTIC_PHONELOG_RATE);

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_phonelog)

SHELL_SUBCMD_ADD((meshtastic), phonelog, NULL,
		 SHELL_HELP("Log forwarding to the phone: state + counters, or set the "
		 "severity ceiling.",
		 "[off|err|wrn|inf|dbg|reset]"),
		 cmd_phonelog_on_work, 0, 0);
