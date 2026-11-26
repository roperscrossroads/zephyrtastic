/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic time`: the wall clock. Built with the shell (src/CMakeLists.txt); moved
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

#include "meshtastic_clock_persist.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

static int cmd_time(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		uint32_t epoch = meshtastic_clock_now_epoch();
		enum meshtastic_clock_quality q = meshtastic_clock_get_quality();

		if (q == MESHTASTIC_CLOCK_QUALITY_NONE) {
			shell_print(sh, "clock: UNSET (epoch 0; logs run on uptime) — "
					"`meshtastic time set <unix-epoch-seconds>`");
			return 0;
		}
		shell_print(sh, "clock: epoch %u, source %s", epoch, mtsh_clock_quality_name(q));
#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
		{
			int32_t ppb;
			uint32_t window_s;

			/* Worth a line of its own: this is the only place the board's
			 * measured oscillator error is visible, and it is a health metric
			 * as much as a clock one — an estimate that wanders between reads
			 * is a hardware problem nothing else here would show. */
			if (meshtastic_clock_skew(&ppb, &window_s)) {
				shell_print(sh, "       skew %d ppb (%d.%03d ppm) over %u s",
					    ppb, ppb / 1000, (ppb < 0 ? -ppb : ppb) % 1000,
					    window_s);
			} else {
				shell_print(sh,
					    "       skew: not yet measured (%u s of the %d s "
					    "window)",
					    window_s,
					    CONFIG_MESHTASTIC_CLOCK_SKEW_MIN_WINDOW_S);
			}
		}
#endif
#if defined(CONFIG_MESHTASTIC_CLOCK_PERSIST)
		{
			enum meshtastic_clock_persist_result pr;
			uint32_t down_ms;

			/* Worth surfacing rather than leaving implicit: a restored clock
			 * reads exactly like a synced one, and the difference matters —
			 * it is only as good as the epoch that was saved plus the
			 * counter's estimate of the gap, and it deliberately carries
			 * DEVICE quality so a real source can still replace it. */
			meshtastic_clock_persist_status(&pr, &down_ms);
			if (pr == MESHTASTIC_CLOCK_PERSIST_RESTORED) {
				shell_print(sh, "       %s (%u ms gap)",
					    meshtastic_clock_persist_result_str(pr), down_ms);
			} else if (pr != MESHTASTIC_CLOCK_PERSIST_COLD) {
				shell_print(sh, "       persist: %s",
					    meshtastic_clock_persist_result_str(pr));
			}
		}
#endif
		return 0;
	}
	if (argc != 3U || strcmp(argv[1], "set") != 0) {
		shell_error(sh, "usage: meshtastic time [set <unix-epoch-seconds>]");
		return -EINVAL;
	}
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	/* Managed only, not the lockdown gate: the clock is not sealed, and a
	 * token's expiry epoch needs one on a locked node. */
	if (mtsh_managed_refused(sh)) {
		return -EACCES;
	}

	unsigned long epoch = strtoul(argv[2], NULL, 10);

	/* Validate here with the setter's own bounds: the setter silently ignores
	 * an out-of-window value, and "silently" is wrong for an interactive
	 * command (the clock may still hold an older good value, so its quality
	 * cannot reveal the rejection). */
	if (epoch < MESHTASTIC_EPOCH_MIN || epoch >= MESHTASTIC_EPOCH_MAX) {
		shell_error(sh, "rejected: outside the sane window [%u, %llu)",
			    MESHTASTIC_EPOCH_MIN, (unsigned long long)MESHTASTIC_EPOCH_MAX);
		return -EINVAL;
	}

	meshtastic_clock_set_epoch((uint32_t)epoch, MESHTASTIC_CLOCK_QUALITY_NTP);
	/* The ladder may have kept a better source (e.g. a GPS fix) — report what
	 * the clock actually holds rather than claiming the write won. */
	shell_print(sh, "clock now: epoch %u, source %s", meshtastic_clock_now_epoch(),
		    mtsh_clock_quality_name(meshtastic_clock_get_quality()));
	return 0;
#endif
}
MTSH_DEFINE_ON_WORK(cmd_time)

SHELL_SUBCMD_ADD((meshtastic), time, NULL,
		 SHELL_HELP("Show or set the wall clock.", "[set <unix-epoch-seconds>]"),
		 cmd_time_on_work, 0, 0);
