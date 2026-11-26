/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic duty`: the regulatory duty cycle. Built with the shell and
 * CONFIG_MESHTASTIC_DUTY_CYCLE (src/CMakeLists.txt); moved from meshtastic_shell.c
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

#include "meshtastic_airtime.h"
#include "meshtastic_core.h"
#include "meshtastic_duty.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic duty` — the ceiling, how much of it is spent, and what has been
 * refused. Worth surfacing: once the gate engages EVERY send is refused, which
 * from the outside looks exactly like a dead radio. This is how you tell the
 * difference in one command. */
static int cmd_duty(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_duty_stats stats;
	float ceiling = meshtastic_duty_effective_pct();
	float used = meshtastic_airtime_tx_util_percent();
	int32_t c10 = mtsh_scaled_tenths(ceiling), u10 = mtsh_scaled_tenths(used);
	uint8_t silent = 0U;
	bool blocked;

	if (argc >= 2 && strcmp(argv[1], "reset") == 0) {
		meshtastic_duty_stats_reset();
		shell_print(sh, "duty counters cleared");
		return 0;
	}

	blocked = meshtastic_duty_blocked(&silent);
	meshtastic_duty_stats_get(&stats);

	if (ceiling >= 100.0f) {
		shell_print(sh, "ceiling : none (this region sets no duty cycle)");
	} else {
		shell_print(sh, "ceiling : %d.%u%%", mtsh_scaled_whole(c10, 10),
			    mtsh_scaled_fraction(c10, 10));
	}
	shell_print(sh, "used    : %d.%u%% of the last hour", mtsh_scaled_whole(u10, 10),
		    mtsh_scaled_fraction(u10, 10));
	shell_print(sh, "state   : %s", blocked ? "BLOCKING egress" : "sending allowed");
	if (blocked) {
		shell_print(sh, "clear in: %u min", silent);
	}
	shell_print(sh, "refused : %u (%u of them relays)", stats.blocked, stats.blocked_relay);

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_duty)

SHELL_SUBCMD_ADD((meshtastic), duty, NULL,
		 SHELL_HELP("Regulatory duty cycle: ceiling, hour used, refusals.",
		 "[reset]"),
		 cmd_duty_on_work, 0, 0);
