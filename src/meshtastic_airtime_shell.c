/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic airtime`: channel utilization per preset, and band-wide TX duty. Built with the shell and
 * CONFIG_MESHTASTIC_AIRTIME (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include "meshtastic_shell_internal.h"

/* `meshtastic airtime` — the two questions the accounting answers, kept apart.
 *
 * Channel utilization is PER PRESET, because two presets are two channels that
 * cannot hear each other and CSMA must back off against the one it is actually
 * on. TX utilization is BAND-WIDE, because the duty cycle it feeds is a
 * regulatory limit on the band and does not care which spreading factor the PA
 * was keyed at. Showing them in one place with that distinction spelled out is
 * the point: they look like the same number and are not. */
static int cmd_airtime(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t cur = meshtastic_airtime_current_slot();
	int32_t tx10 = mtsh_scaled_tenths(meshtastic_airtime_tx_util_percent());
	bool any = false;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "tx (band-wide, 1 h): %d.%u%%  <- what the duty cycle measures",
		    mtsh_scaled_whole(tx10, 10), mtsh_scaled_fraction(tx10, 10));
	shell_print(sh, "channel (per preset, 60 s)  <- what CSMA and the beacon gate read");

	for (uint8_t slot = 0; slot < MESHTASTIC_AIRTIME_PRESET_SLOTS; slot++) {
		float pct = meshtastic_airtime_channel_util_percent_slot(slot);
		int32_t p10 = mtsh_scaled_tenths(pct);
		const char *name;

		if (p10 == 0 && slot != cur) {
			continue; /* quiet and not where we are: nothing to say */
		}
		any = true;

		name = (slot == MESHTASTIC_AIRTIME_SLOT_CUSTOM)
			       ? "custom"
			       : meshtastic_preset_display_name(
					 (meshtastic_Config_LoRaConfig_ModemPreset)slot, true);

		shell_print(sh, "  %-12s %d.%u%%%s", name, mtsh_scaled_whole(p10, 10),
			    mtsh_scaled_fraction(p10, 10), (slot == cur) ? "   <- on air now" : "");
	}

	if (!any) {
		shell_print(sh, "  (all quiet)");
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_airtime)

SHELL_SUBCMD_ADD((meshtastic), airtime, NULL,
		 SHELL_HELP("Channel utilization per preset, and band-wide TX duty.", NULL),
		 cmd_airtime_on_work, 0, 0);
