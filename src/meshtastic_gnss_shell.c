/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic gnss`: the position source. Built with the shell and
 * CONFIG_MESHTASTIC_GNSS (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include <zephyr/meshtastic/gnss.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/meshtastic/gnss.h>
#include <zephyr/meshtastic/gnss_pps.h>

#include "meshtastic_core.h"
#include "meshtastic_position.h"
#include "meshtastic_shell_internal.h"

static int shell_position_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_position(dest);
}

static int cmd_gnss_send(const struct shell *sh, size_t argc, char **argv)
{
	return mtsh_deferred_send(sh, argc, argv, shell_position_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_gnss_send)

static const char *gnss_fix_status_name(uint8_t s)
{
	switch (s) {
	case GNSS_FIX_STATUS_NO_FIX:
		return "no fix";
	case GNSS_FIX_STATUS_GNSS_FIX:
		return "GNSS fix";
	case GNSS_FIX_STATUS_DGNSS_FIX:
		return "DGNSS fix";
	case GNSS_FIX_STATUS_ESTIMATED_FIX:
		return "estimated";
	default:
		return "?";
	}
}

/* Seconds, or "never" — an age of -1 is "it has not happened", which a plain
 * number would misreport as "just now". */
static void shell_age(char *buf, size_t len, int64_t age_ms)
{
	if (age_ms < 0) {
		snprintf(buf, len, "never");
	} else {
		snprintf(buf, len, "%lld.%01llds ago", (long long)(age_ms / 1000),
			 (long long)((age_ms % 1000) / 100));
	}
}

/* `meshtastic gnss status` — read-only: is the receiver alive, is it tracking,
 * has it fixed, did the fix reach the clock and the air. Added 2026-09-02 after
 * a bench move to a window left four GNSS-fitted nodes with no way to say
 * whether "no fix yet" meant a cold start in progress or a dead receiver. */
static int cmd_gnss_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_gnss_status st;
	struct meshtastic_gnss_pps_stats pps;
	char a1[24], a2[24], a3[24];
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_gnss_status_get(&st);
	if (ret == -ENODEV) {
		shell_print(sh, "gnss: no receiver in this build's devicetree");
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "gnss status failed: %d", ret);
		return ret;
	}

	shell_print(sh, "receiver: %s (%s)", st.dev_name ? st.dev_name : "?",
		    st.ready ? "ready" : "NOT READY");
	shell_age(a1, sizeof(a1), st.last_data_age_ms);
	shell_print(sh, "data:     %u callbacks, last %s%s", st.callbacks, a1,
		    (st.callbacks == 0U) ? "  <- no NMEA at all: UART/power/reset line" : "");
	shell_print(sh, "tracking: %u satellites, %s, hdop %u.%02u", st.sats,
		    gnss_fix_status_name(st.fix_status), st.hdop_centi / 100U,
		    st.hdop_centi % 100U);
	shell_age(a2, sizeof(a2), st.last_fix_age_ms);
	shell_print(sh, "fix:      %s, %u fixes, last %s", st.has_fix ? "held" : "none yet",
		    st.fixes, a2);
	shell_age(a3, sizeof(a3), st.last_send_age_ms);
	shell_print(sh, "sent:     %u positions, last %s", st.sends, a3);

	meshtastic_gnss_pps_get_stats(&pps);
	if (pps.armed) {
		int64_t edge_age = (pps.count == 0U) ? -1 : k_uptime_get() - pps.last_uptime_ms;

		shell_age(a1, sizeof(a1), edge_age);
		shell_print(sh, "1PPS:     armed, %s, %u edges, last %s", pps.locked ? "LOCKED 1 Hz"
										     : "not locked",
			    pps.count, a1);
	} else {
		shell_print(sh, "1PPS:     not armed");
	}
	shell_print(sh, "clock:    %s", mtsh_clock_quality_name(meshtastic_clock_get_quality()));

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_gnss_status)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_gnss_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send GNSS position.", "[dest|broadcast]"),
					 cmd_gnss_send_on_work),
			       SHELL_CMD(status, NULL,
					 SHELL_HELP("Receiver, tracking, fix, 1PPS and clock — read-only.",
						    NULL),
					 cmd_gnss_status_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), gnss, &meshtastic_gnss_cmds,
		 SHELL_HELP("GNSS commands.", NULL),
		 NULL, 0, 0);
