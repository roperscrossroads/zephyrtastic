/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic traffic`: traffic management (position dedup, rate limit, unknown filter). Built with the shell and
 * CONFIG_MESHTASTIC_TRAFFIC (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include "meshtastic_shell_internal.h"
#include "meshtastic_traffic.h"

/* `meshtastic traffic` (agents-dnr4.20). Reads always; writes gated. */
static int cmd_traffic_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_traffic_settings s;
	meshtastic_TrafficManagementStats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_traffic_settings(&s);
	meshtastic_traffic_stats(&st);
	shell_print(sh, "traffic: position dedup %s (%u s), rate limit %s (%u per %u s), "
			"unknown filter %s (>%u per 5 min)",
		    s.position_min_interval_secs ? "on" : "off", s.position_min_interval_secs,
		    (s.rate_limit_window_secs && s.rate_limit_max_packets) ? "on" : "off",
		    s.rate_limit_max_packets, s.rate_limit_window_secs,
		    s.unknown_packet_threshold ? "on" : "off", s.unknown_packet_threshold);
	shell_print(sh, "  inspected %u; dropped: position-dedup %u, rate-limit %u, unknown %u; "
			"%zu node(s) tracked",
		    st.packets_inspected, st.position_dedup_drops, st.rate_limit_drops,
		    st.unknown_packet_drops, meshtastic_traffic_tracked());
	shell_print(sh, "  nodeinfo direct response: not supported on this port (a config "
			"enabling it is refused)");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_traffic_show)

static void traffic_stored(meshtastic_ModuleConfig_TrafficManagementConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_traffic_management_tag,
						 &mod);
	*cfg = mod.payload_variant.traffic_management;
}

/* traffic set <dedup|window|max|unknown> <n> */
static int cmd_traffic_set(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_TrafficManagementConfig cfg;
	unsigned long n;
	char *end;

	if (argc != 3U) {
		shell_error(sh, "usage: meshtastic traffic set <dedup|window|max|unknown> <n>");
		return -EINVAL;
	}
	n = strtoul(argv[2], &end, 10);
	if (*end != '\0' || n > UINT32_MAX) {
		shell_error(sh, "invalid value: %s", argv[2]);
		return -EINVAL;
	}
	traffic_stored(&cfg);
	if (strcmp(argv[1], "dedup") == 0) {
		cfg.position_min_interval_secs = (uint32_t)n;
	} else if (strcmp(argv[1], "window") == 0) {
		cfg.rate_limit_window_secs = (uint32_t)n;
	} else if (strcmp(argv[1], "max") == 0) {
		cfg.rate_limit_max_packets = (uint32_t)n;
	} else if (strcmp(argv[1], "unknown") == 0) {
		cfg.unknown_packet_threshold = (uint32_t)n;
	} else {
		shell_error(sh, "unknown key: %s", argv[1]);
		return -EINVAL;
	}

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
		ret = meshtastic_traffic_set(&cfg);
		if (ret < 0) {
			shell_error(sh, "traffic set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "persisted; applied live, no reboot");
		return cmd_traffic_show(sh, 1U, NULL);
	}
#endif
}
MTSH_DEFINE_ON_WORK(cmd_traffic_set)

static int cmd_traffic_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_traffic_reset();
	shell_print(sh, "traffic state and counters cleared");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_traffic_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_traffic_cmds,
			       SHELL_CMD_ARG(set, NULL,
					     SHELL_HELP("Set one knob (0 disables it).",
							"<dedup|window|max|unknown> <n>"),
					     cmd_traffic_set_on_work, 3, 0),
			       SHELL_CMD(reset, NULL,
					 SHELL_HELP("Forget every node's state; zero counters.",
						    NULL),
					 cmd_traffic_reset_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), traffic, &meshtastic_traffic_cmds,
		 SHELL_HELP("Traffic management: position dedup, rate limit, unknown filter; "
		 "counters.",
		 NULL),
		 cmd_traffic_show_on_work, 0, 0);
