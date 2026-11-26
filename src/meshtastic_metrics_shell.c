/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic metrics`: device metrics and local stats. Built with the shell and
 * CONFIG_MESHTASTIC_DEVICE_METRICS (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include <zephyr/meshtastic/telemetry.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"
#include "meshtastic_telemetry_internal.h"

static int shell_metrics_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_device_metrics(dest, K_FOREVER);
}

static int cmd_metrics_send(const struct shell *sh, size_t argc, char **argv)
{
	return mtsh_deferred_send(sh, argc, argv, shell_metrics_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_metrics_send)

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
/* Print LocalStats as the node would report them, and optionally push one to
 * the phone right now. Reading them here is the point: LocalStats is normally
 * phone-only, so without this the only way to see the numbers on a bench node
 * with no phone attached would be to infer them from `meshtastic status`. */
static int cmd_metrics_localstats(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_LocalStats stats;
	int32_t chan_util, tx_util;
	int ret;

	ret = meshtastic_collect_local_stats(&stats);
	if (ret < 0) {
		shell_error(sh, "local stats unavailable (%d)", ret);
		return ret;
	}

	chan_util = mtsh_scaled_tenths(stats.channel_utilization);
	tx_util = mtsh_scaled_tenths(stats.air_util_tx);

	shell_print(sh, "uptime  : %us", stats.uptime_seconds);
	shell_print(sh, "util    : chan %d.%u%% tx %d.%u%%", mtsh_scaled_whole(chan_util, 10),
		    mtsh_scaled_fraction(chan_util, 10), mtsh_scaled_whole(tx_util, 10),
		    mtsh_scaled_fraction(tx_util, 10));
	shell_print(sh, "packets : tx %u, rx %u (%u bad), dupe %u, tx dropped %u",
		    stats.num_packets_tx, stats.num_packets_rx, stats.num_packets_rx_bad,
		    stats.num_rx_dupe, stats.num_tx_dropped);
	shell_print(sh, "relay   : %u sent, %u cancelled", stats.num_tx_relay,
		    stats.num_tx_relay_canceled);
	shell_print(sh, "nodes   : %u online / %u total", stats.num_online_nodes,
		    stats.num_total_nodes);
	shell_print(sh, "heap    : %u free / %u total", stats.heap_free_bytes,
		    stats.heap_total_bytes);

	if (argc >= 2 && strcmp(argv[1], "push") == 0) {
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE)
		ret = meshtastic_send_local_stats_to_phone();
		shell_print(sh, "pushed to phone transports: %s", ret == 0 ? "ok" : "failed");
		return ret;
#else
		shell_warn(sh, "built without CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE");
		return -ENOTSUP;
#endif
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_metrics_localstats)
#endif /* CONFIG_MESHTASTIC_LOCAL_STATS */

/* The resolved broadcast cadence: what ModuleConfig.telemetry, the role, the
 * default-channel coercion and the online-node scaling add up to right now. */
static int cmd_metrics_cadence(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_telemetry_settings s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_telemetry_settings(&s);
	shell_print(sh, "device      : %s, every %u s", s.device_enabled ? "on" : "off",
		    s.device_interval_sec);
	shell_print(sh, "environment : %s, every %u s", s.environment_enabled ? "on" : "off",
		    s.environment_interval_sec);
	shell_print(sh, "scaled by   : %u online nodes", s.online_nodes);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_metrics_cadence)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_metrics_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send device metrics.", "[dest|broadcast]"),
					 cmd_metrics_send_on_work),
			       SHELL_CMD(cadence, NULL,
					 SHELL_HELP("Show the resolved telemetry broadcast "
						    "cadence (ModuleConfig.telemetry).", NULL),
					 cmd_metrics_cadence_on_work),
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
			       SHELL_CMD(localstats, NULL,
					 SHELL_HELP("Show LocalStats (mesh counters). "
						    "`push` also sends one to the phone.",
						    "[push]"),
					 cmd_metrics_localstats_on_work),
#endif
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), metrics, &meshtastic_metrics_cmds,
		 SHELL_HELP("Device metrics commands.", NULL),
		 NULL, 0, 0);
