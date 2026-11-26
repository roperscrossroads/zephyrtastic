/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic nodeinfo`: NodeInfo send, request and cadence. Built with the shell and
 * CONFIG_MESHTASTIC_NODEINFO (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include <zephyr/meshtastic/nodeinfo.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

static int shell_nodeinfo_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_node_info(dest);
}

static int cmd_nodeinfo_send(const struct shell *sh, size_t argc, char **argv)
{
	return mtsh_deferred_send(sh, argc, argv, shell_nodeinfo_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_send)

/* Unlike `send`, this actually asks: want_response=true, so a compliant peer
 * replies with ITS NodeInfo. It goes through the same work queue `send` does: the
 * request runs the whole send path, which the shell thread's stack cannot carry on
 * a XIAO (CONFIG_SHELL_STACK_SIZE=2048; run inline it reset the board every time).
 * The per-peer cooldown and the K_NO_WAIT send live in meshtastic_nodeinfo_request(). */
/* Bench 2026-10-09: run inline, `nodeinfo request` rebooted kit1 every time
 * (agents-yhdm). */
static int shell_nodeinfo_request_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);
	int ret;

	ret = meshtastic_nodeinfo_request(dest);
	if (ret == -EAGAIN) {
		shell_print(sh, "suppressed: already requested 0x%08x within "
			    "the last %d s (CONFIG_MESHTASTIC_NODEINFO_UNKNOWN_SUPPRESS_SEC)",
			    dest, CONFIG_MESHTASTIC_NODEINFO_UNKNOWN_SUPPRESS_SEC);
		ret = 0;
	} else if (ret == 0) {
		shell_print(sh, "requested -- watch for a NodeInfo reply from "
			    "0x%08x", dest);
	}
	return ret;
}

static int cmd_nodeinfo_request(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dest;
	int ret;

	ARG_UNUSED(argc);

	ret = mtsh_parse_u32(sh, argv[1], &dest);
	if (ret < 0) {
		return ret;
	}

	return mtsh_enqueue(sh, shell_nodeinfo_request_run, &dest, sizeof(dest));
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_request)

/* Get/set device.node_info_broadcast_secs (agents-t2hb.1). Same shape as
 * `meshtastic device role`/`rebroadcast`: read always available, write gated
 * behind CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE. */
static int cmd_nodeinfo_interval(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		meshtastic_Config cfg;
		bool stored = meshtastic_config_store_get_config(meshtastic_Config_device_tag,
								 &cfg) == 0 &&
			      cfg.which_payload_variant == meshtastic_Config_device_tag &&
			      cfg.payload_variant.device.node_info_broadcast_secs != 0U;

		shell_print(sh, "interval: %u s%s", meshtastic_nodeinfo_interval_secs(),
			    stored ? "" : " (compiled default; node_info_broadcast_secs unset)");
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic nodeinfo interval [secs]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		unsigned long secs;
		char *end;
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}

		secs = strtoul(argv[1], &end, 10);
		if (*end != '\0' || secs > UINT32_MAX) {
			shell_error(sh, "invalid seconds value: %s", argv[1]);
			return -EINVAL;
		}

		ret = meshtastic_config_store_set_node_info_interval((uint32_t)secs);
		if (ret < 0) {
			shell_error(sh, "interval set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "interval -> %u s%s (persisted; next broadcast uses it, no reboot)",
			    meshtastic_nodeinfo_interval_secs(),
			    secs == 0U ? " (0 == compiled default)" : "");
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_interval)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_nodeinfo_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send node information.", "[dest|broadcast]"),
					 cmd_nodeinfo_send_on_work),
			       SHELL_CMD_ARG(request, NULL,
					     SHELL_HELP("Ask a peer for ITS NodeInfo "
							"(want_response=true).",
							"<dest>"),
					     cmd_nodeinfo_request_on_work, 2, 0),
			       SHELL_CMD(interval, NULL,
					 SHELL_HELP("Get/set the auto-broadcast interval "
						    "(0 = compiled default).",
						    "[secs]"),
					 cmd_nodeinfo_interval_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), nodeinfo, &meshtastic_nodeinfo_cmds,
		 SHELL_HELP("NodeInfo commands.", NULL),
		 NULL, 0, 0);
