/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic worker` on a keyless worker: its hub and status. Built with the
 * shell and CONFIG_MESHTASTIC_WORKER (src/CMakeLists.txt); moved from
 * meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_worker.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic worker` on a keyless worker (ATTACHMENT-DESIGN S5): whose radio
 * this is, and what has passed through. */
static int cmd_worker_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_worker_stats st;
	uint32_t hub = meshtastic_worker_get_hub();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_worker_stats_get(&st);
	shell_print(sh, "role: keyless worker (transmits what its hub hands it)");
	shell_print(sh, "hub: 0x%08x%s", hub, hub == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "preset: %s (%u)", meshtastic_preset_display_name(mt.modem_preset, true),
		    (unsigned int)mt.modem_preset);
	shell_print(sh, "heard %u  forwarded %u  no_hub %u  queue_full %u  send_failed %u",
		    st.heard, st.forwarded, st.no_hub, st.queue_full, st.send_failed);
	shell_print(sh, "status_sent %u  controls %u  refused %u  untrusted %u  rejected %u",
		    st.status_sent, st.controls, st.refused, st.untrusted, st.rejected);
	shell_print(sh, "tx: sent %u  failed %u  deferred %u  queue_full %u  cancelled %u  late %u  late_dropped %u  duty_blocked %u",
		    st.tx_sent, st.tx_failed, st.tx_deferred, st.tx_queue_full, st.tx_cancelled,
		    st.tx_late, st.tx_late_dropped, st.tx_duty_blocked);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_worker_show)

static int cmd_worker_hub(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_worker_set_hub(node);
	shell_print(sh, "hub: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_worker_hub)

static int shell_worker_status_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_worker_status_send();
	if (ret == 0) {
		shell_print(sh, "status sent");
	}
	return ret;
}

static int cmd_worker_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_worker_status_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_worker_status)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_worker_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Hub and counters.", NULL), cmd_worker_show_on_work),
	SHELL_CMD_ARG(hub, NULL,
		      SHELL_HELP("The hub's node number (0 stops forwarding).", "<hex node id>"),
		      cmd_worker_hub_on_work, 2, 0),
	SHELL_CMD(status, NULL, SHELL_HELP("Send a STATUS to the hub now.", NULL),
		  cmd_worker_status_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), worker, &meshtastic_worker_cmds,
		 SHELL_HELP("This keyless worker: show, hub, status.", NULL),
		 cmd_worker_show_on_work, 0, 0);

/* `meshtastic attach ...` on a worker: the old name, for one release (CLASS-NAMING, renamed
 * 2026-11-27). Each alias says the new name, then runs it. */
MTSH_RENAMED(cmd_attach_show, cmd_worker_show, "worker show")
MTSH_RENAMED(cmd_attach_brain, cmd_worker_hub, "worker hub")
MTSH_RENAMED(cmd_attach_status, cmd_worker_status, "worker status")

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_attach_alias_cmds,
	SHELL_CMD(show, NULL, NULL, cmd_attach_show_on_work),
	SHELL_CMD_ARG(brain, NULL, NULL, cmd_attach_brain_on_work, 2, 0),
	SHELL_CMD(status, NULL, NULL, cmd_attach_status_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), attach, &meshtastic_attach_alias_cmds, NULL,
		 cmd_attach_show_on_work, 0, 0);
