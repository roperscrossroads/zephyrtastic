/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic worker-rx`: the cross-preset relay's rx-only worker. Built with the shell and
 * CONFIG_MESHTASTIC_WORKER_RX (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include "meshtastic_relay.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic worker-rx`: the relay's rx-only worker. The peer is saved (MESHTASTIC_SETTINGS). */
static int cmd_worker_rx_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_worker_rx_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_worker_rx_stats_get(&st);
	shell_print(sh, "peer: 0x%08x%s", meshtastic_worker_rx_get_peer(),
		    meshtastic_worker_rx_get_peer() == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "heard %u  forwarded %u  send_failed %u  queue_full %u", st.heard,
		    st.forwarded, st.send_failed, st.queue_full);
	shell_print(sh, "skipped: not_text %u  not_broadcast %u  no_peer %u  rate_dropped %u",
		    st.not_text, st.not_broadcast, st.no_peer, st.rate_dropped);
	shell_print(sh, "cap: %d frames per %d s", CONFIG_MESHTASTIC_WORKER_RX_RATE_MAX,
		    CONFIG_MESHTASTIC_WORKER_RX_RATE_WINDOW_SEC);
	shell_print(sh, "transmit: %s",
		    IS_ENABLED(CONFIG_MESHTASTIC_WORKER_RX_RX_ONLY) ? "compiled out" : "possible");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_worker_rx_show)

static int cmd_worker_rx_peer(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_worker_rx_set_peer(node);
	shell_print(sh, "peer: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_worker_rx_peer)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_worker_rx_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Peer and counters.", NULL), cmd_worker_rx_show_on_work),
	SHELL_CMD_ARG(peer, NULL,
		      SHELL_HELP("The receiving half's node id (0 stops forwarding).",
				 "<hex node id>"),
		      cmd_worker_rx_peer_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

/* `meshtastic worker-rx`. SHELL_SUBCMD_ADD() takes the name as a C token, which cannot hold
 * the '-', so the entry is written out as that macro writes it, with the name as a string. */
static const TYPE_SECTION_ITERABLE(struct shell_static_entry,
				   Z_SHELL_SUBCMD_NAME(meshtastic, worker_rx), shell_subcmds,
				   Z_SHELL_SUBCMD_ADD_SECTION_TAG((meshtastic), worker_rx)) = {
	.syntax = "worker-rx",
	.help = SHELL_HELP("Cross-preset rx-only worker: show, peer.", NULL),
	.subcmd = (const union shell_cmd_entry *)&meshtastic_worker_rx_cmds,
	.handler = cmd_worker_rx_show_on_work,
	.args = {.mandatory = 0, .optional = 0},
};

/* `meshtastic ear ...`: the old name, for one release (CLASS-NAMING, renamed 2026-11-27). Each
 * alias says the new name, then runs it. */
MTSH_RENAMED(cmd_ear_show, cmd_worker_rx_show, "worker-rx show")
MTSH_RENAMED(cmd_ear_peer, cmd_worker_rx_peer, "worker-rx peer")

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_ear_alias_cmds,
	SHELL_CMD(show, NULL, NULL, cmd_ear_show_on_work),
	SHELL_CMD_ARG(peer, NULL, NULL, cmd_ear_peer_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), ear, &meshtastic_ear_alias_cmds, NULL, cmd_ear_show_on_work, 0, 0);
