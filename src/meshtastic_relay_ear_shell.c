/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic ear`: the cross-preset relay's ear. Built with the shell and
 * CONFIG_MESHTASTIC_RELAY_EAR (src/CMakeLists.txt); moved from meshtastic_shell.c
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

/* `meshtastic ear`: the relay's ear. The peer is saved (MESHTASTIC_SETTINGS). */
static int cmd_ear_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_relay_ear_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_relay_ear_stats_get(&st);
	shell_print(sh, "peer: 0x%08x%s", meshtastic_relay_ear_get_peer(),
		    meshtastic_relay_ear_get_peer() == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "heard %u  forwarded %u  send_failed %u  queue_full %u", st.heard,
		    st.forwarded, st.send_failed, st.queue_full);
	shell_print(sh, "skipped: not_text %u  not_broadcast %u  no_peer %u  rate_dropped %u",
		    st.not_text, st.not_broadcast, st.no_peer, st.rate_dropped);
	shell_print(sh, "cap: %d frames per %d s", CONFIG_MESHTASTIC_RELAY_EAR_RATE_MAX,
		    CONFIG_MESHTASTIC_RELAY_EAR_RATE_WINDOW_SEC);
	shell_print(sh, "transmit: %s",
		    IS_ENABLED(CONFIG_MESHTASTIC_RELAY_EAR_RX_ONLY) ? "compiled out" : "possible");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_ear_show)

static int cmd_ear_peer(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_relay_ear_set_peer(node);
	shell_print(sh, "peer: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_ear_peer)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_ear_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Peer and counters.", NULL), cmd_ear_show_on_work),
	SHELL_CMD_ARG(peer, NULL,
		      SHELL_HELP("The receiving half's node id (0 stops forwarding).",
				 "<hex node id>"),
		      cmd_ear_peer_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), ear, &meshtastic_ear_cmds,
		 SHELL_HELP("Cross-preset relay ear: show, peer.", NULL),
		 cmd_ear_show_on_work, 0, 0);
