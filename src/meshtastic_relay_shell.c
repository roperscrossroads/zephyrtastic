/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic relay`: the cross-preset text relay (receiving half). Built with the shell and
 * CONFIG_MESHTASTIC_RELAY (src/CMakeLists.txt); moved from meshtastic_shell.c
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

/* `meshtastic relay` (agents-jbrq.12): the cross-preset relay's receiving
 * half. The direction and ignore list are saved (MESHTASTIC_SETTINGS); `reset`
 * forgets them. */
static const char *const relay_dir_names[] = { "off", "in", "out", "both" };

static int cmd_relay_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_relay_stats st;
	enum meshtastic_relay_dir dir = meshtastic_relay_get_direction();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_relay_stats_get(&st);
	shell_print(sh, "direction: %s", relay_dir_names[dir & 3]);
	shell_print(sh, "considered %u  relayed %u  sent %u  tx_failed %u", st.considered,
		    st.relayed, st.sent, st.tx_failed);
	shell_print(sh, "refused: dir_off %u  not_broadcast %u  not_text %u  reaction %u  ignored %u",
		    st.dir_off, st.not_broadcast, st.not_text, st.reaction, st.ignored);
	shell_print(sh, "         bad_text %u  prefixed %u  no_mapping %u  seen %u",
		    st.bad_text, st.prefixed, st.no_mapping, st.seen);
	shell_print(sh, "         rate_dropped %u  origin_limited %u  queue_full %u", st.rate_dropped,
		    st.origin_limited, st.queue_full);
	shell_print(sh, "relayed-but: sanitized %u  unprefixed %u;  log lines held back %u",
		    st.sanitized, st.unprefixed, st.log_suppressed);
	shell_print(sh, "cap %d per %d s, per origin %d per %d s, seen TTL %d s",
		    CONFIG_MESHTASTIC_RELAY_RATE_MAX, CONFIG_MESHTASTIC_RELAY_RATE_WINDOW_SEC,
		    CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_MAX,
		    CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_WINDOW_SEC,
		    CONFIG_MESHTASTIC_RELAY_SEEN_TTL_SEC);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_show)

static int cmd_relay_dir(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	for (size_t i = 0U; i < ARRAY_SIZE(relay_dir_names); i++) {
		if (strcmp(argv[1], relay_dir_names[i]) == 0) {
			ret = meshtastic_relay_set_direction((enum meshtastic_relay_dir)i);
			if (ret == -ENOTSUP) {
				shell_error(sh, "%s needs an ear that transmits (not in v1)",
					    argv[1]);
			} else if (ret < 0) {
				shell_error(sh, "failed (%d)", ret);
			} else {
				shell_print(sh, "direction: %s", argv[1]);
			}
			return ret;
		}
	}
	shell_error(sh, "usage: meshtastic relay dir <off|in|out|both>");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_relay_dir)

static int cmd_relay_ignore(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node;
	int ret;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "clear") == 0) {
		meshtastic_relay_ignore_clear();
		shell_print(sh, "ignore list cleared");
		return 0;
	}
	node = (uint32_t)strtoul(argv[1], NULL, 16);
	if (node == 0U) {
		shell_error(sh, "usage: meshtastic relay ignore <hex node id>|clear");
		return -EINVAL;
	}
	ret = meshtastic_relay_ignore_add(node);
	if (ret < 0) {
		shell_error(sh, "ignore list full (%d)", ret);
		return ret;
	}
	shell_print(sh, "never relaying 0x%08x", node);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_ignore)

static int cmd_relay_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	meshtastic_relay_reset();
	shell_print(sh, "relay reset: boot direction, caches, counters, ignore list; saved settings forgotten");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_relay_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Direction and counters.", NULL), cmd_relay_show_on_work),
	SHELL_CMD_ARG(dir, NULL,
		      SHELL_HELP("Set the direction (v1: off or in).", "<off|in|out|both>"),
		      cmd_relay_dir_on_work, 2, 0),
	SHELL_CMD_ARG(ignore, NULL,
		      SHELL_HELP("Never relay a node (another relay), or clear the list.",
				 "<hex node id>|clear"),
		      cmd_relay_ignore_on_work, 2, 0),
	SHELL_CMD(reset, NULL, SHELL_HELP("Back to boot state; forgets the saved settings.", NULL),
		  cmd_relay_reset_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), relay, &meshtastic_relay_cmds,
		 SHELL_HELP("Cross-preset text relay: show, dir, ignore, reset.", NULL),
		 cmd_relay_show_on_work, 0, 0);
