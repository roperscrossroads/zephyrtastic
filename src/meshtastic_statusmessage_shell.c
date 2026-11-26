/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic statusmsg`: the node status message (away-message) module. Built with the shell and
 * CONFIG_MESHTASTIC_STATUSMESSAGE (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include "meshtastic_shell_internal.h"
#include "meshtastic_statusmessage.h"

/* `meshtastic status` -- the node's away-message (agents-dnr4.26). Same shape as
 * `nodeinfo interval`: reads always available, writes gated behind
 * CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and the is_managed policy. */
static int cmd_status_show(const struct shell *sh, size_t argc, char **argv)
{
	char status[80];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (meshtastic_statusmessage_get(status, sizeof(status)) == 0U) {
		shell_print(sh, "status: (none)");
	} else {
		shell_print(sh, "status: \"%s\"", status);
	}
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)
	shell_print(sh, "auto-send: %d s after a change, then every %d s",
		    CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC,
		    CONFIG_MESHTASTIC_STATUSMESSAGE_INTERVAL_SEC);
#else
	shell_print(sh, "auto-send: off (CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND=n)");
#endif
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_status_show)

static int cmd_status_write(const struct shell *sh, const char *text)
{
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	ARG_UNUSED(text);
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = meshtastic_statusmessage_set(text);
	if (ret < 0) {
		shell_error(sh, "status set failed: %d", ret);
		return ret;
	}
	if (text == NULL || text[0] == '\0') {
		shell_print(sh, "status cleared (persisted; announce cancelled, no reboot)");
	} else {
		char stored[80];

		(void)meshtastic_statusmessage_get(stored, sizeof(stored));
		shell_print(sh, "status -> \"%s\" (persisted; announce re-armed, no reboot)",
			    stored);
	}
	return 0;
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

static int cmd_status_set(const struct shell *sh, size_t argc, char **argv)
{
	char text[80];
	size_t len = 0U;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic statusmsg set <text...>");
		return -EINVAL;
	}

	/* Join the words back: the shell split them, the status is one string. */
	text[0] = '\0';
	for (size_t i = 1; i < argc; i++) {
		int n = snprintk(text + len, sizeof(text) - len, "%s%s", (i > 1U) ? " " : "",
				 argv[i]);

		if (n < 0 || (size_t)n >= sizeof(text) - len) {
			shell_warn(sh, "status truncated to %zu chars", sizeof(text) - 1U);
			break;
		}
		len += (size_t)n;
	}

	return cmd_status_write(sh, text);
}
MTSH_DEFINE_ON_WORK(cmd_status_set)

static int cmd_status_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return cmd_status_write(sh, "");
}
MTSH_DEFINE_ON_WORK(cmd_status_clear)

static int shell_status_send_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_statusmessage_send();
	if (ret == -ENODATA) {
		shell_error(sh, "no status set (meshtastic statusmsg set <text>)");
	} else if (ret == 0) {
		shell_print(sh, "status broadcast sent");
	}
	return ret;
}

static int cmd_status_send(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_status_send_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_status_send)

static int cmd_status_peers(const struct shell *sh, size_t argc, char **argv)
{
	char status[80];
	uint32_t node;
	int64_t age_ms;
	size_t shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < CONFIG_MESHTASTIC_STATUSMESSAGE_CACHE_SIZE; i++) {
		if (!meshtastic_statusmessage_peer_at(i, &node, status, sizeof(status), &age_ms)) {
			continue;
		}
		shell_print(sh, "0x%08x  %6lld s ago  \"%s\"", node, (long long)(age_ms / 1000),
			    status);
		shown++;
	}
	if (shown == 0U) {
		shell_print(sh, "no peer status heard yet");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_status_peers)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_status_cmds,
			       SHELL_CMD(set, NULL,
					 SHELL_HELP("Set and persist the status; re-arms the "
						    "announce, no reboot.",
						    "<text...>"),
					 cmd_status_set_on_work),
			       SHELL_CMD(clear, NULL,
					 SHELL_HELP("Clear the status and cancel the announce.",
						    NULL),
					 cmd_status_clear_on_work),
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Broadcast the status now.", NULL),
					 cmd_status_send_on_work),
			       SHELL_CMD(peers, NULL,
					 SHELL_HELP("Last status heard from each peer.", NULL),
					 cmd_status_peers_on_work),
			       SHELL_SUBCMD_SET_END);

/* `statusmsg`, not `status` (agents-ls39.1.4): `status` is the node's own status,
 * which the bench tools read; a second `status` was never reachable. */
SHELL_SUBCMD_ADD((meshtastic), statusmsg, &meshtastic_status_cmds,
		 SHELL_HELP("Node status message (away-message): show, set, clear, send, "
		 "peers.",
		 NULL),
		 cmd_status_show_on_work, 0, 0);
