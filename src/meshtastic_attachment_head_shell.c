/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic attach` on a keyless radio head: its brain and status. Built with the
 * shell and CONFIG_MESHTASTIC_ATTACHMENT_HEAD (src/CMakeLists.txt); moved from
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

#include "meshtastic_attachment_head.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic attach` on a keyless head (ATTACHMENT-DESIGN S5): whose radio
 * this is, and what has passed through. */
static int cmd_attach_head_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_attachment_head_stats st;
	uint32_t brain = meshtastic_attachment_head_get_brain();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_attachment_head_stats_get(&st);
	shell_print(sh, "role: keyless radio head (transmits what its brain hands it)");
	shell_print(sh, "brain: 0x%08x%s", brain, brain == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "preset: %s (%u)", meshtastic_preset_display_name(mt.modem_preset, true),
		    (unsigned int)mt.modem_preset);
	shell_print(sh, "heard %u  forwarded %u  no_brain %u  queue_full %u  send_failed %u",
		    st.heard, st.forwarded, st.no_brain, st.queue_full, st.send_failed);
	shell_print(sh, "status_sent %u  controls %u  refused %u  untrusted %u  rejected %u",
		    st.status_sent, st.controls, st.refused, st.untrusted, st.rejected);
	shell_print(sh, "tx: sent %u  failed %u  deferred %u  queue_full %u  cancelled %u  late %u  late_dropped %u  duty_blocked %u",
		    st.tx_sent, st.tx_failed, st.tx_deferred, st.tx_queue_full, st.tx_cancelled,
		    st.tx_late, st.tx_late_dropped, st.tx_duty_blocked);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_show)

static int cmd_attach_head_brain(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_attachment_head_set_brain(node);
	shell_print(sh, "brain: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_brain)

static int shell_head_status_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_attachment_head_status_send();
	if (ret == 0) {
		shell_print(sh, "status sent");
	}
	return ret;
}

static int cmd_attach_head_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_head_status_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_status)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_attach_head_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Brain and counters.", NULL), cmd_attach_head_show_on_work),
	SHELL_CMD_ARG(brain, NULL,
		      SHELL_HELP("The brain's node number (0 stops forwarding).", "<hex node id>"),
		      cmd_attach_head_brain_on_work, 2, 0),
	SHELL_CMD(status, NULL, SHELL_HELP("Send a STATUS to the brain now.", NULL),
		  cmd_attach_head_status_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), attach, &meshtastic_attach_head_cmds,
		 SHELL_HELP("This keyless radio head: show, brain, status.", NULL),
		 cmd_attach_head_show_on_work, 0, 0);
