/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic text`: send a text message, and the inbox. Built with the shell and
 * CONFIG_MESHTASTIC_MESSAGE (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
#include "meshtastic_text_inbox.h"
#endif

struct shell_text_work {
	uint32_t dest;
	uint8_t channel_index; /* MESHTASTIC_CHANNEL_INDEX_INVALID: the default channel */
	uint8_t text[MESHTASTIC_MAX_TEXT_LEN + 1U]; /* NUL-terminated */
};

BUILD_ASSERT(sizeof(struct shell_text_work) <= MTSH_WORK_ARG_MAX,
	     "a text message rides in a shell work item");

static int shell_text_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_text_work *tw = arg;

	ARG_UNUSED(sh);
	if (tw->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
		struct meshtastic_packet packet = {
			.to = tw->dest,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = tw->text,
			.payload_len = len - offsetof(struct shell_text_work, text),
			.channel_index = tw->channel_index,
		};

		return meshtastic_send_packet(&packet, K_FOREVER);
	}
	return meshtastic_send_text(tw->dest, (const char *)tw->text);
}

static int cmd_text_send(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t channel_index = MESHTASTIC_CHANNEL_INDEX_INVALID;
	uint32_t dest = MESHTASTIC_NODE_BROADCAST;
	struct shell_text_work *tw;
	struct mtsh_work *w;
	size_t msg_arg = 1U;
	size_t first;
	int ret;
	int len;

	if (argc < 2U) {
		shell_error(sh,
			    "usage: meshtastic text send [-c <index>] [dest|broadcast] <message>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "-c") == 0) {
		if (argc < 4U) {
			shell_error(sh, "usage: meshtastic text send -c <index> [dest|broadcast] "
					"<message>");
			return -EINVAL;
		}
		ret = mtsh_parse_channel_index(sh, argv[2], &channel_index);
		if (ret < 0) {
			return ret;
		}
		msg_arg = 3U;
	}

	if (argc == (msg_arg + 1U)) {
		first = msg_arg;
	} else if (argc >= (msg_arg + 2U)) {
		ret = mtsh_parse_u32(sh, argv[msg_arg], &dest);
		if (ret < 0) {
			return ret;
		}
		first = msg_arg + 1U;
	} else {
		shell_error(sh, "message required");
		return -EINVAL;
	}

	/* The message is built in the queued item itself, not on this stack. */
	w = mtsh_work_alloc(sh, shell_text_run);
	if (w == NULL) {
		return -ENOMSG;
	}
	tw = (struct shell_text_work *)w->arg;
	tw->dest = dest;
	tw->channel_index = channel_index;
	len = mtsh_append_argv(tw->text, MESHTASTIC_MAX_TEXT_LEN, argc, argv, first);

	if (len < 0) {
		mtsh_work_cancel(w);
		shell_error(sh, "message too long (max %u)", MESHTASTIC_MAX_TEXT_LEN);
		return len;
	}

	if (len == 0) {
		mtsh_work_cancel(w);
		shell_error(sh, "message must not be empty");
		return -EINVAL;
	}

	tw->text[len] = '\0';

	return mtsh_work_submit(w, offsetof(struct shell_text_work, text) + (size_t)len);
}
MTSH_DEFINE_ON_WORK(cmd_text_send)

#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
/* One kept message as one line: what a test matches and a person reads. The text
 * comes last, so everything before it has a fixed shape; a byte below 0x20 or DEL
 * is shown as '?' so a message cannot break the line or the terminal. */
static void text_inbox_print(const struct shell *sh, const struct meshtastic_text_inbox_entry *e)
{
	char via[16];
	char text[CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN + 1];
	const char *on = "?";
	int64_t age_s = (k_uptime_get() - e->uptime_ms) / 1000;

	if (e->rx_heard_on > 0U) {
		on = meshtastic_preset_display_name(
			(meshtastic_Config_LoRaConfig_ModemPreset)(e->rx_heard_on - 1U), true);
	}
	if (e->rx_attach == 0U) {
		snprintk(via, sizeof(via), "own");
	} else {
		snprintk(via, sizeof(via), "head %u", e->rx_attach);
	}
	for (size_t i = 0; i <= e->len; i++) {
		unsigned char c = (unsigned char)e->text[i];

		text[i] = (i < e->len && (c < 0x20U || c == 0x7FU)) ? '?' : (char)c;
	}
	shell_print(sh, "#%u %llds ago from 0x%08x to 0x%08x ch %u on %s via %s rssi %d snr %d "
		    "hops %u%s%s: %s",
		    e->seq, (long long)age_s, e->from, e->to, e->channel_index, on, via, e->rssi,
		    e->snr, e->hops, e->pki ? " pki" : "", e->truncated ? " truncated" : "", text);
}

static int cmd_text_inbox(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_text_inbox_entry e;
	unsigned int show = CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES;

	if (argc > 1 && strcmp(argv[1], "clear") == 0) {
		meshtastic_text_inbox_clear();
		shell_print(sh, "inbox cleared");
		return 0;
	}
	if (argc > 1) {
		char *end;
		unsigned long n = strtoul(argv[1], &end, 10);

		if (*end != '\0' || n == 0UL) {
			shell_error(sh, "usage: meshtastic text inbox [<count>|clear]");
			return -EINVAL;
		}
		show = (unsigned int)MIN(n, (unsigned long)show);
	}

	shell_print(sh, "inbox: %u kept of %u, %u received since boot",
		    meshtastic_text_inbox_count(), CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES,
		    meshtastic_text_inbox_total());
	for (unsigned int i = 0; i < show && meshtastic_text_inbox_get(i, &e) == 0; i++) {
		text_inbox_print(sh, &e);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_text_inbox)
#endif /* CONFIG_MESHTASTIC_TEXT_INBOX */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_text_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send text message.",
						    "[-c <index>] [dest|broadcast] <message>"),
					 cmd_text_send_on_work),
#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
			       SHELL_CMD_ARG(inbox, NULL,
					     SHELL_HELP("The last text messages received, newest first.",
							"[<count>|clear]"),
					     cmd_text_inbox_on_work, 1, 1),
#endif
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), text, &meshtastic_text_cmds,
		 SHELL_HELP("Text message commands.", NULL),
		 NULL, 0, 0);
