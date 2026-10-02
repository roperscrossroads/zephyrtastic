/* SPDX-License-Identifier: GPL-3.0
 *
 * `meshtastic api`: the phone API (ToRadio in, FromRadio out) as text over the shell.
 *
 * A stock Meshtastic node speaks its phone API on the USB serial port. Here that port is the
 * shell and the log, and it cannot carry the binary framing: the shell's UART backend hands
 * every received byte to the SMP parser first, and rewrites newlines on the way out. So the
 * frames travel as base64 inside ordinary commands and ordinary output lines, and a client
 * library that speaks the phone API works over the same wire as a person typing.
 *
 *     meshtastic api                    serve what is waiting
 *     meshtastic api + <base64>         part of a ToRadio, more to come
 *     meshtastic api <base64> <crc16>   the last (or only) part; the CRC-16 (XMODEM, hex) is
 *                                       over the whole decoded ToRadio
 *
 * Each part is base64 by itself. A ToRadio is cut into parts because a shell line is short
 * (CONFIG_SHELL_CMD_BUFF_SIZE), and carries a CRC because a console drops bytes in silence
 * and a damaged frame could still decode as a different, valid one.
 *
 * The answer is zero or more frames and exactly one status line:
 *
 *     ~P<base64 FromRadio>*<crc16>      one frame; the CRC is over the base64 text
 *     ~P.<n>                            n frames served, nothing more waiting
 *     ~P+<n>                            n frames served, more waiting: ask again
 *     ~P:<bytes>                        part stored, this many bytes held
 *     ~P!<why>                          refused: args, b64, len, crc, busy
 *
 * The sentinel keeps a log line on the same console from being read as a frame, as with
 * `meshtastic state`. A send serves frames in the same answer, so a request and its reply
 * are one command.
 *
 * The phone API core runs on the shell work thread, not the shell's own: the shell thread's
 * stack cannot carry the admin and send paths (2 KB on the XIAO), and every transport must
 * have one thread that touches its instance. The shell thread decodes and prints; it waits
 * for the work thread between steps, because only the shell thread may print while a
 * command is running.
 *
 * This is a transport like BLE: is_managed and lockdown apply to it unchanged. It is also a
 * way to write configuration from an unauthenticated console, so it is not built where the
 * shell's own writes are compiled out.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/base64.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "meshtastic_ext_ram.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_shell_api.h"

/* How long the shell thread waits for one step. A send can sit behind the radio. */
#define API_STEP_TIMEOUT K_SECONDS(10)

/* Raw bytes per output chunk: a multiple of 3, so every chunk but the last encodes without
 * padding and the chunks concatenate into one valid base64 text. */
#define API_OUT_CHUNK 48U

enum api_op {
	API_OP_HANDLE,
	API_OP_POP,
};

static MESHTASTIC_EXT_RAM_BSS_ATTR
	struct meshtastic_phoneapi_frame api_queue[CONFIG_MESHTASTIC_SHELL_PHONEAPI_QUEUE_SIZE];
static MESHTASTIC_EXT_RAM_BSS_ATTR meshtastic_ToRadio api_to_scratch;
static MESHTASTIC_EXT_RAM_BSS_ATTR meshtastic_FromRadio api_from_scratch;

/* One buffer for both directions: the ToRadio being assembled, then each FromRadio being
 * served. They never overlap: a ToRadio is decoded out of it before the first pop, and
 * serving frames drops a ToRadio left half assembled. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_phoneapi_frame api_io;

static struct {
	struct meshtastic_phoneapi api;
	struct k_sem done;
	/* Set while the work thread owns api_io. Outlives a step the shell thread gave up
	 * waiting for, so the next command is refused and not run on top of it. */
	atomic_t busy;
	enum api_op op;
	bool got;
	uint16_t rx_len;
} tun;

static void api_disconnect(struct meshtastic_phoneapi *api)
{
	/* The client said goodbye: the next one starts a new session, and is still owed
	 * what arrives in between. */
	meshtastic_phoneapi_session_reset(api);
}

void meshtastic_shell_api_init(void)
{
	k_sem_init(&tun.done, 0, 1);
	meshtastic_phoneapi_init(&tun.api, "shell", api_queue, ARRAY_SIZE(api_queue), NULL,
				 api_disconnect, NULL, NULL, &api_to_scratch, &api_from_scratch);
	meshtastic_phoneapi_register(&tun.api);
}

void meshtastic_shell_api_work(void)
{
	if (tun.op == API_OP_HANDLE) {
		meshtastic_phoneapi_handle_toradio(&tun.api, api_io.data, tun.rx_len);
		tun.rx_len = 0U;
	} else {
		tun.got = meshtastic_phoneapi_pop_frame(&tun.api, &api_io);
	}
	atomic_clear(&tun.busy);
	k_sem_give(&tun.done);
}

static int api_step(enum api_op op)
{
	int ret;

	tun.op = op;
	tun.got = false;
	k_sem_reset(&tun.done);
	atomic_set(&tun.busy, 1);
	ret = meshtastic_shell_api_submit();
	if (ret < 0) {
		atomic_clear(&tun.busy);
		return ret;
	}
	return k_sem_take(&tun.done, API_STEP_TIMEOUT);
}

static int api_refuse(const struct shell *sh, const char *why)
{
	tun.rx_len = 0U;
	shell_fprintf(sh, SHELL_NORMAL, "~P!%s\n", why);
	return -EINVAL;
}

static void api_print_frame(const struct shell *sh)
{
	char text[API_OUT_CHUNK / 3U * 4U + 1U];
	uint16_t crc = 0U;
	size_t n;

	shell_fprintf(sh, SHELL_NORMAL, "~P");
	for (size_t off = 0U; off < api_io.len; off += API_OUT_CHUNK) {
		if (base64_encode((uint8_t *)text, sizeof(text), &n, &api_io.data[off],
				  MIN(API_OUT_CHUNK, api_io.len - off)) != 0) {
			break;
		}
		crc = crc16_itu_t(crc, (const uint8_t *)text, n);
		shell_fprintf(sh, SHELL_NORMAL, "%s", text);
	}
	shell_fprintf(sh, SHELL_NORMAL, "*%04x\n", crc);
}

/* Decode one part behind what is already held. */
static const char *api_append(const char *b64)
{
	size_t n = 0U;
	int ret = base64_decode(&api_io.data[tun.rx_len], sizeof(api_io.data) - tun.rx_len, &n,
				(const uint8_t *)b64, strlen(b64));

	if (ret == -ENOMEM) {
		return "len";
	}
	if (ret != 0 || n == 0U) {
		return "b64";
	}
	tun.rx_len += (uint16_t)n;
	return NULL;
}

int meshtastic_shell_api_cmd(const struct shell *sh, size_t argc, char **argv)
{
	unsigned int served = 0U;
	bool more = false;
	const char *why;

	if (atomic_get(&tun.busy) != 0) {
		shell_fprintf(sh, SHELL_NORMAL, "~P!busy\n");
		return -EBUSY;
	}

	if (argc == 3U && strcmp(argv[1], "+") == 0) {
		why = api_append(argv[2]);
		if (why != NULL) {
			return api_refuse(sh, why);
		}
		shell_fprintf(sh, SHELL_NORMAL, "~P:%u\n", (unsigned int)tun.rx_len);
		return 0;
	}

	if (argc == 3U) {
		char *end;
		unsigned long want = strtoul(argv[2], &end, 16);

		why = api_append(argv[1]);
		if (why != NULL) {
			return api_refuse(sh, why);
		}
		if (end == argv[2] || *end != '\0' ||
		    want != crc16_itu_t(0U, api_io.data, tun.rx_len)) {
			return api_refuse(sh, "crc");
		}
		if (api_step(API_OP_HANDLE) != 0) {
			return api_refuse(sh, "busy");
		}
	} else if (argc != 1U) {
		return api_refuse(sh, "args");
	}

	tun.rx_len = 0U;
	while (true) {
		if (served == CONFIG_MESHTASTIC_SHELL_PHONEAPI_BURST) {
			more = meshtastic_phoneapi_pending_count(&tun.api) > 0U;
			break;
		}
		if (api_step(API_OP_POP) != 0) {
			return api_refuse(sh, "busy");
		}
		if (!tun.got) {
			break;
		}
		api_print_frame(sh);
		served++;
	}
	shell_fprintf(sh, SHELL_NORMAL, "~P%c%u\n", more ? '+' : '.', served);
	return 0;
}
