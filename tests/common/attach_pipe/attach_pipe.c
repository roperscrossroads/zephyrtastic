/* SPDX-License-Identifier: GPL-3.0 */
/* The attachment pipe, Zephyr half (attach_pipe.h). */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "attach_pipe.h"
#include "attach_pipe_bottom.h"

#define PIPE_HDR_LEN   4U /* len + crc */
#define PIPE_BODY_HDR  5U /* kind + peer */
#define PIPE_BODY_MAX  (PIPE_BODY_HDR + 300U)
#define PIPE_LINKS_MAX 4U

struct pipe_link {
	uint32_t peer;
	struct meshtastic_attach_link_info info;
};

static struct {
	int fd;
	struct pipe_link links[PIPE_LINKS_MAX];
	attach_pipe_rf_cb rf;
	attach_pipe_cmd_cb cmd;
	uint8_t rx[2U * (PIPE_HDR_LEN + PIPE_BODY_MAX)];
	size_t rx_len;
	uint32_t crc_errors;
} pipe = { .fd = -1 };

static K_MUTEX_DEFINE(pipe_lock); /* the socket's write side and the link table */

static uint16_t pipe_crc(const uint8_t *body, size_t len)
{
	return crc16_itu_t(0xFFFFU, body, len); /* CRC-16/CCITT-FALSE */
}

static int pipe_write(uint8_t kind, uint32_t peer, const uint8_t *payload, size_t len)
{
	uint8_t buf[PIPE_HDR_LEN + PIPE_BODY_MAX];
	uint8_t *body = &buf[PIPE_HDR_LEN];
	int ret;

	if (pipe.fd < 0) {
		return -ENOTCONN;
	}
	if (PIPE_BODY_HDR + len > PIPE_BODY_MAX) {
		return -EMSGSIZE;
	}
	body[0] = kind;
	sys_put_le32(peer, &body[1]);
	if (len > 0U) {
		memcpy(&body[PIPE_BODY_HDR], payload, len);
	}
	sys_put_le16((uint16_t)(PIPE_BODY_HDR + len), &buf[0]);
	sys_put_le16(pipe_crc(body, PIPE_BODY_HDR + len), &buf[2]);

	k_mutex_lock(&pipe_lock, K_FOREVER);
	ret = attach_pipe_bottom_write(pipe.fd, buf, (int)(PIPE_HDR_LEN + PIPE_BODY_HDR + len));
	k_mutex_unlock(&pipe_lock);
	return (ret == 0) ? 0 : -EIO;
}

/* ---- the bearer ------------------------------------------------------------ */

static struct pipe_link *link_find_locked(uint32_t peer)
{
	for (unsigned int i = 0U; i < PIPE_LINKS_MAX; i++) {
		if (pipe.links[i].peer == peer && peer != 0U) {
			return &pipe.links[i];
		}
	}
	return NULL;
}

static int pipe_send(uint32_t peer, const uint8_t *env, size_t len)
{
	bool up;

	k_mutex_lock(&pipe_lock, K_FOREVER);
	struct pipe_link *l = link_find_locked(peer);

	up = (l != NULL) && l->info.up;
	k_mutex_unlock(&pipe_lock);
	if (!up) {
		return -EHOSTUNREACH;
	}
	return pipe_write(ATTACH_PIPE_ENV, peer, env, len);
}

static bool pipe_link_info(uint32_t peer, struct meshtastic_attach_link_info *out)
{
	bool known = false;

	k_mutex_lock(&pipe_lock, K_FOREVER);
	struct pipe_link *l = link_find_locked(peer);

	if (l != NULL) {
		*out = l->info;
		known = true;
	}
	k_mutex_unlock(&pipe_lock);
	return known;
}

const struct meshtastic_attach_bearer attach_pipe_bearer = {
	.name = "pipe",
	.send = pipe_send,
	.link_info = pipe_link_info,
};

/* ---- RX -------------------------------------------------------------------- */

static void on_link(uint32_t peer, const uint8_t *p, size_t len)
{
	struct pipe_link *l;
	bool was_up = false;
	bool up;

	if (len < 5U || peer == 0U) {
		return;
	}
	up = p[0] != 0U;
	k_mutex_lock(&pipe_lock, K_FOREVER);
	l = link_find_locked(peer);
	if (l == NULL) {
		for (unsigned int i = 0U; i < PIPE_LINKS_MAX; i++) {
			if (pipe.links[i].peer == 0U) {
				l = &pipe.links[i];
				l->peer = peer;
				break;
			}
		}
	}
	if (l != NULL) {
		was_up = l->info.up;
		l->info.up = up;
		l->info.auth = (enum meshtastic_attach_auth)p[1];
		l->info.takes_envelopes = p[2] != 0U;
		l->info.mtu = 0U; /* a stream */
		l->info.rtt_ms = sys_get_le16(&p[3]);
	}
	k_mutex_unlock(&pipe_lock);
	if (l != NULL && was_up && !up) {
		meshtastic_attach_bearer_link_down(&attach_pipe_bearer, peer);
	}
}

static void on_frame(const uint8_t *body, size_t len)
{
	const uint8_t kind = body[0];
	const uint32_t peer = sys_get_le32(&body[1]);
	const uint8_t *p = &body[PIPE_BODY_HDR];
	const size_t plen = len - PIPE_BODY_HDR;

	switch (kind) {
	case ATTACH_PIPE_ENV: {
		int ret = meshtastic_attach_bearer_rx(&attach_pipe_bearer, peer, p, plen);

		if (ret < 0) {
			attach_pipe_event("pipe env_rx peer=%08x rc=%d", peer, ret);
		}
		break;
	}
	case ATTACH_PIPE_LINK:
		on_link(peer, p, plen);
		break;
	case ATTACH_PIPE_RF:
		if (plen > 11U && pipe.rf != NULL) {
			const struct attach_pipe_rf rf = {
				.preset = p[0],
				.rssi = (int16_t)sys_get_le16(&p[1]),
				.snr = (int8_t)p[3],
				.freq_hz = sys_get_le32(&p[4]),
				.sf = p[8],
				.bw_khz = sys_get_le16(&p[9]),
			};

			pipe.rf(&rf, &p[11], plen - 11U);
		}
		break;
	case ATTACH_PIPE_CMD:
		if (pipe.cmd != NULL && plen < 128U) {
			char cmd[128];

			memcpy(cmd, p, plen);
			cmd[plen] = '\0';
			pipe.cmd(cmd);
		}
		break;
	default:
		break;
	}
}

static void rx_parse(void)
{
	while (pipe.rx_len >= PIPE_HDR_LEN) {
		const uint16_t blen = sys_get_le16(&pipe.rx[0]);
		const uint16_t crc = sys_get_le16(&pipe.rx[2]);
		const size_t flen = PIPE_HDR_LEN + blen;

		if (blen < PIPE_BODY_HDR || blen > PIPE_BODY_MAX) {
			/* Unframeable: the stream is lost. Drop everything. */
			pipe.crc_errors++;
			pipe.rx_len = 0U;
			return;
		}
		if (pipe.rx_len < flen) {
			return;
		}
		if (pipe_crc(&pipe.rx[PIPE_HDR_LEN], blen) == crc) {
			on_frame(&pipe.rx[PIPE_HDR_LEN], blen);
		} else {
			pipe.crc_errors++;
		}
		memmove(pipe.rx, &pipe.rx[flen], pipe.rx_len - flen);
		pipe.rx_len -= flen;
	}
}

static void pipe_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		int n = attach_pipe_bottom_read(pipe.fd, &pipe.rx[pipe.rx_len],
						(int)(sizeof(pipe.rx) - pipe.rx_len));

		if (n < 0) {
			/* The hub is gone: so is the test. */
			printk("attach_pipe: hub closed the socket\n");
			k_msleep(100);
			continue;
		}
		if (n > 0) {
			pipe.rx_len += (size_t)n;
			rx_parse();
			continue;
		}
		k_msleep(1);
	}
}

K_THREAD_STACK_DEFINE(pipe_stack, 4096);
static struct k_thread pipe_thread;

/* ---- API ------------------------------------------------------------------- */

const char *attach_pipe_getenv(const char *name, const char *dflt)
{
	const char *v = attach_pipe_bottom_getenv(name);

	return (v != NULL) ? v : dflt;
}

void attach_pipe_event(const char *fmt, ...)
{
	char line[200];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n < 0) {
		return;
	}
	(void)pipe_write(ATTACH_PIPE_EVENT, 0U, (const uint8_t *)line,
			 MIN((size_t)n, sizeof(line) - 1U));
}

int attach_pipe_start(enum attach_pipe_role role, uint32_t node, attach_pipe_rf_cb rf,
		      attach_pipe_cmd_cb cmd)
{
	const char *path = attach_pipe_getenv("ATTACH_PIPE_SOCK", NULL);
	const uint8_t hello = (uint8_t)role;
	int ret;

	if (path == NULL) {
		return -EINVAL;
	}
	pipe.fd = attach_pipe_bottom_connect(path);
	if (pipe.fd < 0) {
		return -ECONNREFUSED;
	}
	pipe.rf = rf;
	pipe.cmd = cmd;
	ret = meshtastic_attach_bearer_register(&attach_pipe_bearer);
	if (ret < 0) {
		return ret;
	}
	k_thread_create(&pipe_thread, pipe_stack, K_THREAD_STACK_SIZEOF(pipe_stack), pipe_thread_fn,
			NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&pipe_thread, "attach_pipe");
	return pipe_write(ATTACH_PIPE_HELLO, node, &hello, 1U);
}
