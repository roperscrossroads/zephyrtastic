/* SPDX-FileCopyrightText: The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * Meshtastic TCP PhoneAPI transport. Exposes the PhoneAPI over TCP using the
 * stock Meshtastic StreamAPI framing (0x94 0xc3 <len16> <protobuf>), so the
 * Android app or `meshtastic --host <ip>` can connect over IP.
 *
 * One client at a time, "newest wins": a new connection preempts any existing
 * one. This matters because the reference CLI opens a fresh connection per
 * command and often tears it down uncleanly (half-open, or NAT-dropped) — the
 * next command's connection simply replaces the stale one instead of queueing
 * behind a dead socket. SO_KEEPALIVE reaps peers that vanish without a FIN even
 * if nothing reconnects, and an idle timeout is a final backstop.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "meshtastic_phoneapi.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define TCP_START1   0x94U
#define TCP_START2   0xc3U
#define TCP_HEADER   4U
#define TCP_WIRE_MAX (MESHTASTIC_API_FRAME_MAX + TCP_HEADER)
#define TCP_PORT     CONFIG_MESHTASTIC_TCP_PORT
#define TCP_IDLE_MS  ((int64_t)CONFIG_MESHTASTIC_TCP_IDLE_TIMEOUT_SEC * 1000)

/* Keepalive: detect a peer that disappeared without a FIN in ~idle+intvl*cnt. */
#define TCP_KA_IDLE_SEC  30
#define TCP_KA_INTVL_SEC 5
#define TCP_KA_CNT       3

/* StreamAPI receive parser states (byte-wise, survives recv() boundaries). */
enum tcp_rx_state {
	TCP_RX_WAIT_START1,
	TCP_RX_WAIT_START2,
	TCP_RX_LEN_HI,
	TCP_RX_LEN_LO,
	TCP_RX_PAYLOAD,
};

static struct {
	struct meshtastic_phoneapi api;
	struct meshtastic_phoneapi_frame queue[CONFIG_MESHTASTIC_TCP_FROMRADIO_QUEUE_SIZE];
	int client_fd;
	enum tcp_rx_state rx_state;
	uint16_t rx_len;
	uint16_t rx_pos;
	uint8_t rx_payload[MESHTASTIC_API_FRAME_MAX];
} tcp = {
	.client_fd = -1,
};

K_THREAD_STACK_DEFINE(tcp_thread_stack, CONFIG_MESHTASTIC_TCP_THREAD_STACK_SIZE);
static struct k_thread tcp_thread_data;

static size_t tcp_encode_frame(const uint8_t *payload, size_t payload_len, uint8_t *out,
			       size_t out_len)
{
	if (payload_len > MESHTASTIC_API_FRAME_MAX || out_len < payload_len + TCP_HEADER) {
		return 0U;
	}

	out[0] = TCP_START1;
	out[1] = TCP_START2;
	out[2] = (uint8_t)(payload_len >> 8);
	out[3] = (uint8_t)payload_len;
	memcpy(&out[TCP_HEADER], payload, payload_len);

	return payload_len + TCP_HEADER;
}

static int tcp_send_all(int fd, const uint8_t *buf, size_t len)
{
	size_t off = 0U;

	while (off < len) {
		ssize_t n = zsock_send(fd, buf + off, len - off, 0);

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}
			return -errno;
		}
		off += (size_t)n;
	}

	return 0;
}

/* Drain queued FromRadio frames (incl. self-pumped config stream) to the client.
 * Returns false if the socket write failed (caller should drop the client).
 * Runs only on the TCP thread, so all socket writes are single-threaded. */
static bool tcp_drain_tx(void)
{
	struct meshtastic_phoneapi_frame frame;
	uint8_t wire[TCP_WIRE_MAX];

	if (tcp.client_fd < 0) {
		return true;
	}

	while (meshtastic_phoneapi_pop_frame(&tcp.api, &frame)) {
		size_t wire_len = tcp_encode_frame(frame.data, frame.len, wire, sizeof(wire));

		if (wire_len == 0U) {
			continue;
		}

		if (tcp_send_all(tcp.client_fd, wire, wire_len) < 0) {
			/* Keep the frame for the next client; peer is gone. */
			meshtastic_phoneapi_push_frame_front(&tcp.api, &frame);
			return false;
		}
	}

	return true;
}

static void tcp_rx_byte(uint8_t byte)
{
	switch (tcp.rx_state) {
	case TCP_RX_WAIT_START1:
		if (byte == TCP_START1) {
			tcp.rx_state = TCP_RX_WAIT_START2;
		}
		break;
	case TCP_RX_WAIT_START2:
		if (byte == TCP_START2) {
			tcp.rx_state = TCP_RX_LEN_HI;
		} else if (byte == TCP_START1) {
			/* stay, waiting for the matching START2 */
		} else {
			tcp.rx_state = TCP_RX_WAIT_START1;
		}
		break;
	case TCP_RX_LEN_HI:
		tcp.rx_len = ((uint16_t)byte) << 8;
		tcp.rx_state = TCP_RX_LEN_LO;
		break;
	case TCP_RX_LEN_LO:
		tcp.rx_len |= byte;
		tcp.rx_pos = 0U;
		if (tcp.rx_len > MESHTASTIC_API_FRAME_MAX) {
			LOG_DBG("TCP PhoneAPI oversize len %u, resyncing", tcp.rx_len);
			tcp.rx_state = TCP_RX_WAIT_START1;
			break;
		}
		if (tcp.rx_len == 0U) {
			tcp.rx_state = TCP_RX_WAIT_START1;
			break;
		}
		tcp.rx_state = TCP_RX_PAYLOAD;
		break;
	case TCP_RX_PAYLOAD:
		tcp.rx_payload[tcp.rx_pos++] = byte;
		if (tcp.rx_pos == tcp.rx_len) {
			LOG_DBG("TCP PhoneAPI RX StreamAPI frame len=%u", tcp.rx_len);
			meshtastic_phoneapi_handle_toradio(&tcp.api, tcp.rx_payload, tcp.rx_len);
			tcp.rx_state = TCP_RX_WAIT_START1;
		}
		break;
	default:
		tcp.rx_state = TCP_RX_WAIT_START1;
		break;
	}
}

static void tcp_data_ready(struct meshtastic_phoneapi *api)
{
	/* No cross-thread socket I/O: the TCP thread drains on every poll cycle
	 * (short timeout) and immediately after each RX, so async FromRadio
	 * frames flush without waking here. */
	ARG_UNUSED(api);
}

static void tcp_disconnect(struct meshtastic_phoneapi *api)
{
	ARG_UNUSED(api);
}

static void tcp_set_keepalive(int fd)
{
#if defined(CONFIG_NET_TCP_KEEPALIVE)
	int on = 1;
	int idle = TCP_KA_IDLE_SEC;
	int intvl = TCP_KA_INTVL_SEC;
	int cnt = TCP_KA_CNT;

	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
	(void)zsock_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
	(void)zsock_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
	(void)zsock_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#else
	ARG_UNUSED(fd);
#endif
}

static void tcp_close_client(void)
{
	if (tcp.client_fd < 0) {
		return;
	}

	zsock_close(tcp.client_fd);
	tcp.client_fd = -1;
	meshtastic_phoneapi_reset(&tcp.api);
	LOG_INF("Meshtastic TCP PhoneAPI client disconnected");
}

static void tcp_accept_client(int lfd)
{
	int nfd = zsock_accept(lfd, NULL, NULL);

	if (nfd < 0) {
		return;
	}

	/* Newest wins: drop any stale/half-open client so this one is served. */
	if (tcp.client_fd >= 0) {
		LOG_INF("Meshtastic TCP PhoneAPI new client preempts previous");
		tcp_close_client();
	}

	tcp_set_keepalive(nfd);
	tcp.client_fd = nfd;
	tcp.rx_state = TCP_RX_WAIT_START1;

	/* Fresh session: reset the PhoneAPI and greet with the rebooted marker
	 * so the client (re)starts its want_config handshake. */
	meshtastic_phoneapi_reset(&tcp.api);
	meshtastic_phoneapi_enqueue_rebooted(&tcp.api);
	(void)tcp_drain_tx();

	LOG_INF("Meshtastic TCP PhoneAPI client connected");
}

static int tcp_open_listener(void)
{
	struct sockaddr_in addr;
	int fd;
	int opt = 1;

	fd = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		return -errno;
	}

	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(TCP_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (zsock_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		int err = -errno;

		zsock_close(fd);
		return err;
	}

	if (zsock_listen(fd, 1) < 0) {
		int err = -errno;

		zsock_close(fd);
		return err;
	}

	return fd;
}

static void tcp_thread(void *p1, void *p2, void *p3)
{
	int lfd;
	int64_t last_rx = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* The net stack may still be initialising at boot; retry the listener. */
	while (true) {
		lfd = tcp_open_listener();
		if (lfd >= 0) {
			break;
		}
		LOG_DBG("TCP PhoneAPI listener not ready (%d), retrying", lfd);
		k_sleep(K_SECONDS(2));
	}

	LOG_INF("Meshtastic TCP PhoneAPI listening on :%u", TCP_PORT);

	while (true) {
		struct zsock_pollfd fds[2];
		int nfds = 1;
		int ret;

		fds[0].fd = lfd;
		fds[0].events = ZSOCK_POLLIN;
		fds[0].revents = 0;

		if (tcp.client_fd >= 0) {
			fds[1].fd = tcp.client_fd;
			fds[1].events = ZSOCK_POLLIN;
			fds[1].revents = 0;
			nfds = 2;
		}

		ret = zsock_poll(fds, nfds, 100);
		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			}
			k_sleep(K_MSEC(200));
			continue;
		}

		/* New connection: accept and preempt any stale client. */
		if (fds[0].revents & ZSOCK_POLLIN) {
			tcp_accept_client(lfd);
			last_rx = k_uptime_get();
		}

		/* Existing client activity. */
		if (nfds == 2 && tcp.client_fd >= 0) {
			if (fds[1].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)) {
				tcp_close_client();
				continue;
			}

			if (fds[1].revents & ZSOCK_POLLIN) {
				uint8_t chunk[256];
				ssize_t n = zsock_recv(tcp.client_fd, chunk, sizeof(chunk), 0);

				if (n == 0) {
					tcp_close_client(); /* orderly close */
					continue;
				}
				if (n < 0) {
					if (errno == EINTR || errno == EAGAIN) {
						continue;
					}
					tcp_close_client();
					continue;
				}

				last_rx = k_uptime_get();
				for (ssize_t i = 0; i < n; i++) {
					tcp_rx_byte(chunk[i]);
				}
			}
		}

		/* Flush handshake responses and async FromRadio frames; a failed
		 * write means the peer is gone. */
		if (tcp.client_fd >= 0 && !tcp_drain_tx()) {
			LOG_DBG("TCP PhoneAPI send failed, dropping client");
			tcp_close_client();
			continue;
		}

		/* Idle backstop: reclaim a client that went silent with no clean
		 * close and no keepalive drop. */
		if (tcp.client_fd >= 0 && TCP_IDLE_MS > 0 &&
		    (k_uptime_get() - last_rx) > TCP_IDLE_MS) {
			LOG_INF("Meshtastic TCP PhoneAPI client idle timeout");
			tcp_close_client();
		}
	}
}

int meshtastic_tcp_init(void)
{
	meshtastic_phoneapi_init(&tcp.api, "tcp", tcp.queue, ARRAY_SIZE(tcp.queue),
				 tcp_data_ready, tcp_disconnect, NULL, NULL);
	meshtastic_phoneapi_register(&tcp.api);

	k_thread_create(&tcp_thread_data, tcp_thread_stack,
			K_THREAD_STACK_SIZEOF(tcp_thread_stack), tcp_thread, NULL, NULL, NULL,
			CONFIG_MESHTASTIC_TCP_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tcp_thread_data, "meshtastic_tcp");

	LOG_INF("Meshtastic TCP PhoneAPI init (port %u)", TCP_PORT);
	return 0;
}
