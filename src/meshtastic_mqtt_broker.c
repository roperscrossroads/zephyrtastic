/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic MQTT bridge -- the direct broker back end.
 *
 * The native MQTT client: its own thread, the socket, TLS, and the reconnect
 * backoff. Everything the bridge knows about MESHTASTIC (topics, envelopes,
 * what may be uplinked, what a downlink may inject) is in meshtastic_mqtt.c,
 * which needs no network and is shared with the client-proxy back end; this
 * file only moves the core's queued publishes to the broker and the broker's
 * deliveries back to the core. The seam is meshtastic_mqtt_internal.h.
 *
 * Split out of meshtastic_mqtt.c unchanged apart from that seam (agents-kx8d).
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include <zephyr/zvfs/eventfd.h>

#include "meshtastic_ext_ram.h"

#if defined(CONFIG_MESHTASTIC_MQTT_TLS)
#include <zephyr/net/tls_credentials.h>
#if !defined(CONFIG_MESHTASTIC_MQTT_TLS_NO_VERIFY)
#include "meshtastic_mqtt_ca_cert.h"
#endif
#endif

#include "meshtastic_mqtt_internal.h"

LOG_MODULE_DECLARE(meshtastic_mqtt, CONFIG_MESHTASTIC_LOG_LEVEL);

#define MQTT_NET_WAIT_MS               2000
/* Limit broker publishes per MQTT thread loop to avoid TCP pkt pool exhaustion. */
#define MQTT_PUBLISH_BUDGET_PER_LOOP   2
#define MQTT_PUBLISH_BUDGET_ON_CONNECT 8

static struct {
	struct mqtt_client client;
	struct sockaddr_storage broker;
	uint8_t rx_buf[CONFIG_MESHTASTIC_MQTT_RX_BUFFER_SIZE];
	uint8_t tx_buf[CONFIG_MESHTASTIC_MQTT_TX_BUFFER_SIZE];
	uint8_t client_id[16];
	struct mqtt_utf8 username;
	struct mqtt_utf8 password;
	char sub_topic[MESHTASTIC_MQTT_TOPIC_MAX];
	bool connected;
	bool thread_running;
	bool disconnect_pending;
	/* EXT_RAM_BSS_ATTR must sit AFTER the declarator (below): placed after the anonymous
	 * struct's '}' it binds to the TYPE and is silently ignored (stays in internal DRAM). */
} broker MESHTASTIC_EXT_RAM_BSS_ATTR;

/* The settings the bridge started with; the core owns them. */
static const struct meshtastic_mqtt_settings *bcfg;

static struct net_mgmt_event_callback mqtt_net_mgmt_cb;
static bool mqtt_net_has_ipv4;

static struct k_thread mqtt_thread;
static K_THREAD_STACK_DEFINE(mqtt_stack, CONFIG_MESHTASTIC_MQTT_THREAD_STACK_SIZE);

static struct zsock_pollfd mqtt_fds[1];
static int mqtt_nfds;

/* Wake the MQTT thread's blocking poll() from another thread. The thread parks
 * in zsock_poll([socket, wake_efd], keepalive_deadline); writing this eventfd
 * makes that poll return immediately — used when the publish/uplink queue gains
 * work and when a network-loss event needs prompt handling. All MQTT *client*
 * access stays on the MQTT thread; only this fd is touched cross-thread, so no
 * client lock is needed. -1 until init creates it. */
static int mqtt_wake_efd = -1;

static void mqtt_work_notify(void)
{
	if (mqtt_wake_efd >= 0) {
		(void)zvfs_eventfd_write(mqtt_wake_efd, 1);
	}
}

static void mqtt_drain_queue(int max_publish);

/* Broker-reconnect backoff (operator-specified ladder): 1 s, 8 s, 32 s, 64 s,
 * 4 min — then hold at 4 min plus random jitter up to +4 min (4-8 min), so a
 * long broker outage costs a handful of connect attempts per hour instead of
 * hundreds (each attempt against an unreachable broker burns seconds of SYN
 * retransmits — wasted power and buffers), and a fleet of nodes doesn't
 * re-thunder at a recovering broker in lockstep. The ladder resets on a
 * successful connect and on a fresh IPv4 lease (new network, new chances —
 * the reset also cuts short an in-flight backoff sleep). MQTT failures never
 * drive WiFi reconnects. */
static const uint16_t mqtt_backoff_ladder_s[] = {1U, 8U, 32U, 64U, 240U};
static uint8_t mqtt_backoff_step;
static volatile bool mqtt_backoff_skip;

static uint32_t mqtt_backoff_next_ms(void)
{
	uint32_t delay_s;

	if (mqtt_backoff_step < ARRAY_SIZE(mqtt_backoff_ladder_s) - 1U) {
		delay_s = mqtt_backoff_ladder_s[mqtt_backoff_step];
		mqtt_backoff_step++;
	} else {
		uint32_t cap_s = mqtt_backoff_ladder_s[ARRAY_SIZE(mqtt_backoff_ladder_s) - 1U];

		delay_s = cap_s + (sys_rand32_get() % cap_s);
	}

	return delay_s * MSEC_PER_SEC;
}

static void mqtt_backoff_reset(void)
{
	mqtt_backoff_step = 0U;
	mqtt_backoff_skip = true;
}

/* Chunked backoff sleep: stays responsive to thread shutdown and to a ladder
 * reset (fresh network) instead of holding the thread for minutes. */
static void mqtt_backoff_sleep_ms(uint32_t ms)
{
	mqtt_backoff_skip = false;
	while (ms > 0U && broker.thread_running && !mqtt_backoff_skip) {
		uint32_t slice = MIN(ms, 1000U);

		k_sleep(K_MSEC(slice));
		ms -= slice;
	}
}

static void mqtt_net_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				   struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		mqtt_net_has_ipv4 = true;
		mqtt_backoff_reset();
		LOG_INF("Network ready for MQTT");
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_DEL) {
		mqtt_net_has_ipv4 = false;
		broker.disconnect_pending = true;
		/* Wake the thread out of its keepalive-length poll so it acts on the
		 * pending disconnect promptly instead of up to 30 s later. */
		mqtt_work_notify();
		LOG_INF("Network lost, MQTT will disconnect");
	}
}

static bool mqtt_network_is_ready(void)
{
	struct net_if *iface = net_if_get_default();

	if (!mqtt_net_has_ipv4 || iface == NULL || !net_if_is_up(iface)) {
		return false;
	}

	return net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL;
}

static void mqtt_prepare_fds(void)
{
	if (broker.client.transport.type == MQTT_TRANSPORT_NON_SECURE) {
		mqtt_fds[0].fd = broker.client.transport.tcp.sock;
#if defined(CONFIG_MESHTASTIC_MQTT_TLS)
	} else if (broker.client.transport.type == MQTT_TRANSPORT_SECURE) {
		mqtt_fds[0].fd = broker.client.transport.tls.sock;
#endif
	} else {
		mqtt_fds[0].fd = -1;
	}

	mqtt_fds[0].events = ZSOCK_POLLIN;
	mqtt_nfds = (mqtt_fds[0].fd >= 0) ? 1 : 0;
}

static void mqtt_clear_fds(void)
{
	mqtt_nfds = 0;
}

static void mqtt_handle_disconnect_pending(void)
{
	if (!broker.disconnect_pending) {
		return;
	}

	broker.disconnect_pending = false;

	if (!broker.connected) {
		return;
	}

	LOG_DBG("Disconnecting MQTT (network lost or pending)");
	(void)mqtt_disconnect(&broker.client, NULL);
	broker.connected = false;
	mqtt_clear_fds();
}

static int mqtt_poll_socket(int timeout_ms)
{
	if (mqtt_nfds <= 0) {
		return 0;
	}

	return zsock_poll(mqtt_fds, mqtt_nfds, timeout_ms);
}

static int mqtt_do_publish(const char *topic, const uint8_t *payload, size_t len)
{
	struct mqtt_publish_param param = {0};
	struct mqtt_topic pub_topic;
	int ret;

	if (!broker.connected) {
		return -ENOTCONN;
	}

	pub_topic.topic.utf8 = (uint8_t *)topic;
	pub_topic.topic.size = strlen(topic);
	pub_topic.qos = MQTT_QOS_0_AT_MOST_ONCE;

	param.message.topic = pub_topic;
	param.message.payload.data = (uint8_t *)payload;
	param.message.payload.len = len;
	param.message_id = sys_rand16_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	ret = mqtt_publish(&broker.client, &param);
	if (ret != 0) {
		LOG_WRN("mqtt_publish failed (%d)", ret);
	} else {
		LOG_DBG("MQTT published %s (%zu bytes)", topic, len);
	}

	return ret;
}

static void mqtt_drain_queue(int max_publish)
{
	/* Static: the thread is the only caller, and an entry is a whole encoded
	 * envelope -- too big to carry on the stack beside a socket call. */
	static struct meshtastic_mqtt_pub_entry entry;
	int published = 0;

	if (max_publish <= 0) {
		return;
	}

	while (broker.connected && published < max_publish &&
	       meshtastic_mqtt_core_dequeue(&entry)) {
		LOG_DBG("Flushing queued MQTT publish: %s (%u bytes)", entry.topic, entry.len);
		(void)mqtt_do_publish(entry.topic, entry.payload, entry.len);
		published++;

		/*
		 * Sleep briefly between publishes to let the TCP/IP stack
		 * process packets and avoid TCP buffer exhaustion.
		 */
		k_msleep(20);
	}
}

static void mqtt_on_publish_evt(struct mqtt_client *client, const struct mqtt_evt *evt)
{
	uint8_t payload[CONFIG_MESHTASTIC_MQTT_RX_BUFFER_SIZE];
	int rc;
	char topic_buf[128];
	size_t topic_len;

	if (evt->param.publish.message.topic.topic.size >= sizeof(topic_buf)) {
		LOG_DBG("MQTT publish topic too long (%u bytes)",
			evt->param.publish.message.topic.topic.size);
		return;
	}

	topic_len = evt->param.publish.message.topic.topic.size;
	memcpy(topic_buf, evt->param.publish.message.topic.topic.utf8, topic_len);
	topic_buf[topic_len] = '\0';

	rc = mqtt_read_publish_payload(client, payload, sizeof(payload));
	if (rc < 0) {
		LOG_WRN("mqtt_read_publish_payload failed (%d)", rc);
		return;
	}

	meshtastic_mqtt_core_downlink(payload, (size_t)rc, topic_buf);

	if (evt->param.publish.message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
		const struct mqtt_puback_param ack = {
			.message_id = evt->param.publish.message_id,
		};

		(void)mqtt_publish_qos1_ack(client, &ack);
	}
}

static void mqtt_evt_handler(struct mqtt_client *client, const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_WRN("MQTT connect failed (%d)", evt->result);
			broker.connected = false;
			break;
		}

		broker.connected = true;
		meshtastic_mqtt_core_link_up(); /* broker back: warn afresh next episode */
		mqtt_backoff_step = 0U;       /* connected: next outage starts at 1 s */
		LOG_INF("MQTT connected to %s", bcfg->host);
		mqtt_prepare_fds();
		mqtt_drain_queue(MQTT_PUBLISH_BUDGET_ON_CONNECT);
		mqtt_work_notify();
		break;

	case MQTT_EVT_DISCONNECT:
		broker.connected = false;
		mqtt_clear_fds();
		LOG_INF("MQTT disconnected");
		break;

	case MQTT_EVT_PUBLISH:
		mqtt_on_publish_evt(client, evt);
		break;

	default:
		break;
	}
}

static int mqtt_resolve_broker(void)
{
	struct zsock_addrinfo *result = NULL;
	const struct zsock_addrinfo hints = {
		.ai_family = NET_PF_INET,
		.ai_socktype = NET_SOCK_STREAM,
	};
	char port_str[8];
	int rc;

	snprintk(port_str, sizeof(port_str), "%u", (unsigned int)bcfg->port);

	rc = zsock_getaddrinfo(bcfg->host, port_str, &hints, &result);
	if (rc != 0) {
		if (result != NULL) {
			zsock_freeaddrinfo(result);
		}
		LOG_ERR("MQTT broker resolve failed (%d %s)", rc, zsock_gai_strerror(rc));
		return -ENOENT;
	}

	if (result == NULL || result->ai_addr == NULL) {
		if (result != NULL) {
			zsock_freeaddrinfo(result);
		}
		LOG_ERR("MQTT broker resolve returned no address");
		return -ENOENT;
	}

	if (result->ai_addrlen > sizeof(broker.broker)) {
		zsock_freeaddrinfo(result);
		return -ENOMEM;
	}

	memset(&broker.broker, 0, sizeof(broker.broker));
	memcpy(&broker.broker, result->ai_addr, result->ai_addrlen);
	zsock_freeaddrinfo(result);

	LOG_DBG("MQTT broker %s:%u resolved", bcfg->host, (unsigned int)bcfg->port);
	return 0;
}

#if defined(CONFIG_MESHTASTIC_MQTT_TLS)
#if defined(CONFIG_MESHTASTIC_MQTT_TLS_NO_VERIFY)
/* Encrypted-but-unauthenticated transport: no CA trust anchor is loaded and the
 * broker certificate is NOT validated. The channel is still TLS-encrypted, but a
 * MITM cannot be detected. This mode exists because the on-device mbedTLS build
 * cannot parse the RSA-signed Let's Encrypt chain the broker currently serves;
 * flip it off (proper verify via the CA cert below) once the broker serves an
 * all-ECDSA chain anchored at ISRG Root X2. SNI is still sent so the broker
 * selects the right vhost/cert. */
static void mqtt_tls_configure(void)
{
	struct mqtt_sec_config *tls = &broker.client.transport.tls.config;

	broker.client.transport.type = MQTT_TRANSPORT_SECURE;
	tls->peer_verify = TLS_PEER_VERIFY_NONE;
	tls->cipher_list = NULL;
	tls->sec_tag_list = NULL;
	tls->sec_tag_count = 0;
	tls->hostname = bcfg->tls_hostname;
}
#else /* proper verification against the embedded CA */
static const sec_tag_t mqtt_sec_tags[] = {
	CONFIG_MESHTASTIC_MQTT_TLS_SEC_TAG,
};

/* Register the CA trust anchor once, then point the MQTT transport at it.
 * hostname drives both SNI and certificate CN/SAN verification, so it stays the
 * broker's DNS name even when BROKER_HOST is a raw IP (DNS-bypass). */
static void mqtt_tls_configure(void)
{
	static bool ca_registered;
	struct mqtt_sec_config *tls = &broker.client.transport.tls.config;

	if (!ca_registered) {
		int rc = tls_credential_add(CONFIG_MESHTASTIC_MQTT_TLS_SEC_TAG,
					    TLS_CREDENTIAL_CA_CERTIFICATE,
					    mqtt_ca_cert, sizeof(mqtt_ca_cert));
		if (rc != 0 && rc != -EEXIST) {
			LOG_ERR("Failed to register MQTT CA cert (%d)", rc);
		} else {
			ca_registered = true;
		}
	}

	broker.client.transport.type = MQTT_TRANSPORT_SECURE;
	tls->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	tls->cipher_list = NULL;
	tls->sec_tag_list = mqtt_sec_tags;
	tls->sec_tag_count = ARRAY_SIZE(mqtt_sec_tags);
	tls->hostname = bcfg->tls_hostname;
}
#endif /* CONFIG_MESHTASTIC_MQTT_TLS_NO_VERIFY */
#endif /* CONFIG_MESHTASTIC_MQTT_TLS */

static void mqtt_client_configure(void)
{
	mqtt_client_init(&broker.client);

	broker.client.broker = &broker.broker;
	broker.client.evt_cb = mqtt_evt_handler;
	broker.client.client_id.utf8 = broker.client_id;
	broker.client.client_id.size = strlen((char *)broker.client_id);
	broker.username.utf8 = (uint8_t *)bcfg->username;
	broker.username.size = strlen(bcfg->username);
	broker.password.utf8 = (uint8_t *)bcfg->password;
	broker.password.size = strlen(bcfg->password);
	/* An anonymous custom broker: the MQTT CONNECT must omit the fields, not send
	 * empty ones (a NULL pointer is how Zephyr's client expresses "absent"). */
	broker.client.user_name = (broker.username.size != 0U) ? &broker.username : NULL;
	broker.client.password = (broker.password.size != 0U) ? &broker.password : NULL;
	broker.client.protocol_version = MQTT_VERSION_3_1_1;
	broker.client.rx_buf = broker.rx_buf;
	broker.client.rx_buf_size = sizeof(broker.rx_buf);
	broker.client.tx_buf = broker.tx_buf;
	broker.client.tx_buf_size = sizeof(broker.tx_buf);
	/* Kconfig decides whether a TLS transport is COMPILED IN (the ceiling);
	 * ModuleConfig.mqtt.tls_enabled decides whether this connection USES it.
	 * A section that asks for TLS on a build without it never gets here —
	 * meshtastic_mqtt_init() refuses to start rather than downgrade. */
#if defined(CONFIG_MESHTASTIC_MQTT_TLS)
	if (bcfg->tls_enabled) {
		mqtt_tls_configure();
	} else {
		broker.client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	}
#else
	broker.client.transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
}

static int mqtt_subscribe_downlink(void)
{
	struct mqtt_topic topic = {0};
	struct mqtt_subscription_list sub = {0};
	int ret;

	ret = meshtastic_mqtt_core_subscribe_topic(broker.sub_topic, sizeof(broker.sub_topic));
	if (ret < 0 || ret >= (int)sizeof(broker.sub_topic)) {
		return -EINVAL;
	}

	topic.topic.utf8 = (uint8_t *)broker.sub_topic;
	topic.topic.size = strlen(broker.sub_topic);
	topic.qos = MQTT_QOS_0_AT_MOST_ONCE;

	sub.list = &topic;
	sub.list_count = 1U;
	sub.message_id = sys_rand16_get();

	ret = mqtt_subscribe(&broker.client, &sub);
	if (ret != 0) {
		LOG_ERR("mqtt_subscribe failed (%d)", ret);
	}

	LOG_INF("MQTT subscribed to %s", broker.sub_topic);
	return ret;
}

static int mqtt_try_connect(void)
{
	int ret;

	mqtt_client_configure();

	ret = mqtt_connect(&broker.client);
	if (ret != 0) {
		return ret;
	}

	mqtt_prepare_fds();
	if (mqtt_poll_socket(5000) > 0) {
		(void)mqtt_input(&broker.client);
	}

	if (!broker.connected) {
		mqtt_abort(&broker.client);
		return -ENOTCONN;
	}

	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_DOWNLINK_ENABLED)) {
		ret = mqtt_subscribe_downlink();
		if (ret != 0) {
			mqtt_disconnect(&broker.client, NULL);
			broker.connected = false;
			return ret;
		}
	}

	return 0;
}

/* Block until the broker socket is readable, the keepalive is due, or another
 * thread wakes us via mqtt_wake_efd (queued publish work / network loss). This
 * replaces the old fixed 200 ms poll: steady state now sleeps for up to a whole
 * keepalive interval (~CONFIG_MQTT_KEEPALIVE) instead of spinning at ~5 Hz, so
 * the CPU can idle, while socket data and publish work still wake us instantly.
 * mqtt_live() only emits a PINGREQ when one is actually due, so calling it each
 * pass is cheap. Runs only on the MQTT thread. */
static int mqtt_wait_and_process(void)
{
	struct zsock_pollfd fds[2];
	int nfds = 0;
	int sock_idx = -1;
	int efd_idx;
	int left;
	int timeout_ms;
	int ret;

	if (!broker.connected) {
		return -ENOTCONN;
	}

	if (mqtt_nfds > 0) {
		sock_idx = nfds;
		fds[nfds].fd = mqtt_fds[0].fd;
		fds[nfds].events = ZSOCK_POLLIN;
		fds[nfds].revents = 0;
		nfds++;
	}

	efd_idx = nfds;
	fds[nfds].fd = mqtt_wake_efd;
	fds[nfds].events = ZSOCK_POLLIN;
	fds[nfds].revents = 0;
	nfds++;

	/* Wake no later than the next keepalive deadline. If keepalive is disabled
	 * (-1) or the eventfd is unavailable, fall back to a bounded wait so queued
	 * work still drains promptly. */
	left = mqtt_keepalive_time_left(&broker.client);
	if (mqtt_wake_efd < 0) {
		timeout_ms = (left < 0) ? 200 : CLAMP(left, 10, 200);
	} else {
		timeout_ms = (left < 0) ? 30000 : CLAMP(left, 10, 30000);
	}

	ret = zsock_poll(fds, nfds, timeout_ms);
	if (ret < 0) {
		return (errno == EINTR) ? 0 : -errno;
	}

	/* Drain the wake eventfd if it fired (coalesces multiple writes). */
	if (fds[efd_idx].revents & ZSOCK_POLLIN) {
		zvfs_eventfd_t val;

		(void)zvfs_eventfd_read(mqtt_wake_efd, &val);
	}

	/* Incoming broker data (PUBLISH downlink, PINGRESP, PUBACK, ...). */
	if (sock_idx >= 0 && (fds[sock_idx].revents & ZSOCK_POLLIN)) {
		ret = mqtt_input(&broker.client);
		if (ret != 0) {
			return ret;
		}
	}

	/* Send a keepalive PINGREQ if one is now due. */
	ret = mqtt_live(&broker.client);
	if (ret != 0 && ret != -EAGAIN) {
		return ret;
	}

	return 0;
}

static void mqtt_thread_fn(void *p1, void *p2, void *p3)
{
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	broker.thread_running = true;

	while (broker.thread_running) {
		mqtt_handle_disconnect_pending();

		if (!broker.connected) {
			if (!mqtt_network_is_ready()) {
				k_sleep(K_MSEC(MQTT_NET_WAIT_MS));
				continue;
			}

			LOG_DBG("Connecting to MQTT broker %s:%u", bcfg->host,
				(unsigned int)bcfg->port);
			ret = mqtt_resolve_broker();
			if (ret == 0) {
				ret = mqtt_try_connect();
			}
			if (ret != 0) {
				uint32_t backoff_ms = mqtt_backoff_next_ms();

				LOG_DBG("MQTT connect failed (%d), retry in %u ms", ret,
					backoff_ms);
				mqtt_backoff_sleep_ms(backoff_ms);
				continue;
			}
		}

		if (!mqtt_network_is_ready()) {
			broker.disconnect_pending = true;
			mqtt_handle_disconnect_pending();
			k_sleep(K_MSEC(MQTT_NET_WAIT_MS));
			continue;
		}

		mqtt_drain_queue(MQTT_PUBLISH_BUDGET_PER_LOOP);

		/* Blocks here (up to a keepalive interval) until real work arrives —
		 * broker data, a queued publish (via mqtt_wake_efd), or the keepalive
		 * deadline. No more fixed-interval polling. */
		ret = mqtt_wait_and_process();
		if (ret != 0) {
			/* The session had connected, so the ladder was reset: the first
			 * reconnect attempt after a session error comes quickly (1 s)
			 * and only escalates if the broker stays gone. */
			uint32_t backoff_ms = mqtt_backoff_next_ms();

			LOG_DBG("MQTT session error (%d), reconnecting in %u ms", ret, backoff_ms);
			mqtt_disconnect(&broker.client, NULL);
			broker.connected = false;
			mqtt_clear_fds();
			mqtt_backoff_sleep_ms(backoff_ms);
		} else {
			mqtt_drain_queue(MQTT_PUBLISH_BUDGET_PER_LOOP);
			meshtastic_mqtt_core_perhaps_report_to_map();

			/* If a burst exceeded one publish budget, self-wake so the
			 * leftover drains on the next pass instead of waiting out a
			 * whole keepalive interval. */
			if (meshtastic_mqtt_core_pending()) {
				mqtt_work_notify();
			}
		}
	}
}

void meshtastic_mqtt_broker_notify(void)
{
	mqtt_work_notify();
}

bool meshtastic_mqtt_broker_connected(void)
{
	return broker.connected;
}

int meshtastic_mqtt_broker_start(void)
{
	int ret;

	bcfg = meshtastic_mqtt_core_settings();

	/* Wake eventfd for the thread's blocking poll (queued work / net loss).
	 * Non-fatal if unavailable — mqtt_wait_and_process() then falls back to a
	 * bounded 200 ms wait, i.e. the old behaviour. */
	mqtt_wake_efd = zvfs_eventfd(0, ZVFS_EFD_NONBLOCK);
	if (mqtt_wake_efd < 0) {
		LOG_WRN("MQTT wake eventfd unavailable (%d); using bounded poll", mqtt_wake_efd);
		mqtt_wake_efd = -1;
	}

	net_mgmt_init_event_callback(&mqtt_net_mgmt_cb, mqtt_net_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&mqtt_net_mgmt_cb);
	mqtt_net_has_ipv4 = mqtt_network_is_ready();

	meshtastic_mqtt_core_node_id_str((char *)broker.client_id, sizeof(broker.client_id));

	ret = meshtastic_mqtt_core_subscribe_topic(broker.sub_topic, sizeof(broker.sub_topic));
	if (ret < 0) {
		LOG_ERR("MQTT topic setup failed (%d)", ret);
		return ret;
	}

	k_thread_create(&mqtt_thread, mqtt_stack, K_THREAD_STACK_SIZEOF(mqtt_stack), mqtt_thread_fn,
			NULL, NULL, NULL, CONFIG_MESHTASTIC_MQTT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mqtt_thread, "meshtastic_mqtt");

	return 0;
}
