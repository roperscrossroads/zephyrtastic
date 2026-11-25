/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * The mesh gateway as a Sparkplug B Edge Node: the MQTT side of meshtastic_sparkplug_session.c.
 *
 * One thread, its own MQTT connection. Per connection: the next bdSeq (kept in settings, so a
 * reboot never repeats the last session's), the NDEATH Will, CONNECT; on the CONNACK the NCMD
 * subscription and the births; then, every MESHTASTIC_SPARKPLUG_INTERVAL_SEC, a snapshot of the
 * NodeDB (the most recently heard nodes inside the death window, at most MAX_DEVICES) handed to
 * the session, which sends what changed. A dropped connection is retried with a backoff and a
 * new bdSeq. Telemetry only: an NCMD rebirth request is answered, nothing else is acted on.
 *
 * Broker, port, group, edge id and the MQTT login are settings (`sparkplug set|unset`, subtree
 * mtspb), the Kconfig values their defaults; a change ends the session cleanly and reconnects
 * with the new values. The username is meant to be the edge id (one login per Edge Node, the
 * bench broker's ACL). The subtree is sealed when lockdown is provisioned, the password with it
 * (agents-ls39.1.5); without lockdown it is stored in the clear, like the hawkBit token.
 *
 * A planned reboot (admin, lockdown, `intents clear`) or `sparkplug stop` ends the session on
 * purpose first: the NDEATH (QoS 1, its PUBACK awaited briefly), then MQTT DISCONNECT, then no
 * reconnect until `sparkplug start` (meshtastic_sparkplug_stop() / _start()).
 *
 * With MESHTASTIC_SPARKPLUG_TLS every connection is TLS, the broker's certificate checked against
 * the root in meshtastic_sparkplug_ca.h (ISRG Root X2) and its name against the broker setting (also the SNI);
 * no plaintext fallback, no way to skip the check.
 *
 * Timestamps are UTC milliseconds when the node's clock is set (SNTP, GNSS, a phone); before
 * that, milliseconds of uptime, and `sparkplug status` says so.
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/random/random.h>
#include <zephyr/settings/settings.h>

#include "meshtastic_lockdown.h"
#include <zephyr/shell/shell.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/sparkplug.h>

#include "meshtastic_build.h"
#include "meshtastic_clock.h"
#include "meshtastic_sparkplug_session.h"

#if defined(CONFIG_MESHTASTIC_SPARKPLUG_TLS)
#include <zephyr/net/tls_credentials.h>
#include "meshtastic_sparkplug_ca.h"

static const sec_tag_t spb_sec_tags[] = {CONFIG_MESHTASTIC_SPARKPLUG_TLS_SEC_TAG};
#endif

LOG_MODULE_REGISTER(mt_sparkplug, CONFIG_MESHTASTIC_LOG_LEVEL);

#define SNAP_MAX CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES

static K_THREAD_STACK_DEFINE(spb_stack, CONFIG_MESHTASTIC_SPARKPLUG_STACK_SIZE);
static struct k_thread spb_thread;

static struct mt_spb_session sess;
static struct mqtt_client client;
static struct net_sockaddr_storage broker_addr;
static uint8_t rx_buf[512];
static uint8_t tx_buf[CONFIG_MESHTASTIC_SPARKPLUG_BUF_SIZE + MT_SPB_TOPIC_MAX + 16];
static uint8_t in_buf[256];
static char client_id[48];
static char will_topic[MT_SPB_TOPIC_MAX];
static uint8_t will_payload[64];
static char ncmd_topic[MT_SPB_TOPIC_MAX];
static struct mqtt_topic will_t;
static struct mqtt_utf8 will_m;
static struct mt_spb_node snap[SNAP_MAX];
static bool connected;
static uint32_t sessions;
static int last_err;

/* Stopping on purpose: the request, the thread's answer, and its wake-up from a backoff. */
static atomic_t stop_req;
static bool stopped;
static K_SEM_DEFINE(stopped_sem, 0, 1);
static K_SEM_DEFINE(wake_sem, 0, 1);
#define NDEATH_MSG_ID 0xDEADU
static bool ndeath_acked;
static uint32_t intentional_deaths;

/* --- settings: bdSeq across reboots, and where and as whom to connect ---------------------- */

#define SPB_HOST_MAX 64
#define SPB_CRED_MAX 64

struct spb_cfg {
	char broker[SPB_HOST_MAX];
	uint16_t port;
	char group[MT_SPB_ID_MAX];
	char edge[MT_SPB_ID_MAX];       /* empty: gw-<node id> */
	char user[SPB_CRED_MAX];        /* empty: no login */
	char pass[SPB_CRED_MAX];
};

static K_MUTEX_DEFINE(cfg_lock);
static struct spb_cfg cfg;              /* what the settings say (under cfg_lock) */
static struct spb_cfg live;             /* what this session uses (the thread's copy) */
static struct mqtt_utf8 user_m, pass_m;
static atomic_t restart_req;
static bool bdseq_known;
static uint8_t bdseq_last;

static void cfg_defaults(struct spb_cfg *c)
{
	memset(c, 0, sizeof(*c));
	strncpy(c->broker, CONFIG_MESHTASTIC_SPARKPLUG_BROKER, sizeof(c->broker) - 1);
	c->port = (uint16_t)CONFIG_MESHTASTIC_SPARKPLUG_PORT;
	strncpy(c->group, CONFIG_MESHTASTIC_SPARKPLUG_GROUP, sizeof(c->group) - 1);
	strncpy(c->edge, CONFIG_MESHTASTIC_SPARKPLUG_EDGE, sizeof(c->edge) - 1);
}

/* A setting's text value, NUL-terminated; a value that does not fit is ignored. */
static void set_str(char *dst, size_t cap, const uint8_t *val, size_t n)
{
	if (n == 0 || n >= cap) {
		return;
	}
	memcpy(dst, val, n);
	dst[n] = '\0';
}

/* The subtree is sealed by lockdown (agents-ls39.1.5): read through it, which hands
 * back the plaintext of a sealed record and passes a clear one (any record written
 * before lockdown was provisioned) through. On a locked boot it refuses, and the
 * build's defaults stand until the unlock. */
static int spb_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	char name[24];
	uint8_t val[SPB_HOST_MAX];
	ssize_t n;

	(void)snprintk(name, sizeof(name), "mtspb/%s", key);
	n = meshtastic_lockdown_read(name, len, read_cb, cb_arg, val, sizeof(val));
	if (n < 0) {
		return 0;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	if (strcmp(key, "bdseq") == 0 && n == 1) {
		bdseq_last = val[0];
		bdseq_known = true;
	} else if (strcmp(key, "broker") == 0) {
		set_str(cfg.broker, sizeof(cfg.broker), val, (size_t)n);
	} else if (strcmp(key, "port") == 0 && n == sizeof(cfg.port)) {
		memcpy(&cfg.port, val, sizeof(cfg.port));
	} else if (strcmp(key, "group") == 0) {
		set_str(cfg.group, sizeof(cfg.group), val, (size_t)n);
	} else if (strcmp(key, "edge") == 0) {
		set_str(cfg.edge, sizeof(cfg.edge), val, (size_t)n);
	} else if (strcmp(key, "user") == 0) {
		set_str(cfg.user, sizeof(cfg.user), val, (size_t)n);
	} else if (strcmp(key, "pass") == 0) {
		set_str(cfg.pass, sizeof(cfg.pass), val, (size_t)n);
	}
	k_mutex_unlock(&cfg_lock);
	memset(val, 0, sizeof(val)); /* the password passed through here */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_spb, "mtspb", NULL, spb_set, NULL, NULL);

static uint8_t bdseq_next(void)
{
	uint8_t next = bdseq_known ? (uint8_t)(bdseq_last + 1U) : 0U;   /* 255 -> 0 */

	bdseq_last = next;
	bdseq_known = true;
	(void)meshtastic_lockdown_save_one("mtspb/bdseq", &next, 1); /* before the CONNECT */
	return next;
}

/* --- the session's hooks ------------------------------------------------------------------- */

static int64_t now_ms(void)
{
	return meshtastic_clock_valid() ? meshtastic_clock_now_epoch_ms() : k_uptime_get();
}

static int publish(void *ctx, const char *topic, const uint8_t *payload, size_t len, int qos,
		   bool retain)
{
	ARG_UNUSED(ctx);
	if (!connected) {
		return -ENOTCONN;
	}
	struct mqtt_publish_param p = {0};

	p.message.topic.topic.utf8 = (const uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = (uint8_t)qos;
	p.message.payload.data = (uint8_t *)payload;
	p.message.payload.len = len;
	p.message_id = sys_rand16_get();
	p.retain_flag = retain ? 1U : 0U;
	return mqtt_publish(&client, &p);
}

/* --- NodeDB snapshot ----------------------------------------------------------------------- */

static size_t snapshot(uint32_t now_s)
{
	struct meshtastic_nodedb_node nd;
	size_t n = 0;
	size_t count = meshtastic_nodedb_count();
	uint32_t own = meshtastic_get_node_id();

	for (size_t i = 0; i < count; i++) {
		if (meshtastic_nodedb_get_by_index(i, &nd) != 0 || nd.num == own ||
		    nd.last_heard_uptime_sec == 0U ||
		    now_s - nd.last_heard_uptime_sec > CONFIG_MESHTASTIC_SPARKPLUG_DEATH_SEC) {
			continue;
		}
		size_t slot = n;

		if (n == SNAP_MAX) {            /* full: replace the least recently heard */
			slot = 0;
			for (size_t k = 1; k < n; k++) {
				if (snap[k].last_heard_uptime_s < snap[slot].last_heard_uptime_s) {
					slot = k;
				}
			}
			if (snap[slot].last_heard_uptime_s >= nd.last_heard_uptime_sec) {
				continue;
			}
		} else {
			n++;
		}
		struct mt_spb_node *s = &snap[slot];

		memset(s, 0, sizeof(*s));
		s->num = nd.num;
		s->last_heard_uptime_s = nd.last_heard_uptime_sec;
		s->last_heard_epoch = nd.last_heard_epoch;
		s->snr = nd.snr;
		s->has_hops = nd.has_hops_away;
		s->hops_away = nd.hops_away;
		s->channel = nd.channel;
		s->via_mqtt = nd.via_mqtt;
		strncpy(s->long_name, nd.long_name, sizeof(s->long_name) - 1);
		strncpy(s->short_name, nd.short_name, sizeof(s->short_name) - 1);
		s->hw_model = nd.hw_model;
		s->role = nd.role;
		s->has_position = nd.has_position;
		s->latitude_i = nd.position_latitude_i;
		s->longitude_i = nd.position_longitude_i;
	}
	return n;
}

/* --- MQTT ------------------------------------------------------------------------------------- */

static void subscribe_ncmd(void)
{
	struct mqtt_topic t = {
		.topic = {.utf8 = (const uint8_t *)ncmd_topic, .size = strlen(ncmd_topic)},
		.qos = MQTT_QOS_1_AT_LEAST_ONCE,
	};
	struct mqtt_subscription_list l = {.list = &t, .list_count = 1, .message_id = 1};

	(void)mqtt_subscribe(&client, &l);
}

static void on_publish(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	const struct mqtt_publish_param *p = &evt->param.publish;
	size_t len = p->message.payload.len;
	bool ours = p->message.topic.topic.size == strlen(ncmd_topic) &&
		    memcmp(p->message.topic.topic.utf8, ncmd_topic, strlen(ncmd_topic)) == 0;

	if (len > sizeof(in_buf)) {
		/* read and drop what does not fit; a rebirth request is ~40 bytes */
		uint8_t sink[32];

		while (len > 0) {
			size_t k = MIN(len, sizeof(sink));

			if (mqtt_read_publish_payload_blocking(c, sink, k) < 0) {
				break;
			}
			len -= k;
		}
		ours = false;
	} else if (mqtt_readall_publish_payload(c, in_buf, len) != 0) {
		return;
	}
	if (p->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
		struct mqtt_puback_param ack = {.message_id = p->message_id};

		(void)mqtt_publish_qos1_ack(c, &ack);
	}
	if (ours) {
		int rc = mt_spb_ncmd(&sess, in_buf, p->message.payload.len);

		LOG_INF("Sparkplug NCMD: %s", rc == 1 ? "rebirth" : rc == 0 ? "nothing acted on"
							       : "not decoded");
	}
}

static void evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_WRN("Sparkplug broker refused the connection (%d)", evt->result);
			last_err = evt->result;
			break;
		}
		connected = true;
		sessions++;
		subscribe_ncmd();
		mt_spb_connected(&sess);
		LOG_INF("Sparkplug: connected, bdSeq %u", sess.bdseq);
		break;
	case MQTT_EVT_DISCONNECT:
		connected = false;
		mt_spb_disconnected(&sess);
		break;
	case MQTT_EVT_PUBLISH:
		on_publish(c, evt);
		break;
	case MQTT_EVT_PUBACK:
		if (evt->param.puback.message_id == NDEATH_MSG_ID) {
			ndeath_acked = true;
		}
		break;
	default:
		break;
	}
}

static int resolve(void)
{
	struct zsock_addrinfo *res = NULL;
	const struct zsock_addrinfo hints = {.ai_family = NET_AF_INET,
					     .ai_socktype = NET_SOCK_STREAM};
	char port[8];
	int rc;

	snprintf(port, sizeof(port), "%u", (unsigned int)live.port);
	rc = zsock_getaddrinfo(live.broker, port, &hints, &res);
	if (rc != 0 || res == NULL || res->ai_addrlen > sizeof(broker_addr)) {
		if (res != NULL) {
			zsock_freeaddrinfo(res);
		}
		return -ENOENT;
	}
	memcpy(&broker_addr, res->ai_addr, res->ai_addrlen);
	zsock_freeaddrinfo(res);
	return 0;
}

/* The connection's socket, for poll(): the transport keeps it in its TLS or TCP half. */
static int sock_fd(void)
{
#if defined(CONFIG_MESHTASTIC_SPARKPLUG_TLS)
	return client.transport.tls.sock;
#else
	return client.transport.tcp.sock;
#endif
}

#if defined(CONFIG_MESHTASTIC_SPARKPLUG_TLS)
/* The trust anchors are registered once; each connection then asks for the peer to be verified
 * against them, under the broker's name. */
static int tls_setup(void)
{
	static bool ca_added;
	struct mqtt_sec_config *t = &client.transport.tls.config;

	if (!ca_added) {
		int rc = tls_credential_add(CONFIG_MESHTASTIC_SPARKPLUG_TLS_SEC_TAG,
					    TLS_CREDENTIAL_CA_CERTIFICATE, spb_ca_pem,
					    sizeof(spb_ca_pem));

		if (rc < 0 && rc != -EEXIST) {
			LOG_ERR("Sparkplug: trust anchors not registered (%d)", rc);
			return rc;
		}
		ca_added = true;
	}
	t->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	t->cipher_list = NULL;
	t->sec_tag_list = spb_sec_tags;
	t->sec_tag_count = ARRAY_SIZE(spb_sec_tags);
	t->hostname = live.broker;
	client.transport.type = MQTT_TRANSPORT_SECURE;
	return 0;
}
#endif

static int connect_once(void)
{
	size_t wlen = sizeof(will_payload);
	int rc;

	mt_spb_session_start(&sess, bdseq_next());
	rc = mt_spb_will(&sess, will_topic, sizeof(will_topic), will_payload, &wlen);
	if (rc != 0) {
		return rc;
	}
	mqtt_client_init(&client);
	client.broker = &broker_addr;
	client.evt_cb = evt_handler;
	client.client_id.utf8 = (const uint8_t *)client_id;
	client.client_id.size = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.clean_session = 1U;
	client.user_name = NULL;
	client.password = NULL;
	if (live.user[0] != '\0') {
		user_m.utf8 = (const uint8_t *)live.user;
		user_m.size = strlen(live.user);
		client.user_name = &user_m;
		if (live.pass[0] != '\0') {
			pass_m.utf8 = (const uint8_t *)live.pass;
			pass_m.size = strlen(live.pass);
			client.password = &pass_m;
		}
	}
	will_t.topic.utf8 = (const uint8_t *)will_topic;
	will_t.topic.size = strlen(will_topic);
	will_t.qos = MQTT_QOS_1_AT_LEAST_ONCE;           /* spec 5.4: QoS 1, not retained */
	will_m.utf8 = will_payload;
	will_m.size = wlen;
	client.will_topic = &will_t;
	client.will_message = &will_m;
	client.will_retain = 0U;
	client.rx_buf = rx_buf;
	client.rx_buf_size = sizeof(rx_buf);
	client.tx_buf = tx_buf;
	client.tx_buf_size = sizeof(tx_buf);
#if defined(CONFIG_MESHTASTIC_SPARKPLUG_TLS)
	rc = tls_setup();
	if (rc != 0) {
		return rc;
	}
#else
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
	return mqtt_connect(&client);
}

static void serve(void)
{
	int64_t next_update = 0;
	int64_t deadline = k_uptime_get() + 10000;   /* CONNACK within 10 s */

	while (true) {
		struct zsock_pollfd fd = {.fd = sock_fd(), .events = ZSOCK_POLLIN};
		int64_t now = k_uptime_get();
		int timeout = 1000;

		if (connected) {
			int left = mqtt_keepalive_time_left(&client);

			timeout = (int)MIN((int64_t)MIN(left, 1000), MAX(next_update - now, 0));
		} else if (now > deadline) {
			LOG_WRN("Sparkplug: no CONNACK");
			return;
		}
		if (atomic_get(&stop_req) || atomic_get(&restart_req)) {
			return;
		}
		int rc = zsock_poll(&fd, 1, timeout);

		if (rc < 0 || (rc > 0 && (fd.revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP)))) {
			return;
		}
		if (rc > 0 && (fd.revents & ZSOCK_POLLIN)) {
			if (mqtt_input(&client) != 0) {
				return;
			}
		}
		if (!connected && sessions > 0 && now > deadline) {
			return;
		}
		if (connected) {
			if (mqtt_live(&client) < 0 && mqtt_keepalive_time_left(&client) == 0) {
				return;
			}
			now = k_uptime_get();
			if (now >= next_update) {
				uint32_t now_s = (uint32_t)(now / 1000);

				mt_spb_update(&sess, snap, snapshot(now_s), now_s);
				next_update = now + CONFIG_MESHTASTIC_SPARKPLUG_INTERVAL_SEC * 1000LL;
			}
		}
	}
}

/* The NDEATH an intentional disconnect owes (the same message as the Will), at QoS 1 with its
 * PUBACK awaited up to 500 ms, so it has reached the broker before the DISCONNECT. */
static void say_ndeath(void)
{
	struct mqtt_publish_param p = {0};
	int64_t end;

	p.message.topic.topic.utf8 = (const uint8_t *)will_topic;
	p.message.topic.topic.size = strlen(will_topic);
	p.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
	p.message.payload.data = will_payload;
	p.message.payload.len = will_m.size;
	p.message_id = NDEATH_MSG_ID;
	ndeath_acked = false;
	if (mqtt_publish(&client, &p) != 0) {
		return;
	}
	intentional_deaths++;
	end = k_uptime_get() + 500;
	while (!ndeath_acked && connected && k_uptime_get() < end) {
		struct zsock_pollfd fd = {.fd = sock_fd(), .events = ZSOCK_POLLIN};

		if (zsock_poll(&fd, 1, 50) > 0 && mqtt_input(&client) != 0) {
			break;
		}
	}
}

int meshtastic_sparkplug_stop(k_timeout_t timeout)
{
	if (k_current_get() == &spb_thread) {
		return -EDEADLK;
	}
	k_sem_reset(&stopped_sem);
	atomic_set(&stop_req, 1);
	k_sem_give(&wake_sem);          /* out of a backoff at once */
	if (stopped) {
		return 0;
	}
	return k_sem_take(&stopped_sem, timeout);
}

void meshtastic_sparkplug_start(void)
{
	atomic_clear(&stop_req);
	k_sem_give(&wake_sem);
}

/* The settings, for this session: a new group or edge id starts a new Edge Node session. */
static int take_config(uint32_t own)
{
	struct spb_cfg next;

	k_mutex_lock(&cfg_lock, K_FOREVER);
	next = cfg;
	k_mutex_unlock(&cfg_lock);
	if (next.edge[0] == '\0') {
		snprintf(next.edge, sizeof(next.edge), "gw-%08x", own);
	}
	if (strcmp(next.group, sess.group) != 0 || strcmp(next.edge, sess.edge) != 0) {
		if (mt_spb_init(&sess, next.group, next.edge, publish, NULL, now_ms,
				CONFIG_MESHTASTIC_SPARKPLUG_DEATH_SEC, meshtastic_build_id(), own) != 0) {
			LOG_ERR("Sparkplug: group %s / edge %s not usable", next.group, next.edge);
			return -EINVAL;
		}
		(void)mt_spb_ncmd_topic(&sess, ncmd_topic, sizeof(ncmd_topic));
	}
	live = next;
	return 0;
}

static void spb_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint32_t backoff = 2;

	k_sleep(K_SECONDS(3));          /* the node id and settings are ready by now */
	/* The firmware loads named subtrees only (a full settings_load() runs on BLE images
	 * alone), so the last bdSeq is loaded here, before the first CONNECT uses the next. */
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg_defaults(&cfg);
	k_mutex_unlock(&cfg_lock);
	(void)settings_load_subtree("mtspb");
	uint32_t own = meshtastic_get_node_id();

	snprintf(client_id, sizeof(client_id), "zt-spb-%08x", own);

	while (true) {
		if (atomic_get(&stop_req)) {
			stopped = true;
			k_sem_give(&stopped_sem);
			LOG_INF("Sparkplug: stopped");
			while (atomic_get(&stop_req)) {
				(void)k_sem_take(&wake_sem, K_FOREVER);
			}
			stopped = false;
			LOG_INF("Sparkplug: started");
		}
		atomic_clear(&restart_req);
		int rc = take_config(own);

		if (rc == 0) {
			rc = resolve();
		}
		if (rc == 0) {
			rc = connect_once();
		}
		if (rc == 0) {
			uint32_t before = sessions;

			serve();
			if (connected) {
				if (atomic_get(&stop_req) || atomic_get(&restart_req)) {
					say_ndeath();
				}
				(void)mqtt_disconnect(&client, NULL);
			} else {
				(void)mqtt_abort(&client);
			}
			connected = false;
			mt_spb_disconnected(&sess);
			/* refused (a login the broker does not take) or no CONNACK: back off as for a
			 * failed connect, not every 2 s for ever */
			backoff = sessions != before ? 2U : MIN(backoff * 2, 60U);
		} else {
			last_err = rc;
			backoff = MIN(backoff * 2, 60U);
		}
		if (atomic_get(&restart_req)) {
			continue;               /* new settings: reconnect at once */
		}
		if (!atomic_get(&stop_req)) {
			(void)k_sem_take(&wake_sem, K_SECONDS(backoff));
		}
	}
}

static int meshtastic_sparkplug_init(void)
{
	k_thread_create(&spb_thread, spb_stack, K_THREAD_STACK_SIZEOF(spb_stack), spb_thread_fn,
			NULL, NULL, NULL, CONFIG_MESHTASTIC_SPARKPLUG_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&spb_thread, "mt_sparkplug");
	return 0;
}

SYS_INIT(meshtastic_sparkplug_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if defined(CONFIG_SHELL)
#if defined(CONFIG_COMMON_LIBC_MALLOC) && defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
/* lib/libc/common/include/sys_malloc.h is not on an application's include path. */
int malloc_runtime_stats_get(struct sys_memory_stats *stats);
#endif

/* What the TLS session costs: mbedTLS takes its record buffers and handshake state from the C
 * library's malloc arena (MBEDTLS_ENABLE_HEAP is off), the handshake runs on this module's
 * thread. The peaks are since boot, so they include the first handshake. */
static void print_memory(const struct shell *sh)
{
#if defined(CONFIG_COMMON_LIBC_MALLOC) && defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
	struct sys_memory_stats m;

	if (malloc_runtime_stats_get(&m) == 0) {
		shell_print(sh, "malloc:   %zu B in use, peak %zu B, %zu B free", m.allocated_bytes,
			    m.max_allocated_bytes, m.free_bytes);
	}
#endif
#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_STACK_INFO)
	size_t unused;

	if (k_thread_stack_space_get(&spb_thread, &unused) == 0) {
		size_t size = K_THREAD_STACK_SIZEOF(spb_stack);

		shell_print(sh, "stack:    peak %zu of %zu B", size - unused, size);
	}
#endif
	ARG_UNUSED(sh);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct mt_spb_stats *st = &sess.stats;

	struct spb_cfg c;

	k_mutex_lock(&cfg_lock, K_FOREVER);
	c = cfg;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "broker:   %s:%u %s  %s (sessions %u, intentional NDEATH %u, last error %d)",
		    c.broker, c.port,
		    IS_ENABLED(CONFIG_MESHTASTIC_SPARKPLUG_TLS) ? "TLS (verified)" : "plain",
		    stopped ? "stopped" : connected ? "connected" : "not connected", sessions,
		    intentional_deaths, last_err);
	shell_print(sh, "login:    %s%s", c.user[0] ? c.user : "none",
		    c.user[0] ? (c.pass[0] ? " (password set)" : " (no password)") : "");
	shell_print(sh, "edge:     spBv1.0/%s/+/%s  bdSeq %u  seq %u", sess.group, sess.edge,
		    sess.bdseq, sess.seq);
	shell_print(sh, "clock:    %s", meshtastic_clock_valid() ? "UTC" : "not set: timestamps are uptime");
	shell_print(sh, "births:   node %u, device %u; DDATA %u; DDEATH %u; NDATA %u",
		    st->nbirths, st->dbirths, st->ddata, st->ddeaths, st->ndata);
	shell_print(sh, "rebirths: %u asked; errors: publish %u, encode %u; table full %u",
		    st->rebirth_requests, st->publish_errors, st->encode_errors, st->table_full);
	print_memory(sh);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	int rc = meshtastic_sparkplug_stop(MESHTASTIC_SPARKPLUG_STOP_TIMEOUT);

	shell_print(sh, rc == 0 ? "stopped (NDEATH, DISCONNECT); `sparkplug start` resumes"
				: "not stopped yet (%d)", rc);
	return rc;
}

/* sparkplug set|unset <key> [value]: keys broker port group edge user password */
static const char *const cfg_keys[] = {"broker", "port", "group", "edge", "user", "password"};

static int cfg_key(const struct shell *sh, const char *k)
{
	for (size_t i = 0; i < ARRAY_SIZE(cfg_keys); i++) {
		if (strcmp(k, cfg_keys[i]) == 0) {
			return (int)i;
		}
	}
	shell_error(sh, "keys: broker port group edge user password");
	return -1;
}

static int cfg_apply(const struct shell *sh, int key, const char *value)
{
	struct spb_cfg def;
	char *dst;
	size_t cap;
	const char *skey;
	int rc;

	cfg_defaults(&def);
	k_mutex_lock(&cfg_lock, K_FOREVER);
	switch (key) {
	case 0: dst = cfg.broker; cap = sizeof(cfg.broker); skey = "mtspb/broker"; break;
	case 2: dst = cfg.group; cap = sizeof(cfg.group); skey = "mtspb/group"; break;
	case 3: dst = cfg.edge; cap = sizeof(cfg.edge); skey = "mtspb/edge"; break;
	case 4: dst = cfg.user; cap = sizeof(cfg.user); skey = "mtspb/user"; break;
	case 5: dst = cfg.pass; cap = sizeof(cfg.pass); skey = "mtspb/pass"; break;
	default: dst = NULL; cap = 0; skey = "mtspb/port"; break;
	}
	if (key == 1) {
		unsigned long port = value ? strtoul(value, NULL, 10) : def.port;

		if (port == 0 || port > 65535) {
			k_mutex_unlock(&cfg_lock);
			shell_error(sh, "port: 1-65535");
			return -EINVAL;
		}
		cfg.port = (uint16_t)port;
		rc = value ? meshtastic_lockdown_save_one(skey, &cfg.port, sizeof(cfg.port))
			   : settings_delete(skey);
	} else {
		const char *v = value;

		if (v == NULL) {        /* unset: back to the build's default */
			v = key == 0 ? def.broker : key == 2 ? def.group : key == 3 ? def.edge : "";
		}
		if (strlen(v) >= cap || ((key == 2 || (key == 3 && v[0])) && !mt_spb_valid_id(v))) {
			k_mutex_unlock(&cfg_lock);
			shell_error(sh, "%s: not usable (at most %u chars%s)", cfg_keys[key],
				    (unsigned int)cap - 1U, key == 2 || key == 3 ? ", no + / #" : "");
			return -EINVAL;
		}
		strncpy(dst, v, cap - 1);
		dst[cap - 1] = '\0';
		rc = value ? meshtastic_lockdown_save_one(skey, dst, strlen(dst)) : settings_delete(skey);
	}
	k_mutex_unlock(&cfg_lock);
	if (rc != 0) {
		shell_error(sh, "saving %s: %d", skey, rc);
	}
	atomic_set(&restart_req, 1);    /* NDEATH, DISCONNECT, reconnect with it */
	k_sem_give(&wake_sem);
	shell_print(sh, "%s %s; reconnecting", cfg_keys[key], value ? "set" : "back to the default");
	return rc;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	int key = cfg_key(sh, argv[1]);

	return key < 0 ? -EINVAL : cfg_apply(sh, key, argv[2]);
}

static int cmd_unset(const struct shell *sh, size_t argc, char **argv)
{
	int key = cfg_key(sh, argv[1]);

	return key < 0 ? -EINVAL : cfg_apply(sh, key, NULL);
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	meshtastic_sparkplug_start();
	shell_print(sh, "started: a new session (next bdSeq)");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(spb_cmds,
	SHELL_CMD(status, NULL, "Connection, session and counters", cmd_status),
	SHELL_CMD(stop, NULL, "End the session on purpose: NDEATH, DISCONNECT, no reconnect", cmd_stop),
	SHELL_CMD(start, NULL, "Resume after `sparkplug stop`", cmd_start),
	SHELL_CMD_ARG(set, NULL, "set <broker|port|group|edge|user|password> <value>: saved, "
		      "reconnects", cmd_set, 3, 0),
	SHELL_CMD_ARG(unset, NULL, "unset <key>: back to the build's default, reconnects",
		      cmd_unset, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(sparkplug, &spb_cmds, "The gateway's Sparkplug B Edge Node", NULL);
#endif /* CONFIG_SHELL */
