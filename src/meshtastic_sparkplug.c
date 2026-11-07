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
 * Timestamps are UTC milliseconds when the node's clock is set (SNTP, GNSS, a phone); before
 * that, milliseconds of uptime, and `sparkplug status` says so.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_build.h"
#include "meshtastic_clock.h"
#include "meshtastic_sparkplug_session.h"

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
static char edge_id[MT_SPB_ID_MAX];
static char will_topic[MT_SPB_TOPIC_MAX];
static uint8_t will_payload[64];
static char ncmd_topic[MT_SPB_TOPIC_MAX];
static struct mqtt_topic will_t;
static struct mqtt_utf8 will_m;
static struct mt_spb_node snap[SNAP_MAX];
static bool connected;
static uint32_t sessions;
static int last_err;

/* --- bdSeq across reboots ----------------------------------------------------------------- */

static bool bdseq_known;
static uint8_t bdseq_last;

static int bdseq_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(key, "bdseq") == 0 && len == 1 && read_cb(cb_arg, &bdseq_last, 1) == 1) {
		bdseq_known = true;
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_spb, "mtspb", NULL, bdseq_set, NULL, NULL);

static uint8_t bdseq_next(void)
{
	uint8_t next = bdseq_known ? (uint8_t)(bdseq_last + 1U) : 0U;   /* 255 -> 0 */

	bdseq_last = next;
	bdseq_known = true;
	(void)settings_save_one("mtspb/bdseq", &next, 1);   /* before the CONNECT that uses it */
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

	snprintf(port, sizeof(port), "%u", (unsigned int)CONFIG_MESHTASTIC_SPARKPLUG_PORT);
	rc = zsock_getaddrinfo(CONFIG_MESHTASTIC_SPARKPLUG_BROKER, port, &hints, &res);
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
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	return mqtt_connect(&client);
}

static void serve(void)
{
	int64_t next_update = 0;
	int64_t deadline = k_uptime_get() + 10000;   /* CONNACK within 10 s */

	while (true) {
		struct zsock_pollfd fd = {.fd = client.transport.tcp.sock, .events = ZSOCK_POLLIN};
		int64_t now = k_uptime_get();
		int timeout = 1000;

		if (connected) {
			int left = mqtt_keepalive_time_left(&client);

			timeout = (int)MIN((int64_t)MIN(left, 1000), MAX(next_update - now, 0));
		} else if (now > deadline) {
			LOG_WRN("Sparkplug: no CONNACK");
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

static void spb_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint32_t backoff = 2;

	k_sleep(K_SECONDS(3));          /* the node id and settings are ready by now */
	/* The firmware loads named subtrees only (a full settings_load() runs on BLE images
	 * alone), so the last bdSeq is loaded here, before the first CONNECT uses the next. */
	(void)settings_load_subtree("mtspb");
	uint32_t own = meshtastic_get_node_id();

	if (CONFIG_MESHTASTIC_SPARKPLUG_EDGE[0] != '\0') {
		strncpy(edge_id, CONFIG_MESHTASTIC_SPARKPLUG_EDGE, sizeof(edge_id) - 1);
	} else {
		snprintf(edge_id, sizeof(edge_id), "gw-%08x", own);
	}
	snprintf(client_id, sizeof(client_id), "zt-spb-%08x", own);
	if (mt_spb_init(&sess, CONFIG_MESHTASTIC_SPARKPLUG_GROUP, edge_id, publish, NULL, now_ms,
			CONFIG_MESHTASTIC_SPARKPLUG_DEATH_SEC, meshtastic_build_id(), own) != 0) {
		LOG_ERR("Sparkplug: group or edge id not usable (no + / #, at most %d chars)",
			MT_SPB_ID_MAX - 1);
		return;
	}
	(void)mt_spb_ncmd_topic(&sess, ncmd_topic, sizeof(ncmd_topic));

	while (true) {
		int rc = resolve();

		if (rc == 0) {
			rc = connect_once();
		}
		if (rc == 0) {
			serve();
			if (connected) {
				(void)mqtt_disconnect(&client, NULL);
			} else {
				(void)mqtt_abort(&client);
			}
			connected = false;
			mt_spb_disconnected(&sess);
			backoff = 2;
		} else {
			last_err = rc;
			backoff = MIN(backoff * 2, 60U);
		}
		k_sleep(K_SECONDS(backoff));
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
static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct mt_spb_stats *st = &sess.stats;

	shell_print(sh, "broker:   %s:%d  %s (sessions %u, last error %d)",
		    CONFIG_MESHTASTIC_SPARKPLUG_BROKER, CONFIG_MESHTASTIC_SPARKPLUG_PORT,
		    connected ? "connected" : "not connected", sessions, last_err);
	shell_print(sh, "edge:     spBv1.0/%s/+/%s  bdSeq %u  seq %u", sess.group, sess.edge,
		    sess.bdseq, sess.seq);
	shell_print(sh, "clock:    %s", meshtastic_clock_valid() ? "UTC" : "not set: timestamps are uptime");
	shell_print(sh, "births:   node %u, device %u; DDATA %u; DDEATH %u; NDATA %u",
		    st->nbirths, st->dbirths, st->ddata, st->ddeaths, st->ndata);
	shell_print(sh, "rebirths: %u asked; errors: publish %u, encode %u; table full %u",
		    st->rebirth_requests, st->publish_errors, st->encode_errors, st->table_full);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(spb_cmds,
	SHELL_CMD(status, NULL, "Connection, session and counters", cmd_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(sparkplug, &spb_cmds, "The gateway's Sparkplug B Edge Node", NULL);
#endif /* CONFIG_SHELL */
