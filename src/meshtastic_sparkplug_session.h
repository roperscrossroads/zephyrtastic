/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * A Sparkplug B 3.0 Edge Node session for a mesh gateway: the nodes it hears are its Devices.
 * No network here: the caller owns the MQTT connection and hands in a publish function, the
 * time, and snapshots of the nodes heard. meshtastic_sparkplug.c is that caller.
 *
 * Per MQTT connection: mt_spb_session_start() with this connection's bdSeq (the caller keeps
 * it across reboots), mt_spb_will() for CONNECT's Will, mt_spb_connected() once the CONNACK is
 * good (NBIRTH, then a DBIRTH per live device); then mt_spb_update() with each snapshot and
 * mt_spb_ncmd() with each NCMD; mt_spb_disconnected() when the connection drops.
 *
 * The rules kept (spec chapter 5): the Will is NDEATH, QoS 1, carrying only bdSeq (Int64);
 * NBIRTH carries the same bdSeq, `Node Control/Rebirth` (false) and the node metrics, seq 0;
 * one seq per Edge Node, +1 per message but NDEATH, wrapping at 255; DBIRTH with every device
 * metric (datatypes in births only); DDATA with what changed; DDEATH when a node has not been
 * heard for `death_after_s` or leaves the NodeDB; a Rebirth NCMD re-sends the births.
 */

#ifndef MESHTASTIC_SPARKPLUG_SESSION_H_
#define MESHTASTIC_SPARKPLUG_SESSION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES
#define CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES 32
#endif
#ifndef CONFIG_MESHTASTIC_SPARKPLUG_BUF_SIZE
#define CONFIG_MESHTASTIC_SPARKPLUG_BUF_SIZE 1024
#endif

#define MT_SPB_ID_MAX 32
#define MT_SPB_TOPIC_MAX 112

/* Sparkplug datatypes used here (spec 6.4.16) */
#define MT_SPB_INT64   4
#define MT_SPB_DOUBLE  10
#define MT_SPB_BOOLEAN 11
#define MT_SPB_STRING  12

/* One heard node, as the gateway reports it. */
struct mt_spb_node {
	uint32_t num;
	uint32_t last_heard_uptime_s;  /* 0: not heard since boot (not reported) */
	uint32_t last_heard_epoch;     /* UTC seconds; 0 unknown */
	float snr;
	bool has_hops;
	uint8_t hops_away;
	uint8_t channel;
	bool via_mqtt;
	char long_name[25];
	char short_name[5];
	uint8_t hw_model;
	uint8_t role;
	bool has_position;
	int32_t latitude_i;            /* 1e-7 degrees */
	int32_t longitude_i;
};

typedef int (*mt_spb_publish_fn)(void *ctx, const char *topic, const uint8_t *payload,
				 size_t len, int qos, bool retain);
typedef int64_t (*mt_spb_now_ms_fn)(void);

struct mt_spb_dev {
	bool used;
	bool born;          /* a DBIRTH went out in this session */
	struct mt_spb_node last;
};

struct mt_spb_stats {
	uint32_t nbirths;
	uint32_t dbirths;
	uint32_t ddata;
	uint32_t ddeaths;
	uint32_t ndata;
	uint32_t rebirth_requests;
	uint32_t publish_errors;
	uint32_t encode_errors;
	uint32_t table_full;
};

struct mt_spb_session {
	char group[MT_SPB_ID_MAX];
	char edge[MT_SPB_ID_MAX];
	mt_spb_publish_fn publish;
	void *ctx;
	mt_spb_now_ms_fn now_ms;
	uint32_t death_after_s;

	uint8_t bdseq;
	uint8_t seq;
	bool born;

	/* node metrics */
	char build[24];
	uint32_t own_num;
	int64_t sent_devices;   /* last "gateway/devices" sent; -1 none */

	struct mt_spb_dev devs[CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES];
	struct mt_spb_stats stats;
	uint8_t buf[CONFIG_MESHTASTIC_SPARKPLUG_BUF_SIZE];
	char topic[MT_SPB_TOPIC_MAX];
};

/* A usable group or edge node id: non-empty, shorter than MT_SPB_ID_MAX, no '+', '/', '#'. */
bool mt_spb_valid_id(const char *id);

/* Set up; ids must be non-empty and free of '+', '/', '#'. -EINVAL otherwise. */
int mt_spb_init(struct mt_spb_session *s, const char *group, const char *edge,
		mt_spb_publish_fn publish, void *ctx, mt_spb_now_ms_fn now_ms,
		uint32_t death_after_s, const char *build, uint32_t own_num);

/* A new connection with this bdSeq (the caller has stored it). Devices will be born again. */
void mt_spb_session_start(struct mt_spb_session *s, uint8_t bdseq);

/* The NDEATH Will for this session: topic and payload (into the caller's buffer). */
int mt_spb_will(struct mt_spb_session *s, char *topic, size_t topic_len, uint8_t *payload,
		size_t *payload_len);

/* The NCMD topic to subscribe to (QoS 1). */
int mt_spb_ncmd_topic(const struct mt_spb_session *s, char *topic, size_t len);

/* CONNACK good: NBIRTH and a DBIRTH for each known live device. */
void mt_spb_connected(struct mt_spb_session *s);
void mt_spb_disconnected(struct mt_spb_session *s);

/* A snapshot of the nodes heard (any order; the gateway's own and never-heard ones are
 * skipped). `now_uptime_s` decides who has been silent past death_after_s. */
void mt_spb_update(struct mt_spb_session *s, const struct mt_spb_node *nodes, size_t n,
		   uint32_t now_uptime_s);

/* An NCMD payload. Returns 1 when it asked for (and got) a rebirth, 0 when it asked for nothing
 * acted on, -EBADMSG when it does not decode. */
int mt_spb_ncmd(struct mt_spb_session *s, const uint8_t *payload, size_t len);

/* The device id a node is published under: its node number, 8 lowercase hex digits. */
void mt_spb_device_id(uint32_t num, char out[9]);

#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_SPARKPLUG_SESSION_H_ */
