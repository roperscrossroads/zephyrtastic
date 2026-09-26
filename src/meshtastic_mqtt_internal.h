/* SPDX-License-Identifier: GPL-3.0 */

/*
 * The seam between the MQTT bridge core (meshtastic_mqtt.c: topics, envelopes,
 * the uplink gate, downlink decode, map reports, the publish queue -- none of
 * which needs a network) and the direct broker back end
 * (meshtastic_mqtt_broker.c: the socket, TLS, the thread and its backoff). The
 * other back end, the client proxy, has no file of its own: it is the core
 * handing each publish to the PhoneAPI (meshtastic_phoneapi_enqueue_mqtt_proxy)
 * and the PhoneAPI handing each delivery back (meshtastic_mqtt_proxy_receive).
 *
 * Private to those two files.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_MQTT_INTERNAL_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_MQTT_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_mqtt_config.h"

#define MESHTASTIC_MQTT_TOPIC_MAX 128U

/* One encoded publish waiting for its back end. */
struct meshtastic_mqtt_pub_entry {
	char topic[MESHTASTIC_MQTT_TOPIC_MAX];
	uint8_t payload[CONFIG_MESHTASTIC_MQTT_TX_BUFFER_SIZE];
	uint16_t len;
};

/* ---- core, for the broker back end -------------------------------------------- */

/** The resolved ModuleConfig.mqtt the bridge started with. */
const struct meshtastic_mqtt_settings *meshtastic_mqtt_core_settings(void);

/** Take the oldest queued publish. False when the queue is empty. */
bool meshtastic_mqtt_core_dequeue(struct meshtastic_mqtt_pub_entry *out);

/** True while anything is queued. */
bool meshtastic_mqtt_core_pending(void);

/** The broker is reachable again: the next queue-full episode warns afresh. */
void meshtastic_mqtt_core_link_up(void);

/** A ServiceEnvelope the broker delivered on @p topic. */
void meshtastic_mqtt_core_downlink(const uint8_t *payload, size_t len, const char *topic);

/** Publish a MapReport if one is due (no-op when map reporting is off). */
void meshtastic_mqtt_core_perhaps_report_to_map(void);

/** "{root}/2/e/{channel}/+", the downlink subscription. */
int meshtastic_mqtt_core_subscribe_topic(char *topic, size_t topic_len);

/** "!xxxxxxxx", this node's id as MQTT spells it. */
void meshtastic_mqtt_core_node_id_str(char *buf, size_t len);

/* ---- broker back end, for the core -------------------------------------------- */

#if defined(CONFIG_MESHTASTIC_MQTT_BROKER)
/** Start the broker thread. Called once, from meshtastic_mqtt_init(). */
int meshtastic_mqtt_broker_start(void);

/** Wake the broker thread: the queue gained work. Safe from any thread. */
void meshtastic_mqtt_broker_notify(void);

/** True while the broker session is up. */
bool meshtastic_mqtt_broker_connected(void);
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_MQTT_INTERNAL_H_ */
