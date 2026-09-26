/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_MQTT_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_MQTT_H_

#include "meshtastic_core.h"
#include "meshtastic/mesh.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the bridge if ModuleConfig.mqtt enables it. @p network says whether this
 * boot brought up a network transport: the direct broker back end needs one, the
 * client proxy does not, so a BLE-only boot still starts a proxying bridge.
 */
int meshtastic_mqtt_init(bool network);

/**
 * A ToRadio.mqttClientProxyMessage from the connected client: something the
 * broker delivered on a topic the client subscribed to. Decoded and injected as a
 * downlink exactly as a direct broker delivery would be -- and ignored unless the
 * bridge is running in proxy mode (reference PhoneAPI.cpp).
 */
void meshtastic_mqtt_proxy_receive(const meshtastic_MqttClientProxyMessage *msg);

/** True while the MQTT client has a live broker connection. */
bool meshtastic_mqtt_is_connected(void);

/**
 * DontMqttMeBro consent gate (parity: mqtt #1) — pure, no socket/networking
 * dependency, so it is unit-testable outside CONFIG_MESHTASTIC_MQTT.
 *
 * Republishing another node's traffic takes it off the local RF mesh and puts
 * it somewhere the sender may never have agreed to. Mirrors the reference
 * (MQTT.cpp onSend): an absent bitfield reads as "no" — a node too old to
 * express a preference cannot be assumed to have given one. Two exemptions,
 * both from the reference: our own packets (we consented via
 * config_ok_to_mqtt) and a private broker (the concern is exposure to a
 * public broker, not one on the operator's own network).
 *
 * mqtt_queue_uplink (meshtastic_mqtt.c) is the only production caller —
 * mirror any change there.
 */
static inline bool meshtastic_mqtt_consent_allows_uplink(bool from_us, bool broker_is_private,
							   bool has_bitfield, uint8_t bitfield)
{
	if (from_us || broker_is_private) {
		return true;
	}

	return has_bitfield && (bitfield & MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK);
}
void meshtastic_mqtt_on_tx(const struct meshtastic_packet *packet, const uint8_t *wire,
			   size_t wire_len, const meshtastic_MeshPacket *mesh);
void meshtastic_mqtt_on_rx(const struct meshtastic_packet *packet, const uint8_t *wire,
			   size_t wire_len, const meshtastic_MeshPacket *mesh);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_MQTT_H_ */
