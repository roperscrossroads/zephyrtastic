/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_PACKET_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_PACKET_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESHTASTIC_FLAGS_HOP_LIMIT_MASK  0x07U
#define MESHTASTIC_FLAGS_WANT_ACK        BIT(3)
#define MESHTASTIC_FLAGS_VIA_MQTT        BIT(4)
#define MESHTASTIC_FLAGS_HOP_START_SHIFT 5U
#define MESHTASTIC_FLAGS_HOP_START_MASK  (0x07U << MESHTASTIC_FLAGS_HOP_START_SHIFT)

struct __packed meshtastic_wire_header {
	uint32_t dest;
	uint32_t src;
	uint32_t id;
	uint8_t flags;
	uint8_t channel;
	uint8_t next_hop;
	uint8_t relay_node;
};

/**
 * @brief Why a wire packet could not be decoded.
 *
 * Surfaced by @ref meshtastic_try_decode_wire_packet so the router can return a
 * ROUTING error to the sender of a want_ack unicast addressed to us instead of
 * silently dropping it (mirrors the reference ReliableRouter). @c NONE covers
 * both a successful decode and an ignored frame that must never be NAKed.
 */
enum meshtastic_decode_fail {
	MESHTASTIC_DECODE_FAIL_NONE = 0,
	MESHTASTIC_DECODE_FAIL_NO_CHANNEL,
	MESHTASTIC_DECODE_FAIL_PKI_UNKNOWN_PUBKEY,
};

int meshtastic_encode_data(uint32_t portnum, const uint8_t *payload, size_t payload_len,
			   uint8_t *buf, size_t buf_len, size_t *encoded_len);
int meshtastic_packet_to_mesh_pb(const struct meshtastic_packet *packet,
				 meshtastic_MeshPacket *mesh);
void meshtastic_mesh_packet_copy(meshtastic_MeshPacket *dst, const meshtastic_MeshPacket *src);
int meshtastic_mesh_pb_try_decode(meshtastic_MeshPacket *mesh);
int meshtastic_mesh_pb_to_packet(const meshtastic_MeshPacket *mesh,
				 struct meshtastic_packet *packet, uint8_t *payload,
				 size_t payload_len);
int meshtastic_decode_wire_packet(const uint8_t *buf, int len, int16_t rssi, int8_t snr,
				  struct meshtastic_packet *packet, uint8_t *payload,
				  size_t payload_len);
int meshtastic_try_decode_wire_packet(const uint8_t *buf, int len, int16_t rssi, int8_t snr,
				      struct meshtastic_packet *packet, uint8_t *payload,
				      size_t payload_len, bool *decoded,
				      enum meshtastic_decode_fail *fail_reason);
uint8_t meshtastic_packet_wire_hash_for_index(uint8_t channel_index);
int meshtastic_build_wire_packet(const struct meshtastic_packet *packet, uint8_t *out,
				 uint32_t *out_len);
int meshtastic_send_mesh_pb(const meshtastic_MeshPacket *mesh);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_PACKET_H_ */
