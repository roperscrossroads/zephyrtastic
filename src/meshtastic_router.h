/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/**
 * @file meshtastic_router.h
 * @brief Internal packet ingress, delivery, relay, and MQTT downlink injection.
 *
 * The router sits between the LoRa radio driver and higher layers. Responsibilities include
 * duplicate suppression, wire decode, local delivery, flood rebroadcast, and injecting packets
 * received from MQTT back onto the mesh.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_ROUTER_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_ROUTER_H_

#include <stddef.h>
#include <stdint.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Process one raw LoRa frame from the radio RX path.
 *
 * Filters duplicates, decodes the wire packet, then passes the result to
 * @ref meshtastic_handle_inbound_packet.
 */
void meshtastic_router_process_lora_rx(const uint8_t *buf, int len, int16_t rssi, int8_t snr);

/**
 * @brief Common inbound path after LoRa RX or MQTT downlink injection.
 *
 * Delivers locally and runs port/routing hooks when @p decoded is true; may flood-relay the
 * original wire frame when @p wire is non-NULL. @p wire may be @c NULL for gateway- injected
 * packets.
 */
void meshtastic_handle_inbound_packet(const struct meshtastic_packet *packet, const uint8_t *wire,
				      size_t wire_len, bool decoded);

/**
 * @brief Inject a @c MeshPacket from a gateway (e.g. MQTT) into the mesh pipeline.
 *
 * Applies duplicate filtering, optionally re-transmits onto LoRa, then delivers locally when
 * addressed to this node or broadcast.
 */
int meshtastic_inject_downlink_mesh_packet(const meshtastic_MeshPacket *mesh);

/**
 * @brief Flood-relay a foreign packet when rebroadcast policy allows.
 *
 * Re-sends the wire frame with hop limit decremented; does not require payload decode.
 */
void meshtastic_routing_sniff_rebroadcast(const struct meshtastic_wire_header *hdr,
					  const uint8_t *wire, size_t wire_len,
					  const struct meshtastic_packet *packet);

/** @brief Routing-port handling for decoded packets (e.g. ACK replies). */
void meshtastic_routing_on_decoded(const struct meshtastic_packet *packet);

/**
 * @brief Stamp next-hop routing fields on a directed unicast this node originates.
 *
 * Next-hop routing (increment 2). For a directed unicast we originate
 * (@p from is this node), marks us as the relayer (@p relay_node low byte) and
 * sets the learned next hop toward @p to (@p next_hop, 0 = none → flood).
 * Broadcasts and packets we relay for others are left untouched, so only traffic
 * we source carries a routing preference. The next-hop byte comes from
 * @ref meshtastic_nodedb_get_next_hop, populated by
 * @ref meshtastic_routing_learn_next_hop (increment 3); an unlearned destination
 * stamps @p relay_node only and floods.
 *
 * @param to         Destination node number of the packet.
 * @param from       Originating node number (already resolved, never 0).
 * @param next_hop   In/out: learned next-hop byte; set when 0 and we originate.
 * @param relay_node In/out: relayer byte; set to our low byte when 0 and we originate.
 */
void meshtastic_router_stamp_originated(uint32_t to, uint32_t from, uint8_t *next_hop,
					uint8_t *relay_node);

/**
 * @brief Learn the next hop toward a node from a unicast it sent us.
 *
 * Next-hop route learning (increment 3). When @p packet is a decoded unicast
 * addressed to this node (an ACK/reply to one of our want_ack sends, or a direct
 * message), its @c relay_node byte is the neighbour that last relayed it toward
 * us — i.e. our next hop back to @c packet->from. Records that in the NodeDB so
 * @ref meshtastic_nodedb_get_next_hop can steer our future unicasts to that node.
 * Broadcasts and MQTT-injected packets are ignored (no confirmed LoRa return
 * path). Must be called after NodeDB module dispatch so the source entry exists.
 */
void meshtastic_routing_learn_next_hop(const struct meshtastic_packet *packet);

/**
 * @brief Send a ROUTING error (NAK) to the sender of an undecodable want_ack unicast.
 *
 * When a want_ack unicast addressed to us cannot be decoded, reply with a ROUTING
 * error (e.g. @c NO_CHANNEL / @c PKI_UNKNOWN_PUBKEY) referencing @p req's packet id
 * so the sender stops retransmitting and surfaces the real reason instead of timing
 * out (mirrors the reference ReliableRouter). Sent on the primary channel, since the
 * frame's real channel is unknown. No-op for @p req == NULL or a packet we sourced.
 */
void meshtastic_routing_send_error(const struct meshtastic_packet *req,
				   meshtastic_Routing_Error err);

/**
 * @brief Record that a neighbour rebroadcast one of our own packets.
 *
 * Called from the own-echo RX branch (wire src == this node) with the echo's
 * @c relay_node byte. Feeds the two-way relayer correlation that gates
 * next-hop learning (@ref meshtastic_routing_learn_next_hop): a relayer is
 * only trusted once it demonstrably carried our packet outbound and the
 * ACK/reply back. Zero / own-byte relayers are ignored.
 */
void meshtastic_routing_note_own_echo(uint32_t id, uint8_t relay_node);

/**
 * @brief Re-ACK the retransmission of a reliable unicast we already handled.
 *
 * Called from the dedup drop path when a duplicate arrives with the
 * repeated-reliable signature (@c hop_start == @c hop_limit, i.e. straight from
 * the originator — our first ACK was lost). Sends a fresh ROUTING ACK for
 * @p id back to @p from without re-delivering the packet, so the sender stops
 * retrying instead of reporting a false delivery failure (mirrors the reference
 * NextHopRouter dup-path re-ACK). Works from wire-header fields alone — no
 * payload decode; @p wire_hash selects the reply channel.
 */
void meshtastic_routing_reack_duplicate(uint32_t from, uint32_t id, uint8_t wire_hash,
					uint8_t hop_limit, uint8_t hop_start);

/**
 * @brief Post-RX hook for relay policy.
 *
 * Currently delegates to @ref meshtastic_routing_sniff_rebroadcast.
 */
void meshtastic_routing_sniff(const struct meshtastic_wire_header *hdr, const uint8_t *wire,
			      size_t wire_len, const struct meshtastic_packet *packet,
			      bool decoded);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_ROUTER_H_ */
