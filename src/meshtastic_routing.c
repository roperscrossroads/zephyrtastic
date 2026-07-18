/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <string.h>

#include <pb_encode.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define ROUTING_REPLY_HOP_MARGIN 2U

static bool packet_is_to_us(const struct meshtastic_packet *packet)
{
	return packet != NULL &&
	       (packet->to == mt.node_id || packet->to == MESHTASTIC_NODE_BROADCAST);
}

static uint8_t routing_hop_limit_for_reply(const struct meshtastic_packet *req)
{
	int16_t hops_used;

	if (req == NULL) {
		return mt.hop_limit;
	}

	if (req->hop_start == 0U) {
		return 0U;
	}

	if (req->hop_start < req->hop_limit) {
		return mt.hop_limit;
	}

	hops_used = (int16_t)req->hop_start - (int16_t)req->hop_limit;
	if (hops_used > (int16_t)mt.hop_limit) {
		return (uint8_t)hops_used;
	}

	/*
	 * Match upstream reply routing: use the return path we observed, plus a
	 * small margin because the route back may differ slightly.
	 */
	if ((uint8_t)(hops_used + ROUTING_REPLY_HOP_MARGIN) < mt.hop_limit) {
		return (uint8_t)(hops_used + ROUTING_REPLY_HOP_MARGIN);
	}

	return mt.hop_limit;
}

static bool routing_ack_should_request_ack(const struct meshtastic_packet *req)
{
	/*
	 * Mirror Meshtastic firmware's special case for direct text messages:
	 * request an ACK for the ROUTING ACK itself so DMs get reliable delivery
	 * confirmation back to the sender.
	 */
	return req != NULL && req->want_ack && req->to == mt.node_id &&
	       (req->portnum == MESHTASTIC_PORT_TEXT_MESSAGE ||
		req->portnum == meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP);
}

static int routing_send_ack(const struct meshtastic_packet *req)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	pb_ostream_t stream;
	uint8_t ch_index;

	if (req == NULL || req->to != mt.node_id) {
		return -EINVAL;
	}

	ch_index = (req->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID)
			   ? req->channel_index
			   : meshtastic_channels_primary_index();

	routing.error_reason = meshtastic_Routing_Error_NONE;
	routing.which_variant = meshtastic_Routing_error_reason_tag;

	mesh.from = mt.node_id;
	mesh.to = req->from;
	mesh.id = meshtastic_allocate_packet_id();
	mesh.channel = ch_index;
	mesh.hop_limit = routing_hop_limit_for_reply(req);
	mesh.hop_start = mesh.hop_limit;
	mesh.want_ack = routing_ack_should_request_ack(req);
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = meshtastic_PortNum_ROUTING_APP;
	mesh.decoded.request_id = req->id;
	mesh.decoded.want_response = false;
	mesh.priority = meshtastic_MeshPacket_Priority_ACK;

	stream = pb_ostream_from_buffer(mesh.decoded.payload.bytes,
					sizeof(mesh.decoded.payload.bytes));
	if (!pb_encode(&stream, meshtastic_Routing_fields, &routing)) {
		LOG_ERR("Routing ACK encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	mesh.decoded.payload.size = (pb_size_t)stream.bytes_written;

	LOG_DBG("Sending ROUTING ACK to 0x%08x for id=0x%08x ch=%u want_ack=%u hop_limit=%u",
		mesh.to, req->id, ch_index, mesh.want_ack ? 1U : 0U, mesh.hop_limit);

	return meshtastic_send_mesh_pb(&mesh);
}

void meshtastic_routing_on_decoded(const struct meshtastic_packet *packet)
{
	if (packet == NULL || !packet_is_to_us(packet) || packet->from == mt.node_id) {
		return;
	}

	/* An incoming ROUTING packet may be an ACK/NAK for something we sent. */
	if (packet->portnum == MESHTASTIC_PORT_ROUTING) {
		meshtastic_reliable_on_routing(packet);
	}

	if (packet->want_ack && packet->to == mt.node_id) {
		(void)routing_send_ack(packet);
	}
}

void meshtastic_routing_learn_next_hop(const struct meshtastic_packet *packet)
{
	uint8_t self_byte;

	if (packet == NULL) {
		return;
	}

	/* Next-hop route learning (increment 3). Learn a route only from traffic
	 * that crossed the LoRa mesh addressed to this node specifically -- a unicast
	 * we can attribute a working return path to. The relay_node byte is then the
	 * neighbour that last transmitted the frame toward us, i.e. our next hop back
	 * to the source. ROUTING ACKs/replies to our own want_ack sends are the
	 * canonical case (the "ACK/relay correlation" the NodeDB tracks); direct
	 * messages to us qualify too. Excluded: broadcasts (no confirmed return
	 * path) and MQTT-gateway injections (relay_node never rode the air, so it
	 * says nothing about the LoRa topology). */
	if (packet->to != mt.node_id || packet->from == 0U || packet->from == mt.node_id ||
	    packet->via_mqtt) {
		return;
	}

	/* relay_node 0 carries no route hint; our own low byte would mean relaying
	 * through ourselves, which is never a valid next hop. */
	self_byte = (uint8_t)(mt.node_id & 0xFFU);
	if (packet->relay_node == 0U || packet->relay_node == self_byte) {
		return;
	}

	if (meshtastic_nodedb_set_next_hop(packet->from, packet->relay_node) == 0) {
		LOG_DBG("route learn: next_hop(0x%08x)=0x%02x", (unsigned int)packet->from,
			packet->relay_node);
	}
}

void meshtastic_routing_sniff(const struct meshtastic_wire_header *hdr, const uint8_t *wire,
			      size_t wire_len, const struct meshtastic_packet *packet, bool decoded)
{
	ARG_UNUSED(decoded);

	meshtastic_routing_sniff_rebroadcast(hdr, wire, wire_len, packet);
}
