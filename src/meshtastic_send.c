/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * The send engine: every packet this node originates goes through here.
 *
 * Two entry points build a frame: meshtastic_send_packet() (the struct
 * originators: modules, the shell, the application) and
 * meshtastic_send_mesh_decoded() (a decoded MeshPacket from the phone, whose
 * Data is sent as it came). Both build under mt_ws.lock, then share the
 * lock-free tail: scheduling tier, airtime gate, own-broadcast contention
 * delay, radio or worker submit, reliable tracking, MQTT uplink, TX_DONE.
 * meshtastic_send_mesh_pb() takes the phone's packets, and sends an already
 * encrypted one (PKC) as it is, with only the header stamped here.
 *
 * Split out of meshtastic.c, with send_mesh_pb from meshtastic_packet.c
 * (agents-ls39.3.2). Unchanged code.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <pb_encode.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_contention.h"
#include "meshtastic_core.h"
#include "meshtastic_modules.h"
#include "meshtastic_mqtt.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"
#include "meshtastic_sched.h"
#if defined(CONFIG_MESHTASTIC_HUB)
#include "meshtastic_hub.h"
#endif
#if defined(CONFIG_MESHTASTIC_AIRTIME)
#include "meshtastic_airtime.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* C3 Phase 6c: build the outgoing wire from the staged MeshPacket (mt_ws.tx_mesh) — the
 * core of the mesh-native send engine. PRECONDITION: mt_ws.lock is held and mt_ws.tx_mesh
 * carries the envelope, the authoritative decoded Data, and mesh->channel already resolved
 * to the send index. Applies the mesh-native envelope defaults, runs the TX-sanitiser on
 * our own originations (POS-1), stamps next-hop routing, encodes+encrypts the frame into
 * @p wire, then materialises the flat @p local (payload copied into @p local_payload, which
 * must outlive the send) for the lock-free tail. Returns the build status: <0 aborts the
 * send (incl. -ENODATA from a sanitiser that fail-closes on a no-share channel). */
static int mt_ws_build_wire_locked(uint8_t *wire, uint32_t *pkt_len,
				   struct meshtastic_packet *local, uint8_t *local_payload,
				   meshtastic_MeshPacket *tx_local)
{
#if defined(CONFIG_MESHTASTIC_WORKER)
	/* A keyless worker has no identity to originate with (ATTACHMENT-DESIGN S5).
	 * The PhoneAPI and the shell still reach this step; refusing HERE, before
	 * anything is built, is what keeps the frame builder (and every key it
	 * would use) out of the image -- cmake/keyless-assert.cmake checks. */
	ARG_UNUSED(wire);
	ARG_UNUSED(pkt_len);
	ARG_UNUSED(local);
	ARG_UNUSED(local_payload);
	ARG_UNUSED(tx_local);
	return -EPERM;
#else
	meshtastic_MeshPacket *mesh = &mt_ws.tx_mesh;
	uint8_t next_hop;
	uint8_t relay_node;
	int ret;

	if (mesh->from == 0U) {
		mesh->from = mt.node_id;
	}
	if (mesh->id == 0U) {
		mesh->id = meshtastic_allocate_packet_id();
	}
	/* 0 means "the node default" -- unless the sender asked for a literal zero-hop
	 * frame (mt_ws.tx_zero_hop): then hop_limit stays 0 so no receiver relays it,
	 * and hop_start stays whatever the sender set (0, or 1 for the reference's
	 * legacy override that pre-2.7.20 receivers need). */
	if (mesh->hop_limit == 0U && !mt_ws.tx_zero_hop) {
		mesh->hop_limit = mt.hop_limit;
	}
	if (mesh->hop_start == 0U && !mt_ws.tx_zero_hop) {
		mesh->hop_start = mesh->hop_limit;
	}

	/* TX-side sanitisation for packets we originate — the send-path mirror of the RX
	 * module dispatch, now operating on the outbound MeshPacket. Lets a module rewrite
	 * mesh->decoded.payload before the wire is built (mask an outbound Position to its
	 * channel's sharing precision, POS-1). Relays re-transmit the raw wire and never
	 * reach this path; MQTT-injected third-party frames carry a foreign `from`/via_mqtt
	 * and are skipped — so only our own traffic is altered. A negative return suppresses
	 * the send (e.g. -ENODATA on a channel that shares no position, fail-closed like
	 * upstream). */
	if (mesh->from == mt.node_id && !mesh->via_mqtt) {
		ret = meshtastic_modules_sanitise_tx(mesh);
		if (ret < 0) {
			return ret;
		}
	}

	/* Next-hop routing (increment 2): stamp our relayer byte + learned next hop on
	 * directed unicasts we originate. MeshPacket's next_hop/relay_node are wider ints, so
	 * marshal through uint8_t for the stamper (mirrors the encrypted-path stamp). */
	next_hop = (uint8_t)mesh->next_hop;
	relay_node = (uint8_t)mesh->relay_node;
	meshtastic_router_stamp_originated(mesh->to, mesh->from, &next_hop, &relay_node);
	mesh->next_hop = next_hop;
	mesh->relay_node = relay_node;

	ret = meshtastic_build_wire_from_mesh(mesh, wire, pkt_len);
	if (ret < 0) {
		return ret;
	}

	/* C3 Phase 7d: snapshot the final outgoing MeshPacket so the lock-free tail's
	 * mesh-native hooks (reliable_on_tx / mqtt_on_tx) read the authoritative Data — emoji
	 * and any field the flat struct never modelled survive our own reliable/uplink path.
	 * Copied off the shared workspace so a concurrent send can't overwrite it during our
	 * blocking radio submit. */
	meshtastic_mesh_packet_copy(tx_local, mesh);

	/* Materialise the flat struct the lock-free tail (airtime / tier / TX_DONE event)
	 * still consumes — the TX_DONE event is a public struct boundary. Its payload is copied
	 * into local_payload, stable across the blocking radio send, so the tail never reaches
	 * back into mt_ws.tx_mesh after we drop the lock. */
	return meshtastic_mesh_pb_to_packet(mesh, local, local_payload,
					    MESHTASTIC_MAX_PAYLOAD_LEN);
#endif /* CONFIG_MESHTASTIC_WORKER */
}

static int send_packet_complete(const struct meshtastic_packet *local,
				const meshtastic_MeshPacket *mesh, const uint8_t *wire,
				uint32_t pkt_len, int tx_ret, bool emit_done)
{
	if (tx_ret < 0) {
		LOG_ERR("Packet send failed (%d)", tx_ret);
		return tx_ret;
	}

	/* The MQTT uplink is for every frame we originate, once the send engine took it
	 * (upstream Router::send uplinks after iface->send(), whoever called and whether or
	 * not it waited): beacons, telemetry, module replies and ACKs are all K_NO_WAIT. It
	 * only queues, so an ACK sent from the RX thread still never waits for the air. */
#if defined(CONFIG_MESHTASTIC_MQTT)
	meshtastic_mqtt_on_tx(local, wire, pkt_len, mesh);
#endif

	/* TX_DONE is the blocking caller's "sent" (documented in meshtastic.h): a
	 * fire-and-forget send has nobody waiting and no completion to report. */
	if (emit_done) {
		LOG_INF("TX to 0x%08x port=%u len=%u", local->to, (unsigned int)local->portnum,
			(unsigned int)pkt_len);
		meshtastic_emit_event(MESHTASTIC_EVENT_TX_DONE, 0, local);
	}

	return 0;
}

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE) || defined(CONFIG_MESHTASTIC_HUB)
static void routing_nak_to_phone(const struct meshtastic_packet *blocked,
				 meshtastic_Routing_Error reason);
#endif

#if defined(CONFIG_MESHTASTIC_HUB)
/* Which radio an ORIGINATED packet on @p send_index leaves by (agents-pcs2.11):
 * a slot named after a preset a worker is on sends through that worker. With no
 * ready worker on that preset the send is refused -- and the phone is told
 * (NO_INTERFACE, upstream's "no suitable interface for this packet") -- rather
 * than keyed up on our own radio with another preset's channel hash, where
 * nothing would decode it. @p id is the phone's packet id, 0 if not from one. */
static int link_for_send(uint8_t send_index, uint32_t to, uint32_t id, uint8_t *radio)
{
	int a;

	if (*radio != 0U && *radio != MESHTASTIC_RADIO_ALL) {
		return 0; /* a reply: already bound to the radio it came in on */
	}
	a = meshtastic_hub_for_channel(send_index);
	if (a < 0) {
		LOG_WRN("TX refused: channel %u is for a preset no ready worker is on",
			(unsigned int)send_index);
		if (id != 0U) {
			const struct meshtastic_packet blocked = {
				.id = id,
				.channel_index = send_index,
			};

			routing_nak_to_phone(&blocked, meshtastic_Routing_Error_NO_INTERFACE);
		}
		return a;
	}
	if (a > 0) {
		*radio = (uint8_t)a; /* a preset's own channel: that preset's worker */
		return 0;
	}
#if defined(CONFIG_MESHTASTIC_NODEDB)
	/* A unicast goes to the peer on the radio that last heard it
	 * (agents-pcs2.13): one transmission on the preset it is on, not one on
	 * every preset we cover. Unknown since boot, or no worker ready on that
	 * preset any more: the caller's default stands. */
	if (to != MESHTASTIC_NODE_BROADCAST && to != 0U && to != mt.node_id) {
		struct meshtastic_nodedb_node peer;

		if (meshtastic_nodedb_get(to, &peer) == 0 &&
		    peer.heard_preset != MESHTASTIC_PRESET_UNKNOWN) {
			if (peer.heard_preset == (uint8_t)mt.modem_preset) {
				*radio = 0U;
				return 1; /* our own radio, and settled */
			}
			a = (int)meshtastic_hub_for_preset(peer.heard_preset);
			if (a > 0) {
				*radio = (uint8_t)a;
				return 1;
			}
		}
	}
#endif
	return 0;
}
#endif

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
/* Emit a ROUTING_APP NAK carrying DUTY_CYCLE_LIMIT straight to the phone.
 *
 * Not meshtastic_routing_send_error(): that one is the RX-path NAK, it refuses a
 * request whose `from` is us, and it would go out over the radio — which is the
 * one thing this situation forbids. This mirrors the local-admin reply path
 * (meshtastic_admin.c admin_emit_reply, PhoneAPI branch): build the packet,
 * hand it to the transports, never touch the air. */
static void duty_nak_to_phone(const struct meshtastic_packet *blocked)
{
	routing_nak_to_phone(blocked, meshtastic_Routing_Error_DUTY_CYCLE_LIMIT);
}
#endif

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE) || defined(CONFIG_MESHTASTIC_HUB)
static void routing_nak_to_phone(const struct meshtastic_packet *blocked,
				 meshtastic_Routing_Error reason)
{
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	uint8_t buf[16];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet nak = {0};

	routing.which_variant = meshtastic_Routing_error_reason_tag;
	routing.error_reason = reason;

	if (!pb_encode(&stream, meshtastic_Routing_fields, &routing)) {
		return;
	}

	nak.from = mt.node_id;
	nak.to = mt.node_id;
	nak.id = meshtastic_allocate_packet_id();
	nak.request_id = blocked->id;
	nak.portnum = meshtastic_PortNum_ROUTING_APP;
	nak.payload = buf;
	nak.payload_len = (uint16_t)stream.bytes_written;
	nak.channel_index = blocked->channel_index;

	meshtastic_phoneapi_on_packet(&nak, NULL);
}
#endif

/* Lock-free send tail shared by both mesh-native entry points: schedule tier, airtime
 * gate, radio submit (own-CW contention for broadcasts), reliable tracking, TX_DONE +
 * MQTT uplink. Operates only on the materialised @p local + built @p wire — mt_ws is no
 * longer touched here, so the blocking radio submit runs without the workspace lock. */
static int send_wire_tail(const struct meshtastic_packet *local,
			  const meshtastic_MeshPacket *mesh, uint8_t *wire, uint32_t pkt_len,
			  k_timeout_t wait, uint8_t radio)
{
	uint8_t tier = meshtastic_sched_tier_for(local->portnum);
	const bool all = (radio == MESHTASTIC_RADIO_ALL);
	int ret;

	if (all) {
		/* Every radio: ours through everything below, then the workers. On a
		 * node without workers this is an ordinary send. */
		radio = 0U;
	}

	if (radio != 0U) {
		/* Through a worker (ATTACHMENT-P3-PLAN slice 2): the worker keys
		 * up on its own clock with its own contention window and CAD; the
		 * hub's airtime gate and own-CW below are its own radio's. A
		 * blocking caller waits for the hand-over to the worker, not for the
		 * air. */
		if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
			ret = meshtastic_radio_send_wire_after_on(wire, pkt_len, tier, 0U, radio);
		} else {
			ret = meshtastic_radio_send_wire_wait_prio_on(wire, pkt_len, tier, wait, radio);
		}
		if (ret == -EHOSTUNREACH) {
			LOG_DBG("TX via radio %u refused: worker not ready", (unsigned int)radio);
		}
		if (ret >= 0) {
			/* Retransmits, and the implicit ACK heard back through the
			 * worker, belong to this radio. */
			meshtastic_reliable_on_tx(local, wire, pkt_len, mesh, radio);
		}
		/* "Done" for a worker send is the hand-over: queued at the worker (a blocking
		 * caller waited for exactly that), as ours is queued at the radio. The
		 * worker's own TX_RESULT comes later and is the hub's, not this event's. */
		if (ret < 0) {
			return ret; /* a refused worker send stays a debug line, as before */
		}
		return send_packet_complete(local, mesh, wire, pkt_len, ret,
					    K_TIMEOUT_EQ(wait, K_FOREVER));
	}

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	/* Airtime gate: throttle only the node's own periodic background beacons
	 * (broadcast, fire-and-forget). Requested replies (unicast and/or a caller
	 * that waits) and every higher tier are never gated. */
	if (tier == MT_SCHED_TIER_BG && K_TIMEOUT_EQ(wait, K_NO_WAIT) &&
	    local->to == MESHTASTIC_NODE_BROADCAST) {
		/* Single scalar, captured once — a direct atomic read is sufficient
		 * (see the concurrency note in meshtastic_sched.h). */
		uint8_t max_util = meshtastic_sched_get()->airtime_max_util;

		if (max_util != 0U &&
		    meshtastic_airtime_channel_util_percent() >= (float)max_util) {
			meshtastic_sched_stat_airtime_drop();
			LOG_DBG("airtime gate: suppressed BG broadcast port=%u (chan util >= %u%%)",
				(unsigned int)local->portnum, max_util);
			return 0;
		}
	}
#endif

	if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
		/* OWN-CW: stagger our own fire-and-forget broadcasts with the CSMA
		 * contention window (reference setTransmitDelay/getTxDelayMsec), so
		 * simultaneous beacons/telemetry across nearby nodes don't key up in
		 * lockstep. delay_ms of 0 (window disabled, or the draw landing on slot 0)
		 * is identical to the immediate path. Directed unicasts keep immediate
		 * timing; the relay path has its own (weighted) jitter. */
		if (local->to == MESHTASTIC_NODE_BROADCAST) {
			struct meshtastic_contention_plan plan;
			uint8_t util = 0U;

#if defined(CONFIG_MESHTASTIC_AIRTIME)
			util = (uint8_t)meshtastic_airtime_channel_util_percent();
#endif
			meshtastic_contention_plan_own(util, mt.modem.spread_factor,
						       mt.modem.bandwidth_hz, false, &plan);
			ret = meshtastic_radio_send_wire_after(wire, pkt_len, tier, plan.delay_ms);
		} else {
			ret = meshtastic_radio_send_wire_prio(wire, pkt_len, tier);
		}
	} else {
		ret = meshtastic_radio_send_wire_wait_prio(wire, pkt_len, tier, wait);
	}

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
	if (ret == -ECANCELED) {
		/* The regulatory gate refused this frame. Tell the client WHY rather
		 * than letting a text message vanish: upstream NAKs the API with
		 * DUTY_CYCLE_LIMIT here (Router::send -> abortSendAndNak), and a
		 * silently dropped message is exactly the failure that gets reported
		 * as "the radio is broken".
		 *
		 * Straight to the PhoneAPI, never over the air: the frame is ours, so
		 * upstream's NAK is a local delivery too — and sending it on the radio
		 * would be a transmission made because we were told not to transmit,
		 * which would also recurse into this same gate. */
		duty_nak_to_phone(local);
	}
#endif

	if (ret >= 0) {
		/* Track for retransmission if it is a want_ack packet we originate
		 * (the hook self-filters everything else). */
		meshtastic_reliable_on_tx(local, wire, pkt_len, mesh,
					  all ? MESHTASTIC_RADIO_ALL : 0U);
#if defined(CONFIG_MESHTASTIC_HUB)
		if (all) {
			(void)meshtastic_hub_fan_out(wire, pkt_len, tier,
							    local->channel_index);
		}
#endif
	}

	return send_packet_complete(local, mesh, wire, pkt_len, ret, K_TIMEOUT_EQ(wait, K_FOREVER));
}

int meshtastic_send_packet(const struct meshtastic_packet *packet, k_timeout_t wait)
{
	struct meshtastic_packet local;
	meshtastic_MeshPacket tx_local;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t local_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint32_t pkt_len = 0U;
	uint8_t channel_index;
	uint8_t send_index;
	uint8_t radio;
	int ret;

	if (!mt.initialized) {
		return -EINVAL;
	}

	if (packet == NULL || packet->to == 0U ||
	    (packet->payload == NULL && packet->payload_len != 0U) ||
	    packet->payload_len > MESHTASTIC_MAX_PAYLOAD_LEN) {
		LOG_ERR("Invalid packet");
		return -EINVAL;
	}

	/* Resolve the send channel from the struct while its wire-hash field is still in hand:
	 * default an unset index to the primary, then run the resolution the wire build keys
	 * on. to_mesh_pb() stores only the index in mesh->channel and drops the hash, so this
	 * must happen here (identical inputs/order to the pre-6c struct path). */
	channel_index = (packet->channel_index == MESHTASTIC_CHANNEL_INDEX_INVALID)
				? meshtastic_channels_primary_index()
				: packet->channel_index;
	send_index = meshtastic_channels_resolve_send_index(packet->to, channel_index,
							    packet->channel);
	radio = packet->tx_radio;
	if (radio == MESHTASTIC_RADIO_OWN) {
		radio = 0U; /* a reply to something our own radio heard: stays there */
	} else {
#if defined(CONFIG_MESHTASTIC_HUB)
		ret = link_for_send(send_index, packet->to, 0U, &radio);
		if (ret < 0) {
			return ret;
		}
#endif
	}

	/* C3 Phase 6c: the wire is built mesh-native. Convert the struct to the outgoing
	 * MeshPacket in mt_ws scratch (off the right-sized app-thread send stacks) and build
	 * under the workspace lock; the mesh-native defaults / sanitise / stamp run inside. */
	k_mutex_lock(&mt_ws.lock, K_FOREVER);
	ret = meshtastic_packet_to_mesh_pb(packet, &mt_ws.tx_mesh);
	if (ret == 0) {
		mt_ws.tx_mesh.channel = send_index;
		mt_ws.tx_zero_hop = packet->zero_hop;
		mt_ws.tx_no_pkc = packet->no_pkc;
		mt_ws.tx_radio = (radio == MESHTASTIC_RADIO_ALL) ? 0U : radio;
		ret = mt_ws_build_wire_locked(wire, &pkt_len, &local, local_payload, &tx_local);
		mt_ws.tx_zero_hop = false;
		mt_ws.tx_no_pkc = false;
		mt_ws.tx_radio = 0U;
	}
	k_mutex_unlock(&mt_ws.lock);
	if (ret < 0) {
		return ret;
	}

	return send_wire_tail(&local, &tx_local, wire, pkt_len, wait, radio);
}

int meshtastic_send_mesh_decoded(const meshtastic_MeshPacket *mesh, k_timeout_t wait)
{
	struct meshtastic_packet local;
	meshtastic_MeshPacket tx_local;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t local_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint32_t pkt_len = 0U;
	uint32_t to_norm;
	uint8_t channel_index;
	uint8_t send_index;
	uint8_t radio = 0U;
	int ret;

	if (!mt.initialized || mesh == NULL) {
		return -EINVAL;
	}

	/* Phone/PKC decoded send path (C3 Phase 6c): the MeshPacket's decoded Data is
	 * authoritative — emoji and every field the flat struct never modelled ride through
	 * by construction, so the Phase-3 `base` hop is gone. Normalise the destination and
	 * resolve the send channel exactly as the struct path does (default an out-of-range
	 * index to the primary; the phone carries no wire hash). */
	to_norm = (mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST;
	channel_index = (mesh->channel < MESHTASTIC_MAX_CHANNELS)
				? (uint8_t)mesh->channel
				: meshtastic_channels_primary_index();
	send_index = meshtastic_channels_resolve_send_index(to_norm, channel_index, 0U);
#if defined(CONFIG_MESHTASTIC_HUB)
	ret = link_for_send(send_index, to_norm, mesh->id, &radio);
	if (ret < 0) {
		return ret;
	}
	/* What the user says from the app (not an MQTT relay, whose from is not
	 * ours) goes out on every radio when no single preset owns it: a direct
	 * message to a peer not heard since boot, which may be on any of them
	 * (one heard since went to its radio above, ret 1), and anything on a
	 * channel with a name of its own, which is one chat across all presets --
	 * as it already is on receive. A preset's own channel went to that
	 * preset's worker above; the primary is our own radio's. */
	if (ret == 0 && radio == 0U && (mesh->from == 0U || mesh->from == mt.node_id) &&
	    (to_norm != MESHTASTIC_NODE_BROADCAST ||
	     meshtastic_channels_custom_named(send_index))) {
		radio = MESHTASTIC_RADIO_ALL;
	}
#endif

	k_mutex_lock(&mt_ws.lock, K_FOREVER);
	meshtastic_mesh_packet_copy(&mt_ws.tx_mesh, mesh);
	mt_ws.tx_mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mt_ws.tx_mesh.to = to_norm;
	mt_ws.tx_mesh.channel = send_index;
	mt_ws.tx_zero_hop = false;
	mt_ws.tx_no_pkc = false;
	mt_ws.tx_radio = (radio == MESHTASTIC_RADIO_ALL) ? 0U : radio;
	ret = mt_ws_build_wire_locked(wire, &pkt_len, &local, local_payload, &tx_local);
	mt_ws.tx_radio = 0U;
	k_mutex_unlock(&mt_ws.lock);
	if (ret < 0) {
		return ret;
	}

	return send_wire_tail(&local, &tx_local, wire, pkt_len, wait, radio);
}

int meshtastic_send_data(uint32_t dest, uint32_t portnum, const uint8_t *payload,
			 size_t payload_len, k_timeout_t wait)
{
	struct meshtastic_packet packet = {
		.to = dest,
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
		/* Request an ack for directed unicasts (reliable delivery, like the
		 * reference app does for DMs); broadcasts stay unacked. */
		.want_ack = (dest != MESHTASTIC_NODE_BROADCAST && dest != 0U),
	};

	return meshtastic_send_packet(&packet, wait);
}

int meshtastic_send_text(uint32_t dest, const char *text)
{
	size_t len;

	if (text == NULL) {
		return -EINVAL;
	}

	len = strlen(text);
	if (len == 0U || len > MESHTASTIC_MAX_TEXT_LEN) {
		return -EINVAL;
	}

	return meshtastic_send_data(dest, MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)text, len,
				    K_FOREVER);
}

int meshtastic_send_mesh_pb(const meshtastic_MeshPacket *mesh)
{
	uint32_t encrypted_len;
	struct meshtastic_wire_header *hdr;
	uint8_t hop_limit;
	uint8_t hop_start;
	int ret;

	if (mesh == NULL) {
		return -EINVAL;
	}

	if (mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		/* C3 Phase 6c: feed the phone's decoded MeshPacket straight into the
		 * mesh-native send engine. Its decoded Data is the authoritative outgoing
		 * payload, so fields the flat struct never models (Data.emoji, a reaction's
		 * flag, anything upstream adds next) reach the wire by construction — no struct
		 * round-trip, no hand-patched `base`. */
		return meshtastic_send_mesh_decoded(mesh, K_FOREVER);
	}

	if (mesh->which_payload_variant != meshtastic_MeshPacket_encrypted_tag) {
		return -ENOTSUP;
	}

	encrypted_len = mesh->encrypted.size;
	if (encrypted_len > MESHTASTIC_PAYLOAD_MAX) {
		return -EINVAL;
	}

	hop_limit = (mesh->hop_limit == 0U) ? mt.hop_limit : mesh->hop_limit;
	hop_start = (mesh->hop_start == 0U) ? hop_limit : mesh->hop_start;

	k_mutex_lock(&mt_ws.lock, K_FOREVER);

	hdr = (struct meshtastic_wire_header *)mt_ws.wire;
	hdr->dest = sys_cpu_to_le32((mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST);
	hdr->src = sys_cpu_to_le32((mesh->from != 0U) ? mesh->from : mt.node_id);
	hdr->id = sys_cpu_to_le32((mesh->id != 0U) ? mesh->id : meshtastic_allocate_packet_id());
	hdr->flags = (hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
		     ((hop_start & 0x07U) << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	if (mesh->want_ack) {
		hdr->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	}
	if (mesh->via_mqtt) {
		hdr->flags |= MESHTASTIC_FLAGS_VIA_MQTT;
	}
	if (mesh->pki_encrypted) {
		/* PKC-encrypted DM (the phone did the X25519+AES): the wire channel-hash
		 * byte is the 0x00 PKC marker so peers route it to PKI decryption, not
		 * channel decryption. Without this a private DM goes out stamped with the
		 * primary channel hash (e.g. LongFast) — peers try the channel key, it is
		 * neither private nor reliably delivered, and interop breaks. Matches the
		 * decoded-path PKI branch in meshtastic_build_wire_packet. */
		hdr->channel = 0x00U;
	} else if (mesh->channel < MESHTASTIC_MAX_CHANNELS) {
		/* Our own radio's hash. This pre-encrypted path always leaves by this
		 * board's radio: mt_ws.tx_radio is set only around the encode path
		 * (mt_ws_build_wire_locked), never here, so the worker-preset hash that
		 * used to sit in a branch here could not be reached (agents-ls39.1.10).
		 * A worker is chosen for a send on the encode path, which hashes for it. */
		hdr->channel = meshtastic_channels_get_hash((uint8_t)mesh->channel);
	} else {
		hdr->channel = (mesh->channel != 0U) ? (uint8_t)mesh->channel : mt.ch_hash;
	}
	uint8_t nh = mesh->next_hop;
	uint8_t rn = mesh->relay_node;

	/* Next-hop routing (increment 2): the app/PKC path arrives already
	 * encrypted and bypasses the mesh-native send engine, so stamp our relayer byte +
	 * learned next hop here. hdr->dest/src are already normalized above. */
	meshtastic_router_stamp_originated(sys_le32_to_cpu(hdr->dest), sys_le32_to_cpu(hdr->src),
					   &nh, &rn);
	hdr->next_hop = nh;
	hdr->relay_node = rn;
	memcpy(mt_ws.wire + MESHTASTIC_HDR_LEN, mesh->encrypted.bytes, encrypted_len);

	{
		uint8_t wire[MESHTASTIC_PKT_MAX];
		const uint32_t wire_len = MESHTASTIC_HDR_LEN + encrypted_len;

		memcpy(wire, mt_ws.wire, wire_len);
		k_mutex_unlock(&mt_ws.lock);
		/* From here on mt_ws.wire is another sender's: read the header from our
		 * copy (agents-ls39.1.10). */
		hdr = (struct meshtastic_wire_header *)wire;

		ret = meshtastic_radio_send_wire_wait(wire, wire_len, K_FOREVER);

		if (ret == 0) {
#if defined(CONFIG_MESHTASTIC_MQTT)
			struct meshtastic_packet tx_packet = {
				.from = (mesh->from != 0U) ? mesh->from : mt.node_id,
				.to = (mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST,
				.id = sys_le32_to_cpu(hdr->id),
				.hop_limit = hop_limit,
				.hop_start = hop_start,
				.channel = hdr->channel,
				.want_ack = mesh->want_ack,
				.via_mqtt = mesh->via_mqtt,
				.next_hop = nh,
				.relay_node = rn,
			};

			/* Pre-encrypted PKC send: no decoded MeshPacket in hand — the mqtt
			 * uplink falls back to the wire (encrypted) / struct path. */
			meshtastic_mqtt_on_tx(&tx_packet, wire, wire_len, NULL);
#endif
		}
	}

	return ret;
}
