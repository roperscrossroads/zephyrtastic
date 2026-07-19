/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Duplicate filtering, delivery, relay, and gateway injection paths.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/nodedb.h>

#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#include "meshtastic_channels.h"
#include "meshtastic_contention.h"
#include "meshtastic_core.h"
#include "meshtastic_modules.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"

#include "meshtastic_mqtt.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"
#include "meshtastic_sched.h"

#if defined(CONFIG_MESHTASTIC_AIRTIME)
#include "meshtastic_airtime.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

enum dup_verdict {
	DUP_NEW = 0,  /* first sighting — process normally */
	DUP_SEEN,     /* plain duplicate — drop */
	DUP_UPGRADE,  /* duplicate with strictly more hops left — relay once */
};

static enum dup_verdict dup_check_hops(uint32_t src, uint32_t id, uint8_t hop_limit,
				       bool allow_upgrade)
{
	/* Single scalar, captured once for the whole scan — a direct atomic read is
	 * sufficient (see the concurrency note in meshtastic_sched.h). */
	uint32_t ttl_ms = (uint32_t)meshtastic_sched_get()->dedup_ttl_sec * 1000U;
	uint32_t now = k_uptime_get_32();
	bool saw_expired = false;

	/* Scan the whole cache: a fresh match wins over any stale copy of the
	 * same (src,id), so an expired entry can never mask a real duplicate. */
	for (int i = 0; i < CONFIG_MESHTASTIC_DUP_CACHE_SIZE; i++) {
		if (mt.dup_cache[i].src != src || mt.dup_cache[i].id != id) {
			continue;
		}
		if (ttl_ms != 0U && (now - mt.dup_cache[i].ms) > ttl_ms) {
			saw_expired = true;
			continue;
		}
		if (allow_upgrade && hop_limit > mt.dup_cache[i].hop_limit) {
			/* A later copy with more hops left reaches further than the
			 * one we already handled (upstream PacketHistory hop
			 * upgrade). Remember the higher budget so this fires once. */
			mt.dup_cache[i].hop_limit = hop_limit;
			return DUP_UPGRADE;
		}
		mt.status.duplicate_packets++;
		return DUP_SEEN;
	}

	if (saw_expired) {
		meshtastic_sched_stat_dedup_expired();
	}

	return DUP_NEW;
}

static bool dup_check(uint32_t src, uint32_t id)
{
	return dup_check_hops(src, id, 0U, false) != DUP_NEW;
}

static struct meshtastic_dup_entry *dup_find(uint32_t src, uint32_t id)
{
	for (int i = 0; i < CONFIG_MESHTASTIC_DUP_CACHE_SIZE; i++) {
		if (mt.dup_cache[i].src == src && mt.dup_cache[i].id == id) {
			return &mt.dup_cache[i];
		}
	}
	return NULL;
}

/* Record that we transmitted a relay of this (src,id), so a later duplicate can
 * tell whether a peer relayed the same frame we already put on air. */
static void dup_mark_relayed(uint32_t src, uint32_t id)
{
	struct meshtastic_dup_entry *e = dup_find(src, id);

	if (e != NULL) {
		e->relayed = true;
		e->relayed_ms = k_uptime_get_32();
	}
	meshtastic_sched_stat_relay_sent();
}

enum relay_dupe_action {
	RELAY_DUPE_CANCEL = 0, /* drop our queued relay */
	RELAY_DUPE_KEEP,       /* leave it exactly as scheduled */
	RELAY_DUPE_LATE,       /* re-schedule it to the back of the window */
};

static enum relay_dupe_action relay_dupe_action(uint32_t from, uint32_t to);

/*
 * Flood-redundancy measurement.
 *
 * We relay immediately and synchronously, with no delay and no way to cancel a
 * queued relay, so every node that hears a broadcast transmits. This counts the
 * cases where that was redundant: we relayed (src,id), and afterwards heard a
 * *peer* relay the same frame.
 *
 * Two things are deliberately excluded, because counting them would inflate the
 * result and argue for work that would not actually help:
 *   - relay_node == 0 or == our own low byte: not another node's relay.
 *   - hop_start == hop_limit: the frame came straight from the originator, i.e.
 *     a reliable-delivery retransmission rather than a relay.
 */
static void note_possible_redundant_relay(const struct meshtastic_wire_header *hdr, uint32_t src,
					  uint32_t id, uint8_t rx_hop_limit, int8_t snr)
{
	const struct meshtastic_dup_entry *e = dup_find(src, id);
	uint8_t hop_start;
	uint32_t gap;

	if (e == NULL || !e->relayed) {
		return;
	}

	if (hdr->relay_node == 0U || hdr->relay_node == (uint8_t)(mt.node_id & 0xFFU)) {
		return;
	}

	hop_start = (hdr->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
		    MESHTASTIC_FLAGS_HOP_START_SHIFT;
	if (hop_start != 0U && hop_start == rx_hop_limit) {
		return; /* originator retransmission, not a relay */
	}

	gap = k_uptime_get_32() - e->relayed_ms;
	meshtastic_sched_stat_relay_redundant(gap);
	LOG_DBG("Redundant relay: peer 0x%02x also relayed (src=0x%08x id=0x%08x) %u ms after us",
		hdr->relay_node, src, id, gap);

	/* Our own relay may still be sitting in its contention window; what we do
	 * with it depends on the role. Only reachable for a LoRa duplicate — this
	 * runs on the LoRa RX path, matching the reference's TRANSPORT_LORA gate.
	 * Nothing happens when our copy is already on air, the common case for a
	 * short window. */
	switch (relay_dupe_action(src, sys_le32_to_cpu(hdr->dest))) {
	case RELAY_DUPE_CANCEL:
		if (meshtastic_outbound_cancel(src, id) > 0) {
			meshtastic_sched_stat_relay_cancelled();
			LOG_DBG("Cancelled our queued relay of (src=0x%08x id=0x%08x)", src, id);
		}
		break;
	case RELAY_DUPE_LATE: {
		/* A fresh worst-case delay from now, so our copy lands after every
		 * peer that is still working through its own window. */
		uint32_t late_ms = meshtastic_contention_delay_relay_worst_ms(
			snr, meshtastic_contention_effective_slot_ms(mt.modem.spread_factor,
								     mt.modem.bandwidth_hz, false));

		if (meshtastic_outbound_defer_late(src, id, late_ms) > 0) {
			meshtastic_sched_stat_relay_deferred_late();
			LOG_DBG("Deferred our relay of (src=0x%08x id=0x%08x) to +%u ms", src, id,
				late_ms);
		}
		break;
	}
	case RELAY_DUPE_KEEP:
	default:
		break;
	}
}

static void dup_add(uint32_t src, uint32_t id, uint8_t hop_limit)
{
	mt.dup_cache[mt.dup_head].src = src;
	mt.dup_cache[mt.dup_head].id = id;
	mt.dup_cache[mt.dup_head].ms = k_uptime_get_32();
	mt.dup_cache[mt.dup_head].hop_limit = hop_limit;
	/* Ring slots are reused: clear the relay marks or a new (src,id) inherits
	 * the previous occupant's and mis-attributes a redundant relay. */
	mt.dup_cache[mt.dup_head].relayed = false;
	mt.dup_cache[mt.dup_head].relayed_ms = 0U;
	mt.dup_head = (uint8_t)((mt.dup_head + 1U) % CONFIG_MESHTASTIC_DUP_CACHE_SIZE);
}

#if defined(CONFIG_MESHTASTIC_PACKET_HEXDUMP)
static void log_wire_rx(const uint8_t *pkt, int len, int16_t rssi, int8_t snr)
{
	const struct meshtastic_wire_header *hdr = (const struct meshtastic_wire_header *)pkt;

	LOG_DBG("LoRa RX %08x->%08x id=%08x ch=0x%02x len=%d rssi=%d snr=%d",
		(unsigned int)sys_le32_to_cpu(hdr->src), (unsigned int)sys_le32_to_cpu(hdr->dest),
		(unsigned int)sys_le32_to_cpu(hdr->id), hdr->channel, len, (int)rssi, (int)snr);
	LOG_HEXDUMP_DBG(pkt, len, "LoRa RX");
}
#endif /* CONFIG_MESHTASTIC_PACKET_HEXDUMP */

/* ROUTER, ROUTER_LATE and CLIENT_BASE are "router-like": infrastructure that
 * carries traffic for others rather than merely participating. The reference
 * groups them the same way for hop-limit preservation. */
static bool role_is_router_like(meshtastic_Config_DeviceConfig_Role role)
{
	return role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
	       role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE ||
	       role == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE;
}

/*
 * Whether to decrement hop_limit when relaying. Ports the reference
 * Router::shouldDecrementHopLimit().
 *
 * Normally every relay decrements, which is what bounds a flood. The exception
 * is a hop between two router-like nodes that trust each other: preserving the
 * count there lets a backbone carry traffic further without spending the
 * sender's hop budget on infrastructure links.
 *
 * This is the only wire-visible piece of the role work — it changes hop_limit
 * on frames we put on air — so every condition is a reason to decrement unless
 * *all* of them say otherwise:
 *
 *   - The first hop always decrements. Preserving there would let a packet
 *     leave its originator with more reach than it asked for, and the reference
 *     notes it also breaks retry handling.
 *   - We must be router-like ourselves.
 *   - The previous relayer must resolve UNAMBIGUOUSLY from its last byte. The
 *     relay_node field is one byte of a 32-bit node number, so on a dense mesh
 *     it collides; a "first match wins" scan would preserve hops for whichever
 *     node happened to sort first. Ambiguous or unknown means decrement.
 *   - That resolved node must be a favourite, have a real User record, and be
 *     router-like itself. Favourite is what makes it "trusted" rather than
 *     merely "claims to be a router".
 */
static bool relay_should_decrement_hop_limit(const struct meshtastic_wire_header *hdr,
					     uint8_t hop_limit)
{
	uint8_t hop_start = (hdr->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
			    MESHTASTIC_FLAGS_HOP_START_SHIFT;
	struct meshtastic_nodedb_node peer;
	uint32_t resolved;

	/* hops_away == 0: straight from the originator. */
	if (hop_start == 0U || hop_start <= hop_limit) {
		return true;
	}

	if (!role_is_router_like(meshtastic_device_role())) {
		return true;
	}

	resolved = meshtastic_nodedb_resolve_unique_last_byte(hdr->relay_node);
	if (resolved == 0U) {
		return true; /* ambiguous or unknown — the safe default */
	}

	if (meshtastic_nodedb_get(resolved, &peer) != 0) {
		return true;
	}

	if (peer.is_favorite && peer.has_user && role_is_router_like(peer.role)) {
		LOG_DBG("Preserving hop_limit: relayer 0x%02x resolved to favourite router 0x%08x",
			hdr->relay_node, resolved);
		return false;
	}

	return true;
}

/* True when this node relays without waiting out the client offset. Mirrors the
 * reference shouldRebroadcastEarlyLikeRouter(): ROUTER only. A router is
 * infrastructure — its relay is the one most worth having, so it is given
 * priority over every client's. */
static bool relay_early_like_router(void)
{
	return meshtastic_device_role() == meshtastic_Config_DeviceConfig_Role_ROUTER;
}

/*
 * What to do with our own queued relay when we hear a peer relay the same frame.
 *
 * The reference expresses this as two independent conditionals in
 * perhapsCancelDupe() — a roleAllowsCancelingDupe() gate on cancelling, then
 * separate ROUTER_LATE and CLIENT_BASE-favourite clamps. Written out as one
 * decision the truth table is identical and easier to check:
 *
 *   CLIENT                  -> CANCEL  (save the airtime, the peer covered it)
 *   CLIENT_MUTE             -> CANCEL  (never relays, so nothing is queued)
 *   ROUTER                  -> KEEP    (relay on the original early schedule)
 *   ROUTER_LATE             -> LATE    (still relay, but after everyone else)
 *   CLIENT_BASE, favourite  -> LATE    (router-like, but yields first)
 *   CLIENT_BASE, otherwise  -> CANCEL  (client-like)
 */
static enum relay_dupe_action relay_dupe_action(uint32_t from, uint32_t to)
{
	switch (meshtastic_device_role()) {
	case meshtastic_Config_DeviceConfig_Role_ROUTER:
		return RELAY_DUPE_KEEP;
	case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
		return RELAY_DUPE_LATE;
	case meshtastic_Config_DeviceConfig_Role_CLIENT_BASE:
		return meshtastic_nodedb_is_from_or_to_favorite(from, to) ? RELAY_DUPE_LATE
									 : RELAY_DUPE_CANCEL;
	default:
		return RELAY_DUPE_CANCEL;
	}
}

static void relay_packet(const uint8_t *buf, int len, const struct meshtastic_wire_header *hdr,
			 uint8_t hop_limit, int8_t snr)
{
	uint8_t relay_buf[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *relay_hdr;
	struct meshtastic_contention_plan plan;
	uint8_t out_hop_limit;
	int ret;

	if (len > (int)MESHTASTIC_PKT_MAX || hop_limit == 0U) {
		return;
	}

	memcpy(relay_buf, buf, (size_t)len);
	relay_hdr = (struct meshtastic_wire_header *)relay_buf;
	out_hop_limit = relay_should_decrement_hop_limit(hdr, hop_limit) ? (hop_limit - 1U)
									: hop_limit;
	relay_hdr->flags = (hdr->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
			   (out_hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK);
	/* Stamp ourselves as the relayer (low byte of our node id) so downstream
	 * nodes can attribute the rebroadcast — required for next-hop learning and
	 * loop attribution. */
	relay_hdr->relay_node = (uint8_t)(mt.node_id & 0xFFU);
	/* Onward next hop: our own learned route toward the final destination
	 * (0 = none → flood onward). Also clears the incoming byte so a frame
	 * addressed to us isn't re-addressed to us again. Routes are learned from
	 * unicasts we receive (increment 3, meshtastic_routing_learn_next_hop); an
	 * unlearned destination still resolves to 0 = flood. */
	relay_hdr->next_hop = meshtastic_nodedb_get_next_hop(sys_le32_to_cpu(hdr->dest));

	/* Contention window. wide_lora is false to match how the modem itself is
	 * configured (meshtastic.c resolves the preset with wide_lora=false); if
	 * 2.4 GHz support ever lands, both call sites move together. */
	meshtastic_contention_plan_relay(snr, relay_early_like_router(), mt.modem.spread_factor,
					 mt.modem.bandwidth_hz, false, &plan);

	ret = meshtastic_radio_send_wire_after(relay_buf, (uint32_t)len, MT_SCHED_TIER_NORMAL,
					       plan.delay_ms);
	if (ret < 0) {
		LOG_ERR("Relay TX failed (%d)", ret);
	} else {
		/* Counted as relayed at queue time, not at transmit time. The frame
		 * is committed here — the only thing that can still stop it is an
		 * overhear-cancel, which is precisely what we want the redundancy
		 * counters to measure once that lands. */
		mt.status.relayed_packets++;
		dup_mark_relayed(sys_le32_to_cpu(hdr->src), sys_le32_to_cpu(hdr->id));
		LOG_DBG("Relay queued id=0x%08x after %u ms (cw=%u slot=%u snr=%d)",
			(unsigned int)sys_le32_to_cpu(hdr->id), plan.delay_ms, plan.cw,
			plan.slot_ms, snr);
	}
}

static int relay_injected_encrypted_mesh_packet(const meshtastic_MeshPacket *mesh)
{
	struct meshtastic_wire_header hdr;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	size_t enc_len;
	uint8_t wire_hash;

	if (mesh == NULL || mesh->which_payload_variant != meshtastic_MeshPacket_encrypted_tag) {
		return -EINVAL;
	}

	enc_len = mesh->encrypted.size;
	if (enc_len > MESHTASTIC_PAYLOAD_MAX) {
		return -EINVAL;
	}

	if (mesh->channel < MESHTASTIC_MAX_CHANNELS) {
		wire_hash = meshtastic_channels_get_hash((uint8_t)mesh->channel);
	} else {
		wire_hash = (mesh->channel != 0U) ? (uint8_t)mesh->channel : mt.ch_hash;
	}

	hdr.dest = sys_cpu_to_le32((mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST);
	hdr.src = sys_cpu_to_le32((mesh->from != 0U) ? mesh->from : mt.node_id);
	hdr.id = sys_cpu_to_le32((mesh->id != 0U) ? mesh->id : meshtastic_allocate_packet_id());
	hdr.flags = (mesh->hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
		    ((mesh->hop_start & 0x07U) << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	if (mesh->want_ack) {
		hdr.flags |= MESHTASTIC_FLAGS_WANT_ACK;
	}
	if (mesh->via_mqtt) {
		hdr.flags |= MESHTASTIC_FLAGS_VIA_MQTT;
	}
	hdr.channel = wire_hash;
	hdr.next_hop = mesh->next_hop;
	hdr.relay_node = mesh->relay_node;

	memcpy(wire, &hdr, sizeof(hdr));
	memcpy(wire + MESHTASTIC_HDR_LEN, mesh->encrypted.bytes, enc_len);

	return meshtastic_radio_send_wire(wire, MESHTASTIC_HDR_LEN + (uint32_t)enc_len);
}

static void deliver_packet(const struct meshtastic_packet *packet)
{
	if (mt.recv_cb != NULL) {
		mt.recv_cb(packet->from, packet->to, packet->portnum, packet->payload,
			   packet->payload_len, packet->rssi, packet->snr);
	}

	meshtastic_emit_event(MESHTASTIC_EVENT_PACKET_RECEIVED, 0, packet);
	meshtastic_phoneapi_on_packet(packet);
}

void meshtastic_routing_sniff_rebroadcast(const struct meshtastic_wire_header *hdr,
					  const uint8_t *wire, size_t wire_len,
					  const struct meshtastic_packet *packet)
{
	uint32_t src;
	uint32_t dest;
	uint8_t hop_limit;

	if (hdr == NULL || wire == NULL || wire_len < MESHTASTIC_HDR_LEN || packet == NULL) {
		return;
	}

	src = sys_le32_to_cpu(hdr->src);
	dest = sys_le32_to_cpu(hdr->dest);
	hop_limit = hdr->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK;

	if (!meshtastic_is_rebroadcaster()) {
		return;
	}

	if (src == mt.node_id || dest == mt.node_id) {
		return;
	}

	if (hop_limit == 0U) {
		return;
	}

	if (dest == MESHTASTIC_NODE_BROADCAST && packet->id == 0U) {
		LOG_DBG("Ignore id=0 broadcast relay");
		return;
	}

	if (meshtastic_rebroadcast_mode() ==
	    meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY) {
		return;
	}

	/* Next-hop honor (increment 2): if this frame names a specific next relay
	 * that isn't us, stay quiet — the addressed node relays it, cutting the
	 * duplicate airtime a pure flood would spend. NO_PREF (0) still floods.
	 * Residual: a remote node sharing our low byte also matches here; narrowing
	 * that needs a wider on-wire field (see resolve_unique_last_byte). */
	if (hdr->next_hop != 0U && hdr->next_hop != (uint8_t)(mt.node_id & 0xFFU)) {
		LOG_DBG("Not relaying id=0x%08x: next_hop=0x%02x addressed elsewhere",
			(unsigned int)sys_le32_to_cpu(hdr->id), hdr->next_hop);
		return;
	}

	relay_packet(wire, (int)wire_len, hdr, hop_limit, packet->snr);
}

static meshtastic_Routing_Error decode_fail_to_routing_err(enum meshtastic_decode_fail r)
{
	if (r == MESHTASTIC_DECODE_FAIL_PKI_UNKNOWN_PUBKEY) {
		return meshtastic_Routing_Error_PKI_UNKNOWN_PUBKEY;
	}
	return meshtastic_Routing_Error_NO_CHANNEL;
}

void meshtastic_router_process_lora_rx(const uint8_t *buf, int len, int16_t rssi, int8_t snr)
{
	const struct meshtastic_wire_header *hdr;
	uint32_t src;
	uint32_t pkt_id;
	struct meshtastic_packet packet;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool decoded = false;
	enum meshtastic_decode_fail fail_reason = MESHTASTIC_DECODE_FAIL_NONE;
	int ret;
#if defined(CONFIG_MESHTASTIC_AIRTIME)
	uint32_t airtime_ms;
#endif

	if (buf == NULL || len < (int)MESHTASTIC_HDR_LEN) {
		LOG_DBG("Packet too short (%d bytes)", len);
		return;
	}

	hdr = (const struct meshtastic_wire_header *)buf;
	src = sys_le32_to_cpu(hdr->src);
	pkt_id = sys_le32_to_cpu(hdr->id);

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	airtime_ms = meshtastic_airtime_packet_ms((uint32_t)len);
#endif

#if defined(CONFIG_MESHTASTIC_PACKET_HEXDUMP)
	log_wire_rx(buf, len, rssi, snr);
#endif

	/* Ingress guards, before anything touches the dedup cache or NodeDB
	 * (upstream Router::perhapsHandleReceived order: ignored-node,
	 * broadcast-source, via_mqtt). */
	if (meshtastic_nodedb_is_ignored(src)) {
		LOG_DBG("Ignoring packet from ignored node 0x%08x", src);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (src == MESHTASTIC_NODE_BROADCAST) {
		/* A source claiming the broadcast address is spoofed/broken; it would
		 * poison the dedup cache and NodeDB if processed. */
		LOG_DBG("Ignoring packet with broadcast source");
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_IGNORE_MQTT) &&
	    ((hdr->flags & MESHTASTIC_FLAGS_VIA_MQTT) != 0U)) {
		LOG_DBG("Ignoring packet with via_mqtt set");
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (src == mt.node_id) {
		/* A neighbour rebroadcast one of our own packets: implicit ACK that it
		 * reached the mesh. We never relay or deliver our own echo. The
		 * relayer byte feeds the next-hop learn correlation (M2). */
		meshtastic_routing_note_own_echo(pkt_id, hdr->relay_node);
		meshtastic_reliable_on_implicit_ack(pkt_id);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, airtime_ms);
#endif
		return;
	}

	uint8_t rx_hop_limit = hdr->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK;

	switch (dup_check_hops(src, pkt_id, rx_hop_limit, true)) {
	case DUP_SEEN: {
		uint8_t hop_start = (hdr->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
				    MESHTASTIC_FLAGS_HOP_START_SHIFT;
		bool want_ack = (hdr->flags & MESHTASTIC_FLAGS_WANT_ACK) != 0U;

		/* Repeated-reliable signature: hop_start == hop_limit means this copy
		 * came straight from the originator — it retransmitted because our
		 * first ACK was lost. Re-ACK (no re-delivery) so the sender stops
		 * retrying instead of reporting a false failure. All fields needed are
		 * in the wire header; no decode. */
		if (sys_le32_to_cpu(hdr->dest) == mt.node_id && want_ack && hop_start != 0U &&
		    hop_start == rx_hop_limit) {
			LOG_DBG("Re-ACK repeated reliable id=0x%08x from 0x%08x", pkt_id, src);
			meshtastic_routing_reack_duplicate(src, pkt_id, hdr->channel, rx_hop_limit,
							   hop_start);
		}

		note_possible_redundant_relay(hdr, src, pkt_id, rx_hop_limit, snr);

		LOG_DBG("Duplicate (src=0x%08x id=0x%08x)", src, pkt_id);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}
	case DUP_UPGRADE: {
		/* Already handled once, but this copy has strictly more hops left:
		 * relay the wider-reach copy (policy gates still apply), never
		 * re-deliver. The payload stays undecoded. */
		struct meshtastic_packet upgraded = {
			.from = src,
			.to = sys_le32_to_cpu(hdr->dest),
			.id = pkt_id,
		};

		note_possible_redundant_relay(hdr, src, pkt_id, rx_hop_limit, snr);

		LOG_DBG("Hop-upgraded duplicate (src=0x%08x id=0x%08x hops=%u): relay", src,
			pkt_id, rx_hop_limit);
		meshtastic_routing_sniff_rebroadcast(hdr, buf, (size_t)len, &upgraded);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}
	case DUP_NEW:
		break;
	}

	dup_add(src, pkt_id, rx_hop_limit);

	ret = meshtastic_try_decode_wire_packet(buf, len, rssi, snr, &packet, payload,
						sizeof(payload), &decoded, &fail_reason);
	if (ret < 0) {
		LOG_DBG("RX header parse failed (%d)", ret);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	if (packet.from == 0U) {
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
	} else {
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, airtime_ms);
	}
#endif

	if (!decoded) {
		mt.status.decode_failures++;

		/* A want_ack unicast addressed to us that we can't decode: tell the
		 * sender why (NO_CHANNEL / PKI_UNKNOWN_PUBKEY) so its reliable layer
		 * stops retransmitting and surfaces the reason, instead of timing out.
		 * Only for frames to us (never broadcasts/relays) and only want_ack. */
		if (fail_reason != MESHTASTIC_DECODE_FAIL_NONE && packet.want_ack &&
		    packet.to == mt.node_id) {
			meshtastic_routing_send_error(&packet,
						      decode_fail_to_routing_err(fail_reason));
		}
	}

	mt.status.rx_packets++;
	mt.status.last_rx_from = src;
	mt.status.last_rssi = rssi;
	mt.status.last_snr = snr;

	meshtastic_handle_inbound_packet(&packet, buf, (size_t)len, decoded);
}

static void log_inject_mesh_packet(const char *phase, const meshtastic_MeshPacket *mesh)
{
	unsigned int portnum = 0U;
	const char *payload_kind = "unknown";
	size_t enc_len = 0U;

	if (mesh == NULL) {
		LOG_DBG("inject %s: (null mesh)", phase);
		return;
	}

	if (mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		payload_kind = "decoded";
		portnum = (unsigned int)mesh->decoded.portnum;
	} else if (mesh->which_payload_variant == meshtastic_MeshPacket_encrypted_tag) {
		payload_kind = "encrypted";
		enc_len = mesh->encrypted.size;
	} else if (mesh->encrypted.size > 0U) {
		payload_kind = "encrypted?";
		enc_len = mesh->encrypted.size;
	}

	LOG_DBG("inject %s: 0x%08x->0x%08x id=0x%08x port=%u %s enc=%zu hop=%u/%u ch=0x%02x "
		"via_mqtt=%d",
		phase, mesh->from, mesh->to, mesh->id, portnum, payload_kind, enc_len,
		mesh->hop_limit, mesh->hop_start, mesh->channel, mesh->via_mqtt ? 1 : 0);
}

/* True if a mesh packet carries a visible ADMIN_APP payload.
 *
 * Admin never legitimately arrives over a downlink: the broker is not a mesh
 * peer and the packet's channel is forced to primary on the way in, so the
 * remote-admin dispatcher would authorize it by channel *name* alone — an
 * identity-less gate reachable by any peer on a plaintext or bridged broker.
 * Upstream rejects admin on the downlink path; so do we, for the relay onto RF
 * as well as for local delivery (relaying someone else's admin traffic onto our
 * physical mesh is the same exposure one hop removed).
 *
 * Only the decoded case is visible here. An encrypted admin frame is caught
 * after decrypt, and admin.c independently refuses the identity-less channel
 * gate for any via_mqtt packet.
 */
static bool inject_is_admin(const meshtastic_MeshPacket *mesh)
{
	return mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
	       mesh->decoded.portnum == (meshtastic_PortNum)MESHTASTIC_PORT_ADMIN;
}

int meshtastic_inject_downlink_mesh_packet(const meshtastic_MeshPacket *mesh)
{
	meshtastic_MeshPacket work;
	struct meshtastic_packet packet;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool local;
	bool relay;
	bool decoded = false;
	int ret;

	if (mesh == NULL || !mt.initialized) {
		LOG_DBG("inject rejected: mesh=%p initialized=%d", (void *)mesh, mt.initialized);
		return -EINVAL;
	}

	meshtastic_mesh_packet_copy(&work, mesh);
	work.via_mqtt = true;
	work.transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT;

	log_inject_mesh_packet("entry", &work);

	if (inject_is_admin(&work)) {
		LOG_WRN("inject rejected: ADMIN_APP is not accepted from a downlink "
			"(0x%08x->0x%08x id=0x%08x)",
			work.from, work.to, work.id);
		return -EACCES;
	}

	if (dup_check(work.from, work.id)) {
		LOG_DBG("inject duplicate (src=0x%08x id=0x%08x)", work.from, work.id);
		return -EALREADY;
	}

	dup_add(work.from, work.id, (uint8_t)(work.hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK));

	local = (work.to == mt.node_id || work.to == MESHTASTIC_NODE_BROADCAST);
	relay = (work.to != mt.node_id &&
		 (work.hop_limit > 0U ||
		  (work.hop_limit == 0U &&
		   work.which_payload_variant == meshtastic_MeshPacket_encrypted_tag)));

	LOG_DBG("inject plan: local=%d relay=%d (node=0x%08x)", local, relay, mt.node_id);

	if (relay) {
		if (work.hop_limit == 0U &&
		    work.which_payload_variant == meshtastic_MeshPacket_encrypted_tag) {
			LOG_DBG("inject relaying terminal MQTT hop onto LoRa (hop_limit=0)");
			ret = relay_injected_encrypted_mesh_packet(&work);
		} else {
			LOG_DBG("inject relaying onto LoRa (hop_limit=%u)", work.hop_limit);
			ret = meshtastic_send_mesh_pb(&work);
		}
		if (ret < 0) {
			LOG_DBG("inject relay TX failed (%d)", ret);
			return ret;
		}

		LOG_DBG("inject relay TX ok");
	}

	if (!local) {
		LOG_DBG("inject done (relayed only, not for us)");
		return 0;
	}

	if (work.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		decoded = true;
	} else {
		ret = meshtastic_mesh_pb_try_decode(&work);
		if (ret < 0) {
			if (ret == -ENOTSUP) {
				LOG_DBG("inject ignored: packet carries no payload (e.g. "
					"ACK/control)");
			} else {
				LOG_DBG("inject decode failed (%d), dropping local delivery", ret);
			}
			return 0;
		}

		decoded = true;

		/* The payload was opaque at ingest; now that it has decrypted,
		 * re-apply the ADMIN_APP rejection above. */
		if (inject_is_admin(&work)) {
			LOG_WRN("inject rejected: encrypted downlink decrypted to ADMIN_APP "
				"(0x%08x->0x%08x id=0x%08x)",
				work.from, work.to, work.id);
			return -EACCES;
		}
	}

	ret = meshtastic_mesh_pb_to_packet(&work, &packet, payload, sizeof(payload));
	if (ret < 0) {
		LOG_DBG("inject mesh_pb_to_packet failed (%d)", ret);
		return ret;
	}

	packet.via_mqtt = true;

	LOG_DBG("inject delivering locally port=%u payload_len=%zu", (unsigned int)packet.portnum,
		packet.payload_len);
	meshtastic_handle_inbound_packet(&packet, NULL, 0U, decoded);
	LOG_DBG("inject done (local delivery)");

	return 0;
}

void meshtastic_handle_inbound_packet(const struct meshtastic_packet *packet, const uint8_t *wire,
				      size_t wire_len, bool decoded)
{
	const struct meshtastic_wire_header *hdr = NULL;
	bool suppress_relay = false;

	if (packet == NULL) {
		return;
	}

	if (wire != NULL && wire_len >= MESHTASTIC_HDR_LEN) {
		hdr = (const struct meshtastic_wire_header *)wire;
	}

	if (decoded) {
		LOG_INF("RX from 0x%08x to 0x%08x port=%u len=%zu ch_idx=%u", packet->from,
			packet->to, (unsigned int)packet->portnum, packet->payload_len,
			packet->channel_index);

		if (meshtastic_rebroadcast_mode() ==
			    meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY &&
		    packet->portnum != MESHTASTIC_PORT_TEXT_MESSAGE &&
		    packet->portnum != MESHTASTIC_PORT_POSITION &&
		    packet->portnum != MESHTASTIC_PORT_NODEINFO &&
		    packet->portnum != MESHTASTIC_PORT_ROUTING &&
		    packet->portnum != MESHTASTIC_PORT_TELEMETRY) {
			LOG_DBG("CORE_PORTNUMS_ONLY: drop port %u", (unsigned int)packet->portnum);
			/* The mode suppresses this portnum: not delivered above, and not
			 * relayed either (upstream's skipHandle covers both with one gate).
			 * Encrypted relays (the !decoded branch) are unaffected. */
			suppress_relay = true;
		} else if (packet->to == mt.node_id || packet->to == MESHTASTIC_NODE_BROADCAST) {
#if defined(CONFIG_MESHTASTIC_ADMIN)
			/* Remote admin: an ADMIN_APP unicast to us from another node is
			 * authorized + applied on the mesh (PKC admin_key / passkey), not
			 * delivered to the phone as an ordinary RX packet. */
			if (packet->portnum == MESHTASTIC_PORT_ADMIN && packet->to == mt.node_id &&
			    packet->from != mt.node_id) {
				meshtastic_admin_handle_remote(packet);
			} else
#endif
			{
				deliver_packet(packet);
			}
		}

#if defined(CONFIG_MESHTASTIC_MQTT)
		meshtastic_mqtt_on_rx(packet, wire, wire_len);
#endif
		meshtastic_routing_on_decoded(packet);
		meshtastic_dispatch_modules(packet);
		/* After module dispatch: the NodeDB has now created/refreshed the
		 * source entry, so a learned next hop has somewhere to land. */
		meshtastic_routing_learn_next_hop(packet);
	} else if (hdr != NULL) {
		LOG_DBG("RX encrypted relay 0x%08x->0x%08x id=0x%08x", packet->from, packet->to,
			packet->id);
	}

	if (hdr != NULL && !suppress_relay) {
		meshtastic_routing_sniff_rebroadcast(hdr, wire, wire_len, packet);
	}
}

void meshtastic_router_stamp_originated(uint32_t to, uint32_t from, uint8_t *next_hop,
					uint8_t *relay_node)
{
	/* Only stamp directed unicasts this node actually sources. Broadcasts carry
	 * no next-hop preference, and a packet we relay for someone else keeps the
	 * originator's routing fields untouched (from != our id, or relay_node
	 * already set by the relay path). */
	if (to == MESHTASTIC_NODE_BROADCAST || from != mt.node_id) {
		return;
	}

	if (relay_node != NULL && *relay_node == 0U) {
		*relay_node = (uint8_t)(mt.node_id & 0xFFU);
	}
	if (next_hop != NULL && *next_hop == 0U) {
		*next_hop = meshtastic_nodedb_get_next_hop(to);
	}
}
