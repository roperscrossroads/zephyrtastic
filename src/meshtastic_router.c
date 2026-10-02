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
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
#include "meshtastic_attachment.h"
#endif
#include "meshtastic_channels.h"
#include "meshtastic_contention.h"
#include "meshtastic_core.h"
#include "meshtastic_modules.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"

#include "meshtastic_mqtt.h"
#if defined(CONFIG_MESHTASTIC_CLUSTER)
#include "meshtastic_cluster.h"
#endif
#include "meshtastic_phoneapi.h"
#include "meshtastic_reliable.h"
#if defined(CONFIG_MESHTASTIC_RELAY) || defined(CONFIG_MESHTASTIC_RELAY_EAR)
#include "meshtastic_relay.h"
#endif
#include "meshtastic_router.h"
#if defined(CONFIG_MESHTASTIC_TRAFFIC)
#include "meshtastic_traffic.h"
#endif
#if defined(CONFIG_MESHTASTIC_XEDDSA)
#include "meshtastic_xeddsa.h"
#endif
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
			/* The entry's time is the first copy's; this copy is another
			 * transmission, so the entry no longer measures a link. */
			mt.dup_cache[i].attach = 0xFFU;
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
 * Relays now flow through the contention window and can be cancelled or
 * late-deferred when we overhear a peer's copy first (see relay_dupe_action).
 * This counts the cases where a relay we still committed turned out redundant:
 * we relayed (src,id), and afterwards heard a *peer* relay the same frame. The
 * common residual case is our copy already being on air when the peer's arrives.
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
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
	if (e->relay_attach != 0U) {
		/* Our relay sits on a head. The head cancels or clamps itself on a
		 * copy IT hears; this copy reached us by another radio, so tell it
		 * -- for CANCEL. LATE and KEEP still relay: nothing to say. */
		if (relay_dupe_action(src, sys_le32_to_cpu(hdr->dest)) == RELAY_DUPE_CANCEL &&
		    meshtastic_attachment_cancel(e->relay_attach, src, id) == 0) {
			meshtastic_sched_stat_relay_cancelled();
			LOG_DBG("Cancelled our relay of (src=0x%08x id=0x%08x) on attachment %u",
				src, id, (unsigned int)e->relay_attach);
		}
		return;
	}
#endif
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

static void dup_add(uint32_t src, uint32_t id, uint8_t hop_limit, uint8_t attach,
		    uint8_t relay_node, uint32_t attach_rx_ms)
{
	mt.dup_cache[mt.dup_head].relay_attach = 0U;
	mt.dup_cache[mt.dup_head].attach_rx_ms = attach_rx_ms;
	mt.dup_cache[mt.dup_head].src = src;
	mt.dup_cache[mt.dup_head].id = id;
	mt.dup_cache[mt.dup_head].ms = k_uptime_get_32();
	mt.dup_cache[mt.dup_head].hop_limit = hop_limit;
	mt.dup_cache[mt.dup_head].attach = attach;
	mt.dup_cache[mt.dup_head].relay_node = relay_node;
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
	/* Deep log-stack formatter (~256 B) — gated + off by default; see the
	 * stack-overflow warning on CONFIG_MESHTASTIC_PACKET_HEXDUMP. */
	LOG_HEXDUMP_DBG(pkt, len, "LoRa RX");
}
#endif /* CONFIG_MESHTASTIC_PACKET_HEXDUMP */

/* May a frame that arrived on @p bearer be flood-relayed on THIS board's radio?
 * Only a frame this radio heard: a relay goes back out on the air it came in
 * on, and until P3 gives the outbound path a target attachment, a head's frame
 * (on another preset, another radio) has nowhere to go here -- it is delivered
 * and counted, never relayed locally. Both relay sites (first copy and the
 * hop-upgraded duplicate) use this one predicate (review F1). The peer-link
 * bearer is not RF at all: the link-local rule (agents-xhli.2). */
static bool relay_local_ok(enum meshtastic_bearer bearer)
{
	return bearer == MESHTASTIC_BEARER_LORA;
}

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
			 uint8_t hop_limit, int8_t snr, uint8_t attach)
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

#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
	if (attach != 0U) {
		/* Through the head that heard it (ATTACHMENT-DESIGN §12): the brain
		 * decides WHETHER and by what rule; the head decides WHEN, on its own
		 * clock of reception, with its own CAD. The window is planned with
		 * the HEAD's modem and the SNR the HEAD heard the frame at. */
		const struct meshtastic_dup_entry *e = dup_find(sys_le32_to_cpu(hdr->src),
								sys_le32_to_cpu(hdr->id));
		uint8_t sf;
		uint32_t bw;
		uint8_t dupe;

		if (e == NULL || !meshtastic_attachment_modem(attach, &sf, &bw)) {
			LOG_DBG("Not relaying id=0x%08x via attachment %u: no modem/entry",
				(unsigned int)sys_le32_to_cpu(hdr->id), (unsigned int)attach);
			return;
		}
		meshtastic_contention_plan_relay(snr, relay_early_like_router(), sf, bw, false, &plan);
		switch (relay_dupe_action(sys_le32_to_cpu(hdr->src), sys_le32_to_cpu(hdr->dest))) {
		case RELAY_DUPE_KEEP:
			dupe = MESHTASTIC_ATTACHMENT_DUPE_KEEP;
			break;
		case RELAY_DUPE_LATE:
			dupe = MESHTASTIC_ATTACHMENT_DUPE_LATE;
			break;
		default:
			dupe = MESHTASTIC_ATTACHMENT_DUPE_CANCEL;
			break;
		}
		ret = meshtastic_attachment_relay(attach, relay_buf, (size_t)len,
						  sys_le32_to_cpu(hdr->src), sys_le32_to_cpu(hdr->id),
						  e->attach_rx_ms, plan.delay_ms, dupe);
		if (ret < 0) {
			LOG_DBG("Relay via attachment %u failed (%d)", (unsigned int)attach, ret);
		} else {
			struct meshtastic_dup_entry *me = dup_find(sys_le32_to_cpu(hdr->src),
								   sys_le32_to_cpu(hdr->id));

			mt.status.relayed_packets++;
			dup_mark_relayed(sys_le32_to_cpu(hdr->src), sys_le32_to_cpu(hdr->id));
			if (me != NULL) {
				me->relay_attach = attach;
			}
			LOG_DBG("Relay handed to attachment %u id=0x%08x not_before %u ms (cw=%u slot=%u snr=%d)",
				(unsigned int)attach, (unsigned int)sys_le32_to_cpu(hdr->id),
				plan.delay_ms, plan.cw, plan.slot_ms, snr);
		}
		return;
	}
#else
	ARG_UNUSED(attach);
#endif
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

/* Shared body behind the public meshtastic_handle_inbound_packet() (below) and the
 * RF RX path, which passes the decoded MeshPacket so the phone gets it verbatim (C3
 * Phase 2). Forward-declared here because the RX path calls it before its definition. */
/* decoded_mesh is NOT const: the signature gate stamps MeshPacket.xeddsa_signed on it, and
 * that flag MUST be rewritten rather than forwarded. The phone receives this struct verbatim
 * (meshtastic_phoneapi_on_packet does `from->packet = *decoded_mesh`), so an inbound flag set
 * by the sender would otherwise reach the app as "this packet was signed" with nothing having
 * verified anything. Both callers own writable objects. */
static void handle_inbound_impl(const struct meshtastic_packet *packet, const uint8_t *wire,
				size_t wire_len, bool decoded,
				meshtastic_MeshPacket *decoded_mesh,
				enum meshtastic_bearer bearer);

static void deliver_packet(const struct meshtastic_packet *packet,
			   const meshtastic_MeshPacket *decoded_mesh)
{
	if (mt.recv_cb != NULL) {
		mt.recv_cb(packet->from, packet->to, packet->portnum, packet->payload,
			   packet->payload_len, packet->rssi, packet->snr);
	}

	meshtastic_emit_event(MESHTASTIC_EVENT_PACKET_RECEIVED, 0, packet);
#if defined(CONFIG_MESHTASTIC_CLUSTER)
	/* The cluster's broadcasts are fleet protocol, not messages: keep them off
	 * the phone queue (see meshtastic_cluster_is_internal_frame). The module
	 * still gets them through dispatch; only the phone hand-off is skipped. */
	if (!meshtastic_cluster_is_internal_frame(packet))
#endif
	{
		meshtastic_phoneapi_on_packet(packet, decoded_mesh);
	}

	/* Light-sleep governor: a packet delivered to us is activity, so refresh the
	 * min_wake_secs wake window. This is the single delivery choke point; it counts
	 * delivered broadcasts too (the intended min_wake_secs meaning). Narrow to
	 * (packet->to == mt.node_id) here if only direct traffic should hold us awake. */
	meshtastic_power_note_activity();
}

void meshtastic_routing_sniff_rebroadcast(const struct meshtastic_wire_header *hdr,
					  const uint8_t *wire, size_t wire_len,
					  const struct meshtastic_packet *packet,
					  const meshtastic_MeshPacket *mesh)
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

	/* Phase 4b: id comes off the wire header (always present, even on the encrypted
	 * relay path where there is no decoded MeshPacket) -- the same source dedup keys on. */
	if (dest == MESHTASTIC_NODE_BROADCAST && sys_le32_to_cpu(hdr->id) == 0U) {
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

	/* Phase 4b: rx SNR from the decoded MeshPacket when the RF path supplied one
	 * (rx_mesh->rx_snr == (float)packet->snr by construction), else the flat struct on
	 * the encrypted-relay / DUP_UPGRADE / inject paths that carry no MeshPacket. */
	relay_packet(wire, (int)wire_len, hdr, hop_limit,
		     mesh != NULL ? (int8_t)mesh->rx_snr : packet->snr, packet->rx_attach);
}

static meshtastic_Routing_Error decode_fail_to_routing_err(enum meshtastic_decode_fail r)
{
	if (r == MESHTASTIC_DECODE_FAIL_PKI_UNKNOWN_PUBKEY) {
		return meshtastic_Routing_Error_PKI_UNKNOWN_PUBKEY;
	}
	return meshtastic_Routing_Error_NO_CHANNEL;
}

#if defined(CONFIG_MESHTASTIC_AIRTIME)
/* RX airtime is an RF quantity: a frame from a non-LoRa bearer spent no air,
 * so folding it into the duty-cycle ledger would corrupt the budget. */
static void rx_airtime_log(bool rf, enum meshtastic_airtime_type type, uint32_t ms)
{
	if (rf) {
		meshtastic_airtime_log(type, ms);
	}
}
#endif

void meshtastic_router_process_rx(const uint8_t *buf, int len, int16_t rssi, int8_t snr,
				  enum meshtastic_bearer bearer)
{
	const struct meshtastic_rx_meta meta = {
		.bearer = (uint8_t)bearer,
		.attach = 0U,
		.preset = (bearer == MESHTASTIC_BEARER_LORA) ? (uint8_t)mt.modem_preset
							      : MESHTASTIC_PRESET_UNKNOWN,
		.rssi = rssi,
		.snr = snr,
		.rx_ms = (uint32_t)k_uptime_get(),
	};

	meshtastic_router_process_rx_meta(buf, len, &meta);
}

void meshtastic_router_process_rx_meta(const uint8_t *buf, int len,
				       const struct meshtastic_rx_meta *meta)
{
	const enum meshtastic_bearer bearer = (enum meshtastic_bearer)meta->bearer;
	const int16_t rssi = meta->rssi;
	const int8_t snr = meta->snr;
	/* The link-local rule (agents-xhli.2) hangs off this one flag: everything
	 * that describes RF — relay, signal stats, route learning, the MQTT uplink
	 * — happens only for a frame that actually crossed the air. A radio head's
	 * frame did (ATTACHMENT-DESIGN S1); the BLE peer link's did not. */
	const bool rf = meshtastic_bearer_is_rf(bearer);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
	/* The airtime ledger is priced with THIS board's modem and budgets THIS
	 * board's air: a head's frame spent no time on it. */
	const bool lora = (bearer == MESHTASTIC_BEARER_LORA);
#endif
	const struct meshtastic_wire_header *hdr;
	uint32_t src;
	uint32_t pkt_id;
	struct meshtastic_packet packet;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	/* C3 Phase 2: try_decode fills this with the fully decoded MeshPacket (emoji,
	 * rx_time) so the phone gets the frame verbatim. ~430 B on the RX-thread stack --
	 * re-check `kernel thread stacks` high-water at the bench (Phase-0 deferred item). */
	meshtastic_MeshPacket rx_mesh = meshtastic_MeshPacket_init_zero;
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
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (src == MESHTASTIC_NODE_BROADCAST) {
		/* A source claiming the broadcast address is spoofed/broken; it would
		 * poison the dedup cache and NodeDB if processed. */
		LOG_DBG("Ignoring packet with broadcast source");
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_IGNORE_MQTT) &&
	    ((hdr->flags & MESHTASTIC_FLAGS_VIA_MQTT) != 0U)) {
		LOG_DBG("Ignoring packet with via_mqtt set");
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

	if (src == mt.node_id) {
		/* Our own frame arriving over a non-LoRa bearer is not an RF echo:
		 * nothing rebroadcast it, so it proves nothing about the mesh and
		 * must not feed the implicit-ACK / relayer correlation. (v1 peers
		 * never relay, so this is a misbehaving sender — drop it.) */
		if (!rf) {
			LOG_DBG("Own frame id=0x%08x via bearer %u ignored", pkt_id,
				(unsigned int)bearer);
			return;
		}
		/* Our own transmission, heard by ANOTHER of our radios (a head, or
		 * the local radio hearing a head's TX): the relay byte is still ours,
		 * so nobody rebroadcast it -- it is our voice, not an echo, and must
		 * not count as an implicit ACK. A single-radio node can never hear
		 * itself, so this is asked only of frames that came through an
		 * attachment (S1): on the local radio a matching relay byte is a
		 * neighbour that shares our low byte relaying us -- a real implicit
		 * ACK, as upstream treats it (review F7). */
		if (meta->attach != 0U && hdr->relay_node == (uint8_t)(mt.node_id & 0xFFU)) {
			mt.status.self_heard++;
			LOG_DBG("Own frame id=0x%08x heard on attach %u: our own voice",
				pkt_id, (unsigned int)meta->attach);
			return;
		}
		/* A neighbour rebroadcast one of our own packets: implicit ACK that it
		 * reached the mesh. We never relay or deliver our own echo. The
		 * relayer byte feeds the next-hop learn correlation (M2). */
		meshtastic_routing_note_own_echo(pkt_id, hdr->relay_node);
		meshtastic_reliable_on_implicit_ack(pkt_id);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX, airtime_ms);
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
							   hop_start, meta->attach);
		}

		/* Redundancy accounting reasons about peers RELAYING on the air; a
		 * bearer copy proves nothing about RF flooding (and must not cancel
		 * a queued LoRa relay). */
		if (rf) {
			note_possible_redundant_relay(hdr, src, pkt_id, rx_hop_limit, snr);
		}
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
		/* The same air frame, heard first by our own radio and now again
		 * through a head: the two radios heard it at the same instant, so
		 * the gap between the arrivals, on this one clock, is the head's
		 * link latency (ATTACHMENT-SCOPE F5 / R2). Only the local-first
		 * order measures anything -- a head's copy cannot precede the air --
		 * and only a copy of the SAME transmission: a neighbour's rebroadcast
		 * (another relay byte, one hop fewer) heard through the head is
		 * seconds later and says nothing about the link (bench, 22:06Z). */
		if (meta->attach != 0U) {
			const struct meshtastic_dup_entry *e = dup_find(src, pkt_id);

			if (e != NULL && e->attach == 0U && e->hop_limit == rx_hop_limit &&
			    e->relay_node == hdr->relay_node) {
				uint32_t delta = k_uptime_get_32() - e->ms;

				meshtastic_attachment_note_latency(meta->attach, delta);
				/* INF, not DBG: fleet images compile DBG out, and this line
				 * is the bench's per-frame sample (one per diversity frame). */
				LOG_INF("attach %u: link latency %u ms (src=0x%08x id=0x%08x len=%d)",
					(unsigned int)meta->attach, delta, src, pkt_id, len);
			}
		}
#endif

		LOG_DBG("Duplicate (src=0x%08x id=0x%08x)", src, pkt_id);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
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
			.rx_attach = meta->attach,
		};

		if (rf) {
			note_possible_redundant_relay(hdr, src, pkt_id, rx_hop_limit, snr);
		}
		if (relay_local_ok(bearer) ||
		    (IS_ENABLED(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN) && meta->attach != 0U)) {
			LOG_DBG("Hop-upgraded duplicate (src=0x%08x id=0x%08x hops=%u): relay",
				src, pkt_id, rx_hop_limit);
			meshtastic_routing_sniff_rebroadcast(hdr, buf, (size_t)len, &upgraded,
							     NULL);
		}
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}
	case DUP_NEW:
		break;
	}

	dup_add(src, pkt_id, rx_hop_limit, meta->attach, hdr->relay_node, meta->rx_ms);

	ret = meshtastic_try_decode_wire_packet_on(buf, len, rssi, snr, meta->preset, &packet,
						   payload, sizeof(payload), &decoded, &fail_reason,
						   &rx_mesh);
	packet.rx_attach = meta->attach;
	packet.rx_heard_on = (rf && meta->preset != MESHTASTIC_PRESET_UNKNOWN)
				     ? (uint8_t)(meta->preset + 1U)
				     : 0U;
	if (ret < 0) {
		LOG_DBG("RX header parse failed (%d)", ret);
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
#endif
		return;
	}

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	if (packet.from == 0U) {
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX_ALL, airtime_ms);
	} else {
		rx_airtime_log(lora, MESHTASTIC_AIRTIME_RX, airtime_ms);
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
	if (rf) {
		/* "Last heard" RF bookkeeping: a bearer frame carries no signal
		 * measurement, and zeroing these would overwrite the last real
		 * RF reading the bench reads off `meshtastic status`. */
		mt.status.last_rx_from = src;
		mt.status.last_rssi = rssi;
		mt.status.last_snr = snr;
	}

	if (!rf && decoded) {
		/* The conversion stamped TRANSPORT_LORA (its via_mqtt-derived
		 * default). Upstream has no BLE-bearer value; TRANSPORT_API is the
		 * nearest "arrived over a local link, not the air" the phone can
		 * display without being lied to about LoRa. */
		rx_mesh.transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_API;
	}

	/* Pass the decoded MeshPacket (valid only when decoded==true; consumed only on the
	 * deliver path, which runs only for a decoded frame) so the phone sees it verbatim. */
	handle_inbound_impl(&packet, buf, (size_t)len, decoded, decoded ? &rx_mesh : NULL, bearer);
}

void meshtastic_router_process_lora_rx(const uint8_t *buf, int len, int16_t rssi, int8_t snr)
{
	meshtastic_router_process_rx(buf, len, rssi, snr, MESHTASTIC_BEARER_LORA);
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

	dup_add(work.from, work.id, (uint8_t)(work.hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK), 0U,
		0U, 0U);

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

	LOG_DBG("inject delivering locally port=%u payload_len=%u",
		(unsigned int)work.decoded.portnum, (unsigned int)work.decoded.payload.size);
	/* C3 Phase 4b/8d: deliver the decoded injected MeshPacket (work) verbatim via the
	 * RX-currency path — handle_inbound_impl materialises the flat struct from it on demand
	 * for the public boundaries (recv_cb/event), exactly as the RF path does, so Data.emoji
	 * (TXT-1) survives to the phone. We pass packet == NULL: there is no separate struct to
	 * build here (that was a redundant second mesh_pb_to_packet). via_mqtt/TRANSPORT_MQTT are
	 * already set on `work` above; wire==NULL keeps sniff skipped and gates learn_next_hop. */
	/* Bearer LORA is behaviour-neutral here: wire==NULL already skips the sniff,
	 * and via_mqtt/TRANSPORT_MQTT gate route learning + the transport tag. */
	handle_inbound_impl(NULL, NULL, 0U, decoded, decoded ? &work : NULL,
			    MESHTASTIC_BEARER_LORA);
	LOG_DBG("inject done (local delivery)");

	return 0;
}

static void handle_inbound_impl(const struct meshtastic_packet *packet, const uint8_t *wire,
				size_t wire_len, bool decoded,
				meshtastic_MeshPacket *decoded_mesh,
				enum meshtastic_bearer bearer)
{
	const bool rf = meshtastic_bearer_is_rf(bearer);
	const struct meshtastic_wire_header *hdr = NULL;
	const struct meshtastic_packet *pkt = packet;
	struct meshtastic_packet materialized;
	uint8_t mpayload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool suppress_relay = false;
	/* A module answered this request itself; the generic want_ack ACK is then not sent. */
	bool module_answered = false;

	/* C3 Phase 8d: a caller may pass packet == NULL when it hands us a decoded MeshPacket
	 * (the MQTT-downlink inject) — we materialise the struct from it below. Only both-NULL
	 * is nothing to do. */
	if (packet == NULL && decoded_mesh == NULL) {
		return;
	}

	if (wire != NULL && wire_len >= MESHTASTIC_HDR_LEN) {
		hdr = (const struct meshtastic_wire_header *)wire;
	}

	/* Phase 4c: on the RF decoded path, the decoded MeshPacket (decoded_mesh / rx_mesh) is the
	 * primary representation -- materialise the flat struct on demand from it for the consumers
	 * not yet migrated to the MeshPacket (recv_cb via deliver_packet, dispatch_modules, admin,
	 * mqtt), so process_lora_rx can stop building the struct itself (4d). The struct<->MeshPacket
	 * round-trip is lossless (test_c3_struct_mesh_roundtrip_lossless) EXCEPT the wire channel hash:
	 * the MeshPacket stores the resolved channel index, so a PKC frame's 0x00 marker would come
	 * back as a channel hash and misroute the reply modules build from req->channel. Restore it
	 * from the wire header, which is authoritative and always present here (identical to what
	 * try_decode sets). On the encrypted-relay / public inject / test paths there is no MeshPacket,
	 * so pkt stays the passed-in struct. */
	if (decoded && decoded_mesh != NULL &&
	    meshtastic_mesh_pb_to_packet(decoded_mesh, &materialized, mpayload, sizeof(mpayload)) == 0) {
		if (hdr != NULL) {
			materialized.channel = hdr->channel;
		}
		/* The MeshPacket has no notion of which of our radios heard it;
		 * the flat struct does. Carry it across (SCOPE E1/E2). */
		materialized.rx_attach = (packet != NULL) ? packet->rx_attach : 0U;
		materialized.rx_heard_on = (packet != NULL) ? packet->rx_heard_on : 0U;
		pkt = &materialized;
	}

	/* If the caller gave us only a decoded MeshPacket (packet == NULL) and materialising it
	 * failed, there is no struct to fall back on — nothing to deliver. */
	if (pkt == NULL) {
		return;
	}

	/* Never forward a sender's claim that its packet was signed. The phone gets this
	 * MeshPacket verbatim, so an inbound xeddsa_signed=true would show in the app as a
	 * verified packet with nothing having verified it. Cleared unconditionally -- a build
	 * with no verifier is the one that needs this most, which is why it sits outside the
	 * guard below (the TX-side clear in meshtastic_packet.c is outside it for the mirror
	 * image of this reason). The gate below re-sets it only for a signature it checked. */
	if (decoded && decoded_mesh != NULL) {
		decoded_mesh->xeddsa_signed = false;
	}

#if defined(CONFIG_MESHTASTIC_XEDDSA)
	/* Signature policy (reference: checkXeddsaReceivePolicy, called from perhapsDecode --
	 * i.e. before any module sees the packet). A bad or malformed signature is dropped
	 * here, so a forgery reaches neither a module, nor the phone, nor the rest of the mesh
	 * through our relay. Only decoded packets have a signature to check: an encrypted
	 * relay carries a payload we cannot read, and dropping those would make a verifying
	 * node a black hole for every channel it does not hold. */
	if (decoded && !meshtastic_xeddsa_check_rx_policy(pkt, decoded_mesh)) {
		return;
	}
#endif

#if defined(CONFIG_MESHTASTIC_TRAFFIC)
	/* Traffic management (reference: TrafficManagementModule runs first in
	 * callModules(); STOP consumes the packet -- no delivery, no rebroadcast).
	 * Runs for decoded AND undecodable frames: the unknown-packet filter is
	 * about the latter. */
	if (meshtastic_traffic_inspect(pkt, decoded ? decoded_mesh : NULL, decoded) ==
	    MESHTASTIC_TRAFFIC_DROP) {
		return;
	}
#endif

	if (decoded) {
		LOG_INF("RX from 0x%08x to 0x%08x port=%u len=%zu ch_idx=%u", pkt->from,
			pkt->to, (unsigned int)pkt->portnum, pkt->payload_len,
			pkt->channel_index);

		if (meshtastic_rebroadcast_mode() ==
			    meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY &&
		    pkt->portnum != MESHTASTIC_PORT_TEXT_MESSAGE &&
		    pkt->portnum != MESHTASTIC_PORT_POSITION &&
		    pkt->portnum != MESHTASTIC_PORT_NODEINFO &&
		    pkt->portnum != MESHTASTIC_PORT_ROUTING &&
		    pkt->portnum != MESHTASTIC_PORT_TELEMETRY) {
			LOG_DBG("CORE_PORTNUMS_ONLY: drop port %u", (unsigned int)pkt->portnum);
			/* The mode suppresses this portnum: not delivered above, and not
			 * relayed either (upstream's skipHandle covers both with one gate).
			 * Encrypted relays (the !decoded branch) are unaffected. */
			suppress_relay = true;
		} else if (pkt->to == mt.node_id || pkt->to == MESHTASTIC_NODE_BROADCAST) {
#if defined(CONFIG_MESHTASTIC_ADMIN)
			/* Remote admin: an ADMIN_APP unicast to us from another node is
			 * authorized + applied on the mesh (PKC admin_key / passkey), not
			 * delivered to the phone as an ordinary RX packet. */
			if (pkt->portnum == MESHTASTIC_PORT_ADMIN && pkt->to == mt.node_id &&
			    pkt->from != mt.node_id) {
				/* ...unless it is the answer to a request the phone sent
				 * through us: that one is the phone's (agents-dnr4.33). */
				if (meshtastic_admin_take_solicited_response(pkt, decoded_mesh)) {
					deliver_packet(pkt, decoded_mesh);
				} else {
					module_answered =
						meshtastic_admin_handle_remote(pkt, decoded_mesh);
				}
			} else
#endif
			{
				deliver_packet(pkt, decoded_mesh);
			}
		}

#if defined(CONFIG_MESHTASTIC_MQTT)
		/* Uplinking a peer-link frame to the broker would bridge the
		 * bearers one hop removed — the same graph the no-relay rule
		 * refuses. LoRa-borne traffic only. */
		if (rf) {
			meshtastic_mqtt_on_rx(pkt, wire, wire_len, decoded_mesh);
		}
#endif
		/* One request, one answer: when a module already answered this packet
		 * (a NAK, a response, its own ACK) the generic want_ack ACK is not sent
		 * as well -- reference ReliableRouter::sniffReceived acks only when
		 * !MeshModule::currentReply. Sending both put an ACK NONE ahead of a
		 * remote admin refusal, so a client taking the first answer was told a
		 * refused write succeeded (agents-dnr4.32). An admin packet is never a
		 * ROUTING packet, so skipping this call skips no ACK/NAK bookkeeping. */
		if (!module_answered) {
			meshtastic_routing_on_decoded(pkt, decoded_mesh);
		}
		meshtastic_dispatch_modules(pkt, decoded_mesh);
#if defined(CONFIG_MESHTASTIC_RELAY)
		/* The cross-preset relay (agents-jbrq.12) is the one deliberate way a
		 * bearer frame's CONTENT reaches our air: never the frame itself (the
		 * link-local gate below still holds), only a new text packet from us.
		 * It needs the bearer, which modules are not given. */
		meshtastic_relay_on_rx(pkt, decoded_mesh, bearer);
#endif
#if defined(CONFIG_MESHTASTIC_RELAY_EAR)
		/* The relay's ear: a decoded broadcast heard on LoRa goes, byte for
		 * byte, to the receiving half over the BLE peer link. Only frames
		 * THIS radio heard: a bearer frame -- and an attachment's -- is
		 * someone else's hearing, already forwarded once. */
		if (bearer == MESHTASTIC_BEARER_LORA && wire != NULL) {
			meshtastic_relay_ear_on_rx(pkt, wire, wire_len);
		}
#endif
		/* After module dispatch: the NodeDB has now created/refreshed the
		 * source entry, so a learned next hop has somewhere to land.
		 * Phase 4b: pass rx_mesh (NULL on the public inject/test path -> struct
		 * fallback inside). LoRa only: a bearer frame's relay_node never rode
		 * the air and says nothing about RF topology. And only OUR radio's
		 * air: a next hop learned through a head is a neighbour on the
		 * head's preset, which our radio cannot reach (SCOPE E1). */
		if (rf && pkt->rx_attach == 0U) {
			meshtastic_routing_learn_next_hop(pkt, decoded_mesh);
		}
	} else if (hdr != NULL) {
		LOG_DBG("RX encrypted relay 0x%08x->0x%08x id=0x%08x", pkt->from, pkt->to,
			pkt->id);
	}

	/* A relay goes back out on the air the frame came in on: relay_local_ok()
	 * says whether this board's radio is that air. */
	if (hdr != NULL && !suppress_relay &&
	    (relay_local_ok(bearer) ||
	     (IS_ENABLED(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN) && pkt->rx_attach != 0U))) {
		/* Phase 4b: decoded_mesh is NULL on the encrypted-relay path (no decode) and
		 * the public inject/test path -> struct-fallback for snr inside. Phase 4c: pkt is
		 * the materialised struct on the decoded RF path, else the passed-in struct.
		 * A head's frame relays THROUGH that head (relay_packet, P3 slice 3),
		 * never on our own radio. */
		meshtastic_routing_sniff_rebroadcast(hdr, wire, wire_len, pkt, decoded_mesh);
	}
}

void meshtastic_handle_inbound_packet(const struct meshtastic_packet *packet, const uint8_t *wire,
				      size_t wire_len, bool decoded)
{
	/* Public boundary: callers that hold only the flat struct (locally injected
	 * downlinks, tests) get the to_mesh_pb rebuild on the phone path. The RF RX path
	 * uses handle_inbound_impl() directly to carry the decoded MeshPacket verbatim. */
	handle_inbound_impl(packet, wire, wire_len, decoded, NULL, MESHTASTIC_BEARER_LORA);
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
