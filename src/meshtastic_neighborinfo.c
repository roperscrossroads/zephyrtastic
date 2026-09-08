/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_neighborinfo.h for the design and the divergences. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/config.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic/module_config.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_modules.h"
#include "meshtastic_neighborinfo.h"
#include "meshtastic_phoneapi.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* The protobuf's bound on neighbors per packet (10 upstream, MAX_NUM_NEIGHBORS). */
#define NEIGHBORS_PER_PACKET ARRAY_SIZE(((meshtastic_NeighborInfo *)0)->neighbors)

struct neighbor {
	uint32_t node; /* 0: empty slot */
	float snr;
	int64_t heard_ms;
	uint32_t interval_secs;
};

static K_MUTEX_DEFINE(table_lock);
static struct neighbor table[CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE];

static struct k_work_delayable announce_work;
static int64_t last_reply_ms;
static bool last_reply_valid;

/* ---- settings ------------------------------------------------------------- */

static bool lora_on_default_slot(void)
{
	meshtastic_Config cfg;

	/* channel_num 0 means "derive the slot from the channel name" -- the
	 * reference's uses_default_frequency_slot. */
	return meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg) != 0 ||
	       cfg.which_payload_variant != meshtastic_Config_lora_tag ||
	       cfg.payload_variant.lora.channel_num == 0U;
}

void meshtastic_neighborinfo_settings(struct meshtastic_neighborinfo_settings *out)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	const meshtastic_ModuleConfig_NeighborInfoConfig *cfg = &mod.payload_variant.neighbor_info;

	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_neighbor_info_tag, &mod);

	out->enabled = cfg->enabled;
	out->transmit_over_lora = cfg->transmit_over_lora;

	/* Reference: 0 -> default; below the minimum -> default (not the minimum). */
	if (cfg->update_interval == 0U ||
	    cfg->update_interval < CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC) {
		out->interval_secs = CONFIG_MESHTASTIC_NEIGHBORINFO_INTERVAL_SEC;
	} else {
		out->interval_secs = cfg->update_interval;
	}

	/* Reference runOnce: on the default channel AND the default frequency slot
	 * the broadcast is kept off the air (NODENUM_BROADCAST_NO_LORA) -- that is
	 * the public channel, and everyone's neighbor tables on it are noise. */
	out->lora_allowed =
		cfg->transmit_over_lora &&
		!(meshtastic_channels_is_default(meshtastic_channels_primary_index()) &&
		  lora_on_default_slot());
}

/* ---- table ------------------------------------------------------------------ */

/* Reference cleanUpNeighbors: a neighbor silent for twice its own broadcast
 * interval is forgotten. Called with the lock held. */
static void table_expire_locked(int64_t now_ms)
{
	for (size_t i = 0; i < ARRAY_SIZE(table); i++) {
		int64_t tolerated_ms = (int64_t)table[i].interval_secs * 2 * MSEC_PER_SEC;

		if (table[i].node != 0U && (now_ms - table[i].heard_ms) > tolerated_ms) {
			LOG_DBG("Neighbor 0x%08x silent for >2x%u s, forgotten", table[i].node,
				table[i].interval_secs);
			table[i] = (struct neighbor){0};
		}
	}
}

/* Reference getOrCreateNeighbor. @p interval_hint is the neighbor's own stated
 * interval (from a NeighborInfo it sent), 0 when unknown. */
static void table_remember(uint32_t node, float snr, uint32_t interval_hint, int64_t now_ms)
{
	struct neighbor *slot = NULL;
	struct neighbor *oldest = &table[0];

	k_mutex_lock(&table_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(table); i++) {
		if (table[i].node == node) {
			slot = &table[i];
			break;
		}
		if (table[i].node == 0U && slot == NULL) {
			slot = &table[i];
		}
		if (table[i].heard_ms < oldest->heard_ms) {
			oldest = &table[i];
		}
	}

	if (slot == NULL) {
		LOG_WRN("Neighbor table full, replacing oldest (0x%08x)", oldest->node);
		slot = oldest;
		*slot = (struct neighbor){0};
	}

	if (slot->node != node) {
		struct meshtastic_neighborinfo_settings s;

		/* New: assume our own interval for it until it tells us. */
		meshtastic_neighborinfo_settings(&s);
		slot->node = node;
		slot->interval_secs = s.interval_secs;
	}
	slot->snr = snr;
	slot->heard_ms = now_ms;
	if (interval_hint != 0U) {
		slot->interval_secs = interval_hint;
	}

	k_mutex_unlock(&table_lock);
}

size_t meshtastic_neighborinfo_count(void)
{
	size_t n = 0U;

	k_mutex_lock(&table_lock, K_FOREVER);
	table_expire_locked(k_uptime_get());
	for (size_t i = 0; i < ARRAY_SIZE(table); i++) {
		n += (table[i].node != 0U);
	}
	k_mutex_unlock(&table_lock);

	return n;
}

bool meshtastic_neighborinfo_at(size_t index, struct meshtastic_neighborinfo_entry *out)
{
	int64_t now_ms = k_uptime_get();
	bool present;

	if (index >= ARRAY_SIZE(table) || out == NULL) {
		return false;
	}

	k_mutex_lock(&table_lock, K_FOREVER);
	table_expire_locked(now_ms);
	present = table[index].node != 0U;
	if (present) {
		out->node = table[index].node;
		out->snr = table[index].snr;
		out->age_secs = (uint32_t)((now_ms - table[index].heard_ms) / MSEC_PER_SEC);
		out->interval_secs = table[index].interval_secs;
	}
	k_mutex_unlock(&table_lock);

	return present;
}

void meshtastic_neighborinfo_reset(void)
{
	k_mutex_lock(&table_lock, K_FOREVER);
	memset(table, 0, sizeof(table));
	k_mutex_unlock(&table_lock);
}

/* Reference collectNeighborInfo. */
static void collect(meshtastic_NeighborInfo *info, const struct meshtastic_neighborinfo_settings *s)
{
	uint32_t me = meshtastic_get_node_id();

	*info = (meshtastic_NeighborInfo)meshtastic_NeighborInfo_init_zero;
	info->node_id = me;
	info->last_sent_by_id = me;
	info->node_broadcast_interval_secs = s->interval_secs;

	k_mutex_lock(&table_lock, K_FOREVER);
	table_expire_locked(k_uptime_get());
	for (size_t i = 0; i < ARRAY_SIZE(table) && info->neighbors_count < NEIGHBORS_PER_PACKET;
	     i++) {
		if (table[i].node == 0U || table[i].node == me) {
			continue;
		}
		info->neighbors[info->neighbors_count].node_id = table[i].node;
		info->neighbors[info->neighbors_count].snr = table[i].snr;
		/* last_rx_time / node_broadcast_interval_secs stay 0: local-only
		 * fields, never sent (reference does the same). */
		info->neighbors_count++;
	}
	k_mutex_unlock(&table_lock);
}

static int encode(const meshtastic_NeighborInfo *info, uint8_t *payload, size_t cap, size_t *len)
{
	pb_ostream_t stream = pb_ostream_from_buffer(payload, cap);

	if (!pb_encode(&stream, meshtastic_NeighborInfo_fields, info)) {
		LOG_ERR("NeighborInfo encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}
	*len = stream.bytes_written;
	return 0;
}

/* ---- sending ---------------------------------------------------------------- */

int meshtastic_neighborinfo_send(void)
{
	struct meshtastic_neighborinfo_settings s;
	meshtastic_NeighborInfo info;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_packet pkt = {0};
	size_t len;
	int ret;

	meshtastic_neighborinfo_settings(&s);
	collect(&info, &s);
	if (info.neighbors_count == 0U) {
		return -ENODATA; /* reference: "only send neighbours if we have some" */
	}

	ret = encode(&info, payload, sizeof(payload), &len);
	if (ret < 0) {
		return ret;
	}

	pkt.to = MESHTASTIC_NODE_BROADCAST;
	pkt.portnum = MESHTASTIC_PORT_NEIGHBORINFO;
	pkt.payload = payload;
	pkt.payload_len = len;

	if (s.lora_allowed) {
		/* BG tier by port; K_NO_WAIT puts it behind the airtime gate
		 * (the reference checks isTxAllowedChannelUtil/AirUtil itself). */
		ret = meshtastic_send_packet(&pkt, K_NO_WAIT);
		if (ret == 0) {
			LOG_INF("NeighborInfo broadcast: %u neighbors", info.neighbors_count);
		} else {
			LOG_WRN("NeighborInfo not queued (%d)", ret);
		}
		return ret;
	}

	/* Reference NODENUM_BROADCAST_NO_LORA: the phone still gets the table (an
	 * app or MQTT consumer draws the map from it); the air does not. */
	pkt.from = meshtastic_get_node_id();
	pkt.id = meshtastic_allocate_packet_id();
	meshtastic_phoneapi_on_packet(&pkt, NULL);
	LOG_INF("NeighborInfo to phone only: %u neighbors (%s)", info.neighbors_count,
		s.transmit_over_lora ? "default channel + slot" : "transmit_over_lora off");
	return 0;
}

static void announce_work_fn(struct k_work *work)
{
	struct meshtastic_neighborinfo_settings s;

	ARG_UNUSED(work);

	meshtastic_neighborinfo_settings(&s);
	if (!s.enabled) {
		return;
	}
	(void)meshtastic_neighborinfo_send();
	(void)k_work_reschedule(&announce_work, K_SECONDS(s.interval_secs));
}

void meshtastic_neighborinfo_config_changed(void)
{
	struct meshtastic_neighborinfo_settings s;

	meshtastic_neighborinfo_settings(&s);
	if (s.enabled) {
		/* Reference setIntervalFromNow: the first broadcast is a full interval
		 * after enabling, not immediate -- the table has to fill first. */
		(void)k_work_reschedule(&announce_work, K_SECONDS(s.interval_secs));
		LOG_DBG("NeighborInfo enabled: every %u s, lora %s", s.interval_secs,
			s.lora_allowed ? "on" : "off");
	} else {
		(void)k_work_cancel_delayable(&announce_work);
		LOG_DBG("NeighborInfo disabled");
	}
}

int meshtastic_neighborinfo_set(bool enabled, uint32_t interval_secs, bool transmit_over_lora)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	int ret;

	mod.which_payload_variant = meshtastic_ModuleConfig_neighbor_info_tag;
	mod.payload_variant.neighbor_info.enabled = enabled;
	mod.payload_variant.neighbor_info.update_interval = interval_secs;
	mod.payload_variant.neighbor_info.transmit_over_lora = transmit_over_lora;

	ret = meshtastic_config_store_set_module(&mod);
	if (ret < 0) {
		return ret;
	}
	meshtastic_neighborinfo_config_changed();
	return 0;
}

/* ---- receiving -------------------------------------------------------------- */

/* Same rule as NodeDB's packet_hops_away: hop_start 0 is "unknown" (pre-2.3
 * firmware), never "adjacent". */
static bool heard_directly(const struct meshtastic_packet *packet,
			   const meshtastic_MeshPacket *mesh)
{
	uint8_t hop_start = mesh ? (uint8_t)mesh->hop_start : packet->hop_start;
	uint8_t hop_limit = mesh ? (uint8_t)mesh->hop_limit : packet->hop_limit;

	return hop_start != 0U && hop_limit == hop_start;
}

static void neighborinfo_on_packet(const struct meshtastic_packet *packet,
				   const meshtastic_MeshPacket *mesh)
{
	uint32_t from;
	uint32_t portnum;
	bool via_mqtt;
	float snr;
	uint32_t interval_hint = 0U;

	if (packet == NULL) {
		return;
	}

	from = mesh ? mesh->from : packet->from;
	portnum = mesh ? (uint32_t)mesh->decoded.portnum : packet->portnum;
	via_mqtt = mesh ? mesh->via_mqtt : packet->via_mqtt;
	snr = mesh ? mesh->rx_snr : (float)packet->snr;

	/* Promiscuous, but only what the radio actually heard from an adjacent
	 * node counts. A broker is not a neighbor (reference wantPacket:
	 * !via_mqtt). The table fills even while the module is disabled -- it
	 * costs nothing and `meshtastic neighbors` is useful on its own; only the
	 * broadcast and the reply honour `enabled`. */
	if (from == 0U || from == meshtastic_get_node_id() || via_mqtt ||
	    !heard_directly(packet, mesh)) {
		return;
	}

	if (portnum == MESHTASTIC_PORT_NEIGHBORINFO) {
		meshtastic_NeighborInfo np = meshtastic_NeighborInfo_init_zero;
		const uint8_t *payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
		size_t payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
		pb_istream_t is = pb_istream_from_buffer(payload, payload_len);

		if (payload != NULL && pb_decode(&is, meshtastic_NeighborInfo_fields, &np)) {
			/* Reference: ignore the dummy "single neighbor 0 / snr 0" packet. */
			bool dummy = np.neighbors_count == 1U && np.neighbors[0].node_id == 0U &&
				     np.neighbors[0].snr == 0.0f;

			if (!dummy && np.node_id == from) {
				interval_hint = np.node_broadcast_interval_secs;
			}
			LOG_INF("NeighborInfo from 0x%08x: %u neighbors, every %u s", from,
				np.neighbors_count, np.node_broadcast_interval_secs);
		}
	}

	table_remember(from, snr, interval_hint, k_uptime_get());
}

/* Reference allocReply: answer a want_response with our table, at most once
 * per REPLY_SUPPRESS_SEC whoever asks. The dispatcher already limits this to
 * a NEIGHBORINFO request unicast to us. */
static int neighborinfo_alloc_reply(const struct meshtastic_packet *req,
				    const meshtastic_MeshPacket *mesh,
				    struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_neighborinfo_settings s;
	meshtastic_NeighborInfo info;
	int64_t now_ms = k_uptime_get();
	size_t len;
	int ret;

	ARG_UNUSED(mesh);

	if (req == NULL || reply == NULL || req->from == 0U ||
	    req->from == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	meshtastic_neighborinfo_settings(&s);
	if (!s.enabled) {
		return -ENOENT; /* reference: a disabled module never answers */
	}

	if (last_reply_valid &&
	    (now_ms - last_reply_ms) <
		    (int64_t)CONFIG_MESHTASTIC_NEIGHBORINFO_REPLY_SUPPRESS_SEC * MSEC_PER_SEC) {
		LOG_DBG("NeighborInfo request from 0x%08x suppressed (replied <%d s ago)",
			req->from, CONFIG_MESHTASTIC_NEIGHBORINFO_REPLY_SUPPRESS_SEC);
		return -ENOENT;
	}

	collect(&info, &s);
	ret = encode(&info, payload, sizeof(payload), &len);
	if (ret < 0) {
		return ret;
	}

	*reply = (struct meshtastic_packet){
		.to = req->from,
		.portnum = MESHTASTIC_PORT_NEIGHBORINFO,
		.payload = payload,
		.payload_len = len,
		.request_id = req->id,
	};
	last_reply_ms = now_ms;
	last_reply_valid = true;
	LOG_INF("NeighborInfo request from 0x%08x, replying with %u neighbors", req->from,
		info.neighbors_count);
	return 0;
}

MESHTASTIC_MODULE_DEFINE(neighborinfo, MESHTASTIC_PORT_NEIGHBORINFO, MESHTASTIC_MODULE_ALL_PACKETS,
			 neighborinfo_on_packet, neighborinfo_alloc_reply);

int meshtastic_neighborinfo_init(void)
{
	k_work_init_delayable(&announce_work, announce_work_fn);
	meshtastic_neighborinfo_config_changed();
	return 0;
}
