/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_traffic.h. Reference: firmware/src/modules/TrafficManagementModule.cpp
 * (handleReceived, shouldDropPosition, isRateLimited, shouldDropUnknown,
 * computePositionFingerprint). The reference packs its per-node state into 10 bytes
 * with tick clocks; this keeps plain millisecond stamps in a small table -- the same
 * decisions, without the modular-arithmetic hazards. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>

#include <zephyr/meshtastic/meshtastic.h>
#if defined(CONFIG_MESHTASTIC_NODEDB)
#include <zephyr/meshtastic/nodedb.h>
#endif

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_position.h"
#include "meshtastic_traffic.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Reference caps and role windows (Default.h, TrafficManagementModule.cpp). */
#define TRAFFIC_COUNT_CAP              60U
#define TRAFFIC_UNKNOWN_WINDOW_MS      (5 * 60 * MSEC_PER_SEC)
#define TRAFFIC_TRACKER_CAP_MS         (60 * 60 * MSEC_PER_SEC)
#define TRAFFIC_LOST_AND_FOUND_CAP_MS  (15 * 60 * MSEC_PER_SEC)

struct traffic_entry {
	uint32_t node; /* 0: empty */
	int64_t seen_ms;
	/* position dedup */
	uint8_t pos_fingerprint; /* 0: none seen */
	int64_t pos_ms;
	/* rate limit */
	uint8_t rate_count; /* 0: no window open */
	int64_t rate_start_ms;
	/* unknown filter */
	uint8_t unknown_count;
	int64_t unknown_start_ms;
};

static K_MUTEX_DEFINE(table_lock);
static struct traffic_entry table[CONFIG_MESHTASTIC_TRAFFIC_TABLE_SIZE];
static meshtastic_TrafficManagementStats stats;

/* ---- settings ------------------------------------------------------------- */

void meshtastic_traffic_settings(struct meshtastic_traffic_settings *out)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	const meshtastic_ModuleConfig_TrafficManagementConfig *cfg =
		&mod.payload_variant.traffic_management;

	if (out == NULL) {
		return;
	}
	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_traffic_management_tag,
						 &mod);
	out->position_min_interval_secs = cfg->position_min_interval_secs;
	out->rate_limit_window_secs = cfg->rate_limit_window_secs;
	/* Reference: thresholds capped at 60 so a saturated counter always exceeds them. */
	out->rate_limit_max_packets = MIN(cfg->rate_limit_max_packets, TRAFFIC_COUNT_CAP);
	out->unknown_packet_threshold = MIN(cfg->unknown_packet_threshold, TRAFFIC_COUNT_CAP);
}

int meshtastic_traffic_validate(const meshtastic_ModuleConfig_TrafficManagementConfig *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (cfg->nodeinfo_direct_response_max_hops != 0U) {
		return -ENOTSUP;
	}
	return 0;
}

int meshtastic_traffic_set(const meshtastic_ModuleConfig_TrafficManagementConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	int ret = meshtastic_traffic_validate(cfg);

	if (ret < 0) {
		return ret;
	}
	mod.which_payload_variant = meshtastic_ModuleConfig_traffic_management_tag;
	mod.payload_variant.traffic_management = *cfg;
	ret = meshtastic_config_store_set_module(&mod);
	if (ret < 0) {
		return ret;
	}
	meshtastic_traffic_config_changed();
	return 0;
}

void meshtastic_traffic_config_changed(void)
{
	struct meshtastic_traffic_settings s;

	/* Read per packet, so nothing to re-arm; just say what is on. */
	meshtastic_traffic_settings(&s);
	LOG_INF("Traffic: dedup %u s, rate %u/%u s, unknown >%u/5 min",
		s.position_min_interval_secs, s.rate_limit_max_packets, s.rate_limit_window_secs,
		s.unknown_packet_threshold);
}

/* ---- table ------------------------------------------------------------------ */

/* Find or create the entry for @p node (stalest replaced when full). Lock held. */
static struct traffic_entry *entry_locked(uint32_t node, int64_t now_ms, bool *is_new)
{
	struct traffic_entry *slot = NULL;
	struct traffic_entry *oldest = &table[0];

	*is_new = false;
	for (size_t i = 0; i < ARRAY_SIZE(table); i++) {
		if (table[i].node == node) {
			table[i].seen_ms = now_ms;
			return &table[i];
		}
		if (table[i].node == 0U && slot == NULL) {
			slot = &table[i];
		}
		if (table[i].seen_ms < oldest->seen_ms) {
			oldest = &table[i];
		}
	}
	if (slot == NULL) {
		slot = oldest;
	}
	*slot = (struct traffic_entry){.node = node, .seen_ms = now_ms};
	*is_new = true;
	return slot;
}

size_t meshtastic_traffic_tracked(void)
{
	size_t n = 0U;

	k_mutex_lock(&table_lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(table); i++) {
		n += (table[i].node != 0U);
	}
	k_mutex_unlock(&table_lock);
	return n;
}

void meshtastic_traffic_stats(meshtastic_TrafficManagementStats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&table_lock, K_FOREVER);
	*out = stats;
	k_mutex_unlock(&table_lock);
}

void meshtastic_traffic_reset(void)
{
	k_mutex_lock(&table_lock, K_FOREVER);
	memset(table, 0, sizeof(table));
	memset(&stats, 0, sizeof(stats));
	k_mutex_unlock(&table_lock);
}

/* ---- position dedup ----------------------------------------------------------- */

/* Reference computePositionFingerprint: the low 4 significant bits of each
 * truncated coordinate, so adjacent grid cells never collide; 0 is the "none
 * seen" sentinel and remaps to 0xFF. */
static uint8_t position_fingerprint(int32_t lat_t, int32_t lon_t, uint8_t precision)
{
	uint8_t bits = (precision < 4U) ? precision : 4U;
	uint8_t shift = 32U - precision;
	uint8_t lat_bits = (uint8_t)(((uint32_t)lat_t >> shift) & ((1U << bits) - 1U));
	uint8_t lon_bits = (uint8_t)(((uint32_t)lon_t >> shift) & ((1U << bits) - 1U));
	uint8_t fp = (uint8_t)((lat_bits << 4) | lon_bits);

	return fp ? fp : 0xFFU;
}

/* The channel's own position_precision, else the reference default. */
static uint8_t dedup_precision(uint8_t channel_index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(channel_index);
	uint32_t p = 0U;

	if (ch != NULL && ch->has_settings && ch->settings.has_module_settings) {
		p = ch->settings.module_settings.position_precision;
	}
	if (p == 0U || p > 32U) {
		p = CONFIG_MESHTASTIC_TRAFFIC_POSITION_PRECISION_BITS;
	}
	return (uint8_t)p;
}

static meshtastic_Config_DeviceConfig_Role sender_role(uint32_t node)
{
#if defined(CONFIG_MESHTASTIC_NODEDB)
	struct meshtastic_nodedb_node n;

	if (meshtastic_nodedb_get(node, &n) == 0 && n.has_user) {
		return (meshtastic_Config_DeviceConfig_Role)n.role;
	}
#else
	ARG_UNUSED(node);
#endif
	return meshtastic_Config_DeviceConfig_Role_CLIENT;
}

/* Reference shouldDropPosition. */
static bool should_drop_position(const struct meshtastic_packet *pkt, const uint8_t *payload,
				 size_t payload_len, uint32_t min_interval_secs, int64_t now_ms)
{
	meshtastic_Position pos = meshtastic_Position_init_zero;
	pb_istream_t is = pb_istream_from_buffer(payload, payload_len);
	int32_t lat_t;
	int32_t lon_t;
	uint8_t precision;
	uint8_t fp;
	int64_t window_ms;
	struct traffic_entry *e;
	bool is_new;
	bool drop;

	if (!pb_decode(&is, meshtastic_Position_fields, &pos) || !pos.has_latitude_i ||
	    !pos.has_longitude_i) {
		return false;
	}

	precision = dedup_precision(pkt->channel_index);
	lat_t = pos.latitude_i;
	lon_t = pos.longitude_i;
	meshtastic_position_truncate_latlon(&lat_t, &lon_t, precision);
	fp = position_fingerprint(lat_t, lon_t, precision);

	/* Raw configured interval: 0 means dedup disabled. */
	window_ms = (int64_t)min_interval_secs * MSEC_PER_SEC;

	/* Role caps, never lengthening: lost-and-found refreshes every 15 min at
	 * most, trackers hourly. */
	switch (sender_role(pkt->from)) {
	case meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND:
		if (window_ms != 0 && window_ms > TRAFFIC_LOST_AND_FOUND_CAP_MS) {
			window_ms = TRAFFIC_LOST_AND_FOUND_CAP_MS;
		}
		break;
	case meshtastic_Config_DeviceConfig_Role_TRACKER:
	case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
		if (window_ms > TRAFFIC_TRACKER_CAP_MS) {
			window_ms = TRAFFIC_TRACKER_CAP_MS;
		}
		break;
	default:
		break;
	}

	k_mutex_lock(&table_lock, K_FOREVER);
	e = entry_locked(pkt->from, now_ms, &is_new);
	{
		bool has_state = !is_new && e->pos_fingerprint != 0U;
		bool same = has_state && e->pos_fingerprint == fp;
		bool within = has_state && window_ms != 0 && (now_ms - e->pos_ms) < window_ms;

		drop = same && within;
		/* Stamp only what we let through: re-stamping a dropped duplicate
		 * would slide the window forward on every repeat and mute the node. */
		if (!drop) {
			e->pos_fingerprint = fp;
			e->pos_ms = now_ms;
		}
	}
	k_mutex_unlock(&table_lock);

	return drop;
}

/* ---- rate limit ----------------------------------------------------------------- */

/* Reference isRateLimited. */
static bool is_rate_limited(uint32_t from, uint32_t window_secs, uint32_t max_packets,
			    int64_t now_ms)
{
	int64_t window_ms = (int64_t)window_secs * MSEC_PER_SEC;
	struct traffic_entry *e;
	bool is_new;
	bool limited;

	k_mutex_lock(&table_lock, K_FOREVER);
	e = entry_locked(from, now_ms, &is_new);
	if (is_new || e->rate_count == 0U || (now_ms - e->rate_start_ms) >= window_ms) {
		e->rate_start_ms = now_ms;
		e->rate_count = 1U;
		limited = false;
	} else {
		if (e->rate_count < UINT8_MAX) {
			e->rate_count++;
		}
		limited = e->rate_count > max_packets;
		if (limited || e->rate_count == max_packets) {
			LOG_DBG("Traffic: rate 0x%08x count=%u threshold=%u -> %s", from,
				e->rate_count, max_packets, limited ? "DROP" : "at-limit");
		}
	}
	k_mutex_unlock(&table_lock);

	return limited;
}

/* ---- unknown filter ------------------------------------------------------------- */

/* Reference shouldDropUnknown: fixed 5-minute window. */
static bool should_drop_unknown(uint32_t from, uint32_t threshold, int64_t now_ms)
{
	struct traffic_entry *e;
	bool is_new;
	bool drop;

	k_mutex_lock(&table_lock, K_FOREVER);
	e = entry_locked(from, now_ms, &is_new);
	if (is_new || e->unknown_count == 0U ||
	    (now_ms - e->unknown_start_ms) >= TRAFFIC_UNKNOWN_WINDOW_MS) {
		e->unknown_start_ms = now_ms;
		e->unknown_count = 0U;
	}
	if (e->unknown_count < UINT8_MAX) {
		e->unknown_count++;
	}
	drop = e->unknown_count > threshold;
	if (drop || e->unknown_count == threshold) {
		LOG_DBG("Traffic: unknown 0x%08x count=%u threshold=%u -> %s", from, e->unknown_count,
			threshold, drop ? "DROP" : "at-limit");
	}
	k_mutex_unlock(&table_lock);

	return drop;
}

/* ---- the gate ------------------------------------------------------------------- */

enum meshtastic_traffic_verdict meshtastic_traffic_inspect(const struct meshtastic_packet *pkt,
							  const meshtastic_MeshPacket *mesh,
							  bool decoded)
{
	struct meshtastic_traffic_settings s;
	int64_t now_ms = k_uptime_get();
	uint32_t from;
	uint32_t to;
	uint32_t portnum;
	const uint8_t *payload;
	size_t payload_len;

	if (pkt == NULL) {
		return MESHTASTIC_TRAFFIC_PASS;
	}

	meshtastic_traffic_settings(&s);
	k_mutex_lock(&table_lock, K_FOREVER);
	stats.packets_inspected++;
	k_mutex_unlock(&table_lock);

	from = mesh ? mesh->from : pkt->from;
	to = mesh ? mesh->to : pkt->to;

	/* A frame we could not decode: the source may be misbehaving (wrong key,
	 * corruption). Count it and, past the threshold, stop relaying it. */
	if (!decoded) {
		if (s.unknown_packet_threshold != 0U && from != 0U &&
		    should_drop_unknown(from, s.unknown_packet_threshold, now_ms)) {
			LOG_INF("Traffic: drop encrypted from=0x%08x to=0x%08x reason=unknown", from,
				to);
			k_mutex_lock(&table_lock, K_FOREVER);
			stats.unknown_packet_drops++;
			k_mutex_unlock(&table_lock);
			return MESHTASTIC_TRAFFIC_DROP;
		}
		return MESHTASTIC_TRAFFIC_PASS;
	}

	/* Our own traffic and traffic for us are never shaped. */
	if (from == 0U || from == meshtastic_get_node_id() || to == meshtastic_get_node_id()) {
		return MESHTASTIC_TRAFFIC_PASS;
	}

	portnum = mesh ? (uint32_t)mesh->decoded.portnum : pkt->portnum;
	payload = mesh ? mesh->decoded.payload.bytes : pkt->payload;
	payload_len = mesh ? mesh->decoded.payload.size : pkt->payload_len;

	if (s.position_min_interval_secs != 0U && portnum == MESHTASTIC_PORT_POSITION &&
	    payload != NULL && meshtastic_channels_is_well_known(pkt->channel_index) &&
	    should_drop_position(pkt, payload, payload_len, s.position_min_interval_secs,
				 now_ms)) {
		LOG_INF("Traffic: drop POSITION from=0x%08x to=0x%08x reason=position-dedup", from,
			to);
		k_mutex_lock(&table_lock, K_FOREVER);
		stats.position_dedup_drops++;
		k_mutex_unlock(&table_lock);
		return MESHTASTIC_TRAFFIC_DROP;
	}

	if (s.rate_limit_window_secs != 0U && s.rate_limit_max_packets != 0U &&
	    portnum != MESHTASTIC_PORT_ROUTING && portnum != MESHTASTIC_PORT_ADMIN &&
	    is_rate_limited(from, s.rate_limit_window_secs, s.rate_limit_max_packets, now_ms)) {
		LOG_INF("Traffic: drop port=%u from=0x%08x to=0x%08x reason=rate-limit",
			(unsigned int)portnum, from, to);
		k_mutex_lock(&table_lock, K_FOREVER);
		stats.rate_limit_drops++;
		k_mutex_unlock(&table_lock);
		return MESHTASTIC_TRAFFIC_DROP;
	}

	return MESHTASTIC_TRAFFIC_PASS;
}

int meshtastic_traffic_init(void)
{
	meshtastic_traffic_config_changed();
	return 0;
}
