/* SPDX-License-Identifier: GPL-3.0
 *
 * Scheduler / QoS policy surface. See meshtastic_sched.h.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_sched.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define OB_MAX      CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX
#define OB_DEFAULT  MIN(CONFIG_MESHTASTIC_OUTBOUND_QUEUE_DEPTH, OB_MAX)

/* Default airtime gate + dedup TTL. Chosen to protect a congested channel while
 * staying invisible on a quiet one. */
#define AIRTIME_MAX_DEFAULT 40U    /* % channel util; ~Meshtastic's background gate */
#define DEDUP_TTL_DEFAULT   300U   /* seconds a (src,id) is remembered as a duplicate */
#define RELIABLE_RETRIES_DEFAULT 3U    /* retransmits before giving up (Meshtastic default) */
#define RELIABLE_TIMEOUT_DEFAULT 3000U /* ms between retransmits of an unacked send */
#define ROUTE_TTL_DEFAULT        1800U /* s a learned next-hop stays trusted (upstream 30 min) */

/* Guards writes to, and whole-struct snapshots of, cfg. A leaf mutex: nothing is
 * called while it is held, so it may be nested under any other lock. */
static K_MUTEX_DEFINE(cfg_lock);

/* Live policy. Boot value = the "default" preset (see presets[0]). */
static struct meshtastic_sched_config cfg = {
	.tx_order = MT_SCHED_ORDER_PRIORITY,
	.tx_overflow = MT_SCHED_OVF_DROP_LOWEST,
	.tx_depth = OB_DEFAULT,
	.phone_evict = MT_SCHED_PHONE_PROTECT,
	.airtime_max_util = AIRTIME_MAX_DEFAULT,
	.dedup_ttl_sec = DEDUP_TTL_DEFAULT,
	.reliable_retries = RELIABLE_RETRIES_DEFAULT,
	.reliable_timeout_ms = RELIABLE_TIMEOUT_DEFAULT,
	.route_ttl_sec = ROUTE_TTL_DEFAULT,
};

struct preset {
	const char *name;
	struct meshtastic_sched_config cfg;
};

/* Presets — named policy bundles. presets[0] is the compiled default. */
static const struct preset presets[] = {
	{"default",
	 {MT_SCHED_ORDER_PRIORITY, MT_SCHED_OVF_DROP_LOWEST, OB_DEFAULT, MT_SCHED_PHONE_PROTECT,
	  AIRTIME_MAX_DEFAULT, DEDUP_TTL_DEFAULT, RELIABLE_RETRIES_DEFAULT,
	  RELIABLE_TIMEOUT_DEFAULT, ROUTE_TTL_DEFAULT}},
	/* Reliable delivery and route health are correctness features, not
	 * egress-policy choices, so "legacy" keeps the "default" behavior for both. */
	{"legacy",
	 {MT_SCHED_ORDER_FIFO, MT_SCHED_OVF_DROP_NEWEST, OB_DEFAULT, MT_SCHED_PHONE_DROP_OLDEST,
	  0U, 0U, RELIABLE_RETRIES_DEFAULT, RELIABLE_TIMEOUT_DEFAULT, ROUTE_TTL_DEFAULT}},
};

const struct meshtastic_sched_config *meshtastic_sched_get(void)
{
	return &cfg;
}

void meshtastic_sched_snapshot(struct meshtastic_sched_config *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	*out = cfg;
	k_mutex_unlock(&cfg_lock);
}

uint8_t meshtastic_sched_tier_for(uint32_t portnum)
{
	switch (portnum) {
	case MESHTASTIC_PORT_ROUTING:
		return MT_SCHED_TIER_ACK;
	case MESHTASTIC_PORT_POSITION:
	case MESHTASTIC_PORT_NODEINFO:
	case MESHTASTIC_PORT_TELEMETRY:
		return MT_SCHED_TIER_BG;
	default:
		/* Text and everything else default to NORMAL; relays are tiered
		 * explicitly by the caller. */
		return MT_SCHED_TIER_NORMAL;
	}
}

void meshtastic_sched_defaults(void)
{
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg = presets[0].cfg;
	k_mutex_unlock(&cfg_lock);
	meshtastic_sched_stats_reset();
}

int meshtastic_sched_apply_preset(const char *name)
{
	if (name == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(presets); i++) {
		if (strcmp(name, presets[i].name) == 0) {
			k_mutex_lock(&cfg_lock, K_FOREVER);
			cfg = presets[i].cfg;
			k_mutex_unlock(&cfg_lock);
			meshtastic_sched_stats_reset();
			LOG_INF("sched: policy '%s' applied", name);
			return 0;
		}
	}

	return -ENOENT;
}

const char *meshtastic_sched_preset_name(int index)
{
	if (index < 0 || index >= (int)ARRAY_SIZE(presets)) {
		return NULL;
	}
	return presets[index].name;
}

/* Parse a base-10 integer in [lo, hi]. @retval 0 ok (*out set), -EINVAL bad. */
static int parse_uint(const char *val, long lo, long hi, long *out)
{
	char *end = NULL;
	long v = strtol(val, &end, 10);

	if (end == val || *end != '\0' || v < lo || v > hi) {
		return -EINVAL;
	}
	*out = v;
	return 0;
}

int meshtastic_sched_set(const char *key, const char *val)
{
	long v;
	int ret = -ENOENT;

	if (key == NULL || val == NULL) {
		return -EINVAL;
	}

	/* Hold cfg_lock across the whole decision so a concurrent snapshot sees the
	 * config either fully before or fully after this change. Only non-blocking
	 * calls (strcmp/strtol) run under the lock. */
	k_mutex_lock(&cfg_lock, K_FOREVER);

	if (strcmp(key, "tx.order") == 0) {
		if (strcmp(val, "fifo") == 0) {
			cfg.tx_order = MT_SCHED_ORDER_FIFO;
			ret = 0;
		} else if (strcmp(val, "priority") == 0) {
			cfg.tx_order = MT_SCHED_ORDER_PRIORITY;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
	} else if (strcmp(key, "tx.overflow") == 0) {
		if (strcmp(val, "drop-newest") == 0) {
			cfg.tx_overflow = MT_SCHED_OVF_DROP_NEWEST;
			ret = 0;
		} else if (strcmp(val, "drop-lowest") == 0) {
			cfg.tx_overflow = MT_SCHED_OVF_DROP_LOWEST;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
	} else if (strcmp(key, "tx.depth") == 0) {
		ret = parse_uint(val, 1, OB_MAX, &v);
		if (ret == 0) {
			cfg.tx_depth = (uint8_t)v;
		}
	} else if (strcmp(key, "phone.evict") == 0) {
		if (strcmp(val, "drop-oldest") == 0) {
			cfg.phone_evict = MT_SCHED_PHONE_DROP_OLDEST;
			ret = 0;
		} else if (strcmp(val, "protect") == 0) {
			cfg.phone_evict = MT_SCHED_PHONE_PROTECT;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
	} else if (strcmp(key, "airtime.max") == 0) {
		ret = parse_uint(val, 0, 100, &v);
		if (ret == 0) {
			cfg.airtime_max_util = (uint8_t)v;
		}
	} else if (strcmp(key, "dedup.ttl") == 0) {
		ret = parse_uint(val, 0, UINT16_MAX, &v);
		if (ret == 0) {
			cfg.dedup_ttl_sec = (uint16_t)v;
		}
	} else if (strcmp(key, "reliable.retries") == 0) {
		ret = parse_uint(val, 0, 10, &v);
		if (ret == 0) {
			cfg.reliable_retries = (uint8_t)v;
		}
	} else if (strcmp(key, "reliable.timeout") == 0) {
		ret = parse_uint(val, 50, 60000, &v);
		if (ret == 0) {
			cfg.reliable_timeout_ms = (uint16_t)v;
		}
	} else if (strcmp(key, "route.ttl") == 0) {
		ret = parse_uint(val, 0, UINT16_MAX, &v);
		if (ret == 0) {
			cfg.route_ttl_sec = (uint16_t)v;
		}
	}

	k_mutex_unlock(&cfg_lock);

	/* A committed change starts a fresh measurement window. */
	if (ret == 0) {
		meshtastic_sched_stats_reset();
	}

	return ret;
}

const char *meshtastic_sched_order_name(enum meshtastic_sched_order o)
{
	return (o == MT_SCHED_ORDER_PRIORITY) ? "priority" : "fifo";
}

const char *meshtastic_sched_overflow_name(enum meshtastic_sched_overflow o)
{
	return (o == MT_SCHED_OVF_DROP_LOWEST) ? "drop-lowest" : "drop-newest";
}

const char *meshtastic_sched_phone_evict_name(enum meshtastic_sched_phone_evict e)
{
	return (e == MT_SCHED_PHONE_PROTECT) ? "protect" : "drop-oldest";
}

const char *meshtastic_sched_tier_name(uint8_t tier)
{
	switch (tier) {
	case MT_SCHED_TIER_BG:
		return "bg";
	case MT_SCHED_TIER_NORMAL:
		return "normal";
	case MT_SCHED_TIER_HIGH:
		return "high";
	case MT_SCHED_TIER_ACK:
		return "ack";
	default:
		return "?";
	}
}

/* ---- Stats ---- */

static atomic_t st_enq[MT_SCHED_TIER_COUNT];
static atomic_t st_drop[MT_SCHED_TIER_COUNT];
static atomic_t st_hiwater;
static atomic_t st_phone_drop;
static atomic_t st_phone_drop_protected;
static atomic_t st_airtime_drop;
static atomic_t st_dedup_expired;
static atomic_t st_reliable_acked;
static atomic_t st_reliable_failed;

void meshtastic_sched_stat_enq(uint8_t tier, uint8_t occupancy)
{
	if (tier < MT_SCHED_TIER_COUNT) {
		atomic_inc(&st_enq[tier]);
	}
	if ((atomic_val_t)occupancy > atomic_get(&st_hiwater)) {
		atomic_set(&st_hiwater, (atomic_val_t)occupancy);
	}
}

void meshtastic_sched_stat_drop(uint8_t tier)
{
	if (tier < MT_SCHED_TIER_COUNT) {
		atomic_inc(&st_drop[tier]);
	}
}

void meshtastic_sched_stat_phone_drop(bool protected_frame)
{
	atomic_inc(&st_phone_drop);
	if (protected_frame) {
		atomic_inc(&st_phone_drop_protected);
	}
}

void meshtastic_sched_stat_airtime_drop(void)
{
	atomic_inc(&st_airtime_drop);
}

void meshtastic_sched_stat_dedup_expired(void)
{
	atomic_inc(&st_dedup_expired);
}

void meshtastic_sched_stat_reliable_ack(void)
{
	atomic_inc(&st_reliable_acked);
}

void meshtastic_sched_stat_reliable_fail(void)
{
	atomic_inc(&st_reliable_failed);
}

void meshtastic_sched_stats_get(struct meshtastic_sched_stats *out)
{
	if (out == NULL) {
		return;
	}

	for (int i = 0; i < MT_SCHED_TIER_COUNT; i++) {
		out->tx_enq[i] = (uint32_t)atomic_get(&st_enq[i]);
		out->tx_drop[i] = (uint32_t)atomic_get(&st_drop[i]);
	}
	out->ob_hiwater = (uint32_t)atomic_get(&st_hiwater);
	out->phone_drop = (uint32_t)atomic_get(&st_phone_drop);
	out->phone_drop_protected = (uint32_t)atomic_get(&st_phone_drop_protected);
	out->tx_airtime_drop = (uint32_t)atomic_get(&st_airtime_drop);
	out->dedup_expired = (uint32_t)atomic_get(&st_dedup_expired);
	out->reliable_acked = (uint32_t)atomic_get(&st_reliable_acked);
	out->reliable_failed = (uint32_t)atomic_get(&st_reliable_failed);
}

void meshtastic_sched_stats_reset(void)
{
	for (int i = 0; i < MT_SCHED_TIER_COUNT; i++) {
		atomic_set(&st_enq[i], 0);
		atomic_set(&st_drop[i], 0);
	}
	atomic_set(&st_hiwater, 0);
	atomic_set(&st_phone_drop, 0);
	atomic_set(&st_phone_drop_protected, 0);
	atomic_set(&st_airtime_drop, 0);
	atomic_set(&st_dedup_expired, 0);
	atomic_set(&st_reliable_acked, 0);
	atomic_set(&st_reliable_failed, 0);
}
