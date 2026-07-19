/* SPDX-License-Identifier: GPL-3.0
 *
 * Meshtastic scheduler / QoS policy surface — a central, runtime-tunable set of
 * knobs governing how packets are prioritized, ordered, and dropped, plus live
 * counters to observe the effect. Think Linux I/O schedulers: named policy
 * presets with sane defaults, tweakable at runtime (over the shell) with no
 * rebuild/reflash.
 *
 * v1 scope: TX egress ordering + overflow policy + soft depth, and stats.
 * Phone-queue eviction, dedup TTL, airtime gating, and broadcast intervals are
 * planned follow-ups that will hang off this same struct.
 *
 * Tuning is RAM-only: a reboot always returns to the compiled defaults.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_SCHED_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_SCHED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Egress priority tiers (low to high). A packet's tier is derived from its
 * portnum (see meshtastic_sched_tier_for). Higher tiers are transmitted first
 * and are protected from being dropped by lower-tier traffic. */
enum meshtastic_sched_tier {
	MT_SCHED_TIER_BG = 0,     /* position / telemetry / nodeinfo periodic beacons */
	MT_SCHED_TIER_NORMAL = 1, /* text, relayed traffic, default */
	MT_SCHED_TIER_HIGH = 2,   /* admin, want_response replies */
	MT_SCHED_TIER_ACK = 3,    /* routing ACK/NAK */
	MT_SCHED_TIER_COUNT = 4,
};

/* How the outbound worker picks the next frame to send. */
enum meshtastic_sched_order {
	MT_SCHED_ORDER_FIFO = 0,     /* strict arrival order (legacy behavior) */
	MT_SCHED_ORDER_PRIORITY = 1, /* highest tier first, FIFO within a tier */
};

/* What happens when a fire-and-forget (K_NO_WAIT) send hits a full queue.
 * Callers that pass a timeout are never silently dropped — they wait for space
 * regardless of this setting. */
enum meshtastic_sched_overflow {
	MT_SCHED_OVF_DROP_NEWEST = 0, /* reject the incoming frame (legacy) */
	MT_SCHED_OVF_DROP_LOWEST = 1, /* evict the lowest-tier queued frame if the
				       * incoming one outranks it, else reject */
};

/* How a full per-transport phone FromRadio queue makes room. */
enum meshtastic_sched_phone_evict {
	MT_SCHED_PHONE_DROP_OLDEST = 0, /* evict the oldest queued frame (legacy) */
	MT_SCHED_PHONE_PROTECT = 1,     /* evict the oldest *droppable* frame
					 * (position/telemetry/nodeinfo/queueStatus)
					 * first, so text / routing / admin /
					 * handshake frames survive a burst */
};

struct meshtastic_sched_config {
	enum meshtastic_sched_order tx_order;
	enum meshtastic_sched_overflow tx_overflow;
	uint8_t tx_depth; /* soft cap, 1..CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX */
	enum meshtastic_sched_phone_evict phone_evict;
	uint8_t airtime_max_util; /* % channel utilization at/above which the node
				   * suppresses its own background broadcasts
				   * (position/telemetry/nodeinfo). 0 = gate off. */
	uint16_t dedup_ttl_sec;   /* dup-cache entry lifetime in seconds; a matching
				   * (src,id) older than this is treated as fresh so
				   * a legitimately re-sent packet is not swallowed.
				   * 0 = never expire (legacy fixed ring). */
	uint8_t reliable_retries; /* retransmits of an unacked want_ack unicast we
				   * originate, before giving up. 0 = disable
				   * sender-side reliable delivery. */
	uint16_t reliable_timeout_ms; /* base interval between those retransmits */
	uint16_t route_ttl_sec;   /* learned next-hop route lifetime in seconds; a
				   * route older than this decays back to flood at
				   * read time (upstream RouteHealth staleness).
				   * 0 = never expire. */
	/* Contention window (see meshtastic_contention.h). Defaults reproduce the
	 * reference firmware; the "legacy" preset zeroes them, which restores this
	 * port's original transmit-immediately behaviour. Being able to switch that
	 * at runtime is what makes an A/B on a live mesh possible without a
	 * reflash. */
	uint8_t cw_min;           /* min window exponent, pool = 1 << cw (ref 3) */
	uint8_t cw_max;           /* max window exponent (ref 8). 0 = no delay. */
	uint8_t cw_relay_offset;  /* client relays wait cw_relay_offset * cw_max
				   * slots before their own random window, so a
				   * router's relay reliably precedes theirs
				   * (ref 2). 0 = no router priority. */
	uint16_t cw_slot_ms;      /* slot-time override; 0 = derive from the active
				   * modem preset, which is what the reference
				   * does. Non-zero is for experiments. */
};

/* Gap buckets (ms) between our relay of a (src,id) and hearing a peer relay the
 * same one: <25, <50, <100, <250, <500, <1000, >=1000. The point is to see
 * whether redundant relays land inside the window a contention delay would
 * occupy — those are the transmissions an overhear-cancel would have saved. */
#define MT_RELAY_GAP_BUCKETS 7
extern const uint16_t meshtastic_relay_gap_bounds[MT_RELAY_GAP_BUCKETS - 1];

struct meshtastic_sched_stats {
	uint32_t tx_enq[MT_SCHED_TIER_COUNT];  /* frames queued, per tier */
	uint32_t tx_drop[MT_SCHED_TIER_COUNT]; /* frames dropped at egress, per tier */
	uint32_t ob_hiwater;                   /* peak outbound queue occupancy */
	uint32_t phone_drop;                   /* phone FromRadio frames dropped (droppable) */
	uint32_t phone_drop_protected;         /* protected frames dropped (queue saturated) */
	uint32_t tx_airtime_drop;              /* background self-broadcasts suppressed by gate */
	uint32_t dedup_expired;                /* dup-cache hits ignored because TTL had elapsed */
	uint32_t reliable_acked;               /* want_ack sends confirmed delivered (ack/implicit) */
	uint32_t reliable_failed;              /* want_ack sends that exhausted all retransmits */
	/* Flood-redundancy measurement (see meshtastic_sched_stat_relay_redundant).
	 * relay_sent is the denominator; relay_redundant counts our relays that a
	 * peer also relayed, and relay_gap buckets how long after ours theirs came. */
	uint32_t relay_sent;
	uint32_t relay_redundant;
	uint32_t relay_gap[MT_RELAY_GAP_BUCKETS];
};

/*
 * Concurrency model
 * -----------------
 * The live config is mutated by the shell thread (set / apply_preset / defaults)
 * while several other threads read it (TX worker, enqueue callers, RX/router,
 * phone-enqueue, send_packet). Each field is an aligned scalar, so a *single*
 * field read can never tear into a garbage value. The only hazards are (a) a
 * multi-field preset write being observed half-applied, and (b) one decision
 * reading several fields that then disagree. Both are closed by taking a
 * whole-struct snapshot:
 *
 *   - A consumer that reads TWO OR MORE fields for one decision MUST call
 *     meshtastic_sched_snapshot() once at the start of the operation and decide
 *     from the local copy — never re-read the live config mid-decision.
 *   - A consumer that reads a SINGLE scalar once (e.g. the airtime gate or the
 *     dedup TTL) may use meshtastic_sched_get() directly; the aligned load is
 *     atomic and there is no second field to be inconsistent with.
 *
 * Lock ordering: the snapshot lock is a leaf. It is legal to snapshot while
 * holding a higher lock (e.g. the outbound queue lock); the reverse never
 * happens (nothing is called while the snapshot lock is held).
 */

/** Pointer to the live (mutable) policy. Safe only for a single atomic scalar
 * read; for any multi-field decision use meshtastic_sched_snapshot(). */
const struct meshtastic_sched_config *meshtastic_sched_get(void);

/** Copy the whole live policy atomically (under the config lock) into @p out, so
 * every field of the returned snapshot belongs to one and the same policy. */
void meshtastic_sched_snapshot(struct meshtastic_sched_config *out);

/** Map a portnum to its egress tier. */
uint8_t meshtastic_sched_tier_for(uint32_t portnum);

/** Restore the compiled default policy (the "default" preset). */
void meshtastic_sched_defaults(void);

/**
 * Apply a named policy preset ("default", "legacy", ...).
 * @retval 0 applied, -ENOENT unknown preset.
 */
int meshtastic_sched_apply_preset(const char *name);

/**
 * Set one knob by key ("tx.order", "tx.overflow", "tx.depth", "phone.evict",
 * "airtime.max", "dedup.ttl", "reliable.retries", "reliable.timeout",
 * "route.ttl") to a string value. A successful change resets the
 * live stats so behavior can be evaluated from a clean slate ("change modes,
 * fresh counters").
 * @retval 0 applied, -ENOENT unknown key, -EINVAL bad value.
 */
int meshtastic_sched_set(const char *key, const char *val);

/* Rendering helpers (for the shell). */
const char *meshtastic_sched_order_name(enum meshtastic_sched_order o);
const char *meshtastic_sched_overflow_name(enum meshtastic_sched_overflow o);
const char *meshtastic_sched_phone_evict_name(enum meshtastic_sched_phone_evict e);
const char *meshtastic_sched_tier_name(uint8_t tier);
const char *meshtastic_sched_preset_name(int index); /* NULL past the end */

/* Stats. */
void meshtastic_sched_stats_get(struct meshtastic_sched_stats *out);
void meshtastic_sched_stats_reset(void);

/* Counter hooks, called from the outbound, phone-queue, TX-gate and dedup paths. */
void meshtastic_sched_stat_enq(uint8_t tier, uint8_t occupancy);
void meshtastic_sched_stat_drop(uint8_t tier);
void meshtastic_sched_stat_phone_drop(bool protected_frame);
void meshtastic_sched_stat_airtime_drop(void);
void meshtastic_sched_stat_dedup_expired(void);
void meshtastic_sched_stat_reliable_ack(void);
void meshtastic_sched_stat_reliable_fail(void);

/** Count a relay we transmitted (denominator for the redundancy ratio). */
void meshtastic_sched_stat_relay_sent(void);

/**
 * Count a relay of ours that a peer also performed, @p gap_ms after we sent it.
 *
 * Measurement for the flood-contention work: the port relays immediately, with
 * no delay and no way to cancel a pending relay, so every node that hears a
 * broadcast transmits. This counts how often that was redundant, and how
 * tightly the redundant transmissions cluster in time.
 */
void meshtastic_sched_stat_relay_redundant(uint32_t gap_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_SCHED_H_ */
