/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_FLEET_H_
#define MESHTASTIC_FLEET_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic_hlc.h"

/*
 * The fleet's firmware intent (DECLARATIVE-FLEET.md §7, §8): what each image
 * class SHOULD be running, carried on the cluster document as the private
 * section FW at BASE (the fleet) and NODE (a pin for one node). This module
 * is that section's writer and reader — and, from F3 on, its actuator (the
 * courier loop). F2 scope: intent in, intent out, nothing pushed yet.
 *
 * Versions are (major << 24) | (minor << 16) | revision — the MCUboot header
 * fields, orderable; build is informational and never on the wire.
 */

#define MESHTASTIC_FLEET_FLAG_PAUSE           0x1U
#define MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE 0x2U

static inline uint32_t meshtastic_fleet_version_pack(uint8_t major, uint8_t minor,
						      uint16_t revision)
{
	return ((uint32_t)major << 24) | ((uint32_t)minor << 16) | revision;
}

struct meshtastic_fleet_intent {
	uint8_t class_id;      /* the row that answered (0 = a node pin's wildcard) */
	uint32_t version;      /* packed */
	uint8_t hash_prefix[4];
	bool has_hash;
	uint32_t flags;
	uint16_t hw_model;     /* meshtastic_HardwareModel the row is for; 0 = unstated */
	bool pinned;           /* answered by a NODE entry rather than BASE */
	struct meshtastic_hlc_stamp stamp; /* of the entry that answered */
};

/*
 * Publish the fleet's intent for @p class_id: upsert the row in BASE/FW (the
 * other classes' rows are carried through). hash4 may be NULL; @p hw_model is
 * the meshtastic_HardwareModel the class is for, or 0 to leave it unstated
 * (then no courier cross-checks it). -EPERM unless this node may write the
 * document; -ENOSPC if four other classes already have rows.
 */
int meshtastic_fleet_desire(uint8_t class_id, uint32_t version, const uint8_t *hash4,
			    uint32_t flags, uint16_t hw_model);

/*
 * Pin one node: NODE/<node_id>/FW with a single wildcard row. version 0
 * withdraws the pin (a tombstone; the node falls back to BASE).
 */
int meshtastic_fleet_pin(uint32_t node_id, uint32_t version, const uint8_t *hash4,
			 uint32_t flags);

/*
 * What @p node_id should run, given it is of @p class_id: its pin if it has
 * one, else the BASE row for its class. False if the document says nothing
 * for that pair (a courier then does nothing — silence is not an order).
 */
bool meshtastic_fleet_desired_for(uint32_t node_id, uint8_t class_id,
				  struct meshtastic_fleet_intent *out);

/* Called by the cluster reconciler after any change touching a private
 * section. F2: log it. F3: wake the courier loop. */
void meshtastic_fleet_intent_changed(void);

/* ---- the courier loop (F3, DECLARATIVE-FLEET.md §8) -----------------------
 *
 * "The courier decides, the target consents." A courier watches its BLE
 * neighbours' beats (running version, class, HOLD/TESTBOOT) against the
 * document's intent, and pushes an image from its depot to any neighbour that
 * is behind and willing. One target at a time (smpc is single-link);
 * manual-armed in F3.
 */

/* Why a neighbour is or is not a push candidate — the pure heart of the loop,
 * decided from values alone so it is testable with no BLE. */
enum meshtastic_fleet_verdict {
	MESHTASTIC_FLEET_PUSH = 0,     /* desired > running (or a sanctioned downgrade) */
	MESHTASTIC_FLEET_SKIP_NO_INTENT,   /* the document says nothing for this class */
	MESHTASTIC_FLEET_SKIP_UP_TO_DATE,  /* already at the desired version */
	MESHTASTIC_FLEET_SKIP_UNKNOWN_VER, /* its beat does not report a version */
	MESHTASTIC_FLEET_SKIP_HOLD,        /* it asked not to be pushed to */
	MESHTASTIC_FLEET_SKIP_TESTBOOT,    /* it is mid-update / unconfirmed already */
	MESHTASTIC_FLEET_SKIP_PAUSED,      /* the intent row is paused */
	MESHTASTIC_FLEET_SKIP_DOWNGRADE,   /* desired < running and downgrade not allowed */
	MESHTASTIC_FLEET_SKIP_HW_MISMATCH, /* the row names a board the node is not */
};

/*
 * Decide, from a neighbour's beat and the intent for it: @p running is its
 * packed running version, @p running_known false when the beat reports none;
 * @p beat_flags are the MESHTASTIC_BLE_PEER_FLAG_* bits; @p nbr_hw_model is
 * the node's hw_model from OUR NodeDB (0 if we have no NodeInfo for it);
 * @p want is meshtastic_fleet_desired_for() output, or NULL when the document
 * says nothing. Never-downgrade (R2), the beat's consent bits and the
 * board cross-check (a row that names a board, offered to a node that is
 * known to be another) live here.
 */
enum meshtastic_fleet_verdict
meshtastic_fleet_evaluate(uint32_t running, bool running_known, uint8_t beat_flags,
			  uint16_t nbr_hw_model, const struct meshtastic_fleet_intent *want);

const char *meshtastic_fleet_verdict_str(enum meshtastic_fleet_verdict v);

/* ---- which neighbour first (§8.4), pure ------------------------------------
 *
 * F6: a courier with more than one target (BT_MAX_CONN 3 on the R8: two peer
 * links plus the host) is still ONE job at a time (R6) — the star is served by
 * ordering, not concurrency. The order is decided here from values alone so it
 * is provable on native_sim, and `fleet status` marks the row this returns so
 * the bench sees the same choice the loop will make.
 */
struct meshtastic_fleet_candidate {
	uint32_t node;
	bool present;      /* a fresh v2 beat within the last few seconds */
	bool latched;      /* REVERTED: off limits until `fleet clear` */
	bool in_flight;    /* updating, or delivered and awaiting its self-confirm */
	bool has_cargo;    /* the depot holds a classed image of the wanted version */
	enum meshtastic_fleet_verdict verdict; /* meshtastic_fleet_evaluate() for it */
	uint32_t attempts;
	int64_t last_beat_ms;
	int64_t next_try_ms; /* backoff: not before this uptime */
};

/*
 * The index of the candidate to push to next, or -1 when none is eligible.
 * Eligible: present, not latched, not in flight, past its backoff, verdict
 * PUSH, cargo on hand. Among those: fewest attempts, then freshest beat, then
 * lowest node id — deterministic, so two ticks with the same view agree, and
 * a node in flight is never picked twice however often the loop is re-armed.
 */
int meshtastic_fleet_pick(const struct meshtastic_fleet_candidate *c, unsigned int n,
			  int64_t now);

#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
/* Arm/disarm the courier loop (manual in F3; off at boot). */
void meshtastic_fleet_courier_arm(bool on);
bool meshtastic_fleet_courier_armed(void);

/* Forget a neighbour's REVERTED latch so it may be tried again. */
void meshtastic_fleet_courier_clear(uint32_t node_id);

/* One row of the courier's per-neighbour view, for `fleet status`. */
struct meshtastic_fleet_courier_row {
	uint32_t node_id;
	uint8_t class_id;
	uint32_t running;      /* packed; 0 = unknown */
	uint32_t desired;      /* packed; 0 = no intent */
	uint8_t flags;         /* last beat flags */
	const char *state;     /* "idle"/"updating"/"wait-confirm"/"done"/"reverted"/… */
	uint32_t attempts;
	bool next;             /* the row meshtastic_fleet_pick() would serve next */
};
uint16_t meshtastic_fleet_courier_rows(struct meshtastic_fleet_courier_row *out, uint16_t max);
#endif /* CONFIG_MESHTASTIC_FLEET_COURIER */

#endif /* MESHTASTIC_FLEET_H_ */
