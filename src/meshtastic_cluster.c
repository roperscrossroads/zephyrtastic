/* SPDX-License-Identifier: GPL-3.0
 *
 * Cluster config sync, module half (agents-xhli.4, docs/CLUSTER-SYNC-M4.md).
 *
 * Follows the same shape upstream builds every core feature with — a
 * per-portnum module (their SinglePortModule/ProtobufModule, our
 * MESHTASTIC_MODULE_DEFINE with the protobuf decoded in on_packet). The port
 * number is PRIVATE_APP (256): the range upstream reserves for private
 * applications, and the dedicated cluster channel — not the portnum — is what
 * scopes the traffic, so ingest is double-gated on both.
 *
 * M4a: digests out at a cadence, digests in compared against the local doc.
 * M4b: what a mismatch DOES — the anti-entropy walk of §3.3 (ClusterVectorReq
 * → stamp rows → ClusterEntryReq → entries → LWW merge) and the reconciler
 * that turns a changed document into applied configuration.
 * M4c (here): the per-node layer — `pin` writes nodes/<me>/<sec>, `unpin`
 * tombstones it, and a local write is broadcast once instead of waiting for
 * the next digest.
 *
 * Two properties are worth naming because they are why this needs no delivery
 * tracking, no acknowledgements and no retry queues:
 *
 *  - It is LEVEL-triggered, not edge-triggered. Nothing here has to succeed.
 *    A lost frame, a preempted responder, a peer that reboots mid-walk — all
 *    leave the two documents still differing, so the next digest states the
 *    difference again and the walk simply reruns. The only cost of failure is
 *    latency, bounded by the digest period.
 *  - Nobody pushes uninvited ABOUT SOMEONE ELSE'S WRITE. A node that finds it
 *    is AHEAD of a peer does nothing at all; its own digest is the invitation
 *    for the peer to pull, and an entry that merely arrived here is never
 *    re-broadcast. That is what keeps a mute node (radio TX off) a full
 *    participant: it pulls over its BLE peer link, which carries unicasts
 *    (M2's divert), and it never has to originate anything to converge.
 *
 *    Push-on-change (M4c, D11) is the single, deliberate exception, and it is
 *    narrow on purpose: only a write MADE HERE — a promote, a pin, an unpin —
 *    is announced, exactly once. It is a latency optimisation layered on top
 *    of the digest, never a delivery mechanism: dropping one costs a digest
 *    period and can never cost correctness. That is what lets its rate bound
 *    (below) simply throw pushes away instead of queueing them.
 *
 * The document logic lives in meshtastic_cluster_doc.c where it is
 * unit-testable without any of this.
 *
 * WHAT IS BOUNDED, AND BY WHAT. Level-triggered means "retry forever" is the
 * normal state of affairs, so every loop in here needs a bound that does not
 * depend on the other end cooperating. The complete list:
 *
 *   what could run away              bounded by
 *   ------------------------------   ------------------------------------------
 *   digests we send                  the Kconfig cadence
 *   the narrowed node's backstop     its own Kconfig cadence, in digest
 *   walk                             periods — a timer like the digest, which
 *                                    is why the fruitless backoff does not
 *                                    apply to it
 *   narrowing the claim              one way, and at most once: a CORE claim
 *                                    is twice the allowlist and MAX_ENTRIES
 *                                    has a floor of exactly that
 *   NVS deletes when narrowing       the table cap, once in a node's life
 *   our pulls                        one exchange at a time + a timeout; and
 *                                    consecutive FRUITLESS walks back off
 *                                    exponentially to a cap, so a peer we can
 *                                    never converge with costs a probe, not a
 *                                    conversation. Fruitless counts BOTH
 *                                    shapes: a walk that asked for entries and
 *                                    merged none, and a walk that asked for
 *                                    nothing because we were already ahead
 *   frames we serve on request       one walk at a time (never rewound) AND a
 *                                    minimum interval between walks, so one
 *                                    small request cannot buy unbounded airtime
 *   broadcasts we originate on       a token bucket, applied when the push is
 *   change (M4c push-on-change)      QUEUED rather than when it is sent: over
 *                                    budget the push is DROPPED, not deferred,
 *                                    because deferring only moves an unbounded
 *                                    backlog. Held-down up-arrow, a config that
 *                                    flaps, a script in a loop — all cost one
 *                                    broadcast per refill period, and the
 *                                    digest still carries every change
 *   rows we ask for                  the ClusterEntryReq key cap, and never for
 *                                    a key a full table would refuse
 *   entries the table accepts        the table cap, plus the owner-existence
 *                                    gate so ids cannot be invented
 *   how far ahead a stamp may be     the HLC drift horizon — without which one
 *                                    far-future stamp wins permanently and no
 *                                    honest write can ever beat it
 *   flash writes                     everything above, transitively: a write
 *                                    happens only when an entry is accepted
 *   log output                       a throttle on the refusal paths, because
 *                                    the log is shipped off-node over the
 *                                    network and a frame flood must not become
 *                                    a packet flood
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic/config.pb.h"
#include "zephyrtastic/cluster.pb.h"

#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_hlc.h"
#include "meshtastic_modules.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* The wire cap (cluster.options) and the table cap must agree, and the whole
 * envelope must fit a frame's Data payload with margin for the Data wrapper. */
BUILD_ASSERT(MESHTASTIC_CLUSTER_PAYLOAD_MAX == 128U, "doc cap must match cluster.options");
BUILD_ASSERT(zephyrtastic_ClusterMessage_size + 16U <= MESHTASTIC_MAX_PAYLOAD_LEN,
	     "ClusterMessage must fit one frame");
/* The v1 allowlist's largest section must fit the payload cap (wrapper ~6 B). */
BUILD_ASSERT(meshtastic_Config_DeviceConfig_size + 8U <= MESHTASTIC_CLUSTER_PAYLOAD_MAX,
	     "DeviceConfig outgrew the cluster payload cap");

/* How many stamp rows ride in one ClusterVector, and how many keys in one
 * ClusterEntryReq. Both are wire facts (cluster.options max_count), read back
 * off the generated structs so a change there cannot silently desync the
 * buffers sized against them here. */
#define CLUSTER_VECTOR_ROWS ARRAY_SIZE(((zephyrtastic_ClusterVector *)NULL)->entries)
#define CLUSTER_PULL_KEYS   ARRAY_SIZE(((zephyrtastic_ClusterEntryReq *)NULL)->keys)

/*
 * Push-on-change budget. The burst matters more than it looks: an operator
 * setting a node up types several commands in a row and every one of them is
 * legitimate, so a flat minimum interval would punish exactly the case the
 * feature exists for. The refill period is the ceiling on sustained rate.
 *
 * The queue only has to hold a burst — a push refused for budget is dropped at
 * enqueue, never parked — so anything beyond the burst size would be dead
 * storage.
 */
#define CLUSTER_PUSH_BURST 3U
#define CLUSTER_PUSH_QUEUE 4U
BUILD_ASSERT(CLUSTER_PUSH_QUEUE >= CLUSTER_PUSH_BURST,
	     "the push queue must be able to hold a full burst");

static K_MUTEX_DEFINE(cluster_lock);

/* PSRAM on the V4 family (no-op elsewhere): CPU-only, mutex-guarded, never a
 * DMA source — passes the meshtastic_ext_ram.h placement rules. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_cluster_entry
	cluster_storage[CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES];

/*
 * One exchange in each direction at a time (§3.3's rate limit), which is all
 * the state the walk needs: as the REQUESTER we are pulling from one peer, and
 * as the RESPONDER we are serving one. A second peer asking mid-serve is told
 * nothing and simply retries after its next digest — refusing is cheaper and
 * more predictable than queueing, and costs only latency.
 */
enum cluster_pull_state {
	PULL_IDLE = 0,
	PULL_SEND_VECTOR_REQ, /* mismatch seen; owe pull_peer a ClusterVectorReq */
	PULL_AWAIT_VECTOR,    /* asked; waiting for stamp rows */
	PULL_SEND_ENTRY_REQ,  /* rows diffed; owe pull_peer a ClusterEntryReq */
};

/*
 * WHY a walk was opened — because it decides whether the walk gets JUDGED.
 *
 * The fruitless-walk backoff bounds a conversation that can never succeed, and
 * "never succeeds" is only a meaningful verdict on the walks this node opens on
 * its own account, on a cadence, about a mismatch it did not choose. An
 * operator typing `cluster pull` has already decided the round trip is worth
 * it. Judging a walk somebody asked for would let the backoff suppress the
 * cadence that is the actual bound.
 */
enum cluster_pull_reason {
	PULL_REASON_DIGEST = 0, /* a digest mismatch we noticed — judged */
	PULL_REASON_SHELL,	/* `cluster pull <node>` — the operator's call */
	PULL_REASON_BACKSTOP,	/* the narrowed node's scheduled self-check */
};

static struct {
	struct meshtastic_cluster_doc doc;
	/* WHAT THIS NODE CLAIMS TO TRACK (agents-xhli.10, §6.1). Every digest
	 * leg, the diff predicate, the ingest gate, the vector filter and the
	 * persistence load filter are taken over this and nothing else — the
	 * moment two of them disagree, this node advertises a document it does
	 * not hold, which is the one failure the mechanism exists to prevent.
	 *
	 * Held as a KIND, not a whole scope: the owner of a CORE claim is this
	 * node's id, which is not necessarily known when the module starts. */
	uint8_t scope_kind;
	bool scope_pinned;   /* compiled FULL/CORE, or an operator's choice */
	bool scope_id_warned;
	/* The backstop: the last peer that claimed EVERYTHING, and how many
	 * digests until we walk it. Only a narrowed node uses either. */
	uint32_t backstop_peer;
	uint16_t backstop_countdown;
	struct meshtastic_hlc hlc;
	struct meshtastic_cluster_stats stats;
	bool channel_missing_logged;

	/* Requester half. */
	enum cluster_pull_state pull_state;
	uint8_t pull_reason; /* enum cluster_pull_reason — why this walk exists */
	uint32_t pull_peer;
	int64_t pull_deadline_ms;
	struct meshtastic_cluster_key pull_keys[CLUSTER_PULL_KEYS];
	uint8_t pull_key_count;

	/* Responder half: a vector walk (resumable by offset) and a burst of
	 * entry replies, each aimed at one peer. */
	uint32_t vec_dest;
	uint16_t vec_offset;	/* table index the walk resumes from */
	uint16_t vec_emitted;	/* rows already sent — the wire offset, which is
				 * in FILTERED coordinates when the requester
				 * claims less than the whole document */
	uint8_t vec_scope_kind; /* the REQUESTER's claim, held for the walk */
	uint32_t ent_dest;
	struct meshtastic_cluster_key ent_keys[CLUSTER_PULL_KEYS];
	uint8_t ent_count;
	uint8_t ent_next;

	/* Push-on-change: a short FIFO of OUR OWN just-written keys, each to be
	 * broadcast once. The entry itself is read from the document at send
	 * time, so a key written twice before it airs simply broadcasts the
	 * newer version. */
	struct meshtastic_cluster_key push_keys[CLUSTER_PUSH_QUEUE];
	uint8_t push_head;
	uint8_t push_count;

	/* Bounds. */
	uint8_t serve_tokens;	    /* reply-walk budget (token bucket) */
	int64_t serve_refill_ms;    /* when the bucket last earned a token */
	uint8_t push_tokens;	    /* push budget (token bucket) */
	int64_t push_refill_ms;
	int64_t pull_hold_until_ms; /* fruitless-walk backoff */
	uint32_t pull_last_applied; /* entry_rx_applied when we last asked */
	bool pull_asked_entries;    /* the last walk actually requested entries */
	uint8_t pull_fruitless;	    /* consecutive walks that merged nothing */
	int64_t refuse_log_next_ms; /* refusal-warning throttle */
	uint32_t refuse_since_log;
} cluster;

static void my_scope_locked(struct meshtastic_cluster_scope *out);
static bool pull_start_locked(uint32_t peer, uint32_t delay_ms,
			      enum cluster_pull_reason reason);

/* How many reply walks may be served back-to-back before the rate limit bites.
 * A fleet coming up together has every member pull at once, and making them
 * queue behind each other for no reason would be a self-inflicted convergence
 * delay — the bound that matters is the SUSTAINED rate, not the first few. */
#define CLUSTER_SERVE_BURST 3U

/* Consecutive fruitless walks tolerated before backing off. Two is noise (a
 * race with a peer mid-write); three is a pattern. */
#define CLUSTER_FRUITLESS_THRESHOLD 3U
/* Backoff ceiling, in digest periods. */
#define CLUSTER_FRUITLESS_MAX_SHIFT 5U

static void digest_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(digest_work, digest_work_fn);
static void tx_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tx_work, tx_work_fn);
static void reconcile_work_fn(struct k_work *work);
static K_WORK_DEFINE(reconcile_work, reconcile_work_fn);

/* ---- the v1 allowlist (D9) ------------------------------------------------ */

/* security (private key) and network (WiFi PSK) are the named bans; everything
 * unlisted is banned by default. Enforced on BOTH sides — a doc build never
 * emits one, an ingest always refuses one, and the reconciler only ever walks
 * this list — so no single mistake can carry a secret onto the channel. */
static const uint16_t shareable_sections[] = {
	meshtastic_Config_device_tag,	 meshtastic_Config_position_tag,
	meshtastic_Config_power_tag,	 meshtastic_Config_display_tag,
	meshtastic_Config_lora_tag, /* replicable; promote refuses it separately */
	meshtastic_Config_bluetooth_tag,
};

/* A CORE-scope node holds base + its own sections, so its claim is exactly
 * twice the allowlist. A node that cannot hold its own claim cannot honestly
 * advertise one — so growing the allowlist must fail the build, not ship a
 * node that is over capacity at boot. Kconfig's `range` cannot see this array;
 * this is the gate that actually holds. */
BUILD_ASSERT(2U * ARRAY_SIZE(shareable_sections) <= CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES,
	     "MAX_ENTRIES must hold base + this node's own sections (a CORE scope)");

static bool section_shareable(uint16_t section)
{
	for (size_t i = 0; i < ARRAY_SIZE(shareable_sections); i++) {
		if (shareable_sections[i] == section) {
			return true;
		}
	}
	return false;
}

/*
 * The payload must BE the section its key claims.
 *
 * The document is deliberately payload-agnostic — pure table logic, no protobuf
 * — so if this check does not happen at the module boundary it happens nowhere,
 * and the allowlist above is tested against key.section alone. A well-formed key
 * could then carry anything: undecodable bytes, or a banned section wearing a
 * permitted key.
 *
 * Refusing at APPLY time is not enough, which is the whole point. An entry the
 * reconciler will never apply still replicates to every member, changes every
 * document hash, and at the BASE layer can never be withdrawn — base has no
 * tombstone by design (§2.1), so "no fleet default" is the key's ABSENCE and
 * there is no way to express its removal. One frame, permanent fleet-wide junk.
 *
 * Both doors use this: frames off the air, and records off flash at boot. Flash
 * is more trusted, but it is also where an entry accepted by an OLDER, laxer
 * build is still sitting — so validating on load is what makes the upgrade
 * clean out what the previous rules let in.
 */
static bool payload_is_its_section(uint16_t section, const uint8_t *payload, size_t len,
				   bool tombstone)
{
	/* Static: the callers are the RX thread (single-threaded module
	 * dispatch) and the one-shot settings load, never concurrently, and a
	 * decoded Config is a union of every section — too big for either
	 * stack. */
	static meshtastic_Config probe;
	pb_istream_t is;

	if (tombstone) {
		return len == 0U; /* nothing to check; accept() enforces the rest */
	}
	is = pb_istream_from_buffer(payload, len);
	probe = (meshtastic_Config)meshtastic_Config_init_zero;
	return pb_decode(&is, meshtastic_Config_fields, &probe) &&
	       probe.which_payload_variant == section;
}

/*
 * THE DRIFT HORIZON — an LWW-layer policy the clock deliberately left to us.
 *
 * meshtastic_hlc.h is explicit: a stamp far beyond our own clock is not allowed
 * to drag our physical component forward, but observe() does NOT reject it,
 * because "whether such a write WINS is an LWW-layer policy decision, not a
 * clock one". This is that decision, and without it the total order is a
 * weapon: one entry stamped for the year 2100 wins every comparison, forever,
 * and no honest write can ever beat it until real time catches up. Not
 * rewritten config — UNBEATABLE config. It also gives an attacker an unbounded
 * flash-write loop for free, since each ever-larger stamp is "newer" and every
 * accepted entry is persisted.
 *
 * A node whose own clock is unseeded cannot judge, and says so by accepting:
 * that is D12's designed degradation, not a hole to plug here — an unseeded
 * node has no opinion about when anything happened.
 */
static bool stamp_within_horizon(const struct meshtastic_hlc_stamp *stamp)
{
	int64_t now = meshtastic_clock_now_epoch_ms();

	if (now <= 0) {
		return true;
	}
	return stamp->physical_ms <= now + MESHTASTIC_HLC_MAX_DRIFT_MS;
}

/*
 * Count a refusal and warn about it — but not once per frame. The log stream is
 * shipped off this node over the network, so an attacker who can make us refuse
 * a frame can otherwise make us emit a packet, and a frame flood becomes a
 * packet flood aimed at the collector. First one speaks, the rest are counted
 * and summarised. Called with the lock NOT held.
 */
static void refuse_entry(const char *why, uint32_t detail)
{
	int64_t now = k_uptime_get();
	uint32_t suppressed = 0U;
	bool speak;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	cluster.stats.entry_rx_refused++;
	speak = (now >= cluster.refuse_log_next_ms);
	if (speak) {
		suppressed = cluster.refuse_since_log;
		cluster.refuse_since_log = 0U;
		cluster.refuse_log_next_ms = now + 60 * MSEC_PER_SEC;
	} else {
		cluster.refuse_since_log++;
	}
	k_mutex_unlock(&cluster_lock);

	if (speak) {
		if (suppressed > 0U) {
			LOG_WRN("cluster: refused an entry (%s, 0x%08x); %u further refusals "
				"were suppressed", why, detail, suppressed);
		} else {
			LOG_WRN("cluster: refused an entry (%s, 0x%08x)", why, detail);
		}
	}
}

/* ---- authorship (D4) ------------------------------------------------------ */

static bool author_is_master(uint32_t node_id)
{
#if defined(CONFIG_MESHTASTIC_ADMIN)
	return meshtastic_admin_node_is_trusted(node_id);
#else
	/* No admin module means no admin_key[] list to consult, so base
	 * authorship is unverifiable. Fail CLOSED: the channel PSK still bounds
	 * replication, but "anyone on the channel may rewrite fleet defaults"
	 * is not a default worth shipping. A build that wants fleet base config
	 * wants MESHTASTIC_ADMIN (default y wherever the phone API exists).
	 * Per-node entries authored by their own owner still replicate. */
	ARG_UNUSED(node_id);
	return false;
#endif
}

/*
 * Who may author what (§4):
 *
 *   base/<sec>      — a master only: the stamp's author key is in MY admin_key[].
 *   nodes/<X>/<sec> — X itself, or a master.
 *
 * Plus, at either layer, MY OWN past writes. A node restored from a factory
 * reset pulls its own entries back off its neighbours (§2.3's self-healing
 * property), and it has no reason to distrust a version it minted itself.
 *
 * What this is NOT: proof. Meshtastic PKC is X25519 ECDH *encryption* with no
 * signing primitive, so a channel member can mint a stamp claiming any author
 * and no receiver can refute it (§4 says so in as many words; ClusterEntry
 * field 15 is reserved for the signature that would fix it). The gate stops
 * accidents and honest misconfiguration, not an adversary holding the PSK.
 */
static bool entry_authorized(const struct meshtastic_cluster_key *key,
			     const struct meshtastic_hlc_stamp *stamp)
{
	uint32_t author = stamp->node_id;

	if (author == meshtastic_get_node_id()) {
		return true;
	}
	if (key->layer == MESHTASTIC_CLUSTER_LAYER_NODE && author == key->node_id) {
		return true;
	}
	return author_is_master(author);
}

/*
 * THE TABLE-EXHAUSTION GATE, and why it is separate from authorship.
 *
 * A per-node key names its owner, and nothing about the owner has to be real:
 * authorship (above) is satisfied by an author claiming to BE the owner, and a
 * channel member can claim anything (§4). So without this, one member can mint
 * entries for invented node ids until the table is full — and that damage does
 * not heal. There is no eviction (§6 defers it) and BASE entries cannot be
 * tombstoned at all, so a full table stays full and the fleet's real config can
 * no longer be stored. Rewriting config — the worse-sounding attack §4 already
 * concedes — is repairable by a master writing again. This is not.
 *
 * The gate is EXISTENCE, not permission: a per-node entry is accepted only for
 * a node this one has actually heard of, or for itself (so a wiped node can
 * still pull its own entries back — §2.3). That bounds the table by the size of
 * the real mesh instead of by an attacker's imagination. It is not a complete
 * answer — a large mesh can still outgrow a small table, and the eviction
 * policy that would finish the job is still deferred — it removes the part an
 * attacker gets for free.
 *
 * Hot tier OR warm key tier, deliberately: a node evicted from the hot store
 * whose key survives is still a node we have met, and refusing its pin would be
 * a silent, self-inflicted divergence.
 */
static bool node_owner_is_known(uint32_t node_id)
{
	struct meshtastic_nodedb_node node;
	uint8_t key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];

	if (node_id == meshtastic_get_node_id()) {
		return true;
	}
	if (meshtastic_nodedb_get(node_id, &node) == 0) {
		return true;
	}
	return meshtastic_nodedb_copy_pubkey(node_id, key) == 0;
}

/* ---- channel binding ------------------------------------------------------ */

/* The cluster channel is found by NAME at use time, never cached across
 * config writes: channels are operator-editable at runtime and a stale index
 * would aim frames at whatever channel got renumbered into the slot. */
static bool cluster_channel_index(uint8_t *out)
{
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *ch = meshtastic_channels_get(i);
		const char *name = meshtastic_channels_get_name(i);

		if (ch == NULL || !ch->has_settings ||
		    ch->role == meshtastic_Channel_Role_DISABLED) {
			continue;
		}
		if (name != NULL && strcmp(name, CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

bool meshtastic_cluster_channel_resolved(uint8_t *ch_index)
{
	uint8_t idx;
	bool ok = cluster_channel_index(&idx);

	if (ok && ch_index != NULL) {
		*ch_index = idx;
	}
	return ok;
}

/* ---- NVS persistence ------------------------------------------------------ */

#if defined(CONFIG_MESHTASTIC_SETTINGS)

/* One settings record per entry under mtclus/<l><node-hex>/<sec>; the value
 * is stamp + tombstone + payload, packed. Mirrors the hlc/config parallel-
 * subtree pattern: nothing existing is touched. */

struct cluster_rec {
	int64_t physical_ms;
	uint32_t counter;
	uint32_t author;
	uint8_t tombstone;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
} __packed;

static void rec_name(const struct meshtastic_cluster_key *key, char *buf, size_t len)
{
	snprintf(buf, len, "mtclus/%c%08x/%u",
		 key->layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n', key->node_id,
		 (unsigned int)key->section);
}

static void persist_entry(const struct meshtastic_cluster_entry *e)
{
	struct cluster_rec rec = {
		.physical_ms = e->stamp.physical_ms,
		.counter = e->stamp.counter,
		.author = e->stamp.node_id,
		.tombstone = e->tombstone ? 1U : 0U,
	};
	char name[40];
	size_t len = offsetof(struct cluster_rec, payload) + e->payload_len;

	memcpy(rec.payload, e->payload, e->payload_len);
	rec_name(&e->key, name, sizeof(name));
	if (settings_save_one(name, &rec, len) != 0) {
		LOG_WRN("cluster: persist of %s failed", name);
	}
}

/*
 * Forget one key in flash. The retain() callback, so a narrowed claim does not
 * quietly refill itself from NVS on the next boot.
 *
 * Correctness must not depend on these deletes landing. A crash part-way leaves
 * records for keys we no longer claim, and the LOAD PATH's scope filter is what
 * makes that harmless — the same reasoning that already re-runs the allowlist
 * on load so an upgrade drops what a laxer build accepted. These deletes are
 * flash housekeeping, not a correctness step.
 */
static void forget_entry(const struct meshtastic_cluster_key *key, void *ctx)
{
	char name[40];

	ARG_UNUSED(ctx);
	rec_name(key, name, sizeof(name));
	(void)settings_delete(name);
}

/*
 * The claim, remembered. A node that narrowed because it genuinely could not
 * hold the fleet's document would otherwise refill from the fleet on every
 * boot, overflow again, and evict again — and since every accepted entry is
 * persisted, that is flash WEAR repeated every boot on the weakest node in the
 * fleet. That is what settles this: not the airtime, the wear.
 *
 * The obvious objection to remembering it is sticky-wrong state, and the cap
 * recorded alongside is what kills that cheaply. Raise MAX_ENTRIES, reflash,
 * and the record is discarded on sight — "give it a bigger table" becomes the
 * natural cure rather than something an operator has to know to undo.
 */
struct cluster_scope_rec {
	uint8_t kind;
	uint16_t cap;
} __packed;

#define CLUSTER_SCOPE_REC_NAME "mtclus/scope"

static void persist_scope(uint8_t kind)
{
	struct cluster_scope_rec rec = {
		.kind = kind,
		.cap = (uint16_t)CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES,
	};

	if (settings_save_one(CLUSTER_SCOPE_REC_NAME, &rec, sizeof(rec)) != 0) {
		LOG_WRN("cluster: could not record the scope — it will not survive a reboot");
	}
}

static void forget_scope(void)
{
	(void)settings_delete(CLUSTER_SCOPE_REC_NAME);
}

/*
 * Forget records in flash that the TABLE does not know about.
 *
 * Deleting what the table holds covers the ordinary case, but not every record
 * has a table entry: one refused by the load-path checks (an older, laxer build
 * wrote it) or dropped for capacity is skipped at boot and then invisible. A
 * "clean" that leaves those behind is not clean — they come back the moment the
 * rules or the capacity change.
 *
 * Collected then deleted, a bounded batch at a time: deleting from inside the
 * backend's own walk is not safe, and buffering every possible key would put
 * MAX_ENTRIES × a name on the caller's stack.
 */
#define CLUSTER_SWEEP_BATCH 8U

struct cluster_sweep {
	char names[CLUSTER_SWEEP_BATCH][40];
	uint8_t count;
};

static int sweep_collect(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			 void *param)
{
	struct cluster_sweep *sw = param;

	ARG_UNUSED(len);
	ARG_UNUSED(read_cb);
	ARG_UNUSED(cb_arg);

	if (sw->count < CLUSTER_SWEEP_BATCH) {
		snprintf(sw->names[sw->count], sizeof(sw->names[0]), "mtclus/%s", key);
		sw->count++;
	}
	return 0;
}

static void forget_subtree(void)
{
	/* One pass per batch, and a cap so a backend that never stops handing the
	 * same key back cannot spin here forever. */
	for (uint16_t pass = 0U;
	     pass < (CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES / CLUSTER_SWEEP_BATCH) + 4U; pass++) {
		struct cluster_sweep sw = {.count = 0U};

		(void)settings_load_subtree_direct("mtclus", sweep_collect, &sw);
		if (sw.count == 0U) {
			return;
		}
		for (uint8_t i = 0U; i < sw.count; i++) {
			(void)settings_delete(sw.names[i]);
		}
	}
	LOG_WRN("cluster: the persisted document did not empty — records may remain");
}

static int cluster_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				void *cb_arg)
{
	struct cluster_rec rec;
	struct meshtastic_cluster_key k;
	struct meshtastic_hlc_stamp stamp;
	unsigned int node_id;
	unsigned int section;
	char layer;
	size_t plen;

	/* The claim comes first — it is not a key, and it decides which keys are
	 * even loadable below. Init loads this record's own subtree before the
	 * entries so the order is not left to however flash happens to be laid
	 * out. */
	if (strcmp(key, "scope") == 0) {
		struct cluster_scope_rec srec;

		if (len != sizeof(srec) || read_cb(cb_arg, &srec, len) != (ssize_t)len) {
			return -EINVAL;
		}
		if (!IS_ENABLED(CONFIG_MESHTASTIC_CLUSTER_SCOPE_AUTO)) {
			/* A compiled choice is an explicit statement about THIS
			 * build; a record is a decision an earlier one made. The
			 * build wins, or "never narrow" would not mean it. */
			return 0;
		}
		if (CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES > srec.cap) {
			LOG_INF("cluster: capacity grew from %u to %u — discarding the "
				"recorded scope and claiming everything again",
				srec.cap,
				(unsigned int)CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES);
			forget_scope();
			return 0;
		}
		if (srec.kind == MESHTASTIC_CLUSTER_SCOPE_CORE ||
		    srec.kind == MESHTASTIC_CLUSTER_SCOPE_FULL) {
			k_mutex_lock(&cluster_lock, K_FOREVER);
			cluster.scope_kind = srec.kind;
			k_mutex_unlock(&cluster_lock);
		}
		return 0;
	}

	/* key is the remainder after "mtclus/": "<l><node-hex>/<sec>" */
	if (sscanf(key, "%c%8x/%u", &layer, &node_id, &section) != 3) {
		return -ENOENT;
	}
	if (len < offsetof(struct cluster_rec, payload) || len > sizeof(rec)) {
		return -EINVAL;
	}
	if (read_cb(cb_arg, &rec, len) != (ssize_t)len) {
		return -EINVAL;
	}

	k.layer = (layer == 'b') ? MESHTASTIC_CLUSTER_LAYER_BASE : MESHTASTIC_CLUSTER_LAYER_NODE;
	k.node_id = node_id;
	k.section = (uint16_t)section;
	stamp.physical_ms = rec.physical_ms;
	stamp.counter = rec.counter;
	stamp.node_id = rec.author;
	plen = len - offsetof(struct cluster_rec, payload);

	/* Flash is more trusted than the air, but not exempt: these same records
	 * may have been accepted by an older build with laxer rules, so the load
	 * path runs the static checks too and an upgrade quietly drops what it
	 * should never have stored. Authorship is deliberately NOT re-checked
	 * here — it depends on the NodeDB, and refusing a persisted entry because
	 * we have not re-learned a master's key yet would throw away this node's
	 * own configuration on every cold boot. */
	if (!section_shareable(k.section) ||
	    !payload_is_its_section(k.section, rec.payload, plen, rec.tombstone != 0U)) {
		LOG_WRN("cluster: dropping persisted %s — it does not pass the current "
			"ingest rules", key);
		return 0;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	{
		/* The same scope filter the air is held to. A record for a key
		 * this build no longer claims must not come back at boot: it
		 * would put the table out of step with the digest before the
		 * first frame goes out, and on a narrowed node it is also how
		 * the table refills itself into overflow again. */
		struct meshtastic_cluster_scope mine;

		my_scope_locked(&mine);
		if (!meshtastic_cluster_scope_contains(&mine, &k)) {
			k_mutex_unlock(&cluster_lock);
			forget_entry(&k, NULL);
			return 0;
		}
	}
	(void)meshtastic_cluster_doc_accept(&cluster.doc, &k, &stamp, rec.tombstone != 0U,
					    rec.payload, plen);
	(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
	k_mutex_unlock(&cluster_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtclus, "mtclus", NULL, cluster_settings_set, NULL, NULL);

#else /* !CONFIG_MESHTASTIC_SETTINGS */

/* No settings backend: the doc is RAM-only and re-converges from the fleet
 * after a reboot — slower, never wrong (level-triggered). */
static void persist_entry(const struct meshtastic_cluster_entry *e)
{
	ARG_UNUSED(e);
}

static void forget_entry(const struct meshtastic_cluster_key *key, void *ctx)
{
	ARG_UNUSED(key);
	ARG_UNUSED(ctx);
}

static void forget_subtree(void)
{
}

static void persist_scope(uint8_t kind)
{
	ARG_UNUSED(kind);
}

static void forget_scope(void)
{
}

#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* ---- the reconciler (D10) -------------------------------------------------- */

/*
 * THE ORIGIN MARKER.
 *
 * D10 requires that a store write the document caused is distinguishable from a
 * human's local edit, so the former can never be lifted back into the document
 * (every applied entry would otherwise mint a fresh stamp and gossip forever).
 * The marker is not a side flag: a doc-derived write copies the DOCUMENT's
 * stamp into the store slot verbatim — merge_config takes the stamp — so
 * "this section's value came from the document" is a fact the two stamps state
 * themselves.
 *
 * That is worth more than a bool. There is nothing to keep in sync, nothing to
 * lose across a reboot (both stamps already persist), and it cannot be forged
 * by accident: a local edit mints a fresh stamp carrying THIS node's id and its
 * own counter, which can never equal a stamp minted on another node.
 */
static bool section_is_doc_derived(uint16_t section, const struct meshtastic_hlc_stamp *doc_stamp)
{
	struct meshtastic_hlc_stamp store_stamp;

	if (meshtastic_config_store_get_config_stamp((pb_size_t)section, &store_stamp) != 0) {
		return false;
	}
	return meshtastic_hlc_compare(&store_stamp, doc_stamp) == 0;
}

static void reconcile_section(uint16_t section)
{
	/* Static scratch, not stack: this runs on the system workqueue (the one
	 * whose 2 KB nRF default the M4a digest already overflowed once), the
	 * decoded Config is a union of every section, and merge_config's apply
	 * path puts a full channel table on the stack below us. The workqueue is
	 * a single executor, so one copy is all that is ever needed. */
	static uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	static meshtastic_Config cfg;
	const struct meshtastic_cluster_entry *e;
	struct meshtastic_hlc_stamp version;
	uint16_t payload_len;
	pb_istream_t is;
	int ret;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	e = meshtastic_cluster_doc_effective(&cluster.doc, meshtastic_get_node_id(), section);
	if (e == NULL ||
	    !meshtastic_cluster_doc_effective_version(&cluster.doc, meshtastic_get_node_id(),
						      section, &version)) {
		/* The document holds no opinion about this section — not the
		 * same as holding an empty one. Local config stands untouched.
		 * A tombstoned pin with no fleet base underneath lands here
		 * too: there is nothing to revert TO, so unpinning stops the
		 * advertisement and leaves the node running what it has. */
		k_mutex_unlock(&cluster_lock);
		return;
	}
	payload_len = e->payload_len;
	memcpy(payload, e->payload, payload_len);
	k_mutex_unlock(&cluster_lock);

	/*
	 * The version is the newest stamp across BOTH layers, not the winning
	 * entry's own — see meshtastic_cluster_doc_effective_version(). It is
	 * what carries an `unpin` through: the tombstone erases the pin's stamp
	 * from the document, so scoring the reversion by the base entry's own
	 * (older) stamp would lose the store's LWW merge and leave this node
	 * running the value it was just told to stop running.
	 */
	if (section_is_doc_derived(section, &version)) {
		return; /* already applied; re-writing would only churn */
	}

	/*
	 * THE SECTION THIS NODE REPLICATES BUT REFUSES TO ACT ON.
	 *
	 * `promote` will not CREATE a base/lora entry, because a fleet preset
	 * change permanently orphans any node that misses it and the straggler
	 * sweep (CONFIG-CONVERGENCE.md §7.9) does not exist yet. Nothing stops
	 * one ARRIVING, though, and applying it re-keys the radio: new preset,
	 * new channel hashes, node gone from the mesh. One frame from anyone
	 * holding the cluster PSK would take the whole fleet off the air, and
	 * the nodes that missed the frame could not be told.
	 *
	 * So the two halves are kept apart. The document is replicated state
	 * and this entry stays in it — refusing it at ingest instead would
	 * strand this node's document from the fleet's forever, trading a
	 * recoverable problem for a permanent one. The reconciler is the
	 * actuator, and this is where the trigger does not get pulled.
	 *
	 * When the straggler sweep lands, this is the gate that opens.
	 */
	if (section == meshtastic_Config_lora_tag) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		if (cluster.stats.sections_held == 0U) {
			k_mutex_unlock(&cluster_lock);
			LOG_WRN("cluster: HOLDING a fleet LoRa section — replicated, "
				"deliberately not applied (a missed preset change orphans a "
				"node until the straggler sweep exists)");
			k_mutex_lock(&cluster_lock, K_FOREVER);
		}
		cluster.stats.sections_held++;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	cfg = (meshtastic_Config)meshtastic_Config_init_zero;
	is = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&is, meshtastic_Config_fields, &cfg) ||
	    cfg.which_payload_variant != section) {
		LOG_WRN("cluster: entry for section %u does not decode as that section",
			(unsigned int)section);
		return;
	}

	ret = meshtastic_config_store_merge_config(&cfg, &version);
	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (ret == 1) {
		cluster.stats.sections_applied++;
	} else if (ret == 0) {
		cluster.stats.sections_kept_local++;
	}
	k_mutex_unlock(&cluster_lock);

	if (ret == 1) {
		LOG_INF("cluster: applied section %u from the document (stamp %lld.%u by "
			"0x%08x)",
			(unsigned int)section, (long long)version.physical_ms, version.counter,
			version.node_id);
	} else if (ret == 0) {
		/* Not a failure and not a retry loop: LWW said our own version
		 * is newer, so ours stands and our digest invites the fleet to
		 * take it. The operator's route to make it fleet-wide is an
		 * explicit promote (or, in M4c, a pin). */
		LOG_DBG("cluster: section %u kept (local version is newer than the document)",
			(unsigned int)section);
	} else {
		LOG_WRN("cluster: section %u merge failed (%d)", (unsigned int)section, ret);
	}
}

/* Recompute effective(me) across the whole allowlist and write what changed.
 * Cheap enough to run wholesale on any document change: six decodes, and every
 * one of them short-circuits on the stamp comparison once applied. */
static void reconcile_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	for (size_t i = 0; i < ARRAY_SIZE(shareable_sections); i++) {
		reconcile_section(shareable_sections[i]);
	}
}

/* ---- TX ------------------------------------------------------------------- */

static int send_cluster_message(const zephyrtastic_ClusterMessage *msg, uint32_t to,
				uint8_t ch_index)
{
	/* Static, not stack: every caller runs on the single system workqueue
	 * (the digest timer and the anti-entropy walk), which serialises them by
	 * construction — and this path continues into channel encryption, whose
	 * PSA stack appetite is exactly what ejected the first bench digest on a
	 * 2 KB sysworkq. Keep the frame off the stack. */
	static uint8_t buf[zephyrtastic_ClusterMessage_size];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};

	if (!pb_encode(&os, zephyrtastic_ClusterMessage_fields, msg)) {
		LOG_ERR("cluster: encode failed: %s", PB_GET_ERROR(&os));
		return -EINVAL;
	}

	pkt.portnum = MESHTASTIC_PORT_PRIVATE;
	pkt.from = meshtastic_get_node_id();
	pkt.to = to;
	pkt.channel_index = ch_index;
	pkt.payload = buf;
	pkt.payload_len = os.bytes_written;
	return meshtastic_send_packet(&pkt, K_NO_WAIT);
}

static void stamp_to_pb(const struct meshtastic_hlc_stamp *in, zephyrtastic_HlcStamp *out)
{
	out->physical_ms = in->physical_ms;
	out->counter = in->counter;
	out->node_id = in->node_id;
}

static void stamp_from_pb(const zephyrtastic_HlcStamp *in, struct meshtastic_hlc_stamp *out)
{
	out->physical_ms = in->physical_ms;
	out->counter = in->counter;
	out->node_id = in->node_id;
}

static void key_to_pb(const struct meshtastic_cluster_key *in, zephyrtastic_ClusterKey *out)
{
	out->layer = (in->layer == MESHTASTIC_CLUSTER_LAYER_BASE) ? zephyrtastic_ClusterLayer_BASE
								  : zephyrtastic_ClusterLayer_NODE;
	out->node_id = in->node_id;
	out->section = in->section;
}

static void key_from_pb(const zephyrtastic_ClusterKey *in, struct meshtastic_cluster_key *out)
{
	out->layer = (in->layer == zephyrtastic_ClusterLayer_BASE)
			     ? MESHTASTIC_CLUSTER_LAYER_BASE
			     : MESHTASTIC_CLUSTER_LAYER_NODE;
	out->node_id = in->node_id;
	/* A section tag wider than the doc's key field could alias a different
	 * section; refuse it by clamping to something the allowlist rejects. */
	out->section = (in->section <= UINT16_MAX) ? (uint16_t)in->section : 0U;
}

/*
 * The scope a digest sender is claiming. The OWNER comes from the mesh header,
 * never from the frame: a wire-carried owner would let one node advertise a
 * claim over another node's rows, and every receiver would believe it.
 *
 * A sender predating scopes says nothing, which decodes as SCOPE_UNSPECIFIED.
 * Its first three legs do cover its whole document, so treat it as FULL — but
 * it carries no base leg, and *has_base_leg says so. Getting that wrong is not
 * cosmetic: with SCOPE_FULL == 0 an absent field would read as "FULL with a
 * base hash of zero", and a narrowed receiver would mismatch such a peer
 * forever — exactly the permanent divergence this change exists to remove.
 */
static void scope_from_pb(zephyrtastic_ClusterScope in, uint32_t from,
			  struct meshtastic_cluster_scope *out, bool *has_base_leg)
{
	switch (in) {
	case zephyrtastic_ClusterScope_SCOPE_CORE:
		meshtastic_cluster_scope_make(out, MESHTASTIC_CLUSTER_SCOPE_CORE, from);
		*has_base_leg = true;
		break;
	case zephyrtastic_ClusterScope_SCOPE_FULL:
		meshtastic_cluster_scope_make(out, MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
		*has_base_leg = true;
		break;
	default:
		meshtastic_cluster_scope_make(out, MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
		*has_base_leg = false;
		break;
	}
}

static zephyrtastic_ClusterScope scope_to_pb(const struct meshtastic_cluster_scope *scope)
{
	return (scope->kind == MESHTASTIC_CLUSTER_SCOPE_CORE)
		       ? zephyrtastic_ClusterScope_SCOPE_CORE
		       : zephyrtastic_ClusterScope_SCOPE_FULL;
}

static bool scope_eq(const struct meshtastic_cluster_scope *a,
		     const struct meshtastic_cluster_scope *b)
{
	return (a->kind == b->kind) && (a->owner == b->owner);
}

static void base_scope(struct meshtastic_cluster_scope *out)
{
	meshtastic_cluster_scope_make(out, MESHTASTIC_CLUSTER_SCOPE_BASE, 0U);
}

/*
 * The scope this node advertises, resolved now. CORE names an owner and the
 * owner is this node's id, which the module cannot assume it had at init — so
 * resolve at use rather than caching a scope with a stale owner in it.
 *
 * A CORE claim with no node id would be a claim over nobody's entries, which
 * would evict this node's own rows and advertise a document it does not hold.
 * Fall back to FULL and say so once: over-claiming is recoverable (the walk
 * simply refuses what it cannot store), under-claiming your own keys is not.
 */
static void my_scope_locked(struct meshtastic_cluster_scope *out)
{
	uint32_t me = meshtastic_get_node_id();

	if (cluster.scope_kind == MESHTASTIC_CLUSTER_SCOPE_CORE && me == 0U) {
		if (!cluster.scope_id_warned) {
			cluster.scope_id_warned = true;
			LOG_WRN("cluster: CORE scope asked for before this node has an id — "
				"claiming FULL until it does");
		}
		meshtastic_cluster_scope_make(out, MESHTASTIC_CLUSTER_SCOPE_FULL, 0U);
		return;
	}
	meshtastic_cluster_scope_make(out, cluster.scope_kind, me);
}

static void pull_reset_locked(void)
{
	cluster.pull_state = PULL_IDLE;
	cluster.pull_peer = 0U;
	cluster.pull_key_count = 0U;
	cluster.pull_deadline_ms = 0;
}

/* ---- push-on-change (D11) -------------------------------------------------- */

/*
 * THE BOUND ON THE ONLY FRAMES WE ORIGINATE UNINVITED.
 *
 * Everything else this module transmits is a timer (the digest) or an answer to
 * a request, and both are bounded by something the other end cannot influence.
 * A push is different: it is caused by a LOCAL EVENT, and local events have no
 * natural rate. An operator on the up-arrow, a script in a loop, a config that
 * flaps between two values — each would otherwise be a broadcast per write, on
 * a shared channel, flood-relayed by every member.
 *
 * The budget is spent at ENQUEUE, and over budget the push is DROPPED rather
 * than deferred. Deferring would only rename the backlog: the writes keep
 * coming, the queue grows, and the frames still air eventually. Dropping is
 * safe here in a way it is nowhere else in this module, because the push is
 * pure latency optimisation — the entry is already in the document, the digest
 * already advertises it, and a peer that misses the broadcast pulls it on the
 * next period. Correctness never depends on a push arriving, which is exactly
 * why one may be thrown away.
 *
 * A bucket rather than a flat interval, for the same reason as the serve limit:
 * setting a node up means several legitimate writes in a row.
 *
 * Called with the lock held.
 */
static bool may_push_locked(void)
{
	int64_t now = k_uptime_get();
	int64_t elapsed;

	if (CONFIG_MESHTASTIC_CLUSTER_PUSH_MIN_INTERVAL_MS == 0) {
		return true; /* limit disabled (tests) */
	}

	elapsed = now - cluster.push_refill_ms;
	if (elapsed >= CONFIG_MESHTASTIC_CLUSTER_PUSH_MIN_INTERVAL_MS) {
		uint32_t gained =
			(uint32_t)(elapsed / CONFIG_MESHTASTIC_CLUSTER_PUSH_MIN_INTERVAL_MS);

		cluster.push_tokens =
			(uint8_t)MIN(cluster.push_tokens + gained, CLUSTER_PUSH_BURST);
		cluster.push_refill_ms +=
			(int64_t)gained * CONFIG_MESHTASTIC_CLUSTER_PUSH_MIN_INTERVAL_MS;
	}
	return cluster.push_tokens > 0U;
}

/*
 * Queue ONE key of ours for a single broadcast. Called with the lock held,
 * immediately after the local write that changed it.
 *
 * Only keys written HERE ever reach this. An entry that merely arrived is never
 * re-broadcast: that would turn one write into a flood of re-pushes proportional
 * to fleet size, and the anti-entropy walk already covers everyone the original
 * broadcast missed.
 */
static void push_enqueue_locked(const struct meshtastic_cluster_key *key)
{
	uint8_t idx;

	/* Already queued? The frame is built from the document at SEND time, so
	 * the push still pending will carry this newer version anyway — a second
	 * slot would only broadcast the same key twice. Counted as suppressed
	 * because that is what the counter means to a reader: this write did not
	 * get a broadcast of its own. push_tx + push_suppressed is then exactly
	 * the number of local writes, with nothing unaccounted for. */
	for (uint8_t i = 0U; i < cluster.push_count; i++) {
		idx = (uint8_t)((cluster.push_head + i) % CLUSTER_PUSH_QUEUE);
		if (meshtastic_cluster_key_cmp(&cluster.push_keys[idx], key) == 0) {
			cluster.stats.push_suppressed++;
			return;
		}
	}

	if (cluster.push_count >= CLUSTER_PUSH_QUEUE || !may_push_locked()) {
		cluster.stats.push_suppressed++;
		return;
	}

	if (cluster.push_tokens > 0U) {
		cluster.push_tokens--;
	}
	idx = (uint8_t)((cluster.push_head + cluster.push_count) % CLUSTER_PUSH_QUEUE);
	cluster.push_keys[idx] = *key;
	cluster.push_count++;
}

/*
 * Pick ONE outbound job and describe it in @p msg; returns the destination, or
 * 0 when there is nothing to send. Called with cluster_lock held.
 *
 * Replies outrank our own requests. A peer blocked on a reply we owe it loses a
 * whole digest period if we make it wait, whereas deferring our own pull by one
 * frame gap costs nothing — we are already behind and staying behind a moment
 * longer changes nothing.
 */
static uint32_t tx_next_locked(zephyrtastic_ClusterMessage *msg)
{
	*msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;

	/* 1. An entry a peer asked for. */
	while (cluster.ent_dest != 0U && cluster.ent_next < cluster.ent_count) {
		const struct meshtastic_cluster_entry *e = meshtastic_cluster_doc_find(
			&cluster.doc, &cluster.ent_keys[cluster.ent_next]);
		uint32_t dest = cluster.ent_dest;

		cluster.ent_next++;
		if (cluster.ent_next >= cluster.ent_count) {
			cluster.ent_dest = 0U;
		}
		if (e == NULL) {
			/* Gone between request and reply. Not sending it is the
			 * whole recovery: the requester's next digest asks again. */
			continue;
		}

		msg->which_variant = zephyrtastic_ClusterMessage_entry_tag;
		key_to_pb(&e->key, &msg->variant.entry.key);
		msg->variant.entry.has_key = true;
		stamp_to_pb(&e->stamp, &msg->variant.entry.stamp);
		msg->variant.entry.has_stamp = true;
		msg->variant.entry.tombstone = e->tombstone;
		msg->variant.entry.payload.size = e->payload_len;
		memcpy(msg->variant.entry.payload.bytes, e->payload, e->payload_len);
		cluster.stats.entry_tx++;
		return dest;
	}
	cluster.ent_dest = 0U;

	/* 2. The next chunk of a stamp vector a peer asked for. Sent even when
	 * the document is empty (total = 0): the requester needs the reply to
	 * know there is nothing to pull, not a silence to time out on.
	 *
	 * The walk resumes by INDEX, so an entry arriving mid-walk (the table is
	 * sorted, so an insert shifts everything after it) can make one chunk
	 * skip or repeat a row. That costs a round of latency and nothing else:
	 * a repeated row is diffed to "already have it", a skipped one still
	 * differs and the next digest says so. Freezing a snapshot to avoid it
	 * would cost a second copy of the document to defend against a
	 * self-correcting inaccuracy. */
	if (cluster.vec_dest != 0U) {
		zephyrtastic_ClusterVector *v = &msg->variant.vector;
		uint32_t dest = cluster.vec_dest;
		struct meshtastic_cluster_scope want;

		/* Serve only what the REQUESTER claims. Two cursors, because
		 * offset/total are the requester's coordinates and must count
		 * the rows it can actually see: vec_emitted is what goes on the
		 * wire, vec_offset is where to resume in our own table. Mixing
		 * them breaks the receiver's end-of-walk test. */
		meshtastic_cluster_scope_make(&want, cluster.vec_scope_kind, dest);

		msg->which_variant = zephyrtastic_ClusterMessage_vector_tag;
		v->offset = cluster.vec_emitted;
		v->total = meshtastic_cluster_doc_count_scoped(&cluster.doc, &want);
		while (v->entries_count < CLUSTER_VECTOR_ROWS &&
		       cluster.vec_offset < cluster.doc.count) {
			const struct meshtastic_cluster_entry *e =
				&cluster.doc.entries[cluster.vec_offset];
			zephyrtastic_KeyStamp *row;

			cluster.vec_offset++;
			if (!meshtastic_cluster_scope_contains(&want, &e->key)) {
				continue;
			}
			row = &v->entries[v->entries_count];
			key_to_pb(&e->key, &row->key);
			row->has_key = true;
			stamp_to_pb(&e->stamp, &row->stamp);
			row->has_stamp = true;
			v->entries_count++;
			cluster.vec_emitted++;
		}
		if (cluster.vec_offset >= cluster.doc.count) {
			cluster.vec_dest = 0U;
		}
		cluster.stats.vector_tx++;
		return dest;
	}

	/* 3. Our own entry request, once the vector told us which keys to want. */
	if (cluster.pull_state == PULL_SEND_ENTRY_REQ) {
		zephyrtastic_ClusterEntryReq *r = &msg->variant.entry_req;
		uint32_t dest = cluster.pull_peer;

		msg->which_variant = zephyrtastic_ClusterMessage_entry_req_tag;
		for (uint8_t i = 0U; i < cluster.pull_key_count; i++) {
			key_to_pb(&cluster.pull_keys[i], &r->keys[i]);
			r->keys_count++;
		}
		/* The exchange ends here from our side. Entries arrive when they
		 * arrive; nothing waits on them, and whatever does not turn up
		 * still differs at the next digest. Remember that we ASKED,
		 * though: the next pull judges this one by whether anything
		 * actually merged in the meantime. */
		cluster.pull_asked_entries = true;
		cluster.pull_last_applied = cluster.stats.entry_rx_applied;
		pull_reset_locked();
		return dest;
	}

	/* 4. Our own vector request, carrying what we claim so the responder
	 * does not walk us through rows we would refuse at ingest anyway. */
	if (cluster.pull_state == PULL_SEND_VECTOR_REQ) {
		struct meshtastic_cluster_scope mine;

		my_scope_locked(&mine);
		msg->which_variant = zephyrtastic_ClusterMessage_vector_req_tag;
		msg->variant.vector_req.scope = scope_to_pb(&mine);
		cluster.pull_state = PULL_AWAIT_VECTOR;
		return cluster.pull_peer;
	}

	/*
	 * 5. A change of ours, broadcast once (D11). LAST deliberately: it is
	 * the only frame in this module that nobody is waiting for. Everything
	 * above either unblocks a peer or advances our own convergence, and a
	 * push that waits a frame gap behind them costs nothing — the digest is
	 * already the guarantee.
	 */
	while (cluster.push_count > 0U) {
		const struct meshtastic_cluster_key *key = &cluster.push_keys[cluster.push_head];
		const struct meshtastic_cluster_entry *e =
			meshtastic_cluster_doc_find(&cluster.doc, key);

		cluster.push_head = (uint8_t)((cluster.push_head + 1U) % CLUSTER_PUSH_QUEUE);
		cluster.push_count--;
		if (e == NULL) {
			continue; /* superseded out of existence; nothing to say */
		}

		msg->which_variant = zephyrtastic_ClusterMessage_entry_tag;
		key_to_pb(&e->key, &msg->variant.entry.key);
		msg->variant.entry.has_key = true;
		stamp_to_pb(&e->stamp, &msg->variant.entry.stamp);
		msg->variant.entry.has_stamp = true;
		msg->variant.entry.tombstone = e->tombstone;
		msg->variant.entry.payload.size = e->payload_len;
		memcpy(msg->variant.entry.payload.bytes, e->payload, e->payload_len);
		cluster.stats.push_tx++;
		/* A BROADCAST, so it rides the cluster channel and is flood-
		 * relayed — one frame reaches the fleet instead of N unicasts.
		 * On a node whose radio TX is off it airs nowhere and no peer
		 * bearer carries it either (broadcasts are not diverted), which
		 * is not a special case to fix: that node's next digest
		 * advertises the entry and the fleet pulls it, one period later
		 * (§3.2). */
		return MESHTASTIC_NODE_BROADCAST;
	}

	return 0U;
}

static void tx_work_fn(struct k_work *work)
{
	/* Static for the same sysworkq-stack reason as send_cluster_message's
	 * buffer (single executor serialises access). */
	static zephyrtastic_ClusterMessage msg;
	uint8_t ch_index;
	uint32_t dest;
	int64_t now = k_uptime_get();
	k_timeout_t again = K_NO_WAIT;
	bool reschedule = false;

	ARG_UNUSED(work);

	if (!cluster_channel_index(&ch_index)) {
		/* The channel was renamed or disabled mid-walk. Drop everything
		 * pending rather than aim frames at whatever now sits in the
		 * slot; the digest timer re-announces when a channel reappears. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		pull_reset_locked();
		cluster.vec_dest = 0U;
		cluster.ent_dest = 0U;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (cluster.pull_state == PULL_AWAIT_VECTOR && now > cluster.pull_deadline_ms) {
		LOG_INF("cluster: no vector from 0x%08x — giving up this round",
			cluster.pull_peer);
		cluster.stats.pull_timed_out++;
		pull_reset_locked();
	}
	dest = tx_next_locked(&msg);
	if (dest != 0U) {
		/* Something else may still be queued behind this frame; space
		 * them so a multi-frame reply does not seize the airtime the
		 * peer needs to answer. */
		reschedule = true;
		again = K_MSEC(CONFIG_MESHTASTIC_CLUSTER_TX_GAP_MS);
	} else if (cluster.pull_state == PULL_AWAIT_VECTOR) {
		/* Nothing to send, but a deadline to enforce. Capped at a second
		 * rather than sleeping to the deadline: a local write can queue
		 * a push at any moment, and the pushers deliberately do not
		 * reschedule a worker that is already pending (that would let an
		 * optional frame collapse the spacing of a reply burst). Waking
		 * to re-check costs one workqueue slot a second for at most the
		 * sync timeout. */
		reschedule = true;
		again = K_MSEC(CLAMP(cluster.pull_deadline_ms - now, 100, 1000));
	}
	k_mutex_unlock(&cluster_lock);

	if (dest != 0U) {
		int ret = send_cluster_message(&msg, dest, ch_index);

		if (ret == -EACCES) {
			/* The send path refuses to downgrade a unicast to a
			 * node whose public key we lack into channel traffic.
			 * Nothing to work around here: NodeInfo exchange
			 * supplies the key, and the walk reruns on the next
			 * digest. */
			LOG_INF("cluster: no public key for 0x%08x yet — the walk resumes once "
				"its NodeInfo arrives", dest);
		} else if (ret != 0) {
			LOG_WRN("cluster: send to 0x%08x failed (%d)", dest, ret);
		}
	}
	if (reschedule) {
		(void)k_work_reschedule(&tx_work, again);
	}
}

/* ---- the declared scope, and narrowing it --------------------------------- */

static void forget_entry(const struct meshtastic_cluster_key *key, void *ctx);

/*
 * Change what this node claims, and make the table match the claim in the same
 * breath. Called with the lock held.
 *
 * The order matters and is the opposite of the intuitive one: narrow the CLAIM
 * first, then drop the rows. A digest that went out in between would advertise
 * the old claim over the new content — briefly describing a document nobody
 * has, which is precisely the state this whole mechanism exists to make
 * impossible. Both happen under the lock, so no digest can be built between
 * them.
 *
 * Eviction is the whole of the policy: one pass, no victim choice, no LRU. A
 * per-entry policy is exactly what makes the held set diverge from the declared
 * set, and only the declared set can be honestly advertised.
 *
 * No reconcile is submitted, deliberately. CORE(me) is exactly the closure of
 * effective(me, ·), so narrowing cannot change what this node runs, and a
 * reconcile pass would be pure churn — a claim about the fleet, not about us.
 */
static void scope_set_locked(uint8_t kind, bool pinned)
{
	struct meshtastic_cluster_scope now;
	uint16_t dropped;

	cluster.scope_kind = kind;
	cluster.scope_pinned = pinned;
	my_scope_locked(&now);

	dropped = meshtastic_cluster_doc_retain(&cluster.doc, &now, forget_entry, NULL);
	cluster.stats.entries_evicted += dropped;

	if (dropped > 0U) {
		/* An in-flight exchange may be carrying keys we have just
		 * stopped claiming, and a walk in progress is describing a
		 * document that no longer exists. Both are cheap to abandon:
		 * the next digest re-opens whatever still matters. */
		pull_reset_locked();
		cluster.vec_dest = 0U;
		cluster.ent_dest = 0U;
		cluster.ent_count = 0U;
		cluster.ent_next = 0U;
	}
}

/*
 * Narrow the claim because the table could not hold something we wanted.
 * Called with the lock held.
 *
 * WHICH SIGNALS MAY DO THIS, and the asymmetry is deliberate. A row the diff
 * predicate wanted, or one of our OWN writes, is evidence about this node: we
 * chose the peer, or we made the write, and running out of table is then a fact
 * about our capacity. An UNSOLICITED push that will not fit is not evidence
 * about anything — it is an arbitrary frame from whoever holds the channel key,
 * and letting it force a narrowing would hand a channel member a cheap way to
 * knock the fleet's restore source out of service. Nothing is lost by ignoring
 * it: if the entry is real, the peer's digest still mismatches and our own walk
 * reaches the same conclusion on our own terms.
 *
 * One way only. Re-widening on its own would refill, overflow and narrow again
 * on a cycle, flipping the advertised claim each time and forcing every peer to
 * switch which leg it compares. Widening again is an operator's act
 * (`cluster scope auto|full`) or a bigger table plus a reflash, which the
 * recorded capacity turns into an automatic cure.
 *
 * It can fire at most once: a CORE claim is at most twice the allowlist, and
 * MAX_ENTRIES has a floor of exactly that, so a narrowed node can always hold
 * everything it claims.
 */
static void demote_locked(const char *why)
{
	if (cluster.scope_pinned || cluster.scope_kind == MESHTASTIC_CLUSTER_SCOPE_CORE) {
		return;
	}

	scope_set_locked(MESHTASTIC_CLUSTER_SCOPE_CORE, false);
	cluster.stats.scope_demotions++;
	persist_scope(MESHTASTIC_CLUSTER_SCOPE_CORE);

	LOG_WRN("cluster: table full (%u) — %s. Now claiming CORE: base + my own sections. "
		"What this node RUNS is unchanged; the fleet has one fewer place to recover "
		"another node's pins from. Raise MESHTASTIC_CLUSTER_MAX_ENTRIES to undo it.",
		(unsigned int)CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES, why);

	/* Say so at once: every peer's comparison against this node has just
	 * changed which set it covers. */
	(void)k_work_reschedule(&digest_work, K_NO_WAIT);
}

/* ---- digest TX ------------------------------------------------------------ */

static uint32_t digest_period_ms(void)
{
	/* ± ~12% jitter so a fleet booted together does not sync its digests
	 * into one contention slot forever. */
	uint32_t base = CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC * MSEC_PER_SEC;
	uint32_t jitter = base / 8U;

	return base - jitter / 2U + (sys_rand32_get() % (jitter + 1U));
}

/*
 * THE BACKSTOP: how a narrowed node ever learns about its OWN entries.
 *
 * A narrowed node can only compare the base leg of an unnarrowed peer's digest,
 * because that peer never advertised one over the smaller set the two of them
 * share. So nodes/<me> is invisible to every digest it hears — and level
 * triggering means nobody will tell it either, since a peer that is ahead never
 * pushes uninvited. After a factory reset that is not a slow recovery, it is no
 * recovery: §2.3 simply does not fire for the constrained tier.
 *
 * One scope-filtered walk every N digest periods closes it. It also covers the
 * pushes a narrowed node happens to miss — including a master's override of one
 * of its sections, which arrives as a single broadcast and is otherwise gone.
 *
 * A node claiming everything does not run this: its digest comparison already
 * covers every key it could want.
 */
static void backstop_tick(void)
{
	bool fire = false;
	uint32_t peer = 0U;

	if (CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS == 0) {
		return;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (cluster.scope_kind != MESHTASTIC_CLUSTER_SCOPE_CORE) {
		cluster.backstop_countdown = 0U;
	} else if (cluster.backstop_countdown > 0U) {
		cluster.backstop_countdown--;
	} else {
		cluster.backstop_countdown =
			(uint16_t)CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS;
		peer = cluster.backstop_peer;
		/* No peer that claims everything has been heard yet, so there
		 * is nowhere our rows could be. Nothing to do, and nothing to
		 * complain about: an all-narrowed fleet has no restore source
		 * by construction (§7.7). */
		if (peer != 0U) {
			fire = pull_start_locked(peer, 0U, PULL_REASON_BACKSTOP);
			if (fire) {
				cluster.stats.backstop_walks++;
			}
		}
	}
	k_mutex_unlock(&cluster_lock);

	if (fire) {
		LOG_DBG("cluster: backstop walk against 0x%08x", peer);
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void digest_work_fn(struct k_work *work)
{
	/* Static for the same sysworkq-stack reason as send_cluster_message's
	 * buffer (single executor serialises access). */
	static zephyrtastic_ClusterMessage msg;
	struct meshtastic_cluster_scope mine;
	struct meshtastic_hlc_stamp max;
	uint8_t ch_index;
	int ret;

	ARG_UNUSED(work);
	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;

	if (!cluster_channel_index(&ch_index)) {
		if (!cluster.channel_missing_logged) {
			LOG_INF("cluster: no channel named \"%s\" — idle until one exists",
				CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
			cluster.channel_missing_logged = true;
		}
		(void)k_work_reschedule(&digest_work, K_MSEC(digest_period_ms()));
		return;
	}
	cluster.channel_missing_logged = false;

	msg.which_variant = zephyrtastic_ClusterMessage_digest_tag;
	k_mutex_lock(&cluster_lock, K_FOREVER);
	my_scope_locked(&mine);
	/* The three legs cover what we CLAIM, not what we hold — which for a
	 * node that has never narrowed is the same thing, so a fleet where
	 * nobody has narrowed advertises byte-identically to a pre-scope one. */
	msg.variant.digest.doc_hash = meshtastic_cluster_doc_hash_scoped(&cluster.doc, &mine);
	msg.variant.digest.entry_count =
		meshtastic_cluster_doc_count_scoped(&cluster.doc, &mine);
	meshtastic_cluster_doc_max_stamp_scoped(&cluster.doc, &mine, &max);
	msg.variant.digest.scope = scope_to_pb(&mine);
	{
		/* The second leg. Base is the one set inside every scope, so it
		 * is what a peer on a different tier can always compare. */
		struct meshtastic_cluster_scope only_base;

		base_scope(&only_base);
		msg.variant.digest.base_hash =
			meshtastic_cluster_doc_hash_scoped(&cluster.doc, &only_base);
		msg.variant.digest.base_count =
			meshtastic_cluster_doc_count_scoped(&cluster.doc, &only_base);
	}
	k_mutex_unlock(&cluster_lock);
	msg.variant.digest.has_max_stamp = true;
	stamp_to_pb(&max, &msg.variant.digest.max_stamp);

	ret = send_cluster_message(&msg, MESHTASTIC_NODE_BROADCAST, ch_index);
	if (ret == 0) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.digest_tx++;
		k_mutex_unlock(&cluster_lock);
	} else {
		LOG_WRN("cluster: digest send failed (%d)", ret);
	}

	backstop_tick();
	(void)k_work_reschedule(&digest_work, K_MSEC(digest_period_ms()));
}

/* ---- RX ------------------------------------------------------------------- */

/* Open an anti-entropy exchange with @p peer after @p delay_ms. Called with the
 * lock held. Returns false if one is already in flight (§3.3: one per node). */
static bool pull_start_locked(uint32_t peer, uint32_t delay_ms,
			      enum cluster_pull_reason reason)
{
	int64_t now = k_uptime_get();

	if (cluster.pull_state != PULL_IDLE || peer == 0U ||
	    peer == meshtastic_get_node_id()) {
		return false;
	}

	/*
	 * FRUITLESS-WALK BACKOFF — the bound on a conversation that can never
	 * succeed.
	 *
	 * Level-triggering means a mismatch we cannot resolve is stated again
	 * every digest period, forever. Usually that is the point: the next
	 * round fixes it. But some mismatches are structural — a peer holding
	 * an entry authored by a master WE do not trust, or one whose section
	 * this build refuses — and those never resolve however many times we
	 * ask. Without a bound the two nodes pull at each other for the life of
	 * the deployment, achieving nothing and spending airtime to do it.
	 *
	 * So: count walks that asked for entries and merged none. Under the
	 * threshold, keep trying (a couple of empty walks is ordinary — a race
	 * with a peer mid-write). Over it, back off by doubling, capped, and
	 * let exactly one probe through when the hold expires. Any successful
	 * merge anywhere clears the whole thing (see on_entry), so recovery is
	 * immediate the moment the situation changes — which matters, because
	 * the usual cure is an operator adding a key, not the peer changing.
	 *
	 * A FRUITLESS WALK HAS TWO SHAPES, and this half only sees one of them.
	 * The other is a walk that fetched a vector, found nothing worth asking
	 * for, and stopped — the "we are strictly AHEAD of that peer" case. It
	 * merges nothing by construction, so there is nothing to wait for and
	 * on_vector() judges it on the spot. Both feed the same counter; the
	 * split exists only because one verdict is available immediately and the
	 * other is not until entries have had time to arrive.
	 */
	if (cluster.pull_asked_entries) {
		cluster.pull_asked_entries = false;
		if (cluster.stats.entry_rx_applied == cluster.pull_last_applied) {
			if (cluster.pull_fruitless < UINT8_MAX) {
				cluster.pull_fruitless++;
			}
			cluster.stats.pull_fruitless++;
		} else {
			cluster.pull_fruitless = 0U;
		}
	}
	/* The backstop is a TIMER, not a reaction, and its rate bound is its own
	 * Kconfig cadence — the same shape as the digest. Holding it back on a
	 * fruitless run would suppress the one mechanism by which a narrowed
	 * node ever recovers its own entries, which is the opposite of what the
	 * backoff is for. (The judgement above still runs: it belongs to the
	 * PREVIOUS walk, whoever opened this one.) */
	if (reason != PULL_REASON_BACKSTOP &&
	    cluster.pull_fruitless >= CLUSTER_FRUITLESS_THRESHOLD) {
		uint32_t shift = MIN((uint32_t)cluster.pull_fruitless -
					     CLUSTER_FRUITLESS_THRESHOLD,
				     CLUSTER_FRUITLESS_MAX_SHIFT);

		if (now < cluster.pull_hold_until_ms) {
			cluster.stats.pull_held++;
			return false;
		}
		cluster.pull_hold_until_ms =
			now + (int64_t)CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC *
				      MSEC_PER_SEC * (1 << shift);
	}
	cluster.pull_state = PULL_SEND_VECTOR_REQ;
	cluster.pull_reason = (uint8_t)reason;
	cluster.pull_peer = peer;
	cluster.pull_key_count = 0U;
	cluster.pull_deadline_ms = k_uptime_get() + (int64_t)delay_ms +
				   (int64_t)CONFIG_MESHTASTIC_CLUSTER_SYNC_TIMEOUT_SEC *
					   MSEC_PER_SEC;
	cluster.stats.pull_started++;
	return true;
}

/*
 * COMPARING TWO DIGESTS THAT MAY NOT COVER THE SAME KEYS.
 *
 * The general rule is: compare over the INTERSECTION of the two declared
 * scopes, because that is the only set both nodes have said anything about.
 * Two cases fall out of it, and no others:
 *
 *  - The intersection IS the sender's whole scope (we claim everything they
 *    claim). Their triple describes exactly the set we can re-hash, so compare
 *    it directly. Between two un-narrowed nodes this is the whole document and
 *    the arithmetic is identical to what shipped before scopes existed.
 *
 *  - It is smaller — we are narrowed and they are not, or we are two different
 *    constrained nodes. Then their triple covers rows we never claimed, and the
 *    only thing both advertised is base. Compare the base leg.
 *
 * And one non-case: a sender predating scopes carries no base leg at all, so a
 * narrowed receiver has nothing comparable and must do NOTHING rather than
 * declare a mismatch it can never resolve. That gap is what the scheduled
 * backstop walk covers.
 *
 * Returns false when the two digests are not comparable at all.
 */
static bool digest_compare_locked(uint32_t from, const zephyrtastic_ClusterDigest *d,
				  const struct meshtastic_hlc_stamp *theirs, bool *match,
				  uint32_t *ours, uint32_t *theirs_hash)
{
	struct meshtastic_cluster_scope mine, peer, inter;
	bool has_base_leg;

	scope_from_pb(d->scope, from, &peer, &has_base_leg);
	my_scope_locked(&mine);
	meshtastic_cluster_scope_intersect(&mine, &peer, &inter);

	if (peer.kind == MESHTASTIC_CLUSTER_SCOPE_FULL && from != meshtastic_get_node_id()) {
		/* Worth walking later: it claims everything, so it is the kind
		 * of peer that could be holding our own rows for us. A peer
		 * that has narrowed too cannot help — CORE(x) never contains
		 * nodes/us. */
		cluster.backstop_peer = from;
	}

	if (scope_eq(&inter, &peer)) {
		struct meshtastic_hlc_stamp max;

		*ours = meshtastic_cluster_doc_hash_scoped(&cluster.doc, &peer);
		*theirs_hash = d->doc_hash;
		meshtastic_cluster_doc_max_stamp_scoped(&cluster.doc, &peer, &max);
		*match = (*ours == d->doc_hash) &&
			 (meshtastic_cluster_doc_count_scoped(&cluster.doc, &peer) ==
			  d->entry_count) &&
			 (meshtastic_hlc_compare(&max, theirs) == 0);
		return true;
	}

	if (!has_base_leg) {
		return false;
	}

	{
		struct meshtastic_cluster_scope only_base;

		base_scope(&only_base);
		*ours = meshtastic_cluster_doc_hash_scoped(&cluster.doc, &only_base);
		*theirs_hash = d->base_hash;
		*match = (*ours == d->base_hash) &&
			 (meshtastic_cluster_doc_count_scoped(&cluster.doc, &only_base) ==
			  d->base_count);
		return true;
	}
}

static void on_digest(uint32_t from, const zephyrtastic_ClusterDigest *d)
{
	struct meshtastic_hlc_stamp theirs;
	uint32_t hash = 0U;
	uint32_t theirs_hash = 0U;
	uint32_t delay = 0U;
	bool match = false;
	bool comparable;
	bool pulling = false;

	stamp_from_pb(&d->max_stamp, &theirs);

	k_mutex_lock(&cluster_lock, K_FOREVER);
	(void)meshtastic_hlc_observe(&cluster.hlc, &theirs);
	comparable = digest_compare_locked(from, d, &theirs, &match, &hash, &theirs_hash);

	if (!comparable) {
		/* A peer from before scopes existed, heard by a node that has
		 * narrowed its claim. Nothing it advertised describes a set we
		 * can re-hash. Counted, not chased: pulling on a mismatch we
		 * cannot even establish is the unbounded conversation. */
		cluster.stats.digest_rx_incomparable++;
	} else if (match) {
		cluster.stats.digest_rx_match++;
	} else {
		cluster.stats.digest_rx_mismatch++;
		/* Jitter before answering: a digest is a broadcast, so every
		 * diverged member hears the same one and would otherwise pull
		 * from the same sender in the same slot. */
		delay = sys_rand32_get() % (CONFIG_MESHTASTIC_CLUSTER_PULL_DELAY_MS + 1U);
		pulling = pull_start_locked(from, delay, PULL_REASON_DIGEST);
	}
	k_mutex_unlock(&cluster_lock);

	if (!comparable) {
		LOG_INF("cluster: digest from 0x%08x declares no scope — nothing this node "
			"claims is comparable with it (pre-scope build?)",
			from);
	} else if (!match) {
		LOG_INF("cluster: DIVERGED from 0x%08x — hash %08x vs local %08x%s", from,
			(unsigned int)theirs_hash, (unsigned int)hash,
			pulling ? " — pulling" : " (an exchange is already in flight)");
	} else {
		LOG_DBG("cluster: digest from 0x%08x matches", from);
	}

	if (pulling && !k_work_delayable_is_pending(&tx_work)) {
		/* Only when the worker is idle — never reschedule a pending one
		 * out to our jitter, which would push back a reply we already
		 * owe some other peer. Nothing is lost by leaving it: a pull can
		 * only start from PULL_IDLE, and the sole delay reachable from
		 * that state is the inter-frame gap, so a pending worker fires
		 * within CONFIG_MESHTASTIC_CLUSTER_TX_GAP_MS and picks the
		 * request up then. */
		(void)k_work_reschedule(&tx_work, K_MSEC(delay));
	}
}

/*
 * May we start serving a reply right now? Called with the lock held.
 *
 * Two bounds, answering different questions. IDLE answers "are we already
 * mid-reply?" — a repeat request must not rewind a walk in flight, or one small
 * frame resets our offset to zero and the walk never reaches the end of the
 * document. The TOKEN BUCKET answers what idleness cannot: a peer that simply
 * waits for each walk to finish before asking again extracts a full reply every
 * time, indefinitely. "One at a time" bounds concurrency, not rate — and rate is
 * where the bound has to live, because serving a vector or an entry burst is by
 * far the most expensive thing anyone can make this node do: tens of bytes in,
 * most of a document out.
 *
 * A bucket rather than a flat interval, because a flat interval would punish
 * the one case that matters most — a fleet powering up together, where every
 * member pulls at once and all of them are legitimate. The burst absorbs that;
 * the refill rate is the actual ceiling on sustained abuse.
 *
 * Per-peer accounting would be fairer and is deliberately NOT used: the sender
 * id is attacker-chosen, so a per-peer budget is one a hostile node grants
 * itself simply by inventing more ids. A global bound is the only one that
 * actually bounds.
 */
static bool may_serve_locked(void)
{
	int64_t now = k_uptime_get();
	int64_t elapsed;

	if (CONFIG_MESHTASTIC_CLUSTER_SERVE_MIN_INTERVAL_MS == 0) {
		return true; /* limit disabled (tests) */
	}

	elapsed = now - cluster.serve_refill_ms;
	if (elapsed >= CONFIG_MESHTASTIC_CLUSTER_SERVE_MIN_INTERVAL_MS) {
		uint32_t gained =
			(uint32_t)(elapsed / CONFIG_MESHTASTIC_CLUSTER_SERVE_MIN_INTERVAL_MS);

		cluster.serve_tokens =
			(uint8_t)MIN(cluster.serve_tokens + gained, CLUSTER_SERVE_BURST);
		cluster.serve_refill_ms +=
			(int64_t)gained * CONFIG_MESHTASTIC_CLUSTER_SERVE_MIN_INTERVAL_MS;
	}

	if (cluster.serve_tokens == 0U) {
		cluster.stats.tx_busy++;
		return false;
	}
	return true;
}

static void serve_started_locked(void)
{
	if (cluster.serve_tokens > 0U) {
		cluster.serve_tokens--;
	}
}

static void on_vector_req(uint32_t from, const zephyrtastic_ClusterVectorReq *r)
{
	bool serve;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	serve = (cluster.vec_dest == 0U);
	if (!serve) {
		cluster.stats.tx_busy++;
	} else if (!may_serve_locked()) {
		serve = false;
	} else {
		struct meshtastic_cluster_scope req;
		bool unused;

		/* Held for the whole walk: it spans several tx_work turns, and
		 * a requester that claimed less than everything must keep
		 * getting the same filtered view or the offsets stop lining up.
		 * A requester predating scopes says nothing, which resolves to
		 * FULL — the old behaviour, unchanged. */
		scope_from_pb(r->scope, from, &req, &unused);
		cluster.vec_scope_kind = req.kind;
		cluster.vec_dest = from;
		cluster.vec_offset = 0U;
		cluster.vec_emitted = 0U;
		serve_started_locked();
	}
	k_mutex_unlock(&cluster_lock);

	if (serve) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_vector(uint32_t from, const zephyrtastic_ClusterVector *v)
{
	struct meshtastic_cluster_scope mine;
	bool no_space = false;
	bool fire = false;
	bool last;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	my_scope_locked(&mine);
	if (cluster.pull_state != PULL_AWAIT_VECTOR || from != cluster.pull_peer) {
		/* A later chunk from the peer we WERE walking is not unsolicited,
		 * it is late: a full batch of keys ends the vector phase early, so
		 * the tail of a long vector arrives after we have moved on. Ignore
		 * it quietly; counting it as unsolicited would make an ordinary
		 * multi-chunk walk look like someone was talking out of turn. */
		if (from != cluster.pull_peer) {
			cluster.stats.rx_unsolicited++;
		}
		k_mutex_unlock(&cluster_lock);
		return;
	}
	cluster.stats.vector_rx++;

	/* Chunks are diffed INDEPENDENTLY — nothing is assembled. A lost chunk
	 * therefore costs only the keys it carried, and the next digest asks
	 * for them again; there is no reassembly buffer to size, stall on, or
	 * get wrong. */
	for (pb_size_t i = 0; i < v->entries_count && cluster.pull_key_count < CLUSTER_PULL_KEYS;
	     i++) {
		const zephyrtastic_KeyStamp *row = &v->entries[i];
		struct meshtastic_cluster_key key;
		struct meshtastic_hlc_stamp stamp;

		if (!row->has_key || !row->has_stamp) {
			continue;
		}
		key_from_pb(&row->key, &key);
		stamp_from_pb(&row->stamp, &stamp);
		if (!section_shareable(key.section) || !stamp_within_horizon(&stamp)) {
			/* Never spend a round trip on a row ingest would refuse
			 * anyway — that is a request loop with extra steps. */
			continue;
		}
		(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
		switch (meshtastic_cluster_doc_want(&cluster.doc, &mine, &key, &stamp)) {
		case MESHTASTIC_CLUSTER_WANT_YES:
			cluster.pull_keys[cluster.pull_key_count++] = key;
			break;
		case MESHTASTIC_CLUSTER_WANT_NO_SPACE:
			/* We claim this row and cannot hold it: this node has
			 * outgrown its table. THE honest trigger — we chose to
			 * walk this peer and we wanted this row. */
			cluster.stats.want_no_space++;
			no_space = true;
			break;
		default:
			break;
		}
	}

	if (no_space) {
		/* Narrow first, then abandon the walk: the keys we collected
		 * describe a document we have just stopped claiming, and the
		 * next digest re-opens whatever still matters. */
		demote_locked("the fleet's document no longer fits");
		cluster.pull_key_count = 0U;
		pull_reset_locked();
		k_mutex_unlock(&cluster_lock);
		return;
	}

	last = (v->total == 0U) || ((uint32_t)v->offset + v->entries_count >= v->total);
	if (cluster.pull_key_count >= CLUSTER_PULL_KEYS || last) {
		if (cluster.pull_key_count > 0U) {
			/* More than one batch of keys behind? Ask for this one;
			 * the rest still differ, so the next digest re-opens the
			 * walk. Convergence takes another round, never a hang. */
			cluster.pull_state = PULL_SEND_ENTRY_REQ;
			fire = true;
		} else {
			/* Their document holds nothing we lack. If they are the
			 * ones behind, our digest is already inviting them.
			 *
			 * This walk is over and it merged nothing, so judge it
			 * NOW rather than deferring to the next one: unlike the
			 * entry-request case there is nothing still in flight
			 * that could change the verdict. Without this the "we
			 * are permanently ahead of that peer" mismatch — a peer
			 * on an older build, or one that structurally refuses
			 * something we hold — costs a vector exchange every
			 * digest period for the life of the deployment, which
			 * is precisely what the backoff exists to stop.
			 *
			 * Only for a walk WE chose to open on a mismatch. An
			 * operator's `cluster pull` is not evidence about the
			 * fleet. (The deferred half above stays reason-agnostic
			 * on purpose: asking for entries and merging none says
			 * something is structurally wrong whoever asked.)
			 */
			cluster.stats.pull_empty++;
			if (cluster.pull_reason == (uint8_t)PULL_REASON_DIGEST) {
				if (cluster.pull_fruitless < UINT8_MAX) {
					cluster.pull_fruitless++;
				}
				cluster.stats.pull_fruitless++;
			}
			pull_reset_locked();
		}
	}
	k_mutex_unlock(&cluster_lock);

	if (fire) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_entry_req(uint32_t from, const zephyrtastic_ClusterEntryReq *r)
{
	bool serve;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	/* Idle only AND rate-limited, for the same reasons as the vector walk
	 * above: a repeat request must not rewind a burst already being served,
	 * and "one at a time" bounds concurrency but not rate. */
	serve = (cluster.ent_dest == 0U) && may_serve_locked();
	if (serve) {
		serve_started_locked();
		cluster.ent_dest = from;
		cluster.ent_count = 0U;
		cluster.ent_next = 0U;
		for (pb_size_t i = 0; i < r->keys_count && cluster.ent_count < CLUSTER_PULL_KEYS;
		     i++) {
			key_from_pb(&r->keys[i], &cluster.ent_keys[cluster.ent_count]);
			cluster.ent_count++;
		}
		if (cluster.ent_count == 0U) {
			cluster.ent_dest = 0U;
			serve = false;
		}
	} else if (cluster.ent_dest != 0U) {
		cluster.stats.tx_busy++; /* mid-burst; may_serve counts its own */
	}
	k_mutex_unlock(&cluster_lock);

	if (serve) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_entry(uint32_t from, const zephyrtastic_ClusterEntry *e)
{
	struct meshtastic_cluster_key key;
	struct meshtastic_hlc_stamp stamp;
	bool changed = false;
	int ret;

	if (!e->has_key || !e->has_stamp) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_undecodable++;
		k_mutex_unlock(&cluster_lock);
		return;
	}
	key_from_pb(&e->key, &key);
	stamp_from_pb(&e->stamp, &stamp);

	/* The secret boundary, ingest side (D9). Refused even when well-formed
	 * and master-authored: this is the half of the ban that does not depend
	 * on every peer in the fleet running correct code. */
	if (!section_shareable(key.section)) {
		refuse_entry("not a shareable section", key.section);
		return;
	}

	/*
	 * Outside what we claim to track. Cheapest gate that rejects the most
	 * traffic on a narrowed node — every broadcast push of another node's
	 * pin lands here — and it leaks nothing, so it goes ahead of the
	 * authorship check rather than after it.
	 *
	 * This is safe in a way the `lora` case was not. Refusing lora at
	 * ingest would strand this node's document from the fleet's forever,
	 * because we still advertise a hash that says we hold it. A row outside
	 * our scope is different: the digest we advertise EXCLUDES it, so no
	 * peer ever expects it of us and no mismatch survives the refusal. That
	 * argument holds only while the refusal predicate and the hash
	 * predicate are the same function — which is why both call
	 * meshtastic_cluster_scope_contains() and nothing else.
	 *
	 * Counted apart from refusals on purpose. entry_rx_refused means
	 * something tried to put something it should not into this document;
	 * on a narrowed node in a busy fleet this path is simply the normal
	 * case, and folding it in would drown the counter that matters.
	 */
	{
		struct meshtastic_cluster_scope mine;
		bool in_scope;

		k_mutex_lock(&cluster_lock, K_FOREVER);
		my_scope_locked(&mine);
		in_scope = meshtastic_cluster_scope_contains(&mine, &key);
		if (!in_scope) {
			cluster.stats.entry_rx_out_of_scope++;
		}
		k_mutex_unlock(&cluster_lock);
		if (!in_scope) {
			return;
		}
	}
	if (!entry_authorized(&key, &stamp)) {
		refuse_entry("author is not a master", stamp.node_id);
		return;
	}
	if (!stamp_within_horizon(&stamp)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.entry_rx_future++;
		k_mutex_unlock(&cluster_lock);
		refuse_entry("stamped beyond the clock drift horizon", stamp.node_id);
		return;
	}
	if (key.layer == MESHTASTIC_CLUSTER_LAYER_NODE && !node_owner_is_known(key.node_id)) {
		refuse_entry("owner is a node this mesh has never seen", key.node_id);
		return;
	}
	/*
	 * The payload must BE the section its key claims.
	 *
	 * The document is deliberately payload-agnostic — pure table logic, no
	 * protobuf — so if this check does not happen here it happens nowhere,
	 * and the allowlist above is checked against key.section alone. A
	 * well-formed key could then carry anything: undecodable bytes, or a
	 * banned section wearing a permitted key.
	 *
	 * Refusing at APPLY time is not enough, which is the whole point. An
	 * entry the reconciler will never apply still replicates to every
	 * member, changes every document hash, and at the BASE layer can never
	 * be withdrawn — base has no tombstone by design (§2.1), so "no fleet
	 * default" is the key's absence and there is no way to express its
	 * removal. One frame, permanent fleet-wide junk. Ingest is the only
	 * place that can still say no.
	 */
	if (!payload_is_its_section(key.section, e->payload.bytes, e->payload.size,
				    e->tombstone)) {
		refuse_entry("payload is not the section its key claims", key.section);
		return;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
	ret = meshtastic_cluster_doc_accept(&cluster.doc, &key, &stamp, e->tombstone,
					    e->payload.bytes, e->payload.size);
	if (ret == 1) {
		persist_entry(meshtastic_cluster_doc_find(&cluster.doc, &key));
		cluster.stats.entry_rx_applied++;
		/* Progress. Whatever we were backing off from, we are not any
		 * more — one real result forgives the whole fruitless run. */
		cluster.pull_fruitless = 0U;
		cluster.pull_hold_until_ms = 0;
		changed = true;
	} else if (ret == 0) {
		/* Replay defence, for free: a re-sent old entry carries its old
		 * stamp and loses LWW like any other stale write (§4). */
		cluster.stats.entry_rx_stale++;
	} else if (ret == -ENOSPC) {
		/* Counted apart from malformed input because it means something
		 * completely different and needs a different response: the frame
		 * was fine, this node has simply run out of table. A climbing
		 * number here is the fleet outgrowing MAX_ENTRIES, not an attack.
		 *
		 * It does NOT mean we are re-requesting this key forever: since
		 * 23b47de the diff predicate is capacity-aware and never asks
		 * for a new key a full table would refuse (tests/cluster:
		 * test_full_table_stops_asking_for_what_it_cannot_store). What
		 * reaches here is therefore an UNSOLICITED push — which is also
		 * why this path counts and does NOT narrow the claim: see the
		 * asymmetry argued at demote_locked(). */
		cluster.stats.entry_rx_no_space++;
	} else {
		cluster.stats.rx_undecodable++;
	}
	k_mutex_unlock(&cluster_lock);

	if (changed) {
		LOG_INF("cluster: merged %c/%08x/%u from 0x%08x (stamp %lld.%u by 0x%08x)",
			key.layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n', key.node_id,
			(unsigned int)key.section, from, (long long)stamp.physical_ms,
			stamp.counter, stamp.node_id);
		(void)k_work_submit(&reconcile_work);
	}
}

static void cluster_on_packet(const struct meshtastic_packet *packet,
			      const meshtastic_MeshPacket *mesh)
{
	/* Static, guarded by cluster_lock only for the doc it touches — but the
	 * decode itself is on the RX thread, and a ClusterMessage embeds a
	 * 128-byte payload plus four stamp rows. Keeping it off that stack is
	 * the same discipline the TX side follows; the module dispatch is
	 * single-threaded, so one copy suffices. */
	static zephyrtastic_ClusterMessage msg;
	const uint8_t *payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	size_t payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	uint8_t ch_index;
	pb_istream_t is;

	/*
	 * Double gate (D1): port 256 got us dispatched; only the cluster
	 * channel's frames are ours to interpret — with one exception that is
	 * not a loosening.
	 *
	 * The walk's pulls are UNICASTS, and the send path encrypts a unicast to
	 * a peer whose public key we hold with PKC rather than the channel key
	 * (meshtastic_packet.c, mirroring upstream). A PKC frame has no channel:
	 * it decrypts to channel_index 0, the PKC pseudo-channel, with
	 * pki_encrypted set. Requiring the cluster channel index would therefore
	 * refuse every vector and entry exchanged between two nodes that know
	 * each other's keys — which is every node pair the M3 admin trust
	 * already set up, i.e. exactly the fleet this feature is for. The walk
	 * would open, be answered, and have the answer thrown away.
	 *
	 * Accepting them does not widen the trust boundary, it narrows it: a PKC
	 * frame that decrypted here was addressed to THIS node and authenticated
	 * (X25519 + AES-CCM) to a sender whose key we already hold — a stronger
	 * claim than "knows the cluster PSK", which is one fleet-wide shared
	 * secret with no per-node authentication in it at all. What an accepted
	 * frame may then DO is governed the same way either way: the D9 section
	 * allowlist and the D4 authorship gate, both downstream of here.
	 *
	 * Broadcasts are never PKC, so digests still require the channel — the
	 * scoping property D1 is actually about is untouched.
	 */
	if (!(mesh ? mesh->pki_encrypted : packet->pki_encrypted)) {
		if (!cluster_channel_index(&ch_index) || packet->channel_index != ch_index) {
			k_mutex_lock(&cluster_lock, K_FOREVER);
			cluster.stats.rx_wrong_channel++;
			k_mutex_unlock(&cluster_lock);
			return;
		}
	} else if (!cluster_channel_index(&ch_index)) {
		/* Not a member: no cluster channel provisioned means this node
		 * is not participating, whatever a peer sends it. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_wrong_channel++;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
	is = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&is, zephyrtastic_ClusterMessage_fields, &msg)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_undecodable++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: undecodable frame from 0x%08x", packet->from);
		return;
	}

	switch (msg.which_variant) {
	case zephyrtastic_ClusterMessage_digest_tag:
		on_digest(packet->from, &msg.variant.digest);
		break;
	case zephyrtastic_ClusterMessage_vector_req_tag:
		on_vector_req(packet->from, &msg.variant.vector_req);
		break;
	case zephyrtastic_ClusterMessage_vector_tag:
		on_vector(packet->from, &msg.variant.vector);
		break;
	case zephyrtastic_ClusterMessage_entry_req_tag:
		on_entry_req(packet->from, &msg.variant.entry_req);
		break;
	case zephyrtastic_ClusterMessage_entry_tag:
		on_entry(packet->from, &msg.variant.entry);
		break;
	default:
		/* All five v1 verbs are handled above, so this is a peer running
		 * a protocol we do not have. Counted so a mixed-version bench is
		 * visible rather than silent. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_not_implemented++;
		k_mutex_unlock(&cluster_lock);
		break;
	}
}

MESHTASTIC_MODULE_DEFINE(cluster, MESHTASTIC_PORT_PRIVATE, 0, cluster_on_packet, NULL);

/* ---- the local writers: promote, pin, unpin ------------------------------- */

/*
 * A managed node's configuration is its master's to set (SecurityConfig
 * .is_managed — the same gate the local admin path and the shell's config
 * commands already honour). Writing the cluster document is a configuration
 * write like any other: `pin` would let a managed worker override what its
 * master just sent it, and `promote` would let it rewrite the fleet's defaults
 * from a node that is not supposed to have opinions. Remote admin (M3) still
 * works, which is the point of being managed.
 */
static bool local_writes_allowed(void)
{
#if defined(CONFIG_MESHTASTIC_ADMIN)
	return !meshtastic_admin_is_managed();
#else
	return true;
#endif
}

/* Wake the TX worker for a queued push — but never pull a PENDING one forward.
 * A push is the one optional frame here (see tx_next_locked step 5), and
 * collapsing the gap of a reply burst mid-flight to get it out sooner would be
 * the wrong trade. An idle worker starts now; a busy one picks the push up
 * within a frame gap, or within a second while a pull is outstanding. */
static void push_kick(void)
{
	if (!k_work_delayable_is_pending(&tx_work)) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

/*
 * The write itself, shared by all three writers: mint, accept, persist, queue
 * the push, reconcile. Called with the lock NOT held.
 *
 * @p store_stamp is folded into the cluster clock before minting. The document
 * and the config store each run their own HLC instance, so without this a fresh
 * entry can compare OLDER than the very store value it was made from and the
 * reconciler then declines to apply it — a real defect found in M4b review. It
 * matters for a tombstone too, where there is no value to carry: the tombstone
 * has to out-rank the store's current version or `unpin` cannot revert it.
 */
static int doc_write_local(const struct meshtastic_cluster_key *key,
			   const struct meshtastic_hlc_stamp *store_stamp, bool tombstone,
			   const uint8_t *payload, size_t payload_len,
			   struct meshtastic_hlc_stamp *minted)
{
	int ret;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	meshtastic_hlc_observe(&cluster.hlc, store_stamp);
	meshtastic_hlc_local(&cluster.hlc, meshtastic_get_node_id(), minted);
	ret = meshtastic_cluster_doc_accept(&cluster.doc, key, minted, tombstone, payload,
					    payload_len);
	if (ret == -ENOSPC) {
		/* Our OWN write, refused for room. The most honest overflow
		 * signal there is -- nobody chose this key for us -- and the one
		 * an operator actually meets, as `cluster pin display` failing
		 * for reasons about other nodes' rows. Narrowing frees exactly
		 * those, so the retry is not optimism: a CORE claim is inside
		 * the table's floor by construction, and this key is in it. */
		demote_locked("a local write had nowhere to go");
		ret = meshtastic_cluster_doc_accept(&cluster.doc, key, minted, tombstone,
						    payload, payload_len);
	}
	if (ret == 1) {
		persist_entry(meshtastic_cluster_doc_find(&cluster.doc, key));
		push_enqueue_locked(key);
		ret = 0;
	} else if (ret == 0) {
		/* Freshly minted stamps are strictly newer than anything seen;
		 * a stale verdict here means a logic error, not a race. */
		ret = -EINVAL;
	}
	k_mutex_unlock(&cluster_lock);

	if (ret == 0) {
		/* Reconcile so this node's own store records the new stamp —
		 * which is what makes the section doc-derived here too, and the
		 * -EALREADY refusals meaningful. */
		(void)k_work_submit(&reconcile_work);
		push_kick();
	}
	return ret;
}

/* Read MY current value of @p section and encode it as the document carries it
 * (an upstream meshtastic_Config with one payload variant set — D8). */
static int section_snapshot(uint16_t section, uint8_t *out, size_t out_len, size_t *written,
			    struct meshtastic_hlc_stamp *store_stamp)
{
	meshtastic_Config cfg;
	pb_ostream_t os = pb_ostream_from_buffer(out, out_len);
	int ret = meshtastic_config_store_get_config((pb_size_t)section, &cfg);

	if (ret < 0 || cfg.which_payload_variant != section) {
		return -ENOENT;
	}
	if (meshtastic_config_store_get_config_stamp((pb_size_t)section, store_stamp) != 0) {
		return -ENOENT;
	}
	if (!pb_encode(&os, meshtastic_Config_fields, &cfg)) {
		LOG_ERR("cluster: section %u encode failed: %s", (unsigned int)section,
			PB_GET_ERROR(&os));
		return -EINVAL;
	}
	*written = os.bytes_written;
	return 0;
}

/*
 * THE ORIGIN MARKER, ENFORCED (D10) — the shared half of promote and pin.
 *
 * True when the value this node currently stores for @p section came FROM the
 * entry at @p key: the reconciler wrote it carrying that entry's stamp, so the
 * two stamps being equal is the fact itself (§2.2). Lifting it back into the
 * document would mint a second stamp for bytes the fleet already has, and every
 * member would then churn through an apply that changes nothing. A local edit
 * moves the store's stamp and lifts the refusal, which is exactly the intended
 * workflow.
 *
 * Note which key each writer asks about, because it is the whole difference
 * between them: promote asks about `base/<sec>`, pin about `nodes/<me>/<sec>`.
 * Pinning a value inherited from the base is therefore NOT a re-pin — it is the
 * meaningful act of freezing today's fleet default as this node's own.
 *
 * A TOMBSTONE never counts as the origin, even when the stamps match. After an
 * unpin the store legitimately carries the tombstone's stamp (that is the
 * version the reversion was applied under — see the reconciler), and reading
 * that as "already pinned" would refuse the operator's next pin on a section
 * they have just unpinned. There is no value to have come from a tombstone.
 */
static bool store_came_from(const struct meshtastic_cluster_key *key,
			    const struct meshtastic_hlc_stamp *store_stamp)
{
	const struct meshtastic_cluster_entry *existing;
	bool same;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	existing = meshtastic_cluster_doc_find(&cluster.doc, key);
	same = (existing != NULL) && !existing->tombstone &&
	       (meshtastic_hlc_compare(store_stamp, &existing->stamp) == 0);
	k_mutex_unlock(&cluster_lock);
	return same;
}

int meshtastic_cluster_promote(uint16_t section)
{
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	struct meshtastic_cluster_key key = {
		.layer = MESHTASTIC_CLUSTER_LAYER_BASE,
		.node_id = 0U,
		.section = section,
	};
	struct meshtastic_hlc_stamp stamp;
	struct meshtastic_hlc_stamp store_stamp;
	size_t payload_len;
	int ret;

	if (!section_shareable(section) || !local_writes_allowed()) {
		return -EPERM;
	}
	if (section == meshtastic_Config_lora_tag) {
		/* A fleet preset change orphans any node that misses it, and
		 * the straggler sweep (§7.9) does not exist yet. Refused, not
		 * warned: the failure is silent and permanent. */
		LOG_WRN("cluster: refusing to promote lora — a missed preset change orphans "
			"nodes until the straggler sweep exists");
		return -EPERM;
	}

	ret = section_snapshot(section, payload, sizeof(payload), &payload_len, &store_stamp);
	if (ret != 0) {
		return ret;
	}
	if (store_came_from(&key, &store_stamp)) {
		LOG_INF("cluster: section %u is already the fleet base", (unsigned int)section);
		return -EALREADY;
	}

	ret = doc_write_local(&key, &store_stamp, false, payload, payload_len, &stamp);
	if (ret == 0) {
		LOG_INF("cluster: promoted section %u to base (stamp %lld.%u by 0x%08x)",
			(unsigned int)section, (long long)stamp.physical_ms, stamp.counter,
			stamp.node_id);
	}
	return ret;
}

int meshtastic_cluster_pin(uint16_t section)
{
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	struct meshtastic_cluster_key key = {
		.layer = MESHTASTIC_CLUSTER_LAYER_NODE,
		.node_id = meshtastic_get_node_id(),
		.section = section,
	};
	struct meshtastic_hlc_stamp stamp;
	struct meshtastic_hlc_stamp store_stamp;
	size_t payload_len;
	int ret;

	if (!section_shareable(section) || !local_writes_allowed()) {
		return -EPERM;
	}
	if (section == meshtastic_Config_lora_tag) {
		/* Not the fleet-wide hazard promote refuses — a pin binds only
		 * this node — but the reconciler HOLDS every doc-borne lora
		 * section rather than applying it (see reconcile_section), so a
		 * lora pin could never take effect. Accepting a write that is
		 * guaranteed to do nothing, and replicating it fleet-wide into a
		 * table that never evicts, would be worse than saying no. The
		 * same straggler sweep opens both gates. */
		LOG_WRN("cluster: refusing to pin lora — doc-borne lora sections are "
			"replicated but never applied, so the pin could not take effect");
		return -EPERM;
	}
	if (key.node_id == 0U) {
		return -EINVAL; /* no node id yet: nothing to own the entry */
	}

	ret = section_snapshot(section, payload, sizeof(payload), &payload_len, &store_stamp);
	if (ret != 0) {
		return ret;
	}
	if (store_came_from(&key, &store_stamp)) {
		LOG_INF("cluster: section %u is already this node's pin", (unsigned int)section);
		return -EALREADY;
	}

	ret = doc_write_local(&key, &store_stamp, false, payload, payload_len, &stamp);
	if (ret == 0) {
		LOG_INF("cluster: pinned section %u for this node (stamp %lld.%u by 0x%08x)",
			(unsigned int)section, (long long)stamp.physical_ms, stamp.counter,
			stamp.node_id);
	}
	return ret;
}

int meshtastic_cluster_unpin(uint16_t section)
{
	struct meshtastic_cluster_key key = {
		.layer = MESHTASTIC_CLUSTER_LAYER_NODE,
		.node_id = meshtastic_get_node_id(),
		.section = section,
	};
	const struct meshtastic_cluster_entry *existing;
	struct meshtastic_hlc_stamp stamp;
	struct meshtastic_hlc_stamp store_stamp;
	bool present;
	bool already;
	int ret;

	if (!local_writes_allowed()) {
		return -EPERM;
	}
	if (key.node_id == 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	existing = meshtastic_cluster_doc_find(&cluster.doc, &key);
	present = (existing != NULL);
	already = present && existing->tombstone;
	k_mutex_unlock(&cluster_lock);

	if (!present) {
		return -ENOENT;
	}
	if (already) {
		/* The tombstone IS the removal and it is already replicating.
		 * Minting a second one would be a new version of "nothing",
		 * which the fleet would dutifully carry for no reason. */
		return -EALREADY;
	}

	/* Fold in the store's stamp for the section as usual — the tombstone has
	 * no value of its own, but it must out-rank the store's current version
	 * or the reconciler cannot revert this node off the pin it is running. */
	if (meshtastic_config_store_get_config_stamp((pb_size_t)section, &store_stamp) != 0) {
		memset(&store_stamp, 0, sizeof(store_stamp));
	}

	ret = doc_write_local(&key, &store_stamp, true, NULL, 0U, &stamp);
	if (ret == 0) {
		LOG_INF("cluster: unpinned section %u — reverting to the fleet base as it "
			"stands now (stamp %lld.%u by 0x%08x)",
			(unsigned int)section, (long long)stamp.physical_ms, stamp.counter,
			stamp.node_id);
	}
	return ret;
}

int meshtastic_cluster_pull(uint32_t node_id)
{
	bool started;
	uint8_t ch_index;

	if (node_id == 0U || node_id == meshtastic_get_node_id()) {
		return -EINVAL;
	}
	if (!cluster_channel_index(&ch_index)) {
		return -ENOTCONN;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	started = pull_start_locked(node_id, 0U, PULL_REASON_SHELL);
	k_mutex_unlock(&cluster_lock);
	if (!started) {
		return -EBUSY;
	}
	(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	return 0;
}

int meshtastic_cluster_scope_select(int choice)
{
	uint8_t kind;
	bool pinned;

	switch (choice) {
	case MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO:
		kind = MESHTASTIC_CLUSTER_SCOPE_FULL;
		pinned = false;
		break;
	case MESHTASTIC_CLUSTER_SCOPE_CHOICE_FULL:
		kind = MESHTASTIC_CLUSTER_SCOPE_FULL;
		pinned = true;
		break;
	case MESHTASTIC_CLUSTER_SCOPE_CHOICE_CORE:
		kind = MESHTASTIC_CLUSTER_SCOPE_CORE;
		pinned = true;
		break;
	default:
		return -EINVAL;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (cluster.scope_kind == kind && cluster.scope_pinned == pinned) {
		k_mutex_unlock(&cluster_lock);
		return -EALREADY;
	}
	scope_set_locked(kind, pinned);
	if (choice == MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO) {
		/* "auto" is how an operator says: forget what you decided. */
		forget_scope();
	} else {
		persist_scope(kind);
	}
	k_mutex_unlock(&cluster_lock);

	LOG_INF("cluster: now claiming %s%s", kind == MESHTASTIC_CLUSTER_SCOPE_CORE
						      ? "CORE (base + my own sections)"
						      : "FULL (the whole document)",
		pinned ? "" : " — free to narrow under table pressure");
	/* Say so at once rather than waiting for the cadence: every peer's
	 * comparison against this node just changed which set it covers. */
	(void)k_work_reschedule(&digest_work, K_NO_WAIT);
	return 0;
}

/*
 * Forget the replicated document entirely — table, every persisted record, and
 * the recorded scope — and go back to the claim this build was compiled with.
 *
 * There was no way to do this before, and a factory reset did NOT do it either:
 * the wipe walks the config store's own keys under "meshtastic/", while the
 * document lives in a separate "mtclus/" subtree, so `factory_reset_device`
 * logged "full wipe" and left the whole fleet document behind.
 *
 * NOT a fleet operation, and that is what makes it safe to offer at all: it
 * pushes nothing, announces nothing, and cannot remove an entry from anyone
 * else. Anti-entropy is level-triggered, so a node cleared on its own simply
 * pulls the document back from the first peer that still has it — which is also
 * the warning the shell prints. Clearing a FLEET means clearing every member
 * before any of them can re-seed the others.
 *
 * ⚠️ AND ISOLATING A MEMBER TAKES BOTH BEARERS. `lora tx off` is not enough: a
 * pull is a unicast, so it diverts to a live BLE peer link, and a LoRa-mute
 * node converges over it — the property M4b was built to prove. The peer links
 * have to come down too (`blepeer scan off` + `blepeer disconnect` on the
 * central side). Learned the hard way on the bench, 2026-08-25: four nodes were
 * cleared, all four read 0, and the document reassembled itself over a
 * kit2 → kit1 → rxru BLE chain that LoRa TX state said nothing about.
 *
 * Deliberately NOT gated on is_managed, unlike promote/pin/unpin. Those write
 * to the fleet; this drops a local copy of what the fleet already holds, so
 * refusing it on a managed node would remove a recovery tool without protecting
 * anything.
 */
int meshtastic_cluster_reset(void)
{
	uint16_t dropped;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	dropped = meshtastic_cluster_doc_clear(&cluster.doc, forget_entry, NULL);
	forget_subtree();
	forget_scope();

	/* Back to the compiled claim: a scope narrowed under pressure is a fact
	 * about a document that no longer exists. */
#if defined(CONFIG_MESHTASTIC_CLUSTER_SCOPE_CORE)
	cluster.scope_kind = MESHTASTIC_CLUSTER_SCOPE_CORE;
	cluster.scope_pinned = true;
#else
	cluster.scope_kind = MESHTASTIC_CLUSTER_SCOPE_FULL;
	cluster.scope_pinned = IS_ENABLED(CONFIG_MESHTASTIC_CLUSTER_SCOPE_FULL);
#endif
	cluster.stats.scope_demotions = 0U;
	cluster.stats.entries_evicted = 0U;

	/* Anything in flight describes a document that is gone. */
	pull_reset_locked();
	cluster.vec_dest = 0U;
	cluster.ent_dest = 0U;
	cluster.ent_count = 0U;
	cluster.ent_next = 0U;
	cluster.push_count = 0U;
	cluster.push_head = 0U;
	k_mutex_unlock(&cluster_lock);

	LOG_WRN("cluster: document cleared — %u entr%s dropped, claim back to %s. Nothing was "
		"told to forget anything: the next digest from a peer that still holds it "
		"will bring it all back.",
		dropped, dropped == 1U ? "y" : "ies",
		cluster.scope_kind == MESHTASTIC_CLUSTER_SCOPE_CORE ? "CORE" : "FULL");
	return (int)dropped;
}

const char *meshtastic_cluster_scope_name(bool *pinned)
{
	const char *name;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	name = (cluster.scope_kind == MESHTASTIC_CLUSTER_SCOPE_CORE) ? "CORE" : "FULL";
	if (pinned != NULL) {
		*pinned = cluster.scope_pinned;
	}
	k_mutex_unlock(&cluster_lock);
	return name;
}

void meshtastic_cluster_digest_now(void)
{
	(void)k_work_reschedule(&digest_work, K_NO_WAIT);
}

/* ---- introspection -------------------------------------------------------- */

void meshtastic_cluster_stats_get(struct meshtastic_cluster_stats *out)
{
	k_mutex_lock(&cluster_lock, K_FOREVER);
	*out = cluster.stats;
	k_mutex_unlock(&cluster_lock);
}

const char *meshtastic_cluster_sync_state(uint32_t *peer)
{
	static const char *const names[] = {
		[PULL_IDLE] = "idle",
		[PULL_SEND_VECTOR_REQ] = "sending vector request",
		[PULL_AWAIT_VECTOR] = "awaiting vector",
		[PULL_SEND_ENTRY_REQ] = "sending entry request",
	};
	const char *s;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	s = names[cluster.pull_state];
	if (peer != NULL) {
		*peer = cluster.pull_peer;
	}
	k_mutex_unlock(&cluster_lock);
	return s;
}

uint16_t meshtastic_cluster_entry_count(void)
{
	uint16_t n;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	n = cluster.doc.count;
	k_mutex_unlock(&cluster_lock);
	return n;
}

uint32_t meshtastic_cluster_doc_hash_now(void)
{
	uint32_t h;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	h = meshtastic_cluster_doc_hash(&cluster.doc);
	k_mutex_unlock(&cluster_lock);
	return h;
}

bool meshtastic_cluster_entry_get(uint16_t idx, struct meshtastic_cluster_entry *out)
{
	bool ok = false;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (idx < cluster.doc.count) {
		*out = cluster.doc.entries[idx];
		ok = true;
	}
	k_mutex_unlock(&cluster_lock);
	return ok;
}

/* ---- init ----------------------------------------------------------------- */

int meshtastic_cluster_init(void)
{
	meshtastic_cluster_doc_init(&cluster.doc, cluster_storage,
				    ARRAY_SIZE(cluster_storage));
#if defined(CONFIG_MESHTASTIC_CLUSTER_SCOPE_CORE)
	cluster.scope_kind = MESHTASTIC_CLUSTER_SCOPE_CORE;
	cluster.scope_pinned = true;
#else
	/* AUTO and FULL both start out claiming everything; they differ only in
	 * whether table pressure is allowed to narrow the claim later. */
	cluster.scope_kind = MESHTASTIC_CLUSTER_SCOPE_FULL;
	cluster.scope_pinned = IS_ENABLED(CONFIG_MESHTASTIC_CLUSTER_SCOPE_FULL);
#endif

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	/* Persisted entries load through the settings handler; the config
	 * store's subtree load has already run by the time we are called, so
	 * pull ours explicitly rather than depending on a global load.
	 *
	 * The CLAIM first, in its own pass. It decides which entries are even
	 * loadable, and flash hands records back in whatever order it stored
	 * them — leaving that to luck would let a narrowed node refill itself
	 * with rows it does not claim on some boots and not others. */
	(void)settings_load_subtree(CLUSTER_SCOPE_REC_NAME);
	(void)settings_load_subtree("mtclus");
#endif

	/* Start with a full reply budget: the fleet this node is joining may
	 * already be waiting to pull from it. Likewise a full push budget — the
	 * first thing an operator does with a fresh node is configure it, and
	 * every one of those writes deserves to reach the fleet at once. */
	cluster.serve_tokens = CLUSTER_SERVE_BURST;
	cluster.serve_refill_ms = k_uptime_get();
	cluster.push_tokens = CLUSTER_PUSH_BURST;
	cluster.push_refill_ms = cluster.serve_refill_ms;

	(void)k_work_schedule(&digest_work, K_MSEC(digest_period_ms()));
	if (cluster.doc.count > 0U) {
		/* A document restored from NVS describes configuration this
		 * node may not currently be running (it was wiped, or the doc
		 * moved on while it was off). Reconcile before the first
		 * digest so what we advertise is what we are actually running. */
		(void)k_work_submit(&reconcile_work);
	}
	LOG_INF("cluster: up — %u entr%s, digest every ~%u s on channel \"%s\"",
		cluster.doc.count, cluster.doc.count == 1U ? "y" : "ies",
		(unsigned int)CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC,
		CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
	return 0;
}
