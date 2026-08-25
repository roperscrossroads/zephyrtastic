/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_CLUSTER_DOC_H_
#define MESHTASTIC_CLUSTER_DOC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_hlc.h"

/*
 * The cluster document (agents-xhli.4, docs/CLUSTER-SYNC-M4.md §2): a sorted
 * map of independently-stamped entries,
 *
 *   key   = (layer, node_id, section)
 *   entry = (stamp, tombstone, payload)
 *
 * with LWW-by-HLC as the only merge rule and effective(me, sec) = the NODE
 * entry when present and live, else BASE. base/ and nodes/<id>/ are separate
 * keys with separate stamps — the §7.7 anti-shadowing shape: a pin never
 * competes with base for a stamp, it merely wins at apply time, so unpinning
 * lands on the CURRENT base, not a frozen one.
 *
 * Pure logic over caller-owned storage: no locks, no Zephyr subsystems beyond
 * sys/crc, so every merge/hash/effective decision is unit-testable on
 * native_sim (tests/cluster). The caller (meshtastic_cluster.c) serialises
 * access and owns persistence.
 */

/* Encoded meshtastic_Config (one section) upper bound. Largest allowlisted
 * section is DeviceConfig (100 B) plus the oneof wrapper; asserted against
 * the generated sizes where they are visible (meshtastic_cluster.c). Must
 * match ClusterEntry.payload max_size in cluster.options. */
#define MESHTASTIC_CLUSTER_PAYLOAD_MAX 128U

enum meshtastic_cluster_layer {
	MESHTASTIC_CLUSTER_LAYER_BASE = 0,
	MESHTASTIC_CLUSTER_LAYER_NODE = 1,
};

struct meshtastic_cluster_key {
	uint8_t layer;    /* enum meshtastic_cluster_layer */
	uint32_t node_id; /* 0 for BASE */
	uint16_t section; /* meshtastic_Config payload-variant tag */
};

struct meshtastic_cluster_entry {
	bool used;
	struct meshtastic_cluster_key key;
	struct meshtastic_hlc_stamp stamp;
	bool tombstone;
	uint16_t payload_len;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
};

/* The doc borrows its entry array from the caller so capacity is a caller
 * decision (Kconfig on device, arbitrary in tests). Entries are kept sorted
 * by key, so hashing and vector iteration are index order. */
struct meshtastic_cluster_doc {
	struct meshtastic_cluster_entry *entries;
	uint16_t cap;
	uint16_t count;
};

/*
 * A SCOPE: the set of keys a node CLAIMS to track (agents-xhli.10,
 * docs/CLUSTER-SYNC-M4.md §6.1).
 *
 * The document has always been able to run out of table, and until now the
 * digest was taken over what a node HAPPENED TO HOLD. That is why eviction was
 * unbuildable: drop a row and you advertise a hash nobody else has, so you
 * mismatch every peer forever. Declaring a scope inverts it — the digest covers
 * what the node says it tracks, holding less than that is a bug rather than a
 * policy, and a node that cannot hold everything narrows its claim instead of
 * lying about it.
 *
 *   FULL     every key
 *   CORE(x)  base ∪ nodes/x
 *   BASE     base alone — an INTERSECTION RESULT only; nothing declares it
 *
 * Two properties make this work, and both are load-bearing:
 *
 * 1. EVERY SCOPE IS EXACTLY EVALUABLE BY A RECEIVER, from the sender's node id
 *    alone. So a receiver can re-take its own hash over the sender's scope and
 *    compare like for like. Any future scope must keep that property exactly —
 *    an approximate membership test (a bloom filter, say) makes two nodes
 *    disagree about whether a key is in scope, so their hashes differ over
 *    identical content and they never converge.
 *
 * 2. CORE(me) IS EXACTLY THE CLOSURE OF effective(me, ·) — it reads nodes/me
 *    and base, and nothing else. Narrowing to CORE therefore cannot change what
 *    this node runs. It costs only the ability to serve OTHER nodes their
 *    entries back (§2.3), which is precisely the "capability tier, not a
 *    different protocol" the design register §7.7 promised.
 */
enum meshtastic_cluster_scope_kind {
	MESHTASTIC_CLUSTER_SCOPE_BASE = 0,
	MESHTASTIC_CLUSTER_SCOPE_CORE = 1,
	MESHTASTIC_CLUSTER_SCOPE_FULL = 2,
};

struct meshtastic_cluster_scope {
	uint8_t kind;   /* enum meshtastic_cluster_scope_kind */
	uint32_t owner; /* whose CORE this is; ignored for BASE and FULL */
};

void meshtastic_cluster_scope_make(struct meshtastic_cluster_scope *out, uint8_t kind,
				   uint32_t owner);

/* THE ONE MEMBERSHIP RULE. Hash, count, max-stamp, want, ingest, the vector
 * filter and the persistence load filter all go through this and nothing else.
 * The moment two of them disagree, a node advertises a document it does not
 * hold — which is the failure this whole mechanism exists to prevent. */
bool meshtastic_cluster_scope_contains(const struct meshtastic_cluster_scope *scope,
				       const struct meshtastic_cluster_key *key);

/* The keys BOTH scopes claim — the only set two nodes can meaningfully compare.
 * CORE(x) ∩ CORE(y) is BASE for x != y, which is why base is the leg every
 * node can always compare against whatever tier its peer is on. */
void meshtastic_cluster_scope_intersect(const struct meshtastic_cluster_scope *a,
					const struct meshtastic_cluster_scope *b,
					struct meshtastic_cluster_scope *out);

void meshtastic_cluster_doc_init(struct meshtastic_cluster_doc *doc,
				 struct meshtastic_cluster_entry *storage, uint16_t cap);

/* Total order over keys: layer, then node_id, then section. */
int meshtastic_cluster_key_cmp(const struct meshtastic_cluster_key *a,
			       const struct meshtastic_cluster_key *b);

/* The entry for @p key, or NULL. Index into the sorted table via doc->entries
 * for vector building; treat as invalidated by any accept. */
const struct meshtastic_cluster_entry *
meshtastic_cluster_doc_find(const struct meshtastic_cluster_doc *doc,
			    const struct meshtastic_cluster_key *key);

/*
 * The one merge rule. Stores (key, stamp, tombstone, payload) iff @p stamp is
 * strictly newer than what the doc holds for @p key (a new key counts as
 * newer-than-unset). Returns 1 stored, 0 stale-or-equal (ignored — the
 * caller's digest will invite the sender to pull OUR copy instead), -ENOSPC
 * table full, -EINVAL malformed (payload too large, tombstone with payload,
 * BASE tombstone, non-zero node_id on BASE / zero on NODE).
 */
int meshtastic_cluster_doc_accept(struct meshtastic_cluster_doc *doc,
				  const struct meshtastic_cluster_key *key,
				  const struct meshtastic_hlc_stamp *stamp, bool tombstone,
				  const uint8_t *payload, size_t payload_len);

/* The digest triple over the sorted rows (key, stamp, tombstone — payloads
 * deliberately excluded: the stamp IS the version, and hashing values would
 * make byte-identical re-encodings look like divergence). CRC-32, not a
 * cryptographic hash: collisions only delay convergence until the next
 * change or vector exchange, and a channel member — the only party who could
 * engineer one — is already trusted for more (CLUSTER-SYNC-M4.md §4). */
uint32_t meshtastic_cluster_doc_hash(const struct meshtastic_cluster_doc *doc);

/* The same three legs, restricted to @p scope. The unscoped forms above are
 * exactly the FULL-scope case — same bytes, same CRC — so a fleet where nobody
 * has narrowed its claim hashes identically to one built before scopes existed.
 * A receiver takes these over the SENDER's scope to compare like with like. */
uint32_t meshtastic_cluster_doc_hash_scoped(const struct meshtastic_cluster_doc *doc,
					    const struct meshtastic_cluster_scope *scope);
uint16_t meshtastic_cluster_doc_count_scoped(const struct meshtastic_cluster_doc *doc,
					     const struct meshtastic_cluster_scope *scope);
void meshtastic_cluster_doc_max_stamp_scoped(const struct meshtastic_cluster_doc *doc,
					     const struct meshtastic_cluster_scope *scope,
					     struct meshtastic_hlc_stamp *out);

/* Max stamp across the doc; the unset stamp when empty. */
void meshtastic_cluster_doc_max_stamp(const struct meshtastic_cluster_doc *doc,
				      struct meshtastic_hlc_stamp *out);

/*
 * The anti-entropy diff predicate (CLUSTER-SYNC-M4.md §3.3): true when a peer's
 * (key, stamp) row describes a version we do NOT have — a key we lack, or one we
 * hold at an older stamp — i.e. exactly the rows worth asking for. Rows where OUR
 * copy is newer return false and are deliberately not acted on: our own next
 * digest invites the peer to pull from us, so neither side ever pushes uninvited.
 *
 * Malformed keys (unknown layer, an owner on BASE or none on NODE) and unset
 * stamps return false — a row that meshtastic_cluster_doc_accept() would refuse
 * is not worth a round trip to fetch.
 *
 * CAPACITY IS PART OF THAT RULE. A NEW key on a full table returns false, because
 * accept() would refuse it with -ENOSPC and asking again next round would refuse
 * it again — forever, once per digest period, with nothing ever changing. Wanting
 * what cannot be stored is not optimism, it is an unbounded request loop. An
 * update to a key already held is still wanted: replacing an entry needs no free
 * slot, so a full table still tracks the fleet on everything it already knows
 * about.
 */
bool meshtastic_cluster_doc_wants(const struct meshtastic_cluster_doc *doc,
				  const struct meshtastic_cluster_key *key,
				  const struct meshtastic_hlc_stamp *stamp);

/*
 * The same predicate, scope-aware, and telling the two "no"s apart — because
 * they call for opposite responses.
 *
 *   NO        stale, malformed, or OUTSIDE the scope we advertise. Nothing to
 *             do: we never claimed it, so no peer expects it of us.
 *   YES       fetch it.
 *   NO_SPACE  we would fetch it and the table is full. The honest signal that
 *             this node has outgrown MAX_ENTRIES, and the trigger for narrowing
 *             the claim (§6.1) rather than silently tracking less than we say.
 *
 * An out-of-scope key must never report NO_SPACE: a node that has already
 * narrowed its claim must not narrow it again over a row it does not want.
 */
enum meshtastic_cluster_want {
	MESHTASTIC_CLUSTER_WANT_NO = 0,
	MESHTASTIC_CLUSTER_WANT_YES,
	MESHTASTIC_CLUSTER_WANT_NO_SPACE,
};

enum meshtastic_cluster_want
meshtastic_cluster_doc_want(const struct meshtastic_cluster_doc *doc,
			    const struct meshtastic_cluster_scope *mine,
			    const struct meshtastic_cluster_key *key,
			    const struct meshtastic_hlc_stamp *stamp);

/*
 * Drop everything outside @p scope, in one compacting pass that preserves sort
 * order. @p on_evict (may be NULL) is called once per dropped key BEFORE it is
 * overwritten, so the caller can forget it in persistent storage too. Returns
 * how many were dropped.
 *
 * This is the whole eviction policy. There is no victim choice and no LRU, and
 * that is the point: a per-entry policy makes the held set diverge from the
 * declared set, and the declared set is the only thing a digest can honestly
 * describe.
 */
/*
 * Drop EVERYTHING, firing @p on_evict per key first so the caller can forget it
 * in persistent storage too. Returns how many were dropped.
 *
 * Not expressible as a retain(): base is inside every scope by construction, so
 * no scope means "nothing". This is a deliberate local act — an operator saying
 * "forget the fleet's document and start again" — and it is safe precisely
 * because it writes nothing to the fleet: anti-entropy is level-triggered, so a
 * node cleared alone simply pulls the document back from a peer that still has
 * it. Clearing a whole fleet therefore means clearing every member before any
 * of them can re-seed the others.
 */
uint16_t meshtastic_cluster_doc_clear(struct meshtastic_cluster_doc *doc,
				      void (*on_evict)(const struct meshtastic_cluster_key *key,
						       void *ctx),
				      void *ctx);

uint16_t meshtastic_cluster_doc_retain(struct meshtastic_cluster_doc *doc,
				       const struct meshtastic_cluster_scope *scope,
				       void (*on_evict)(const struct meshtastic_cluster_key *key,
							void *ctx),
				       void *ctx);

/*
 * effective(me, section): the NODE entry for @p node_id when present and not
 * a tombstone, else the BASE entry, else NULL. Returns a payload-bearing
 * entry only — a tombstoned NODE entry falls through to BASE by design.
 */
const struct meshtastic_cluster_entry *
meshtastic_cluster_doc_effective(const struct meshtastic_cluster_doc *doc, uint32_t node_id,
				 uint16_t section);

/*
 * THE VERSION OF THAT ANSWER — the newest stamp across BOTH layers for the
 * section, whichever layer actually supplied the value. False when the document
 * holds no entry for the section at either layer.
 *
 * Why it spans both layers rather than just reporting the winning entry's own
 * stamp: the version is what the reconciler writes into the config store, and
 * therefore what says "the store is running the document's current answer".
 * `unpin` is what forces the distinction. A tombstone REMOVES the pin's stamp
 * from the document, so after an unpin the answer is the base entry — whose
 * stamp is OLDER than the pin the store is still holding. Feed the base's own
 * stamp to the store's last-writer-wins merge and it loses, the write is
 * declined, and the node quietly keeps running the value it was just told to
 * stop running. The tombstone is a write, minted after the pin, and counting it
 * in the version is what makes "unpin reverts to the current base" true rather
 * than aspirational (D6/D7).
 *
 * The rule generalises the same way for the pin direction: a per-node entry
 * pulled back by a wiped node (§2.3) can be older than a base promoted while it
 * was away, and taking the newest stamp of the pair is what lets that pin still
 * reach the store.
 *
 * The one consequence, named because it is visible in the counters: while a
 * section is pinned, a NEW base for it bumps the version without changing the
 * answer, so the reconciler rewrites identical bytes once per base update. That
 * is a redundant flash write on a rare, operator-driven event — the alternative
 * is `unpin` silently doing nothing, which is not a trade.
 */
bool meshtastic_cluster_doc_effective_version(const struct meshtastic_cluster_doc *doc,
					      uint32_t node_id, uint16_t section,
					      struct meshtastic_hlc_stamp *out);

#endif /* MESHTASTIC_CLUSTER_DOC_H_ */
