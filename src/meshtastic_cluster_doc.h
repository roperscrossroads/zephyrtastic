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
