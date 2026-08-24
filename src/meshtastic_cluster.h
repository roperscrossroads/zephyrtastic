/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_CLUSTER_H_
#define MESHTASTIC_CLUSTER_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Cluster config sync (agents-xhli.4, docs/CLUSTER-SYNC-M4.md): the module
 * half — digest cadence, PRIVATE_APP (256) ingest gated to the cluster
 * channel, NVS persistence, and the promote entry point. The document logic
 * itself is meshtastic_cluster_doc.{c,h}.
 *
 * M4b scope: the doc + digests + the anti-entropy walk (vector → entry pull →
 * merge) + the reconciler that applies effective(me) to the config store.
 * pin/unpin/tombstones and push-on-change are M4c.
 */

#if defined(CONFIG_MESHTASTIC_CLUSTER)

int meshtastic_cluster_init(void);

/* Fire one digest immediately (queued to the system workqueue — the same
 * context and code path as the timer, deliberately: this seam exists because
 * the timer-only TX path reached hardware untested once and cost a bench
 * cycle). Shell `cluster digest` and the sim tests use it. */
void meshtastic_cluster_digest_now(void);

/*
 * Promote MY current value of @p section (a meshtastic_Config payload-variant
 * tag) to the fleet base layer, stamped now. v1 allowlist: device, position,
 * power, display, bluetooth. Refused: security/network (the secret boundary),
 * anything unlisted, and — deliberately — lora: a fleet preset change can
 * permanently orphan any node that misses it until the straggler sweep
 * (CONFIG-CONVERGENCE.md §7.9) exists.
 *
 * Returns 0, -EPERM (not allowlisted / lora hazard), -ENOENT (section not in
 * the store), -EALREADY (this section's stored value IS the current fleet base
 * — it was applied FROM the document, so re-promoting it would only churn),
 * -ENOSPC (doc full), else an encode/store error.
 */
int meshtastic_cluster_promote(uint16_t section);

/*
 * Start an anti-entropy exchange with @p node_id now, as if its digest had just
 * mismatched: ClusterVectorReq → diff → ClusterEntryReq → merge → reconcile.
 * The bench and the sim use it so convergence is provable on demand instead of
 * on the digest cadence.
 *
 * Returns 0, -EBUSY (an exchange is already in flight — §3.3 allows one),
 * -ENOTCONN (the cluster channel is not provisioned), -EINVAL (not a node id).
 */
int meshtastic_cluster_pull(uint32_t node_id);

struct meshtastic_cluster_stats {
	/* digest_tx counts digests HANDED TO the send path. On a node whose
	 * radio TX is disabled (config.lora.tx_enabled = false) the frame is
	 * dropped at the radio choke point afterwards and never airs — and a
	 * digest is a broadcast, so no peer bearer carries it either. The shell
	 * says so rather than letting the counter imply transmission. */
	uint32_t digest_tx;
	uint32_t digest_rx_match;
	uint32_t digest_rx_mismatch;
	uint32_t rx_wrong_channel; /* port-256 frames not on the cluster channel */
	uint32_t rx_undecodable;
	uint32_t rx_not_implemented; /* M4c verbs received, counted, dropped */

	/* The anti-entropy walk (M4b). */
	uint32_t pull_started;	  /* exchanges we opened on a digest mismatch */
	uint32_t pull_timed_out;  /* opened, no vector came back in time */
	uint32_t vector_tx;	  /* vector chunks we served */
	uint32_t vector_rx;	  /* vector chunks we consumed */
	uint32_t entry_tx;	  /* entries we served */
	uint32_t entry_rx_applied;
	uint32_t entry_rx_stale;   /* arrived, ours was newer — LWW kept ours */
	uint32_t entry_rx_refused;  /* secret boundary, D4 authorship, unknown owner */
	uint32_t entry_rx_no_space; /* table full — the fleet outgrew MAX_ENTRIES */
	uint32_t rx_unsolicited;   /* a reply we had not asked for */
	uint32_t tx_busy;	   /* a peer asked while we were serving another */

	/* The reconciler (effective(me) → config store). */
	uint32_t sections_applied;
	uint32_t sections_kept_local; /* our store's version was newer */
	/* Sections the document holds and this node deliberately does NOT apply.
	 * v1: lora only — replicating a fleet preset change is safe, ACTING on
	 * one orphans any node that missed it until the straggler sweep exists.
	 * A non-zero value here is not an error; it is the fleet asking for
	 * something this firmware will not do yet. */
	uint32_t sections_held;
};

void meshtastic_cluster_stats_get(struct meshtastic_cluster_stats *out);

/* Human-readable state of the anti-entropy exchange in flight, and (when
 * non-NULL) the peer it is with — 0 when idle. */
const char *meshtastic_cluster_sync_state(uint32_t *peer);

/* Snapshot for the shell: entry count, doc hash, whether the cluster channel
 * currently resolves (index in *ch_index when true). */
bool meshtastic_cluster_channel_resolved(uint8_t *ch_index);
uint16_t meshtastic_cluster_entry_count(void);
uint32_t meshtastic_cluster_doc_hash_now(void);

/* Row copy for the shell's doc listing. Returns false past the end. */
struct meshtastic_cluster_entry;
bool meshtastic_cluster_entry_get(uint16_t idx, struct meshtastic_cluster_entry *out);

#endif /* CONFIG_MESHTASTIC_CLUSTER */

#endif /* MESHTASTIC_CLUSTER_H_ */
