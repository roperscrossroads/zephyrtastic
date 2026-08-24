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
 * M4a scope: the doc + digests + mismatch DETECTION (logged and counted, not
 * yet acted on). Vector/entry pull is M4b; pin/unpin/push-on-change is M4c.
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
 * the store), -ENOSPC (doc full), else an encode/store error.
 */
int meshtastic_cluster_promote(uint16_t section);

struct meshtastic_cluster_stats {
	uint32_t digest_tx;
	uint32_t digest_rx_match;
	uint32_t digest_rx_mismatch;
	uint32_t rx_wrong_channel; /* port-256 frames not on the cluster channel */
	uint32_t rx_undecodable;
	uint32_t rx_not_implemented; /* M4b/M4c verbs received, counted, dropped */
};

void meshtastic_cluster_stats_get(struct meshtastic_cluster_stats *out);

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
