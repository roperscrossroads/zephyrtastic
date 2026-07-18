/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/**
 * @file
 * @brief Meshtastic in-RAM NodeDB public API.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_NODEDB_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_NODEDB_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MESHTASTIC_NODEDB_LONG_NAME_LEN            25U
#define MESHTASTIC_NODEDB_SHORT_NAME_LEN           5U
#define MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN       32U

struct meshtastic_nodedb_node {
	uint32_t num;
	uint32_t last_heard_uptime_sec;
	float snr;
	uint8_t channel;
	uint8_t next_hop;
	bool via_mqtt;
	bool has_hops_away;
	uint8_t hops_away;

	bool is_favorite;
	bool is_ignored;

	bool has_user;
	char long_name[MESHTASTIC_NODEDB_LONG_NAME_LEN];
	char short_name[MESHTASTIC_NODEDB_SHORT_NAME_LEN];
	uint8_t hw_model;
	uint8_t role;
	bool is_licensed;
	bool has_is_unmessagable;
	bool is_unmessagable;
	size_t public_key_len;
	uint8_t public_key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
};

/**
 * @brief Return the number of entries currently stored in the in-RAM NodeDB.
 */
size_t meshtastic_nodedb_count(void);

/**
 * @brief Copy one NodeDB entry by node number.
 *
 * @retval 0 Entry copied.
 * @retval -EINVAL @p out is NULL.
 * @retval -ENOENT The node is not present.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_get(uint32_t node_num, struct meshtastic_nodedb_node *out);

/**
 * @brief Copy one NodeDB entry by table index.
 *
 * Entries are indexed from 0 to meshtastic_nodedb_count() - 1.
 *
 * @retval 0 Entry copied.
 * @retval -EINVAL @p out is NULL.
 * @retval -ENOENT @p index is out of range.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_get_by_index(size_t index, struct meshtastic_nodedb_node *out);

/**
 * @brief Copy a node's 32-byte X25519 public key.
 *
 * Looks in the hot NodeDB first, then the warm key tier, so a key remains
 * usable for PKC direct messages after the node's full record is evicted from
 * the hot store.
 *
 * @retval 0 Key copied into @p out.
 * @retval -EINVAL @p out is NULL.
 * @retval -ENOENT No public key known for @p node_num.
 */
int meshtastic_nodedb_copy_pubkey(uint32_t node_num,
				  uint8_t out[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN]);

/**
 * @brief Number of live entries in the warm key tier.
 *
 * Returns 0 when the warm tier is not compiled in (no key persistence).
 */
size_t meshtastic_nodedb_warm_count(void);

/**
 * @brief Read the @p index-th live warm-tier entry (introspection/tuning).
 *
 * Entries are indexed 0 .. meshtastic_nodedb_warm_count()-1 over the non-empty
 * slots. @p last_seen is the LRU recency stamp (wall-clock epoch once seeded).
 *
 * @retval 0 Entry returned.
 * @retval -EINVAL @p num or @p last_seen is NULL.
 * @retval -ENOENT @p index is out of range.
 * @retval -ENOTSUP The warm tier is not compiled in.
 */
int meshtastic_nodedb_warm_get(size_t index, uint32_t *num, uint32_t *last_seen);

/**
 * @brief Resolve a 1-byte next_hop/relay id back to a full node number.
 *
 * On-wire next_hop/relay_node carry only the low byte of a node number. Returns
 * the unique known node (never the local node) whose low byte matches, or 0 if
 * no match or the byte is ambiguous (>1 match) — callers then fall back to flood.
 */
uint32_t meshtastic_nodedb_resolve_unique_last_byte(uint8_t last_byte);

/**
 * @brief Learned next-hop (low byte of the relay to reach @p dest), 0 if none.
 */
uint8_t meshtastic_nodedb_get_next_hop(uint32_t dest);

/**
 * @brief Set the learned next-hop byte for @p dest.
 *
 * @retval 0 Updated.
 * @retval -ENOENT @p dest is not in the NodeDB.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_set_next_hop(uint32_t dest, uint8_t next_hop);

/**
 * @brief Mark a node favorited / un-favorited in the in-RAM NodeDB.
 *
 * Favorited nodes are protected from cache eviction. Best-effort: acting on a
 * node not present in the NodeDB is a no-op.
 *
 * @retval 0 Flag updated.
 * @retval -ENOENT The node is not present.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_set_favorite(uint32_t node_num, bool favorite);

/**
 * @brief Mark a node ignored / un-ignored in the in-RAM NodeDB.
 *
 * @retval 0 Flag updated.
 * @retval -ENOENT The node is not present.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_set_ignored(uint32_t node_num, bool ignored);

/**
 * @brief Remove a node from the in-RAM NodeDB.
 *
 * The local node cannot be removed.
 *
 * @retval 0 Entry removed.
 * @retval -EINVAL @p node_num is the local node.
 * @retval -ENOENT The node is not present.
 * @retval -ENOTSUP NodeDB support is not enabled.
 */
int meshtastic_nodedb_remove(uint32_t node_num);

/**
 * @brief Clear the in-RAM NodeDB (and persisted peer keys), keeping the local node.
 *
 * Mirrors the reference NodeDB::resetNodes: the local node is always retained.
 * When @p keep_favorites is true, favorited peers survive; otherwise every peer
 * is removed. Persisted peer public keys (the warm ring and its NVS records) are
 * always cleared — warm entries are never favorites. The store is persisted
 * synchronously, so the caller may reboot without flushing the peer-key subtree.
 *
 * No-op when NodeDB support is not enabled (weak stub).
 */
void meshtastic_nodedb_reset(bool keep_favorites);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_NODEDB_H_ */
