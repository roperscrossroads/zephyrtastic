/* SPDX-License-Identifier: GPL-3.0
 *
 * Node identity derived from the X25519 public key (CONFIG_MESHTASTIC_NODE_ID_FROM_KEY).
 *
 * The reference firmware (>= 2.8, commit 8267bb22b "Packet Signing via XEdDSA") sets the node
 * number to crc32(public_key) in NodeDB::createNewIdentity(). It shipped in the same change as
 * packet signing because it makes the id a COMMITMENT to the key: a forger cannot choose both,
 * so a receiver may trust a stranger's first signed NodeInfo when crc32(key) == its id. Without
 * this binding that first-contact rule is unportable (docs: SIGNING-AND-IDENTITY-DESIGN.md §3).
 *
 * The id this port used before — hardware bytes [2..5] — is the reference's own older rule,
 * still used there as the provisional value until a key exists. It remains the provisional
 * value here, from whatever MESHTASTIC_NODE_ID_SOURCE selects.
 */

#ifndef MESHTASTIC_NODE_IDENTITY_H_
#define MESHTASTIC_NODE_IDENTITY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Settings subtree holding the last adopted id. Deliberately OUTSIDE lockdown's sealed set
 * (meshtastic, mtnode, mtrec, mtclus, mtbackup), so a locked boot — which cannot read the
 * sealed keypair — still boots under the same id. A node id is transmitted in the clear in
 * every frame, so storing it unsealed discloses nothing. */
#define MESHTASTIC_NODE_ID_SUBTREE "mtid"

/* The reference's rule: standard CRC-32 (reflected, poly 0xEDB88320, init and final xor
 * 0xFFFFFFFF — the ErriezCRC32 crc32Buffer upstream links) over the key bytes in index order,
 * used as the id with no byte swap. */
uint32_t meshtastic_node_id_from_public_key(const uint8_t *key, size_t len);

/* Not 0..3 (reserved) and not broadcast. */
bool meshtastic_node_id_usable(uint32_t id);

enum meshtastic_node_id_origin {
	MESHTASTIC_NODE_ID_ORIGIN_KEY,         /* crc32 of our public key */
	MESHTASTIC_NODE_ID_ORIGIN_STORE,       /* no key readable (locked boot): the last one */
	MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL, /* no key and nothing stored: the source choice */
};

/* The decision, with no I/O, so every branch is testable. `key` is NULL when no identity key
 * is available. A derived id that is not usable is refused and the no-key rules apply — a
 * deliberate divergence: the reference has no such check, but a node answering to the
 * broadcast address would be unreachable, whereas one on its hardware id merely loses the
 * id-key binding. */
uint32_t meshtastic_node_id_choose(const uint8_t *key, bool have_stored, uint32_t stored,
				   uint32_t provisional, enum meshtastic_node_id_origin *origin);

/* Boot: read the stored id, read our public key, choose, persist a key-derived id that differs
 * from the stored one, and log the transition. Call after meshtastic_pki_init() and before
 * anything that captures the id (radio, NodeDB, cluster, BLE). Never fails: the worst case is
 * the provisional id, logged. */
uint32_t meshtastic_node_identity_adopt(uint32_t provisional);

/* After a lockdown unlock re-read the keypair: if the key implies a different id than the one
 * this boot runs under, persist it and warn. It is NOT applied at runtime — the id is captured
 * by the radio, NodeDB and cluster at init — so it takes effect at the next boot. */
void meshtastic_node_identity_key_reloaded(uint32_t current);

#endif /* MESHTASTIC_NODE_IDENTITY_H_ */
