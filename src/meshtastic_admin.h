/* SPDX-License-Identifier: GPL-3.0
 *
 * AdminMessage handling (portnum ADMIN_APP) — lets a client WRITE config, not
 * just read it. Two entry points: the directly-connected app (local, trusted)
 * and an authorized remote node over the mesh (PKC admin_key / session passkey).
 */
#ifndef MESHTASTIC_ADMIN_H_
#define MESHTASTIC_ADMIN_H_

#include <stdbool.h>

#include "meshtastic/mesh.pb.h"

struct meshtastic_packet;

/**
 * Handle a locally-submitted ADMIN_APP packet (to == this node), from the
 * directly-connected app via ToRadio — trusted transport, no passkey required
 * (but refused outright when the node is SecurityConfig.is_managed).
 *
 * @param pkt Decoded MeshPacket from ToRadio (which_payload_variant decoded).
 * @return true if consumed as admin (caller must NOT transmit it on the mesh).
 */
bool meshtastic_admin_handle_local(const meshtastic_MeshPacket *pkt);

/**
 * Handle an ADMIN_APP packet received over the mesh (to == this node,
 * from != self). Authorizes the sender (PKC key in SecurityConfig.admin_key, or
 * the legacy admin channel) and enforces the session passkey on mutating ops,
 * replying — including any ACK/error — back over the mesh. Always consumes the
 * packet (the caller must not deliver it to the phone as a normal RX packet).
 *
 * @param pkt Decoded internal packet carrying the AdminMessage bytes.
 */
void meshtastic_admin_handle_remote(const struct meshtastic_packet *pkt,
				    const meshtastic_MeshPacket *mesh);

/** Reset admin edit-transaction state (call on phone disconnect). */
void meshtastic_admin_reset(void);

/**
 * True if @p node_id's known public key is in SecurityConfig.admin_key[] — this
 * node's "is that one of my masters?" test, over the hot→warm NodeDB key lookup
 * so an evicted admin is not locked out (A-1).
 *
 * Two callers ask it about different subjects. Remote admin asks it about a
 * packet's SENDER. Cluster sync asks it about the AUTHOR recorded in a document
 * entry's stamp — the D4 base-authorship gate (docs/CLUSTER-SYNC-M4.md §4) —
 * which is a claim no receiver can cryptographically refute today (there is no
 * signing primitive in Meshtastic PKC; §4 states that plainly). The gate is
 * still worth having: it stops an ordinary fleet member's honest mistake from
 * rewriting fleet base config, and the channel PSK remains the real boundary.
 */
bool meshtastic_admin_node_is_trusted(uint32_t node_id);

/**
 * True when this node is administratively managed (SecurityConfig.is_managed):
 * configuration may be changed only by an authorized remote admin, so the local
 * path is refused.
 *
 * Exported so other local write surfaces can honour the same policy. The gate is
 * only meaningful if every path that writes config consults it — a surface that
 * skips it silently reopens what a managed node is asserting is closed.
 */
bool meshtastic_admin_is_managed(void);

/**
 * @brief Whether a deferred config-change reboot is currently scheduled.
 *
 * A LoRa config write (F-1), a module-config write, and other sections that only
 * take effect on restart schedule a reboot via the set_config path so the change
 * is actually applied. Exposed for introspection and tests.
 */
bool meshtastic_admin_reboot_scheduled(void);

/** Cancel a scheduled config-change reboot (used by tests to avoid rebooting). */
void meshtastic_admin_cancel_reboot(void);

#endif /* MESHTASTIC_ADMIN_H_ */
