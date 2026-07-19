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
void meshtastic_admin_handle_remote(const struct meshtastic_packet *pkt);

/** Reset admin edit-transaction state (call on phone disconnect). */
void meshtastic_admin_reset(void);

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

#endif /* MESHTASTIC_ADMIN_H_ */
