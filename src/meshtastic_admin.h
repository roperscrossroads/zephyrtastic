/* SPDX-License-Identifier: GPL-3.0
 *
 * Local AdminMessage handling — lets the directly-connected app WRITE config
 * (portnum ADMIN_APP) via the PhoneAPI, not just read it.
 */
#ifndef MESHTASTIC_ADMIN_H_
#define MESHTASTIC_ADMIN_H_

#include <stdbool.h>

#include "meshtastic/mesh.pb.h"

/**
 * Handle a locally-submitted ADMIN_APP packet (to == this node).
 *
 * @param pkt Decoded MeshPacket from ToRadio (which_payload_variant decoded).
 * @return true if consumed as admin (caller must NOT transmit it on the mesh).
 */
bool meshtastic_admin_handle_local(const meshtastic_MeshPacket *pkt);

/** Reset admin edit-transaction state (call on phone disconnect). */
void meshtastic_admin_reset(void);

#endif /* MESHTASTIC_ADMIN_H_ */
