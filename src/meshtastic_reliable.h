/* SPDX-License-Identifier: GPL-3.0
 *
 * Sender-side reliable delivery: retransmit want_ack unicast packets this node
 * originates until they are acknowledged (explicitly by a ROUTING ACK, or
 * implicitly by hearing a neighbour rebroadcast them) or the retry budget runs
 * out. See Kconfig.reliable and the scheduler knobs reliable.retries /
 * reliable.timeout.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_RELIABLE_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_RELIABLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_MESHTASTIC_RELIABLE_DELIVERY)

/**
 * Register a just-transmitted packet for reliable delivery, if it qualifies:
 * originated by us, want_ack set, unicast (not broadcast, not to ourselves) and
 * not a ROUTING control packet. Non-qualifying packets are ignored.
 *
 * @param local  Flat-struct view of the packet as sent (fallback source).
 * @param wire   The exact on-air bytes to retransmit.
 * @param wire_len Length of @p wire.
 * @param mesh   The outgoing MeshPacket when the send engine supplied one, else NULL;
 *               read in preference to @p local (C3 Phase 7d currency).
 */
void meshtastic_reliable_on_tx(const struct meshtastic_packet *local, const uint8_t *wire,
			       uint32_t wire_len, const meshtastic_MeshPacket *mesh);

/**
 * Consume a decoded ROUTING packet addressed to us. If its request_id matches a
 * pending send, resolve that send: a NONE error_reason is a delivery ACK, any
 * other error_reason is a NAK. Both stop retransmission.
 *
 * @param routing Flat-struct view (the fallback on the public inject / test path).
 * @param mesh    Decoded MeshPacket when the RF path supplied one, else NULL; read
 *                in preference to @p routing (C3 Phase 7 currency).
 */
void meshtastic_reliable_on_routing(const struct meshtastic_packet *routing,
				    const meshtastic_MeshPacket *mesh);

/**
 * Note that we heard our own packet @p id rebroadcast on-air (wire src == our
 * node id). This is an implicit ACK: a neighbour received and is forwarding it,
 * so stop retransmitting locally.
 */
void meshtastic_reliable_on_implicit_ack(uint32_t id);

/** Drop all pending trackers and cancel the retransmit timer (test/reset use). */
void meshtastic_reliable_reset(void);

#else

static inline void meshtastic_reliable_on_tx(const struct meshtastic_packet *local,
					     const uint8_t *wire, uint32_t wire_len,
					     const meshtastic_MeshPacket *mesh)
{
	ARG_UNUSED(local);
	ARG_UNUSED(wire);
	ARG_UNUSED(wire_len);
	ARG_UNUSED(mesh);
}

static inline void meshtastic_reliable_on_routing(const struct meshtastic_packet *routing,
						  const meshtastic_MeshPacket *mesh)
{
	ARG_UNUSED(routing);
	ARG_UNUSED(mesh);
}

static inline void meshtastic_reliable_on_implicit_ack(uint32_t id)
{
	ARG_UNUSED(id);
}

static inline void meshtastic_reliable_reset(void)
{
}

#endif /* CONFIG_MESHTASTIC_RELIABLE_DELIVERY */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_RELIABLE_H_ */
