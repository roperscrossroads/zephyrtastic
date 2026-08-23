/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_PEER_H_
#define MESHTASTIC_BLE_PEER_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic_ble_peer_codec.h"

/*
 * The peripheral half of the node-to-node BLE peer link (agents-a4it.3): this
 * port's own peer GATT service, deliberately separate from the Meshtastic
 * phone service — the phone service is a single-client stateful session whose
 * frames a peer would consume, and reusing it would make a peer
 * indistinguishable from a phone.
 */

#if defined(CONFIG_MESHTASTIC_BLE_PEER)

/* Notify one heartbeat to the subscribed peer central. The first beat after a
 * subscribe carries HELLO and restarts seq from 0 (the receiver resyncs).
 * Returns 0, -ENOTCONN when nobody is subscribed, else the bt_gatt_notify
 * error. */
int meshtastic_ble_peer_send_beat(void);

/* True while a connected central has notifications enabled on the beat. */
bool meshtastic_ble_peer_notify_ready(void);

/* Copy the receive-side accounting for the connection in registry slot
 * `index`. Returns true if that slot has received at least one beat. */
bool meshtastic_ble_peer_rx_get(unsigned int index, struct meshtastic_ble_peer_rx *out);

/* Called from the BLE disconnect path for every connection: drops the slot's
 * beat accounting so a recycled bt_conn_index never inherits a dead link's
 * counters. */
void meshtastic_ble_peer_conn_down(unsigned int index);

#else /* !CONFIG_MESHTASTIC_BLE_PEER */

static inline void meshtastic_ble_peer_conn_down(unsigned int index)
{
	(void)index;
}

#endif /* CONFIG_MESHTASTIC_BLE_PEER */

#endif /* MESHTASTIC_BLE_PEER_H_ */
