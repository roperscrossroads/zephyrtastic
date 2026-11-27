/* SPDX-License-Identifier: GPL-3.0 */

/*
 * The BLE peer link as a link bearer (ATTACHMENT-SCOPE §4). Link
 * identity is the peer's beat node number; an envelope is frame kind LINK on
 * the peer link's frame channel; the link is trusted when the connection is
 * encrypted (bonded, security >= L2). The BLE module delivers arrivals into
 * meshtastic_link_bearer_rx() and reports disconnects.
 */

#include <errno.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>

#include "meshtastic_link_bearer.h"
#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_peer_codec.h"
#include "meshtastic_ble_registry.h"

static int ble_send(uint32_t peer, const uint8_t *env, size_t len)
{
	return meshtastic_ble_peer_frame_send_to_kind(peer, env, len, MESHTASTIC_BLE_PEER_KIND_LINK);
}

static bool ble_link_info(uint32_t peer, struct meshtastic_link_info *out)
{
	uint8_t flags;

	if (peer == 0U || out == NULL) {
		return false;
	}
	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		struct meshtastic_ble_peer_rx rx;

		if (!meshtastic_ble_peer_rx_get(i, &rx, NULL) || rx.last.node_num != peer) {
			continue;
		}
		out->up = true;
		out->takes_envelopes = meshtastic_ble_peer_node_flags(peer, &flags) &&
				       (flags & MESHTASTIC_BLE_PEER_FLAG_LINK) != 0U;
		out->auth = (meshtastic_ble_conn_security(i) >= BT_SECURITY_L2)
				    ? MESHTASTIC_LINK_AUTH_ENCRYPTED
				    : MESHTASTIC_LINK_AUTH_NONE;
		out->mtu = 20U; /* the guaranteed ATT payload the chunker assumes */
		out->rtt_ms = 0U;
		return true;
	}
	return false;
}

const struct meshtastic_link_bearer meshtastic_link_bearer_ble = {
	.name = "ble",
	.send = ble_send,
	.link_info = ble_link_info,
};
