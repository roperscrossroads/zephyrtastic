/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_PEER_H_
#define MESHTASTIC_BLE_PEER_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic_ble_peer_codec.h"

/*
 * The node-to-node BLE peer link (agents-a4it.3/.5): this port's own peer
 * GATT service (peripheral half) plus the passive scanner, central role and
 * beat engine (central half). Deliberately separate from the Meshtastic phone
 * service — that is a single-client stateful session whose frames a peer
 * would consume, and reusing it would make a peer indistinguishable from a
 * phone.
 */

#if defined(CONFIG_MESHTASTIC_BLE_PEER)

/* ---- peripheral half (the service) ---- */

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

/* ---- central half (scanner + outbound link) ---- */

/* Arm/disarm the passive peer scanner. Default off — armed from the shell so
 * a first bench session can attribute any instability to it. Scanning runs
 * only while there is no outbound peer link and resumes automatically after a
 * connect attempt completes or fails. */
int meshtastic_ble_peer_scan_set(bool on);
bool meshtastic_ble_peer_scan_armed(void);

/* Target one node number (0 = connect to any peer found) and arm the scan. */
int meshtastic_ble_peer_connect(uint32_t node_num);

/* Tear down the outbound peer link (if any). */
int meshtastic_ble_peer_disconnect(void);

/* One row of the scanner's advert table. */
struct meshtastic_ble_peer_seen {
	uint32_t node_num;
	int8_t rssi;
	int64_t last_ms; /* k_uptime_get() when last heard */
};

/* Copy row i (0-based). Returns false past the end of the table. */
bool meshtastic_ble_peer_seen_get(unsigned int i, struct meshtastic_ble_peer_seen *out);

/* Outbound link state for the shell. */
struct meshtastic_ble_peer_link {
	bool connected;   /* outbound conn exists */
	bool ready;       /* discovery + subscribe complete, beats flowing */
	uint32_t node_num; /* peer's advertised node number */
	unsigned int index; /* registry slot of the outbound conn */
	uint32_t tx_beats;
};

void meshtastic_ble_peer_link_get(struct meshtastic_ble_peer_link *out);

/* Counters over assumptions. */
struct meshtastic_ble_peer_stats {
	uint32_t adverts_matched;    /* peer adverts seen (uuid + blob) */
	uint32_t reflections;        /* own node_num in an advert, rejected */
	uint32_t connects_attempted;
	uint32_t connects_failed;
	uint32_t discovery_failures;
	uint32_t notify_tx_beats;    /* peripheral -> central */
	uint32_t write_tx_beats;     /* central -> peripheral */
	uint32_t hello_malformed;    /* beat writes that failed decode */
	uint32_t hello_rejected_late; /* peer evidence on an already-phone slot */
};

void meshtastic_ble_peer_stats_get(struct meshtastic_ble_peer_stats *out);

/* ---- provided by meshtastic_ble.c ---- */

/* Peer evidence (a valid beat) for classification. Returns 0 on the
 * transition to peer, 1 if already a peer, -EBUSY if already the phone
 * (too late), -EINVAL for a dead slot. */
int meshtastic_ble_classify_peer_evidence(unsigned int index);

/* The 16 bytes (LE) of the Meshtastic service UUID, for the scan filter. */
const uint8_t *meshtastic_ble_service_uuid128(void);

#else /* !CONFIG_MESHTASTIC_BLE_PEER */

static inline void meshtastic_ble_peer_conn_down(unsigned int index)
{
	(void)index;
}

#endif /* CONFIG_MESHTASTIC_BLE_PEER */

#endif /* MESHTASTIC_BLE_PEER_H_ */
