/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_PEER_H_
#define MESHTASTIC_BLE_PEER_H_

#include <errno.h>
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
 * `index` (and the k_uptime of the last beat, if last_ms is non-NULL).
 * Returns true if that slot has received at least one beat. */
bool meshtastic_ble_peer_rx_get(unsigned int index, struct meshtastic_ble_peer_rx *out,
				int64_t *last_ms);

/* Poke the beat engine: one immediate beat on every active link. */
void meshtastic_ble_peer_beat_now(void);

/* ---- frame channel (agents-xhli.1, PEER-TRANSPORT-DESIGN.md §2) ---- */

/* Notify one wire frame to the subscribed peer central, chunked at the
 * guaranteed 20-byte ATT payload. Returns 0 once every chunk is queued,
 * -ENOTCONN when nobody has enabled frame notifications, -EINVAL/-EMSGSIZE
 * from the chunker, else the first bt_gatt_notify error (frame abandoned —
 * the receiver's partial dies on the next FIRST chunk). */
int meshtastic_ble_peer_frame_notify(const uint8_t *frame, size_t len);

/* True while a connected central has notifications enabled on the frame
 * channel. */
bool meshtastic_ble_peer_frame_notify_ready(void);

/* The M2 seam (bearer ingest): frames completed on the frame channel — peer
 * chunk-writes on the peripheral side, notifications on the central side —
 * land here. The callback runs on the BT RX thread WITH the peer lock held —
 * copy or queue the bytes and return; never call back into blepeer APIs from
 * it. NULL (the default) means completed frames are counted and dropped. */
typedef void (*meshtastic_ble_peer_frame_cb_t)(unsigned int index, const uint8_t *frame,
					       size_t len);
void meshtastic_ble_peer_frame_rx_register(meshtastic_ble_peer_frame_cb_t cb);

/* Send one wire frame to the named node over whichever live BLE peer link
 * reaches it: chunk-writes up the outbound (central) link, or notifications
 * down the peripheral characteristic for an inbound central. Returns 0,
 * -EHOSTUNREACH when no live link reaches that node, else the codec/GATT
 * error. */
int meshtastic_ble_peer_frame_send_to(uint32_t node_num, const uint8_t *frame, size_t len);

/* TX divert (agents-xhli.2): called at the LoRa TX choke point with a queued
 * wire frame. Sends it over a BLE peer link INSTEAD of the radio when — and
 * only when — this node originated it (wire src is us; a relayed frame kept
 * its originator's src and must never bridge onto BLE), it is a unicast, and
 * a live link reaches the destination. Returns 0 when the frame left over
 * BLE; any nonzero means "put it on the air as usual". */
int meshtastic_ble_peer_tx_try_divert(const uint8_t *wire, uint32_t len);

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
	bool frame_ready; /* frame channel discovered + subscribed */
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
	uint32_t frame_rx_frames;    /* frames completed from peer chunk-writes */
	uint32_t frame_rx_rejected;  /* chunk-writes the reassembler refused */
	uint32_t frame_tx_frames;    /* frames fully notified to the central */
	uint32_t frame_tx_failed;    /* frames abandoned on a notify error */
};

void meshtastic_ble_peer_stats_get(struct meshtastic_ble_peer_stats *out);

/* ---- provided by meshtastic_ble.c ---- */

/* Peer evidence (a valid beat) for classification. Returns 0 on the
 * transition to peer, 1 if already a peer, -EBUSY if already the phone
 * (too late), -EINVAL for a dead slot. */
int meshtastic_ble_classify_peer_evidence(unsigned int index);

/* The 16 bytes (LE) of the Meshtastic service UUID, for the scan filter. */
const uint8_t *meshtastic_ble_service_uuid128(void);

/* Peer address string + connection age for slot `index` (false if free). */
bool meshtastic_ble_slot_info(unsigned int index, char *addr, size_t addr_len, int64_t *age_ms);

/* Advertiser state, tracked in meshtastic_ble.c (a4it.2). */
bool meshtastic_ble_adv_active(void);
uint32_t meshtastic_ble_adv_starts(void);

#else /* !CONFIG_MESHTASTIC_BLE_PEER */

static inline void meshtastic_ble_peer_conn_down(unsigned int index)
{
	(void)index;
}

/* No peer link in this image: every frame goes on the air. */
static inline int meshtastic_ble_peer_tx_try_divert(const uint8_t *wire, uint32_t len)
{
	(void)wire;
	(void)len;
	return -ENOTSUP;
}

#endif /* CONFIG_MESHTASTIC_BLE_PEER */

#endif /* MESHTASTIC_BLE_PEER_H_ */
