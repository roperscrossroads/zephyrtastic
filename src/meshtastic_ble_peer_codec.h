/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_PEER_CODEC_H_
#define MESHTASTIC_BLE_PEER_CODEC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Wire formats for the node-to-node BLE peer link (agents-a4it.3/.4), plus the
 * receive-side sequence accounting. Pure C — no Bluetooth or kernel
 * dependency — so every byte of framing and the loss arithmetic are
 * unit-testable on native_sim, where no BLE controller exists.
 *
 * NOT AUTHENTICATION: magic and version are framing sanity only. They travel
 * in the clear on an unencrypted link whose only payload is data the node
 * already broadcasts (the node number is in the advertisement). Do not build
 * trust decisions on them.
 */

/*
 * The beat: 16 bytes, all little-endian, deliberately inside the default
 * 23-byte ATT MTU so the link never depends on MTU exchange succeeding (the
 * flakiest part of the existing phone path — meshtastic_ble.c mtu_exchange).
 *
 *   [0]      magic     0x4D
 *   [1]      version   1
 *   [2]      flags     bit0 = HELLO (link start: receiver resyncs its seq)
 *   [3]      reserved  0 on encode, ignored on decode
 *   [4..7]   node_num  LE
 *   [8..11]  seq       LE — per-link, from 0, so gaps PROVE loss
 *   [12..15] uptime_s  LE — sender uptime in seconds
 */
#define MESHTASTIC_BLE_PEER_BEAT_LEN     16U
#define MESHTASTIC_BLE_PEER_BEAT_MAGIC   0x4DU
#define MESHTASTIC_BLE_PEER_BEAT_VERSION 1U
#define MESHTASTIC_BLE_PEER_FLAG_HELLO   0x01U

struct meshtastic_ble_peer_beat {
	uint8_t flags;
	uint32_t node_num;
	uint32_t seq;
	uint32_t uptime_s;
};

void meshtastic_ble_peer_beat_encode(const struct meshtastic_ble_peer_beat *beat,
				     uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN]);

/*
 * Returns 0, -EINVAL on wrong length, -EBADMSG on wrong magic, -ENOTSUP on a
 * version this build does not speak. `beat` is untouched on error.
 */
int meshtastic_ble_peer_beat_decode(const uint8_t *buf, size_t len,
				    struct meshtastic_ble_peer_beat *beat);

/*
 * The advertisement blob: 7 bytes of BT_DATA_MANUFACTURER_DATA payload.
 *
 *   [0..1] company id LE — 0xFFFF, the Bluetooth SIG's reserved
 *          "internal/test use only" identifier. Correct for a project with no
 *          assigned Company ID; MUST NOT ship in a product that gets one.
 *   [2]    format 0x01 — makes the blob versionable
 *   [3..6] node_num LE
 *
 * A scanner must require BOTH the Meshtastic service UUID and this blob
 * (an unrelated 0xFFFF user is not a peer), then reject its own node_num
 * (reflection guard). Those checks live with the scanner (agents-a4it.5).
 */
#define MESHTASTIC_BLE_PEER_ADV_LEN     7U
#define MESHTASTIC_BLE_PEER_ADV_COMPANY 0xFFFFU
#define MESHTASTIC_BLE_PEER_ADV_FORMAT  0x01U

void meshtastic_ble_peer_adv_encode(uint8_t buf[MESHTASTIC_BLE_PEER_ADV_LEN],
				    uint32_t node_num);

/* Returns 0, -EINVAL on wrong length, -EBADMSG on wrong company or format. */
int meshtastic_ble_peer_adv_decode(const uint8_t *buf, size_t len, uint32_t *node_num);

/*
 * Receive-side accounting for one link. seq gaps are LOSS (proven, not
 * guessed); a HELLO — or the first beat ever — resyncs; a seq at or below the
 * last one seen without HELLO is treated as a peer restart (resync, counted).
 */
struct meshtastic_ble_peer_rx {
	bool synced;
	uint32_t beats;   /* beats accepted (including the resyncing ones) */
	uint32_t lost;    /* sum of proven seq gaps */
	uint32_t resyncs; /* HELLOs after sync + backwards seqs (peer restarts) */
	struct meshtastic_ble_peer_beat last;
};

void meshtastic_ble_peer_rx_reset(struct meshtastic_ble_peer_rx *st);
void meshtastic_ble_peer_rx_account(struct meshtastic_ble_peer_rx *st,
				    const struct meshtastic_ble_peer_beat *beat);

#endif /* MESHTASTIC_BLE_PEER_CODEC_H_ */
