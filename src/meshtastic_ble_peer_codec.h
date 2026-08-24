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

/*
 * The frame channel (agents-xhli.1, PEER-TRANSPORT-DESIGN.md §2): whole
 * Meshtastic wire frames chunked over a GATT characteristic. The chunk header
 * is one byte, so the channel works at the default 23-byte ATT MTU (20-byte
 * payloads) and never depends on MTU exchange succeeding; a larger MTU merely
 * means fewer chunks.
 *
 *   chunk := [0]   flags/seq: bit7 FIRST, bit6 LAST,
 *                  bits 0..5 seq (mod 64, from 0, +1 per chunk of a frame)
 *            [1..] fragment payload
 *   FIRST chunk payload begins [len_lo][len_hi] (total frame length, LE),
 *   then frame bytes.
 *
 * One frame in flight per direction per link. A FIRST chunk aborts any
 * partial reassembly (the sender restarted); a seq gap or overrun kills the
 * partial rather than delivering a corrupt frame. The frame bytes themselves
 * are channel- or PKC-encrypted Meshtastic airframes — this layer carries
 * water, it does not authenticate the pipe.
 */
#define MESHTASTIC_BLE_PEER_CHUNK_FIRST    0x80U
#define MESHTASTIC_BLE_PEER_CHUNK_LAST     0x40U
#define MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK 0x3FU
#define MESHTASTIC_BLE_PEER_CHUNK_HDR_LEN  1U
/* Smallest chunk buffer that always makes progress: header + the FIRST
 * chunk's 2-byte length prefix + one frame byte. */
#define MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF  4U
/* The guaranteed ATT payload: MTU 23 minus the 3-byte ATT header. */
#define MESHTASTIC_BLE_PEER_CHUNK_MTU23    20U
/* Mirrors MESHTASTIC_PKT_MAX without importing it (this header stays free of
 * project and Zephyr includes); equality is BUILD_ASSERTed where both are
 * visible (meshtastic_ble_peer.c). */
#define MESHTASTIC_BLE_PEER_FRAME_MAX      255U

/* Sender side: an iterator over one frame. The frame pointer must stay valid
 * until the chunker is done (nothing is copied). */
struct meshtastic_ble_peer_chunker {
	const uint8_t *frame;
	uint16_t len;
	uint16_t off;      /* frame bytes emitted so far */
	uint8_t seq;       /* next chunk's seq (mod 64) */
	bool first_sent;
};

/* Returns 0, -EINVAL on an empty frame, -EMSGSIZE past FRAME_MAX. */
int meshtastic_ble_peer_chunker_start(struct meshtastic_ble_peer_chunker *ck,
				      const uint8_t *frame, size_t len);

/*
 * Emit the next chunk into out (out_size >= MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF,
 * typically the link's ATT payload). Returns the chunk length (> 0), 0 when
 * the frame has been fully emitted, -EINVAL on a too-small buffer.
 */
int meshtastic_ble_peer_chunker_next(struct meshtastic_ble_peer_chunker *ck,
				     uint8_t *out, size_t out_size);

/* Receive side: one in-flight frame per link direction, plus counters so the
 * shell can prove what the channel did rather than guess. */
struct meshtastic_ble_peer_reasm {
	bool active;
	uint8_t next_seq;
	uint16_t expect;   /* declared frame length */
	uint16_t got;      /* frame bytes received */
	uint8_t frame[MESHTASTIC_BLE_PEER_FRAME_MAX];
	uint32_t frames;   /* frames completed */
	uint32_t aborted;  /* partials killed (fresh FIRST, seq gap, overrun) */
	uint32_t rejected; /* chunks refused */
};

void meshtastic_ble_peer_reasm_reset(struct meshtastic_ble_peer_reasm *rs);

/*
 * Ingest one chunk. Returns 1 with the complete frame in rs->frame (length in
 * *frame_len) — consume it before the next ingest; 0 when the chunk was
 * accepted and more are needed; -EMSGSIZE on a declared length past FRAME_MAX
 * (refused, nothing in flight afterwards); -EBADMSG on anything malformed —
 * empty chunk, zero-length frame, a non-FIRST chunk with nothing in flight
 * (the FIRST was missed), a seq gap, an overrun, or a LAST that arrives
 * early. Any error kills a partial in flight; the next FIRST starts clean.
 */
int meshtastic_ble_peer_reasm_ingest(struct meshtastic_ble_peer_reasm *rs,
				     const uint8_t *chunk, size_t len, size_t *frame_len);

#endif /* MESHTASTIC_BLE_PEER_CODEC_H_ */
