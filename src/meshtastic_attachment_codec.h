/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_ATTACHMENT_CODEC_H_
#define MESHTASTIC_ATTACHMENT_CODEC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The attachment envelope (ATTACHMENT-DESIGN S4): what a radio head and its
 * brain say to each other. Pure C -- no kernel, no Bluetooth -- so every byte is
 * unit-testable on native_sim, and the same bytes ride any bearer: today the BLE
 * peer link's frame channel as frame kind MESHTASTIC_BLE_PEER_KIND_ATTACH, later
 * a UART behind COBS + CRC16.
 *
 * All multi-byte fields little-endian.
 *
 *   [0] type
 *   1 RX_FRAME   [1]preset [2..3]rssi i16 [4]snr i8 [5..8]rx_ms u32 [9]flags  + wire
 *   2 TX_FRAME   [1]preset [2]flags [3..4]tx_seq u16                         + wire
 *                with flag RELAY: [5..8]relay_src [9..12]relay_id [13..16]rx_ms
 *                (the head's, of the frame being relayed) [17..20]not_before_ms
 *                (relative to that rx_ms) [21]dupe (KEEP/CANCEL/LATE)  + wire
 *   3 TX_RESULT  [1..2]tx_seq [3]rc i8 [4]defers u8 [5..8]tx_ms u32
 *   4 STATUS     [1]preset [2]flags [3..6]hwid [7..10]brain [11..14]rx_frames
 *                [15..18]tx_frames [19..22]uptime_s [23..26]rx_dropped
 *                [27..30]lat [31..34]lon [35..38]alt   (the three only with HAS_POS)
 *   5 SET_PRESET [1]preset
 *   6 TIME_BEAT  reserved (C007)
 *   7 TX_CANCEL  [1..4]src [5..8]id   (the brain withdraws a relay it handed over)
 *
 * The wire in RX_FRAME/TX_FRAME is a Meshtastic airframe, encrypted, untouched:
 * a head never decodes it (it holds no keys) and never builds one. Who a frame
 * came from is the LINK's identity (the peer beat's node number), so RX_FRAME
 * carries no attachment id; a multiplexing bearer adds one in a later version.
 *
 * NOT AUTHENTICATION. RX_FRAME and STATUS carry nothing a stranger could not put
 * on the air anyway; TX_FRAME and SET_PRESET are CONTROLS and are gated by the
 * receiving head on the link (the configured brain, over a bonded connection),
 * not by anything in these bytes.
 */

/*
 * Compatibility rule (ATTACHMENT-SCOPE D5): there is no version byte. A sender
 * may APPEND fields to a fixed-layout type (STATUS, TX_RESULT, SET_PRESET); a
 * receiver checks that the prefix it knows is present (`len >=`) and ignores
 * the tail. Fields are never reordered or removed. The two types that carry a
 * wire frame (RX_FRAME, TX_FRAME) have a fixed header followed by the frame;
 * they grow through their flags byte, not by appending.
 */
#define MESHTASTIC_ATTACHMENT_WIRE_MAX 255U
#define MESHTASTIC_ATTACHMENT_HDR_MAX  16U
/* Mirrors MESHTASTIC_BLE_PEER_ENV_MAX (BUILD_ASSERTed where both are visible). */
#define MESHTASTIC_ATTACHMENT_ENV_MAX  (MESHTASTIC_ATTACHMENT_WIRE_MAX + MESHTASTIC_ATTACHMENT_HDR_MAX)

enum meshtastic_attachment_type {
	MESHTASTIC_ATTACHMENT_RX_FRAME = 1,
	MESHTASTIC_ATTACHMENT_TX_FRAME = 2,
	MESHTASTIC_ATTACHMENT_TX_RESULT = 3,
	MESHTASTIC_ATTACHMENT_STATUS = 4,
	MESHTASTIC_ATTACHMENT_SET_PRESET = 5,
	MESHTASTIC_ATTACHMENT_TIME_BEAT = 6,
	MESHTASTIC_ATTACHMENT_TX_CANCEL = 7,
};

#define MESHTASTIC_ATTACHMENT_RX_HDR_LEN     10U
#define MESHTASTIC_ATTACHMENT_TX_HDR_LEN     5U
#define MESHTASTIC_ATTACHMENT_TX_RELAY_HDR_LEN 22U
#define MESHTASTIC_ATTACHMENT_TX_CANCEL_LEN  9U
#define MESHTASTIC_ATTACHMENT_TX_RESULT_LEN  9U
#define MESHTASTIC_ATTACHMENT_STATUS_LEN     27U
#define MESHTASTIC_ATTACHMENT_STATUS_POS_LEN 39U
#define MESHTASTIC_ATTACHMENT_SET_PRESET_LEN 2U

/* TX_FRAME flags */
#define MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT 0x01U
/* An own frame (an origination or a reply): the head draws the reference's
 * own-TX contention delay with ITS modem and channel utilisation before it
 * keys up. Absent: a relay, timed by not_before (slice 3). */
#define MESHTASTIC_ATTACHMENT_TXF_OWN_DELAY   0x02U
/* A relay (ATTACHMENT-DESIGN §12): the header carries relay_of {src, id, the
 * head's rx_ms} and not_before_ms relative to that rx_ms -- the window runs
 * on the HEAD's clock of reception -- and what to do if the head hears the
 * same frame again before it keys up. */
#define MESHTASTIC_ATTACHMENT_TXF_RELAY       0x04U
/* TX_FRAME.dupe: the brain's relay_dupe_action for this frame. */
#define MESHTASTIC_ATTACHMENT_DUPE_KEEP   0U
#define MESHTASTIC_ATTACHMENT_DUPE_CANCEL 1U
#define MESHTASTIC_ATTACHMENT_DUPE_LATE   2U
/* STATUS flags */
#define MESHTASTIC_ATTACHMENT_ST_TX_ENABLED 0x01U
#define MESHTASTIC_ATTACHMENT_ST_RADIO_HELD 0x02U
#define MESHTASTIC_ATTACHMENT_ST_HAS_POS    0x04U
#define MESHTASTIC_ATTACHMENT_ST_RX_ONLY    0x08U /* TX compiled out of this image */
#define MESHTASTIC_ATTACHMENT_ST_IS_HEAD    0x10U /* a keyless head (not an ear) */

struct meshtastic_attachment_rx_frame {
	uint8_t preset;   /* meshtastic_Config_LoRaConfig_ModemPreset */
	int16_t rssi;
	int8_t snr;
	uint32_t rx_ms;   /* the head's uptime at reception */
	uint8_t flags;    /* reserved, 0 */
	const uint8_t *wire; /* points into the envelope on decode */
	uint16_t wire_len;
};

struct meshtastic_attachment_tx_frame {
	uint8_t preset;
	uint8_t flags;    /* MESHTASTIC_ATTACHMENT_TXF_* */
	uint16_t tx_seq;
	/* With TXF_RELAY only. */
	uint32_t relay_src;
	uint32_t relay_id;
	uint32_t rx_ms;         /* the head's uptime when it heard the original */
	uint32_t not_before_ms; /* key up no earlier than rx_ms + this, on the head's clock */
	uint8_t dupe;           /* MESHTASTIC_ATTACHMENT_DUPE_* */
	const uint8_t *wire;
	uint16_t wire_len;
};

struct meshtastic_attachment_tx_cancel {
	uint32_t src;
	uint32_t id;
};

struct meshtastic_attachment_tx_result {
	uint16_t tx_seq;
	int8_t rc;
	uint8_t defers;
	uint32_t tx_ms;
};

struct meshtastic_attachment_status {
	uint8_t preset;
	uint8_t flags;    /* MESHTASTIC_ATTACHMENT_ST_* */
	uint32_t hwid;    /* the head's own (hardware) node number */
	uint32_t brain;   /* the brain it is configured for, 0 = none */
	uint32_t rx_frames;
	uint32_t tx_frames;
	uint32_t uptime_s;
	uint32_t rx_dropped;
	int32_t lat;      /* 1e-7 deg, with HAS_POS */
	int32_t lon;
	int32_t alt;      /* m */
};

struct meshtastic_attachment_msg {
	uint8_t type; /* enum meshtastic_attachment_type */
	union {
		struct meshtastic_attachment_rx_frame rx;
		struct meshtastic_attachment_tx_frame tx;
		struct meshtastic_attachment_tx_result result;
		struct meshtastic_attachment_status status;
		struct meshtastic_attachment_tx_cancel cancel;
		uint8_t preset; /* SET_PRESET */
	} u;
};

/* Encoders return the envelope length (> 0) or -EMSGSIZE when @p out_size is too
 * small or the wire is over WIRE_MAX; -EINVAL on a NULL/empty wire where one is
 * required. */
int meshtastic_attachment_encode_rx_frame(const struct meshtastic_attachment_rx_frame *m,
					  uint8_t *out, size_t out_size);
int meshtastic_attachment_encode_tx_frame(const struct meshtastic_attachment_tx_frame *m,
					  uint8_t *out, size_t out_size);
int meshtastic_attachment_encode_tx_result(const struct meshtastic_attachment_tx_result *m,
					   uint8_t *out, size_t out_size);
int meshtastic_attachment_encode_status(const struct meshtastic_attachment_status *m,
					uint8_t *out, size_t out_size);
int meshtastic_attachment_encode_set_preset(uint8_t preset, uint8_t *out, size_t out_size);
int meshtastic_attachment_encode_tx_cancel(uint32_t src, uint32_t id, uint8_t *out,
					   size_t out_size);

/* Decode one envelope. Returns 0, -EINVAL on NULL/short input, -EBADMSG on an
 * unknown type or a length that does not fit the type. The wire pointers in
 * @p out point INTO @p env (nothing is copied). */
int meshtastic_attachment_decode(const uint8_t *env, size_t len,
				 struct meshtastic_attachment_msg *out);

#endif /* MESHTASTIC_ATTACHMENT_CODEC_H_ */
