/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_ATTACHMENT_H_
#define MESHTASTIC_ATTACHMENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_attachment_codec.h"

/*
 * The brain's side of the attachment architecture (ATTACHMENT-DESIGN S6, D1):
 * a table of the radios this node hears through -- its own SX1262 as attachment
 * 0, and every radio head that has spoken to it over the peer link -- and the
 * ingest that turns a head's RX_FRAME envelope into an ordinary RF frame on the
 * router's queue, tagged with the head's preset and signal.
 *
 * Nothing here holds BLE types: the send seam is a __weak function the BLE
 * module fulfils and a test overrides, exactly like the relay ear's.
 */

struct meshtastic_attachment_info {
	uint8_t id;          /* 0 = this board's radio */
	uint32_t node;       /* the head's link identity (beat node number); 0 for id 0 */
	uint8_t preset;      /* meshtastic_Config_LoRaConfig_ModemPreset it reports */
	bool have_status;
	struct meshtastic_attachment_status status; /* last STATUS, when have_status */
	int64_t last_ms;     /* k_uptime at the last envelope from it */
	uint32_t rx_frames;  /* RX_FRAMEs ingested */
	uint32_t rx_dropped; /* RX_FRAMEs the RX queue refused */
	uint32_t tx_frames;  /* TX_FRAMEs sent to it */
	uint32_t tx_results; /* TX_RESULTs received */
	uint32_t rejected;   /* envelopes that failed to decode */
	int16_t last_rssi;
	int8_t last_snr;
	int16_t rssi_min;
	int16_t rssi_max;
};

/* An envelope arrived from the link whose peer identity is @p node (the beat's
 * node number). Admits an unknown head into a free slot. Runs on the BT RX
 * thread with the peer lock held: it only parses, updates the table and does a
 * non-blocking queue put. Returns 0, -EBADMSG on a malformed envelope, -ENOSPC
 * when the table is full, -ENOBUFS when the RX queue refused the frame. */
int meshtastic_attachment_ingest(uint32_t node, const uint8_t *env, size_t len);

/* The table: attachment 0 is always present (the local radio). */
unsigned int meshtastic_attachment_count(void);
bool meshtastic_attachment_get(uint8_t id, struct meshtastic_attachment_info *out);
/* The attachment id a head with this link identity has, or 0 if none. */
uint8_t meshtastic_attachment_id_for_node(uint32_t node);
/* Forget a head (its link went down for good, or the operator says so). */
int meshtastic_attachment_forget(uint8_t id);

/* Ask a head to retune. Sends SET_PRESET; the head answers with STATUS.
 * Returns the send result; -ENOENT for an unknown id, -EINVAL for id 0. */
int meshtastic_attachment_set_preset(uint8_t id, uint8_t preset);

/* The send seam: deliver an envelope to the head with this link identity. The
 * default sends it as frame kind ATTACH over the BLE peer link, or -ENOTSUP
 * without one; a test overrides it. */
int meshtastic_attachment_send(uint32_t node, const uint8_t *env, size_t len);

#endif /* MESHTASTIC_ATTACHMENT_H_ */
