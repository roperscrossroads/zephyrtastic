/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_ATTACHMENT_H_
#define MESHTASTIC_ATTACHMENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_attach_bearer.h"
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
	const struct meshtastic_attach_bearer *bearer; /* the link it speaks over; NULL = the test seam */
	bool link_up;        /* the bearer reported the link up at the last envelope */
	int64_t down_ms;     /* k_uptime when the bearer reported it down (0 = never) */
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
	/* Link latency, measured by the router when our own radio heard the same
	 * frame first (ATTACHMENT-SCOPE F5 / R2): the gap between the local copy
	 * and this head's copy. Only accrues while the brain's radio shares the
	 * head's preset. */
	uint32_t lat_n;
	uint32_t lat_min_ms;
	uint32_t lat_max_ms;
	uint32_t lat_sum_ms;
	/* TX through this head (P3 slice 1): the sequence stamped on each
	 * TX_FRAME, and what the last TX_RESULT said. */
	uint16_t tx_seq;
	int8_t last_tx_rc;
	uint8_t last_tx_defers;
	uint32_t tx_failed;   /* TX_RESULTs with rc != 0 */
};

/* Can a frame be handed to attachment @p id right now: known, link up, and
 * not reporting itself receive-only. id 0 (the local radio) is never asked
 * here. */
bool meshtastic_attachment_tx_ready(uint8_t id);
/* Hand @p wire to head @p id as a TX_FRAME on its preset. Called from the
 * outbound worker. @p want_result asks the head for a TX_RESULT. */
int meshtastic_attachment_tx(uint8_t id, const uint8_t *wire, size_t len, bool want_result);

/* The router noted that attachment @p id delivered a copy of a frame the local
 * radio had delivered @p ms earlier. */
void meshtastic_attachment_note_latency(uint8_t id, uint32_t ms);

/* Counters that are not per head. */
struct meshtastic_attachment_stats {
	uint32_t admission_refused; /* envelopes from a link that is neither trusted nor allowed */
	uint32_t not_admitted_malformed; /* unknown peers whose first envelope did not decode */
	uint32_t evicted;           /* slots freed after the link-down grace */
};
void meshtastic_attachment_stats_get(struct meshtastic_attachment_stats *out);

/* The operator's allow-list (ATTACHMENT-SCOPE C1): a head admitted although
 * its link is not trusted (an unbonded BLE link, a wire the Kconfig does not
 * vouch for). Saved with MESHTASTIC_SETTINGS (mtattach/allow). */
int meshtastic_attachment_allow_add(uint32_t node);
void meshtastic_attachment_allow_clear(void);
bool meshtastic_attachment_allow_get(unsigned int i, uint32_t *node);
bool meshtastic_attachment_is_allowed(uint32_t node);

/* Arm the eviction sweep. Called once from meshtastic_init(). */
void meshtastic_attachment_start(void);

/* An envelope arrived from the link whose peer identity is @p node (the beat's
 * node number). An unknown head is admitted into a free slot only after its
 * envelope decodes, and only if the bearer reports the link trusted
 * (ENCRYPTED or PHYSICAL) or the node is on the allow-list (ATTACHMENT-SCOPE
 * C1/C3): the metadata an envelope carries -- bearer, preset, signal, time --
 * is trusted by the router, so its source must be. Runs on the BT RX thread
 * with the peer lock held: it only parses, updates the table and does a
 * non-blocking queue put. Returns 0, -EBADMSG on a malformed envelope, -EACCES
 * when an unknown peer is neither trusted nor allowed, -ENOSPC when the table
 * is full, -ENOBUFS when the RX queue refused the frame. */
int meshtastic_attachment_ingest(uint32_t node, const uint8_t *env, size_t len);
/* The same, from a bearer's RX path: @p b is the bearer the envelope arrived
 * over (its link_info answers the admission gate), NULL for the test seam. */
int meshtastic_attachment_ingest_from(const struct meshtastic_attach_bearer *b, uint32_t node,
				      const uint8_t *env, size_t len);
/* A bearer's link to a head went down. The head stays in the table, marked
 * down, until the eviction grace passes or it speaks again. */
void meshtastic_attachment_link_down(const struct meshtastic_attach_bearer *b, uint32_t node);

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
