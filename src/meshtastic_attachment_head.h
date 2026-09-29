/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_ATTACHMENT_HEAD_H_
#define MESHTASTIC_ATTACHMENT_HEAD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_attach_bearer.h"

/*
 * A keyless radio head (ATTACHMENT-DESIGN S5, D1): one radio, no identity of
 * its own on the air. Every frame it hears goes, undecoded, to its brain as an
 * RX_FRAME envelope with the signal and preset it was heard on; the brain
 * decodes it with the brain's keys and treats it as heard by the brain's own
 * radio. The head accepts one control from its brain, SET_PRESET (TX_FRAME is
 * phase 3), and reports STATUS on a timer and after every retune.
 *
 * The image holds no channel keys, no PKI, no NodeDB: the build asserts it
 * (cmake/keyless-assert.cmake). What the head persists is exactly one record,
 * mtattach/brain -- the lease (S6): moving it is rewriting that record.
 *
 * Nothing here holds BLE types: the send seam is a __weak function the BLE
 * module fulfils and a test overrides, as the relay ear's is.
 */

struct meshtastic_attachment_head_stats {
	uint32_t heard;        /* frames the radio delivered */
	uint32_t forwarded;    /* ...sent to the brain as RX_FRAME */
	uint32_t no_brain;     /* heard with no brain configured */
	uint32_t queue_full;
	uint32_t send_failed;  /* the peer link refused */
	uint32_t status_sent;
	uint32_t controls;     /* SET_PRESET accepted */
	uint32_t refused;      /* a control from a node that is not the brain, or TX (phase 3) */
	uint32_t untrusted;    /* a control from the brain over a link the bearer does not vouch for */
	uint32_t rejected;     /* envelopes that failed to decode, or types a head does not take */
	uint32_t tx_sent;      /* TX_FRAMEs from the brain that went on the air */
	uint32_t tx_failed;    /* ...that the radio refused for good */
	uint32_t tx_deferred;  /* radio said DEFER; re-queued */
	uint32_t tx_queue_full;
	uint32_t tx_cancelled;   /* a relay withdrawn: by the brain, or by a copy we heard first */
	uint32_t tx_late;        /* a relay pushed to the end of the window by a copy we heard */
	uint32_t tx_late_dropped; /* a CLIENT relay whose window had closed when it reached us (D3) */
	uint32_t tx_duty_blocked; /* refused by THIS radio's regulatory duty gate */
};

/* The radio's frame, from the RX thread: queued, forwarded from a work queue
 * (a GATT write can block). */
void meshtastic_attachment_head_on_rx(const uint8_t *wire, uint16_t len, int16_t rssi, int8_t snr,
				      uint8_t preset, uint32_t rx_ms);

/* An envelope from the link whose peer identity is @p node. Returns 0,
 * -EPERM when @p node is not the brain (or the control is not yet supported),
 * -EBADMSG for anything a head does not take. */
int meshtastic_attachment_head_on_envelope(uint32_t node, const uint8_t *env, size_t len);
/* The same from a bearer's RX path (@p b answers whether the link is trusted;
 * NULL for the test seam). */
int meshtastic_attachment_head_on_envelope_from(const struct meshtastic_attach_bearer *b,
						uint32_t node, const uint8_t *env, size_t len);

/* The bearer's link to @p peer came up: if it is the brain, a STATUS goes out
 * at once (the boot-time one is lost before any link exists). */
void meshtastic_attachment_head_link_up(uint32_t peer);

/* The brain's link identity (its node number); 0 = none, nothing forwarded.
 * Saved with MESHTASTIC_SETTINGS (mtattach/brain). */
void meshtastic_attachment_head_set_brain(uint32_t node);
uint32_t meshtastic_attachment_head_get_brain(void);

/* Send a STATUS now (the timer does it every
 * MESHTASTIC_ATTACHMENT_HEAD_STATUS_PERIOD_SEC). -EHOSTUNREACH without a brain. */
int meshtastic_attachment_head_status_send(void);

void meshtastic_attachment_head_stats_get(struct meshtastic_attachment_head_stats *out);

/* Boot state: brain per Kconfig, counters zero, the saved brain forgotten. */
void meshtastic_attachment_head_reset(void);

/* Arm the STATUS timer. Called once from meshtastic_init(). */
void meshtastic_attachment_head_start(void);

/* The send seam: an envelope to the brain, as frame kind ATTACH. The default
 * goes over the BLE peer link; a test overrides it. */
int meshtastic_attachment_head_send(uint32_t brain, const uint8_t *env, size_t len);

#endif /* MESHTASTIC_ATTACHMENT_HEAD_H_ */
