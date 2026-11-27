/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_WORKER_H_
#define MESHTASTIC_WORKER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_link_bearer.h"

/*
 * A keyless worker (ATTACHMENT-DESIGN S5, D1): one radio, no identity of
 * its own on the air. Every frame it hears goes, undecoded, to its hub as an
 * RX_FRAME envelope with the signal and preset it was heard on; the hub
 * decodes it with the hub's keys and treats it as heard by the hub's own
 * radio. The worker accepts one control from its hub, SET_PRESET (TX_FRAME is
 * phase 3), and reports STATUS on a timer and after every retune.
 *
 * The image holds no channel keys, no PKI, no NodeDB: the build asserts it
 * (cmake/keyless-assert.cmake). What the worker persists is exactly one record,
 * mtattach/brain -- the lease (S6): moving it is rewriting that record.
 *
 * Nothing here holds BLE types: the send seam is a __weak function the BLE
 * module fulfils and a test overrides, as the rx-only worker's is.
 */

struct meshtastic_worker_stats {
	uint32_t heard;        /* frames the radio delivered */
	uint32_t forwarded;    /* ...sent to the hub as RX_FRAME */
	uint32_t no_hub;     /* heard with no hub configured */
	uint32_t queue_full;
	uint32_t send_failed;  /* the peer link refused */
	uint32_t status_sent;
	uint32_t controls;     /* SET_PRESET accepted */
	uint32_t refused;      /* a control from a node that is not the hub, or TX (phase 3) */
	uint32_t untrusted;    /* a control from the hub over a link the bearer does not vouch for */
	uint32_t rejected;     /* envelopes that failed to decode, or types a worker does not take */
	uint32_t tx_sent;      /* TX_FRAMEs from the hub that went on the air */
	uint32_t tx_failed;    /* ...that the radio refused for good */
	uint32_t tx_deferred;  /* radio said DEFER; re-queued */
	uint32_t tx_queue_full;
	uint32_t tx_cancelled;   /* a relay withdrawn: by the hub, or by a copy we heard first */
	uint32_t tx_late;        /* a relay pushed to the end of the window by a copy we heard */
	uint32_t tx_late_dropped; /* a CLIENT relay past orig_rx + 2*worst when it reached us: stale (D3) */
	uint32_t tx_duty_blocked; /* refused by THIS radio's regulatory duty gate */
};

/* The radio's frame, from the RX thread: queued, forwarded from a work queue
 * (a GATT write can block). */
void meshtastic_worker_on_rx(const uint8_t *wire, uint16_t len, int16_t rssi, int8_t snr,
				      uint8_t preset, uint32_t rx_ms);

/* An envelope from the link whose peer identity is @p node. Returns 0,
 * -EPERM when @p node is not the hub (or the control is not yet supported),
 * -EBADMSG for anything a worker does not take. */
int meshtastic_worker_on_envelope(uint32_t node, const uint8_t *env, size_t len);
/* The same from a bearer's RX path (@p b answers whether the link is trusted;
 * NULL for the test seam). */
int meshtastic_worker_on_envelope_from(const struct meshtastic_link_bearer *b,
						uint32_t node, const uint8_t *env, size_t len);

/* The bearer's link to @p peer came up: if it is the hub, a STATUS goes out
 * at once (the boot-time one is lost before any link exists). */
void meshtastic_worker_link_up(uint32_t peer);

/* The hub's link identity (its node number); 0 = none, nothing forwarded.
 * Saved with MESHTASTIC_SETTINGS (mtattach/brain). */
void meshtastic_worker_set_hub(uint32_t node);
uint32_t meshtastic_worker_get_hub(void);

/* Send a STATUS now (the timer does it every
 * MESHTASTIC_WORKER_STATUS_PERIOD_SEC). -EHOSTUNREACH without a hub. */
int meshtastic_worker_status_send(void);

void meshtastic_worker_stats_get(struct meshtastic_worker_stats *out);

/* Boot state: hub per Kconfig, counters zero, the saved hub forgotten. */
void meshtastic_worker_reset(void);

/* Arm the STATUS timer. Called once from meshtastic_init(). */
void meshtastic_worker_start(void);

/* The send seam: an envelope to the hub, as frame kind LINK. The default
 * goes over the BLE peer link; a test overrides it. */
int meshtastic_worker_send(uint32_t hub, const uint8_t *env, size_t len);

#endif /* MESHTASTIC_WORKER_H_ */
