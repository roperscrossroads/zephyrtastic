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
	uint32_t tx_failed;    /* TX_RESULTs that are a real failure */
	uint32_t tx_cancelled; /* relays withdrawn on a heard copy or a CANCEL */
	uint32_t tx_late;      /* relays dropped as stale on arrival (-ETIME) */
	/* DESIGN §13: the peer link's cost per relay, as the head reports it in
	 * TX_RESULT (arrival of the decision minus its reception of the frame, on
	 * its clock). A 16-sample ring; p50/p90/max recomputed per sample. */
	uint16_t lag[16];
	uint8_t lag_n;
	uint8_t lag_next;
	uint16_t lag_p50_ms;
	uint16_t lag_p90_ms;
	uint16_t lag_max_ms;
	/* The placement guard (§13): FAST-HEAD -- the head's slot time is shorter
	 * than our own radio's (a fast preset belongs on the brain); LAG -- the
	 * lag p90 exceeds half the head preset's ROUTER window while our role
	 * relays early (ROUTER / ROUTER_LATE). Each warns once per admission. */
	/* k_uptime when SET_PRESET last went out because the head was not on the preset
	 * wanted of it (0 = never): the retry is paced on this. */
	int64_t want_asked_ms;
	bool want_due;
	bool warn_fast_head;
	bool warn_lag;
	bool warned_fast_head;
	bool warned_lag;
};

/* Can a frame be handed to attachment @p id right now: known, link up, and
 * not reporting itself receive-only. id 0 (the local radio) is never asked
 * here. */
bool meshtastic_attachment_tx_ready(uint8_t id);
/* Hand @p wire to head @p id as a TX_FRAME on its preset. Called from the
 * outbound worker. @p flags are MESHTASTIC_ATTACHMENT_TXF_*. */
int meshtastic_attachment_tx(uint8_t id, const uint8_t *wire, size_t len, uint8_t flags);
/* Hand a RELAY to head @p id (ATTACHMENT-DESIGN §12): @p wire is the relay as
 * the brain built it (hop decremented, our relay byte); the head keys up no
 * earlier than @p not_before_ms after ITS reception of the original (@p rx_ms,
 * the head's stamp on that RX_FRAME) and applies @p dupe if it hears the
 * frame again first. */
int meshtastic_attachment_relay(uint8_t id, const uint8_t *wire, size_t len, uint32_t src,
				uint32_t pkt_id, uint32_t rx_ms, uint32_t not_before_ms,
				uint8_t dupe);
/* Configure head @p id's radio: its transmit power in dBm at the antenna (0 =
 * the region's maximum). A head runs no phone service, so its brain is the
 * only thing that can set this. The head answers with STATUS. */
int meshtastic_attachment_set_tx_power(uint8_t id, int8_t dbm);
/* Withdraw a relay handed to head @p id (a duplicate reached the brain first). */
int meshtastic_attachment_cancel(uint8_t id, uint32_t src, uint32_t pkt_id);
/* The modem the head's preset implies (for the relay window). false = unknown. */
bool meshtastic_attachment_modem(uint8_t id, uint8_t *spread_factor, uint32_t *bandwidth_hz);
/* The channel hash a frame leaving by attachment @p id must carry for slot
 * @p index: an unnamed slot hashes under THAT radio's preset (S2). Falls back
 * to the local hash when the head's preset is unknown. */
uint8_t meshtastic_attachment_tx_hash(uint8_t id, uint8_t index);

/**
 * @brief The radio a packet ORIGINATED on channel @p index leaves by.
 *
 * A slot named after a preset this brain covers through a head is that
 * preset's channel (meshtastic_channels_named_for_preset): what is sent on it
 * leaves through a head on that preset, the most recently heard one that can
 * transmit. Every other channel is the brain's own radio, as before.
 *
 * @return 0 for our own radio, an attachment id, or -ENETUNREACH when the
 *         channel names a preset no ready head is on: never the wrong radio.
 */
int meshtastic_attachment_for_channel(uint8_t index);

/**
 * @brief A head that can transmit on @p preset: the most recently heard one.
 *
 * @return its attachment id, or 0 when there is none (or @p preset is ours:
 *         that is our own radio).
 */
uint8_t meshtastic_attachment_for_preset(uint8_t preset);

/**
 * @brief Send @p wire through one ready head on each preset other than ours.
 *
 * The other half of MESHTASTIC_ATTACH_ALL (the caller sends on our own radio):
 * the same packet, same id, on every other preset the node covers -- one head
 * per preset, the most recently heard. A frame stamped with slot @p index's
 * hash is re-stamped with that slot's hash under each head's preset (a default
 * channel hashes by preset; a channel with its own name does not change, and
 * neither does a PKC frame's 0).
 *
 * @return how many heads it was handed to.
 */
unsigned int meshtastic_attachment_fan_out(const uint8_t *wire, uint32_t len, uint8_t tier,
					   uint8_t index);

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

/* Put a head on a preset, and keep it there. The preset is stored against the head's link
 * identity (mtattach/want) and SET_PRESET is sent now; whenever that head later reports
 * another preset (it rebooted, it was reflashed, it is a replacement that kept the
 * identity) SET_PRESET is sent again. A head does not store its preset: the brain owns it.
 * MESHTASTIC_PRESET_UNKNOWN forgets the wish and sends nothing.
 * Returns the send result (the wish is stored even if the link is down); -ENOENT for an
 * unknown id, -EINVAL for id 0 or a number that is not a preset. */
int meshtastic_attachment_set_preset(uint8_t id, uint8_t preset);
/* Forget every stored wish (a factory path, and the tests' clean slate). */
void meshtastic_attachment_want_clear(void);
/* The preset wanted of the head with this link identity, or MESHTASTIC_PRESET_UNKNOWN. */
uint8_t meshtastic_attachment_wanted_preset(uint32_t node);

/* The send seam: deliver an envelope to the head with this link identity. The
 * default sends it as frame kind ATTACH over the BLE peer link, or -ENOTSUP
 * without one; a test overrides it. */
int meshtastic_attachment_send(uint32_t node, const uint8_t *env, size_t len);

#endif /* MESHTASTIC_ATTACHMENT_H_ */
