/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Cross-preset text relay, the receiving half (agents-jbrq.12). See
 * Kconfig.relay for what it does and the tooling repo's docs/RELAY-TESTBED.md
 * for how it is tested.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_RELAY_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_RELAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Which way text crosses. INBOUND is ear -> this node's preset. OUTBOUND
 * (this preset -> the ear's) needs an ear that transmits, which v1's ear does
 * not, so it is refused for now. */
enum meshtastic_relay_dir {
	MESHTASTIC_RELAY_OFF = 0,
	MESHTASTIC_RELAY_INBOUND = 1,
	MESHTASTIC_RELAY_OUTBOUND = 2,
	MESHTASTIC_RELAY_BOTH = 3,
};

struct meshtastic_relay_stats {
	uint32_t considered;    /* decoded frames that arrived from the ear */
	uint32_t relayed;       /* texts queued for re-origination */
	uint32_t sent;          /* ...and actually handed to the radio */
	uint32_t tx_failed;     /* send refused (queue full, no airtime) */
	uint32_t dir_off;       /* direction does not include inbound */
	uint32_t not_broadcast; /* DMs (and PKC) never cross */
	uint32_t not_text;      /* only TEXT_MESSAGE_APP crosses */
	uint32_t ignored;       /* our own, or on the relay-id list */
	uint32_t bad_text;      /* empty or not valid UTF-8 */
	uint32_t prefixed;      /* already carries a relay-style prefix */
	uint32_t no_mapping;    /* no equivalent channel on this preset */
	uint32_t seen;          /* same (origin, text) inside the TTL */
	uint32_t rate_dropped;  /* over the per-direction cap */
	uint32_t queue_full;    /* the send queue had no room */
	uint32_t unprefixed;    /* too long for the prefix, sent without it */
};

int meshtastic_relay_set_direction(enum meshtastic_relay_dir dir);
enum meshtastic_relay_dir meshtastic_relay_get_direction(void);

/* Never translate text from this node (another relay's destination half). */
int meshtastic_relay_ignore_add(uint32_t node_id);
void meshtastic_relay_ignore_clear(void);

void meshtastic_relay_stats_get(struct meshtastic_relay_stats *out);

/* Back to boot state: direction per Kconfig, caches, counters and the
 * ignore list cleared, and the saved settings forgotten. */
void meshtastic_relay_reset(void);

/* True if @p text starts with a relay-style prefix: '[', 1..8 printable
 * non-space ASCII characters, ']', ' '. Any relay's, not only ours. */
bool meshtastic_relay_has_prefix(const uint8_t *text, size_t len);

/* The router's hook: every decoded frame, with the bearer it arrived on. */
void meshtastic_relay_on_rx(const struct meshtastic_packet *pkt, enum meshtastic_bearer bearer);

/* ---- the ear (MESHTASTIC_RELAY_EAR) ---------------------------------------- */

struct meshtastic_relay_ear_stats {
	uint32_t heard;         /* decoded broadcast frames heard on LoRa */
	uint32_t forwarded;     /* handed to the peer link */
	uint32_t not_text;      /* only text can cross, so only text is forwarded */
	uint32_t not_broadcast; /* DMs never cross */
	uint32_t no_peer;       /* no receiving half configured */
	uint32_t queue_full;
	uint32_t send_failed;   /* the peer link refused (no live link, GATT error) */
};

/* The receiving half's node id; 0 stops forwarding. Saved with
 * MESHTASTIC_SETTINGS; reset() returns to the Kconfig value and forgets it. */
void meshtastic_relay_ear_set_peer(uint32_t node_id);
uint32_t meshtastic_relay_ear_get_peer(void);
void meshtastic_relay_ear_stats_get(struct meshtastic_relay_ear_stats *out);
void meshtastic_relay_ear_reset(void);

/* The router's hook: a decoded frame heard on LoRa, with its wire bytes. */
void meshtastic_relay_ear_on_rx(const struct meshtastic_packet *pkt, const uint8_t *wire,
				size_t wire_len);

/* The send seam. The default forwards over the BLE peer link
 * (meshtastic_ble_peer_frame_send_to); a test overrides it. */
int meshtastic_relay_ear_send(uint32_t peer, const uint8_t *wire, size_t wire_len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_RELAY_H_ */
