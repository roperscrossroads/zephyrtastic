/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_LINK_BEARER_H_
#define MESHTASTIC_LINK_BEARER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The board-to-board link behind the hub-worker architecture
 * (ATTACHMENT-SCOPE §4). Envelopes (meshtastic_link_codec.h) are bearer
 * agnostic; this is the small table that carries them over whichever link a
 * hub and its workers share -- the BLE peer link today, a UART or a test pipe
 * tomorrow -- and answers the two questions the hub's admission gate needs
 * (who is at the other end, can the link be trusted) and the one the timing
 * policy needs (how slow is it).
 *
 * A bearer adds no thread, queue or copy here: it owns its own work queue and
 * delivers into meshtastic_link_bearer_rx() from whatever context it has,
 * which must be one where a non-blocking queue put is allowed.
 */

enum meshtastic_link_auth {
	MESHTASTIC_LINK_AUTH_NONE = 0,  /* plain BLE, an unknown IP peer */
	MESHTASTIC_LINK_AUTH_PHYSICAL,  /* a wire trusted by construction (Kconfig) */
	MESHTASTIC_LINK_AUTH_ENCRYPTED, /* BLE bonded >= L2, or an envelope MAC */
};

struct meshtastic_link_info {
	enum meshtastic_link_auth auth;
	bool up;              /* a live link reaches this peer */
	bool takes_envelopes; /* the peer speaks envelopes (BLE: its beat's LINK flag) */
	uint16_t mtu;         /* per-write bytes; 0 = stream */
	uint16_t rtt_ms;      /* last measured, 0 = unknown */
};

struct meshtastic_link_bearer {
	const char *name; /* "ble", "pipe", "uart" */
	/* Deliver one envelope to the peer with this link identity. Must not block
	 * the caller indefinitely. Returns 0, -EHOSTUNREACH when no live link
	 * reaches the peer, else the link's error. */
	int (*send)(uint32_t peer, const uint8_t *env, size_t len);
	/* What the bearer knows about its link to @p peer. False when it has never
	 * seen that peer. */
	bool (*link_info)(uint32_t peer, struct meshtastic_link_info *out);
};

/* Bearers register once at init. Unregister exists for tests. */
int meshtastic_link_bearer_register(const struct meshtastic_link_bearer *b);
void meshtastic_link_bearer_unregister(const struct meshtastic_link_bearer *b);

/* Send over whichever registered bearer has a live link to @p peer. */
int meshtastic_link_bearer_send(uint32_t peer, const uint8_t *env, size_t len);

/* The link to @p peer, from the first bearer that has one up. @p which (may be
 * NULL) receives that bearer. False when no bearer knows the peer. */
bool meshtastic_link_bearer_link_info(uint32_t peer, struct meshtastic_link_info *out,
					const struct meshtastic_link_bearer **which);

/* A bearer's RX path: an envelope arrived from @p peer over @p b. Routed to the
 * hub's ingest or the worker's control handler, whichever this image is. */
int meshtastic_link_bearer_rx(const struct meshtastic_link_bearer *b, uint32_t peer,
				const uint8_t *env, size_t len);

/* A bearer's link to @p peer came up (envelopes can flow). A worker introduces
 * itself to its hub at once; a hub marks a known worker up. */
void meshtastic_link_bearer_link_up(const struct meshtastic_link_bearer *b, uint32_t peer);
/* A bearer's link to @p peer went down. */
void meshtastic_link_bearer_link_down(const struct meshtastic_link_bearer *b, uint32_t peer);

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
/* The BLE peer link as a bearer (meshtastic_link_bearer_ble.c). */
extern const struct meshtastic_link_bearer meshtastic_link_bearer_ble;
#endif

#endif /* MESHTASTIC_LINK_BEARER_H_ */
