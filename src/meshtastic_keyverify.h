/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_KEYVERIFY_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_KEYVERIFY_H_

/*
 * Manual key verification (agents-dnr4.13) -- the reference's
 * KeyVerificationModule, rule for rule. The handshake (A = initiator, B =
 * responder; every message is a KeyVerification protobuf on port 12):
 *
 *   M1  A -> B  nonce, hash1 = PK_A. want_response. PKC if A already holds
 *               PK_B, else channel-encrypted (bootstrap: B learns PK_A here).
 *   M2  B -> A  nonce, hash1 = PK_B, hash2 = SHA256(nonce || H1), where
 *               H1 = SHA256(number || nonce || A || B || PK_A || PK_B) and
 *               `number` is a fresh 6-digit security number B SHOWS its user.
 *               PKC only if B already held PK_A; else channel-encrypted so A can
 *               read PK_B. Cooldown: B opens at most one remote session per
 *               minute.
 *   (user)      A's user types the number B displayed. A recomputes H1 and
 *               checks SHA256(nonce || H1) == hash2 -- a wrong number goes
 *               nowhere on the air.
 *   M3  A -> B  nonce, hash1 = H1. MUST be PKC (proves A holds the private key
 *               for PK_A). B checks H1 against its own.
 *   (user)      Both sides show an 8-character code derived from H1; the user
 *               confirms they match. DO_VERIFY commits the peer's key to the
 *               NodeDB with the "manually verified" flag; DO_NOT_VERIFY, a
 *               60 s idle timeout or the 3-minute session cap discards
 *               everything learned, including a pending key.
 *
 * Integers are hashed little-endian, as the reference hashes their raw bytes on
 * its (all little-endian) targets, so the two firmwares agree on H1.
 *
 * Where this port differs: no screen prompts (the app gets the same
 * ClientNotification the reference sends; the bench has the shell). Sending a
 * bootstrap message channel-encrypted needs meshtastic_packet.no_pkc, the one
 * sanctioned exception to "never downgrade a DM" -- the payload IS the key the
 * peer is about to learn.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum meshtastic_keyverify_state {
	MESHTASTIC_KEYVERIFY_IDLE = 0,
	MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED,
	MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER,
	MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER,
	MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER,
	MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1,
};

struct meshtastic_keyverify_status {
	enum meshtastic_keyverify_state state;
	uint32_t remote_node;
	uint64_t nonce;
	/* The security number: the one WE generated (responder) to show the
	 * user, or the one the user gave us (initiator, after it checked out). */
	uint32_t security_number;
	/* "XXXX XXXX" once H1 is known on this side (else ""). */
	char code[10];
};

int meshtastic_keyverify_init(void);

/** @brief Start verifying @p remote_node (M1). -EBUSY while a session is open. */
int meshtastic_keyverify_start(uint32_t remote_node);

/**
 * @brief The user typed the number the peer displayed (initiator side).
 *        -EINVAL: wrong state or nonce; -EACCES: the number does not
 *        reproduce the peer's hash2 (wrong number, or a MitM); 0: M3 sent.
 */
int meshtastic_keyverify_provide_number(uint64_t nonce, uint32_t number);

/** @brief The user confirmed the codes match: commit the key as verified. */
int meshtastic_keyverify_accept(uint64_t nonce);

/** @brief The user rejected, or gave up: discard everything learned. */
void meshtastic_keyverify_reject(void);

void meshtastic_keyverify_status(struct meshtastic_keyverify_status *out);

/** @brief Drop any session and the remote cooldown (tests). */
void meshtastic_keyverify_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_KEYVERIFY_H_ */
