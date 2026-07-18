/* SPDX-License-Identifier: GPL-3.0
 *
 * Admin session-passkey engine — replay protection for AdminMessage writes.
 *
 * The node issues an 8-byte key in every getter/response AdminMessage; a remote
 * client MUST echo that key back with any mutating admin op. Mirrors the
 * reference firmware AdminModule: the key is regenerated once it is older than
 * 150 s and accepted for 300 s, so the two windows overlap and a client that
 * read config within the last 150 s always holds a still-valid key.
 *
 * Local (from == self) admin never needs a passkey; validation is for the
 * remote/mesh admin path (see meshtastic_admin.c).
 */
#ifndef MESHTASTIC_ADMIN_SESSION_H_
#define MESHTASTIC_ADMIN_SESSION_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/** Admin session passkey length (AdminMessage.session_passkey max_size). */
#define MESHTASTIC_ADMIN_SESSION_KEY_LEN 8

/**
 * Copy the current session passkey into @p out, generating a fresh one first if
 * none exists yet or the current one is older than the 150 s rotation window.
 * Call when emitting an admin response so the client receives a live key.
 *
 * @param out Buffer of at least @ref MESHTASTIC_ADMIN_SESSION_KEY_LEN bytes.
 */
void meshtastic_admin_session_current(uint8_t out[MESHTASTIC_ADMIN_SESSION_KEY_LEN]);

/**
 * Validate a client-supplied session passkey.
 *
 * @param key Candidate key bytes.
 * @param len Candidate key length.
 * @return true iff @p key is exactly @ref MESHTASTIC_ADMIN_SESSION_KEY_LEN bytes,
 *         matches the current key, and that key was issued within the 300 s
 *         validity window. A wrong length, mismatch, expired key, or no-key-yet
 *         all return false.
 */
bool meshtastic_admin_session_valid(const uint8_t *key, size_t len);

/** Forget the current session passkey (any subsequent validate fails until a
 *  new key is issued). Available for a full logout/reset; not tied to the phone
 *  connect/disconnect path, which must not invalidate a remote admin's key. */
void meshtastic_admin_session_reset(void);

#endif /* MESHTASTIC_ADMIN_SESSION_H_ */
