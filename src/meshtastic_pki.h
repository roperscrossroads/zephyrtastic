/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * Meshtastic PKC (public-key crypto) for direct messages. X25519 (Curve25519)
 * ECDH → SHA-256 → AES-256-CCM, matching the reference firmware CryptoEngine so
 * DMs interoperate. The keypair is generated once and persisted in the
 * SecurityConfig (NVS storage_partition, survives reboot + reflash); a changed
 * key would make peers throw key-mismatch warnings, so generation is
 * strictly once-if-absent and the key is flushed to NVS before it is advertised.
 */

#ifndef MESHTASTIC_PKI_H_
#define MESHTASTIC_PKI_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/** X25519 public/private key length. */
#define MESHTASTIC_PKI_KEY_LEN 32
/** Bytes added on the wire by PKC: 8-byte CCM tag + 4-byte extra nonce. */
#define MESHTASTIC_PKI_OVERHEAD 12

/**
 * Load our X25519 keypair from the SecurityConfig, or generate + persist one if
 * absent. Call once at boot AFTER settings are loaded. Non-fatal on failure
 * (the node still works with PSK channels); returns 0 when a key is ready.
 */
int meshtastic_pki_init(void);

/** True once a keypair is loaded (and, if generated, persisted) — safe to
 *  advertise the public key and to do PKC. */
bool meshtastic_pki_have_key(void);

/** Copy our 32-byte X25519 public key into @p out. Returns 32, or 0 if none. */
size_t meshtastic_pki_get_public_key(uint8_t out[MESHTASTIC_PKI_KEY_LEN]);

/**
 * PKC-decrypt a direct message addressed to us. @p enc / @p enc_len is the
 * on-wire PKC payload (ciphertext || tag[8] || extraNonce[4]). The sender's
 * public key is looked up in the NodeDB by @p from. Writes plaintext to @p out
 * (capacity @p out_cap), sets @p out_len. Returns 0 on authenticated success.
 */
int meshtastic_pki_decrypt(uint32_t from, uint32_t id, const uint8_t *enc, size_t enc_len,
			   uint8_t *out, size_t out_cap, size_t *out_len);

/**
 * PKC-encrypt @p plain (a serialized Data protobuf) for node @p to (public key
 * from the NodeDB). Writes ciphertext || tag[8] || extraNonce[4] to @p out;
 * @p out_len = @p plain_len + 12. Returns 0 on success, -ENOKEY if we have no
 * public key for @p to. (TX path — wired in Stage 3.)
 */
int meshtastic_pki_encrypt(uint32_t to, uint32_t from, uint32_t id, const uint8_t *plain,
			   size_t plain_len, uint8_t *out, size_t out_cap, size_t *out_len);

#endif /* MESHTASTIC_PKI_H_ */
