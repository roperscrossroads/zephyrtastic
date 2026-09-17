/* SPDX-License-Identifier: GPL-3.0 */

/* XEdDSA signature verification (CONFIG_MESHTASTIC_XEDDSA).
 *
 * Meshtastic signs some broadcasts with XEdDSA: the signature is made with the node's
 * X25519 *identity* key, so a verifier needs no second key and no key exchange -- it
 * converts the sender's X25519 public key to the equivalent Ed25519 public key and runs a
 * standard Ed25519 verify. That conversion is the whole reason a plain EdDSA API is not
 * enough (see meshtastic_xeddsa.c).
 *
 * This port VERIFIES ONLY; it never signs (agents-ooma.5). A node that cannot sign is
 * still a good citizen on a signing mesh: upstream's default policy accepts unsigned
 * traffic.
 *
 * Reference: firmware/src/mesh/CryptoEngine.cpp (xeddsa_verify, curve_to_ed_pub).
 */

#ifndef MESHTASTIC_XEDDSA_H_
#define MESHTASTIC_XEDDSA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"

#define MESHTASTIC_XEDDSA_SIGNATURE_LEN 64U
#define MESHTASTIC_XEDDSA_KEY_LEN       32U

/**
 * Convert an X25519 (Montgomery u) public key to its Ed25519 (Edwards y) form.
 *
 * @param curve_pub 32-byte X25519 public key.
 * @param ed_pub    32-byte Ed25519 public key out.
 */
void meshtastic_xeddsa_curve_to_ed_pub(const uint8_t curve_pub[MESHTASTIC_XEDDSA_KEY_LEN],
				       uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN]);

/* Upstream signs a 12-byte header plus the payload: fromNode | packetId | portnum, each a
 * little-endian uint32, then the payload bytes. The ceiling is upstream's MAX_BLOCKSIZE. */
#define MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN 12U
#define MESHTASTIC_XEDDSA_SIGBUF_MAX        256U

/**
 * Build the byte string upstream signs, into @p buf.
 *
 * Reference: firmware CryptoEngine.cpp buildSigningBuffer. Getting this wrong is the
 * silent failure mode -- the crypto is fine and every signature still fails -- so it is
 * pinned by harvested vectors, not by inspection.
 *
 * @return the length written, or 0 if it would not fit.
 */
size_t meshtastic_xeddsa_build_signing_buffer(uint8_t *buf, size_t buf_size, uint32_t from_node,
					      uint32_t packet_id, uint32_t portnum,
					      const uint8_t *payload, size_t payload_len);

/**
 * Verify an XEdDSA signature over @p msg made by the holder of @p curve_pub.
 *
 * @return true only on a good signature. Any bad input -- wrong length, a public key that is
 *         not a valid point, a malformed signature -- is false, never an error to ignore.
 */
bool meshtastic_xeddsa_verify(const uint8_t curve_pub[MESHTASTIC_XEDDSA_KEY_LEN],
			      const uint8_t *msg, size_t msg_len,
			      const uint8_t sig[MESHTASTIC_XEDDSA_SIGNATURE_LEN]);

/**
 * Receive-side signature policy gate.
 *
 * Runs after decode and BEFORE delivery or relay, like the reference's
 * checkXeddsaReceivePolicy. Drops a packet whose signature is present and bad, or malformed,
 * or (under STRICT) absent -- so a forged broadcast never reaches a module, the phone, or
 * the rest of the mesh through us.
 *
 * @param pkt   the decoded packet (from / id / portnum / payload are what get signed).
 * @param mesh  the decoded MeshPacket, which carries the signature bytes and the
 *              xeddsa_signed flag; may be NULL, which means "no signature present".
 *
 * @return true to accept the packet, false to drop it.
 */
bool meshtastic_xeddsa_check_rx_policy(const struct meshtastic_packet *pkt,
				       meshtastic_MeshPacket *mesh);

#if defined(CONFIG_MESHTASTIC_XEDDSA_SIGN)
/**
 * Derive the Ed25519 signing key pair from this node's X25519 identity key.
 *
 * XEdDSA's convention: the scalar is the clamped X25519 private key, negated when that
 * would otherwise give a public key with its sign bit set -- which is what lets a verifier
 * recover the public key from the X25519 one by clearing that bit.
 */
void meshtastic_xeddsa_derive_ed_keys(const uint8_t x_priv[MESHTASTIC_XEDDSA_KEY_LEN],
				      uint8_t ed_priv[MESHTASTIC_XEDDSA_KEY_LEN],
				      uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN]);

/**
 * Sign @p msg with this node's X25519 identity key.
 *
 * @param z 32 bytes of randomness mixed into the nonce ("hedged" signing). The nonce is
 *          safe without it -- it already derives from the key and the message -- so a weak
 *          @p z degrades defence in depth, never correctness. Pass the same bytes as the
 *          reference to reproduce its signature exactly, which is how the tests pin this.
 *
 * @return true on success; false if the key is unusable.
 */
bool meshtastic_xeddsa_sign(const uint8_t x_priv[MESHTASTIC_XEDDSA_KEY_LEN], const uint8_t *msg,
			    size_t msg_len, const uint8_t z[MESHTASTIC_XEDDSA_KEY_LEN],
			    uint8_t sig[MESHTASTIC_XEDDSA_SIGNATURE_LEN]);
#endif /* CONFIG_MESHTASTIC_XEDDSA_SIGN */

#endif /* MESHTASTIC_XEDDSA_H_ */
