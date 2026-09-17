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

#endif /* MESHTASTIC_XEDDSA_H_ */
