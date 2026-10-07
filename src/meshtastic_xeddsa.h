/* SPDX-License-Identifier: GPL-3.0 */

/* XEdDSA packet signatures (CONFIG_MESHTASTIC_XEDDSA verifies, _SIGN signs too).
 *
 * Meshtastic signs some broadcasts with XEdDSA: the signature is made with the node's
 * X25519 *identity* key, so a verifier needs no second key and no key exchange -- it
 * converts the sender's X25519 public key to the equivalent Ed25519 public key and runs a
 * standard Ed25519 verify. That conversion is the whole reason a plain EdDSA API is not
 * enough (see meshtastic_xeddsa.c).
 *
 * Every build verifies. Signing is a separate option (about 38 KB of flash, which the XIAO
 * cannot spare), and a node that cannot sign is still a good citizen on a signing mesh:
 * upstream's default policy accepts unsigned traffic.
 *
 * Reference: firmware/src/mesh/CryptoEngine.cpp (buildSigningBuffer, xeddsa_sign,
 * xeddsa_verify, curve_to_ed_pub) at v2.8.1.
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

/* The signed bytes, since upstream 2.8.1 (#11422): one fixed 34-byte header, then the
 * payload --
 *
 *   version(1)=0x01 | from | id | to | portnum | request_id | reply_id | emoji | bitfield
 *                   | flags(1) | payload
 *
 * every integer a little-endian uint32. The whole Data envelope is covered, not only the
 * payload: channel crypto is AES-CTR with no MAC, so a field left out of the signature is
 * rewritable in flight by any PSK holder while the signature still verifies (reply_id would
 * re-point a signed reply, emoji turn it into a reaction, `to` deliver a signed broadcast as
 * an apparent DM, bitfield grant OK_TO_MQTT). `bitfield` signs as 0 when absent and its
 * presence rides in the flags byte, so stripping the field is not the same as sending it
 * empty. The header is fixed-length so the payload boundary never depends on content.
 *
 * Not covered, as upstream: dest and source (unused), channel (a hash on the wire, an index
 * once decoded) and the hop fields, which relays rewrite by design.
 *
 * 2.8.0 signed `from | id | portnum | payload` (12 bytes). There is no negotiation between
 * the layouts: a verifier that holds the sender's key and sees the other layout drops the
 * packet under every policy, in both directions. Upstream revoked 2.8.0 over this, so the
 * old layout is not kept here either. */
#define MESHTASTIC_XEDDSA_SIGNING_VERSION       0x01U
#define MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN     (1U + 8U * 4U + 1U)
#define MESHTASTIC_XEDDSA_SIGBUF_MAX            (MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN + \
						 MESHTASTIC_MAX_PAYLOAD_LEN)
#define MESHTASTIC_XEDDSA_SIGNED_FLAG_WANT_RESPONSE 0x01U
#define MESHTASTIC_XEDDSA_SIGNED_FLAG_HAS_BITFIELD  0x02U

/**
 * Build the byte string upstream signs, into @p buf.
 *
 * Takes the Data rather than a field list, as upstream does, so adding a field to the
 * covered set cannot silently miss a call site. @p from_node, @p packet_id and @p to_node
 * come from the MeshPacket header (@p to_node is MESHTASTIC_NODE_BROADCAST for a
 * broadcast); everything else is read from @p data.
 *
 * Reference: firmware CryptoEngine.cpp buildSigningBuffer. Getting this wrong is the
 * silent failure mode -- the crypto is fine and every signature still fails -- so it is
 * pinned by harvested vectors (tests/vectors/meshtastic_xeddsa_vectors.h), not by
 * inspection.
 *
 * @return the length written, or 0 if it would not fit.
 */
size_t meshtastic_xeddsa_build_signing_buffer(uint8_t *buf, size_t buf_size, uint32_t from_node,
					      uint32_t packet_id, uint32_t to_node,
					      const meshtastic_Data *data);

/**
 * Verify an XEdDSA signature over @p msg made by the holder of @p curve_pub.
 *
 * @return true only on a good signature. Any bad input -- wrong length, a public key that is
 *         not a valid point, a malformed signature -- is false, never an error to ignore.
 */
bool meshtastic_xeddsa_verify(const uint8_t curve_pub[MESHTASTIC_XEDDSA_KEY_LEN],
			      const uint8_t *msg, size_t msg_len,
			      const uint8_t sig[MESHTASTIC_XEDDSA_SIGNATURE_LEN]);

/** Counters behind `meshtastic xeddsa`. The only way to see this feature working on a node:
 *  the verify path's success log is DBG, which the bench images compile out at inf. */
struct meshtastic_xeddsa_stats {
	uint32_t verified;      /**< signatures checked and good */
	uint32_t failed;        /**< present, checked, BAD -- the packet was dropped */
	uint32_t malformed;     /**< length neither 0 nor 64 -- dropped */
	uint32_t no_key;        /**< signed by a node we hold no key for */
	uint32_t bootstrapped;  /**< first-contact NodeInfo whose id committed to its key */
	uint32_t unsigned_ok;   /**< unsigned, accepted under the policy */
	uint32_t unsigned_drop; /**< unsigned, dropped (STRICT) */
	uint32_t balanced_drop; /**< unsigned, dropped (BALANCED: from a known signer, would have fit signed) */
	uint32_t signed_tx;     /**< packets we signed on the way out */
	uint32_t sign_skipped;  /**< we would have signed, but the signed form did not fit */
};

/** Snapshot the counters. */
void meshtastic_xeddsa_get_stats(struct meshtastic_xeddsa_stats *out);

/** Zero the counters (shell `meshtastic xeddsa reset`). */
void meshtastic_xeddsa_reset_stats(void);

/** Count a packet we signed, or one we skipped for size. Called by the encoder. */
void meshtastic_xeddsa_note_tx(bool signed_ok);

/**
 * Receive-side signature policy gate.
 *
 * Runs after decode and BEFORE delivery or relay, like the reference's
 * checkXeddsaReceivePolicy. Drops a packet whose signature is present and bad, or malformed,
 * or (under STRICT) absent -- so a forged broadcast never reaches a module, the phone, or
 * the rest of the mesh through us.
 *
 * @param pkt   the decoded packet (from / id / to are the header part of what gets signed).
 * @param mesh  the decoded MeshPacket, which carries the signature bytes, the Data envelope
 *              the signature covers, and the xeddsa_signed flag; may be NULL, which means
 *              "no signature present".
 *
 * @return true to accept the packet, false to drop it.
 */
bool meshtastic_xeddsa_check_rx_policy(const struct meshtastic_packet *pkt,
				       meshtastic_MeshPacket *mesh);

#if defined(CONFIG_MESHTASTIC_XEDDSA_SIGN_CORE)
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
#endif /* CONFIG_MESHTASTIC_XEDDSA_SIGN_CORE */

#endif /* MESHTASTIC_XEDDSA_H_ */
