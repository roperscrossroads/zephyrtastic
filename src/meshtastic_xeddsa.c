/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_xeddsa.h. Reference: firmware/src/mesh/CryptoEngine.cpp. */

#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "meshtastic_xeddsa.h"

/* Vendored orlp/ed25519 (zlib) -- src/crypto/ed25519/PROVENANCE.md. `fe` is its field
 * element type; the conversion below needs field arithmetic, which is exactly why a
 * high-level EdDSA API (PSA included) cannot do this job. */
#include "crypto/ed25519/ed25519.h"
#include "crypto/ed25519/fe.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

void meshtastic_xeddsa_curve_to_ed_pub(const uint8_t curve_pub[MESHTASTIC_XEDDSA_KEY_LEN],
				       uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN])
{
	fe u;
	fe y;
	fe one;
	fe u_minus_one;
	fe u_plus_one;
	fe u_plus_one_inv;

	/* The birational map of RFC 7748 §4.1: the same curve in two coordinate systems, so
	 * y = (u - 1) / (u + 1) (mod p). fe_frombytes masks the high bit, as RFC 7748 §5
	 * requires of an X25519 public key. */
	fe_frombytes(u, curve_pub);
	fe_1(one);
	fe_sub(u_minus_one, u, one);
	fe_add(u_plus_one, u, one);
	fe_invert(u_plus_one_inv, u_plus_one);
	fe_mul(y, u_minus_one, u_plus_one_inv);
	fe_tobytes(ed_pub, y);

	/* An X25519 public key is only the u coordinate, so the Edwards x coordinate -- which
	 * an Ed25519 key carries as a single sign bit -- cannot be recovered from it. XEdDSA
	 * resolves that by convention rather than by guessing: the signer negates its key pair
	 * when needed so the public key's sign bit is always zero. Clearing it here is therefore
	 * the right key, not an approximation of it.
	 *
	 * In practice this mask is a no-op -- fe_tobytes reduces mod 2^255-19, so bit 255 is
	 * already clear (proven by mutation: removing this line fails no test). It stays because
	 * upstream has it and because it states the convention at the point it is relied on;
	 * tests/xeddsa characterises the encoding guarantee that makes it redundant. */
	ed_pub[31] &= 0x7F;
}

size_t meshtastic_xeddsa_build_signing_buffer(uint8_t *buf, size_t buf_size, uint32_t from_node,
					      uint32_t packet_id, uint32_t portnum,
					      const uint8_t *payload, size_t payload_len)
{
	const size_t total = MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN + payload_len;

	if (buf == NULL || total > buf_size || (payload == NULL && payload_len != 0U)) {
		return 0U;
	}
	/* Upstream memcpy()s the raw uint32s, so the layout is the host's byte order. Both
	 * implementations are little-endian; upstream's own comment flags this as a hazard for
	 * "oddball platforms", so spell it out rather than inherit a memcpy. */
	sys_put_le32(from_node, buf);
	sys_put_le32(packet_id, buf + 4);
	sys_put_le32(portnum, buf + 8);
	if (payload_len != 0U) {
		memcpy(buf + MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN, payload, payload_len);
	}
	return total;
}

bool meshtastic_xeddsa_verify(const uint8_t curve_pub[MESHTASTIC_XEDDSA_KEY_LEN],
			      const uint8_t *msg, size_t msg_len,
			      const uint8_t sig[MESHTASTIC_XEDDSA_SIGNATURE_LEN])
{
	uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN];

	if (curve_pub == NULL || sig == NULL || (msg == NULL && msg_len != 0U)) {
		return false;
	}
	/* An all-zero key is the "we have no key for this node" sentinel everywhere else in the
	 * port; it is not a point anyone can sign with, and letting it reach the map would spend
	 * a field inversion to reach the same answer. */
	uint8_t zero[MESHTASTIC_XEDDSA_KEY_LEN] = {0};

	if (memcmp(curve_pub, zero, sizeof(zero)) == 0) {
		return false;
	}

	meshtastic_xeddsa_curve_to_ed_pub(curve_pub, ed_pub);

	/* ed25519_verify rejects a non-canonical S (signature[63] & 224) and a public key that
	 * is not on the curve, so both are handled inside, not here. */
	return ed25519_verify(sig, msg, msg_len, ed_pub) == 1;
}

/* ==========================================================================
 * Receive-side policy gate. Reference: Router.cpp checkXeddsaReceivePolicy
 * and verifyFirstContactNodeInfo.
 * ========================================================================== */

#include <pb_decode.h>

#include <zephyr/sys/crc.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

static meshtastic_Config_SecurityConfig_PacketSignaturePolicy rx_policy(void)
{
	meshtastic_Config cfg;

	/* Unreadable config means the permissive default, never a stricter one: a node that
	 * cannot read its own SecurityConfig must not silently start dropping the mesh. */
	if (meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg) != 0 ||
	    cfg.which_payload_variant != meshtastic_Config_security_tag) {
		return meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_COMPATIBLE;
	}
	return cfg.payload_variant.security.packet_signature_policy;
}

/* First contact with a signer we hold no key for. Safe ONLY because the id commits to the
 * key: id == crc32(public key), so a stranger cannot claim an id whose key it does not hold
 * (the reason the bench migrated its node ids). The key rides in the NodeInfo payload, and
 * the signature is checked against that same key before a byte of it is stored. */
static bool verify_first_contact_nodeinfo(const struct meshtastic_packet *pkt,
					  const uint8_t *sig, uint8_t sigbuf[], size_t siglen,
					  bool *applicable)
{
	meshtastic_User user = meshtastic_User_init_zero;
	pb_istream_t is;

	*applicable = (pkt->portnum == MESHTASTIC_PORT_NODEINFO);
	if (!*applicable) {
		return false;
	}
	is = pb_istream_from_buffer(pkt->payload, pkt->payload_len);
	if (!pb_decode(&is, meshtastic_User_fields, &user) ||
	    user.public_key.size != MESHTASTIC_XEDDSA_KEY_LEN) {
		return false;
	}
	/* Zephyr's crc32_ieee is the reference's crc32Buffer (pinned by the node_identity
	 * suite's known-answer vector). */
	if (crc32_ieee(user.public_key.bytes, user.public_key.size) != pkt->from) {
		return false;
	}
	if (!meshtastic_xeddsa_verify(user.public_key.bytes, sigbuf, siglen, sig)) {
		return false;
	}
	/* Deliberately does NOT store the key. The gate decides accept-or-drop; the NodeDB
	 * learns this key from the very packet we are accepting, on the ordinary NodeInfo path
	 * (meshtastic_nodedb's packet hook decodes the same User and applies it). Upstream
	 * stores it here because its NodeDB write happens later in a different order; copying
	 * that would mean two code paths writing the same key with different pinning rules --
	 * and this one runs BEFORE the node exists, where commit_pubkey answers -ENOENT. */
	return true;
}

bool meshtastic_xeddsa_check_rx_policy(const struct meshtastic_packet *pkt,
				       meshtastic_MeshPacket *mesh)
{
	const meshtastic_Config_SecurityConfig_PacketSignaturePolicy policy = rx_policy();
	const bool strict =
		policy == meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_STRICT;
	uint8_t sigbuf[MESHTASTIC_XEDDSA_SIGBUF_MAX];
	uint8_t key[MESHTASTIC_XEDDSA_KEY_LEN];
	size_t siglen;
	pb_size_t sig_size;

	if (pkt == NULL) {
		return true;
	}
	/* Never trust an inbound flag: only a signature verified below may mark a packet
	 * signed. Upstream clears it here for the same reason. */
	if (mesh != NULL) {
		mesh->xeddsa_signed = false;
	}
	sig_size = (mesh != NULL) ? mesh->decoded.xeddsa_signature.size : 0U;

	if (sig_size == 0U) {
		/* PKC already authenticates the sender: only the holder of the private key could
		 * have produced a frame that decrypted. */
		if (pkt->pki_encrypted) {
			return true;
		}
		if (strict) {
			LOG_WRN("XEdDSA: unsigned packet from 0x%08x dropped (strict)",
				(unsigned int)pkt->from);
			return false;
		}
		/* COMPATIBLE and BALANCED both accept here. BALANCED's extra rule -- drop an
		 * unsigned signable broadcast from a node KNOWN to sign -- is deliberately not
		 * implemented yet: it needs a per-node signer flag that survives warm-tier
		 * eviction (see the trap in SIGNING-AND-IDENTITY-DESIGN.md §7), so a half
		 * version would forget the protection exactly when a node ages out and call
		 * that "balanced". Until then BALANCED behaves as COMPATIBLE. */
		return true;
	}

	if (sig_size != MESHTASTIC_XEDDSA_SIGNATURE_LEN) {
		/* Honest senders emit 0 or 64 bytes and nothing else. A partial signature is
		 * how a forgery would try to land in the unsigned branch above while its bytes
		 * still inflate the encoded size. */
		LOG_WRN("XEdDSA: malformed signature (%u bytes) from 0x%08x, drop",
			(unsigned int)sig_size, (unsigned int)pkt->from);
		return false;
	}

	siglen = meshtastic_xeddsa_build_signing_buffer(sigbuf, sizeof(sigbuf), pkt->from, pkt->id,
							pkt->portnum, pkt->payload,
							pkt->payload_len);
	if (siglen == 0U) {
		/* Longer than anything the reference can sign, so no signature over it can be
		 * genuine. */
		LOG_WRN("XEdDSA: unsignable payload (%zu bytes) from 0x%08x, drop",
			pkt->payload_len, (unsigned int)pkt->from);
		return false;
	}

	if (meshtastic_nodedb_copy_pubkey(pkt->from, key) == 0) {
		/* The NodeDB's stored key only. An opportunistic key (meshtastic_pki's pending
		 * slot, learned from an unverified handshake) must never authenticate a
		 * signature -- that would let a planted key vouch for its own node. */
		if (!meshtastic_xeddsa_verify(key, sigbuf, siglen,
					      mesh->decoded.xeddsa_signature.bytes)) {
			LOG_WRN("XEdDSA: signature verify FAILED from 0x%08x, drop",
				(unsigned int)pkt->from);
			return false;
		}
		mesh->xeddsa_signed = true;
		LOG_DBG("XEdDSA: verified signature from 0x%08x", (unsigned int)pkt->from);
		return true;
	}

	bool applicable = false;

	if (verify_first_contact_nodeinfo(pkt, mesh->decoded.xeddsa_signature.bytes, sigbuf,
					  siglen, &applicable)) {
		mesh->xeddsa_signed = true;
		LOG_INF("XEdDSA: verified first-contact NodeInfo from 0x%08x",
			(unsigned int)pkt->from);
		return true;
	}
	if (applicable) {
		LOG_WRN("XEdDSA: invalid first-contact NodeInfo from 0x%08x, drop",
			(unsigned int)pkt->from);
		return false;
	}
	/* Signed by a node we hold no key for, and not a NodeInfo that could carry one. */
	LOG_DBG("XEdDSA: no key for 0x%08x, cannot verify", (unsigned int)pkt->from);
	return !strict;
}
