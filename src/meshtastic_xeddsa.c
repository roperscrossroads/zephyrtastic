/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_xeddsa.h. Reference: firmware/src/mesh/CryptoEngine.cpp. */

#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
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
					      uint32_t packet_id, uint32_t to_node,
					      const meshtastic_Data *data)
{
	size_t total;
	uint8_t *w = buf;

	if (buf == NULL || data == NULL) {
		return 0U;
	}
	total = MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN + data->payload.size;
	if (total > buf_size || data->payload.size > sizeof(data->payload.bytes)) {
		return 0U;
	}
	/* The layout is the header comment's, byte for byte: upstream 2.8.1 writes every
	 * integer little-endian explicitly ("the encoding is pinned by the protocol, not by
	 * the host"), and so does this. */
	*w++ = MESHTASTIC_XEDDSA_SIGNING_VERSION;
	sys_put_le32(from_node, w);
	w += 4;
	sys_put_le32(packet_id, w);
	w += 4;
	sys_put_le32(to_node, w);
	w += 4;
	sys_put_le32((uint32_t)data->portnum, w);
	w += 4;
	sys_put_le32(data->request_id, w);
	w += 4;
	sys_put_le32(data->reply_id, w);
	w += 4;
	sys_put_le32(data->emoji, w);
	w += 4;
	/* An absent bitfield signs as zero; its presence is in the flags byte. */
	sys_put_le32(data->has_bitfield ? (uint32_t)data->bitfield : 0U, w);
	w += 4;
	*w++ = (uint8_t)((data->want_response ? MESHTASTIC_XEDDSA_SIGNED_FLAG_WANT_RESPONSE : 0U) |
			 (data->has_bitfield ? MESHTASTIC_XEDDSA_SIGNED_FLAG_HAS_BITFIELD : 0U));
	__ASSERT_NO_MSG((size_t)(w - buf) == MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN);
	if (data->payload.size != 0U) {
		memcpy(w, data->payload.bytes, data->payload.size);
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
#include <pb_encode.h>

#include <zephyr/sys/crc.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/telemetry.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

static struct meshtastic_xeddsa_stats stats;

void meshtastic_xeddsa_get_stats(struct meshtastic_xeddsa_stats *out)
{
	if (out != NULL) {
		*out = stats;
	}
}

void meshtastic_xeddsa_reset_stats(void)
{
	memset(&stats, 0, sizeof(stats));
}

void meshtastic_xeddsa_note_tx(bool signed_ok)
{
	if (signed_ok) {
		stats.signed_tx++;
	} else {
		stats.sign_skipped++;
	}
}

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

/*
 * Whether a decoded Data, canonicalized and sized as if it carried a real signature, would
 * still have fit the frame -- the same fit rule sign_our_packet already applies on the TX
 * side (meshtastic_packet.c), reused here so the two never disagree about what "would have
 * fit signed" means. Canonicalizing first (decode as the known type, re-encode) strips any
 * padding a forger left in Data.payload: without that, an unsigned broadcast could be
 * inflated past the signable budget on paper and dodge the downgrade-drop below. Reference:
 * Router.cpp canonicalSignableSize + the size check in checkXeddsaReceivePolicy's Balanced
 * branch.
 *
 * Sizes only what this build's schema decodes as Position/Telemetry/NodeInfo (User);
 * anything else, or a payload that fails to decode as its own type, falls back to the raw
 * encoded size with a full signature attached -- matching upstream's own fallback. Mutates
 * @p data's payload/signature size fields and restores them before returning either way.
 *
 * Static scratch, not stack: these decoded structs are large for the smaller MCU targets.
 * Safe because this runs synchronously inside meshtastic_xeddsa_check_rx_policy, which this
 * port calls from one RX-processing context at a time, never reentered mid-check.
 */
static bool signed_form_would_fit(meshtastic_Data *data)
{
	static union {
		meshtastic_Position position;
		meshtastic_Telemetry telemetry;
		meshtastic_User user;
	} inner;
	const pb_msgdesc_t *fields = NULL;
	const pb_size_t saved_payload_size = data->payload.size;
	const pb_size_t saved_sig_size = data->xeddsa_signature.size;
	size_t canonical_payload;
	size_t signed_size;
	bool fits;

	switch (data->portnum) {
	case meshtastic_PortNum_POSITION_APP:
		fields = meshtastic_Position_fields;
		break;
	case meshtastic_PortNum_TELEMETRY_APP:
		fields = meshtastic_Telemetry_fields;
		break;
	case meshtastic_PortNum_NODEINFO_APP:
		fields = meshtastic_User_fields;
		break;
	default:
		break;
	}

	if (fields != NULL) {
		pb_istream_t is = pb_istream_from_buffer(data->payload.bytes, data->payload.size);

		memset(&inner, 0, sizeof(inner));
		if (pb_decode(&is, fields, &inner) &&
		    pb_get_encoded_size(&canonical_payload, fields, &inner) &&
		    canonical_payload <= data->payload.size) {
			data->payload.size = (pb_size_t)canonical_payload;
		}
	}

	data->xeddsa_signature.size = MESHTASTIC_XEDDSA_SIGNATURE_LEN;
	memset(data->xeddsa_signature.bytes, 0, MESHTASTIC_XEDDSA_SIGNATURE_LEN);
	fits = pb_get_encoded_size(&signed_size, meshtastic_Data_fields, data) &&
	       MESHTASTIC_HDR_LEN + signed_size <= MESHTASTIC_PKT_MAX;

	data->payload.size = saved_payload_size;
	data->xeddsa_signature.size = saved_sig_size;
	return fits;
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
			stats.unsigned_drop++;
			LOG_WRN("XEdDSA: unsigned packet from 0x%08x dropped (strict)",
				(unsigned int)pkt->from);
			return false;
		}
		/* BALANCED's extra rule (agents-ooma.32): drop an unsigned broadcast from a node
		 * KNOWN to sign, unless its signed form genuinely would not have fit. Unicasts
		 * get the same treatment only when WE are a licensed sender -- mirrors
		 * sign_our_packet's own signable rule (meshtastic_packet.c) and upstream's
		 * checkXeddsaReceivePolicy, which checks the RECEIVER's own owner.is_licensed
		 * here, not the sender's. mesh == NULL (the flat-struct fallback boundary) has
		 * no Data to canonicalize -- want_response/bitfield/emoji have no flat-struct
		 * home -- so it falls through to the same "never drop on a sizing failure" rule
		 * a genuine sizing failure gets. */
		if (policy == meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_BALANCED &&
		    mesh != NULL && meshtastic_nodedb_is_xeddsa_signer(pkt->from)) {
			const bool broadcast = (pkt->to == MESHTASTIC_NODE_BROADCAST);
			bool is_licensed = false;

			if (!broadcast) {
				bool is_unmessagable = false;

				meshtastic_config_store_get_owner_flags(&is_licensed,
									&is_unmessagable);
			}
			if ((broadcast || is_licensed) &&
			    signed_form_would_fit(&mesh->decoded)) {
				stats.balanced_drop++;
				LOG_WRN("XEdDSA: dropped unsigned packet from 0x%08x that "
					"previously signed (balanced)",
					(unsigned int)pkt->from);
				return false;
			}
		}
		stats.unsigned_ok++;
		return true;
	}

	if (sig_size != MESHTASTIC_XEDDSA_SIGNATURE_LEN) {
		/* Honest senders emit 0 or 64 bytes and nothing else. A partial signature is
		 * how a forgery would try to land in the unsigned branch above while its bytes
		 * still inflate the encoded size. */
		stats.malformed++;
		LOG_WRN("XEdDSA: malformed signature (%u bytes) from 0x%08x, drop",
			(unsigned int)sig_size, (unsigned int)pkt->from);
		return false;
	}

	/* The envelope the signature covers is the decoded Data itself (mesh is non-NULL
	 * here: the signature came out of it); from, id and to are the wire header's. */
	siglen = meshtastic_xeddsa_build_signing_buffer(sigbuf, sizeof(sigbuf), pkt->from, pkt->id,
							pkt->to, &mesh->decoded);
	if (siglen == 0U) {
		/* Longer than anything the reference can sign, so no signature over it can be
		 * genuine. */
		LOG_WRN("XEdDSA: unsignable payload (%u bytes) from 0x%08x, drop",
			(unsigned int)mesh->decoded.payload.size, (unsigned int)pkt->from);
		return false;
	}

	if (meshtastic_nodedb_copy_pubkey(pkt->from, key) == 0) {
		/* The NodeDB's stored key only. An opportunistic key (meshtastic_pki's pending
		 * slot, learned from an unverified handshake) must never authenticate a
		 * signature -- that would let a planted key vouch for its own node. */
		if (!meshtastic_xeddsa_verify(key, sigbuf, siglen,
					      mesh->decoded.xeddsa_signature.bytes)) {
			stats.failed++;
			LOG_WRN("XEdDSA: signature verify FAILED from 0x%08x, drop",
				(unsigned int)pkt->from);
			return false;
		}
		mesh->xeddsa_signed = true;
		stats.verified++;
		/* agents-ooma.32: learn this node as a signer so a later unsigned signable
		 * broadcast from it is dropped under BALANCED. The node already has a key on
		 * file (that is what we just verified against), so its warm-tier slot -- if
		 * it isn't hot-resident -- already exists and this reaches it. */
		meshtastic_nodedb_note_xeddsa_signer(pkt->from);
		LOG_DBG("XEdDSA: verified signature from 0x%08x", (unsigned int)pkt->from);
		return true;
	}

	bool applicable = false;

	if (verify_first_contact_nodeinfo(pkt, mesh->decoded.xeddsa_signature.bytes, sigbuf,
					  siglen, &applicable)) {
		mesh->xeddsa_signed = true;
		stats.verified++;
		stats.bootstrapped++;
		/* Marks the HOT entry immediately -- BALANCED protection is live right away.
		 * The warm tier catches up on this node's next verified signature instead of
		 * this one: verify_first_contact_nodeinfo deliberately does not commit the key
		 * here (see its own comment), so no warm slot exists yet for this node to mark
		 * a signer on. A first-contact node that is evicted before its next signed
		 * packet loses BALANCED protection for that one window -- acceptable, and no
		 * worse than the status quo before this bead. */
		meshtastic_nodedb_note_xeddsa_signer(pkt->from);
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
	stats.no_key++;
	LOG_DBG("XEdDSA: no key for 0x%08x, cannot verify", (unsigned int)pkt->from);
	return !strict;
}

#if defined(CONFIG_MESHTASTIC_XEDDSA_SIGN_CORE)
/* ==========================================================================
 * Signing. Reference: meshtastic/Crypto XEdDSA::sign -- whose source carries no licence
 * (SIGNING-AND-IDENTITY-DESIGN.md §6.1), so this is written from the scheme and pinned
 * byte-for-byte against signatures that implementation produced (tests/xeddsa_sign).
 *
 * It is Ed25519 with three differences, all of them XEdDSA's:
 *   - the scalar is the clamped X25519 private key, not a hash of a seed;
 *   - it is negated when the public key would otherwise have its sign bit set, which is
 *     what makes the verifier's "clear the sign bit" recovery correct;
 *   - the nonce hash takes a caller-supplied Z as well as the prefix and message.
 * ========================================================================== */

#include "crypto/ed25519/ge.h"
#include "crypto/ed25519/sc.h"
#include "crypto/ed25519/sha512.h"

/* Scalars for the negation: -a mod L computed as (L-1) * a + 0 through sc_muladd. */
static const uint8_t SC_ZERO[32] = {0};
static const uint8_t SC_MINUS_ONE[32] = {
	0xec, 0xd3, 0xf5, 0x5c, 0x1a, 0x63, 0x12, 0x58, 0xd6, 0x9c, 0xf7,
	0xa2, 0xde, 0xf9, 0xde, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
};

static int sha512_of(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len,
		     const uint8_t *c, size_t c_len, uint8_t out[64])
{
	sha512_context h;

	if (sha512_init(&h) != 0 || sha512_update(&h, a, a_len) != 0 ||
	    sha512_update(&h, b, b_len) != 0 || sha512_update(&h, c, c_len) != 0 ||
	    sha512_final(&h, out) != 0) {
		return -1;
	}
	return 0;
}

void meshtastic_xeddsa_derive_ed_keys(const uint8_t x_priv[MESHTASTIC_XEDDSA_KEY_LEN],
				      uint8_t ed_priv[MESHTASTIC_XEDDSA_KEY_LEN],
				      uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN])
{
	ge_p3 A;

	memcpy(ed_priv, x_priv, MESHTASTIC_XEDDSA_KEY_LEN);
	ed_priv[0] &= 0xF8;
	ed_priv[31] &= 0x7F;
	ed_priv[31] |= 0x40;

	ge_scalarmult_base(&A, ed_priv);
	ge_p3_tobytes(ed_pub, &A);

	if ((ed_pub[31] & 0x80) != 0U) {
		uint8_t neg[MESHTASTIC_XEDDSA_KEY_LEN];

		/* Both (a, A) and (-a, -A) are valid key pairs; XEdDSA always publishes the
		 * one whose sign bit is zero, so the verifier can reconstruct A from the
		 * X25519 key alone. */
		sc_muladd(neg, SC_MINUS_ONE, ed_priv, SC_ZERO);
		memcpy(ed_priv, neg, sizeof(neg));
		ge_scalarmult_base(&A, ed_priv);
		ge_p3_tobytes(ed_pub, &A);
	}
}

bool meshtastic_xeddsa_sign(const uint8_t x_priv[MESHTASTIC_XEDDSA_KEY_LEN], const uint8_t *msg,
			    size_t msg_len, const uint8_t z[MESHTASTIC_XEDDSA_KEY_LEN],
			    uint8_t sig[MESHTASTIC_XEDDSA_SIGNATURE_LEN])
{
	uint8_t a[MESHTASTIC_XEDDSA_KEY_LEN];
	uint8_t ed_pub[MESHTASTIC_XEDDSA_KEY_LEN];
	uint8_t prefix[64];
	uint8_t r[64];
	uint8_t k[64];
	ge_p3 R;
	bool ok = false;

	if (x_priv == NULL || sig == NULL || z == NULL || (msg == NULL && msg_len != 0U)) {
		return false;
	}

	meshtastic_xeddsa_derive_ed_keys(x_priv, a, ed_pub);

	/* The nonce prefix is the second half of SHA-512 over the scalar: it keeps the nonce
	 * tied to the key without ever exposing the key to the nonce hash directly. */
	if (sha512_of(a, sizeof(a), NULL, 0, NULL, 0, prefix) != 0) {
		goto out;
	}
	if (sha512_of(prefix + 32, 32, msg, msg_len, z, MESHTASTIC_XEDDSA_KEY_LEN, r) != 0) {
		goto out;
	}
	sc_reduce(r);

	ge_scalarmult_base(&R, r);
	ge_p3_tobytes(sig, &R);

	if (sha512_of(sig, 32, ed_pub, sizeof(ed_pub), msg, msg_len, k) != 0) {
		goto out;
	}
	sc_reduce(k);

	/* s = r + k*a (mod L) */
	sc_muladd(sig + 32, k, a, r);
	ok = true;

out:
	/* The scalar and the nonce are the two secrets here: a leaked nonce yields the key. */
	memset(a, 0, sizeof(a));
	memset(prefix, 0, sizeof(prefix));
	memset(r, 0, sizeof(r));
	memset(k, 0, sizeof(k));
	if (!ok) {
		memset(sig, 0, MESHTASTIC_XEDDSA_SIGNATURE_LEN);
	}
	return ok;
}
#endif /* CONFIG_MESHTASTIC_XEDDSA_SIGN_CORE */
