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
