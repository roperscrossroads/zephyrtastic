/* SPDX-License-Identifier: GPL-3.0 */

/* XEdDSA verify, measured against UPSTREAM's own signer.
 *
 * This port never signs, so it cannot check itself: a self-loopback would only prove our
 * verify agrees with our (absent) signer. Every signature here was produced by compiling
 * and running upstream's XEdDSA against upstream's signing-buffer layout
 * (tools/vectors/harvest_xeddsa.py), which is what lets these tests fail when we are
 * consistently wrong rather than only when we are inconsistently wrong.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "meshtastic_xeddsa.h"
#include "vectors/meshtastic_xeddsa_vectors.h"

#define N_VECTORS ARRAY_SIZE(mt_xeddsa_vectors)

/* --- the map ------------------------------------------------------------------------ */

/* The X25519 -> Ed25519 conversion must land on exactly the key upstream derived from the
 * private key. If it does not, every signature fails and the cause looks like "the crypto
 * is broken" rather than "the key is the wrong one". */
ZTEST(xeddsa, test_curve_to_ed_pub_matches_upstream)
{
	for (size_t i = 0; i < N_VECTORS; i++) {
		const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[i];
		uint8_t ed[32];

		meshtastic_xeddsa_curve_to_ed_pub(v->x_pub, ed);
		zassert_mem_equal(ed, v->ed_pub, 32, "%s: derived Ed25519 key differs", v->label);
	}
}

/* CHARACTERISATION, not a test of our mask -- and the difference matters.
 *
 * The map ends with `ed_pub[31] &= 0x7F`, mirroring upstream. Deleting that line breaks
 * NOTHING: fe_tobytes reduces mod 2^255-19, so bit 255 of its output is already always
 * zero (verified by mutation, 2026-09-17 -- the first version of this test passed with the
 * mask removed, which made it worthless). The mask is therefore belt-and-braces that states
 * the XEdDSA convention in code, not a behaviour this suite can fail on.
 *
 * What this test pins is the field encoding the convention RELIES on: if some future
 * vendored arithmetic ever returned an unreduced high bit, the signer's "sign bit is always
 * zero" guarantee would stop holding and the mask would become load-bearing. Then this test
 * starts failing on the input below rather than in the field.
 */
ZTEST(xeddsa, test_derived_key_high_bit_is_always_clear)
{
	static const uint8_t patterns[][32] = {
		{[0 ... 31] = 0xFF}, /* not a valid u, but fe_frombytes masks bit 255 */
		{[0 ... 31] = 0x00},
		{[0 ... 31] = 0x7F},
		{[0] = 0x09},        /* the X25519 base point */
	};

	for (size_t i = 0; i < ARRAY_SIZE(patterns); i++) {
		uint8_t ed[32];

		meshtastic_xeddsa_curve_to_ed_pub(patterns[i], ed);
		zassert_equal(ed[31] & 0x80, 0, "pattern %zu: high bit set in a derived key", i);
	}
}

/* --- the signed bytes --------------------------------------------------------------- */

/* Parity of the byte string, not of the crypto. A wrong header layout here is invisible to
 * every self-test and fatal on a real mesh. */
ZTEST(xeddsa, test_signing_buffer_matches_upstream)
{
	for (size_t i = 0; i < N_VECTORS; i++) {
		const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[i];
		uint8_t buf[MESHTASTIC_XEDDSA_SIGBUF_MAX];
		size_t len;

		len = meshtastic_xeddsa_build_signing_buffer(buf, sizeof(buf), v->from, v->id,
							     v->portnum, v->payload,
							     v->payload_len);
		zassert_equal(len, v->signed_len, "%s: signed length differs", v->label);
		zassert_mem_equal(buf, v->signed_bytes, len, "%s: signed bytes differ", v->label);
	}
}

ZTEST(xeddsa, test_signing_buffer_refuses_overflow)
{
	uint8_t buf[MESHTASTIC_XEDDSA_SIGBUF_MAX];
	uint8_t payload[MESHTASTIC_XEDDSA_SIGBUF_MAX];

	memset(payload, 0xA5, sizeof(payload));
	/* One byte more than the header leaves room for. */
	zassert_equal(meshtastic_xeddsa_build_signing_buffer(
			      buf, sizeof(buf), 1, 2, 3, payload,
			      sizeof(buf) - MESHTASTIC_XEDDSA_SIGBUF_HEADER_LEN + 1U),
		      0U, "an oversized payload must be refused, not truncated");
}

/* --- verify -------------------------------------------------------------------------- */

ZTEST(xeddsa, test_verify_accepts_upstream_signatures)
{
	for (size_t i = 0; i < N_VECTORS; i++) {
		const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[i];

		zassert_true(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes, v->signed_len,
						      v->sig),
			     "%s: a genuine upstream signature was rejected", v->label);
	}
}

/* The whole point of verifying: a forged or altered packet must not pass. Each mutation is
 * one byte, because a test that mangles everything proves only that something is checked. */
ZTEST(xeddsa, test_verify_rejects_tampering)
{
	for (size_t i = 0; i < N_VECTORS; i++) {
		const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[i];
		uint8_t sig[64];
		uint8_t msg[MESHTASTIC_XEDDSA_SIGBUF_MAX];
		uint8_t key[32];

		/* R half of the signature */
		memcpy(sig, v->sig, sizeof(sig));
		sig[0] ^= 0x01;
		zassert_false(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes, v->signed_len,
						       sig),
			      "%s: altered R accepted", v->label);

		/* S half */
		memcpy(sig, v->sig, sizeof(sig));
		sig[32] ^= 0x01;
		zassert_false(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes, v->signed_len,
						       sig),
			      "%s: altered S accepted", v->label);

		/* the signed bytes: flip a header bit -- i.e. claim another sender */
		memcpy(msg, v->signed_bytes, v->signed_len);
		msg[0] ^= 0x01;
		zassert_false(meshtastic_xeddsa_verify(v->x_pub, msg, v->signed_len, v->sig),
			      "%s: altered sender accepted", v->label);

		/* a different signer's key */
		memcpy(key, v->x_pub, sizeof(key));
		key[0] ^= 0x01;
		zassert_false(meshtastic_xeddsa_verify(key, v->signed_bytes, v->signed_len,
						       v->sig),
			      "%s: signature accepted under the wrong key", v->label);

		/* truncation: the same bytes, one short */
		if (v->signed_len > 0U) {
			zassert_false(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes,
							       v->signed_len - 1U, v->sig),
				      "%s: truncated message accepted", v->label);
		}
	}
}

/* A signature is not transferable between packets: the vectors use different keys and
 * different headers, so no signature may verify against another vector's bytes. */
ZTEST(xeddsa, test_verify_rejects_cross_vector_signatures)
{
	for (size_t i = 0; i < N_VECTORS; i++) {
		for (size_t j = 0; j < N_VECTORS; j++) {
			if (i == j) {
				continue;
			}
			zassert_false(meshtastic_xeddsa_verify(mt_xeddsa_vectors[i].x_pub,
							       mt_xeddsa_vectors[j].signed_bytes,
							       mt_xeddsa_vectors[j].signed_len,
							       mt_xeddsa_vectors[i].sig),
				      "%s's signature verified over %s's bytes",
				      mt_xeddsa_vectors[i].label, mt_xeddsa_vectors[j].label);
		}
	}
}

/* Upstream's verify rejects a non-canonical S (the top three bits of the last byte). Ours
 * inherits that; assert it, because accepting them is a malleability bug. */
ZTEST(xeddsa, test_verify_rejects_non_canonical_s)
{
	const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[0];
	uint8_t sig[64];

	memcpy(sig, v->sig, sizeof(sig));
	sig[63] |= 0x20;
	zassert_false(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes, v->signed_len, sig),
		      "a non-canonical S must be rejected");
}

/* An all-zero key is the port's "no key for this node" sentinel. It must never verify, and
 * it must not be spent on a field inversion first. */
ZTEST(xeddsa, test_verify_refuses_absent_key)
{
	const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[0];
	uint8_t zero[32] = {0};

	zassert_false(meshtastic_xeddsa_verify(zero, v->signed_bytes, v->signed_len, v->sig),
		      "an all-zero key must never verify");
}

ZTEST(xeddsa, test_verify_refuses_null_arguments)
{
	const struct mt_xeddsa_vector *v = &mt_xeddsa_vectors[0];

	zassert_false(meshtastic_xeddsa_verify(NULL, v->signed_bytes, v->signed_len, v->sig),
		      "NULL key");
	zassert_false(meshtastic_xeddsa_verify(v->x_pub, NULL, 4, v->sig), "NULL message");
	zassert_false(meshtastic_xeddsa_verify(v->x_pub, v->signed_bytes, v->signed_len, NULL),
		      "NULL signature");
}

ZTEST_SUITE(xeddsa, NULL, NULL, NULL, NULL, NULL);
