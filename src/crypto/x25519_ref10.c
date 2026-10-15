/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * X25519 (RFC 7748 §5) over the ref10 field arithmetic vendored for XEdDSA
 * (crypto/ed25519/fe.c, orlp/ed25519, zlib). The ladder is RFC 7748's, in the operation
 * order of SUPERCOP's ref10 crypto_scalarmult: A24 enters as 121666 with BB, which is
 * the RFC's AA + 121665 * E rewritten (AA = BB + E). orlp's own key_exchange.c is not
 * used because it takes an Ed25519 public key and maps it to Montgomery form first.
 * Every step runs whatever the scalar's bits: the swaps are fe_cswap's masks.
 */

#include <string.h>

#include "x25519_ref10.h"
#include "ed25519/fe.h"

void meshtastic_x25519_ref10(uint8_t out[32], const uint8_t scalar[32], const uint8_t u[32])
{
	uint8_t e[32];
	fe x1, x2, z2, x3, z3, tmp0, tmp1;
	unsigned int swap = 0U;

	memcpy(e, scalar, sizeof(e));
	e[0] &= 248U;
	e[31] &= 127U;
	e[31] |= 64U;

	fe_frombytes(x1, u); /* drops the top bit; a value >= p is reduced by the arithmetic */
	fe_1(x2);
	fe_0(z2);
	fe_copy(x3, x1);
	fe_1(z3);

	for (int pos = 254; pos >= 0; pos--) {
		unsigned int b = (e[pos / 8] >> (pos & 7)) & 1U;

		swap ^= b;
		fe_cswap(x2, x3, swap);
		fe_cswap(z2, z3, swap);
		swap = b;

		fe_sub(tmp0, x3, z3);    /* D  = x3 - z3 */
		fe_sub(tmp1, x2, z2);    /* B  = x2 - z2 */
		fe_add(x2, x2, z2);      /* A  = x2 + z2 */
		fe_add(z2, x3, z3);      /* C  = x3 + z3 */
		fe_mul(z3, tmp0, x2);    /* DA */
		fe_mul(z2, z2, tmp1);    /* CB */
		fe_sq(tmp0, tmp1);       /* BB */
		fe_sq(tmp1, x2);         /* AA */
		fe_add(x3, z3, z2);      /* DA + CB */
		fe_sub(z2, z3, z2);      /* DA - CB */
		fe_mul(x2, tmp1, tmp0);  /* x2 = AA * BB */
		fe_sub(tmp1, tmp1, tmp0); /* E = AA - BB */
		fe_sq(z2, z2);           /* (DA - CB)^2 */
		fe_mul121666(z3, tmp1);  /* 121666 * E */
		fe_sq(x3, x3);           /* x3 = (DA + CB)^2 */
		fe_add(tmp0, tmp0, z3);  /* BB + 121666 * E */
		fe_mul(z3, x1, z2);      /* z3 = x1 * (DA - CB)^2 */
		fe_mul(z2, tmp1, tmp0);  /* z2 = E * (BB + 121666 * E) */
	}
	fe_cswap(x2, x3, swap);
	fe_cswap(z2, z3, swap);

	fe_invert(z2, z2);
	fe_mul(x2, x2, z2);
	fe_tobytes(out, x2);
	memset(e, 0, sizeof(e));
}
