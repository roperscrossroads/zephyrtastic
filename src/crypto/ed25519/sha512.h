/* SPDX-License-Identifier: GPL-3.0 */

/* NOT part of orlp/ed25519 — this file and sha512_psa.c REPLACE its sha512.{c,h}.
 *
 * `verify.c` (vendored verbatim, zlib — see PROVENANCE.md) includes "sha512.h" and calls
 * the streaming API below. Rather than vendor a second SHA-512 implementation, we satisfy
 * that API with Zephyr's PSA crypto, which every build already links for PKC. That keeps
 * one hash implementation in the image, and it is the platform's, not a bundled copy.
 *
 * Only what verify.c uses is implemented: init / update / final. orlp's one-shot `sha512()`
 * and its internal state layout are deliberately absent — nothing here needs them.
 */

#ifndef MESHTASTIC_CRYPTO_ED25519_SHA512_H_
#define MESHTASTIC_CRYPTO_ED25519_SHA512_H_

#include <stddef.h>

#include <psa/crypto.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The name and shape orlp's verify.c expects; the contents are ours. */
typedef struct sha512_context_ {
	psa_hash_operation_t op;
	int failed; /* sticky: a PSA error anywhere makes final() produce nothing usable */
} sha512_context;

int sha512_init(sha512_context *md);
int sha512_update(sha512_context *md, const unsigned char *in, size_t inlen);
int sha512_final(sha512_context *md, unsigned char *out);

#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_CRYPTO_ED25519_SHA512_H_ */
