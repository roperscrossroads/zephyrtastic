/* SPDX-License-Identifier: GPL-3.0 */

/* PSA-backed SHA-512 for the vendored orlp/ed25519 verify path. See sha512.h. */

#include <string.h>

#include "sha512.h"

int sha512_init(sha512_context *md)
{
	if (md == NULL) {
		return -1;
	}
	md->op = psa_hash_operation_init();
	md->failed = 0;
	/* psa_crypto_init() is idempotent and already called by the PKI init path; calling it
	 * here too keeps this usable from a test that pulls in verify alone. */
	if (psa_crypto_init() != PSA_SUCCESS ||
	    psa_hash_setup(&md->op, PSA_ALG_SHA_512) != PSA_SUCCESS) {
		md->failed = 1;
		return -1;
	}
	return 0;
}

int sha512_update(sha512_context *md, const unsigned char *in, size_t inlen)
{
	if (md == NULL || md->failed) {
		return -1;
	}
	if (inlen == 0U) {
		return 0; /* PSA rejects a NULL buffer; an empty update is a no-op anyway */
	}
	if (in == NULL || psa_hash_update(&md->op, in, inlen) != PSA_SUCCESS) {
		md->failed = 1;
		return -1;
	}
	return 0;
}

int sha512_final(sha512_context *md, unsigned char *out)
{
	size_t olen = 0U;

	if (md == NULL || out == NULL) {
		return -1;
	}
	if (md->failed) {
		(void)psa_hash_abort(&md->op);
		return -1;
	}
	if (psa_hash_finish(&md->op, out, 64U, &olen) != PSA_SUCCESS || olen != 64U) {
		md->failed = 1;
		/* Never leave a half-written digest behind: a caller that ignores the return
		 * value must not verify against a partial hash. */
		memset(out, 0, 64U);
		return -1;
	}
	return 0;
}
