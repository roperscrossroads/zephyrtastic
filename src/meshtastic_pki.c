/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * Meshtastic PKC — see meshtastic_pki.h. Wire format locked to the reference
 * firmware CryptoEngine.cpp:
 *   key      = SHA256( X25519(our_priv, their_pub) )        [32B -> AES-256]
 *   cipher   = AES-CCM, nonce 13B, tag 8B, no AAD
 *   nonce    = [0..3]=id LE32, [4..7]=extraNonce LE32, [8..11]=from LE32, [12]=0
 *   wire     = ciphertext || tag(8) || extraNonce(4)   (plaintext_len + 12)
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <psa/crypto.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_config_store.h"
#include "meshtastic_settings.h"
#include "meshtastic_pki.h"
#include "meshtastic/config.pb.h"

LOG_MODULE_REGISTER(meshtastic_pki, CONFIG_MESHTASTIC_LOG_LEVEL);

#define PKI_KEY_LEN     MESHTASTIC_PKI_KEY_LEN
#define PKI_TAG_LEN     8
#define PKI_EXTRA_LEN   4
#define PKI_NONCE_LEN   13
#define PKI_CCM_ALG     PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, PKI_TAG_LEN)

/* Our keypair, kept in RAM after load/generate. g_priv is the raw X25519 scalar
 * (also persisted in SecurityConfig NVS); re-imported per crypto op. */
static uint8_t g_priv[PKI_KEY_LEN];
static uint8_t g_pub[PKI_KEY_LEN];
static bool g_have_key;

/* ---- key management ---------------------------------------------------- */

static int pki_generate(uint8_t priv[PKI_KEY_LEN], uint8_t pub[PKI_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	size_t olen;
	psa_status_t st;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);

	st = psa_generate_key(&attr, &kid);
	if (st != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key failed (%d)", (int)st);
		return -EIO;
	}

	st = psa_export_key(kid, priv, PKI_KEY_LEN, &olen);
	if (st == PSA_SUCCESS && olen == PKI_KEY_LEN) {
		st = psa_export_public_key(kid, pub, PKI_KEY_LEN, &olen);
	}
	(void)psa_destroy_key(kid);

	if (st != PSA_SUCCESS || olen != PKI_KEY_LEN) {
		LOG_ERR("pki key export failed (%d)", (int)st);
		return -EIO;
	}
	return 0;
}

static int pki_derive_public(const uint8_t priv[PKI_KEY_LEN], uint8_t pub[PKI_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	size_t olen;
	psa_status_t st;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);

	st = psa_import_key(&attr, priv, PKI_KEY_LEN, &kid);
	if (st == PSA_SUCCESS) {
		st = psa_export_public_key(kid, pub, PKI_KEY_LEN, &olen);
		(void)psa_destroy_key(kid);
	}
	if (st != PSA_SUCCESS) {
		LOG_ERR("pki derive public failed (%d)", (int)st);
		return -EIO;
	}
	return 0;
}

int meshtastic_pki_init(void)
{
	meshtastic_Config cfg = meshtastic_Config_init_zero;
	int ret;

	ret = meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg);
	if (ret == 0 && cfg.which_payload_variant == meshtastic_Config_security_tag &&
	    cfg.payload_variant.security.private_key.size == PKI_KEY_LEN) {
		/* Reuse the persisted key — NEVER regenerate when one exists. */
		memcpy(g_priv, cfg.payload_variant.security.private_key.bytes, PKI_KEY_LEN);
		if (cfg.payload_variant.security.public_key.size == PKI_KEY_LEN) {
			memcpy(g_pub, cfg.payload_variant.security.public_key.bytes, PKI_KEY_LEN);
		} else if (pki_derive_public(g_priv, g_pub) != 0) {
			return -EIO;
		}
		g_have_key = true;
		LOG_INF("PKI keypair loaded from NVS (pub %02x%02x%02x..)", g_pub[0], g_pub[1],
			g_pub[2]);
		return 0;
	}

	/* None stored — generate one, persist it, and only then advertise. */
	if (pki_generate(g_priv, g_pub) != 0) {
		return -EIO;
	}

	/* Preserve any other SecurityConfig fields already present in cfg. */
	cfg.which_payload_variant = meshtastic_Config_security_tag;
	cfg.payload_variant.security.private_key.size = PKI_KEY_LEN;
	memcpy(cfg.payload_variant.security.private_key.bytes, g_priv, PKI_KEY_LEN);
	cfg.payload_variant.security.public_key.size = PKI_KEY_LEN;
	memcpy(cfg.payload_variant.security.public_key.bytes, g_pub, PKI_KEY_LEN);

	ret = meshtastic_config_store_set_config(&cfg);
	if (ret == 0) {
		/* Force a SYNCHRONOUS NVS write: the store's save is otherwise
		 * scheduled (~1s), and we must not advertise a key that a reboot
		 * could lose (→ different key next boot → peer key-mismatch). */
		ret = meshtastic_settings_flush();
	}
	if (ret != 0) {
		LOG_ERR("PKI keypair persist failed (%d) — PKC disabled until next boot", ret);
		memset(g_priv, 0, sizeof(g_priv));
		memset(g_pub, 0, sizeof(g_pub));
		g_have_key = false;
		return ret;
	}

	g_have_key = true;
	LOG_INF("PKI keypair generated + persisted (pub %02x%02x%02x..)", g_pub[0], g_pub[1],
		g_pub[2]);
	return 0;
}

bool meshtastic_pki_have_key(void)
{
	return g_have_key;
}

size_t meshtastic_pki_get_public_key(uint8_t out[PKI_KEY_LEN])
{
	if (!g_have_key) {
		return 0;
	}
	memcpy(out, g_pub, PKI_KEY_LEN);
	return PKI_KEY_LEN;
}

/* ---- crypto primitives ------------------------------------------------- */

/* AES key = SHA256( X25519(our_priv, peer_pub) ). */
static int pki_shared_aes_key(const uint8_t peer_pub[PKI_KEY_LEN], uint8_t out_key[PKI_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	uint8_t secret[PKI_KEY_LEN];
	size_t olen;
	psa_status_t st;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);

	st = psa_import_key(&attr, g_priv, PKI_KEY_LEN, &kid);
	if (st != PSA_SUCCESS) {
		LOG_ERR("pki import priv failed (%d)", (int)st);
		return -EIO;
	}

	st = psa_raw_key_agreement(PSA_ALG_ECDH, kid, peer_pub, PKI_KEY_LEN, secret, sizeof(secret),
				   &olen);
	(void)psa_destroy_key(kid);
	if (st != PSA_SUCCESS || olen != PKI_KEY_LEN) {
		LOG_DBG("pki ECDH failed (%d)", (int)st);
		return -EIO;
	}

	st = psa_hash_compute(PSA_ALG_SHA_256, secret, sizeof(secret), out_key, PKI_KEY_LEN, &olen);
	memset(secret, 0, sizeof(secret));
	if (st != PSA_SUCCESS || olen != PKI_KEY_LEN) {
		LOG_ERR("pki SHA256 failed (%d)", (int)st);
		return -EIO;
	}
	return 0;
}

static void pki_build_nonce(uint8_t nonce[PKI_NONCE_LEN], uint32_t id, uint32_t from,
			    uint32_t extra)
{
	memset(nonce, 0, PKI_NONCE_LEN);
	sys_put_le32(id, nonce);          /* [0..3]  packet id  */
	sys_put_le32(extra, nonce + 4U);  /* [4..7]  extra nonce */
	sys_put_le32(from, nonce + 8U);   /* [8..11] from node  */
	/* nonce[12] stays 0 */
}

static int pki_peer_pub(uint32_t node, uint8_t out[PKI_KEY_LEN])
{
	struct meshtastic_nodedb_node n;

	if (meshtastic_nodedb_get(node, &n) != 0 || n.public_key_len != PKI_KEY_LEN) {
		return -ENOENT; /* no public key on record for this node */
	}
	memcpy(out, n.public_key, PKI_KEY_LEN);
	return 0;
}

int meshtastic_pki_decrypt(uint32_t from, uint32_t id, const uint8_t *enc, size_t enc_len,
			   uint8_t *out, size_t out_cap, size_t *out_len)
{
	uint8_t peer_pub[PKI_KEY_LEN];
	uint8_t aes_key[PKI_KEY_LEN];
	uint8_t nonce[PKI_NONCE_LEN];
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	psa_status_t st;
	uint32_t extra;
	size_t ct_tag_len;
	int ret;

	if (enc == NULL || out == NULL || out_len == NULL || !g_have_key) {
		return -EINVAL;
	}
	/* need at least 1 ciphertext byte + tag + extra nonce */
	if (enc_len <= MESHTASTIC_PKI_OVERHEAD) {
		return -EINVAL;
	}

	ret = pki_peer_pub(from, peer_pub);
	if (ret != 0) {
		return ret; /* -ENOKEY: sender pubkey unknown */
	}

	ret = pki_shared_aes_key(peer_pub, aes_key);
	if (ret != 0) {
		return ret;
	}

	/* wire = ciphertext || tag(8) || extraNonce(4). PSA wants ct||tag together. */
	ct_tag_len = enc_len - PKI_EXTRA_LEN;                  /* ciphertext + 8-byte tag */
	extra = sys_get_le32(enc + enc_len - PKI_EXTRA_LEN);
	pki_build_nonce(nonce, id, from, extra);

	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 256);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attr, PKI_CCM_ALG);

	st = psa_import_key(&attr, aes_key, PKI_KEY_LEN, &kid);
	memset(aes_key, 0, sizeof(aes_key));
	if (st != PSA_SUCCESS) {
		LOG_ERR("pki AES import failed (%d)", (int)st);
		return -EIO;
	}

	st = psa_aead_decrypt(kid, PKI_CCM_ALG, nonce, PKI_NONCE_LEN, NULL, 0, enc, ct_tag_len, out,
			      out_cap, out_len);
	(void)psa_destroy_key(kid);
	if (st != PSA_SUCCESS) {
		LOG_DBG("pki CCM decrypt/auth failed (%d)", (int)st);
		return -EBADMSG;
	}
	return 0;
}

int meshtastic_pki_encrypt(uint32_t to, uint32_t from, uint32_t id, const uint8_t *plain,
			   size_t plain_len, uint8_t *out, size_t out_cap, size_t *out_len)
{
	uint8_t peer_pub[PKI_KEY_LEN];
	uint8_t aes_key[PKI_KEY_LEN];
	uint8_t nonce[PKI_NONCE_LEN];
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	psa_status_t st;
	uint32_t extra;
	size_t ct_tag_len;
	int ret;

	if (plain == NULL || out == NULL || out_len == NULL || !g_have_key) {
		return -EINVAL;
	}
	if (out_cap < plain_len + MESHTASTIC_PKI_OVERHEAD) {
		return -ENOSPC;
	}

	ret = pki_peer_pub(to, peer_pub);
	if (ret != 0) {
		return ret;
	}
	ret = pki_shared_aes_key(peer_pub, aes_key);
	if (ret != 0) {
		return ret;
	}

	st = psa_generate_random((uint8_t *)&extra, sizeof(extra));
	if (st != PSA_SUCCESS) {
		memset(aes_key, 0, sizeof(aes_key));
		return -EIO;
	}
	pki_build_nonce(nonce, id, from, extra);

	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 256);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attr, PKI_CCM_ALG);

	st = psa_import_key(&attr, aes_key, PKI_KEY_LEN, &kid);
	memset(aes_key, 0, sizeof(aes_key));
	if (st != PSA_SUCCESS) {
		return -EIO;
	}

	/* PSA writes ciphertext||tag(8) contiguously into out. */
	st = psa_aead_encrypt(kid, PKI_CCM_ALG, nonce, PKI_NONCE_LEN, NULL, 0, plain, plain_len, out,
			      out_cap, &ct_tag_len);
	(void)psa_destroy_key(kid);
	if (st != PSA_SUCCESS || ct_tag_len != plain_len + PKI_TAG_LEN) {
		LOG_ERR("pki CCM encrypt failed (%d)", (int)st);
		return -EIO;
	}

	/* Append the 4-byte extra nonce after ct||tag. */
	sys_put_le32(extra, out + ct_tag_len);
	*out_len = ct_tag_len + PKI_EXTRA_LEN;
	return 0;
}
