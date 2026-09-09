/* SPDX-License-Identifier: GPL-3.0
 *
 * Lockdown storage core -- see meshtastic_lockdown.h and docs/LOCKDOWN-DESIGN.md.
 *
 * What is mirrored from the reference's EncryptedStorage, rule for rule: the key
 * hierarchy and the artifact set; the token's boots / epoch / session / monotonic
 * counter semantics and every lock reason string; the backoff curve (5 s doubling
 * to 900 s), its cross-reboot boot counter, and "a present-but-bad backoff
 * record counts as max attempts, a missing one as none"; a bad or missing
 * monotonic-counter record reads as 0.
 *
 * What differs, on purpose: AES-GCM for every artifact and record (one PSA AEAD
 * in place of AES-CTR + HMAC-SHA256), PBKDF2 for the passphrase KEK in place of a
 * single SHA-256, and settings records in place of files.
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <psa/crypto.h>

#include "meshtastic_clock.h"
#include "meshtastic_lockdown.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define LD_KEY_LEN   16U
#define LD_NONCE_LEN 12U
#define LD_TAG_LEN   16U
#define LD_DEVID_MAX 16U

#define LD_SUBTREE     "mtlock"
#define LD_KEY_DEK     LD_SUBTREE "/dek"
#define LD_KEY_TOKEN   LD_SUBTREE "/token"
#define LD_KEY_MONO    LD_SUBTREE "/mono"
#define LD_KEY_BACKOFF LD_SUBTREE "/backoff"

#define LD_MAGIC_SEAL  0x4D454E43U /* "MENC" */
#define LD_MAGIC_DEK   0x4D44454BU /* "MDEK" */
#define LD_MAGIC_TOKEN 0x55544F4BU /* "UTOK" */

/* Domain / label strings, the reference's. */
static const char KEK_DOMAIN[] = "meshtastic-tak-kek-v2";
static const char EKEK_DOMAIN[] = "meshtastic-tak-ephemeral-v1";
static const char DEK_LABEL[] = "mdek-auth";
static const char TOKEN_LABEL[] = "utok-auth";
static const char MONO_LABEL[] = "tokmono-auth";
static const char BACKOFF_LABEL[] = "backoff-auth";
static const char SEAL_LABEL[] = "menc-auth";

/* Record layouts (all little-endian). */
#define DEK_REC_LEN     (4U + LD_NONCE_LEN + LD_KEY_LEN + LD_TAG_LEN)            /* 48 */
#define TOKEN_META_LEN  (1U + 4U + 4U + 4U)                                       /* boots, until, session, mono */
#define TOKEN_REC_LEN   (4U + LD_NONCE_LEN + LD_KEY_LEN + TOKEN_META_LEN + LD_TAG_LEN) /* 61 */
#define MONO_REC_LEN    (4U + LD_NONCE_LEN + LD_TAG_LEN)                          /* 32 */
#define BACKOFF_BODY    6U
#define BACKOFF_REC_LEN (BACKOFF_BODY + LD_NONCE_LEN + LD_TAG_LEN)                /* 34 */

#define BACKOFF_MAX_ATTEMPTS 255U
#define BACKOFF_CAP_S        900U

static struct {
	uint8_t ekek[LD_KEY_LEN];
	uint8_t kek[LD_KEY_LEN];
	uint8_t dek[LD_KEY_LEN];
	bool ekek_ok;
	bool kek_ok;
	bool dek_ok;
	bool provisioned;
	const char *lock_reason;
	uint8_t boots_remaining;
	uint32_t valid_until;
	uint32_t session_max_ms;
	int64_t session_started_ms;
	uint32_t backoff_remaining_s;
	int64_t last_fail_ms; /* uptime of the last wrong passphrase this boot, 0 = none */
} ld = {.lock_reason = "not_provisioned"};

/* ---- small helpers ------------------------------------------------------------ */

static void zero(void *p, size_t n)
{
	volatile uint8_t *v = p;

	while (n-- > 0U) {
		*v++ = 0U;
	}
}

static uint32_t now_epoch(void)
{
	return meshtastic_clock_valid() ? meshtastic_clock_now_epoch() : 0U;
}

static int device_id(uint8_t out[LD_DEVID_MAX], size_t *len)
{
	ssize_t n = hwinfo_get_device_id(out, LD_DEVID_MAX);

	if (n <= 0) {
		return -EIO;
	}
	*len = (size_t)n;
	return 0;
}

/* AES-128-GCM one-shot under a raw 16-byte key. Empty plaintext is allowed: the
 * output is then just the tag, which is how the counter and backoff records are
 * authenticated without a ciphertext. */
static int aead(bool encrypt, const uint8_t key[LD_KEY_LEN], const uint8_t nonce[LD_NONCE_LEN],
		const uint8_t *aad, size_t aad_len, const uint8_t *in, size_t in_len, uint8_t *out,
		size_t cap, size_t *out_len)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	psa_status_t st;

	psa_set_key_usage_flags(&attr, encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_GCM);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 8U * LD_KEY_LEN);
	st = psa_import_key(&attr, key, LD_KEY_LEN, &kid);
	if (st != PSA_SUCCESS) {
		LOG_ERR("Lockdown: key import failed (%d)", (int)st);
		return -EIO;
	}
	if (encrypt) {
		st = psa_aead_encrypt(kid, PSA_ALG_GCM, nonce, LD_NONCE_LEN, aad, aad_len, in,
				      in_len, out, cap, out_len);
	} else {
		st = psa_aead_decrypt(kid, PSA_ALG_GCM, nonce, LD_NONCE_LEN, aad, aad_len, in,
				      in_len, out, cap, out_len);
	}
	(void)psa_destroy_key(kid);
	if (st != PSA_SUCCESS) {
		return encrypt ? -EIO : -EBADMSG;
	}
	return 0;
}

static int random_bytes(uint8_t *out, size_t n)
{
	return psa_generate_random(out, n) == PSA_SUCCESS ? 0 : -EIO;
}

/* ---- settings records ---------------------------------------------------------- */

struct rec_read {
	uint8_t *buf;
	size_t cap;
	size_t len;
	bool found;
};

static int rec_read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		       void *param)
{
	struct rec_read *r = param;

	ARG_UNUSED(key);
	r->found = true;
	if (len > r->cap) {
		r->len = 0U;
		return 1;
	}
	if (read_cb(cb_arg, r->buf, len) != (ssize_t)len) {
		r->len = 0U;
		return 1;
	}
	r->len = len;
	return 1; /* one record wanted */
}

/* Read one artifact. Returns its length, 0 when absent or unreadable
 * (@p found tells the two apart). */
static size_t rec_read(const char *name, uint8_t *buf, size_t cap, bool *found)
{
	struct rec_read r = {.buf = buf, .cap = cap};

	(void)settings_load_subtree_direct(name, rec_read_cb, &r);
	if (found != NULL) {
		*found = r.found;
	}
	return r.len;
}

/* ---- key derivation ------------------------------------------------------------- */

static int derive_ekek(void)
{
	uint8_t id[LD_DEVID_MAX];
	uint8_t msg[LD_DEVID_MAX + sizeof(EKEK_DOMAIN)];
	uint8_t hash[32];
	size_t id_len, hash_len;
	int ret;

	if (ld.ekek_ok) {
		return 0;
	}
	ret = device_id(id, &id_len);
	if (ret < 0) {
		return ret;
	}
	memcpy(msg, id, id_len);
	memcpy(msg + id_len, EKEK_DOMAIN, sizeof(EKEK_DOMAIN) - 1U);
	if (psa_hash_compute(PSA_ALG_SHA_256, msg, id_len + sizeof(EKEK_DOMAIN) - 1U, hash,
			     sizeof(hash), &hash_len) != PSA_SUCCESS) {
		return -EIO;
	}
	memcpy(ld.ekek, hash, LD_KEY_LEN);
	ld.ekek_ok = true;
	zero(hash, sizeof(hash));
	zero(msg, sizeof(msg));
	return 0;
}

/* KEK = PBKDF2-HMAC-SHA256(passphrase, salt = device id || domain). */
static int derive_kek(const uint8_t *passphrase, size_t len)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	uint8_t id[LD_DEVID_MAX];
	uint8_t salt[LD_DEVID_MAX + sizeof(KEK_DOMAIN)];
	size_t id_len;
	psa_status_t st;
	int ret;

	ret = device_id(id, &id_len);
	if (ret < 0) {
		return ret;
	}
	memcpy(salt, id, id_len);
	memcpy(salt + id_len, KEK_DOMAIN, sizeof(KEK_DOMAIN) - 1U);

	st = psa_key_derivation_setup(&op, PSA_ALG_PBKDF2_HMAC(PSA_ALG_SHA_256));
	if (st == PSA_SUCCESS) {
		st = psa_key_derivation_input_integer(&op, PSA_KEY_DERIVATION_INPUT_COST,
						      CONFIG_MESHTASTIC_LOCKDOWN_PBKDF2_ITERATIONS);
	}
	if (st == PSA_SUCCESS) {
		st = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT, salt,
						    id_len + sizeof(KEK_DOMAIN) - 1U);
	}
	if (st == PSA_SUCCESS) {
		st = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_PASSWORD,
						    passphrase, len);
	}
	if (st == PSA_SUCCESS) {
		st = psa_key_derivation_output_bytes(&op, ld.kek, LD_KEY_LEN);
	}
	(void)psa_key_derivation_abort(&op);
	zero(salt, sizeof(salt));
	if (st != PSA_SUCCESS) {
		LOG_ERR("Lockdown: PBKDF2 failed (%d)", (int)st);
		zero(ld.kek, sizeof(ld.kek));
		ld.kek_ok = false;
		return -EIO;
	}
	ld.kek_ok = true;
	return 0;
}

/* ---- artifacts ----------------------------------------------------------------- */

/* mtlock/dek: MDEK | nonce | GCM(KEK, DEK) | tag, AAD = label. */
static int write_dek(void)
{
	uint8_t rec[DEK_REC_LEN];
	size_t out_len;
	int ret;

	sys_put_le32(LD_MAGIC_DEK, rec);
	ret = random_bytes(rec + 4, LD_NONCE_LEN);
	if (ret < 0) {
		return ret;
	}
	ret = aead(true, ld.kek, rec + 4, (const uint8_t *)DEK_LABEL, sizeof(DEK_LABEL) - 1U,
		   ld.dek, LD_KEY_LEN, rec + 4 + LD_NONCE_LEN, LD_KEY_LEN + LD_TAG_LEN, &out_len);
	if (ret < 0) {
		return ret;
	}
	ret = settings_save_one(LD_KEY_DEK, rec, sizeof(rec));
	zero(rec, sizeof(rec));
	return ret;
}

/* Unwrap the DEK with the KEK in RAM: the passphrase check. */
static int load_dek(void)
{
	uint8_t rec[DEK_REC_LEN];
	size_t len, out_len;
	int ret;

	len = rec_read(LD_KEY_DEK, rec, sizeof(rec), NULL);
	if (len != DEK_REC_LEN || sys_get_le32(rec) != LD_MAGIC_DEK) {
		return -ENOENT;
	}
	ret = aead(false, ld.kek, rec + 4, (const uint8_t *)DEK_LABEL, sizeof(DEK_LABEL) - 1U,
		   rec + 4 + LD_NONCE_LEN, LD_KEY_LEN + LD_TAG_LEN, ld.dek, LD_KEY_LEN, &out_len);
	if (ret < 0 || out_len != LD_KEY_LEN) {
		zero(ld.dek, sizeof(ld.dek));
		return -EACCES;
	}
	return 0;
}

static bool dek_record_exists(void)
{
	uint8_t rec[DEK_REC_LEN];
	bool found = false;

	(void)rec_read(LD_KEY_DEK, rec, sizeof(rec), &found);
	return found;
}

/* mtlock/mono: counter | nonce | tag(GCM(ekek), empty, AAD = label || counter). */
static uint32_t read_mono(void)
{
	uint8_t rec[MONO_REC_LEN];
	uint8_t aad[sizeof(MONO_LABEL) - 1U + 4U];
	size_t len, out_len;

	len = rec_read(LD_KEY_MONO, rec, sizeof(rec), NULL);
	if (len != MONO_REC_LEN) {
		return 0U; /* reference readMonoCounter: missing or malformed reads as 0 */
	}
	memcpy(aad, MONO_LABEL, sizeof(MONO_LABEL) - 1U);
	memcpy(aad + sizeof(MONO_LABEL) - 1U, rec, 4U);
	if (aead(false, ld.ekek, rec + 4, aad, sizeof(aad), rec + 4 + LD_NONCE_LEN, LD_TAG_LEN,
		 NULL, 0U, &out_len) < 0) {
		return 0U;
	}
	return sys_get_le32(rec);
}

static int write_mono(uint32_t counter)
{
	uint8_t rec[MONO_REC_LEN];
	uint8_t aad[sizeof(MONO_LABEL) - 1U + 4U];
	size_t out_len;
	int ret;

	sys_put_le32(counter, rec);
	ret = random_bytes(rec + 4, LD_NONCE_LEN);
	if (ret < 0) {
		return ret;
	}
	memcpy(aad, MONO_LABEL, sizeof(MONO_LABEL) - 1U);
	memcpy(aad + sizeof(MONO_LABEL) - 1U, rec, 4U);
	ret = aead(true, ld.ekek, rec + 4, aad, sizeof(aad), NULL, 0U, rec + 4 + LD_NONCE_LEN,
		   LD_TAG_LEN, &out_len);
	if (ret < 0) {
		return ret;
	}
	return settings_save_one(LD_KEY_MONO, rec, sizeof(rec));
}

/* mtlock/backoff: attempts | boots_since_fail | last_fail_epoch | nonce | tag.
 * Missing -> none; present but wrong or unauthenticated -> max attempts. */
static void read_backoff(uint8_t *attempts, uint8_t *boots_since_fail, uint32_t *last_fail_epoch)
{
	uint8_t rec[BACKOFF_REC_LEN];
	uint8_t aad[sizeof(BACKOFF_LABEL) - 1U + BACKOFF_BODY];
	size_t len, out_len;
	bool found;

	*attempts = 0U;
	*boots_since_fail = 0U;
	*last_fail_epoch = 0U;
	len = rec_read(LD_KEY_BACKOFF, rec, sizeof(rec), &found);
	if (!found) {
		return;
	}
	if (len != BACKOFF_REC_LEN) {
		*attempts = BACKOFF_MAX_ATTEMPTS;
		return;
	}
	memcpy(aad, BACKOFF_LABEL, sizeof(BACKOFF_LABEL) - 1U);
	memcpy(aad + sizeof(BACKOFF_LABEL) - 1U, rec, BACKOFF_BODY);
	if (aead(false, ld.ekek, rec + BACKOFF_BODY, aad, sizeof(aad),
		 rec + BACKOFF_BODY + LD_NONCE_LEN, LD_TAG_LEN, NULL, 0U, &out_len) < 0) {
		*attempts = BACKOFF_MAX_ATTEMPTS;
		return;
	}
	*attempts = rec[0];
	*boots_since_fail = rec[1];
	*last_fail_epoch = sys_get_le32(rec + 2);
}

static int write_backoff(uint8_t attempts, uint8_t boots_since_fail, uint32_t last_fail_epoch)
{
	uint8_t rec[BACKOFF_REC_LEN];
	uint8_t aad[sizeof(BACKOFF_LABEL) - 1U + BACKOFF_BODY];
	size_t out_len;
	int ret;

	rec[0] = attempts;
	rec[1] = boots_since_fail;
	sys_put_le32(last_fail_epoch, rec + 2);
	ret = random_bytes(rec + BACKOFF_BODY, LD_NONCE_LEN);
	if (ret < 0) {
		return ret;
	}
	memcpy(aad, BACKOFF_LABEL, sizeof(BACKOFF_LABEL) - 1U);
	memcpy(aad + sizeof(BACKOFF_LABEL) - 1U, rec, BACKOFF_BODY);
	ret = aead(true, ld.ekek, rec + BACKOFF_BODY, aad, sizeof(aad), NULL, 0U,
		   rec + BACKOFF_BODY + LD_NONCE_LEN, LD_TAG_LEN, &out_len);
	if (ret < 0) {
		return ret;
	}
	return settings_save_one(LD_KEY_BACKOFF, rec, sizeof(rec));
}

static void clear_backoff(void)
{
	(void)write_backoff(0U, 0U, 0U);
	ld.backoff_remaining_s = 0U;
	ld.last_fail_ms = 0;
}

/* Reference backoffDelay: 0, 5, 10, 20, ... capped at 900 s. */
static uint32_t backoff_delay(uint8_t attempts)
{
	uint32_t delay = 5U;

	if (attempts == 0U) {
		return 0U;
	}
	for (uint8_t i = 1U; i < attempts; i++) {
		delay *= 2U;
		if (delay >= BACKOFF_CAP_S) {
			return BACKOFF_CAP_S;
		}
	}
	return delay;
}

/* mtlock/token: UTOK | nonce | GCM(ekek, DEK) | boots | until | session | mono | tag,
 * AAD = label || the four meta fields. */
static int write_token(uint8_t boots, uint32_t valid_until, uint32_t session_s)
{
	uint8_t rec[TOKEN_REC_LEN];
	uint8_t aad[sizeof(TOKEN_LABEL) - 1U + TOKEN_META_LEN];
	uint8_t *meta = rec + 4 + LD_NONCE_LEN + LD_KEY_LEN;
	uint32_t mono = read_mono() + 1U;
	size_t out_len;
	int ret;

	sys_put_le32(LD_MAGIC_TOKEN, rec);
	ret = random_bytes(rec + 4, LD_NONCE_LEN);
	if (ret < 0) {
		return ret;
	}
	meta[0] = boots;
	sys_put_le32(valid_until, meta + 1);
	sys_put_le32(session_s, meta + 5);
	sys_put_le32(mono, meta + 9);
	memcpy(aad, TOKEN_LABEL, sizeof(TOKEN_LABEL) - 1U);
	memcpy(aad + sizeof(TOKEN_LABEL) - 1U, meta, TOKEN_META_LEN);
	/* Ciphertext lands right after the nonce; the tag goes after the meta. */
	{
		uint8_t ct_tag[LD_KEY_LEN + LD_TAG_LEN];

		ret = aead(true, ld.ekek, rec + 4, aad, sizeof(aad), ld.dek, LD_KEY_LEN, ct_tag,
			   sizeof(ct_tag), &out_len);
		if (ret < 0) {
			return ret;
		}
		memcpy(rec + 4 + LD_NONCE_LEN, ct_tag, LD_KEY_LEN);
		memcpy(meta + TOKEN_META_LEN, ct_tag + LD_KEY_LEN, LD_TAG_LEN);
		zero(ct_tag, sizeof(ct_tag));
	}
	ret = settings_save_one(LD_KEY_TOKEN, rec, sizeof(rec));
	zero(rec, sizeof(rec));
	if (ret == 0) {
		(void)write_mono(mono);
	}
	return ret;
}

static void delete_token(void)
{
	(void)settings_delete(LD_KEY_TOKEN);
}

/* Reference readAndConsumeToken: every refusal deletes the token and names why. */
static bool consume_token(void)
{
	uint8_t rec[TOKEN_REC_LEN];
	uint8_t aad[sizeof(TOKEN_LABEL) - 1U + TOKEN_META_LEN];
	uint8_t ct_tag[LD_KEY_LEN + LD_TAG_LEN];
	const uint8_t *meta = rec + 4 + LD_NONCE_LEN + LD_KEY_LEN;
	uint8_t boots;
	uint32_t valid_until, session_s, mono, max_seen;
	size_t len, out_len;
	bool found;

	len = rec_read(LD_KEY_TOKEN, rec, sizeof(rec), &found);
	if (!found) {
		ld.lock_reason = "token_missing";
		return false;
	}
	if (len != TOKEN_REC_LEN) {
		ld.lock_reason = "token_wrong_size";
		delete_token();
		return false;
	}
	if (sys_get_le32(rec) != LD_MAGIC_TOKEN) {
		ld.lock_reason = "token_bad_magic";
		delete_token();
		return false;
	}
	memcpy(aad, TOKEN_LABEL, sizeof(TOKEN_LABEL) - 1U);
	memcpy(aad + sizeof(TOKEN_LABEL) - 1U, meta, TOKEN_META_LEN);
	memcpy(ct_tag, rec + 4 + LD_NONCE_LEN, LD_KEY_LEN);
	memcpy(ct_tag + LD_KEY_LEN, meta + TOKEN_META_LEN, LD_TAG_LEN);
	/* Authenticate (and unwrap) first: the meta fields are only trusted once the
	 * tag over them holds -- the reference checks the HMAC before reading them. */
	if (aead(false, ld.ekek, rec + 4, aad, sizeof(aad), ct_tag, sizeof(ct_tag), ld.dek,
		 LD_KEY_LEN, &out_len) < 0 || out_len != LD_KEY_LEN) {
		zero(ld.dek, sizeof(ld.dek));
		ld.lock_reason = "token_hmac_fail";
		delete_token();
		return false;
	}
	boots = meta[0];
	valid_until = sys_get_le32(meta + 1);
	session_s = sys_get_le32(meta + 5);
	mono = sys_get_le32(meta + 9);

	max_seen = read_mono();
	if (mono < max_seen) {
		zero(ld.dek, sizeof(ld.dek));
		ld.lock_reason = "token_rollback";
		delete_token();
		return false;
	}
	if (mono > max_seen) {
		(void)write_mono(mono);
	}
	if (boots == 0U) {
		zero(ld.dek, sizeof(ld.dek));
		ld.lock_reason = "token_boots_zero";
		delete_token();
		return false;
	}
	if (valid_until != 0U) {
		uint32_t now = now_epoch();

		if (now == 0U) {
			LOG_WRN("Lockdown: token epoch TTL unverifiable (no clock); boot count rules (%u left)",
				boots);
		} else if (now > valid_until) {
			zero(ld.dek, sizeof(ld.dek));
			ld.lock_reason = "token_expired";
			delete_token();
			return false;
		}
	}

	ld.dek_ok = true;
	ld.boots_remaining = boots - 1U;
	ld.valid_until = valid_until;
	if (ld.boots_remaining == 0U) {
		delete_token(); /* last boot consumed; this boot stays unlocked */
	} else {
		(void)write_token(ld.boots_remaining, valid_until, session_s);
	}
	meshtastic_lockdown_set_session(session_s);
	ld.lock_reason = "ok";
	LOG_INF("Lockdown: unlocked via token (%u boots remaining%s)", ld.boots_remaining,
		session_s != 0U ? ", session cap armed" : "");
	return true;
}

/* Reference bumpBootsSinceFailOnBoot. */
static void bump_boots_since_fail(void)
{
	uint8_t attempts, boots_since_fail;
	uint32_t last_fail_epoch;

	read_backoff(&attempts, &boots_since_fail, &last_fail_epoch);
	if (attempts == 0U || attempts == BACKOFF_MAX_ATTEMPTS) {
		return;
	}
	if (boots_since_fail < 255U) {
		boots_since_fail++;
	}
	(void)write_backoff(attempts, boots_since_fail, last_fail_epoch);
}

/* ---- public: state ------------------------------------------------------------- */

void meshtastic_lockdown_reset(void)
{
	zero(&ld, sizeof(ld));
	ld.lock_reason = "not_provisioned";
}

void meshtastic_lockdown_init(void)
{
	(void)psa_crypto_init();
	meshtastic_lockdown_reset();
	if (derive_ekek() < 0) {
		LOG_ERR("Lockdown: ephemeral KEK derivation failed");
		return;
	}
	ld.provisioned = dek_record_exists();
	if (!ld.provisioned) {
		return; /* inactive: the node behaves like stock */
	}
	bump_boots_since_fail();
	if (consume_token()) {
		return;
	}
	LOG_WRN("Lockdown: device LOCKED (%s)", ld.lock_reason);
}

bool meshtastic_lockdown_active(void)
{
	return ld.provisioned;
}

bool meshtastic_lockdown_unlocked(void)
{
	return ld.dek_ok;
}

const char *meshtastic_lockdown_lock_reason(void)
{
	return ld.lock_reason;
}

uint8_t meshtastic_lockdown_boots_remaining(void)
{
	return ld.boots_remaining;
}

uint32_t meshtastic_lockdown_valid_until(void)
{
	return ld.valid_until;
}

uint32_t meshtastic_lockdown_backoff_remaining(void)
{
	return ld.backoff_remaining_s;
}

/* ---- public: provision / unlock / lock ---------------------------------------- */

static uint8_t boots_or_default(uint8_t boots)
{
	return boots != 0U ? boots : (uint8_t)CONFIG_MESHTASTIC_LOCKDOWN_DEFAULT_BOOTS;
}

static uint32_t session_or_default(uint32_t session_s)
{
	return session_s != 0U ? session_s : (uint32_t)CONFIG_MESHTASTIC_LOCKDOWN_SESSION_DEFAULT_SECONDS;
}

int meshtastic_lockdown_provision(const uint8_t *passphrase, size_t len, uint8_t boots,
				  uint32_t valid_until, uint32_t session_s)
{
	int ret;

	if (passphrase == NULL || len == 0U || len > MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX) {
		return -EINVAL;
	}
	if (ld.provisioned) {
		return -EALREADY;
	}
	ret = derive_ekek();
	if (ret < 0) {
		return ret;
	}
	ret = random_bytes(ld.dek, LD_KEY_LEN);
	if (ret < 0) {
		return ret;
	}
	ret = derive_kek(passphrase, len);
	if (ret < 0) {
		zero(ld.dek, sizeof(ld.dek));
		return ret;
	}
	ret = write_dek();
	zero(ld.kek, sizeof(ld.kek));
	ld.kek_ok = false;
	if (ret < 0) {
		zero(ld.dek, sizeof(ld.dek));
		LOG_ERR("Lockdown: writing the wrapped DEK failed (%d)", ret);
		return -EIO;
	}
	ld.provisioned = true;
	ld.dek_ok = true;
	clear_backoff();
	boots = boots_or_default(boots);
	session_s = session_or_default(session_s);
	if (write_token(boots, valid_until, session_s) < 0) {
		LOG_WRN("Lockdown: token write failed after provisioning (unlocked this boot)");
	}
	ld.boots_remaining = boots;
	ld.valid_until = valid_until;
	meshtastic_lockdown_set_session(session_s);
	ld.lock_reason = "ok";
	LOG_INF("Lockdown: provisioned (%u boots, epoch %u, session %u s)", boots, valid_until,
		session_s);
	return 0;
}

int meshtastic_lockdown_unlock(const uint8_t *passphrase, size_t len, uint8_t boots,
			       uint32_t valid_until, uint32_t session_s)
{
	uint8_t attempts, boots_since_fail, reserved;
	uint32_t last_fail_epoch;
	int ret;

	if (passphrase == NULL || len == 0U || len > MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX) {
		return -EINVAL;
	}
	if (!ld.provisioned) {
		return -ENOENT;
	}
	ret = derive_ekek();
	if (ret < 0) {
		return ret;
	}

	/* Backoff gate (reference): the longest of the within-boot wait (uptime),
	 * the wall-clock wait (epoch, when a clock exists) and the reboot-counted
	 * wait (one boot ~ 5 s of the delay) still to run. */
	read_backoff(&attempts, &boots_since_fail, &last_fail_epoch);
	if (attempts > 0U) {
		uint32_t delay = backoff_delay(attempts);
		uint32_t remaining = 0U;
		uint32_t now = now_epoch();
		uint32_t boots_needed;

		if (ld.last_fail_ms != 0) {
			uint32_t elapsed = (uint32_t)((k_uptime_get() - ld.last_fail_ms) / 1000);

			if (elapsed < delay) {
				remaining = MAX(remaining, delay - elapsed);
			}
		}
		if (now != 0U && last_fail_epoch != 0U && now >= last_fail_epoch) {
			uint32_t elapsed = now - last_fail_epoch;

			if (elapsed < delay) {
				remaining = MAX(remaining, delay - elapsed);
			}
		}
		boots_needed = MIN(255U, (delay + 4U) / 5U);
		if (boots_needed == 0U) {
			boots_needed = 1U;
		}
		if (boots_since_fail < boots_needed) {
			remaining = MAX(remaining, (boots_needed - boots_since_fail) * 5U);
		}
		if (remaining > 0U) {
			ld.backoff_remaining_s = remaining;
			LOG_WRN("Lockdown: passphrase blocked by backoff (~%u s left)", remaining);
			return -EAGAIN;
		}
	}
	ld.backoff_remaining_s = 0U;

	ret = derive_kek(passphrase, len);
	if (ret < 0) {
		return ret;
	}
	/* Reserve the attempt BEFORE the check, so a power cut mid-check still counts. */
	reserved = attempts < BACKOFF_MAX_ATTEMPTS ? attempts + 1U : attempts;
	(void)write_backoff(reserved, 0U, now_epoch());

	ret = load_dek();
	zero(ld.kek, sizeof(ld.kek));
	ld.kek_ok = false;
	if (ret < 0) {
		ld.last_fail_ms = k_uptime_get();
		if (ld.last_fail_ms == 0) {
			ld.last_fail_ms = 1;
		}
		ld.backoff_remaining_s = backoff_delay(reserved);
		LOG_WRN("Lockdown: wrong passphrase (attempt %u, next in ~%u s)", reserved,
			ld.backoff_remaining_s);
		return -EACCES;
	}
	clear_backoff();
	boots = boots_or_default(boots);
	session_s = session_or_default(session_s);
	if (write_token(boots, valid_until, session_s) < 0) {
		LOG_WRN("Lockdown: token write failed after unlock (unlocked this boot)");
	}
	ld.dek_ok = true;
	ld.boots_remaining = boots;
	ld.valid_until = valid_until;
	meshtastic_lockdown_set_session(session_s);
	ld.lock_reason = "ok";
	LOG_INF("Lockdown: unlocked with passphrase");
	return 0;
}

void meshtastic_lockdown_lock_now(void)
{
	delete_token();
	zero(ld.dek, sizeof(ld.dek));
	zero(ld.kek, sizeof(ld.kek));
	ld.dek_ok = false;
	ld.kek_ok = false;
	ld.session_max_ms = 0U;
	ld.session_started_ms = 0;
	ld.boots_remaining = 0U;
	ld.valid_until = 0U;
	ld.lock_reason = "token_missing";
	LOG_INF("Lockdown: locked now (token deleted, keys zeroed)");
}

int meshtastic_lockdown_remove_artifacts(void)
{
	(void)settings_delete(LD_KEY_DEK);
	(void)settings_delete(LD_KEY_TOKEN);
	(void)settings_delete(LD_KEY_MONO);
	(void)settings_delete(LD_KEY_BACKOFF);
	meshtastic_lockdown_lock_now();
	ld.provisioned = false;
	ld.lock_reason = "not_provisioned";
	LOG_INF("Lockdown: artifacts removed, inactive");
	return 0;
}

/* ---- public: sealed records ----------------------------------------------------- */

bool meshtastic_lockdown_is_sealed(const void *buf, size_t len)
{
	return buf != NULL && len >= MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD &&
	       sys_get_le32(buf) == LD_MAGIC_SEAL;
}

/* AAD = label || name: a record cannot be replayed under another name. */
static size_t seal_aad(const char *name, uint8_t *aad, size_t cap)
{
	size_t nlen = strlen(name);

	nlen = MIN(nlen, cap - (sizeof(SEAL_LABEL) - 1U));
	memcpy(aad, SEAL_LABEL, sizeof(SEAL_LABEL) - 1U);
	memcpy(aad + sizeof(SEAL_LABEL) - 1U, name, nlen);
	return sizeof(SEAL_LABEL) - 1U + nlen;
}

int meshtastic_lockdown_seal(const char *name, const void *in, size_t len, void *out, size_t cap)
{
	uint8_t aad[96];
	uint8_t *o = out;
	size_t aad_len, out_len;
	int ret;

	if (name == NULL || (in == NULL && len != 0U) || out == NULL) {
		return -EINVAL;
	}
	if (!ld.provisioned) {
		return -ENOTSUP;
	}
	if (!ld.dek_ok) {
		return -EACCES;
	}
	if (cap < len + MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD) {
		return -EMSGSIZE;
	}
	sys_put_le32(LD_MAGIC_SEAL, o);
	ret = random_bytes(o + 4, LD_NONCE_LEN);
	if (ret < 0) {
		return ret;
	}
	aad_len = seal_aad(name, aad, sizeof(aad));
	ret = aead(true, ld.dek, o + 4, aad, aad_len, in, len, o + 4 + LD_NONCE_LEN,
		   cap - 4U - LD_NONCE_LEN, &out_len);
	if (ret < 0) {
		return ret;
	}
	return (int)(4U + LD_NONCE_LEN + out_len);
}

int meshtastic_lockdown_open(const char *name, const void *in, size_t len, void *out, size_t cap)
{
	uint8_t aad[96];
	const uint8_t *i = in;
	size_t aad_len, out_len;
	int ret;

	if (name == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (!meshtastic_lockdown_is_sealed(in, len)) {
		return -EBADMSG;
	}
	if (!ld.provisioned) {
		return -ENOTSUP;
	}
	if (!ld.dek_ok) {
		return -EACCES;
	}
	aad_len = seal_aad(name, aad, sizeof(aad));
	ret = aead(false, ld.dek, i + 4, aad, aad_len, i + 4 + LD_NONCE_LEN, len - 4U - LD_NONCE_LEN,
		   out, cap, &out_len);
	if (ret < 0) {
		return -EBADMSG;
	}
	return (int)out_len;
}

/* ---- public: session cap ---------------------------------------------------------- */

void meshtastic_lockdown_set_session(uint32_t max_s)
{
	ld.session_max_ms = max_s * 1000U;
	ld.session_started_ms = k_uptime_get();
}

bool meshtastic_lockdown_session_expired(void)
{
	if (ld.session_max_ms == 0U) {
		return false;
	}
	return (k_uptime_get() - ld.session_started_ms) > (int64_t)ld.session_max_ms;
}

uint8_t meshtastic_lockdown_consume_session_boot(void)
{
	uint8_t new_boots;

	if (ld.boots_remaining == 0U) {
		return 0U;
	}
	new_boots = ld.boots_remaining - 1U;
	if (new_boots == 0U) {
		delete_token();
	} else {
		(void)write_token(new_boots, ld.valid_until, ld.session_max_ms / 1000U);
	}
	ld.boots_remaining = new_boots;
	meshtastic_lockdown_set_session(ld.session_max_ms / 1000U);
	return new_boots;
}
