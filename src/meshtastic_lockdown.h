/* SPDX-License-Identifier: GPL-3.0 */
#ifndef ZEPHYR_SUBSYS_MESHTASTIC_LOCKDOWN_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_LOCKDOWN_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Lockdown storage core (agents-dnr4.15, phase 1): the reference's
 *        EncryptedStorage over Zephyr settings records and PSA.
 *
 * Key hierarchy, as the reference:
 *   ephemeral KEK = SHA-256(device id || domain)[:16]        device-bound, no secret
 *   KEK           = PBKDF2-HMAC-SHA256(passphrase, device id || domain)[:16]  never stored
 *   DEK           = 16 random bytes, wrapped by the KEK in `mtlock/dek`
 *   unlock token  = DEK wrapped by the ephemeral KEK plus boots / epoch / session cap /
 *                   monotonic counter, in `mtlock/token`
 *   `mtlock/mono` highest counter ever issued; `mtlock/backoff` failed-attempt state.
 * Every artifact and every sealed record is AES-128-GCM with the record's name (or
 * an artifact label) as additional data, so a record cannot be replayed under
 * another name.
 *
 * States: INACTIVE (never provisioned or disabled: the node behaves like stock),
 * LOCKED (provisioned, DEK not in RAM), UNLOCKED (DEK in RAM).
 */

#define MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX 32U
#define MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD  32U /* magic 4 + nonce 12 + tag 16 */

/** @brief Boot-time init: derive the ephemeral KEK, bump the backoff boot counter,
 *         try the unlock token. Call before the settings load. Never fails the boot. */
void meshtastic_lockdown_init(void);

/** @brief Forget every key and all RAM state (a simulated reboot; the tests' reset). */
void meshtastic_lockdown_reset(void);

bool meshtastic_lockdown_active(void);   /**< a passphrase is provisioned */
bool meshtastic_lockdown_unlocked(void); /**< the DEK is in RAM */
const char *meshtastic_lockdown_lock_reason(void);
uint8_t meshtastic_lockdown_boots_remaining(void);
uint32_t meshtastic_lockdown_valid_until(void);
uint32_t meshtastic_lockdown_backoff_remaining(void);

/**
 * @brief First-time provisioning: set the passphrase, generate a DEK, write the
 *        wrapped DEK and a token. Leaves the node UNLOCKED.
 * @param boots 0 = Kconfig default; @param valid_until 0 = no epoch limit;
 * @param session_s 0 = Kconfig default (0 = no cap).
 * @retval -EINVAL bad passphrase length, -EALREADY already provisioned, -EIO storage/crypto.
 */
int meshtastic_lockdown_provision(const uint8_t *passphrase, size_t len, uint8_t boots,
				  uint32_t valid_until, uint32_t session_s);

/**
 * @brief Unlock (or re-verify) with the passphrase and issue a fresh token.
 * @retval 0 unlocked; -EACCES wrong passphrase; -EAGAIN blocked by backoff
 *         (see meshtastic_lockdown_backoff_remaining()); -ENOENT not provisioned.
 */
int meshtastic_lockdown_unlock(const uint8_t *passphrase, size_t len, uint8_t boots,
			       uint32_t valid_until, uint32_t session_s);

/** @brief Delete the token and zero every key: the next boot needs the passphrase. */
void meshtastic_lockdown_lock_now(void);

/** @brief Disable's artifact half: delete dek/token/mono/backoff, zero keys, INACTIVE. */
int meshtastic_lockdown_remove_artifacts(void);

/**
 * @brief Seal a record for storage under the DEK (name is authenticated).
 * @retval >=0 sealed length; -ENOTSUP inactive (store plaintext); -EACCES locked.
 */
/* ---- phase 2: the record wrap over the settings store --------------------------
 *
 * Every persisted record of a wrapped subtree goes out through one of the two
 * write helpers and comes back through the read helper. Inactive (no
 * passphrase): plaintext, exactly as before. Active and unlocked: sealed on
 * the way out, opened on the way in; a plaintext record found on an active
 * device is accepted (a pre-provisioning leftover) and sealed at its next
 * save. Active and locked, or unlocked but not yet reloaded: writes are
 * refused (-EACCES) so nothing can go over the encrypted store, and sealed
 * records cannot be opened (-EACCES) so the loader keeps its defaults -- the
 * reference's locked-boot placeholders. Callers treat -EACCES as "skip
 * quietly". With CONFIG_MESHTASTIC_LOCKDOWN=n these are plain passthroughs. */
#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
#include <sys/types.h>
#include <zephyr/settings/settings.h>

/** The store may be read/written in the clear by this build right now. */
bool meshtastic_lockdown_store_ready(void);
/** Provisioned and the DEK is not in RAM: the boot is a locked boot. */
bool meshtastic_lockdown_locked(void);

int meshtastic_lockdown_export(int (*export_func)(const char *name, const void *val,
						  size_t val_len),
			       const char *name, const void *val, size_t len);
int meshtastic_lockdown_save_one(const char *name, const void *val, size_t len);
ssize_t meshtastic_lockdown_read(const char *name, size_t len, settings_read_cb read_cb,
				 void *cb_arg, void *buf, size_t cap);

/** The stack's reload after a deferred unlock (settings re-read, config
 *  re-applied, radio released). Registered by meshtastic_init; runs on the
 *  system workqueue, never on the transport thread that verified the passphrase. */
void meshtastic_lockdown_set_reload_hook(void (*hook)(void));
/** Verify the passphrase and, on the workqueue, rewrite every wrapped record
 *  in the clear, remove the artifacts and leave the device stock. The caller
 *  reboots afterwards, as the reference does. */
int meshtastic_lockdown_disable(const uint8_t *passphrase, size_t len);
/** The disable's plaintext rewrite finished (for callers that wait on it). */
bool meshtastic_lockdown_disable_done(void);
/** A reload or a seal/unseal rewrite is still queued on the workqueue. */
bool meshtastic_lockdown_busy(void);
#else
#include <sys/types.h>
#include <zephyr/settings/settings.h>

static inline bool meshtastic_lockdown_store_ready(void) { return true; }
static inline bool meshtastic_lockdown_locked(void) { return false; }
static inline int meshtastic_lockdown_export(int (*export_func)(const char *name,
							       const void *val, size_t val_len),
					     const char *name, const void *val, size_t len)
{
	return export_func(name, val, len);
}
static inline int meshtastic_lockdown_save_one(const char *name, const void *val, size_t len)
{
	return settings_save_one(name, val, len);
}
static inline ssize_t meshtastic_lockdown_read(const char *name, size_t len,
					       settings_read_cb read_cb, void *cb_arg, void *buf,
					       size_t cap)
{
	(void)name;
	if (len > cap) {
		return -EMSGSIZE;
	}
	return read_cb(cb_arg, buf, len);
}
#endif

int meshtastic_lockdown_seal(const char *name, const void *in, size_t len, void *out,
			     size_t cap);

/**
 * @brief Open a sealed record.
 * @retval >=0 plaintext length; -EBADMSG not a sealed record (or tampered);
 *         -EACCES sealed but locked; -ENOTSUP inactive.
 */
int meshtastic_lockdown_open(const char *name, const void *in, size_t len, void *out,
			     size_t cap);

/** @brief Whether a stored blob carries the sealed-record magic. */
bool meshtastic_lockdown_is_sealed(const void *buf, size_t len);

/* Session cap (reference setSession / isSessionExpired / consumeSessionBoot). */
void meshtastic_lockdown_set_session(uint32_t max_s);
bool meshtastic_lockdown_session_expired(void);
uint8_t meshtastic_lockdown_consume_session_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_LOCKDOWN_H_ */
