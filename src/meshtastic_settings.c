/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "meshtastic_config_store.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_settings.h"
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
#include "meshtastic_storage.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define MESHTASTIC_SETTINGS_SUBTREE "meshtastic"

static void save_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_handler);
static int (*active_export_func)(const char *name, const void *val, size_t val_len);

/*
 * What each record held when it last reached (or came from) flash, so the
 * coalesced save writes only the records that changed (agents-pcs2.1).
 *
 * Keyed by the CRC of the unprefixed name, judged by the CRC and length of the
 * PLAINTEXT value: under lockdown the bytes on flash are re-sealed with a fresh
 * nonce on every write, so comparing them could never say "unchanged". Touched
 * only by the save path, the load callback and the wipe -- all on the settings
 * side of the store, none inside a flash operation -- so PSRAM is fine.
 */
#define TRACK_CAP CONFIG_MESHTASTIC_SETTINGS_SAVE_TRACK_ENTRIES

struct track_entry {
	uint32_t name_crc;
	uint32_t val_crc;
	uint16_t len;
	bool used;
};

#if TRACK_CAP > 0
static MESHTASTIC_EXT_RAM_BSS_ATTR struct track_entry track[TRACK_CAP];
#endif
static K_MUTEX_DEFINE(track_lock);
static uint32_t save_written;
static uint32_t save_skipped;

#if TRACK_CAP > 0
static uint32_t track_name_crc(const char *name)
{
	return crc32_ieee((const uint8_t *)name, strlen(name));
}
#endif

/* Remember @p name as holding @p val on flash. A table that is full leaves the
 * record untracked, which only means it is always written. */
static void track_note(const char *name, const void *val, size_t len)
{
#if TRACK_CAP > 0
	uint32_t nc = track_name_crc(name);
	struct track_entry *slot = NULL;

	k_mutex_lock(&track_lock, K_FOREVER);
	for (size_t i = 0; i < TRACK_CAP; i++) {
		if (track[i].used && track[i].name_crc == nc) {
			slot = &track[i];
			break;
		}
		if (!track[i].used && slot == NULL) {
			slot = &track[i];
		}
	}
	if (slot != NULL) {
		slot->name_crc = nc;
		slot->val_crc = crc32_ieee(val, len);
		slot->len = (uint16_t)len;
		slot->used = true;
	}
	k_mutex_unlock(&track_lock);
#else
	ARG_UNUSED(name);
	ARG_UNUSED(val);
	ARG_UNUSED(len);
#endif
}

/* True when flash is known to hold exactly @p val for @p name already. */
static bool track_unchanged(const char *name, const void *val, size_t len)
{
	bool same = false;
#if TRACK_CAP > 0
	uint32_t nc = track_name_crc(name);

	k_mutex_lock(&track_lock, K_FOREVER);
	for (size_t i = 0; i < TRACK_CAP; i++) {
		if (track[i].used && track[i].name_crc == nc) {
			same = track[i].len == len && track[i].val_crc == crc32_ieee(val, len);
			break;
		}
	}
	k_mutex_unlock(&track_lock);
#else
	ARG_UNUSED(name);
	ARG_UNUSED(val);
	ARG_UNUSED(len);
#endif
	return same;
}

static void track_clear(void)
{
#if TRACK_CAP > 0
	k_mutex_lock(&track_lock, K_FOREVER);
	memset(track, 0, sizeof(track));
	k_mutex_unlock(&track_lock);
#endif
}

static int settings_get_cb(const char *key, char *val, int val_len_max)
{
	return meshtastic_config_store_setting_get(key, val, (size_t)val_len_max);
}

static int settings_set_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MESHTASTIC_STORE_VALUE_MAX];
	char full_name[SETTINGS_MAX_NAME_LEN + SETTINGS_EXTRA_LEN + 1];
	ssize_t read;
	int ret;

	if (len > sizeof(buf) + MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD) {
		LOG_WRN("Ignoring oversized Meshtastic setting '%s' (%zu bytes)", key, len);
		return 0;
	}

	/* The record is bound to its full name (lockdown seals under it). */
	(void)snprintk(full_name, sizeof(full_name), MESHTASTIC_SETTINGS_SUBTREE "/%s", key);
	read = meshtastic_lockdown_read(full_name, len, read_cb, cb_arg, buf, sizeof(buf));
	if (read == -EACCES) {
		LOG_DBG("Meshtastic setting '%s' sealed and the store is locked: default kept", key);
		return 0;
	}
	if (read < 0) {
		LOG_WRN("Reading Meshtastic setting '%s' failed (%d)", key, (int)read);
		return 0;
	}
	len = (size_t)read;

	/* What flash holds now. Not under lockdown: a plaintext leftover there must
	 * still be re-sealed at the next save, so nothing is taken as already on
	 * flash until a save has written it under the current sealing. */
#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
	if (!meshtastic_lockdown_active())
#endif
	{
		track_note(key, buf, len);
	}

	ret = meshtastic_config_store_setting_set(key, buf, len);
	if (ret < 0) {
		LOG_WRN("Ignoring invalid Meshtastic setting '%s' (%d)", key, ret);
	}

	return 0;
}

static int settings_export_prefixed(const char *name, const void *val, size_t val_len)
{
	char full_name[SETTINGS_MAX_NAME_LEN + SETTINGS_EXTRA_LEN + 1];
	int ret;

	ret = snprintk(full_name, sizeof(full_name), MESHTASTIC_SETTINGS_SUBTREE "/%s", name);
	if (ret < 0 || ret >= sizeof(full_name)) {
		return -EINVAL;
	}

	ret = meshtastic_lockdown_export(active_export_func, full_name, val, val_len);
	if (ret == 0) {
		track_note(name, val, val_len);
	}
	return ret;
}

/* The coalesced save's writer: only what changed, and a tick off the CPU after
 * each real write. The save runs on the cooperative system workqueue, so
 * without the sleep a save that does write (a sealed store, an NVS garbage
 * collection) holds every other thread off for its whole length -- the LoRa
 * receive path and the BLE host among them. */
static int save_changed_one(const char *name, const void *val, size_t val_len)
{
	char full_name[SETTINGS_MAX_NAME_LEN + SETTINGS_EXTRA_LEN + 1];
	int ret;

	if (track_unchanged(name, val, val_len)) {
		save_skipped++;
		return 0;
	}

	ret = snprintk(full_name, sizeof(full_name), MESHTASTIC_SETTINGS_SUBTREE "/%s", name);
	if (ret < 0 || ret >= sizeof(full_name)) {
		return -EINVAL;
	}

	ret = meshtastic_lockdown_save_one(full_name, val, val_len);
	if (ret == 0) {
		track_note(name, val, val_len);
		save_written++;
		k_sleep(K_TICKS(1));
	}
	return ret;
}

static int settings_export_cb(int (*export_func)(const char *name, const void *val, size_t val_len))
{
	int ret;

	active_export_func = export_func;
	ret = meshtastic_config_store_export(settings_export_prefixed);
	active_export_func = NULL;

	return ret;
}

/* After every load of the subtree -- the boot load, and the FULL settings_load()
 * the BLE bring-up does later (meshtastic_ble_init), which re-sets every record
 * from NVS over what apply_core() seeded in RAM. Re-reconciling here is what
 * keeps a seed in place; a load that changed nothing finds nothing to do. */
static int settings_commit_cb(void)
{
	meshtastic_config_store_reconcile_seeds();
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(meshtastic, MESHTASTIC_SETTINGS_SUBTREE, settings_get_cb,
			       settings_set_cb, settings_commit_cb, settings_export_cb);

static void save_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	/*
	 * A save already queued (within the debounce window) before an admin
	 * begin_edit_settings opened would otherwise fire right here, exporting
	 * a partially-edited store to flash and defeating the transaction's
	 * atomicity. meshtastic_config_store_set_save_suppressed() only stops
	 * NEW saves from being scheduled -- this is the other half. commit
	 * (or a future idle-timeout auto-commit) does the real flush once the
	 * transaction actually closes, so skipping here loses nothing.
	 */
	if (meshtastic_config_store_save_suppressed()) {
		LOG_DBG("Meshtastic settings save skipped (edit transaction open)");
		return;
	}

	{
		uint32_t w0 = save_written, s0 = save_skipped;

		ret = meshtastic_config_store_export(save_changed_one);
		LOG_DBG("Meshtastic settings save: %u written, %u unchanged",
			save_written - w0, save_skipped - s0);
	}
	if (ret < 0) {
		LOG_WRN("Meshtastic settings save failed (%d)", ret);
	}
}

int meshtastic_settings_init(void)
{
	int ret;

	ret = settings_subsys_init();
	if (ret < 0) {
		LOG_ERR("settings_subsys_init failed (%d)", ret);
		return ret;
	}

	{
		uint32_t t0 = k_cycle_get_32();

		ret = settings_load_subtree(MESHTASTIC_SETTINGS_SUBTREE);
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
		meshtastic_storage_note_load(MESHTASTIC_STORAGE_LOAD_CONFIG,
					     k_cycle_get_32() - t0);
#else
		ARG_UNUSED(t0);
#endif
	}
	if (ret < 0) {
		LOG_ERR("Meshtastic settings load failed (%d)", ret);
		return ret;
	}

	return 0;
}

void meshtastic_settings_schedule_save(void)
{
	(void)k_work_reschedule(&save_work, K_MSEC(CONFIG_MESHTASTIC_SETTINGS_SAVE_DELAY_MS));
}

bool meshtastic_settings_save_pending(void)
{
	return k_work_delayable_is_pending(&save_work);
}

int meshtastic_settings_flush(void)
{
	(void)k_work_cancel_delayable(&save_work);

	return settings_save_subtree(MESHTASTIC_SETTINGS_SUBTREE);
}

static bool wipe_preserve_security;

/* Delete one persisted key. Reuses the config-store key iteration (same as the
 * export path) so we never duplicate the key-name list: @p name is the unprefixed
 * store key ("owner", "channel/N", "config/<sec>", "module/<sec>"). */
static int settings_wipe_one(const char *name, const void *val, size_t val_len)
{
	char full_name[SETTINGS_MAX_NAME_LEN + SETTINGS_EXTRA_LEN + 1];
	int ret;

	ARG_UNUSED(val);
	ARG_UNUSED(val_len);

	/* Keep the X25519 identity on a config-only factory reset — and its LWW
	 * stamp with it. Dropping the stamp while keeping the value would leave the
	 * identity looking unversioned, so the next peer to gossip a security
	 * section would win and overwrite the one record this reset exists to save. */
	if (wipe_preserve_security && (strcmp(name, "config/security") == 0 ||
				       strcmp(name, "hlc/config/security") == 0)) {
		return 0;
	}

	ret = snprintk(full_name, sizeof(full_name), MESHTASTIC_SETTINGS_SUBTREE "/%s", name);
	if (ret < 0 || ret >= (int)sizeof(full_name)) {
		return -EINVAL;
	}

	return settings_delete(full_name);
}

int meshtastic_settings_wipe(bool preserve_security)
{
	int ret;

	/* Cancel any pending save so it can't re-persist the still-populated in-RAM
	 * store after we delete. The caller reboots without a flush. */
	(void)k_work_cancel_delayable(&save_work);

	wipe_preserve_security = preserve_security;
	ret = meshtastic_config_store_export(settings_wipe_one);
	wipe_preserve_security = false;
	track_clear();

	return ret;
}

void meshtastic_settings_save_stats(uint32_t *written, uint32_t *unchanged)
{
	if (written != NULL) {
		*written = save_written;
	}
	if (unchanged != NULL) {
		*unchanged = save_skipped;
	}
}
