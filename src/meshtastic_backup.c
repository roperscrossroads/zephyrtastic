/* SPDX-License-Identifier: GPL-3.0
 *
 * Preferences backup/restore over the settings subsystem -- see the header.
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include "meshtastic_lockdown.h"
#include "meshtastic_backup.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_settings.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define BACKUP_SUBTREE "mtbackup"
#define BACKUP_META_KEY "meta"
#define BACKUP_VERSION  1U

/* What the load callback is being asked to do. A plain settings_load() from
 * elsewhere (the BLE host loads its bonds that way) visits this subtree too,
 * and must never apply a backup as a side effect. */
enum backup_mode {
	BACKUP_IDLE,
	BACKUP_PROBE,   /* only note whether the meta record is there */
	BACKUP_RESTORE, /* feed every record into the config store */
};

static struct {
	enum backup_mode mode;
	bool saw_meta;
	struct meshtastic_backup_meta meta;
	int err;
	uint32_t restored;
} st;

static int full_name(char *out, size_t cap, const char *name)
{
	int ret = snprintk(out, cap, BACKUP_SUBTREE "/%s", name);

	return (ret < 0 || ret >= (int)cap) ? -EINVAL : 0;
}

/* The LWW stamps are deliberately NOT copied: a restore is a fresh local write
 * and is stamped as one, so the cluster adopts it instead of out-voting it
 * with the stamps it already holds. */
static bool is_stamp(const char *name)
{
	return strncmp(name, "hlc/", 4U) == 0;
}

static int backup_copy_one(const char *name, const void *val, size_t val_len)
{
	char key[64];
	int ret;

	if (is_stamp(name)) {
		return 0;
	}
	ret = full_name(key, sizeof(key), name);
	if (ret < 0) {
		return ret;
	}
	return meshtastic_lockdown_save_one(key, val, val_len);
}

static int backup_delete_one(const char *name, const void *val, size_t val_len)
{
	char key[64];
	int ret;

	ARG_UNUSED(val);
	ARG_UNUSED(val_len);

	if (is_stamp(name)) {
		return 0;
	}
	ret = full_name(key, sizeof(key), name);
	if (ret < 0) {
		return ret;
	}
	/* A record that is not there is the outcome wanted. */
	(void)settings_delete(key);
	return 0;
}

int meshtastic_backup_save(void)
{
	struct meshtastic_backup_meta meta = {
		.version = BACKUP_VERSION,
		.timestamp = meshtastic_clock_valid() ? meshtastic_clock_now_epoch() : 0U,
	};
	int ret;

	ret = meshtastic_config_store_export(backup_copy_one);
	if (ret < 0) {
		LOG_ERR("Backup: copying the config store failed (%d)", ret);
		return ret;
	}
	ret = meshtastic_lockdown_save_one(BACKUP_SUBTREE "/" BACKUP_META_KEY, &meta, sizeof(meta));
	if (ret < 0) {
		LOG_ERR("Backup: writing the meta record failed (%d)", ret);
		return ret;
	}
	LOG_INF("Backup: preferences saved (epoch %u)", meta.timestamp);
	return 0;
}

static int backup_set_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MESHTASTIC_STORE_VALUE_MAX];
	ssize_t read;
	int ret;

	if (st.mode == BACKUP_IDLE) {
		return 0;
	}

	if (strcmp(key, BACKUP_META_KEY) == 0) {
		if (meshtastic_lockdown_read(BACKUP_SUBTREE "/" BACKUP_META_KEY, len, read_cb, cb_arg,
					     &st.meta, sizeof(st.meta)) == (ssize_t)sizeof(st.meta)) {
			st.saw_meta = true;
		}
		return 0;
	}

	if (st.mode != BACKUP_RESTORE) {
		return 0;
	}

	if (len > sizeof(buf) + MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD) {
		LOG_WRN("Backup: oversized record '%s' (%zu bytes) skipped", key, len);
		st.err = -EMSGSIZE;
		return 0;
	}
	{
		char full[64];

		(void)full_name(full, sizeof(full), key);
		read = meshtastic_lockdown_read(full, len, read_cb, cb_arg, buf, sizeof(buf));
	}
	if (read <= 0) {
		LOG_WRN("Backup: reading '%s' failed (%d)", key, (int)read);
		st.err = read == -EACCES ? -EACCES : -EIO;
		return 0;
	}
	len = (size_t)read;
	/* The config store's own load path: the same decode and version window a
	 * boot-time load gets. */
	ret = meshtastic_config_store_setting_set(key, buf, len);
	if (ret < 0) {
		LOG_WRN("Backup: record '%s' rejected by the store (%d)", key, ret);
		st.err = ret;
		return 0;
	}
	st.restored++;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtbackup, BACKUP_SUBTREE, NULL, backup_set_cb, NULL, NULL);

static int walk(enum backup_mode mode)
{
	int ret;

	memset(&st, 0, sizeof(st));
	st.mode = mode;
	ret = settings_load_subtree(BACKUP_SUBTREE);
	st.mode = BACKUP_IDLE;
	return ret;
}

bool meshtastic_backup_exists(struct meshtastic_backup_meta *meta)
{
	if (walk(BACKUP_PROBE) < 0 || !st.saw_meta) {
		return false;
	}
	if (meta != NULL) {
		*meta = st.meta;
	}
	return true;
}

int meshtastic_backup_restore(void)
{
	int ret;

	ret = walk(BACKUP_RESTORE);
	if (ret < 0) {
		LOG_ERR("Backup: loading the subtree failed (%d)", ret);
		return ret;
	}
	if (!st.saw_meta) {
		LOG_WRN("Backup: nothing to restore");
		return -ENOENT;
	}
	if (st.err != 0) {
		/* Some records landed before one failed: the store is now a mix.
		 * Say so; the caller's reboot-without-flush would drop it, and a
		 * flush would persist it -- the admin path refuses and does neither. */
		LOG_ERR("Backup: restore incomplete (%d) after %u records", st.err, st.restored);
		return st.err;
	}

	meshtastic_config_store_stamp_all_local();
	ret = meshtastic_config_store_apply_core();
	if (ret < 0) {
		LOG_WRN("Backup: restored config failed to apply (%d)", ret);
	}
	meshtastic_settings_schedule_save();
	LOG_INF("Backup: %u records restored (taken at epoch %u)", st.restored, st.meta.timestamp);
	return 0;
}

int meshtastic_backup_remove(void)
{
	int ret = meshtastic_config_store_export(backup_delete_one);

	(void)settings_delete(BACKUP_SUBTREE "/" BACKUP_META_KEY);
	if (ret == 0) {
		LOG_INF("Backup: removed");
	}
	return ret;
}
