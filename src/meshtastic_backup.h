/* SPDX-License-Identifier: GPL-3.0 */
#ifndef ZEPHYR_SUBSYS_MESHTASTIC_BACKUP_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_BACKUP_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Preferences backup and restore (AdminMessage backup_preferences /
 *        restore_preferences / remove_backup_preferences, agents-dnr4.14).
 *
 * The reference writes one BackupPreferences proto file to its filesystem. No
 * board this port runs on has one -- the config lives in NVS settings records
 * -- so the backup is a second copy of every config, module, channel and owner
 * record under the `mtbackup` settings subtree, one record each. A restore
 * feeds them back through the config store's own load path, stamps every
 * section as a fresh local write (a restore IS a write, and the cluster must
 * see it as newer than what it holds), re-applies the core config and asks
 * for a save; the admin path then reboots, as the reference does.
 */

/** Metadata record kept beside the copies. */
struct meshtastic_backup_meta {
	uint8_t version;
	uint32_t timestamp; /**< epoch seconds when the backup was taken, 0 if no clock */
};

/** @brief Mirror the current config store into the backup subtree. */
int meshtastic_backup_save(void);

/**
 * @brief Load the backup over the live config store.
 * @retval 0       Restored; core config re-applied, save scheduled.
 * @retval -ENOENT There is no backup.
 */
int meshtastic_backup_restore(void);

/** @brief Delete the backup subtree. Not an error when there is none. */
int meshtastic_backup_remove(void);

/** @brief Whether a backup exists; fills @p meta when it does and @p meta is non-NULL. */
bool meshtastic_backup_exists(struct meshtastic_backup_meta *meta);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_BACKUP_H_ */
