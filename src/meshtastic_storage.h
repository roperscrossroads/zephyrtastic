/* SPDX-License-Identifier: GPL-3.0
 *
 * Storage report: how full the settings NVS is, and who is using it.
 *
 * Read-only. Written because nothing reported settings use, while the XIAO's
 * 32 KB settings partition (28.5 KB usable) is shared by config, node keys,
 * node records, the cluster document, lockdown, bootlog, backup, fleet and BLE
 * bonds, and every plan to store more (NodeDB growth, a message store) has to
 * start from a measured number, not an estimate (agents-oaz1).
 */

#ifndef MESHTASTIC_STORAGE_H_
#define MESHTASTIC_STORAGE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MESHTASTIC_STORAGE_MAX_SUBTREES 16
#define MESHTASTIC_STORAGE_NAME_LEN     12

/* One settings subtree: everything before the first '/' of a key. */
struct meshtastic_storage_subtree {
	char name[MESHTASTIC_STORAGE_NAME_LEN];
	uint16_t records;
	uint16_t sealed;       /* values that carry a lockdown seal */
	uint32_t name_bytes;   /* key text, as stored */
	uint32_t value_bytes;  /* value payloads */
	uint32_t flash_bytes;  /* estimated on-flash cost: both NVS entries + ATEs */
};

/* Time spent restoring a table at boot, in microseconds (0 = not measured). */
enum meshtastic_storage_load {
	MESHTASTIC_STORAGE_LOAD_CONFIG = 0, /* meshtastic/ subtree */
	MESHTASTIC_STORAGE_LOAD_NODE_KEYS,  /* mtnode/ (warm tier) */
	MESHTASTIC_STORAGE_LOAD_NODE_RECS,  /* mtrec/ (hot store) */
	MESHTASTIC_STORAGE_LOAD_COUNT,
};

struct meshtastic_storage_stats {
	bool nvs;               /* false: the settings backend is not NVS */
	uint16_t sector_count;
	uint32_t sector_size;
	uint16_t write_sector;  /* sector the next write lands in */
	int32_t free_bytes;     /* nvs_calc_free_space(); negative errno on failure */
	uint32_t total_records;
	uint32_t total_flash_bytes;
	uint8_t subtree_count;
	bool subtrees_truncated; /* more subtrees than the table holds */
	struct meshtastic_storage_subtree subtree[MESHTASTIC_STORAGE_MAX_SUBTREES];
	uint32_t load_us[MESHTASTIC_STORAGE_LOAD_COUNT];
};

/* Fill *out. Walks every stored key and scans the NVS allocation table, so it
 * is O(records x ATEs): for the shell and tests, never a hot path. */
int meshtastic_storage_stats(struct meshtastic_storage_stats *out);

/* Record how long a boot-time restore took (cycles from k_cycle_get_32()). */
void meshtastic_storage_note_load(enum meshtastic_storage_load what, uint32_t cycles);

#if defined(CONFIG_SHELL)
struct shell;
int meshtastic_storage_shell_cmd(const struct shell *sh, size_t argc, char **argv);
#endif

#endif /* MESHTASTIC_STORAGE_H_ */
