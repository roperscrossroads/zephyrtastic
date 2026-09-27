/* SPDX-License-Identifier: GPL-3.0
 *
 * The bulk store: a second NVS instance for large, rebuildable tables (node
 * records, per-node satellites, the message store), kept apart from the settings
 * NVS that holds identity, keys and config (agents-2dk3).
 *
 * Why a second store rather than more settings records:
 *  - The XIAO's settings partition is 32 KB of internal flash; the node counts
 *    upstream keeps (120 records, 100 keys) would nearly fill it. The bulk
 *    partition lives on the external QSPI flash there, and in the idle part of the
 *    Heltec's 512 KB storage partition.
 *  - One settings record per node makes every boot walk the whole NVS once per
 *    subtree (~1 s per subtree on a V4 at 93 records). The bulk store holds
 *    tables as a few large blobs, so a restore is a handful of reads.
 *  - Losing it costs nothing that cannot be relearned from the mesh, so it can be
 *    reformatted on any doubt; the settings NVS is never touched.
 *
 * Every blob carries a CRC32 of its plaintext (the settings NVS's own global
 * CONFIG_NVS_DATA_CRC must never be turned on: it would change the on-flash
 * format of every existing settings record). Under lockdown a blob is sealed with
 * the name "mtbulk/<id>" as associated data, so it cannot be replayed under
 * another id.
 */

#ifndef MESHTASTIC_BULK_H_
#define MESHTASTIC_BULK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/* Largest blob a caller may store (plaintext). */
#define MESHTASTIC_BULK_BLOB_MAX 2048U

/* Ids are 16-bit NVS ids; 0xFF00 and up are the engine's own. */
#define MESHTASTIC_BULK_ID_USER_MAX 0xFEFFU

/* Mount the store, verify its magic, and reformat it if it is foreign, garbage
 * or from another layout. Idempotent. -ENODEV when the build has no partition. */
int meshtastic_bulk_init(void);
bool meshtastic_bulk_ready(void);

/* Write `len` bytes under `id`. -EACCES while lockdown holds the store locked. */
int meshtastic_bulk_write(uint16_t id, const void *data, size_t len);

/* Read the blob under `id` into `buf`. Returns its length; -ENOENT when absent;
 * -EBADMSG when its CRC or seal does not verify (treat as absent: the data is
 * rebuildable); -EACCES while locked. */
ssize_t meshtastic_bulk_read(uint16_t id, void *buf, size_t cap);

int meshtastic_bulk_delete(uint16_t id);

/* Erase every blob (factory reset, NodeDB reset). The store stays mounted. */
int meshtastic_bulk_wipe(void);

/* Layout of the mounted store, for `meshtastic storage` and tests. */
struct meshtastic_bulk_info {
	bool ready;
	uint16_t sector_count;
	uint32_t sector_size;
	int32_t free_bytes;
	uint32_t formats; /* reformats since boot (a foreign or damaged store) */
};
void meshtastic_bulk_info_get(struct meshtastic_bulk_info *out);

#if defined(CONFIG_ZTEST)
/* Tests only: forget the mount so the next init re-reads the flash (a reboot). */
void meshtastic_bulk_test_unmount(void);
#endif

#endif /* MESHTASTIC_BULK_H_ */
