/* SPDX-License-Identifier: GPL-3.0
 *
 * The bulk store — see meshtastic_bulk.h.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include "meshtastic_bulk.h"
#include "meshtastic_lockdown.h"

LOG_MODULE_REGISTER(mt_bulk, CONFIG_MESHTASTIC_LOG_LEVEL);

#define BULK_PARTITION PARTITION_ID(mt_bulk_partition)

/* The engine's own record: identifies a store written by this layout. A store
 * without it (fresh, foreign, or an older layout) is reformatted. */
#define BULK_ID_MAGIC      0xFF00U
#define BULK_MAGIC         0x4B42544DU /* "MTBK" */
#define BULK_LAYOUT        1U

/* Per-blob frame: tag, plaintext length, CRC32 of the plaintext, then the
 * plaintext as the lockdown wrap left it (verbatim, or sealed). */
#define BULK_FRAME_TAG     0x4B42U /* "BK" */
#define BULK_FRAME_HDR     8U

#define BULK_SCRATCH_LEN   (BULK_FRAME_HDR + MESHTASTIC_BULK_BLOB_MAX + \
			    MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD)

static struct {
	struct k_mutex lock;
	bool lock_ready;
	bool ready;
	uint32_t formats;
	const struct flash_area *fa;
	struct nvs_fs fs;
	/* Static, not on a stack: a frame is ~2 KB. On ESP32 this must stay in
	 * internal RAM (flash writes cannot source from PSRAM), which plain .bss is. */
	uint8_t scratch[BULK_SCRATCH_LEN];
} bulk;

static void bulk_lock(void)
{
	if (!bulk.lock_ready) {
		k_mutex_init(&bulk.lock);
		bulk.lock_ready = true;
	}
	k_mutex_lock(&bulk.lock, K_FOREVER);
}

static void bulk_unlock(void)
{
	k_mutex_unlock(&bulk.lock);
}

static int mount_locked(void)
{
	struct flash_pages_info info;
	int ret;

	memset(&bulk.fs, 0, sizeof(bulk.fs));
	bulk.fs.flash_device = bulk.fa->fa_dev;
	bulk.fs.offset = bulk.fa->fa_off;
	ret = flash_get_page_info_by_offs(bulk.fs.flash_device, bulk.fs.offset, &info);
	if (ret != 0) {
		return ret;
	}
	bulk.fs.sector_size = (uint32_t)info.size;
	bulk.fs.sector_count = (uint16_t)(bulk.fa->fa_size / info.size);
	return nvs_mount(&bulk.fs);
}

static bool magic_ok_locked(void)
{
	uint8_t m[8];

	if (nvs_read(&bulk.fs, BULK_ID_MAGIC, m, sizeof(m)) != (ssize_t)sizeof(m)) {
		return false;
	}
	return sys_get_le32(m) == BULK_MAGIC && sys_get_le16(m + 4) == BULK_LAYOUT &&
	       sys_get_le16(m + 6) == bulk.fs.sector_count;
}

/* Erase the whole partition and start a fresh store. Everything in it is
 * rebuildable, so this is the answer to any doubt about what is on the flash. */
static int format_locked(void)
{
	uint8_t m[8];
	int ret;

	ret = flash_area_erase(bulk.fa, 0, bulk.fa->fa_size);
	if (ret != 0) {
		return ret;
	}
	ret = mount_locked();
	if (ret != 0) {
		return ret;
	}
	sys_put_le32(BULK_MAGIC, m);
	sys_put_le16(BULK_LAYOUT, m + 4);
	sys_put_le16(bulk.fs.sector_count, m + 6);
	ret = nvs_write(&bulk.fs, BULK_ID_MAGIC, m, sizeof(m));
	if (ret < 0) {
		return ret;
	}
	bulk.formats++;
	return 0;
}

int meshtastic_bulk_init(void)
{
	int ret;

	bulk_lock();
	if (bulk.ready) {
		bulk_unlock();
		return 0;
	}
	if (bulk.fa == NULL) {
		ret = flash_area_open(BULK_PARTITION, &bulk.fa);
		if (ret != 0) {
			bulk_unlock();
			LOG_ERR("bulk: partition open failed (%d)", ret);
			return ret;
		}
	}

	ret = mount_locked();
	if (ret != 0 || !magic_ok_locked()) {
		LOG_WRN("bulk: %s store; formatting",
			ret != 0 ? "unmountable" : "foreign or empty");
		ret = format_locked();
		if (ret != 0) {
			bulk_unlock();
			LOG_ERR("bulk: format failed (%d)", ret);
			return ret;
		}
	}
	bulk.ready = true;
	bulk_unlock();
	return 0;
}

bool meshtastic_bulk_ready(void)
{
	return bulk.ready;
}

static void blob_name(uint16_t id, char *name, size_t cap)
{
	(void)snprintk(name, cap, "mtbulk/%04x", id);
}

int meshtastic_bulk_write(uint16_t id, const void *data, size_t len)
{
	char name[16];
	int n;
	ssize_t w;

	if (id > MESHTASTIC_BULK_ID_USER_MAX || (data == NULL && len != 0U)) {
		return -EINVAL;
	}
	if (len > MESHTASTIC_BULK_BLOB_MAX) {
		return -EMSGSIZE;
	}
	bulk_lock();
	if (!bulk.ready) {
		bulk_unlock();
		return -ENODEV;
	}
	blob_name(id, name, sizeof(name));
	n = meshtastic_lockdown_wrap_buf(name, data, len, bulk.scratch + BULK_FRAME_HDR,
					 sizeof(bulk.scratch) - BULK_FRAME_HDR);
	if (n < 0) {
		bulk_unlock();
		return n;
	}
	sys_put_le16(BULK_FRAME_TAG, bulk.scratch);
	sys_put_le16((uint16_t)len, bulk.scratch + 2);
	sys_put_le32(crc32_ieee(data, len), bulk.scratch + 4);
	/* NVS skips the write when the stored bytes are identical, so an unchanged
	 * plaintext table costs nothing -- unless lockdown sealed it with a fresh
	 * nonce; callers keep their own per-blob dirty state for that. */
	w = nvs_write(&bulk.fs, id, bulk.scratch, BULK_FRAME_HDR + (size_t)n);
	bulk_unlock();
	return (w < 0) ? (int)w : 0;
}

ssize_t meshtastic_bulk_read(uint16_t id, void *buf, size_t cap)
{
	char name[16];
	ssize_t got;
	size_t plain_len;
	int n;

	if (id > MESHTASTIC_BULK_ID_USER_MAX || buf == NULL) {
		return -EINVAL;
	}
	bulk_lock();
	if (!bulk.ready) {
		bulk_unlock();
		return -ENODEV;
	}
	got = nvs_read(&bulk.fs, id, bulk.scratch, sizeof(bulk.scratch));
	if (got < 0) {
		bulk_unlock();
		return (got == -ENOENT) ? -ENOENT : got;
	}
	if ((size_t)got > sizeof(bulk.scratch) || got < (ssize_t)BULK_FRAME_HDR ||
	    sys_get_le16(bulk.scratch) != BULK_FRAME_TAG) {
		bulk_unlock();
		return -EBADMSG;
	}
	plain_len = sys_get_le16(bulk.scratch + 2);
	if (plain_len > cap) {
		bulk_unlock();
		return -EMSGSIZE;
	}
	blob_name(id, name, sizeof(name));
	n = meshtastic_lockdown_unwrap_buf(name, bulk.scratch + BULK_FRAME_HDR,
					   (size_t)got - BULK_FRAME_HDR, buf, cap);
	if (n < 0) {
		bulk_unlock();
		return n;
	}
	if ((size_t)n != plain_len || crc32_ieee(buf, plain_len) != sys_get_le32(bulk.scratch + 4)) {
		bulk_unlock();
		return -EBADMSG;
	}
	bulk_unlock();
	return (ssize_t)plain_len;
}

int meshtastic_bulk_delete(uint16_t id)
{
	int ret;

	if (id > MESHTASTIC_BULK_ID_USER_MAX) {
		return -EINVAL;
	}
	bulk_lock();
	ret = bulk.ready ? nvs_delete(&bulk.fs, id) : -ENODEV;
	bulk_unlock();
	return ret;
}

int meshtastic_bulk_wipe(void)
{
	int ret;

	bulk_lock();
	if (bulk.fa == NULL) {
		bulk_unlock();
		return -ENODEV;
	}
	ret = format_locked();
	bulk.ready = (ret == 0);
	bulk_unlock();
	return ret;
}

void meshtastic_bulk_info_get(struct meshtastic_bulk_info *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	bulk_lock();
	out->ready = bulk.ready;
	out->formats = bulk.formats;
	if (bulk.ready) {
		out->sector_count = bulk.fs.sector_count;
		out->sector_size = bulk.fs.sector_size;
		out->free_bytes = (int32_t)nvs_calc_free_space(&bulk.fs);
	}
	bulk_unlock();
}

#if defined(CONFIG_ZTEST)
void meshtastic_bulk_test_unmount(void)
{
	bulk_lock();
	bulk.ready = false;
	bulk_unlock();
}
#endif
