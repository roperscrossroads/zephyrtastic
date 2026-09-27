/* SPDX-License-Identifier: GPL-3.0
 *
 * The bulk store (agents-2dk3 Phase 3a): a second NVS on its own partition for
 * large, rebuildable tables, apart from the settings NVS. Real NVS on the flash
 * simulator; the whole stack is up so lockdown sealing is the real code.
 *
 * Test names carry a letter prefix: ztest runs in name order and the store's
 * state (and lockdown's) carries from one test to the next.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/ztest.h>
#include <psa/crypto.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_bulk.h"
#include "meshtastic_core.h"
#include "meshtastic_lockdown.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const uint8_t PP[] = "correct horse";
#define PPLEN (sizeof(PP) - 1U)

static struct meshtastic_config cfg = {
	.node_id = 0x11223344U,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "bulk test",
	.short_name = "bulk",
};

static uint8_t blob[MESHTASTIC_BULK_BLOB_MAX];
static uint8_t back[MESHTASTIC_BULK_BLOB_MAX];

static void fill(uint8_t *p, size_t n, uint8_t seed)
{
	for (size_t i = 0U; i < n; i++) {
		p[i] = (uint8_t)(seed + i * 7U);
	}
}

static void wait_idle(void)
{
	for (int i = 0; i < 200 && meshtastic_lockdown_busy(); i++) {
		k_sleep(K_MSEC(20));
	}
	k_sleep(K_MSEC(300));
}

/* Peek at what is physically stored under `id`: a private, read-only mount of
 * the same partition, the way a thief with the flash chip would see it. */
static ssize_t raw_peek(uint16_t id, uint8_t *buf, size_t cap)
{
	static struct nvs_fs fs;
	const struct flash_area *fa;
	struct flash_pages_info info;

	zassert_ok(flash_area_open(PARTITION_ID(mt_bulk_partition), &fa), "");
	memset(&fs, 0, sizeof(fs));
	fs.flash_device = fa->fa_dev;
	fs.offset = fa->fa_off;
	zassert_ok(flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info), "");
	fs.sector_size = (uint32_t)info.size;
	fs.sector_count = (uint16_t)(fa->fa_size / info.size);
	zassert_ok(nvs_mount(&fs), "raw mount");
	return nvs_read(&fs, id, buf, cap);
}

static bool contains(const uint8_t *hay, size_t n, const uint8_t *needle, size_t m)
{
	for (size_t i = 0U; i + m <= n; i++) {
		if (memcmp(hay + i, needle, m) == 0) {
			return true;
		}
	}
	return false;
}

static void *suite_setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa");
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	return NULL;
}

ZTEST_SUITE(bulk_store, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(bulk_store, test_a_first_mount_formats_a_fresh_partition)
{
	struct meshtastic_bulk_info info;

	zassert_ok(meshtastic_bulk_init(), "init");
	zassert_true(meshtastic_bulk_ready(), "");
	meshtastic_bulk_info_get(&info);
	zassert_true(info.ready, "");
	zassert_equal(info.formats, 1U, "an erased partition has no magic: formatted once");
	zassert_equal(info.sector_count, 64U, "256 KB in 4 KB sectors");
	zassert_true(info.free_bytes > 200000, "nearly all free (%d)", info.free_bytes);

	/* A remount (a reboot) finds its own magic and does not format again. */
	meshtastic_bulk_test_unmount();
	zassert_ok(meshtastic_bulk_init(), "re-init");
	meshtastic_bulk_info_get(&info);
	zassert_equal(info.formats, 1U, "our own store is kept");
}

ZTEST(bulk_store, test_b_round_trip_overwrite_and_delete)
{
	ssize_t n;

	fill(blob, sizeof(blob), 1U);
	zassert_ok(meshtastic_bulk_write(0x0101U, blob, sizeof(blob)), "full-size write");
	memset(back, 0, sizeof(back));
	n = meshtastic_bulk_read(0x0101U, back, sizeof(back));
	zassert_equal(n, (ssize_t)sizeof(blob), "length back (%d)", (int)n);
	zassert_mem_equal(back, blob, sizeof(blob), "bytes back");

	fill(blob, 100U, 9U);
	zassert_ok(meshtastic_bulk_write(0x0101U, blob, 100U), "overwrite shorter");
	n = meshtastic_bulk_read(0x0101U, back, sizeof(back));
	zassert_equal(n, 100, "");
	zassert_mem_equal(back, blob, 100U, "");

	/* It survives a remount. */
	meshtastic_bulk_test_unmount();
	zassert_ok(meshtastic_bulk_init(), "");
	n = meshtastic_bulk_read(0x0101U, back, sizeof(back));
	zassert_equal(n, 100, "persisted across a remount");
	zassert_mem_equal(back, blob, 100U, "");

	zassert_ok(meshtastic_bulk_delete(0x0101U), "delete");
	zassert_equal(meshtastic_bulk_read(0x0101U, back, sizeof(back)), -ENOENT, "gone");
}

ZTEST(bulk_store, test_c_an_identical_write_costs_no_flash)
{
	struct meshtastic_bulk_info before, after;

	fill(blob, 500U, 3U);
	zassert_ok(meshtastic_bulk_write(0x0202U, blob, 500U), "");
	meshtastic_bulk_info_get(&before);
	for (int i = 0; i < 10; i++) {
		zassert_ok(meshtastic_bulk_write(0x0202U, blob, 500U), "");
	}
	meshtastic_bulk_info_get(&after);
	zassert_equal(after.free_bytes, before.free_bytes,
		      "rewriting unchanged plaintext must not consume flash");
}

ZTEST(bulk_store, test_d_bad_arguments_are_refused)
{
	zassert_equal(meshtastic_bulk_write(0x0303U, blob, MESHTASTIC_BULK_BLOB_MAX + 1U),
		      -EMSGSIZE, "");
	zassert_equal(meshtastic_bulk_write(0xFF00U, blob, 4U), -EINVAL, "engine ids are reserved");
	zassert_equal(meshtastic_bulk_read(0xFF00U, back, sizeof(back)), -EINVAL, "");
	fill(blob, 64U, 5U);
	zassert_ok(meshtastic_bulk_write(0x0303U, blob, 64U), "");
	zassert_equal(meshtastic_bulk_read(0x0303U, back, 10U), -EMSGSIZE, "buffer too small");
}

ZTEST(bulk_store, test_e_sealed_under_lockdown_and_refused_while_locked)
{
	static uint8_t raw[MESHTASTIC_BULK_BLOB_MAX + 64U];
	static const uint8_t secret[] = "THE-PLAINTEXT-MARKER-0123456789";
	ssize_t n;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "provision");
	wait_idle();

	zassert_ok(meshtastic_bulk_write(0x0404U, secret, sizeof(secret)), "sealed write");
	n = raw_peek(0x0404U, raw, sizeof(raw));
	zassert_true(n > (ssize_t)sizeof(secret), "stored with frame + seal (%d)", (int)n);
	zassert_false(contains(raw, (size_t)n, secret, sizeof(secret) - 1U),
		      "the plaintext must not be on the flash");
	n = meshtastic_bulk_read(0x0404U, back, sizeof(back));
	zassert_equal(n, (ssize_t)sizeof(secret), "opens with the DEK");
	zassert_mem_equal(back, secret, sizeof(secret), "");

	meshtastic_lockdown_lock_now();
	zassert_equal(meshtastic_bulk_read(0x0404U, back, sizeof(back)), -EACCES,
		      "locked: unreadable");
	zassert_equal(meshtastic_bulk_write(0x0404U, secret, sizeof(secret)), -EACCES,
		      "locked: no placeholder may be written");

	zassert_ok(meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U), "unlock");
	wait_idle();
	n = meshtastic_bulk_read(0x0404U, back, sizeof(back));
	zassert_equal(n, (ssize_t)sizeof(secret), "readable again after unlock");

	zassert_ok(meshtastic_lockdown_disable(PP, PPLEN), "disable");
	wait_idle();
}

ZTEST(bulk_store, test_f_a_garbage_partition_is_reformatted)
{
	static uint8_t junk[4096];
	const struct flash_area *fa;
	struct meshtastic_bulk_info info;

	meshtastic_bulk_info_get(&info);
	zassert_equal(info.formats, 1U, "precondition");

	/* Scribble over the first sectors, as leftovers from another layout would. */
	zassert_ok(flash_area_open(PARTITION_ID(mt_bulk_partition), &fa), "");
	for (size_t i = 0U; i < sizeof(junk); i++) {
		junk[i] = (uint8_t)(i * 31U + 0x5AU);
	}
	meshtastic_bulk_test_unmount();
	zassert_ok(flash_area_erase(fa, 0, 3U * 4096U), "");
	for (off_t off = 0; off < 3 * 4096; off += (off_t)sizeof(junk)) {
		zassert_ok(flash_area_write(fa, off, junk, sizeof(junk)), "");
	}

	zassert_ok(meshtastic_bulk_init(), "init over garbage");
	meshtastic_bulk_info_get(&info);
	zassert_true(info.ready, "");
	zassert_equal(info.formats, 2U, "garbage means reformat");
	zassert_equal(meshtastic_bulk_read(0x0202U, back, sizeof(back)), -ENOENT,
		      "a reformatted store is empty");
	fill(blob, 32U, 7U);
	zassert_ok(meshtastic_bulk_write(0x0505U, blob, 32U), "usable after the reformat");
}
