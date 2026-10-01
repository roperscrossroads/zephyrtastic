/* SPDX-License-Identifier: GPL-3.0
 *
 * agents-pcs2.1: the coalesced settings save writes only the records whose
 * plaintext changed, and the NVS caches are on.
 *
 * On hardware the old save rewrote all ~66 records of the "meshtastic" subtree
 * after any one setting changed, with every name lookup scanning NVS: a 17-27 s
 * freeze on an ESP32 node (LoRa, BLE and shell all stopped), ~6 s on a XIAO.
 * The caches fix the lookup cost; the tracking here fixes the write count,
 * which matters most under lockdown, where a re-sealed record never reads back
 * identical and NVS cannot skip it (that half is proven in lockdown_store).
 *
 * The same cases run three ways (testcase.yaml): tracked, untracked (every
 * save writes everything, as before) and with a table far smaller than the
 * subtree. Only the write COUNT differs; what reaches flash must not.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_settings.h"

#define TRACK CONFIG_MESHTASTIC_SETTINGS_SAVE_TRACK_ENTRIES

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static struct meshtastic_config cfg = {
	.node_id = 0x11223344U,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "save test",
	.short_name = "save",
};

struct raw {
	uint8_t buf[320];
	size_t len;
	bool found;
};

static int raw_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param)
{
	struct raw *r = param;

	ARG_UNUSED(key);
	if (len <= sizeof(r->buf) && read_cb(cb_arg, r->buf, len) == (ssize_t)len) {
		r->len = len;
		r->found = true;
	}
	return 1;
}

static bool raw_read(const char *name, struct raw *r)
{
	r->len = 0U;
	r->found = false;
	(void)settings_load_subtree_direct(name, raw_cb, r);
	return r->found;
}

static uint32_t written(void)
{
	uint32_t w;

	meshtastic_settings_save_stats(&w, NULL);
	return w;
}

/* Until the coalesced save has fired and finished: nothing pending, and the
 * write counter still for a while (each real write sleeps a tick). */
static void wait_saved(void)
{
	uint32_t last;

	for (int i = 0; i < 100 && meshtastic_settings_save_pending(); i++) {
		k_sleep(K_MSEC(20));
	}
	zassert_false(meshtastic_settings_save_pending(), "the save never fired");
	do {
		last = written();
		k_sleep(K_MSEC(200));
	} while (written() != last);
}

static meshtastic_Config_DeviceConfig_Role stored_role(void)
{
	meshtastic_Config c;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &c), "");
	return c.payload_variant.device.role;
}

static void *suite_setup(void)
{
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	return NULL;
}

static void suite_before(void *f)
{
	ARG_UNUSED(f);
	/* Every case starts as CLIENT, with flash agreeing with RAM. */
	zassert_ok(meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT),
		   "");
	wait_saved();
	zassert_ok(meshtastic_settings_flush(), "");
}

ZTEST_SUITE(settings_save, NULL, suite_setup, suite_before, NULL, NULL);

ZTEST(settings_save, test_the_caches_are_on)
{
	zassert_true(IS_ENABLED(CONFIG_SETTINGS_NVS_NAME_CACHE), "MESHTASTIC_SETTINGS implies it");
	zassert_true(IS_ENABLED(CONFIG_NVS_LOOKUP_CACHE), "MESHTASTIC_SETTINGS implies it");
}

ZTEST(settings_save, test_one_change_writes_its_record_and_stamp_only)
{
	uint32_t before = written();
	uint32_t n;

	zassert_ok(meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER),
		   "");
	wait_saved();
	n = written() - before;
#if TRACK >= 64
	zassert_equal(n, 2U, "config/device and its HLC stamp, nothing else (wrote %u)", n);
#else
	/* Untracked records are always written: the change is among them. */
	zassert_true(n >= 2U, "wrote %u", n);
#endif
}

ZTEST(settings_save, test_a_save_with_nothing_changed_writes_nothing)
{
	uint32_t before = written();

	/* The setter stamps and schedules a save even when the value is the same:
	 * the stamp moves (a newer write), so that record is written -- and only
	 * that one. A save with no setter at all writes nothing. */
	meshtastic_settings_schedule_save();
	wait_saved();
#if TRACK >= 64
	zassert_equal(written() - before, 0U, "an idle save wrote %u", written() - before);
#else
	zassert_true(written() - before > 0U, "untracked records are rewritten");
#endif
}

ZTEST(settings_save, test_the_change_reaches_flash_and_loads_back)
{
	struct raw before, after;

	zassert_true(raw_read("meshtastic/config/device", &before), "");
	zassert_ok(meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER),
		   "");
	wait_saved();
	zassert_true(raw_read("meshtastic/config/device", &after), "");
	zassert_false(before.len == after.len && memcmp(before.buf, after.buf, before.len) == 0,
		      "the record on flash changed");

	zassert_ok(settings_load_subtree("meshtastic"), "");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_ROUTER, "loads back");
}

/* A record rewritten behind the save's back (a backup restore, an older image)
 * is what the next load reads -- and the load re-seeds the table, so a later
 * change to it is not mistaken for "already on flash". */
ZTEST(settings_save, test_a_load_reseeds_what_flash_holds)
{
	struct raw client;

	zassert_true(raw_read("meshtastic/config/device", &client), "CLIENT on flash");
	zassert_ok(meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER),
		   "");
	wait_saved();

	/* Flash goes back to CLIENT behind the save; the load brings RAM with it. */
	zassert_ok(settings_save_one("meshtastic/config/device", client.buf, client.len), "");
	zassert_ok(settings_load_subtree("meshtastic"), "");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_CLIENT, "");

	/* ROUTER again: the table must not still think ROUTER is on flash. */
	zassert_ok(meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role_ROUTER),
		   "");
	wait_saved();
	zassert_ok(settings_load_subtree("meshtastic"), "");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "the second change reached flash");
}

ZTEST(settings_save, test_a_flush_writes_everything_and_counts_in_neither)
{
	uint32_t w0, s0, w1, s1;
	struct raw r;

	meshtastic_settings_save_stats(&w0, &s0);
	zassert_ok(meshtastic_settings_flush(), "");
	meshtastic_settings_save_stats(&w1, &s1);
	zassert_equal(w1, w0, "");
	zassert_equal(s1, s0, "");
	zassert_true(raw_read("meshtastic/owner", &r), "owner on flash");
	zassert_true(raw_read("meshtastic/config/lora", &r), "lora on flash");
}
