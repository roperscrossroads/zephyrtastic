/* SPDX-License-Identifier: GPL-3.0
 *
 * Durable (flash-backed) boot history — agents-l984.
 *
 * The retained-RAM ring reports a power event by absence, so after a power cycle
 * you learn one happened and nothing else. That is a structural hole on a fleet
 * whose only bearers are LoRa and BLE: recovering a node that answers neither
 * means carrying it to a USB host, which removes power, which clears the very
 * record of why it needed recovering.
 *
 * So the property worth testing is not "does the ring index correctly" but "does
 * a record still exist after the RAM copy is gone". The reload test below is the
 * one that matters; the arithmetic tests are there because wrap-around is where
 * ring bugs hide.
 */
#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/bootlog.h>

#define ENTRIES CONFIG_MESHTASTIC_BOOTLOG_DURABLE_ENTRIES

static struct meshtastic_boot_durable mk(uint32_t n)
{
	struct meshtastic_boot_durable r = {
		.boot_num = n,
		.cause = 0x1000U + n,
		.wall_s = 1700000000U + n,
		.flags = (uint16_t)((n % 2U) ? MESHTASTIC_BOOT_F_WARM : 0U),
		.prev_uptime_s = (uint16_t)(n * 10U),
	};
	return r;
}

static void reset_ring(void *unused)
{
	ARG_UNUSED(unused);
	meshtastic_bootlog_test_durable_reset();
}

ZTEST(bootlog_durable, test_empty_history_reads_nothing)
{
	struct meshtastic_boot_durable out[ENTRIES];

	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), 0,
		      "a ring nothing has been written to must report zero entries, not "
		      "whatever was in the buffer");
}

ZTEST(bootlog_durable, test_single_record_round_trips_every_field)
{
	struct meshtastic_boot_durable in = mk(7);
	struct meshtastic_boot_durable out[ENTRIES];

	meshtastic_bootlog_test_durable_append(&in);
	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), 1, NULL);

	/* Every field, because a boot record that silently drops one is worse than
	 * none: the reader trusts what it prints. */
	zassert_equal(out[0].boot_num, in.boot_num, NULL);
	zassert_equal(out[0].cause, in.cause, NULL);
	zassert_equal(out[0].wall_s, in.wall_s, NULL);
	zassert_equal(out[0].flags, in.flags, NULL);
	zassert_equal(out[0].prev_uptime_s, in.prev_uptime_s, NULL);
}

ZTEST(bootlog_durable, test_partial_fill_is_oldest_first)
{
	struct meshtastic_boot_durable out[ENTRIES];
	const size_t n = ENTRIES - 1U;

	for (uint32_t i = 1; i <= n; i++) {
		struct meshtastic_boot_durable r = mk(i);

		meshtastic_bootlog_test_durable_append(&r);
	}

	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), n, NULL);
	for (uint32_t i = 0; i < n; i++) {
		zassert_equal(out[i].boot_num, i + 1U,
			      "history must read oldest-first; entry %u was #%u", i,
			      out[i].boot_num);
	}
}

ZTEST(bootlog_durable, test_wrap_keeps_the_newest_and_stays_ordered)
{
	struct meshtastic_boot_durable out[ENTRIES];
	const uint32_t total = ENTRIES * 2U + 1U; /* forces more than one wrap */

	for (uint32_t i = 1; i <= total; i++) {
		struct meshtastic_boot_durable r = mk(i);

		meshtastic_bootlog_test_durable_append(&r);
	}

	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), ENTRIES,
		      "a wrapped ring is full, not empty");
	/* The OLDEST retained is total-ENTRIES+1: everything before it fell off. */
	for (uint32_t i = 0; i < ENTRIES; i++) {
		zassert_equal(out[i].boot_num, total - ENTRIES + 1U + i,
			      "after wrapping, entry %u should be #%u but was #%u", i,
			      total - ENTRIES + 1U + i, out[i].boot_num);
	}
}

ZTEST(bootlog_durable, test_max_smaller_than_count_returns_the_newest)
{
	struct meshtastic_boot_durable out[2];

	for (uint32_t i = 1; i <= ENTRIES; i++) {
		struct meshtastic_boot_durable r = mk(i);

		meshtastic_bootlog_test_durable_append(&r);
	}

	/* Truncation must drop the OLDEST, not the newest: a caller with a small
	 * buffer wants the most recent boots, which are the ones explaining now. */
	zassert_equal(meshtastic_bootlog_durable_history(out, 2), 2, NULL);
	zassert_equal(out[0].boot_num, ENTRIES - 1U, NULL);
	zassert_equal(out[1].boot_num, ENTRIES, NULL);
}

ZTEST(bootlog_durable, test_history_survives_losing_the_ram_copy)
{
	struct meshtastic_boot_durable out[ENTRIES];
	const uint32_t n = 3U;

	/* This is agents-l984 in one test: write some history, destroy the RAM copy
	 * exactly as a power cycle would, reload from flash, and require it back. */
	for (uint32_t i = 1; i <= n; i++) {
		struct meshtastic_boot_durable r = mk(i);

		meshtastic_bootlog_test_durable_append(&r);
	}
	zassert_equal(meshtastic_bootlog_test_durable_save(), 0, "durable save failed");

	meshtastic_bootlog_test_durable_reset();
	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), 0,
		      "the RAM copy should be gone after a reset — otherwise the reload "
		      "below proves nothing");

	/* Deliberately the module's OWN load, not settings_load(). The boot path
	 * loads its subtree itself because nothing else does -- meshtastic_settings_init()
	 * runs from meshtastic_init() at runtime, after every SYS_INIT, and loads only
	 * its own subtree. Calling settings_load() here instead is precisely what hid
	 * that missing load: the test passed while every real boot wiped the ring and
	 * kept a single entry (caught on hardware 2026-09-01). Exercise what boots. */
	meshtastic_bootlog_test_durable_load();

	zassert_equal(meshtastic_bootlog_durable_history(out, ARRAY_SIZE(out)), n,
		      "history did NOT survive the reload — this is the whole feature");
	for (uint32_t i = 0; i < n; i++) {
		zassert_equal(out[i].boot_num, i + 1U, NULL);
		zassert_equal(out[i].wall_s, 1700000000U + i + 1U, NULL);
		zassert_equal(out[i].cause, 0x1000U + i + 1U, NULL);
	}
}

static void *suite_setup(void)
{
	/* NVS needs mounting before the first save; settings_subsys_init is
	 * idempotent, and the module's own SYS_INIT may already have run. */
	(void)settings_subsys_init();
	return NULL;
}

ZTEST_SUITE(bootlog_durable, NULL, suite_setup, reset_ring, NULL, NULL);
