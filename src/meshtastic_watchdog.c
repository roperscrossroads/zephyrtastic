/* SPDX-License-Identifier: GPL-3.0
 *
 * Hardware watchdog. See meshtastic_watchdog.h and Kconfig.watchdog for what
 * this exists to catch and why.
 *
 * Design: a single periodic feed worker only calls wdt_feed() if a check-in
 * landed within the last timeout window; otherwise it logs a warning and lets
 * the hardware timer run out on its own. The watchdog's own pre-reset ISR
 * callback (fired one stage before the actual SoC reset — see wdt_esp32.c)
 * also logs, as a second, independent breadcrumb in case the feed worker's
 * own warning never made it out over the network before the reset severed
 * the connection, and additionally forces a full coredump — see
 * watchdog_pre_reset_cb() for why there's time for that to actually land.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/debug/coredump.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/sys_heap.h>

#include <zephyr/meshtastic/diagnostics.h>

#include "meshtastic_watchdog.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if DT_HAS_ALIAS(watchdog0)

/* ESP-IDF HAL header, for RTC_NOINIT_ATTR below -- not available outside an
 * ESP32 build (e.g. native_sim), hence guarded inside this board-has-a-wdt0
 * branch rather than included unconditionally at file scope. */
#include <esp_attr.h>

/* _system_heap backs k_malloc() -- and on this Zephyr port, the vendored
 * WiFi/BT HAL's heap_caps_malloc() is a thin wrapper straight over k_malloc()
 * too (modules/hal/espressif/zephyr/port/heap/heap_caps_zephyr.c), so this is
 * genuinely the pool "esp32_wifi_adapter: memory allocation failed" is
 * failing to allocate from -- the one recurring log line every hang
 * investigated on the bench this session had in common. Undocumented-public
 * (leading underscore) but this is the same symbol Zephyr's own `kernel
 * heap` shell command uses (subsys/shell/modules/kernel_service/heap.c) --
 * riding the watchdog's own periodic wake instead of needing a separate
 * mechanism. heap_caps_get_free_size() looked like the natural call instead,
 * but this Zephyr port stubs it to an unconditional 0 -- worth knowing since
 * it doesn't fail loudly, it just silently lies. */
extern struct k_heap _system_heap;

static int64_t last_checkin_ms;
static int wdt_channel_id = -1;
static struct k_work_delayable feed_work;

/* RTC-persistent crash breadcrumb (see meshtastic_watchdog.h) -- survives the
 * warm reset the watchdog itself forces, the same way the reset-cause history
 * in samples/meshtastic/src/main.c does. magic is written LAST in the ISR
 * (below), after every other field, so a reader that checks it first can
 * trust the rest of the struct is complete -- protects against the (unlikely
 * but real, given the measured seconds-not-minutes stage0->stage1 gap) case
 * of the hardware reset landing mid-write. */
#define WATCHDOG_CRASH_MAGIC 0x43524153U /* "CRAS" */

static RTC_NOINIT_ATTR uint32_t rtc_crash_magic;
static RTC_NOINIT_ATTR struct meshtastic_watchdog_crash_info rtc_crash_info;

/* Fires in ISR context, one watchdog stage before the hardware forces a
 * reset (wdt_esp32.c: STAGE0 = this interrupt, STAGE1 = the actual reset).
 * MEASURED on real hardware (rzr3, 2026-08-08): the gap between the two is
 * seconds, not the "another full timeout" the driver's stage config
 * suggested -- so there's very little runway here, not generous margin.
 *
 * More importantly: LOG_PANIC() does NOT get this out over netlog. Every
 * registered log backend's panic() callback runs, and log_backend_net's
 * (the only backend this profile enables -- see overlay-v4-unified.conf)
 * unconditionally no-ops every subsequent process() call once panic_mode is
 * set (BSD sockets aren't safe to drive synchronously from panic/ISR
 * context) -- so this LOG_ERR, and the coredump() output below, are both
 * silently dropped on this transport. Confirmed by a live test that
 * deliberately starved check-ins: the "no check-in" WRN from the feed
 * worker (deferred-logged, BEFORE panic mode) reached netlog fine; this
 * function's own LOG_ERR right below, called AFTER LOG_PANIC(), did not --
 * netlog went straight from the last pre-panic line to the post-reboot
 * boot banner, nothing in between.
 *
 * coredump() is left in as the seed for a real fix (it's inert-but-harmless
 * on this transport) rather than reverted -- CONFIG_DEBUG_COREDUMP_BACKEND_FLASH_PARTITION
 * turned out to be a dead end too: this Zephyr port's flash driver refuses
 * writes from ISR context outright (flash_esp32_write_async() returns
 * -EINVAL if k_is_in_isr()), which is exactly the context this callback and
 * z_fatal_error() both run in -- so it could never write anything here
 * either. What actually works: the RTC-persistent breadcrumb below, plain
 * memory stores rather than anything that goes through a driver or the log
 * subsystem, so nothing here can fail or block. */
static void watchdog_pre_reset_cb(const struct device *dev, int channel_id)
{
	struct sys_memory_stats stats;
	const char *tname;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	LOG_ERR("WATCHDOG: no check-in for >= %d ms -- forcing reset",
		CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS);
	LOG_PANIC();

	coredump(K_ERR_KERNEL_PANIC, NULL, k_current_get());

	/* Everything below is plain memory stores into RTC-persistent memory --
	 * no driver, no log subsystem, can't fail or block, so this survives
	 * regardless of how little runway is left before the hardware reset. */
	rtc_crash_info.since_checkin_ms = (uint32_t)(k_uptime_get() - last_checkin_ms);

	if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
		rtc_crash_info.heap_free = (uint32_t)stats.free_bytes;
		rtc_crash_info.heap_allocated = (uint32_t)stats.allocated_bytes;
		rtc_crash_info.heap_max_allocated = (uint32_t)stats.max_allocated_bytes;
	}

	/* Whatever thread the ISR interrupted -- informative, not necessarily
	 * the culprit (see meshtastic_watchdog.h). k_thread_name_get() returns
	 * "" rather than NULL if CONFIG_THREAD_NAME is on but no name was set. */
	tname = k_thread_name_get(k_current_get());
	strncpy(rtc_crash_info.thread_name, tname != NULL ? tname : "",
		sizeof(rtc_crash_info.thread_name) - 1);
	rtc_crash_info.thread_name[sizeof(rtc_crash_info.thread_name) - 1] = '\0';

	/* Written last, deliberately -- see the file comment on rtc_crash_magic. */
	rtc_crash_magic = WATCHDOG_CRASH_MAGIC;
}

static void feed_work_fn(struct k_work *work)
{
	const struct device *wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	int64_t since = k_uptime_get() - last_checkin_ms;

	/* System-heap trend (see the file comment above on why this is the pool
	 * that matters). A downward trend across several of these log lines
	 * ahead of a later silent hang would confirm/refute memory exhaustion as
	 * the cause; today we have no visibility into this at all between one
	 * "allocation failed" line and the hang. */
	{
		struct sys_memory_stats stats;

		if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
			LOG_INF("Watchdog: system heap free=%zu allocated=%zu max_allocated=%zu",
				stats.free_bytes, stats.allocated_bytes,
				stats.max_allocated_bytes);
		}
	}

	if (since < CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS) {
		(void)wdt_feed(wdt_dev, wdt_channel_id);
	} else {
		/* Don't feed -- if this is a real hang, the hardware timer (already
		 * counting since the last successful feed) will fire on its own
		 * shortly. Logging here is a second attempt at a breadcrumb,
		 * independent of watchdog_pre_reset_cb() above. */
		LOG_WRN("Watchdog: no check-in for %lld ms (timeout %d ms), not feeding",
			since, CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS);
	}

	(void)k_work_reschedule(&feed_work, K_MSEC(CONFIG_MESHTASTIC_WATCHDOG_FEED_INTERVAL_MS));
}

void meshtastic_watchdog_checkin(void)
{
	last_checkin_ms = k_uptime_get();
}

bool meshtastic_watchdog_take_last_crash(struct meshtastic_watchdog_crash_info *out)
{
	if (rtc_crash_magic != WATCHDOG_CRASH_MAGIC) {
		return false;
	}

	*out = rtc_crash_info;
	rtc_crash_magic = 0U; /* one-shot: don't re-report on a later, unrelated boot */

	return true;
}

void meshtastic_watchdog_init(void)
{
	const struct device *wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	struct wdt_timeout_cfg cfg = {
		.window.min = 0U,
		.window.max = CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS,
		.callback = watchdog_pre_reset_cb,
		.flags = WDT_FLAG_RESET_SOC,
	};
	int ret;

	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("Watchdog device not ready; hangs will not auto-recover");
		return;
	}

	ret = wdt_install_timeout(wdt_dev, &cfg);
	if (ret < 0) {
		LOG_WRN("Watchdog install_timeout failed (%d); hangs will not auto-recover", ret);
		return;
	}
	wdt_channel_id = ret;

	ret = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret < 0) {
		LOG_WRN("Watchdog setup failed (%d); hangs will not auto-recover", ret);
		wdt_channel_id = -1;
		return;
	}

	last_checkin_ms = k_uptime_get();
	k_work_init_delayable(&feed_work, feed_work_fn);
	(void)k_work_schedule(&feed_work, K_MSEC(CONFIG_MESHTASTIC_WATCHDOG_FEED_INTERVAL_MS));

	LOG_INF("Watchdog armed: timeout %d ms, feed-check every %d ms",
		CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS, CONFIG_MESHTASTIC_WATCHDOG_FEED_INTERVAL_MS);
}

#else /* no watchdog0 alias on this board */

void meshtastic_watchdog_init(void)
{
	LOG_WRN("No watchdog0 alias on this board; hangs will not auto-recover");
}

void meshtastic_watchdog_checkin(void)
{
}

bool meshtastic_watchdog_take_last_crash(struct meshtastic_watchdog_crash_info *out)
{
	ARG_UNUSED(out);
	return false;
}

#endif /* DT_HAS_ALIAS(watchdog0) */
