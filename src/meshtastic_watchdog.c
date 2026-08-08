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
 * the connection.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/sys_heap.h>

#include "meshtastic_watchdog.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if DT_HAS_ALIAS(watchdog0)

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

/* Fires in ISR context, one watchdog stage before the hardware forces a
 * reset (wdt_esp32.c: STAGE0 = this interrupt, STAGE1 = the actual reset,
 * same configured timeout apart). LOG_PANIC forces an immediate synchronous
 * flush attempt instead of leaving this in the deferred log buffer, which
 * may never get drained before the reset lands a moment later. */
static void watchdog_pre_reset_cb(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	LOG_ERR("WATCHDOG: no check-in for >= %d ms -- forcing reset",
		CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS);
	LOG_PANIC();
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

#endif /* DT_HAS_ALIAS(watchdog0) */
