/* SPDX-License-Identifier: GPL-3.0
 *
 * Task-level software watchdog (Zephyr's subsys/task_wdt) with the SoC's
 * hardware watchdog (wdt0) wired as its own fallback. See Kconfig.watchdog
 * for what this exists to catch and why, and for the two channels' exact
 * feed triggers.
 *
 * Design: task_wdt owns the scheduling (one kernel timer covering whichever
 * channel is soonest to expire) and the hardware-watchdog feeding, so this
 * file only needs to register channels and feed them. Each channel writes an
 * RTC-persistent breadcrumb before rebooting, read back and logged once
 * netlog is usable again -- see channel_timeout_cb()'s own comment for why
 * that callback is deliberately minimal (logging from here, and calling
 * coredump(), both turned out to cost real stack/call-depth for zero
 * benefit -- see docs/KNOWN-ISSUES.md).
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/stats/stats.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/task_wdt/task_wdt.h>

#include <zephyr/meshtastic/diagnostics.h>

#include "meshtastic_core.h" /* meshtastic_radio_cad_/agc_ counter getters */
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
 * riding the sysworkq heartbeat instead of needing a separate mechanism.
 * heap_caps_get_free_size() looked like the natural call instead, but this
 * Zephyr port stubs it to an unconditional 0 -- worth knowing since it
 * doesn't fail loudly, it just silently lies. */
extern struct k_heap _system_heap;

/* Watchdog channel feed counts -- visible via the `stats list` shell command
 * (CONFIG_STATS_SHELL). A running total across the current boot: not a
 * crash-forensics tool (plain RAM, lost on the same reset a timeout would
 * cause), just a sanity check that both channels are ticking roughly as
 * often as their feed source implies -- a channel feeding far less often
 * than expected is itself a useful early signal, well before it actually
 * times out. */
STATS_SECT_START(mt_wdt)
STATS_SECT_ENTRY(radio_feeds)
STATS_SECT_ENTRY(sysworkq_feeds)
STATS_SECT_ENTRY(hw_stage0_recovered)
STATS_SECT_END;

STATS_NAME_START(mt_wdt)
STATS_NAME(mt_wdt, radio_feeds)
STATS_NAME(mt_wdt, sysworkq_feeds)
STATS_NAME(mt_wdt, hw_stage0_recovered)
STATS_NAME_END(mt_wdt);

static STATS_SECT_DECL(mt_wdt) mt_wdt;

/* Mirror of the CAD/AGC counters meshtastic_radio.c already tracks
 * (sx126x_cad_*_count_get() etc., surfaced today via the hand-rolled
 * `meshtastic sched stats` shell command) -- refreshed each sysworkq
 * heartbeat so they're also visible via the generic `stats list` command,
 * without touching the patched vendor sx126x driver at all. */
STATS_SECT_START(mt_radio_cad_agc)
STATS_SECT_ENTRY(cad_clear)
STATS_SECT_ENTRY(cad_busy)
STATS_SECT_ENTRY(cad_timeout)
STATS_SECT_ENTRY(cad_error)
STATS_SECT_ENTRY(agc_reset_ok)
STATS_SECT_ENTRY(agc_reset_fail)
STATS_SECT_ENTRY(agc_reset_skipped)
STATS_SECT_ENTRY(agc_patch_fail)
STATS_SECT_END;

STATS_NAME_START(mt_radio_cad_agc)
STATS_NAME(mt_radio_cad_agc, cad_clear)
STATS_NAME(mt_radio_cad_agc, cad_busy)
STATS_NAME(mt_radio_cad_agc, cad_timeout)
STATS_NAME(mt_radio_cad_agc, cad_error)
STATS_NAME(mt_radio_cad_agc, agc_reset_ok)
STATS_NAME(mt_radio_cad_agc, agc_reset_fail)
STATS_NAME(mt_radio_cad_agc, agc_reset_skipped)
STATS_NAME(mt_radio_cad_agc, agc_patch_fail)
STATS_NAME_END(mt_radio_cad_agc);

static STATS_SECT_DECL(mt_radio_cad_agc) mt_radio_cad_agc;

static int radio_channel_id = -1;
static int sysworkq_channel_id = -1;
static struct k_work_delayable heartbeat_work;

/* RTC-persistent crash breadcrumb (see zephyr/meshtastic/diagnostics.h) --
 * survives the warm reset the watchdog itself forces, the same way the
 * reset-cause history in samples/meshtastic/src/main.c does. magic is
 * written LAST in the callback (below), after every other field, so a
 * reader that checks it first can trust the rest of the struct is complete
 * -- protects against the hardware reset landing mid-write. */
#define WATCHDOG_CRASH_MAGIC 0x43524153U /* "CRAS" */

static RTC_NOINIT_ATTR uint32_t rtc_crash_magic;
static RTC_NOINIT_ATTR struct meshtastic_watchdog_crash_info rtc_crash_info;

/* Separate breadcrumb + magic for the hardware-ISR path (see
 * hw_wdt_stage0_callback() below) -- deliberately its own RTC-persistent
 * slot, not reused from rtc_crash_info/rtc_crash_magic above, so a boot can
 * tell unambiguously which path actually caught the hang: this one firing
 * with the software one absent means the scheduler itself was unresponsive,
 * not just one application channel. */
#define HW_WATCHDOG_CRASH_MAGIC 0x48435241U /* "HCRA" */

static RTC_NOINIT_ATTR uint32_t rtc_hw_crash_magic;
static RTC_NOINIT_ATTR struct meshtastic_hw_watchdog_crash_info rtc_hw_crash_info;

/* Boot-time latch of the above, taken in meshtastic_watchdog_init() before the
 * hardware callback is installed -- see hw_crash_latch() for why the RTC slot
 * cannot be read directly at report time. */
static struct meshtastic_hw_watchdog_crash_info hw_crash_latched_info;
static bool hw_crash_latched_valid;

/* Static, not a local -- see the file comment on channel_timeout_cb() below
 * for why this callback's own stack footprint is treated as a hard
 * constraint, not a nicety. */
static struct sys_memory_stats channel_timeout_heap_stats;

/* Fires from task_wdt's internal k_timer expiry (subsys/task_wdt/task_wdt.c)
 * when the channel named by user_data goes unfed for
 * CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS. Shared by both channels -- see
 * task_wdt_add() call sites in meshtastic_watchdog_init() for which name
 * each one gets. user_data is a string literal (rodata, valid for the
 * program's whole lifetime), not heap/stack state, so it's safe to read
 * from whatever context this runs in.
 *
 * 2026-08-09: this function used to also call LOG_ERR() + LOG_PANIC() +
 * coredump() here, mirroring meshtastic_fatal.c's k_sys_fatal_error_handler()
 * -- reasonable on paper (same "nothing reaches netlog from here anyway, so
 * at least try" logic), but it caused a REAL crash on rzr3 during testing:
 * the channel timeout fired, and instead of a clean reboot, the board landed
 * in the exact same Xtensa double-exception signature this arc has been
 * chasing all along (EXCVADDR pointing into code space, DEPC in the
 * register-window-spill dispatch code) -- three occurrences total this
 * session, this one self-inflicted. task_wdt's k_timer expiry runs in a far
 * more stack-constrained context than the hardware WDT ISR the OLD
 * single-channel design used (CONFIG_ISR_STACK_SIZE=2048, shared across
 * every interrupt on this arch, and Xtensa's windowed-register ABI adds
 * real spill cost per call level that this function's own reported
 * `-fstack-usage` (80 B) does not capture) -- LOG_ERR's format-string
 * machinery and coredump()'s call chain (dump_header ->
 * process_memory_region_list -> ...) were each adding real call depth for
 * ZERO benefit, since both were already established to produce no visible
 * output on this project's network-only logging profile (see
 * docs/KNOWN-ISSUES.md). Removed entirely rather than trimmed -- what's left
 * is plain memory stores into RTC-persistent memory (can't fail or block
 * regardless of context) plus two cheap, already-proven-safe reads
 * (sys_heap_runtime_stats_get(), k_thread_name_get()). Keep it that way; any
 * future addition here needs its own stack-usage/call-depth justification,
 * not just "it seemed useful." */
static void channel_timeout_cb(int channel_id, void *user_data)
{
	const char *channel_name = (const char *)user_data;
	const char *tname;

	ARG_UNUSED(channel_id);

	strncpy(rtc_crash_info.channel, channel_name, sizeof(rtc_crash_info.channel) - 1);
	rtc_crash_info.channel[sizeof(rtc_crash_info.channel) - 1] = '\0';

	if (sys_heap_runtime_stats_get(&_system_heap.heap, &channel_timeout_heap_stats) == 0) {
		rtc_crash_info.heap_free = (uint32_t)channel_timeout_heap_stats.free_bytes;
		rtc_crash_info.heap_allocated = (uint32_t)channel_timeout_heap_stats.allocated_bytes;
		rtc_crash_info.heap_max_allocated =
			(uint32_t)channel_timeout_heap_stats.max_allocated_bytes;
	}

	/* Whatever thread happened to be running when the timeout fired -- not
	 * necessarily the one that stopped feeding the channel (see
	 * meshtastic_watchdog.h). k_thread_name_get() returns "" rather than
	 * NULL if CONFIG_THREAD_NAME is on but no name was set. */
	tname = k_thread_name_get(k_current_get());
	strncpy(rtc_crash_info.thread_name, tname != NULL ? tname : "",
		sizeof(rtc_crash_info.thread_name) - 1);
	rtc_crash_info.thread_name[sizeof(rtc_crash_info.thread_name) - 1] = '\0';

	/* Written last, deliberately -- see the file comment on rtc_crash_magic. */
	rtc_crash_magic = WATCHDOG_CRASH_MAGIC;

	sys_reboot(SYS_REBOOT_WARM);
}

/* Fires from the hardware watchdog's OWN interrupt stage (wdt_esp32_isr(),
 * zephyr/drivers/watchdog/wdt_esp32.c) -- a genuine hardware ISR (IRAM-
 * resident, esp_intr_alloc()'d), not a k_timer/software-timeout context like
 * channel_timeout_cb() above. task_wdt's own hardware-fallback wiring
 * (task_wdt_init(), zephyr/subsys/task_wdt/task_wdt.c) configures this same
 * device+timeout with callback=NULL; meshtastic_watchdog_init() below
 * re-installs the timeout with THIS callback plugged in afterward.
 * wdt_install_timeout() only updates the ESP32 driver's software-side
 * data->callback field -- it does not re-touch the already-configured
 * hardware timer stages (only wdt_setup(), which we do not call again here,
 * does that) -- so this is additive: task_wdt's own feeding/timeout behavior
 * is completely unaffected, this callback just gets invoked at the SAME
 * interrupt-stage moment that was previously silently discarded
 * (`if (data->callback) { ... }` never true).
 *
 * Reaching this callback at all -- as opposed to channel_timeout_cb() -- is
 * itself the diagnostic: it means the scheduler was unresponsive enough that
 * task_wdt's own k_timer-driven background-channel feed of this same
 * hardware watchdog also stopped, which is a strictly more severe condition
 * than any single application channel (radio/sysworkq) going unfed while the
 * scheduler otherwise keeps running fine.
 *
 * Kept deliberately MORE minimal than channel_timeout_cb() -- a true hardware
 * ISR can interrupt code at any point, including mid-heap-operation, where
 * channel_timeout_cb()'s sys_heap_runtime_stats_get() call would risk
 * deadlocking on a lock the interrupted code already holds (that call is
 * safe in channel_timeout_cb()'s k_timer context only because Zephyr's timer
 * expiry there cannot itself preempt a heap operation the same way a raw ISR
 * can). k_thread_name_get()/k_current_get()/k_uptime_get_32() are plain
 * pointer/counter reads with no locking, safe from any context. Does NOT
 * call sys_reboot() -- the hardware watchdog's own STAGE1 timer, already
 * running, forces the actual reset; this callback only needs to record the
 * breadcrumb before that happens (~10s later on this build's timeout).
 */
static void IRAM_ATTR hw_wdt_stage0_callback(const struct device *dev, int channel_id)
{
	const char *tname;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	rtc_hw_crash_info.uptime_ms = k_uptime_get_32();

	tname = k_thread_name_get(k_current_get());
	strncpy(rtc_hw_crash_info.thread_name, tname != NULL ? tname : "",
		sizeof(rtc_hw_crash_info.thread_name) - 1);
	rtc_hw_crash_info.thread_name[sizeof(rtc_hw_crash_info.thread_name) - 1] = '\0';

	/* Written last, deliberately -- see the file comment on rtc_crash_magic
	 * (same reasoning applies to this separate breadcrumb). */
	rtc_hw_crash_magic = HW_WATCHDOG_CRASH_MAGIC;
}

/* Feeds the "sysworkq" channel -- proof that the system workqueue thread
 * itself is scheduling and running, independently of the radio subsystem's
 * own "radio" channel. Also carries the two periodic diagnostics that used
 * to ride the old single feed worker: heap-stat logging and the CAD/AGC
 * stats mirror above. */
static void heartbeat_work_fn(struct k_work *work)
{
	struct sys_memory_stats stats;

	ARG_UNUSED(work);

	if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
		LOG_INF("Watchdog: system heap free=%zu allocated=%zu max_allocated=%zu",
			stats.free_bytes, stats.allocated_bytes, stats.max_allocated_bytes);
	}

	STATS_SET(mt_radio_cad_agc, cad_clear, meshtastic_radio_cad_clear_count());
	STATS_SET(mt_radio_cad_agc, cad_busy, meshtastic_radio_cad_busy_count());
	STATS_SET(mt_radio_cad_agc, cad_timeout, meshtastic_radio_cad_timeout_count());
	STATS_SET(mt_radio_cad_agc, cad_error, meshtastic_radio_cad_error_count());
	STATS_SET(mt_radio_cad_agc, agc_reset_ok, meshtastic_radio_agc_reset_ok_count());
	STATS_SET(mt_radio_cad_agc, agc_reset_fail, meshtastic_radio_agc_reset_fail_count());
	STATS_SET(mt_radio_cad_agc, agc_reset_skipped,
		  meshtastic_radio_agc_reset_skipped_count());
	STATS_SET(mt_radio_cad_agc, agc_patch_fail, meshtastic_radio_agc_patch_fail_count());

	/* A stage-0 fire that did NOT go on to reset the board -- the hardware
	 * watchdog decided the software layer was gone, then the next feed
	 * arrived before stage 1 expired. Distinct from, and much less severe
	 * than, the breadcrumb reported at boot: this one means "task_wdt's
	 * background feed was more than CONFIG_TASK_WDT_HW_FALLBACK_DELAY late",
	 * not "we were reset". Worth a warning either way -- it is the only
	 * warning of an eroding hardware-fallback margin before that margin
	 * actually runs out. Racy by a hair (the ISR could fire between the read
	 * and the clear, losing one event); acceptable for a counter whose
	 * purpose is "is this happening at all". */
	if (rtc_hw_crash_magic == HW_WATCHDOG_CRASH_MAGIC) {
		uint32_t at_ms = rtc_hw_crash_info.uptime_ms;

		rtc_hw_crash_magic = 0U;
		STATS_INC(mt_wdt, hw_stage0_recovered);
		LOG_WRN("HW watchdog stage 0 fired at uptime=%u ms but the board recovered "
			"(task_wdt background feed was late; no reset)",
			at_ms);
	}

	if (sysworkq_channel_id >= 0) {
		STATS_INC(mt_wdt, sysworkq_feeds);
		(void)task_wdt_feed(sysworkq_channel_id);
	}

	(void)k_work_reschedule(&heartbeat_work, K_MSEC(CONFIG_MESHTASTIC_WATCHDOG_FEED_INTERVAL_MS));
}

void meshtastic_watchdog_checkin(void)
{
	if (radio_channel_id < 0) {
		return;
	}

	STATS_INC(mt_wdt, radio_feeds);
	(void)task_wdt_feed(radio_channel_id);
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

/* Copy the RTC slot into plain RAM and clear it, BEFORE anything can install
 * hw_wdt_stage0_callback() -- so what gets reported is unambiguously "what was
 * in RTC memory when this boot started", i.e. an event that genuinely survived
 * a reset.
 *
 * 2026-08-10: this exists because the naive version (read the RTC slot at
 * report time) reported nothing but false positives. Unlike the other two
 * breadcrumbs in this project, the hardware stage-0 path does NOT reboot --
 * stage 0 raises an interrupt and stage 1 does the reset a full timeout later,
 * so a stage-0 fire that the very next feed clears leaves the board running
 * with a freshly-written breadcrumb in RTC memory. The report site
 * (log_boot_reset_cause() in samples/meshtastic/src/main.c) runs on the first
 * IPv4 lease, which on a WiFi node lands tens of seconds into boot -- i.e.
 * AFTER a stage-0 fire during association. Every "HW watchdog crash info" line
 * seen on the bench was that: the breadcrumb's uptime always fell a few
 * seconds *before* the same boot's own DHCP lease (rzr3 16.8 s vs 23.7 s,
 * rzr2 48.7 vs 58.5, rzr1 68.0 vs 77.5), never surviving a reset at all, on a
 * node that then ran 20 hours without incident. Latching at boot closes that
 * off structurally rather than by tuning: a same-boot fire now cannot reach
 * the report path no matter what the timings do. */
static void hw_crash_latch(void)
{
	if (rtc_hw_crash_magic == HW_WATCHDOG_CRASH_MAGIC) {
		hw_crash_latched_info = rtc_hw_crash_info;
		hw_crash_latched_valid = true;
	}

	rtc_hw_crash_magic = 0U;
}

bool meshtastic_hw_watchdog_take_last_crash(struct meshtastic_hw_watchdog_crash_info *out)
{
	if (!hw_crash_latched_valid) {
		return false;
	}

	*out = hw_crash_latched_info;
	hw_crash_latched_valid = false; /* one-shot, as before */

	return true;
}

void meshtastic_watchdog_init(void)
{
	const struct device *wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	int ret;

	/* First thing, unconditionally: nothing below this line may run before
	 * the RTC slot has been snapshotted, or a stage-0 fire from THIS boot
	 * becomes indistinguishable from one that survived a reset. Deliberately
	 * ahead of every early-return path too -- the slot must be consumed even
	 * on a board where the watchdog fails to arm, so a stale breadcrumb
	 * cannot be re-reported on an unrelated later boot. */
	hw_crash_latch();

	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("Watchdog device not ready; hangs will not auto-recover");
		return;
	}

	ret = task_wdt_init(wdt_dev);
	if (ret < 0) {
		LOG_WRN("task_wdt_init failed (%d); hangs will not auto-recover", ret);
		return;
	}

	(void)STATS_INIT_AND_REG(mt_wdt, STATS_SIZE_32, "mt_wdt");
	(void)STATS_INIT_AND_REG(mt_radio_cad_agc, STATS_SIZE_32, "mt_radio_cad_agc");

	radio_channel_id = task_wdt_add(CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS, channel_timeout_cb,
					 (void *)"radio");
	if (radio_channel_id < 0) {
		LOG_WRN("task_wdt_add(radio) failed (%d); that channel will not auto-recover",
			radio_channel_id);
	}

	sysworkq_channel_id = task_wdt_add(CONFIG_MESHTASTIC_WATCHDOG_TIMEOUT_MS,
					    channel_timeout_cb, (void *)"sysworkq");
	if (sysworkq_channel_id < 0) {
		LOG_WRN("task_wdt_add(sysworkq) failed (%d); that channel will not auto-recover",
			sysworkq_channel_id);
	}

#if defined(CONFIG_TASK_WDT_HW_FALLBACK)
	/* Plug our own callback into the hardware watchdog's interrupt stage --
	 * see hw_wdt_stage0_callback()'s own comment for the full rationale.
	 * task_wdt_init() (above) already called wdt_install_timeout() on this
	 * same device with callback=NULL; the first task_wdt_add() call (above)
	 * already triggered task_wdt's own wdt_setup(), which is what actually
	 * programs the hardware timer stages -- calling wdt_install_timeout()
	 * again here, AFTER that, only updates the ESP32 driver's software-side
	 * callback pointer (see zephyr/drivers/watchdog/wdt_esp32.c,
	 * wdt_esp32_install_timeout() does not touch hardware registers) and is
	 * additive: task_wdt's own feed/timeout/reset behavior is unaffected.
	 * window.max mirrors task_wdt.c's own internal call exactly, so the
	 * (unused, since wdt_setup() won't run again) recorded values stay
	 * consistent with what is actually programmed in hardware. */
	{
		struct wdt_timeout_cfg hw_cfg = {
			.window = {.min = 0U,
				   .max = CONFIG_TASK_WDT_MIN_TIMEOUT +
					  CONFIG_TASK_WDT_HW_FALLBACK_DELAY},
			.callback = hw_wdt_stage0_callback,
			.flags = WDT_FLAG_RESET_SOC,
		};
		int hw_ret = wdt_install_timeout(wdt_dev, &hw_cfg);

		if (hw_ret < 0) {
			LOG_WRN("hw watchdog diagnostic callback install failed (%d); "
				"a scheduler-stalling hang will still auto-recover, but "
				"leave no breadcrumb",
				hw_ret);
		}
	}
#endif /* CONFIG_TASK_WDT_HW_FALLBACK */

	k_work_init_delayable(&heartbeat_work, heartbeat_work_fn);
	(void)k_work_schedule(&heartbeat_work, K_MSEC(CONFIG_MESHTASTIC_WATCHDOG_FEED_INTERVAL_MS));

	LOG_INF("Task watchdog armed: radio+sysworkq channels, timeout %d ms, "
		"sysworkq heartbeat every %d ms",
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

bool meshtastic_hw_watchdog_take_last_crash(struct meshtastic_hw_watchdog_crash_info *out)
{
	ARG_UNUSED(out);
	return false;
}

#endif /* DT_HAS_ALIAS(watchdog0) */
