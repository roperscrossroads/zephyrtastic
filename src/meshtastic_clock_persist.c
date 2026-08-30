/* SPDX-License-Identifier: GPL-3.0
 *
 * Carry the wall clock across a warm reset — see meshtastic_clock_persist.h.
 */

#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>

#include <zephyr/meshtastic/bootlog.h>

#include "meshtastic_clock.h"
#include "meshtastic_clock_persist.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/*
 * Retained across a reset, by the same mechanism and with the same caveats as
 * the boot log: RTC memory on this SoC, .noinit elsewhere, and an ordinary
 * static on native_sim where there is no reset to survive.
 */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#define PERSIST_ATTR RTC_NOINIT_ATTR
#elif defined(CONFIG_ARCH_POSIX)
#define PERSIST_ATTR
#else
#define PERSIST_ATTR __noinit
#endif

#define PERSIST_MAGIC   0x4D435031U /* "MCP1" */
#define PERSIST_VERSION 1U

static PERSIST_ATTR struct {
	uint32_t magic;
	uint16_t version;
	uint16_t quality; /* what the clock was worth when saved; reporting only */
	int64_t epoch_ms;
	uint32_t counter_ticks;
} saved;

static const struct device *const counter_dev =
	DEVICE_DT_GET(DT_CHOSEN(meshtastic_persist_counter));

static enum meshtastic_clock_persist_result last_result;
static uint32_t last_downtime_ms;

static void persist_save(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(save_work, persist_save);

static void persist_save(struct k_work *work)
{
	uint32_t ticks;
	int64_t now_ms;

	ARG_UNUSED(work);

	if (!meshtastic_clock_valid() || !device_is_ready(counter_dev) ||
	    counter_get_value(counter_dev, &ticks) != 0) {
		goto reschedule;
	}

	now_ms = meshtastic_clock_now_epoch_ms();
	if (now_ms <= 0) {
		goto reschedule;
	}

	/*
	 * Magic cleared first, set last. A reset landing in the middle of this
	 * then leaves a record that fails its own check, rather than one that
	 * passes with half its fields from the previous save — which would restore
	 * a confident wrong time, the failure this whole module must not produce.
	 */
	saved.magic = 0U;
	barrier_dmem_fence_full();
	saved.version = PERSIST_VERSION;
	saved.quality = (uint16_t)meshtastic_clock_get_quality();
	saved.epoch_ms = now_ms;
	saved.counter_ticks = ticks;
	barrier_dmem_fence_full();
	saved.magic = PERSIST_MAGIC;

reschedule:
	k_work_schedule(&save_work, K_SECONDS(CONFIG_MESHTASTIC_CLOCK_PERSIST_INTERVAL_S));
}

void meshtastic_clock_persist_status(enum meshtastic_clock_persist_result *result,
				     uint32_t *downtime_ms)
{
	if (result != NULL) {
		*result = last_result;
	}
	if (downtime_ms != NULL) {
		*downtime_ms = last_downtime_ms;
	}
}

const char *meshtastic_clock_persist_result_str(enum meshtastic_clock_persist_result result)
{
	switch (result) {
	case MESHTASTIC_CLOCK_PERSIST_RESTORED:
		return "restored across the reset";
	case MESHTASTIC_CLOCK_PERSIST_COLD:
		return "cold boot — retained RAM did not survive";
	case MESHTASTIC_CLOCK_PERSIST_NO_RECORD:
		return "warm, but no clock had been saved";
	case MESHTASTIC_CLOCK_PERSIST_NO_COUNTER:
		return "the persist counter could not be read";
	case MESHTASTIC_CLOCK_PERSIST_IMPLAUSIBLE:
		return "the measured downtime failed its sanity bound";
	default:
		return "not attempted";
	}
}

/*
 * Decide and act, given a measured gap. Split out from the boot path so a test
 * can reach it without a real reset: the two claims worth pinning are that a gap
 * beyond the bound is refused, and that a restore lands at DEVICE quality rather
 * than at the quality the time originally had. Both are silent if wrong.
 */
static enum meshtastic_clock_persist_result persist_apply(uint64_t downtime_ms)
{
	if (downtime_ms > (uint64_t)CONFIG_MESHTASTIC_CLOCK_PERSIST_MAX_DOWNTIME_S * MSEC_PER_SEC) {
		/* Longer than any warm reset takes, so the number is not a downtime:
		 * the counter wrapped, or the record survived its magic check by
		 * accident. Declining costs one missing clock; believing it would put
		 * a confident wrong epoch on everything stamped afterwards. */
		LOG_WRN("clock: not restoring — measured downtime %llu ms exceeds the "
			"plausible bound", (unsigned long long)downtime_ms);
		last_downtime_ms = (uint32_t)MIN(downtime_ms, (uint64_t)UINT32_MAX);
		return MESHTASTIC_CLOCK_PERSIST_IMPLAUSIBLE;
	}

	/*
	 * Restored at DEVICE quality, NOT at the quality it had when saved, and
	 * this is a correctness decision rather than a conservative one.
	 *
	 * DEVICE means precisely "time from an onboard peripheral after boot",
	 * which is what a remembered epoch plus a counter reading is. Restoring at
	 * the ORIGINAL quality would make a remembered time outrank a live one: at
	 * NTP it would block a real NTP source for the whole re-apply window, and
	 * at GPS it would block everything except GPS indefinitely — so a node
	 * would come back from a reset holding a stale time it refused to correct.
	 */
	meshtastic_clock_set_epoch_ms(saved.epoch_ms + (int64_t)downtime_ms,
				      MESHTASTIC_CLOCK_QUALITY_DEVICE,
				      MESHTASTIC_CLOCK_PRECISION_SUBSECOND);
	last_downtime_ms = (uint32_t)downtime_ms;
	LOG_INF("clock: restored across a %u ms reset (saved at quality %u)",
		(unsigned int)downtime_ms, (unsigned int)saved.quality);
	return MESHTASTIC_CLOCK_PERSIST_RESTORED;
}

#if defined(CONFIG_ZTEST)
enum meshtastic_clock_persist_result
meshtastic_clock_persist_test_restore(int64_t epoch_ms, enum meshtastic_clock_quality saved_q,
				      uint32_t gap_ms)
{
	saved.magic = PERSIST_MAGIC;
	saved.version = PERSIST_VERSION;
	saved.epoch_ms = epoch_ms;
	saved.quality = (uint16_t)saved_q;
	last_result = persist_apply(gap_ms);
	return last_result;
}
#endif

static int persist_restore(void)
{
	struct meshtastic_boot_record boot;
	uint32_t now_ticks, delta, freq;
	uint64_t downtime_ms;

	/*
	 * Gate on the WARM flag, not on the reset CAUSE. The cause register is
	 * frequently empty on this SoC and, more decisively, the boot log clears it
	 * at PRE_KERNEL_1 so every later reader sees zero — a check against it would
	 * misclassify every reset while looking entirely reasonable. The warm flag is
	 * derived from the retained memory itself, which is precisely the thing that
	 * has to have survived for any of this to mean anything.
	 */
	meshtastic_bootlog_this_boot(&boot);
	if ((boot.flags & MESHTASTIC_BOOT_F_WARM) == 0U) {
		last_result = MESHTASTIC_CLOCK_PERSIST_COLD;
		goto start_saving;
	}

	if (saved.magic != PERSIST_MAGIC || saved.version != PERSIST_VERSION) {
		last_result = MESHTASTIC_CLOCK_PERSIST_NO_RECORD;
		goto start_saving;
	}

	if (!device_is_ready(counter_dev) || counter_get_value(counter_dev, &now_ticks) != 0) {
		last_result = MESHTASTIC_CLOCK_PERSIST_NO_COUNTER;
		goto start_saving;
	}

	freq = counter_get_frequency(counter_dev);
	if (freq == 0U) {
		last_result = MESHTASTIC_CLOCK_PERSIST_NO_COUNTER;
		goto start_saving;
	}

	/* Unsigned subtraction: correct across one wrap of the counter's width. */
	delta = now_ticks - saved.counter_ticks;
	downtime_ms = ((uint64_t)delta * MSEC_PER_SEC) / freq;
	last_result = persist_apply(downtime_ms);

start_saving:
	k_work_schedule(&save_work, K_SECONDS(CONFIG_MESHTASTIC_CLOCK_PERSIST_INTERVAL_S));
	return 0;
}

SYS_INIT(persist_restore, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
