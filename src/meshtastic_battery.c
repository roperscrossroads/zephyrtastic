/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Battery voltage / charge from the board's VBAT voltage-divider (the `vbatt`
 * DT node — ADC1 ch0 / GPIO1 on the Heltec V4). Split out of the display so the
 * read path has a single owner. See docs/battery-provenance.md for every
 * constant's source.
 *
 * Radio-safety: the devicetree channel MUST be &adc0 (hardware ADC1, GPIO1),
 * never &adc1 (hardware ADC2, GPIO11 -- this board's SX1262 SPI MISO line).
 * Zephyr's own esp32s3_common.dtsi names the two backwards from the SoC's own
 * "ADC1"/"ADC2" numbering, and getting this wrong (as this board file did for
 * seven weeks) breaks the radio outright when the channel is set up -- not via
 * some RTC-IO side effect, just because it's the wrong physical pin. See
 * docs/battery-provenance.md for the full account. Carried patch 0001
 * (adc_esp32 GPIO-disconnect skip) may no longer be necessary now that the
 * DTS points at the right pin -- bench-tested safe to revert on one board, not
 * yet proven fleet-wide -- but it's harmless to leave in place either way.
 *
 * Read strategy: mirrors upstream Meshtastic's espAdcRead() (firmware/src/
 * Power.cpp) on this identical circuit — a short ADC_CTRL settle then multiple
 * averaged samples — rather than a single sample after a long settle (an
 * earlier version of this driver did that and needed a large empirical
 * correction factor to compensate for the resulting under-read; that under-read
 * was an artifact of the read strategy, not the hardware. MeshCore's
 * HeltecV4Board::getBattMilliVolts() independently confirms the same
 * short-settle-plus-averaging shape on this board).
 *
 * This module also owns the low-voltage cutoff. That is not a nicety: upstream
 * Meshtastic's ONLY over-discharge protection on these boards is a voltage
 * trigger (Power.cpp:1061 -> deep sleep), so a port that cannot read the divider
 * has no protection at all. Two bench cells swelled while this port ran without
 * it. See docs/battery-provenance.md.
 */

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#if defined(CONFIG_MESHTASTIC_BATTERY_CUTOFF)
#include <zephyr/sys/poweroff.h>
#include "meshtastic_core.h" /* meshtastic_radio_disarm_dio1_wake() */
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include "meshtastic_settings.h"
#endif
#endif

#include "meshtastic_battery.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if DT_NODE_EXISTS(DT_NODELABEL(vbatt))

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/voltage_divider.h>
#include <zephyr/drivers/gpio.h>

/* Matches upstream's BATTERY_SENSE_SAMPLES default (Power.cpp) for this same
 * read path (ESP32 adc_oneshot_read loop). */
#define BATTERY_SENSE_SAMPLES 15

/* How long a sampled value is reused before the ADC is touched again, so a
 * display redraw does not re-sample on every frame. Named (rather than a bare
 * literal at the comparison) because the cutoff's poll interval has to stay
 * clear of it — see the BUILD_ASSERT under CONFIG_MESHTASTIC_BATTERY_CUTOFF. */
#define BATTERY_CACHE_MS 5000

/* Single-cell LiPo open-circuit-voltage curve, byte-for-byte upstream's
 * OCV_ARRAY (firmware/src/Power.h). 10 % per segment. File scope because the
 * two decision thresholds below are derived from its endpoints, exactly as
 * upstream derives them (Power.cpp:621-622) — keeping them tied to the curve
 * rather than restating magic numbers that could drift apart from it. */
static const uint16_t ocv[] = {
	4190, 4050, 3990, 3890, 3800, 3720, 3630, 3530, 3420, 3300, 3100,
};

/* Above OCV_max + 10 mV the pack cannot be resting — something is charging it.
 * On the Heltec V4/V4-R8 this inference is ALL we have: neither board routes a
 * VBUS sense or a charger status pin to a GPIO, so upstream falls back to the
 * identical voltage test (Power.cpp:572, `getBattVoltage() > chargingVolt`). */
#define BATTERY_EXT_PWR_MV ((int)ocv[0] + 10)

/* Below OCV_min - 500 mV, assume no cell is fitted rather than a desperately
 * flat one — a USB-powered board with an empty JST lands here. */
#define BATTERY_NO_BATT_MV ((int)ocv[ARRAY_SIZE(ocv) - 1] - 500)

/* The esp32 ADC driver returns a calibrated pseudo-raw, so
 * adc_raw_to_millivolts_dt yields true pin millivolts. */
static const struct voltage_divider_dt_spec vbatt =
	VOLTAGE_DIVIDER_DT_SPEC_GET(DT_NODELABEL(vbatt));

#if DT_NODE_HAS_PROP(DT_NODELABEL(vbatt), power_gpios)
/* The V4 gates the divider behind ADC_CTRL (GPIO37) — the voltage-divider
 * binding's own `power-gpios`. Drive it high only while sampling: per the
 * schematic, asserting it costs ~3 mA (Q6's base current through the 1k R25 plus
 * the 10k R23 from VBAT), which dwarfs the 8.6 uA the divider itself draws. So
 * this is pulsed, never held — the same shape upstream Meshtastic and MeshCore
 * both use. Absent on the V4-R8, whose divider is permanently connected. */
static const struct gpio_dt_spec adc_ctrl_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(vbatt), power_gpios);
#define HAS_ADC_CTRL 1
#else
#define HAS_ADC_CTRL 0
#endif

static bool batt_setup_done;
static bool batt_ready;
static int batt_mv_cached = -1;
static int64_t batt_last_ms;

/* Serialises setup-and-sample. See the comment in meshtastic_battery_millivolts(). */
static K_MUTEX_DEFINE(batt_lock);

static void battery_setup(void)
{
	batt_setup_done = true;

	/* Safe to run lazily (after the radio is up): the channel is &adc0/GPIO1,
	 * which nothing else on this board uses, so setting it up here cannot
	 * disturb the radio's SPI lines regardless of when it runs. */
	if (!adc_is_ready_dt(&vbatt.port) || adc_channel_setup_dt(&vbatt.port) != 0) {
		LOG_WRN("vbatt ADC not ready; battery readout off");
		return;
	}
#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_configure_dt(&adc_ctrl_en, GPIO_OUTPUT_INACTIVE);
	}
#endif
	batt_ready = true;
}

int meshtastic_battery_millivolts(void)
{
	int64_t now;
	int64_t sum_mv = 0;
	int samples = 0;
	int32_t avg_mv;
	int result;

	/*
	 * Everything below runs under the lock. Four thread contexts reach this
	 * getter — the display, the telemetry collector, the shell, and the
	 * cutoff monitor's work item — and the sample sequence is not atomic: it
	 * pulses a shared GPIO, then k_sleep()s (which yields), then samples.
	 *
	 * Two callers that both miss the cache could otherwise interleave, and
	 * the first to finish would drop ADC_CTRL while the second was still
	 * sampling. That second caller then averages 15 reads of a divider that
	 * is no longer connected and returns a near-zero voltage — into, among
	 * other things, the low-voltage counter that can power the node off.
	 * The `batt_last_ms` write below hides this most of the time, but only
	 * once a cached value exists; the first read after boot is unguarded,
	 * which is exactly when the display, telemetry and the monitor's first
	 * poll are all warming up.
	 *
	 * A mutex is the right primitive: this blocks (k_sleep, ADC transfers),
	 * so it must not be a spinlock, and it is never called from an ISR.
	 */
	k_mutex_lock(&batt_lock, K_FOREVER);

	now = k_uptime_get();

	if (!batt_setup_done) {
		battery_setup();
	}
	if (!batt_ready) {
		result = -1;
		goto out;
	}
	if (batt_mv_cached >= 0 && (now - batt_last_ms) < BATTERY_CACHE_MS) {
		result = batt_mv_cached;
		goto out;
	}
	batt_last_ms = now;

#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_set_dt(&adc_ctrl_en, 1);
		k_sleep(K_MSEC(10)); /* matches upstream's battery_adcEnable() settle */
	}
#endif

	for (int i = 0; i < BATTERY_SENSE_SAMPLES; i++) {
		uint16_t raw = 0;
		int32_t mv;
		struct adc_sequence seq = {
			.buffer = &raw,
			.buffer_size = sizeof(raw),
		};

		if (adc_sequence_init_dt(&vbatt.port, &seq) != 0 ||
		    adc_read_dt(&vbatt.port, &seq) != 0) {
			continue;
		}
		mv = raw;
		if (adc_raw_to_millivolts_dt(&vbatt.port, &mv) != 0) {
			continue;
		}
		sum_mv += mv;
		samples++;
	}

#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_set_dt(&adc_ctrl_en, 0);
	}
#endif

	if (samples == 0) {
		batt_mv_cached = -1;
		result = -1;
		goto out;
	}

	/* Average pin mV, then x divider ratio (4.9, from the DT), then the
	 * upstream ADC_MULTIPLIER empirical term (CAL_PERMILLE/1000). */
	avg_mv = (int32_t)(sum_mv / samples);

	(void)voltage_divider_scale_dt(&vbatt, &avg_mv);
	avg_mv = (int32_t)((int64_t)avg_mv * CONFIG_MESHTASTIC_BATTERY_CAL_PERMILLE / 1000);
	batt_mv_cached = avg_mv;
	result = batt_mv_cached;

out:
	k_mutex_unlock(&batt_lock);
	return result;
}

int meshtastic_battery_percent(void)
{
	/* Interpolate the OCV curve, mirroring upstream's integer math. */
	const int n = (int)ARRAY_SIZE(ocv);
	int mv = meshtastic_battery_millivolts();

	if (mv < 0 || mv < BATTERY_NO_BATT_MV) {
		return -1; /* unavailable, or below the "no battery" floor (~2600 mV) */
	}
	for (int i = 0; i < n; i++) {
		if (mv >= (int)ocv[i]) {
			if (i == 0) {
				return 100;
			}
			int seg = (int)ocv[i - 1] - (int)ocv[i];
			int soc = 10 * (n - 1 - i) + (10 * (mv - (int)ocv[i])) / seg;

			return CLAMP(soc, 0, 100);
		}
	}
	return 0;
}

bool meshtastic_battery_present(void)
{
	int mv = meshtastic_battery_millivolts();

	return (mv >= 0) && (mv >= BATTERY_NO_BATT_MV);
}

bool meshtastic_battery_external_power(void)
{
	int mv = meshtastic_battery_millivolts();

	return (mv >= 0) && (mv > BATTERY_EXT_PWR_MV);
}

#if defined(CONFIG_MESHTASTIC_BATTERY_CUTOFF)

/*
 * Low-voltage cutoff — the thing that keeps a flat cell from being destroyed.
 *
 * A 1S LiPo taken below ~2.5 V dissolves the copper current collector on the
 * anode; the copper replates on the next charge, which generates gas and swells
 * the pouch (and can short it). Cell protection circuits normally stop this, but
 * they cut off at their own threshold, cannot be relied on when the cell's
 * provenance is unknown, and leave the pack sitting at that threshold to
 * self-discharge further. So the firmware stops first.
 *
 * Shape mirrors upstream (Power.cpp:1061): require N consecutive low readings
 * rather than acting on one, because voltage sags hard under a TX burst or a
 * WiFi association and recovers afterwards. At the default 30 s poll x 10
 * readings the pack must be genuinely low for ~5 minutes.
 */

/*
 * That guard only works if consecutive polls are independent measurements. Poll
 * faster than the read cache and meshtastic_battery_millivolts() hands back the
 * same sampled value repeatedly, so N "readings" collapse into one sample
 * counted N times — the counter reaches its threshold without the pack ever
 * being re-measured, and a single sag could power the node off.
 *
 * The Kconfig range keeps the interval clear of the cache window; this asserts
 * the relationship the range is protecting, so changing either one without the
 * other fails the build instead of quietly weakening the cutoff.
 */
BUILD_ASSERT(CONFIG_MESHTASTIC_BATTERY_CUTOFF_POLL_SEC * MSEC_PER_SEC > BATTERY_CACHE_MS,
	     "battery cutoff poll interval must exceed the read cache window, or "
	     "consecutive readings are the same cached sample counted repeatedly");

static uint8_t low_voltage_counter;
static bool battery_critical;

static void battery_monitor_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(battery_monitor_work, battery_monitor_work_fn);

static void battery_power_off(int mv)
{
	LOG_ERR("Battery critically low (%d mV, %u readings below %d mV): powering "
		"off to protect the cell. Recharge, then press RESET.",
		mv, (unsigned int)low_voltage_counter, CONFIG_MESHTASTIC_BATTERY_CUTOFF_MV);

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	(void)meshtastic_settings_flush();
#endif

	/* Same sequence as the admin shutdown path (meshtastic_admin.c) — keep the
	 * two in step. This call is the load-bearing one: DIO1 is armed as a wake
	 * source, so without disarming it the next frame off the mesh would wake a
	 * node that just powered off to save its cell, and it would keep doing that
	 * until the pack was destroyed. */
	meshtastic_radio_disarm_dio1_wake();

	/* sys_poweroff() irq_lock()s and never returns, so nothing queued survives
	 * it. LOG_PANIC() drains the backends synchronously; the extra delay gives
	 * the netlog's UDP socket time to actually put the frame on the wire, since
	 * that one leaves the SoC asynchronously. */
	LOG_PANIC();
	k_sleep(K_MSEC(CONFIG_MESHTASTIC_BATTERY_CUTOFF_FLUSH_MS));

	/* Nothing else is armed either, so this is off-until-reset rather than a
	 * nap: the cell stops discharging instead of trickling down to destruction
	 * while a timer keeps waking a node that has nothing left to run on. */
	sys_poweroff();
}

#if defined(CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN)
/*
 * The other way these cells die: held at full charge on a self-heating board.
 * Firmware cannot stop it — the charge IC is autonomous and neither Heltec V4
 * variant routes a charge-enable to a GPIO — so all we can do is say so.
 *
 * Deliberately low-rate: one line per day while the condition holds, nothing on
 * the display, nothing in telemetry.
 *
 * Two honest limitations, both consequences of having only a voltage to go on:
 *  - It measures uptime, not wall-clock history, so a node that reboots more
 *    often than the hold time never warns.
 *  - A board on USB with NO cell fitted is expected to read near the charge
 *    voltage as well (nothing loads VBAT down), and would warn identically.
 *    VERIFY(hardware): confirm what a cell-less board actually reads before
 *    trusting this not to cry wolf; if it does, this feature wants a real
 *    charger-status input rather than a cleverer threshold.
 */
static int64_t float_since_ms;
static int64_t float_last_warn_ms;

static void battery_float_check(int mv, int64_t now)
{
	int64_t held_h;

	if (mv < CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN_MV) {
		float_since_ms = 0;
		float_last_warn_ms = 0;
		return;
	}

	if (float_since_ms == 0) {
		float_since_ms = now;
		return;
	}

	held_h = (now - float_since_ms) / (3600 * 1000);
	if (held_h < CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN_HOURS) {
		return;
	}

	if (float_last_warn_ms != 0 && (now - float_last_warn_ms) < (int64_t)24 * 3600 * 1000) {
		return;
	}
	float_last_warn_ms = now;

	LOG_WRN("Battery has sat at %d mV (>= %d) for %lld h. Holding a LiPo at full "
		"charge on a warm board ages it and can swell the pouch — unplug the "
		"cell if this node lives on USB.",
		mv, CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN_MV, held_h);
}
#endif /* CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN */

static void battery_monitor_work_fn(struct k_work *work)
{
	int mv = meshtastic_battery_millivolts();

	ARG_UNUSED(work);

	if (mv < 0) {
		/* No reading at all (ADC not ready, or every sample failed). Never
		 * act on an absent measurement — an unreadable ADC must not be able
		 * to switch the node off. */
		low_voltage_counter = 0;
		battery_critical = false;
		goto reschedule;
	}

	if (mv < BATTERY_NO_BATT_MV) {
		/* No cell fitted. A mains-powered node with an empty JST reads here,
		 * and powering that off would strand it — upstream gates the same
		 * check on getHasBattery() for exactly this reason. */
		low_voltage_counter = 0;
		battery_critical = false;
		goto reschedule;
	}

#if defined(CONFIG_MESHTASTIC_BATTERY_FLOAT_WARN)
	battery_float_check(mv, k_uptime_get());
#endif

	if (mv >= CONFIG_MESHTASTIC_BATTERY_CUTOFF_MV) {
		if (low_voltage_counter != 0) {
			LOG_INF("Battery recovered to %d mV; low-voltage count cleared", mv);
		}
		low_voltage_counter = 0;
		battery_critical = false;
		goto reschedule;
	}

	battery_critical = true;
	if (low_voltage_counter < UINT8_MAX) {
		low_voltage_counter++;
	}
	LOG_WRN("Battery low: %d mV (%u/%d readings below %d mV)", mv,
		(unsigned int)low_voltage_counter, CONFIG_MESHTASTIC_BATTERY_CUTOFF_COUNT,
		CONFIG_MESHTASTIC_BATTERY_CUTOFF_MV);

	if (low_voltage_counter >= CONFIG_MESHTASTIC_BATTERY_CUTOFF_COUNT) {
		battery_power_off(mv); /* does not return */
	}

reschedule:
	k_work_schedule(&battery_monitor_work,
			K_SECONDS(CONFIG_MESHTASTIC_BATTERY_CUTOFF_POLL_SEC));
}

bool meshtastic_battery_is_critical(void)
{
	return battery_critical;
}

static int battery_monitor_init(void)
{
	/* Deliberately late: the first seconds after boot are the worst possible
	 * time to judge the pack, with radio and WiFi bring-up inrush sagging the
	 * rail and the ADC not necessarily set up yet. */
	k_work_schedule(&battery_monitor_work,
			K_SECONDS(CONFIG_MESHTASTIC_BATTERY_CUTOFF_START_DELAY_SEC));
	return 0;
}

SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else /* !CONFIG_MESHTASTIC_BATTERY_CUTOFF */

bool meshtastic_battery_is_critical(void)
{
	return false;
}

#endif /* CONFIG_MESHTASTIC_BATTERY_CUTOFF */

#else /* no vbatt node on this board */

int meshtastic_battery_millivolts(void)
{
	return -1;
}

int meshtastic_battery_percent(void)
{
	return -1;
}

bool meshtastic_battery_present(void)
{
	return false;
}

bool meshtastic_battery_external_power(void)
{
	return false;
}

bool meshtastic_battery_is_critical(void)
{
	return false;
}

#endif /* vbatt */
