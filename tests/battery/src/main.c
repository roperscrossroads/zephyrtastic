/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Battery divider read-path tests (agents-wu94.1).
 *
 * meshtastic_battery.c had ZERO automated coverage before this suite (noted in
 * docs/power-and-battery/BATTERY-STATUS.md as one of the reasons the whole
 * arc sat unmerged for a month). Drives the module's REAL implementation --
 * not a reimplementation of it -- against native_sim's emulated ADC
 * (zephyr,adc-emul), wired up via a `vbatt` voltage-divider node in the board
 * overlay with the same channel config (ADC_GAIN_4_5 / ADC_REF_INTERNAL /
 * 12-bit) as the real Heltec V4/V4-R8 board file.
 *
 * Out of scope here: the low-voltage cutoff state machine
 * (CONFIG_MESHTASTIC_BATTERY_CUTOFF). It depends on HAS_POWEROFF, which
 * native_sim does not have, so it cannot even be selected in this image. Its
 * pure decision logic is covered separately by main/tests/battery_cutoff,
 * which has no ADC or poweroff dependency at all.
 *
 * Expected values are computed with the SAME integer arithmetic
 * meshtastic_battery_millivolts() uses (pin_mv * full_ohms / output_ohms,
 * then * CAL_PERMILLE / 1000, both truncating), not floating point, so a
 * changed rounding behaviour shows up as a real assertion failure rather
 * than being absorbed by an overly generous tolerance. A small epsilon still
 * covers the ADC emulator's own raw<->mV round-trip through its resolution.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "meshtastic_battery.h"

/* Real Heltec V4 divider: 390k upper / 100k lower -> full/output = 4.9. */
#define DIVIDER_FULL_OHMS   490000
#define DIVIDER_OUTPUT_OHMS 100000

/* Default CAL_PERMILLE (Kconfig default, no R8 override applies on
 * native_sim -- there is no board-specific defconfig for a host arch). */
#define CAL_PERMILLE 1045

/* meshtastic_battery.c's own BATTERY_CACHE_MS. Not exposed via the header
 * (it is a read-path implementation detail, not part of the module's
 * contract), so this is a second source of truth that has to be kept in step
 * by hand if that constant ever moves -- same tradeoff the GNSS suite makes
 * with EMUL_FIX_INTERVAL_MS. */
#define CACHE_MS 5000

/* ADC round-trip tolerance (emulator quantization at 12-bit resolution,
 * compounded through two integer-truncating multiply/divide stages). */
#define MV_EPS 40

static const struct device *adc_dev;

static void *battery_suite_setup(void)
{
	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
	zassert_true(device_is_ready(adc_dev), "emulated ADC not ready");
	return NULL;
}

/* Sets the pin voltage (post-divider, at the ADC input -- what a multimeter
 * at GPIO1 would read) and waits past the read cache so the next call to
 * meshtastic_battery_millivolts() takes a fresh sample rather than returning
 * whatever the previous test case cached. */
static void set_pin_mv_and_wait(int pin_mv)
{
	int ret = adc_emul_const_value_set(adc_dev, 0, (uint32_t)pin_mv);

	zassert_ok(ret, "adc_emul_const_value_set(%d) failed: %d", pin_mv, ret);
	k_sleep(K_MSEC(CACHE_MS + 200));
}

/* Same integer arithmetic as meshtastic_battery_millivolts(): scale by the
 * divider ratio, then apply CAL_PERMILLE, both truncating. */
static int expected_final_mv(int pin_mv)
{
	int32_t scaled = (int32_t)((int64_t)pin_mv * DIVIDER_FULL_OHMS / DIVIDER_OUTPUT_OHMS);

	return (int32_t)((int64_t)scaled * CAL_PERMILLE / 1000);
}

ZTEST_SUITE(meshtastic_battery, NULL, battery_suite_setup, NULL, NULL, NULL);

ZTEST(meshtastic_battery, test_reads_scaled_voltage)
{
	/* Comfortably mid-curve: not near either threshold, so this is purely a
	 * scaling-math check. */
	const int pin_mv = 742;
	const int expected = expected_final_mv(pin_mv);
	int mv;

	set_pin_mv_and_wait(pin_mv);

	mv = meshtastic_battery_millivolts();
	zassert_within(mv, expected, MV_EPS, "got %d mV, expected ~%d mV", mv, expected);
	zassert_true(meshtastic_battery_present(), "a mid-curve reading must count as present");
	zassert_false(meshtastic_battery_external_power(), "mid-curve is not charging");
	zassert_false(meshtastic_battery_is_critical(),
		      "cutoff is compiled out on native_sim; must never report critical");
}

ZTEST(meshtastic_battery, test_top_of_curve_is_100_percent)
{
	/* Comfortably above the OCV table's top entry (4190 mV) -- comfortably
	 * meaning well clear of the emulated ADC's own quantization noise (see
	 * MV_EPS), not merely above 4190 on paper. The real gap between "top of
	 * the resting curve" and "charge-termination threshold" (4200 mV) is
	 * only 10 mV, too narrow to also assert not-charging here without
	 * flaking on ADC noise; that boundary gets its own test below instead. */
	const int pin_mv = 850;

	set_pin_mv_and_wait(pin_mv);

	zassert_equal(meshtastic_battery_percent(), 100, "at/above the curve's top is 100%%");
}

ZTEST(meshtastic_battery, test_above_charge_termination_reads_external_power)
{
	/* Above OCV_max + 10 mV: nothing can rest there, so this is the port's
	 * only "is something charging this" signal (see meshtastic_battery.c's
	 * top-of-file comment on why that's the only signal available at all). */
	const int pin_mv = 840;

	set_pin_mv_and_wait(pin_mv);

	zassert_true(meshtastic_battery_external_power(),
		     "above the charge-termination voltage must read as charging");
	zassert_true(meshtastic_battery_present(), "still a valid, present reading");
}

ZTEST(meshtastic_battery, test_bottom_of_curve_is_0_percent_not_absent)
{
	/* Below the OCV table's last entry (3100 mV) but well above the
	 * "no battery" floor (2600 mV): a genuinely flat-but-fitted cell, which
	 * must read 0%, not "absent". */
	const int pin_mv = 605;

	set_pin_mv_and_wait(pin_mv);

	zassert_true(meshtastic_battery_present(), "still above the no-battery floor");
	zassert_equal(meshtastic_battery_percent(), 0, "below the curve's bottom clamps to 0%%");
}

ZTEST(meshtastic_battery, test_below_floor_reads_absent_not_a_bad_percent)
{
	/* Below the ~2600 mV no-battery floor: what a USB-powered board with an
	 * empty JST reads. millivolts() still reports the real divider voltage
	 * (nothing gates that) -- only present()/percent() treat it specially,
	 * because a mains node must never be penalised as if it had a dying cell. */
	const int pin_mv = 390;
	const int expected = expected_final_mv(pin_mv);
	int mv;

	set_pin_mv_and_wait(pin_mv);

	mv = meshtastic_battery_millivolts();
	zassert_within(mv, expected, MV_EPS, "got %d mV, expected ~%d mV", mv, expected);
	zassert_false(meshtastic_battery_present(), "below the floor must read as no battery");
	zassert_equal(meshtastic_battery_percent(), -1, "no battery has no percentage");
	zassert_false(meshtastic_battery_external_power(), "an absent reading is not 'charging'");
}

ZTEST(meshtastic_battery, test_cache_holds_then_refreshes)
{
	const int pin_a = 742;
	const int pin_b = 390;
	const int expected_a = expected_final_mv(pin_a);
	const int expected_b = expected_final_mv(pin_b);
	int mv;

	/* Populate the cache with a known reading. */
	set_pin_mv_and_wait(pin_a);
	mv = meshtastic_battery_millivolts();
	zassert_within(mv, expected_a, MV_EPS, "initial sample: got %d, expected ~%d", mv,
		       expected_a);

	/* Change the input, but read again immediately -- inside the cache
	 * window, this MUST still return the stale value. If a caller could see
	 * a fresh sample here, two racing callers (see the module's own comment
	 * on why this matters) could observe the pin mid-transition instead of a
	 * single consistent reading. */
	zassert_ok(adc_emul_const_value_set(adc_dev, 0, (uint32_t)pin_b));
	mv = meshtastic_battery_millivolts();
	zassert_within(mv, expected_a, MV_EPS,
		       "read inside the cache window returned %d, expected the STALE ~%d", mv,
		       expected_a);

	/* Past the cache window, the next read must reflect the new input. */
	k_sleep(K_MSEC(CACHE_MS + 200));
	mv = meshtastic_battery_millivolts();
	zassert_within(mv, expected_b, MV_EPS,
		       "read past the cache window returned %d, expected the fresh ~%d", mv,
		       expected_b);
}
