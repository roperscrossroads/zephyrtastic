/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The bench LED (meshtastic_led_status.h), observed on emulated GPIO: each
 * mode's pattern on a one-LED board (native_sim's led0) and on the XIAO's RGB
 * (boards/rgb.overlay, active low), locate's timeout and its interruptions,
 * and -- in the settings build -- the mode surviving a reboot.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include "meshtastic_led_status.h"

#define HAS_COLOR (DT_NODE_EXISTS(DT_ALIAS(led1)) && DT_NODE_EXISTS(DT_ALIAS(led2)))

static const struct gpio_dt_spec red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#if HAS_COLOR
static const struct gpio_dt_spec green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

/* The LED as a person sees it: lit or not, whatever the pin's polarity. */
static bool lit(const struct gpio_dt_spec *s)
{
	int phys = gpio_emul_output_get(s->port, s->pin);

	return ((s->dt_flags & GPIO_ACTIVE_LOW) != 0U) ? (phys == 0) : (phys == 1);
}

struct watch {
	uint32_t on_edges; /* dark -> lit transitions */
	uint32_t lit_ms;
	uint32_t ms;
};

/* Sample @p s every 5 ms for @p ms. */
static struct watch watch(const struct gpio_dt_spec *s, uint32_t ms)
{
	struct watch w = { 0 };
	bool was = lit(s);
	const int64_t start = k_uptime_get();
	int64_t t = start;

	/* Bounded by the clock, not by a count of sleeps: a sleep rounds up to
	 * the tick. */
	while (t - start < (int64_t)ms) {
		bool now = lit(s);
		int64_t next;

		if (now && !was) {
			w.on_edges++;
		}
		was = now;
		k_msleep(5);
		next = k_uptime_get();
		w.lit_ms += now ? (uint32_t)(next - t) : 0U;
		t = next;
	}
	w.ms = (uint32_t)(t - start);
	return w;
}

static void *setup(void)
{
#if defined(CONFIG_MESHTASTIC_SETTINGS)
	zassert_ok(settings_subsys_init());
	(void)settings_load();
#endif
	zassert_ok(meshtastic_led_status_init());
	return NULL;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	meshtastic_led_status_locate(0);
	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_IDLE));
	k_msleep(10);
}

ZTEST_SUITE(led_status, NULL, setup, before, NULL, NULL);

ZTEST(led_status, test_color_detection_matches_the_board)
{
	zassert_equal(meshtastic_led_status_has_color(), HAS_COLOR);
}

/* Idle: a slow blink on led0 (red on the XIAO) -- lit briefly about every 2 s. */
ZTEST(led_status, test_idle_is_a_slow_blink)
{
	struct watch w = watch(&red, 4100);

	zassert_true(w.on_edges >= 2U && w.on_edges <= 3U, "idle edges %u in 4.1 s", w.on_edges);
	zassert_true(w.lit_ms < w.ms / 4U, "idle lit %u of %u ms: not a slow blink", w.lit_ms, w.ms);
#if HAS_COLOR
	zassert_false(lit(&green) || lit(&blue), "idle is red only");
#endif
}

/* In use: steady and quiet -- solid green on the XIAO, dark on a one-LED board. */
ZTEST(led_status, test_in_use_is_steady)
{
	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_IN_USE));
	k_msleep(20);
	struct watch w = watch(&red, 2500);

	zassert_equal(w.lit_ms, 0U, "led0 lit %u ms while in use", w.lit_ms);
#if HAS_COLOR
	struct watch g = watch(&green, 1500);

	zassert_equal(g.lit_ms, g.ms, "green not solid while in use");
	zassert_false(lit(&blue));
#endif
	zassert_equal(meshtastic_led_status_get_mode(), MESHTASTIC_LED_IN_USE);
}

ZTEST(led_status, test_off_is_dark)
{
	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_OFF));
	k_msleep(20);
	zassert_equal(watch(&red, 2500).lit_ms, 0U);
#if HAS_COLOR
	zassert_false(lit(&green) || lit(&blue));
#endif
}

/* Locate: a fast blink (blue on the XIAO) that ends by itself and returns to
 * the mode it interrupted. */
ZTEST(led_status, test_locate_blinks_fast_then_returns)
{
#if HAS_COLOR
	const struct gpio_dt_spec *loc = &blue;
#else
	const struct gpio_dt_spec *loc = &red;
#endif
	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_IN_USE));
	meshtastic_led_status_locate(1);
	zassert_equal(meshtastic_led_status_locate_left(), 1U);
	struct watch w = watch(loc, 900);

	zassert_true(w.on_edges >= 4U, "locate edges %u in 0.9 s: not fast", w.on_edges);
	k_msleep(300);
	zassert_equal(meshtastic_led_status_locate_left(), 0U);
	zassert_equal(watch(&red, 1200).lit_ms, 0U, "not back to in-use after locate");
#if HAS_COLOR
	zassert_true(lit(&green), "not back to green after locate");
	zassert_false(lit(&blue));
#endif
	zassert_equal(meshtastic_led_status_get_mode(), MESHTASTIC_LED_IN_USE,
		      "locate must not change the saved mode");
}

ZTEST(led_status, test_locate_zero_stops_and_a_mode_ends_it)
{
	meshtastic_led_status_locate(60);
	k_msleep(50);
	meshtastic_led_status_locate(0);
	zassert_equal(meshtastic_led_status_locate_left(), 0U);

	meshtastic_led_status_locate(60);
	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_OFF));
	zassert_equal(meshtastic_led_status_locate_left(), 0U, "a new mode ends a locate");
	k_msleep(20);
	zassert_equal(watch(&red, 500).lit_ms, 0U);
}

ZTEST(led_status, test_an_unknown_mode_is_refused)
{
	zassert_equal(meshtastic_led_status_set_mode((enum meshtastic_led_mode)7), -EINVAL);
	zassert_equal(meshtastic_led_status_get_mode(), MESHTASTIC_LED_IDLE);
}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static int read_mode_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			void *param)
{
	uint8_t *out = param;

	if (key == NULL && len == 1U) {
		(void)read_cb(cb_arg, out, 1);
	}
	return 0;
}

/* The mode is written to mtled/mode, and a value found there at boot is the
 * mode the LED starts in. */
ZTEST(led_status, test_the_mode_survives_a_reboot)
{
	uint8_t stored = 0xFF;

	zassert_ok(meshtastic_led_status_set_mode(MESHTASTIC_LED_IN_USE));
	zassert_ok(settings_load_subtree_direct("mtled/mode", read_mode_cb, &stored));
	zassert_equal(stored, (uint8_t)MESHTASTIC_LED_IN_USE, "not saved");

	/* A "reboot": what is in flash is what the handler hands the module. */
	const uint8_t off = (uint8_t)MESHTASTIC_LED_OFF;

	zassert_ok(settings_save_one("mtled/mode", &off, 1));
	zassert_ok(settings_load_subtree("mtled"));
	zassert_equal(meshtastic_led_status_get_mode(), MESHTASTIC_LED_OFF, "not restored");
}
#endif
