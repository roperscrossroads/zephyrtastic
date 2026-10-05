/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_led_status.h. */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include "meshtastic_led_status.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* led0 is the one LED every board has: red on the XIAO's RGB, white on the
 * Heltecs. Green and blue exist only where the board declares led1 and led2. */
#define LED_HAS_MAIN  DT_NODE_EXISTS(DT_ALIAS(led0))
#define LED_HAS_COLOR (LED_HAS_MAIN && DT_NODE_EXISTS(DT_ALIAS(led1)) && \
		       DT_NODE_EXISTS(DT_ALIAS(led2)))

#if LED_HAS_MAIN
static const struct gpio_dt_spec led_main = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#if LED_HAS_COLOR
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

/* The patterns, in ms. Slow enough that a hub of idle boards is calm; fast
 * enough that a locating board is unmistakable beside them. */
#define IDLE_ON_MS    150
#define IDLE_OFF_MS   1850
#define LOCATE_MS     100
#define STEADY_MS     1000 /* re-assert a steady state now and then */

static struct k_work_delayable led_work;
static enum meshtastic_led_mode led_mode = MESHTASTIC_LED_IDLE;
static enum meshtastic_led_color led_color = MESHTASTIC_LED_COLOR_DEFAULT;
static int64_t locate_until; /* k_uptime; 0 when not locating */
static bool phase;

/* Drive the LEDs. On a one-LED board, "red" and "blue" both mean led0 and
 * "green" means dark: in-use is the quiet state there. */
static void drive(bool red, bool green, bool blue)
{
#if LED_HAS_COLOR
	(void)gpio_pin_set_dt(&led_main, red ? 1 : 0);
	(void)gpio_pin_set_dt(&led_green, green ? 1 : 0);
	(void)gpio_pin_set_dt(&led_blue, blue ? 1 : 0);
#elif LED_HAS_MAIN
	ARG_UNUSED(green);
	(void)gpio_pin_set_dt(&led_main, (red || blue) ? 1 : 0);
#else
	ARG_UNUSED(red);
	ARG_UNUSED(green);
	ARG_UNUSED(blue);
#endif
}

static const char *const color_names[] = {
	[MESHTASTIC_LED_COLOR_DEFAULT] = "default", [MESHTASTIC_LED_COLOR_RED] = "red",
	[MESHTASTIC_LED_COLOR_GREEN] = "green",     [MESHTASTIC_LED_COLOR_BLUE] = "blue",
	[MESHTASTIC_LED_COLOR_YELLOW] = "yellow",   [MESHTASTIC_LED_COLOR_CYAN] = "cyan",
	[MESHTASTIC_LED_COLOR_MAGENTA] = "magenta",
};

/* The colour lit, or dark. */
static void show(enum meshtastic_led_color c, bool on)
{
	static const uint8_t rgb[] = {
		[MESHTASTIC_LED_COLOR_DEFAULT] = 0, [MESHTASTIC_LED_COLOR_RED] = 4,
		[MESHTASTIC_LED_COLOR_GREEN] = 2,   [MESHTASTIC_LED_COLOR_BLUE] = 1,
		[MESHTASTIC_LED_COLOR_YELLOW] = 6,  [MESHTASTIC_LED_COLOR_CYAN] = 3,
		[MESHTASTIC_LED_COLOR_MAGENTA] = 5,
	};
	const uint8_t v = on ? rgb[c] : 0U;

	drive((v & 4U) != 0U, (v & 2U) != 0U, (v & 1U) != 0U);
}

static void led_work_fn(struct k_work *work)
{
	const int64_t now = k_uptime_get();
	/* The colour is shown only where there are colours: a one-LED board keeps its
	 * patterns whatever colour is stored (drive() ORs red and blue onto its LED, so a
	 * magenta "in use" would light it, and a green "idle" would never blink). */
	const bool tint = LED_HAS_COLOR && led_color != MESHTASTIC_LED_COLOR_DEFAULT;
	uint32_t next;

	ARG_UNUSED(work);

	if (locate_until != 0 && now < locate_until) {
		phase = !phase;
		if (!tint) {
			drive(false, false, phase);
		} else {
			drive(phase, phase, phase); /* white: no purpose has it */
		}
		next = LOCATE_MS;
	} else {
		locate_until = 0;
		switch (led_mode) {
		case MESHTASTIC_LED_IN_USE:
			if (!tint) {
				drive(false, true, false);
			} else {
				show(led_color, true);
			}
			next = STEADY_MS;
			break;
		case MESHTASTIC_LED_OFF:
			drive(false, false, false);
			next = STEADY_MS;
			break;
		case MESHTASTIC_LED_IDLE:
		default:
			phase = !phase;
			if (!tint) {
				drive(phase, false, false);
			} else {
				show(led_color, phase);
			}
			next = phase ? IDLE_ON_MS : IDLE_OFF_MS;
			break;
		}
	}
	(void)k_work_reschedule(&led_work, K_MSEC(next));
}

const char *meshtastic_led_mode_name(enum meshtastic_led_mode mode)
{
	switch (mode) {
	case MESHTASTIC_LED_IDLE:
		return "idle";
	case MESHTASTIC_LED_IN_USE:
		return "in-use";
	case MESHTASTIC_LED_OFF:
		return "off";
	default:
		return "?";
	}
}

bool meshtastic_led_status_has_color(void)
{
	return LED_HAS_COLOR;
}

/* ---- persistence: the mode and colour survive a reboot (mtled/mode, mtled/color) -- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static int led_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t m;
	const bool is_mode = strcmp(key, "mode") == 0;

	if (!is_mode && strcmp(key, "color") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(m) || read_cb(cb_arg, &m, len) != (ssize_t)len) {
		return -EINVAL;
	}
	if (is_mode && m <= (uint8_t)MESHTASTIC_LED_OFF) {
		led_mode = (enum meshtastic_led_mode)m;
	} else if (!is_mode && m <= (uint8_t)MESHTASTIC_LED_COLOR_MAGENTA) {
		led_color = (enum meshtastic_led_color)m;
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_led, "mtled", NULL, led_settings_set, NULL, NULL);

static void led_save(const char *key, uint8_t v)
{
	if (settings_save_one(key, &v, sizeof(v)) != 0) {
		LOG_WRN("led: settings save failed");
	}
}
#else
static void led_save(const char *key, uint8_t v)
{
	ARG_UNUSED(key);
	ARG_UNUSED(v);
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

int meshtastic_led_status_set_mode(enum meshtastic_led_mode mode)
{
	if (mode > MESHTASTIC_LED_OFF) {
		return -EINVAL;
	}
	led_mode = mode;
	locate_until = 0;
	phase = false;
	led_save("mtled/mode", (uint8_t)mode);
	(void)k_work_reschedule(&led_work, K_NO_WAIT);
	return 0;
}

int meshtastic_led_status_set_color(enum meshtastic_led_color color)
{
	if (color > MESHTASTIC_LED_COLOR_MAGENTA) {
		return -EINVAL;
	}
	led_color = color;
	phase = false;
	led_save("mtled/color", (uint8_t)color);
	(void)k_work_reschedule(&led_work, K_NO_WAIT);
	return 0;
}

enum meshtastic_led_color meshtastic_led_status_get_color(void)
{
	return led_color;
}

const char *meshtastic_led_color_name(enum meshtastic_led_color color)
{
	return (color <= MESHTASTIC_LED_COLOR_MAGENTA) ? color_names[color] : "?";
}

int meshtastic_led_color_parse(const char *name, enum meshtastic_led_color *out)
{
	for (size_t i = 0; i < ARRAY_SIZE(color_names); i++) {
		if (strcmp(name, color_names[i]) == 0) {
			*out = (enum meshtastic_led_color)i;
			return 0;
		}
	}
	return -EINVAL;
}

enum meshtastic_led_mode meshtastic_led_status_get_mode(void)
{
	return led_mode;
}

void meshtastic_led_status_locate(uint32_t seconds)
{
	locate_until = (seconds == 0U) ? 0 : k_uptime_get() + (int64_t)seconds * 1000;
	phase = false;
	(void)k_work_reschedule(&led_work, K_NO_WAIT);
}

uint32_t meshtastic_led_status_locate_left(void)
{
	const int64_t left = locate_until - k_uptime_get();

	return (locate_until == 0 || left <= 0) ? 0U : (uint32_t)((left + 999) / 1000);
}

int meshtastic_led_status_init(void)
{
	k_work_init_delayable(&led_work, led_work_fn);
#if LED_HAS_MAIN
	if (!gpio_is_ready_dt(&led_main)) {
		LOG_WRN("led: led0 not ready");
		return -ENODEV;
	}
	(void)gpio_pin_configure_dt(&led_main, GPIO_OUTPUT_INACTIVE);
#if LED_HAS_COLOR
	(void)gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
#endif
#else
	LOG_WRN("led: no led0 on this board -- enabled with nothing to drive");
#endif
	(void)k_work_reschedule(&led_work, K_NO_WAIT);
	return 0;
}
