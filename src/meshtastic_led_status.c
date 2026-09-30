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

static void led_work_fn(struct k_work *work)
{
	const int64_t now = k_uptime_get();
	uint32_t next;

	ARG_UNUSED(work);

	if (locate_until != 0 && now < locate_until) {
		phase = !phase;
		drive(false, false, phase);
		next = LOCATE_MS;
	} else {
		locate_until = 0;
		switch (led_mode) {
		case MESHTASTIC_LED_IN_USE:
			drive(false, true, false);
			next = STEADY_MS;
			break;
		case MESHTASTIC_LED_OFF:
			drive(false, false, false);
			next = STEADY_MS;
			break;
		case MESHTASTIC_LED_IDLE:
		default:
			phase = !phase;
			drive(phase, false, false);
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

/* ---- persistence: the mode survives a reboot (mtled/mode) ------------------- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static int led_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t m;

	if (strcmp(key, "mode") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(m) || read_cb(cb_arg, &m, len) != (ssize_t)len) {
		return -EINVAL;
	}
	if (m <= (uint8_t)MESHTASTIC_LED_OFF) {
		led_mode = (enum meshtastic_led_mode)m;
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_led, "mtled", NULL, led_settings_set, NULL, NULL);

static void led_save(enum meshtastic_led_mode mode)
{
	const uint8_t m = (uint8_t)mode;

	if (settings_save_one("mtled/mode", &m, sizeof(m)) != 0) {
		LOG_WRN("led: settings save failed");
	}
}
#else
static void led_save(enum meshtastic_led_mode mode)
{
	ARG_UNUSED(mode);
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
	led_save(mode);
	(void)k_work_reschedule(&led_work, K_NO_WAIT);
	return 0;
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
