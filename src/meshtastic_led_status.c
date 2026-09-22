/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_led_status.h. */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_led_status.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define LED_STATUS_HAS_LED DT_NODE_EXISTS(DT_ALIAS(led0))

#if LED_STATUS_HAS_LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif

static struct k_work_delayable tick_work;
static bool led_on;

/* Any peer link up, either direction: an outbound link this node dialed
 * (ready = discovery/subscribe complete, beats flowing), or an inbound one
 * where a connected central has enabled notifications on our peripheral
 * service. Either is real evidence of an active BLE mesh link, not just a
 * pending connection attempt. */
static bool peer_meshing(void)
{
	struct meshtastic_ble_peer_link link;

	meshtastic_ble_peer_link_get(&link);
	return link.ready || meshtastic_ble_peer_notify_ready();
}

static void set_led(bool on)
{
	led_on = on;
#if LED_STATUS_HAS_LED
	(void)gpio_pin_set_dt(&led, on ? 1 : 0);
#endif
}

static void tick_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (peer_meshing()) {
		/* Toggle: with a periodic reschedule at the tick period, this is
		 * exactly the 1-on/1-off blink. */
		set_led(!led_on);
	} else {
		/* Re-assert solid every tick rather than only on the
		 * blink->solid edge: simpler, and a stray external GPIO write
		 * (there is none today, but nothing enforces that) self-heals
		 * within one tick instead of staying wrong until the next
		 * meshing transition. */
		set_led(true);
	}
	(void)k_work_reschedule(&tick_work, K_MSEC(CONFIG_MESHTASTIC_LED_STATUS_TICK_MS));
}

int meshtastic_led_status_init(void)
{
	k_work_init_delayable(&tick_work, tick_work_fn);
#if LED_STATUS_HAS_LED
	if (!gpio_is_ready_dt(&led)) {
		LOG_WRN("LedStatus: led0 not ready");
		return -ENODEV;
	}
	(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
#else
	LOG_WRN("LedStatus: no led0 on this board — enabled with nothing to drive");
#endif
	set_led(true); /* solid until the first tick finds a live peer link */
	(void)k_work_reschedule(&tick_work, K_MSEC(CONFIG_MESHTASTIC_LED_STATUS_TICK_MS));
	return 0;
}
