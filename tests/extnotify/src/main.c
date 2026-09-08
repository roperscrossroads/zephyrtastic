/* SPDX-License-Identifier: GPL-3.0 */
/*
 * External notification (agents-dnr4.17), driven through the sim radio and
 * observed on the emulated GPIO behind led0.
 *
 * In the reference's terms (ExternalNotificationModule.cpp):
 *   - a text from another node drives the output for output_ms, then off;
 *   - with nag_timeout the output toggles every output_ms until the timeout;
 *   - alert_bell alone fires only on a message carrying ASCII BEL;
 *   - a broadcast on a muted channel does not alert (a bell still does);
 *   - `active` false inverts the drive level;
 *   - our own text, or the module disabled, never alerts; stop ends a cycle;
 *   - a config naming outputs this port has none of is refused.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_extnotify.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU
#define PEER_A       0x0B000001U

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const struct device *const led_port = DEVICE_DT_GET(DT_GPIO_CTLR(DT_ALIAS(led0), gpios));
#define LED_PIN DT_GPIO_PIN(DT_ALIAS(led0), gpios)

static int led_level(void)
{
	return gpio_emul_output_get(led_port, LED_PIN);
}

static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

static uint32_t next_id = 0x50000000U;

static void inject_text(uint32_t from, uint32_t to, const char *text)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = next_id++,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)text,
		.payload_len = strlen(text),
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, &wire_len), "build failed");
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -60, 6), "inject failed");
	k_msleep(50);
}

static meshtastic_ModuleConfig_ExternalNotificationConfig cfg_led(bool enabled, uint32_t out_ms,
								    uint32_t nag_s, bool msg,
								    bool bell)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c =
		meshtastic_ModuleConfig_ExternalNotificationConfig_init_zero;

	c.enabled = enabled;
	c.active = true;
	c.output_ms = out_ms;
	c.nag_timeout = nag_s;
	c.alert_message = msg;
	c.alert_bell = bell;
	return c;
}

static void *extnotify_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_true(device_is_ready(led_port), "emulated gpio not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void extnotify_before(void *fixture)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig off = cfg_led(false, 0U, 0U, true, false);

	ARG_UNUSED(fixture);

	zassert_ok(meshtastic_extnotify_set(&off), "disable failed");
	meshtastic_extnotify_reset();
	/* A test that aborts inside the mute case must not leave the channel
	 * muted for the ones after it. */
	{
		uint8_t primary = meshtastic_channels_primary_index();
		meshtastic_Channel ch = *meshtastic_channels_get(primary);

		if (ch.settings.has_module_settings && ch.settings.module_settings.is_muted) {
			ch.settings.module_settings.is_muted = false;
			zassert_ok(meshtastic_channels_set_slot(primary, &ch), "unmute");
		}
	}
	k_msleep(50);
	lora_sim_reset(lora_dev);
}

ZTEST_SUITE(extnotify, NULL, extnotify_setup, extnotify_before, NULL, NULL);

ZTEST(extnotify, test_a_text_pulses_the_led_for_output_ms)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 0U, true, false);
	struct meshtastic_extnotify_stats st;

	zassert_ok(meshtastic_extnotify_set(&c), "");
	zassert_equal(led_level(), 0, "idle: off");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "hello");
	zassert_equal(led_level(), 1, "alert: on");
	zassert_true(meshtastic_extnotify_nagging(), "");

	k_sleep(K_MSEC(700));
	zassert_equal(led_level(), 1, "still on inside output_ms");
	k_sleep(K_MSEC(500));
	zassert_equal(led_level(), 0, "off after output_ms (no nag)");
	zassert_false(meshtastic_extnotify_nagging(), "cycle ended");
	meshtastic_extnotify_stats(&st);
	zassert_equal(st.alerts, 1U, "");
}

ZTEST(extnotify, test_nag_toggles_every_output_ms_until_the_timeout)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 5U, true, false);

	zassert_ok(meshtastic_extnotify_set(&c), "");
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "hello");
	zassert_equal(led_level(), 1, "t=0: on");
	k_sleep(K_MSEC(1200));
	zassert_equal(led_level(), 0, "t=1.2: toggled off");
	k_sleep(K_MSEC(1000));
	zassert_equal(led_level(), 1, "t=2.2: toggled on");
	zassert_true(meshtastic_extnotify_nagging(), "still nagging");
	k_sleep(K_MSEC(3500));
	zassert_equal(led_level(), 0, "t=5.7: past nag_timeout, off");
	zassert_false(meshtastic_extnotify_nagging(), "cycle ended");
}

ZTEST(extnotify, test_bell_only_fires_on_a_bel_character)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 0U, false, true);
	struct meshtastic_extnotify_stats st;

	zassert_ok(meshtastic_extnotify_set(&c), "");
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "plain text");
	zassert_equal(led_level(), 0, "no bell, alert_message off: nothing");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "ding \a");
	zassert_equal(led_level(), 1, "a BEL alerts");
	meshtastic_extnotify_stats(&st);
	zassert_equal(st.bells, 1U, "");
	meshtastic_extnotify_stop();
}

ZTEST(extnotify, test_a_muted_channel_does_not_alert_but_a_bell_still_does)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 0U, true, true);
	uint8_t primary = meshtastic_channels_primary_index();
	meshtastic_Channel saved = *meshtastic_channels_get(primary);
	meshtastic_Channel muted = saved;
	struct meshtastic_extnotify_stats st;

	zassert_ok(meshtastic_extnotify_set(&c), "");
	muted.settings.has_module_settings = true;
	muted.settings.module_settings.is_muted = true;
	zassert_ok(meshtastic_channels_set_slot(primary, &muted), "");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "quiet please");
	zassert_equal(led_level(), 0, "muted channel: no alert");
	meshtastic_extnotify_stats(&st);
	zassert_equal(st.muted, 1U, "");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "\a");
	zassert_equal(led_level(), 1, "a bell is not subject to the mute (reference)");
	meshtastic_extnotify_stop();

	zassert_ok(meshtastic_channels_set_slot(primary, &saved), "restore");
}

ZTEST(extnotify, test_active_low_inverts_the_drive)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 0U, true, false);

	c.active = false;
	zassert_ok(meshtastic_extnotify_set(&c), "");
	/* config_changed leaves the output "off", which for active-low means
	 * the LED's active level -- the reference's !on. */
	meshtastic_extnotify_stop();
	zassert_equal(led_level(), 1, "active-low idle: driven high");
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "hello");
	zassert_equal(led_level(), 0, "active-low alert: driven low");
	meshtastic_extnotify_stop();
	zassert_equal(led_level(), 1, "");
}

ZTEST(extnotify, test_own_text_or_disabled_never_alerts_and_stop_ends_a_cycle)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 60U, true, false);

	zassert_ok(meshtastic_extnotify_set(&c), "");
	inject_text(TEST_NODE_ID, MESHTASTIC_NODE_BROADCAST, "me");
	zassert_equal(led_level(), 0, "our own text: nothing");

	inject_text(PEER_A, TEST_NODE_ID, "dm");
	zassert_equal(led_level(), 1, "a DM to us alerts");
	meshtastic_extnotify_stop();
	zassert_equal(led_level(), 0, "stop: off");
	zassert_false(meshtastic_extnotify_nagging(), "");

	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "again");
	zassert_true(meshtastic_extnotify_nagging(), "a new alert re-arms");
	c.enabled = false;
	zassert_ok(meshtastic_extnotify_set(&c), "");
	zassert_false(meshtastic_extnotify_nagging(), "disabling stops the cycle");
	zassert_equal(led_level(), 0, "");
	inject_text(PEER_A, MESHTASTIC_NODE_BROADCAST, "silent");
	zassert_equal(led_level(), 0, "disabled: nothing");
	zassert_equal(meshtastic_extnotify_trigger(), -EPERM, "trigger while disabled: refused");
}

ZTEST(extnotify, test_outputs_this_port_cannot_drive_are_refused)
{
	meshtastic_ModuleConfig_ExternalNotificationConfig c = cfg_led(true, 1000U, 0U, true, false);

	zassert_ok(meshtastic_extnotify_validate(&c), "the LED-only config is fine");
	c.output = 35U;
	zassert_equal(meshtastic_extnotify_validate(&c), -ENOTSUP, "a raw pin number: refused");
	c.output = 0U;
	c.output_buzzer = 4U;
	zassert_equal(meshtastic_extnotify_validate(&c), -ENOTSUP, "buzzer pin: refused");
	c.output_buzzer = 0U;
	c.alert_message_vibra = true;
	zassert_equal(meshtastic_extnotify_validate(&c), -ENOTSUP, "vibra alert: refused");
	c.alert_message_vibra = false;
	c.use_pwm = true;
	zassert_equal(meshtastic_extnotify_validate(&c), -ENOTSUP, "PWM ringtone: refused");
	c.use_pwm = false;
	c.use_i2s_as_buzzer = true;
	zassert_equal(meshtastic_extnotify_validate(&c), -ENOTSUP, "I2S ringtone: refused");
	c.use_i2s_as_buzzer = false;
	zassert_equal(meshtastic_extnotify_set(&c), 0, "and back to fine");
}
