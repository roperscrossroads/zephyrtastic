/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_extnotify.h. Reference: firmware/src/modules/ExternalNotificationModule.cpp
 * (handleReceived, armNagCycle, runOnce, setExternalState, stopNow). */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_extnotify.h"
#include "meshtastic_modules.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define EXTNOTIFY_DEFAULT_OUTPUT_MS 1000U /* reference EXT_NOTIFICATION_MODULE_OUTPUT_MS */
#define ASCII_BELL                  0x07U

#define EXTNOTIFY_HAS_LED DT_NODE_EXISTS(DT_ALIAS(led0))

#if EXTNOTIFY_HAS_LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif

static struct k_work_delayable tick_work;
static bool nagging;
static bool output_on;
static int64_t cutoff_ms;
static struct meshtastic_extnotify_stats stats;

/* ---- settings ------------------------------------------------------------- */

void meshtastic_extnotify_settings(struct meshtastic_extnotify_settings *out)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	const meshtastic_ModuleConfig_ExternalNotificationConfig *cfg =
		&mod.payload_variant.external_notification;

	if (out == NULL) {
		return;
	}
	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_external_notification_tag,
						 &mod);
	out->enabled = cfg->enabled;
	out->active = cfg->active;
	out->alert_message = cfg->alert_message;
	out->alert_bell = cfg->alert_bell;
	out->output_ms = cfg->output_ms ? cfg->output_ms : EXTNOTIFY_DEFAULT_OUTPUT_MS;
	out->nag_timeout = cfg->nag_timeout;
	out->output_present = EXTNOTIFY_HAS_LED;
}

int meshtastic_extnotify_validate(const meshtastic_ModuleConfig_ExternalNotificationConfig *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	/* A raw pin number is meaningless here: 0 ("the board's notification
	 * output") is the only value the port can map, to led0. */
	if (cfg->output != 0U || cfg->output_vibra != 0U || cfg->output_buzzer != 0U) {
		return -ENOTSUP;
	}
	if (cfg->alert_message_vibra || cfg->alert_bell_vibra || cfg->alert_message_buzzer ||
	    cfg->alert_bell_buzzer || cfg->use_pwm || cfg->use_i2s_as_buzzer) {
		return -ENOTSUP;
	}
	if (cfg->enabled && !EXTNOTIFY_HAS_LED) {
		return -ENOTSUP;
	}
	return 0;
}

int meshtastic_extnotify_set(const meshtastic_ModuleConfig_ExternalNotificationConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	int ret = meshtastic_extnotify_validate(cfg);

	if (ret < 0) {
		return ret;
	}
	mod.which_payload_variant = meshtastic_ModuleConfig_external_notification_tag;
	mod.payload_variant.external_notification = *cfg;
	ret = meshtastic_config_store_set_module(&mod);
	if (ret < 0) {
		return ret;
	}
	meshtastic_extnotify_config_changed();
	return 0;
}

/* ---- the output ------------------------------------------------------------- */

static void output_set(bool on)
{
	struct meshtastic_extnotify_settings s;

	meshtastic_extnotify_settings(&s);
	output_on = on;
#if EXTNOTIFY_HAS_LED
	/* Reference: digitalWrite(output, active ? on : !on), layered on the
	 * devicetree's own active level. */
	(void)gpio_pin_set_dt(&led, (s.active ? on : !on) ? 1 : 0);
#endif
}

void meshtastic_extnotify_stop(void)
{
	(void)k_work_cancel_delayable(&tick_work);
	nagging = false;
	cutoff_ms = 0;
	output_set(false);
}

bool meshtastic_extnotify_nagging(void)
{
	return nagging;
}

bool meshtastic_extnotify_output_on(void)
{
	return output_on;
}

/* Reference runOnce while nagging: toggle every output_ms; when the window
 * has passed, stop. */
static void tick_work_fn(struct k_work *work)
{
	struct meshtastic_extnotify_settings s;

	ARG_UNUSED(work);

	if (!nagging || k_uptime_get() >= cutoff_ms) {
		meshtastic_extnotify_stop();
		return;
	}
	meshtastic_extnotify_settings(&s);
	output_set(!output_on);
	(void)k_work_reschedule(&tick_work, K_MSEC(s.output_ms));
}

/* Reference armNagCycle + setExternalState(0, true). */
static void arm(const struct meshtastic_extnotify_settings *s)
{
	uint32_t duration_ms = s->nag_timeout ? s->nag_timeout * MSEC_PER_SEC : s->output_ms;

	cutoff_ms = k_uptime_get() + duration_ms;
	nagging = true;
	stats.alerts++;
	output_set(true);
	(void)k_work_reschedule(&tick_work, K_MSEC(s->output_ms));
}

int meshtastic_extnotify_trigger(void)
{
	struct meshtastic_extnotify_settings s;

	meshtastic_extnotify_settings(&s);
	if (!s.enabled) {
		return -EPERM;
	}
	if (!s.output_present) {
		return -ENODEV;
	}
	arm(&s);
	return 0;
}

void meshtastic_extnotify_config_changed(void)
{
	struct meshtastic_extnotify_settings s;

	meshtastic_extnotify_settings(&s);
	if (!s.enabled) {
		meshtastic_extnotify_stop();
	}
	LOG_INF("ExtNotify: %s, message %s, bell %s, %u ms, nag %u s, led %s",
		s.enabled ? "enabled" : "disabled", s.alert_message ? "on" : "off",
		s.alert_bell ? "on" : "off", s.output_ms, s.nag_timeout,
		s.output_present ? "present" : "ABSENT");
}

void meshtastic_extnotify_stats(struct meshtastic_extnotify_stats *out)
{
	if (out != NULL) {
		*out = stats;
	}
}

void meshtastic_extnotify_reset(void)
{
	meshtastic_extnotify_stop();
	memset(&stats, 0, sizeof(stats));
}

/* ---- receiving -------------------------------------------------------------- */

static bool is_text_port(uint32_t portnum)
{
	/* Reference MeshService::isTextPayload (RANGE_TEST left out: no module). */
	return portnum == MESHTASTIC_PORT_TEXT_MESSAGE || portnum == MESHTASTIC_PORT_ALERT ||
	       portnum == MESHTASTIC_PORT_DETECTION_SENSOR;
}

static bool channel_muted(uint8_t index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(index);

	return ch != NULL && ch->has_settings && ch->settings.has_module_settings &&
	       ch->settings.module_settings.is_muted;
}

static void extnotify_on_packet(const struct meshtastic_packet *packet,
				const meshtastic_MeshPacket *mesh)
{
	struct meshtastic_extnotify_settings s;
	uint32_t from;
	uint32_t to;
	uint32_t portnum;
	const uint8_t *payload;
	size_t payload_len;
	bool bell = false;
	bool muted;
	bool alert;

	if (packet == NULL) {
		return;
	}

	from = mesh ? mesh->from : packet->from;
	to = mesh ? mesh->to : packet->to;
	portnum = mesh ? (uint32_t)mesh->decoded.portnum : packet->portnum;
	if (from == 0U || from == meshtastic_get_node_id() || !is_text_port(portnum)) {
		return;
	}

	meshtastic_extnotify_settings(&s);
	if (!s.enabled) {
		return;
	}

	payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	for (size_t i = 0; payload != NULL && i < payload_len; i++) {
		if (payload[i] == ASCII_BELL) {
			bell = true;
			break;
		}
	}

	/* Reference: a broadcast honours the channel's mute; a DM to us the
	 * sender's -- which this NodeDB does not carry, so never muted. */
	muted = (to == MESHTASTIC_NODE_BROADCAST) &&
		channel_muted(packet->channel_index == MESHTASTIC_CHANNEL_INDEX_INVALID
				      ? meshtastic_channels_primary_index()
				      : packet->channel_index);
	if (muted) {
		stats.muted++;
	}

	alert = (s.alert_bell && bell) || (s.alert_message && !muted);
	if (!alert) {
		return;
	}
	if (bell) {
		stats.bells++;
	}
	LOG_INF("ExtNotify: alert from 0x%08x (%s)", from, bell ? "bell" : "message");
	arm(&s);
}

MESHTASTIC_MODULE_DEFINE(extnotify, MESHTASTIC_PORT_TEXT_MESSAGE, MESHTASTIC_MODULE_ALL_PACKETS,
			 extnotify_on_packet, NULL);

int meshtastic_extnotify_init(void)
{
	k_work_init_delayable(&tick_work, tick_work_fn);
#if EXTNOTIFY_HAS_LED
	if (!gpio_is_ready_dt(&led)) {
		LOG_WRN("ExtNotify: led0 not ready");
	} else {
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}
#endif
	output_set(false);
	meshtastic_extnotify_config_changed();
	return 0;
}
