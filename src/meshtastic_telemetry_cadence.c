/* SPDX-License-Identifier: GPL-3.0
 *
 * Telemetry cadence: ModuleConfig.telemetry resolved into "send it, and how often".
 *
 * The reference keeps the rules in three places -- Default.h/.cpp (the
 * role-aware defaults and minimums, the congestion scaling), NodeDB::init
 * (the coercion to the minimum on a default channel) and the two telemetry
 * modules' runOnce (the enable flags). They live together here so that the
 * broadcast threads, the shell and the tests all read ONE resolution
 * (agents-dnr4.10).
 *
 * What is deliberately not here: the reference's per-region telemetryThrottle
 * (a region-profile multiplier this port's region table does not carry) and
 * the air-quality / power / health senders (no such sensor on any board this
 * port runs on; their flags are accepted and inert, exactly as the reference
 * behaves on a board without the sensor).
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_telemetry_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Reference Default.h: IF_ROUTER(ONE_DAY / 2, ...) for both the default and the
 * minimum; the non-router default is the Kconfig value (3600 s, the reference's
 * default_telemetry_broadcast_interval_secs), the non-router minimum 30 min. */
#define TELEMETRY_ONE_DAY_SEC      (24U * 60U * 60U)
#define TELEMETRY_ROUTER_SEC       (TELEMETRY_ONE_DAY_SEC / 2U)
#define TELEMETRY_CLIENT_MIN_SEC   (30U * 60U)
/* Reference MAX_INTERVAL is INT32_MAX milliseconds ("to avoid overflow issues
 * with Apple clients"); the same ceiling expressed in seconds. */
#define TELEMETRY_MAX_SEC          ((uint32_t)INT32_MAX / 1000U)
/* Reference congestionScalingCoefficient: no scaling up to this many online nodes. */
#define TELEMETRY_SCALE_FREE_NODES 40U

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS_INTERVAL_SEC)
#define TELEMETRY_DEVICE_DEFAULT_SEC CONFIG_MESHTASTIC_DEVICE_METRICS_INTERVAL_SEC
#else
#define TELEMETRY_DEVICE_DEFAULT_SEC 3600U
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS_INTERVAL_SEC)
#define TELEMETRY_ENV_DEFAULT_SEC CONFIG_MESHTASTIC_ENVIRONMENT_METRICS_INTERVAL_SEC
#else
#define TELEMETRY_ENV_DEFAULT_SEC 3600U
#endif

static bool role_is_router(meshtastic_Config_DeviceConfig_Role role)
{
	return role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
	       role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE;
}

/* Reference getConfiguredOrDefaultMsScaled: routers are "already significantly
 * higher", and sensors/trackers get priority for their telemetry. */
static bool role_is_unscaled(meshtastic_Config_DeviceConfig_Role role)
{
	return role_is_router(role) || role == meshtastic_Config_DeviceConfig_Role_SENSOR ||
	       role == meshtastic_Config_DeviceConfig_Role_TRACKER ||
	       role == meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;
}

/* Reference Channels::hasDefaultChannel: any enabled slot on the default key. */
static bool any_default_channel(void)
{
	for (uint8_t ch = 0U; ch < MESHTASTIC_MAX_CHANNELS; ch++) {
		const meshtastic_Channel *slot = meshtastic_channels_get(ch);

		if (slot != NULL && slot->role != meshtastic_Channel_Role_DISABLED &&
		    meshtastic_channels_is_default(ch)) {
			return true;
		}
	}
	return false;
}

uint32_t meshtastic_telemetry_scaled_interval_sec(uint32_t base_sec, uint32_t online_nodes)
{
	float bw_khz;
	float factor;
	float coef;
	double scaled;
	uint8_t sf;

	if (role_is_unscaled(meshtastic_device_role()) ||
	    online_nodes <= TELEMETRY_SCALE_FREE_NODES) {
		return MIN(base_sec, TELEMETRY_MAX_SEC);
	}

	/* The modem the radio is actually on (resolved from the preset, or the
	 * manual SF/BW); the reference clamps both before use. */
	sf = (uint8_t)CLAMP(mt.modem.spread_factor, 7U, 12U);
	bw_khz = (mt.modem.bandwidth_hz != 0U) ? (float)mt.modem.bandwidth_hz / 1000.0f : 250.0f;

	/* throttlingFactor = 2^SF / (BW_kHz * 100); each online node past 40 adds
	 * one factor to the multiplier. SF11/250 kHz (LongFast) -> 0.08192,
	 * SF7/500 kHz (ShortTurbo) -> 0.00256. */
	factor = (float)(1U << sf) / (bw_khz * 100.0f);
	coef = 1.0f + (float)(online_nodes - TELEMETRY_SCALE_FREE_NODES) * factor;
	scaled = (double)base_sec * (double)coef;
	if (scaled >= (double)TELEMETRY_MAX_SEC) {
		return TELEMETRY_MAX_SEC;
	}
	return (uint32_t)scaled;
}

static uint32_t resolve_interval(uint32_t configured, uint32_t client_default_sec,
				 bool default_channel, uint32_t online_nodes)
{
	meshtastic_Config_DeviceConfig_Role role = meshtastic_device_role();
	uint32_t min_sec = role_is_router(role) ? TELEMETRY_ROUTER_SEC : TELEMETRY_CLIENT_MIN_SEC;
	uint32_t v = configured;

	/* Reference NodeDB::init: on a default channel a CONFIGURED interval is
	 * coerced up to the role-aware minimum. Zero is left alone -- it
	 * coalesces to the default just below (getConfiguredOrMinimumValue). */
	if (v != 0U && default_channel && v < min_sec) {
		v = min_sec;
	}
	if (v == 0U) {
		v = role_is_router(role) ? TELEMETRY_ROUTER_SEC : client_default_sec;
	}
	return meshtastic_telemetry_scaled_interval_sec(v, online_nodes);
}

void meshtastic_telemetry_settings(struct meshtastic_telemetry_settings *out)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	const meshtastic_ModuleConfig_TelemetryConfig *cfg = &mod.payload_variant.telemetry;
	bool default_channel;
	uint32_t online;

	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_telemetry_tag, &mod);
	default_channel = any_default_channel();
	online = meshtastic_telemetry_online_nodes();

	out->device_enabled = cfg->device_telemetry_enabled;
	out->device_interval_sec = resolve_interval(cfg->device_update_interval,
						    TELEMETRY_DEVICE_DEFAULT_SEC, default_channel,
						    online);
	out->environment_enabled = cfg->environment_measurement_enabled;
	out->environment_interval_sec = resolve_interval(cfg->environment_update_interval,
							 TELEMETRY_ENV_DEFAULT_SEC,
							 default_channel, online);
	out->online_nodes = online;
}

/* ---- the "changed" fan-out ---------------------------------------------------- */

/* The broadcast threads sleep until their next deadline; a config change must
 * cut that short so a new interval or a disable takes effect now, not an hour
 * from now. Each thread registers itself; k_wakeup() makes its k_sleep() return
 * early and its loop re-resolves. */
#define TELEMETRY_WATCHERS 2U

static struct k_thread *watchers[TELEMETRY_WATCHERS];
static atomic_t generation;

void meshtastic_telemetry_cadence_watch(struct k_thread *thread)
{
	for (size_t i = 0; i < TELEMETRY_WATCHERS; i++) {
		if (watchers[i] == NULL || watchers[i] == thread) {
			watchers[i] = thread;
			return;
		}
	}
	LOG_WRN("Telemetry: no watcher slot for a broadcast thread");
}

uint32_t meshtastic_telemetry_cadence_generation(void)
{
	return (uint32_t)atomic_get(&generation);
}

void meshtastic_telemetry_config_changed(void)
{
	struct meshtastic_telemetry_settings s;

	atomic_inc(&generation);
	meshtastic_telemetry_settings(&s);
	LOG_INF("Telemetry: device %s every %u s, environment %s every %u s",
		s.device_enabled ? "on" : "off", s.device_interval_sec,
		s.environment_enabled ? "on" : "off", s.environment_interval_sec);

	for (size_t i = 0; i < TELEMETRY_WATCHERS; i++) {
		if (watchers[i] != NULL) {
			k_wakeup(watchers[i]);
		}
	}
}

#if !defined(CONFIG_MESHTASTIC_LOCAL_STATS)
/* Without LocalStats there is no online-node count to scale by; the reference's
 * coefficient then stays at 1. */
uint32_t meshtastic_telemetry_online_nodes(void)
{
	return 0U;
}
#endif
