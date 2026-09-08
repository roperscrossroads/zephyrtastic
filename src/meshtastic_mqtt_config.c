/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_mqtt_config.h for the contract and the reference mapping. */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/sys/util.h>

#include "meshtastic_mqtt_config.h"

static void copy_bounded(char *dst, size_t cap, const char *src)
{
	if (cap == 0U) {
		return;
	}
	if (src == NULL) {
		dst[0] = '\0';
		return;
	}
	strncpy(dst, src, cap - 1U);
	dst[cap - 1U] = '\0';
}

void meshtastic_mqtt_split_host_port(const char *address, char *host, size_t host_cap,
				     uint16_t *port)
{
	const char *delim;
	size_t host_len;

	if (host_cap == 0U) {
		return;
	}
	if (address == NULL) {
		host[0] = '\0';
		return;
	}

	delim = strchr(address, ':');
	/* Reference: indexOf(':') > 0 — a leading colon is not a port separator. */
	if (delim == NULL || delim == address) {
		copy_bounded(host, host_cap, address);
		return;
	}

	host_len = MIN((size_t)(delim - address), host_cap - 1U);
	memcpy(host, address, host_len);
	host[host_len] = '\0';

	if (port != NULL) {
		char *end = NULL;
		unsigned long parsed = strtoul(delim + 1, &end, 10);

		/* Only a clean, in-range number replaces the default; "host:" and
		 * "host:99999" keep it, as the reference's toInt() path does. */
		if (end != delim + 1 && *end == '\0' && parsed >= 1UL && parsed <= 65535UL) {
			*port = (uint16_t)parsed;
		}
	}
}

void meshtastic_mqtt_settings_resolve(const meshtastic_ModuleConfig_MQTTConfig *cfg,
				      struct meshtastic_mqtt_settings *out)
{
	static const meshtastic_ModuleConfig_MQTTConfig zero =
		meshtastic_ModuleConfig_MQTTConfig_init_zero;

	if (out == NULL) {
		return;
	}
	if (cfg == NULL) {
		cfg = &zero;
	}

	memset(out, 0, sizeof(*out));

	out->enabled = cfg->enabled;
	out->proxy_to_client = cfg->proxy_to_client_enabled;
	out->tls_enabled = cfg->tls_enabled;
	out->encryption_enabled = cfg->encryption_enabled;
	out->map_reporting_enabled = cfg->map_reporting_enabled;

	if (cfg->address[0] != '\0') {
		out->address_is_custom = true;
		out->port = cfg->tls_enabled ? MESHTASTIC_MQTT_PORT_TLS : MESHTASTIC_MQTT_PORT_PLAIN;
		meshtastic_mqtt_split_host_port(cfg->address, out->host, sizeof(out->host),
						&out->port);
		/* As given, even when empty: a private broker may be anonymous, and the
		 * reference honours an empty username/password verbatim for a custom
		 * server. */
		copy_bounded(out->username, sizeof(out->username), cfg->username);
		copy_bounded(out->password, sizeof(out->password), cfg->password);
		copy_bounded(out->tls_hostname, sizeof(out->tls_hostname), out->host);
	} else {
		copy_bounded(out->host, sizeof(out->host), MESHTASTIC_MQTT_FALLBACK_HOST);
		out->port = (uint16_t)MESHTASTIC_MQTT_FALLBACK_PORT;
		copy_bounded(out->username, sizeof(out->username), MESHTASTIC_MQTT_FALLBACK_USERNAME);
		copy_bounded(out->password, sizeof(out->password), MESHTASTIC_MQTT_FALLBACK_PASSWORD);
#if defined(CONFIG_MESHTASTIC_MQTT_TLS_HOSTNAME)
		copy_bounded(out->tls_hostname, sizeof(out->tls_hostname),
			     CONFIG_MESHTASTIC_MQTT_TLS_HOSTNAME);
#else
		copy_bounded(out->tls_hostname, sizeof(out->tls_hostname), out->host);
#endif
	}

	out->default_broker = (strcmp(out->host, MESHTASTIC_MQTT_DEFAULT_BROKER) == 0);

	copy_bounded(out->root, sizeof(out->root),
		     (cfg->root[0] != '\0') ? cfg->root : MESHTASTIC_MQTT_FALLBACK_ROOT);

	/* Map report: the reference treats 0 as "use the default" and clamps a
	 * precision outside 12..15 back to it. The interval floor matches the Kconfig
	 * range so a runtime value cannot go below what the build allows. */
	if (cfg->has_map_report_settings && cfg->map_report_settings.publish_interval_secs != 0U) {
		out->map_publish_interval_secs = cfg->map_report_settings.publish_interval_secs;
	} else {
		out->map_publish_interval_secs = MESHTASTIC_MQTT_FALLBACK_MAP_INTERVAL_SEC;
	}
	out->map_publish_interval_secs =
		MAX(out->map_publish_interval_secs, MESHTASTIC_MQTT_MAP_INTERVAL_MIN_SEC);

	if (cfg->has_map_report_settings && cfg->map_report_settings.position_precision >= 12U &&
	    cfg->map_report_settings.position_precision <= 15U) {
		out->map_position_precision = cfg->map_report_settings.position_precision;
	} else {
		out->map_position_precision = MESHTASTIC_MQTT_FALLBACK_MAP_PRECISION;
	}

	/* The store seeds should_report_location=true only when the build opted into
	 * map reporting (CONFIG_MESHTASTIC_MQTT_MAP_REPORT); a section that never
	 * carried the sub-message inherits that same build-level opt-in. */
	out->map_should_report_location =
		cfg->has_map_report_settings ? cfg->map_report_settings.should_report_location
					     : IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT);
}

int meshtastic_mqtt_config_validate(const meshtastic_ModuleConfig_MQTTConfig *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->tls_enabled && !IS_ENABLED(CONFIG_MESHTASTIC_MQTT_TLS)) {
		return -ENOTSUP;
	}

	return 0;
}
