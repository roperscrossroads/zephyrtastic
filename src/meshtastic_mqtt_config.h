/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_MQTT_CONFIG_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_MQTT_CONFIG_H_

/*
 * ModuleConfig.mqtt -> effective MQTT settings (agents-dnr4.8).
 *
 * The MQTT module used to read every knob straight from CONFIG_MESHTASTIC_MQTT_*
 * and never looked at the ModuleConfig.mqtt section the admin channel persists.
 * This resolver is the one place that turns the stored section into what the
 * module actually connects with, mirroring the reference's PubSubConfig
 * (mqtt/MQTT.cpp):
 *
 *   - address empty  -> the build's Kconfig broker/port/username/password. On a
 *                       fresh node the config store is SEEDED from those same
 *                       Kconfig values (meshtastic_config_store.c
 *                       seed_module_defaults), so "unset" and "first boot" agree.
 *   - address set    -> "<host>[:<port>]"; username/password taken from the
 *                       section AS GIVEN, even when empty (a custom broker may
 *                       be anonymous). Port defaults to 1883, or 8883 when
 *                       tls_enabled, unless the address carries one.
 *   - root empty     -> the Kconfig root topic.
 *   - map settings   -> 0 / out-of-range fall back to the Kconfig defaults, as
 *                       the reference does with Default::getConfiguredOrDefault.
 *
 * Pure: no sockets, no store access, no logging — compiled into every build
 * (not just CONFIG_MESHTASTIC_MQTT) so the admin path can validate a section
 * and the sim suites can test the mapping without a network stack.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic/module_config.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The reference's secretReserved (AdminModule.cpp): what a remote get_module_config
 * returns in place of the password, and what a set carrying it means — "keep the
 * password you already have". */
#define MESHTASTIC_MQTT_SECRET_RESERVED "sekrit"

#define MESHTASTIC_MQTT_DEFAULT_BROKER "mqtt.meshtastic.org"
#define MESHTASTIC_MQTT_PORT_PLAIN     1883U
#define MESHTASTIC_MQTT_PORT_TLS       8883U

/* The Kconfig fallbacks, visible so a test can assert against the same values the
 * resolver uses regardless of whether the build has MQTT at all. */
#if defined(CONFIG_MESHTASTIC_MQTT_BROKER_HOST)
#define MESHTASTIC_MQTT_FALLBACK_HOST CONFIG_MESHTASTIC_MQTT_BROKER_HOST
#else
#define MESHTASTIC_MQTT_FALLBACK_HOST MESHTASTIC_MQTT_DEFAULT_BROKER
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_BROKER_PORT)
#define MESHTASTIC_MQTT_FALLBACK_PORT CONFIG_MESHTASTIC_MQTT_BROKER_PORT
#else
#define MESHTASTIC_MQTT_FALLBACK_PORT MESHTASTIC_MQTT_PORT_PLAIN
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_USERNAME)
#define MESHTASTIC_MQTT_FALLBACK_USERNAME CONFIG_MESHTASTIC_MQTT_USERNAME
#else
#define MESHTASTIC_MQTT_FALLBACK_USERNAME "meshdev"
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_PASSWORD)
#define MESHTASTIC_MQTT_FALLBACK_PASSWORD CONFIG_MESHTASTIC_MQTT_PASSWORD
#else
#define MESHTASTIC_MQTT_FALLBACK_PASSWORD "large4cats"
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_ROOT)
#define MESHTASTIC_MQTT_FALLBACK_ROOT CONFIG_MESHTASTIC_MQTT_ROOT
#else
#define MESHTASTIC_MQTT_FALLBACK_ROOT "msh"
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_MAP_REPORT_INTERVAL_SEC)
#define MESHTASTIC_MQTT_FALLBACK_MAP_INTERVAL_SEC CONFIG_MESHTASTIC_MQTT_MAP_REPORT_INTERVAL_SEC
#else
#define MESHTASTIC_MQTT_FALLBACK_MAP_INTERVAL_SEC 900U
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_MAP_REPORT_POSITION_PRECISION)
#define MESHTASTIC_MQTT_FALLBACK_MAP_PRECISION CONFIG_MESHTASTIC_MQTT_MAP_REPORT_POSITION_PRECISION
#else
#define MESHTASTIC_MQTT_FALLBACK_MAP_PRECISION 14U
#endif
/* Reference: 60 s floor on the map publish interval (Kconfig range starts there too). */
#define MESHTASTIC_MQTT_MAP_INTERVAL_MIN_SEC 60U

struct meshtastic_mqtt_settings {
	bool enabled;
	bool proxy_to_client;
	bool tls_enabled;
	bool encryption_enabled;
	bool map_reporting_enabled;
	bool map_should_report_location;
	/* MQTTConfig.address was non-empty: host/port/creds came from the section,
	 * not the Kconfig fallbacks. */
	bool address_is_custom;
	/* The resolved host is the public Meshtastic broker — gates the reference's
	 * "public broker" behaviours (portnum skip list, OK_TO_MQTT consent). */
	bool default_broker;
	uint16_t port;
	uint32_t map_publish_interval_secs;
	uint32_t map_position_precision;
	char host[sizeof(((meshtastic_ModuleConfig_MQTTConfig *)0)->address)];
	char username[sizeof(((meshtastic_ModuleConfig_MQTTConfig *)0)->username)];
	char password[sizeof(((meshtastic_ModuleConfig_MQTTConfig *)0)->password)];
	char root[sizeof(((meshtastic_ModuleConfig_MQTTConfig *)0)->root)];
	/* SNI + certificate CN/SAN name. A custom address names itself; the Kconfig
	 * fallback keeps CONFIG_MESHTASTIC_MQTT_TLS_HOSTNAME so a raw-IP BROKER_HOST
	 * (DNS bypass) still verifies against the broker's DNS name. */
	char tls_hostname[sizeof(((meshtastic_ModuleConfig_MQTTConfig *)0)->address)];
};

/**
 * @brief Resolve a stored ModuleConfig.mqtt section into effective settings.
 *
 * Never fails: every field has a fallback. @p cfg may be NULL (treated as an
 * all-zero section, i.e. every fallback and enabled=false).
 */
void meshtastic_mqtt_settings_resolve(const meshtastic_ModuleConfig_MQTTConfig *cfg,
				      struct meshtastic_mqtt_settings *out);

/**
 * @brief Can this build honour the section? (reference: MQTT::isValidConfig)
 *
 * @return 0 when acceptable; -ENOTSUP when tls_enabled is set but the image has no
 *         TLS transport compiled in (CONFIG_MESHTASTIC_MQTT_TLS) — a config that
 *         asks for TLS must never be silently downgraded to plaintext.
 *         -EINVAL on a NULL section.
 */
int meshtastic_mqtt_config_validate(const meshtastic_ModuleConfig_MQTTConfig *cfg);

/**
 * @brief Split "<host>[:<port>]" (reference: parseHostAndPort).
 *
 * Copies the host part into @p host (NUL-terminated, truncated to @p host_cap).
 * The port is only replaced when the suffix parses to 1..65535; otherwise
 * @p *port keeps its incoming default. An IPv6 literal is not handled (the
 * transport is IPv4-only), matching the reference's first-colon split.
 */
void meshtastic_mqtt_split_host_port(const char *address, char *host, size_t host_cap,
				     uint16_t *port);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_MQTT_CONFIG_H_ */
