/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_TELEMETRY_INTERNAL_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_TELEMETRY_INTERNAL_H_

#include "meshtastic_core.h"

#include "meshtastic/telemetry.pb.h"

#if defined(CONFIG_MESHTASTIC_NODEDB)
#include <zephyr/meshtastic/nodedb.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decode a port-67 Telemetry want_response probe.
 *
 * Takes the decoded payload bytes directly so callers can feed either the
 * decoded MeshPacket (@c mesh->decoded.payload) or the flat struct's payload.
 * An empty payload decodes as @c Telemetry_init_zero (variant 0).
 */
bool meshtastic_telemetry_decode_request(const uint8_t *payload, size_t payload_len,
					 meshtastic_Telemetry *request);

/**
 * @brief Encode a Telemetry protobuf into a port-67 reply packet.
 */
int meshtastic_telemetry_encode_packet(uint32_t dest, uint32_t response_to_id,
				       const meshtastic_Telemetry *telemetry, uint8_t *payload,
				       struct meshtastic_packet *packet);

/**
 * @brief Populate DeviceMetrics (uptime, battery, channel/air utilization).
 *
 * With no valid battery source, @c battery_level is set to the "powered" sentinel
 * (>100) rather than left unset, matching upstream's MAGIC_USB_BATTERY_LEVEL.
 */
int meshtastic_collect_device_metrics(meshtastic_DeviceMetrics *metrics);

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
/**
 * @brief Populate LocalStats from counters the stack already keeps.
 *
 * Nothing here is measured specially for the wire: uptime, the airtime ring,
 * @ref meshtastic_status and the scheduler stats are all read as they stand.
 * Fields this port has no source for are left zero — currently only
 * @c noise_floor, and @c heap_* without @kconfig{CONFIG_SYS_HEAP_RUNTIME_STATS}.
 *
 * @retval 0       Populated.
 * @retval -EINVAL @p stats is NULL.
 */
int meshtastic_collect_local_stats(meshtastic_LocalStats *stats);

#if defined(CONFIG_MESHTASTIC_NODEDB)
/**
 * @brief Age of a NodeDB entry in seconds, or @c UINT32_MAX when unknowable.
 *
 * Exposed rather than kept file-static because the rule it encodes is the one
 * thing about num_online_nodes worth getting wrong: an entry that was never
 * heard THIS boot and carries no persisted wall-clock epoch has an UNKNOWN age,
 * and unknown must never be rounded down to "recent". Every node on this fleet
 * has that shape after a reboot (no wall clock anywhere), so the rule needs a
 * test that does not depend on bringing the whole NodeDB up.
 */
uint32_t meshtastic_local_stats_node_age_sec(const struct meshtastic_nodedb_node *node);
#endif
#endif

/* ---- cadence: ModuleConfig.telemetry resolved (meshtastic_telemetry_cadence.c) ---- */

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS) || defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
/**
 * @brief The stored telemetry section resolved into what the broadcasters do.
 *
 * Intervals are the reference's rules applied in order: a configured value
 * below the role-aware minimum is coerced up when a default channel is
 * present, zero coalesces to the role-aware default (the Kconfig interval, or
 * 12 h for a router), the result is scaled by the online-node count past 40
 * (routers, sensors and trackers exempt) and capped at INT32_MAX ms.
 */
struct meshtastic_telemetry_settings {
	bool device_enabled;              /**< TelemetryConfig.device_telemetry_enabled */
	uint32_t device_interval_sec;     /**< resolved device_update_interval */
	bool environment_enabled;         /**< TelemetryConfig.environment_measurement_enabled */
	uint32_t environment_interval_sec; /**< resolved environment_update_interval */
	uint32_t online_nodes;            /**< the count the scaling used */
};

void meshtastic_telemetry_settings(struct meshtastic_telemetry_settings *out);

/**
 * @brief Reference Default::getConfiguredOrDefaultMsScaled's scaling, in seconds.
 *
 * Exposed for the tests: the coefficient depends on the modem the radio is on
 * (2^SF / (BW_kHz * 100) per online node past 40) and on the role.
 */
uint32_t meshtastic_telemetry_scaled_interval_sec(uint32_t base_sec, uint32_t online_nodes);

/** @brief Online nodes as LocalStats counts them (0 without LocalStats). */
uint32_t meshtastic_telemetry_online_nodes(void);

/** @brief Wake a registered broadcast thread whenever the section changes. */
void meshtastic_telemetry_cadence_watch(struct k_thread *thread);

/** @brief Bumped on every change; a thread compares it to know a sleep was cut short. */
uint32_t meshtastic_telemetry_cadence_generation(void);

/**
 * @brief The admin path's hook: ModuleConfig.telemetry was written.
 *
 * Re-resolves, logs the new cadence and wakes the broadcast threads so the
 * change applies without a reboot (the reference reboots for this section;
 * here the threads read the store at every deadline, so there is nothing to
 * restart).
 */
void meshtastic_telemetry_config_changed(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_TELEMETRY_INTERNAL_H_ */
