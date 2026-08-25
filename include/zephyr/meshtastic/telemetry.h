/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/**
 * @file
 * @brief Meshtastic telemetry public API.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_TELEMETRY_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_TELEMETRY_H_

#include <zephyr/meshtastic/meshtastic.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Collect and send device metrics.
 *
 * Always includes uptime. Battery level and voltage are included when
 * @kconfig{CONFIG_MESHTASTIC_FUEL_GAUGE} is enabled and a ready
 * @c fuel_gauge0 devicetree alias provides the corresponding properties.
 *
 * @param dest Destination node ID, or @ref MESHTASTIC_NODE_BROADCAST.
 * @param wait @c K_FOREVER to block until transmission completes;
 *             @c K_NO_WAIT to queue only (-ENOMSG if the TX queue is full).
 *
 * @retval 0        Success.
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -EIO     Crypto or radio transmission failed.
 * @retval -ENOMSG  TX queue full (@p wait is @c K_NO_WAIT).
 * @retval -ENOTSUP Device metrics support is not compiled in.
 */
int meshtastic_send_device_metrics(uint32_t dest, k_timeout_t wait);

/**
 * @brief Collect environment sensors and send EnvironmentMetrics telemetry.
 *
 * Requires @kconfig{CONFIG_MESHTASTIC_ENVIRONMENT_METRICS}. Temperature,
 * humidity, barometric pressure, gas resistance and illuminance are included
 * for each quantity whose devicetree alias (@c ambient_temp0, @c humidity0,
 * @c pressure_sensor, @c gas_sensor, @c light_sensor) exists and is ready.
 *
 * @param dest Destination node ID, or @ref MESHTASTIC_NODE_BROADCAST.
 * @param wait @c K_FOREVER to block until transmission completes;
 *             @c K_NO_WAIT to queue only (-ENOMSG if the TX queue is full).
 *
 * @retval 0        Success.
 * @retval -ENODEV  No sensor produced a reading.
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -EIO     Crypto or radio transmission failed.
 * @retval -ENOMSG  TX queue full (@p wait is @c K_NO_WAIT).
 * @retval -ENOTSUP Environment metrics support is not compiled in.
 */
int meshtastic_send_environment(uint32_t dest, k_timeout_t wait);

/**
 * @brief Hand LocalStats to every attached phone transport. Costs no airtime.
 *
 * LocalStats is a node's description of its own mesh behaviour — uptime,
 * packet/relay/dupe counters, node counts, heap. It goes to the phone and only
 * to the phone, matching upstream's DeviceTelemetryModule, which calls
 * sendToPhone() and never sendToMesh() for this variant. The mesh sees
 * LocalStats only as a reply to an explicit Telemetry(local_stats) request.
 *
 * Requires @kconfig{CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE}. Called on a timer
 * by the telemetry thread; exposed so the shell (and tests) can force one.
 *
 * @retval 0        Enqueued to every registered transport.
 * @retval -EINVAL  Stack not initialized.
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -ENOTSUP Not compiled in.
 */
int meshtastic_send_local_stats_to_phone(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_TELEMETRY_H_ */
