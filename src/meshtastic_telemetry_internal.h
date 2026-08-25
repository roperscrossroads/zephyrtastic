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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_TELEMETRY_INTERNAL_H_ */
