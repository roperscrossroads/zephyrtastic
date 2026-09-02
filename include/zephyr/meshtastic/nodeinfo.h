/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/**
 * @file
 * @brief Meshtastic NodeInfo public API.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_NODEINFO_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_NODEINFO_H_

#include <zephyr/meshtastic/meshtastic.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Announce this node to the mesh.
 *
 * Encodes the node's @c User identity (id, long/short name, MAC address
 * derived from the node ID, hardware model and role) as a @c NODEINFO_APP
 * (port 4) Meshtastic packet and transmits it. Other nodes use this to
 * populate their node database; without it peers only see a bare node
 * number with no name, MAC address or hardware model.
 *
 * When @kconfig{CONFIG_MESHTASTIC_NODEINFO_AUTO_SEND} is enabled the
 * subsystem already broadcasts this periodically; calling this directly is
 * only needed for an explicit, immediate announcement.
 *
 * @param dest Destination node ID, or @ref MESHTASTIC_NODE_BROADCAST.
 *
 * @retval 0        Success.
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -EIO     Crypto or radio transmission failed.
 * @retval -ENOTSUP NodeInfo support is not compiled in.
 */
int meshtastic_send_node_info(uint32_t dest);

/**
 * @brief The interval between automatic NodeInfo broadcasts, in effect right now.
 *
 * Resolves @c DeviceConfig.node_info_broadcast_secs the way the rest of this
 * port resolves an unset numeric config value: 0 (never explicitly
 * configured, or config unavailable) falls back to
 * @kconfig{CONFIG_MESHTASTIC_NODEINFO_INTERVAL_SEC}. A nonzero stored value
 * is used as-is, honouring an operator's explicit request rather than
 * clamping it to the Kconfig prompt's suggested range.
 *
 * Read fresh from the config store on every call rather than cached: the
 * auto-send thread only calls this once per broadcast (hours apart), so
 * there is no hot-path cost to paying for a config read there instead of
 * keeping a cache in sync.
 *
 * @return The effective interval in seconds, never 0.
 */
uint32_t meshtastic_nodeinfo_interval_secs(void);

/**
 * @brief Ask one peer for its NodeInfo (a directed, want_response=true send).
 *
 * Throttled through the same per-peer request cooldown
 * (@kconfig{CONFIG_MESHTASTIC_NODEINFO_UNKNOWN_SUPPRESS_SEC}) the
 * unknown-PKC-sender path uses internally, so calling this repeatedly for
 * the same peer in quick succession is safe -- a suppressed repeat returns
 * -EAGAIN rather than sending.
 *
 * @param peer Destination node ID. Must not be 0, broadcast, or our own id.
 *
 * @retval 0        Sent.
 * @retval -EAGAIN  Suppressed by the per-peer cooldown (already asked recently).
 * @retval -EINVAL  Invalid peer (0, broadcast, or self).
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -EIO     Crypto or radio transmission failed.
 */
int meshtastic_nodeinfo_request(uint32_t peer);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_NODEINFO_H_ */
