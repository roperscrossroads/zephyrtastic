/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_TRAFFIC_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_TRAFFIC_H_

/*
 * Traffic management (agents-dnr4.20) -- the reference TrafficManagementModule's
 * packet inspection, on the statusmessage template.
 *
 * Unlike the other modules this is not a port handler: it is a GATE the router
 * consults for every received frame, decoded or not, before the frame is relayed
 * or delivered (the reference runs it first in callModules() and a STOP verdict
 * "fully consumes the packet -- no rebroadcast"). Three of the reference's
 * features, each mirrored rule for rule:
 *
 *   position dedup      -- a peer's position broadcast on a well-known channel
 *                          whose truncated coordinates fingerprint the same grid
 *                          cell as its last one, inside position_min_interval_secs,
 *                          is dropped. Trackers may refresh hourly and
 *                          lost-and-found every 15 min regardless (role caps).
 *                          Only a frame let through re-stamps the window, so a
 *                          fast repeater cannot mute itself.
 *   rate limit          -- more than rate_limit_max_packets (capped at 60) from
 *                          one node inside rate_limit_window_secs are dropped;
 *                          ROUTING and ADMIN are exempt.
 *   unknown filter      -- frames this node cannot decode are counted per
 *                          source in a fixed 5-minute window; past
 *                          unknown_packet_threshold (capped at 60) the source is
 *                          no longer relayed.
 *
 * Every feature is "non-zero implies enabled" (the proto's convention), and our
 * own frames and frames addressed to us are never shaped.
 *
 * NOT done, and why:
 *   - NodeInfo direct response (nodeinfo_direct_response_max_hops): the
 *     reference answers a NodeInfo request on another node's behalf from a
 *     cache, spoofing `from`, and by default only for a node whose key is
 *     PROVEN (XEdDSA-signed or manually verified). This port has neither
 *     signing nor manual verification yet, so the honest answer is to REFUSE a
 *     config that enables it (meshtastic_traffic_validate -> -ENOTSUP -> NAK)
 *     rather than store it and do nothing.
 *   - The relayed-position precision clamp (reference alterReceived): the port
 *     relays wire bytes unchanged. Our OWN positions are clamped on the way out
 *     (meshtastic_position_sanitise_tx).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/module_config.pb.h"
#include "meshtastic/telemetry.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_packet;

enum meshtastic_traffic_verdict {
	MESHTASTIC_TRAFFIC_PASS = 0,
	/* Consumed: neither delivered nor relayed (reference ProcessMessage::STOP). */
	MESHTASTIC_TRAFFIC_DROP,
};

/** Effective settings after the reference's caps. */
struct meshtastic_traffic_settings {
	uint32_t position_min_interval_secs; /* 0: dedup off */
	uint32_t rate_limit_window_secs;     /* 0: rate limit off */
	uint32_t rate_limit_max_packets;     /* 0: off; capped at 60 */
	uint32_t unknown_packet_threshold;   /* 0: off; capped at 60 */
};

int meshtastic_traffic_init(void);

void meshtastic_traffic_settings(struct meshtastic_traffic_settings *out);

/**
 * @brief Can this port honour the section?
 *
 * @return 0, or -ENOTSUP when nodeinfo_direct_response_max_hops is non-zero.
 */
int meshtastic_traffic_validate(const meshtastic_ModuleConfig_TrafficManagementConfig *cfg);

/** @brief Validate, persist, re-read. */
int meshtastic_traffic_set(const meshtastic_ModuleConfig_TrafficManagementConfig *cfg);

/** @brief ModuleConfig.traffic_management was written by someone else. */
void meshtastic_traffic_config_changed(void);

/**
 * @brief The gate. Called by the router for every received frame after dedup
 *        and before relay/delivery. @p mesh is the decoded MeshPacket on the RF
 *        path (NULL on the inject/test boundary); @p decoded false means the
 *        payload could not be decrypted/decoded and only the header fields of
 *        @p pkt are meaningful.
 */
enum meshtastic_traffic_verdict meshtastic_traffic_inspect(const struct meshtastic_packet *pkt,
							  const meshtastic_MeshPacket *mesh,
							  bool decoded);

/** @brief Counters, in the reference's own protobuf. */
void meshtastic_traffic_stats(meshtastic_TrafficManagementStats *out);

/** @brief Nodes currently tracked. */
size_t meshtastic_traffic_tracked(void);

/** @brief Forget every node and zero the counters (tests, shell). */
void meshtastic_traffic_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_TRAFFIC_H_ */
