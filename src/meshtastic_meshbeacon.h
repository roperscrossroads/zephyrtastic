/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_MESHBEACON_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_MESHBEACON_H_

/*
 * Mesh beacon (agents-dnr4.25) -- the reference's MeshBeaconBroadcastModule +
 * MeshBeaconListenerModule, on the statusmessage template.
 *
 * A beacon is a zero-hop "join my mesh" announcement on MESH_BEACON_APP: a short
 * text and/or an offer (channel name + PSK, region, modem preset) for a listening
 * client app to act on. Two independent halves, each behind a flag in
 * ModuleConfig.mesh_beacon.flags:
 *
 *   BROADCAST -- every interval (min 1 h), send the configured content as one
 *     MESH_BEACON_APP packet, or under FLAG_LEGACY_SPLIT as two (offer-only on
 *     MESH_BEACON_APP + the text on TEXT_MESSAGE_APP, for receivers that only
 *     decode text). Always hop_limit 0; with the legacy flag hop_start 1, so
 *     receivers that predate hop_start 0 still accept the frame. Not while the
 *     role is CLIENT_HIDDEN.
 *   LISTEN -- decode a heard beacon, log its text, and cache its offer for the
 *     client app (never applied automatically). The packet itself reaches the
 *     phone like any other; the text is deliberately NOT re-injected as a text
 *     message (the reference does not either -- an aware client renders it from
 *     the beacon, and a synthesised copy would duplicate it).
 *
 * WHERE THIS PORT STOPS SHORT OF THE REFERENCE, on purpose:
 *   - A broadcast target names a radio config: preset, region and a channel.
 *     The reference switches the radio around each transmission whose preset or
 *     region differs from the running one (a ~350-line sidecar in its TX hook).
 *     Here a target is honoured only on the RUNNING preset and region; a target
 *     asking for another is skipped, logged and counted. Switching the radio
 *     under live traffic belongs to the multi-preset arc
 *     (docs/MULTI-PRESET-OPERATION.md), which owns the hold/restore rules.
 *   - A channel is honoured only as a channel-table slot: a target's
 *     channel_index, or an inline broadcast_on_channel whose name+PSK match a
 *     slot. The reference encrypts an inline channel by swapping it into the
 *     primary slot for the duration of the send; this port's send path encrypts
 *     by slot and its other senders run concurrently, so an inline channel with
 *     no slot is skipped, logged and counted.
 *   - broadcast_send_as_node is not acted on. The reference's code does not act
 *     on it either (only the proto comment describes it), and a spoofed `from`
 *     is not something to introduce ahead of the reference.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic/channel.pb.h"
#include "meshtastic/config.pb.h"
#include "meshtastic/module_config.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESHTASTIC_MESHBEACON_FLAG_LISTEN                                                        \
	meshtastic_ModuleConfig_MeshBeaconConfig_Flags_FLAG_LISTEN_ENABLED
#define MESHTASTIC_MESHBEACON_FLAG_BROADCAST                                                     \
	meshtastic_ModuleConfig_MeshBeaconConfig_Flags_FLAG_BROADCAST_ENABLED
#define MESHTASTIC_MESHBEACON_FLAG_LEGACY_SPLIT                                                  \
	meshtastic_ModuleConfig_MeshBeaconConfig_Flags_FLAG_LEGACY_SPLIT

/* The reference's hard cap on the text (proto max_size 101 = 100 + NUL). */
#define MESHTASTIC_MESHBEACON_MESSAGE_MAX 100U

/** The last offer heard while listening (reference BeaconOffer). */
struct meshtastic_meshbeacon_offer {
	bool valid;
	uint32_t sender;
	bool has_channel;
	meshtastic_ChannelSettings channel;
	meshtastic_Config_LoRaConfig_RegionCode region;
	bool has_preset;
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	/* Uptime when it was heard; the shell shows the age. */
	int64_t heard_uptime_ms;
};

struct meshtastic_meshbeacon_stats {
	uint32_t frames_sent;
	uint32_t cycles_empty;          /* broadcast cycles with nothing to say */
	uint32_t targets_radio_switch;  /* skipped: needs another preset/region */
	uint32_t targets_no_slot;       /* skipped: inline channel not in the table */
	uint32_t beacons_heard;
	uint32_t offers_cached;
};

int meshtastic_meshbeacon_init(void);

/**
 * @brief Apply the reference AdminModule's write-time rules to a section, in
 *        place: text capped at 100, interval 0/low -> the minimum, unknown
 *        region/preset cleared, out-of-range channel_index cleared. Pure.
 */
void meshtastic_meshbeacon_sanitise(meshtastic_ModuleConfig_MeshBeaconConfig *cfg);

/** @brief Sanitise, persist and re-arm. */
int meshtastic_meshbeacon_set(const meshtastic_ModuleConfig_MeshBeaconConfig *cfg);

/** @brief ModuleConfig.mesh_beacon was written by someone else: re-arm. */
void meshtastic_meshbeacon_config_changed(void);

/** @brief Effective broadcast interval (0/low resolved to the minimum). */
uint32_t meshtastic_meshbeacon_interval_secs(void);

/**
 * @brief Beacon now, regardless of the BROADCAST flag and the interval (the
 *        shell's `beacon send`). Honours the role gate.
 *
 * @return 0 when at least one frame was queued; -ENODATA when the section has
 *         neither text nor offer; -ENOENT when every target was skipped;
 *         -EPERM for a CLIENT_HIDDEN node; else the send's errno.
 */
int meshtastic_meshbeacon_send(void);

bool meshtastic_meshbeacon_last_offer(struct meshtastic_meshbeacon_offer *out);
void meshtastic_meshbeacon_clear_offer(void);
void meshtastic_meshbeacon_stats(struct meshtastic_meshbeacon_stats *out);

/** @brief Forget the cached offer and zero the counters (tests). */
void meshtastic_meshbeacon_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_MESHBEACON_H_ */
