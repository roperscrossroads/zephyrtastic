/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_NEIGHBORINFO_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_NEIGHBORINFO_H_

/*
 * Neighbor info (agents-dnr4.19) -- the reference's NeighborInfoModule.
 *
 * Built on the statusmessage template (meshtastic_statusmessage.h states the
 * rules). What is specific here:
 *
 *   - The neighbor table is fed PROMISCUOUSLY: every packet heard with
 *     hop_limit == hop_start (zero hops away) names a direct neighbor and its
 *     SNR, whatever port it was on. A packet that arrived via MQTT never does.
 *   - A NeighborInfo heard from a direct neighbor additionally records that
 *     neighbor's own broadcast interval, which sets how long its silence is
 *     tolerated (twice its interval, then it is forgotten -- reference
 *     cleanUpNeighbors).
 *   - NOT mirrored: the reference rewrites last_sent_by_id to its own node
 *     number in a NeighborInfo it RELAYS, so a two-hop receiver can attribute
 *     the edge to the relay. This port's relay path forwards the wire bytes
 *     unchanged (no payload rewrite hook exists, and rewriting would mean
 *     re-encrypting), so a relayed NeighborInfo makes no edge here at all --
 *     only direct reception does, which is the only edge either firmware can
 *     actually vouch for.
 *   - The LoRa gate is the reference's: transmit_over_lora, AND NOT (the
 *     primary channel is the default one -- default PSK plus the preset's own
 *     name -- on the default frequency slot). On the public default channel
 *     the packet still goes to the phone, just not on the air.
 *   - Applies live (agents-dnr4.26's hook pattern) even though the reference
 *     reboots for this section: enabled/interval/transmit are read from the
 *     store at every cycle, so a reboot would buy nothing.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_neighborinfo_settings {
	bool enabled;
	bool transmit_over_lora;
	/* Effective interval: update_interval, or the Kconfig default when that is
	 * 0 or below the Kconfig minimum. */
	uint32_t interval_secs;
	/* Whether a broadcast would actually go on the air right now: the flag
	 * above AND the primary channel is not the default channel on the default
	 * frequency slot. */
	bool lora_allowed;
};

struct meshtastic_neighborinfo_entry {
	uint32_t node;
	float snr;
	/* Seconds since this neighbor was last heard. */
	uint32_t age_secs;
	/* The neighbor's own broadcast interval when it told us (via a
	 * NeighborInfo it sent), else the interval we assumed for it. */
	uint32_t interval_secs;
};

int meshtastic_neighborinfo_init(void);

/** @brief Resolve ModuleConfig.neighbor_info into what the module runs with. */
void meshtastic_neighborinfo_settings(struct meshtastic_neighborinfo_settings *out);

/**
 * @brief Persist enabled / interval / transmit_over_lora (0 interval = default)
 *        and re-arm. Goes through the config store like the admin path does.
 */
int meshtastic_neighborinfo_set(bool enabled, uint32_t interval_secs, bool transmit_over_lora);

/** @brief ModuleConfig.neighbor_info was written by someone else: re-arm. */
void meshtastic_neighborinfo_config_changed(void);

/**
 * @brief Broadcast the table now -- on the air when the gate allows, else to
 *        the phone only (reference NODENUM_BROADCAST_NO_LORA).
 *
 * @return 0 queued; -ENODATA when the table is empty (the reference sends
 *         nothing then); other errno from the send.
 */
int meshtastic_neighborinfo_send(void);

/** @brief Number of neighbors currently in the table (expired ones dropped). */
size_t meshtastic_neighborinfo_count(void);

/** @brief Read table slot @p index. false for an empty slot. */
bool meshtastic_neighborinfo_at(size_t index, struct meshtastic_neighborinfo_entry *out);

/** @brief Forget every neighbor (reference resetNeighbors; tests). */
void meshtastic_neighborinfo_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_NEIGHBORINFO_H_ */
