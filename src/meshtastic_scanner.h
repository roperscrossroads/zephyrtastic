/* SPDX-License-Identifier: GPL-3.0
 *
 * Multi-preset RX-only survey — docs/MULTI-PRESET-OPERATION.md §3.
 *
 * Cycles the radio across the region's presets and logs the plaintext header of
 * every frame it hears, from any mesh, with no keys and no channel setup. That
 * works because meshtastic_try_decode_wire_packet() fills the header fields
 * BEFORE attempting any decryption, so sender, destination, hop depth, MQTT
 * flag, channel hash, RSSI and SNR are all readable regardless of whose channel
 * a packet belongs to. Payload contents are never touched — this logs the
 * equivalent of packet headers, not traffic.
 *
 * Every existing node can only ever see its own preset, so which presets are in
 * use nearby is genuinely unmeasured information.
 */
#ifndef MESHTASTIC_SCANNER_H_
#define MESHTASTIC_SCANNER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic/config.pb.h"

/** One heard frame. 28 bytes packed — at 4 MB of PSRAM that is ~150k records. */
struct meshtastic_scan_record {
	uint32_t epoch_sec; /**< absolute; needs the wall clock (4f63e11) */
	uint16_t epoch_ms;
	uint32_t from;
	uint32_t to;
	uint32_t id;
	uint8_t  preset;      /**< which preset heard it (ModemPreset) */
	uint8_t  flags;       /**< hop_limit | hop_start | want_ack | via_mqtt */
	uint8_t  chan_hash;   /**< the sender's channel hash, NOT ours */
	uint8_t  next_hop;
	uint8_t  relay_node;
	int16_t  rssi;
	int8_t   snr;
	uint8_t  payload_len; /**< length only — contents are never stored */
} __packed;

/** What the survey has observed for one preset. */
struct meshtastic_scan_stats {
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	uint32_t frequency_hz;
	uint32_t heard;          /**< frames captured on this preset */
	uint32_t dwell_ms_total; /**< time actually spent listening here */
	uint32_t visits;
};

/** Most presets a sweep can be limited to (the full US PROFILE_STD set). */
#define MESHTASTIC_SCANNER_MAX_PRESETS 10

/**
 * @brief Restrict the sweep to a subset of presets.
 *
 * Narrowing the list raises the capture probability on each remaining preset,
 * because the cycle gets shorter while each dwell stays the same: P = c/T, and T
 * is the sum of the dwells actually visited. Four presets instead of ten roughly
 * doubles P on each of them — worth doing when you already know which presets are
 * in use and want a rate estimate rather than a discovery sweep.
 *
 * Takes effect at the next preset change, not mid-dwell.
 *
 * @param list  Presets to visit, in order. NULL or @p n == 0 restores the full set.
 * @param n     How many, up to MESHTASTIC_SCANNER_MAX_PRESETS.
 * @return 0, -EINVAL on too many or an unknown preset.
 */
int meshtastic_scanner_set_presets(const meshtastic_Config_LoRaConfig_ModemPreset *list, size_t n);

/**
 * @brief The presets currently being swept.
 * @return number written to @p out, or negative errno.
 */
int meshtastic_scanner_get_presets(meshtastic_Config_LoRaConfig_ModemPreset *out, size_t max);

/** @brief Begin cycling. Idempotent. @return 0, or -EALREADY if already running. */
int meshtastic_scanner_start(void);

/**
 * @brief Stop cycling and return the radio to the configured preset.
 *
 * Restores via meshtastic_preset_switch() rather than tuning back by hand, so
 * mt.modem_preset, the channel hashes and the radio all agree again.
 */
int meshtastic_scanner_stop(void);

/**
 * @brief True while the node is OFF its operating preset.
 *
 * The single condition governing both directions: while it holds, the node
 * neither transmits nor lets received frames into the participant stack —
 * anything it hears belongs to a mesh it is not a member of.
 *
 * It spans more than the sweep itself: it stays true through the restore, and
 * stays true forever if the restore fails, because a node that could not get
 * back to its operating preset is still sitting on a scan frequency.
 */
bool meshtastic_scanner_active(void);

/** @brief True while the sweep thread is actually cycling presets. */
bool meshtastic_scanner_sweeping(void);

/**
 * @brief Per-preset observations.
 *
 * @param out   Array to fill.
 * @param max   Capacity of @p out.
 * @return number written, or negative errno.
 */
int meshtastic_scanner_stats(struct meshtastic_scan_stats *out, size_t max);

/**
 * @brief Copy out captured records, oldest first.
 *
 * @param out    Array to fill.
 * @param max    Capacity of @p out.
 * @param from   Index of the first record to copy (0 = oldest retained).
 * @return number written, or negative errno.
 */
int meshtastic_scanner_records(struct meshtastic_scan_record *out, size_t max, uint32_t from);

/** @brief Total records ever captured (may exceed what the ring retains). */
uint32_t meshtastic_scanner_total(void);

/**
 * @brief Count a transmission refused because a sweep was running.
 *
 * Called from the TX choke point. Refusals are counted rather than silent so
 * that "nothing tried to transmit while scanning" is an assertion rather than an
 * assumption — a non-zero count means some path kept trying, which is a bug
 * worth seeing rather than a silence worth trusting.
 */
void meshtastic_scanner_note_tx_blocked(void);

/** @brief Transmissions refused since the last reset. Should be ZERO; see above. */
uint32_t meshtastic_scanner_tx_blocked(void);

/** @brief Count a received frame withheld from the participant stack. */
void meshtastic_scanner_note_rx_dropped(void);

/**
 * @brief Frames surveyed but not passed on, since the last reset.
 *
 * Expected to be non-zero and roughly the capture count — unlike tx_blocked,
 * this is normal operation rather than an alarm.
 */
uint32_t meshtastic_scanner_rx_dropped(void);

/** @brief Drop every retained record and reset the per-preset stats. */
void meshtastic_scanner_reset(void);

/**
 * @brief Feed one heard frame to the survey. Called from the radio RX path.
 *
 * Deliberately takes the RAW buffer rather than a decoded packet: the scanner
 * must see frames the normal stack discards (wrong channel, undecryptable,
 * filtered), which is most of what a foreign mesh produces.
 */
void meshtastic_scanner_on_frame(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr);

#endif /* MESHTASTIC_SCANNER_H_ */
