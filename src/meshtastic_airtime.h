/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_AIRTIME_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_AIRTIME_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One channel-utilization ring per modem preset, plus one for a custom
 * (use_preset=false) radio config. ModemPreset runs 0..13.
 *
 * WHY PER PRESET, AND WHY *ONLY* CHANNEL UTILIZATION. Presets are doubly
 * orthogonal — different frequency AND different spreading factor — so two
 * presets are two different channels that cannot hear each other. Channel
 * utilization is a statement about ONE channel's business, and it drives CSMA
 * backoff and the background-beacon gate; blending two channels' traffic into
 * one number makes both decisions wrong (MULTI-PRESET-OPERATION.md §4.3).
 *
 * TX utilization is deliberately NOT split, and that is not an omission. It
 * feeds the regulatory duty cycle, which is a property of the BAND, not of the
 * channel: two presets a node slices between sit in the same regional
 * allocation, and the regulator counts every second the PA is keyed regardless
 * of which spreading factor it was keyed at. Splitting it per preset would let
 * a slicing node transmit up to the ceiling TWICE — which is why the ring stays
 * whole and meshtastic_duty.c keeps reading it.
 */
#define MESHTASTIC_AIRTIME_PRESET_SLOTS 15
#define MESHTASTIC_AIRTIME_SLOT_CUSTOM  (MESHTASTIC_AIRTIME_PRESET_SLOTS - 1)

#define MESHTASTIC_CHANNEL_UTILIZATION_PERIODS 6
#define MESHTASTIC_MINUTES_IN_HOUR             60
#define MESHTASTIC_SECONDS_IN_MINUTE           60
#define MESHTASTIC_MS_IN_MINUTE                (MESHTASTIC_SECONDS_IN_MINUTE * 1000)
#define MESHTASTIC_MS_IN_HOUR                                                  \
	(MESHTASTIC_MINUTES_IN_HOUR * MESHTASTIC_SECONDS_IN_MINUTE * 1000)

enum meshtastic_airtime_type {
	MESHTASTIC_AIRTIME_TX,
	MESHTASTIC_AIRTIME_RX,
	MESHTASTIC_AIRTIME_RX_ALL,
};

int meshtastic_airtime_init(void);

uint32_t meshtastic_airtime_packet_ms(uint32_t wire_len);

void meshtastic_airtime_log(enum meshtastic_airtime_type type, uint32_t ms);

/**
 * @brief Channel utilization (%) of the preset the radio is on RIGHT NOW.
 *
 * The 60 s window, scoped to the current channel — what CSMA backoff and the
 * background-beacon gate must see.
 */
float meshtastic_airtime_channel_util_percent(void);

/**
 * @brief Channel utilization (%) of one preset slot, for diagnostics.
 *
 * @param slot 0..MESHTASTIC_AIRTIME_PRESET_SLOTS-1; the last is the custom
 *             (use_preset=false) slot. Out of range returns 0.
 */
float meshtastic_airtime_channel_util_percent_slot(uint8_t slot);

/** @brief The slot the radio's current configuration accounts to. */
uint8_t meshtastic_airtime_current_slot(void);

/**
 * @brief TX utilization (%) over the last hour, across EVERY preset.
 *
 * Band-wide on purpose — see the note on MESHTASTIC_AIRTIME_PRESET_SLOTS. This
 * is the number the regulatory duty cycle is measured against, and it must stay
 * whole even on a node that slices between presets.
 */
float meshtastic_airtime_tx_util_percent(void);

/**
 * @brief Minutes until the rolling TX-utilization window falls back under a ceiling.
 *
 * Answers "when can I send again" for the duty-cycle gate: each passing minute
 * drops the oldest bucket out of the one-hour window, so this walks the window
 * oldest-first and reports the first age at which the total is under
 * @p duty_cycle_pct. 0 when already under it, 60 when even an empty window would
 * not be (which cannot happen with a sane ceiling, but is the safe answer).
 */
uint8_t meshtastic_airtime_silent_minutes(float tx_percent, float duty_cycle_pct);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_AIRTIME_H_ */
