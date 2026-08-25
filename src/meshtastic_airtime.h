/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_AIRTIME_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_AIRTIME_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

float meshtastic_airtime_channel_util_percent(void);

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
