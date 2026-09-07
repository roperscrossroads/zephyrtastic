/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Battery voltage / state-of-charge from the board's VBAT voltage-divider.
 * One owner for the ADC read so any consumer (status UI, telemetry, the
 * low-voltage cutoff) shares it, instead of the read living inside the display
 * module.
 */

#ifndef MESHTASTIC_BATTERY_H_
#define MESHTASTIC_BATTERY_H_

#include <stdbool.h>

/**
 * Battery voltage in millivolts (single cell), or -1 if unavailable
 * (no vbatt node, ADC not ready, or the feature is compiled out). Cached for a
 * few seconds so repeated callers do not re-sample the ADC on every redraw.
 */
int meshtastic_battery_millivolts(void);

/**
 * Estimated state of charge 0-100 from a generic single-cell LiPo OCV curve,
 * or -1 if unknown / no battery (below the ~2600 mV floor).
 */
int meshtastic_battery_percent(void);

/**
 * True if a cell appears to be fitted, i.e. the divider reads at or above the
 * "no battery" floor (OCV_min - 500 mV, ~2600 mV). A USB-powered board with an
 * empty JST reads below this, so anything that would penalise a flat cell must
 * check here first.
 */
bool meshtastic_battery_present(void);

/**
 * True if the pack reads above the charge-termination voltage (OCV_max + 10 mV,
 * ~4200 mV), which on these boards is the only available evidence of external
 * power — see the note in meshtastic_battery.c. False when no reading exists.
 */
bool meshtastic_battery_external_power(void);

/**
 * True while the cell is fitted and below the configured cutoff voltage, i.e.
 * the low-voltage counter is running and the node is heading for a power-off.
 * Always false when the cutoff is compiled out.
 */
bool meshtastic_battery_is_critical(void);

#endif /* MESHTASTIC_BATTERY_H_ */
