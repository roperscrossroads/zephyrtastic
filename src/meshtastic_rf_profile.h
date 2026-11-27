/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Radio profiles: a few named sets of the receive and power settings
 * (docs: tooling RF-PROFILES.md). A profile is not stored. It is a set of the
 * standard LoRa config fields (sx126x_rx_boosted_gain, fem_lna_mode and, for
 * Bench only, tx_power), and the node reports whichever profile its fields
 * match, or "custom", so a phone app that changes one field is reported
 * correctly with no extra state.
 */
#ifndef MESHTASTIC_RF_PROFILE_H_
#define MESHTASTIC_RF_PROFILE_H_

#include <stdbool.h>

#include "meshtastic/config.pb.h"

/** The transmit power the Bench profile sets, in dBm at the antenna. */
#define MESHTASTIC_RF_PROFILE_BENCH_DBM 2

/** The profile a LoRa config matches: "standard", "battery", "crowded", "bench" or "custom".
 *  @p lna_controllable: whether this board's front end has a switchable LNA; without one
 *  the LNA field does not tell profiles apart. */
const char *meshtastic_rf_profile_of(const meshtastic_Config_LoRaConfig *l, bool lna_controllable);

/** Write profile @p name's fields into @p l. Only Bench changes tx_power.
 *  @return 0, or -EINVAL for an unknown name (including "adaptive", not built). */
int meshtastic_rf_profile_set(meshtastic_Config_LoRaConfig *l, const char *name);

/** One line: what profile @p name is for and what it costs, or NULL. */
const char *meshtastic_rf_profile_tradeoff(const char *name);

/** The profile names, in display order, ending with NULL. */
extern const char *const meshtastic_rf_profile_names[];

#endif /* MESHTASTIC_RF_PROFILE_H_ */
