/* SPDX-License-Identifier: GPL-3.0
 *
 * Runtime modem-preset switching. See meshtastic_preset.c for why a preset
 * change moves the frequency and the channel hash as well as the modem.
 */
#ifndef MESHTASTIC_PRESET_H_
#define MESHTASTIC_PRESET_H_

#include <stdint.h>

#include "meshtastic/config.pb.h"

/** What a switch actually resolved to. Every field is a thing that must change
 *  together; a caller (or test) asserting on only one of them cannot tell a
 *  complete switch from a half-applied one. */
struct meshtastic_preset_result {
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	uint32_t frequency_hz;
	uint32_t bandwidth_hz;
	uint8_t  spread_factor;
	uint8_t  channel_hash;
};

/**
 * @brief Move the radio to a different modem preset, live, without a reboot.
 *
 * Applies the modem params, re-derives every channel hash, resolves the new
 * frequency slot, pushes it all to the radio and re-arms RX — in that order,
 * because the frequency depends on the refreshed channel name.
 *
 * @warning This makes the node deaf to every peer still on the old preset, and
 * inaudible to them, immediately. Presets are doubly orthogonal (different
 * frequency AND different spreading factor), so there is no partial overlap to
 * fall back on. Switching a whole fleet is a partition-inducing operation —
 * see docs/MULTI-PRESET-OPERATION.md §1.1.
 *
 * Atomicity: an unknown preset changes nothing. A preset with no frequency plan
 * for the current region is rolled back. Only a radio-level failure can leave
 * the node mid-switch, and that is already fatal for TX either way.
 *
 * @param preset Target preset.
 * @param out    Optional; receives what the switch resolved to.
 *
 * @return 0 on success, -EINVAL for an unknown preset, -ENOTSUP when the region
 *         has no frequency plan (rolled back), or a negative errno from the
 *         radio.
 */
int meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset preset,
			     struct meshtastic_preset_result *out);

#endif /* MESHTASTIC_PRESET_H_ */
