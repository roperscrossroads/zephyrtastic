/* SPDX-License-Identifier: GPL-3.0
 *
 * Runtime modem-preset switching — the primitive the multi-preset arc is built
 * on (docs/MULTI-PRESET-OPERATION.md).
 *
 * Changing a preset is NOT just changing SF/BW. The frequency slot and the wire
 * channel hash are both derived from the preset's *display name*, because a
 * default channel stores an empty name and get_name() resolves that to the
 * active preset's name (see meshtastic_channels.c:159-167 — this was a real bug
 * once, where a node pinned to a literal channel name only interoperated on
 * LongFast and silently sat on the wrong frequency everywhere else). So one
 * switch has to move four things:
 *
 *   1. mt.modem_preset + mt.modem   (SF / BW / CR)
 *   2. every channel's hash          (empty names re-resolve to the new preset)
 *   3. mt.frequency                  (djb2 of the primary channel NAME, and the
 *                                     slot width comes from the new bandwidth)
 *   4. the radio itself, then re-arm RX
 *
 * Order matters: the frequency plan reads the primary channel's resolved name,
 * so the channel refresh (2) must precede the frequency resolve (3). Getting
 * that backwards yields a node transmitting on the previous preset's slot with
 * the new modem settings — inaudible to everyone, and invisible locally.
 *
 * Upstream reboots on any LoRaConfig change and never does this live. That is a
 * deliberate divergence, not an oversight: the radio is already reconfigured on
 * every single TX (meshtastic_radio.c), so the machinery is proven — it simply
 * was never driven from a preset change.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
#include "meshtastic_outbound.h"
#include "meshtastic_region_presets.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* The region the frequency plan should be resolved against. Read from the stored
 * LoRaConfig on each switch rather than cached: an admin can change the region,
 * and a stale copy would put us on a legal-elsewhere frequency. UNSET keeps the
 * compile-time default, mirroring meshtastic_config_store.c. */
meshtastic_Config_LoRaConfig_RegionCode meshtastic_preset_region(void)
{
	meshtastic_Config cfg;
	meshtastic_Config_LoRaConfig_RegionCode region =
		(meshtastic_Config_LoRaConfig_RegionCode)CONFIG_MESHTASTIC_DEFAULT_REGION;

	if (meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_lora_tag &&
	    cfg.payload_variant.lora.region != meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
		region = cfg.payload_variant.lora.region;
	}

	return region;
}

int meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset preset,
			     struct meshtastic_preset_result *out)
{
	struct meshtastic_modem_params modem;
	struct meshtastic_freq_plan plan;
	meshtastic_Config_LoRaConfig_ModemPreset prev_preset;
	uint32_t prev_freq;
	int ret;

	/* Validate BEFORE mutating anything: a partially-applied switch is the one
	 * state with no way back on the air.
	 *
	 * meshtastic_preset_to_params() cannot be used as the validity test — it
	 * deliberately folds an unknown preset into the LongFast default row and
	 * returns 0, mirroring the reference's `default:` branch. That is right for
	 * decoding someone else's config, and quite wrong here: silently switching a
	 * node to LongFast because a caller passed garbage is a fleet-partitioning
	 * event (§1.1 of the multi-preset doc). Gate on the display name instead,
	 * which is the reference's own notion of a legal preset. */
	if (strcmp(meshtastic_preset_display_name(preset, true), "Invalid") == 0) {
		LOG_ERR("preset switch: refusing unknown preset %d", (int)preset);
		return -EINVAL;
	}

	ret = meshtastic_preset_to_params(preset, false, &modem);
	if (ret < 0) {
		LOG_ERR("preset switch: params unresolvable for preset %d", (int)preset);
		return ret;
	}

	k_mutex_lock(&mt.lock, K_FOREVER);
	prev_preset = mt.modem_preset;
	prev_freq = mt.frequency;
	mt.modem_preset = preset;
	mt.modem = modem;
	k_mutex_unlock(&mt.lock);

	/* (2) Re-derive every channel hash. Must precede (3): the frequency slot is
	 * djb2 of the primary channel's RESOLVED name, which has just changed for
	 * any channel that stores an empty name. */
	meshtastic_channels_refresh_derived();

	/* (3) Resolve the frequency for the new preset + the refreshed channel name. */
	ret = meshtastic_region_freq_plan(meshtastic_preset_region(), preset,
					  meshtastic_channels_primary_name(), mt.use_preset,
					  &plan);
	if (ret < 0) {
		/* No frequency plan for this region: roll back rather than transmit on
		 * a guess, mirroring meshtastic_region_freq_plan's own refusal. */
		LOG_ERR("preset switch: no frequency plan for preset %d (%d) — rolling back",
			(int)preset, ret);
		k_mutex_lock(&mt.lock, K_FOREVER);
		mt.modem_preset = prev_preset;
		(void)meshtastic_preset_to_params(prev_preset, false, &mt.modem);
		k_mutex_unlock(&mt.lock);
		meshtastic_channels_refresh_derived();
		return ret;
	}

	k_mutex_lock(&mt.lock, K_FOREVER);
	mt.frequency = plan.frequency_hz;
	k_mutex_unlock(&mt.lock);

	/* (4) Push it to the silicon and re-arm RX. */
	ret = meshtastic_radio_retune();
	if (ret < 0) {
		LOG_ERR("preset switch: retune failed (%d)", ret);
		return ret;
	}

	LOG_INF("preset switch: %s -> %s, %u Hz (was %u), SF%u BW%u ch_hash 0x%02x",
		meshtastic_preset_display_name(prev_preset, true),
		meshtastic_preset_display_name(preset, true), plan.frequency_hz, prev_freq,
		modem.spread_factor, modem.bandwidth_hz / 1000U, mt.ch_hash);

	if (out != NULL) {
		out->preset = preset;
		out->frequency_hz = plan.frequency_hz;
		out->spread_factor = modem.spread_factor;
		out->bandwidth_hz = modem.bandwidth_hz;
		out->channel_hash = mt.ch_hash;
	}

	return 0;
}
