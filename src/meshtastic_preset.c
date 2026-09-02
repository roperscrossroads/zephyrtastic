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
 * Upstream does NOT reboot on a LoRaConfig change — verified against
 * firmware/src/modules/AdminModule.cpp:1108-1110, which sets
 * `requiresReboot = false` unconditionally for the lora section, because
 * every save fires the configChanged observer -> RadioInterface::reconfigure()
 * live. This file's machinery (meshtastic_preset_switch()/_apply_stored()) is
 * what closes that parity gap (agents-k8oe): the radio is already reconfigured
 * on every single TX (meshtastic_radio.c), so the machinery was proven before
 * it was ever driven from a preset or admin config change — see
 * meshtastic_preset_apply_stored() below for the admin/config-store path.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
#include "meshtastic_outbound.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_reliable.h"
#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_scanner.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Interlock state. Small enough to be plain scalars under one mutex; the
 * contenders are the outbound worker (reading, on every transmit) and whichever
 * thread drives the schedule (writing, once a slot). */
static struct {
	/* Bumped once per successful switch; stamped onto every queued frame so a
	 * frame composed for an older preset can be recognised and dropped rather
	 * than transmitted with a stale channel hash. Starts at 1 so that 0 is
	 * never a legitimate stamp. */
	uint32_t generation;
	int64_t settle_until; /* k_uptime_get() ms; TX waits this out */
	uint32_t hops;
	uint32_t holds[MESHTASTIC_PRESET_HOLD_COUNT];
	uint32_t tx_stale;
	uint32_t settle_waits;
} gate = {
	.generation = 1U,
};

/* Serialises hops against each other. NOT a TX gate — the generation stamp is
 * what protects frames; this only keeps two schedules from driving one radio
 * into a half-applied switch. */
static K_MUTEX_DEFINE(hop_lock);

static K_MUTEX_DEFINE(gate_lock_m);

static void gate_lock(void)
{
	k_mutex_lock(&gate_lock_m, K_FOREVER);
}

static void gate_unlock(void)
{
	k_mutex_unlock(&gate_lock_m);
}

/* Arm the settle window. Called on the way out of every successful switch,
 * including the ones an operator or the scanner's restore drives — the radio
 * does not know or care why it was retuned. */
static void settle_arm(void)
{
	if (CONFIG_MESHTASTIC_PRESET_SETTLE_MS == 0) {
		return;
	}

	gate_lock();
	gate.settle_until = k_uptime_get() + CONFIG_MESHTASTIC_PRESET_SETTLE_MS;
	gate_unlock();
}

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

	/* Both of these belong HERE rather than in hop() below, because the radio
	 * does not know or care why it was retuned: an operator-driven `lora
	 * preset`, an admin config change and a slot boundary all leave exactly the
	 * same two problems behind them — an unsettled front end, and a queue that
	 * may still hold frames addressed to the preset we just left. */
	gate_lock();
	gate.generation++;
	gate_unlock();
	settle_arm();

	return 0;
}

int meshtastic_preset_apply_stored(struct meshtastic_preset_result *out)
{
	struct meshtastic_freq_plan plan;
	int ret;

	/* apply_core() has already resolved mt.modem_preset/mt.use_preset/mt.modem
	 * (preset or custom) and every channel's hash from the config that was
	 * just written — nothing to validate or recompute here, just resolve the
	 * frequency for that resolved state and push it all to the radio. */
	ret = meshtastic_region_freq_plan(meshtastic_preset_region(), mt.modem_preset,
					  meshtastic_channels_primary_name(), mt.use_preset,
					  &plan);
	if (ret < 0) {
		LOG_ERR("apply stored config: no frequency plan for preset %d (%d)",
			(int)mt.modem_preset, ret);
		return ret;
	}

	k_mutex_lock(&mt.lock, K_FOREVER);
	mt.frequency = plan.frequency_hz;
	k_mutex_unlock(&mt.lock);

	ret = meshtastic_radio_retune();
	if (ret < 0) {
		LOG_ERR("apply stored config: retune failed (%d)", ret);
		return ret;
	}

	LOG_INF("apply stored config: %s, %u Hz, SF%u BW%u ch_hash 0x%02x",
		meshtastic_preset_display_name(mt.modem_preset, mt.use_preset), plan.frequency_hz,
		mt.modem.spread_factor, mt.modem.bandwidth_hz / 1000U, mt.ch_hash);

	if (out != NULL) {
		out->preset = mt.modem_preset;
		out->frequency_hz = plan.frequency_hz;
		out->spread_factor = mt.modem.spread_factor;
		out->bandwidth_hz = mt.modem.bandwidth_hz;
		out->channel_hash = mt.ch_hash;
	}

	/* Same tail as meshtastic_preset_switch() — see the comment there: the
	 * radio does not know or care why it was retuned. */
	gate_lock();
	gate.generation++;
	gate_unlock();
	settle_arm();

	return 0;
}


/* --- Interlocks (docs/MULTI-PRESET-OPERATION.md §4.4) ---------------------- */

const char *meshtastic_preset_hold_name(enum meshtastic_preset_hold reason)
{
	switch (reason) {
	case MESHTASTIC_PRESET_HOLD_NONE:
		return "none";
	case MESHTASTIC_PRESET_HOLD_NO_CLOCK:
		return "no-clock";
	case MESHTASTIC_PRESET_HOLD_SCANNING:
		return "scanning";
	case MESHTASTIC_PRESET_HOLD_RELIABLE:
		return "reliable-in-flight";
	case MESHTASTIC_PRESET_HOLD_TX_QUEUED:
		return "tx-queued";
	case MESHTASTIC_PRESET_HOLD_SWITCHING:
		return "switch-in-progress";
	default:
		return "?";
	}
}

enum meshtastic_preset_hold meshtastic_preset_hold_check(void)
{
	/* Order is the API's promise, not an implementation detail: first match
	 * wins and the list runs from "will not clear by itself" to "clears in a
	 * frame time", so whatever gets reported is the thing actually worth
	 * acting on. */
	if (!meshtastic_clock_valid()) {
		return MESHTASTIC_PRESET_HOLD_NO_CLOCK;
	}

#if defined(CONFIG_MESHTASTIC_SCANNER)
	/* A sweep already has the radio somewhere this node was never configured
	 * for, and owns the restore. Hopping now would fight it for the tuning. */
	if (meshtastic_scanner_active()) {
		return MESHTASTIC_PRESET_HOLD_SCANNING;
	}
#endif

	if (meshtastic_reliable_pending() > 0U) {
		return MESHTASTIC_PRESET_HOLD_RELIABLE;
	}

	if (meshtastic_outbound_pending() > 0U) {
		return MESHTASTIC_PRESET_HOLD_TX_QUEUED;
	}

	return MESHTASTIC_PRESET_HOLD_NONE;
}

uint32_t meshtastic_preset_generation(void)
{
	uint32_t gen;

	gate_lock();
	gen = gate.generation;
	gate_unlock();

	return gen;
}

void meshtastic_preset_note_tx_stale(void)
{
	gate_lock();
	gate.tx_stale++;
	gate_unlock();
}

uint32_t meshtastic_preset_settle_remaining_ms(void)
{
	int64_t left;

	gate_lock();
	left = gate.settle_until - k_uptime_get();
	gate_unlock();

	if (left <= 0) {
		return 0U;
	}

	return (uint32_t)left;
}

void meshtastic_preset_note_settle_wait(void)
{
	gate_lock();
	gate.settle_waits++;
	gate_unlock();
}

/* Record a refusal and return the errno hop() reports. Split out so every
 * refusal path counts, including the one after the drain times out — an
 * uncounted refusal is exactly the silence these counters exist to remove. */
static int hop_refuse(enum meshtastic_preset_hold reason)
{
	gate_lock();
	gate.holds[reason]++;
	gate_unlock();

	LOG_DBG("preset hop: held off (%s)", meshtastic_preset_hold_name(reason));

	return -EBUSY;
}

int meshtastic_preset_hop(meshtastic_Config_LoRaConfig_ModemPreset preset,
			  struct meshtastic_preset_result *out)
{
	enum meshtastic_preset_hold reason;
	int64_t deadline;
	int ret;

	/* Re-entrancy only. Held across the whole hop so two schedules cannot
	 * interleave a drain with someone else's retune; it gates no transmit. */
	if (k_mutex_lock(&hop_lock, K_NO_WAIT) != 0) {
		return hop_refuse(MESHTASTIC_PRESET_HOLD_SWITCHING);
	}

	/* The three refuse-don't-wait conditions. Checked first because a hop held
	 * off by any of them costs nothing at all — no drain, no stalled TX. */
	reason = meshtastic_preset_hold_check();
	if (reason != MESHTASTIC_PRESET_HOLD_NONE && reason != MESHTASTIC_PRESET_HOLD_TX_QUEUED) {
		k_mutex_unlock(&hop_lock);
		return hop_refuse(reason);
	}

	/* Fourth interlock: drain rather than refuse, because unlike the other
	 * three this one is guaranteed to clear on its own — the worker is already
	 * emptying the queue.
	 *
	 * Nothing is frozen while this runs, deliberately. Those frames were
	 * composed for the preset the node is still on, and letting them go out on
	 * it is the entire reason for waiting instead of hopping immediately.
	 * Refusing them would turn "drain the queue before leaving" into "discard
	 * the queue before leaving", which is a different and worse thing. */
	deadline = k_uptime_get() + CONFIG_MESHTASTIC_PRESET_DRAIN_MS;
	while (meshtastic_outbound_pending() > 0U) {
		if (k_uptime_get() >= deadline) {
			k_mutex_unlock(&hop_lock);
			return hop_refuse(MESHTASTIC_PRESET_HOLD_TX_QUEUED);
		}
		k_sleep(K_MSEC(1));
	}

	/* Anything composed from here on is racing the retune, and is caught after
	 * the fact by the generation stamp rather than by a lock. */
	ret = meshtastic_preset_switch(preset, out);

	if (ret == 0) {
		gate_lock();
		gate.hops++;
		gate_unlock();
	}

	k_mutex_unlock(&hop_lock);

	return ret;
}

uint32_t meshtastic_preset_hops(void)
{
	uint32_t n;

	gate_lock();
	n = gate.hops;
	gate_unlock();

	return n;
}

uint32_t meshtastic_preset_holds(enum meshtastic_preset_hold reason)
{
	uint32_t n;

	if ((unsigned int)reason >= (unsigned int)MESHTASTIC_PRESET_HOLD_COUNT) {
		return 0U;
	}

	gate_lock();
	n = gate.holds[reason];
	gate_unlock();

	return n;
}

uint32_t meshtastic_preset_tx_stale(void)
{
	uint32_t n;

	gate_lock();
	n = gate.tx_stale;
	gate_unlock();

	return n;
}

uint32_t meshtastic_preset_settle_waits(void)
{
	uint32_t n;

	gate_lock();
	n = gate.settle_waits;
	gate_unlock();

	return n;
}

void meshtastic_preset_stats_reset(void)
{
	gate_lock();
	gate.hops = 0U;
	gate.tx_stale = 0U;
	gate.settle_waits = 0U;
	memset(gate.holds, 0, sizeof(gate.holds));
	gate_unlock();
}
