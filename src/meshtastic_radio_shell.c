/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic lora` (the modem preset, TX enable and power) and `meshtastic preset`
 * (live preset switching). Built with the shell (src/CMakeLists.txt); moved from
 * meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/fem.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_pki.h"
#include "meshtastic_preset.h"
#include "meshtastic_shell_internal.h"
#include "meshtastic_tx_power.h"

/* --- LoRa modem preset ------------------------------------------------------
 * Gives the shell parity with the phone app / Python CLI for changing the modem
 * preset. `meshtastic lora` shows the current preset/region/primary hash;
 * `meshtastic lora preset <name>` sets it. The write persists through the same
 * config_store path, but the SX1262 is only (re)configured at radio init, so the
 * change takes effect on the next boot (F-1) -- the app/CLI admin path reboots
 * automatically; the shell prints the hint, matching `transport`. */
static void cmd_lora_show(const struct shell *sh)
{
	meshtastic_Config cfg;
	uint8_t pi = meshtastic_channels_primary_index();

	if (meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg) < 0) {
		shell_error(sh, "lora get failed");
		return;
	}
	shell_print(sh, "modem preset: %s%s",
		    meshtastic_preset_display_name(cfg.payload_variant.lora.modem_preset,
						  cfg.payload_variant.lora.use_preset),
		    cfg.payload_variant.lora.use_preset ? "" : " (custom; use_preset=off)");
	shell_print(sh, "region:       %d", (int)cfg.payload_variant.lora.region);
	shell_print(sh, "primary chan: name=\"%s\" hash=0x%02x",
		    meshtastic_channels_get_name(pi), meshtastic_channels_get_hash(pi));

	/* Both numbers, because on a board with a PA front-end they differ and the
	 * difference is the whole point: tx power is what leaves the ANTENNA (the
	 * reference's units, and what the phone and admin API show), drive is what
	 * the transceiver is programmed with. A stored 0 means "as much as the
	 * region allows", so show what that resolved to rather than the 0. */
	{
		bool licensed = false;
		int8_t radiated;
		int8_t drive;

		meshtastic_config_store_get_owner_flags(&licensed, NULL);
		radiated = meshtastic_tx_power_resolve(cfg.payload_variant.lora.tx_power,
						       cfg.payload_variant.lora.region, licensed);
		drive = meshtastic_tx_power_chip_drive(radiated, licensed);

		const char *fem = meshtastic_radio_fem_name();
		const char *dflt =
			(cfg.payload_variant.lora.tx_power == 0) ? " [region default]" : "";

		if (drive == radiated) {
			shell_print(sh, "tx power:     %d dBm at the antenna%s (radio drive "
					"%d dBm)", radiated, dflt, drive);
		} else if (fem != NULL) {
			shell_print(sh, "tx power:     %d dBm at the antenna%s (radio drive "
					"%d dBm; %s makes up the rest)",
				    radiated, dflt, drive, fem);
		} else {
			/* No front end, so nothing makes up the difference — the drive IS
			 * what leaves the antenna, and the requested figure is simply not
			 * reachable on this board.
			 *
			 * The old text said "FEM makes up the rest" whenever the two
			 * numbers differed, which on a bare transceiver was a claim about
			 * hardware that is not fitted: a XIAO + Wio-SX1262 in region US
			 * resolves 30 dBm, clamps the chip to 22, and reported "30 dBm at
			 * the antenna (FEM makes up the rest)" while radiating 22 into an
			 * antenna with no FEM behind it (agents-tosb). Report what the
			 * board will actually do, and say why it is not what was asked
			 * for. (`meshtastic rf path` had this right already — it gates the
			 * gain claim on fem_name — which is where the correct model came
			 * from.) */
			if (meshtastic_radio_fem_state() == MESHTASTIC_FEM_STATE_FAILED) {
				/* "No front end" would be a lie here in the other
				 * direction: this board HAS one, we just could not
				 * identify it, so no gain table is in use and the drive
				 * is the request minus nothing. Saying "no front end"
				 * would send someone looking at the antenna instead of
				 * the FEM rail. */
				shell_print(sh, "tx power:     %d dBm at the antenna "
						"(requested %d%s, clamped to the radio's %d "
						"dBm max; FRONT-END DETECTION FAILED, so no "
						"gain is being compensated — see "
						"`meshtastic rf`)",
					    drive, radiated, dflt, drive);
			} else {
				shell_print(sh, "tx power:     %d dBm at the antenna "
						"(requested %d%s, clamped to the radio's %d "
						"dBm max; no front end to make up the "
						"difference)",
					    drive, radiated, dflt, drive);
			}
		}
	}

	/* The other two knobs a config tool can set that change what the radio
	 * does. Shown because "stored but not doing anything" is exactly the
	 * failure these two used to have. */
	/* The stored flag is the build's seed until the section is written; what the
	 * node does is mt.tx_enabled, which the first-boot hold keeps off meanwhile
	 * (agents-xhli.21). Say which, not "yes" over a silent radio. */
	if (cfg.payload_variant.lora.tx_enabled && !mt.tx_enabled) {
		shell_print(sh, "tx enabled:   NO — held until the LoRa config is written "
				"(stored: yes; `meshtastic lora tx on` releases it)");
	} else {
		shell_print(sh, "tx enabled:   %s",
			    cfg.payload_variant.lora.tx_enabled ? "yes" : "NO — receive only");
	}
	switch (meshtastic_radio_chip_present()) {
	case MESHTASTIC_RADIO_TRI_ON:
		shell_print(sh, "radio chip:   answered at boot");
		break;
	case MESHTASTIC_RADIO_TRI_OFF:
		shell_print(sh, "radio chip:   NOT ANSWERING since boot (absent, unseated or "
				"dead): this node can neither send nor hear");
		break;
	default:
		shell_print(sh, "radio chip:   unknown (the driver cannot say)");
		break;
	}
	switch (cfg.payload_variant.lora.fem_lna_mode) {
	case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED:
		shell_print(sh, "fem lna:      enabled%s",
			    meshtastic_radio_fem_lna_can_control() ? "" : " (but not controllable)");
		break;
	case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED:
		shell_print(sh, "fem lna:      DISABLED (receive path bypasses the amplifier)");
		break;
	default:
		shell_print(sh, "fem lna:      not present on this hardware");
		break;
	}
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_lora_preset_set(const struct shell *sh, const char *name)
{
	meshtastic_Config cfg;
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	if (mtsh_parse_modem_preset(name, &preset) < 0) {
		shell_error(sh, "unknown preset '%s' (e.g. LongFast, MediumFast, ShortTurbo; "
				"see `meshtastic lora`)", name);
		return -EINVAL;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
	if (ret < 0) {
		shell_error(sh, "lora get failed: %d", ret);
		return ret;
	}
	cfg.which_payload_variant = meshtastic_Config_lora_tag;
	cfg.payload_variant.lora.use_preset = true;
	cfg.payload_variant.lora.modem_preset = preset;

	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "lora set failed: %d", ret);
		return ret;
	}
	/* set_config schedules the coalesced flash save; that's independent of the
	 * live radio retune meshtastic_config_store_set_config() already did
	 * above (agents-k8oe) -- no reboot needed either way. */

	shell_print(sh, "modem preset -> %s (persisted and applied live)",
		    meshtastic_preset_display_name(preset, true));
	return 0;
}

/* The "receive only" switch, from the bench. Exists because the stored flag is
 * the ONLY thing standing between the PA and an open antenna connector on a
 * node with damaged RF hardware (agents-a4it.8) — that safety config must be
 * settable before any config tool is paired, and provable afterwards
 * (`meshtastic lora` / `meshtastic rf`). Unlike the preset, this applies LIVE:
 * set_config -> apply_core -> mt.tx_enabled, no reboot involved. */
static int cmd_lora_tx_set(const struct shell *sh, const char *arg)
{
	meshtastic_Config cfg;
	bool enable;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	if (strcmp(arg, "on") == 0) {
		enable = true;
	} else if (strcmp(arg, "off") == 0) {
		enable = false;
	} else {
		shell_error(sh, "usage: meshtastic lora tx on|off");
		return -EINVAL;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
	if (ret < 0) {
		shell_error(sh, "lora get failed: %d", ret);
		return ret;
	}
	cfg.which_payload_variant = meshtastic_Config_lora_tag;
	cfg.payload_variant.lora.tx_enabled = enable;

	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "lora set failed: %d", ret);
		return ret;
	}

	shell_print(sh, "tx %s (persisted, applied now%s)", enable ? "enabled" : "DISABLED",
		    enable ? "" : " — receive only");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

/* `meshtastic preset` — the interlock surface (docs/MULTI-PRESET-OPERATION.md
 * §4.4). Distinct from `meshtastic lora preset`, which PERSISTS a preset and
 * needs a reboot: this one is about the live radio and the conditions that
 * govern moving it on a schedule.
 *
 * The counters are the point of the bare command. A slicing node that quietly
 * never hops, or that hops and strands traffic each time, looks from the
 * outside like a flaky mesh; here it reads as a number next to a reason.
 */
static void cmd_preset_show(const struct shell *sh)
{
	enum meshtastic_preset_hold hold = meshtastic_preset_hold_check();
	uint32_t settle = meshtastic_preset_settle_remaining_ms();

	shell_print(sh, "preset      %s (generation %u)",
		    meshtastic_preset_display_name(mt.modem_preset, true),
		    meshtastic_preset_generation());
	shell_print(sh, "a hop now   %s%s%s", hold == MESHTASTIC_PRESET_HOLD_NONE ? "would go" :
						      "would be REFUSED (",
		    hold == MESHTASTIC_PRESET_HOLD_NONE ? "" : meshtastic_preset_hold_name(hold),
		    hold == MESHTASTIC_PRESET_HOLD_NONE ? "" : ")");
	shell_print(sh, "settle      %u ms remaining of %u", settle,
		    (unsigned int)CONFIG_MESHTASTIC_PRESET_SETTLE_MS);
	shell_print(sh, "");
	shell_print(sh, "hops        %u completed", meshtastic_preset_hops());
	shell_print(sh, "held off    %u no-clock, %u scanning, %u reliable-in-flight,",
		    meshtastic_preset_holds(MESHTASTIC_PRESET_HOLD_NO_CLOCK),
		    meshtastic_preset_holds(MESHTASTIC_PRESET_HOLD_SCANNING),
		    meshtastic_preset_holds(MESHTASTIC_PRESET_HOLD_RELIABLE));
	shell_print(sh, "            %u tx-queued (drain bound %u ms), %u switch-in-progress",
		    meshtastic_preset_holds(MESHTASTIC_PRESET_HOLD_TX_QUEUED),
		    (unsigned int)CONFIG_MESHTASTIC_PRESET_DRAIN_MS,
		    meshtastic_preset_holds(MESHTASTIC_PRESET_HOLD_SWITCHING));
	/* Two different things, and the distinction is worth reading twice: a
	 * settle wait cost a frame some milliseconds; a stale drop cost a frame
	 * entirely, because it outlived the preset it was addressed to. */
	shell_print(sh, "tx settled  %u frames waited out a settle window",
		    meshtastic_preset_settle_waits());
	shell_print(sh, "tx stale    %u frames DROPPED for outliving their preset",
		    meshtastic_preset_tx_stale());
}

static int cmd_preset(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config_LoRaConfig_ModemPreset target;
	struct meshtastic_preset_result got;
	int ret;

	if (argc == 1U) {
		cmd_preset_show(sh);
		return 0;
	}

	if (argc == 2U && strcmp(argv[1], "reset") == 0) {
		meshtastic_preset_stats_reset();
		shell_print(sh, "counters cleared");
		return 0;
	}

	if (argc != 3U || strcmp(argv[1], "hop") != 0) {
		shell_error(sh, "usage: meshtastic preset [hop <name> | reset]");
		return -EINVAL;
	}

	if (mtsh_parse_modem_preset(argv[2], &target) < 0) {
		shell_error(sh, "unknown preset '%s' (e.g. LongFast, ShortTurbo)", argv[2]);
		return -EINVAL;
	}

	ret = meshtastic_preset_hop(target, &got);
	if (ret == -EBUSY) {
		/* Not an error worth a stack trace: this is the interlock doing its
		 * job, and the reason is the useful half. */
		shell_warn(sh, "held off (%s) — still on %s",
			   meshtastic_preset_hold_name(meshtastic_preset_hold_check()),
			   meshtastic_preset_display_name(mt.modem_preset, true));
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "hop failed: %d", ret);
		return ret;
	}

	shell_print(sh, "hopped to %s, %u Hz, SF%u BW%u ch_hash 0x%02x",
		    meshtastic_preset_display_name(got.preset, true), got.frequency_hz,
		    got.spread_factor, got.bandwidth_hz / 1000U, got.channel_hash);
	shell_print(sh, "NOTE: live only — `meshtastic lora preset` is what persists a preset");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_preset)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/* `meshtastic lora power <dBm>`: this node's own transmit power, at the
 * antenna, from its console. Until 2026-10-01 only the phone API could set
 * it -- a brain could set a head's (`attach power`) but nothing could set its
 * own without a BLE config write, which is how the bench's brains sat at
 * 14 dBm while every head was at 2. Same pattern as the head's SET_POLICY:
 * read the LoRa section, change the field, write it back -- applied now,
 * persisted, region-clamped by the store. 0 = the region's maximum. */
static int cmd_lora_power_set(const struct shell *sh, const char *arg)
{
	meshtastic_Config cfg;
	char *end;
	long dbm = strtol(arg, &end, 10);
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	if (end == arg || *end != '\0' || dbm < 0 || dbm > 30) {
		shell_error(sh, "usage: meshtastic lora power <0..30 dBm>  (0 = region maximum)");
		return -EINVAL;
	}
	ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
	if (ret < 0) {
		shell_error(sh, "lora get failed: %d", ret);
		return ret;
	}
	cfg.which_payload_variant = meshtastic_Config_lora_tag;
	cfg.payload_variant.lora.tx_power = (int32_t)dbm;
	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "lora power set failed: %d", ret);
		return ret;
	}
	cmd_lora_show(sh);
	return 0;
}
#endif

static int cmd_lora(const struct shell *sh, size_t argc, char **argv)
{
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc == 1U) {
		cmd_lora_show(sh);
		return 0;
	}
	if (argc != 3U ||
	    (strcmp(argv[1], "preset") != 0 && strcmp(argv[1], "tx") != 0 &&
	     strcmp(argv[1], "power") != 0)) {
		shell_error(sh, "usage: meshtastic lora [preset <name>] [tx on|off] [power <dBm>]");
		return -EINVAL;
	}
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	if (strcmp(argv[1], "tx") == 0) {
		return cmd_lora_tx_set(sh, argv[2]);
	}
	if (strcmp(argv[1], "power") == 0) {
		return cmd_lora_power_set(sh, argv[2]);
	}
	return cmd_lora_preset_set(sh, argv[2]);
#endif
}
MTSH_DEFINE_ON_WORK(cmd_lora)

SHELL_SUBCMD_ADD((meshtastic), lora, NULL,
		 SHELL_HELP("Show or set the LoRa modem preset (reboot to apply), "
		 "the TX-enable switch or the TX power (both apply live).",
		 "[preset <name>] [tx on|off] [power <dBm>]"),
		 cmd_lora_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), preset, NULL,
		 SHELL_HELP("Live preset switching: what would hold a scheduled hop off, "
		 "and the counters for the ones that were.",
		 "[hop <name> | reset]"),
		 cmd_preset_on_work, 0, 0);
