/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic rf`: the gain-path report (CONFIG_MESHTASTIC_RF_PATH_REPORT) and signal
 * measurement (CONFIG_MESHTASTIC_RF_HIST). Built with the shell and either of them
 * (src/CMakeLists.txt); moved from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

#if defined(CONFIG_MESHTASTIC_RF_PATH_REPORT)
#include "meshtastic_rf_path.h"

/*
 * `meshtastic rf` — the gain path, in signal order, reporting what is IN EFFECT.
 *
 * Rows carry a state marker rather than being omitted when they do not apply,
 * because an omitted row cannot be told apart from a forgotten one. The three
 * that matter:
 *   [ok] the radio is doing what the configuration asks
 *   [!!] the configuration asks for something the radio is NOT doing
 *   [--] this hardware has no such control
 *   [??] the driver cannot tell us (never reported as "off")
 */
static const char *tri_str(enum meshtastic_radio_tristate t)
{
	switch (t) {
	case MESHTASTIC_RADIO_TRI_ON:
		return "ON";
	case MESHTASTIC_RADIO_TRI_OFF:
		return "OFF";
	default:
		return "unknown";
	}
}

static const char *lna_mode_str(meshtastic_Config_LoRaConfig_FEM_LNA_Mode m)
{
	switch (m) {
	case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED:
		return "ENABLED";
	case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED:
		return "DISABLED";
	default:
		return "NOT_PRESENT";
	}
}

static int cmd_rf_path(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_rf_path p;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_rf_path_get(&p);
	if (ret < 0) {
		shell_error(sh, "rf: could not read the gain path (%d)", ret);
		return ret;
	}

	shell_print(sh, "RF path — effective values from the driver and board hooks, not "
			"stored config.");
	shell_print(sh, "  [ok] in effect   [!!] configured but NOT in effect   "
			"[--] not on this hardware   [??] unknown");

	shell_print(sh, "");
	shell_print(sh, "FRONT END");
	/* These four states used to share two lines, and the NULL-name line had to
	 * say "none fitted, or detection did not complete" because the board hook
	 * could not tell the reader which. They are now distinct, because a healthy
	 * board with no front-end and a board that is silently under-driving
	 * transmit by 11-13 dB must not print the same thing. */
	switch (p.fem_state) {
	case MESHTASTIC_FEM_STATE_DETECTED:
		shell_print(sh, "  [ok] front-end       %s  (detected at boot)",
			    (p.fem_name != NULL) ? p.fem_name : "(unnamed)");
		break;
	case MESHTASTIC_FEM_STATE_MISMATCH:
		shell_print(sh, "  [!!] front-end       %s  — DETECTION DISAGREED WITH THE BOARD",
			    (p.fem_name != NULL) ? p.fem_name : "(unnamed)");
		shell_print(sh, "                       This board is only ever fitted with the "
				"part named above,");
		shell_print(sh, "                       so that is what is in use — but the "
				"detect line said");
		shell_print(sh, "                       otherwise. Suspect the FEM rail or the "
				"CSD line; the");
		shell_print(sh, "                       radio is working, the evidence for WHY "
				"is not.");
		break;
	case MESHTASTIC_FEM_STATE_FAILED:
		shell_print(sh, "  [!!] front-end       DETECTION FAILED — this board has one "
				"and we cannot tell which");
		shell_print(sh, "                       Transmit assumes the HIGHEST-gain part "
				"so radiated power");
		shell_print(sh, "                       stays at or below the request whichever "
				"is really fitted");
		shell_print(sh, "                       (and whether or not it is powered) — so "
				"expect up to 13 dB");
		shell_print(sh, "                       LESS range, deliberately. Receive is "
				"whatever the FEM");
		shell_print(sh, "                       came up in; the LNA is not being driven. "
				"Not a config");
		shell_print(sh, "                       problem — suspect the FEM rail or the "
				"CSD line.");
		break;
	case MESHTASTIC_FEM_STATE_NONE:
	default:
		shell_print(sh, "  [--] front-end       none fitted on this hardware");
		break;
	}
	shell_print(sh, "  [%s] T/R switch      %s",
		    p.dio2_rf_switch ? "ok" : "--",
		    p.dio2_rf_switch ? "driven by the transceiver (dio2-tx-enable)"
				     : "not driven by the transceiver on this board");

	shell_print(sh, "");
	shell_print(sh, "RECEIVE");
	shell_print(sh, "  [%s] rx boosted gain  staged %s, applied %s   (config %s)",
		    meshtastic_rf_row_mark(p.rx_boost_row), tri_str(p.rx_boost_staged),
		    tri_str(p.rx_boost_applied), p.rx_boost_config ? "ON" : "OFF");
	if (p.rx_boost_row == MESHTASTIC_RF_ROW_INEFFECTIVE) {
		shell_print(sh, "       -> the chip is not on the configured gain; RX "
				"sensitivity is ~2-3 dB below what the config claims");
	} else if (p.rx_boost_row == MESHTASTIC_RF_ROW_UNKNOWN) {
		shell_print(sh, "       -> this radio's driver cannot report the applied "
				"gain; not assuming it is off");
	}

	if (p.lna_can_control) {
		shell_print(sh, "  [%s] fem lna          %s (board intent; the pin is shared "
				"with TX mode)", meshtastic_rf_row_mark(p.lna_row),
			    p.lna_intent ? "LNA" : "bypass");
		shell_print(sh, "                        config %s", lna_mode_str(p.lna_config));
	} else {
		shell_print(sh, "  [--] fem lna          no controllable receive path on this "
				"front-end");
		shell_print(sh, "                        stored %s", lna_mode_str(p.lna_config));
	}
	shell_print(sh, "  [%s] rx armed         %s", p.rx_armed ? "ok" : "!!",
		    p.rx_armed ? "yes" : "NO — the radio is not listening");

	shell_print(sh, "");
	shell_print(sh, "TRANSMIT");
	shell_print(sh, "  [%s] tx enabled       %s", p.tx_enabled ? "ok" : "!!",
		    p.tx_enabled ? "yes" : "NO — receive only");
	shell_print(sh, "  [ok] tx power         %d dBm requested at the antenna%s",
		    p.tx_power_radiated, (p.tx_power_stored == 0) ? "  [region default]" : "");
	shell_print(sh, "                        region %d limit %d dBm, %s",
		    (int)p.region, p.region_limit_dbm,
		    p.licensed ? "licensed" : "unlicensed");
	shell_print(sh, "  [%s] drive level      %d dBm%s",
		    p.tx_drive_was_clamped ? "!!" : "ok", p.tx_drive_clamped,
		    p.tx_fem_gain_applied ? "  (the front-end adds the rest)" : "");
	if (p.tx_drive_was_clamped) {
		/* Two distinct stories share this line, and conflating them is exactly
		 * how an operator ends up believing a bare SX1262 radiates 30 dBm. */
		if (p.tx_drive_clamped > p.tx_drive_wanted) {
			/* The LOW end of the clamp, and the only case here that radiates
			 * MORE than was asked for. The radio cannot be driven below its
			 * floor, so a request the front-end's gain would have met at a
			 * lower drive comes out hot by the difference. Worth its own text:
			 * the shortfall wording below describes the opposite problem, and
			 * an operator who deliberately turned power DOWN to be a good
			 * neighbour is exactly the person this line is for. */
			shell_print(sh, "  [!!] OVER the request  wanted drive %d dBm, but the "
					"radio floor is %d dBm",
				    p.tx_drive_wanted, CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER);
			shell_print(sh, "       -> radiating about %d dBm, roughly %d dB MORE "
					"than the %d dBm requested.",
				    (int)p.tx_drive_clamped +
					    ((int)p.tx_power_radiated - (int)p.tx_drive_wanted),
				    (int)p.tx_drive_clamped - (int)p.tx_drive_wanted,
				    p.tx_power_radiated);
			shell_print(sh, "          Ask for less than the front-end's gain and "
					"the transceiver cannot go");
			shell_print(sh, "          low enough to deliver it. Not a fault — a "
					"floor. Nothing in software");
			shell_print(sh, "          can fix it; use a lower-gain antenna or "
					"accept the figure above.");
		} else if (p.tx_fem_gain_applied) {
			shell_print(sh, "       -> wanted drive %d dBm, CLAMPED to [%d..%d]; "
					"the front-end cannot make up the shortfall",
				    p.tx_drive_wanted, CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
				    CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER);
		} else {
			shell_print(sh, "       -> no front-end, so this IS the radiated power: "
					"%d dBm, not the %d dBm requested (radio range [%d..%d])",
				    p.tx_drive_clamped, p.tx_power_radiated,
				    CONFIG_MESHTASTIC_RADIO_MIN_TX_POWER,
				    CONFIG_MESHTASTIC_RADIO_MAX_TX_POWER);
		}
	}

	shell_print(sh, "");
	shell_print(sh, "LAST PUSHED TO THE RADIO");
	if (p.eff.generation == 0U) {
		shell_print(sh, "  [??] never configured — the radio has not accepted a "
				"config since boot");
	} else {
		shell_print(sh, "  [%s] %u Hz  SF%u  BW%uk  CR4/%u   rc=%d  gen %u",
			    (p.eff.last_rc == 0) ? "ok" : "!!", p.eff.frequency,
			    p.eff.spread_factor, p.eff.bandwidth_hz / 1000U, p.eff.coding_rate,
			    p.eff.last_rc, p.eff.generation);
		shell_print(sh, "       drive: %d dBm on the last RX config, %d dBm on the "
				"last TX", p.eff.tx_power_rx_cfg, p.eff.tx_power_tx_cfg);
	}

	shell_print(sh, "");
	shell_print(sh, "HEALTH");
	shell_print(sh, "  [%s] CAD              clear %u  busy %u  timeout %u  error %u",
		    (p.cad_timeout == 0U && p.cad_error == 0U) ? "ok" : "!!", p.cad_clear,
		    p.cad_busy, p.cad_timeout, p.cad_error);
	shell_print(sh, "  [%s] AGC reset        ok %u  fail %u  skipped %u  deferred %u  "
			"radio-busy %u  patch-fail %u",
		    (p.agc_fail == 0U && p.agc_patch_fail == 0U) ? "ok" : "!!", p.agc_ok,
		    p.agc_fail, p.agc_skipped, p.rx_act.agc_deferred, p.rx_act.agc_radio_busy,
		    p.agc_patch_fail);
	/* "skipped" is a TX in flight; "deferred" is a packet arriving. The
	 * activity row is the mechanism: the chip's latched flags this instant
	 * (unknown on a driver that cannot report them), the two timing rules'
	 * verdicts, and the windows they judge against on the current modem. */
	shell_print(sh, "  [%s] RX activity      now: preamble %s  header %s   busy-rx %u  "
			"false-preamble %u  false-header %u  bad-crc %u",
		    (p.rx_act_preamble == MESHTASTIC_RADIO_TRI_UNKNOWN) ? "??" : "--",
		    tri_str(p.rx_act_preamble), tri_str(p.rx_act_header), p.rx_act.busy_rx,
		    p.rx_act.false_preamble, p.rx_act.false_header,
		    (unsigned int)meshtastic_radio_crc_error_count());
	shell_print(sh, "       windows: preamble %u ms, max packet %u ms", p.rx_act.preamble_ms,
		    p.rx_act.max_packet_ms);
	/* A defer is a transmit the radio refused for now (a packet arriving, or
	 * CAD hearing one) and the queue re-rolled; a drop used up every defer on
	 * a channel that never went quiet. Drops are the only bad number here. */
	shell_print(sh, "  [%s] TX defer         busy-rx %u  cad-busy %u  requeued %u  dropped %u",
		    (p.tx_defer.dropped == 0U) ? "ok" : "!!", p.tx_defer.busy_rx,
		    p.tx_defer.cad_busy, p.tx_defer.requeued, p.tx_defer.dropped);
	if (p.tx_defer.dropped != 0U) {
		/* Attribute the drops. The two causes are unrelated -- one is the channel,
		 * one is queue depth -- and reporting them as a single number sent readers
		 * to the RF environment for what was queue-full (agents-q0c6). */
		shell_print(sh, "       dropped: %u at the defer cap (channel never quiet), "
				"%u with the outbound queue full",
			    p.tx_defer.dropped_cap, p.tx_defer.dropped_qfull);
	}
	shell_print(sh, "  [%s] SPI BUSY streak  %u", (p.busy_streak == 0U) ? "ok" : "!!",
		    p.busy_streak);
	/* Cumulative, unlike the streak above: a wedge clears the streak the
	 * instant it resets the radio, so this is the only durable evidence a
	 * wedge ever fired this boot. */
	shell_print(sh, "  [%s] radio wedge-resets  %u", (p.wedge_resets == 0U) ? "ok" : "!!",
		    p.wedge_resets);

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_rf_path)
#endif /* CONFIG_MESHTASTIC_RF_PATH_REPORT */

#if defined(CONFIG_MESHTASTIC_RF_HIST)
#include "meshtastic_rf_measure.h"

/* Integer mean scaled by 10, so a tenth of a dB shows without pulling float
 * formatting into the shell (which several boards build without). */
static int32_t mean_x10(int64_t sum, uint32_t n)
{
	if (n == 0U) {
		return 0;
	}
	/* Computed in 64-bit and only then narrowed: sum * 10 on a long window
	 * would overflow a 32-bit intermediate long before the sum itself did.
	 * The result is a mean in tenths of a dB, which always fits. */
	return (int32_t)((sum * 10) / (int64_t)n);
}

/* Tenths digit of a scaled mean, always positive: printing "-88.-4" instead of
 * "-88.4" is the classic way a hand-rolled fixed-point formatter goes wrong. */
static int32_t tenth(int32_t x10)
{
	int32_t t = x10 % 10;

	return (t < 0) ? -t : t;
}

static void print_bar(const struct shell *sh, const char *label, uint32_t count, uint32_t max)
{
	char bar[33];
	size_t w = 0;

	if (max > 0U && count > 0U) {
		w = (size_t)((count * 32U) / max);
		if (w == 0U) {
			w = 1U; /* never render a nonzero count as empty */
		}
	}
	memset(bar, '#', w);
	bar[w] = '\0';
	shell_print(sh, "  %-14s %6u  %s", label, count, bar);
}

static uint32_t max_of(const uint32_t *a, size_t n)
{
	uint32_t m = 0;

	for (size_t i = 0; i < n; i++) {
		m = MAX(m, a[i]);
	}
	return m;
}

static int cmd_rf_hist(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_rf_window w;
	char label[16];
	uint32_t max;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_rf_window_get(&w);

	shell_print(sh, "window %u s   frames %u   lifetime %u", w.window_ms / 1000U,
		    w.hist.frames, w.lifetime);
	if (w.hist.frames == 0U) {
		shell_print(sh, "nothing heard yet — no distribution to show");
		return 0;
	}
	if (w.preset_mixed) {
		shell_print(sh, "WARNING: the modem preset changed during this window. "
				"Spreading factor moves sensitivity by far more than any "
				"gain setting, so these bins are NOT comparable.");
	}

	shell_print(sh, "");
	shell_print(sh, "RSSI dBm         count");
	max = max_of(w.hist.rssi, MESHTASTIC_RF_RSSI_BINS);
	for (int i = MESHTASTIC_RF_RSSI_BINS - 1; i >= 0; i--) {
		if (i == MESHTASTIC_RF_RSSI_RAIL_BIN) {
			strcpy(label, ">= 0  RAIL");
		} else if (i == 0) {
			snprintk(label, sizeof(label), "< %d", MESHTASTIC_RF_RSSI_FLOOR);
		} else {
			int lo = MESHTASTIC_RF_RSSI_FLOOR + ((i - 1) * MESHTASTIC_RF_RSSI_STEP);

			snprintk(label, sizeof(label), "%d..%d", lo,
				 lo + MESHTASTIC_RF_RSSI_STEP);
		}
		if (w.hist.rssi[i] > 0U) {
			print_bar(sh, label, w.hist.rssi[i], max);
		}
	}
	shell_print(sh, "  mean %d.%d  min %d  max %d",
		    mean_x10(w.hist.rssi_sum, w.hist.frames) / 10,
		    tenth(mean_x10(w.hist.rssi_sum, w.hist.frames)), w.hist.rssi_min,
		    w.hist.rssi_max);

	shell_print(sh, "");
	shell_print(sh, "SNR dB           count");
	max = max_of(w.hist.snr, MESHTASTIC_RF_SNR_BINS);
	for (int i = MESHTASTIC_RF_SNR_BINS - 1; i >= 0; i--) {
		if (i == MESHTASTIC_RF_SNR_BINS - 1) {
			/* >=, not >: 18 two-dB bins from -22 reach +13, so +14 shares
			 * the overflow bin. See meshtastic_rf_snr_bin(). */
			strcpy(label, ">= +14");
		} else if (i == 0) {
			snprintk(label, sizeof(label), "< %d", MESHTASTIC_RF_SNR_FLOOR);
		} else {
			int lo = MESHTASTIC_RF_SNR_FLOOR + ((i - 1) * MESHTASTIC_RF_SNR_STEP);

			snprintk(label, sizeof(label), "%+d..%+d", lo,
				 lo + MESHTASTIC_RF_SNR_STEP);
		}
		if (w.hist.snr[i] > 0U) {
			print_bar(sh, label, w.hist.snr[i], max);
		}
	}
	shell_print(sh, "  mean %d.%d  min %d  max %d",
		    mean_x10(w.hist.snr_sum, w.hist.frames) / 10,
		    tenth(mean_x10(w.hist.snr_sum, w.hist.frames)), w.hist.snr_min,
		    w.hist.snr_max);

	/*
	 * The saturation caveat. A receiver pinned at the top of the scale cannot
	 * show an improvement from ANY gain change, so this is not a footnote —
	 * it decides whether the numbers above can answer the question at all.
	 */
	if (w.rail_frames > 0U) {
		uint32_t pct = (w.rail_frames * 100U) / w.hist.frames;

		shell_print(sh, "");
		shell_print(sh, "note: %u frames (%u%%) pinned the RSSI rail. Above ~10%% the "
				"receiver is saturated", w.rail_frames, pct);
		shell_print(sh, "      and no gain change can show an improvement — move the "
				"nodes apart.");
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_rf_hist)

static int cmd_rf_peers(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_rf_peer peers[CONFIG_MESHTASTIC_RF_PEERS];
	struct meshtastic_rf_window w;
	int n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_rf_window_get(&w);
	n = meshtastic_rf_peers_get(peers, ARRAY_SIZE(peers));
	if (n <= 0) {
		shell_print(sh, "no peers heard this window");
		return 0;
	}

	shell_print(sh, "window %u s", w.window_ms / 1000U);
	shell_print(sh, "node         direct  relayed   direct/h   mean SNR  best SNR  mean RSSI");
	for (int i = 0; i < n; i++) {
		const struct meshtastic_rf_peer *p = &peers[i];
		uint32_t span_s = (p->last_ms - p->first_ms) / 1000U;
		uint32_t per_h = (span_s > 0U) ? (p->frames_direct * 3600U) / span_s : 0U;
		int32_t msnr = mean_x10(p->snr_sum, p->frames_direct);
		int32_t mrssi = mean_x10(p->rssi_sum, p->frames_direct);

		/*
		 * Only direct frames carry a mean, and the rate column blanks when
		 * the span is too short to divide by. A rate printed from two frames
		 * seconds apart is a fabricated number.
		 */
		if (p->frames_direct == 0U) {
			shell_print(sh, "0x%08x  %6u  %7u        —          —         —          —",
				    p->num, p->frames_direct, p->frames_relayed);
			continue;
		}
		shell_print(sh, "0x%08x  %6u  %7u   %8u   %5d.%d     %+5d   %5d.%d", p->num,
			    p->frames_direct, p->frames_relayed, per_h, msnr / 10,
			    tenth(msnr), p->snr_best, mrssi / 10, tenth(mrssi));
	}

	shell_print(sh, "%d peer%s (table holds %d)", n, (n == 1) ? "" : "s",
		    CONFIG_MESHTASTIC_RF_PEERS);
	if (w.evicted > 0U) {
		shell_print(sh, "%u peer%s evicted — this list is PARTIAL; raise "
				"CONFIG_MESHTASTIC_RF_PEERS", w.evicted,
			    (w.evicted == 1U) ? " was" : "s were");
	}
	shell_print(sh, "relayed frames measure the RELAY's link to us, not this node's — "
			"they are counted but never averaged.");

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_rf_peers)

static int cmd_rf_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_rf_reset();
	shell_print(sh, "measurement window restarted (lifetime count kept)");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_rf_reset)
#endif /* CONFIG_MESHTASTIC_RF_HIST */

#if defined(CONFIG_MESHTASTIC_RF_HIST)
SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_rf_cmds,
	SHELL_CMD(hist, NULL, SHELL_HELP("RSSI/SNR distribution for this window.", NULL),
		  cmd_rf_hist_on_work),
	SHELL_CMD(peers, NULL, SHELL_HELP("Per-peer frame counts and link quality.", NULL),
		  cmd_rf_peers_on_work),
	SHELL_CMD(reset, NULL, SHELL_HELP("Start a new measurement window.", NULL),
		  cmd_rf_reset_on_work),
	SHELL_SUBCMD_SET_END);
#endif

#if defined(CONFIG_MESHTASTIC_RF_PATH_REPORT) && defined(CONFIG_MESHTASTIC_RF_HIST)
/* Bare `meshtastic rf` runs the gain-path report; the subcommands cover
 * measurement. Both halves are independently configurable, hence three
 * cases rather than one. */
SHELL_SUBCMD_ADD((meshtastic), rf, &meshtastic_rf_cmds,
		 SHELL_HELP("Gain path in signal order, plus signal measurement.", NULL),
		 cmd_rf_path_on_work, 0, 0);
#elif defined(CONFIG_MESHTASTIC_RF_PATH_REPORT)
SHELL_SUBCMD_ADD((meshtastic), rf, NULL,
		 SHELL_HELP("Gain path in signal order: what the radio is REALLY doing.", NULL),
		 cmd_rf_path_on_work, 0, 0);
#elif defined(CONFIG_MESHTASTIC_RF_HIST)
SHELL_SUBCMD_ADD((meshtastic), rf, &meshtastic_rf_cmds,
		 SHELL_HELP("Signal measurement: distributions and per-peer rates.", NULL),
		 NULL, 0, 0);
#endif


