/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic scan`: the multi-preset RX-only survey. Built with the shell and
 * CONFIG_MESHTASTIC_SCANNER (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

#include "meshtastic_packet.h"
#include "meshtastic_scanner.h"

static int cmd_scan_start(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_scanner_start();
	if (ret == -EALREADY) {
		shell_warn(sh, "already scanning");
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "scan start failed: %d", ret);
		return ret;
	}

	shell_print(sh, "scanning — this node is OFF its channel and cannot transmit");
	shell_print(sh, "run 'meshtastic scan stop' to rejoin the mesh");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_start)

static int cmd_scan_stop(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_scanner_stop();
	if (ret == -EALREADY) {
		shell_warn(sh, "not scanning");
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "scan stop FAILED: %d — radio is not back on the operating "
				"preset and TX stays disabled", ret);
		return ret;
	}

	shell_print(sh, "rejoined: back on the operating preset, TX re-enabled");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_stop)

static int cmd_scan_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_scan_stats st[16];
	uint32_t total;
	uint32_t blocked;
	int n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	n = meshtastic_scanner_stats(st, ARRAY_SIZE(st));
	if (n < 0) {
		shell_error(sh, "stats failed: %d", n);
		return n;
	}
	total = meshtastic_scanner_total();
	blocked = meshtastic_scanner_tx_blocked();

	{
		bool sweeping = meshtastic_scanner_sweeping();
		bool shut = meshtastic_scanner_active();

		/* Shut-but-not-sweeping is a real state on an autostart image — the gate
		 * closes before main() while the sweep starts at the end of init — so
		 * report it as itself rather than as "scanning", which would be a lie
		 * during the boot window and, if the sweep failed to start, a permanent
		 * one. */
		shell_print(sh, "sweeping: %s   tx: %s", sweeping ? "yes" : "no",
			    !shut	 ? "allowed"
			    : sweeping	 ? "REFUSED (scanning)"
					 : "REFUSED (gate shut, sweep not running)");
	}
	shell_print(sh, "captured: %u total   withheld from stack: %u", total,
		    meshtastic_scanner_rx_dropped());
	if (blocked > 0U) {
		/* Not cosmetic: a non-zero count means some path kept trying to
		 * transmit while parked on a foreign frequency. The gate held, but
		 * something upstream should not have been asking. */
		uint32_t bdest;
		uint8_t bchan;
		uint16_t blen;

		shell_warn(sh, "tx refused while scanning: %u (expected 0 — investigate)",
			   blocked);
		/* Say WHAT, not just how many. The portnum lives in the encrypted
		 * payload and is invisible at the TX choke point, but dest and the
		 * channel hash name the subsystem well enough to go looking. */
		if (meshtastic_scanner_tx_blocked_first(&bdest, &bchan, &blen)) {
			shell_warn(sh, "  first: to 0x%08x ch 0x%02x len %u%s", bdest, bchan,
				   (unsigned int)blen,
				   bdest == 0xffffffffU ? " (a broadcast — a beacon or a digest)"
							: " (a unicast — a reply or a walk)");
		}
	}
	shell_print(sh, "");
	shell_print(sh, "%-12s %11s %8s %8s %10s", "preset", "freq", "heard", "visits", "listen_s");
	for (int i = 0; i < n; i++) {
		shell_print(sh, "%-12s %8u.%03u %8u %8u %10u",
			    meshtastic_preset_display_name(st[i].preset, true),
			    st[i].frequency_hz / 1000000U, (st[i].frequency_hz / 1000U) % 1000U,
			    st[i].heard, st[i].visits, st[i].dwell_ms_total / 1000U);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_status)

static int cmd_scan_dump(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_scan_record rec[8];
	uint32_t from = 0U;
	uint32_t want = 32U;
	uint32_t done = 0U;

	if (argc > 1) {
		want = (uint32_t)strtoul(argv[1], NULL, 0);
	}

	/*
	 * `uptime` and `id` are what turn a survey into an instrument, so they lead.
	 * uptime_ms is what the record actually stores and is always meaningful;
	 * epoch_ms is DERIVED here and is 0 when the clock has never been seeded.
	 *
	 * Both columns, deliberately. Printing only the derived epoch is how the old
	 * absolute-stamp arrangement made a clockless node's capture look like a
	 * capture of 1970 instead of an untimed one; printing only uptime would lose
	 * the correlation to anything off-node. With both, "no clock" and "clock says
	 * X" are distinguishable at a glance.
	 *
	 * The anchor is snapshotted ONCE before the loop. Resolving per row would let
	 * a seed landing mid-dump split the output across two different anchors, so
	 * rows would disagree with each other about when the same second was.
	 */
	int64_t anchor_ms = meshtastic_clock_uptime_ms_to_epoch_ms(0);

	shell_print(sh, "%-12s %-14s %-10s %-12s %-10s %-10s %5s %4s %4s %5s %5s %5s %4s",
		    "uptime_ms", "epoch_ms", "id", "preset", "from", "to", "rssi", "snr", "hops",
		    "strt", "relay", "chan", "len");

	while (done < want) {
		int n = meshtastic_scanner_records(rec, ARRAY_SIZE(rec), from);

		if (n <= 0) {
			break;
		}
		for (int i = 0; i < n && done < want; i++, done++) {
			shell_print(sh,
				    "%-12u %-14lld 0x%08x %-12s 0x%08x 0x%08x %5d %4d %4u %5u  0x%02x  0x%02x %4u",
				    rec[i].uptime_ms,
				    anchor_ms ? (long long)(anchor_ms + rec[i].uptime_ms) : 0LL,
				    rec[i].id,
				    meshtastic_preset_display_name(
					    (meshtastic_Config_LoRaConfig_ModemPreset)rec[i].preset,
					    true),
				    rec[i].from, rec[i].to, rec[i].rssi, rec[i].snr,
				    (unsigned int)(rec[i].flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK),
				    (unsigned int)((rec[i].flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
						   MESHTASTIC_FLAGS_HOP_START_SHIFT),
				    rec[i].relay_node, rec[i].chan_hash, rec[i].payload_len);
		}
		from += (uint32_t)n;
	}

	shell_print(sh, "(%u shown of %u captured)", done, meshtastic_scanner_total());
	if (anchor_ms == 0) {
		shell_print(sh, "clock UNSEEDED — epoch_ms is 0, not 1970. Deltas are still exact; "
				"`meshtastic time set` now re-dates the whole ring retroactively.");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_dump)

static int cmd_scan_presets(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config_LoRaConfig_ModemPreset list[MESHTASTIC_SCANNER_MAX_PRESETS];
	int n;

	if (argc == 1U) {
		n = meshtastic_scanner_get_presets(list, ARRAY_SIZE(list));
		if (n < 0) {
			shell_error(sh, "get failed: %d", n);
			return n;
		}
		shell_fprintf(sh, SHELL_NORMAL, "sweeping %d preset%s:", n, n == 1 ? "" : "s");
		for (int i = 0; i < n; i++) {
			shell_fprintf(sh, SHELL_NORMAL, " %s",
				      meshtastic_preset_display_name(list[i], true));
		}
		shell_print(sh, "");
		shell_print(sh, "('scan presets all' restores the full set)");
		return 0;
	}

	if (argc == 2U && strcmp(argv[1], "all") == 0) {
		(void)meshtastic_scanner_pin_presets(NULL, 0U);
		shell_print(sh, "sweeping the full preset set (stats cleared; nothing stored: "
				"the next boot uses the build's list)");
		return 0;
	}

	if (argc - 1U > ARRAY_SIZE(list)) {
		shell_error(sh, "at most %u presets", (unsigned int)ARRAY_SIZE(list));
		return -EINVAL;
	}

	for (size_t i = 1; i < argc; i++) {
		if (mtsh_parse_modem_preset(argv[i], &list[i - 1U]) < 0) {
			shell_error(sh, "unknown preset '%s'", argv[i]);
			return -EINVAL;
		}
	}

	if (meshtastic_scanner_pin_presets(list, argc - 1U) < 0) {
		shell_error(sh, "set failed");
		return -EINVAL;
	}

	/* Narrowing the list shortens the cycle, so each remaining preset is listened
	 * to a larger fraction of the time — say so, since that is the point. */
	shell_print(sh, "sweeping %u presets (stats cleared; shorter cycle = higher capture "
			"rate on each)", (unsigned int)(argc - 1U));
	if (IS_ENABLED(CONFIG_MESHTASTIC_SETTINGS)) {
		shell_print(sh, "stored: a reboot comes back to this list ('scan presets all' "
				"forgets it)");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_presets)

static int cmd_scan_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_scanner_reset();
	shell_print(sh, "scan records and stats cleared");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_scan_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_scan_cmds,
	SHELL_CMD(start, NULL,
		  SHELL_HELP("Begin sweeping presets. Leaves the mesh; TX is refused.", NULL),
		  cmd_scan_start_on_work),
	SHELL_CMD(stop, NULL, SHELL_HELP("Stop sweeping and rejoin the mesh.", NULL),
		  cmd_scan_stop_on_work),
	SHELL_CMD(status, NULL, SHELL_HELP("Per-preset capture counts.", NULL), cmd_scan_status_on_work),
	SHELL_CMD(dump, NULL, SHELL_HELP("Print captured headers.", "[count]"), cmd_scan_dump_on_work),
	SHELL_CMD(presets, NULL,
		  SHELL_HELP("Show or limit which presets are swept.",
			     "[all | <name> ...]  e.g. scan presets LongFast ShortTurbo"),
		  cmd_scan_presets_on_work),
	SHELL_CMD(reset, NULL, SHELL_HELP("Clear captured records and stats.", NULL),
		  cmd_scan_reset_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), scan, &meshtastic_scan_cmds,
		 SHELL_HELP("Multi-preset survey. Leaves the mesh while running.", NULL),
		 NULL, 0, 0);
