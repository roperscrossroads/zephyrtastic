/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic blepeer`: the node-to-node BLE peer link. Built with the shell and
 * CONFIG_MESHTASTIC_BLE_PEER (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/bluetooth/addr.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* ---- node-to-node BLE peer link (a4it.6) ---- */

static const char *blepeer_kind_str(enum meshtastic_ble_conn_kind kind)
{
	switch (kind) {
	case MESHTASTIC_BLE_CONN_UNCLASSIFIED:
		return "unclassified";
	case MESHTASTIC_BLE_CONN_PHONE:
		return "phone";
	case MESHTASTIC_BLE_CONN_PEER:
		return "peer";
	default:
		return "free";
	}
}

static int cmd_blepeer_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_ble_reg_stats rs;
	struct meshtastic_ble_peer_stats ps;
	struct meshtastic_ble_peer_link link;
	unsigned int active;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_ble_reg_stats(&rs);
	meshtastic_ble_peer_stats_get(&ps);
	meshtastic_ble_peer_link_get(&link);
	active = meshtastic_ble_reg_active();

	shell_print(sh, "conn slots    : %u/%u used (%u free), advertising %s (starts %u)",
		    active, CONFIG_BT_MAX_CONN, CONFIG_BT_MAX_CONN - active,
		    meshtastic_ble_adv_active() ? "yes" : "NO", meshtastic_ble_adv_starts());
	shell_print(sh, "config        : classify window %u ms, beat period %u ms",
		    CONFIG_MESHTASTIC_BLE_PEER_CLASSIFY_WINDOW_MS,
		    CONFIG_MESHTASTIC_BLE_PEER_BEAT_PERIOD_MS);
	/* THE bench line: notes and unnotes converge to equal whenever no
	 * phone is connected — the direct read-out of the PM-inhibit ledger. */
	shell_print(sh, "PM ledger     : phone_notes=%u phone_unnotes=%u (in-flight %u)",
		    rs.phone_notes, rs.phone_unnotes, rs.phone_notes - rs.phone_unnotes);
	shell_print(sh, "classified    : peer/role=%u peer/hello=%u phone/traffic=%u phone/default=%u",
		    rs.classified_peer_by_role, rs.classified_peer_by_hello,
		    rs.classified_phone_by_traffic, rs.classified_phone_default);
	if (rs.slot_index_out_of_range != 0U) {
		shell_print(sh, "[!!] slot_index_out_of_range=%u — bt_conn_index invariant broken",
			    rs.slot_index_out_of_range);
	}
	shell_print(sh, "beats         : hello_malformed=%u hello_rejected_late=%u tx notify=%u write=%u",
		    ps.hello_malformed, ps.hello_rejected_late, ps.notify_tx_beats,
		    ps.write_tx_beats);
	shell_print(sh, "frames        : rx=%u rejected=%u tx=%u tx_failed=%u notify=%s",
		    ps.frame_rx_frames, ps.frame_rx_rejected, ps.frame_tx_frames,
		    ps.frame_tx_failed,
		    meshtastic_ble_peer_frame_notify_ready() ? "on" : "off");
	if (meshtastic_ble_peer_scan_armed() && meshtastic_ble_peer_scan_target() != 0U) {
		shell_print(sh, "scan          : armed (target 0x%08x), adverts matched=%u "
			    "reflections=%u", meshtastic_ble_peer_scan_target(),
			    ps.adverts_matched, ps.reflections);
	} else if (meshtastic_ble_peer_last_node() != 0U) {
		shell_print(sh, "scan          : %s (any peer, prefers last 0x%08x for %u s; deferred %u), adverts matched=%u reflections=%u",
			    meshtastic_ble_peer_scan_armed() ? "armed" : "off",
			    meshtastic_ble_peer_last_node(),
			    CONFIG_MESHTASTIC_BLE_PEER_STICKY_MS / 1000U, ps.sticky_deferred,
			    ps.adverts_matched, ps.reflections);
	} else {
		shell_print(sh, "scan          : %s, adverts matched=%u reflections=%u",
			    meshtastic_ble_peer_scan_armed() ? "armed" : "off",
			    ps.adverts_matched, ps.reflections);
	}
	shell_print(sh, "connects      : attempted=%u failed=%u discovery_failures=%u",
		    ps.connects_attempted, ps.connects_failed, ps.discovery_failures);
	shell_print(sh, "retries       : scan_start_failures=%u last_scan_err=%d "
			"already_linked=%u bringup_timeouts=%u",
		    ps.scan_start_failures, meshtastic_ble_peer_last_scan_err(),
		    ps.already_linked, ps.bringup_timeouts);
	if (link.connected) {
		shell_print(sh, "outbound      : 0x%08x %s (slot %u, tx beats %u, frames %s)",
			    link.node_num, link.ready ? "READY" : "connecting", link.index,
			    link.tx_beats, link.frame_ready ? "ready" : "no");
	} else {
		shell_print(sh, "outbound      : none");
	}

	for (unsigned int i = 0U; i < CONFIG_BT_MAX_CONN; i++) {
		enum meshtastic_ble_conn_kind kind = meshtastic_ble_reg_kind(i);
		struct meshtastic_ble_peer_rx rx;
		char addr[BT_ADDR_LE_STR_LEN] = "?";
		int64_t age_ms = 0;
		int64_t last_ms = 0;

		if (kind == MESHTASTIC_BLE_CONN_NONE) {
			continue;
		}
		(void)meshtastic_ble_slot_info(i, addr, sizeof(addr), &age_ms);
		if (meshtastic_ble_peer_rx_get(i, &rx, &last_ms)) {
			/* The version-2 columns: what the neighbour runs and whether
			 * it may be pushed to. A version-1 sender shows v1 and dashes. */
			char flags[5] = {
				(rx.last.flags & MESHTASTIC_BLE_PEER_FLAG_HOLD) ? 'H' : '-',
				(rx.last.flags & MESHTASTIC_BLE_PEER_FLAG_TESTBOOT) ? 'T' : '-',
				(rx.last.flags & MESHTASTIC_BLE_PEER_FLAG_COURIER) ? 'C' : '-',
				'\0', '\0'};

			shell_print(sh,
				    "slot %u: %-12s %s age=%llds rx beats=%u lost=%u resyncs=%u "
				    "from=0x%08x last=%llds ago",
				    i, blepeer_kind_str(kind), addr,
				    (long long)(age_ms / 1000), rx.beats, rx.lost, rx.resyncs,
				    rx.last.node_num,
				    (long long)((k_uptime_get() - last_ms) / 1000));
			if (rx.last.version >= 2U) {
				shell_print(sh, "        beat v%u fw=%u.%u.%u class=%u flags=%s",
					    rx.last.version, rx.last.fw_major, rx.last.fw_minor,
					    rx.last.fw_revision, rx.last.class_id, flags);
			} else {
				shell_print(sh, "        beat v%u fw=? class=? flags=? (pre-fleet sender)",
					    rx.last.version);
			}
		} else {
			shell_print(sh, "slot %u: %-12s %s age=%llds%s", i, blepeer_kind_str(kind),
				    addr, (long long)(age_ms / 1000),
				    meshtastic_ble_reg_phone_noted(i) ? " (phone note held)" : "");
		}
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_status)

static int cmd_blepeer_scan(const struct shell *sh, size_t argc, char **argv)
{
	int err;
	bool on;

	if (argc < 2) {
		shell_print(sh, "peer scan %s", meshtastic_ble_peer_scan_armed() ? "armed" : "off");
		return 0;
	}
	if (strcmp(argv[1], "on") == 0) {
		on = true;
	} else if (strcmp(argv[1], "off") == 0) {
		on = false;
	} else {
		shell_error(sh, "usage: blepeer scan [on|off]");
		return -EINVAL;
	}

	err = meshtastic_ble_peer_scan_set(on);
	if (err != 0) {
		shell_error(sh, "scan %s failed (%d)", argv[1], err);
		return err;
	}
	shell_print(sh, "peer scan %s", on ? "armed" : "off");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_scan)

static int cmd_blepeer_list(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_ble_peer_seen seen;
	unsigned int n = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (unsigned int i = 0U; meshtastic_ble_peer_seen_get(i, &seen); i++) {
		shell_print(sh, "0x%08x  rssi=%-4d last=%llds ago", seen.node_num, seen.rssi,
			    (long long)((k_uptime_get() - seen.last_ms) / 1000));
		n++;
	}
	if (n == 0U) {
		shell_print(sh, "no peer adverts seen%s",
			    meshtastic_ble_peer_scan_armed() ? "" : " (scan is off)");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_list)

static int cmd_blepeer_connect(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = 0U;
	int err;

	if (argc >= 2) {
		node = (uint32_t)strtoul(argv[1], NULL, 16);
		if (node == 0U) {
			shell_error(sh, "bad node number (hex expected)");
			return -EINVAL;
		}
	}

	err = meshtastic_ble_peer_connect(node);
	if (err == -EALREADY) {
		shell_error(sh, "already connected — blepeer disconnect first");
		return err;
	}
	if (err != 0) {
		shell_error(sh, "connect failed (%d)", err);
		return err;
	}
	if (node != 0U) {
		shell_print(sh, "scanning for 0x%08x...", node);
	} else {
		shell_print(sh, "scanning for any peer...");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_connect)

static int cmd_blepeer_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = meshtastic_ble_peer_disconnect();
	if (err == -ENOTCONN) {
		shell_print(sh, "no outbound peer link");
		return 0;
	}
	if (err != 0) {
		shell_error(sh, "disconnect failed (%d)", err);
		return err;
	}
	shell_print(sh, "disconnecting");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_disconnect)

static int cmd_blepeer_hold(const struct shell *sh, size_t argc, char **argv)
{
	if (argc >= 2) {
		if (strcmp(argv[1], "on") == 0) {
			meshtastic_ble_peer_hold_set(true);
		} else if (strcmp(argv[1], "off") == 0) {
			meshtastic_ble_peer_hold_set(false);
		} else {
			shell_error(sh, "usage: blepeer hold [on|off]");
			return -EINVAL;
		}
	}
	shell_print(sh, "hold: %s (HOLD %s in this node's beats)",
		    meshtastic_ble_peer_hold_get() ? "on" : "off",
		    meshtastic_ble_peer_hold_get() ? "set" : "clear");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_hold)

static int cmd_blepeer_beat(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_ble_peer_beat_now();
	shell_print(sh, "beat scheduled on every active link (none = no-op; see blepeer status)");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_blepeer_beat)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_blepeer_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Slots, ledger, classification and link counters.", NULL),
		  cmd_blepeer_status_on_work),
	SHELL_CMD(scan, NULL, SHELL_HELP("Arm/disarm the passive peer scanner.", "[on|off]"),
		  cmd_blepeer_scan_on_work),
	SHELL_CMD(list, NULL, SHELL_HELP("Peers heard advertising.", NULL), cmd_blepeer_list_on_work),
	SHELL_CMD(connect, NULL,
		  SHELL_HELP("Scan for and connect to a peer.", "[node-num-hex]"),
		  cmd_blepeer_connect_on_work),
	SHELL_CMD(disconnect, NULL, SHELL_HELP("Drop the outbound peer link.", NULL),
		  cmd_blepeer_disconnect_on_work),
	SHELL_CMD(beat, NULL, SHELL_HELP("Send one heartbeat now on every active link.", NULL),
		  cmd_blepeer_beat_on_work),
	SHELL_CMD(hold, NULL,
		  SHELL_HELP("Show or set HOLD (\"do not push firmware to me\") in this node's beats.",
			     "[on|off]"),
		  cmd_blepeer_hold_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), blepeer, &meshtastic_blepeer_cmds,
		 SHELL_HELP("Node-to-node BLE peer link.", NULL),
		 cmd_blepeer_status_on_work, 0, 0);
