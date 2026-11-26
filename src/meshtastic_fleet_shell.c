/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic fleet`: the fleet firmware intent (what each class should run). Built with
 * the shell and CONFIG_MESHTASTIC_FLEET (src/CMakeLists.txt); moved from
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
#include <zephyr/meshtastic/diagnostics.h>

#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_core.h"
#include "meshtastic_fleet.h"
#include "meshtastic_shell_internal.h"
#if defined(CONFIG_MESHTASTIC_DEPOT)
#include "meshtastic_smp_central.h"
#endif
#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
#include <zephyr/bluetooth/addr.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#endif

enum shell_fleet_verb {
	SHELL_FLEET_DESIRE,
	SHELL_FLEET_PIN,
	SHELL_FLEET_UNPIN,
};

/* What the shell thread parsed; the work thread publishes it and prints the answer. */
struct shell_fleet_work {
	uint8_t verb; /* enum shell_fleet_verb */
	uint8_t class_id;
	bool has_hash;
	uint8_t hash[4];
	uint16_t hw_model;
	uint32_t node;
	uint32_t version;
	uint32_t flags;
	char version_str[16]; /* as typed, for the confirmation line */
};

/* `fleet desire` / `pin` / `unpin`. The publish copies an entry, encodes the intent,
 * signs the cluster entry and stores it; on the XIAO's 2048-byte shell stack `fleet
 * desire` overflowed it every time (bench 2026-11-24, rzr7, agents-vnfu). */
static int shell_fleet_work_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_fleet_work *fw = arg;
	const uint8_t *hash = fw->has_hash ? fw->hash : NULL;
	int ret;

	ARG_UNUSED(len);

	switch (fw->verb) {
	case SHELL_FLEET_DESIRE:
		ret = meshtastic_fleet_desire(fw->class_id, fw->version, hash, fw->flags,
					      fw->hw_model);
		if (ret == -EPERM) {
			shell_error(sh, "refused: this node may not write the document (managed?)");
		} else if (ret == -ENOSPC) {
			shell_error(sh, "refused: four classes already have rows");
		} else if (ret == 0) {
			shell_print(sh, "published: class %u -> %s%s", fw->class_id,
				    fw->version_str, fw->flags ? " (flagged)" : "");
		}
		return ret;
	case SHELL_FLEET_PIN:
	case SHELL_FLEET_UNPIN: {
		bool withdraw = (fw->verb == SHELL_FLEET_UNPIN);

		ret = meshtastic_fleet_pin(fw->node, withdraw ? 0U : fw->version, hash, fw->flags);
		if (ret == 0) {
			shell_print(sh, "%s 0x%08x%s%s", withdraw ? "withdrew the pin on" : "pinned",
				    fw->node, withdraw ? "" : " -> ",
				    withdraw ? "" : fw->version_str);
		}
		return ret;
	}
	default:
		return -EINVAL;
	}
}

BUILD_ASSERT(sizeof(struct shell_fleet_work) <= MTSH_WORK_ARG_MAX,
	     "the fleet verb's arguments ride in a shell work item");

/* ---- `meshtastic fleet` — firmware intent on the cluster document ---------- */

/* "maj.min.rev", each a decimal in the MCUboot header's range. strtoul rather
 * than sscanf: the shell file has no stdio, and scanf is not worth its flash. */
static int fleet_version_parse(const char *s, uint32_t *out)
{
	unsigned long part[3];
	char *end;

	for (int i = 0; i < 3; i++) {
		if (*s < '0' || *s > '9') {
			return -EINVAL;
		}
		part[i] = strtoul(s, &end, 10);
		if (i < 2) {
			if (*end != '.') {
				return -EINVAL;
			}
			s = end + 1;
		} else if (*end != '\0') {
			return -EINVAL;
		}
	}
	if (part[0] > 255UL || part[1] > 255UL || part[2] > 65535UL) {
		return -EINVAL;
	}
	*out = meshtastic_fleet_version_pack((uint8_t)part[0], (uint8_t)part[1],
					     (uint16_t)part[2]);
	return 0;
}

static int fleet_hash_parse(const char *hex, uint8_t out[4])
{
	if (strlen(hex) != 8U) {
		return -EINVAL;
	}
	for (int i = 0; i < 4; i++) {
		char pair[3] = {hex[2 * i], hex[2 * i + 1], '\0'};
		char *end;
		unsigned long b = strtoul(pair, &end, 16);

		if (*end != '\0' || pair[0] == '\0' || pair[1] == '\0') {
			return -EINVAL;
		}
		out[i] = (uint8_t)b;
	}
	return 0;
}

static void fleet_print_row(const struct shell *sh, const char *what, uint32_t node_id,
			    const struct meshtastic_fleet_intent *w)
{
	char hw[16] = "";

	if (w->hw_model != 0U) {
		(void)snprintf(hw, sizeof(hw), " hw=%u", (unsigned int)w->hw_model);
	}
	shell_print(sh, "%s%s%08x: class %u -> %u.%u.%u%s%s%s%s  (stamp %lld.%u by 0x%08x)", what,
		    node_id ? " 0x" : "", node_id, w->class_id, (unsigned int)(w->version >> 24),
		    (unsigned int)((w->version >> 16) & 0xFFU), (unsigned int)(w->version & 0xFFFFU),
		    hw, w->has_hash ? " hash=" : "", w->has_hash ? "yes" : "",
		    (w->flags & MESHTASTIC_FLEET_FLAG_PAUSE) ? " PAUSED" :
		    (w->flags & MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE) ? " allow-downgrade" : "",
		    (long long)w->stamp.physical_ms, w->stamp.counter, w->stamp.node_id);
}

/* fleet status: the base rows per class, then every pin in the document. */
static int cmd_fleet_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_cluster_entry e;
	struct meshtastic_fleet_intent w;
	unsigned int base_rows = 0U, pins = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (unsigned int c = 1U; c <= 255U; c++) {
		/* Ask for a node that cannot have a pin, so the answer is base. */
		if (meshtastic_fleet_desired_for(0U, (uint8_t)c, &w) && !w.pinned) {
			fleet_print_row(sh, "base   ", 0U, &w);
			base_rows++;
		}
	}
	for (uint16_t i = 0U; meshtastic_cluster_entry_get(i, &e); i++) {
		if (e.key.section != MESHTASTIC_CLUSTER_SECTION_FW ||
		    e.key.layer != MESHTASTIC_CLUSTER_LAYER_NODE) {
			continue;
		}
		if (e.tombstone) {
			shell_print(sh, "pin     0x%08x: withdrawn (tombstone)", e.key.node_id);
		} else if (meshtastic_fleet_desired_for(e.key.node_id, 0U, &w)) {
			fleet_print_row(sh, "pin    ", e.key.node_id, &w);
		}
		pins++;
	}
	shell_print(sh, "%u base row%s, %u pin%s; this build is class %u", base_rows,
		    base_rows == 1U ? "" : "s", pins, pins == 1U ? "" : "s",
		    CONFIG_MESHTASTIC_FLEET_CLASS);
	if (meshtastic_fleet_desired_for(meshtastic_get_node_id(), CONFIG_MESHTASTIC_FLEET_CLASS,
					 &w)) {
		fleet_print_row(sh, "me     ", 0U, &w);
	} else {
		shell_print(sh, "me      : no intent for my class — nobody will push to me");
	}
#if defined(CONFIG_MESHTASTIC_DEPOT)
	{
		struct meshtastic_smpc_depot_row d[MESHTASTIC_SMPC_DEPOT_ROWS];
		uint16_t skipped = 0U;
		uint16_t nd = meshtastic_smpc_depot_rows(d, ARRAY_SIZE(d), &skipped);

		if (skipped != 0U) {
			shell_print(sh, "depot   : %u image(s) — %u file(s) NOT indexed (over the %u-row cap, or not a signed image): prune /depot",
				    nd, skipped, (unsigned int)MESHTASTIC_SMPC_DEPOT_ROWS);
		} else {
			shell_print(sh, "depot   : %u image(s)", nd);
		}
		for (uint16_t i = 0U; i < nd; i++) {
			if (d[i].class_id == 0U) {
				shell_print(sh, "  %-24s class -  %u.%u.%u  %u B  (unclassed, offered to nobody)",
					    d[i].name, (unsigned int)(d[i].version >> 24),
					    (unsigned int)((d[i].version >> 16) & 0xFFU),
					    (unsigned int)(d[i].version & 0xFFFFU), d[i].size);
			} else {
				shell_print(sh, "  %-24s class %u  %u.%u.%u  %u B", d[i].name,
					    d[i].class_id, (unsigned int)(d[i].version >> 24),
					    (unsigned int)((d[i].version >> 16) & 0xFFU),
					    (unsigned int)(d[i].version & 0xFFFFU), d[i].size);
			}
		}
	}
#endif
#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
	{
		struct meshtastic_fleet_courier_row rows[MESHTASTIC_BLE_REG_SLOTS];
		uint16_t nr = meshtastic_fleet_courier_rows(rows, ARRAY_SIZE(rows));

		/* F6: the star. Every row is a neighbour the loop can serve; `->`
		 * marks the one meshtastic_fleet_pick() would push next (none while
		 * a job is in flight). Slots: BT_MAX_CONN, minus one for the host. */
		{
			uint32_t hold = meshtastic_fleet_courier_holdoff_s();
			char armed[48];

			/* A restored arm that is still inside its health window is
			 * ARMED and not pushing — say both, or `status` reads as a
			 * courier that is ignoring its orders. */
			if (meshtastic_fleet_courier_armed() && hold != 0U) {
				snprintf(armed, sizeof(armed),
					 "ARMED (restored — holding %u s)", hold);
			} else {
				strncpy(armed, meshtastic_fleet_courier_armed() ? "ARMED"
									       : "disarmed",
					sizeof(armed) - 1U);
				armed[sizeof(armed) - 1U] = '\0';
			}
			shell_print(sh, "courier : %s — %u row(s) of %u slot(s)", armed, nr,
				    (unsigned int)MESHTASTIC_BLE_REG_SLOTS);
		}
		for (uint16_t i = 0U; i < nr; i++) {
			shell_print(sh, "%s0x%08x class %u run %u.%u.%u want %u.%u.%u [%c%c%c] %s x%u",
				    rows[i].next ? "-> " : "   ", rows[i].node_id, rows[i].class_id,
				    (unsigned int)(rows[i].running >> 24),
				    (unsigned int)((rows[i].running >> 16) & 0xFFU),
				    (unsigned int)(rows[i].running & 0xFFFFU),
				    (unsigned int)(rows[i].desired >> 24),
				    (unsigned int)((rows[i].desired >> 16) & 0xFFU),
				    (unsigned int)(rows[i].desired & 0xFFFFU),
				    (rows[i].flags & MESHTASTIC_BLE_PEER_FLAG_HOLD) ? 'H' : '-',
				    (rows[i].flags & MESHTASTIC_BLE_PEER_FLAG_TESTBOOT) ? 'T' : '-',
				    (rows[i].flags & MESHTASTIC_BLE_PEER_FLAG_COURIER) ? 'C' : '-',
				    rows[i].state, rows[i].attempts);
		}
	}
#endif
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_fleet_status)

#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
static int cmd_fleet_arm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc >= 2) {
		if (strcmp(argv[1], "on") == 0) {
			meshtastic_fleet_courier_arm(true);
		} else if (strcmp(argv[1], "off") == 0) {
			meshtastic_fleet_courier_arm(false);
		} else {
			shell_error(sh, "usage: fleet arm [on|off]");
			return -EINVAL;
		}
	}
	{
		uint32_t hold = meshtastic_fleet_courier_holdoff_s();

		if (meshtastic_fleet_courier_armed() && hold != 0U) {
			shell_print(sh, "courier: ARMED (restored — holding pushes %u s)", hold);
		} else {
			shell_print(sh, "courier: %s",
				    meshtastic_fleet_courier_armed() ? "ARMED" : "disarmed");
		}
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_fleet_arm)

static int cmd_fleet_clear(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node;

	if (argc < 2) {
		shell_error(sh, "usage: fleet clear <node-hex>");
		return -EINVAL;
	}
	node = (uint32_t)strtoul(argv[1], NULL, 16);
	meshtastic_fleet_courier_clear(node);
	shell_print(sh, "cleared any reverted/backoff latch on 0x%08x", node);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_fleet_clear)
#endif /* CONFIG_MESHTASTIC_FLEET_COURIER */

static void fleet_version_copy(struct shell_fleet_work *fw, const char *v)
{
	strncpy(fw->version_str, v, sizeof(fw->version_str) - 1U);
	fw->version_str[sizeof(fw->version_str) - 1U] = '\0';
}

/* Parses on the shell thread (cheap, and a bad argument is answered at once); the publish
 * runs on the work thread (agents-vnfu: it overflowed the XIAO's shell stack). */
static int cmd_fleet_desire(const struct shell *sh, size_t argc, char **argv)
{
	struct shell_fleet_work work = {0};
	struct shell_fleet_work *fw = &work;
	unsigned long cls;

	if (argc < 3U) {
		shell_error(sh, "usage: fleet desire <class> <maj.min.rev> [hash8hex] [pause|downgrade] [hw=<model>]");
		return -EINVAL;
	}
	cls = strtoul(argv[1], NULL, 0);
	if (cls == 0UL || cls > 255UL || fleet_version_parse(argv[2], &fw->version) != 0) {
		shell_error(sh, "class 1..255 and a version maj.min.rev are required");
		return -EINVAL;
	}
	fw->verb = SHELL_FLEET_DESIRE;
	fw->class_id = (uint8_t)cls;
	fleet_version_copy(fw, argv[2]);
	for (size_t i = 3U; i < argc; i++) {
		if (strcmp(argv[i], "pause") == 0) {
			fw->flags |= MESHTASTIC_FLEET_FLAG_PAUSE;
		} else if (strcmp(argv[i], "downgrade") == 0) {
			fw->flags |= MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE;
		} else if (strncmp(argv[i], "hw=", 3) == 0) {
			unsigned long m = strtoul(argv[i] + 3, NULL, 0);

			if (m == 0UL || m > 65535UL) {
				shell_error(sh, "hw=<meshtastic HardwareModel number>, e.g. hw=88");
				return -EINVAL;
			}
			fw->hw_model = (uint16_t)m;
		} else if (fleet_hash_parse(argv[i], fw->hash) == 0) {
			fw->has_hash = true;
		} else {
			shell_error(sh, "unknown argument '%s'", argv[i]);
			return -EINVAL;
		}
	}
	return mtsh_enqueue(sh, shell_fleet_work_run, fw, sizeof(*fw));
}
MTSH_DEFINE_ON_WORK(cmd_fleet_desire)

static int cmd_fleet_pin(const struct shell *sh, size_t argc, char **argv)
{
	struct shell_fleet_work work = {0};
	struct shell_fleet_work *fw = &work;
	bool withdraw = (strcmp(argv[0], "unpin") == 0);

	if (argc < (withdraw ? 2U : 3U)) {
		shell_error(sh, "usage: fleet pin <node-hex> <maj.min.rev> [hash8hex] [downgrade] | "
				"fleet unpin <node-hex>");
		return -EINVAL;
	}
	fw->verb = withdraw ? SHELL_FLEET_UNPIN : SHELL_FLEET_PIN;
	fw->node = (uint32_t)strtoul(argv[1], NULL, 16);
	if (fw->node == 0U) {
		shell_error(sh, "node id required (hex)");
		return -EINVAL;
	}
	if (!withdraw) {
		if (fleet_version_parse(argv[2], &fw->version) != 0) {
			shell_error(sh, "version maj.min.rev required");
			return -EINVAL;
		}
		fleet_version_copy(fw, argv[2]);
		for (size_t i = 3U; i < argc; i++) {
			if (strcmp(argv[i], "downgrade") == 0) {
				fw->flags |= MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE;
			} else if (fleet_hash_parse(argv[i], fw->hash) == 0) {
				fw->has_hash = true;
			} else {
				shell_error(sh, "unknown argument '%s'", argv[i]);
				return -EINVAL;
			}
		}
	}
	return mtsh_enqueue(sh, shell_fleet_work_run, fw, sizeof(*fw));
}
MTSH_DEFINE_ON_WORK(cmd_fleet_pin)

#if defined(CONFIG_MESHTASTIC_DEPOT)
static int cmd_fleet_rescan(const struct shell *sh, size_t argc, char **argv)
{
	int rc = meshtastic_smpc_depot_rescan();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (rc < 0) {
		shell_error(sh, "depot rescan failed (%d)", rc);
		return rc;
	}
	shell_print(sh, "depot: %d image(s)", rc);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_fleet_rescan)
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_fleet_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Base rows per class, pins, and what I should run.", NULL),
		  cmd_fleet_status_on_work),
	SHELL_CMD(desire, NULL,
		  SHELL_HELP("Publish the fleet's intent for an image class (master only).",
			     "<class> <maj.min.rev> [hash8hex] [pause|downgrade] [hw=<model>]"),
		  cmd_fleet_desire_on_work),
	SHELL_CMD(pin, NULL, SHELL_HELP("Pin one node to a version.",
					"<node-hex> <maj.min.rev> [hash8hex] [downgrade]"),
		  cmd_fleet_pin_on_work),
	SHELL_CMD(unpin, NULL, SHELL_HELP("Withdraw a node's pin (it falls back to base).",
					  "<node-hex>"),
		  cmd_fleet_pin_on_work),
#if defined(CONFIG_MESHTASTIC_DEPOT)
	SHELL_CMD(rescan, NULL, SHELL_HELP("Re-read /depot (after loading cargo).", NULL),
		  cmd_fleet_rescan_on_work),
#endif
#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
	SHELL_CMD(arm, NULL, SHELL_HELP("Arm/disarm the courier loop.", "[on|off]"), cmd_fleet_arm_on_work),
	SHELL_CMD(clear, NULL, SHELL_HELP("Clear a node's reverted/refused/backoff latch.", "<node-hex>"),
		  cmd_fleet_clear_on_work),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), fleet, &meshtastic_fleet_cmds,
		 SHELL_HELP("Fleet firmware intent (what each class should run).", NULL),
		 cmd_fleet_status_on_work, 0, 0);
