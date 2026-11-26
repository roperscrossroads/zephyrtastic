/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic nodedb`: the NodeDB. Built with the shell and
 * CONFIG_MESHTASTIC_NODEDB (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

static void shell_print_node_summary(const struct shell *sh,
				     const struct meshtastic_nodedb_node *node)
{
	int32_t snr = mtsh_scaled_tenths(node->snr);
	const char *long_name = node->has_user ? node->long_name : "";
	const char *short_name = node->has_user ? node->short_name : "";
	char heard[24];

	/*
	 * last_heard_uptime_sec is the UPTIME AT WHICH we last heard the node,
	 * not an age — and zero means "not this boot" (the load path resets it
	 * deliberately, resolving age from the persisted epoch instead). Printing
	 * the raw number after "last=" invites exactly the opposite reading, and
	 * did: on 2026-08-25 a stale entry showing `last=0s snr=13.0` was read as
	 * "heard just now, strong signal" when the node had not been heard at all
	 * and both numbers were leftovers from a previous boot.
	 *
	 * So say it in the units a reader assumes: an age, or "never".
	 */
	if (node->last_heard_uptime_sec == 0U) {
		strncpy(heard, "never-this-boot", sizeof(heard) - 1U);
		heard[sizeof(heard) - 1U] = '\0';
	} else {
		uint32_t now = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
		uint32_t age = (now > node->last_heard_uptime_sec)
				       ? (now - node->last_heard_uptime_sec)
				       : 0U;

		snprintk(heard, sizeof(heard), "%us ago", age);
	}

	if (node->has_hops_away) {
		shell_print(sh, "0x%08x last=%s snr=%d.%u hops=%u via=%s long=\"%s\" short=\"%s\"",
			    node->num, heard, mtsh_scaled_whole(snr, 10), mtsh_scaled_fraction(snr, 10),
			    node->hops_away, node->via_mqtt ? "yes" : "no", long_name,
			    short_name);
	} else {
		shell_print(sh, "0x%08x last=%s snr=%d.%u hops=? via=%s long=\"%s\" short=\"%s\"",
			    node->num, heard, mtsh_scaled_whole(snr, 10), mtsh_scaled_fraction(snr, 10),
			    node->via_mqtt ? "yes" : "no", long_name, short_name);
	}
}

static int cmd_nodedb_list(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_nodedb_node node;
	size_t count;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	count = meshtastic_nodedb_count();
	shell_print(sh, "nodes: %u", (unsigned int)count);

	for (size_t i = 0U; i < count; i++) {
		ret = meshtastic_nodedb_get_by_index(i, &node);
		if (ret < 0) {
			shell_warn(sh, "index %u unavailable (%d)", (unsigned int)i, ret);
			continue;
		}

		shell_print_node_summary(sh, &node);
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_nodedb_list)

static int cmd_nodedb_warm(const struct shell *sh, size_t argc, char **argv)
{
	size_t count;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	count = meshtastic_nodedb_warm_count();
	shell_print(sh, "warm keys: %u", (unsigned int)count);

	for (size_t i = 0U; i < count; i++) {
		uint32_t num;
		uint32_t last_seen;
		const char *kind;

		if (meshtastic_nodedb_warm_get(i, &num, &last_seen) < 0) {
			break;
		}

		/* Distinguish a persisted wall-clock stamp (epoch, > ~2001) from a
		 * pre-sync uptime stamp or a legacy-restored 0. */
		kind = (last_seen > 1000000000U) ? "epoch"
			: (last_seen == 0U) ? "none" : "uptime";
		shell_print(sh, "0x%08x last_seen=%u (%s)", num,
			    (unsigned int)last_seen, kind);
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_nodedb_warm)

static int cmd_nodedb_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_nodedb_node node;
	uint32_t node_num;
	int32_t snr;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic nodedb show <node|0xnode>");
		return -EINVAL;
	}

	ret = mtsh_parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	ret = meshtastic_nodedb_get(node_num, &node);
	if (ret < 0) {
		shell_error(sh, "node 0x%08x not found", node_num);
		return ret;
	}

	snr = mtsh_scaled_tenths(node.snr);
	shell_print(sh, "node: 0x%08x", node.num);
	/* An age, as `nodedb list` prints it -- the raw uptime stamp printed here
	 * until 2026-09-27 read like an age ("last heard: 82908s") and was not. */
	if (node.last_heard_uptime_sec == 0U) {
		shell_print(sh, "last heard: never-this-boot");
	} else {
		uint32_t now = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);

		shell_print(sh, "last heard: %us ago",
			    (now > node.last_heard_uptime_sec)
				    ? (now - node.last_heard_uptime_sec) : 0U);
	}
	shell_print(sh, "snr: %d.%u", mtsh_scaled_whole(snr, 10), mtsh_scaled_fraction(snr, 10));
	shell_print(sh, "channel: %u", node.channel);
	if (node.heard_preset != 0xFFU) {
		shell_print(sh, "heard on: %s",
			    meshtastic_preset_display_name(
				    (meshtastic_Config_LoRaConfig_ModemPreset)node.heard_preset,
				    true));
	} else {
		shell_print(sh, "heard on: (not since boot)");
	}
	shell_print(sh, "next hop: 0x%02x", node.next_hop);
	shell_print(sh, "via mqtt: %s", node.via_mqtt ? "yes" : "no");
	if (node.has_hops_away) {
		shell_print(sh, "hops away: %u", node.hops_away);
	}
	shell_print(sh, "favorite: %s", node.is_favorite ? "yes" : "no");
#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
	shell_print(sh, "key verified: %s", node.is_key_manually_verified ? "yes" : "no");
#endif
	/* The cached peer position (agents-ooma.39) as it arrived on air: `time` is 0
	 * from a sender whose clock is below NTP quality, `precision` is the sender's
	 * channel precision -- the two fields a bench cannot otherwise read off a
	 * received position without a packet hex dump. */
	if (node.has_position) {
		shell_print(sh, "position: lat=%d lon=%d alt=%d time=%u src=%u precision=%u",
			    node.position_latitude_i, node.position_longitude_i,
			    node.position_altitude, node.position_time,
			    node.position_location_source, node.position_precision_bits);
	}

	if (node.has_user) {
		shell_print(sh, "long name: %s", node.long_name);
		shell_print(sh, "short name: %s", node.short_name);
		shell_print(sh, "hw model: %u", node.hw_model);
		shell_print(sh, "role: %u", node.role);
		shell_print(sh, "licensed: %s", node.is_licensed ? "yes" : "no");
		if (node.has_is_unmessagable) {
			shell_print(sh, "unmessagable: %s", node.is_unmessagable ? "yes" : "no");
		}
		if (node.public_key_len >= 8U) {
			shell_print(sh, "public key: %02x%02x%02x%02x%02x%02x%02x%02x… (%u bytes, pinned)",
				    node.public_key[0], node.public_key[1], node.public_key[2],
				    node.public_key[3], node.public_key[4], node.public_key[5],
				    node.public_key[6], node.public_key[7],
				    (unsigned int)node.public_key_len);
		} else {
			shell_print(sh, "public key bytes: %u", (unsigned int)node.public_key_len);
		}
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_nodedb_show)

/* `node forget <id>`: drop a peer from the NodeDB — the only way past key
 * pinning for a peer that legitimately re-keyed (a re-flashed kit): its next
 * NodeInfo is then accepted with the new key, and PKC to it works again. */
static int cmd_node_forget(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node_num;
	int ret;

	if (argc < 2) {
		shell_error(sh, "usage: node forget <node-hex>");
		return -EINVAL;
	}
	node_num = strtoul(argv[1], NULL, 16);
	ret = meshtastic_nodedb_forget(node_num);
	if (ret != 0) {
		shell_error(sh, "0x%08x: not in the NodeDB (%d)", node_num, ret);
		return ret;
	}
	shell_print(sh, "0x%08x forgotten — its next NodeInfo re-learns it (and its key)", node_num);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_node_forget)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_nodedb_favorite(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node_num;
	bool favorite = true;
	int ret;

	if (argc < 2U || argc > 3U) {
		shell_error(sh, "usage: meshtastic nodedb favorite <node|0xnode> [on|off]");
		return -EINVAL;
	}

	/* A favourite is persisted device state that changes how we relay; treat it
	 * as a config write, so a managed node refuses it the same way it refuses
	 * the other mutating shell commands. */
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = mtsh_parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	if (argc == 3U) {
		if (strcmp(argv[2], "on") == 0) {
			favorite = true;
		} else if (strcmp(argv[2], "off") == 0) {
			favorite = false;
		} else {
			shell_error(sh, "expected on or off, got %s", argv[2]);
			return -EINVAL;
		}
	}

	ret = meshtastic_nodedb_set_favorite(node_num, favorite);
	if (ret < 0) {
		shell_error(sh, "favorite failed: %d (is the node in the DB?)", ret);
		return ret;
	}

	shell_print(sh, "node 0x%08x favorite %s", node_num, favorite ? "on" : "off");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_nodedb_favorite)
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_nodedb_cmds,
			       SHELL_CMD(list, NULL, SHELL_HELP("List NodeDB entries.", NULL),
					 cmd_nodedb_list_on_work),
			       SHELL_CMD(show, NULL,
					 SHELL_HELP("Show one NodeDB entry.", "<node|0xnode>"),
					 cmd_nodedb_show_on_work),
			       SHELL_CMD(warm, NULL,
					 SHELL_HELP("List warm key-tier entries (num + LRU recency).",
						    NULL),
					 cmd_nodedb_warm_on_work),
			       SHELL_CMD_ARG(forget, NULL,
					     SHELL_HELP("Drop a peer from the NodeDB (past key pinning for a re-keyed peer).",
							"<node-hex>"),
					     cmd_node_forget_on_work, 2, 0),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
			       SHELL_CMD(favorite, NULL,
					 SHELL_HELP("Mark a node favorite / not.",
						    "<node|0xnode> [on|off]"),
					 cmd_nodedb_favorite_on_work),
#endif
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), nodedb, &meshtastic_nodedb_cmds,
		 SHELL_HELP("NodeDB commands.", NULL),
		 NULL, 0, 0);
