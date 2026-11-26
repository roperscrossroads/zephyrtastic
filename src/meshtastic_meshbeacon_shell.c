/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic beacon`: the mesh beacon (join-my-mesh announcement). Built with the shell and
 * CONFIG_MESHTASTIC_MESHBEACON (src/CMakeLists.txt); moved from meshtastic_shell.c
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

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_meshbeacon.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic beacon` (agents-dnr4.25). Reads always; writes gated. */
static void beacon_stored(meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_mesh_beacon_tag, &mod);
	*cfg = mod.payload_variant.mesh_beacon;
}

static int cmd_beacon_show(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	struct meshtastic_meshbeacon_offer offer;
	struct meshtastic_meshbeacon_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	beacon_stored(&cfg);
	meshtastic_meshbeacon_stats(&st);
	shell_print(sh, "beacon: listen %s, broadcast %s%s, every %u s",
		    (cfg.flags & MESHTASTIC_MESHBEACON_FLAG_LISTEN) ? "on" : "off",
		    (cfg.flags & MESHTASTIC_MESHBEACON_FLAG_BROADCAST) ? "on" : "off",
		    (cfg.flags & MESHTASTIC_MESHBEACON_FLAG_LEGACY_SPLIT) ? " (legacy split)" : "",
		    meshtastic_meshbeacon_interval_secs());
	shell_print(sh, "  text: %s%s%s", cfg.broadcast_message[0] ? "\"" : "(none)",
		    cfg.broadcast_message, cfg.broadcast_message[0] ? "\"" : "");
	shell_print(sh, "  offer: channel %s%s%s, preset %d, region %d",
		    cfg.has_broadcast_offer_channel ? "\"" : "",
		    cfg.has_broadcast_offer_channel ? cfg.broadcast_offer_channel.name : "-",
		    cfg.has_broadcast_offer_channel ? "\"" : "",
		    cfg.has_broadcast_offer_preset ? (int)cfg.broadcast_offer_preset : -1,
		    (int)cfg.broadcast_offer_region);
	shell_print(sh, "  targets: %u (radio switching not supported: %u skipped; inline "
			"channel without a slot: %u skipped)",
		    (unsigned int)cfg.broadcast_targets_count, st.targets_radio_switch,
		    st.targets_no_slot);
	shell_print(sh, "  sent %u frame(s), %u empty cycle(s); heard %u beacon(s), %u offer(s)",
		    st.frames_sent, st.cycles_empty, st.beacons_heard, st.offers_cached);
	if (meshtastic_meshbeacon_last_offer(&offer)) {
		shell_print(sh, "  last offer: from 0x%08x %lld s ago: channel %s%s%s, preset %d, "
				"region %d (not applied)",
			    offer.sender, (long long)((k_uptime_get() - offer.heard_uptime_ms) / 1000),
			    offer.has_channel ? "\"" : "", offer.has_channel ? offer.channel.name : "-",
			    offer.has_channel ? "\"" : "", offer.has_preset ? (int)offer.preset : -1,
			    (int)offer.region);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_beacon_show)

static int beacon_write(const struct shell *sh, const meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	ARG_UNUSED(cfg);
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	ret = meshtastic_meshbeacon_set(cfg);
	if (ret < 0) {
		shell_error(sh, "beacon set failed: %d", ret);
		return ret;
	}
	shell_print(sh, "persisted; applied live, no reboot");
	return 0;
#endif
}

/* beacon flag <listen|broadcast|legacy> <on|off> */
static int cmd_beacon_flag(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	uint32_t bit;

	if (argc != 3U || (strcmp(argv[2], "on") != 0 && strcmp(argv[2], "off") != 0)) {
		shell_error(sh, "usage: meshtastic beacon flag <listen|broadcast|legacy> on|off");
		return -EINVAL;
	}
	if (strcmp(argv[1], "listen") == 0) {
		bit = MESHTASTIC_MESHBEACON_FLAG_LISTEN;
	} else if (strcmp(argv[1], "broadcast") == 0) {
		bit = MESHTASTIC_MESHBEACON_FLAG_BROADCAST;
	} else if (strcmp(argv[1], "legacy") == 0) {
		bit = MESHTASTIC_MESHBEACON_FLAG_LEGACY_SPLIT;
	} else {
		shell_error(sh, "unknown flag: %s", argv[1]);
		return -EINVAL;
	}
	beacon_stored(&cfg);
	if (strcmp(argv[2], "on") == 0) {
		cfg.flags |= bit;
	} else {
		cfg.flags &= ~bit;
	}
	return beacon_write(sh, &cfg);
}
MTSH_DEFINE_ON_WORK(cmd_beacon_flag)

static int cmd_beacon_text(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	size_t len = 0U;

	beacon_stored(&cfg);
	cfg.broadcast_message[0] = '\0';
	for (size_t i = 1; i < argc; i++) {
		int n = snprintk(cfg.broadcast_message + len, sizeof(cfg.broadcast_message) - len,
				 "%s%s", (i > 1U) ? " " : "", argv[i]);

		if (n < 0 || (size_t)n >= sizeof(cfg.broadcast_message) - len) {
			shell_warn(sh, "text truncated to %u chars", MESHTASTIC_MESHBEACON_MESSAGE_MAX);
			break;
		}
		len += (size_t)n;
	}
	return beacon_write(sh, &cfg);
}
MTSH_DEFINE_ON_WORK(cmd_beacon_text)

static int cmd_beacon_interval(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	unsigned long secs;
	char *end;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic beacon interval <secs> (0 = minimum)");
		return -EINVAL;
	}
	secs = strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs > UINT32_MAX) {
		shell_error(sh, "invalid seconds value: %s", argv[1]);
		return -EINVAL;
	}
	beacon_stored(&cfg);
	cfg.broadcast_interval_secs = (uint32_t)secs;
	return beacon_write(sh, &cfg);
}
MTSH_DEFINE_ON_WORK(cmd_beacon_interval)

static int shell_beacon_send_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_meshbeacon_send();
	switch (ret) {
	case 0:
		shell_print(sh, "beacon sent");
		break;
	case -ENODATA:
		shell_error(sh, "nothing to beacon: no text and no offer configured");
		break;
	case -ENOENT:
		shell_error(sh, "every target was skipped (see `meshtastic beacon`)");
		break;
	case -EPERM:
		shell_error(sh, "a CLIENT_HIDDEN node never beacons");
		break;
	default:
		break;
	}
	return ret;
}

static int cmd_beacon_send(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_beacon_send_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_beacon_send)

static int cmd_beacon_forget(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_meshbeacon_clear_offer();
	shell_print(sh, "cached offer forgotten");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_beacon_forget)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_beacon_cmds,
			       SHELL_CMD_ARG(flag, NULL,
					     SHELL_HELP("Set a flag.",
							"<listen|broadcast|legacy> on|off"),
					     cmd_beacon_flag_on_work, 3, 0),
			       SHELL_CMD(text, NULL,
					 SHELL_HELP("Set (or with no words, clear) the beacon text.",
						    "[text...]"),
					 cmd_beacon_text_on_work),
			       SHELL_CMD_ARG(interval, NULL,
					     SHELL_HELP("Set the broadcast interval (0 = minimum).",
							"<secs>"),
					     cmd_beacon_interval_on_work, 2, 0),
			       SHELL_CMD(send, NULL, SHELL_HELP("Beacon now.", NULL),
					 cmd_beacon_send_on_work),
			       SHELL_CMD(forget, NULL,
					 SHELL_HELP("Forget the cached offer.", NULL),
					 cmd_beacon_forget_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), beacon, &meshtastic_beacon_cmds,
		 SHELL_HELP("Mesh beacon (join-my-mesh announcement): show, flag, text, "
		 "interval, send, forget.",
		 NULL),
		 cmd_beacon_show_on_work, 0, 0);
