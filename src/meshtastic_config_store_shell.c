/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic owner`, `meshtastic device` (role and rebroadcast mode) and, on a unified
 * BLE+TCP image, `meshtastic transport`: the node's own config. Built with the shell
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

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* Wall-clock bench surface. A BLE-only node has no SNTP and an indoor bench
 * no GNSS fix, so without this the only time source is a paired phone — and
 * every log/rx_time sits in 1970. `set` takes a Unix epoch (seconds; the
 * operator's host clock) at NTP quality, the same trust class as a phone
 * set_time_only, so a later GPS fix still outranks it and a mesh-relayed
 * value cannot clobber it. */
/*
 * meshtastic owner [set <long> [short]]
 *
 * The owner names were the one piece of node identity the shell could not
 * touch: it already writes channels, device role, rebroadcast mode, the LoRa
 * preset, admin keys and the clock, but the only owner writers were the phone
 * app and the remote-admin client (agents-xhli.15/.14 sessions). That gap is
 * not academic — it makes two states unrecoverable without a second node:
 *
 *   - a node whose admin_key list is EMPTY cannot be renamed by anything,
 *     because nothing is permitted to administer it;
 *   - a node that cannot TRANSMIT on the mesh (config.lora.tx_enabled = false)
 *     cannot answer the getter that must precede any mutating admin op, so
 *     remote admin cannot reach it over the radio at all — which is exactly the
 *     bench's receive-only node.
 *
 * Both have a working USB console. So the console can do it.
 *
 * Gated like every other config write: compiled out with
 * MESHTASTIC_SHELL_CONFIG_WRITE=n, and refused at runtime on a managed node.
 * Setting the owner does NOT mint an HLC version (the owner is not a shareable
 * Config section), so this never touches the cluster document.
 */
static int cmd_owner(const struct shell *sh, size_t argc, char **argv)
{
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc == 1U) {
		bool licensed = false;
		bool unmessagable = false;

		meshtastic_config_store_get_owner_flags(&licensed, &unmessagable);
		shell_print(sh, "owner: long=\"%s\" short=\"%s\"",
			    meshtastic_config_store_long_name(),
			    meshtastic_config_store_short_name());
		shell_print(sh, "flags: licensed=%s unmessagable=%s", licensed ? "yes" : "no",
			    unmessagable ? "yes" : "no");
		return 0;
	}

	if ((strcmp(argv[1], "set") != 0) || argc < 3U || argc > 4U) {
		shell_error(sh, "usage: meshtastic owner [set <long> [short]]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		meshtastic_User user = meshtastic_User_init_zero;
		bool licensed = false;
		bool unmessagable = false;
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}

		/*
		 * Carry the CURRENT flags through. set_owner takes a whole User and
		 * reads is_licensed from it as a plain proto3 bool — an unset field
		 * is indistinguishable from an explicit false — so building one from
		 * zero here would silently clear a licence as a side effect of a
		 * rename. (has_is_unmessagable does have presence, but pass it too so
		 * both flags are handled the same way rather than one by luck.)
		 */
		meshtastic_config_store_get_owner_flags(&licensed, &unmessagable);
		user.is_licensed = licensed;
		user.has_is_unmessagable = true;
		user.is_unmessagable = unmessagable;

		strncpy(user.long_name, argv[2], sizeof(user.long_name) - 1U);
		if (argc == 4U) {
			strncpy(user.short_name, argv[3], sizeof(user.short_name) - 1U);
		}

		ret = meshtastic_config_store_set_owner(&user);
		if (ret == -EINVAL) {
			/* The store's rule, the same one a phone's set_owner meets. */
			shell_error(sh, "owner set refused: a name may not be only whitespace");
			return ret;
		}
		if (ret < 0) {
			shell_error(sh, "owner set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "owner set to long=\"%s\" short=\"%s\"",
			    meshtastic_config_store_long_name(),
			    meshtastic_config_store_short_name());
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_owner)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_parse_device_role(const struct shell *sh, const char *arg,
				   meshtastic_Config_DeviceConfig_Role *role)
{
	if (strcmp(arg, "client") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_CLIENT;
	} else if (strcmp(arg, "client_mute") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
	} else if (strcmp(arg, "router") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_ROUTER;
	} else if (strcmp(arg, "router_late") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_ROUTER_LATE;
	} else if (strcmp(arg, "client_base") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_CLIENT_BASE;
	} else if (strcmp(arg, "sensor") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_SENSOR;
	} else if (strcmp(arg, "tracker") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_TRACKER;
	} else if (strcmp(arg, "tak") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_TAK;
	} else if (strcmp(arg, "client_hidden") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN;
	} else if (strcmp(arg, "lost_and_found") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND;
	} else if (strcmp(arg, "tak_tracker") == 0) {
		*role = meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;
	} else {
		shell_error(sh, "invalid device role: %s", arg);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_parse_rebroadcast_mode(const struct shell *sh, const char *arg,
					meshtastic_Config_DeviceConfig_RebroadcastMode *mode)
{
	if (strcmp(arg, "all") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL;
	} else if (strcmp(arg, "all_skip_decoding") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING;
	} else if (strcmp(arg, "local_only") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY;
	} else if (strcmp(arg, "known_only") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY;
	} else if (strcmp(arg, "none") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_NONE;
	} else if (strcmp(arg, "core_portnums_only") == 0) {
		*mode = meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY;
	} else {
		shell_error(sh, "invalid rebroadcast mode: %s", arg);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static int cmd_device_role(const struct shell *sh, size_t argc, char **argv)
{
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc == 1U) {
		shell_print(sh, "role: %s", mtsh_device_role_name(meshtastic_device_role()));
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic device role [name]");
		return -EINVAL;
	}

	/* Reading the role is always allowed (handled above); only the write below
	 * is gated. */
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		meshtastic_Config_DeviceConfig_Role role;
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}

		ret = shell_parse_device_role(sh, argv[1], &role);
		if (ret < 0) {
			return ret;
		}

		ret = meshtastic_config_store_set_device_role(role);
		if (ret < 0) {
			shell_error(sh, "role set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "role set to %s", mtsh_device_role_name(role));
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_device_role)

static int cmd_device_rebroadcast(const struct shell *sh, size_t argc, char **argv)
{
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc == 1U) {
		shell_print(sh, "rebroadcast: %s",
			    mtsh_rebroadcast_mode_name(meshtastic_rebroadcast_mode()));
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic device rebroadcast [mode]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		meshtastic_Config_DeviceConfig_RebroadcastMode mode;
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}

		ret = shell_parse_rebroadcast_mode(sh, argv[1], &mode);
		if (ret < 0) {
			return ret;
		}

		ret = meshtastic_config_store_set_rebroadcast_mode(mode);
		if (ret < 0) {
			shell_error(sh, "rebroadcast set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "rebroadcast set to %s", mtsh_rebroadcast_mode_name(mode));
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_device_rebroadcast)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_device_cmds,
			       SHELL_CMD(role, NULL, SHELL_HELP("Get/set device role.", "[name]"),
					 cmd_device_role_on_work),
			       SHELL_CMD(rebroadcast, NULL,
					 SHELL_HELP("Get/set rebroadcast mode.", "[mode]"),
					 cmd_device_rebroadcast_on_work),
			       SHELL_SUBCMD_SET_END);

#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
/*
 * `meshtastic transport [ble|wifi]` — only meaningful in a unified image, where
 * both transports are compiled and the persisted network.wifi_enabled picks the
 * one that boots. The switch takes effect on reboot (only one radio stack comes
 * up per boot, so it cannot flip live). Bench counterpart to the phone app's
 * "WiFi enabled" toggle, for a serial/telnet console with no phone attached.
 */
static int cmd_transport_show(const struct shell *sh)
{
	meshtastic_Config config;
	bool wifi = meshtastic_transport_prefer_wifi();
	bool stored = (meshtastic_config_store_get_config(meshtastic_Config_network_tag, &config) ==
			       0 &&
		       config.which_payload_variant == meshtastic_Config_network_tag);

	shell_print(sh, "phone transport:  %s%s", wifi ? "wifi" : "ble",
		    stored ? "" : "  (default; no NetworkConfig stored)");
	shell_print(sh, "active this boot: %s  (a change applies on reboot)",
		    wifi ? "wifi/net" : "ble");
	return 0;
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_transport_set(const struct shell *sh, bool wifi)
{
	meshtastic_Config config;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_network_tag, &config);
	if (ret < 0) {
		shell_error(sh, "network get failed: %d", ret);
		return ret;
	}

	/* Touch only the field we own and write the whole NetworkConfig back, as the
	 * admin set_config path does; get_config leaves the slot tagged network. */
	config.which_payload_variant = meshtastic_Config_network_tag;
	config.payload_variant.network.wifi_enabled = wifi;

	ret = meshtastic_config_store_set_config(&config);
	if (ret < 0) {
		shell_error(sh, "network set failed: %d", ret);
		return ret;
	}

	/* set_config only schedules the save; flush so the choice survives the reboot
	 * that applies it. */
	ret = meshtastic_settings_flush();
	if (ret < 0) {
		shell_error(sh, "flush failed: %d", ret);
		return ret;
	}

	shell_print(sh, "phone transport -> %s (persisted); reboot to apply "
			"(`kernel reboot cold`)",
		    wifi ? "wifi" : "ble");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static int cmd_transport(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		return cmd_transport_show(sh);
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic transport [ble|wifi]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	if (strcmp(argv[1], "wifi") == 0) {
		return cmd_transport_set(sh, true);
	}
	if (strcmp(argv[1], "ble") == 0) {
		return cmd_transport_set(sh, false);
	}

	shell_error(sh, "expected ble or wifi, got %s", argv[1]);
	return -EINVAL;
#endif
}
MTSH_DEFINE_ON_WORK(cmd_transport)
#endif /* CONFIG_MESHTASTIC_BLE && CONFIG_MESHTASTIC_TCP */

SHELL_SUBCMD_ADD((meshtastic), owner, NULL,
		 SHELL_HELP("Show or set this node's owner names.",
		 "[set <long> [short]]"),
		 cmd_owner_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
SHELL_SUBCMD_ADD((meshtastic), transport, NULL,
		 SHELL_HELP("Show or set the phone transport, reboot to apply (unified image).",
		 "[ble|wifi]"),
		 cmd_transport_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), device, &meshtastic_device_cmds,
		 SHELL_HELP("Device role and rebroadcast commands.", NULL),
		 NULL, 0, 0);
