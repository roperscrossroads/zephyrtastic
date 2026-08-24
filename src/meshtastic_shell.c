/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/version.h>
#if defined(CONFIG_PM)
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#endif
#if defined(CONFIG_NETWORKING)
#include <zephyr/net/net_if.h>
#endif

#include <zephyr/meshtastic/gnss.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include <zephyr/meshtastic/telemetry.h>

#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include <zephyr/bluetooth/addr.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#endif
#if defined(CONFIG_MESHTASTIC_DFU_TRIGGER)
#include "meshtastic_dfu_trigger.h"
#endif
#include "meshtastic_build.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_powermon.h"
#include <zephyr/meshtastic/fem.h>

#include "meshtastic_tx_power.h"
#include "meshtastic_sched.h"
#include "meshtastic_settings.h"

#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
#include <zephyr/logging/log_backend_net.h>
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

enum shell_work_op {
	SHELL_WORK_SEND_TEXT,
	SHELL_WORK_SEND_POSITION,
	SHELL_WORK_SEND_METRICS,
	SHELL_WORK_SEND_ENVIRONMENT,
	SHELL_WORK_SEND_NODEINFO,
};

struct shell_work_item {
	const struct shell *sh;
	enum shell_work_op op;
	uint32_t dest;
	uint32_t portnum;
	uint8_t channel_index;
	size_t payload_len;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
};

static const char *shell_err_msg(int ret)
{
	switch (ret) {
	case -ENODATA:
		return "no GNSS fix available yet";
	case -ENODEV:
		return "no sensor readings available";
	case -ENOTSUP:
		return "not enabled in Kconfig";
	default:
		return strerror(-ret);
	}
}

static void shell_report_result(const struct shell *sh, int ret)
{
	if (ret < 0) {
		LOG_ERR("Meshtastic shell command failed: %s (%d)", shell_err_msg(ret), -ret);
	}

	if (sh == NULL) {
		return;
	}

	/*
	 * shell_print() can block on the shell TX mutex while the shell thread
	 * is busy (e.g. with log output). Log the outcome and attempt a shell
	 * message without stalling the work queue indefinitely.
	 */
	if (ret < 0) {
		shell_error(sh, "failed: %s (%d)", shell_err_msg(ret), -ret);
	} else {
		LOG_INF("Meshtastic shell command completed");
		shell_print(sh, "done");
	}
}

K_MSGQ_DEFINE(shell_work_msgq, sizeof(struct shell_work_item), CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE,
	      4);
K_THREAD_STACK_DEFINE(shell_work_stack, CONFIG_MESHTASTIC_SHELL_WORK_STACK_SIZE);
static struct k_thread shell_work_thread;

static void shell_work_thread_fn(void *p1, void *p2, void *p3)
{
	struct shell_work_item item;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msgq_get(&shell_work_msgq, &item, K_FOREVER);

		switch (item.op) {
		case SHELL_WORK_SEND_TEXT:
			if (item.channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
				struct meshtastic_packet packet = {
					.to = item.dest,
					.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
					.payload = item.payload,
					.payload_len = item.payload_len,
					.channel_index = item.channel_index,
				};

				ret = meshtastic_send_packet(&packet, K_FOREVER);
			} else {
				ret = meshtastic_send_text(item.dest, (const char *)item.payload);
			}
			break;
		case SHELL_WORK_SEND_POSITION:
			ret = meshtastic_send_position(item.dest);
			break;
		case SHELL_WORK_SEND_METRICS:
			ret = meshtastic_send_device_metrics(item.dest, K_FOREVER);
			break;
		case SHELL_WORK_SEND_ENVIRONMENT:
			ret = meshtastic_send_environment(item.dest, K_FOREVER);
			break;
		case SHELL_WORK_SEND_NODEINFO:
			ret = meshtastic_send_node_info(item.dest);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		shell_report_result(item.sh, ret);
	}
}

static int enqueue_shell_work(const struct shell *sh, struct shell_work_item *item)
{
	int ret;

	item->sh = sh;
	ret = k_msgq_put(&shell_work_msgq, item, K_NO_WAIT);
	if (ret < 0) {
		shell_error(sh, "Meshtastic shell queue is full");
		return ret;
	}

	shell_print(sh, "queued");
	return 0;
}

static int parse_u32(const struct shell *sh, const char *arg, uint32_t *value)
{
	char *end;
	unsigned long parsed;

	if (strcmp(arg, "broadcast") == 0) {
		*value = MESHTASTIC_NODE_BROADCAST;
		return 0;
	}

	parsed = strtoul(arg, &end, 0);
	if (*end != '\0' || parsed > UINT32_MAX) {
		shell_error(sh, "invalid integer: %s", arg);
		return -EINVAL;
	}

	*value = (uint32_t)parsed;
	return 0;
}

static int parse_optional_dest(const struct shell *sh, size_t argc, char **argv, uint32_t *dest)
{
	if (argc == 1U) {
		*dest = MESHTASTIC_NODE_BROADCAST;
		return 0;
	}

	if (argc == 2U) {
		return parse_u32(sh, argv[1], dest);
	}

	shell_error(sh, "too many arguments");
	return -EINVAL;
}

static int cmd_deferred_send(const struct shell *sh, size_t argc, char **argv,
			     enum shell_work_op op)
{
	struct shell_work_item item = {
		.op = op,
	};
	int ret;

	ret = parse_optional_dest(sh, argc, argv, &item.dest);
	if (ret < 0) {
		return ret;
	}

	return enqueue_shell_work(sh, &item);
}

static int append_message_from_argv(uint8_t *buf, size_t buf_max, size_t argc, char **argv,
				    size_t first_arg)
{
	size_t pos = 0;

	for (size_t i = first_arg; i < argc; i++) {
		size_t len = strlen(argv[i]);

		if (i > first_arg) {
			if (pos >= buf_max) {
				return -ENOMEM;
			}
			buf[pos++] = ' ';
		}

		if (pos + len > buf_max) {
			return -ENOMEM;
		}

		memcpy(buf + pos, argv[i], len);
		pos += len;
	}

	return (int)pos;
}

static int32_t scaled_tenths(float value)
{
	return (int32_t)(value * 10.0f);
}

static int32_t scaled_whole(int32_t scaled, int32_t divisor)
{
	return scaled / divisor;
}

static uint32_t scaled_fraction(int32_t scaled, int32_t divisor)
{
	int32_t fraction = scaled % divisor;

	if (fraction < 0) {
		fraction = -fraction;
	}

	return (uint32_t)fraction;
}

static const char *shell_device_role_name(meshtastic_Config_DeviceConfig_Role role);
static const char *shell_rebroadcast_mode_name(meshtastic_Config_DeviceConfig_RebroadcastMode mode);

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Developer build identity — distinct from the "2.7.4.zephyr" phone-protocol
	 * version. Answers "old or new image?" at the console. */
	shell_print(sh, "build:  %s (built %s UTC)", meshtastic_build_id(),
		    meshtastic_build_time());
	shell_print(sh, "board:  %s", CONFIG_BOARD);
	shell_print(sh, "zephyr: %s", KERNEL_VERSION_STRING);
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_status status;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_get_status(&status);
	if (ret < 0) {
		shell_error(sh, "status failed: %d", ret);
		return ret;
	}

	shell_print(sh, "node: 0x%08x", status.node_id);
	shell_print(sh, "initialized: %s", status.initialized ? "yes" : "no");
	shell_print(sh, "ble connected: %s", status.ble_connected ? "yes" : "no");
	shell_print(sh, "tx: %u ok, %u failed", status.tx_packets, status.tx_failures);
	shell_print(sh, "rx: %u decoded, %u duplicates, %u decode failures", status.rx_packets,
		    status.duplicate_packets, status.decode_failures);
	shell_print(sh, "rx dropped: %u, rx re-arm failures: %u", status.rx_dropped,
		    status.rx_rearm_failures);
	shell_print(sh, "relayed: %u", status.relayed_packets);
	shell_print(sh, "last rx: from=0x%08x rssi=%d snr=%d", status.last_rx_from,
		    status.last_rssi, status.last_snr);
	shell_print(sh, "primary channel: %u \"%s\" hash=0x%02x",
		    (unsigned int)meshtastic_channels_primary_index(),
		    meshtastic_channels_primary_name(), meshtastic_channels_primary_hash());
	shell_print(sh, "device role: %s", shell_device_role_name(meshtastic_device_role()));
	shell_print(sh, "rebroadcast mode: %s",
		    shell_rebroadcast_mode_name(meshtastic_rebroadcast_mode()));
	shell_print(sh, "rebroadcasting: %s", meshtastic_is_rebroadcaster() ? "yes" : "no");

	return 0;
}

static const char *shell_channel_role_name(meshtastic_Channel_Role role)
{
	switch (role) {
	case meshtastic_Channel_Role_PRIMARY:
		return "primary";
	case meshtastic_Channel_Role_SECONDARY:
		return "secondary";
	case meshtastic_Channel_Role_DISABLED:
	default:
		return "disabled";
	}
}

/* Parsers used only by the config-write commands below. */
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_parse_channel_role(const struct shell *sh, const char *arg,
				    meshtastic_Channel_Role *role)
{
	if (strcmp(arg, "primary") == 0) {
		*role = meshtastic_Channel_Role_PRIMARY;
	} else if (strcmp(arg, "secondary") == 0) {
		*role = meshtastic_Channel_Role_SECONDARY;
	} else if (strcmp(arg, "disabled") == 0) {
		*role = meshtastic_Channel_Role_DISABLED;
	} else {
		shell_error(sh, "invalid channel role: %s", arg);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static const char *shell_device_role_name(meshtastic_Config_DeviceConfig_Role role)
{
	switch (role) {
	case meshtastic_Config_DeviceConfig_Role_CLIENT:
		return "client";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE:
		return "client_mute";
	case meshtastic_Config_DeviceConfig_Role_ROUTER:
		return "router";
	case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
		return "router_late";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_BASE:
		return "client_base";
	case meshtastic_Config_DeviceConfig_Role_SENSOR:
		return "sensor";
	case meshtastic_Config_DeviceConfig_Role_TRACKER:
		return "tracker";
	case meshtastic_Config_DeviceConfig_Role_TAK:
		return "tak";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN:
		return "client_hidden";
	case meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND:
		return "lost_and_found";
	case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
		return "tak_tracker";
	default:
		return "unknown";
	}
}

/* Parsers used only by the config-write commands below. */
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

static const char *shell_rebroadcast_mode_name(meshtastic_Config_DeviceConfig_RebroadcastMode mode)
{
	switch (mode) {
	case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL:
		return "all";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING:
		return "all_skip_decoding";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY:
		return "local_only";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY:
		return "known_only";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_NONE:
		return "none";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY:
		return "core_portnums_only";
	default:
		return "unknown";
	}
}

/* Parsers used only by the config-write commands below. */
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

static int shell_parse_channel_index(const struct shell *sh, const char *arg, uint8_t *index)
{
	unsigned long parsed;
	char *end;

	parsed = strtoul(arg, &end, 0);
	if (*end != '\0' || parsed >= MESHTASTIC_MAX_CHANNELS) {
		shell_error(sh, "invalid channel index: %s", arg);
		return -EINVAL;
	}

	*index = (uint8_t)parsed;
	return 0;
}

/* Parsers used only by the config-write commands below. */
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_parse_hex_nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	return -1;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

/* Parsers used only by the config-write commands below. */
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_parse_hex_psk(const struct shell *sh, const char *hex, uint8_t *out,
			       size_t *out_len)
{
	size_t hex_len = strlen(hex);
	size_t byte_len;
	int hi;
	int lo;

	if (hex_len != 32U && hex_len != 64U) {
		shell_error(sh, "psk hex must be 32 or 64 characters");
		return -EINVAL;
	}

	byte_len = hex_len / 2U;
	for (size_t i = 0; i < byte_len; i++) {
		hi = shell_parse_hex_nibble(hex[i * 2U]);
		lo = shell_parse_hex_nibble(hex[(i * 2U) + 1U]);
		if (hi < 0 || lo < 0) {
			shell_error(sh, "invalid hex in psk");
			return -EINVAL;
		}
		out[i] = (uint8_t)((hi << 4) | lo);
	}

	*out_len = byte_len;
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static void shell_print_psk_summary(const struct shell *sh, const meshtastic_Channel *ch)
{
	const meshtastic_ChannelSettings *settings;

	if (ch == NULL || !ch->has_settings) {
		shell_print(sh, "psk: (none)");
		return;
	}

	settings = &ch->settings;
	if (ch->role == meshtastic_Channel_Role_DISABLED) {
		shell_print(sh, "psk: n/a");
		return;
	}

	if (settings->psk.size == 0U) {
		if (ch->role == meshtastic_Channel_Role_SECONDARY) {
			shell_print(sh, "psk: inherit primary");
		} else {
			shell_print(sh, "psk: cleartext");
		}
		return;
	}

	if (settings->psk.size == 1U) {
		shell_print(sh, "psk: shorthand %u", settings->psk.bytes[0]);
		return;
	}

	shell_print(sh, "psk: %u-byte key", (unsigned int)settings->psk.size);
}

/* Raw key material, so it is opt-in at build time
 * (CONFIG_MESHTASTIC_SHELL_PSK_HEX). The console has no authentication of any
 * kind; printing a live PSK there hands the channel to anyone who can read the
 * output or scroll back through it. The summary above says which key a slot
 * uses without disclosing it. */
static void shell_print_psk_hex(const struct shell *sh, const meshtastic_Channel *ch)
{
#if defined(CONFIG_MESHTASTIC_SHELL_PSK_HEX)
	const meshtastic_ChannelSettings *settings;

	if (ch == NULL || !ch->has_settings || ch->role == meshtastic_Channel_Role_DISABLED) {
		return;
	}

	settings = &ch->settings;
	if (settings->psk.size != 16U && settings->psk.size != 32U) {
		shell_print_psk_summary(sh, ch);
		return;
	}

	shell_fprintf(sh, SHELL_NORMAL, "psk hex: ");
	for (pb_size_t i = 0; i < settings->psk.size; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x", settings->psk.bytes[i]);
	}
	shell_print(sh, "");
#else
	ARG_UNUSED(sh);
	ARG_UNUSED(ch);
#endif
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/* Refuse a config write when the node is administratively managed. The admin
 * model says a managed node takes configuration only from an authorized remote
 * admin; the shell writes the same config store, so it has to honour the same
 * answer or the gate means nothing. Returns true when the caller should stop.
 *
 * With admin compiled out there is no admin model to defer to, so nothing is
 * refused — the compile-time CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE switch is the
 * control that still applies in that build. */
static bool shell_config_write_refused(const struct shell *sh)
{
#if defined(CONFIG_MESHTASTIC_ADMIN)
	if (meshtastic_admin_is_managed()) {
		shell_error(sh, "refused: node is managed (SecurityConfig.is_managed) — "
				"configuration is set by an authorized remote admin");
		return true;
	}
#else
	ARG_UNUSED(sh);
#endif
	return false;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static void shell_print_channel_line(const struct shell *sh, uint8_t index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(index);
	const char *name;

	if (ch == NULL) {
		return;
	}

	name = meshtastic_channels_get_name(index);
	shell_print(sh, "[%u] role=%s name=\"%s\" hash=0x%02x uplink=%s downlink=%s",
		    (unsigned int)index, shell_channel_role_name(ch->role), name,
		    meshtastic_channels_get_hash(index),
		    ch->has_settings && ch->settings.uplink_enabled ? "on" : "off",
		    ch->has_settings && ch->settings.downlink_enabled ? "on" : "off");
	shell_print_psk_summary(sh, ch);
}

static int cmd_channel_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (uint8_t i = 0; i < MESHTASTIC_MAX_CHANNELS; i++) {
		shell_print_channel_line(sh, i);
	}

	return 0;
}

static int cmd_channel_show(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t index;
	const meshtastic_Channel *ch;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic channel show <index>");
		return -EINVAL;
	}

	ret = shell_parse_channel_index(sh, argv[1], &index);
	if (ret < 0) {
		return ret;
	}

	ch = meshtastic_channels_get(index);
	if (ch == NULL) {
		return -EINVAL;
	}

	shell_print(sh, "index: %u", (unsigned int)index);
	shell_print(sh, "role: %s", shell_channel_role_name(ch->role));
	shell_print(sh, "name: \"%s\"", meshtastic_channels_get_name(index));
	shell_print(sh, "hash: 0x%02x", meshtastic_channels_get_hash(index));
	if (ch->has_settings) {
		shell_print(sh, "uplink: %s", ch->settings.uplink_enabled ? "on" : "off");
		shell_print(sh, "downlink: %s", ch->settings.downlink_enabled ? "on" : "off");
	}
	shell_print_psk_summary(sh, ch);
	shell_print_psk_hex(sh, ch);

	return 0;
}

/* Parsers used only by the config-write commands below. */
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int shell_apply_channel_psk(const struct shell *sh, meshtastic_Channel *ch, const char *kind,
				   const char *arg)
{
	size_t psk_len = 0U;
	unsigned long shorthand;
	char *end;
	int ret;

	if (strcmp(kind, "none") == 0) {
		ch->settings.psk.size = 0U;
		return 0;
	}

	if (strcmp(kind, "default") == 0) {
		ch->settings.psk.bytes[0] = 1U;
		ch->settings.psk.size = 1U;
		return 0;
	}

	if (strcmp(kind, "hex") == 0) {
		if (arg == NULL) {
			shell_error(sh, "usage: psk hex <32|64 hex chars>");
			return -EINVAL;
		}

		ret = shell_parse_hex_psk(sh, arg, ch->settings.psk.bytes, &psk_len);
		if (ret < 0) {
			return ret;
		}
		ch->settings.psk.size = (pb_size_t)psk_len;
		return 0;
	}

	shorthand = strtoul(kind, &end, 0);
	if (*end != '\0' || shorthand > 10U) {
		shell_error(sh, "invalid psk: %s", kind);
		return -EINVAL;
	}

	ch->settings.psk.bytes[0] = (uint8_t)shorthand;
	ch->settings.psk.size = 1U;
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_channel_set(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t index;
	meshtastic_Channel ch;
	int ret;
	bool changed = false;

	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	if (argc < 3U) {
		shell_error(sh, "usage: meshtastic channel set <index> "
				"[name <str>] [role primary|secondary|disabled] "
				"[psk none|default|<0-10>|hex <hex>] "
				"[uplink on|off] [downlink on|off]");
		return -EINVAL;
	}

	ret = shell_parse_channel_index(sh, argv[1], &index);
	if (ret < 0) {
		return ret;
	}

	{
		const meshtastic_Channel *cur = meshtastic_channels_get(index);

		if (cur == NULL) {
			return -EINVAL;
		}
		ch = *cur;
	}
	ch.index = index;
	ch.has_settings = true;

	for (size_t i = 2; i < argc; i++) {
		if (strcmp(argv[i], "name") == 0) {
			if (++i >= argc) {
				shell_error(sh, "name requires a value");
				return -EINVAL;
			}
			strncpy(ch.settings.name, argv[i], sizeof(ch.settings.name) - 1U);
			ch.settings.name[sizeof(ch.settings.name) - 1U] = '\0';
			changed = true;
		} else if (strcmp(argv[i], "role") == 0) {
			if (++i >= argc) {
				shell_error(sh, "role requires a value");
				return -EINVAL;
			}
			ret = shell_parse_channel_role(sh, argv[i], &ch.role);
			if (ret < 0) {
				return ret;
			}
			changed = true;
		} else if (strcmp(argv[i], "psk") == 0) {
			const char *psk_arg = NULL;

			if (++i >= argc) {
				shell_error(sh, "psk requires a value");
				return -EINVAL;
			}
			if (strcmp(argv[i], "hex") == 0) {
				if (++i >= argc) {
					shell_error(sh, "psk hex requires hex digits");
					return -EINVAL;
				}
				psk_arg = argv[i];
				ret = shell_apply_channel_psk(sh, &ch, "hex", psk_arg);
			} else {
				ret = shell_apply_channel_psk(sh, &ch, argv[i], NULL);
			}
			if (ret < 0) {
				return ret;
			}
			changed = true;
		} else if (strcmp(argv[i], "uplink") == 0) {
			if (++i >= argc) {
				shell_error(sh, "uplink requires on|off");
				return -EINVAL;
			}
			ch.settings.uplink_enabled = (strcmp(argv[i], "on") == 0);
			changed = true;
		} else if (strcmp(argv[i], "downlink") == 0) {
			if (++i >= argc) {
				shell_error(sh, "downlink requires on|off");
				return -EINVAL;
			}
			ch.settings.downlink_enabled = (strcmp(argv[i], "on") == 0);
			changed = true;
		} else {
			shell_error(sh, "unknown option: %s", argv[i]);
			return -EINVAL;
		}
	}

	if (!changed) {
		shell_error(sh, "no channel options specified");
		return -EINVAL;
	}

	ret = meshtastic_config_store_set_channel(index, &ch);
	if (ret < 0) {
		shell_error(sh, "channel set failed: %d", ret);
		return ret;
	}

	shell_print(sh, "channel %u updated (hash=0x%02x)", (unsigned int)index,
		    meshtastic_channels_get_hash(index));
	return 0;
}

static int cmd_channel_disable(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t index;
	meshtastic_Channel ch;
	int ret;

	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic channel disable <index>");
		return -EINVAL;
	}

	ret = shell_parse_channel_index(sh, argv[1], &index);
	if (ret < 0) {
		return ret;
	}

	{
		const meshtastic_Channel *cur = meshtastic_channels_get(index);

		if (cur == NULL) {
			return -EINVAL;
		}
		ch = *cur;
	}

	ch.role = meshtastic_Channel_Role_DISABLED;
	ch.has_settings = true;

	ret = meshtastic_config_store_set_channel(index, &ch);
	if (ret < 0) {
		shell_error(sh, "channel disable failed: %d", ret);
		return ret;
	}

	shell_print(sh, "channel %u disabled", (unsigned int)index);
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_channel_cmds,
	SHELL_CMD(list, NULL, SHELL_HELP("List channel slots.", NULL), cmd_channel_list),
	SHELL_CMD(show, NULL, SHELL_HELP("Show one channel slot.", "<index>"), cmd_channel_show),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	SHELL_CMD(set, NULL,
		  SHELL_HELP("Update a channel slot.",
			     "<index> [name|role|psk|uplink|downlink]..."),
		  cmd_channel_set),
	SHELL_CMD(disable, NULL, SHELL_HELP("Disable a channel slot.", "<index>"),
		  cmd_channel_disable),
#endif
	SHELL_SUBCMD_SET_END);

static int cmd_device_role(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		shell_print(sh, "role: %s", shell_device_role_name(meshtastic_device_role()));
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

		if (shell_config_write_refused(sh)) {
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
		shell_print(sh, "role set to %s", shell_device_role_name(role));
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

static int cmd_device_rebroadcast(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		shell_print(sh, "rebroadcast: %s",
			    shell_rebroadcast_mode_name(meshtastic_rebroadcast_mode()));
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

		if (shell_config_write_refused(sh)) {
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
		shell_print(sh, "rebroadcast set to %s", shell_rebroadcast_mode_name(mode));
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_device_cmds,
			       SHELL_CMD(role, NULL, SHELL_HELP("Get/set device role.", "[name]"),
					 cmd_device_role),
			       SHELL_CMD(rebroadcast, NULL,
					 SHELL_HELP("Get/set rebroadcast mode.", "[mode]"),
					 cmd_device_rebroadcast),
			       SHELL_SUBCMD_SET_END);

#if defined(CONFIG_MESHTASTIC_NODEDB)
static void shell_print_node_summary(const struct shell *sh,
				     const struct meshtastic_nodedb_node *node)
{
	int32_t snr = scaled_tenths(node->snr);
	const char *long_name = node->has_user ? node->long_name : "";
	const char *short_name = node->has_user ? node->short_name : "";

	if (node->has_hops_away) {
		shell_print(sh, "0x%08x last=%us snr=%d.%u hops=%u via=%s long=\"%s\" short=\"%s\"",
			    node->num, node->last_heard_uptime_sec, scaled_whole(snr, 10),
			    scaled_fraction(snr, 10), node->hops_away,
			    node->via_mqtt ? "yes" : "no", long_name, short_name);
	} else {
		shell_print(sh, "0x%08x last=%us snr=%d.%u hops=? via=%s long=\"%s\" short=\"%s\"",
			    node->num, node->last_heard_uptime_sec, scaled_whole(snr, 10),
			    scaled_fraction(snr, 10), node->via_mqtt ? "yes" : "no", long_name,
			    short_name);
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

	ret = parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	ret = meshtastic_nodedb_get(node_num, &node);
	if (ret < 0) {
		shell_error(sh, "node 0x%08x not found", node_num);
		return ret;
	}

	snr = scaled_tenths(node.snr);
	shell_print(sh, "node: 0x%08x", node.num);
	shell_print(sh, "last heard: %us", node.last_heard_uptime_sec);
	shell_print(sh, "snr: %d.%u", scaled_whole(snr, 10), scaled_fraction(snr, 10));
	shell_print(sh, "channel: %u", node.channel);
	shell_print(sh, "next hop: 0x%02x", node.next_hop);
	shell_print(sh, "via mqtt: %s", node.via_mqtt ? "yes" : "no");
	if (node.has_hops_away) {
		shell_print(sh, "hops away: %u", node.hops_away);
	}
	shell_print(sh, "favorite: %s", node.is_favorite ? "yes" : "no");

	if (node.has_user) {
		shell_print(sh, "long name: %s", node.long_name);
		shell_print(sh, "short name: %s", node.short_name);
		shell_print(sh, "hw model: %u", node.hw_model);
		shell_print(sh, "role: %u", node.role);
		shell_print(sh, "licensed: %s", node.is_licensed ? "yes" : "no");
		if (node.has_is_unmessagable) {
			shell_print(sh, "unmessagable: %s", node.is_unmessagable ? "yes" : "no");
		}
		shell_print(sh, "public key bytes: %u", (unsigned int)node.public_key_len);
	}

	return 0;
}

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
	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = parse_u32(sh, argv[1], &node_num);
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
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_packet.h"
#include "meshtastic_scanner.h"

/* Defined further down, alongside the `lora preset` command that also uses it.
 * Forward-declared rather than moved, so each stays next to its own command
 * group; sharing the parser is the point — the accepted preset spellings must
 * not drift between `lora preset` and `scan presets`. */
static int shell_parse_modem_preset(const char *name,
				    meshtastic_Config_LoRaConfig_ModemPreset *out);

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

	shell_print(sh, "sweeping: %s   tx: %s", meshtastic_scanner_sweeping() ? "yes" : "no",
		    meshtastic_scanner_active() ? "REFUSED (scanning)" : "allowed");
	shell_print(sh, "captured: %u total   withheld from stack: %u", total,
		    meshtastic_scanner_rx_dropped());
	if (blocked > 0U) {
		/* Not cosmetic: a non-zero count means some path kept trying to
		 * transmit while parked on a foreign frequency. The gate held, but
		 * something upstream should not have been asking. */
		shell_warn(sh, "tx refused while scanning: %u (expected 0 — investigate)",
			   blocked);
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

static int cmd_scan_dump(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_scan_record rec[8];
	uint32_t from = 0U;
	uint32_t want = 32U;
	uint32_t done = 0U;

	if (argc > 1) {
		want = (uint32_t)strtoul(argv[1], NULL, 0);
	}

	shell_print(sh, "%-10s %-12s %-10s %-10s %5s %4s %4s %5s %5s %5s %4s", "epoch", "preset",
		    "from", "to", "rssi", "snr", "hops", "strt", "relay", "chan", "len");

	while (done < want) {
		int n = meshtastic_scanner_records(rec, ARRAY_SIZE(rec), from);

		if (n <= 0) {
			break;
		}
		for (int i = 0; i < n && done < want; i++, done++) {
			shell_print(sh, "%-10u %-12s 0x%08x 0x%08x %5d %4d %4u %5u  0x%02x  0x%02x %4u",
				    rec[i].epoch_sec,
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
	return 0;
}

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
		(void)meshtastic_scanner_set_presets(NULL, 0U);
		shell_print(sh, "sweeping the full preset set (stats cleared)");
		return 0;
	}

	if (argc - 1U > ARRAY_SIZE(list)) {
		shell_error(sh, "at most %u presets", (unsigned int)ARRAY_SIZE(list));
		return -EINVAL;
	}

	for (size_t i = 1; i < argc; i++) {
		if (shell_parse_modem_preset(argv[i], &list[i - 1U]) < 0) {
			shell_error(sh, "unknown preset '%s'", argv[i]);
			return -EINVAL;
		}
	}

	if (meshtastic_scanner_set_presets(list, argc - 1U) < 0) {
		shell_error(sh, "set failed");
		return -EINVAL;
	}

	/* Narrowing the list shortens the cycle, so each remaining preset is listened
	 * to a larger fraction of the time — say so, since that is the point. */
	shell_print(sh, "sweeping %u presets (stats cleared; shorter cycle = higher capture "
			"rate on each)", (unsigned int)(argc - 1U));
	return 0;
}

static int cmd_scan_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_scanner_reset();
	shell_print(sh, "scan records and stats cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_scan_cmds,
	SHELL_CMD(start, NULL,
		  SHELL_HELP("Begin sweeping presets. Leaves the mesh; TX is refused.", NULL),
		  cmd_scan_start),
	SHELL_CMD(stop, NULL, SHELL_HELP("Stop sweeping and rejoin the mesh.", NULL),
		  cmd_scan_stop),
	SHELL_CMD(status, NULL, SHELL_HELP("Per-preset capture counts.", NULL), cmd_scan_status),
	SHELL_CMD(dump, NULL, SHELL_HELP("Print captured headers.", "[count]"), cmd_scan_dump),
	SHELL_CMD(presets, NULL,
		  SHELL_HELP("Show or limit which presets are swept.",
			     "[all | <name> ...]  e.g. scan presets LongFast ShortTurbo"),
		  cmd_scan_presets),
	SHELL_CMD(reset, NULL, SHELL_HELP("Clear captured records and stats.", NULL),
		  cmd_scan_reset),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_SCANNER */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_nodedb_cmds,
			       SHELL_CMD(list, NULL, SHELL_HELP("List NodeDB entries.", NULL),
					 cmd_nodedb_list),
			       SHELL_CMD(show, NULL,
					 SHELL_HELP("Show one NodeDB entry.", "<node|0xnode>"),
					 cmd_nodedb_show),
			       SHELL_CMD(warm, NULL,
					 SHELL_HELP("List warm key-tier entries (num + LRU recency).",
						    NULL),
					 cmd_nodedb_warm),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
			       SHELL_CMD(favorite, NULL,
					 SHELL_HELP("Mark a node favorite / not.",
						    "<node|0xnode> [on|off]"),
					 cmd_nodedb_favorite),
#endif
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_NODEDB */

#if defined(CONFIG_MESHTASTIC_MESSAGE)
static int cmd_text_send(const struct shell *sh, size_t argc, char **argv)
{
	struct shell_work_item item = {
		.op = SHELL_WORK_SEND_TEXT,
		.channel_index = MESHTASTIC_CHANNEL_INDEX_INVALID,
	};
	size_t msg_arg = 1U;
	int ret;
	int len;

	if (argc < 2U) {
		shell_error(sh,
			    "usage: meshtastic text send [-c <index>] [dest|broadcast] <message>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "-c") == 0) {
		if (argc < 4U) {
			shell_error(sh, "usage: meshtastic text send -c <index> [dest|broadcast] "
					"<message>");
			return -EINVAL;
		}
		ret = shell_parse_channel_index(sh, argv[2], &item.channel_index);
		if (ret < 0) {
			return ret;
		}
		msg_arg = 3U;
	}

	if (argc == (msg_arg + 1U)) {
		item.dest = MESHTASTIC_NODE_BROADCAST;
		len = append_message_from_argv(item.payload, sizeof(item.payload), argc, argv,
					       msg_arg);
	} else if (argc >= (msg_arg + 2U)) {
		ret = parse_u32(sh, argv[msg_arg], &item.dest);
		if (ret < 0) {
			return ret;
		}

		len = append_message_from_argv(item.payload, sizeof(item.payload), argc, argv,
					       msg_arg + 1U);
	} else {
		shell_error(sh, "message required");
		return -EINVAL;
	}

	if (len < 0) {
		shell_error(sh, "message too long (max %u)", MESHTASTIC_MAX_TEXT_LEN);
		return len;
	}

	if (len == 0) {
		shell_error(sh, "message must not be empty");
		return -EINVAL;
	}

	item.payload_len = (size_t)len;
	item.payload[len] = '\0';

	return enqueue_shell_work(sh, &item);
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_text_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send text message.",
						    "[-c <index>] [dest|broadcast] <message>"),
					 cmd_text_send),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_MESSAGE */

#if defined(CONFIG_MESHTASTIC_GNSS)
static int cmd_gnss_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, SHELL_WORK_SEND_POSITION);
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_gnss_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send GNSS position.", "[dest|broadcast]"),
					 cmd_gnss_send),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_GNSS */

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
static int cmd_metrics_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, SHELL_WORK_SEND_METRICS);
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_metrics_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send device metrics.", "[dest|broadcast]"),
					 cmd_metrics_send),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_DEVICE_METRICS */

#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
static int cmd_environment_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, SHELL_WORK_SEND_ENVIRONMENT);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_environment_cmds,
	SHELL_CMD(send, NULL, SHELL_HELP("Send environment telemetry.", "[dest|broadcast]"),
		  cmd_environment_send),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ENVIRONMENT_METRICS */

#if defined(CONFIG_MESHTASTIC_NODEINFO)
static int cmd_nodeinfo_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, SHELL_WORK_SEND_NODEINFO);
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_nodeinfo_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send node information.", "[dest|broadcast]"),
					 cmd_nodeinfo_send),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_NODEINFO */

static int cmd_sched_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_sched_config c;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_sched_snapshot(&c);

	shell_print(sh, "TX egress:");
	shell_print(sh, "  tx.order     %-9s [fifo|priority]",
		    meshtastic_sched_order_name(c.tx_order));
	shell_print(sh, "  tx.overflow  %-9s [drop-newest|drop-lowest]",
		    meshtastic_sched_overflow_name(c.tx_overflow));
	shell_print(sh, "  tx.depth     %u / %u max", c.tx_depth,
		    CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX);
	shell_print(sh, "Phone queue:");
	shell_print(sh, "  phone.evict  %-9s [drop-oldest|protect]",
		    meshtastic_sched_phone_evict_name(c.phone_evict));
	shell_print(sh, "Airtime / dedup:");
	if (c.airtime_max_util != 0U) {
		shell_print(sh, "  airtime.max  %u%%        [0=off, else max chan-util for BG TX]",
			    c.airtime_max_util);
	} else {
		shell_print(sh, "  airtime.max  off       [0=off, else max chan-util for BG TX]");
	}
	if (c.dedup_ttl_sec != 0U) {
		shell_print(sh, "  dedup.ttl    %us       [0=never expire] (cache %u entries)",
			    c.dedup_ttl_sec, CONFIG_MESHTASTIC_DUP_CACHE_SIZE);
	} else {
		shell_print(sh, "  dedup.ttl    off       [0=never expire] (cache %u entries)",
			    CONFIG_MESHTASTIC_DUP_CACHE_SIZE);
	}
	shell_print(sh, "Reliable delivery:");
	if (c.reliable_retries != 0U) {
		shell_print(sh, "  reliable.retries %u     [0=off]", c.reliable_retries);
		shell_print(sh, "  reliable.timeout %ums", c.reliable_timeout_ms);
	} else {
		shell_print(sh, "  reliable.retries off   [0=off]");
	}
	if (c.route_ttl_sec != 0U) {
		shell_print(sh, "  route.ttl    %us       [0=never expire]", c.route_ttl_sec);
	} else {
		shell_print(sh, "  route.ttl    off       [0=never expire]");
	}

	shell_print(sh, "Contention window:");
	if (c.cw_max != 0U) {
		shell_print(sh, "  cw.min/max   %u/%u       [pool = 1<<cw slots]", c.cw_min,
			    c.cw_max);
		shell_print(sh, "  cw.offset    %u         [client waits offset*cw.max slots]",
			    c.cw_relay_offset);
	} else {
		shell_print(sh, "  cw.max       off       [0 = transmit without waiting]");
	}
	if (c.cw_slot_ms != 0U) {
		shell_print(sh, "  cw.slot      %ums      [override; 0 = derive from preset]",
			    c.cw_slot_ms);
	} else {
		shell_print(sh, "  cw.slot      derived   [from the active modem preset]");
	}

	shell_fprintf(sh, SHELL_NORMAL, "presets:");
	for (int i = 0; meshtastic_sched_preset_name(i) != NULL; i++) {
		shell_fprintf(sh, SHELL_NORMAL, " %s", meshtastic_sched_preset_name(i));
	}
	shell_print(sh, "");
	return 0;
}

static int cmd_sched_policy(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic sched policy <name>");
		return -EINVAL;
	}

	ret = meshtastic_sched_apply_preset(argv[1]);
	if (ret == -ENOENT) {
		shell_error(sh, "unknown policy '%s'", argv[1]);
		return ret;
	}

	shell_print(sh, "policy '%s' applied (stats reset)", argv[1]);
	return 0;
}

static int cmd_sched_set(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc < 3U) {
		shell_error(sh, "usage: meshtastic sched set <key> <value>");
		shell_print(sh, "keys: tx.order tx.overflow tx.depth phone.evict "
				"airtime.max dedup.ttl reliable.retries reliable.timeout "
				"route.ttl cw.min cw.max cw.offset cw.slot");
		return -EINVAL;
	}

	ret = meshtastic_sched_set(argv[1], argv[2]);
	if (ret == -ENOENT) {
		shell_error(sh, "unknown key '%s'", argv[1]);
	} else if (ret == -EINVAL) {
		shell_error(sh, "bad value '%s' for %s", argv[2], argv[1]);
	} else {
		shell_print(sh, "%s = %s (stats reset)", argv[1], argv[2]);
	}
	return ret;
}

static int cmd_sched_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_sched_stats st;
	struct meshtastic_status status;

	if (argc >= 2U && strcmp(argv[1], "reset") == 0) {
		meshtastic_sched_stats_reset();
		meshtastic_radio_cad_agc_stats_reset();
		shell_print(sh, "sched stats reset");
		return 0;
	}

	meshtastic_sched_stats_get(&st);
	(void)meshtastic_get_status(&status);

	shell_print(sh, "RX   pkts %u  drop(queue) %u  dup %u  decode-fail %u", status.rx_packets,
		    status.rx_dropped, status.duplicate_packets, status.decode_failures);
	shell_print(sh, "TX   pkts %u  failed %u  relayed %u", status.tx_packets,
		    status.tx_failures, status.relayed_packets);
	shell_print(sh, "TX egress by tier (enqueued / dropped):");
	for (uint8_t t = 0; t < MT_SCHED_TIER_COUNT; t++) {
		shell_print(sh, "  %-6s %u / %u", meshtastic_sched_tier_name(t), st.tx_enq[t],
			    st.tx_drop[t]);
	}
	shell_print(sh, "outbound hi-water: %u / %u", st.ob_hiwater,
		    CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX);
	shell_print(sh, "phone FromRadio drops: %u (%u protected)", st.phone_drop,
		    st.phone_drop_protected);
	shell_print(sh, "airtime-gated BG broadcasts: %u", st.tx_airtime_drop);
	shell_print(sh, "dedup TTL expiries: %u", st.dedup_expired);
	shell_print(sh, "reliable delivery: %u acked, %u failed", st.reliable_acked,
		    st.reliable_failed);

	/* CAD (listen-before-talk) and periodic AGC-reset outcomes -- driver-owned
	 * counters (sx126x.c), zero on a build without CONFIG_LORA_SX126X. A
	 * nonzero "timeout" here specifically is worth watching: it is the exact
	 * symptom of the DIO1/irq_work race G-3 fixed (2026-08-06) -- see
	 * meshtastic_radio.c and zephyr/patches/0005-*. */
	shell_print(sh, "CAD: clear %u  busy %u  timeout %u  error %u",
		    meshtastic_radio_cad_clear_count(), meshtastic_radio_cad_busy_count(),
		    meshtastic_radio_cad_timeout_count(), meshtastic_radio_cad_error_count());
	shell_print(sh, "AGC reset: ok %u  fail %u", meshtastic_radio_agc_reset_ok_count(),
		    meshtastic_radio_agc_reset_fail_count());
	if (meshtastic_radio_agc_reset_skipped_count() > 0U ||
	    meshtastic_radio_agc_patch_fail_count() > 0U) {
		shell_print(sh, "  skipped (TX in flight) %u  sensitivity-patch re-apply failed %u",
			    meshtastic_radio_agc_reset_skipped_count(),
			    meshtastic_radio_agc_patch_fail_count());
	}

	/* Flood redundancy: how many of our relays a peer also relayed, and how
	 * soon after ours theirs arrived. Gaps inside a plausible contention window
	 * are the transmissions a delay + overhear-cancel would have saved. */
	if (st.relay_deferred_late > 0U) {
		shell_print(sh, "relays deferred to the late window: %u", st.relay_deferred_late);
	}
	shell_print(sh, "relays cancelled on overhear: %u (transmitted %u of %u queued)",
		    st.relay_cancelled, st.relay_sent - MIN(st.relay_cancelled, st.relay_sent),
		    st.relay_sent);
	shell_print(sh, "relays sent: %u  also relayed by a peer: %u (%u%%)", st.relay_sent,
		    st.relay_redundant,
		    st.relay_sent ? (unsigned int)((st.relay_redundant * 100U) / st.relay_sent) : 0U);
	if (st.relay_redundant > 0U) {
		shell_fprintf(sh, SHELL_NORMAL, "  peer-relay gap after ours:");
		for (int i = 0; i < MT_RELAY_GAP_BUCKETS; i++) {
			if (i == MT_RELAY_GAP_BUCKETS - 1) {
				shell_fprintf(sh, SHELL_NORMAL, " >=%ums:%u",
					      meshtastic_relay_gap_bounds[i - 1], st.relay_gap[i]);
			} else {
				shell_fprintf(sh, SHELL_NORMAL, " <%ums:%u",
					      meshtastic_relay_gap_bounds[i], st.relay_gap[i]);
			}
		}
		shell_print(sh, "");
	}
	return 0;
}

static int cmd_sched_defaults(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_sched_defaults();
	shell_print(sh, "sched reverted to compiled defaults");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_sched_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Show scheduler policy and knobs.", NULL), cmd_sched_show),
	SHELL_CMD(policy, NULL, SHELL_HELP("Apply a named policy preset.", "<name>"),
		  cmd_sched_policy),
	SHELL_CMD(set, NULL, SHELL_HELP("Set one knob.", "<key> <value>"), cmd_sched_set),
	SHELL_CMD(stats, NULL, SHELL_HELP("Show or reset live counters.", "[reset]"),
		  cmd_sched_stats),
	SHELL_CMD(defaults, NULL, SHELL_HELP("Revert to compiled defaults.", NULL),
		  cmd_sched_defaults),
	SHELL_SUBCMD_SET_END);


#if defined(CONFIG_NETWORKING)
/*
 * netpause <secs> — take the network fully down for a bounded window, then bring it back
 * up unattended.
 *
 * This exists because every network-based way of observing light sleep perturbs it: a
 * telnet query is WiFi traffic, and WiFi traffic makes the esp_pm "wifi" lock (and the
 * whole net/MQTT/syslog stack) wake the SoC. Residency read while connected therefore
 * measures the measurement. Here the shell thread sleeps (so the CPU may idle), the
 * network is genuinely off, and because the PM residency counters are cumulative you can
 * read them afterwards for an UNPERTURBED figure.
 *
 * Best driven over the USB/serial console: the session survives the network going down.
 * Over telnet it still works, but the session drops and you reconnect after `secs`.
 */
static int cmd_net_pause(const struct shell *sh, size_t argc, char **argv)
{
	struct net_if *iface = net_if_get_default();
	uint32_t secs;
	char *end;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic netpause <secs>");
		return -EINVAL;
	}

	secs = (uint32_t)strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs == 0U || secs > 600U) {
		shell_error(sh, "secs must be 1..600");
		return -EINVAL;
	}

	if (iface == NULL) {
		shell_error(sh, "no default network interface");
		return -ENODEV;
	}

	shell_print(sh, "network DOWN for %u s (telnet sessions will drop)...", secs);
	(void)net_if_down(iface);

	k_sleep(K_SECONDS(secs));

	(void)net_if_up(iface);
	shell_print(sh, "network back UP; read residency with `stats list`");

	return 0;
}
#endif /* CONFIG_NETWORKING */

#if defined(CONFIG_PM)
/*
 * power [on|off] — inspect or set the light-sleep power-saving policy.
 *
 * No argument prints the governor's live state: the stored PowerConfig fields
 * (is_power_saving / min_wake_secs / wait_bluetooth_secs), the live inhibitor mask
 * (STANDBY is blocked while any bit is set — an empty mask means the node is free to
 * sleep), and the cumulative light-sleep entry count. That last pair is the answer
 * to "why isn't it sleeping": a WiFi node shows [WIFI] while it holds a lease, and a
 * saving=off node shows [POLICY].
 *
 * `on` / `off` writes is_power_saving into the config store and applies it live via
 * the same meshtastic_power_config_apply() the admin PhoneAPI PowerConfig write
 * calls — no reboot. This is the only way to toggle saving on a BLE-only node with
 * no phone paired, and it persists across reboots.
 */
static int cmd_power_show(const struct shell *sh)
{
	meshtastic_Config config;
	bool saving = IS_ENABLED(CONFIG_MESHTASTIC_POWER_SAVE_DEFAULT);
	uint32_t min_wake = 0U;
	uint32_t wait_bt = 0U;
	bool stored = false;
	uint32_t inh;
	bool locked;
	char inhbuf[64];

	if (meshtastic_config_store_get_config(meshtastic_Config_power_tag, &config) == 0 &&
	    config.which_payload_variant == meshtastic_Config_power_tag) {
		saving = config.payload_variant.power.is_power_saving;
		min_wake = config.payload_variant.power.min_wake_secs;
		wait_bt = config.payload_variant.power.wait_bluetooth_secs;
		stored = true;
	}

	inh = meshtastic_power_inhibitors();
	meshtastic_power_inhibitors_str(inh, inhbuf, sizeof(inhbuf));
	/* Ground truth: is PM_STATE_STANDBY actually locked by ANYONE? This catches the
	 * esp-idf radio-controller locks (WiFi APB / BLE controller) that route through
	 * the HAL's pm_policy_state_all_lock_get() and are INVISIBLE to the governor
	 * mask above -- the reason a BLE node reads inhibitors 0x00 yet never sleeps. */
	locked = pm_policy_state_lock_is_active(PM_STATE_STANDBY, PM_ALL_SUBSTATES);

	shell_print(sh, "power saving:        %s%s", saving ? "on" : "off",
		    stored ? "" : " (compiled default; no PowerConfig stored)");
	shell_print(sh, "min_wake_secs:       %u%s", min_wake,
		    min_wake == 0U ? "  (activity wake window disabled)" : "");
	shell_print(sh, "wait_bluetooth_secs: %u%s", wait_bt,
		    wait_bt == 0U ? "  (boot pairing window disabled)" : "");
	shell_print(sh, "governor inhibitors: 0x%02x [%s]", inh, inhbuf);
	if (!locked) {
		shell_print(sh, "STANDBY lock:        clear -- SoC free to light-sleep");
	} else if (inh != 0U) {
		shell_print(sh, "STANDBY lock:        held (governor inhibiting; see mask above)");
	} else {
		shell_print(sh, "STANDBY lock:        held by esp-idf radio controller "
				"(WiFi/BLE), NOT the governor");
	}
	shell_print(sh, "light-sleep entries: %u%s", meshtastic_powermon_sleep_count(),
		    meshtastic_powermon_sleep_count() == 0U
			    ? "  (0 == SoC has never light-slept; expected while a radio "
			      "controller holds STANDBY)"
			    : "");
	if (meshtastic_power_manual_inhibit()) {
		shell_print(sh, "manual pm hold:      ENGAGED -- pinned awake by `meshtastic pm "
				"off` (volatile; cleared on reboot)");
	}
	return 0;
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_power_set(const struct shell *sh, bool on)
{
	meshtastic_Config config;
	int ret;

	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_power_tag, &config);
	if (ret < 0) {
		shell_error(sh, "power get failed: %d", ret);
		return ret;
	}

	/* get_config returns the slot with which_payload_variant already set to the
	 * power tag; touch only the field we own and write the whole PowerConfig back,
	 * exactly as the admin set_config path does. */
	config.which_payload_variant = meshtastic_Config_power_tag;
	config.payload_variant.power.is_power_saving = on;

	ret = meshtastic_config_store_set_config(&config);
	if (ret < 0) {
		shell_error(sh, "power set failed: %d", ret);
		return ret;
	}

	/* Apply live -- the same call the admin PowerConfig write makes, so the POLICY
	 * inhibitor flips now, no reboot. */
	meshtastic_power_config_apply();

	shell_print(sh, "power saving %s (applied live, persisted)", on ? "on" : "off");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		return cmd_power_show(sh);
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic power [on|off]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	if (strcmp(argv[1], "on") == 0) {
		return cmd_power_set(sh, true);
	}
	if (strcmp(argv[1], "off") == 0) {
		return cmd_power_set(sh, false);
	}

	shell_error(sh, "expected on or off, got %s", argv[1]);
	return -EINVAL;
#endif
}

/*
 * pm [on|off] — a volatile debug hold that pins the node awake, distinct from
 * `power`.
 *
 *   pm off  disables light sleep now (holds the governor's MANUAL inhibitor).
 *   pm on   re-enables it (releases the hold; sleep is governed by config again).
 *   pm      reports whether the hold is engaged.
 *
 * Why this is separate from `power on|off`:
 *   - `power off` writes is_power_saving=false into the PERSISTED PowerConfig (a
 *     config write: it survives reboot, is refused on a managed node, and a phone
 *     can flip it back). It expresses the node's power-saving *preference*.
 *   - `pm off` sets a VOLATILE inhibitor only. It persists nothing, is cleared on
 *     reboot, touches no config, and is never refused — because ruling light sleep
 *     in or out for a debugging session is an observability action, not a
 *     configuration one. Use it to pin a bench node awake without disturbing its
 *     stored config or fighting an attached phone.
 *
 * Note this holds only the GOVERNOR's lock; on a radio-up node the esp-idf WiFi/BLE
 * controller already pins STANDBY independently (see `power`), so there the hold is
 * belt-and-suspenders. It bites where the governor is the deciding vote (radio down,
 * e.g. under `netpause`, or a non-radio build).
 */
static int cmd_pm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		shell_print(sh, "manual pm hold: %s",
			    meshtastic_power_manual_inhibit()
				    ? "engaged (light sleep disabled; pinned awake)"
				    : "released (light sleep governed by power config)");
		shell_print(sh, "(`meshtastic power` shows the full governor + STANDBY-lock "
				"state)");
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic pm [on|off]");
		return -EINVAL;
	}

	if (strcmp(argv[1], "off") == 0) {
		meshtastic_power_set_manual_inhibit(true);
		shell_print(sh, "light sleep DISABLED (node pinned awake; volatile, cleared on "
				"reboot)");
		return 0;
	}
	if (strcmp(argv[1], "on") == 0) {
		meshtastic_power_set_manual_inhibit(false);
		shell_print(sh, "light sleep RE-ENABLED (governed by power config)");
		return 0;
	}

	shell_error(sh, "expected on or off, got %s", argv[1]);
	return -EINVAL;
}
#endif /* CONFIG_PM */

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

	if (shell_config_write_refused(sh)) {
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
#endif /* CONFIG_MESHTASTIC_BLE && CONFIG_MESHTASTIC_TCP */

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
		drive = meshtastic_tx_power_chip_drive(radiated);

		shell_print(sh, "tx power:     %d dBm at the antenna%s (radio drive %d dBm%s)",
			    radiated,
			    (cfg.payload_variant.lora.tx_power == 0) ? " [region default]" : "",
			    drive, (drive != radiated) ? ", FEM makes up the rest" : "");
	}

	/* The other two knobs a config tool can set that change what the radio
	 * does. Shown because "stored but not doing anything" is exactly the
	 * failure these two used to have. */
	shell_print(sh, "tx enabled:   %s",
		    cfg.payload_variant.lora.tx_enabled ? "yes" : "NO — receive only");
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
/* Match a preset display name (as reported by meshtastic_preset_display_name, so
 * the accepted spellings always equal what `meshtastic lora` shows). */
static int shell_parse_modem_preset(const char *s,
				    meshtastic_Config_LoRaConfig_ModemPreset *out)
{
	static const meshtastic_Config_LoRaConfig_ModemPreset presets[] = {
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE,
		meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
	};

	for (size_t i = 0U; i < ARRAY_SIZE(presets); i++) {
		if (strcmp(s, meshtastic_preset_display_name(presets[i], true)) == 0) {
			*out = presets[i];
			return 0;
		}
	}
	return -EINVAL;
}

static int cmd_lora_preset_set(const struct shell *sh, const char *name)
{
	meshtastic_Config cfg;
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	int ret;

	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}
	if (shell_parse_modem_preset(name, &preset) < 0) {
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
	/* set_config schedules the coalesced save (like `channel set`); it lands well
	 * before a manual reboot, so no explicit flush is needed here. */

	shell_print(sh, "modem preset -> %s (persisted); reboot to apply "
			"(`kernel reboot cold`)",
		    meshtastic_preset_display_name(preset, true));
	return 0;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

static int cmd_lora(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		cmd_lora_show(sh);
		return 0;
	}
	if (argc != 3U || strcmp(argv[1], "preset") != 0) {
		shell_error(sh, "usage: meshtastic lora [preset <name>]");
		return -EINVAL;
	}
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	return cmd_lora_preset_set(sh, argv[2]);
#endif
}

#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
/* Remote syslog no longer starts itself -- see the rationale on
 * CONFIG_LOG_BACKEND_NET_AUTOSTART in overlay-v4-unified.conf. It stays compiled
 * so a soak that genuinely wants continuous remote capture can turn it on for
 * that boot without a reflash. Deliberately NOT persisted: the default has to
 * stay "off" across a reset, or the reasons it was turned off quietly reassert
 * themselves the next time a node reboots unattended.
 */
static int cmd_netlog(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || strcmp(argv[1], "on") != 0) {
		shell_print(sh, "usage: meshtastic netlog on");
		shell_print(sh, "  (off by default; not persisted across a reboot)");
		return 0;
	}

	log_backend_net_start();
	shell_print(sh, "netlog: started for this boot only");

	return 0;
}
#endif

#if defined(CONFIG_MESHTASTIC_DFU_TRIGGER)
static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	bool serial_only = (argc >= 2) && (strcmp(argv[1], "serial") == 0);

	if (argc >= 2 && !serial_only) {
		shell_error(sh, "usage: dfu [serial]  (default: UF2 mode, drive + serial DFU)");
		return -EINVAL;
	}

	shell_print(sh, "rebooting into the bootloader (%s mode) — this console goes away now",
		    serial_only ? "serial-DFU" : "UF2");
	/* Give the shell a beat to push the line out the CDC pipe. */
	k_sleep(K_MSEC(100));
	meshtastic_dfu_enter(serial_only);
	return 0; /* unreachable */
}
#endif

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
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
	shell_print(sh, "scan          : %s, adverts matched=%u reflections=%u",
		    meshtastic_ble_peer_scan_armed() ? "armed" : "off", ps.adverts_matched,
		    ps.reflections);
	shell_print(sh, "connects      : attempted=%u failed=%u discovery_failures=%u",
		    ps.connects_attempted, ps.connects_failed, ps.discovery_failures);
	if (link.connected) {
		shell_print(sh, "outbound      : 0x%08x %s (slot %u, tx beats %u)", link.node_num,
			    link.ready ? "READY" : "connecting", link.index, link.tx_beats);
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
			shell_print(sh,
				    "slot %u: %-12s %s age=%llds rx beats=%u lost=%u resyncs=%u "
				    "from=0x%08x last=%llds ago",
				    i, blepeer_kind_str(kind), addr,
				    (long long)(age_ms / 1000), rx.beats, rx.lost, rx.resyncs,
				    rx.last.node_num,
				    (long long)((k_uptime_get() - last_ms) / 1000));
		} else {
			shell_print(sh, "slot %u: %-12s %s age=%llds%s", i, blepeer_kind_str(kind),
				    addr, (long long)(age_ms / 1000),
				    meshtastic_ble_reg_phone_noted(i) ? " (phone note held)" : "");
		}
	}
	return 0;
}

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

static int cmd_blepeer_beat(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_ble_peer_beat_now();
	shell_print(sh, "beat scheduled on every active link (none = no-op; see blepeer status)");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_blepeer_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Slots, ledger, classification and link counters.", NULL),
		  cmd_blepeer_status),
	SHELL_CMD(scan, NULL, SHELL_HELP("Arm/disarm the passive peer scanner.", "[on|off]"),
		  cmd_blepeer_scan),
	SHELL_CMD(list, NULL, SHELL_HELP("Peers heard advertising.", NULL), cmd_blepeer_list),
	SHELL_CMD(connect, NULL,
		  SHELL_HELP("Scan for and connect to a peer.", "[node-num-hex]"),
		  cmd_blepeer_connect),
	SHELL_CMD(disconnect, NULL, SHELL_HELP("Drop the outbound peer link.", NULL),
		  cmd_blepeer_disconnect),
	SHELL_CMD(beat, NULL, SHELL_HELP("Send one heartbeat now on every active link.", NULL),
		  cmd_blepeer_beat),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_BLE_PEER */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Show Meshtastic status.", NULL), cmd_status),
	SHELL_CMD(version, NULL, SHELL_HELP("Show build id / firmware version.", NULL),
		  cmd_version),
#if defined(CONFIG_NETWORKING)
	SHELL_CMD(netpause, NULL,
		  SHELL_HELP("Take the network down for N seconds, then restore it. "
			     "Lets PM residency be measured without network traffic waking "
			     "the SoC.", "<secs>"),
		  cmd_net_pause),
#endif
#if defined(CONFIG_PM)
	SHELL_CMD(power, NULL,
		  SHELL_HELP("Show or set the light-sleep power-saving policy.", "[on|off]"),
		  cmd_power),
	SHELL_CMD(pm, NULL,
		  SHELL_HELP("Volatile bench hold: pin the node awake (off) or release (on). "
			     "Not persisted; distinct from `power`.",
			     "[on|off]"),
		  cmd_pm),
#endif
#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
	SHELL_CMD(transport, NULL,
		  SHELL_HELP("Show or set the phone transport, reboot to apply (unified image).",
			     "[ble|wifi]"),
		  cmd_transport),
#endif
	SHELL_CMD(sched, &meshtastic_sched_cmds,
		  SHELL_HELP("Scheduler / QoS policy commands.", NULL), cmd_sched_show),
	SHELL_CMD(channel, &meshtastic_channel_cmds, SHELL_HELP("Channel table commands.", NULL),
		  NULL),
	SHELL_CMD(device, &meshtastic_device_cmds,
		  SHELL_HELP("Device role and rebroadcast commands.", NULL), NULL),
	SHELL_CMD(lora, NULL,
		  SHELL_HELP("Show or set the LoRa modem preset (reboot to apply).",
			     "[preset <name>]"),
		  cmd_lora),
#if defined(CONFIG_MESHTASTIC_MESSAGE)
	SHELL_CMD(text, &meshtastic_text_cmds, SHELL_HELP("Text message commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB)
	SHELL_CMD(nodedb, &meshtastic_nodedb_cmds, SHELL_HELP("NodeDB commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_GNSS)
	SHELL_CMD(gnss, &meshtastic_gnss_cmds, SHELL_HELP("GNSS commands.", NULL), NULL),
#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
	SHELL_CMD(netlog, NULL,
		  SHELL_HELP("Start the remote syslog backend for this boot.",
			     "netlog on"),
		  cmd_netlog),
#endif
#endif
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
	SHELL_CMD(metrics, &meshtastic_metrics_cmds, SHELL_HELP("Device metrics commands.", NULL),
		  NULL),
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
	SHELL_CMD(environment, &meshtastic_environment_cmds,
		  SHELL_HELP("Environment telemetry commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_NODEINFO)
	SHELL_CMD(nodeinfo, &meshtastic_nodeinfo_cmds, SHELL_HELP("NodeInfo commands.", NULL),
		  NULL),
#endif
#if defined(CONFIG_MESHTASTIC_SCANNER)
	SHELL_CMD(scan, &meshtastic_scan_cmds,
		  SHELL_HELP("Multi-preset survey. Leaves the mesh while running.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	SHELL_CMD(blepeer, &meshtastic_blepeer_cmds,
		  SHELL_HELP("Node-to-node BLE peer link.", NULL), cmd_blepeer_status),
#endif
#if defined(CONFIG_MESHTASTIC_DFU_TRIGGER)
	SHELL_CMD(dfu, NULL,
		  SHELL_HELP("Reboot into the bootloader for reflashing.", "[serial]"),
		  cmd_dfu),
#endif
#if defined(CONFIG_MESHTASTIC_RF_PATH_REPORT) && defined(CONFIG_MESHTASTIC_RF_HIST)
	/* Bare `meshtastic rf` runs the gain-path report; the subcommands cover
	 * measurement. Both halves are independently configurable, hence three
	 * cases rather than one. */
	SHELL_CMD(rf, &meshtastic_rf_cmds,
		  SHELL_HELP("Gain path in signal order, plus signal measurement.", NULL),
		  cmd_rf_path),
#elif defined(CONFIG_MESHTASTIC_RF_PATH_REPORT)
	SHELL_CMD(rf, NULL,
		  SHELL_HELP("Gain path in signal order: what the radio is REALLY doing.", NULL),
		  cmd_rf_path),
#elif defined(CONFIG_MESHTASTIC_RF_HIST)
	SHELL_CMD(rf, &meshtastic_rf_cmds,
		  SHELL_HELP("Signal measurement: distributions and per-peer rates.", NULL),
		  NULL),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(meshtastic, &meshtastic_cmds,
		   SHELL_HELP("Meshtastic mesh radio commands.", NULL), NULL);

static int meshtastic_shell_init(void)
{
	k_thread_create(&shell_work_thread, shell_work_stack,
			K_THREAD_STACK_SIZEOF(shell_work_stack), shell_work_thread_fn, NULL, NULL,
			NULL, CONFIG_MESHTASTIC_SHELL_WORK_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&shell_work_thread, "meshtastic_shell");

	return 0;
}

SYS_INIT(meshtastic_shell_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
