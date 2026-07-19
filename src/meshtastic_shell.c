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

#include <zephyr/meshtastic/gnss.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include <zephyr/meshtastic/telemetry.h>

#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_sched.h"

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

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Show Meshtastic status.", NULL), cmd_status),
	SHELL_CMD(sched, &meshtastic_sched_cmds,
		  SHELL_HELP("Scheduler / QoS policy commands.", NULL), cmd_sched_show),
	SHELL_CMD(channel, &meshtastic_channel_cmds, SHELL_HELP("Channel table commands.", NULL),
		  NULL),
	SHELL_CMD(device, &meshtastic_device_cmds,
		  SHELL_HELP("Device role and rebroadcast commands.", NULL), NULL),
#if defined(CONFIG_MESHTASTIC_MESSAGE)
	SHELL_CMD(text, &meshtastic_text_cmds, SHELL_HELP("Text message commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB)
	SHELL_CMD(nodedb, &meshtastic_nodedb_cmds, SHELL_HELP("NodeDB commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_GNSS)
	SHELL_CMD(gnss, &meshtastic_gnss_cmds, SHELL_HELP("GNSS commands.", NULL), NULL),
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
