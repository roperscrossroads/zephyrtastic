/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/meshtastic/bootlog.h>
#include <zephyr/meshtastic/diagnostics.h>
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
#include "meshtastic_admin_client.h"
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include <zephyr/bluetooth/addr.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#endif
#if defined(CONFIG_MESHTASTIC_DFU_TRIGGER)
#include "meshtastic_dfu_trigger.h"
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#if defined(CONFIG_MESHTASTIC_FLEET)
#include "meshtastic_fleet.h"
#if defined(CONFIG_MESHTASTIC_DEPOT)
#include "meshtastic_smp_central.h"
#endif
#endif
#endif
#include "meshtastic_build.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_clock_persist.h"
#include "meshtastic_hlc.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_powermon.h"
#include <zephyr/meshtastic/fem.h>

#include "meshtastic_tx_power.h"
#include "meshtastic_sched.h"
#include "meshtastic_airtime.h"
#include "meshtastic_duty.h"
#include "meshtastic_phonelog.h"
#include "meshtastic_settings.h"
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
#include "meshtastic_telemetry_internal.h"
#endif

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
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static bool shell_config_write_refused(const struct shell *sh);
#endif

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

	/* The orderable one: the MCUboot image header of the running slot. This
	 * is what a peer's SMP client compares against, and what the fleet
	 * orders by. Absent on an image MCUboot does not manage. */
	struct meshtastic_image_version v;
	int rc = meshtastic_image_version(&v);

	if (rc == 0) {
		shell_print(sh, "image:  %u.%u.%u+%u (MCUboot header, slot0)", v.major, v.minor,
			    v.revision, v.build);
	} else if (rc == -ENOTSUP) {
		shell_print(sh, "image:  n/a (not an MCUboot-managed image)");
	} else {
		shell_print(sh, "image:  header read failed (%d)", rc);
	}
	shell_print(sh, "class:  %u%s", CONFIG_MESHTASTIC_FLEET_CLASS,
		    CONFIG_MESHTASTIC_FLEET_CLASS == 0 ? " (unset: no courier will offer this node an image)"
						       : "");
	return 0;
}

static const char *clock_quality_name(enum meshtastic_clock_quality q)
{
	switch (q) {
	case MESHTASTIC_CLOCK_QUALITY_NONE:
		return "NONE (never set)";
	case MESHTASTIC_CLOCK_QUALITY_DEVICE:
		return "device RTC";
	case MESHTASTIC_CLOCK_QUALITY_NET:
		return "mesh-relayed";
	case MESHTASTIC_CLOCK_QUALITY_NTP:
		return "NTP/operator";
	case MESHTASTIC_CLOCK_QUALITY_GPS:
		return "GPS";
	default:
		return "?";
	}
}

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

		if (shell_config_write_refused(sh)) {
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

#if defined(CONFIG_MESHTASTIC_SHELL_CRASHTEST)
/*
 * Deliberately fault, so the crash-capture pipeline can be proven on demand
 * rather than discovered broken by the bug it was meant to catch. See
 * CONFIG_MESHTASTIC_SHELL_CRASHTEST for why this exists.
 */

static volatile uint32_t crashtest_sink;

/*
 * Recurse for real, until the stack runs out.
 *
 * The obvious version of this does not work, and failed silently on the bench
 * before this comment existed: marking the locals `volatile` and using the
 * result is NOT enough, because the compiler can still see that the recursion
 * is self-contained and turn the whole thing into a LOOP. A loop never grows
 * the stack, so instead of overflowing, the node simply spins in this thread
 * forever -- the console stops accepting input while everything else keeps
 * running, which looks nothing like the fault being reproduced. The
 * disassembly gave it away: no `entry` instruction, no call, just a branch.
 *
 * So the recursive call goes through a VOLATILE function pointer. The compiler
 * must reload it every iteration and cannot prove what it points at, so it can
 * neither inline the call nor convert it to a loop, and each level gets a real
 * stack frame. `pad` is written and read so the frame cannot be elided either.
 */
static uint32_t crashtest_recurse(uint32_t depth);
static uint32_t (*volatile crashtest_indirect)(uint32_t) = crashtest_recurse;

__attribute__((noinline)) static uint32_t crashtest_recurse(uint32_t depth)
{
	volatile uint32_t pad[16];

	pad[depth % ARRAY_SIZE(pad)] = depth;
	crashtest_sink = pad[depth % ARRAY_SIZE(pad)];

	return pad[0] + crashtest_indirect(depth + 1);
}

static int cmd_crashtest(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic crashtest <panic|stack>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "panic") == 0) {
		shell_print(sh, "crashtest: panicking on purpose");
		/* Let the line reach the console before the node stops running. */
		k_sleep(K_MSEC(200));
		k_panic();
	} else if (strcmp(argv[1], "stack") == 0) {
		shell_print(sh, "crashtest: overflowing this thread's stack on purpose");
		k_sleep(K_MSEC(200));
		crashtest_sink = crashtest_recurse(0);
	} else {
		shell_error(sh, "unknown kind '%s' (want panic or stack)", argv[1]);
		return -EINVAL;
	}

	/* Unreachable: both paths above are fatal. If control ever arrives here the
	 * fatal path did NOT fire, which is itself the finding -- say so rather
	 * than returning success. */
	shell_error(sh, "crashtest: still running — the fatal path did not fire");
	return -EIO;
}
#endif /* CONFIG_MESHTASTIC_SHELL_CRASHTEST */

static int cmd_time(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		uint32_t epoch = meshtastic_clock_now_epoch();
		enum meshtastic_clock_quality q = meshtastic_clock_get_quality();

		if (q == MESHTASTIC_CLOCK_QUALITY_NONE) {
			shell_print(sh, "clock: UNSET (epoch 0; logs run on uptime) — "
					"`meshtastic time set <unix-epoch-seconds>`");
			return 0;
		}
		shell_print(sh, "clock: epoch %u, source %s", epoch, clock_quality_name(q));
#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
		{
			int32_t ppb;
			uint32_t window_s;

			/* Worth a line of its own: this is the only place the board's
			 * measured oscillator error is visible, and it is a health metric
			 * as much as a clock one — an estimate that wanders between reads
			 * is a hardware problem nothing else here would show. */
			if (meshtastic_clock_skew(&ppb, &window_s)) {
				shell_print(sh, "       skew %d ppb (%d.%03d ppm) over %u s",
					    ppb, ppb / 1000, (ppb < 0 ? -ppb : ppb) % 1000,
					    window_s);
			} else {
				shell_print(sh,
					    "       skew: not yet measured (%u s of the %d s "
					    "window)",
					    window_s,
					    CONFIG_MESHTASTIC_CLOCK_SKEW_MIN_WINDOW_S);
			}
		}
#endif
#if defined(CONFIG_MESHTASTIC_CLOCK_PERSIST)
		{
			enum meshtastic_clock_persist_result pr;
			uint32_t down_ms;

			/* Worth surfacing rather than leaving implicit: a restored clock
			 * reads exactly like a synced one, and the difference matters —
			 * it is only as good as the epoch that was saved plus the
			 * counter's estimate of the gap, and it deliberately carries
			 * DEVICE quality so a real source can still replace it. */
			meshtastic_clock_persist_status(&pr, &down_ms);
			if (pr == MESHTASTIC_CLOCK_PERSIST_RESTORED) {
				shell_print(sh, "       %s (%u ms gap)",
					    meshtastic_clock_persist_result_str(pr), down_ms);
			} else if (pr != MESHTASTIC_CLOCK_PERSIST_COLD) {
				shell_print(sh, "       persist: %s",
					    meshtastic_clock_persist_result_str(pr));
			}
		}
#endif
		return 0;
	}
	if (argc != 3U || strcmp(argv[1], "set") != 0) {
		shell_error(sh, "usage: meshtastic time [set <unix-epoch-seconds>]");
		return -EINVAL;
	}
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	unsigned long epoch = strtoul(argv[2], NULL, 10);

	/* Validate here with the setter's own bounds: the setter silently ignores
	 * an out-of-window value, and "silently" is wrong for an interactive
	 * command (the clock may still hold an older good value, so its quality
	 * cannot reveal the rejection). */
	if (epoch < MESHTASTIC_EPOCH_MIN || epoch >= MESHTASTIC_EPOCH_MAX) {
		shell_error(sh, "rejected: outside the sane window [%u, %llu)",
			    MESHTASTIC_EPOCH_MIN, (unsigned long long)MESHTASTIC_EPOCH_MAX);
		return -EINVAL;
	}

	meshtastic_clock_set_epoch((uint32_t)epoch, MESHTASTIC_CLOCK_QUALITY_NTP);
	/* The ladder may have kept a better source (e.g. a GPS fix) — report what
	 * the clock actually holds rather than claiming the write won. */
	shell_print(sh, "clock now: epoch %u, source %s", meshtastic_clock_now_epoch(),
		    clock_quality_name(meshtastic_clock_get_quality()));
	return 0;
#endif
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

	/* An EMPTY name means "the default channel for the active preset", and
	 * get_name() substitutes that preset name because it is protocol data —
	 * hashed for the channel byte and the frequency slot. On a DISABLED slot
	 * that substitution is just misleading: it makes an unused slot read as a
	 * configured channel with a real name. Report what is stored instead.
	 *
	 * Found 2026-08-25 when a phone showed three channels — 0 ShortTurbo,
	 * 1 ShortTurbo, 2 cluster — and slot 1 turned out to be a disabled slot
	 * with no name, no PSK and hash 0x00. */
	name = (ch->role == meshtastic_Channel_Role_DISABLED &&
		(!ch->has_settings || ch->settings.name[0] == '\0'))
		       ? "(unset)"
		       : meshtastic_channels_get_name(index);
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
			    node->num, heard, scaled_whole(snr, 10), scaled_fraction(snr, 10),
			    node->hops_away, node->via_mqtt ? "yes" : "no", long_name,
			    short_name);
	} else {
		shell_print(sh, "0x%08x last=%s snr=%d.%u hops=? via=%s long=\"%s\" short=\"%s\"",
			    node->num, heard, scaled_whole(snr, 10), scaled_fraction(snr, 10),
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
 * not drift between `lora preset`, `scan presets` and `preset hop`. */
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
	if (p.fem_name != NULL) {
		shell_print(sh, "  [ok] front-end       %s", p.fem_name);
	} else {
		/* Two very different situations share this line on a board that
		 * detects at runtime, so say both: with no FEM the weak identity
		 * conversion is correct, but a FAILED detection also lands here and
		 * silently under-drives transmit power. */
		shell_print(sh, "  [--] front-end       none fitted, or detection did not "
				"complete");
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
		if (p.tx_fem_gain_applied) {
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
	shell_print(sh, "  [%s] AGC reset        ok %u  fail %u  skipped %u  patch-fail %u",
		    (p.agc_fail == 0U && p.agc_patch_fail == 0U) ? "ok" : "!!", p.agc_ok,
		    p.agc_fail, p.agc_skipped, p.agc_patch_fail);
	shell_print(sh, "  [%s] SPI BUSY streak  %u", (p.busy_streak == 0U) ? "ok" : "!!",
		    p.busy_streak);

	return 0;
}
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

static int cmd_rf_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_rf_reset();
	shell_print(sh, "measurement window restarted (lifetime count kept)");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_RF_HIST */

#if defined(CONFIG_MESHTASTIC_RF_HIST)
SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_rf_cmds,
	SHELL_CMD(hist, NULL, SHELL_HELP("RSSI/SNR distribution for this window.", NULL),
		  cmd_rf_hist),
	SHELL_CMD(peers, NULL, SHELL_HELP("Per-peer frame counts and link quality.", NULL),
		  cmd_rf_peers),
	SHELL_CMD(reset, NULL, SHELL_HELP("Start a new measurement window.", NULL),
		  cmd_rf_reset),
	SHELL_SUBCMD_SET_END);
#endif

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
			       SHELL_CMD_ARG(forget, NULL,
					     SHELL_HELP("Drop a peer from the NodeDB (past key pinning for a re-keyed peer).",
							"<node-hex>"),
					     cmd_node_forget, 2, 0),
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

#if defined(CONFIG_MESHTASTIC_AIRTIME)
/* `meshtastic airtime` — the two questions the accounting answers, kept apart.
 *
 * Channel utilization is PER PRESET, because two presets are two channels that
 * cannot hear each other and CSMA must back off against the one it is actually
 * on. TX utilization is BAND-WIDE, because the duty cycle it feeds is a
 * regulatory limit on the band and does not care which spreading factor the PA
 * was keyed at. Showing them in one place with that distinction spelled out is
 * the point: they look like the same number and are not. */
static int cmd_airtime(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t cur = meshtastic_airtime_current_slot();
	int32_t tx10 = scaled_tenths(meshtastic_airtime_tx_util_percent());
	bool any = false;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "tx (band-wide, 1 h): %d.%u%%  <- what the duty cycle measures",
		    scaled_whole(tx10, 10), scaled_fraction(tx10, 10));
	shell_print(sh, "channel (per preset, 60 s)  <- what CSMA and the beacon gate read");

	for (uint8_t slot = 0; slot < MESHTASTIC_AIRTIME_PRESET_SLOTS; slot++) {
		float pct = meshtastic_airtime_channel_util_percent_slot(slot);
		int32_t p10 = scaled_tenths(pct);
		const char *name;

		if (p10 == 0 && slot != cur) {
			continue; /* quiet and not where we are: nothing to say */
		}
		any = true;

		name = (slot == MESHTASTIC_AIRTIME_SLOT_CUSTOM)
			       ? "custom"
			       : meshtastic_preset_display_name(
					 (meshtastic_Config_LoRaConfig_ModemPreset)slot, true);

		shell_print(sh, "  %-12s %d.%u%%%s", name, scaled_whole(p10, 10),
			    scaled_fraction(p10, 10), (slot == cur) ? "   <- on air now" : "");
	}

	if (!any) {
		shell_print(sh, "  (all quiet)");
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_AIRTIME */

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
/* `meshtastic duty` — the ceiling, how much of it is spent, and what has been
 * refused. Worth surfacing: once the gate engages EVERY send is refused, which
 * from the outside looks exactly like a dead radio. This is how you tell the
 * difference in one command. */
static int cmd_duty(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_duty_stats stats;
	float ceiling = meshtastic_duty_effective_pct();
	float used = meshtastic_airtime_tx_util_percent();
	int32_t c10 = scaled_tenths(ceiling), u10 = scaled_tenths(used);
	uint8_t silent = 0U;
	bool blocked;

	if (argc >= 2 && strcmp(argv[1], "reset") == 0) {
		meshtastic_duty_stats_reset();
		shell_print(sh, "duty counters cleared");
		return 0;
	}

	blocked = meshtastic_duty_blocked(&silent);
	meshtastic_duty_stats_get(&stats);

	if (ceiling >= 100.0f) {
		shell_print(sh, "ceiling : none (this region sets no duty cycle)");
	} else {
		shell_print(sh, "ceiling : %d.%u%%", scaled_whole(c10, 10),
			    scaled_fraction(c10, 10));
	}
	shell_print(sh, "used    : %d.%u%% of the last hour", scaled_whole(u10, 10),
		    scaled_fraction(u10, 10));
	shell_print(sh, "state   : %s", blocked ? "BLOCKING egress" : "sending allowed");
	if (blocked) {
		shell_print(sh, "clear in: %u min", silent);
	}
	shell_print(sh, "refused : %u (%u of them relays)", stats.blocked, stats.blocked_relay);

	return 0;
}
#endif /* CONFIG_MESHTASTIC_DUTY_CYCLE */

#if defined(CONFIG_MESHTASTIC_PHONELOG)
static const char *const phonelog_level_names[] = { "off", "err", "wrn", "inf", "dbg" };

/* `meshtastic phonelog` -- state + counters; `meshtastic phonelog <level>` sets
 * the ceiling; `meshtastic phonelog reset` zeroes the counters.
 *
 * The counters are the point. A phone that shows nothing could mean the level is
 * too low, the queue is full because the app is not draining it, or the rate cap
 * is eating a flood -- three very different problems that look identical from
 * the phone end. */
static int cmd_phonelog(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_phonelog_stats stats;
	uint8_t level;

	if (argc >= 2) {
		if (strcmp(argv[1], "reset") == 0) {
			meshtastic_phonelog_reset_stats();
			shell_print(sh, "phonelog counters cleared");
			return 0;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(phonelog_level_names); i++) {
			if (strcmp(argv[1], phonelog_level_names[i]) == 0) {
				int ret = meshtastic_phonelog_set_level(i);

				if (ret < 0) {
					shell_error(sh, "set level failed (%d)", ret);
					return ret;
				}
				shell_print(sh, "phonelog level: %s", phonelog_level_names[i]);
				return 0;
			}
		}

		shell_error(sh, "usage: phonelog [off|err|wrn|inf|dbg|reset]");
		return -EINVAL;
	}

	level = meshtastic_phonelog_get_level();
	meshtastic_phonelog_get_stats(&stats);

	shell_print(sh, "level     : %s%s",
		    level < ARRAY_SIZE(phonelog_level_names) ? phonelog_level_names[level] : "?",
		    stats.panicked ? " (STOOD DOWN: panic)" : "");
	shell_print(sh, "forwarded : %u", stats.forwarded);
	shell_print(sh, "dropped   : %u rate, %u queue-full, %u below level, %u log-core",
		    stats.dropped_rate, stats.dropped_queue, stats.dropped_level,
		    stats.dropped_core);
	shell_print(sh, "rate cap  : %u/s", (unsigned int)CONFIG_MESHTASTIC_PHONELOG_RATE);

	return 0;
}
#endif /* CONFIG_MESHTASTIC_PHONELOG */

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
static int cmd_metrics_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, SHELL_WORK_SEND_METRICS);
}

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
/* Print LocalStats as the node would report them, and optionally push one to
 * the phone right now. Reading them here is the point: LocalStats is normally
 * phone-only, so without this the only way to see the numbers on a bench node
 * with no phone attached would be to infer them from `meshtastic status`. */
static int cmd_metrics_localstats(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_LocalStats stats;
	int32_t chan_util, tx_util;
	int ret;

	ret = meshtastic_collect_local_stats(&stats);
	if (ret < 0) {
		shell_error(sh, "local stats unavailable (%d)", ret);
		return ret;
	}

	chan_util = scaled_tenths(stats.channel_utilization);
	tx_util = scaled_tenths(stats.air_util_tx);

	shell_print(sh, "uptime  : %us", stats.uptime_seconds);
	shell_print(sh, "util    : chan %d.%u%% tx %d.%u%%", scaled_whole(chan_util, 10),
		    scaled_fraction(chan_util, 10), scaled_whole(tx_util, 10),
		    scaled_fraction(tx_util, 10));
	shell_print(sh, "packets : tx %u, rx %u (%u bad), dupe %u, tx dropped %u",
		    stats.num_packets_tx, stats.num_packets_rx, stats.num_packets_rx_bad,
		    stats.num_rx_dupe, stats.num_tx_dropped);
	shell_print(sh, "relay   : %u sent, %u cancelled", stats.num_tx_relay,
		    stats.num_tx_relay_canceled);
	shell_print(sh, "nodes   : %u online / %u total", stats.num_online_nodes,
		    stats.num_total_nodes);
	shell_print(sh, "heap    : %u free / %u total", stats.heap_free_bytes,
		    stats.heap_total_bytes);

	if (argc >= 2 && strcmp(argv[1], "push") == 0) {
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE)
		ret = meshtastic_send_local_stats_to_phone();
		shell_print(sh, "pushed to phone transports: %s", ret == 0 ? "ok" : "failed");
		return ret;
#else
		shell_warn(sh, "built without CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE");
		return -ENOTSUP;
#endif
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_LOCAL_STATS */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_metrics_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send device metrics.", "[dest|broadcast]"),
					 cmd_metrics_send),
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
			       SHELL_CMD(localstats, NULL,
					 SHELL_HELP("Show LocalStats (mesh counters). "
						    "`push` also sends one to the phone.",
						    "[push]"),
					 cmd_metrics_localstats),
#endif
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
			shell_print(sh, "tx power:     %d dBm at the antenna (requested %d%s, "
					"clamped to the radio's %d dBm max; no front end to "
					"make up the difference)",
				    drive, radiated, dflt, drive);
		}
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

/* Match a preset display name (as reported by meshtastic_preset_display_name, so
 * the accepted spellings always equal what `meshtastic lora` shows).
 *
 * Outside the config-write guard, because parsing a preset name has nothing to
 * do with writing config: `scan presets` and `preset hop` both move the live
 * radio only, and a build with writes compiled out still wants them. */
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

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
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

	if (shell_config_write_refused(sh)) {
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

	if (shell_parse_modem_preset(argv[2], &target) < 0) {
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

#if defined(CONFIG_MESHTASTIC_BOOTLOG)
/* `meshtastic resets` — the retrieval half of the boot history.
 *
 * The boot-time report goes to whatever log backend happens to be live at the
 * time, which on this fleet is often nobody: there is no netlog, and a console
 * only exists while something is attached. A reboot at 03:50 is therefore
 * unobserved by construction. This is how you ask afterwards.
 */
static int cmd_resets(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_boot_record hist[CONFIG_MESHTASTIC_BOOTLOG_ENTRIES];
	struct meshtastic_boot_record now;
	char cbuf[64];
	size_t n;

	/* `resets durable` prints the FLASH ring instead: the one that survives the
	 * power cycle, and therefore the only one that still exists after a node has
	 * been carried to a USB host to be revived. */
	if (argc >= 2U && strcmp(argv[1], "durable") == 0) {
		struct meshtastic_boot_durable d[8];
		size_t dn = meshtastic_bootlog_durable_history(d, ARRAY_SIZE(d));

		if (dn == 0U) {
			shell_print(sh, "durable history: empty — nothing written yet, or "
					"CONFIG_MESHTASTIC_BOOTLOG_DURABLE=n");
			return 0;
		}
		shell_print(sh, "%-6s %-5s %-12s %-11s %s", "boot", "kind", "prev-ran(s)",
			    "wall(epoch)", "started-by");
		for (size_t i = 0; i < dn; i++) {
			char wbuf[16];

			/* 0 is "the clock was not valid when this was written", which is
			 * ordinary on a node that has not heard the mesh yet. Printing a
			 * bare 0 would read as 1970. */
			if (d[i].wall_s == 0U) {
				(void)snprintk(wbuf, sizeof(wbuf), "%s", "-");
			} else {
				(void)snprintk(wbuf, sizeof(wbuf), "%u", d[i].wall_s);
			}
			shell_print(sh, "#%-5u %-5s %-12u %-11s 0x%08x", d[i].boot_num,
				    (d[i].flags & MESHTASTIC_BOOT_F_WARM) ? "warm" : "COLD",
				    d[i].prev_uptime_s, wbuf, d[i].cause);
		}
		shell_print(sh, "");
		shell_print(sh, "This ring is in FLASH and survives power loss, so a COLD row "
				"here still has");
		shell_print(sh, "rows before it — which is the whole difference from the "
				"retained-RAM view.");
		shell_print(sh, "wall(epoch) '-' means the clock was not valid yet when the "
				"record was written.");
		return 0;
	}

	meshtastic_bootlog_this_boot(&now);

	shell_print(sh, "this boot   #%u, %s", now.boot_num,
		    (now.flags & MESHTASTIC_BOOT_F_WARM)
			    ? "WARM reset (retained RAM survived)"
			    : "retained RAM LOST (power cycle, brownout, or a RAM-clearing reset)");
	if (now.flags & MESHTASTIC_BOOT_F_WARM) {
		shell_print(sh, "previous    ran %u s before it ended", now.prev_uptime_s);
	}
	if (now.flags & MESHTASTIC_BOOT_F_CAUSE_OK) {
		shell_print(sh, "cause       0x%08x  %s", now.cause,
			    meshtastic_bootlog_cause_str(now.cause, cbuf, sizeof(cbuf)));
	} else {
		/* Deliberately not "cause: none". Claiming a reading we did not get is
		 * how a diagnostic starts lying: on this board the bootloader consumes
		 * the reset-reason register before the app ever runs. */
		shell_print(sh, "cause       unavailable — register empty, or consumed by the "
				"bootloader before the app ran");
	}
	shell_print(sh, "uptime      %lld s", (long long)k_uptime_seconds());

	n = meshtastic_bootlog_history(hist, ARRAY_SIZE(hist));
	if (n == 0U) {
		shell_print(sh, "");
		shell_print(sh, "no retained history (this is boot 1 of a fresh one)");
		return 0;
	}

	shell_print(sh, "");
	/*
	 * Both data columns describe the TRANSITION INTO the row's boot, not the
	 * row's boot itself: bootlog_init() stamps each record with how long the
	 * PREVIOUS run lasted and with the reset cause it read on the way in. The
	 * header used to say "ran(s)", which reads as "boot #N ran this long" and is
	 * off by one boot. It showed on screen as a contradiction — rzr2 reporting
	 * "this boot #50 ... uptime 7870" directly above a row "#50 ... 8731" — and
	 * cost a session the time to go read the source. Name the columns for what
	 * they hold.
	 */
	shell_print(sh, "%-6s %-5s %-12s %s", "boot", "kind", "prev-ran(s)", "started-by");
	for (size_t i = 0; i < n; i++) {
		/* prev_uptime_s is a uint16 fed by the heartbeat, so it saturates at
		 * 65535 s (18.2 h). Say so rather than report a cap as a measurement —
		 * a soak longer than that is exactly when someone reads this column. */
		if (hist[i].prev_uptime_s == UINT16_MAX) {
			shell_print(sh, "#%-5u %-5s %-12s 0x%08x", hist[i].boot_num,
				    (hist[i].flags & MESHTASTIC_BOOT_F_WARM) ? "warm" : "COLD",
				    ">=65535", hist[i].cause);
		} else {
			shell_print(sh, "#%-5u %-5s %-12u 0x%08x", hist[i].boot_num,
				    (hist[i].flags & MESHTASTIC_BOOT_F_WARM) ? "warm" : "COLD",
				    hist[i].prev_uptime_s, hist[i].cause);
		}
	}
	shell_print(sh, "");
	shell_print(sh, "Each row is a boot STARTING: how long the run before it lasted, and "
			"the reset");
	shell_print(sh, "cause read on the way in — so both numbers belong to boot #N-1's "
			"ending, not");
	shell_print(sh, "to #N. prev-ran(s) saturates at 65535 (18.2 h).");
	shell_print(sh, "A COLD row means retained RAM did not survive into that boot, so "
			"everything before it is gone.");
	return 0;
}
#endif /* CONFIG_MESHTASTIC_BOOTLOG */

/* `meshtastic crashinfo` -- the three RTC-persistent crash breadcrumbs
 * (watchdog channel timeout, hardware-watchdog stage-0, and fatal error), in
 * one place, read non-destructively so this can be run more than once
 * without disturbing what the boot-time log still owns. See
 * zephyr/meshtastic/diagnostics.h.
 */
static const char *crashinfo_fatal_reason_name(uint32_t reason)
{
	switch (reason) {
	case K_ERR_CPU_EXCEPTION:
		return "CPU exception";
	case K_ERR_SPURIOUS_IRQ:
		return "unhandled interrupt";
	case K_ERR_STACK_CHK_FAIL:
		return "stack overflow";
	case K_ERR_KERNEL_OOPS:
		return "kernel oops";
	case K_ERR_KERNEL_PANIC:
		return "kernel panic";
	default:
		return "unknown";
	}
}

static int cmd_crashinfo(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_fatal_crash_info fatal_crash;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_MESHTASTIC_WATCHDOG)
	/* Declared inside the guard, not above it: the XIAO class-1 image builds
	 * MESHTASTIC_WATCHDOG=n, where both of these are unused and warn. Harmless
	 * today only because the app build carries no -Werror — twister does. */
	struct meshtastic_watchdog_crash_info wdt_crash;
	struct meshtastic_hw_watchdog_crash_info hw_wdt_crash;

	if (meshtastic_watchdog_peek_last_crash(&wdt_crash)) {
		shell_print(sh, "watchdog     channel \"%s\" timed out, heap free=%u "
				"allocated=%u max_allocated=%u, running thread \"%s\"",
			    wdt_crash.channel, wdt_crash.heap_free, wdt_crash.heap_allocated,
			    wdt_crash.heap_max_allocated, wdt_crash.thread_name);
	} else {
		shell_print(sh, "watchdog     none pending");
	}

	if (meshtastic_hw_watchdog_peek_last_crash(&hw_wdt_crash)) {
		shell_print(sh, "hw watchdog  uptime=%u ms, running thread \"%s\"",
			    hw_wdt_crash.uptime_ms, hw_wdt_crash.thread_name);
	} else {
		shell_print(sh, "hw watchdog  none pending");
	}
#else
	shell_print(sh, "watchdog     unsupported (CONFIG_MESHTASTIC_WATCHDOG=n on this build)");
	shell_print(sh, "hw watchdog  unsupported (CONFIG_MESHTASTIC_WATCHDOG=n on this build)");
#endif

	if (meshtastic_fatal_peek_last_crash(&fatal_crash)) {
		shell_print(sh, "fatal        %s (%u), heap free=%u allocated=%u "
				"max_allocated=%u, faulting thread \"%s\"",
			    crashinfo_fatal_reason_name(fatal_crash.reason), fatal_crash.reason,
			    fatal_crash.heap_free, fatal_crash.heap_allocated,
			    fatal_crash.heap_max_allocated, fatal_crash.thread_name);
	} else {
		shell_print(sh, "fatal        none pending");
	}

	shell_print(sh, "");
	shell_print(sh, "Non-destructive: does not clear what `meshtastic resets` reports "
			"at boot.");

	return 0;
}

static int cmd_lora(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		cmd_lora_show(sh);
		return 0;
	}
	if (argc != 3U ||
	    (strcmp(argv[1], "preset") != 0 && strcmp(argv[1], "tx") != 0)) {
		shell_error(sh, "usage: meshtastic lora [preset <name>] [tx on|off]");
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
	SHELL_CMD(hold, NULL,
		  SHELL_HELP("Show or set HOLD (\"do not push firmware to me\") in this node's beats.",
			     "[on|off]"),
		  cmd_blepeer_hold),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_BLE_PEER */

#if defined(CONFIG_MESHTASTIC_ADMIN)
/* meshtastic admin trust [<node> [off]] — manage SecurityConfig.admin_key by
 * NodeDB lookup: "trust that node as my remote admin". The headless-bench
 * equivalent of the app's key-entry screen — the key comes from the target's
 * own NodeInfo broadcast, so trust-on-first-use applies: verify the node id
 * belongs to hardware you hold before trusting it.
 *
 * `admin trust key:<hex-prefix> off` removes an entry by its leading bytes
 * (>= 4, unique) instead: the form for a key whose node no longer exists in
 * the NodeDB — a re-keyed peer's old key (bench 2026-08-27: kit2 kept the
 * courier's stale `510dd592…` beside its real one, and nothing could name it). */
static int cmd_admin_trust(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config cfg;
	meshtastic_Config_SecurityConfig *sec;
	uint8_t key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
	uint32_t node_num = 0U;
	bool remove;
	bool by_prefix;
	size_t prefix_len = 0U;
	pb_size_t match;
	int ret;

	ret = meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg);
	if (ret < 0 || cfg.which_payload_variant != meshtastic_Config_security_tag) {
		shell_error(sh, "security config unavailable (%d)", ret);
		return ret < 0 ? ret : -EIO;
	}
	sec = &cfg.payload_variant.security;

	if (argc == 1U) {
		shell_print(sh, "admin keys: %u/%u", (unsigned int)sec->admin_key_count,
			    (unsigned int)ARRAY_SIZE(sec->admin_key));
		for (pb_size_t i = 0; i < sec->admin_key_count; i++) {
			shell_print(sh, "  [%u] %02x%02x%02x%02x… (%u bytes)", (unsigned int)i,
				    sec->admin_key[i].bytes[0], sec->admin_key[i].bytes[1],
				    sec->admin_key[i].bytes[2], sec->admin_key[i].bytes[3],
				    (unsigned int)sec->admin_key[i].size);
		}
		return 0;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "config writes are compiled out of this build");
	ARG_UNUSED(remove);
	ARG_UNUSED(by_prefix);
	ARG_UNUSED(prefix_len);
	ARG_UNUSED(match);
	ARG_UNUSED(node_num);
	ARG_UNUSED(key);
	return -ENOTSUP;
#else
	if (shell_config_write_refused(sh)) {
		return -EACCES;
	}

	remove = (argc >= 3U && strcmp(argv[2], "off") == 0);
	by_prefix = (strncmp(argv[1], "key:", 4U) == 0);

	if (by_prefix) {
		const char *hex = argv[1] + 4;
		size_t n = strlen(hex);

		if (!remove) {
			shell_error(sh, "key:<prefix> only removes (`off`); trust a node by its id");
			return -EINVAL;
		}
		if (n < 8U || (n & 1U) != 0U || n / 2U > sizeof(key) ||
		    hex2bin(hex, n, key, sizeof(key)) != n / 2U) {
			shell_error(sh, "key:<prefix> needs 4..%u bytes of hex",
				    (unsigned int)sizeof(key));
			return -EINVAL;
		}
		prefix_len = n / 2U;
	} else {
		ret = parse_u32(sh, argv[1], &node_num);
		if (ret < 0) {
			return ret;
		}
		if (meshtastic_nodedb_copy_pubkey(node_num, key) != 0) {
			shell_error(sh, "no public key for 0x%08x in the NodeDB (heard its NodeInfo?)",
				    node_num);
			return -ENOENT;
		}
		prefix_len = sizeof(key);
	}

	match = sec->admin_key_count;
	for (pb_size_t i = 0; i < sec->admin_key_count; i++) {
		if (sec->admin_key[i].size == sizeof(key) &&
		    memcmp(sec->admin_key[i].bytes, key, prefix_len) == 0) {
			if (match != sec->admin_key_count) {
				shell_error(sh, "key:%s matches more than one entry — give more bytes",
					    argv[1] + 4);
				return -EINVAL;
			}
			match = i;
			if (!by_prefix) {
				break;
			}
		}
	}

	if (remove) {
		if (match == sec->admin_key_count) {
			if (by_prefix) {
				shell_print(sh, "no admin key starts with %s", argv[1] + 4);
			} else {
				shell_print(sh, "0x%08x was not trusted", node_num);
			}
			return 0;
		}
		for (pb_size_t i = match; i + 1U < sec->admin_key_count; i++) {
			sec->admin_key[i] = sec->admin_key[i + 1U];
		}
		sec->admin_key_count--;
	} else {
		if (match != sec->admin_key_count) {
			shell_print(sh, "0x%08x already trusted", node_num);
			return 0;
		}
		if (sec->admin_key_count >= ARRAY_SIZE(sec->admin_key)) {
			shell_error(sh, "admin key list full (%u)",
				    (unsigned int)ARRAY_SIZE(sec->admin_key));
			return -ENOSPC;
		}
		sec->admin_key[sec->admin_key_count].size = sizeof(key);
		memcpy(sec->admin_key[sec->admin_key_count].bytes, key, sizeof(key));
		sec->admin_key_count++;
	}

	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "config write failed (%d)", ret);
		return ret;
	}
	if (by_prefix) {
		shell_print(sh, "key %s… removed (%u key%s)", argv[1] + 4,
			    (unsigned int)sec->admin_key_count,
			    sec->admin_key_count == 1U ? "" : "s");
	} else {
		shell_print(sh, "0x%08x %s as remote admin (%u key%s)", node_num,
			    remove ? "untrusted" : "trusted", (unsigned int)sec->admin_key_count,
			    sec->admin_key_count == 1U ? "" : "s");
	}
	return 0;
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
/* meshtastic admin remote <node> get owner
 *                        <node> get config <type 0-8>
 *                        <node> set-owner <long> [short]
 *                        <node> set-time [<unix-epoch-seconds>]  (default: ours)
 * Responses arrive asynchronously on the RX path and print as `admin client:`
 * log lines; a getter also caches the target's session passkey for the
 * mutating op that follows. */
static int cmd_admin_remote(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node_num;
	int ret;

	if (argc < 3U) {
		goto usage;
	}
	ret = parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	if (strcmp(argv[2], "get") == 0 && argc >= 4U) {
		if (strcmp(argv[3], "owner") == 0) {
			ret = meshtastic_admin_client_get_owner(node_num);
		} else if (strcmp(argv[3], "config") == 0 && argc >= 5U) {
			uint32_t type;

			ret = parse_u32(sh, argv[4], &type);
			if (ret < 0) {
				return ret;
			}
			ret = meshtastic_admin_client_get_config(node_num, type);
		} else {
			goto usage;
		}
	} else if (strcmp(argv[2], "set-time") == 0) {
		/*
		 * Relay a wall clock to a peer (agents-xhli.14). With no explicit
		 * epoch, relay OUR OWN — which is the whole point: a node that has a
		 * time source hands one to a node that does not.
		 *
		 * Refusing when our own clock is unset is not a nicety. An unset
		 * clock reads epoch 0, which the target's sanity window rejects, so
		 * the request would look sent and do nothing; and a node that cannot
		 * say when anything happened has no business being a time source.
		 */
		uint32_t epoch;

		if (argc >= 4U) {
			ret = parse_u32(sh, argv[3], &epoch);
			if (ret < 0) {
				return ret;
			}
		} else {
			epoch = meshtastic_clock_now_epoch();
			if (epoch == 0U) {
				shell_error(sh, "refused: this node's own clock is unset, so "
						"it cannot be a time source (set it first, or "
						"pass an explicit epoch)");
				return -EAGAIN;
			}
		}

		if (!meshtastic_admin_client_have_passkey(node_num)) {
			shell_error(sh, "no fresh session passkey for 0x%08x — run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    node_num, node_num);
			return -EACCES;
		}
		ret = meshtastic_admin_client_set_time(node_num, epoch);
	} else if (strcmp(argv[2], "set-owner") == 0 && argc >= 4U) {
		if (!meshtastic_admin_client_have_passkey(node_num)) {
			shell_error(sh, "no fresh session passkey for 0x%08x — run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    node_num, node_num);
			return -EACCES;
		}
		ret = meshtastic_admin_client_set_owner(node_num, argv[3],
							argc >= 5U ? argv[4] : NULL);
	} else {
		goto usage;
	}

	if (ret < 0) {
		shell_error(sh, "request failed (%d)%s", ret,
			    ret == -EACCES ? " — no public key / passkey" : "");
		return ret;
	}
	shell_print(sh, "request sent — watch for `admin client:` log lines");
	return 0;

usage:
	shell_error(sh, "usage: meshtastic admin remote <node> get owner\n"
		    "       meshtastic admin remote <node> get config <type 0-8>\n"
		    "       meshtastic admin remote <node> set-owner <long> [short]\n"
		    "       meshtastic admin remote <node> set-time [<unix-epoch-seconds>]");
	return -EINVAL;
}
#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */

#if defined(CONFIG_MESHTASTIC_CLUSTER)
/* Config-section names for `cluster promote`/`pin`/`unpin` — the shareable set
 * plus lora (which promote and pin both refuse with the §7.9 explanation, a
 * better teacher than "unknown section"). */
static int cluster_section_parse(const char *name, uint16_t *tag)
{
	static const struct {
		const char *name;
		uint16_t tag;
	} sections[] = {
		{"device", meshtastic_Config_device_tag},
		{"position", meshtastic_Config_position_tag},
		{"power", meshtastic_Config_power_tag},
		{"display", meshtastic_Config_display_tag},
		{"lora", meshtastic_Config_lora_tag},
		{"bluetooth", meshtastic_Config_bluetooth_tag},
	};

	for (size_t i = 0; i < ARRAY_SIZE(sections); i++) {
		if (strcmp(name, sections[i].name) == 0) {
			*tag = sections[i].tag;
			return 0;
		}
	}
	return -EINVAL;
}

static int cmd_cluster_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_cluster_stats st;
	struct meshtastic_cluster_entry e;
	const char *sync;
	uint32_t sync_peer = 0U;
	uint8_t ch_index;
	bool have_ch;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_cluster_stats_get(&st);
	have_ch = meshtastic_cluster_channel_resolved(&ch_index);
	sync = meshtastic_cluster_sync_state(&sync_peer);

	if (have_ch) {
		shell_print(sh, "channel : \"%s\" = index %u",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME, ch_index);
	} else {
		shell_print(sh, "channel : \"%s\" NOT PROVISIONED — module idle",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
	}
	shell_print(sh, "doc     : %u entr%s, hash %08x",
		    (unsigned int)meshtastic_cluster_entry_count(),
		    meshtastic_cluster_entry_count() == 1U ? "y" : "ies",
		    (unsigned int)meshtastic_cluster_doc_hash_now());
	/*
	 * WHAT THIS NODE CLAIMS TO TRACK — printed next to the document because
	 * the two only mean something together: the hash above is taken over
	 * the claim, not over the table.
	 *
	 * CORE says so loudly. It is not a fault and it costs this node nothing
	 * in what it RUNS (CORE is exactly the closure of effective(me, ·)),
	 * but it does mean the fleet has one fewer place to recover a node's
	 * pins from, and nothing else on the node would ever say so.
	 */
	{
		bool pinned;
		const char *scope = meshtastic_cluster_scope_name(&pinned);
		bool core = (scope[0] == 'C');

		shell_print(sh, "scope   : %s (%s) — %u of %u entries%s", scope,
			    st.scope_demotions ? "narrowed under table pressure"
					       : (pinned ? "pinned" : "auto"),
			    (unsigned int)meshtastic_cluster_entry_count(),
			    (unsigned int)CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES,
			    core ? "" : ", tracking the whole fleet");
		if (core) {
			shell_print(sh, "        : base + MY OWN sections only — this node is "
					"NOT a restore source for other nodes (§7.7)");
		}
		if (st.scope_demotions) {
			shell_print(sh, "        : the fleet outgrew this table — raise "
					"MESHTASTIC_CLUSTER_MAX_ENTRIES and reflash to claim "
					"everything again");
		}
		if (st.entry_rx_out_of_scope || st.entries_evicted || st.want_no_space) {
			shell_print(sh, "        : out_of_scope=%u evicted=%u no_space=%u",
				    st.entry_rx_out_of_scope, st.entries_evicted,
				    st.want_no_space);
		}
		if (core) {
			/* The one mechanism by which a narrowed node ever hears
			 * about its OWN entries — worth being able to see. */
			if (CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS == 0) {
				shell_print(sh, "backstop: OFF — this node cannot recover "
						"its own pins from the fleet");
			} else {
				shell_print(sh, "backstop: every %u digest periods, walks=%u%s",
					    (unsigned int)
						    CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS,
					    st.backstop_walks,
					    st.backstop_walks
						    ? ""
						    : "  (none yet — no peer claiming the "
						      "whole document has been heard)");
			}
		}
	}
	/*
	 * THE DRIFT HORIZON'S PRECONDITION, said out loud (agents-xhli.14).
	 *
	 * The horizon (D12) is measured against the wall clock, so a node whose
	 * clock is unset has NO horizon and accepts a stamp from any year — one
	 * entry dated 2100 would then win every comparison for good. That is the
	 * deliberate choice (a node that cannot say when anything happened has no
	 * basis to refuse, and refusing everything would strand a fresh node
	 * permanently — the same unrecoverable trade the lora ingest gate turned
	 * down). What was NOT deliberate was that nothing said so: the horizon
	 * simply never fired and the status was silent about it.
	 *
	 * So it is a line in status now, for the same reason `sections_held` is:
	 * a deliberate non-enforcement that nobody can see is indistinguishable
	 * from a broken one. Fix it with `admin remote <us> set-time` from a node
	 * that has a clock.
	 */
	{
		uint32_t epoch = meshtastic_clock_now_epoch();

		if (epoch == 0U) {
			shell_print(sh, "horizon : ⚠ NONE — clock unset, so EVERY stamp is "
					"accepted (see CLUSTER-SYNC-M4.md D12)");
		} else {
			shell_print(sh, "horizon : ±%u min around epoch %u (clock source %s)",
				    (unsigned int)(MESHTASTIC_HLC_MAX_DRIFT_MS / 60000U), epoch,
				    clock_quality_name(meshtastic_clock_get_quality()));
		}
	}
	/* "queued", not "sent": the counter increments where the digest enters
	 * the send path, and a node with config.lora.tx_enabled = false has its
	 * frames dropped further down, at the radio choke point. A digest is a
	 * broadcast, so no peer bearer carries it either — on such a node this
	 * number counts digests that never left. */
	shell_print(sh, "digests : queued=%u%s rx match=%u MISMATCH=%u incomparable=%u%s",
		    st.digest_tx, mt.tx_enabled ? "" : " (radio TX OFF — none of these aired)",
		    st.digest_rx_match, st.digest_rx_mismatch, st.digest_rx_incomparable,
		    st.digest_rx_incomparable ? "  (a peer declares no document scope — an "
						"older build; it cannot be compared, only "
						"waited out)"
					      : "");
	if (sync_peer != 0U) {
		shell_print(sh, "sync    : %s with 0x%08x", sync, sync_peer);
	} else {
		shell_print(sh, "sync    : %s", sync);
	}
	shell_print(sh, "walk    : pulls=%u timed_out=%u vector tx=%u rx=%u entry tx=%u",
		    st.pull_started, st.pull_timed_out, st.vector_tx, st.vector_rx,
		    st.entry_tx);
	/* fruitless climbing with applied flat is the signature of a mismatch
	 * that cannot be resolved from here — usually a peer holding an entry
	 * authored by a master this node does not trust. held>0 means the
	 * backoff has taken over, which is the bound doing its job, not a
	 * fault. */
	shell_print(sh, "        : fruitless=%u (empty=%u) held=%u%s", st.pull_fruitless,
		    st.pull_empty, st.pull_held,
		    st.pull_held ? "  (backing off — an unresolvable mismatch; check "
				   "`admin trust` on both ends)"
				 : "");
	shell_print(sh, "merge   : applied=%u stale=%u REFUSED=%u unsolicited=%u busy=%u%s",
		    st.entry_rx_applied, st.entry_rx_stale, st.entry_rx_refused,
		    st.rx_unsolicited, st.tx_busy,
		    st.entry_rx_no_space ? "  ** DOCUMENT TABLE FULL — pushed entries are "
					   "being dropped (a walk would narrow the claim "
					   "instead; an unsolicited push deliberately does "
					   "not) **"
					 : "");
	/* suppressed>0 is the rate bound working, not a fault: a push is a
	 * latency optimisation over the digest, so dropping one costs a digest
	 * period and never correctness. */
	shell_print(sh, "push    : sent=%u suppressed=%u%s", st.push_tx, st.push_suppressed,
		    st.push_suppressed ? "  (rate-bounded — the digest still carries every "
					 "change)"
				       : "");
	shell_print(sh, "config  : sections applied=%u kept_local=%u%s", st.sections_applied,
		    st.sections_kept_local,
		    st.sections_held ? "  ** a fleet LoRa section is REPLICATED BUT HELD "
				       "(applying it would re-key this radio; see §7.9) **"
				     : "");
	shell_print(sh, "rx      : wrong_channel=%u undecodable=%u not_implemented=%u%s",
		    st.rx_wrong_channel, st.rx_undecodable, st.rx_not_implemented,
		    st.entry_rx_future ? "  ** entries refused for being stamped beyond the "
					 "clock drift horizon — someone's clock is wrong **"
				       : "");

	/* The per-node rows for THIS node are the ones an operator is reasoning
	 * about during a pin/unpin, and they are visually identical to every
	 * other node's. Say which are ours: a live one is what effective() is
	 * currently returning for that section, a tombstoned one is why it is
	 * NOT (§2.2). */
	for (uint16_t i = 0U; meshtastic_cluster_entry_get(i, &e); i++) {
		bool mine = (e.key.layer == MESHTASTIC_CLUSTER_LAYER_NODE) &&
			    (e.key.node_id == meshtastic_get_node_id());

		shell_print(sh, "  [%c/%08x/%u] %s%u B, stamp %lld.%u by 0x%08x%s",
			    e.key.layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n',
			    e.key.node_id, (unsigned int)e.key.section,
			    e.tombstone ? "TOMBSTONE, " : "", (unsigned int)e.payload_len,
			    (long long)e.stamp.physical_ms, e.stamp.counter, e.stamp.node_id,
			    mine ? (e.tombstone ? "  <- MY UNPIN (base applies)"
						: "  <- MY PIN (wins over base)")
				 : "");
	}
	return 0;
}

static int cmd_cluster_promote(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster promote "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_promote(tag);
	if (ret == -EPERM) {
		shell_error(sh, "refused: section not shareable (secret boundary), or lora "
			    "(a missed fleet preset change orphans nodes — see "
			    "CONFIG-CONVERGENCE.md §7.9)");
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s already IS the fleet base — this node is running the "
			    "document's own copy, so there is nothing new to promote "
			    "(edit the section first)", argv[1]);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "promote failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s promoted to fleet base (broadcast once; the digest backs it up)",
		    argv[1]);
	return 0;
}

/*
 * meshtastic cluster pin <sec> — this node's own override for one section.
 *
 * The pin does not replace or hide the fleet base: base/<sec> keeps replicating
 * underneath with its own stamp and the pin wins only when effective(me) is
 * computed. That separation is the whole reason the document has two layers
 * (CONFIG-CONVERGENCE.md §7.7) — it is what makes `unpin` land on the base as
 * it stands AT THAT MOMENT rather than on the one frozen when the pin was made.
 */
static int cmd_cluster_pin(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster pin "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_pin(tag);
	if (ret == -EPERM) {
		shell_error(sh, "refused: section not shareable (secret boundary), lora (a "
			    "doc-borne lora section is replicated but never applied, so the pin "
			    "could not take effect), or this node is managed");
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s is already this node's pin — the stored value came FROM "
			    "that pin, so re-pinning would only mint a second stamp for the "
			    "same bytes (edit the section first)", argv[1]);
		return ret;
	}
	if (ret == -ENOENT) {
		shell_error(sh, "no %s section in the config store", argv[1]);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "pin failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s pinned for this node — the fleet base keeps replicating "
		    "underneath and `unpin` returns to whatever it holds then", argv[1]);
	return 0;
}

/* meshtastic cluster unpin <sec> — drop the override. A TOMBSTONE, not a
 * deletion: removal has to replicate, or the next anti-entropy pass pulls the
 * pin back from a peer that still holds it (D7). */
static int cmd_cluster_unpin(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster unpin "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_unpin(tag);
	if (ret == -ENOENT) {
		shell_error(sh, "this node has no pin on %s", argv[1]);
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s is already unpinned (the tombstone is replicating)",
			    argv[1]);
		return ret;
	}
	if (ret == -EPERM) {
		shell_error(sh, "refused: this node is managed (SecurityConfig.is_managed)");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "unpin failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s unpinned — this node now follows the fleet base as it stands "
		    "now (with no fleet base for the section, it keeps what it has)", argv[1]);
	return 0;
}

static int cmd_cluster_digest(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_cluster_digest_now();
	shell_print(sh, "digest queued");
	return 0;
}

/* meshtastic cluster pull <node> — run the anti-entropy walk against one peer
 * now instead of waiting for its next digest. The convergence proof, on demand:
 * the walk is identical either way, only the trigger differs. */
static int cmd_cluster_pull(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node_num;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic cluster pull <node>");
		return -EINVAL;
	}
	ret = parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	ret = meshtastic_cluster_pull(node_num);
	if (ret == -EBUSY) {
		shell_error(sh, "an exchange is already in flight (one at a time)");
		return ret;
	}
	if (ret == -ENOTCONN) {
		shell_error(sh, "no channel named \"%s\" — nothing to pull over",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "pull failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "pulling from 0x%08x — `cluster status` shows the walk", node_num);
	return 0;
}

static int cmd_cluster_scope(const struct shell *sh, size_t argc, char **argv)
{
	int choice;
	int ret;

	if (argc != 2) {
		shell_error(sh, "usage: meshtastic cluster scope <full|core|auto>");
		return -EINVAL;
	}
	if (strcmp(argv[1], "auto") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO;
	} else if (strcmp(argv[1], "full") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_FULL;
	} else if (strcmp(argv[1], "core") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_CORE;
	} else {
		shell_error(sh, "unknown scope \"%s\" — full, core or auto", argv[1]);
		return -EINVAL;
	}

	ret = meshtastic_cluster_scope_select(choice);
	if (ret == -EALREADY) {
		shell_print(sh, "already claiming that");
		return 0;
	}
	if (ret != 0) {
		shell_error(sh, "scope change failed (%d)", ret);
		return ret;
	}

	if (choice == MESHTASTIC_CLUSTER_SCOPE_CHOICE_CORE) {
		shell_print(sh, "now claiming CORE — base + this node's own sections. "
				"Everything else has been dropped, from flash too. This node "
				"is no longer a restore source for other nodes; what it RUNS "
				"is unchanged.");
	} else {
		shell_print(sh, "now claiming FULL%s. The fleet's rows come back on the "
				"next walk.",
			    choice == MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO
				    ? " (auto — free to narrow again under table pressure)"
				    : " (pinned — it will NOT narrow, so a full table stops "
				      "tracking part of the fleet)");
	}
	return 0;
}

static int cmd_cluster_reset(const struct shell *sh, size_t argc, char **argv)
{
	int dropped;

	/* Confirmation required. Every other destructive verb here writes a
	 * versioned entry that the fleet can argue with; this one throws away
	 * the whole local copy, and a fat-fingered `cluster reset` next to
	 * `cluster status` should not be able to do that silently. */
	if (argc != 2 || strcmp(argv[1], "--confirm") != 0) {
		shell_error(sh, "usage: meshtastic cluster reset --confirm");
		shell_print(sh, "drops the ENTIRE document — every entry, the persisted "
				"records, and any recorded scope.");
		return -EINVAL;
	}

	dropped = meshtastic_cluster_reset();
	if (dropped < 0) {
		shell_error(sh, "reset failed (%d)", dropped);
		return dropped;
	}
	shell_print(sh, "document cleared — %d entr%s dropped.", dropped,
		    dropped == 1 ? "y" : "ies");
	shell_print(sh, "This node told NOBODY to forget anything. The next digest from a "
			"peer that still holds the document brings it all back.");
	shell_print(sh, "To clear a FLEET, isolate every member on BOTH bearers first — "
			"`lora tx off` AND `blepeer scan off` + `blepeer disconnect` on the "
			"central side of each peer link. A cluster pull is a unicast, so it "
			"diverts to a live BLE link and a LoRa-mute node still converges: "
			"that is the peer-transport property working, not a fault.");
	return 0;
}

/* `cluster fleet` — who agrees with me, and can I safely make a fleet-wide move.
 *
 * The pre-flight half of the straggler sweep (CONFIG-CONVERGENCE.md §7.9). A
 * fleet preset change orphans any member that misses it — new frequency, new
 * channel hashes, mutually deaf — so the question worth answering BEFORE the
 * move is "does every member already hold what I am about to make current".
 *
 * The verdict deliberately reports what it cannot see as loudly as what it can.
 * A node that is already orphaned has no digest on record and so cannot appear
 * as a disagreement; it is simply absent, and an absence must not read as
 * assent. */
static int cmd_cluster_fleet(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_cluster_fleet f;
	uint16_t n = meshtastic_cluster_peer_count();
	uint32_t now = (uint32_t)k_uptime_seconds();
	bool converged;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	converged = meshtastic_cluster_fleet_converged(&f);

	shell_print(sh, "my doc  : %08x / %u entries",
		    (unsigned int)meshtastic_cluster_doc_hash_now(),
		    meshtastic_cluster_entry_count());
	shell_print(sh, "window  : %us (3x the digest period)", f.window_sec);

	if (n == 0U) {
		shell_print(sh, "peers   : none heard");
	}
	for (uint16_t i = 0; i < n; i++) {
		struct meshtastic_cluster_peer p;
		uint32_t age;
		const char *verdict;

		if (!meshtastic_cluster_peer_get(i, &p)) {
			continue;
		}
		age = (now > p.heard_uptime_sec) ? (now - p.heard_uptime_sec) : 0U;

		if (age > f.window_sec) {
			verdict = "STALE";
		} else if (!p.comparable) {
			verdict = "not comparable";
		} else if (p.agreed) {
			verdict = "agrees";
		} else {
			verdict = "DIVERGED";
		}

		shell_print(sh, "  0x%08x %-14s doc=%08x/%-3u %us ago", p.node_id, verdict,
			    (unsigned int)p.doc_hash, p.entry_count, age);
	}

	shell_print(sh, "totals  : %u known — %u agree, %u diverged, %u stale, %u incomparable",
		    f.known, f.agreed, f.diverged, f.stale, f.incomparable);

	if (converged) {
		shell_print(sh, "verdict : every peer I KNOW OF agrees");
	} else if (f.known == 0U) {
		shell_print(sh, "verdict : NO — nothing heard, which is not the same as "
				"agreement");
	} else {
		shell_print(sh, "verdict : NO — do not make a fleet-wide preset change");
	}
	shell_warn(sh, "a node already orphaned, asleep or out of range has no digest here "
		       "and cannot fail this check — it is absent, not agreeing");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_cluster_cmds,
			       SHELL_CMD(fleet, NULL,
					 SHELL_HELP("Who agrees with my document — the "
						    "pre-flight for a fleet-wide change.",
						    NULL),
					 cmd_cluster_fleet),
			       SHELL_CMD(status, NULL,
					 SHELL_HELP("Cluster doc, digest stats, channel binding.",
						    NULL),
					 cmd_cluster_status),
			       SHELL_CMD(promote, NULL,
					 SHELL_HELP("Promote my current section to fleet base.",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_promote),
			       SHELL_CMD(pin, NULL,
					 SHELL_HELP("Override one section for THIS node "
						    "(base keeps replicating underneath).",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_pin),
			       SHELL_CMD(unpin, NULL,
					 SHELL_HELP("Drop this node's override (a replicating "
						    "tombstone); follow the current base.",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_unpin),
			       SHELL_CMD(reset, NULL,
					 SHELL_HELP("Forget the whole document (local only — a "
						    "peer that still holds it will re-seed).",
						    "--confirm"),
					 cmd_cluster_reset),
			       SHELL_CMD(scope, NULL,
					 SHELL_HELP("What this node claims to track. core = "
						    "base + my own sections (constrained tier).",
						    "<full|core|auto>"),
					 cmd_cluster_scope),
			       SHELL_CMD(digest, NULL,
					 SHELL_HELP("Broadcast one digest now.", NULL),
					 cmd_cluster_digest),
			       SHELL_CMD(pull, NULL,
					 SHELL_HELP("Run the anti-entropy walk against a peer "
						    "now.", "<node>"),
					 cmd_cluster_pull),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_CLUSTER */

#if defined(CONFIG_MESHTASTIC_FLEET)
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
#endif /* CONFIG_MESHTASTIC_FLEET_COURIER */

static int cmd_fleet_desire(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t version;
	uint16_t hw_model = 0U;
	uint8_t hash[4];
	const uint8_t *hash_p = NULL;
	uint32_t flags = 0U;
	unsigned long cls;
	int ret;

	if (argc < 3U) {
		shell_error(sh, "usage: fleet desire <class> <maj.min.rev> [hash8hex] [pause|downgrade] [hw=<model>]");
		return -EINVAL;
	}
	cls = strtoul(argv[1], NULL, 0);
	if (cls == 0UL || cls > 255UL || fleet_version_parse(argv[2], &version) != 0) {
		shell_error(sh, "class 1..255 and a version maj.min.rev are required");
		return -EINVAL;
	}
	for (size_t i = 3U; i < argc; i++) {
		if (strcmp(argv[i], "pause") == 0) {
			flags |= MESHTASTIC_FLEET_FLAG_PAUSE;
		} else if (strcmp(argv[i], "downgrade") == 0) {
			flags |= MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE;
		} else if (strncmp(argv[i], "hw=", 3) == 0) {
			unsigned long m = strtoul(argv[i] + 3, NULL, 0);

			if (m == 0UL || m > 65535UL) {
				shell_error(sh, "hw=<meshtastic HardwareModel number>, e.g. hw=88");
				return -EINVAL;
			}
			hw_model = (uint16_t)m;
		} else if (fleet_hash_parse(argv[i], hash) == 0) {
			hash_p = hash;
		} else {
			shell_error(sh, "unknown argument '%s'", argv[i]);
			return -EINVAL;
		}
	}
	ret = meshtastic_fleet_desire((uint8_t)cls, version, hash_p, flags, hw_model);
	if (ret == -EPERM) {
		shell_error(sh, "refused: this node may not write the document (managed?)");
	} else if (ret == -ENOSPC) {
		shell_error(sh, "refused: four classes already have rows");
	} else if (ret < 0) {
		shell_error(sh, "desire failed (%d)", ret);
	} else {
		shell_print(sh, "published: class %lu -> %s%s", cls, argv[2],
			    flags ? " (flagged)" : "");
	}
	return ret;
}

static int cmd_fleet_pin(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node, version = 0U;
	uint8_t hash[4];
	const uint8_t *hash_p = NULL;
	uint32_t flags = 0U;
	bool withdraw = (strcmp(argv[0], "unpin") == 0);
	int ret;

	if (argc < (withdraw ? 2U : 3U)) {
		shell_error(sh, "usage: fleet pin <node-hex> <maj.min.rev> [hash8hex] [downgrade] | "
				"fleet unpin <node-hex>");
		return -EINVAL;
	}
	node = (uint32_t)strtoul(argv[1], NULL, 16);
	if (node == 0U) {
		shell_error(sh, "node id required (hex)");
		return -EINVAL;
	}
	if (!withdraw) {
		if (fleet_version_parse(argv[2], &version) != 0) {
			shell_error(sh, "version maj.min.rev required");
			return -EINVAL;
		}
		for (size_t i = 3U; i < argc; i++) {
			if (strcmp(argv[i], "downgrade") == 0) {
				flags |= MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE;
			} else if (fleet_hash_parse(argv[i], hash) == 0) {
				hash_p = hash;
			} else {
				shell_error(sh, "unknown argument '%s'", argv[i]);
				return -EINVAL;
			}
		}
	}
	ret = meshtastic_fleet_pin(node, version, hash_p, flags);
	if (ret < 0) {
		shell_error(sh, "%s failed (%d)", withdraw ? "unpin" : "pin", ret);
	} else {
		shell_print(sh, "%s 0x%08x%s%s", withdraw ? "withdrew the pin on" : "pinned", node,
			    withdraw ? "" : " -> ", withdraw ? "" : argv[2]);
	}
	return ret;
}

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
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_fleet_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Base rows per class, pins, and what I should run.", NULL),
		  cmd_fleet_status),
	SHELL_CMD(desire, NULL,
		  SHELL_HELP("Publish the fleet's intent for an image class (master only).",
			     "<class> <maj.min.rev> [hash8hex] [pause|downgrade] [hw=<model>]"),
		  cmd_fleet_desire),
	SHELL_CMD(pin, NULL, SHELL_HELP("Pin one node to a version.",
					"<node-hex> <maj.min.rev> [hash8hex] [downgrade]"),
		  cmd_fleet_pin),
	SHELL_CMD(unpin, NULL, SHELL_HELP("Withdraw a node's pin (it falls back to base).",
					  "<node-hex>"),
		  cmd_fleet_pin),
#if defined(CONFIG_MESHTASTIC_DEPOT)
	SHELL_CMD(rescan, NULL, SHELL_HELP("Re-read /depot (after loading cargo).", NULL),
		  cmd_fleet_rescan),
#endif
#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)
	SHELL_CMD(arm, NULL, SHELL_HELP("Arm/disarm the courier loop.", "[on|off]"), cmd_fleet_arm),
	SHELL_CMD(clear, NULL, SHELL_HELP("Clear a node's reverted/refused/backoff latch.", "<node-hex>"),
		  cmd_fleet_clear),
#endif
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_FLEET */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_admin_cmds,
			       SHELL_CMD(trust, NULL,
					 SHELL_HELP("List/manage trusted remote-admin keys.",
						    "[<node> [off] | key:<hex-prefix> off]"),
					 cmd_admin_trust),
#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
			       SHELL_CMD(remote, NULL,
					 SHELL_HELP("Administer a peer node (PKC, any bearer).",
						    "<node> get owner | get config <t> | "
						    "set-owner <long> [short] | "
						    "set-time [<epoch>]"),
					 cmd_admin_remote),
#endif
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ADMIN */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_cmds,
	SHELL_CMD(status, NULL, SHELL_HELP("Show Meshtastic status.", NULL), cmd_status),
	SHELL_CMD(version, NULL, SHELL_HELP("Show build id / firmware version.", NULL),
		  cmd_version),
	SHELL_CMD(time, NULL,
		  SHELL_HELP("Show or set the wall clock.", "[set <unix-epoch-seconds>]"),
		  cmd_time),
	SHELL_CMD(owner, NULL,
		  SHELL_HELP("Show or set this node's owner names.",
			     "[set <long> [short]]"),
		  cmd_owner),
#if defined(CONFIG_MESHTASTIC_SHELL_CRASHTEST)
	SHELL_CMD(crashtest, NULL,
		  SHELL_HELP("Crash on purpose, to prove the coredump pipeline works "
			     "before a real fault needs it.", "<panic|stack>"),
		  cmd_crashtest),
#endif
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
#if defined(CONFIG_MESHTASTIC_ADMIN)
	SHELL_CMD(admin, &meshtastic_admin_cmds,
		  SHELL_HELP("Remote-admin trust and client commands.", NULL), NULL),
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
	SHELL_CMD(cluster, &meshtastic_cluster_cmds,
		  SHELL_HELP("Fleet config convergence.", NULL), cmd_cluster_status),
#endif
#if defined(CONFIG_MESHTASTIC_FLEET)
	SHELL_CMD(fleet, &meshtastic_fleet_cmds,
		  SHELL_HELP("Fleet firmware intent (what each class should run).", NULL),
		  cmd_fleet_status),
#endif
	SHELL_CMD(lora, NULL,
		  SHELL_HELP("Show or set the LoRa modem preset (reboot to apply) "
			     "or the TX-enable switch (applies live).",
			     "[preset <name>] [tx on|off]"),
		  cmd_lora),
#if defined(CONFIG_MESHTASTIC_BOOTLOG)
	SHELL_CMD(resets, NULL,
		  SHELL_HELP("Why this node last rebooted, and the boot history. "
			     "`durable` reads the FLASH ring, which survives power loss.",
			     "[durable]"),
		  cmd_resets),
#endif
	SHELL_CMD(crashinfo, NULL,
		  SHELL_HELP("Durable crash breadcrumbs: watchdog, hw-watchdog and "
			     "fatal-error info from the last abnormal reset.",
			     NULL),
		  cmd_crashinfo),
	SHELL_CMD(preset, NULL,
		  SHELL_HELP("Live preset switching: what would hold a scheduled hop off, "
			     "and the counters for the ones that were.",
			     "[hop <name> | reset]"),
		  cmd_preset),
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
#if defined(CONFIG_MESHTASTIC_AIRTIME)
	SHELL_CMD(airtime, NULL,
		  SHELL_HELP("Channel utilization per preset, and band-wide TX duty.", NULL),
		  cmd_airtime),
#endif
#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
	SHELL_CMD(duty, NULL,
		  SHELL_HELP("Regulatory duty cycle: ceiling, hour used, refusals.",
			     "[reset]"),
		  cmd_duty),
#endif
#if defined(CONFIG_MESHTASTIC_PHONELOG)
	SHELL_CMD(phonelog, NULL,
		  SHELL_HELP("Log forwarding to the phone: state + counters, or set the "
			     "severity ceiling.",
			     "[off|err|wrn|inf|dbg|reset]"),
		  cmd_phonelog),
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
