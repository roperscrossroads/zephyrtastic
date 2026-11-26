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
#include <zephyr/meshtastic/reboot_trace.h>
#include <zephyr/meshtastic/sparkplug.h>
#include <zephyr/meshtastic/intents.h>
#include <zephyr/sys/reboot.h>
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
#include <zephyr/meshtastic/gnss_pps.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include <zephyr/meshtastic/telemetry.h>

#include "meshtastic_storage.h" /* declarations only; safe unconditionally */
#if defined(CONFIG_MESHTASTIC_PKI)
#include "meshtastic_pki.h"
#endif
#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#include "meshtastic_admin_client.h"
#endif
#if defined(CONFIG_MESHTASTIC_BATTERY)
#include "meshtastic_battery.h"
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
#include "meshtastic_shell_internal.h"
#include "meshtastic_shell_state.h"
#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
#include "meshtastic_shell_api.h"
#endif
#if defined(CONFIG_MESHTASTIC_POSITION)
#include "meshtastic_position.h"
#endif
#if defined(CONFIG_MESHTASTIC_XEDDSA)
#include "meshtastic_xeddsa.h"
#endif
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
#include "meshtastic_statusmessage.h"
#endif
#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
#include "meshtastic_text_inbox.h"
#endif
#if defined(CONFIG_MESHTASTIC_RELAY) || defined(CONFIG_MESHTASTIC_RELAY_EAR)
#include "meshtastic_relay.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
#include "meshtastic_attachment.h"
#endif
#if defined(CONFIG_MESHTASTIC_LED_STATUS)
#include "meshtastic_led_status.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
#include "meshtastic_attachment_head.h"
#endif
#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
#include "meshtastic_neighborinfo.h"
#endif
#if defined(CONFIG_MESHTASTIC_MESHBEACON)
#include "meshtastic_meshbeacon.h"
#endif
#if defined(CONFIG_MESHTASTIC_TRAFFIC)
#include "meshtastic_traffic.h"
#endif
#if defined(CONFIG_MESHTASTIC_EXTNOTIFY)
#include "meshtastic_extnotify.h"
#endif
#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
#include "meshtastic_keyverify.h"
#include "meshtastic_lockdown.h"
#endif
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
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include "meshtastic_backup.h"
#endif
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
#include "meshtastic_telemetry_internal.h"
#endif

#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
#include <zephyr/logging/log_backend_net.h>
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* One writer per console. The serial shell prints the logs itself (its log backend) and
 * holds its output lock while a command prints; log_backend_uart writes the same UART with
 * uart_poll_out() from the log thread, outside that lock. Both on one UART print every log
 * line twice and put a log line's bytes inside a command's output: a `meshtastic api`
 * frame split that way reached the phone-API tunnel as a damaged line (agents-62n9). The
 * plain backend picks zephyr,log-uart when chosen, else zephyr,console. */
#if defined(CONFIG_LOG_BACKEND_UART) && defined(CONFIG_SHELL_BACKEND_SERIAL) &&                    \
	DT_HAS_CHOSEN(zephyr_shell_uart)
#if DT_HAS_CHOSEN(zephyr_log_uart)
#define MT_LOG_UART_NODE DT_CHOSEN(zephyr_log_uart)
#else
#define MT_LOG_UART_NODE DT_CHOSEN(zephyr_console)
#endif
BUILD_ASSERT(!DT_SAME_NODE(MT_LOG_UART_NODE, DT_CHOSEN(zephyr_shell_uart)),
	     "CONFIG_LOG_BACKEND_UART on the serial shell's UART: the shell's own log backend "
	     "already prints the logs there; drop CONFIG_LOG_BACKEND_UART");
#endif

enum admin_remote_op {
	ADMIN_REMOTE_GET_OWNER,
	ADMIN_REMOTE_GET_CONFIG,
	ADMIN_REMOTE_SET_TIME,
	ADMIN_REMOTE_SET_OWNER,
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

/* The pool holds the queued items' arguments; the queue carries pointers into it. An
 * item goes back to the pool as the thread takes it, so the depth is what it was when
 * the queue carried whole items: CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE waiting, one running. */
K_MEM_SLAB_DEFINE_STATIC(shell_work_slab, sizeof(struct mtsh_work), CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE,
			 4);
K_MSGQ_DEFINE(shell_work_msgq, sizeof(struct mtsh_work *), CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE, 4);
K_THREAD_STACK_DEFINE(shell_work_stack, CONFIG_MESHTASTIC_SHELL_WORK_STACK_SIZE);
static struct k_thread shell_work_thread;

static void shell_work_thread_fn(void *p1, void *p2, void *p3)
{
	struct mtsh_work *queued;
	struct mtsh_work item;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msgq_get(&shell_work_msgq, &queued, K_FOREVER);
		memcpy(&item, queued, sizeof(item));
		memset(queued, 0, sizeof(*queued)); /* a passphrase may have been here */
		k_mem_slab_free(&shell_work_slab, queued);

		ret = item.run(item.sh, item.arg, item.len);
		if (!item.quiet) {
			shell_report_result(item.sh, ret);
		}
		memset(item.arg, 0, sizeof(item.arg));
	}
}

/* An empty item for @p run, from the pool. NULL when the queue is full (said on @p sh). */
struct mtsh_work *mtsh_work_alloc(const struct shell *sh, mtsh_work_fn run)
{
	struct mtsh_work *w;

	if (k_mem_slab_alloc(&shell_work_slab, (void **)&w, K_NO_WAIT) != 0) {
		if (sh != NULL) {
			shell_error(sh, "Meshtastic shell queue is full");
		}
		return NULL;
	}
	memset(w, 0, sizeof(*w));
	w->sh = sh;
	w->run = run;
	return w;
}

/* Give an item back unqueued: its command refused its arguments after all. */
void mtsh_work_cancel(struct mtsh_work *w)
{
	memset(w, 0, sizeof(*w));
	k_mem_slab_free(&shell_work_slab, w);
}

/* Queue a filled item (@p len bytes of argument) and say "queued". */
int mtsh_work_submit(struct mtsh_work *w, size_t len)
{
	const struct shell *sh = w->sh;
	int ret;

	w->len = len;
	ret = k_msgq_put(&shell_work_msgq, &w, K_NO_WAIT);
	if (ret < 0) {
		/* Not reached: the queue has a slot for every pool item. */
		mtsh_work_cancel(w);
		if (sh != NULL) {
			shell_error(sh, "Meshtastic shell queue is full");
		}
		return ret;
	}
	if (sh != NULL && !w->quiet) {
		shell_print(sh, "queued");
	}
	return 0;
}

/* Queue @p run with a copy of @p len bytes at @p arg. */
int mtsh_enqueue(const struct shell *sh, mtsh_work_fn run, const void *arg, size_t len)
{
	struct mtsh_work *w;

	__ASSERT_NO_MSG(len <= sizeof(w->arg));
	w = mtsh_work_alloc(sh, run);
	if (w == NULL) {
		return -ENOMSG;
	}
	if (len > 0U) {
		memcpy(w->arg, arg, len);
	}
	return mtsh_work_submit(w, len);
}

/* A command run on the work thread for a waiting shell thread: mtsh_run_on_work(). */
struct mtsh_call {
	shell_cmd_handler handler;
	size_t argc;
	char **argv;
	int rc;
	struct k_sem done;
};

static int mtsh_call_run(const struct shell *sh, const void *arg, size_t len)
{
	struct mtsh_call *call;

	ARG_UNUSED(len);
	memcpy(&call, arg, sizeof(call));
	call->rc = call->handler(sh, call->argc, call->argv);
	k_sem_give(&call->done);
	return 0;
}

int mtsh_run_on_work(const struct shell *sh, shell_cmd_handler handler, size_t argc, char **argv)
{
	struct mtsh_call call = {.handler = handler, .argc = argc, .argv = argv};
	struct mtsh_call *ptr = &call;
	struct mtsh_work *w;

	/* Already on the work thread (a queued command that runs another): no one to
	 * wait for, so run it here. */
	if (k_current_get() == &shell_work_thread) {
		return handler(sh, argc, argv);
	}

	/* The caller waits anyway, so it also waits for a free item rather than being
	 * told the queue is full: a command that used to run inline must not start
	 * failing because sends are queued. */
	if (k_mem_slab_alloc(&shell_work_slab, (void **)&w, K_FOREVER) != 0) {
		return -ENOMSG;
	}
	memset(w, 0, sizeof(*w));
	w->sh = sh;
	w->run = mtsh_call_run;
	w->quiet = true;
	w->len = sizeof(ptr);
	memcpy(w->arg, &ptr, sizeof(ptr));
	k_sem_init(&call.done, 0, 1);
	(void)k_msgq_put(&shell_work_msgq, &w, K_FOREVER);
	(void)k_sem_take(&call.done, K_FOREVER);
	return call.rc;
}

/* The commands that stay on the shell thread. Every other `meshtastic` command runs on
 * the work thread through mtsh_run_on_work() (agents-ls39.4.4.1), and tests/shell
 * walks the registered tree to check that each command is one or the other. A path is
 * the words after `meshtastic`. */
const struct mtsh_shell_thread_cmd mtsh_shell_thread_cmds[] = {
	{"api", "waits on the work thread for each step: there it would wait on itself"},
	{"crashtest", "`stack` overflows the shell thread's own stack on purpose"},
	{"dfu", "reboots into the bootloader at once: nothing to protect"},
	{"netpause", "sleeps for N s by design: on the work thread it would hold queued sends"},
	{"lockdown provision", "prompts for the passphrase (shell_readline); its work is queued"},
	{"lockdown unlock", "prompts for the passphrase (shell_readline); its work is queued"},
	{"lockdown disable", "prompts for the passphrase (shell_readline); its work is queued"},
};

const size_t mtsh_shell_thread_cmds_len = ARRAY_SIZE(mtsh_shell_thread_cmds);

#if defined(CONFIG_ZTEST)
k_tid_t mtsh_test_work_thread(void)
{
	return &shell_work_thread;
}
#endif

/* The common argument: one node id. */
uint32_t mtsh_work_dest(const void *arg, size_t len)
{
	uint32_t dest;

	__ASSERT_NO_MSG(len == sizeof(dest));
	ARG_UNUSED(len);
	memcpy(&dest, arg, sizeof(dest));
	return dest;
}


/* Shared by the position / metrics / environment / nodeinfo "send" commands. A keyless
 * head (class 12) compiles all four out, and the fleet lane builds with -Werror, so the
 * helper is __unused there rather than guarded by a list of callers that would rot
 * (found by the lane 2026-09-29). */
static __unused int cmd_deferred_send(const struct shell *sh, size_t argc, char **argv,
				      mtsh_work_fn run)
{
	uint32_t dest;
	int ret;

	ret = mtsh_parse_optional_dest(sh, argc, argv, &dest);
	if (ret < 0) {
		return ret;
	}

	return mtsh_enqueue(sh, run, &dest, sizeof(dest));
}


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
	/* "class:  15 (r8-gateway)": the number first, so a reader that matches
	 * `^class:\s+(\d+)` (rxrbench's probe) still reads it. */
	if (CONFIG_MESHTASTIC_FLEET_CLASS == 0) {
		shell_print(sh, "class:  0 (unset: no courier will offer this node an image)");
	} else if (CONFIG_MESHTASTIC_FLEET_BUNDLE[0] != '\0') {
		shell_print(sh, "class:  %u (%s)", CONFIG_MESHTASTIC_FLEET_CLASS,
			    CONFIG_MESHTASTIC_FLEET_BUNDLE);
	} else {
		shell_print(sh, "class:  %u", CONFIG_MESHTASTIC_FLEET_CLASS);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_version)

static int cmd_state(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return meshtastic_shell_state(sh);
}
MTSH_DEFINE_ON_WORK(cmd_state)

#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
/* Prints nothing on the work thread: the shell thread is waiting for this step and
 * does the printing. */
static int shell_api_run(const struct shell *sh, const void *arg, size_t len)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	meshtastic_shell_api_work();
	return 0;
}

int meshtastic_shell_api_submit(void)
{
	struct mtsh_work *w = mtsh_work_alloc(NULL, shell_api_run);

	if (w == NULL) {
		return -ENOMSG;
	}
	w->quiet = true;
	return mtsh_work_submit(w, 0U);
}
#endif


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
		/* NOT a test of the double-exception breadcrumb, though it does end
		 * in one: on the ESP32-S3 the SRAM that holds IRAM code is the same
		 * memory as low DRAM at another address, and an unbounded recursion
		 * walking down from this stack reaches that alias and overwrites the
		 * code it then faults into -- the interrupt trampolines, the fatal
		 * handler, the recorder itself. That is why this has always ended in
		 * a bare WATCHDOG with nothing said. `dblexc` is the clean version. */
		shell_print(sh, "crashtest: overflowing this thread's stack on purpose");
		k_sleep(K_MSEC(200));
		crashtest_sink = crashtest_recurse(0);
	} else if (strcmp(argv[1], "dblexc") == 0) {
#if defined(CONFIG_XTENSA)
		/* A double exception with memory still intact: set PS.EXCM by hand,
		 * then load from address 0. The load raises LoadProhibited; because
		 * EXCM is already set the CPU takes the double-exception vector
		 * instead of the level-1 one. No handler runs there -- the hardware
		 * watchdog reboots the node ~30 s later, and `meshtastic resets` then
		 * shows the breadcrumb (CONFIG_XTENSA_FAULT_BREADCRUMBS) naming this
		 * thread, cause 28 and the l32i below as depc. `stack` is the same
		 * fault reached the dirty way, after the recursion has walked through
		 * whatever lay below this thread's stack. */
		shell_print(sh, "crashtest: raising a double exception on purpose (load from 0 "
				"with PS.EXCM set)");
		k_sleep(K_MSEC(200));
		(void)irq_lock();
		__asm__ volatile("rsr.ps a2\n"
				 "movi a3, 0x10\n" /* PS.EXCM */
				 "or a2, a2, a3\n"
				 "wsr.ps a2\n"
				 "rsync\n"
				 "movi a2, 0\n"
				 "l32i a2, a2, 0\n" ::: "a2", "a3", "memory");
#else
		shell_error(sh, "dblexc: Xtensa only");
		return -ENOTSUP;
#endif
	} else {
		shell_error(sh, "unknown kind '%s' (want panic, stack or dblexc)", argv[1]);
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
		shell_print(sh, "clock: epoch %u, source %s", epoch, mtsh_clock_quality_name(q));
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
	/* Managed only, not the lockdown gate: the clock is not sealed, and a
	 * token's expiry epoch needs one on a locked node. */
	if (mtsh_managed_refused(sh)) {
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
		    mtsh_clock_quality_name(meshtastic_clock_get_quality()));
	return 0;
#endif
}
MTSH_DEFINE_ON_WORK(cmd_time)

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
#if defined(CONFIG_MESHTASTIC_BATTERY)
	{
		int bmv = meshtastic_battery_millivolts();

		if (bmv < 0) {
			shell_print(sh, "battery: n/a");
		} else if (!meshtastic_battery_present()) {
			shell_print(sh, "battery: none fitted (%d mV on the divider)", bmv);
		} else {
			shell_print(sh, "battery: %d mV (%d%%)%s%s", bmv,
				    meshtastic_battery_percent(),
				    meshtastic_battery_external_power() ? " charging" : "",
				    meshtastic_battery_is_critical() ? " CRITICAL" : "");
		}
	}
#endif
	shell_print(sh, "tx: %u ok, %u failed", status.tx_packets, status.tx_failures);
	shell_print(sh, "rx: %u decoded, %u duplicates, %u decode failures", status.rx_packets,
		    status.duplicate_packets, status.decode_failures);
	shell_print(sh, "rx dropped: %u (%u injected), queue peak %u/%u, rx re-arm failures: %u",
		    status.rx_dropped, status.rx_dropped_injected, status.rx_queue_peak,
		    (unsigned int)CONFIG_MESHTASTIC_RX_QUEUE_DEPTH, status.rx_rearm_failures);
	shell_print(sh, "relayed: %u", status.relayed_packets);
	shell_print(sh, "last rx: from=0x%08x rssi=%d snr=%d", status.last_rx_from,
		    status.last_rssi, status.last_snr);
	shell_print(sh, "primary channel: %u \"%s\" hash=0x%02x",
		    (unsigned int)meshtastic_channels_primary_index(),
		    meshtastic_channels_primary_name(), meshtastic_channels_primary_hash());
	shell_print(sh, "device role: %s", mtsh_device_role_name(meshtastic_device_role()));
	shell_print(sh, "rebroadcast mode: %s",
		    mtsh_rebroadcast_mode_name(meshtastic_rebroadcast_mode()));
	shell_print(sh, "rebroadcasting: %s", meshtastic_is_rebroadcaster() ? "yes" : "no");

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_status)

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

/* Refuse a config read or write while lockdown holds the store: locked, or
 * unlocked but not yet reloaded (agents-ls39.1.8). RAM then holds the seed's
 * placeholders, not the node's config. A write would change them live and its
 * save would be refused by the wrap, with nothing said; a read would show them
 * as if they were the node's. The admin dispatcher drops every payload in the
 * same state; this is the console's half. `lockdown`, `version`, `status`,
 * `resets` and the clock stay usable: they are how the operator gets out.
 *
 * Here and not in the config store's setters for now: the store's one
 * config_write_allowed(origin) is agents-ls39.4.2. */
bool mtsh_store_locked(const struct shell *sh)
{
#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
	if (meshtastic_lockdown_active() && !meshtastic_lockdown_store_ready()) {
		shell_error(sh, "refused: the store is locked (`meshtastic lockdown unlock` "
				"first); config is not readable or writable until then");
		return true;
	}
#else
	ARG_UNUSED(sh);
#endif
	return false;
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
bool mtsh_managed_refused(const struct shell *sh)
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

/* Every config write: refused while the store is locked, then on a managed node. */
bool mtsh_config_write_refused(const struct shell *sh)
{
	return mtsh_store_locked(sh) || mtsh_managed_refused(sh);
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

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}

	for (uint8_t i = 0; i < MESHTASTIC_MAX_CHANNELS; i++) {
		shell_print_channel_line(sh, i);
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_channel_list)

static int cmd_channel_show(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t index;
	const meshtastic_Channel *ch;
	int ret;

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic channel show <index>");
		return -EINVAL;
	}

	ret = mtsh_parse_channel_index(sh, argv[1], &index);
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
	/* The precision our positions go out with on this slot (0 = never on it). */
	shell_print(sh, "position_precision: %u",
		    (unsigned int)ch->settings.module_settings.position_precision);
	if (ch->has_settings) {
		shell_print(sh, "uplink: %s", ch->settings.uplink_enabled ? "on" : "off");
		shell_print(sh, "downlink: %s", ch->settings.downlink_enabled ? "on" : "off");
	}
	shell_print_psk_summary(sh, ch);
	shell_print_psk_hex(sh, ch);

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_channel_show)

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

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	if (argc < 3U) {
		shell_error(sh, "usage: meshtastic channel set <index> "
				"[name <str>] [role primary|secondary|disabled] "
				"[psk none|default|<0-10>|hex <hex>] "
				"[uplink on|off] [downlink on|off]");
		return -EINVAL;
	}

	ret = mtsh_parse_channel_index(sh, argv[1], &index);
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
MTSH_DEFINE_ON_WORK(cmd_channel_set)

static int cmd_channel_disable(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t index;
	meshtastic_Channel ch;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic channel disable <index>");
		return -EINVAL;
	}

	ret = mtsh_parse_channel_index(sh, argv[1], &index);
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
MTSH_DEFINE_ON_WORK(cmd_channel_disable)
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_channel_cmds,
	SHELL_CMD(list, NULL, SHELL_HELP("List channel slots.", NULL), cmd_channel_list_on_work),
	SHELL_CMD(show, NULL, SHELL_HELP("Show one channel slot.", "<index>"), cmd_channel_show_on_work),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	SHELL_CMD(set, NULL,
		  SHELL_HELP("Update a channel slot.",
			     "<index> [name|role|psk|uplink|downlink]..."),
		  cmd_channel_set_on_work),
	SHELL_CMD(disable, NULL, SHELL_HELP("Disable a channel slot.", "<index>"),
		  cmd_channel_disable_on_work),
#endif
	SHELL_SUBCMD_SET_END);

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

#if defined(CONFIG_MESHTASTIC_NODEDB)
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

#endif /* CONFIG_MESHTASTIC_NODEDB */

/* The RF commands read the radio, not the NodeDB: they must exist on an image
 * without one (the keyless head is the first). */
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
			"false-preamble %u  false-header %u",
		    (p.rx_act_preamble == MESHTASTIC_RADIO_TRI_UNKNOWN) ? "??" : "--",
		    tri_str(p.rx_act_preamble), tri_str(p.rx_act_header), p.rx_act.busy_rx,
		    p.rx_act.false_preamble, p.rx_act.false_header);
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

#if defined(CONFIG_MESHTASTIC_NODEDB)
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
#endif /* CONFIG_MESHTASTIC_NODEDB */

#if defined(CONFIG_MESHTASTIC_MESSAGE)
struct shell_text_work {
	uint32_t dest;
	uint8_t channel_index; /* MESHTASTIC_CHANNEL_INDEX_INVALID: the default channel */
	uint8_t text[MESHTASTIC_MAX_TEXT_LEN + 1U]; /* NUL-terminated */
};

BUILD_ASSERT(sizeof(struct shell_text_work) <= MTSH_WORK_ARG_MAX,
	     "a text message rides in a shell work item");

static int shell_text_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_text_work *tw = arg;

	ARG_UNUSED(sh);
	if (tw->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
		struct meshtastic_packet packet = {
			.to = tw->dest,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = tw->text,
			.payload_len = len - offsetof(struct shell_text_work, text),
			.channel_index = tw->channel_index,
		};

		return meshtastic_send_packet(&packet, K_FOREVER);
	}
	return meshtastic_send_text(tw->dest, (const char *)tw->text);
}

static int cmd_text_send(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t channel_index = MESHTASTIC_CHANNEL_INDEX_INVALID;
	uint32_t dest = MESHTASTIC_NODE_BROADCAST;
	struct shell_text_work *tw;
	struct mtsh_work *w;
	size_t msg_arg = 1U;
	size_t first;
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
		ret = mtsh_parse_channel_index(sh, argv[2], &channel_index);
		if (ret < 0) {
			return ret;
		}
		msg_arg = 3U;
	}

	if (argc == (msg_arg + 1U)) {
		first = msg_arg;
	} else if (argc >= (msg_arg + 2U)) {
		ret = mtsh_parse_u32(sh, argv[msg_arg], &dest);
		if (ret < 0) {
			return ret;
		}
		first = msg_arg + 1U;
	} else {
		shell_error(sh, "message required");
		return -EINVAL;
	}

	/* The message is built in the queued item itself, not on this stack. */
	w = mtsh_work_alloc(sh, shell_text_run);
	if (w == NULL) {
		return -ENOMSG;
	}
	tw = (struct shell_text_work *)w->arg;
	tw->dest = dest;
	tw->channel_index = channel_index;
	len = mtsh_append_argv(tw->text, MESHTASTIC_MAX_TEXT_LEN, argc, argv, first);

	if (len < 0) {
		mtsh_work_cancel(w);
		shell_error(sh, "message too long (max %u)", MESHTASTIC_MAX_TEXT_LEN);
		return len;
	}

	if (len == 0) {
		mtsh_work_cancel(w);
		shell_error(sh, "message must not be empty");
		return -EINVAL;
	}

	tw->text[len] = '\0';

	return mtsh_work_submit(w, offsetof(struct shell_text_work, text) + (size_t)len);
}
MTSH_DEFINE_ON_WORK(cmd_text_send)

#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
/* One kept message as one line: what a test matches and a person reads. The text
 * comes last, so everything before it has a fixed shape; a byte below 0x20 or DEL
 * is shown as '?' so a message cannot break the line or the terminal. */
static void text_inbox_print(const struct shell *sh, const struct meshtastic_text_inbox_entry *e)
{
	char via[16];
	char text[CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN + 1];
	const char *on = "?";
	int64_t age_s = (k_uptime_get() - e->uptime_ms) / 1000;

	if (e->rx_heard_on > 0U) {
		on = meshtastic_preset_display_name(
			(meshtastic_Config_LoRaConfig_ModemPreset)(e->rx_heard_on - 1U), true);
	}
	if (e->rx_attach == 0U) {
		snprintk(via, sizeof(via), "own");
	} else {
		snprintk(via, sizeof(via), "head %u", e->rx_attach);
	}
	for (size_t i = 0; i <= e->len; i++) {
		unsigned char c = (unsigned char)e->text[i];

		text[i] = (i < e->len && (c < 0x20U || c == 0x7FU)) ? '?' : (char)c;
	}
	shell_print(sh, "#%u %llds ago from 0x%08x to 0x%08x ch %u on %s via %s rssi %d snr %d "
		    "hops %u%s%s: %s",
		    e->seq, (long long)age_s, e->from, e->to, e->channel_index, on, via, e->rssi,
		    e->snr, e->hops, e->pki ? " pki" : "", e->truncated ? " truncated" : "", text);
}

static int cmd_text_inbox(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_text_inbox_entry e;
	unsigned int show = CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES;

	if (argc > 1 && strcmp(argv[1], "clear") == 0) {
		meshtastic_text_inbox_clear();
		shell_print(sh, "inbox cleared");
		return 0;
	}
	if (argc > 1) {
		char *end;
		unsigned long n = strtoul(argv[1], &end, 10);

		if (*end != '\0' || n == 0UL) {
			shell_error(sh, "usage: meshtastic text inbox [<count>|clear]");
			return -EINVAL;
		}
		show = (unsigned int)MIN(n, (unsigned long)show);
	}

	shell_print(sh, "inbox: %u kept of %u, %u received since boot",
		    meshtastic_text_inbox_count(), CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES,
		    meshtastic_text_inbox_total());
	for (unsigned int i = 0; i < show && meshtastic_text_inbox_get(i, &e) == 0; i++) {
		text_inbox_print(sh, &e);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_text_inbox)
#endif /* CONFIG_MESHTASTIC_TEXT_INBOX */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_text_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send text message.",
						    "[-c <index>] [dest|broadcast] <message>"),
					 cmd_text_send_on_work),
#if defined(CONFIG_MESHTASTIC_TEXT_INBOX)
			       SHELL_CMD_ARG(inbox, NULL,
					     SHELL_HELP("The last text messages received, newest first.",
							"[<count>|clear]"),
					     cmd_text_inbox_on_work, 1, 1),
#endif
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_MESSAGE */

#if defined(CONFIG_MESHTASTIC_GNSS)
static int shell_position_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_position(dest);
}

static int cmd_gnss_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, shell_position_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_gnss_send)

static const char *gnss_fix_status_name(uint8_t s)
{
	switch (s) {
	case GNSS_FIX_STATUS_NO_FIX:
		return "no fix";
	case GNSS_FIX_STATUS_GNSS_FIX:
		return "GNSS fix";
	case GNSS_FIX_STATUS_DGNSS_FIX:
		return "DGNSS fix";
	case GNSS_FIX_STATUS_ESTIMATED_FIX:
		return "estimated";
	default:
		return "?";
	}
}

/* Seconds, or "never" — an age of -1 is "it has not happened", which a plain
 * number would misreport as "just now". */
static void shell_age(char *buf, size_t len, int64_t age_ms)
{
	if (age_ms < 0) {
		snprintf(buf, len, "never");
	} else {
		snprintf(buf, len, "%lld.%01llds ago", (long long)(age_ms / 1000),
			 (long long)((age_ms % 1000) / 100));
	}
}

/* `meshtastic gnss status` — read-only: is the receiver alive, is it tracking,
 * has it fixed, did the fix reach the clock and the air. Added 2026-09-02 after
 * a bench move to a window left four GNSS-fitted nodes with no way to say
 * whether "no fix yet" meant a cold start in progress or a dead receiver. */
static int cmd_gnss_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_gnss_status st;
	struct meshtastic_gnss_pps_stats pps;
	char a1[24], a2[24], a3[24];
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_gnss_status_get(&st);
	if (ret == -ENODEV) {
		shell_print(sh, "gnss: no receiver in this build's devicetree");
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "gnss status failed: %d", ret);
		return ret;
	}

	shell_print(sh, "receiver: %s (%s)", st.dev_name ? st.dev_name : "?",
		    st.ready ? "ready" : "NOT READY");
	shell_age(a1, sizeof(a1), st.last_data_age_ms);
	shell_print(sh, "data:     %u callbacks, last %s%s", st.callbacks, a1,
		    (st.callbacks == 0U) ? "  <- no NMEA at all: UART/power/reset line" : "");
	shell_print(sh, "tracking: %u satellites, %s, hdop %u.%02u", st.sats,
		    gnss_fix_status_name(st.fix_status), st.hdop_centi / 100U,
		    st.hdop_centi % 100U);
	shell_age(a2, sizeof(a2), st.last_fix_age_ms);
	shell_print(sh, "fix:      %s, %u fixes, last %s", st.has_fix ? "held" : "none yet",
		    st.fixes, a2);
	shell_age(a3, sizeof(a3), st.last_send_age_ms);
	shell_print(sh, "sent:     %u positions, last %s", st.sends, a3);

	meshtastic_gnss_pps_get_stats(&pps);
	if (pps.armed) {
		int64_t edge_age = (pps.count == 0U) ? -1 : k_uptime_get() - pps.last_uptime_ms;

		shell_age(a1, sizeof(a1), edge_age);
		shell_print(sh, "1PPS:     armed, %s, %u edges, last %s", pps.locked ? "LOCKED 1 Hz"
										     : "not locked",
			    pps.count, a1);
	} else {
		shell_print(sh, "1PPS:     not armed");
	}
	shell_print(sh, "clock:    %s", mtsh_clock_quality_name(meshtastic_clock_get_quality()));

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_gnss_status)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_gnss_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send GNSS position.", "[dest|broadcast]"),
					 cmd_gnss_send_on_work),
			       SHELL_CMD(status, NULL,
					 SHELL_HELP("Receiver, tracking, fix, 1PPS and clock — read-only.",
						    NULL),
					 cmd_gnss_status_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_GNSS */

#if defined(CONFIG_MESHTASTIC_POSITION)
/* `meshtastic position` (agents-t2hb.2) -- the two PositionConfig fields with a
 * consumer: gps_mode (the receiver's power) and position_broadcast_secs (both
 * senders' cadence). Same shape as `nodeinfo interval`: reads always available,
 * writes gated behind CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and is_managed. */
static const char *const gps_mode_names[] = {
	[meshtastic_Config_PositionConfig_GpsMode_DISABLED] = "disabled",
	[meshtastic_Config_PositionConfig_GpsMode_ENABLED] = "enabled",
	[meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT] = "not_present",
};

static const char *gps_mode_name(meshtastic_Config_PositionConfig_GpsMode mode)
{
	return ((size_t)mode < ARRAY_SIZE(gps_mode_names) && gps_mode_names[mode] != NULL)
		       ? gps_mode_names[mode]
		       : "?";
}

static const char *loc_source_name(meshtastic_Position_LocSource src)
{
	switch (src) {
	case meshtastic_Position_LocSource_LOC_MANUAL:
		return "fixed";
	case meshtastic_Position_LocSource_LOC_INTERNAL:
		return "gnss";
	case meshtastic_Position_LocSource_LOC_EXTERNAL:
		return "phone";
	default:
		return "unset";
	}
}

static int cmd_position_show(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config cfg;
	meshtastic_Position pos;
	bool have;
	bool stored;

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	have = meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg) == 0 &&
	       cfg.which_payload_variant == meshtastic_Config_position_tag;
	stored = have && cfg.payload_variant.position.position_broadcast_secs != 0U;

	shell_print(sh, "gps_mode: %s", gps_mode_name(meshtastic_position_gps_mode()));
#if defined(CONFIG_MESHTASTIC_GNSS)
	{
		struct meshtastic_gnss_status st;

		if (meshtastic_gnss_status_get(&st) == 0) {
			shell_print(sh, "receiver: %s", st.enabled ? "on" : "off");
		} else {
			shell_print(sh, "receiver: none in this build's devicetree");
		}
	}
#else
	shell_print(sh, "receiver: GNSS not compiled in");
#endif
	shell_print(sh, "interval: %u s%s", meshtastic_position_broadcast_secs(),
		    stored ? "" : " (compiled default; position_broadcast_secs unset)");
	/* The smart trio, the flags and whether anyone ever wrote the section: what
	 * a bench needs to explain a position that went out without altitude
	 * (2026-09-27 -- a pre-Track-C client write had stamped zeros into kit1). */
	if (have) {
		const meshtastic_Config_PositionConfig *pc = &cfg.payload_variant.position;
		struct meshtastic_hlc_stamp stamp;
		bool written = meshtastic_config_store_get_config_stamp(
				       meshtastic_Config_position_tag, &stamp) == 0 &&
			       !meshtastic_hlc_stamp_is_unset(&stamp);

		shell_print(sh, "smart: %s (%u m, %u s)",
			    pc->position_broadcast_smart_enabled ? "on" : "off",
			    pc->broadcast_smart_minimum_distance != 0U
				    ? pc->broadcast_smart_minimum_distance : 100U,
			    pc->broadcast_smart_minimum_interval_secs != 0U
				    ? pc->broadcast_smart_minimum_interval_secs : 300U);
		shell_print(sh, "flags: 0x%x%s", pc->position_flags,
			    pc->position_flags == 0U ? " (lat/lon/time only)" : "");
		shell_print(sh, "section: %s",
			    written ? "written (stamped by a client, the shell or a cluster doc)"
				    : "never written (build seed)");
	}
	if (meshtastic_position_get_current(&pos) == 0) {
		shell_print(sh, "position: lat=%d lon=%d alt=%d (%s)", pos.latitude_i,
			    pos.longitude_i, pos.altitude,
			    loc_source_name(pos.location_source));
	} else {
		shell_print(sh, "position: none");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_position_show)

/* Bare `meshtastic position` shows. Anything else reaching here is a subcommand
 * this build does not have -- the write forms, when config writes are compiled
 * out -- and must fail rather than print the status and report success. */
static int cmd_position_root(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1U) {
		shell_error(sh, "unknown subcommand: %s", argv[1]);
		return -EINVAL;
	}
	return cmd_position_show(sh, argc, argv);
}
MTSH_DEFINE_ON_WORK(cmd_position_root)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_position_interval(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long secs;
	char *end;
	int ret;

	if (argc == 1U) {
		return cmd_position_show(sh, argc, argv);
	}
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	secs = strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs > UINT32_MAX) {
		shell_error(sh, "invalid seconds value: %s", argv[1]);
		return -EINVAL;
	}
	ret = meshtastic_position_set_broadcast_secs((uint32_t)secs);
	if (ret < 0) {
		shell_error(sh, "interval set failed: %d", ret);
		return ret;
	}
	shell_print(sh, "interval -> %u s%s (persisted; applies now, no reboot)",
		    meshtastic_position_broadcast_secs(),
		    (secs == 0UL) ? " (compiled default)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_position_interval)

static int cmd_position_gps_mode(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc == 1U) {
		return cmd_position_show(sh, argc, argv);
	}
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	for (size_t i = 0U; i < ARRAY_SIZE(gps_mode_names); i++) {
		if (gps_mode_names[i] != NULL && strcmp(argv[1], gps_mode_names[i]) == 0) {
			ret = meshtastic_position_set_gps_mode(
				(meshtastic_Config_PositionConfig_GpsMode)i);
			if (ret < 0) {
				shell_error(sh, "gps_mode set failed: %d", ret);
				return ret;
			}
			shell_print(sh, "gps_mode -> %s (persisted; applies now, no reboot)",
				    gps_mode_names[i]);
			return 0;
		}
	}
	shell_error(sh, "usage: meshtastic position gps_mode [enabled|disabled|not_present]");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_position_gps_mode)
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_position_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("gps_mode, broadcast interval, current position.", NULL),
		  cmd_position_show_on_work),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	SHELL_CMD_ARG(interval, NULL,
		      SHELL_HELP("Show/set position_broadcast_secs (0 = compiled default).",
				 "[secs]"),
		      cmd_position_interval_on_work, 1, 1),
	SHELL_CMD_ARG(gps_mode, NULL,
		      SHELL_HELP("Show/set PositionConfig.gps_mode.",
				 "[enabled|disabled|not_present]"),
		      cmd_position_gps_mode_on_work, 1, 1),
#endif
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_POSITION */

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
	int32_t tx10 = mtsh_scaled_tenths(meshtastic_airtime_tx_util_percent());
	bool any = false;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "tx (band-wide, 1 h): %d.%u%%  <- what the duty cycle measures",
		    mtsh_scaled_whole(tx10, 10), mtsh_scaled_fraction(tx10, 10));
	shell_print(sh, "channel (per preset, 60 s)  <- what CSMA and the beacon gate read");

	for (uint8_t slot = 0; slot < MESHTASTIC_AIRTIME_PRESET_SLOTS; slot++) {
		float pct = meshtastic_airtime_channel_util_percent_slot(slot);
		int32_t p10 = mtsh_scaled_tenths(pct);
		const char *name;

		if (p10 == 0 && slot != cur) {
			continue; /* quiet and not where we are: nothing to say */
		}
		any = true;

		name = (slot == MESHTASTIC_AIRTIME_SLOT_CUSTOM)
			       ? "custom"
			       : meshtastic_preset_display_name(
					 (meshtastic_Config_LoRaConfig_ModemPreset)slot, true);

		shell_print(sh, "  %-12s %d.%u%%%s", name, mtsh_scaled_whole(p10, 10),
			    mtsh_scaled_fraction(p10, 10), (slot == cur) ? "   <- on air now" : "");
	}

	if (!any) {
		shell_print(sh, "  (all quiet)");
	}

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_airtime)
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
	int32_t c10 = mtsh_scaled_tenths(ceiling), u10 = mtsh_scaled_tenths(used);
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
		shell_print(sh, "ceiling : %d.%u%%", mtsh_scaled_whole(c10, 10),
			    mtsh_scaled_fraction(c10, 10));
	}
	shell_print(sh, "used    : %d.%u%% of the last hour", mtsh_scaled_whole(u10, 10),
		    mtsh_scaled_fraction(u10, 10));
	shell_print(sh, "state   : %s", blocked ? "BLOCKING egress" : "sending allowed");
	if (blocked) {
		shell_print(sh, "clear in: %u min", silent);
	}
	shell_print(sh, "refused : %u (%u of them relays)", stats.blocked, stats.blocked_relay);

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_duty)
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
MTSH_DEFINE_ON_WORK(cmd_phonelog)
#endif /* CONFIG_MESHTASTIC_PHONELOG */

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
static int shell_metrics_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_device_metrics(dest, K_FOREVER);
}

static int cmd_metrics_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, shell_metrics_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_metrics_send)

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

	chan_util = mtsh_scaled_tenths(stats.channel_utilization);
	tx_util = mtsh_scaled_tenths(stats.air_util_tx);

	shell_print(sh, "uptime  : %us", stats.uptime_seconds);
	shell_print(sh, "util    : chan %d.%u%% tx %d.%u%%", mtsh_scaled_whole(chan_util, 10),
		    mtsh_scaled_fraction(chan_util, 10), mtsh_scaled_whole(tx_util, 10),
		    mtsh_scaled_fraction(tx_util, 10));
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
MTSH_DEFINE_ON_WORK(cmd_metrics_localstats)
#endif /* CONFIG_MESHTASTIC_LOCAL_STATS */

/* The resolved broadcast cadence: what ModuleConfig.telemetry, the role, the
 * default-channel coercion and the online-node scaling add up to right now. */
static int cmd_metrics_cadence(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_telemetry_settings s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_telemetry_settings(&s);
	shell_print(sh, "device      : %s, every %u s", s.device_enabled ? "on" : "off",
		    s.device_interval_sec);
	shell_print(sh, "environment : %s, every %u s", s.environment_enabled ? "on" : "off",
		    s.environment_interval_sec);
	shell_print(sh, "scaled by   : %u online nodes", s.online_nodes);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_metrics_cadence)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_metrics_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send device metrics.", "[dest|broadcast]"),
					 cmd_metrics_send_on_work),
			       SHELL_CMD(cadence, NULL,
					 SHELL_HELP("Show the resolved telemetry broadcast "
						    "cadence (ModuleConfig.telemetry).", NULL),
					 cmd_metrics_cadence_on_work),
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
			       SHELL_CMD(localstats, NULL,
					 SHELL_HELP("Show LocalStats (mesh counters). "
						    "`push` also sends one to the phone.",
						    "[push]"),
					 cmd_metrics_localstats_on_work),
#endif
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_DEVICE_METRICS */

#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
static int shell_environment_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_environment(dest, K_FOREVER);
}

static int cmd_environment_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, shell_environment_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_environment_send)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_environment_cmds,
	SHELL_CMD(send, NULL, SHELL_HELP("Send environment telemetry.", "[dest|broadcast]"),
		  cmd_environment_send_on_work),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ENVIRONMENT_METRICS */

#if defined(CONFIG_MESHTASTIC_NODEINFO)
static int shell_nodeinfo_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_node_info(dest);
}

static int cmd_nodeinfo_send(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_deferred_send(sh, argc, argv, shell_nodeinfo_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_send)

/* Unlike `send`, this actually asks: want_response=true, so a compliant peer
 * replies with ITS NodeInfo. It goes through the same work queue `send` does: the
 * request runs the whole send path, which the shell thread's stack cannot carry on
 * a XIAO (CONFIG_SHELL_STACK_SIZE=2048; run inline it reset the board every time).
 * The per-peer cooldown and the K_NO_WAIT send live in meshtastic_nodeinfo_request(). */
/* Bench 2026-10-09: run inline, `nodeinfo request` rebooted kit1 every time
 * (agents-yhdm). */
static int shell_nodeinfo_request_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);
	int ret;

	ret = meshtastic_nodeinfo_request(dest);
	if (ret == -EAGAIN) {
		shell_print(sh, "suppressed: already requested 0x%08x within "
			    "the last %d s (CONFIG_MESHTASTIC_NODEINFO_UNKNOWN_SUPPRESS_SEC)",
			    dest, CONFIG_MESHTASTIC_NODEINFO_UNKNOWN_SUPPRESS_SEC);
		ret = 0;
	} else if (ret == 0) {
		shell_print(sh, "requested -- watch for a NodeInfo reply from "
			    "0x%08x", dest);
	}
	return ret;
}

static int cmd_nodeinfo_request(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dest;
	int ret;

	ARG_UNUSED(argc);

	ret = mtsh_parse_u32(sh, argv[1], &dest);
	if (ret < 0) {
		return ret;
	}

	return mtsh_enqueue(sh, shell_nodeinfo_request_run, &dest, sizeof(dest));
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_request)

/* Get/set device.node_info_broadcast_secs (agents-t2hb.1). Same shape as
 * `meshtastic device role`/`rebroadcast`: read always available, write gated
 * behind CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE. */
static int cmd_nodeinfo_interval(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		meshtastic_Config cfg;
		bool stored = meshtastic_config_store_get_config(meshtastic_Config_device_tag,
								 &cfg) == 0 &&
			      cfg.which_payload_variant == meshtastic_Config_device_tag &&
			      cfg.payload_variant.device.node_info_broadcast_secs != 0U;

		shell_print(sh, "interval: %u s%s", meshtastic_nodeinfo_interval_secs(),
			    stored ? "" : " (compiled default; node_info_broadcast_secs unset)");
		return 0;
	}

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic nodeinfo interval [secs]");
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		unsigned long secs;
		char *end;
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}

		secs = strtoul(argv[1], &end, 10);
		if (*end != '\0' || secs > UINT32_MAX) {
			shell_error(sh, "invalid seconds value: %s", argv[1]);
			return -EINVAL;
		}

		ret = meshtastic_config_store_set_node_info_interval((uint32_t)secs);
		if (ret < 0) {
			shell_error(sh, "interval set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "interval -> %u s%s (persisted; next broadcast uses it, no reboot)",
			    meshtastic_nodeinfo_interval_secs(),
			    secs == 0U ? " (0 == compiled default)" : "");
		return 0;
	}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_nodeinfo_interval)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_nodeinfo_cmds,
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Send node information.", "[dest|broadcast]"),
					 cmd_nodeinfo_send_on_work),
			       SHELL_CMD_ARG(request, NULL,
					     SHELL_HELP("Ask a peer for ITS NodeInfo "
							"(want_response=true).",
							"<dest>"),
					     cmd_nodeinfo_request_on_work, 2, 0),
			       SHELL_CMD(interval, NULL,
					 SHELL_HELP("Get/set the auto-broadcast interval "
						    "(0 = compiled default).",
						    "[secs]"),
					 cmd_nodeinfo_interval_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_NODEINFO */

#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
/* `meshtastic status` -- the node's away-message (agents-dnr4.26). Same shape as
 * `nodeinfo interval`: reads always available, writes gated behind
 * CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and the is_managed policy. */
static int cmd_status_show(const struct shell *sh, size_t argc, char **argv)
{
	char status[80];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (meshtastic_statusmessage_get(status, sizeof(status)) == 0U) {
		shell_print(sh, "status: (none)");
	} else {
		shell_print(sh, "status: \"%s\"", status);
	}
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)
	shell_print(sh, "auto-send: %d s after a change, then every %d s",
		    CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC,
		    CONFIG_MESHTASTIC_STATUSMESSAGE_INTERVAL_SEC);
#else
	shell_print(sh, "auto-send: off (CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND=n)");
#endif
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_status_show)

static int cmd_status_write(const struct shell *sh, const char *text)
{
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	ARG_UNUSED(text);
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	ret = meshtastic_statusmessage_set(text);
	if (ret < 0) {
		shell_error(sh, "status set failed: %d", ret);
		return ret;
	}
	if (text == NULL || text[0] == '\0') {
		shell_print(sh, "status cleared (persisted; announce cancelled, no reboot)");
	} else {
		char stored[80];

		(void)meshtastic_statusmessage_get(stored, sizeof(stored));
		shell_print(sh, "status -> \"%s\" (persisted; announce re-armed, no reboot)",
			    stored);
	}
	return 0;
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

static int cmd_status_set(const struct shell *sh, size_t argc, char **argv)
{
	char text[80];
	size_t len = 0U;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic statusmsg set <text...>");
		return -EINVAL;
	}

	/* Join the words back: the shell split them, the status is one string. */
	text[0] = '\0';
	for (size_t i = 1; i < argc; i++) {
		int n = snprintk(text + len, sizeof(text) - len, "%s%s", (i > 1U) ? " " : "",
				 argv[i]);

		if (n < 0 || (size_t)n >= sizeof(text) - len) {
			shell_warn(sh, "status truncated to %zu chars", sizeof(text) - 1U);
			break;
		}
		len += (size_t)n;
	}

	return cmd_status_write(sh, text);
}
MTSH_DEFINE_ON_WORK(cmd_status_set)

static int cmd_status_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return cmd_status_write(sh, "");
}
MTSH_DEFINE_ON_WORK(cmd_status_clear)

static int shell_status_send_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_statusmessage_send();
	if (ret == -ENODATA) {
		shell_error(sh, "no status set (meshtastic statusmsg set <text>)");
	} else if (ret == 0) {
		shell_print(sh, "status broadcast sent");
	}
	return ret;
}

static int cmd_status_send(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_status_send_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_status_send)

static int cmd_status_peers(const struct shell *sh, size_t argc, char **argv)
{
	char status[80];
	uint32_t node;
	int64_t age_ms;
	size_t shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < CONFIG_MESHTASTIC_STATUSMESSAGE_CACHE_SIZE; i++) {
		if (!meshtastic_statusmessage_peer_at(i, &node, status, sizeof(status), &age_ms)) {
			continue;
		}
		shell_print(sh, "0x%08x  %6lld s ago  \"%s\"", node, (long long)(age_ms / 1000),
			    status);
		shown++;
	}
	if (shown == 0U) {
		shell_print(sh, "no peer status heard yet");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_status_peers)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_status_cmds,
			       SHELL_CMD(set, NULL,
					 SHELL_HELP("Set and persist the status; re-arms the "
						    "announce, no reboot.",
						    "<text...>"),
					 cmd_status_set_on_work),
			       SHELL_CMD(clear, NULL,
					 SHELL_HELP("Clear the status and cancel the announce.",
						    NULL),
					 cmd_status_clear_on_work),
			       SHELL_CMD(send, NULL,
					 SHELL_HELP("Broadcast the status now.", NULL),
					 cmd_status_send_on_work),
			       SHELL_CMD(peers, NULL,
					 SHELL_HELP("Last status heard from each peer.", NULL),
					 cmd_status_peers_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_STATUSMESSAGE */

#if defined(CONFIG_MESHTASTIC_RELAY)
/* `meshtastic relay` (agents-jbrq.12): the cross-preset relay's receiving
 * half. The direction and ignore list are saved (MESHTASTIC_SETTINGS); `reset`
 * forgets them. */
static const char *const relay_dir_names[] = { "off", "in", "out", "both" };

static int cmd_relay_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_relay_stats st;
	enum meshtastic_relay_dir dir = meshtastic_relay_get_direction();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_relay_stats_get(&st);
	shell_print(sh, "direction: %s", relay_dir_names[dir & 3]);
	shell_print(sh, "considered %u  relayed %u  sent %u  tx_failed %u", st.considered,
		    st.relayed, st.sent, st.tx_failed);
	shell_print(sh, "refused: dir_off %u  not_broadcast %u  not_text %u  reaction %u  ignored %u",
		    st.dir_off, st.not_broadcast, st.not_text, st.reaction, st.ignored);
	shell_print(sh, "         bad_text %u  prefixed %u  no_mapping %u  seen %u",
		    st.bad_text, st.prefixed, st.no_mapping, st.seen);
	shell_print(sh, "         rate_dropped %u  origin_limited %u  queue_full %u", st.rate_dropped,
		    st.origin_limited, st.queue_full);
	shell_print(sh, "relayed-but: sanitized %u  unprefixed %u;  log lines held back %u",
		    st.sanitized, st.unprefixed, st.log_suppressed);
	shell_print(sh, "cap %d per %d s, per origin %d per %d s, seen TTL %d s",
		    CONFIG_MESHTASTIC_RELAY_RATE_MAX, CONFIG_MESHTASTIC_RELAY_RATE_WINDOW_SEC,
		    CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_MAX,
		    CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_WINDOW_SEC,
		    CONFIG_MESHTASTIC_RELAY_SEEN_TTL_SEC);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_show)

static int cmd_relay_dir(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	for (size_t i = 0U; i < ARRAY_SIZE(relay_dir_names); i++) {
		if (strcmp(argv[1], relay_dir_names[i]) == 0) {
			ret = meshtastic_relay_set_direction((enum meshtastic_relay_dir)i);
			if (ret == -ENOTSUP) {
				shell_error(sh, "%s needs an ear that transmits (not in v1)",
					    argv[1]);
			} else if (ret < 0) {
				shell_error(sh, "failed (%d)", ret);
			} else {
				shell_print(sh, "direction: %s", argv[1]);
			}
			return ret;
		}
	}
	shell_error(sh, "usage: meshtastic relay dir <off|in|out|both>");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_relay_dir)

static int cmd_relay_ignore(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node;
	int ret;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "clear") == 0) {
		meshtastic_relay_ignore_clear();
		shell_print(sh, "ignore list cleared");
		return 0;
	}
	node = (uint32_t)strtoul(argv[1], NULL, 16);
	if (node == 0U) {
		shell_error(sh, "usage: meshtastic relay ignore <hex node id>|clear");
		return -EINVAL;
	}
	ret = meshtastic_relay_ignore_add(node);
	if (ret < 0) {
		shell_error(sh, "ignore list full (%d)", ret);
		return ret;
	}
	shell_print(sh, "never relaying 0x%08x", node);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_ignore)

static int cmd_relay_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	meshtastic_relay_reset();
	shell_print(sh, "relay reset: boot direction, caches, counters, ignore list; saved settings forgotten");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_relay_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_relay_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Direction and counters.", NULL), cmd_relay_show_on_work),
	SHELL_CMD_ARG(dir, NULL,
		      SHELL_HELP("Set the direction (v1: off or in).", "<off|in|out|both>"),
		      cmd_relay_dir_on_work, 2, 0),
	SHELL_CMD_ARG(ignore, NULL,
		      SHELL_HELP("Never relay a node (another relay), or clear the list.",
				 "<hex node id>|clear"),
		      cmd_relay_ignore_on_work, 2, 0),
	SHELL_CMD(reset, NULL, SHELL_HELP("Back to boot state; forgets the saved settings.", NULL),
		  cmd_relay_reset_on_work),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_RELAY */

#if defined(CONFIG_MESHTASTIC_RELAY_EAR)
/* `meshtastic ear`: the relay's ear. The peer is saved (MESHTASTIC_SETTINGS). */
static int cmd_ear_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_relay_ear_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_relay_ear_stats_get(&st);
	shell_print(sh, "peer: 0x%08x%s", meshtastic_relay_ear_get_peer(),
		    meshtastic_relay_ear_get_peer() == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "heard %u  forwarded %u  send_failed %u  queue_full %u", st.heard,
		    st.forwarded, st.send_failed, st.queue_full);
	shell_print(sh, "skipped: not_text %u  not_broadcast %u  no_peer %u  rate_dropped %u",
		    st.not_text, st.not_broadcast, st.no_peer, st.rate_dropped);
	shell_print(sh, "cap: %d frames per %d s", CONFIG_MESHTASTIC_RELAY_EAR_RATE_MAX,
		    CONFIG_MESHTASTIC_RELAY_EAR_RATE_WINDOW_SEC);
	shell_print(sh, "transmit: %s",
		    IS_ENABLED(CONFIG_MESHTASTIC_RELAY_EAR_RX_ONLY) ? "compiled out" : "possible");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_ear_show)

static int cmd_ear_peer(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_relay_ear_set_peer(node);
	shell_print(sh, "peer: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_ear_peer)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_ear_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Peer and counters.", NULL), cmd_ear_show_on_work),
	SHELL_CMD_ARG(peer, NULL,
		      SHELL_HELP("The receiving half's node id (0 stops forwarding).",
				 "<hex node id>"),
		      cmd_ear_peer_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_RELAY_EAR */

#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
/* `meshtastic attach`: the radios this brain hears through (ATTACHMENT-DESIGN
 * S6). Attachment 0 is this board's own radio; the rest are heads that spoke
 * to us over the peer link. */
static void attach_print_row(const struct shell *sh, const struct meshtastic_attachment_info *a)
{
	if (a->id == 0U) {
		shell_print(sh, "  0  local            preset %u  last %d dBm / %d dB", a->preset,
			    a->last_rssi, a->last_snr);
		return;
	}
	shell_print(sh, "  %u  0x%08x  preset %u  rx %u (dropped %u)  tx %u  last %d dBm / %d dB"
		    "  [%d..%d]%s%s",
		    a->id, a->node, a->preset, a->rx_frames, a->rx_dropped, a->tx_frames,
		    a->last_rssi, a->last_snr, a->rssi_min, a->rssi_max,
		    a->have_status ? "  status" : "",
		    (a->have_status &&
		     (a->status.flags & MESHTASTIC_ATTACHMENT_ST_IS_HEAD) != 0U) ? " head" : " ear");
	{
		uint8_t wanted = meshtastic_attachment_wanted_preset(a->node);

		if (wanted != MESHTASTIC_PRESET_UNKNOWN) {
			shell_print(sh, "     wanted preset %u%s", (unsigned int)wanted,
				    wanted == a->preset ? "" : "  (NOT there yet)");
		}
	}
	if (a->warn_fast_head || a->warn_lag) {
		shell_print(sh, "     %s%s  (DESIGN §13: the fastest preset belongs on the brain's radio)",
			    a->warn_fast_head ? "FAST-HEAD " : "", a->warn_lag ? "LAG " : "");
	}
	if (!a->link_up) {
		shell_print(sh, "     link DOWN %lld s (evicted after %d s)",
			    (long long)((k_uptime_get() - a->down_ms) / 1000),
			    CONFIG_MESHTASTIC_ATTACHMENT_EVICT_SEC);
	}
}

static int cmd_attach_allow(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_attachment_stats st;
	uint32_t node;

	if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
		meshtastic_attachment_allow_clear();
		shell_print(sh, "allow-list cleared");
		return 0;
	}
	if (argc >= 2) {
		int ret;

		node = (uint32_t)strtoul(argv[1], NULL, 16);
		ret = meshtastic_attachment_allow_add(node);
		if (ret < 0) {
			shell_error(sh, "allow failed (%d)", ret);
			return ret;
		}
		shell_print(sh, "0x%08x allowed", node);
		return 0;
	}
	shell_print(sh, "allowed on an untrusted link:");
	for (unsigned int i = 0U; meshtastic_attachment_allow_get(i, &node); i++) {
		shell_print(sh, "  0x%08x", node);
	}
	meshtastic_attachment_stats_get(&st);
	shell_print(sh, "refused %u  malformed-first-contact %u  evicted %u", st.admission_refused,
		    st.not_admitted_malformed, st.evicted);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_allow)

static int cmd_attach_list(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_attachment_info a;
	unsigned int shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "attachments: %u (of %u + local)", meshtastic_attachment_count(),
		    (unsigned int)CONFIG_MESHTASTIC_ATTACHMENT_MAX);
	for (uint8_t id = 0U; id <= CONFIG_MESHTASTIC_ATTACHMENT_MAX; id++) {
		if (meshtastic_attachment_get(id, &a)) {
			attach_print_row(sh, &a);
			shown++;
		}
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_list)

static int cmd_attach_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_attachment_info a;
	unsigned long id = strtoul(argv[1], NULL, 10);

	ARG_UNUSED(argc);
	if (id > CONFIG_MESHTASTIC_ATTACHMENT_MAX || !meshtastic_attachment_get((uint8_t)id, &a)) {
		shell_error(sh, "no attachment %lu", id);
		return -ENOENT;
	}
	attach_print_row(sh, &a);
	if (a.have_status) {
		shell_print(sh, "  reports: preset %u flags 0x%02x hwid 0x%08x brain 0x%08x",
			    a.status.preset, a.status.flags, a.status.hwid, a.status.brain);
		shell_print(sh, "  reports: rx %u tx %u dropped %u uptime %u s", a.status.rx_frames,
			    a.status.tx_frames, a.status.rx_dropped, a.status.uptime_s);
		if ((a.status.flags & MESHTASTIC_ATTACHMENT_ST_HAS_POS) != 0U) {
			shell_print(sh, "  position: lat=%d lon=%d alt=%d", a.status.lat,
				    a.status.lon, a.status.alt);
		}
	}
	if (a.lag_n != 0U) {
		shell_print(sh, "  link lag (relay decisions, head's clock): n %u  p50 %u  p90 %u  max %u ms%s%s",
			    a.lag_n, a.lag_p50_ms, a.lag_p90_ms, a.lag_max_ms,
			    a.warn_lag ? "  LAG" : "", a.warn_fast_head ? "  FAST-HEAD" : "");
	}
	shell_print(sh, "  last envelope %lld ms ago  rejected %u  tx_results %u",
		    (long long)(k_uptime_get() - a.last_ms), a.rejected, a.tx_results);
	if (a.lat_n != 0U) {
		shell_print(sh, "  link latency vs local radio: n %u  min %u  avg %u  max %u ms",
			    a.lat_n, a.lat_min_ms, a.lat_sum_ms / a.lat_n, a.lat_max_ms);
	}
	if (a.id != 0U) {
		shell_print(sh, "  tx: handed %u (seq %u)  results %u  cancelled %u  late %u  failed %u  "
				"last rc %d after %u defers%s",
			    a.tx_frames, a.tx_seq, a.tx_results, a.tx_cancelled, a.tx_late,
			    a.tx_failed, a.last_tx_rc,
			    a.last_tx_defers, meshtastic_attachment_tx_ready(a.id) ? "" : "  [not ready]");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_status)

/* A preset as a name (ShortFast), its number (6), or "none". Anything else is refused: a
 * name used to parse as the number 0, which is LongFast, and put the head there. */
static int attach_parse_preset(const char *s, uint8_t *out)
{
	meshtastic_Config_LoRaConfig_ModemPreset named;
	unsigned long n;
	char *end;

	if (strcmp(s, "none") == 0) {
		*out = MESHTASTIC_PRESET_UNKNOWN;
		return 0;
	}
	if (mtsh_parse_modem_preset(s, &named) == 0) {
		*out = (uint8_t)named;
		return 0;
	}
	n = strtoul(s, &end, 10);
	if (end == s || *end != '\0' ||
	    n > (unsigned long)_meshtastic_Config_LoRaConfig_ModemPreset_MAX) {
		return -EINVAL;
	}
	*out = (uint8_t)n;
	return 0;
}

static int cmd_attach_preset(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long id = strtoul(argv[1], &end, 10);
	uint8_t preset;
	int ret;

	ARG_UNUSED(argc);
	if (end == argv[1] || *end != '\0' || id == 0U || id > CONFIG_MESHTASTIC_ATTACHMENT_MAX) {
		shell_error(sh, "usage: meshtastic attach preset <id> <name|number|none>");
		return -EINVAL;
	}
	if (attach_parse_preset(argv[2], &preset) < 0) {
		shell_error(sh, "unknown preset '%s' (a name as `meshtastic lora` prints it, "
				"e.g. ShortFast; its number; or none)", argv[2]);
		return -EINVAL;
	}
	ret = meshtastic_attachment_set_preset((uint8_t)id, preset);
	if (preset == MESHTASTIC_PRESET_UNKNOWN) {
		if (ret == 0) {
			shell_print(sh, "attachment %lu: no preset wanted of it any more", id);
		} else {
			shell_error(sh, "failed (%d)", ret);
		}
		return ret;
	}
	if (ret == -EINVAL || ret == -ENOENT || ret == -ENOSPC) {
		shell_error(sh, "set_preset failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "attachment %lu: preset %u wanted and stored; %s", id,
		    (unsigned int)preset,
		    ret == 0 ? "asked it to retune"
			     : "the link is down, it is asked when it next speaks");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_preset)

static int cmd_attach_power(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long id = strtoul(argv[1], NULL, 10);
	long dbm = strtol(argv[2], NULL, 10);
	int ret;

	ARG_UNUSED(argc);
	if (id == 0U || id > CONFIG_MESHTASTIC_ATTACHMENT_MAX || dbm < -128 || dbm > 127) {
		shell_error(sh, "usage: meshtastic attach power <id> <dBm> (0 = region max)");
		return -EINVAL;
	}
	ret = meshtastic_attachment_set_tx_power((uint8_t)id, (int8_t)dbm);
	if (ret < 0) {
		shell_error(sh, "set_policy failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "asked attachment %lu to transmit at %ld dBm (it persists it and reports)", id,
		    dbm);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_power)

static int cmd_attach_forget(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long id = strtoul(argv[1], NULL, 10);
	int ret;

	ARG_UNUSED(argc);
	ret = meshtastic_attachment_forget((uint8_t)id);
	if (ret < 0) {
		shell_error(sh, "forget failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "attachment %lu forgotten", id);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_forget)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_attach_cmds,
	SHELL_CMD(list, NULL, SHELL_HELP("Every radio this brain hears through.", NULL),
		  cmd_attach_list_on_work),
	SHELL_CMD_ARG(status, NULL, SHELL_HELP("One attachment in detail.", "<id>"),
		      cmd_attach_status_on_work, 2, 0),
	SHELL_CMD_ARG(power, NULL, SHELL_HELP("Set a head's transmit power.", "<id> <dBm>"),
		      cmd_attach_power_on_work, 3, 0),
	SHELL_CMD_ARG(preset, NULL,
		      SHELL_HELP("Put a head on a preset and keep it there (stored; re-sent "
				 "when the head comes back on another).",
				 "<id> <name|number|none>"),
		      cmd_attach_preset_on_work, 3, 0),
	SHELL_CMD_ARG(allow, NULL,
		      SHELL_HELP("Heads admitted on an untrusted link: show, add, clear.",
				 "[<hex node id>|clear]"),
		      cmd_attach_allow_on_work, 1, 1),
	SHELL_CMD_ARG(forget, NULL, SHELL_HELP("Drop a head from the table.", "<id>"),
		      cmd_attach_forget_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ATTACHMENT_BRAIN */

#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
/* `meshtastic attach` on a keyless head (ATTACHMENT-DESIGN S5): whose radio
 * this is, and what has passed through. */
static int cmd_attach_head_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_attachment_head_stats st;
	uint32_t brain = meshtastic_attachment_head_get_brain();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_attachment_head_stats_get(&st);
	shell_print(sh, "role: keyless radio head (transmits what its brain hands it)");
	shell_print(sh, "brain: 0x%08x%s", brain, brain == 0U ? " (none: not forwarding)" : "");
	shell_print(sh, "preset: %s (%u)", meshtastic_preset_display_name(mt.modem_preset, true),
		    (unsigned int)mt.modem_preset);
	shell_print(sh, "heard %u  forwarded %u  no_brain %u  queue_full %u  send_failed %u",
		    st.heard, st.forwarded, st.no_brain, st.queue_full, st.send_failed);
	shell_print(sh, "status_sent %u  controls %u  refused %u  untrusted %u  rejected %u",
		    st.status_sent, st.controls, st.refused, st.untrusted, st.rejected);
	shell_print(sh, "tx: sent %u  failed %u  deferred %u  queue_full %u  cancelled %u  late %u  late_dropped %u  duty_blocked %u",
		    st.tx_sent, st.tx_failed, st.tx_deferred, st.tx_queue_full, st.tx_cancelled,
		    st.tx_late, st.tx_late_dropped, st.tx_duty_blocked);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_show)

static int cmd_attach_head_brain(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node = (uint32_t)strtoul(argv[1], NULL, 16);

	ARG_UNUSED(argc);
	meshtastic_attachment_head_set_brain(node);
	shell_print(sh, "brain: 0x%08x%s", node, node == 0U ? " (forwarding stopped)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_brain)

static int shell_head_status_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_attachment_head_status_send();
	if (ret == 0) {
		shell_print(sh, "status sent");
	}
	return ret;
}

static int cmd_attach_head_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_head_status_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_attach_head_status)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_attach_head_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Brain and counters.", NULL), cmd_attach_head_show_on_work),
	SHELL_CMD_ARG(brain, NULL,
		      SHELL_HELP("The brain's node number (0 stops forwarding).", "<hex node id>"),
		      cmd_attach_head_brain_on_work, 2, 0),
	SHELL_CMD(status, NULL, SHELL_HELP("Send a STATUS to the brain now.", NULL),
		  cmd_attach_head_status_on_work),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ATTACHMENT_HEAD */

#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
/* `meshtastic neighbors` (agents-dnr4.19). Reads always; writes gated behind
 * CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and the is_managed policy. */
static int cmd_neighbors_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_neighborinfo_settings s;
	struct meshtastic_neighborinfo_entry e;
	size_t shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_neighborinfo_settings(&s);
	shell_print(sh, "neighbor info: %s, every %u s, lora %s%s", s.enabled ? "enabled" : "disabled",
		    s.interval_secs, s.transmit_over_lora ? "on" : "off",
		    (s.transmit_over_lora && !s.lora_allowed)
			    ? " (held: default channel on default slot -> phone only)"
			    : "");

	for (size_t i = 0; i < CONFIG_MESHTASTIC_NEIGHBORINFO_TABLE_SIZE; i++) {
		if (!meshtastic_neighborinfo_at(i, &e)) {
			continue;
		}
		{
			/* Tenths of a dB without %f: the shell builds do not all
			 * carry CBPRINTF_FP_SUPPORT. */
			int snr10 = (int)(e.snr * 10.0f);
			int frac = snr10 % 10;

			shell_print(sh, "  0x%08x  snr %s%d.%d dB  heard %6u s ago  (its interval %u s)",
				    e.node, (snr10 < 0 && snr10 > -10) ? "-" : "", snr10 / 10,
				    frac < 0 ? -frac : frac, e.age_secs, e.interval_secs);
		}
		shown++;
	}
	shell_print(sh, "%zu direct neighbor%s", shown, shown == 1U ? "" : "s");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_show)

static int cmd_neighbors_write(const struct shell *sh, bool enabled, uint32_t interval_secs,
			       bool transmit_over_lora)
{
#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	ARG_UNUSED(enabled);
	ARG_UNUSED(interval_secs);
	ARG_UNUSED(transmit_over_lora);
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	ret = meshtastic_neighborinfo_set(enabled, interval_secs, transmit_over_lora);
	if (ret < 0) {
		shell_error(sh, "neighbor info set failed: %d", ret);
		return ret;
	}
	shell_print(sh, "persisted; applied live, no reboot");
	return cmd_neighbors_show(sh, 1U, NULL);
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}

/* Read the stored section (not the resolved settings) so an edit of one field
 * does not silently rewrite the others to their resolved values. */
static void neighbors_stored(bool *enabled, uint32_t *interval, bool *lora)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_neighbor_info_tag, &mod);
	*enabled = mod.payload_variant.neighbor_info.enabled;
	*interval = mod.payload_variant.neighbor_info.update_interval;
	*lora = mod.payload_variant.neighbor_info.transmit_over_lora;
}

static int cmd_neighbors_enable(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;

	ARG_UNUSED(argc);
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, strcmp(argv[0], "enable") == 0, interval, lora);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_enable)

static int cmd_neighbors_interval(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;
	unsigned long secs;
	char *end;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic neighbors interval <secs> (0 = default)");
		return -EINVAL;
	}
	secs = strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs > UINT32_MAX) {
		shell_error(sh, "invalid seconds value: %s", argv[1]);
		return -EINVAL;
	}
	if (secs != 0UL && secs < CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC) {
		shell_warn(sh, "%lu s is below the %d s minimum; the default (%d s) will be used",
			   secs, CONFIG_MESHTASTIC_NEIGHBORINFO_MIN_INTERVAL_SEC,
			   CONFIG_MESHTASTIC_NEIGHBORINFO_INTERVAL_SEC);
	}
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, enabled, (uint32_t)secs, lora);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_interval)

static int cmd_neighbors_lora(const struct shell *sh, size_t argc, char **argv)
{
	bool enabled, lora;
	uint32_t interval;

	if (argc != 2U || (strcmp(argv[1], "on") != 0 && strcmp(argv[1], "off") != 0)) {
		shell_error(sh, "usage: meshtastic neighbors lora on|off");
		return -EINVAL;
	}
	neighbors_stored(&enabled, &interval, &lora);
	return cmd_neighbors_write(sh, enabled, interval, strcmp(argv[1], "on") == 0);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_lora)

/* Bench 2026-10-10: run inline, `neighbors send` rebooted kit1 (agents-yhdm). */
static int shell_neighbors_send_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	ret = meshtastic_neighborinfo_send();
	if (ret == -ENODATA) {
		shell_error(sh,
			    "no direct neighbors heard yet; nothing to send");
	} else if (ret == 0) {
		shell_print(sh, "neighbor info sent");
	}
	return ret;
}

static int cmd_neighbors_send(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* On the work queue: the send path does not fit the shell thread's XIAO stack. */
	return mtsh_enqueue(sh, shell_neighbors_send_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_neighbors_send)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_neighbors_cmds,
			       SHELL_CMD(enable, NULL, SHELL_HELP("Enable the periodic broadcast.", NULL),
					 cmd_neighbors_enable_on_work),
			       SHELL_CMD(disable, NULL, SHELL_HELP("Disable it.", NULL),
					 cmd_neighbors_enable_on_work),
			       SHELL_CMD_ARG(interval, NULL,
					     SHELL_HELP("Set the broadcast interval (0 = default).",
							"<secs>"),
					     cmd_neighbors_interval_on_work, 2, 0),
			       SHELL_CMD_ARG(lora, NULL,
					     SHELL_HELP("Transmit over LoRa, or phone-only.",
							"on|off"),
					     cmd_neighbors_lora_on_work, 2, 0),
			       SHELL_CMD(send, NULL, SHELL_HELP("Broadcast the table now.", NULL),
					 cmd_neighbors_send_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_NEIGHBORINFO */

#if defined(CONFIG_MESHTASTIC_MESHBEACON)
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
#endif /* CONFIG_MESHTASTIC_MESHBEACON */

#if defined(CONFIG_MESHTASTIC_TRAFFIC)
/* `meshtastic traffic` (agents-dnr4.20). Reads always; writes gated. */
static int cmd_traffic_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_traffic_settings s;
	meshtastic_TrafficManagementStats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_traffic_settings(&s);
	meshtastic_traffic_stats(&st);
	shell_print(sh, "traffic: position dedup %s (%u s), rate limit %s (%u per %u s), "
			"unknown filter %s (>%u per 5 min)",
		    s.position_min_interval_secs ? "on" : "off", s.position_min_interval_secs,
		    (s.rate_limit_window_secs && s.rate_limit_max_packets) ? "on" : "off",
		    s.rate_limit_max_packets, s.rate_limit_window_secs,
		    s.unknown_packet_threshold ? "on" : "off", s.unknown_packet_threshold);
	shell_print(sh, "  inspected %u; dropped: position-dedup %u, rate-limit %u, unknown %u; "
			"%zu node(s) tracked",
		    st.packets_inspected, st.position_dedup_drops, st.rate_limit_drops,
		    st.unknown_packet_drops, meshtastic_traffic_tracked());
	shell_print(sh, "  nodeinfo direct response: not supported on this port (a config "
			"enabling it is refused)");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_traffic_show)

static void traffic_stored(meshtastic_ModuleConfig_TrafficManagementConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_traffic_management_tag,
						 &mod);
	*cfg = mod.payload_variant.traffic_management;
}

/* traffic set <dedup|window|max|unknown> <n> */
static int cmd_traffic_set(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_ModuleConfig_TrafficManagementConfig cfg;
	unsigned long n;
	char *end;

	if (argc != 3U) {
		shell_error(sh, "usage: meshtastic traffic set <dedup|window|max|unknown> <n>");
		return -EINVAL;
	}
	n = strtoul(argv[2], &end, 10);
	if (*end != '\0' || n > UINT32_MAX) {
		shell_error(sh, "invalid value: %s", argv[2]);
		return -EINVAL;
	}
	traffic_stored(&cfg);
	if (strcmp(argv[1], "dedup") == 0) {
		cfg.position_min_interval_secs = (uint32_t)n;
	} else if (strcmp(argv[1], "window") == 0) {
		cfg.rate_limit_window_secs = (uint32_t)n;
	} else if (strcmp(argv[1], "max") == 0) {
		cfg.rate_limit_max_packets = (uint32_t)n;
	} else if (strcmp(argv[1], "unknown") == 0) {
		cfg.unknown_packet_threshold = (uint32_t)n;
	} else {
		shell_error(sh, "unknown key: %s", argv[1]);
		return -EINVAL;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "refused: shell config writes are compiled out "
			"(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)");
	return -ENOTSUP;
#else
	{
		int ret;

		if (mtsh_config_write_refused(sh)) {
			return -EACCES;
		}
		ret = meshtastic_traffic_set(&cfg);
		if (ret < 0) {
			shell_error(sh, "traffic set failed: %d", ret);
			return ret;
		}
		shell_print(sh, "persisted; applied live, no reboot");
		return cmd_traffic_show(sh, 1U, NULL);
	}
#endif
}
MTSH_DEFINE_ON_WORK(cmd_traffic_set)

static int cmd_traffic_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_traffic_reset();
	shell_print(sh, "traffic state and counters cleared");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_traffic_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_traffic_cmds,
			       SHELL_CMD_ARG(set, NULL,
					     SHELL_HELP("Set one knob (0 disables it).",
							"<dedup|window|max|unknown> <n>"),
					     cmd_traffic_set_on_work, 3, 0),
			       SHELL_CMD(reset, NULL,
					 SHELL_HELP("Forget every node's state; zero counters.",
						    NULL),
					 cmd_traffic_reset_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_TRAFFIC */

#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
/* `meshtastic keyverify` (agents-dnr4.13): the bench's stand-in for the app's
 * key-verification prompts. */
static const char *kv_state_name(enum meshtastic_keyverify_state s)
{
	switch (s) {
	case MESHTASTIC_KEYVERIFY_IDLE: return "idle";
	case MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED: return "sent request, awaiting reply";
	case MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER: return "awaiting the peer's security number";
	case MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER: return "awaiting your accept (compare codes)";
	case MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER: return "awaiting your accept (compare codes)";
	case MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1: return "showing security number, awaiting the peer";
	default: return "?";
	}
}

#if defined(CONFIG_MESHTASTIC_XEDDSA)
/* `meshtastic xeddsa` -- the only way to see signature verification working on a bench node:
 * the verify path's success line is LOG_DBG and the bench images compile logging at inf. */
static int cmd_xeddsa_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_xeddsa_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_xeddsa_get_stats(&st);
	shell_print(sh, "xeddsa: verify %s, sign %s",
		    IS_ENABLED(CONFIG_MESHTASTIC_XEDDSA) ? "on" : "off",
		    IS_ENABLED(CONFIG_MESHTASTIC_XEDDSA_SIGN) ? "on" : "off");
	shell_print(sh, "  rx verified   : %u  (of which first-contact NodeInfo: %u)",
		    st.verified, st.bootstrapped);
	shell_print(sh, "  rx DROPPED    : %u bad signature, %u malformed", st.failed,
		    st.malformed);
	shell_print(sh, "  rx unverifiable: %u (signed, no key for the sender)", st.no_key);
	shell_print(sh, "  rx unsigned   : %u accepted, %u dropped (strict), %u dropped (balanced)",
		    st.unsigned_ok, st.unsigned_drop, st.balanced_drop);
	shell_print(sh, "  tx            : %u signed, %u unsigned (would not fit)", st.signed_tx,
		    st.sign_skipped);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_xeddsa_show)

static int cmd_xeddsa_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_xeddsa_reset_stats();
	shell_print(sh, "xeddsa counters cleared");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_xeddsa_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_xeddsa_cmds,
			       SHELL_CMD(reset, NULL, SHELL_HELP("Zero the counters.", NULL),
					 cmd_xeddsa_reset_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_XEDDSA */

static int cmd_keyverify_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_keyverify_status st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_keyverify_status(&st);
	shell_print(sh, "keyverify: %s", kv_state_name(st.state));
	if (st.state != MESHTASTIC_KEYVERIFY_IDLE) {
		shell_print(sh, "  peer 0x%08x nonce %llu", st.remote_node, (unsigned long long)st.nonce);
	}
	if (st.state == MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1) {
		shell_print(sh, "  SECURITY NUMBER to read out to the peer: %03u %03u",
			    st.security_number / 1000U, st.security_number % 1000U);
	}
	if (st.code[0] != '\0') {
		shell_print(sh, "  verification code: %s (must match the peer's)", st.code);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_show)

/* PSA RNG, protobuf encode, wire build with the PKC encrypt: the shell thread's own
 * stack cannot carry it. On the XIAO (CONFIG_SHELL_STACK_SIZE=2048) `keyverify start`
 * locked the CPU up -- reset cause 0x100, no fatal record, because the overflow lands
 * in the MPU guard and the exception entry cannot stack. Bench 2026-09-09. */
static int shell_keyverify_start_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret = meshtastic_keyverify_start(mtsh_work_dest(arg, len));

	if (ret == -EBUSY) {
		shell_error(sh, "a session is already open (keyverify reject "
			    "to drop it)");
	} else if (ret == 0) {
		shell_print(sh, "request sent; ask the peer for the security "
			    "number it shows, then `meshtastic keyverify number <n>`");
	}
	return ret;
}

static int shell_keyverify_number_run(const struct shell *sh, const void *arg, size_t len)
{
	struct meshtastic_keyverify_status st;
	int ret;

	meshtastic_keyverify_status(&st);
	ret = meshtastic_keyverify_provide_number(st.nonce, mtsh_work_dest(arg, len));
	if (ret == -EACCES) {
		shell_error(sh, "that number does not match the peer's reply: "
			    "retype it, or suspect a man in the middle");
	} else if (ret < 0) {
		shell_error(sh, "not expecting a number now (%d)", ret);
	} else {
		meshtastic_keyverify_status(&st);
		shell_print(sh, "verification code: %s -- if the peer shows "
			    "the same, `meshtastic keyverify accept`", st.code);
	}
	return ret;
}

/* Ends with a NodeInfo send: the same path as `start`. */
static int shell_keyverify_accept_run(const struct shell *sh, const void *arg, size_t len)
{
	struct meshtastic_keyverify_status st;
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	meshtastic_keyverify_status(&st);
	ret = meshtastic_keyverify_accept(st.nonce);
	if (ret < 0) {
		shell_error(sh, "nothing to accept (%d)", ret);
	} else {
		shell_print(sh, "peer 0x%08x marked key-verified",
			    st.remote_node);
	}
	return ret;
}

static int cmd_keyverify_start(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dest;
	unsigned long node;
	char *end;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic keyverify start <node-hex>");
		return -EINVAL;
	}
	node = strtoul(argv[1], &end, 16);
	if (*end != '\0' || node == 0UL || node > UINT32_MAX) {
		shell_error(sh, "invalid node id: %s", argv[1]);
		return -EINVAL;
	}
	/* Deferred to the shell work thread: the send path does not fit the shell
	 * thread's stack (see shell_keyverify_start_run). */
	dest = (uint32_t)node;
	ret = mtsh_enqueue(sh, shell_keyverify_start_run, &dest, sizeof(dest));
	return ret;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_start)

static int cmd_keyverify_number(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t number;
	unsigned long n;
	char *end;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic keyverify number <6 digits>");
		return -EINVAL;
	}
	n = strtoul(argv[1], &end, 10);
	if (*end != '\0' || n == 0UL || n > 999999UL) {
		shell_error(sh, "invalid security number: %s", argv[1]);
		return -EINVAL;
	}
	/* Deferred like `start`: this sends M3 and hashes on the way. */
	number = (uint32_t)n;
	return mtsh_enqueue(sh, shell_keyverify_number_run, &number, sizeof(number));
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_number)

static int cmd_keyverify_accept(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Deferred: accept ends by sending our NodeInfo to the peer (the send
	 * path again). Bench 2026-09-09: inline, it locked kit2 up like `start`. */
	return mtsh_enqueue(sh, shell_keyverify_accept_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_accept)

static int cmd_keyverify_reject(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_keyverify_reject();
	shell_print(sh, "session dropped; nothing learned was kept");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_reject)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_keyverify_cmds,
			       SHELL_CMD_ARG(start, NULL,
					     SHELL_HELP("Start verifying a peer's key.",
							"<node-hex>"),
					     cmd_keyverify_start_on_work, 2, 0),
			       SHELL_CMD_ARG(number, NULL,
					     SHELL_HELP("Enter the security number the peer shows.",
							"<6 digits>"),
					     cmd_keyverify_number_on_work, 2, 0),
			       SHELL_CMD(accept, NULL,
					 SHELL_HELP("The codes match: commit the key as verified.",
						    NULL),
					 cmd_keyverify_accept_on_work),
			       SHELL_CMD(reject, NULL, SHELL_HELP("Drop the session.", NULL),
					 cmd_keyverify_reject_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_KEYVERIFY */

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
MTSH_DEFINE_ON_WORK(cmd_sched_show)

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
MTSH_DEFINE_ON_WORK(cmd_sched_policy)

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
MTSH_DEFINE_ON_WORK(cmd_sched_set)

static int cmd_sched_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_sched_stats st;
	struct meshtastic_status status;

	if (argc >= 2U && strcmp(argv[1], "reset") == 0) {
		meshtastic_sched_stats_reset();
		meshtastic_radio_cad_agc_stats_reset();
		meshtastic_radio_rx_activity_stats_reset();
		meshtastic_radio_rx_queue_peak_reset();
		meshtastic_radio_tx_defer_stats_reset();
#if defined(CONFIG_MESHTASTIC_PKI)
		meshtastic_pki_stats_reset();
#endif
		shell_print(sh, "sched stats reset");
		return 0;
	}

	meshtastic_sched_stats_get(&st);
	(void)meshtastic_get_status(&status);

	shell_print(sh, "RX   pkts %u  drop(queue) %u (injected %u)  peak %u/%u  dup %u  decode-fail %u",
		    status.rx_packets, status.rx_dropped, status.rx_dropped_injected,
		    status.rx_queue_peak, (unsigned int)CONFIG_MESHTASTIC_RX_QUEUE_DEPTH,
		    status.duplicate_packets, status.decode_failures);
	shell_print(sh, "TX   pkts %u  failed %u  relayed %u", status.tx_packets,
		    status.tx_failures, status.relayed_packets);
#if defined(CONFIG_MESHTASTIC_PKI)
	{
		struct meshtastic_pki_stats p;

		meshtastic_pki_stats_get(&p);
		shell_print(sh, "PKI  x25519 %u (avg %u us, max %u us)  cache hits %u of %u slots  "
				"failed %u",
			    p.derived, p.us_avg, p.us_max, p.hits, p.cache_size, p.failed);
	}
#endif
	shell_print(sh, "TX egress by tier (enqueued / dropped):");
	for (uint8_t t = 0; t < MT_SCHED_TIER_COUNT; t++) {
		shell_print(sh, "  %-6s %u / %u", meshtastic_sched_tier_name(t), st.tx_enq[t],
			    st.tx_drop[t]);
	}
	shell_print(sh, "outbound hi-water: %u / %u", st.ob_hiwater,
		    CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX);
	shell_print(sh, "phone FromRadio drops with no client reading: %u", st.phone_drop_idle);
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
MTSH_DEFINE_ON_WORK(cmd_sched_stats)

static int cmd_sched_defaults(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_sched_defaults();
	shell_print(sh, "sched reverted to compiled defaults");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_sched_defaults)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_sched_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Show scheduler policy and knobs.", NULL), cmd_sched_show_on_work),
	SHELL_CMD(policy, NULL, SHELL_HELP("Apply a named policy preset.", "<name>"),
		  cmd_sched_policy_on_work),
	SHELL_CMD(set, NULL, SHELL_HELP("Set one knob.", "<key> <value>"), cmd_sched_set_on_work),
	SHELL_CMD(stats, NULL, SHELL_HELP("Show or reset live counters.", "[reset]"),
		  cmd_sched_stats_on_work),
	SHELL_CMD(defaults, NULL, SHELL_HELP("Revert to compiled defaults.", NULL),
		  cmd_sched_defaults_on_work),
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

	if (mtsh_config_write_refused(sh)) {
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
MTSH_DEFINE_ON_WORK(cmd_power)

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
MTSH_DEFINE_ON_WORK(cmd_pm)
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
			if (meshtastic_radio_fem_state() == MESHTASTIC_FEM_STATE_FAILED) {
				/* "No front end" would be a lie here in the other
				 * direction: this board HAS one, we just could not
				 * identify it, so no gain table is in use and the drive
				 * is the request minus nothing. Saying "no front end"
				 * would send someone looking at the antenna instead of
				 * the FEM rail. */
				shell_print(sh, "tx power:     %d dBm at the antenna "
						"(requested %d%s, clamped to the radio's %d "
						"dBm max; FRONT-END DETECTION FAILED, so no "
						"gain is being compensated — see "
						"`meshtastic rf`)",
					    drive, radiated, dflt, drive);
			} else {
				shell_print(sh, "tx power:     %d dBm at the antenna "
						"(requested %d%s, clamped to the radio's %d "
						"dBm max; no front end to make up the "
						"difference)",
					    drive, radiated, dflt, drive);
			}
		}
	}

	/* The other two knobs a config tool can set that change what the radio
	 * does. Shown because "stored but not doing anything" is exactly the
	 * failure these two used to have. */
	/* The stored flag is the build's seed until the section is written; what the
	 * node does is mt.tx_enabled, which the first-boot hold keeps off meanwhile
	 * (agents-xhli.21). Say which, not "yes" over a silent radio. */
	if (cfg.payload_variant.lora.tx_enabled && !mt.tx_enabled) {
		shell_print(sh, "tx enabled:   NO — held until the LoRa config is written "
				"(stored: yes; `meshtastic lora tx on` releases it)");
	} else {
		shell_print(sh, "tx enabled:   %s",
			    cfg.payload_variant.lora.tx_enabled ? "yes" : "NO — receive only");
	}
	switch (meshtastic_radio_chip_present()) {
	case MESHTASTIC_RADIO_TRI_ON:
		shell_print(sh, "radio chip:   answered at boot");
		break;
	case MESHTASTIC_RADIO_TRI_OFF:
		shell_print(sh, "radio chip:   NOT ANSWERING since boot (absent, unseated or "
				"dead): this node can neither send nor hear");
		break;
	default:
		shell_print(sh, "radio chip:   unknown (the driver cannot say)");
		break;
	}
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
static int cmd_lora_preset_set(const struct shell *sh, const char *name)
{
	meshtastic_Config cfg;
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	if (mtsh_parse_modem_preset(name, &preset) < 0) {
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
	/* set_config schedules the coalesced flash save; that's independent of the
	 * live radio retune meshtastic_config_store_set_config() already did
	 * above (agents-k8oe) -- no reboot needed either way. */

	shell_print(sh, "modem preset -> %s (persisted and applied live)",
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

	if (mtsh_config_write_refused(sh)) {
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

	if (mtsh_parse_modem_preset(argv[2], &target) < 0) {
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
MTSH_DEFINE_ON_WORK(cmd_preset)

#if defined(CONFIG_MESHTASTIC_BOOTLOG)
/* `meshtastic resets` — the retrieval half of the boot history.
 *
 * The boot-time report goes to whatever log backend happens to be live at the
 * time, which on this fleet is often nobody: there is no netlog, and a console
 * only exists while something is attached. A reboot at 03:50 is therefore
 * unobserved by construction. This is how you ask afterwards.
 */
static void resets_shell_line(void *ctx, const char *line)
{
	shell_print((const struct shell *)ctx, "%s", line);
}

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
		shell_print(sh, "cause       " MESHTASTIC_BOOTLOG_NO_CAUSE);
	}
	shell_print(sh, "uptime      %lld s", (long long)k_uptime_seconds());

#if defined(CONFIG_MESHTASTIC_REBOOT_TRACE)
	{
		struct meshtastic_reboot_trace rt;

		/* The half hwinfo cannot answer: WHO asked. A reset cause of "SOFTWARE"
		 * says a sys_reboot() happened and nothing about which one, which is
		 * exactly the gap that left two nodes' restarts unexplained (agents-u0h6). */
		if (meshtastic_reboot_trace_get(&rt)) {
			shell_print(sh, "asked by    %s%s%s%s, from thread \"%s\" at %u s uptime "
					"(%s reboot)",
				    meshtastic_reboot_reason_str(rt.reason),
				    rt.detail[0] != '\0' ? " (" : "",
				    rt.detail[0] != '\0' ? rt.detail : "",
				    rt.detail[0] != '\0' ? ")" : "",
				    rt.thread[0] != '\0' ? rt.thread : "?", rt.uptime_s,
				    meshtastic_reboot_type_str(rt.sys_type));
			if (rt.have_pc) {
				/* Resolve caller_pc MINUS ONE. sys_reboot() is noreturn, so the
				 * call is usually the last instruction in its caller and the
				 * return address lands on the first byte of the NEXT function --
				 * addr2line then names the wrong one. Measured: a shell reboot
				 * recorded 0x420097f8, which addr2line calls z_thread_is_valid
				 * and 0x420097f7 correctly calls cmd_kernel_reboot_warm. */
				shell_print(sh, "            caller 0x%08x  — addr2line 0x%08x "
						"(PC-1: the call is noreturn, so the return address "
						"is the next function) against that build's zephyr.elf",
					    rt.caller_pc, rt.caller_pc - 1U);
			}
			if (rt.reason == MESHTASTIC_REBOOT_UNKNOWN) {
				/* Say plainly that nothing claimed it. An unnamed reboot is a
				 * finding, not a blank field. */
				shell_print(sh, "            NOTHING claimed this reboot — it did "
						"not come through a known path");
			}
		} else {
			shell_print(sh, "asked by    no record (power loss, a reset that cleared "
					"retained RAM, or a build without reboot tracing)");
		}
	}
#endif

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

	/* The faults the rows above cannot explain: a WATCHDOG row with no
	 * crashinfo and no coredump is exactly what a double exception or an
	 * interrupt storm leaves behind, and these are their only trace. */
	shell_print(sh, "");
	meshtastic_bootlog_fault_lines(resets_shell_line, (void *)sh);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_resets)
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

	{
		/* Pending, or already taken by this boot's report (which consumes it so the
		 * next boot does not report it again): either way, the last fatal error. */
		bool pending = meshtastic_fatal_peek_last_crash(&fatal_crash);

		if (pending || meshtastic_fatal_reported_at_boot(&fatal_crash)) {
			shell_print(sh, "fatal        %s (%u), heap free=%u allocated=%u "
					"max_allocated=%u, faulting thread \"%s\"%s",
				    crashinfo_fatal_reason_name(fatal_crash.reason),
				    fatal_crash.reason, fatal_crash.heap_free,
				    fatal_crash.heap_allocated, fatal_crash.heap_max_allocated,
				    fatal_crash.thread_name,
				    pending ? "" : " (reported at this boot)");
			shell_print(sh, "             pc=0x%08x lr=0x%08x xpsr=0x%08x%s, %u nested",
				    fatal_crash.pc, fatal_crash.lr, fatal_crash.xpsr,
				    fatal_crash.in_handler ? " (raised in handler mode)" : "",
				    fatal_crash.nested);
		} else {
			shell_print(sh, "fatal        none pending");
		}
	}

	shell_print(sh, "");
	shell_print(sh, "Non-destructive: does not clear what `meshtastic resets` reports "
			"at boot.");

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_crashinfo)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/* `meshtastic lora power <dBm>`: this node's own transmit power, at the
 * antenna, from its console. Until 2026-10-01 only the phone API could set
 * it -- a brain could set a head's (`attach power`) but nothing could set its
 * own without a BLE config write, which is how the bench's brains sat at
 * 14 dBm while every head was at 2. Same pattern as the head's SET_POLICY:
 * read the LoRa section, change the field, write it back -- applied now,
 * persisted, region-clamped by the store. 0 = the region's maximum. */
static int cmd_lora_power_set(const struct shell *sh, const char *arg)
{
	meshtastic_Config cfg;
	char *end;
	long dbm = strtol(arg, &end, 10);
	int ret;

	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	if (end == arg || *end != '\0' || dbm < 0 || dbm > 30) {
		shell_error(sh, "usage: meshtastic lora power <0..30 dBm>  (0 = region maximum)");
		return -EINVAL;
	}
	ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
	if (ret < 0) {
		shell_error(sh, "lora get failed: %d", ret);
		return ret;
	}
	cfg.which_payload_variant = meshtastic_Config_lora_tag;
	cfg.payload_variant.lora.tx_power = (int32_t)dbm;
	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "lora power set failed: %d", ret);
		return ret;
	}
	cmd_lora_show(sh);
	return 0;
}
#endif

static int cmd_lora(const struct shell *sh, size_t argc, char **argv)
{
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc == 1U) {
		cmd_lora_show(sh);
		return 0;
	}
	if (argc != 3U ||
	    (strcmp(argv[1], "preset") != 0 && strcmp(argv[1], "tx") != 0 &&
	     strcmp(argv[1], "power") != 0)) {
		shell_error(sh, "usage: meshtastic lora [preset <name>] [tx on|off] [power <dBm>]");
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
	if (strcmp(argv[1], "power") == 0) {
		return cmd_lora_power_set(sh, argv[2]);
	}
	return cmd_lora_preset_set(sh, argv[2]);
#endif
}
MTSH_DEFINE_ON_WORK(cmd_lora)

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
MTSH_DEFINE_ON_WORK(cmd_netlog)
#endif

#if defined(CONFIG_MESHTASTIC_SETTINGS)
/* The admin backup/restore, from the console (agents-dnr4.14). A restore
 * leaves the restored config applied and a save queued; reboot to be sure
 * every consumer re-reads it, as the admin path does on its own. */

static int cmd_backup(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_backup_meta meta;
	int ret;

	if (argc < 2) {
		if (meshtastic_backup_exists(&meta)) {
			shell_print(sh, "backup present (version %u, taken at epoch %u)",
				    meta.version, meta.timestamp);
		} else {
			shell_print(sh, "no backup");
		}
		return 0;
	}
	if (strcmp(argv[1], "save") == 0) {
		ret = meshtastic_backup_save();
	} else if (strcmp(argv[1], "restore") == 0) {
		ret = meshtastic_backup_restore();
		if (ret == 0) {
			shell_warn(sh, "restored and applied; reboot to finish");
		}
	} else if (strcmp(argv[1], "remove") == 0) {
		ret = meshtastic_backup_remove();
	} else {
		shell_error(sh, "usage: backup [save|restore|remove]");
		return -EINVAL;
	}
	if (ret < 0) {
		shell_error(sh, "%s failed (%d)", argv[1], ret);
	} else {
		shell_print(sh, "%s: ok", argv[1]);
	}
	return ret;
}
MTSH_DEFINE_ON_WORK(cmd_backup)
#endif /* CONFIG_MESHTASTIC_SETTINGS */

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

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
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
	if (mtsh_config_write_refused(sh)) {
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
		ret = mtsh_parse_u32(sh, argv[1], &node_num);
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
MTSH_DEFINE_ON_WORK(cmd_admin_trust)

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
/* meshtastic admin remote <node> get owner
 *                        <node> get config <type 0-8>
 *                        <node> set-owner <long> [short]
 *                        <node> set-time [<unix-epoch-seconds>]  (default: ours)
 * Responses arrive asynchronously on the RX path and print as `admin client:`
 * log lines; a getter also caches the target's session passkey for the
 * mutating op that follows. */
/* `admin remote`: payload[0] is an enum admin_remote_op, the rest its argument. */
struct shell_admin_remote_work {
	uint32_t dest;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
};

BUILD_ASSERT(sizeof(struct shell_admin_remote_work) <= MTSH_WORK_ARG_MAX,
	     "an admin request rides in a shell work item");

/* Run inline it reset kit1 from the shell thread's stack (bench 2026-10-10). */
static int shell_admin_remote_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_admin_remote_work *aw = arg;
	uint32_t epoch;
	int ret;

	ARG_UNUSED(len);
	switch (aw->payload[0]) {
	case ADMIN_REMOTE_GET_OWNER:
		ret = meshtastic_admin_client_get_owner(aw->dest);
		break;
	case ADMIN_REMOTE_GET_CONFIG:
		ret = meshtastic_admin_client_get_config(aw->dest, aw->payload[1]);
		break;
	case ADMIN_REMOTE_SET_TIME:
		memcpy(&epoch, &aw->payload[1], sizeof(epoch));
		ret = meshtastic_admin_client_set_time(aw->dest, epoch);
		break;
	case ADMIN_REMOTE_SET_OWNER: {
		const char *long_name = (const char *)&aw->payload[2];
		const char *short_name =
			aw->payload[1] != 0U ? long_name + strlen(long_name) + 1 : NULL;

		ret = meshtastic_admin_client_set_owner(aw->dest, long_name, short_name);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}
	if (ret == -EACCES) {
		shell_error(sh, "request failed (%d) -- no public key / passkey", ret);
	} else if (ret == 0) {
		shell_print(sh, "request sent -- watch for `admin client:` log lines");
	}
	return ret;
}

static int cmd_admin_remote(const struct shell *sh, size_t argc, char **argv)
{
	struct shell_admin_remote_work *aw;
	struct mtsh_work *w;
	uint32_t dest;
	int ret;

	if (argc < 3U) {
		goto usage;
	}
	ret = mtsh_parse_u32(sh, argv[1], &dest);
	if (ret < 0) {
		return ret;
	}
	/* The request is built in the queued item itself, not on this stack. */
	w = mtsh_work_alloc(sh, shell_admin_remote_run);
	if (w == NULL) {
		return -ENOMSG;
	}
	aw = (struct shell_admin_remote_work *)w->arg;
	aw->dest = dest;

	/* Everything below only parses and checks. The send itself runs on the work queue:
	 * it is the whole send path, which the XIAO shell stack cannot carry. */
	if (strcmp(argv[2], "get") == 0 && argc >= 4U) {
		if (strcmp(argv[3], "owner") == 0) {
			aw->payload[0] = ADMIN_REMOTE_GET_OWNER;
		} else if (strcmp(argv[3], "config") == 0 && argc >= 5U) {
			uint32_t type;

			ret = mtsh_parse_u32(sh, argv[4], &type);
			if (ret < 0) {
				mtsh_work_cancel(w);
				return ret;
			}
			aw->payload[0] = ADMIN_REMOTE_GET_CONFIG;
			aw->payload[1] = (uint8_t)type;
			if (type > UINT8_MAX) {
				mtsh_work_cancel(w);
				goto usage;
			}
		} else {
			mtsh_work_cancel(w);
			goto usage;
		}
	} else if (strcmp(argv[2], "set-time") == 0) {
		/*
		 * Relay a wall clock to a peer (agents-xhli.14). With no explicit
		 * epoch, relay OUR OWN -- which is the whole point: a node that has a
		 * time source hands one to a node that does not.
		 *
		 * Refusing when our own clock is unset is not a nicety. An unset
		 * clock reads epoch 0, which the target's sanity window rejects, so
		 * the request would look sent and do nothing; and a node that cannot
		 * say when anything happened has no business being a time source.
		 */
		uint32_t epoch;

		if (argc >= 4U) {
			ret = mtsh_parse_u32(sh, argv[3], &epoch);
			if (ret < 0) {
				mtsh_work_cancel(w);
				return ret;
			}
		} else {
			epoch = meshtastic_clock_now_epoch();
			if (epoch == 0U) {
				shell_error(sh, "refused: this node's own clock is unset, so "
						"it cannot be a time source (set it first, or "
						"pass an explicit epoch)");
				mtsh_work_cancel(w);
				return -EAGAIN;
			}
		}

		if (!meshtastic_admin_client_have_passkey(aw->dest)) {
			shell_error(sh, "no fresh session passkey for 0x%08x -- run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    aw->dest, aw->dest);
			mtsh_work_cancel(w);
			return -EACCES;
		}
		aw->payload[0] = ADMIN_REMOTE_SET_TIME;
		memcpy(&aw->payload[1], &epoch, sizeof(epoch));
	} else if (strcmp(argv[2], "set-owner") == 0 && argc >= 4U) {
		size_t long_len = strlen(argv[3]);
		size_t short_len = argc >= 5U ? strlen(argv[4]) : 0U;

		if (!meshtastic_admin_client_have_passkey(aw->dest)) {
			shell_error(sh, "no fresh session passkey for 0x%08x -- run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    aw->dest, aw->dest);
			mtsh_work_cancel(w);
			return -EACCES;
		}
		if (2U + long_len + 1U + short_len + 1U > sizeof(aw->payload)) {
			shell_error(sh, "name too long");
			mtsh_work_cancel(w);
			return -EINVAL;
		}
		aw->payload[0] = ADMIN_REMOTE_SET_OWNER;
		aw->payload[1] = argc >= 5U ? 1U : 0U;
		memcpy(&aw->payload[2], argv[3], long_len + 1U);
		if (argc >= 5U) {
			memcpy(&aw->payload[2U + long_len + 1U], argv[4], short_len + 1U);
		}
	} else {
		mtsh_work_cancel(w);
		goto usage;
	}

	return mtsh_work_submit(w, sizeof(*aw));

usage:
	shell_error(sh, "usage: meshtastic admin remote <node> get owner\n"
		    "       meshtastic admin remote <node> get config <type 0-8>\n"
		    "       meshtastic admin remote <node> set-owner <long> [short]\n"
		    "       meshtastic admin remote <node> set-time [<unix-epoch-seconds>]");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_admin_remote)
#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */
#endif /* CONFIG_MESHTASTIC_ADMIN */

/* The cluster and fleet commands are NOT admin features: MESHTASTIC_CLUSTER does
 * not depend on MESHTASTIC_ADMIN, and their registrations below are guarded by
 * CLUSTER/FLEET alone. They used to sit inside the ADMIN block, so SHELL=y +
 * CLUSTER=y + PHONEAPI=n (which forces ADMIN=n) failed to compile -- found by
 * meshtastic.variants.shell_settings (agents-68g8). */

#if defined(CONFIG_MESHTASTIC_ADMIN)
SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_admin_cmds,
			       SHELL_CMD(trust, NULL,
					 SHELL_HELP("List/manage trusted remote-admin keys.",
						    "[<node> [off] | key:<hex-prefix> off]"),
					 cmd_admin_trust_on_work),
#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
			       SHELL_CMD(remote, NULL,
					 SHELL_HELP("Administer a peer node (PKC, any bearer).",
						    "<node> get owner | get config <t> | "
						    "set-owner <long> [short] | "
						    "set-time [<epoch>]"),
					 cmd_admin_remote_on_work),
#endif
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ADMIN */

/* ---- intents: the roles this node remembers ------------------------------------ */

static void intents_print_one(const struct meshtastic_intent *it, bool set, const char *detail,
			      void *arg)
{
	const struct shell *sh = arg;

	if (set) {
		shell_print(sh, "%-8s set   %s  (%s)", it->name, detail, it->keys);
	} else {
		shell_print(sh, "%-8s none  (%s)", it->name, it->keys);
	}
}

static void intents_reboot_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)meshtastic_sparkplug_stop(MESHTASTIC_SPARKPLUG_STOP_TIMEOUT); /* NDEATH, DISCONNECT */
	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_SHELL, "intents clear");
	sys_reboot(SYS_REBOOT_COLD);
}
static K_WORK_DELAYABLE_DEFINE(intents_reboot_work, intents_reboot_fn);

static int cmd_intents(const struct shell *sh, size_t argc, char **argv)
{
	size_t n;

	if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
		int cleared = meshtastic_intents_clear();

		if (cleared < 0) {
			shell_error(sh, "intents: clear failed (%d); nothing more was touched", cleared);
			return cleared;
		}
		if (cleared == 0) {
			shell_print(sh, "intents: nothing to clear");
			return 0;
		}
		/* The modules stood down in RAM as far as each could; a reboot makes it whole,
		 * the way a config write already reboots the node. Delayed so this answer gets
		 * out first. */
		shell_print(sh, "intents: %d cleared; rebooting", cleared);
		k_work_schedule(&intents_reboot_work, K_MSEC(300));
		return 0;
	}
	if (argc >= 2) {
		shell_error(sh, "usage: meshtastic intents [clear]");
		return -EINVAL;
	}
	meshtastic_intents_each(intents_print_one, (void *)sh);
	n = meshtastic_intents_set();
	shell_print(sh, "intents: %u set", (unsigned int)n);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_intents)

/* The root, `meshtastic`, is a section-backed set (agents-ls39.4.4.1): each command
 * adds itself with SHELL_SUBCMD_ADD((meshtastic), ...), so a command file needs no
 * line in a central table. `meshtastic help` lists them in name order. */
SHELL_SUBCMD_SET_CREATE(meshtastic_cmds, (meshtastic));

SHELL_SUBCMD_ADD((meshtastic), status, NULL,
		 SHELL_HELP("Show Meshtastic status.", NULL),
		 cmd_status_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), version, NULL,
		 SHELL_HELP("Show build id / firmware version.", NULL),
		 cmd_version_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), intents, NULL,
		 SHELL_HELP("The roles this node remembers across reboots (peer target, brain, "
		 "heads, ear, courier arm, relay, pinned presets, cluster document); "
		 "`clear` forgets them all and reboots. Identity, radio, channels and "
		 "owner are untouched.",
		 "[clear]"),
		 cmd_intents_on_work, 1, 1);
SHELL_SUBCMD_ADD((meshtastic), state, NULL,
		 SHELL_HELP("This node's state as one machine-readable line, for tools: "
		 "~S{json}*crc16.", NULL),
		 cmd_state_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
SHELL_SUBCMD_ADD((meshtastic), api, NULL,
		 SHELL_HELP("The phone API as text, for tools: ToRadio in, ~P<FromRadio> lines out.",
		 "[[+] <base64> [<crc16>]]"),
		 meshtastic_shell_api_cmd, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), time, NULL,
		 SHELL_HELP("Show or set the wall clock.", "[set <unix-epoch-seconds>]"),
		 cmd_time_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), owner, NULL,
		 SHELL_HELP("Show or set this node's owner names.",
		 "[set <long> [short]]"),
		 cmd_owner_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_SHELL_CRASHTEST)
SHELL_SUBCMD_ADD((meshtastic), crashtest, NULL,
		 SHELL_HELP("Crash on purpose, to prove the coredump pipeline works "
		 "before a real fault needs it.", "<panic|stack|dblexc>"),
		 cmd_crashtest, 0, 0);
#endif
#if defined(CONFIG_NETWORKING)
SHELL_SUBCMD_ADD((meshtastic), netpause, NULL,
		 SHELL_HELP("Take the network down for N seconds, then restore it. "
		 "Lets PM residency be measured without network traffic waking "
		 "the SoC.", "<secs>"),
		 cmd_net_pause, 0, 0);
#endif
#if defined(CONFIG_PM)
SHELL_SUBCMD_ADD((meshtastic), power, NULL,
		 SHELL_HELP("Show or set the light-sleep power-saving policy.", "[on|off]"),
		 cmd_power_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), pm, NULL,
		 SHELL_HELP("Volatile bench hold: pin the node awake (off) or release (on). "
		 "Not persisted; distinct from `power`.",
		 "[on|off]"),
		 cmd_pm_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
SHELL_SUBCMD_ADD((meshtastic), transport, NULL,
		 SHELL_HELP("Show or set the phone transport, reboot to apply (unified image).",
		 "[ble|wifi]"),
		 cmd_transport_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), sched, &meshtastic_sched_cmds,
		 SHELL_HELP("Scheduler / QoS policy commands.", NULL),
		 cmd_sched_show_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), channel, &meshtastic_channel_cmds,
		 SHELL_HELP("Channel table commands.", NULL),
		 NULL, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), device, &meshtastic_device_cmds,
		 SHELL_HELP("Device role and rebroadcast commands.", NULL),
		 NULL, 0, 0);
#if defined(CONFIG_MESHTASTIC_ADMIN)
SHELL_SUBCMD_ADD((meshtastic), admin, &meshtastic_admin_cmds,
		 SHELL_HELP("Remote-admin trust and client commands.", NULL),
		 NULL, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), lora, NULL,
		 SHELL_HELP("Show or set the LoRa modem preset (reboot to apply), "
		 "the TX-enable switch or the TX power (both apply live).",
		 "[preset <name>] [tx on|off] [power <dBm>]"),
		 cmd_lora_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_BOOTLOG)
SHELL_SUBCMD_ADD((meshtastic), resets, NULL,
		 SHELL_HELP("Why this node last rebooted, and the boot history. "
		 "`durable` reads the FLASH ring, which survives power loss.",
		 "[durable]"),
		 cmd_resets_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), crashinfo, NULL,
		 SHELL_HELP("Durable crash breadcrumbs: watchdog, hw-watchdog and "
		 "fatal-error info from the last abnormal reset.",
		 NULL),
		 cmd_crashinfo_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), preset, NULL,
		 SHELL_HELP("Live preset switching: what would hold a scheduled hop off, "
		 "and the counters for the ones that were.",
		 "[hop <name> | reset]"),
		 cmd_preset_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_MESSAGE)
SHELL_SUBCMD_ADD((meshtastic), text, &meshtastic_text_cmds,
		 SHELL_HELP("Text message commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB)
SHELL_SUBCMD_ADD((meshtastic), nodedb, &meshtastic_nodedb_cmds,
		 SHELL_HELP("NodeDB commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_POSITION)
SHELL_SUBCMD_ADD((meshtastic), position, &meshtastic_position_cmds,
		 SHELL_HELP("Position: gps_mode, broadcast interval.", NULL),
		 cmd_position_root_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_GNSS)
SHELL_SUBCMD_ADD((meshtastic), gnss, &meshtastic_gnss_cmds,
		 SHELL_HELP("GNSS commands.", NULL),
		 NULL, 0, 0);
#endif
/* Not a GNSS command: it used to sit inside that guard, so a GNSS=n build
 * lost it and left cmd_netlog an unused static under -Werror. */
#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
SHELL_SUBCMD_ADD((meshtastic), netlog, NULL,
		 SHELL_HELP("Start the remote syslog backend for this boot.",
		 "netlog on"),
		 cmd_netlog_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_AIRTIME)
SHELL_SUBCMD_ADD((meshtastic), airtime, NULL,
		 SHELL_HELP("Channel utilization per preset, and band-wide TX duty.", NULL),
		 cmd_airtime_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
SHELL_SUBCMD_ADD((meshtastic), duty, NULL,
		 SHELL_HELP("Regulatory duty cycle: ceiling, hour used, refusals.",
		 "[reset]"),
		 cmd_duty_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_PHONELOG)
SHELL_SUBCMD_ADD((meshtastic), phonelog, NULL,
		 SHELL_HELP("Log forwarding to the phone: state + counters, or set the "
		 "severity ceiling.",
		 "[off|err|wrn|inf|dbg|reset]"),
		 cmd_phonelog_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
SHELL_SUBCMD_ADD((meshtastic), metrics, &meshtastic_metrics_cmds,
		 SHELL_HELP("Device metrics commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
SHELL_SUBCMD_ADD((meshtastic), environment, &meshtastic_environment_cmds,
		 SHELL_HELP("Environment telemetry commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_NODEINFO)
SHELL_SUBCMD_ADD((meshtastic), nodeinfo, &meshtastic_nodeinfo_cmds,
		 SHELL_HELP("NodeInfo commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
#if defined(CONFIG_MESHTASTIC_XEDDSA)
SHELL_SUBCMD_ADD((meshtastic), xeddsa, &meshtastic_xeddsa_cmds,
		 SHELL_HELP("Packet signature verification: counters, reset.", NULL),
		 cmd_xeddsa_show_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), keyverify, &meshtastic_keyverify_cmds,
		 SHELL_HELP("Manual key verification (security-number handshake): status, "
		 "start, number, accept, reject.",
		 NULL),
		 cmd_keyverify_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_TRAFFIC)
SHELL_SUBCMD_ADD((meshtastic), traffic, &meshtastic_traffic_cmds,
		 SHELL_HELP("Traffic management: position dedup, rate limit, unknown filter; "
		 "counters.",
		 NULL),
		 cmd_traffic_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_MESHBEACON)
SHELL_SUBCMD_ADD((meshtastic), beacon, &meshtastic_beacon_cmds,
		 SHELL_HELP("Mesh beacon (join-my-mesh announcement): show, flag, text, "
		 "interval, send, forget.",
		 NULL),
		 cmd_beacon_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
SHELL_SUBCMD_ADD((meshtastic), neighbors, &meshtastic_neighbors_cmds,
		 SHELL_HELP("Directly-heard neighbors + the NeighborInfo broadcast: show, "
		 "enable/disable, interval, lora, send.",
		 NULL),
		 cmd_neighbors_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_RELAY_EAR)
SHELL_SUBCMD_ADD((meshtastic), ear, &meshtastic_ear_cmds,
		 SHELL_HELP("Cross-preset relay ear: show, peer.", NULL),
		 cmd_ear_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
SHELL_SUBCMD_ADD((meshtastic), attach, &meshtastic_attach_cmds,
		 SHELL_HELP("Radio heads: list, status, preset, forget.", NULL),
		 cmd_attach_list_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
SHELL_SUBCMD_ADD((meshtastic), attach, &meshtastic_attach_head_cmds,
		 SHELL_HELP("This keyless radio head: show, brain, status.", NULL),
		 cmd_attach_head_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_RELAY)
SHELL_SUBCMD_ADD((meshtastic), relay, &meshtastic_relay_cmds,
		 SHELL_HELP("Cross-preset text relay: show, dir, ignore, reset.", NULL),
		 cmd_relay_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
/* `statusmsg`, not `status` (agents-ls39.1.4): `status` is the node's own status,
 * which the bench tools read; a second `status` was never reachable. */
SHELL_SUBCMD_ADD((meshtastic), statusmsg, &meshtastic_status_cmds,
		 SHELL_HELP("Node status message (away-message): show, set, clear, send, "
		 "peers.",
		 NULL),
		 cmd_status_show_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
MTSH_DEFINE_ON_WORK(meshtastic_storage_shell_cmd)
SHELL_SUBCMD_ADD((meshtastic), storage, NULL,
		 SHELL_HELP("Settings store: free space, per-subtree use, boot restore time.",
		 NULL),
		 meshtastic_storage_shell_cmd_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_SETTINGS)
SHELL_SUBCMD_ADD((meshtastic), backup, NULL,
		 SHELL_HELP("Preferences backup in flash: show, save, restore (then reboot), "
		 "remove.", "[save|restore|remove]"),
		 cmd_backup_on_work, 0, 0);
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

SHELL_CMD_REGISTER(meshtastic, &meshtastic_cmds,
		   SHELL_HELP("Meshtastic mesh radio commands.", NULL), NULL);

static int meshtastic_shell_init(void)
{
#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
	meshtastic_shell_api_init();
#endif
	k_thread_create(&shell_work_thread, shell_work_stack,
			K_THREAD_STACK_SIZEOF(shell_work_stack), shell_work_thread_fn, NULL, NULL,
			NULL, CONFIG_MESHTASTIC_SHELL_WORK_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&shell_work_thread, "meshtastic_shell");

	return 0;
}

SYS_INIT(meshtastic_shell_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
