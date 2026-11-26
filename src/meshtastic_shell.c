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


/* Shared by the position / metrics / environment / nodeinfo "send" commands. */
int mtsh_deferred_send(const struct shell *sh, size_t argc, char **argv, mtsh_work_fn run)
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

/* Parsers used only by the config-write commands below. */


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

/* Parsers used only by the config-write commands below. */

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

/* Parsers used only by the config-write commands below. */

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

/* The RF commands read the radio, not the NodeDB: they must exist on an image
 * without one (the keyless head is the first). */


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

/* The cluster and fleet commands are NOT admin features: MESHTASTIC_CLUSTER does
 * not depend on MESHTASTIC_ADMIN, and their registrations below are guarded by
 * CLUSTER/FLEET alone. They used to sit inside the ADMIN block, so SHELL=y +
 * CLUSTER=y + PHONEAPI=n (which forces ADMIN=n) failed to compile -- found by
 * meshtastic.variants.shell_settings (agents-68g8). */

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
#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
SHELL_SUBCMD_ADD((meshtastic), transport, NULL,
		 SHELL_HELP("Show or set the phone transport, reboot to apply (unified image).",
		 "[ble|wifi]"),
		 cmd_transport_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), device, &meshtastic_device_cmds,
		 SHELL_HELP("Device role and rebroadcast commands.", NULL),
		 NULL, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), crashinfo, NULL,
		 SHELL_HELP("Durable crash breadcrumbs: watchdog, hw-watchdog and "
		 "fatal-error info from the last abnormal reset.",
		 NULL),
		 cmd_crashinfo_on_work, 0, 0);
/* Not a GNSS command: it used to sit inside that guard, so a GNSS=n build
 * lost it and left cmd_netlog an unused static under -Werror. */
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
MTSH_DEFINE_ON_WORK(meshtastic_storage_shell_cmd)
SHELL_SUBCMD_ADD((meshtastic), storage, NULL,
		 SHELL_HELP("Settings store: free space, per-subtree use, boot restore time.",
		 NULL),
		 meshtastic_storage_shell_cmd_on_work, 0, 0);
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
