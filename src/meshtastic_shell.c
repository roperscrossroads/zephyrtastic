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

#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
static int shell_environment_send_run(const struct shell *sh, const void *arg, size_t len)
{
	uint32_t dest = mtsh_work_dest(arg, len);

	ARG_UNUSED(sh);
	return meshtastic_send_environment(dest, K_FOREVER);
}

static int cmd_environment_send(const struct shell *sh, size_t argc, char **argv)
{
	return mtsh_deferred_send(sh, argc, argv, shell_environment_send_run);
}
MTSH_DEFINE_ON_WORK(cmd_environment_send)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_environment_cmds,
	SHELL_CMD(send, NULL, SHELL_HELP("Send environment telemetry.", "[dest|broadcast]"),
		  cmd_environment_send_on_work),
	SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_ENVIRONMENT_METRICS */

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
SHELL_SUBCMD_ADD((meshtastic), lora, NULL,
		 SHELL_HELP("Show or set the LoRa modem preset (reboot to apply), "
		 "the TX-enable switch or the TX power (both apply live).",
		 "[preset <name>] [tx on|off] [power <dBm>]"),
		 cmd_lora_on_work, 0, 0);
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
/* Not a GNSS command: it used to sit inside that guard, so a GNSS=n build
 * lost it and left cmd_netlog an unused static under -Werror. */
#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
SHELL_SUBCMD_ADD((meshtastic), netlog, NULL,
		 SHELL_HELP("Start the remote syslog backend for this boot.",
		 "netlog on"),
		 cmd_netlog_on_work, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
SHELL_SUBCMD_ADD((meshtastic), environment, &meshtastic_environment_cmds,
		 SHELL_HELP("Environment telemetry commands.", NULL),
		 NULL, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
MTSH_DEFINE_ON_WORK(meshtastic_storage_shell_cmd)
SHELL_SUBCMD_ADD((meshtastic), storage, NULL,
		 SHELL_HELP("Settings store: free space, per-subtree use, boot restore time.",
		 NULL),
		 meshtastic_storage_shell_cmd_on_work, 0, 0);
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
