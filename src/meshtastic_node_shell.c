/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic status`, `version`, `state`, `crashinfo`, `intents` and (with
 * CONFIG_MESHTASTIC_SHELL_CRASHTEST) `crashtest`: the node itself. Built with the shell
 * (src/CMakeLists.txt); moved from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/version.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/diagnostics.h>
#include <zephyr/meshtastic/intents.h>
#include <zephyr/meshtastic/reboot_trace.h>
#include <zephyr/meshtastic/sparkplug.h>

#include "meshtastic_battery.h"
#include "meshtastic_build.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"
#include "meshtastic_shell_state.h"

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

SHELL_SUBCMD_ADD((meshtastic), status, NULL,
		 SHELL_HELP("Show Meshtastic status.", NULL),
		 cmd_status_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), version, NULL,
		 SHELL_HELP("Show build id / firmware version.", NULL),
		 cmd_version_on_work, 0, 0);
SHELL_SUBCMD_ADD((meshtastic), intents, NULL,
		 SHELL_HELP("The roles this node remembers across reboots (peer target, hub, "
		 "workers, rx-only worker, courier arm, relay, pinned presets, cluster document); "
		 "`clear` forgets them all and reboots. Identity, radio, channels and "
		 "owner are untouched.",
		 "[clear]"),
		 cmd_intents_on_work, 1, 1);
SHELL_SUBCMD_ADD((meshtastic), state, NULL,
		 SHELL_HELP("This node's state as one machine-readable line, for tools: "
		 "~S{json}*crc16.", NULL),
		 cmd_state_on_work, 0, 0);
#if defined(CONFIG_MESHTASTIC_SHELL_CRASHTEST)
SHELL_SUBCMD_ADD((meshtastic), crashtest, NULL,
		 SHELL_HELP("Crash on purpose, to prove the coredump pipeline works "
		 "before a real fault needs it.", "<panic|stack|dblexc>"),
		 cmd_crashtest, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), crashinfo, NULL,
		 SHELL_HELP("Durable crash breadcrumbs: watchdog, hw-watchdog and "
		 "fatal-error info from the last abnormal reset.",
		 NULL),
		 cmd_crashinfo_on_work, 0, 0);
