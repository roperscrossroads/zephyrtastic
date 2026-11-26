/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic resets`: why this node last rebooted, and the boot history. Built with the shell and
 * CONFIG_MESHTASTIC_BOOTLOG (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include <zephyr/meshtastic/bootlog.h>
#include <zephyr/meshtastic/reboot_trace.h>

#include "meshtastic_core.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_shell_internal.h"

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

SHELL_SUBCMD_ADD((meshtastic), resets, NULL,
		 SHELL_HELP("Why this node last rebooted, and the boot history. "
		 "`durable` reads the FLASH ring, which survives power loss.",
		 "[durable]"),
		 cmd_resets_on_work, 0, 0);
