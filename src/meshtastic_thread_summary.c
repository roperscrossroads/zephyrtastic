/* SPDX-License-Identifier: GPL-3.0
 *
 * Condensed periodic per-thread stack/CPU summary. See Kconfig.thread_summary
 * for the "why" -- replaces CONFIG_THREAD_ANALYZER_AUTO's built-in dump (two
 * LOG_INF lines per thread) with the same public thread_analyzer_run() API,
 * a custom callback batching THREADS_PER_LINE threads into one line each.
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(thread_summary, CONFIG_MESHTASTIC_LOG_LEVEL);

#define THREADS_PER_LINE 3

static char line_buf[128];
static size_t line_off;
static unsigned int line_count;

static void flush_line(void)
{
	if (line_count > 0) {
		LOG_INF("%s", line_buf);
	}
	line_off = 0;
	line_count = 0;
	line_buf[0] = '\0';
}

static void summary_cb(struct thread_analyzer_info *info)
{
	size_t pct = info->stack_size ? (info->stack_used * 100U) / info->stack_size : 0U;
	char cpu_str[8];
	int written;

	/* isr_stack() (zephyr/subsys/debug/thread_analyzer/thread_analyzer.c) names
	 * every ISR-stack entry "ISR<core>" and never populates utilization -- it's
	 * the shared interrupt stack, not a schedulable thread, so "percent of CPU
	 * time" doesn't apply the same way. Show that plainly instead of a
	 * misleading "0%" (which reads as "measured, and it's zero"). */
	if (strncmp(info->name, "ISR", 3) == 0) {
		(void)snprintk(cpu_str, sizeof(cpu_str), "--");
	} else {
#ifdef CONFIG_THREAD_RUNTIME_STATS
		(void)snprintk(cpu_str, sizeof(cpu_str), "%u%%", info->utilization);
#else
		(void)snprintk(cpu_str, sizeof(cpu_str), "--");
#endif
	}

	written = snprintk(line_buf + line_off, sizeof(line_buf) - line_off, "%s%s %zu%%/%s",
			    (line_count > 0) ? "  " : "", info->name, pct, cpu_str);

	if (written > 0 && (size_t)written < sizeof(line_buf) - line_off) {
		line_off += (size_t)written;
		line_count++;
	}

	if (line_count >= THREADS_PER_LINE) {
		flush_line();
	}
}

static void thread_summary_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		LOG_INF("Thread summary (stack%%/cpu%%):");
		thread_analyzer_run(summary_cb, 0);
		flush_line();
		k_sleep(K_SECONDS(CONFIG_MESHTASTIC_THREAD_SUMMARY_INTERVAL));
	}
}

K_THREAD_DEFINE(mt_thread_summary, CONFIG_MESHTASTIC_THREAD_SUMMARY_STACK_SIZE,
		 thread_summary_thread_fn, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
		 0);
