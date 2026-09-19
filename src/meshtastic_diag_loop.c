/* SPDX-License-Identifier: GPL-3.0
 *
 * Combined diagnostic loop (agents-wu94.12.1): mt_thread_summary and
 * mt_supervisor_thread merged onto one thread. Both walked EVERY thread via
 * the same Zephyr thread_analyzer_run() API, at the same priority
 * (K_LOWEST_APPLICATION_THREAD_PRIO), and neither ever ran concurrently with
 * the other in practice -- yet each paid for its own full stack. This is one
 * thread and one stack for both. The two features stay independent
 * (CONFIG_MESHTASTIC_THREAD_SUMMARY / CONFIG_MESHTASTIC_SUPERVISOR can each
 * be on alone), each on its OWN interval via the due-time dispatch
 * meshtastic_metrics.c's telemetry_thread_fn already uses for its own
 * sub-jobs -- one is not glued to the other's cadence. When both are due on
 * the same wake, one shared thread_analyzer_run() sweep feeds both callbacks
 * instead of walking every thread twice.
 */

#include <stdarg.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/stats/stats.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_heap.h>

#include "meshtastic_diag_sched.h"

#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_phoneapi.h"
#endif

LOG_MODULE_REGISTER(mt_diag, CONFIG_MESHTASTIC_LOG_LEVEL);

/* ==========================================================================
 * Thread-summary half (unchanged from meshtastic_thread_summary.c)
 * ========================================================================== */
#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)

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

#endif /* MESHTASTIC_THREAD_SUMMARY */

/* ==========================================================================
 * Supervisor half (unchanged from meshtastic_supervisor.c)
 * ========================================================================== */
#if defined(CONFIG_MESHTASTIC_SUPERVISOR)

/* Same symbol meshtastic_watchdog.c's heartbeat and meshtastic_fatal.c's
 * breadcrumb already read -- see either file's comment for why this is the
 * heap that actually matters (k_malloc()'s backing pool, and on this port
 * the vendored WiFi/BT HAL's heap_caps_malloc() is a thin wrapper over it
 * too). Real on every board with a watchdog0 alias; THREAD_ANALYZER implies
 * a real target too, so native_sim is not a concern here the way it is for
 * meshtastic_fatal.c's portable build. */
extern struct k_heap _system_heap;

STATS_SECT_START(mt_supervisor)
STATS_SECT_ENTRY(stack_warn)
STATS_SECT_ENTRY(stack_recovered)
STATS_SECT_ENTRY(cpu_warn)
STATS_SECT_ENTRY(cpu_recovered)
STATS_SECT_ENTRY(heap_warn)
STATS_SECT_ENTRY(heap_recovered)
STATS_SECT_END;

STATS_NAME_START(mt_supervisor)
STATS_NAME(mt_supervisor, stack_warn)
STATS_NAME(mt_supervisor, stack_recovered)
STATS_NAME(mt_supervisor, cpu_warn)
STATS_NAME(mt_supervisor, cpu_recovered)
STATS_NAME(mt_supervisor, heap_warn)
STATS_NAME(mt_supervisor, heap_recovered)
STATS_NAME_END(mt_supervisor);

static STATS_SECT_DECL(mt_supervisor) mt_supervisor;

#define MAX_TRACKED_THREADS 32

struct thread_state {
	char name[24];
	bool valid;
	bool stack_warn;
	bool cpu_warn;
	uint16_t cpu_over_count;
};

static struct thread_state threads[MAX_TRACKED_THREADS];
static size_t threads_used;

static bool heap_warn_latched;

/* Send a short text alert to whatever PhoneAPI transports are attached,
 * without touching the radio -- same mechanism and same reasoning as
 * meshtastic_send_local_stats_to_phone() (meshtastic_metrics.c), on its own
 * port so it carries no wire-format compatibility burden and is never
 * subject to meshtastic_sched_tier_for() airtime gating (it never reaches
 * meshtastic_send_data() at all). Deliberately independent of
 * MESHTASTIC_PHONELOG's runtime verbosity knob -- see Kconfig.supervisor. */
static void alert_phone(const char *text, size_t len)
{
#if defined(CONFIG_MESHTASTIC_SUPERVISOR_TO_PHONE)
	struct meshtastic_packet pkt = {0};

	if (len > MESHTASTIC_MAX_PAYLOAD_LEN) {
		len = MESHTASTIC_MAX_PAYLOAD_LEN;
	}

	pkt.portnum = MESHTASTIC_PORT_SUPERVISOR_ALERT;
	pkt.from = meshtastic_get_node_id();
	pkt.to = MESHTASTIC_NODE_BROADCAST;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.payload = (const uint8_t *)text;
	pkt.payload_len = (uint16_t)len;

	meshtastic_phoneapi_on_packet(&pkt, NULL);
#else
	ARG_UNUSED(text);
	ARG_UNUSED(len);
#endif
}

static void alert(bool warn, const char *fmt, ...)
{
	char buf[96];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (n <= 0) {
		return;
	}
	if ((size_t)n >= sizeof(buf)) {
		n = (int)sizeof(buf) - 1;
	}

	if (warn) {
		LOG_WRN("%s", buf);
	} else {
		LOG_INF("%s", buf);
	}
	alert_phone(buf, (size_t)n);
}

static struct thread_state *state_for(const char *name)
{
	size_t i;

	for (i = 0; i < threads_used; i++) {
		if (strncmp(threads[i].name, name, sizeof(threads[i].name) - 1) == 0) {
			return &threads[i];
		}
	}

	if (threads_used >= MAX_TRACKED_THREADS) {
		/* More threads than the table holds -- drop silently rather than
		 * corrupt an existing slot. STATS still counts real transitions
		 * for every thread that DOES fit; the thread count itself is
		 * visible via `kernel thread stacks` if this ever needs raising. */
		return NULL;
	}

	strncpy(threads[threads_used].name, name, sizeof(threads[threads_used].name) - 1);
	threads[threads_used].valid = true;
	threads_used++;
	return &threads[threads_used - 1];
}

static void check_thread(struct thread_analyzer_info *info)
{
	struct thread_state *st;
	size_t stack_pct;

	/* ISR<core> is the shared interrupt stack, not a schedulable thread --
	 * same exclusion the thread-summary half makes, for the same reason (no
	 * per-thread utilization, and stack% there measures something different
	 * in kind). */
	if (strncmp(info->name, "ISR", 3) == 0) {
		return;
	}

	st = state_for(info->name);
	if (st == NULL) {
		return;
	}

	stack_pct = info->stack_size ? (info->stack_used * 100U) / info->stack_size : 0U;

	if (stack_pct >= (size_t)CONFIG_MESHTASTIC_SUPERVISOR_STACK_WARN_PCT) {
		if (!st->stack_warn) {
			st->stack_warn = true;
			STATS_INC(mt_supervisor, stack_warn);
			alert(true, "supervisor: WARN stack %s at %zu%% (>= %d%%)", info->name,
			      stack_pct, CONFIG_MESHTASTIC_SUPERVISOR_STACK_WARN_PCT);
		}
	} else if (st->stack_warn) {
		st->stack_warn = false;
		STATS_INC(mt_supervisor, stack_recovered);
		alert(false, "supervisor: OK stack %s recovered, now %zu%%", info->name, stack_pct);
	}

#if defined(CONFIG_THREAD_RUNTIME_STATS) && defined(CONFIG_MESHTASTIC_SUPERVISOR_CPU_WARN_PCT)
	/* The plan's own liveness table says "any NON-IDLE thread" -- the idle
	 * thread being busy IS the system being idle (info->utilization measures
	 * time spent RUNNING, and idle's job is to run whenever nothing else
	 * wants the CPU), so a high idle% is the healthy case, not a runaway
	 * thread. Zephyr names it "idle" (or "idle NN" with
	 * CONFIG_MP_MAX_NUM_CPUS > 1; kernel/init.c) -- prefix match covers both. */
	if (strncmp(info->name, "idle", 4) != 0) {
		if (info->utilization >= (unsigned int)CONFIG_MESHTASTIC_SUPERVISOR_CPU_WARN_PCT) {
			if (st->cpu_over_count < UINT16_MAX) {
				st->cpu_over_count++;
			}
			if (!st->cpu_warn && st->cpu_over_count >=
						     (uint16_t)CONFIG_MESHTASTIC_SUPERVISOR_CPU_WARN_CYCLES) {
				st->cpu_warn = true;
				STATS_INC(mt_supervisor, cpu_warn);
				alert(true, "supervisor: WARN cpu %s at %u%% for %u samples",
				      info->name, info->utilization, st->cpu_over_count);
			}
		} else {
			st->cpu_over_count = 0;
			if (st->cpu_warn) {
				st->cpu_warn = false;
				STATS_INC(mt_supervisor, cpu_recovered);
				alert(false, "supervisor: OK cpu %s recovered, now %u%%",
				      info->name, info->utilization);
			}
		}
	}
#endif
}

static void check_heap(void)
{
	struct sys_memory_stats stats;

	if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) != 0) {
		return;
	}

	if (stats.free_bytes < (size_t)CONFIG_MESHTASTIC_SUPERVISOR_HEAP_MIN_BYTES) {
		if (!heap_warn_latched) {
			heap_warn_latched = true;
			STATS_INC(mt_supervisor, heap_warn);
			alert(true, "supervisor: WARN heap free=%zu (< %d)", stats.free_bytes,
			      CONFIG_MESHTASTIC_SUPERVISOR_HEAP_MIN_BYTES);
		}
	} else if (heap_warn_latched) {
		heap_warn_latched = false;
		STATS_INC(mt_supervisor, heap_recovered);
		alert(false, "supervisor: OK heap recovered, free=%zu", stats.free_bytes);
	}
}

#endif /* MESHTASTIC_SUPERVISOR */

/* ==========================================================================
 * Combined due-time loop
 * ========================================================================== */

#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY) && defined(CONFIG_MESHTASTIC_SUPERVISOR)
/* Set for the duration of one thread_analyzer_run() sweep so the single
 * combined callback below knows which half(ves) actually want this pass --
 * avoids a second full walk of every thread when both happen to be due on
 * the same wake. */
static bool sweep_summary_due;
static bool sweep_supervisor_due;

static void combined_cb(struct thread_analyzer_info *info)
{
	if (sweep_summary_due) {
		summary_cb(info);
	}
	if (sweep_supervisor_due) {
		check_thread(info);
	}
}
#endif

enum diag_job_id {
#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)
	DIAG_JOB_SUMMARY,
#endif
#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
	DIAG_JOB_SUPERVISOR,
#endif
	DIAG_JOB_COUNT,
};

/* Due-time bookkeeping itself lives in meshtastic_diag_sched.c/.h, split out
 * from behind THREAD_ANALYZER's native_sim gating so it stays unit-tested
 * (tests/diag_sched) even though this thread only ever builds for real
 * hardware. */
static struct mt_diag_job jobs[DIAG_JOB_COUNT];

/* Never sleep longer than this even with nothing due -- matches the same
 * belt-and-suspenders cap meshtastic_metrics.c's telemetry_thread_fn uses. */
#define DIAG_LOOP_MAX_SLEEP_SEC 3600U

static void diag_loop_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
	(void)STATS_INIT_AND_REG(mt_supervisor, STATS_SIZE_32, "mt_supervisor");
#endif

#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)
	jobs[DIAG_JOB_SUMMARY].interval_sec = CONFIG_MESHTASTIC_THREAD_SUMMARY_INTERVAL;
#endif
#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
	jobs[DIAG_JOB_SUPERVISOR].interval_sec = CONFIG_MESHTASTIC_SUPERVISOR_INTERVAL;
#endif

	for (;;) {
		uint32_t now = (uint32_t)k_uptime_seconds();
		uint32_t sleep_sec = DIAG_LOOP_MAX_SLEEP_SEC;
#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)
		bool summary_due = mt_diag_job_due(&jobs[DIAG_JOB_SUMMARY], now);
#endif
#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
		bool supervisor_due = mt_diag_job_due(&jobs[DIAG_JOB_SUPERVISOR], now);
#endif

#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY) && defined(CONFIG_MESHTASTIC_SUPERVISOR)
		if (summary_due || supervisor_due) {
			sweep_summary_due = summary_due;
			sweep_supervisor_due = supervisor_due;

			if (summary_due) {
				LOG_INF("Thread summary (stack%%/cpu%%):");
			}
			thread_analyzer_run(combined_cb, 0);
			if (summary_due) {
				flush_line();
				mt_diag_job_mark_fired(&jobs[DIAG_JOB_SUMMARY], now);
			}
			if (supervisor_due) {
				check_heap();
				mt_diag_job_mark_fired(&jobs[DIAG_JOB_SUPERVISOR], now);
			}
		}
#elif defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)
		if (summary_due) {
			LOG_INF("Thread summary (stack%%/cpu%%):");
			thread_analyzer_run(summary_cb, 0);
			flush_line();
			mt_diag_job_mark_fired(&jobs[DIAG_JOB_SUMMARY], now);
		}
#elif defined(CONFIG_MESHTASTIC_SUPERVISOR)
		if (supervisor_due) {
			thread_analyzer_run(check_thread, 0);
			check_heap();
			mt_diag_job_mark_fired(&jobs[DIAG_JOB_SUPERVISOR], now);
		}
#endif

#if defined(CONFIG_MESHTASTIC_THREAD_SUMMARY)
		sleep_sec = MIN(sleep_sec, mt_diag_job_remaining(&jobs[DIAG_JOB_SUMMARY], now));
#endif
#if defined(CONFIG_MESHTASTIC_SUPERVISOR)
		sleep_sec = MIN(sleep_sec, mt_diag_job_remaining(&jobs[DIAG_JOB_SUPERVISOR], now));
#endif

		/* Never a zero sleep: a job already overdue was fired above, and a
		 * rounding-down of sub-second remainder must not spin. */
		k_sleep(K_SECONDS(MAX(sleep_sec, 1U)));
	}
}

K_THREAD_DEFINE(mt_diag_loop, CONFIG_MESHTASTIC_DIAG_LOOP_STACK_SIZE, diag_loop_thread_fn, NULL, NULL,
		 NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
