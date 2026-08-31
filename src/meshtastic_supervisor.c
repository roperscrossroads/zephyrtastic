/* SPDX-License-Identifier: GPL-3.0
 *
 * Health supervisor: watches per-thread stack%/CPU% (thread_analyzer_run(),
 * the same public API meshtastic_thread_summary.c uses) and the system heap
 * floor, and escalates on a state TRANSITION only -- never every sample. See
 * Kconfig.supervisor for the full rationale and the four-rung ladder.
 *
 * Phase 3 of the diagnostics-baseline plan. The real motivation: ESP32-S3
 * declares no ARCH_HAS_STACK_PROTECTION, so STACK_SENTINEL (checked only at
 * context switch) is the only overflow detection that SoC gets, and a fast
 * deep recursion can smash past it before any switch occurs -- demonstrated
 * on hardware by `meshtastic crashtest stack`: a bare WATCHDOG reset, no
 * fatal, no coredump, nothing in the log ring. Detection there is
 * opportunistic and cannot be improved in config. This is the mitigation:
 * watch headroom SHRINK instead of waiting for an overflow that may leave no
 * evidence at all.
 */

#include <stdarg.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>
#include <zephyr/stats/stats.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_heap.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_phoneapi.h"

LOG_MODULE_REGISTER(mt_supervisor, CONFIG_MESHTASTIC_LOG_LEVEL);

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
	 * same exclusion meshtastic_thread_summary.c makes, for the same reason
	 * (no per-thread utilization, and stack% there measures something
	 * different in kind). */
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
	 * thread. Missing this exclusion made every lightly-loaded boot WARN on
	 * "cpu idle at ~96%" within a few samples -- caught on rzr4's first real
	 * boot (2026-08-30), which is also what pushed the supervisor thread's
	 * own stack usage into ITS warning (see CONFIG_MESHTASTIC_SUPERVISOR_STACK_SIZE):
	 * a spurious alert exercises the exact same alert()/phoneapi call depth a
	 * real one would. Zephyr names it "idle" (or "idle NN" with
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

static void supervisor_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	(void)STATS_INIT_AND_REG(mt_supervisor, STATS_SIZE_32, "mt_supervisor");

	for (;;) {
		thread_analyzer_run(check_thread, 0);
		check_heap();

		/* Deliberately NOT a watchdog feed -- see Kconfig.supervisor and
		 * the file comment on meshtastic_watchdog.c's heartbeat_work_fn().
		 * This thread's own liveness is not evidence of anything else's. */
		k_sleep(K_SECONDS(CONFIG_MESHTASTIC_SUPERVISOR_INTERVAL));
	}
}

K_THREAD_DEFINE(mt_supervisor_thread, CONFIG_MESHTASTIC_SUPERVISOR_STACK_SIZE,
		 supervisor_thread_fn, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
