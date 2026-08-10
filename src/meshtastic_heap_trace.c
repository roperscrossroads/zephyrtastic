/* SPDX-License-Identifier: GPL-3.0
 *
 * Diagnostic-only: traces every alloc/free on _system_heap (the general
 * k_malloc pool, CONFIG_HEAP_MEM_POOL_SIZE) to the log. Built to explain why
 * max_allocated (the peak-since-boot high-water mark, sampled periodically
 * by meshtastic_watchdog.c's heartbeat) drifts by a couple of KB between
 * boot sessions on the same board/build even though the steady-state
 * allocated total is flat -- the periodic sample can see THAT it happened
 * but not WHAT or WHEN. See Kconfig.heap_trace, docs/KNOWN-ISSUES.md, and
 * bead agents-5v4g.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/heap_listener.h>
#include <zephyr/logging/log.h>

#include "meshtastic_heap_trace.h"

LOG_MODULE_REGISTER(mt_heap_trace, LOG_LEVEL_INF);

/* _system_heap backs k_malloc()/k_calloc()/k_thread_stack_alloc() -- not
 * declared in any public Zephyr header, so extern it directly. Same pattern
 * already used in meshtastic_watchdog.c and meshtastic_fatal.c. */
extern struct k_heap _system_heap;

/*
 * heap.c calls heap_listener_notify_alloc()/_free() from *inside* the
 * section already holding _system_heap's own spinlock (see
 * zephyr/kernel/mempool.c:z_alloc_helper() / k_free()). That means:
 *   - these two callbacks can never run concurrently with each other (the
 *     lock serializes them), so the plain static counters below don't need
 *     to be atomic;
 *   - they must NOT call anything that takes the same lock -- in particular,
 *     sys_heap_runtime_stats_get(&_system_heap.heap, ...) from in here would
 *     deadlock. The running total is tracked locally instead, which is also
 *     more precise than re-deriving it from the heap (no risk of another
 *     allocation landing between "query stats" and "log it").
 */
static uint32_t trace_event_seq;
static long trace_running_total;

static void heap_trace_alloc_cb(uintptr_t heap_id, void *mem, size_t bytes)
{
	ARG_UNUSED(heap_id);
	const char *tname = k_thread_name_get(k_current_get());

	trace_running_total += (long)bytes;
	trace_event_seq++;

	LOG_INF("heap ALLOC #%u size=%u ptr=%p running_total=%ld thread=\"%s\"",
		trace_event_seq, (unsigned int)bytes, mem, trace_running_total,
		tname != NULL ? tname : "");
}

static void heap_trace_free_cb(uintptr_t heap_id, void *mem, size_t bytes)
{
	ARG_UNUSED(heap_id);
	const char *tname = k_thread_name_get(k_current_get());

	trace_running_total -= (long)bytes;
	trace_event_seq++;

	LOG_INF("heap FREE  #%u size=%u ptr=%p running_total=%ld thread=\"%s\"",
		trace_event_seq, (unsigned int)bytes, mem, trace_running_total,
		tname != NULL ? tname : "");
}

HEAP_LISTENER_ALLOC_DEFINE(heap_trace_alloc_listener,
			    HEAP_ID_FROM_POINTER(&_system_heap.heap),
			    heap_trace_alloc_cb);
HEAP_LISTENER_FREE_DEFINE(heap_trace_free_listener,
			   HEAP_ID_FROM_POINTER(&_system_heap.heap),
			   heap_trace_free_cb);

void meshtastic_heap_trace_init(void)
{
	heap_listener_register(&heap_trace_alloc_listener);
	heap_listener_register(&heap_trace_free_listener);

	LOG_INF("Heap trace armed on _system_heap (CONFIG_HEAP_MEM_POOL_SIZE=%d)",
		CONFIG_HEAP_MEM_POOL_SIZE);
}
