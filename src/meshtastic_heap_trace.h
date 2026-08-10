/* SPDX-License-Identifier: GPL-3.0
 *
 * Diagnostic-only _system_heap alloc/free event trace. See Kconfig.heap_trace
 * for what this exists to answer and meshtastic_heap_trace.c for the
 * implementation notes (why the callback can't just query the heap's own
 * stats).
 */

#ifndef MESHTASTIC_HEAP_TRACE_H_
#define MESHTASTIC_HEAP_TRACE_H_

#if defined(CONFIG_MESHTASTIC_HEAP_TRACE)

/**
 * Register the alloc/free listeners on _system_heap. Safe to call once at
 * boot, as early as possible so boot-time allocations (WiFi HAL, dynamic
 * thread stacks) are captured too, not just steady-state churn.
 */
void meshtastic_heap_trace_init(void);

#else

static inline void meshtastic_heap_trace_init(void)
{
}

#endif /* CONFIG_MESHTASTIC_HEAP_TRACE */

#endif /* MESHTASTIC_HEAP_TRACE_H_ */
