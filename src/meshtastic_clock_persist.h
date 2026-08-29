/* SPDX-License-Identifier: GPL-3.0
 *
 * Carry the wall clock across a warm reset. See meshtastic_clock_persist.c.
 */
#ifndef MESHTASTIC_CLOCK_PERSIST_H_
#define MESHTASTIC_CLOCK_PERSIST_H_

#include <stdbool.h>
#include <stdint.h>

/** What the last restore attempt concluded, for `meshtastic time` to report. */
enum meshtastic_clock_persist_result {
	MESHTASTIC_CLOCK_PERSIST_NOT_TRIED = 0, /**< compiled in, has not run yet */
	MESHTASTIC_CLOCK_PERSIST_RESTORED,      /**< clock seeded from the saved epoch */
	MESHTASTIC_CLOCK_PERSIST_COLD,          /**< retained RAM did not survive */
	MESHTASTIC_CLOCK_PERSIST_NO_RECORD,     /**< warm, but nothing saved yet */
	MESHTASTIC_CLOCK_PERSIST_NO_COUNTER,    /**< the counter could not be read */
	MESHTASTIC_CLOCK_PERSIST_IMPLAUSIBLE,   /**< the measured gap failed its bound */
};

/** Outcome of the boot-time restore, and the gap it measured. */
void meshtastic_clock_persist_status(enum meshtastic_clock_persist_result *result,
				     uint32_t *downtime_ms);

/** Human-readable form of @p result. */
const char *meshtastic_clock_persist_result_str(enum meshtastic_clock_persist_result result);

#if defined(CONFIG_ZTEST)
#include "meshtastic_clock.h"
/**
 * Test seam: place a saved record and run the restore decision against a given
 * gap, without needing a real reset.
 *
 * Reaches the two claims that are silent when wrong — a gap beyond the bound
 * must be refused, and a restore must land at DEVICE quality rather than at the
 * quality the time originally had.
 */
enum meshtastic_clock_persist_result
meshtastic_clock_persist_test_restore(int64_t epoch_ms, enum meshtastic_clock_quality saved_q,
				      uint32_t gap_ms);
#endif

#endif /* MESHTASTIC_CLOCK_PERSIST_H_ */
