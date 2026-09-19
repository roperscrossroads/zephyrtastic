/* SPDX-License-Identifier: GPL-3.0
 *
 * Pure due-time bookkeeping for meshtastic_diag_loop.c's merged
 * thread-summary/supervisor thread (agents-wu94.12.1). Split out of that
 * file, and out from behind its CONFIG_MESHTASTIC_THREAD_SUMMARY /
 * CONFIG_MESHTASTIC_SUPERVISOR gating (both `depends on THREAD_ANALYZER`,
 * which native_sim/ARCH_POSIX cannot build at all), specifically so the
 * scheduling logic that changed with the merge -- whether one job's cadence
 * can end up glued to the other's -- stays testable on native_sim even
 * though the thread that calls it can only ever be built for a real target.
 * No Zephyr kernel dependency beyond stdint/stdbool: the caller supplies
 * "now" and never touches wall-clock/uptime here.
 */
#ifndef MESHTASTIC_DIAG_SCHED_H_
#define MESHTASTIC_DIAG_SCHED_H_

#include <stdbool.h>
#include <stdint.h>

struct mt_diag_job {
	uint32_t interval_sec;
	uint32_t last_sec;
	bool fired_once;
};

/* True the very first time (before anything has fired -- mirrors the
 * pre-merge threads, which both ran their sweep immediately at thread start
 * rather than waiting out a full interval first), then true again once
 * interval_sec has elapsed since the last fire. */
bool mt_diag_job_due(const struct mt_diag_job *job, uint32_t now);

/* Record that the job fired at `now`. */
void mt_diag_job_mark_fired(struct mt_diag_job *job, uint32_t now);

/* Seconds until this job is next due, or 0 if it is due now (or has never
 * fired). The caller MIN()s this across every enabled job to size its sleep
 * -- never a value that could delay an ALREADY-due job. */
uint32_t mt_diag_job_remaining(const struct mt_diag_job *job, uint32_t now);

#endif /* MESHTASTIC_DIAG_SCHED_H_ */
