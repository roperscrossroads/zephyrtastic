/* SPDX-License-Identifier: GPL-3.0 */
#include "meshtastic_diag_sched.h"

bool mt_diag_job_due(const struct mt_diag_job *job, uint32_t now)
{
	if (!job->fired_once) {
		return true;
	}
	return (now - job->last_sec) >= job->interval_sec;
}

void mt_diag_job_mark_fired(struct mt_diag_job *job, uint32_t now)
{
	job->last_sec = now;
	job->fired_once = true;
}

uint32_t mt_diag_job_remaining(const struct mt_diag_job *job, uint32_t now)
{
	uint32_t due;

	if (!job->fired_once) {
		return 0U;
	}
	due = job->last_sec + job->interval_sec;
	return (due > now) ? (due - now) : 0U;
}
