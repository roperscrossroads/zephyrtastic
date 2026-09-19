/* SPDX-License-Identifier: GPL-3.0
 *
 * mt_diag_job due-time bookkeeping (agents-wu94.12.1) -- pure logic, no
 * Zephyr kernel involved: the caller supplies "now" directly, so every
 * assertion here is exact, not "roughly after a sleep".
 */
#include <zephyr/ztest.h>

#include "meshtastic_diag_sched.h"

ZTEST_SUITE(diag_sched, NULL, NULL, NULL, NULL, NULL);

ZTEST(diag_sched, test_a_fresh_job_is_due_immediately_regardless_of_now)
{
	struct mt_diag_job job = {.interval_sec = 300};

	/* Mirrors the pre-merge threads: both ran their sweep immediately at
	 * thread start, not after waiting out a full interval first. */
	zassert_true(mt_diag_job_due(&job, 0), "due at t=0 before ever firing");
	zassert_true(mt_diag_job_due(&job, 12345), "due at any t before ever firing");
	zassert_equal(mt_diag_job_remaining(&job, 999), 0U,
		      "nothing to wait for before the first fire");
}

ZTEST(diag_sched, test_due_and_remaining_track_the_configured_interval)
{
	struct mt_diag_job job = {.interval_sec = 10};

	mt_diag_job_mark_fired(&job, 100);

	zassert_false(mt_diag_job_due(&job, 100), "not due the instant it fired");
	zassert_equal(mt_diag_job_remaining(&job, 100), 10U, "");

	zassert_false(mt_diag_job_due(&job, 105), "not due mid-interval");
	zassert_equal(mt_diag_job_remaining(&job, 105), 5U, "");

	zassert_false(mt_diag_job_due(&job, 109), "not due one second early");
	zassert_equal(mt_diag_job_remaining(&job, 109), 1U, "");

	zassert_true(mt_diag_job_due(&job, 110), "due exactly at the interval");
	zassert_equal(mt_diag_job_remaining(&job, 110), 0U, "");

	zassert_true(mt_diag_job_due(&job, 200), "still due long after -- overdue, not un-due");
	zassert_equal(mt_diag_job_remaining(&job, 200), 0U,
		      "remaining clamps to 0 when overdue, never wraps negative");
}

ZTEST(diag_sched, test_firing_one_job_does_not_touch_another)
{
	struct mt_diag_job a = {.interval_sec = 300};
	struct mt_diag_job b = {.interval_sec = 30};

	mt_diag_job_mark_fired(&a, 1000);
	mt_diag_job_mark_fired(&b, 1000);

	/* b's own several fires in between must leave a's deadline untouched --
	 * the exact coupling bug class the due-time split (as opposed to a
	 * single shared "next wake" timestamp) exists to prevent. */
	for (uint32_t now = 1030; now < 1000 + 300; now += 30) {
		zassert_true(mt_diag_job_due(&b, now), "b due on its own 30s cadence");
		mt_diag_job_mark_fired(&b, now);
		zassert_false(mt_diag_job_due(&a, now), "a must not be pulled early by b firing");
	}

	zassert_true(mt_diag_job_due(&a, 1000 + 300), "a is due once its own interval elapses");
}

ZTEST(diag_sched, test_a_merged_loop_sleeps_for_the_nearer_of_two_deadlines)
{
	struct mt_diag_job slow = {.interval_sec = 300};
	struct mt_diag_job fast = {.interval_sec = 30};
	uint32_t now = 500;

	mt_diag_job_mark_fired(&slow, now);
	mt_diag_job_mark_fired(&fast, now);

	uint32_t sleep_for = mt_diag_job_remaining(&slow, now);
	uint32_t fast_remaining = mt_diag_job_remaining(&fast, now);

	if (fast_remaining < sleep_for) {
		sleep_for = fast_remaining;
	}

	zassert_equal(sleep_for, 30U,
		      "a shared loop must wake for the NEARER deadline, not the farther one "
		      "(a wrong MIN() would silently starve the shorter-interval job)");
}
