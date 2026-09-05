/*
 * SX126x BUSY-timeout accounting.
 *
 * This predicate decides when the driver resets a wedged radio. It was wrong twice, and both
 * bugs reached hardware, because the logic was file-scope static inside a driver that only
 * builds for real targets -- there was nothing to point a test at. These cases are the two
 * failures, written so they fail if either is reintroduced.
 */
#include <zephyr/ztest.h>

#include "sx126x_busy_track.h"

#define SET_TX    0x83
#define SET_SLEEP 0x84
#define RECOVERY_THRESHOLD 4   /* SX126X_BUSY_RECOVERY_THRESHOLD in sx126x.c */

#define PRE  false
#define POST true

static struct sx126x_busy_table t;

static void reset_table(void *unused)
{
	ARG_UNUSED(unused);
	sx126x_busy_reset(&t);
}

ZTEST(sx126x_busy, test_a_timeout_is_counted)
{
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	zassert_equal(1, sx126x_busy_worst(&t), "one timeout should count as one");
}

ZTEST(sx126x_busy, test_a_success_on_the_same_wait_clears_it)
{
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	zassert_equal(2, sx126x_busy_worst(&t));

	sx126x_busy_record_success(&t, SET_TX, POST);
	zassert_equal(0, sx126x_busy_worst(&t),
		      "the radio proved it can complete this command at this phase");
}

ZTEST(sx126x_busy, test_a_success_on_another_opcode_must_not_clear_it)
{
	/* Carried patch 0020. A chip wedged on SET_TX still answers every other command; when
	 * any success cleared the shared count it never reached the threshold, so the recovery
	 * never ran and a board failed 2514 transmits while reporting `streak 0`.
	 */
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	sx126x_busy_record_timeout(&t, SET_TX, POST);

	sx126x_busy_record_success(&t, SET_SLEEP, POST);
	sx126x_busy_record_success(&t, 0x0D /* WRITE_BUFFER */, POST);
	sx126x_busy_record_success(&t, 0xC0 /* GET_STATUS */, PRE);

	zassert_equal(2, sx126x_busy_worst(&t),
		      "an unrelated opcode's success erased the evidence of a per-command wedge");
}

ZTEST(sx126x_busy, test_the_same_opcodes_pre_wait_must_not_clear_its_post_count)
{
	/* Carried patch 0021, and the subtler of the two. Every command waits on BUSY twice:
	 * before the SPI transfer and after. In the wedged state the PRE wait cannot fail --
	 * the chip is idle until the command is issued -- so its success kept clearing the
	 * POST count. Keyed on the opcode alone the count read 0,1,0,1 forever and never
	 * reached 4, while 90% of transmits failed.
	 */
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	sx126x_busy_record_success(&t, SET_TX, PRE);

	zassert_equal(1, sx126x_busy_worst(&t),
		      "the pre-wait of the very command that is failing cleared its post count");
}

ZTEST(sx126x_busy, test_the_real_wedge_sequence_reaches_the_threshold)
{
	/* What a wedged radio actually does, four transmits running: the pre-wait succeeds
	 * because the chip is idle, the command is issued, the post-wait times out. This is the
	 * sequence that must arrive at the threshold -- under 0020 and 0021 it never did.
	 */
	for (int i = 0; i < RECOVERY_THRESHOLD; i++) {
		sx126x_busy_record_success(&t, SET_TX, PRE);    /* chip idle, pre-wait fine */
		sx126x_busy_record_timeout(&t, SET_TX, POST);   /* ...then BUSY never drops */
	}

	zassert_equal(RECOVERY_THRESHOLD, sx126x_busy_worst(&t),
		      "four failed transmits must be visible to the recovery, not hidden by the "
		      "pre-wait successes interleaved between them");
}

ZTEST(sx126x_busy, test_pre_and_post_of_one_opcode_are_separate_waits)
{
	sx126x_busy_record_timeout(&t, SET_TX, PRE);
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	sx126x_busy_record_timeout(&t, SET_TX, POST);

	/* Clearing one phase must leave the other standing. */
	sx126x_busy_record_success(&t, SET_TX, PRE);
	zassert_equal(2, sx126x_busy_worst(&t), "clearing the pre count also cleared the post");
}

ZTEST(sx126x_busy, test_reset_forgets_everything)
{
	sx126x_busy_record_timeout(&t, SET_TX, POST);
	sx126x_busy_record_timeout(&t, SET_SLEEP, PRE);
	zassert_true(sx126x_busy_worst(&t) > 0);

	sx126x_busy_reset(&t);
	zassert_equal(0, sx126x_busy_worst(&t), "the recovery reset the chip; forget the history");
}

ZTEST(sx126x_busy, test_worst_reports_the_most_stuck_wait)
{
	sx126x_busy_record_timeout(&t, 0x01, POST);
	for (int i = 0; i < 5; i++) {
		sx126x_busy_record_timeout(&t, SET_TX, POST);
	}
	sx126x_busy_record_timeout(&t, 0x02, PRE);

	zassert_equal(5, sx126x_busy_worst(&t), "worst must be the max, not the first or last");
}

ZTEST(sx126x_busy, test_a_full_table_keeps_the_worst_offender)
{
	/* Filling the table must not lose the stuck wait: that is the one the recovery needs. */
	for (int i = 0; i < SX126X_BUSY_TRACKED_WAITS; i++) {
		sx126x_busy_record_timeout(&t, (uint8_t)(0x10 + i), POST);
	}
	for (int i = 0; i < 6; i++) {
		sx126x_busy_record_timeout(&t, 0x10, POST);   /* one of them gets much worse */
	}
	zassert_equal(7, sx126x_busy_worst(&t));

	/* A ninth distinct wait evicts the LEAST stuck entry, never the worst. */
	sx126x_busy_record_timeout(&t, 0xEE, POST);
	zassert_equal(7, sx126x_busy_worst(&t),
		      "eviction discarded the most-stuck wait, which is the only one that matters");
}

ZTEST_SUITE(sx126x_busy, NULL, NULL, reset_table, NULL, NULL);
