/* SPDX-License-Identifier: GPL-3.0
 *
 * Log-to-phone backend tests.
 *
 * The interesting properties are not "does a string arrive". They are the three
 * ways this feature could hurt the node it is supposed to be diagnosing:
 *
 *   - it must not evict real traffic out of a bounded phone queue,
 *   - it must not be able to run away (a log backend that logs is a loop), and
 *   - it must stop when told to.
 *
 * Each test emits real LOG_* calls and waits for the logging thread to drain
 * them, so what is asserted is what a phone would actually receive.
 *
 * TWO TRANSPORTS, deliberately. `small` is four frames deep and is what the
 * queue-full behaviour is measured on; `big` is deep enough that it always has
 * room, so the rate-cap test is not silently measuring the queue instead. Both
 * are registered at once because the PhoneAPI fans every record to every
 * transport, which is also the arrangement a real node has (BLE plus serial).
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>

#include "meshtastic/mesh.pb.h"

#include "meshtastic_phoneapi.h"
#include "meshtastic_phonelog.h"

LOG_MODULE_REGISTER(phonelog_test, LOG_LEVEL_DBG);

#define SMALL_Q 4U
#define BIG_Q   250U

static struct meshtastic_phoneapi_frame small_storage[SMALL_Q];
static struct meshtastic_phoneapi small;
static meshtastic_ToRadio small_to;
static meshtastic_FromRadio small_from;

static struct meshtastic_phoneapi_frame big_storage[BIG_Q];
static struct meshtastic_phoneapi big;
static meshtastic_ToRadio big_to;
static meshtastic_FromRadio big_from;

BUILD_ASSERT(BIG_Q > CONFIG_MESHTASTIC_PHONELOG_RATE * 2,
	     "the deep queue must outlast the burst, or the rate test measures the queue");

/* Let the logging thread run. Deferred logging hands messages to a separate
 * thread, so an assertion made immediately after a LOG_INF() would race it. */
static void drain_log(void)
{
	log_flush();
	k_sleep(K_MSEC(50));
}

static void empty(struct meshtastic_phoneapi *api)
{
	struct meshtastic_phoneapi_frame frame;

	while (meshtastic_phoneapi_pop_frame(api, &frame)) {
	}
}

static void phonelog_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Drain anything the previous test queued BEFORE resetting, so a late
	 * message cannot land in the next test's counters. */
	drain_log();
	meshtastic_phoneapi_init(&small, "small", small_storage, SMALL_Q, NULL, NULL, NULL, NULL,
				 &small_to, &small_from);
	meshtastic_phoneapi_init(&big, "big", big_storage, BIG_Q, NULL, NULL, NULL, NULL, &big_to,
				 &big_from);
	meshtastic_phoneapi_register(&small);
	meshtastic_phoneapi_register(&big);
	meshtastic_phoneapi_reset(&small);
	meshtastic_phoneapi_reset(&big);
	(void)meshtastic_phonelog_set_level(LOG_LEVEL_INF);
	meshtastic_phonelog_reset_stats();
}

ZTEST_SUITE(phonelog, NULL, NULL, phonelog_before, NULL, NULL);

/* Drain @p api, counting LogRecords and reporting whether one contained
 * @p needle (NULL to skip the search). The last matching record is copied out. */
static uint32_t drain(struct meshtastic_phoneapi *api, const char *needle,
		      meshtastic_LogRecord *out, bool *found)
{
	struct meshtastic_phoneapi_frame frame;
	uint32_t records = 0U;

	if (found != NULL) {
		*found = false;
	}

	while (meshtastic_phoneapi_pop_frame(api, &frame)) {
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		pb_istream_t stream = pb_istream_from_buffer(frame.data, frame.len);

		if (!pb_decode(&stream, meshtastic_FromRadio_fields, &from)) {
			continue;
		}
		if (from.which_payload_variant != meshtastic_FromRadio_log_record_tag) {
			continue;
		}

		records++;
		if (needle != NULL && strstr(from.log_record.message, needle) != NULL) {
			if (found != NULL) {
				*found = true;
			}
			if (out != NULL) {
				*out = from.log_record;
			}
		}
	}

	return records;
}

/* ------------------------------------------------------------------ */

ZTEST(phonelog, test_message_reaches_the_phone)
{
	meshtastic_LogRecord rec = meshtastic_LogRecord_init_zero;
	bool found = false;

	LOG_WRN("phonelog-marker-alpha %d", 42);
	drain_log();

	(void)drain(&big, "phonelog-marker-alpha 42", &rec, &found);
	zassert_true(found, "the formatted message, arguments and all, should reach the phone");
	zassert_equal(rec.level, meshtastic_LogRecord_Level_WARNING,
		      "Zephyr WRN maps to the wire's WARNING (30), not to its own number");
	zassert_str_equal(rec.source, "phonelog_test",
			  "the source is the log module name, so the app can group by it");
}

/* The message text must NOT be re-prefixed with source/level/timestamp: the app
 * renders those from the record's own fields, and duplicating them would make
 * every line in its Debug panel read twice. */
ZTEST(phonelog, test_message_carries_no_duplicate_prefix)
{
	meshtastic_LogRecord rec = meshtastic_LogRecord_init_zero;
	bool found = false;

	LOG_INF("phonelog-marker-bravo");
	drain_log();

	(void)drain(&big, "phonelog-marker-bravo", &rec, &found);
	zassert_true(found);
	zassert_equal(strcmp(rec.message, "phonelog-marker-bravo"), 0,
		      "the record should be the message and nothing else, got \"%s\"",
		      rec.message);
}

ZTEST(phonelog, test_below_ceiling_is_dropped_and_counted)
{
	struct meshtastic_phonelog_stats after;
	bool found = true;

	zassert_ok(meshtastic_phonelog_set_level(LOG_LEVEL_WRN));
	drain_log();
	empty(&big);
	meshtastic_phonelog_reset_stats();

	LOG_INF("phonelog-marker-charlie");
	drain_log();

	(void)drain(&big, "phonelog-marker-charlie", NULL, &found);
	meshtastic_phonelog_get_stats(&after);

	zassert_false(found, "an INF message must not be forwarded with the ceiling at WRN");
	zassert_true(after.dropped_level > 0U,
		     "and the drop must be counted, so an empty phone panel is diagnosable");
}

ZTEST(phonelog, test_off_forwards_nothing)
{
	zassert_ok(meshtastic_phonelog_set_level(0U));
	drain_log();
	empty(&big);
	empty(&small);

	LOG_ERR("phonelog-marker-delta");
	drain_log();

	zassert_equal(drain(&big, NULL, NULL, NULL), 0U,
		      "level 0 means off, including for errors");
}

ZTEST(phonelog, test_level_above_dbg_rejected)
{
	zassert_equal(meshtastic_phonelog_set_level(LOG_LEVEL_DBG + 1), -EINVAL);
	zassert_equal(meshtastic_phonelog_get_level(), LOG_LEVEL_INF,
		      "a rejected set must not have taken effect");
}

/* The load-bearing one. A full queue must be a silent DROP, not an eviction.
 *
 * Proven by identity rather than by counting: the frames still in the small
 * queue must be the FIRST ones queued. If eviction had run, it would hold the
 * newest instead -- and the eviction path LOGS, which is the feedback loop this
 * design exists to prevent. */
ZTEST(phonelog, test_full_queue_drops_instead_of_evicting)
{
	struct meshtastic_phonelog_stats stats;
	bool found = false;

	/* Yield between messages: without it the test thread never gives the
	 * logging thread a slot and the log CORE overflows, which would test
	 * CONFIG_LOG_BUFFER_SIZE rather than the queue-full path. */
	for (int i = 0; i < (int)SMALL_Q * 3; i++) {
		LOG_INF("phonelog-fill-%02d", i);
		k_sleep(K_MSEC(2));
	}
	drain_log();

	meshtastic_phonelog_get_stats(&stats);
	zassert_true(stats.dropped_queue > 0U,
		     "past the small queue's depth, records must be dropped and counted");
	zassert_equal(meshtastic_phoneapi_pending_count(&small), SMALL_Q,
		      "the queue is bounded and should be exactly full");

	(void)drain(&small, "phonelog-fill-00", NULL, &found);
	zassert_true(found,
		     "the FIRST record must still be there: a full queue drops the newcomer, "
		     "it does not evict to make room (and evicting would LOG)");
}

/* The rate cap. Even with a loop the other guards missed, forwarding saturates
 * rather than consuming the node.
 *
 * No yields, so the whole burst lands inside one refill window, and measured on
 * the deep queue so the queue-full path cannot be what bounds it. dropped_core
 * is asserted zero because a log-core overflow would ALSO hold `forwarded` down
 * and make this pass for the wrong reason. */
ZTEST(phonelog, test_rate_cap_bounds_a_flood)
{
	struct meshtastic_phonelog_stats stats;
	const int burst = CONFIG_MESHTASTIC_PHONELOG_RATE * 2;

	for (int i = 0; i < burst; i++) {
		LOG_INF("phonelog-flood-%03d", i);
	}
	drain_log();

	meshtastic_phonelog_get_stats(&stats);
	zassert_equal(stats.dropped_core, 0U,
		      "the log core dropped %u messages -- raise CONFIG_LOG_BUFFER_SIZE; the "
		      "rate cap cannot be measured through a core overflow",
		      stats.dropped_core);
	zassert_true(stats.forwarded <= (uint32_t)CONFIG_MESHTASTIC_PHONELOG_RATE + 1U,
		     "forwarded (%u) must not exceed the per-second cap (%d)", stats.forwarded,
		     CONFIG_MESHTASTIC_PHONELOG_RATE);
	zassert_true(stats.dropped_rate > 0U, "and the excess must be counted, not silent");
}

/* Guard 1, asserted rather than merely commented: the backend's own module is
 * never forwarded, so nothing it logs can become a record that makes it log
 * again. */
ZTEST(phonelog, test_backend_never_reports_itself)
{
	struct meshtastic_phoneapi_frame frame;

	for (int i = 0; i < (int)SMALL_Q * 6; i++) {
		LOG_INF("phonelog-selftest-%02d", i);
		k_sleep(K_MSEC(2));
	}
	drain_log();

	while (meshtastic_phoneapi_pop_frame(&big, &frame)) {
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		pb_istream_t stream = pb_istream_from_buffer(frame.data, frame.len);

		if (!pb_decode(&stream, meshtastic_FromRadio_fields, &from) ||
		    from.which_payload_variant != meshtastic_FromRadio_log_record_tag) {
			continue;
		}
		zassert_not_equal(strcmp(from.log_record.source, "mt_phonelog"), 0,
				  "the log backend must never forward its own output");
	}
}
