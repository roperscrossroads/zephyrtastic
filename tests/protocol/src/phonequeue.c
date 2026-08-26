/* SPDX-License-Identifier: GPL-3.0
 *
 * Phone FromRadio queue selective-eviction tests.
 *
 * Exercises meshtastic_phoneapi_enqueue_fromradio() under a saturated queue for
 * both phone.evict policies:
 *   - protect     : text/routing/admin frames survive a burst of droppable
 *                   position/telemetry/nodeinfo frames (finding C).
 *   - drop-oldest : legacy strict-oldest eviction.
 *
 * The queue holds encoded FromRadio bytes, so surviving frames are decoded back
 * with nanopb to check *which* portnum survived, not merely how many.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>
#include "meshtastic/mesh.pb.h"

#include "meshtastic_phoneapi.h"
#include "meshtastic_sched.h"

#define Q_SIZE 4U

static struct meshtastic_phoneapi_frame q_storage[Q_SIZE];
static struct meshtastic_phoneapi api;
static meshtastic_ToRadio to_scratch;
static meshtastic_FromRadio from_scratch;

static void phonequeue_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_sched_defaults();
	meshtastic_sched_stats_reset();
	meshtastic_phoneapi_init(&api, "test", q_storage, Q_SIZE, NULL, NULL, NULL, NULL,
				 &to_scratch, &from_scratch);
}

/* Build a FromRadio carrying a decoded MeshPacket on the given portnum. `id`
 * lands in packet.id so a decode can identify the survivor. */
static meshtastic_FromRadio make_pkt(uint32_t id, meshtastic_PortNum portnum)
{
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;

	from.which_payload_variant = meshtastic_FromRadio_packet_tag;
	from.packet.id = id;
	from.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	from.packet.decoded.portnum = portnum;
	from.packet.decoded.payload.size = 1U;
	from.packet.decoded.payload.bytes[0] = (uint8_t)id;
	return from;
}

/* A FromRadio carrying a log record — the cheapest thing above a queue status. */
static meshtastic_FromRadio make_log(const char *msg)
{
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;

	from.which_payload_variant = meshtastic_FromRadio_log_record_tag;
	strncpy(from.log_record.message, msg, sizeof(from.log_record.message) - 1U);
	from.log_record.level = meshtastic_LogRecord_Level_INFO;
	return from;
}

/* A FromRadio carrying a queue status — the most replaceable frame there is,
 * since the next one supersedes it outright. */
static meshtastic_FromRadio make_status(void)
{
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;

	from.which_payload_variant = meshtastic_FromRadio_queueStatus_tag;
	from.queueStatus.free = 1U;
	return from;
}

/* Decode a popped frame and return its packet portnum (or -1 if not a packet). */
static int frame_portnum(const struct meshtastic_phoneapi_frame *f)
{
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
	pb_istream_t stream = pb_istream_from_buffer(f->data, f->len);

	if (!pb_decode(&stream, meshtastic_FromRadio_fields, &from)) {
		return -1;
	}
	if (from.which_payload_variant != meshtastic_FromRadio_packet_tag) {
		return -1;
	}
	return (int)from.packet.decoded.portnum;
}

/* Count surviving frames of a given portnum by draining the queue. */
static uint8_t drain_count_portnum(int portnum)
{
	struct meshtastic_phoneapi_frame f;
	uint8_t n = 0;

	while (meshtastic_phoneapi_pop_frame(&api, &f)) {
		if (frame_portnum(&f) == portnum) {
			n++;
		}
	}
	return n;
}

/* protect: a full queue of protected TEXT frames rejects an incoming droppable
 * TELEMETRY frame rather than evicting a text message. */
ZTEST(phonequeue, test_protect_rejects_incoming_droppable)
{
	struct meshtastic_sched_stats st;

	for (uint32_t i = 0; i < Q_SIZE; i++) {
		meshtastic_FromRadio t = make_pkt(i + 1, meshtastic_PortNum_TEXT_MESSAGE_APP);

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &t));
	}

	meshtastic_FromRadio tel = make_pkt(99, meshtastic_PortNum_TELEMETRY_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &tel));

	/* All four text messages must still be present; the telemetry was dropped. */
	zassert_equal(drain_count_portnum(meshtastic_PortNum_TEXT_MESSAGE_APP), 4,
		      "protected text frames must survive");

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.phone_drop, 1, "one droppable frame dropped");
	zassert_equal(st.phone_drop_protected, 0, "no protected frame dropped");
}

/* protect: with a droppable frame queued, an incoming protected frame evicts
 * the droppable one — not the oldest. */
ZTEST(phonequeue, test_protect_evicts_droppable_not_oldest)
{
	struct meshtastic_sched_stats st;

	/* Queue: TEXT, TELEMETRY, TEXT, TELEMETRY (full). */
	meshtastic_FromRadio a = make_pkt(1, meshtastic_PortNum_TEXT_MESSAGE_APP);
	meshtastic_FromRadio b = make_pkt(2, meshtastic_PortNum_TELEMETRY_APP);
	meshtastic_FromRadio c = make_pkt(3, meshtastic_PortNum_TEXT_MESSAGE_APP);
	meshtastic_FromRadio d = make_pkt(4, meshtastic_PortNum_TELEMETRY_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &a));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &b));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &c));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &d));

	/* Incoming protected TEXT: should evict the oldest droppable (id 2). */
	meshtastic_FromRadio e = make_pkt(5, meshtastic_PortNum_TEXT_MESSAGE_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &e));

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.phone_drop, 1, "one droppable frame evicted");
	zassert_equal(st.phone_drop_protected, 0, "no protected frame dropped");

	/* Now: 3 text (1,3,5) + 1 telemetry (4). */
	zassert_equal(drain_count_portnum(meshtastic_PortNum_TEXT_MESSAGE_APP), 3);
}

/* protect, second telemetry frame case: only the older droppable is evicted. */
ZTEST(phonequeue, test_protect_evicts_oldest_droppable_first)
{
	/* Queue: TELEMETRY(1), TEXT(2), TELEMETRY(3), TEXT(4). */
	meshtastic_FromRadio a = make_pkt(1, meshtastic_PortNum_TELEMETRY_APP);
	meshtastic_FromRadio b = make_pkt(2, meshtastic_PortNum_TEXT_MESSAGE_APP);
	meshtastic_FromRadio c = make_pkt(3, meshtastic_PortNum_TELEMETRY_APP);
	meshtastic_FromRadio d = make_pkt(4, meshtastic_PortNum_TEXT_MESSAGE_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &a));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &b));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &c));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &d));

	/* Incoming protected TEXT(5): evict oldest droppable = telemetry id 1. */
	meshtastic_FromRadio e = make_pkt(5, meshtastic_PortNum_TEXT_MESSAGE_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &e));

	/* Surviving: TEXT(2), TELEMETRY(3), TEXT(4), TEXT(5). */
	struct meshtastic_phoneapi_frame f;
	uint32_t got[Q_SIZE];
	uint8_t n = 0;

	while (n < Q_SIZE && meshtastic_phoneapi_pop_frame(&api, &f)) {
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		pb_istream_t s = pb_istream_from_buffer(f.data, f.len);

		zassert_true(pb_decode(&s, meshtastic_FromRadio_fields, &from));
		got[n++] = from.packet.id;
	}

	zassert_equal(n, 4, "queue stays full");
	zassert_equal(got[0], 2, "oldest telemetry(1) evicted, text(2) now oldest");
	zassert_equal(got[1], 3);
	zassert_equal(got[2], 4);
	zassert_equal(got[3], 5);
}

/* drop-oldest: the no-backoff policy evicts the strict-oldest frame regardless
 * of tier. */
ZTEST(phonequeue, test_drop_oldest_legacy)
{
	struct meshtastic_sched_stats st;

	zassert_ok(meshtastic_sched_set("phone.evict", "drop-oldest"));

	for (uint32_t i = 0; i < Q_SIZE; i++) {
		meshtastic_FromRadio t = make_pkt(i + 1, meshtastic_PortNum_TEXT_MESSAGE_APP);

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &t));
	}

	/* Incoming TEXT(5): drops oldest TEXT(1) despite being protected. */
	meshtastic_FromRadio e = make_pkt(5, meshtastic_PortNum_TEXT_MESSAGE_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &e));

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.phone_drop, 1);
	zassert_equal(st.phone_drop_protected, 1, "legacy dropped a protected frame");

	struct meshtastic_phoneapi_frame f;
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;

	zassert_true(meshtastic_phoneapi_pop_frame(&api, &f));
	pb_istream_t s = pb_istream_from_buffer(f.data, f.len);

	zassert_true(pb_decode(&s, meshtastic_FromRadio_fields, &from));
	zassert_equal(from.packet.id, 2, "oldest (id 1) dropped, id 2 now oldest");
}

/* --- the eviction ladder (parity T-5) --------------------------------------
 *
 * "Droppable" used to be one bit, so a NodeInfo and a Telemetry were
 * interchangeable victims and whichever happened to arrive first lost. They are
 * not interchangeable. A lost Telemetry costs one sample of a value that will be
 * sent again shortly; a lost NodeInfo costs the app knowing a node EXISTS, until
 * that peer next announces itself — which since the CADENCE fix can be an hour.
 *
 * This matters more than the August audit's Tier-2 rating suggests, because the
 * fleet has no WiFi and therefore no netlog: the phone IS the diagnostic
 * channel, and "the app can't see that node" reads like a range problem rather
 * than a full queue.
 */

/* THE T-5 assertion. A burst of samples must not cost the app a node. */
ZTEST(phonequeue, test_nodeinfo_survives_a_burst_of_samples)
{
	meshtastic_FromRadio ni = make_pkt(1, meshtastic_PortNum_NODEINFO_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &ni));

	/* Fill the rest with samples, then keep pushing samples at a full queue —
	 * more than the queue is deep, so under the old single-bit rule the
	 * NodeInfo (the oldest droppable frame) would have been the first victim. */
	for (uint32_t i = 0; i < Q_SIZE * 3U; i++) {
		meshtastic_FromRadio t = make_pkt(100U + i, meshtastic_PortNum_TELEMETRY_APP);

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &t));
	}

	zassert_equal(drain_count_portnum(meshtastic_PortNum_NODEINFO_APP), 1,
		      "a NodeInfo must outrank a Position/Telemetry sample — losing it "
		      "costs the app a node, not a reading");
}

/* The bottom of the ladder, and the property the log-record comment in
 * meshtastic_phoneapi.c has always claimed: diagnostics must never outrank the
 * traffic they describe. A log burst is exactly what fills this queue now, so
 * this is the case that stops the observability layer eating the observations. */
ZTEST(phonequeue, test_a_log_burst_never_evicts_a_sample)
{
	meshtastic_FromRadio t = make_pkt(1, meshtastic_PortNum_TELEMETRY_APP);
	struct meshtastic_sched_stats st;

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &t));
	for (uint32_t i = 0; i < Q_SIZE - 1U; i++) {
		meshtastic_FromRadio l = make_log("filling");

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &l));
	}

	/* Full. Every further log must be refused at the door rather than evict
	 * the sample, even though the sample is older. */
	for (uint32_t i = 0; i < Q_SIZE * 2U; i++) {
		meshtastic_FromRadio l = make_log("more");

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &l));
	}

	zassert_equal(drain_count_portnum(meshtastic_PortNum_TELEMETRY_APP), 1,
		      "a log record must never displace the traffic it describes");
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.phone_drop_protected, 0, "nothing protected should have been dropped");
}

/* A queue status is the most replaceable frame there is — the next one
 * supersedes it outright — so it goes before a log record does. */
ZTEST(phonequeue, test_a_queue_status_is_sacrificed_before_a_log_record)
{
	meshtastic_FromRadio st_frame = make_status();
	meshtastic_FromRadio keep = make_log("keep me");
	uint8_t logs = 0;
	struct meshtastic_phoneapi_frame f;

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &st_frame));
	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &keep));
	for (uint32_t i = 0; i < Q_SIZE - 2U; i++) {
		meshtastic_FromRadio l = make_log("filling");

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &l));
	}

	/* Full, with exactly one status in it. One more log evicts the status. */
	meshtastic_FromRadio extra = make_log("extra");

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &extra));

	while (meshtastic_phoneapi_pop_frame(&api, &f)) {
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		pb_istream_t s = pb_istream_from_buffer(f.data, f.len);

		zassert_true(pb_decode(&s, meshtastic_FromRadio_fields, &from));
		zassert_not_equal(from.which_payload_variant,
				  meshtastic_FromRadio_queueStatus_tag,
				  "the queue status should have been the first thing sacrificed");
		if (from.which_payload_variant == meshtastic_FromRadio_log_record_tag) {
			logs++;
		}
	}
	zassert_equal(logs, Q_SIZE, "every log record should have survived");
}

/* At EQUAL rank the oldest goes, not the newcomer. For a repeating measurement
 * the newer sample is strictly the better one to keep, and this is the case that
 * would silently invert if the comparison were tightened to a strict less-than.
 */
ZTEST(phonequeue, test_at_equal_rank_the_newer_sample_wins)
{
	struct meshtastic_phoneapi_frame f;
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
	pb_istream_t s;

	for (uint32_t i = 0; i < Q_SIZE; i++) {
		meshtastic_FromRadio t = make_pkt(i + 1U, meshtastic_PortNum_TELEMETRY_APP);

		zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &t));
	}

	meshtastic_FromRadio newer = make_pkt(99, meshtastic_PortNum_TELEMETRY_APP);

	zassert_ok(meshtastic_phoneapi_enqueue_fromradio(&api, &newer));

	zassert_true(meshtastic_phoneapi_pop_frame(&api, &f));
	s = pb_istream_from_buffer(f.data, f.len);
	zassert_true(pb_decode(&s, meshtastic_FromRadio_fields, &from));
	zassert_equal(from.packet.id, 2U,
		      "the oldest sample should have gone, not the incoming one");
}

ZTEST_SUITE(phonequeue, NULL, NULL, phonequeue_before, NULL, NULL);
