/* SPDX-License-Identifier: GPL-3.0 */
/*
 * NodeInfo announce/request/reply tests (agents-qjqg).
 *
 * meshtastic_nodeinfo.c is 368 lines and, before this suite, the only
 * assertions that reached it were the public-key pinning tests in
 * tests/protocol -- which are really the PKI path using NodeInfo as a carrier.
 * The module's own behaviour was untested.
 *
 * What that behaviour IS, is airtime policy. Three of the module's Kconfig
 * values decide how much of a shared channel this node spends on identity
 * traffic, and each fails in a different direction:
 *
 *   NODEINFO_INTERVAL_SEC          how often we announce, unprompted
 *   NODEINFO_UNKNOWN_SUPPRESS_SEC  how often we ASK a silent stranger to identify
 *   NODEINFO_REPLY_SUPPRESS_SEC    how often we ANSWER the same asker
 *
 * Get a suppression window wrong in one direction and the node floods the
 * channel; wrong in the other and it never answers a legitimate request and
 * becomes a nameless number on every peer's node list. Neither shows up as a
 * crash, and neither was covered.
 *
 * There is also a coupling worth pinning: the announce is the ONLY thing that
 * re-publishes this node's public key, which is why `nodedb forget` alone did
 * not fix agents-xhli.22 on the bench and a forced broadcast did.
 *
 * SHAPE OF THE SUITE. The announce cadence and the suppression windows cannot
 * be measured in one image -- a start delay short enough to observe would drop
 * announces into the middle of every suppression test's frame capture. So the
 * suite builds three ways (testcase.yaml) and NODEINFO_ANNOUNCE_SCENARIO picks
 * which half compiles.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU

#if defined(CONFIG_MESHTASTIC_NODEINFO_AUTO_SEND) &&                                               \
	CONFIG_MESHTASTIC_NODEINFO_START_DELAY_SEC <= 5
#define NODEINFO_ANNOUNCE_SCENARIO 1
#else
#define NODEINFO_ANNOUNCE_SCENARIO 0
#endif

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* Uptime at meshtastic_init(), so the announce test can measure the start delay
 * from the moment the thread was actually created rather than from t=0. */
static int64_t init_uptime_ms;

/* ==========================================================================
 * Frame construction / capture
 * ========================================================================== */

#if !NODEINFO_ANNOUNCE_SCENARIO

/* The stack cancels RX while it transmits (half duplex), so a frame injected
 * straight after one that provoked a reply is refused with -EAGAIN. Every
 * inject waits for the radio to be listening again first — otherwise a test
 * fails on the injection rather than on the behaviour it is about. */
static void wait_rx_armed(void)
{
	for (int i = 0; i < 1000 && !lora_sim_rx_armed(lora_dev); i++) {
		k_msleep(2);
	}
	zassert_true(lora_sim_rx_armed(lora_dev), "radio never returned to RX");
}

static void build_peer_frame(uint32_t from, uint32_t to, uint32_t id, uint32_t portnum,
			     const uint8_t *payload, size_t payload_len, bool want_response,
			     uint8_t *wire, uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
		.want_response = want_response,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_build_wire_packet(&packet, wire, wire_len),
		   "build_wire_packet failed");
}

/* Any traffic that is NOT NodeInfo — this is what makes a peer "heard but
 * unidentified", the state that triggers a request. */
static void inject_peer_text(uint32_t from, uint32_t id, const char *text)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	build_peer_frame(from, MESHTASTIC_NODE_BROADCAST, id, MESHTASTIC_PORT_TEXT_MESSAGE,
			 (const uint8_t *)text, strlen(text), false, wire, &wire_len);
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
}

/* A peer identifying itself. @p want_response distinguishes "here is who I am"
 * from "here is who I am, now tell me who YOU are"; @p to distinguishes asking
 * US from asking the whole channel. Both matter — see
 * test_a_broadcast_request_is_not_answered. */
static void inject_peer_nodeinfo(uint32_t from, uint32_t to, uint32_t id, bool want_response)
{
	uint8_t user_buf[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	pb_ostream_t stream;
	meshtastic_User user = meshtastic_User_init_zero;

	(void)snprintk(user.id, sizeof(user.id), "!%08x", from);
	(void)strncpy(user.long_name, "Test Peer", sizeof(user.long_name) - 1);
	(void)strncpy(user.short_name, "TP", sizeof(user.short_name) - 1);

	stream = pb_ostream_from_buffer(user_buf, sizeof(user_buf));
	zassert_true(pb_encode(&stream, meshtastic_User_fields, &user), "User encode failed");

	build_peer_frame(from, to, id, MESHTASTIC_PORT_NODEINFO, user_buf, stream.bytes_written,
			 want_response, wire, &wire_len);
	wait_rx_armed();
	zassert_ok(lora_sim_inject(lora_dev, wire, (uint8_t)wire_len, -50, 7), "inject failed");
}

#endif /* !NODEINFO_ANNOUNCE_SCENARIO */

struct captured {
	uint32_t count;    /* NodeInfo frames we originated */
	uint32_t to;       /* destination of the last one */
	uint32_t request_id;
	bool want_response;
};

/*
 * Drain the capture queue, counting only NodeInfo frames THIS node originated.
 *
 * Relayed copies of the injected frame are skipped rather than asserted away:
 * the node rebroadcasts what it hears, so a peer's own NodeInfo comes back out
 * of the radio with the PEER as `from`. Counting those as ours would make every
 * suppression assertion meaningless.
 */
static struct captured drain_nodeinfo_tx(k_timeout_t settle)
{
	struct lora_sim_frame f;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct captured c = {0};

	k_sleep(settle);

	while (lora_sim_take_tx(lora_dev, &f, K_NO_WAIT) == 0) {
		if (meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &decoded, payload,
						  sizeof(payload)) != 0) {
			continue;
		}
		if (decoded.from != TEST_NODE_ID || decoded.portnum != MESHTASTIC_PORT_NODEINFO) {
			continue;
		}
		c.count++;
		c.to = decoded.to;
		c.request_id = decoded.request_id;
		c.want_response = decoded.want_response;
	}
	return c;
}

/* Long enough for the RX thread to run the module, build a User protobuf and
 * push the frame through the outbound queue onto the radio. */
#define SETTLE K_MSEC(500)

static void *nodeinfo_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	init_uptime_ms = k_uptime_get();

	/* No random pre-TX backoff: these tests time frames against suppression
	 * windows, and the contention window only adds jitter to that. */
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void nodeinfo_before(void *fixture)
{
	ARG_UNUSED(fixture);

#if !NODEINFO_ANNOUNCE_SCENARIO
	int stable = 0;

	/* Let the previous test's reply/request finish crossing the outbound queue
	 * before the capture queue is cleared, or it is counted as this test's. */
	k_msleep(200);
	for (int i = 0; i < 400 && stable < 10; i++) {
		int n = lora_sim_tx_pending(lora_dev);

		k_msleep(5);
		if (lora_sim_rx_armed(lora_dev) && lora_sim_tx_pending(lora_dev) == n) {
			stable++;
		} else {
			stable = 0;
		}
	}

	lora_sim_reset(lora_dev);
	/* Peers must not carry identity across tests: a peer this node already
	 * knows is never asked, which would make an "unknown peer" test pass for
	 * the wrong reason. */
	meshtastic_nodedb_reset(false);
#endif
}

#if !NODEINFO_ANNOUNCE_SCENARIO

/* ==========================================================================
 * Asking a stranger. Hearing non-NodeInfo traffic from a node we cannot name
 * is what prompts a request — this is how a node that joined after our last
 * announce still ends up with a name in our list.
 * ========================================================================== */

/* Each test uses its own peer id: the peer cache is module-static and survives
 * meshtastic_nodedb_reset(), so reusing an id would carry the previous test's
 * suppression stamp into this one. Distinct ids make each test's first contact
 * genuinely a first contact. */
#define PEER_UNKNOWN     0x0B000001U
#define PEER_REPEAT      0x0B000002U
#define PEER_KNOWN       0x0B000003U
#define PEER_REQUESTER   0x0B000004U
#define PEER_REQUESTER_2 0x0B000005U
#define PEER_EVICTED     0x0B000006U
#define PEER_SHOUTER     0x0B000007U

ZTEST(nodeinfo, test_unknown_peer_is_asked_to_identify)
{
	struct captured c;

	inject_peer_text(PEER_UNKNOWN, 0x1001U, "who am I");
	c = drain_nodeinfo_tx(SETTLE);

	zassert_equal(c.count, 1U, "an unnameable peer should be asked exactly once");
	zassert_equal(c.to, PEER_UNKNOWN, "the request must be a unicast to that peer");
	zassert_true(c.want_response, "a request without want_response is just an announce");
}

/*
 * The suppression window is the whole defence against amplification: every
 * inbound frame from an unidentified sender would otherwise produce an outbound
 * one, so a stream of frames with rolling ids turns cheap inbound traffic into
 * expensive outbound traffic. The window is 600 s by default; nothing here waits
 * that long, so a second request appearing at all is the failure.
 */
ZTEST(nodeinfo, test_the_same_stranger_is_not_asked_twice_inside_the_window)
{
	struct captured c;

	inject_peer_text(PEER_REPEAT, 0x1002U, "first");
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 1U, "precondition: first contact asks");

	for (int i = 0; i < 5; i++) {
		inject_peer_text(PEER_REPEAT, 0x1003U + i, "again");
	}
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 0U, "five more frames must not produce five more requests");
}

/* A peer that has already identified itself is never asked, however much other
 * traffic it sends. */
ZTEST(nodeinfo, test_a_peer_that_already_identified_is_not_asked)
{
	struct captured c;

	inject_peer_nodeinfo(PEER_KNOWN, MESHTASTIC_NODE_BROADCAST, 0x1010U, false);
	(void)drain_nodeinfo_tx(SETTLE);

	inject_peer_text(PEER_KNOWN, 0x1011U, "ordinary traffic");
	c = drain_nodeinfo_tx(SETTLE);

	zassert_equal(c.count, 0U, "we can already name this peer; asking is wasted airtime");
}

/* ==========================================================================
 * Answering a request.
 * ========================================================================== */

ZTEST(nodeinfo, test_want_response_is_answered_and_correlated)
{
	struct captured c;
	const uint32_t req_id = 0x1020U;

	inject_peer_nodeinfo(PEER_REQUESTER, TEST_NODE_ID, req_id, true);
	c = drain_nodeinfo_tx(SETTLE);

	zassert_equal(c.count, 1U, "a want_response request must be answered");
	zassert_equal(c.to, PEER_REQUESTER, "the reply is a unicast back to the asker");
	zassert_equal(c.request_id, req_id,
		      "the reply must carry the request id, or the asker cannot match it");
	zassert_false(c.want_response, "answering a request must not ask one back");
}

/*
 * A want_response addressed to the whole channel must NOT be answered. Every
 * node that hears it would answer at once — one frame becoming N, all in the
 * same slot, on the busiest possible channel state. The gate is
 * packet_is_to_us() in meshtastic_modules.c, and it is the reason the module's
 * own meshtastic_nodeinfo_request() unicasts.
 */
ZTEST(nodeinfo, test_a_broadcast_request_is_not_answered)
{
	struct captured c;

	inject_peer_nodeinfo(PEER_SHOUTER, MESHTASTIC_NODE_BROADCAST, 0x1050U, true);
	c = drain_nodeinfo_tx(SETTLE);

	zassert_equal(c.count, 0U,
		      "a broadcast want_response must not make every hearer transmit");
}

/*
 * REPLY_SUPPRESS_SEC (12 h by default) is far wider than the request window,
 * and deliberately: our identity does not change, so a peer that asks twice in
 * a day learns nothing new the second time. Pinned because "answers every
 * request" is the intuitive behaviour and is the wrong one.
 */
ZTEST(nodeinfo, test_repeated_requests_are_answered_once_per_window)
{
	struct captured c;

	inject_peer_nodeinfo(PEER_REQUESTER_2, TEST_NODE_ID, 0x1030U, true);
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 1U, "precondition: first request answered");

	for (int i = 0; i < 3; i++) {
		inject_peer_nodeinfo(PEER_REQUESTER_2, TEST_NODE_ID, 0x1031U + i, true);
	}
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 0U, "further requests inside the window must go unanswered");
}

/* ==========================================================================
 * The peer cache. Its size is an airtime knob in disguise: the suppression
 * stamps live in it, so evicting an entry re-opens both windows for that peer.
 * The `small_cache` scenario shrinks it to the Kconfig minimum (4) so the
 * boundary is reachable in a test rather than only on a busy channel.
 * ========================================================================== */

#if CONFIG_MESHTASTIC_NODEINFO_PEER_CACHE_SIZE <= 4
ZTEST(nodeinfo, test_eviction_reopens_the_suppression_window)
{
	struct captured c;

	inject_peer_text(PEER_EVICTED, 0x1040U, "hello");
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 1U, "precondition: first contact asks");

	inject_peer_text(PEER_EVICTED, 0x1041U, "still here");
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 0U, "precondition: suppressed while cached");

	/* Push the entry out: one more distinct peer than the cache holds, each
	 * heard after it, so it is the least recently seen. */
	for (int i = 0; i <= CONFIG_MESHTASTIC_NODEINFO_PEER_CACHE_SIZE; i++) {
		inject_peer_text(0x0C000000U + i, 0x1100U + i, "crowd");
		(void)drain_nodeinfo_tx(K_MSEC(100));
	}

	inject_peer_text(PEER_EVICTED, 0x1042U, "back again");
	c = drain_nodeinfo_tx(SETTLE);
	zassert_equal(c.count, 1U,
		      "an evicted peer is a stranger again — which is why the cache size is "
		      "an airtime decision, not just a memory one");
}
#endif /* small cache */

#if !defined(CONFIG_MESHTASTIC_NODEINFO_AUTO_SEND)
/*
 * AUTO_SEND=n is what fleet class 5 (the listener) needs: identify nobody,
 * announce nothing, but keep learning who is out there. The reactive paths
 * above still work in this build — that is the point of testing them here too
 * — and this asserts the unprompted half is genuinely gone.
 */
ZTEST(nodeinfo, test_auto_send_off_never_announces_unprompted)
{
	struct captured c;

	/* Well past the default start delay; nothing has asked us anything. */
	k_msleep(2000);
	c = drain_nodeinfo_tx(K_MSEC(100));

	zassert_equal(c.count, 0U, "AUTO_SEND=n must put no unprompted NodeInfo on the air");
}
#endif

#else /* NODEINFO_ANNOUNCE_SCENARIO */

/*
 * The unprompted announce, measured end to end in one test because the thread
 * runs once: it is created at meshtastic_init(), sleeps START_DELAY, announces,
 * then announces every INTERVAL. Splitting this across ztest cases would mean
 * each one starting in whatever phase the previous left.
 *
 * The start delay is not decoration. A node announces before it has heard
 * anything, so the delay is what stops it transmitting into a radio that has
 * not settled — and on a never-configured node it is also the last moment
 * before the first frame reaches the air (agents-xhli.21: rzr2 put a NodeInfo
 * onto public LongFast 33 s after boot, which is this delay plus the settle).
 */
ZTEST(nodeinfo, test_announce_waits_for_the_start_delay_then_repeats)
{
	struct captured c;
	int64_t delay_ms = (int64_t)CONFIG_MESHTASTIC_NODEINFO_START_DELAY_SEC * MSEC_PER_SEC;
	int64_t elapsed = k_uptime_get() - init_uptime_ms;

	zassert_true(elapsed < delay_ms,
		     "test started %lld ms after init, past the %lld ms delay — nothing to "
		     "observe",
		     elapsed, delay_ms);

	c = drain_nodeinfo_tx(K_NO_WAIT);
	zassert_equal(c.count, 0U, "nothing may go out before the start delay elapses");

	/* Past the delay, plus room for the send to reach the radio. */
	c = drain_nodeinfo_tx(K_MSEC((delay_ms - elapsed) + 1000));
	zassert_equal(c.count, 1U, "exactly one announce at the start delay");
	zassert_equal(c.to, MESHTASTIC_NODE_BROADCAST, "the announce is a broadcast");
	zassert_false(c.want_response,
		      "an announce that asked for a response would make every peer answer it");

	/* Nothing more until the interval. Checked at half of it, so the assertion
	 * is about the interval rather than about how long this test happens to
	 * sleep. */
	c = drain_nodeinfo_tx(K_SECONDS(CONFIG_MESHTASTIC_NODEINFO_INTERVAL_SEC / 2));
	zassert_equal(c.count, 0U, "an announce inside the interval is unbudgeted airtime");

	c = drain_nodeinfo_tx(K_SECONDS((CONFIG_MESHTASTIC_NODEINFO_INTERVAL_SEC / 2) + 2));
	zassert_equal(c.count, 1U, "and exactly one once the interval has elapsed");
}

#endif /* NODEINFO_ANNOUNCE_SCENARIO */

ZTEST_SUITE(nodeinfo, NULL, nodeinfo_setup, nodeinfo_before, NULL, NULL);
