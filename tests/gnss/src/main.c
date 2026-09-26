/* SPDX-License-Identifier: GPL-3.0 */
/*
 * GNSS position-source tests (agents-isfr).
 *
 * meshtastic_gnss.c had ZERO coverage before this suite, and the cost was
 * concrete: CONFIG_MESHTASTIC_GNSS_AUTO_SEND=n -- a supported Kconfig bool --
 * did not compile, because no build had ever selected it. The listener image
 * (fleet class 5) was the first config to try, on 2026-08-29.
 *
 * The module is driven by Zephyr's emulated GNSS device (zephyr,gnss-emul with
 * CONFIG_GNSS_EMUL_MANUAL_UPDATE, so the test supplies every fix) and observed
 * two ways:
 *   - directly, through the position module's cache and the wall clock, and
 *   - indirectly, by counting Position frames on the simulated LoRa radio.
 * Nothing else in this image beacons (see prj.conf), so a captured frame is
 * unambiguously the GNSS send gate's doing.
 *
 * The suite is built in three configurations (testcase.yaml) and derives its
 * expectations from the Kconfig values rather than hardcoding them, so the same
 * body pins the gate under a send-dominated interval, a retry-dominated one,
 * and with automatic sending compiled out entirely.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/gnss_pps.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_gnss.h"
#include "meshtastic_packet.h"
#include "meshtastic_position.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID 0x0A0A0A0AU

/* The emulator publishes on its own cadence; this is its default fix rate. A
 * wait of a little over one interval therefore guarantees at least one publish. */
#define EMUL_FIX_INTERVAL_MS 1000

/* 2026-08-29 12:00:00 UTC — comfortably inside the clock helper's sane window
 * [2020, ~2060], so a rejection in these tests is never the window's doing. */
#define FIX_EPOCH_SEC 1787054400LL

/* 2010-06-01 00:00:00 UTC — below MESHTASTIC_EPOCH_MIN, which is what an
 * unfixed or rolled-over receiver looks like. */
#define STALE_EPOCH_SEC 1275350400LL

/* The two windows the send gate ANDs together. A send needs both elapsed, so
 * the effective spacing is the larger of the pair — which is the interaction
 * the retry_gate configuration exists to pin. */
#define SEND_INTERVAL_MS ((int64_t)CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC * MSEC_PER_SEC)
#define RETRY_INTERVAL_MS ((int64_t)CONFIG_MESHTASTIC_GNSS_RETRY_INTERVAL_SEC * MSEC_PER_SEC)
#define GATE_MS MAX(SEND_INTERVAL_MS, RETRY_INTERVAL_MS)

/* The board's receiver power switch, observed (agents-t2hb.2). A strong
 * definition here replaces the weak no-op default in meshtastic_gnss.c, the
 * same way the Heltec V4 board file does with its EN line. */
static uint32_t board_power_calls;
static bool board_power_last = true;

int meshtastic_gnss_board_power(bool on)
{
	board_power_calls++;
	board_power_last = on;
	return 0;
}

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const struct device *const gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));

/* A second subscriber on the same publish, so a test can wait for the driver to
 * have delivered a fix rather than sleeping and hoping. It says nothing about
 * whether the module under test has finished reacting — that is what the TX
 * capture timeout and the position cache are for. */
static K_SEM_DEFINE(fix_published, 0, 1);

static void test_gnss_cb(const struct device *dev, const struct gnss_data *data)
{
	ARG_UNUSED(dev);

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		k_sem_give(&fix_published);
	}
}
GNSS_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(gnss)), test_gnss_cb);

/* A plausible fix: 53.4808 N, 2.2426 W, 120 m AMSL, 4.2 m/s on a bearing of
 * 123.4 deg, HDOP 1.234, 9 satellites, 48.5 m geoid separation. Every field is
 * a different unit from the one Meshtastic wants, which is the point. */
static void canonical_fix(struct navigation_data *nav, struct gnss_info *info)
{
	*nav = (struct navigation_data){
		.latitude = 53480800000LL,   /* nanodegrees */
		.longitude = -2242600000LL,  /* nanodegrees */
		.bearing = 123400U,          /* millidegrees */
		.speed = 4200U,              /* mm/s */
		.altitude = 120000,          /* mm */
	};
	*info = (struct gnss_info){
		.satellites_cnt = 9U,
		.hdop = 1234U,             /* 1/1000 */
		.geoid_separation = 48500, /* mm */
		.fix_status = GNSS_FIX_STATUS_GNSS_FIX,
		.fix_quality = GNSS_FIX_QUALITY_GNSS_SPS,
	};
}

/*
 * Hand the emulator a fix dated @p epoch_sec and block until it has been
 * published.
 *
 * The emulator derives UTC from `boot_realtime_ms + <next scheduled fix>`, so
 * anchoring boot at (target - now) lands the published time within one fix
 * interval of the target rather than exactly on it. Every assertion below
 * allows for that; pinning it tighter would be pinning the emulator, not us.
 */
static void publish_fix_at(int64_t epoch_sec, const struct navigation_data *nav,
			   const struct gnss_info *info)
{
	k_sem_reset(&fix_published);
	gnss_emul_set_data(gnss_dev, nav, info, (epoch_sec * MSEC_PER_SEC) - k_uptime_get());
	zassert_ok(k_sem_take(&fix_published, K_MSEC(4 * EMUL_FIX_INTERVAL_MS)),
		   "the emulator never published the fix");
}

static void publish_canonical_fix(void)
{
	struct navigation_data nav;
	struct gnss_info info;

	canonical_fix(&nav, &info);
	publish_fix_at(FIX_EPOCH_SEC, &nav, &info);
}

/* Frames captured since the last lora_sim_reset(), each asserted to be a
 * Position broadcast originated by this node. Anything else in the queue is a
 * bug in the test's isolation, not a detail to skip past. */
static uint32_t drain_position_tx(k_timeout_t first_wait)
{
	struct lora_sim_frame f;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint32_t n = 0U;

	while (lora_sim_take_tx(lora_dev, &f, n == 0U ? first_wait : K_NO_WAIT) == 0) {
		zassert_ok(meshtastic_decode_wire_packet(f.data, f.len, 0, 0, &decoded, payload,
							 sizeof(payload)),
			   "captured frame did not decode");
		zassert_equal(decoded.from, TEST_NODE_ID, "frame did not originate here");
		zassert_equal(decoded.portnum, MESHTASTIC_PORT_POSITION,
			      "only Position frames should be on the air in this image");
		n++;
	}
	return n;
}

/* Read-modify-write of the smart-broadcast trio (agents-t2hb.3). */
static void set_smart(bool enabled, uint32_t distance_m, uint32_t min_interval_s)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg));
	cfg.payload_variant.position.position_broadcast_smart_enabled = enabled;
	cfg.payload_variant.position.broadcast_smart_minimum_distance = distance_m;
	cfg.payload_variant.position.broadcast_smart_minimum_interval_secs = min_interval_s;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
}

static void *gnss_setup(void)
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
	zassert_true(device_is_ready(gnss_dev), "gnss emul device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");

	/* Pin the contention window off: this suite times sends against a gate
	 * measured in seconds, and a random pre-TX backoff only adds jitter to the
	 * one number under test. (mesh_sim does the same, for the same reason.) */
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	return NULL;
}

static void gnss_before(void *fixture)
{
	int stable = 0;

	ARG_UNUSED(fixture);

	/* The gps_mode and interval tests leave the section changed; every test
	 * starts from a receiver that is on and the compiled interval. */
	zassert_ok(meshtastic_position_set_gps_mode(
		meshtastic_Config_PositionConfig_GpsMode_ENABLED));
	zassert_ok(meshtastic_position_set_broadcast_secs(0U));
	set_smart(true, 100U, 300U);

	/* Stop the emulator republishing the previous test's fix: cleared data
	 * reads as NO_FIX, which the module ignores. Do this FIRST, so the quiesce
	 * below has a bounded amount of work to wait out. */
	gnss_emul_clear_data(gnss_dev);

	/*
	 * Quiesce before clearing the capture queue.
	 *
	 * A Position the previous test triggered is not on the air when the module
	 * decides to send it: it crosses a workqueue, the outbound queue and the
	 * contention window first, which was measured at ~70 ms here. Reset the
	 * capture queue any earlier and that frame lands afterwards, where it is
	 * indistinguishable from one this test caused — which is exactly how the
	 * gate test first "failed".
	 *
	 * So: sleep past the observed latency, then wait for the capture queue to
	 * stop growing, and only then clear it.
	 */
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
	meshtastic_clock_test_reset();
	meshtastic_gnss_test_reset();
	meshtastic_position_test_reset(); /* the last-broadcast stamp lives there now */
	k_sem_reset(&fix_published);
}

/* ==========================================================================
 * The clock hand-off. GNSS is the only self-sustaining time source on a fleet
 * with no WiFi, and it sits at the top of the quality ladder, so what it does
 * to the clock is load-bearing for every epoch-stamped field.
 * ========================================================================== */

ZTEST(gnss, test_fix_seeds_the_wall_clock_at_gps_quality)
{
	zassert_false(meshtastic_clock_valid(), "precondition: clock starts unset");

	publish_canonical_fix();

	zassert_true(meshtastic_clock_valid(), "a valid fix must seed the wall clock");
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_GPS,
		      "our own GPS UTC is the top of the source ladder");

	/* Within one emulator fix interval of the target — see publish_fix_at(). */
	int64_t skew = (int64_t)meshtastic_clock_now_epoch() - FIX_EPOCH_SEC;

	zassert_true(skew >= 0 && skew <= 2, "clock seeded %lld s from the fix", skew);
}

/* The sanity floor exists because an unfixed or week-number-rolled receiver
 * presents a confident, wrong date. Accepting it would rewrite every
 * epoch-stamped field on the node. */
ZTEST(gnss, test_pre_2020_fix_does_not_seed_the_clock)
{
	struct navigation_data nav;
	struct gnss_info info;

	canonical_fix(&nav, &info);
	publish_fix_at(STALE_EPOCH_SEC, &nav, &info);

	zassert_false(meshtastic_clock_valid(),
		      "an epoch below MESHTASTIC_EPOCH_MIN must be refused");
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_NONE);
}

/* A published record with no fix carries a UTC the driver made up from its own
 * zeroed state; treating it as time (or as a position) is the failure this
 * guards. */
ZTEST(gnss, test_no_fix_record_is_ignored_entirely)
{
	struct navigation_data nav;
	struct gnss_info info;

	canonical_fix(&nav, &info);
	info.fix_status = GNSS_FIX_STATUS_NO_FIX;
	info.fix_quality = GNSS_FIX_QUALITY_INVALID;

	gnss_emul_set_data(gnss_dev, &nav, &info, (FIX_EPOCH_SEC * MSEC_PER_SEC) - k_uptime_get());
	k_msleep(3 * EMUL_FIX_INTERVAL_MS);

	zassert_false(meshtastic_clock_valid(), "a NO_FIX record must not seed the clock");
	zassert_equal(drain_position_tx(K_NO_WAIT), 0U, "a NO_FIX record must not transmit");
}

/* ==========================================================================
 * The unit conversions. Every field arrives in a different unit from the one
 * Meshtastic wants, and each divisor is a silent-wrong-answer waiting to
 * happen: an off-by-a-factor-of-ten latitude still looks like a coordinate.
 * ========================================================================== */

ZTEST(gnss, test_fix_populates_the_advertised_position)
{
	meshtastic_Position pos;

	publish_canonical_fix();
	zassert_ok(meshtastic_position_get_current(&pos), "no position cached after a fix");

	/* nanodegrees -> 1e-7 degrees */
	zassert_true(pos.has_latitude_i);
	zassert_equal(pos.latitude_i, 534808000, "latitude_i wrong");
	zassert_true(pos.has_longitude_i);
	zassert_equal(pos.longitude_i, -22426000, "longitude_i wrong");

	/* millimetres -> metres */
	zassert_true(pos.has_altitude);
	zassert_equal(pos.altitude, 120, "altitude wrong");
	zassert_true(pos.has_altitude_geoidal_separation);
	zassert_equal(pos.altitude_geoidal_separation, 48, "geoid separation wrong");

	/* 1/1000 -> 1/100 */
	zassert_equal(pos.HDOP, 123U, "HDOP wrong");
	/* millidegrees -> centidegrees */
	zassert_true(pos.has_ground_track);
	zassert_equal(pos.ground_track, 12340U, "ground_track wrong");
	/* mm/s -> m/s */
	zassert_true(pos.has_ground_speed);
	zassert_equal(pos.ground_speed, 4U, "ground_speed wrong");

	zassert_equal(pos.sats_in_view, 9U);
	zassert_equal(pos.fix_quality, (uint32_t)GNSS_FIX_QUALITY_GNSS_SPS);
	/* A plain GNSS fix is type 2 (2D/3D); only a differential fix is 3. */
	zassert_equal(pos.fix_type, 2U, "fix_type wrong for a plain GNSS fix");
	zassert_equal(pos.location_source, meshtastic_Position_LocSource_LOC_INTERNAL);
	zassert_equal(pos.altitude_source, meshtastic_Position_AltSource_ALT_INTERNAL);
	zassert_equal(pos.next_update, CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC);
	/* The cached position is full resolution; masking to the channel's sharing
	 * precision happens on the way out (meshtastic_position_sanitise_tx). */
	zassert_equal(pos.precision_bits, 32U);
	/* The clock was seeded from this same fix, so the stamp is real. */
	zassert_true(pos.time >= FIX_EPOCH_SEC, "position not stamped with the fix time");
}

ZTEST(gnss, test_differential_fix_reports_fix_type_3)
{
	struct navigation_data nav;
	struct gnss_info info;
	meshtastic_Position pos;

	canonical_fix(&nav, &info);
	info.fix_status = GNSS_FIX_STATUS_DGNSS_FIX;
	info.fix_quality = GNSS_FIX_QUALITY_DGNSS;
	publish_fix_at(FIX_EPOCH_SEC, &nav, &info);

	zassert_ok(meshtastic_position_get_current(&pos));
	zassert_equal(pos.fix_type, 3U, "a differential fix is type 3");
}

/* Zero speed and zero bearing are omitted rather than sent as zero. A stationary
 * node reporting "heading 0 deg" is a different claim from reporting nothing,
 * and the optional-field encoding is how the wire says which. */
ZTEST(gnss, test_stationary_fix_omits_speed_and_track)
{
	struct navigation_data nav;
	struct gnss_info info;
	meshtastic_Position pos;

	canonical_fix(&nav, &info);
	nav.speed = 0U;
	nav.bearing = 0U;
	publish_fix_at(FIX_EPOCH_SEC, &nav, &info);

	zassert_ok(meshtastic_position_get_current(&pos));
	zassert_false(pos.has_ground_speed, "zero speed must be absent, not zero");
	zassert_false(pos.has_ground_track, "zero bearing must be absent, not zero");
}

/* ==========================================================================
 * The send gate. Airtime policy: every automatic Position is airtime spent on
 * a shared channel, so how often one goes out is not an implementation detail.
 * ========================================================================== */

#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)

/* Both stamps start one interval in the past (gnss_gate_reset), so a node that
 * has just booted and acquired does not sit on the fix for an interval first. */
ZTEST(gnss, test_first_fix_after_boot_broadcasts_immediately)
{
	publish_canonical_fix();

	zassert_equal(drain_position_tx(K_MSEC(2000)), 1U,
		      "the first fix should produce exactly one Position broadcast");
}

/*
 * The gate ANDs two windows, so the real spacing is the LARGER of
 * SEND_INTERVAL and RETRY_INTERVAL — including between two SUCCESSFUL sends,
 * even though RETRY_INTERVAL is documented as a post-failure backoff. With the
 * shipped defaults (300 s send, 60 s retry) the retry window never binds and
 * the distinction is invisible; the retry_gate configuration inverts them so it
 * is not. Pinned here as the behaviour, not endorsed as the design: if the
 * intent is ever narrowed to failures only, this test is the one to change,
 * deliberately.
 */
ZTEST(gnss, test_further_fixes_are_gated_to_the_wider_window)
{
	publish_canonical_fix();
	zassert_equal(drain_position_tx(K_MSEC(2000)), 1U, "precondition: first fix sent");

	/* Fixes keep arriving on the emulator's 1 s cadence throughout both waits;
	 * the module sees every one of them and must still hold its tongue. */
	k_msleep(GATE_MS - (2 * EMUL_FIX_INTERVAL_MS));
	zassert_equal(drain_position_tx(K_NO_WAIT), 0U,
		      "a send inside the gate window is airtime we did not budget");

	k_msleep(3 * EMUL_FIX_INTERVAL_MS);
	zassert_true(drain_position_tx(K_NO_WAIT) >= 1U,
		     "the gate must reopen once the window has elapsed");
}

#else /* !CONFIG_MESHTASTIC_GNSS_AUTO_SEND */

/*
 * THE CONFIGURATION THAT DID NOT COMPILE (agents-isfr).
 *
 * This scenario's real assertion is that the image builds at all: gnss_send_wq
 * is declared under `#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)` and was
 * once referenced from inside IS_ENABLED(), which keeps both arms compiled by
 * design. The runtime half below is what the listener role actually depends on:
 * a node that never transmits must still discipline its clock and know where it
 * is, because clock quality is a listener's whole product.
 */
ZTEST(gnss, test_no_auto_send_still_tracks_position_and_clock)
{
	meshtastic_Position pos;

	publish_canonical_fix();

	zassert_true(meshtastic_clock_valid(), "a listener still disciplines its clock");
	zassert_equal(meshtastic_clock_get_quality(), MESHTASTIC_CLOCK_QUALITY_GPS);
	zassert_ok(meshtastic_position_get_current(&pos), "a listener still knows where it is");
	zassert_equal(pos.latitude_i, 534808000);

	/* Several more fixes, none of which may reach the radio. */
	k_msleep(4 * EMUL_FIX_INTERVAL_MS);
	zassert_equal(drain_position_tx(K_NO_WAIT), 0U,
		      "AUTO_SEND=n must put nothing on the air");
}

#endif /* CONFIG_MESHTASTIC_GNSS_AUTO_SEND */

ZTEST_SUITE(gnss, NULL, gnss_setup, gnss_before, NULL, NULL);

#if defined(CONFIG_MESHTASTIC_GNSS_PPS)
/* ---- 1PPS: the guards that stop a one-second error ------------------------
 *
 * The clock anchors on the most recent PPS edge rather than on the NMEA
 * sentence's arrival, because the sentence is ~853 ms late and one-sided. That
 * pairing — "the sentence naming second N belongs to the edge just before it" —
 * is sound only while delivery takes under a second. Miss an edge and the most
 * recent one belongs to N+1, making the clock exactly one second fast: silent,
 * plausible, and the worst possible size of error.
 *
 * So the accessor refuses rather than guesses, on two independent conditions,
 * and both are tested here by driving the emulated pin directly.
 */
#include <zephyr/drivers/gpio/gpio_emul.h>

#define PPS_EMUL_NODE DT_NODELABEL(gnss_pps)
static const struct gpio_dt_spec pps_test_pin = GPIO_DT_SPEC_GET(PPS_EMUL_NODE, pps_gpios);

static void pps_edge(void)
{
	/* Low then high: the capture triggers on the rising edge. */
	(void)gpio_emul_input_set(pps_test_pin.port, pps_test_pin.pin, 0);
	(void)gpio_emul_input_set(pps_test_pin.port, pps_test_pin.pin, 1);
}

ZTEST(gnss, test_pps_a_unlocked_train_is_refused)
{
	int64_t edge, age;

	zassert_ok(meshtastic_gnss_pps_start(false), "arming the capture failed");

	/* One edge is not a pulse train. Nothing has established a rate yet, so
	 * there is no basis for calling this a second boundary. */
	pps_edge();
	zassert_false(meshtastic_gnss_pps_last_edge(&edge, &age),
		      "a single edge must not be usable — no interval has been measured");

	/* Two edges a tenth of a second apart are a rate, just not 1 Hz. A floating
	 * pin picking up noise looks exactly like this, and noise timestamped as a
	 * second boundary is worse than having no reference at all. */
	k_msleep(100);
	pps_edge();
	zassert_false(meshtastic_gnss_pps_last_edge(&edge, &age),
		      "a 10 Hz train must not be accepted as 1PPS");
}

ZTEST(gnss, test_pps_b_locked_train_is_usable_and_dated)
{
	int64_t edge, age, before;

	zassert_ok(meshtastic_gnss_pps_start(false), "arming the capture failed");

	pps_edge();
	k_msleep(1000);
	before = k_uptime_get();
	pps_edge();

	zassert_true(meshtastic_gnss_pps_last_edge(&edge, &age),
		     "two edges one second apart must lock and be usable");
	zassert_within(edge, before, 50,
		       "the edge must be stamped when it happened, not when it was read");
	zassert_true(age >= 0 && age < 1000, "a fresh edge's age must be inside a second, got %lld",
		     age);
}

ZTEST(gnss, test_pps_c_a_stale_edge_is_refused)
{
	int64_t edge, age;

	zassert_ok(meshtastic_gnss_pps_start(false), "arming the capture failed");

	pps_edge();
	k_msleep(1000);
	pps_edge();
	zassert_true(meshtastic_gnss_pps_last_edge(&edge, &age), "precondition: locked");

	/* Let the edge age past a second WITHOUT another arriving — a receiver that
	 * lost its fix and stopped pulsing. The last edge is still perfectly valid
	 * data; it is just no longer the second boundary anyone is asking about. */
	k_msleep(1100);
	zassert_false(meshtastic_gnss_pps_last_edge(&edge, &age),
		      "an edge older than a second must be refused: pairing a sentence "
		      "with it would put the clock out by exactly one second");
}

ZTEST(gnss, test_pps_d_disarmed_capture_is_refused)
{
	int64_t edge, age;

	zassert_ok(meshtastic_gnss_pps_start(false), "arming the capture failed");
	pps_edge();
	k_msleep(1000);
	pps_edge();
	zassert_true(meshtastic_gnss_pps_last_edge(&edge, &age), "precondition: locked");

	zassert_ok(meshtastic_gnss_pps_stop(), "stop failed");
	zassert_false(meshtastic_gnss_pps_last_edge(&edge, &age),
		      "a disarmed capture has nothing to say, however good its last edge was");
}
#endif /* CONFIG_MESHTASTIC_GNSS_PPS */

/* ==========================================================================
 * PositionConfig (agents-t2hb.2): gps_mode switches the receiver, and
 * position_broadcast_secs governs the fix-driven gate. Before this both were
 * stored, streamed to the phone, and read by nothing.
 * ========================================================================== */

ZTEST(gnss, test_gps_mode_disabled_powers_off_and_stops_fixes)
{
	struct meshtastic_gnss_status st;
	struct navigation_data nav;
	struct gnss_info info;
	meshtastic_Position pos;

	publish_canonical_fix();
	zassert_ok(meshtastic_position_get_current(&pos), "precondition: a fix is held");
	(void)drain_position_tx(K_MSEC(2000));

	board_power_calls = 0U;
	zassert_ok(meshtastic_position_set_gps_mode(
		meshtastic_Config_PositionConfig_GpsMode_DISABLED));
	zassert_equal(board_power_calls, 1U, "the write must reach the board's power line");
	zassert_false(board_power_last, "... and turn it OFF");
	zassert_ok(meshtastic_gnss_status_get(&st));
	zassert_false(st.enabled, "status reports the receiver off");
	zassert_equal(meshtastic_position_get_current(&pos), -ENODATA,
		      "the last fix is forgotten, not advertised on after the switch-off");

	/* The receiver is suspended: data handed to it is not published at all. */
	canonical_fix(&nav, &info);
	k_sem_reset(&fix_published);
	gnss_emul_set_data(gnss_dev, &nav, &info, (FIX_EPOCH_SEC * MSEC_PER_SEC) - k_uptime_get());
	zassert_not_equal(k_sem_take(&fix_published, K_MSEC(3 * EMUL_FIX_INTERVAL_MS)), 0,
			  "a suspended receiver publishes nothing");
	zassert_equal(meshtastic_position_get_current(&pos), -ENODATA, "no position");
	zassert_equal(drain_position_tx(K_NO_WAIT), 0U, "and nothing on the air");

	/* Idempotent: re-writing the same mode does not touch the hardware. */
	zassert_ok(meshtastic_position_set_gps_mode(
		meshtastic_Config_PositionConfig_GpsMode_DISABLED));
	zassert_equal(board_power_calls, 1U, "an unchanged mode is not re-applied");

	/* And back: power, resume, fixes flow again. */
	zassert_ok(meshtastic_position_set_gps_mode(
		meshtastic_Config_PositionConfig_GpsMode_ENABLED));
	zassert_equal(board_power_calls, 2U, "");
	zassert_true(board_power_last, "ENABLED powers the receiver");
	zassert_ok(k_sem_take(&fix_published, K_SECONDS(15)),
		   "a resumed receiver publishes again");
	k_msleep(50);
	zassert_ok(meshtastic_position_get_current(&pos), "and the module takes the fix");
}

/* NOT_PRESENT is off too (the reference never starts the GPS thread for it). */
ZTEST(gnss, test_gps_mode_not_present_is_off)
{
	struct meshtastic_gnss_status st;

	board_power_calls = 0U;
	zassert_ok(meshtastic_position_set_gps_mode(
		meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT));
	zassert_equal(board_power_calls, 1U, "");
	zassert_false(board_power_last, "");
	zassert_ok(meshtastic_gnss_status_get(&st));
	zassert_false(st.enabled, "");
}

/* A stored position_broadcast_secs beats the compiled default (5 s in this
 * image) at the very next gate decision, with no reboot. */
ZTEST(gnss, test_broadcast_secs_governs_the_gate)
{
#if !defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
	ztest_test_skip();
#else
	const int64_t stored_ms = 15 * MSEC_PER_SEC;
	const int64_t gate_ms = MAX(stored_ms, RETRY_INTERVAL_MS);
	meshtastic_Position pos;

	zassert_ok(meshtastic_position_set_broadcast_secs(15U));

	publish_canonical_fix();
	zassert_equal(drain_position_tx(K_MSEC(2000)), 1U, "precondition: first fix sent");
	zassert_ok(meshtastic_position_get_current(&pos), "");
	zassert_equal(pos.next_update, 15U, "the advertised next_update follows it too");

	k_msleep(gate_ms - (2 * EMUL_FIX_INTERVAL_MS));
	zassert_equal(drain_position_tx(K_NO_WAIT), 0U,
		      "the compiled 5 s would have sent by now; the stored 15 s must not");

	k_msleep(3 * EMUL_FIX_INTERVAL_MS);
	zassert_true(drain_position_tx(K_NO_WAIT) >= 1U, "and it reopens on the stored interval");
#endif
}

/* A fix that moved triggers a smart broadcast from the fix path itself -- even
 * with position_broadcast_smart_enabled OFF, because the reference's
 * handleNewPosition() (called by GPS.cpp for every published fix) never looks at
 * that flag (agents-t2hb.3). The interval is 60 s, so a send well before that is
 * the smart path's doing. */
ZTEST(gnss, test_a_moved_fix_broadcasts_early_even_with_smart_off)
{
#if !defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
	ztest_test_skip();
#else
	struct navigation_data nav;
	struct gnss_info info;
	int64_t start;
	uint32_t n = 0U;

	zassert_ok(meshtastic_position_set_broadcast_secs(60U));
	set_smart(false, 100U, 5U);

	publish_canonical_fix();
	zassert_equal(drain_position_tx(K_MSEC(2000)), 1U, "precondition: first fix sent");
	start = k_uptime_get();

	/* 0.5 deg north: many 13-bit cells away. Fixes keep coming at 1 Hz. */
	canonical_fix(&nav, &info);
	nav.latitude += 500000000LL;
	publish_fix_at(FIX_EPOCH_SEC, &nav, &info);

	while (n == 0U && k_uptime_get() - start < 40 * MSEC_PER_SEC) {
		n = drain_position_tx(K_MSEC(500));
	}
	zassert_true(n >= 1U, "a moved fix must go out before the 60 s interval");
	zassert_true(k_uptime_get() - start >= 4 * MSEC_PER_SEC,
		     "but not inside the 5 s smart minimum");
#endif
}
