/* SPDX-License-Identifier: GPL-3.0
 *
 * The co-site desense model (H18 / attachment measurement R3), tested at the
 * driver level with two lora_sim instances and no mesh stack.
 *
 * What this models and why: a multi-preset supernode is DEFINED by "one radio
 * transmits while a co-sited radio listens on a different preset". The sim's
 * presets are ideally orthogonal (mesh_sim pins that), but the physics is not:
 * a +22 dBm transmitter 10-30 cm away delivers roughly -3..+10 dBm into the
 * neighbouring LNA — orders of magnitude past any SX126x blocking spec — so the
 * victim's front end is compressed for the aggressor's whole airtime, whatever
 * frequency either is on. These tests exercise the sim's model of exactly that
 * (lora_sim_set_cosite), including the duty-cycle loss accounting that the
 * eventual H18 bench measurement plugs into: hardware answers how much real
 * blanking a spacing/power produces; this machinery answers what a blanking
 * fraction COSTS.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <meshtastic/lora_sim.h>

static const struct device *const victim = DEVICE_DT_GET(DT_NODELABEL(lora_sim0));
static const struct device *const aggressor = DEVICE_DT_GET(DT_NODELABEL(lora_sim1));

/* ---- victim RX accounting -------------------------------------------------- */

static unsigned int rx_count;

static void rx_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi,
		  int8_t snr, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	ARG_UNUSED(user_data);
	rx_count++;
}

static const uint8_t payload[8] = {0xC0, 0x51, 0x7E, 0xDE, 0x5E, 0x45, 0xE0, 0x00};

static int inject_victim(void)
{
	return lora_sim_inject(victim, payload, sizeof(payload), -70, 8);
}

/* ShortTurbo-shaped modem config for the aggressor: what it is tuned to is
 * irrelevant to blanking (that is the point), but lora_sim_send() computes the
 * frame's airtime from it, so it must be valid. */
static void configure_aggressor(void)
{
	struct lora_modem_config cfg = {
		.frequency = 926750000,
		.bandwidth = BW_250_KHZ,
		.datarate = SF_7,
		.coding_rate = CR_4_5,
		.preamble_len = 16,
		.tx_power = 22,
		.tx = true,
	};

	zassert_ok(lora_config(aggressor, &cfg), "aggressor config");
}

static void cosite_before(void *fixture)
{
	ARG_UNUSED(fixture);
	lora_sim_reset(victim);
	lora_sim_reset(aggressor);
	rx_count = 0U;
	zassert_ok(lora_recv_async(victim, rx_cb, NULL), "arm victim RX");
}

/* Unpaired radios are the sim's long-standing ideal: however hard the
 * aggressor transmits, the victim hears everything. This pins today's
 * behaviour so the model is provably opt-in. */
ZTEST(cosite, test_unpaired_radios_do_not_interact)
{
	lora_sim_set_busy(aggressor, 500);
	zassert_ok(inject_victim(), "unpaired: delivered despite aggressor TX");
	zassert_equal(rx_count, 1U, "frame must reach the callback");
	zassert_equal(lora_sim_rx_blanked(victim), 0U, "nothing counted");
}

/* The core of the model: paired, the victim is deaf exactly while the
 * aggressor's channel is busy, and hears again the instant it is not. */
ZTEST(cosite, test_paired_victim_is_deaf_while_aggressor_transmits)
{
	zassert_ok(lora_sim_set_cosite(victim, aggressor));

	lora_sim_set_busy(aggressor, 200);
	zassert_equal(inject_victim(), -ECANCELED, "front end compressed");
	zassert_equal(rx_count, 0U, "the callback must never see a blanked frame");
	zassert_equal(lora_sim_rx_blanked(victim), 1U, "the loss is counted");

	k_msleep(201);
	zassert_ok(inject_victim(), "aggressor quiet: delivered");
	zassert_equal(rx_count, 1U, "");
	zassert_equal(lora_sim_rx_blanked(victim), 1U, "counter unchanged");
}

/* Pairing oneself is refused: half-duplex already models self-TX deafness
 * (the stack cancels RX while transmitting). */
ZTEST(cosite, test_self_pairing_is_refused)
{
	zassert_equal(lora_sim_set_cosite(victim, victim), -EINVAL, "");
}

/* Two radios in one enclosure are aggressors of EACH OTHER. The pairing is
 * directional, both directions may be set, and neither blanks the other while
 * idle — and setting both must not deadlock (the locks are never nested). */
ZTEST(cosite, test_mutual_pairing_is_directional_and_deadlock_free)
{
	unsigned int agg_rx = 0U;

	/* Give the aggressor an RX path of its own for the reverse direction. */
	zassert_ok(lora_recv_async(aggressor, rx_cb, NULL), "arm aggressor RX");
	zassert_ok(lora_sim_set_cosite(victim, aggressor));
	zassert_ok(lora_sim_set_cosite(aggressor, victim));

	/* Only lora_sim0's side transmits: lora_sim1-ward injects blank,
	 * lora_sim0-ward injects deliver. */
	lora_sim_set_busy(victim, 200);
	zassert_equal(lora_sim_inject(aggressor, payload, sizeof(payload), -70, 8),
		      -ECANCELED, "reverse direction blanks too");
	zassert_ok(inject_victim(), "victim's own aggressor is idle: delivered");
	agg_rx = rx_count;

	/* Both busy: both deaf. This is the case that would deadlock if the
	 * blanking check nested the two locks. */
	lora_sim_set_busy(victim, 200);
	lora_sim_set_busy(aggressor, 200);
	zassert_equal(inject_victim(), -ECANCELED, "");
	zassert_equal(lora_sim_inject(aggressor, payload, sizeof(payload), -70, 8),
		      -ECANCELED, "");

	k_msleep(201);
	zassert_ok(inject_victim(), "");
	zassert_ok(lora_sim_inject(aggressor, payload, sizeof(payload), -70, 8), "");
	zassert_equal(rx_count, agg_rx + 2U, "both sides hear again");
}

/* ---- a real transmission, not set_busy -------------------------------------- */

#define TX_LEN 200U
static uint8_t tx_buf[TX_LEN];
static K_THREAD_STACK_DEFINE(tx_stack, 2048);
static struct k_thread tx_thread;

static void tx_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	(void)lora_send(aggressor, tx_buf, TX_LEN);
}

/* The blanking window is the aggressor's real modelled airtime: a 200 B SF7
 * frame is mid-air when we inject 20 ms in, and gone once lora_send() returns
 * (it sleeps the airtime). */
ZTEST(cosite, test_a_real_transmission_blanks_for_its_airtime)
{
	struct lora_sim_frame f;
	k_tid_t tid;

	configure_aggressor();
	zassert_ok(lora_sim_set_cosite(victim, aggressor));

	tid = k_thread_create(&tx_thread, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack), tx_fn,
			      NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_msleep(20);
	zassert_equal(inject_victim(), -ECANCELED, "mid-frame: deaf");

	zassert_ok(k_thread_join(tid, K_SECONDS(5)), "TX thread");
	zassert_ok(lora_sim_take_tx(aggressor, &f, K_NO_WAIT), "the frame went out");
	zassert_true(f.air_ms > 20U, "airtime (%u ms) must cover the inject instant",
		     f.air_ms);

	zassert_ok(inject_victim(), "TX over: hearing again");
	zassert_equal(rx_count, 1U, "");
	zassert_equal(lora_sim_rx_blanked(victim), 1U, "");
}

/* ---- the loss-curve machinery ------------------------------------------------
 *
 * What the H18 bench number plugs into. The aggressor transmits with duty D
 * (busy B ms of every P ms); the victim's traffic arrives at instants spread
 * evenly across the period. Delivered fraction must equal the analytic
 * expectation exactly — the sim clock is virtual, so there is no jitter to
 * hide behind. 3 duties x 20 periods x 5 arrivals = 300 events through the
 * counters.
 */
ZTEST(cosite, test_duty_cycle_loss_curve)
{
	static const struct {
		uint32_t busy_ms;      /* of a 100 ms period */
		unsigned int expect_delivered; /* of the 5 arrivals at 10,30,50,70,90 */
	} duty[] = {
		{25U, 4U}, /* only t=10 falls inside the burst */
		{50U, 3U}, /* t=10,30 lost */
		{75U, 1U}, /* only t=90 survives */
	};
	const unsigned int periods = 20U;
	const unsigned int arrivals = 5U;

	zassert_ok(lora_sim_set_cosite(victim, aggressor));

	for (size_t d = 0; d < ARRAY_SIZE(duty); d++) {
		unsigned int delivered = 0U;
		uint32_t blanked_before = lora_sim_rx_blanked(victim);

		for (unsigned int p = 0; p < periods; p++) {
			/* Anchor every arrival to ABSOLUTE virtual time. k_msleep()
			 * rounds up to tick boundaries and the error accumulates —
			 * the first run of this test delivered 4/5 at 50% duty
			 * because arrivals had slid ~20 ms late. Virtual time only
			 * advances across sleeps, so t0 here equals the uptime
			 * set_busy() reads. */
			int64_t t0 = k_uptime_get();

			lora_sim_set_busy(aggressor, duty[d].busy_ms);
			for (unsigned int a = 0; a < arrivals; a++) {
				/* t = 10, 30, 50, 70, 90 into the period */
				k_sleep(K_TIMEOUT_ABS_MS(t0 + 10 + 20 * a));
				if (inject_victim() == 0) {
					delivered++;
				}
			}
			k_sleep(K_TIMEOUT_ABS_MS(t0 + 100)); /* the period boundary */
		}

		zassert_equal(delivered, duty[d].expect_delivered * periods,
			      "duty %u%%: delivered %u, expected %u", duty[d].busy_ms,
			      delivered, duty[d].expect_delivered * periods);
		zassert_equal(lora_sim_rx_blanked(victim) - blanked_before,
			      (arrivals - duty[d].expect_delivered) * periods,
			      "duty %u%%: blanked count must match", duty[d].busy_ms);
	}
}

ZTEST_SUITE(cosite, NULL, NULL, cosite_before, NULL, NULL);
