/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the airtime rolling-window accounting (src/meshtastic_airtime.c),
 * specifically the lazy on-access expiry that replaced the former 1 Hz timer.
 *
 * The module derives its bucket index from k_uptime_get() and clears aged-out
 * buckets when accessed, so these tests drive time with k_sleep() (which
 * fast-forwards under native_sim) and assert the two observable outputs:
 * channel-utilization % (6 x 10 s = 60 s window) and TX-utilization %
 * (60 x 60 s = 1 h window). No meshtastic_init() is called, so nothing
 * transmits behind the tests' back — the accounting is exercised in isolation.
 *
 * Window math (see meshtastic_airtime.c):
 *   channel% = sum(channel_utilization) / (6 * 10 * 1000) * 100  = ms / 600  (per 1000 ms -> 1.667%)
 *   tx%      = sum(utilization_tx)      / (60 * 60 * 1000) * 100 = ms / 36000 (per 1000 ms -> 0.0278%)
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "meshtastic_airtime.h"

/* Tolerances: percentages are small; k_sleep/tick granularity adds a hair. */
#define CH_EPS 0.05f
#define TX_EPS 0.005f

static void airtime_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Reset the static accounting state before every test; the first
	 * access after init re-seeds the bucket ticks from current uptime.
	 */
	(void)meshtastic_airtime_init();
}

ZTEST(airtime, test_fresh_is_zero)
{
	zassert_within(meshtastic_airtime_channel_util_percent(), 0.0f, 0.0001f,
		       "fresh channel util must be 0");
	zassert_within(meshtastic_airtime_tx_util_percent(), 0.0f, 0.0001f,
		       "fresh tx util must be 0");
}

ZTEST(airtime, test_single_rx_percent)
{
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 1000U);

	zassert_within(meshtastic_airtime_channel_util_percent(), 1.6667f, CH_EPS,
		       "1000 ms RX in a 60 s window is ~1.667%%");
	/* RX must not touch the TX window. */
	zassert_within(meshtastic_airtime_tx_util_percent(), 0.0f, 0.0001f,
		       "RX must not accrue TX utilization");
}

ZTEST(airtime, test_tx_touches_both_windows)
{
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX, 1000U);

	/* TX accrues to the channel window (like any airtime) AND the TX window. */
	zassert_within(meshtastic_airtime_channel_util_percent(), 1.6667f, CH_EPS,
		       "TX also counts as channel airtime");
	zassert_within(meshtastic_airtime_tx_util_percent(), 0.02778f, TX_EPS,
		       "1000 ms TX in a 1 h window is ~0.0278%%");
}

ZTEST(airtime, test_accumulates_within_bucket)
{
	/* Two logs with no sleep land in the same 10 s bucket and sum. */
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 1000U);
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 500U);

	zassert_within(meshtastic_airtime_channel_util_percent(), 2.5f, CH_EPS,
		       "1500 ms in a 60 s window is 2.5%%");
}

ZTEST(airtime, test_channel_decays_after_window)
{
	/* Exercises airtime_advance()'s whole-window memset branch: a >60 s gap
	 * advances >= 6 channel buckets, so every bucket is cleared on the next
	 * read. 61 s guarantees the >=6 tick advance regardless of alignment.
	 */
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 1000U);
	zassert_within(meshtastic_airtime_channel_util_percent(), 1.6667f, CH_EPS,
		       "present before the window elapses");

	k_sleep(K_SECONDS(61));

	zassert_within(meshtastic_airtime_channel_util_percent(), 0.0f, 0.001f,
		       "channel utilization must decay to 0 after its 60 s window");
}

ZTEST(airtime, test_channel_rolling_window)
{
	/* Exercises the per-bucket clearing loop (not the memset branch) and the
	 * rolling nature: an old sample expires while a newer one in a different
	 * bucket survives.
	 *
	 * A at t0, B 35 s later (>= 3 buckets away, so a distinct bucket). At +35 s
	 * both are inside the 60 s window. At +65 s (A is 65 s old, B is 30 s old)
	 * A's bucket has been re-entered and cleared while B's has not. This holds
	 * for any bucket alignment: a sample's max age at expiry is 60 s (logged at
	 * a bucket boundary) and min is 50 s (logged at a bucket's end), so 65 s is
	 * always expired and 30 s is always alive.
	 */
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 1000U); /* A */
	k_sleep(K_SECONDS(35));
	meshtastic_airtime_log(MESHTASTIC_AIRTIME_RX, 2000U); /* B */

	zassert_within(meshtastic_airtime_channel_util_percent(), 5.0f, CH_EPS,
		       "A(1000)+B(2000) = 3000 ms -> 5.0%% while both are in-window");

	k_sleep(K_SECONDS(30));

	zassert_within(meshtastic_airtime_channel_util_percent(), 3.3333f, CH_EPS,
		       "A expired, B(2000) survives -> 3.333%%");
}

ZTEST_SUITE(airtime, NULL, NULL, airtime_before, NULL, NULL);
