/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the capped exponential retry delay (src/meshtastic_backoff.c).
 */

#include <stdint.h>

#include <zephyr/ztest.h>

#include "meshtastic_backoff.h"

ZTEST_SUITE(backoff, NULL, NULL, NULL, NULL, NULL);

ZTEST(backoff, test_ladder_doubles_then_caps)
{
	struct meshtastic_backoff b;
	const uint32_t want[] = {500U, 1000U, 2000U, 4000U, 8000U, 10000U, 10000U};

	meshtastic_backoff_init(&b, 500U, 10000U);
	for (size_t i = 0; i < ARRAY_SIZE(want); i++) {
		zassert_equal(meshtastic_backoff_next_ms(&b, 0U), want[i], "step %zu", i);
	}
	zassert_equal(b.attempts, ARRAY_SIZE(want));
}

ZTEST(backoff, test_reset_restarts_from_base)
{
	struct meshtastic_backoff b;

	meshtastic_backoff_init(&b, 500U, 10000U);
	(void)meshtastic_backoff_next_ms(&b, 0U);
	(void)meshtastic_backoff_next_ms(&b, 0U);
	meshtastic_backoff_reset(&b);
	zassert_equal(meshtastic_backoff_next_ms(&b, 0U), 500U);
}

ZTEST(backoff, test_jitter_bounded_by_a_quarter)
{
	struct meshtastic_backoff b;

	meshtastic_backoff_init(&b, 1000U, 1000U);
	/* rand % (1000/4 + 1): the largest jitter is exactly a quarter. */
	zassert_equal(meshtastic_backoff_next_ms(&b, 250U), 1250U);
	zassert_equal(meshtastic_backoff_next_ms(&b, 251U), 1000U);
	zassert_equal(meshtastic_backoff_next_ms(&b, UINT32_MAX), 1000U + UINT32_MAX % 251U);
}

ZTEST(backoff, test_long_failure_never_overflows)
{
	struct meshtastic_backoff b;

	meshtastic_backoff_init(&b, 3000U, 0xF0000000U);
	b.attempts = UINT32_MAX - 1U;
	zassert_equal(meshtastic_backoff_next_ms(&b, 0U), 0xF0000000U);
	zassert_equal(b.attempts, UINT32_MAX);
	zassert_equal(meshtastic_backoff_next_ms(&b, 0U), 0xF0000000U);
	zassert_equal(b.attempts, UINT32_MAX, "saturates");
}

ZTEST(backoff, test_cap_below_base_is_base)
{
	struct meshtastic_backoff b;

	meshtastic_backoff_init(&b, 2000U, 100U);
	zassert_equal(meshtastic_backoff_next_ms(&b, 0U), 2000U);
	zassert_equal(meshtastic_backoff_next_ms(&b, 0U), 2000U);
}
