/* SPDX-License-Identifier: GPL-3.0
 *
 * The composed BLE advertising name.
 *
 * The bug this guards: the name was one compile-time constant, so every node
 * built from this tree advertised the SAME thing and a phone was offered a list
 * of identical entries with nothing to choose between them (agents-xhli.15).
 * The property that matters is therefore not "the name is pretty" but "two
 * different nodes never advertise the same name".
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "meshtastic_ble_name.h"

ZTEST_SUITE(ble_name, NULL, NULL, NULL, NULL, NULL);

/* The ordinary case: the node's own short name is what a phone sees. */
ZTEST(ble_name, test_name_is_the_prefix_plus_the_short_name)
{
	char buf[24];
	size_t n = meshtastic_ble_adv_name_compose(buf, sizeof(buf), "rzr3", 0x04e14bb4U);

	zassert_str_equal(buf, "zeph-rzr3", "got \"%s\"", buf);
	zassert_equal(n, strlen(buf), "the returned length must match the string");
}

/*
 * THE ASSERTION THIS TEST EXISTS FOR. An unnamed node must still be unique.
 * Falling back to a shared constant would leave the original bug alive in
 * exactly the case that produced it — a bench of freshly flashed boards, none
 * of which anyone has named yet.
 */
ZTEST(ble_name, test_unnamed_nodes_do_not_collide)
{
	char a[24];
	char b[24];

	(void)meshtastic_ble_adv_name_compose(a, sizeof(a), NULL, 0x04e14bb4U);
	(void)meshtastic_ble_adv_name_compose(b, sizeof(b), "", 0x299de6c4U);

	zassert_str_equal(a, "zeph-e14bb4", "got \"%s\"", a);
	zassert_str_equal(b, "zeph-9de6c4", "got \"%s\"", b);
	zassert_true(strcmp(a, b) != 0,
		     "two unnamed nodes must not advertise the same name — that is the "
		     "whole defect this rule replaced");
}

/* An empty short name is treated as absent, not as a name of length zero:
 * "zeph-" alone would collide across every unnamed board. */
ZTEST(ble_name, test_empty_short_name_is_not_a_name)
{
	char buf[24];

	(void)meshtastic_ble_adv_name_compose(buf, sizeof(buf), "", 0x00000001U);
	zassert_str_equal(buf, "zeph-000001", "got \"%s\"", buf);
}

/* Truncation must not overrun, and must still terminate. The advert budget makes
 * this unreachable today (31-byte scan response, 4-character short name), which
 * is exactly why it is worth pinning before someone widens either. */
ZTEST(ble_name, test_truncates_rather_than_overruns)
{
	char buf[8];
	char guard[16];
	size_t n;

	memset(guard, 0x5a, sizeof(guard));
	n = meshtastic_ble_adv_name_compose(buf, sizeof(buf), "abcdefghijklmnop", 0U);

	zassert_true(n < sizeof(buf), "must not report more than it wrote");
	zassert_equal(buf[sizeof(buf) - 1U], '\0', "must always terminate");
	zassert_equal(strlen(buf), n, "the returned length must match the string");
	for (size_t i = 0; i < sizeof(guard); i++) {
		zassert_equal((unsigned char)guard[i], 0x5aU, "wrote past the buffer");
	}
}

/* A zero-length buffer is a caller error, not a crash. */
ZTEST(ble_name, test_degenerate_buffers_are_refused)
{
	char buf[4] = {'x', 'x', 'x', 'x'};

	zassert_equal(meshtastic_ble_adv_name_compose(NULL, 8U, "kit1", 1U), 0U);
	zassert_equal(meshtastic_ble_adv_name_compose(buf, 0U, "kit1", 1U), 0U);
	zassert_equal(buf[0], 'x', "a zero-length buffer must not be written");
}
