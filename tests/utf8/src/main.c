/* SPDX-License-Identifier: GPL-3.0 */
/*
 * meshtastic_utf8_sanitize() against the reference's own test vectors
 * (firmware test/test_utf8/test_main.cpp, sanitizeUtf8 cases). Same inputs, same
 * expected outputs, same return values: this is what "does what upstream does"
 * means for the sanitizer the NodeDB and the relay share.
 *
 * Pure function: no stack, no radio, built without the Meshtastic module.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "meshtastic_utf8.h"

ZTEST(utf8, test_ascii_unchanged)
{
	char buf[32] = "Hello World";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "Hello World");
}

ZTEST(utf8, test_valid_2byte_unchanged)
{
	char buf[16] = "caf\xC3\xA9";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "caf\xC3\xA9");
}

ZTEST(utf8, test_valid_3byte_unchanged)
{
	char buf[8] = "\xE2\x82\xAC";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "\xE2\x82\xAC");
}

ZTEST(utf8, test_valid_4byte_emoji_unchanged)
{
	char buf[8] = "\xF0\x9F\x8C\x99";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "\xF0\x9F\x8C\x99");
}

ZTEST(utf8, test_valid_mixed_unchanged)
{
	char buf[16] = "Hi \xF0\x9F\x8C\x99!";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "Hi \xF0\x9F\x8C\x99!");
}

ZTEST(utf8, test_empty_string)
{
	char buf[4] = "";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "");
}

/* Invalid sequences observed in the wild (the reference's comments). */
ZTEST(utf8, test_truncated_4byte_at_end)
{
	char buf[32] = "Lunar Tower \xF0\x9F\x8C\x99\xF0\x9F\x97"
		       "4";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "Lunar Tower \xF0\x9F\x8C\x99???4");
}

ZTEST(utf8, test_lone_lead_bytes_without_continuations)
{
	char buf[32] = "Mesht\xE1\xF3tic 37e2";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "Mesht??tic 37e2");
}

ZTEST(utf8, test_bare_continuation_byte)
{
	char buf[8] = "\x80";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "?");
}

ZTEST(utf8, test_overlong_2byte)
{
	char buf[8] = "\xC0\xAF";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "??");
}

ZTEST(utf8, test_surrogate_half)
{
	char buf[8] = "\xED\xA0\x80";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "???");
}

ZTEST(utf8, test_5byte_sequence_rejected)
{
	char buf[8] = "\xF8\x80\x80\x80\x80";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "?????");
}

ZTEST(utf8, test_truncated_3byte_at_buffer_end)
{
	char buf[4] = { '\xE2', '\x82', '\0', '\0' };

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "??");
}

ZTEST(utf8, test_null_termination_enforced)
{
	char buf[5];

	memset(buf, 'A', sizeof(buf));
	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "AAAA");
}

ZTEST(utf8, test_null_buffer)
{
	zassert_false(meshtastic_utf8_sanitize(NULL, 10));
}

ZTEST(utf8, test_zero_size)
{
	char buf[4] = "Hi";

	zassert_false(meshtastic_utf8_sanitize(buf, 0));
	zassert_str_equal(buf, "Hi");
}

ZTEST(utf8, test_valid_max_codepoint)
{
	char buf[8] = "\xF4\x8F\xBF\xBF";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "\xF4\x8F\xBF\xBF");
}

ZTEST(utf8, test_above_max_codepoint)
{
	char buf[8] = "\xF4\x90\x80\x80";

	zassert_true(meshtastic_utf8_sanitize(buf, sizeof(buf)));
}

/* Ours, not the reference's: control characters pass through, CR included --
 * the reference sanitizer only repairs UTF-8. The relay relies on this. */
ZTEST(utf8, test_control_characters_untouched)
{
	char buf[16] = "a\r\nb\tc";

	zassert_false(meshtastic_utf8_sanitize(buf, sizeof(buf)));
	zassert_str_equal(buf, "a\r\nb\tc");
}

ZTEST_SUITE(utf8, NULL, NULL, NULL, NULL, NULL);
