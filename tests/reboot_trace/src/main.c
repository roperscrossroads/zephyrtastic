/* SPDX-License-Identifier: GPL-3.0
 *
 * The reboot record: does it say who asked, and does it refuse to invent an answer?
 *
 * Background, because it explains what these assertions are protecting. On 2026-09-04 two
 * nodes restarted within a second of each other reporting only "cause 0x00000002 SOFTWARE" --
 * a deliberate reboot from firmware -- and nothing in the tree could name which of several
 * paths had done it. Establishing "not the watchdog, not the fatal handler, not admin, not
 * the DFU guard, not the supervisor" took reading every call site by hand and still ended in
 * "unaccounted" (agents-u0h6). These tests pin the behaviour that makes that answerable.
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/reboot_trace.h>

static void reset_state(void *unused)
{
	ARG_UNUSED(unused);
	meshtastic_reboot_trace_clear();
}

ZTEST_SUITE(meshtastic_reboot_trace, NULL, NULL, reset_state, NULL, NULL);

ZTEST(meshtastic_reboot_trace, test_nothing_is_reported_before_any_reboot)
{
	struct meshtastic_reboot_trace rt;

	zassert_false(meshtastic_reboot_trace_get(&rt),
		      "a fresh node must report NO reboot record; reporting one would put an "
		      "imaginary reboot in front of whoever is diagnosing a real problem");
}

ZTEST(meshtastic_reboot_trace, test_an_unnoted_reboot_is_reported_as_unknown_not_guessed)
{
	struct meshtastic_reboot_trace rt;

	/* The case that matters most: something rebooted the node without going through a path
	 * we instrumented -- the Zephyr shell's `kernel reboot`, MCUmgr, a library. It must be
	 * reported as UNKNOWN with a caller address to chase, never attributed to whichever
	 * reason happens to sit at zero. */
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0x40012345U, true);

	zassert_true(meshtastic_reboot_trace_get(&rt), "capture must leave a readable record");
	zassert_equal(rt.reason, MESHTASTIC_REBOOT_UNKNOWN, "an unnoted reboot must be UNKNOWN");
	zassert_true(rt.have_pc, "the caller address is the only lead an unnoted reboot leaves");
	zassert_equal(rt.caller_pc, 0x40012345U);
	zassert_equal(rt.detail[0], '\0', "no note means no detail, not stale text");
}

ZTEST(meshtastic_reboot_trace, test_a_noted_reboot_carries_its_reason_and_detail)
{
	struct meshtastic_reboot_trace rt;

	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_WATCHDOG, "radio");
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0x1000U, true);

	zassert_true(meshtastic_reboot_trace_get(&rt));
	zassert_equal(rt.reason, MESHTASTIC_REBOOT_WATCHDOG);
	/* The channel name IS the diagnosis for a watchdog reboot -- "a thread stopped feeding
	 * its channel" is only useful once you know which channel. */
	zassert_str_equal(rt.detail, "radio");
}

ZTEST(meshtastic_reboot_trace, test_a_note_belongs_to_exactly_one_reboot)
{
	struct meshtastic_reboot_trace rt;

	/* A note left before a reboot that then does not happen -- an admin request that was
	 * cancelled, a path that returned -- must not attach itself to whatever reboots the
	 * node next. That would be worse than no attribution: it would be confident and wrong,
	 * and it would send the next investigation at the wrong subsystem. */
	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_ADMIN, "first");
	meshtastic_reboot_trace_capture(SYS_REBOOT_COLD, 0x2000U, true);
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0x3000U, true);

	zassert_true(meshtastic_reboot_trace_get(&rt));
	zassert_equal(rt.reason, MESHTASTIC_REBOOT_UNKNOWN,
		      "the second reboot had no note of its own and must not inherit the first's");
	zassert_equal(rt.detail[0], '\0', "nor its detail");
	zassert_equal(rt.caller_pc, 0x3000U, "the record must describe the LATEST reboot");
}

ZTEST(meshtastic_reboot_trace, test_the_record_describes_the_latest_reboot)
{
	struct meshtastic_reboot_trace rt;

	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_ADMIN, NULL);
	meshtastic_reboot_trace_capture(SYS_REBOOT_COLD, 0xAAAAU, true);
	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_FATAL, NULL);
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0xBBBBU, true);

	zassert_true(meshtastic_reboot_trace_get(&rt));
	zassert_equal(rt.reason, MESHTASTIC_REBOOT_FATAL);
	zassert_equal(rt.caller_pc, 0xBBBBU);
}

ZTEST(meshtastic_reboot_trace, test_missing_caller_address_is_flagged_not_zero)
{
	struct meshtastic_reboot_trace rt;

	/* A zero address would render as a plausible-looking 0x00000000 and send someone to
	 * addr2line it. "We did not get one" and "it was zero" must not look alike -- the same
	 * rule the boot log applies to an unavailable reset cause. */
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0U, false);

	zassert_true(meshtastic_reboot_trace_get(&rt));
	zassert_false(rt.have_pc, "an absent caller address must be flagged absent");
}

ZTEST(meshtastic_reboot_trace, test_clear_forgets_it)
{
	struct meshtastic_reboot_trace rt;

	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0x1234U, true);
	zassert_true(meshtastic_reboot_trace_get(&rt));

	/* The boot log clears this when retained RAM did not survive: after a power cycle the
	 * bytes are whatever the RAM happened to hold, and reporting them as this node's last
	 * reboot would be a diagnostic lying at exactly the moment someone is chasing a node
	 * that died in the field. */
	meshtastic_reboot_trace_clear();
	zassert_false(meshtastic_reboot_trace_get(&rt));
}

ZTEST(meshtastic_reboot_trace, test_reason_and_type_render_without_lying)
{
	zassert_str_equal(meshtastic_reboot_reason_str(MESHTASTIC_REBOOT_WATCHDOG), "watchdog");
	zassert_str_equal(meshtastic_reboot_reason_str(MESHTASTIC_REBOOT_UNKNOWN), "unknown");
	/* An out-of-range value must say so rather than index off the end or read as a real
	 * reason. */
	zassert_str_equal(meshtastic_reboot_reason_str(200U), "invalid");
	zassert_str_equal(meshtastic_reboot_type_str((uint8_t)SYS_REBOOT_COLD), "cold");
	zassert_str_equal(meshtastic_reboot_type_str((uint8_t)SYS_REBOOT_WARM), "warm");
}

ZTEST(meshtastic_reboot_trace, test_detail_longer_than_the_field_is_truncated_safely)
{
	struct meshtastic_reboot_trace rt;

	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_WATCHDOG,
				     "a-channel-name-far-longer-than-the-field-allows");
	meshtastic_reboot_trace_capture(SYS_REBOOT_WARM, 0x1U, true);

	zassert_true(meshtastic_reboot_trace_get(&rt));
	zassert_equal(rt.detail[sizeof(rt.detail) - 1], '\0',
		      "detail must stay NUL-terminated: this is written from a fatal handler "
		      "and read by a printf on the next boot");
	zassert_true(strlen(rt.detail) < sizeof(rt.detail));
}
