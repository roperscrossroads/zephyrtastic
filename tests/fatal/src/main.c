/* SPDX-License-Identifier: GPL-3.0
 *
 * The fatal breadcrumb (agents-ninc): k_sys_fatal_error_handler() records the
 * fault that started a chain, and a fault raised inside the fatal path itself
 * (a log flush, the reboot) must not overwrite it -- it only counts. On rzr5
 * and rzr7 (2026-11-24) the record named only the second fault of such a
 * chain, so "meshtastic resets" said "fatal, from thread <isr>" and the real
 * cause survived only in the log ring.
 *
 * native_sim has no exception frame and no fault status registers, so the
 * frame fields stay zero here; what this checks is the record's lifecycle.
 * The handler is called directly; with RESET_ON_FATAL_ERROR off it ends in
 * arch_system_halt(), wrapped at link time (CMakeLists.txt) to jump back into
 * the test.
 */

#include <setjmp.h>
#include <string.h>

#include <zephyr/fatal.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/diagnostics.h>

static jmp_buf halted;
static unsigned int halted_reason;

FUNC_NORETURN void __wrap_arch_system_halt(unsigned int reason)
{
	halted_reason = reason;
	longjmp(halted, 1);
}

/* One pass through the handler, as z_fatal_error() would make it. */
static void fault(unsigned int reason)
{
	if (setjmp(halted) == 0) {
		k_sys_fatal_error_handler(reason, NULL);
	}
	zassert_equal(halted_reason, reason, "the handler halted with the reason it was given");
}

static void drain(void *f)
{
	struct meshtastic_fatal_crash_info info;

	ARG_UNUSED(f);
	(void)meshtastic_fatal_take_last_crash(&info);
}

ZTEST_SUITE(fatal, NULL, NULL, drain, drain, NULL);

ZTEST(fatal, test_a_fault_is_recorded_with_its_reason_and_thread)
{
	struct meshtastic_fatal_crash_info info;

	k_thread_name_set(k_current_get(), "faulter");
	fault(K_ERR_KERNEL_OOPS);

	zassert_true(meshtastic_fatal_peek_last_crash(&info), "no record");
	zassert_equal(info.reason, K_ERR_KERNEL_OOPS);
	zassert_str_equal(info.thread_name, "faulter");
	zassert_equal(info.nested, 0U);
	zassert_true(meshtastic_fatal_take_last_crash(&info), "take");
	zassert_false(meshtastic_fatal_peek_last_crash(&info), "one-shot");
}

/* agents-ninc: the second pass is a fault inside the fatal path. The record
 * keeps the first fault and counts the second. */
ZTEST(fatal, test_a_fault_inside_the_fatal_path_keeps_the_first)
{
	struct meshtastic_fatal_crash_info info;

	k_thread_name_set(k_current_get(), "first");
	fault(K_ERR_KERNEL_PANIC);
	k_thread_name_set(k_current_get(), "second");
	fault(K_ERR_CPU_EXCEPTION);
	fault(K_ERR_STACK_CHK_FAIL);

	zassert_true(meshtastic_fatal_take_last_crash(&info), "no record");
	zassert_equal(info.reason, K_ERR_KERNEL_PANIC, "the first fault, not %u", info.reason);
	zassert_str_equal(info.thread_name, "first");
	zassert_equal(info.nested, 2U, "two faults inside the fatal path");
}

/* Taking the record (the next boot's report) starts a new chain. */
ZTEST(fatal, test_after_the_record_is_taken_a_new_fault_is_recorded)
{
	struct meshtastic_fatal_crash_info info;

	fault(K_ERR_KERNEL_PANIC);
	zassert_true(meshtastic_fatal_take_last_crash(&info));
	fault(K_ERR_KERNEL_OOPS);
	zassert_true(meshtastic_fatal_take_last_crash(&info));
	zassert_equal(info.reason, K_ERR_KERNEL_OOPS);
	zassert_equal(info.nested, 0U);
}

/* The boot report takes the record so a later boot does not report it again; until
 * this case's fix `meshtastic crashinfo`, run after it, then found nothing (rzr5,
 * 2026-11-25). What was taken stays readable for the rest of the boot. */
ZTEST(fatal, test_the_record_taken_at_boot_is_still_readable)
{
	struct meshtastic_fatal_crash_info info;

	k_thread_name_set(k_current_get(), "reported");
	fault(K_ERR_STACK_CHK_FAIL);
	zassert_true(meshtastic_fatal_take_last_crash(&info), "take");
	zassert_false(meshtastic_fatal_peek_last_crash(&info), "taken: nothing pending");

	memset(&info, 0, sizeof(info));
	zassert_true(meshtastic_fatal_reported_at_boot(&info), "the taken record is gone");
	zassert_equal(info.reason, K_ERR_STACK_CHK_FAIL);
	zassert_str_equal(info.thread_name, "reported");
}
