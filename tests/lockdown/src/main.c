/* SPDX-License-Identifier: GPL-3.0
 *
 * Lockdown storage core (agents-dnr4.15, phase 1).
 *
 * The stack is never initialised; the module stands on settings (real NVS on
 * the flash sim) and PSA. A "reboot" is meshtastic_lockdown_init(), which
 * forgets every key and re-reads the artifacts exactly as a cold boot does.
 * Simulated time makes the backoff waits free.
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>

#include "meshtastic_clock.h"
#include "meshtastic_lockdown.h"

static const uint8_t PP[] = "correct horse";
static const uint8_t WRONG[] = "battery staple";

static void *lockdown_setup(void)
{
	zassert_ok(settings_subsys_init(), "settings");
	return NULL;
}

/* Every test starts inactive on a clean flash. */
static void lockdown_before(void *f)
{
	ARG_UNUSED(f);
	meshtastic_lockdown_init();
	if (meshtastic_lockdown_active()) {
		(void)meshtastic_lockdown_remove_artifacts();
	}
	meshtastic_lockdown_init();
	zassert_false(meshtastic_lockdown_active(), "clean start");
}

ZTEST_SUITE(lockdown, NULL, lockdown_setup, lockdown_before, NULL, NULL);

/* Raw artifact access for the tamper/rollback tests. */
struct raw {
	uint8_t buf[128];
	size_t len;
};

static int raw_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param)
{
	struct raw *r = param;

	ARG_UNUSED(key);
	r->len = (len <= sizeof(r->buf) && read_cb(cb_arg, r->buf, len) == (ssize_t)len) ? len : 0U;
	return 1;
}

static size_t raw_read(const char *name, struct raw *r)
{
	r->len = 0U;
	(void)settings_load_subtree_direct(name, raw_cb, r);
	return r->len;
}

static void reboot(void)
{
	meshtastic_lockdown_init();
}

ZTEST(lockdown, test_inactive_device_behaves_like_stock)
{
	uint8_t out[64];

	zassert_false(meshtastic_lockdown_active(), "");
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "not_provisioned", "");
	zassert_equal(meshtastic_lockdown_seal("config/lora", "x", 1U, out, sizeof(out)), -ENOTSUP,
		      "inactive: the caller stores plaintext");
	zassert_equal(meshtastic_lockdown_open("config/lora", "plain", 5U, out, sizeof(out)),
		      -EBADMSG, "a plaintext blob is not a sealed record");
	zassert_equal(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 0U, 0U, 0U), -ENOENT, "");
}

ZTEST(lockdown, test_provision_then_seal_open_bound_to_the_name)
{
	uint8_t sealed[96];
	uint8_t plain[64];
	int n, m;

	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 0U, 0U, 0U), "");
	zassert_true(meshtastic_lockdown_active() && meshtastic_lockdown_unlocked(), "");
	zassert_equal(meshtastic_lockdown_boots_remaining(), CONFIG_MESHTASTIC_LOCKDOWN_DEFAULT_BOOTS, "");
	zassert_equal(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 0U, 0U, 0U), -EALREADY, "");

	n = meshtastic_lockdown_seal("config/lora", "secret", 6U, sealed, sizeof(sealed));
	zassert_equal(n, 6 + (int)MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD, "sealed length (%d)", n);
	zassert_true(meshtastic_lockdown_is_sealed(sealed, (size_t)n), "");
	m = meshtastic_lockdown_open("config/lora", sealed, (size_t)n, plain, sizeof(plain));
	zassert_equal(m, 6, "opened (%d)", m);
	zassert_mem_equal(plain, "secret", 6U, "");

	zassert_equal(meshtastic_lockdown_open("config/device", sealed, (size_t)n, plain, sizeof(plain)),
		      -EBADMSG, "another name: the record does not open (AAD)");
	sealed[n - 1] ^= 0x01U;
	zassert_equal(meshtastic_lockdown_open("config/lora", sealed, (size_t)n, plain, sizeof(plain)),
		      -EBADMSG, "a flipped bit: refused");

	/* Locked: sealing and opening both refuse, and say why. */
	meshtastic_lockdown_lock_now();
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_equal(meshtastic_lockdown_seal("config/lora", "x", 1U, sealed, sizeof(sealed)), -EACCES, "");
	sealed[n - 1] ^= 0x01U;
	zassert_equal(meshtastic_lockdown_open("config/lora", sealed, (size_t)n, plain, sizeof(plain)),
		      -EACCES, "");
}

ZTEST(lockdown, test_token_unlocks_the_next_boots_and_counts_down)
{
	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 3U, 0U, 0U), "");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 3U, "");

	reboot();
	zassert_true(meshtastic_lockdown_unlocked(), "boot 1: token");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 2U, "");
	reboot();
	zassert_true(meshtastic_lockdown_unlocked(), "boot 2");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 1U, "");
	reboot();
	zassert_true(meshtastic_lockdown_unlocked(), "boot 3: the last boot the token buys");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 0U, "");
	reboot();
	zassert_false(meshtastic_lockdown_unlocked(), "boot 4: budget spent");
	zassert_true(meshtastic_lockdown_active(), "still provisioned");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "token_missing", "");

	/* The passphrase issues a fresh token. */
	zassert_ok(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 2U, 0U, 0U), "");
	reboot();
	zassert_true(meshtastic_lockdown_unlocked(), "");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 1U, "");
}

ZTEST(lockdown, test_tampered_and_rolled_back_tokens_are_refused)
{
	struct raw old, cur;

	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 10U, 0U, 0U), "");
	zassert_true(raw_read("mtlock/token", &old) > 0U, "token present");

	/* Tamper: one flipped byte in the wrapped DEK. */
	memcpy(&cur, &old, sizeof(cur));
	cur.buf[20] ^= 0x80U;
	zassert_ok(settings_save_one("mtlock/token", cur.buf, cur.len), "");
	reboot();
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "token_hmac_fail", "");
	zassert_equal(raw_read("mtlock/token", &cur), 0U, "a bad token is deleted");

	/* Rollback: a genuine but older token (its counter is below the persisted
	 * high-water mark once a newer one has been issued). */
	zassert_ok(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 10U, 0U, 0U), "");
	zassert_ok(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 10U, 0U, 0U), "newer token issued");
	zassert_ok(settings_save_one("mtlock/token", old.buf, old.len), "restore the old one");
	reboot();
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "token_rollback", "");
}

ZTEST(lockdown, test_token_epoch_expiry_needs_a_clock)
{
	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 10U, 1700000000U, 0U), "");
	reboot();
	zassert_true(meshtastic_lockdown_unlocked(), "no clock: the boot count rules");

	meshtastic_clock_set_epoch(1700000100U, MESHTASTIC_CLOCK_QUALITY_DEVICE);
	reboot();
	zassert_false(meshtastic_lockdown_unlocked(), "clock past the epoch");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "token_expired", "");
}

/* The reference's three floors after a wrong passphrase: uptime since the
 * failure, wall clock since the failure (when a clock exists), and REBOOTS
 * since the failure -- one boot per 5 s of delay. That last one means a retry
 * in the same boot is always refused (bootsSinceFail is 0 until a reboot):
 * a wrong passphrase costs a reboot. Mirrored, and flagged in the design. */
ZTEST(lockdown, test_wrong_passphrase_backs_off_and_the_right_one_clears_it)
{
	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 0U, 0U, 0U), "");
	meshtastic_lockdown_lock_now();

	zassert_equal(meshtastic_lockdown_unlock(WRONG, sizeof(WRONG) - 1U, 0U, 0U, 0U), -EACCES, "");
	zassert_equal(meshtastic_lockdown_backoff_remaining(), 5U, "first failure: 5 s");
	zassert_equal(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 0U, 0U, 0U), -EAGAIN,
		      "even the right passphrase waits");
	k_sleep(K_SECONDS(6));
	zassert_equal(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 0U, 0U, 0U), -EAGAIN,
		      "uptime alone does not clear it: the reboot floor (reference)");

	reboot();
	zassert_equal(meshtastic_lockdown_unlock(WRONG, sizeof(WRONG) - 1U, 0U, 0U, 0U), -EACCES, "");
	zassert_equal(meshtastic_lockdown_backoff_remaining(), 10U, "second failure: doubled");
	reboot();
	zassert_equal(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 0U, 0U, 0U), -EAGAIN,
		      "10 s is two boots' worth; one has passed");
	reboot();
	/* An earlier test left the simulated clock valid, so the wall-clock floor
	 * applies too: the seconds have to pass, not only the boots. */
	k_sleep(K_SECONDS(11));
	zassert_ok(meshtastic_lockdown_unlock(PP, sizeof(PP) - 1U, 0U, 0U, 0U), "after the wait");
	zassert_true(meshtastic_lockdown_unlocked(), "");
	zassert_equal(meshtastic_lockdown_backoff_remaining(), 0U, "success clears the backoff");
	zassert_equal(meshtastic_lockdown_unlock(WRONG, sizeof(WRONG) - 1U, 0U, 0U, 0U), -EACCES, "");
	zassert_equal(meshtastic_lockdown_backoff_remaining(), 5U, "curve restarted");
}

ZTEST(lockdown, test_backoff_curve_caps_at_fifteen_minutes)
{
	/* Nine failures with the reboots each one demands in between (a reboot
	 * zeroes the RAM copy of the remaining wait, so it is captured first). */
	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 0U, 0U, 0U), "");
	meshtastic_lockdown_lock_now();
	for (int i = 0; i < 9; i++) {
		int ret = meshtastic_lockdown_unlock(WRONG, sizeof(WRONG) - 1U, 0U, 0U, 0U);
		uint32_t rem = meshtastic_lockdown_backoff_remaining();

		zassert_equal(ret, -EACCES, "attempt %d: %d", i, ret);
		if (i == 8) {
			zassert_equal(rem, 900U, "9th failure: capped at 15 min");
			break;
		}
		zassert_equal(rem, 5U << i, "failure %d: %u s", i, rem);
		for (uint32_t b = 0; b < (rem + 4U) / 5U; b++) {
			reboot();
		}
	}
}

ZTEST(lockdown, test_remove_artifacts_returns_to_stock)
{
	struct raw r;

	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 0U, 0U, 0U), "");
	zassert_true(raw_read("mtlock/dek", &r) > 0U, "");
	zassert_ok(meshtastic_lockdown_remove_artifacts(), "");
	zassert_false(meshtastic_lockdown_active(), "");
	zassert_equal(raw_read("mtlock/dek", &r), 0U, "");
	zassert_equal(raw_read("mtlock/token", &r), 0U, "");
	reboot();
	zassert_false(meshtastic_lockdown_active(), "stock on the next boot too");
}

/* Phase 3 turned the reference's once-a-second poll into a timer: the cap
 * consumes its boot by itself and reports what it did. Two boots and a 3 s cap
 * give three sessions -- the reference's "(boots + 1) * max_session" ceiling. */
static enum meshtastic_lockdown_event last_ev = -1;
static unsigned int ev_count;

static void on_ev(enum meshtastic_lockdown_event ev)
{
	last_ev = ev;
	ev_count++;
}

ZTEST(lockdown, test_session_cap_consumes_boots_without_rebooting)
{
	meshtastic_lockdown_set_event_hook(on_ev);
	ev_count = 0U;
	zassert_ok(meshtastic_lockdown_provision(PP, sizeof(PP) - 1U, 2U, 0U, 3U), "");
	zassert_false(meshtastic_lockdown_session_expired(), "");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 2U, "");

	k_sleep(K_SECONDS(4));
	zassert_equal(ev_count, 1U, "the cap fired once");
	zassert_equal(last_ev, MESHTASTIC_LOCKDOWN_EV_SESSION_ROLLED, "budget left: rolled");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 1U, "one boot consumed in place");
	zassert_false(meshtastic_lockdown_session_expired(), "re-armed");
	zassert_true(meshtastic_lockdown_unlocked(), "storage stays unlocked");

	k_sleep(K_SECONDS(3));
	zassert_equal(ev_count, 2U, "");
	zassert_equal(last_ev, MESHTASTIC_LOCKDOWN_EV_SESSION_ROLLED, "");
	zassert_equal(meshtastic_lockdown_boots_remaining(), 0U, "last boot consumed: token deleted");
	zassert_true(meshtastic_lockdown_unlocked(), "this session still runs");

	k_sleep(K_SECONDS(3));
	zassert_equal(ev_count, 3U, "");
	zassert_equal(last_ev, MESHTASTIC_LOCKDOWN_EV_SESSION_EXHAUSTED, "budget spent: locked");
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "session_budget_exhausted", "");
	zassert_equal(meshtastic_lockdown_consume_session_boot(), 0U, "nothing left to consume");

	k_sleep(K_SECONDS(4));
	zassert_equal(ev_count, 3U, "a locked device's timer is gone");
	meshtastic_lockdown_set_event_hook(NULL);
	reboot();
	zassert_false(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(meshtastic_lockdown_lock_reason(), "token_missing", "");
}
