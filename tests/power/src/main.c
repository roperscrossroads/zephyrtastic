/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the light-sleep governor (src/meshtastic_power.c).
 *
 * The governor turns PowerConfig into a single pm_policy STANDBY lock, held while
 * an inhibitor bitmask is nonzero. These tests assert the one observable output —
 * pm_policy_state_lock_is_active(PM_STATE_STANDBY, ...) — as each inhibitor is
 * driven through its edges. The app.overlay defines a real "standby" power-state
 * so the lock is not a no-op (see policy_state_lock.c's DT gate).
 *
 * No meshtastic_init() is called: the governor is exercised directly through its
 * public notes and meshtastic_power_config_apply(). Where a suite needs a stored
 * PowerConfig it seeds one via the real meshtastic_config_store_set_config() path
 * (RAM-only under SETTINGS=n) and then applies it, so the store read in
 * config_apply() is covered too. meshtastic_power_reset() returns the governor to
 * its power-on state between tests.
 *
 * Time is driven with k_sleep(), which fast-forwards under native_sim and lets
 * the system workqueue run the governor's one-shot delayables.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

/* Reset seam exported by meshtastic_power.c (no header decl: test-support only). */
extern void meshtastic_power_reset(void);

/* native_sim has no SoC PM backend, so the PM subsystem's pm_state_set /
 * pm_state_exit_post_ops are unresolved without a provider. The governor test
 * never actually enters the state (it only queries the policy lock), so empty
 * hooks suffice — mirrors zephyr/tests/subsys/pm/policy_api/src/main.c, paired
 * with this test's Kconfig `select HAS_PM`. */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

static inline bool standby_locked(void)
{
	return pm_policy_state_lock_is_active(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
}

/* Seed a PowerConfig into the store and apply it through the real governor path. */
static void apply_power_config(bool is_power_saving, uint32_t min_wake_secs,
			       uint32_t wait_bluetooth_secs)
{
	meshtastic_Config cfg = meshtastic_Config_init_zero;

	cfg.which_payload_variant = meshtastic_Config_power_tag;
	cfg.payload_variant.power.is_power_saving = is_power_saving;
	cfg.payload_variant.power.min_wake_secs = min_wake_secs;
	cfg.payload_variant.power.wait_bluetooth_secs = wait_bluetooth_secs;

	zassert_ok(meshtastic_config_store_set_config(&cfg), "seeding PowerConfig failed");
	meshtastic_power_config_apply();
}

static void power_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_power_reset();
	zassert_false(standby_locked(), "reset must leave STANDBY unlocked");
}

/* PHONE inhibitor: ref-counted across clients; the lock toggles only on the
 * 0<->nonzero edge, and an unmatched disconnect is a no-op (CONFIG_ASSERT would
 * otherwise abort on the unbalanced pm_policy put). */
ZTEST(power, test_phone_refcount)
{
	meshtastic_power_note_phone_connected();
	zassert_true(standby_locked(), "first client must block STANDBY");

	meshtastic_power_note_phone_connected();
	zassert_true(standby_locked(), "second client holds the same lock");

	meshtastic_power_note_phone_disconnected();
	zassert_true(standby_locked(), "still one client attached -> still locked");

	meshtastic_power_note_phone_disconnected();
	zassert_false(standby_locked(), "last client gone -> STANDBY allowed");

	/* Extra disconnect at count 0 must be a guarded no-op, not an underflow. */
	meshtastic_power_note_phone_disconnected();
	zassert_false(standby_locked(), "unmatched disconnect stays unlocked");
}

/* POLICY inhibitor: is_power_saving == false blocks STANDBY; true allows it. */
ZTEST(power, test_policy_toggle)
{
	apply_power_config(false, 0U, 0U);
	zassert_true(standby_locked(), "power saving OFF must block STANDBY");

	apply_power_config(true, 0U, 0U);
	zassert_false(standby_locked(), "power saving ON must allow STANDBY");
}

/* ACTIVITY inhibitor: a note holds the lock for min_wake_secs, then it clears.
 * is_power_saving = true so POLICY is clear and ACTIVITY is observed alone. */
ZTEST(power, test_activity_expires)
{
	apply_power_config(true, 2U, 0U);
	zassert_false(standby_locked(), "idle with power saving on -> unlocked");

	meshtastic_power_note_activity();
	zassert_true(standby_locked(), "activity holds STANDBY awake");

	/* Past the 2 s window the one-shot delayable clears ACTIVITY. */
	k_sleep(K_SECONDS(3));
	zassert_false(standby_locked(), "STANDBY allowed once min_wake_secs elapses");
}

/* min_wake_secs == 0 means activity notes are a no-op (port's value-0 semantics). */
ZTEST(power, test_activity_disabled_when_zero)
{
	apply_power_config(true, 0U, 0U);

	meshtastic_power_note_activity();
	zassert_false(standby_locked(), "min_wake_secs==0 -> activity never inhibits");
}

/* Coalescing: repeated notes extend the single window past any one note's span,
 * exercising the deadline-recheck reschedule. */
ZTEST(power, test_activity_coalesces)
{
	apply_power_config(true, 2U, 0U);

	meshtastic_power_note_activity(); /* deadline ~t+2s */
	k_sleep(K_SECONDS(1));            /* t=1s */
	zassert_true(standby_locked(), "within first window");

	meshtastic_power_note_activity(); /* deadline ~t+3s */
	k_sleep(K_SECONDS(1));            /* t=2s: past the first note's 2s window */
	zassert_true(standby_locked(), "coalesced window still holds past first span");

	meshtastic_power_note_activity(); /* deadline ~t+4s */

	/* Now let it drain with no further notes. */
	k_sleep(K_SECONDS(3));           /* t=5s: past the last deadline */
	zassert_false(standby_locked(), "STANDBY allowed once activity stops");
}

/* BOOT_WAIT inhibitor: armed once per boot for wait_bluetooth_secs; a live
 * re-apply must not re-arm it, and it clears when the window elapses. */
ZTEST(power, test_boot_wait_arms_once)
{
	apply_power_config(true, 0U, 2U);
	zassert_true(standby_locked(), "boot wait window blocks STANDBY");

	/* A second apply (e.g. a live phone write) must not re-arm BOOT_WAIT. */
	apply_power_config(true, 0U, 2U);
	zassert_true(standby_locked(), "re-apply keeps the single boot window");

	k_sleep(K_SECONDS(3));
	zassert_false(standby_locked(), "STANDBY allowed after wait_bluetooth_secs");
}

ZTEST_SUITE(power, NULL, NULL, power_before, NULL, NULL);
