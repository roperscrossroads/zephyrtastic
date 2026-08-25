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

/* WIFI inhibitor: an up note blocks STANDBY (so light sleep can't drop an
 * associated WiFi link); a down note releases it. Not ref-counted, so duplicate
 * up/down events must be idempotent — with CONFIG_ASSERT=y an unbalanced pm_policy
 * put would abort, so a passing run proves the idempotency. is_power_saving = true
 * keeps POLICY clear so WIFI is observed alone. */
ZTEST(power, test_wifi_inhibits)
{
	apply_power_config(true, 0U, 0U);
	zassert_false(standby_locked(), "idle with power saving on -> unlocked");

	meshtastic_power_note_wifi_up();
	zassert_true(standby_locked(), "WiFi link up must block STANDBY");

	/* Duplicate up (e.g. a re-lease firing IPV4_ADDR_ADD again) stays locked. */
	meshtastic_power_note_wifi_up();
	zassert_true(standby_locked(), "duplicate WiFi up holds the same lock");

	meshtastic_power_note_wifi_down();
	zassert_false(standby_locked(), "WiFi link down releases STANDBY");

	/* Unmatched down must be a guarded no-op, not an underflow. */
	meshtastic_power_note_wifi_down();
	zassert_false(standby_locked(), "duplicate WiFi down stays unlocked");
}

/* WIFI composes with the other inhibitors: STANDBY stays blocked until BOTH the
 * WiFi link is down AND power saving is on (no single inhibitor clears it alone). */
ZTEST(power, test_wifi_composes_with_policy)
{
	apply_power_config(false, 0U, 0U); /* POLICY set (power saving off) */
	meshtastic_power_note_wifi_up();   /* WIFI set too */
	zassert_true(standby_locked(), "both inhibitors set -> locked");

	apply_power_config(true, 0U, 0U); /* clear POLICY; WIFI still held */
	zassert_true(standby_locked(), "WiFi alone still blocks STANDBY");

	meshtastic_power_note_wifi_down(); /* clear WIFI; now nothing is set */
	zassert_false(standby_locked(), "last inhibitor gone -> STANDBY allowed");
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

/* MANUAL inhibitor: the volatile `meshtastic pm off` bench hold. It blocks STANDBY
 * on its own, independent of PowerConfig, and its accessor mirrors the state. */
ZTEST(power, test_manual_hold)
{
	apply_power_config(true, 0U, 0U); /* power saving on -> nothing else inhibits */
	zassert_false(standby_locked(), "idle with power saving on -> unlocked");
	zassert_false(meshtastic_power_manual_inhibit(), "hold starts released");

	meshtastic_power_set_manual_inhibit(true);
	zassert_true(standby_locked(), "manual hold must block STANDBY");
	zassert_true(meshtastic_power_manual_inhibit(), "accessor reflects the hold");

	/* Idempotent: a second set holds the same lock (no double get to unbalance). */
	meshtastic_power_set_manual_inhibit(true);
	zassert_true(standby_locked(), "duplicate hold stays locked");

	meshtastic_power_set_manual_inhibit(false);
	zassert_false(standby_locked(), "releasing the hold allows STANDBY");
	zassert_false(meshtastic_power_manual_inhibit(), "accessor reflects the release");

	/* Idempotent release: a second clear stays unlocked (no double put to assert). */
	meshtastic_power_set_manual_inhibit(false);
	zassert_false(standby_locked(), "duplicate release stays unlocked");
}

/* MANUAL composes with POLICY: it is independent of is_power_saving, so the hold
 * keeps STANDBY blocked even after power saving is turned on, and clearing it while
 * power saving is off leaves POLICY still holding the lock. */
ZTEST(power, test_manual_composes_with_policy)
{
	apply_power_config(false, 0U, 0U);    /* POLICY set (power saving off) */
	meshtastic_power_set_manual_inhibit(true); /* MANUAL set too */
	zassert_true(standby_locked(), "both inhibitors set -> locked");

	apply_power_config(true, 0U, 0U); /* clear POLICY; MANUAL still held */
	zassert_true(standby_locked(), "manual hold alone still blocks STANDBY");

	meshtastic_power_set_manual_inhibit(false); /* clear MANUAL; nothing set */
	zassert_false(standby_locked(), "last inhibitor gone -> STANDBY allowed");

	/* And the reverse order: POLICY outlives a released manual hold. */
	meshtastic_power_set_manual_inhibit(true);
	apply_power_config(false, 0U, 0U); /* POLICY set */
	meshtastic_power_set_manual_inhibit(false);
	zassert_true(standby_locked(), "POLICY still holds after the manual release");
}

/* meshtastic_power_reset() clears the manual hold (INH_MANUAL is in INH_ALL), so it
 * cannot leak across test cases or a governor re-init. */
ZTEST(power, test_manual_cleared_by_reset)
{
	meshtastic_power_set_manual_inhibit(true);
	zassert_true(standby_locked(), "hold engaged");

	meshtastic_power_reset();
	zassert_false(standby_locked(), "reset clears the manual hold");
	zassert_false(meshtastic_power_manual_inhibit(), "accessor clear after reset");
}

/* A node nobody has configured yet must not carry other people's traffic.
 *
 * A fresh config is LONG_FAST + the default PSK + a real region, which IS the
 * public mesh — so with the reference's rebroadcast default of ALL, a node
 * starts relaying strangers' packets within seconds of its first boot. That was
 * observed on a XIAO on 2026-08-24 (agents-tosb), before anyone had touched it.
 * The Kconfig seed is what closes that window.
 *
 * Tested by driving the seeding path DIRECTLY rather than by reading whatever
 * the store happens to hold. Reading the live store is what the obvious version
 * of this test does, and it is wrong twice over: the protocol suite has half a
 * dozen admin/set_config tests that legitimately overwrite the device section
 * (an admin set_config carrying a zeroed DeviceConfig lands rebroadcast_mode =
 * 0), so the check would pass or fail on test ORDER; and a suite that never
 * seeds the store at all reads an empty record and fails for a third,
 * unrelated reason. Seeding here makes the assertion about the seed and nothing
 * else.
 */
ZTEST(power, test_fresh_config_seeds_no_rebroadcast)
{
	struct meshtastic_config init = {0};
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_seed(&init), "seeding the store failed");

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg),
		   "device config unavailable");
	zassert_equal(cfg.which_payload_variant, meshtastic_Config_device_tag, "wrong variant");
	zassert_equal(cfg.payload_variant.device.rebroadcast_mode,
		      (meshtastic_Config_DeviceConfig_RebroadcastMode)
			      CONFIG_MESHTASTIC_DEFAULT_REBROADCAST_MODE,
		      "the fresh config must carry the seeded rebroadcast mode");
	zassert_equal(cfg.payload_variant.device.rebroadcast_mode,
		      meshtastic_Config_DeviceConfig_RebroadcastMode_NONE,
		      "and this build's seed is NONE — a never-configured node stays out of "
		      "a mesh its operator never chose");
}

ZTEST_SUITE(power, NULL, NULL, power_before, NULL, NULL);
