/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Light-sleep governor — turn Meshtastic's PowerConfig into pm_policy constraints.
 *
 * Upstream drives light sleep from a 100 ms-ticked PowerFSM that calls
 * esp_light_sleep_start() itself. Zephyr's PM model is the inverse: the idle
 * thread sleeps automatically whenever every thread is blocked, and the policy
 * picks the deepest state that fits the next timeout. So we do not transliterate
 * the FSM — we express "reasons to stay awake" as constraints on a single
 * pm_policy STANDBY lock and let Zephyr's idle path do the sleeping. No periodic
 * tick: one-shot delayables are armed only by activity.
 *
 * The constraints (inhibitors) and what makes each real:
 *   POLICY    — is_power_saving == false (upstream's default: a mains/WiFi node
 *               stays responsive). The only always-considered inhibitor.
 *   PHONE     — a BLE or TCP PhoneAPI client is connected (ref-counted).
 *   ACTIVITY  — within min_wake_secs of the last RX-for-us or button press.
 *   BOOT_WAIT — within wait_bluetooth_secs of boot (a window to pair a phone).
 *
 * STANDBY (light sleep) is blocked while any inhibitor is set; the SoC
 * light-sleeps only when the whole mask clears. This consumes is_power_saving +
 * min_wake_secs + wait_bluetooth_secs from PowerConfig. Deep sleep (ls_secs /
 * sds_secs) is deliberately out of scope — it needs a separate force-sleep +
 * LoRa-wake mechanism.
 *
 * Value-0 semantics: min_wake_secs == 0 means ACTIVITY is never engaged (the
 * activity notes no-op); wait_bluetooth_secs == 0 means BOOT_WAIT is never armed.
 * This diverges from upstream's "0 = compiled default" on purpose, to honour the
 * port's "no wake the config didn't ask for" discipline.
 *
 * Compiled only with CONFIG_PM (there is no light sleep to gate otherwise). When
 * no PowerConfig is stored, POLICY comes from CONFIG_MESHTASTIC_POWER_SAVE_DEFAULT.
 * Applied at settings-load (boot) via the hook below, and live on a phone toggle
 * (the admin set_config path re-calls meshtastic_power_config_apply()).
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include <zephyr/logging/log.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define INH_POLICY    BIT(0) /* is_power_saving == false */
#define INH_PHONE     BIT(1) /* a BLE or TCP PhoneAPI client is connected */
#define INH_ACTIVITY  BIT(2) /* within min_wake_secs of the last activity */
#define INH_BOOT_WAIT BIT(3) /* within wait_bluetooth_secs of boot */
#define INH_ALL       (INH_POLICY | INH_PHONE | INH_ACTIVITY | INH_BOOT_WAIT)

/* Everything below is guarded by gov_lock. The bitmask read-modify-write AND the
 * paired pm_policy get/put happen together inside one region so the lock toggles
 * exactly on the mask 0<->nonzero edge — never a lost or double put (which would
 * trip the underflow assert in policy_state_lock.c). A spinlock, not a mutex, so
 * the notes stay ISR-safe; pm_policy_state_lock_get/put take their own leaf
 * spinlock and never call back into us, so the nesting is a strict one-way order. */
static struct k_spinlock gov_lock;

static uint32_t inhibitors;        /* nonzero <=> we hold the STANDBY lock */
static uint32_t phone_clients;     /* ref count across BLE + TCP clients */
static uint32_t min_wake_secs_cfg; /* 0 => activity notes are a no-op */
static int64_t activity_deadline;  /* uptime ms; INH_ACTIVITY clears at/after */
static bool boot_armed;            /* BOOT_WAIT is armed at most once per boot */

/* Toggle the single pm_policy STANDBY lock on the mask 0<->nonzero edge. Caller
 * must hold gov_lock so the mask update and the get/put are one indivisible unit. */
static void inhibit_update_locked(uint32_t bits, bool set)
{
	uint32_t old = inhibitors;

	if (set) {
		inhibitors |= bits;
	} else {
		inhibitors &= ~bits;
	}

	if (old == 0U && inhibitors != 0U) {
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	} else if (old != 0U && inhibitors == 0U) {
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

static void activity_expiry(struct k_work *work);
static void boot_wait_expiry(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(activity_work, activity_expiry);
static K_WORK_DELAYABLE_DEFINE(boot_wait_work, boot_wait_expiry);

/* Deadline recheck: a stale timer can fire in the gap between a note's unlock and
 * its k_work_reschedule. Re-reading the deadline here keeps a still-live window
 * from being cleared early — if time remains, re-arm for exactly the remainder. */
static void activity_expiry(struct k_work *work)
{
	ARG_UNUSED(work);
	k_spinlock_key_t key = k_spin_lock(&gov_lock);
	int64_t now = k_uptime_get();

	if (now >= activity_deadline) {
		inhibit_update_locked(INH_ACTIVITY, false);
		k_spin_unlock(&gov_lock, key);
	} else {
		int64_t remaining = activity_deadline - now;

		k_spin_unlock(&gov_lock, key);
		(void)k_work_reschedule(&activity_work, K_MSEC(remaining));
	}
}

/* BOOT_WAIT is armed exactly once, so no coalescing/recheck is needed. */
static void boot_wait_expiry(struct k_work *work)
{
	ARG_UNUSED(work);
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	inhibit_update_locked(INH_BOOT_WAIT, false);
	k_spin_unlock(&gov_lock, key);
}

void meshtastic_power_note_phone_connected(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	phone_clients++;
	if (phone_clients == 1U) {
		inhibit_update_locked(INH_PHONE, true);
	}
	k_spin_unlock(&gov_lock, key);
}

void meshtastic_power_note_phone_disconnected(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	/* Guard against an unmatched disconnect (a put at count 0 would assert). */
	if (phone_clients > 0U) {
		phone_clients--;
		if (phone_clients == 0U) {
			inhibit_update_locked(INH_PHONE, false);
		}
	}
	k_spin_unlock(&gov_lock, key);
}

void meshtastic_power_note_activity(void)
{
	uint32_t secs;
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	secs = min_wake_secs_cfg;
	if (secs == 0U) {
		k_spin_unlock(&gov_lock, key);
		return;
	}

	activity_deadline = k_uptime_get() + (int64_t)secs * MSEC_PER_SEC;
	inhibit_update_locked(INH_ACTIVITY, true);
	k_spin_unlock(&gov_lock, key);

	/* Coalescing: reschedule (not schedule) so repeated activity extends the one
	 * timer rather than stacking. Armed outside the lock — the work handler takes
	 * gov_lock, and pm_policy work never calls back here. */
	(void)k_work_reschedule(&activity_work, K_SECONDS(secs));
}

void meshtastic_power_config_apply(void)
{
	meshtastic_Config config;
	bool saving = IS_ENABLED(CONFIG_MESHTASTIC_POWER_SAVE_DEFAULT);
	uint32_t min_wake = 0U;
	uint32_t wait_bt = 0U;
	bool arm_boot = false;
	k_spinlock_key_t key;

	if (meshtastic_config_store_get_config(meshtastic_Config_power_tag, &config) == 0 &&
	    config.which_payload_variant == meshtastic_Config_power_tag) {
		saving = config.payload_variant.power.is_power_saving;
		min_wake = config.payload_variant.power.min_wake_secs;
		wait_bt = config.payload_variant.power.wait_bluetooth_secs;
	}

	key = k_spin_lock(&gov_lock);
	min_wake_secs_cfg = min_wake;
	/* POLICY blocks STANDBY while power saving is OFF (stay responsive). */
	inhibit_update_locked(INH_POLICY, !saving);
	/* Arm BOOT_WAIT once per boot; re-applies (live phone writes) don't re-arm. */
	if (!boot_armed && wait_bt > 0U) {
		boot_armed = true;
		arm_boot = true;
		inhibit_update_locked(INH_BOOT_WAIT, true);
	}
	k_spin_unlock(&gov_lock, key);

	if (arm_boot) {
		(void)k_work_reschedule(&boot_wait_work, K_SECONDS(wait_bt));
	}

	LOG_INF("power governor: saving=%d min_wake=%us wait_bt=%us", saving, min_wake, wait_bt);
}

/* Return the governor to its power-on state. Its only caller is the native_sim
 * ztest suite, which needs it because the file-static state persists across test
 * cases in one binary. It is compiled unconditionally rather than behind
 * CONFIG_ZTEST: it has no header declaration (so it never widens the production
 * API) and, being unreferenced in a firmware image, is dropped by the linker's
 * --gc-sections. A CONFIG_ZTEST guard was tried first but the symbol did not
 * survive into the module object even with CONFIG_ZTEST set, so this is the
 * robust form. */
void meshtastic_power_reset(void)
{
	(void)k_work_cancel_delayable(&activity_work);
	(void)k_work_cancel_delayable(&boot_wait_work);

	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	inhibit_update_locked(INH_ALL, false); /* clears every bit -> one put if held */
	phone_clients = 0U;
	min_wake_secs_cfg = 0U;
	activity_deadline = 0;
	boot_armed = false;

	k_spin_unlock(&gov_lock, key);
}

static int power_apply(void)
{
	meshtastic_power_config_apply();
	return 0;
}

MESHTASTIC_SETTINGS_APPLY_DEFINE(power_saving, power_apply);
