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
 *   WIFI      — the network link is up (an IPv4 lease is held). Reproduces
 *               upstream's !isWifiAvailable() light-sleep gate: the Zephyr esp32
 *               WiFi path has no DTIM/beacon-wakeup coordination, so a CPU-domain-
 *               down light sleep would drop the association (the AP deauths on the
 *               missed keepalives). Driven from IPv4 addr add/del below.
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
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include <zephyr/logging/log.h>

#if defined(CONFIG_NETWORKING)
#include <zephyr/init.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#endif

#include "meshtastic/config.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define INH_POLICY    BIT(0) /* is_power_saving == false */
#define INH_PHONE     BIT(1) /* a BLE or TCP PhoneAPI client is connected */
#define INH_ACTIVITY  BIT(2) /* within min_wake_secs of the last activity */
#define INH_BOOT_WAIT BIT(3) /* within wait_bluetooth_secs of boot */
#define INH_WIFI      BIT(4) /* the network link is up (IPv4 lease held) */
#define INH_MANUAL    BIT(5) /* a bench/debug hold: `meshtastic pm off` (volatile) */
#define INH_ALL                                                                                    \
	(INH_POLICY | INH_PHONE | INH_ACTIVITY | INH_BOOT_WAIT | INH_WIFI | INH_MANUAL)

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

/* Network link up/down. Not ref-counted: a single interface has one link state,
 * and inhibit_update_locked() is idempotent on a set bit — so a duplicate up (e.g.
 * a re-lease firing IPV4_ADDR_ADD again) or an unmatched down is a safe no-op, never
 * a lost or double pm_policy put. */
void meshtastic_power_note_wifi_up(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	inhibit_update_locked(INH_WIFI, true);
	k_spin_unlock(&gov_lock, key);
}

void meshtastic_power_note_wifi_down(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	inhibit_update_locked(INH_WIFI, false);
	k_spin_unlock(&gov_lock, key);
}

/* Bench/debug override: pin the node awake (hold == true) or release the pin
 * (hold == false), independent of every other inhibitor. This is a VOLATILE hold —
 * it is not persisted and does not touch PowerConfig, so it survives no reboot and
 * fights no phone. It exists to answer "is light sleep the cause?" in one command:
 * `meshtastic pm off` sets it so a node stays observable for a session; `meshtastic
 * pm on` clears it. Deliberately distinct from is_power_saving (the persisted
 * preference the POLICY inhibitor tracks) — an observability switch, not a config
 * write, so it is not gated by CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE nor refused on a
 * managed node. Idempotent on the mask, like the WiFi notes. */
void meshtastic_power_set_manual_inhibit(bool hold)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);

	inhibit_update_locked(INH_MANUAL, hold);
	k_spin_unlock(&gov_lock, key);
}

bool meshtastic_power_manual_inhibit(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);
	bool held = (inhibitors & INH_MANUAL) != 0U;

	k_spin_unlock(&gov_lock, key);
	return held;
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

/* Bench diagnostic: the live inhibitor mask, and a decode of the set bits. Lets a
 * console answer "why isn't the SoC light-sleeping right now" — STANDBY is blocked
 * while any bit is set, so an empty decode means the node is free to sleep. Read
 * under the same spinlock the notes use so the snapshot is coherent. */
uint32_t meshtastic_power_inhibitors(void)
{
	k_spinlock_key_t key = k_spin_lock(&gov_lock);
	uint32_t v = inhibitors;

	k_spin_unlock(&gov_lock, key);
	return v;
}

void meshtastic_power_inhibitors_str(uint32_t mask, char *buf, size_t n)
{
	static const struct {
		uint32_t bit;
		const char *name;
	} names[] = {
		{INH_POLICY, "POLICY"},       {INH_PHONE, "PHONE"}, {INH_ACTIVITY, "ACTIVITY"},
		{INH_BOOT_WAIT, "BOOT_WAIT"}, {INH_WIFI, "WIFI"},   {INH_MANUAL, "MANUAL"},
	};
	size_t off = 0;

	if (n > 0U) {
		buf[0] = '\0';
	}

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		if ((mask & names[i].bit) == 0U) {
			continue;
		}

		int w = snprintf(buf + off, n - off, "%s%s", (off > 0U) ? " " : "", names[i].name);

		if (w < 0 || (size_t)w >= n - off) {
			break;
		}
		off += (size_t)w;
	}
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

#if defined(CONFIG_NETWORKING)
/* Hold the WIFI inhibitor whenever the interface has an IPv4 lease. An IPv4 address
 * is the proxy for a usable (associated + addressed) link; a bare L2 association with
 * no address carries no traffic worth staying awake for, and DHCP re-tries on the
 * next wake. This is a second consumer of the IPv4 addr events alongside the MQTT and
 * SNTP callbacks — net_mgmt fans one event out to every registered callback. */
static struct net_mgmt_event_callback power_net_cb;

static void power_net_event(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			    struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		meshtastic_power_note_wifi_up();
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_DEL) {
		meshtastic_power_note_wifi_down();
	}
}

static int power_net_init(void)
{
	net_mgmt_init_event_callback(&power_net_cb, power_net_event,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&power_net_cb);
	return 0;
}

/* APPLICATION phase: the net stack is up but no IPv4 lease exists yet (DHCP runs
 * after WiFi associates), so no ADD event is missed by registering here. */
SYS_INIT(power_net_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /* CONFIG_NETWORKING */
