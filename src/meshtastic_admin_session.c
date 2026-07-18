/* SPDX-License-Identifier: GPL-3.0
 *
 * Admin session-passkey engine. See meshtastic_admin_session.h for the model.
 */

#include "meshtastic_admin_session.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/*
 * Reference firmware AdminModule windows: rotate the key once it is older than
 * 150 s, keep accepting it for 300 s. The overlap lets a client that just read
 * config still write while the node rolls to a fresh key.
 */
#define SESSION_ROTATE_SECONDS 150
#define SESSION_VALID_SECONDS  300

static K_MUTEX_DEFINE(session_lock);
static uint8_t session_key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
static int64_t session_born; /* uptime seconds when session_key was generated */
static bool session_present;

/* Monotonic uptime in seconds — matches the reference firmware's millis()/1000
 * cadence and, unlike wall-clock epoch, is always available and never rewound
 * by a phone-supplied set_time. */
static int64_t now_seconds(void)
{
	return k_uptime_get() / 1000;
}

void meshtastic_admin_session_current(uint8_t out[MESHTASTIC_ADMIN_SESSION_KEY_LEN])
{
	int64_t now = now_seconds();

	k_mutex_lock(&session_lock, K_FOREVER);
	if (!session_present || (now - session_born) > SESSION_ROTATE_SECONDS) {
		/* Non-crypto RNG, matching the reference firmware's random(); the
		 * passkey guards replay, not confidentiality. Harden with
		 * sys_csrand_get() if a threat model demands it. */
		sys_rand_get(session_key, sizeof(session_key));
		session_born = now;
		session_present = true;
		LOG_DBG("admin: issued new session passkey");
	}
	memcpy(out, session_key, sizeof(session_key));
	k_mutex_unlock(&session_lock);
}

bool meshtastic_admin_session_valid(const uint8_t *key, size_t len)
{
	int64_t now = now_seconds();
	bool ok;

	if (key == NULL || len != MESHTASTIC_ADMIN_SESSION_KEY_LEN) {
		return false;
	}

	k_mutex_lock(&session_lock, K_FOREVER);
	ok = session_present && ((now - session_born) < SESSION_VALID_SECONDS) &&
	     memcmp(key, session_key, sizeof(session_key)) == 0;
	k_mutex_unlock(&session_lock);
	return ok;
}

void meshtastic_admin_session_reset(void)
{
	k_mutex_lock(&session_lock, K_FOREVER);
	session_present = false;
	session_born = 0;
	memset(session_key, 0, sizeof(session_key));
	k_mutex_unlock(&session_lock);
}
