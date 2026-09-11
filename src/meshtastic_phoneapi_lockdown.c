/* SPDX-License-Identifier: GPL-3.0
 *
 * Lockdown on the PhoneAPI (agents-dnr4.15 phase 3): per-connection
 * authorization, the ToRadio gate, the inline lockdown_auth, LockdownStatus
 * frames, and what the storage core's events mean for connections.
 *
 * Mirrors the reference's PhoneAPI access-control half (PhoneAPI.cpp under
 * MESHTASTIC_PHONEAPI_ACCESS_CONTROL: handleLockdownAuthInline, the auth
 * slots, queueLockdownStatus / broadcastLockdownStatus, completePendingUnlocks,
 * revokeAllAuth) with two deliberate differences:
 *
 *  - There is no slot table. Each transport owns one PhoneAPI instance and one
 *    connection at a time, so the authorization flag lives on the instance and
 *    a disconnect (meshtastic_phoneapi_reset) clears it. The connection epoch
 *    stands in for the reference's slot identity: a passphrase job finishing
 *    after the connection it came on has gone must not authorize whoever
 *    connects next.
 *
 *  - The passphrase is verified on the system workqueue, never on the
 *    transport's thread (operator decision 2026-09-09, LOCKDOWN-DESIGN.md
 *    §5.3). PBKDF2 at 10 000 rounds is about a second on a Cortex-M4; a
 *    second on the thread that services GATT is a second of no notifications.
 *    The transport thread only peeks at the admin payload, copies the
 *    LockdownAuth into the one job slot, wipes every copy it touched, and
 *    submits. Everything the job then says to the client goes through the
 *    ordinary FromRadio queue.
 *
 * Whoever reads this to understand the mechanism: the storage core
 * (meshtastic_lockdown.c) knows nothing about connections and never sends a
 * frame. It reports five events (locked, reloaded, disabled, session rolled,
 * session exhausted); this file is the only listener and turns each into
 * authorization changes and status frames. The config-stream redaction itself
 * is in meshtastic_phoneapi_config.c, keyed on meshtastic_phoneapi_authorized().
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>

#include <pb_decode.h>

#include <zephyr/meshtastic/reboot_trace.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic_core.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_phoneapi.h"
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include "meshtastic_settings.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Reference DEFAULT_REBOOT_SECONDS: long enough for the status frame to drain. */
#define LOCKDOWN_REBOOT_SECONDS 7

#define TRANSPORTS_MAX 4U

/* The one passphrase job. One at a time is enough: a client waits for its
 * status before sending another, and two connections racing the same PBKDF2
 * would only serialise on the workqueue anyway. Static, never on a stack, and
 * wiped the moment the check is done. */
static struct {
	struct meshtastic_phoneapi *api;
	uint32_t conn_epoch;
	uint8_t passphrase[MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX];
	uint8_t len;
	uint8_t boots;
	uint32_t valid_until;
	uint32_t session_s;
	bool disable;
} job;
static atomic_t job_busy;

/* A disable that came through the PhoneAPI reboots the node once the rewrite
 * is done (the reference's main loop does); one from the console does not --
 * the console is a bench tool that reads status after the disable and reboots
 * when it chooses. The event alone does not say who asked, so the job notes it. */
static bool disable_by_phone;

static void auth_work_fn(struct k_work *work);
static K_WORK_DEFINE(auth_work, auth_work_fn);

static void reboot_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_work_fn);

/* A status frame is built here, not in the transport's from_scratch: that
 * scratch belongs to the transport's own thread, and most status frames are
 * queued from the system workqueue (job completion, the core's events). */
static K_MUTEX_DEFINE(status_lock);
static meshtastic_FromRadio status_scratch;

static void zero(void *p, size_t n)
{
	volatile uint8_t *v = p;

	while (n-- > 0U) {
		*v++ = 0U;
	}
}

/* ---- authorization ------------------------------------------------------------ */

bool meshtastic_phoneapi_authorized(const struct meshtastic_phoneapi *api)
{
	/* The reference's getAdminAuthorized(): nothing to protect while lockdown
	 * is inactive, so every redaction gate is a no-op and the device serves
	 * config exactly like stock. */
	if (!meshtastic_lockdown_active()) {
		return true;
	}
	return api != NULL && api->admin_authorized;
}

static void authorize(struct meshtastic_phoneapi *api)
{
	api->admin_authorized = true;
	api->pending_unlock = false;
}

static void revoke_all(const char *why)
{
	struct meshtastic_phoneapi *t[TRANSPORTS_MAX];
	uint8_t n = meshtastic_phoneapi_snapshot_transports(t, ARRAY_SIZE(t));

	for (uint8_t i = 0U; i < n; i++) {
		t[i]->admin_authorized = false;
		t[i]->pending_unlock = false;
	}
	LOG_INF("lockdown: every connection's authorization revoked (%s)", why);
}

/* ---- status frames -------------------------------------------------------------- */

static void fill_status(meshtastic_FromRadio *from, meshtastic_LockdownStatus_State state,
			const char *reason, uint8_t boots, uint32_t until, uint32_t backoff)
{
	*from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_lockdown_status_tag;
	from->lockdown_status.state = state;
	if (state == meshtastic_LockdownStatus_State_LOCKED && reason != NULL) {
		strncpy(from->lockdown_status.lock_reason, reason,
			sizeof(from->lockdown_status.lock_reason) - 1U);
	}
	from->lockdown_status.boots_remaining = boots;
	from->lockdown_status.valid_until_epoch = until;
	from->lockdown_status.backoff_seconds = backoff;
}

static void queue_status(struct meshtastic_phoneapi *api, meshtastic_LockdownStatus_State state,
			 const char *reason, uint8_t boots, uint32_t until, uint32_t backoff)
{
	k_mutex_lock(&status_lock, K_FOREVER);
	fill_status(&status_scratch, state, reason, boots, until, backoff);
	(void)meshtastic_phoneapi_enqueue_fromradio(api, &status_scratch);
	k_mutex_unlock(&status_lock);
}

static void broadcast_status(meshtastic_LockdownStatus_State state, const char *reason,
			     uint8_t boots, uint32_t until, uint32_t backoff)
{
	struct meshtastic_phoneapi *t[TRANSPORTS_MAX];
	uint8_t n = meshtastic_phoneapi_snapshot_transports(t, ARRAY_SIZE(t));

	k_mutex_lock(&status_lock, K_FOREVER);
	fill_status(&status_scratch, state, reason, boots, until, backoff);
	for (uint8_t i = 0U; i < n; i++) {
		(void)meshtastic_phoneapi_enqueue_fromradio(t[i], &status_scratch);
	}
	k_mutex_unlock(&status_lock);
}

bool meshtastic_phoneapi_lockdown_fill_status(struct meshtastic_phoneapi *api,
					      meshtastic_FromRadio *from)
{
	/* The reference's post-config block: DISABLED tells the app the toggle
	 * exists and is off; LOCKED tells an unauthorized client what to do next.
	 * An authorized connection re-running the handshake is told nothing --
	 * it already knows. NEEDS_PROVISION never arises here: provisioned and
	 * active are the same thing in this port. */
	if (!meshtastic_lockdown_active()) {
		fill_status(from, meshtastic_LockdownStatus_State_DISABLED, NULL, 0U, 0U, 0U);
		return true;
	}
	if (api->admin_authorized) {
		return false;
	}
	fill_status(from, meshtastic_LockdownStatus_State_LOCKED,
		    meshtastic_lockdown_unlocked() ? "needs_auth" : meshtastic_lockdown_lock_reason(),
		    0U, 0U, 0U);
	return true;
}

/* ---- the reboot the reference schedules after lock-now / disable / exhaustion ---- */

static void reboot_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
#if defined(CONFIG_MESHTASTIC_SETTINGS)
	meshtastic_settings_flush();
#endif
	LOG_WRN("lockdown: rebooting now");
	meshtastic_reboot_trace_note(MESHTASTIC_REBOOT_LOCKDOWN, NULL);
	sys_reboot(SYS_REBOOT_COLD);
}

static void schedule_reboot(const char *why)
{
	LOG_INF("lockdown: reboot in %d s (%s)", LOCKDOWN_REBOOT_SECONDS, why);
	(void)k_work_reschedule(&reboot_work, K_SECONDS(LOCKDOWN_REBOOT_SECONDS));
}

bool meshtastic_phoneapi_lockdown_reboot_pending(void)
{
	return k_work_delayable_is_pending(&reboot_work);
}

void meshtastic_phoneapi_lockdown_cancel_reboot(void)
{
	(void)k_work_cancel_delayable(&reboot_work);
}

/* ---- the storage core's events -------------------------------------------------- */

static void complete_pending_unlocks(void)
{
	struct meshtastic_phoneapi *t[TRANSPORTS_MAX];
	uint8_t n = meshtastic_phoneapi_snapshot_transports(t, ARRAY_SIZE(t));
	uint8_t boots = meshtastic_lockdown_boots_remaining();
	uint32_t until = meshtastic_lockdown_valid_until();
	unsigned int done = 0U;

	/* The reference's completePendingUnlocks(): only now, with the real
	 * records in RAM and the radio back up, may the connection that unlocked
	 * see config -- before this it would read placeholders as the operator's
	 * settings, or write them back over the sealed store. */
	for (uint8_t i = 0U; i < n; i++) {
		if (!t[i]->pending_unlock) {
			continue;
		}
		authorize(t[i]);
		queue_status(t[i], meshtastic_LockdownStatus_State_UNLOCKED, NULL, boots, until, 0U);
		done++;
	}
	LOG_INF("lockdown: store reloaded, %u connection(s) authorized", done);
}

static void on_event(enum meshtastic_lockdown_event ev)
{
	switch (ev) {
	case MESHTASTIC_LOCKDOWN_EV_LOCKED:
		revoke_all("locked");
		broadcast_status(meshtastic_LockdownStatus_State_LOCKED,
				 meshtastic_lockdown_lock_reason(), 0U, 0U, 0U);
		break;
	case MESHTASTIC_LOCKDOWN_EV_RELOADED:
		complete_pending_unlocks();
		break;
	case MESHTASTIC_LOCKDOWN_EV_DISABLED:
		/* Every record is in the clear and the artifacts are gone: the
		 * device is stock, and the reference reboots it into that. */
		broadcast_status(meshtastic_LockdownStatus_State_DISABLED, NULL, 0U, 0U, 0U);
		if (disable_by_phone) {
			disable_by_phone = false;
			schedule_reboot("disabled");
		}
		break;
	case MESHTASTIC_LOCKDOWN_EV_SESSION_ROLLED:
		/* Storage stays unlocked and the mesh keeps routing; the phone must
		 * prove the passphrase again to see anything. */
		revoke_all("session cap");
		broadcast_status(meshtastic_LockdownStatus_State_LOCKED, "needs_auth",
				 meshtastic_lockdown_boots_remaining(),
				 meshtastic_lockdown_valid_until(), 0U);
		break;
	case MESHTASTIC_LOCKDOWN_EV_SESSION_EXHAUSTED:
		revoke_all("session budget exhausted");
		broadcast_status(meshtastic_LockdownStatus_State_LOCKED,
				 meshtastic_lockdown_lock_reason(), 0U, 0U, 0U);
		schedule_reboot("session budget exhausted");
		break;
	default:
		break;
	}
}

void meshtastic_phoneapi_lockdown_init(void)
{
	meshtastic_lockdown_set_event_hook(on_event);
}

/* ---- the passphrase job (system workqueue) --------------------------------------- */

static void job_failed(struct meshtastic_phoneapi *api, int ret)
{
	uint32_t backoff = 0U;

	if (ret == -EACCES || ret == -EAGAIN) {
		backoff = meshtastic_lockdown_backoff_remaining();
	}
	/* The reference deliberately does not log the backoff: the client gets
	 * it in the status, and a USB-attached attacker's terminal need not. */
	LOG_WRN("lockdown: passphrase verification failed (%d)", ret);
	queue_status(api, meshtastic_LockdownStatus_State_UNLOCK_FAILED, NULL, 0U, 0U, backoff);
}

static void auth_work_fn(struct k_work *work)
{
	struct meshtastic_phoneapi *api = job.api;
	bool same_conn;
	int ret;

	ARG_UNUSED(work);

	if (job.disable) {
		ret = meshtastic_lockdown_disable(job.passphrase, job.len);
	} else if (!meshtastic_lockdown_active()) {
		LOG_INF("lockdown: first-time provisioning with a passphrase");
		ret = meshtastic_lockdown_provision(job.passphrase, job.len, job.boots,
						    job.valid_until, job.session_s);
	} else {
		ret = meshtastic_lockdown_unlock(job.passphrase, job.len, job.boots,
						 job.valid_until, job.session_s);
	}
	zero(job.passphrase, sizeof(job.passphrase));
	job.len = 0U;

	/* The connection this came on may have gone while PBKDF2 ran. The
	 * storage side effects stand (an unlock is an unlock); the
	 * authorization does not carry over to whoever connects next. */
	same_conn = (api->conn_epoch == job.conn_epoch);

	if (ret < 0) {
		if (same_conn) {
			job_failed(api, ret);
		}
	} else if (job.disable) {
		/* The plaintext rewrite is on the workqueue behind us; DISABLED and
		 * the reboot follow from its event. Authorized meanwhile, as the
		 * reference does. */
		if (same_conn) {
			authorize(api);
		}
		disable_by_phone = true;
		LOG_INF("lockdown: disable authorized, rewriting the store in the clear");
	} else if (!meshtastic_lockdown_store_ready()) {
		/* A cold unlock: the reload is queued behind us. No status yet --
		 * the client keeps seeing LOCKED until EV_RELOADED. */
		if (same_conn) {
			api->pending_unlock = true;
		}
		LOG_INF("lockdown: storage unlocked, awaiting the reload before client visibility");
	} else {
		/* Provision, or a re-verify on an already-unlocked node: authorize now. */
		if (same_conn) {
			authorize(api);
			queue_status(api, meshtastic_LockdownStatus_State_UNLOCKED, NULL,
				     meshtastic_lockdown_boots_remaining(),
				     meshtastic_lockdown_valid_until(), 0U);
		}
		LOG_INF("lockdown: passphrase verified, connection authorized");
	}

	job.api = NULL;
	atomic_clear(&job_busy);
}

/* ---- the transport-thread half ---------------------------------------------------- */

/* Is this admin payload a lockdown_auth? Walk the top-level tags rather than
 * decode the whole AdminMessage: that struct is the largest union in the
 * schema and this runs on the transport's thread for every admin packet the
 * phone sends, active or not. Returns 1 with @p out filled, 0 if not one,
 * <0 if the payload does not parse. */
static int peek_lockdown_auth(const uint8_t *buf, size_t len, meshtastic_LockdownAuth *out)
{
	pb_istream_t is = pb_istream_from_buffer(buf, len);

	while (is.bytes_left > 0U) {
		pb_wire_type_t wt;
		uint32_t tag;
		bool eof;

		if (!pb_decode_tag(&is, &wt, &tag, &eof)) {
			return eof ? 0 : -EBADMSG;
		}
		if (tag == meshtastic_AdminMessage_lockdown_auth_tag && wt == PB_WT_STRING) {
			pb_istream_t sub;
			bool ok;

			if (!pb_make_string_substream(&is, &sub)) {
				return -EBADMSG;
			}
			*out = (meshtastic_LockdownAuth)meshtastic_LockdownAuth_init_zero;
			ok = pb_decode(&sub, meshtastic_LockdownAuth_fields, out);
			(void)pb_close_string_substream(&is, &sub);
			return ok ? 1 : -EBADMSG;
		}
		if (!pb_skip_field(&is, wt)) {
			return -EBADMSG;
		}
	}
	return 0;
}

/* The reference's handleLockdownAuthInline, minus the verification, which is
 * queued. Everything decided here needs no key material. */
static void handle_auth(struct meshtastic_phoneapi *api, const meshtastic_LockdownAuth *la)
{
	if (la->lock_now) {
		/* Only from a connection that has proven the passphrase: the
		 * reference's local-presence DoS fix (any BLE/USB attacker could
		 * otherwise reboot-loop the node). */
		if (!meshtastic_phoneapi_authorized(api) || !meshtastic_lockdown_active()) {
			LOG_WRN("%s lockdown: LOCK NOW refused (unauthorized or inactive)", api->name);
			queue_status(api, meshtastic_LockdownStatus_State_UNLOCK_FAILED, NULL, 0U,
				     0U, 0U);
			return;
		}
		LOG_INF("%s lockdown: LOCK NOW", api->name);
		meshtastic_lockdown_lock_now(); /* EV_LOCKED revokes and broadcasts */
		schedule_reboot("lock now");
		return;
	}
	if (la->disable && !meshtastic_lockdown_active()) {
		/* Already off: say so, so the app's toggle settles. */
		queue_status(api, meshtastic_LockdownStatus_State_DISABLED, NULL, 0U, 0U, 0U);
		return;
	}
	if (la->passphrase.size == 0U ||
	    la->passphrase.size > MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX) {
		/* An empty passphrase used to be a silent no-op in the reference;
		 * it now fails visibly so an honest client can find its own bug. */
		LOG_WRN("%s lockdown: lockdown_auth with a %u-byte passphrase rejected", api->name,
			(unsigned int)la->passphrase.size);
		queue_status(api, meshtastic_LockdownStatus_State_UNLOCK_FAILED, NULL, 0U, 0U, 0U);
		return;
	}
	if (la->boots_remaining > 255U) {
		/* uint32 on the wire, uint8 in the token: 256 would truncate to
		 * "use the default", hiding a client bug. */
		LOG_WRN("%s lockdown: boots_remaining=%u exceeds the token's cap", api->name,
			(unsigned int)la->boots_remaining);
		queue_status(api, meshtastic_LockdownStatus_State_UNLOCK_FAILED, NULL, 0U, 0U, 0U);
		return;
	}
	if (!atomic_cas(&job_busy, 0, 1)) {
		LOG_WRN("%s lockdown: a passphrase check is already running", api->name);
		queue_status(api, meshtastic_LockdownStatus_State_UNLOCK_FAILED, NULL, 0U, 0U, 0U);
		return;
	}
	job.api = api;
	job.conn_epoch = api->conn_epoch;
	memcpy(job.passphrase, la->passphrase.bytes, la->passphrase.size);
	job.len = (uint8_t)la->passphrase.size;
	job.boots = (uint8_t)la->boots_remaining;
	job.valid_until = la->valid_until_epoch;
	job.session_s = la->max_session_seconds;
	job.disable = la->disable;
	k_work_submit(&auth_work);
}

bool meshtastic_phoneapi_lockdown_gate(struct meshtastic_phoneapi *api, meshtastic_MeshPacket *pkt,
				       int *res)
{
	/* An admin packet to us may carry the passphrase, whatever the state:
	 * provisioning happens while inactive, unlocking while locked, and a
	 * re-verify while unlocked. Decided on the connection, never on the
	 * wire `from`, which a client can forge. */
	if (pkt->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
	    pkt->decoded.portnum == meshtastic_PortNum_ADMIN_APP &&
	    pkt->to == meshtastic_get_node_id()) {
		meshtastic_LockdownAuth la;
		int found = peek_lockdown_auth(pkt->decoded.payload.bytes, pkt->decoded.payload.size,
					       &la);

		if (found > 0) {
			handle_auth(api, &la);
			/* Nothing else clears the passphrase from the decoded scratch:
			 * the copy in `la`, and the encoded bytes in the packet. */
			zero(&la, sizeof(la));
			zero(pkt->decoded.payload.bytes, sizeof(pkt->decoded.payload.bytes));
			pkt->decoded.payload.size = 0U;
			*res = 0;
			return true;
		}
	}
	if (meshtastic_phoneapi_authorized(api)) {
		return false;
	}
	/* Unauthorized under active lockdown: no mesh injection, no admin. The
	 * reference drops without a word to the client; this port reports the
	 * refusal in the QueueStatus, which is what it does for every other
	 * refused send, so the app's StreamAPI stays in step. */
	LOG_INF("%s lockdown: dropping ToRadio packet (port %u) from an unauthorized client",
		api->name,
		pkt->which_payload_variant == meshtastic_MeshPacket_decoded_tag
			? (unsigned int)pkt->decoded.portnum
			: 0U);
	*res = -EACCES;
	return true;
}
