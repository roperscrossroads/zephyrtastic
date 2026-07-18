/* SPDX-License-Identifier: GPL-3.0
 *
 * Sender-side reliable delivery. See meshtastic_reliable.h.
 *
 * A want_ack unicast packet this node originates is copied (wire bytes) into a
 * pending table and retransmitted every reliable.timeout ms, up to
 * reliable.retries times. Retransmission stops on:
 *   - an explicit ROUTING ACK (error_reason NONE) addressed to us whose
 *     request_id matches the pending id  -> delivered,
 *   - a ROUTING NAK (any other error_reason)                 -> failed-by-peer,
 *   - an implicit ACK: hearing a neighbour rebroadcast our packet (wire src is
 *     our node id) -> the mesh has taken it, stop retransmitting,
 *   - exhausting the retry budget -> emit MESHTASTIC_EVENT_TX_FAILED and hand
 *     the connected app a MAX_RETRANSMIT routing error for that packet id.
 *
 * Retransmits reuse the stored wire bytes, so the packet id and ciphertext are
 * unchanged and other nodes dedup them correctly.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/mesh.pb.h"

#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_reliable.h"
#include "meshtastic_sched.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define PEND_MAX CONFIG_MESHTASTIC_RELIABLE_MAX_PENDING

struct pending {
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint16_t wire_len;
	uint32_t id;
	uint32_t to;
	int64_t next_due; /* k_uptime_get() ms of the next retransmit */
	uint8_t retries_left;
	uint8_t tier;
	bool active;
};

static struct pending pend[PEND_MAX];
static K_MUTEX_DEFINE(pend_lock);

static void retx_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(retx_work, retx_work_fn);

/* Reschedule the timer to the soonest pending due time, or cancel it if the
 * table is empty. Caller holds pend_lock. */
static void reschedule_locked(int64_t now)
{
	int64_t soonest = INT64_MAX;

	for (int i = 0; i < PEND_MAX; i++) {
		if (pend[i].active && pend[i].next_due < soonest) {
			soonest = pend[i].next_due;
		}
	}

	if (soonest == INT64_MAX) {
		(void)k_work_cancel_delayable(&retx_work);
		return;
	}

	(void)k_work_reschedule(&retx_work, K_MSEC(MAX((int64_t)0, soonest - now)));
}

/* Hand the connected app a synthetic ROUTING error so a failed send surfaces as
 * a delivery failure, and emit a TX-failed event. Called without pend_lock. */
static void notify_failure(uint32_t id, uint32_t to)
{
	meshtastic_sched_stat_reliable_fail();
	LOG_WRN("reliable: id=0x%08x to 0x%08x exhausted retries", id, to);

	/* The learned next hop toward this destination just failed to deliver
	 * despite retransmits: strike its route health (three strikes decay the
	 * route to flood so the next send rediscovers a working path — M4,
	 * upstream RouteHealth). Note this fires only on retransmit exhaustion,
	 * not on a NAK — a NAK means the peer received it, so its route is fine. */
	meshtastic_nodedb_note_route_failure(to);

	meshtastic_emit_event(MESHTASTIC_EVENT_TX_FAILED, -ETIMEDOUT,
			      &(struct meshtastic_packet){
				      .from = mt.node_id,
				      .to = to,
				      .id = id,
			      });

#if defined(CONFIG_MESHTASTIC_PHONEAPI)
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	uint8_t buf[16];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	routing.which_variant = meshtastic_Routing_error_reason_tag;
	routing.error_reason = meshtastic_Routing_Error_MAX_RETRANSMIT;
	if (!pb_encode(&os, meshtastic_Routing_fields, &routing)) {
		return;
	}

	/* Presented as if the unreachable destination reported the failure. */
	meshtastic_phoneapi_on_packet(&(struct meshtastic_packet){
		.from = to,
		.to = mt.node_id,
		.portnum = MESHTASTIC_PORT_ROUTING,
		.request_id = id,
		.payload = buf,
		.payload_len = os.bytes_written,
		.channel_index = meshtastic_channels_primary_index(),
	});
#endif
}

/* Find the active slot tracking @p id, or -1. Caller holds pend_lock. */
static int find_locked(uint32_t id)
{
	for (int i = 0; i < PEND_MAX; i++) {
		if (pend[i].active && pend[i].id == id) {
			return i;
		}
	}
	return -1;
}

/* Resolve (stop tracking) the send for @p id. Caller holds pend_lock.
 * @return true if a pending entry was found and cleared. */
static bool resolve_locked(uint32_t id)
{
	int i = find_locked(id);

	if (i < 0) {
		return false;
	}
	pend[i].active = false;
	return true;
}

void meshtastic_reliable_reset(void)
{
	k_mutex_lock(&pend_lock, K_FOREVER);
	for (int i = 0; i < PEND_MAX; i++) {
		pend[i].active = false;
	}
	(void)k_work_cancel_delayable(&retx_work);
	k_mutex_unlock(&pend_lock);
}

void meshtastic_reliable_on_tx(const struct meshtastic_packet *local, const uint8_t *wire,
			       uint32_t wire_len)
{
	struct meshtastic_sched_config c;
	int64_t now;
	int slot;

	if (local == NULL || wire == NULL || wire_len == 0U || wire_len > MESHTASTIC_PKT_MAX) {
		return;
	}

	/* Only our-origin, want_ack, unicast application packets. ROUTING control
	 * packets (e.g. the ACKs we send) are not themselves retransmitted. */
	if (!local->want_ack || local->from != mt.node_id ||
	    local->to == MESHTASTIC_NODE_BROADCAST || local->to == mt.node_id ||
	    local->portnum == MESHTASTIC_PORT_ROUTING) {
		return;
	}

	meshtastic_sched_snapshot(&c);
	if (c.reliable_retries == 0U) {
		return; /* feature disabled at runtime */
	}

	now = k_uptime_get();

	k_mutex_lock(&pend_lock, K_FOREVER);

	/* Already tracking this id (e.g. a resend through send_packet)? Leave it. */
	if (find_locked(local->id) >= 0) {
		k_mutex_unlock(&pend_lock);
		return;
	}

	slot = -1;
	for (int i = 0; i < PEND_MAX; i++) {
		if (!pend[i].active) {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		/* Table full: drop the oldest tracker (soonest due) to best-effort. */
		int64_t soonest = INT64_MAX;

		slot = 0;
		for (int i = 0; i < PEND_MAX; i++) {
			if (pend[i].next_due < soonest) {
				soonest = pend[i].next_due;
				slot = i;
			}
		}
		LOG_WRN("reliable: table full, evicting id=0x%08x", pend[slot].id);
	}

	memcpy(pend[slot].wire, wire, wire_len);
	pend[slot].wire_len = (uint16_t)wire_len;
	pend[slot].id = local->id;
	pend[slot].to = local->to;
	pend[slot].tier = meshtastic_sched_tier_for(local->portnum);
	pend[slot].retries_left = c.reliable_retries;
	pend[slot].next_due = now + (int64_t)c.reliable_timeout_ms;
	pend[slot].active = true;

	LOG_DBG("reliable: track id=0x%08x to 0x%08x retries=%u", local->id, local->to,
		c.reliable_retries);

	reschedule_locked(now);
	k_mutex_unlock(&pend_lock);
}

void meshtastic_reliable_on_routing(const struct meshtastic_packet *routing)
{
	meshtastic_Routing decoded = meshtastic_Routing_init_zero;
	pb_istream_t is;
	bool is_nak;

	if (routing == NULL || routing->to != mt.node_id || routing->request_id == 0U) {
		return;
	}

	is = pb_istream_from_buffer(routing->payload, routing->payload_len);
	if (!pb_decode(&is, meshtastic_Routing_fields, &decoded)) {
		return; /* not a Routing payload we can read */
	}

	is_nak = (decoded.which_variant == meshtastic_Routing_error_reason_tag) &&
		 (decoded.error_reason != meshtastic_Routing_Error_NONE);

	k_mutex_lock(&pend_lock, K_FOREVER);
	if (!resolve_locked(routing->request_id)) {
		k_mutex_unlock(&pend_lock);
		return; /* not one of ours (or already resolved) */
	}
	reschedule_locked(k_uptime_get());
	k_mutex_unlock(&pend_lock);

	/* Either way the peer answered, so the route that carried the exchange
	 * works: reset its failure strikes (M4). A NAK is a delivery failure for
	 * the app, not for the route. */
	meshtastic_nodedb_note_route_success(routing->from);

	if (is_nak) {
		meshtastic_sched_stat_reliable_fail();
		LOG_INF("reliable: id=0x%08x NAK (err=%d)", routing->request_id,
			(int)decoded.error_reason);
	} else {
		meshtastic_sched_stat_reliable_ack();
		LOG_INF("reliable: id=0x%08x delivered (explicit ack)", routing->request_id);
	}
}

void meshtastic_reliable_on_implicit_ack(uint32_t id)
{
	bool resolved;

	k_mutex_lock(&pend_lock, K_FOREVER);
	resolved = resolve_locked(id);
	if (resolved) {
		reschedule_locked(k_uptime_get());
	}
	k_mutex_unlock(&pend_lock);

	if (resolved) {
		meshtastic_sched_stat_reliable_ack();
		LOG_DBG("reliable: id=0x%08x implicit ack (heard rebroadcast)", id);
	}
}

static void retx_work_fn(struct k_work *work)
{
	struct {
		uint32_t id;
		uint32_t to;
	} failed[PEND_MAX];
	int nfailed = 0;
	struct meshtastic_sched_config c;
	int64_t now = k_uptime_get();

	ARG_UNUSED(work);

	meshtastic_sched_snapshot(&c);

	k_mutex_lock(&pend_lock, K_FOREVER);
	for (int i = 0; i < PEND_MAX; i++) {
		if (!pend[i].active || now < pend[i].next_due) {
			continue;
		}

		if (pend[i].retries_left == 0U) {
			failed[nfailed].id = pend[i].id;
			failed[nfailed].to = pend[i].to;
			nfailed++;
			pend[i].active = false;
			continue;
		}

		pend[i].retries_left--;
		pend[i].next_due = now + (int64_t)c.reliable_timeout_ms;
		LOG_DBG("reliable: retransmit id=0x%08x (%u left)", pend[i].id,
			pend[i].retries_left);
		(void)meshtastic_radio_send_wire_prio(pend[i].wire, pend[i].wire_len, pend[i].tier);
	}
	reschedule_locked(now);
	k_mutex_unlock(&pend_lock);

	for (int i = 0; i < nfailed; i++) {
		notify_failure(failed[i].id, failed[i].to);
	}
}
