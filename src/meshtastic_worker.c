/* SPDX-License-Identifier: GPL-3.0 */

/*
 * A keyless worker (ATTACHMENT-DESIGN S5). See meshtastic_worker.h.
 *
 * The shape is the rx-only worker's (meshtastic_worker_rx.c): the RX thread queues,
 * a work item on the BLE module's queue forwards, because a GATT write can
 * block on buffers and the RX thread must not. Unlike the rx-only worker, this hooks
 * BEFORE the router -- the frame is never decoded here, there is nothing to
 * decode it with.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/meshtastic/intents.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_link_codec.h"
#include "meshtastic_worker.h"
#include "meshtastic_outbound.h"
#include "meshtastic_contention.h"
#include "meshtastic_duty.h"
#if defined(CONFIG_MESHTASTIC_AIRTIME)
#include "meshtastic_airtime.h"
#endif
#include "meshtastic_core.h"
#include "meshtastic_config_store.h"
#include "meshtastic_preset.h"
#include "meshtastic_packet.h"
#include <zephyr/sys/byteorder.h>
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include "meshtastic_ble_peer.h" /* meshtastic_ble_work_submit only */
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

struct worker_frame {
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t preset;
	uint32_t rx_ms;
	uint8_t wire[MESHTASTIC_PKT_MAX];
};

static struct {
	uint32_t hub;
	struct meshtastic_worker_stats stats;
} worker = {
	.hub = CONFIG_MESHTASTIC_WORKER_HUB_ID,
};

/* A frame the hub handed us to transmit (P3 slice 1): the bytes as built by
 * the hub, untouched -- a worker never builds a frame, it keys one up. */
struct worker_tx {
	uint16_t len;
	uint16_t tx_seq;
	uint8_t flags;
	uint8_t defers;
	/* A relay (TXF_RELAY): what it is a relay of, and the rule if we hear
	 * that frame again before we key up. */
	uint32_t relay_src;
	uint32_t relay_id;
	uint32_t orig_rx_ms;
	uint8_t dupe;
	int8_t snr;
	/* k_uptime_get_32() before which this frame must not key up: the own-TX
	 * contention window (OWN_DELAY), drawn HERE with this radio's modem --
	 * the hub's clock and modem are not this radio's. 0 = now. */
	uint32_t not_before;
	/* k_uptime_get_32() when the TX_FRAME arrived: with orig_rx_ms, the peer
	 * link's cost for a relay, on this clock alone (DESIGN §13). */
	uint32_t arrived_ms;
	uint8_t wire[MESHTASTIC_PKT_MAX];
};

/* Lock order (agents-z7rg): worker_lock is never held across a send to the hub.
 * A send enters the bearer (its registry lock) and the BLE peer module (the peer
 * lock, then a GATT write that can wait for controller buffers), and the BLE RX
 * path delivers the hub's envelopes into this file, which takes worker_lock: a
 * send under worker_lock closed an AB-BA with the BT RX thread (rzr5, 2026-11-24:
 * "mutex deadlock", thread worker_tx). Build the envelope under the lock, release
 * it, then send. */
static K_MUTEX_DEFINE(worker_lock);
K_MSGQ_DEFINE(worker_q, sizeof(struct worker_frame), CONFIG_MESHTASTIC_WORKER_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(worker_tx_q, sizeof(struct worker_tx), CONFIG_MESHTASTIC_WORKER_TX_QUEUE_SIZE, 4);
/* The transmit thread (P3 slice 1/2): it sleeps through the contention window
 * and blocks for the airtime, so it is nobody else's queue. */
static K_THREAD_STACK_DEFINE(worker_tx_stack, CONFIG_MESHTASTIC_WORKER_TX_STACK_SIZE);
static struct k_thread worker_tx_thread;
static uint8_t tx_env_buf[MESHTASTIC_LINK_ENV_MAX]; /* the TX thread's own scratch */

/* The heard-history ring (ATTACHMENT-DESIGN §12): every frame this radio hears,
 * by (src, id), with when and how often -- a keyless worker can read the
 * plaintext header. A relay the hub handed us for (src, id) is cancelled or
 * clamped if the ring says we heard that frame AGAIN after the copy the relay
 * is of: someone else relayed it first. */
struct heard {
	uint32_t src;
	uint32_t id;
	uint32_t first_ms;
	uint32_t last_ms;
	int8_t snr;
	uint8_t count;
};
#define HEARD_RING 16U
static struct heard heard_ring[HEARD_RING];
static uint8_t heard_next;

/* Relays the hub withdrew (TX_CANCEL) that may still sit in the TX queue. */
struct cancelled {
	uint32_t src;
	uint32_t id;
	bool set;
};
static struct cancelled cancels[4];

/* The relay the TX thread is waiting to key up, so the RX path can act on a
 * duplicate THE MOMENT it is heard (X6e): the reference clamps ROUTER_LATE to
 * tx_after = now + worst when the copy lands (clampToLateRebroadcastWindow),
 * and a CLIENT drops there and then. Published under worker_lock by the TX
 * thread while it waits; cleared when it stops waiting. */
static struct {
	uint32_t src;
	uint32_t id;
	uint32_t orig_rx_ms;
	uint32_t not_before;
	uint8_t dupe;
	int8_t snr;
	bool set;
	bool clamped;
	bool cancel;
} pending;
static K_SEM_DEFINE(worker_tx_wake, 0, 1);

static uint32_t relay_worst_ms(int8_t snr)
{
	return meshtastic_contention_delay_relay_worst_ms(
		snr, meshtastic_contention_effective_slot_ms(mt.modem.spread_factor,
							     mt.modem.bandwidth_hz, false));
}

static void heard_note_locked(const uint8_t *wire, uint16_t len, int8_t snr, uint32_t rx_ms)
{
	const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)wire;
	uint32_t src;
	uint32_t id;

	if (len < MESHTASTIC_HDR_LEN) {
		return;
	}
	src = sys_le32_to_cpu(h->src);
	id = sys_le32_to_cpu(h->id);
	for (unsigned int i = 0U; i < HEARD_RING; i++) {
		if (heard_ring[i].count != 0U && heard_ring[i].src == src && heard_ring[i].id == id) {
			heard_ring[i].last_ms = rx_ms;
			if (heard_ring[i].count < 255U) {
				heard_ring[i].count++;
			}
			/* A peer's copy of the relay we are waiting to send: decide NOW,
			 * on this copy's reception time, and wake the TX thread. */
			if (pending.set && pending.src == src && pending.id == id &&
			    (int32_t)(rx_ms - pending.orig_rx_ms) > 0) {
				if (pending.dupe == MESHTASTIC_LINK_DUPE_CANCEL) {
					pending.cancel = true;
					k_sem_give(&worker_tx_wake);
				} else if (pending.dupe == MESHTASTIC_LINK_DUPE_LATE &&
					   !pending.clamped) {
					pending.not_before = rx_ms + relay_worst_ms(pending.snr);
					pending.clamped = true;
					k_sem_give(&worker_tx_wake);
				}
			}
			return;
		}
	}
	heard_ring[heard_next] = (struct heard){
		.src = src, .id = id, .first_ms = rx_ms, .last_ms = rx_ms, .snr = snr, .count = 1U,
	};
	heard_next = (uint8_t)((heard_next + 1U) % HEARD_RING);
}

/* Did we hear (src, id) again after @p since_ms? */
static bool heard_again_locked(uint32_t src, uint32_t id, uint32_t since_ms)
{
	for (unsigned int i = 0U; i < HEARD_RING; i++) {
		if (heard_ring[i].count > 1U && heard_ring[i].src == src && heard_ring[i].id == id &&
		    (int32_t)(heard_ring[i].last_ms - since_ms) > 0) {
			return true;
		}
	}
	return false;
}

static bool cancelled_take_locked(uint32_t src, uint32_t id)
{
	for (unsigned int i = 0U; i < ARRAY_SIZE(cancels); i++) {
		if (cancels[i].set && cancels[i].src == src && cancels[i].id == id) {
			cancels[i].set = false;
			return true;
		}
	}
	return false;
}

static void worker_work_fn(struct k_work *work);
static K_WORK_DEFINE(worker_work, worker_work_fn);
static void worker_dial_hub(uint32_t hub);
/* The STATUS timer ticks on the system queue; the send itself runs where the
 * GATT write may block (worker_submit), like every other envelope. */
static void worker_status_fn(struct k_work *work);
static K_WORK_DEFINE(worker_status_work, worker_status_fn);
static void worker_status_tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(worker_status_timer, worker_status_tick);

/* The send seam: whichever registered bearer has a live link to the hub. */
__weak int meshtastic_worker_send(uint32_t hub, const uint8_t *env, size_t len)
{
	return meshtastic_link_bearer_send(hub, env, len);
}

static void worker_submit(struct k_work *work)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	(void)meshtastic_ble_work_submit(work);
#else
	/* No BLE stack (native_sim): nothing on this queue can block on GATT. */
	(void)k_work_submit(work);
#endif
}

/* The forwarding worker's envelope scratch (one worker, one queue). STATUS is
 * small and built on the sender's stack, since it is sent from several threads. */
static uint8_t env_buf[MESHTASTIC_LINK_ENV_MAX];

#define STATUS_ENV_MAX MESHTASTIC_LINK_STATUS_POS_LEN

/* This worker's STATUS for @p hub, into @p buf. Caller holds worker_lock. */
static int status_build_locked(uint32_t hub, uint8_t *buf, size_t size)
{
	struct meshtastic_link_status st = {
		.preset = (uint8_t)mt.modem_preset,
		/* RX_ONLY only when transmit is compiled out of this image: since P3
		 * a worker keys up what its hub hands it, and the hub refuses to
		 * hand anything to a worker that says receive-only. */
#if defined(CONFIG_MESHTASTIC_SCANNER_RX_ONLY) || defined(CONFIG_MESHTASTIC_WORKER_RX_RX_ONLY)
		.flags = MESHTASTIC_LINK_ST_IS_WORKER | MESHTASTIC_LINK_ST_RX_ONLY,
#else
		.flags = MESHTASTIC_LINK_ST_IS_WORKER,
#endif
		.hwid = mt.node_id,
		.hub = hub,
		.rx_frames = worker.stats.forwarded,
		.tx_frames = worker.stats.tx_sent,
		.uptime_s = (uint32_t)(k_uptime_get() / 1000),
		.rx_dropped = worker.stats.queue_full + worker.stats.send_failed,
	};
	int len;

	if (mt.tx_enabled) {
		st.flags |= MESHTASTIC_LINK_ST_TX_ENABLED;
	}
	if (mt.radio_held) {
		st.flags |= MESHTASTIC_LINK_ST_RADIO_HELD;
	}
	len = meshtastic_link_encode_status(&st, buf, size);
	return len;
}

/* Build a STATUS under worker_lock, send it without (see the lock order above). */
static int status_send(uint32_t hub)
{
	uint8_t buf[STATUS_ENV_MAX];
	int len;
	int ret;

	k_mutex_lock(&worker_lock, K_FOREVER);
	len = status_build_locked(hub, buf, sizeof(buf));
	k_mutex_unlock(&worker_lock);
	if (len < 0) {
		return len;
	}
	ret = meshtastic_worker_send(hub, buf, (size_t)len);
	if (ret == 0) {
		k_mutex_lock(&worker_lock, K_FOREVER);
		worker.stats.status_sent++;
		k_mutex_unlock(&worker_lock);
	}
	return ret;
}

/* Transmit what the hub handed us: the one funnel every transmit takes
 * (meshtastic_radio_send_wire_now: CAD/LBT, the scanner gate, tx_enabled,
 * radio_held), so a worker keys up under exactly the rules its own image would.
 * An own frame (OWN_DELAY) waits its contention window first; DEFER is
 * retried, bounded as the hub's own worker bounds it. On its own thread:
 * the radio blocks for the airtime, and the forward path must keep draining. */
static void worker_tx_thread_fn(void *p1, void *p2, void *p3)
{
	struct worker_tx t;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		int ret;
		int elen;
		uint32_t result_to;

		bool is_relay;
		bool clamped = false;
		bool heard_cancel = false;

		(void)k_msgq_get(&worker_tx_q, &t, K_FOREVER);
		is_relay = (t.flags & MESHTASTIC_LINK_TXF_RELAY) != 0U;
		/* D3 as revised (ATTACHMENT-SCOPE §6, tooling 7d5f2a7): a CLIENT relay
		 * whose window has already closed when the decision reaches us applies
		 * its own cancel test, not a blind drop -- the worker KNOWS whether a peer
		 * relayed the frame. A copy heard since the original: dropped, the
		 * ordinary cancel-on-dupe (the check after the wait). No copy: send now,
		 * after CAD, as the on-time CLIENT would have (the wait ends at once).
		 * Past orig_rx + 2 * worst(snr): stale, dropped (-ETIME, late_dropped).
		 * Never promoted to ROUTER_LATE (relaying despite a heard copy).
		 * ROUTER / ROUTER_LATE unchanged: they send late. */
		if (is_relay && t.dupe == MESHTASTIC_LINK_DUPE_CANCEL && t.not_before != 0U &&
		    (int32_t)(t.not_before - k_uptime_get_32()) < 0 &&
		    (int32_t)(k_uptime_get_32() - (t.orig_rx_ms + 2U * relay_worst_ms(t.snr))) > 0) {
			k_mutex_lock(&worker_lock, K_FOREVER);
			worker.stats.tx_late_dropped++;
			k_mutex_unlock(&worker_lock);
			ret = -ETIME;
			goto report;
		}
		if (t.not_before != 0U) {
			/* Wait out the window on this radio's clock -- wakeable: a copy of
			 * a pending relay heard meanwhile cancels it or moves the deadline
			 * (heard_note_locked), which the old k_msleep could not see. */
			k_mutex_lock(&worker_lock, K_FOREVER);
			if (is_relay) {
				pending.src = t.relay_src;
				pending.id = t.relay_id;
				pending.orig_rx_ms = t.orig_rx_ms;
				pending.not_before = t.not_before;
				pending.dupe = t.dupe;
				pending.snr = t.snr;
				pending.clamped = false;
				pending.cancel = false;
				pending.set = true;
				k_sem_reset(&worker_tx_wake);
			}
			k_mutex_unlock(&worker_lock);
			for (;;) {
				uint32_t deadline = t.not_before;
				int32_t left;

				k_mutex_lock(&worker_lock, K_FOREVER);
				if (is_relay) {
					deadline = pending.not_before;
					heard_cancel = pending.cancel;
				}
				k_mutex_unlock(&worker_lock);
				left = (int32_t)(deadline - k_uptime_get_32());
				if (heard_cancel || left <= 0) {
					break;
				}
				(void)k_sem_take(&worker_tx_wake, K_MSEC(left));
			}
			k_mutex_lock(&worker_lock, K_FOREVER);
			if (is_relay) {
				clamped = pending.clamped;
				heard_cancel = pending.cancel;
				pending.set = false;
			}
			k_mutex_unlock(&worker_lock);
		}
		if (is_relay) {
			/* The reference's cancel-on-duplicate / late clamp, on THIS
			 * radio's hearing and clock (ATTACHMENT-DESIGN §12): a copy we
			 * heard after the one this relay is of means a peer relayed it
			 * first. The hub's own withdrawal (a copy that reached it by
			 * another radio) arrives as TX_CANCEL. */
			bool withdrawn;
			bool again;

			k_mutex_lock(&worker_lock, K_FOREVER);
			withdrawn = cancelled_take_locked(t.relay_src, t.relay_id);
			again = heard_again_locked(t.relay_src, t.relay_id, t.orig_rx_ms);
			k_mutex_unlock(&worker_lock);
			if (withdrawn || heard_cancel ||
			    (again && t.dupe == MESHTASTIC_LINK_DUPE_CANCEL)) {
				k_mutex_lock(&worker_lock, K_FOREVER);
				worker.stats.tx_cancelled++;
				k_mutex_unlock(&worker_lock);
				ret = -ECANCELED;
				goto report;
			}
			if (clamped) {
				/* Already waited to dup_rx + worst (X6e): count it, send. */
				k_mutex_lock(&worker_lock, K_FOREVER);
				worker.stats.tx_late++;
				k_mutex_unlock(&worker_lock);
			} else if (again && t.dupe == MESHTASTIC_LINK_DUPE_LATE) {
				/* A copy heard before the wait was published: the old rule,
				 * to the end of the worst-case window from now. */
				/* To the end of the worst-case window, from now, with this
				 * modem and the SNR we heard the original at. */
				uint32_t late = relay_worst_ms(t.snr);

				k_mutex_lock(&worker_lock, K_FOREVER);
				worker.stats.tx_late++;
				k_mutex_unlock(&worker_lock);
				k_msleep(late);
				k_mutex_lock(&worker_lock, K_FOREVER);
				withdrawn = cancelled_take_locked(t.relay_src, t.relay_id);
				k_mutex_unlock(&worker_lock);
				if (withdrawn) {
					ret = -ECANCELED;
					goto report;
				}
			}
		}
		{
			/* The regulatory gate, at THIS radio (ATTACHMENT-SCOPE A4): the
			 * hub's duty ledger prices its own channel, not this one.
			 * A stub (false) on an image without the airtime ledger. */
			uint8_t silent = 0U;

			if (meshtastic_duty_blocked(&silent)) {
				k_mutex_lock(&worker_lock, K_FOREVER);
				worker.stats.tx_duty_blocked++;
				k_mutex_unlock(&worker_lock);
				LOG_WRN("worker: tx seq %u refused by the duty gate (%u min silent)",
					t.tx_seq, silent);
				ret = -ECANCELED;
				goto report;
			}
		}
		ret = meshtastic_radio_send_wire_now(t.wire, t.len);
		while (ret == MESHTASTIC_TX_DEFER && t.defers < CONFIG_MESHTASTIC_TX_DEFER_MAX) {
			t.defers++;
			k_mutex_lock(&worker_lock, K_FOREVER);
			worker.stats.tx_deferred++;
			k_mutex_unlock(&worker_lock);
			k_msleep(20);
			if (is_relay) {
				/* The channel was busy at key-up -- and the commonest reason
				 * on air is a peer relaying THIS frame. The reference keeps a
				 * deferred packet in its queue, where a duplicate heard
				 * meanwhile still cancels it (perhapsCancelDupe); without this
				 * re-check a CLIENT worker waited out the peer's relay and then
				 * sent a second one (bench, 2026-09-30: 3 of 24 floods). */
				bool withdrawn;
				bool again;

				k_mutex_lock(&worker_lock, K_FOREVER);
				withdrawn = cancelled_take_locked(t.relay_src, t.relay_id);
				again = heard_again_locked(t.relay_src, t.relay_id, t.orig_rx_ms);
				if (withdrawn || (again && t.dupe == MESHTASTIC_LINK_DUPE_CANCEL)) {
					worker.stats.tx_cancelled++;
					k_mutex_unlock(&worker_lock);
					ret = -ECANCELED;
					break;
				}
				k_mutex_unlock(&worker_lock);
			}
			ret = meshtastic_radio_send_wire_now(t.wire, t.len);
		}

report:
		result_to = 0U;
		elen = 0;
		k_mutex_lock(&worker_lock, K_FOREVER);
		if (ret == 0) {
			worker.stats.tx_sent++;
		} else if (ret != -ECANCELED) {
			worker.stats.tx_failed++;
		}
		if ((t.flags & MESHTASTIC_LINK_TXF_WANT_RESULT) != 0U && worker.hub != 0U) {
			const struct meshtastic_link_tx_result r = {
				.tx_seq = t.tx_seq,
				.rc = ret == -ECANCELED ? MESHTASTIC_LINK_RC_CANCELLED
							: (int8_t)CLAMP(ret, -127, 127),
				.defers = t.defers,
				.tx_ms = (uint32_t)k_uptime_get(),
				.has_lag = is_relay,
				.lag_ms = is_relay ? (uint16_t)MIN(t.arrived_ms - t.orig_rx_ms, 0xFFFEU)
						   : 0U,
			};
			elen = meshtastic_link_encode_tx_result(&r, tx_env_buf,
								      sizeof(tx_env_buf));
			result_to = worker.hub;
		}
		k_mutex_unlock(&worker_lock);
		if (elen > 0) {
			(void)meshtastic_worker_send(result_to, tx_env_buf, (size_t)elen);
		}
		if (ret != 0) {
			LOG_DBG("worker: tx seq %u failed (%d)", t.tx_seq, ret);
		}
	}
}

static void worker_work_fn(struct k_work *work)
{
	static struct worker_frame f; /* one worker: static keeps it off the stack */

	ARG_UNUSED(work);

	while (k_msgq_get(&worker_q, &f, K_NO_WAIT) == 0) {
		const struct meshtastic_link_rx_frame m = {
			.preset = f.preset,
			.rssi = f.rssi,
			.snr = f.snr,
			.rx_ms = f.rx_ms,
			.flags = 0U,
			.wire = f.wire,
			.wire_len = f.len,
		};
		uint32_t hub;
		int ret;

		hub = meshtastic_worker_get_hub();
		if (hub == 0U) {
			ret = -EHOSTUNREACH;
		} else {
			ret = meshtastic_link_encode_rx_frame(&m, env_buf, sizeof(env_buf));
			if (ret > 0) {
				ret = meshtastic_worker_send(hub, env_buf, (size_t)ret);
			}
		}
		k_mutex_lock(&worker_lock, K_FOREVER);
		if (ret == 0) {
			worker.stats.forwarded++;
		} else {
			worker.stats.send_failed++;
		}
		k_mutex_unlock(&worker_lock);
		if (ret != 0) {
			LOG_DBG("worker: forward to 0x%08x failed (%d)", hub, ret);
		}
	}
}

static void worker_status_fn(struct k_work *work)
{
	uint32_t hub = meshtastic_worker_get_hub();

	ARG_UNUSED(work);

	if (hub != 0U) {
		(void)status_send(hub);
	}
}

/* Is the peer link up to @p hub? If it is up to someone else (a target the
 * link restored from its node days, a stale intent), drop it; if it is not up,
 * dial. The tick asks this every period, so a worker converges on its hub
 * without an operator (ATTACHMENT-SCOPE C7; bench 2026-09-28: kit1 booted
 * scanning for the node it last dialled as a node). */
static void worker_assert_dial(uint32_t hub)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	struct meshtastic_ble_peer_link l;

	if (hub == 0U) {
		return;
	}
	meshtastic_ble_peer_link_get(&l);
	if (l.connected && l.node_num == hub) {
		return;
	}
	if (l.connected) {
		LOG_INF("worker: peer link is to 0x%08x, not the hub 0x%08x: dropping it",
			l.node_num, hub);
		(void)meshtastic_ble_peer_disconnect();
	}
	if (meshtastic_ble_peer_scan_target() != hub || !meshtastic_ble_peer_scan_armed()) {
		worker_dial_hub(hub);
	}
#else
	ARG_UNUSED(hub);
#endif
}

static void worker_status_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	worker_assert_dial(meshtastic_worker_get_hub());
	worker_submit(&worker_status_work);
	(void)k_work_schedule(&worker_status_timer,
			      K_SECONDS(CONFIG_MESHTASTIC_WORKER_STATUS_PERIOD_SEC));
}

void meshtastic_worker_link_up(uint32_t peer)
{
	uint32_t hub = meshtastic_worker_get_hub();

	if (peer != 0U && peer == hub) {
		/* Introduce ourselves now, not at the next tick (review X4). */
		worker_submit(&worker_status_work);
	}
}

void meshtastic_worker_on_rx(const uint8_t *wire, uint16_t len, int16_t rssi, int8_t snr,
				      uint8_t preset, uint32_t rx_ms)
{
	struct worker_frame f;

	if (wire == NULL || len == 0U || len > sizeof(f.wire)) {
		return;
	}

	k_mutex_lock(&worker_lock, K_FOREVER);
	worker.stats.heard++;
	heard_note_locked(wire, len, snr, rx_ms);
	if (worker.hub == 0U) {
		worker.stats.no_hub++;
		k_mutex_unlock(&worker_lock);
		return;
	}
	f.len = len;
	f.rssi = rssi;
	f.snr = snr;
	f.preset = preset;
	f.rx_ms = rx_ms;
	memcpy(f.wire, wire, len);
	if (k_msgq_put(&worker_q, &f, K_NO_WAIT) != 0) {
		worker.stats.queue_full++;
		k_mutex_unlock(&worker_lock);
		return;
	}
	k_mutex_unlock(&worker_lock);
	worker_submit(&worker_work);
}

int meshtastic_worker_on_envelope(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_worker_on_envelope_from(NULL, node, env, len);
}

int meshtastic_worker_on_envelope_from(const struct meshtastic_link_bearer *b,
						uint32_t node, const uint8_t *env, size_t len)
{
	struct meshtastic_link_msg msg;
	uint32_t status_to = 0U;
	int ret;

	if (node == 0U || env == NULL) {
		return -EINVAL;
	}
	ret = meshtastic_link_decode(env, len, &msg);

	k_mutex_lock(&worker_lock, K_FOREVER);
	if (ret < 0) {
		worker.stats.rejected++;
		ret = -EBADMSG;
		goto out;
	}
	switch (msg.type) {
	case MESHTASTIC_LINK_SET_PRESET: {
		struct meshtastic_link_info info;
		bool trusted;

		/* Only the hub retunes this radio, and only over a link the
		 * bearer vouches for (bonded BLE, a trusted wire): a beat's node
		 * number alone is spoofable (ATTACHMENT-SCOPE C4). */
		if (node != worker.hub) {
			worker.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_link_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_LINK_AUTH_NONE;
		if (!trusted) {
			worker.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		if (msg.u.preset > (uint8_t)_meshtastic_Config_LoRaConfig_ModemPreset_MAX) {
			worker.stats.rejected++;
			ret = -EBADMSG;
			break;
		}
		ret = meshtastic_preset_switch((meshtastic_Config_LoRaConfig_ModemPreset)msg.u.preset,
					       NULL);
		if (ret == 0) {
			worker.stats.controls++;
			/* The hub learns the outcome from STATUS, not from an ack;
			 * sent once worker_lock is released (the lock order above). */
			status_to = node;
		}
		break;
	}
	case MESHTASTIC_LINK_TX_FRAME: {
		struct meshtastic_link_info info;
		struct worker_tx t;
		bool trusted;

		/* The same gate as SET_PRESET: the hub, over a link the bearer
		 * vouches for. Transmitting is the one thing a worker does on the
		 * air, and only for its hub. */
		if (node != worker.hub) {
			worker.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_link_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_LINK_AUTH_NONE;
		if (!trusted) {
			worker.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		if (msg.u.tx.wire_len == 0U || msg.u.tx.wire_len > MESHTASTIC_PKT_MAX) {
			worker.stats.rejected++;
			ret = -EBADMSG;
			break;
		}
		t.len = msg.u.tx.wire_len;
		t.tx_seq = msg.u.tx.tx_seq;
		t.flags = msg.u.tx.flags;
		t.defers = 0U;
		t.not_before = 0U;
		t.relay_src = 0U;
		t.relay_id = 0U;
		t.orig_rx_ms = 0U;
		t.dupe = MESHTASTIC_LINK_DUPE_KEEP;
		t.snr = 0;
		if ((t.flags & MESHTASTIC_LINK_TXF_RELAY) != 0U) {
			/* The window runs from OUR reception of the original: the hub
			 * echoes the rx_ms we stamped on it. A stamp that is not recent
			 * (a stale or foreign clock) is replaced by now. */
			uint32_t now = k_uptime_get_32();
			uint32_t base = msg.u.tx.rx_ms;
			int32_t age = (int32_t)(now - base);
			uint32_t due;

			if (age < 0 || age > 60000) {
				base = now;
			}
			due = base + msg.u.tx.not_before_ms;
			t.not_before = (due == 0U) ? 1U : due;
			t.relay_src = msg.u.tx.relay_src;
			t.relay_id = msg.u.tx.relay_id;
			t.orig_rx_ms = base;
			t.arrived_ms = now;
			t.dupe = msg.u.tx.dupe;
			for (unsigned int i = 0U; i < HEARD_RING; i++) {
				if (heard_ring[i].count != 0U && heard_ring[i].src == t.relay_src &&
				    heard_ring[i].id == t.relay_id) {
					t.snr = heard_ring[i].snr;
					break;
				}
			}
		}
		if ((t.flags & MESHTASTIC_LINK_TXF_OWN_DELAY) != 0U) {
			/* An own frame of the hub's: the reference's own-TX window
			 * (getTxDelayMsec), with THIS radio's slot time and, when the
			 * image measures it, its channel utilisation. */
			struct meshtastic_contention_plan plan;
			uint8_t util = 0U;

#if defined(CONFIG_MESHTASTIC_AIRTIME)
			util = (uint8_t)meshtastic_airtime_channel_util_percent();
#endif
			meshtastic_contention_plan_own(util, mt.modem.spread_factor,
						       mt.modem.bandwidth_hz, false, &plan);
			if (plan.delay_ms != 0U) {
				uint32_t due = k_uptime_get_32() + plan.delay_ms;

				t.not_before = (due == 0U) ? 1U : due;
			}
		}
		memcpy(t.wire, msg.u.tx.wire, msg.u.tx.wire_len);
		if (k_msgq_put(&worker_tx_q, &t, K_NO_WAIT) != 0) {
			worker.stats.tx_queue_full++;
			ret = -ENOBUFS;
			break;
		}
		worker.stats.controls++;
		ret = 0;
		break;
	}
	case MESHTASTIC_LINK_SET_POLICY: {
		struct meshtastic_link_info info;
		bool trusted;

		/* The hub configures its worker's radio: the same gate as SET_PRESET. */
		if (node != worker.hub) {
			worker.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_link_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_LINK_AUTH_NONE;
		if (!trusted) {
			worker.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		ret = 0;
		if ((msg.u.policy.flags & MESHTASTIC_LINK_POL_HAS_TX_POWER) != 0U) {
			/* The console's own pattern: read the LoRa section, change the
			 * field, write it back -- applied now, persisted, region-clamped
			 * by the store. */
			meshtastic_Config cfg;

			ret = meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg);
			if (ret == 0) {
				cfg.which_payload_variant = meshtastic_Config_lora_tag;
				cfg.payload_variant.lora.tx_power = msg.u.policy.tx_power;
				ret = meshtastic_config_store_set_config(&cfg);
			}
		}
		if (ret == 0) {
			worker.stats.controls++;
			status_to = node;
		}
		break;
	}
	case MESHTASTIC_LINK_TX_CANCEL: {
		unsigned int slot = ARRAY_SIZE(cancels);

		if (node != worker.hub) {
			worker.stats.refused++;
			ret = -EPERM;
			break;
		}
		for (unsigned int i = 0U; i < ARRAY_SIZE(cancels); i++) {
			if (!cancels[i].set) {
				slot = i;
				break;
			}
		}
		if (slot == ARRAY_SIZE(cancels)) {
			slot = 0U; /* the oldest gives way */
		}
		cancels[slot] = (struct cancelled){
			.src = msg.u.cancel.src, .id = msg.u.cancel.id, .set = true,
		};
		worker.stats.controls++;
		ret = 0;
		break;
	}
	default:
		/* RX_FRAME, STATUS, TX_RESULT are what a worker SENDS. */
		worker.stats.rejected++;
		ret = -EBADMSG;
		break;
	}
out:
	k_mutex_unlock(&worker_lock);
	if (status_to != 0U) {
		(void)status_send(status_to);
	}
	return ret;
}

/* ---- persistence: the hub survives a reboot (mtattach/brain) ---------------- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static void worker_save(uint32_t hub)
{
	if (settings_save_one("mtattach/brain", &hub, sizeof(hub)) != 0) {
		LOG_WRN("worker: settings save failed");
	}
}

static int worker_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint32_t hub;

	if (strcmp(key, "brain") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(hub) || read_cb(cb_arg, &hub, len) != (ssize_t)len) {
		return -EINVAL;
	}
	k_mutex_lock(&worker_lock, K_FOREVER);
	worker.hub = hub;
	k_mutex_unlock(&worker_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_worker, "mtattach", NULL, worker_settings_set, NULL, NULL);

/* The role intent: the hub this worker reports to. */
static bool worker_intent_is_set(char *detail, size_t len)
{
	uint32_t hub;

	k_mutex_lock(&worker_lock, K_FOREVER);
	hub = worker.hub;
	k_mutex_unlock(&worker_lock);
	if (hub != 0U) {
		snprintf(detail, len, "hub 0x%08x", hub);
	}
	return hub != 0U;
}

static int worker_intent_clear(void)
{
	int ret;

	k_mutex_lock(&worker_lock, K_FOREVER);
	worker.hub = 0U;
	k_mutex_unlock(&worker_lock);
	ret = settings_delete("mtattach/brain");
	return (ret == -ENOENT) ? 0 : ret;
}

MESHTASTIC_INTENT_DEFINE(worker, "worker", "mtattach/brain", worker_intent_is_set, worker_intent_clear);

static void worker_forget(void)
{
	(void)settings_delete("mtattach/brain");
}
#else
static void worker_save(uint32_t hub)
{
	ARG_UNUSED(hub);
}

static void worker_forget(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* A worker dials its hub (ATTACHMENT-SCOPE C7): the peer link's central half
 * hunts for the hub's advert and reconnects on its own after either side
 * reboots, and the intent is persisted by the peer link itself. Without a BLE
 * stack (native_sim) there is nothing to dial. */
static void worker_dial_hub(uint32_t hub)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	int ret = (hub != 0U) ? meshtastic_ble_peer_connect(hub) : meshtastic_ble_peer_scan_set(false);

	if (ret != 0 && ret != -EALREADY) {
		LOG_WRN("worker: dialling hub 0x%08x failed (%d)", hub, ret);
	}
#else
	ARG_UNUSED(hub);
#endif
}

void meshtastic_worker_set_hub(uint32_t node)
{
	k_mutex_lock(&worker_lock, K_FOREVER);
	worker.hub = node;
	k_mutex_unlock(&worker_lock);
	worker_save(node);
	worker_dial_hub(node);
	if (node != 0U) {
		/* Introduce ourselves without waiting for the timer. */
		worker_submit(&worker_status_work);
	}
}

uint32_t meshtastic_worker_get_hub(void)
{
	uint32_t hub;

	k_mutex_lock(&worker_lock, K_FOREVER);
	hub = worker.hub;
	k_mutex_unlock(&worker_lock);
	return hub;
}

int meshtastic_worker_status_send(void)
{
	uint32_t hub = meshtastic_worker_get_hub();

	return (hub == 0U) ? -EHOSTUNREACH : status_send(hub);
}

void meshtastic_worker_stats_get(struct meshtastic_worker_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&worker_lock, K_FOREVER);
	*out = worker.stats;
	k_mutex_unlock(&worker_lock);
}

void meshtastic_worker_reset(void)
{
	memset(heard_ring, 0, sizeof(heard_ring));
	heard_next = 0U;
	memset(cancels, 0, sizeof(cancels));
	k_msgq_purge(&worker_q);
	k_mutex_lock(&worker_lock, K_FOREVER);
	worker.hub = CONFIG_MESHTASTIC_WORKER_HUB_ID;
	memset(&worker.stats, 0, sizeof(worker.stats));
	k_mutex_unlock(&worker_lock);
	worker_forget();
}

void meshtastic_worker_start(void)
{
	uint32_t hub = meshtastic_worker_get_hub();

	if (hub != 0U) {
		worker_dial_hub(hub);
	}
	/* The first tick comes soon: the peer link restores its own saved intent
	 * after this runs, and the tick's worker_assert_dial() puts the hub back. */
	(void)k_work_schedule(&worker_status_timer, K_SECONDS(5));
	k_thread_create(&worker_tx_thread, worker_tx_stack, K_THREAD_STACK_SIZEOF(worker_tx_stack),
			worker_tx_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&worker_tx_thread, "worker_tx");
}
