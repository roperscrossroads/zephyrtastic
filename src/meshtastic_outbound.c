/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Serialises all LoRa transmits through a single worker thread, and applies the
 * scheduler policy (meshtastic_sched) to decide the order frames are sent and,
 * under congestion, which frame to drop. Replaces a plain FIFO msgq with a small
 * priority queue so ACKs / replies / relays are never stuck behind — or dropped
 * in favour of — background telemetry.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "meshtastic_packet.h"

#include "meshtastic_airtime.h"
#include "meshtastic_ble_peer.h"
#include "meshtastic_contention.h"
#include "meshtastic_core.h"
#include "meshtastic_duty.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_outbound.h"
#include "meshtastic_sched.h"

#include "meshtastic_preset.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define OB_MAX CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX

struct ob_item {
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	uint8_t tier;
	/* Uptime (k_uptime_get_32) before which this frame must not go out — the
	 * contention window. 0 means "eligible now", which is every frame the
	 * caller did not explicitly defer. Compared with a signed difference so a
	 * 32-bit uptime wrap does not strand an item for 49 days. */
	uint32_t send_after;
	/* Already pushed into the late-rebroadcast window. The reference gets this
	 * for free — there, only a clamped packet carries tx_after at all, since a
	 * normal relay delay lives on a queue-head timer. Our deadline is per-item,
	 * so every deferred relay has send_after and the distinction needs its own
	 * flag. Without it a node hearing repeated copies would re-clamp on each
	 * one and never actually transmit. */
	bool late;
	/* The preset generation this frame was composed under
	 * (meshtastic_preset_generation()). For a default, empty-name channel the
	 * wire channel hash comes from the preset's display name, so a frame that
	 * outlives a preset switch is addressed to a channel nobody on the new
	 * preset is listening for. Stamped at enqueue, checked at dequeue. */
	uint32_t gen;
	/* Times the radio has answered MESHTASTIC_TX_DEFER for this frame and it
	 * went back in the queue. Bounded by CONFIG_MESHTASTIC_TX_DEFER_MAX. */
	uint8_t defers;
	struct k_sem *done; /* non-NULL => a blocking caller is waiting; never evict */
	int *result;
};

static inline bool ob_eligible(const struct ob_item *it, uint32_t now)
{
	return it->send_after == 0U || (int32_t)(now - it->send_after) >= 0;
}

/* PSRAM on V4 (no-op on V3): ~4.5 KB of staged TX frames off scarce internal
 * DRAM. Safe per meshtastic_ext_ram.h — CPU-only, mutex-guarded (no ISR access),
 * and never a DMA/flash source: the worker copies the chosen item to a stack
 * local (`cur`) before the LoRa send, so the radio never reads this array. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct ob_item ob_items[OB_MAX];
static uint8_t ob_count;
/* True from the moment the worker dequeues a frame until its transmit returns.
 *
 * ob_count alone cannot answer "is anything still going out", because the
 * worker removes an item from the queue BEFORE handing it to the radio — so
 * there is a window in which the queue is empty and a frame is nonetheless
 * about to be keyed. A preset hop that read only ob_count could retune in that
 * window and the frame would go out on the new preset's frequency carrying the
 * old preset's channel hash, which is precisely the outcome the drain interlock
 * exists to prevent (docs/MULTI-PRESET-OPERATION.md §4.4).
 *
 * Set under ob_lock in the same critical section as the removal, so the pair is
 * never observed as "queue empty and nothing in flight" while a frame exists. */
static bool ob_inflight;

static K_MUTEX_DEFINE(ob_lock);
static K_SEM_DEFINE(ob_avail, 0, OB_MAX); /* items ready for the worker */
static K_SEM_DEFINE(ob_space, 0, OB_MAX); /* space freed, wakes blocked enqueuers */

static K_THREAD_STACK_DEFINE(mt_outbound_stack, CONFIG_MESHTASTIC_OUTBOUND_STACK_SIZE);
static struct k_thread mt_outbound_thread;

/* --- queue helpers, all called under ob_lock --- */

/* Index of the frame to send next: oldest under FIFO, else highest tier and
 * oldest within that tier. Returns -1 when empty. The ordering policy is passed
 * in from a caller-held snapshot rather than re-read here. */
static int pick_next_locked(enum meshtastic_sched_order order, uint32_t now, uint32_t *wait_ms)
{
	int best = -1;
	uint32_t soonest = 0U;
	bool have_wait = false;

	*wait_ms = 0U;

	for (int i = 0; i < (int)ob_count; i++) {
		if (!ob_eligible(&ob_items[i], now)) {
			uint32_t due = ob_items[i].send_after - now;

			/* Remember when the earliest deferred frame comes due, so the
			 * worker can sleep exactly that long instead of spinning. */
			if (!have_wait || due < soonest) {
				soonest = due;
				have_wait = true;
			}
			continue;
		}

		if (best < 0) {
			best = i;
			if (order == MT_SCHED_ORDER_FIFO) {
				break; /* oldest eligible wins outright */
			}
			continue;
		}
		if (order != MT_SCHED_ORDER_FIFO && ob_items[i].tier > ob_items[best].tier) {
			best = i; /* strictly-greater keeps the oldest of the top tier */
		}
	}

	if (best < 0 && have_wait) {
		*wait_ms = soonest;
	}
	return best;
}

/* Index of the best eviction victim: a fire-and-forget frame (never a blocking
 * caller's) of the lowest tier, newest within that tier. Returns -1 if there is
 * nothing evictable. */
static int evict_victim_locked(void)
{
	int best = -1;

	for (int i = 0; i < (int)ob_count; i++) {
		if (ob_items[i].done != NULL) {
			continue; /* reliable send in flight — protected */
		}
		if (best < 0 || ob_items[i].tier < ob_items[best].tier ||
		    ob_items[i].tier == ob_items[best].tier) {
			/* ascending scan: on a tie this keeps the newest (highest index) */
			best = i;
		}
	}
	return best;
}

static void remove_index_locked(int i)
{
	for (int j = i; j < (int)ob_count - 1; j++) {
		ob_items[j] = ob_items[j + 1];
	}
	ob_count--;
}

/*
 * The radio said "not now" (MESHTASTIC_TX_DEFER): a packet was being
 * demodulated, or CAD heard one. Put the frame back behind a fresh contention
 * delay, exactly as the reference does — RadioLibInterface::onNotify() answers
 * both cases with startReceive() + setTransmitDelay(), keeping the packet at
 * the head of its queue, and never drops a frame for a busy channel. The one
 * divergence is the bound: a channel that is never quiet cannot pin a frame
 * here forever, so the CONFIG_MESHTASTIC_TX_DEFER_MAX-th refusal drops it.
 *
 * The delay is the own-transmit CSMA draw (util-scaled window, reference
 * getTxDelayMsec) with a floor of one slot: a draw of zero slots, or a window
 * pinned off, would otherwise re-ask the radio at once and burn the whole
 * budget inside the airtime of the packet CAD just heard.
 *
 * Called under ob_lock, in the same critical section that clears ob_inflight,
 * so the preset-hop interlock never sees "queue empty and nothing in flight"
 * while this frame exists. Returns true when the frame is back in the queue;
 * false when it was dropped, and the caller completes it as a failure.
 */
static bool requeue_deferred_locked(struct ob_item *it)
{
	uint32_t slot_ms;
	uint32_t delay_ms;
	uint32_t due;
	uint8_t util = 0U;

	if (it->defers >= CONFIG_MESHTASTIC_TX_DEFER_MAX || ob_count >= OB_MAX) {
		return false;
	}
	it->defers++;

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	util = (uint8_t)meshtastic_airtime_channel_util_percent();
#endif
	slot_ms = meshtastic_contention_effective_slot_ms(mt.modem.spread_factor,
							  mt.modem.bandwidth_hz, false);
	delay_ms = meshtastic_contention_delay_own_ms(util, slot_ms);
	delay_ms = MAX(delay_ms, MAX(slot_ms, 1U));
	due = k_uptime_get_32() + delay_ms;

	it->send_after = (due == 0U) ? 1U : due;
	ob_items[ob_count++] = *it;

	/* The drain handed this slot back (ob_space) when it dequeued the frame;
	 * reclaim it if nobody has taken it yet, so the count stays honest. */
	(void)k_sem_take(&ob_space, K_NO_WAIT);
	return true;
}

static void mt_outbound_thread_fn(void *p1, void *p2, void *p3)
{
	static struct ob_item cur; /* single consumer; static keeps it off the stack */
	int idx;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/*
	 * The wait is derived from the queue on every pass, and ob_avail is only a
	 * wakeup hint — never a count of outstanding work.
	 *
	 * That distinction is load-bearing now that frames can be deferred. A pass
	 * that finds nothing due still consumes a semaphore count, so treating the
	 * count as authoritative would eventually leave the worker blocked on
	 * K_FOREVER with sendable frames sitting in the queue. Deciding from queue
	 * state instead makes that unrepresentable: we only ever sleep when there
	 * is genuinely nothing to send, and only for as long as the earliest
	 * deferred frame is not due.
	 */
	while (true) {
		struct meshtastic_sched_config c;
		uint32_t wait_ms = 0U;
		bool empty;

		/* Snapshot the policy once per pass. The snapshot lock is a leaf, so
		 * taking it before ob_lock is safe and keeps ordering consistent. */
		meshtastic_sched_snapshot(&c);

		k_mutex_lock(&ob_lock, K_FOREVER);
		idx = pick_next_locked(c.tx_order, k_uptime_get_32(), &wait_ms);
		if (idx >= 0) {
			cur = ob_items[idx];
			remove_index_locked(idx);
			ob_inflight = true;
			k_mutex_unlock(&ob_lock);

			k_sem_give(&ob_space); /* a slot opened up */

			/* TX divert (agents-xhli.2): a unicast this node
			 * originated leaves over a live BLE peer link to its
			 * destination instead of spending airtime. Anything
			 * else — relays, broadcasts, unlinked destinations, a
			 * failed BLE send — goes on the air as always.
			 *
			 * The BLE peer link is offered the frame even when the
			 * preset moved underneath it: that link is not on the
			 * radio at all, so a preset switch says nothing about
			 * whether the destination can still be reached over it.
			 * Only the LoRa half is preset-bound. */
			ret = meshtastic_ble_peer_tx_try_divert(cur.wire, cur.len);
			if (ret != 0 && cur.gen != meshtastic_preset_generation()) {
				/* Composed for a preset this node has since left.
				 * Dropped rather than transmitted: the wire
				 * channel hash it carries was derived from the
				 * old preset's name, so on the new frequency it
				 * is addressed to nobody. Counted, so a preset
				 * switch that keeps stranding traffic is
				 * visible (`meshtastic preset`) instead of
				 * looking like plain packet loss. */
				meshtastic_preset_note_tx_stale();
				ret = -ESTALE;
			} else if (ret != 0) {
				ret = meshtastic_radio_send_wire_now(cur.wire, cur.len);
			}

			k_mutex_lock(&ob_lock, K_FOREVER);
			ob_inflight = false;
			if (ret == MESHTASTIC_TX_DEFER) {
				if (requeue_deferred_locked(&cur)) {
					k_mutex_unlock(&ob_lock);
					k_sem_give(&ob_avail);
					meshtastic_radio_tx_defer_requeued();
					/* Nobody is told: the frame is still pending. */
					continue;
				}
				ret = -EBUSY;
			}
			k_mutex_unlock(&ob_lock);

			if (ret == -EBUSY) {
				meshtastic_sched_stat_drop(cur.tier);
				meshtastic_radio_tx_dropped_busy();
			}

			if (cur.result != NULL) {
				*cur.result = ret;
			}
			if (cur.done != NULL) {
				k_sem_give(cur.done);
			}
			continue; /* look for more work before considering a sleep */
		}
		empty = (ob_count == 0U);
		k_mutex_unlock(&ob_lock);

		/* Nothing sendable: wait for an enqueue if the queue is empty, else
		 * only until the earliest deferred frame comes due. Any stale counts
		 * left by previous passes simply return immediately and cost one more
		 * cheap pass each; they are bounded by the queue depth. */
		(void)k_sem_take(&ob_avail, empty ? K_FOREVER : K_MSEC(MAX(wait_ms, 1U)));
	}
}

static int outbound_enqueue(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier, k_timeout_t wait,
			    uint32_t delay_ms)
{
	struct k_sem done;
	int result = 0;
	bool blocking = !K_TIMEOUT_EQ(wait, K_NO_WAIT);
	int idx;

	if (pkt == NULL || pkt_len == 0U || pkt_len > MESHTASTIC_PKT_MAX) {
		return -EINVAL;
	}

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)
	/* The regulatory gate, at the one funnel every egress path reaches:
	 * originated sends, relays, reliable retransmits, routing replies. Checked
	 * before the queue rather than at the TX worker's dequeue, because a frame
	 * that is already queued has nobody left to tell -- and telling the caller
	 * is half the point (a locally-originated refusal becomes a
	 * DUTY_CYCLE_LIMIT NAK to the phone; see send_wire_tail).
	 *
	 * Note this catches relays too, deliberately. The airtime gate further up
	 * in send_wire_tail does not, and should not: that one is politeness toward
	 * a busy channel and only throttles our own beacons. A legal ceiling does
	 * not care whose packet it was. */
	{
		uint8_t silent = 0U;

		if (meshtastic_duty_blocked(&silent)) {
			bool is_relay = false;

			if (pkt_len >= MESHTASTIC_HDR_LEN) {
				const struct meshtastic_wire_header *h =
					(const struct meshtastic_wire_header *)pkt;

				is_relay = sys_le32_to_cpu(h->src) != meshtastic_get_node_id();
			}
			/* note_blocked owns the (throttled) logging: a refusal that
			 * logged per frame would itself become the flood. */
			meshtastic_duty_note_blocked(is_relay, silent);
			return -ECANCELED;
		}
	}
#endif

	k_mutex_lock(&ob_lock, K_FOREVER);

	for (;;) {
		/* One consistent policy snapshot per iteration (order/overflow/depth
		 * must agree). Re-snapshotting each iteration means a blocking caller
		 * that waits and retries picks up any policy change in the meantime. */
		struct meshtastic_sched_config c;
		uint8_t soft;

		meshtastic_sched_snapshot(&c);
		soft = (uint8_t)CLAMP(c.tx_depth, 1, OB_MAX);

		if (ob_count < soft) {
			break; /* room to insert */
		}

		if (!blocking) {
			/* Fire-and-forget: apply the overflow policy. */
			if (c.tx_order == MT_SCHED_ORDER_PRIORITY &&
			    c.tx_overflow == MT_SCHED_OVF_DROP_LOWEST) {
				int victim = evict_victim_locked();

				if (victim >= 0 && ob_items[victim].tier < tier) {
					meshtastic_sched_stat_drop(ob_items[victim].tier);
					remove_index_locked(victim);
					/* Rebalance the ready-count for the evicted frame. */
					(void)k_sem_take(&ob_avail, K_NO_WAIT);
					break; /* room made — insert below */
				}
			}
			/* drop-newest, or nothing lower-ranked to evict: reject incoming */
			k_mutex_unlock(&ob_lock);
			meshtastic_sched_stat_drop(tier);
			return -ENOMSG;
		}

		/* Blocking caller: never dropped — wait for the worker to free a slot. */
		k_mutex_unlock(&ob_lock);
		if (k_sem_take(&ob_space, wait) != 0) {
			return -EAGAIN;
		}
		k_mutex_lock(&ob_lock, K_FOREVER);
	}

	idx = (int)ob_count++;
	memcpy(ob_items[idx].wire, pkt, pkt_len);
	ob_items[idx].len = pkt_len;
	ob_items[idx].tier = tier;
	/* A deadline of 0 means "now"; bump a zero-valued uptime by 1 ms so the
	 * sentinel keeps its meaning at the very start of boot. */
	ob_items[idx].late = false;
	ob_items[idx].defers = 0U;
	ob_items[idx].gen = meshtastic_preset_generation();
	if (delay_ms == 0U) {
		ob_items[idx].send_after = 0U;
	} else {
		uint32_t due = k_uptime_get_32() + delay_ms;

		ob_items[idx].send_after = (due == 0U) ? 1U : due;
	}
	if (blocking) {
		k_sem_init(&done, 0, 1);
		ob_items[idx].done = &done;
		ob_items[idx].result = &result;
	} else {
		ob_items[idx].done = NULL;
		ob_items[idx].result = NULL;
	}
	meshtastic_sched_stat_enq(tier, ob_count);
	k_mutex_unlock(&ob_lock);

	k_sem_give(&ob_avail);

	if (!blocking) {
		return 0;
	}

	if (k_sem_take(&done, wait) == -EAGAIN) {
		/* Timed out waiting for completion. Callers that need a hard result
		 * use K_FOREVER, so this path is not exercised in practice. */
		return -EAGAIN;
	}
	return result;
}

int meshtastic_radio_send_wire_prio(uint8_t *pkt, uint32_t pkt_len, uint8_t tier)
{
	int ret = outbound_enqueue(pkt, pkt_len, tier, K_NO_WAIT, 0U);

	return (ret == 0) ? 0 : ret;
}

int meshtastic_radio_send_wire_wait_prio(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
					 k_timeout_t timeout)
{
	return outbound_enqueue(pkt, pkt_len, tier, timeout, 0U);
}

int meshtastic_radio_send_wire_after(uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
				     uint32_t delay_ms)
{
	return outbound_enqueue(pkt, pkt_len, tier, K_NO_WAIT, delay_ms);
}

uint8_t meshtastic_outbound_pending(void)
{
	uint8_t n;

	k_mutex_lock(&ob_lock, K_FOREVER);
	n = (uint8_t)(ob_count + (ob_inflight ? 1U : 0U));
	k_mutex_unlock(&ob_lock);

	return n;
}

int meshtastic_outbound_defer_late(uint32_t src, uint32_t id, uint32_t delay_ms)
{
	int moved = 0;

	k_mutex_lock(&ob_lock, K_FOREVER);

	for (int i = 0; i < (int)ob_count; i++) {
		const struct meshtastic_wire_header *h;
		uint32_t due;

		if (ob_items[i].len < MESHTASTIC_HDR_LEN || ob_items[i].late ||
		    ob_items[i].done != NULL) {
			continue; /* already late, or someone is waiting on it */
		}

		h = (const struct meshtastic_wire_header *)ob_items[i].wire;
		if (sys_le32_to_cpu(h->src) != src || sys_le32_to_cpu(h->id) != id) {
			continue;
		}

		/* Measured from now — the moment we heard the peer — not from the
		 * original deadline, matching the reference. The point is to let
		 * everyone else finish before we add our copy. */
		due = k_uptime_get_32() + delay_ms;
		ob_items[i].send_after = (due == 0U) ? 1U : due;
		ob_items[i].late = true;
		moved++;
	}

	k_mutex_unlock(&ob_lock);

	return moved;
}

int meshtastic_outbound_cancel(uint32_t src, uint32_t id)
{
	int removed = 0;

	k_mutex_lock(&ob_lock, K_FOREVER);

	/* Reverse scan so remove_index_locked()'s shift-down cannot skip an entry.
	 * A given (src,id) should only ever be queued once, but a duplicate would
	 * be a leak rather than an error, so remove every match. */
	for (int i = (int)ob_count - 1; i >= 0; i--) {
		const struct meshtastic_wire_header *h;

		if (ob_items[i].len < MESHTASTIC_HDR_LEN) {
			continue;
		}
		/* Never cancel a frame someone is blocked waiting on: the caller owns
		 * its completion and would wait out its timeout for a result that
		 * never comes. Relays are always fire-and-forget, so this excludes
		 * nothing we want to cancel. */
		if (ob_items[i].done != NULL) {
			continue;
		}

		h = (const struct meshtastic_wire_header *)ob_items[i].wire;
		if (sys_le32_to_cpu(h->src) != src || sys_le32_to_cpu(h->id) != id) {
			continue;
		}

		remove_index_locked(i);
		removed++;
	}

	k_mutex_unlock(&ob_lock);

	if (removed > 0) {
		/* Free the slots so a blocked enqueuer can proceed. */
		for (int i = 0; i < removed; i++) {
			k_sem_give(&ob_space);
		}
	}

	return removed;
}

int meshtastic_radio_send_wire(uint8_t *pkt, uint32_t pkt_len)
{
	return meshtastic_radio_send_wire_prio(pkt, pkt_len, MT_SCHED_TIER_NORMAL);
}

int meshtastic_radio_send_wire_wait(const uint8_t *pkt, uint32_t pkt_len, k_timeout_t timeout)
{
	return outbound_enqueue(pkt, pkt_len, MT_SCHED_TIER_NORMAL, timeout, 0U);
}

int meshtastic_outbound_init(void)
{
	k_thread_create(&mt_outbound_thread, mt_outbound_stack,
			K_THREAD_STACK_SIZEOF(mt_outbound_stack), mt_outbound_thread_fn, NULL, NULL,
			NULL, CONFIG_MESHTASTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mt_outbound_thread, "meshtastic_tx");

	return 0;
}
