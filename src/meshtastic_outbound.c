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

#include "meshtastic_outbound.h"
#include "meshtastic_sched.h"

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
	struct k_sem *done; /* non-NULL => a blocking caller is waiting; never evict */
	int *result;
};

static inline bool ob_eligible(const struct ob_item *it, uint32_t now)
{
	return it->send_after == 0U || (int32_t)(now - it->send_after) >= 0;
}

static struct ob_item ob_items[OB_MAX];
static uint8_t ob_count;

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
			k_mutex_unlock(&ob_lock);

			k_sem_give(&ob_space); /* a slot opened up */

			ret = meshtastic_radio_send_wire_now(cur.wire, cur.len);

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
