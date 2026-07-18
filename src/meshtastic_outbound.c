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
#include <zephyr/sys/util.h>

#include "meshtastic_outbound.h"
#include "meshtastic_sched.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define OB_MAX CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX

struct ob_item {
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t len;
	uint8_t tier;
	struct k_sem *done; /* non-NULL => a blocking caller is waiting; never evict */
	int *result;
};

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
static int pick_next_locked(enum meshtastic_sched_order order)
{
	int best;

	if (ob_count == 0U) {
		return -1;
	}
	if (order == MT_SCHED_ORDER_FIFO) {
		return 0; /* index 0 is the oldest (inserts append, removes shift down) */
	}

	best = 0;
	for (int i = 1; i < (int)ob_count; i++) {
		if (ob_items[i].tier > ob_items[best].tier) {
			best = i; /* strictly-greater keeps the oldest of the top tier */
		}
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

	while (true) {
		struct meshtastic_sched_config c;

		(void)k_sem_take(&ob_avail, K_FOREVER);

		/* Snapshot the policy once per send, before taking the queue lock. */
		meshtastic_sched_snapshot(&c);

		k_mutex_lock(&ob_lock, K_FOREVER);
		idx = pick_next_locked(c.tx_order);
		if (idx < 0) {
			k_mutex_unlock(&ob_lock);
			continue; /* stale wakeup (e.g. after an eviction) */
		}
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
	}
}

static int outbound_enqueue(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier, k_timeout_t wait)
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
	int ret = outbound_enqueue(pkt, pkt_len, tier, K_NO_WAIT);

	return (ret == 0) ? 0 : ret;
}

int meshtastic_radio_send_wire_wait_prio(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
					 k_timeout_t timeout)
{
	return outbound_enqueue(pkt, pkt_len, tier, timeout);
}

int meshtastic_radio_send_wire(uint8_t *pkt, uint32_t pkt_len)
{
	return meshtastic_radio_send_wire_prio(pkt, pkt_len, MT_SCHED_TIER_NORMAL);
}

int meshtastic_radio_send_wire_wait(const uint8_t *pkt, uint32_t pkt_len, k_timeout_t timeout)
{
	return outbound_enqueue(pkt, pkt_len, MT_SCHED_TIER_NORMAL, timeout);
}

int meshtastic_outbound_init(void)
{
	k_thread_create(&mt_outbound_thread, mt_outbound_stack,
			K_THREAD_STACK_SIZEOF(mt_outbound_stack), mt_outbound_thread_fn, NULL, NULL,
			NULL, CONFIG_MESHTASTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mt_outbound_thread, "meshtastic_tx");

	return 0;
}
