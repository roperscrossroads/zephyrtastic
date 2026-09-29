/* SPDX-License-Identifier: GPL-3.0 */

/*
 * A keyless radio head (ATTACHMENT-DESIGN S5). See meshtastic_attachment_head.h.
 *
 * The shape is the relay ear's (meshtastic_relay_ear.c): the RX thread queues,
 * a work item on the BLE module's queue forwards, because a GATT write can
 * block on buffers and the RX thread must not. Unlike the ear, this hooks
 * BEFORE the router -- the frame is never decoded here, there is nothing to
 * decode it with.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_attachment_codec.h"
#include "meshtastic_attachment_head.h"
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

struct head_frame {
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t preset;
	uint32_t rx_ms;
	uint8_t wire[MESHTASTIC_PKT_MAX];
};

static struct {
	uint32_t brain;
	struct meshtastic_attachment_head_stats stats;
} head = {
	.brain = CONFIG_MESHTASTIC_ATTACHMENT_HEAD_BRAIN_ID,
};

/* A frame the brain handed us to transmit (P3 slice 1): the bytes as built by
 * the brain, untouched -- a head never builds a frame, it keys one up. */
struct head_tx {
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
	 * the brain's clock and modem are not this radio's. 0 = now. */
	uint32_t not_before;
	uint8_t wire[MESHTASTIC_PKT_MAX];
};

static K_MUTEX_DEFINE(head_lock);
K_MSGQ_DEFINE(head_q, sizeof(struct head_frame), CONFIG_MESHTASTIC_ATTACHMENT_HEAD_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(head_tx_q, sizeof(struct head_tx), CONFIG_MESHTASTIC_ATTACHMENT_HEAD_TX_QUEUE_SIZE, 4);
/* The transmit thread (P3 slice 1/2): it sleeps through the contention window
 * and blocks for the airtime, so it is nobody else's queue. */
static K_THREAD_STACK_DEFINE(head_tx_stack, CONFIG_MESHTASTIC_ATTACHMENT_HEAD_TX_STACK_SIZE);
static struct k_thread head_tx_thread;
static uint8_t tx_env_buf[MESHTASTIC_ATTACHMENT_ENV_MAX]; /* the TX thread's own scratch */

/* The heard-history ring (ATTACHMENT-DESIGN §12): every frame this radio hears,
 * by (src, id), with when and how often -- a keyless head can read the
 * plaintext header. A relay the brain handed us for (src, id) is cancelled or
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

/* Relays the brain withdrew (TX_CANCEL) that may still sit in the TX queue. */
struct cancelled {
	uint32_t src;
	uint32_t id;
	bool set;
};
static struct cancelled cancels[4];

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

static void head_work_fn(struct k_work *work);
static K_WORK_DEFINE(head_work, head_work_fn);
static void head_dial_brain(uint32_t brain);
/* The STATUS timer ticks on the system queue; the send itself runs where the
 * GATT write may block (head_submit), like every other envelope. */
static void head_status_fn(struct k_work *work);
static K_WORK_DEFINE(head_status_work, head_status_fn);
static void head_status_tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(head_status_timer, head_status_tick);

/* The send seam: whichever registered bearer has a live link to the brain. */
__weak int meshtastic_attachment_head_send(uint32_t brain, const uint8_t *env, size_t len)
{
	return meshtastic_attach_bearer_send(brain, env, len);
}

static void head_submit(struct k_work *work)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	(void)meshtastic_ble_work_submit(work);
#else
	/* No BLE stack (native_sim): nothing on this queue can block on GATT. */
	(void)k_work_submit(work);
#endif
}

/* One envelope scratch for both workers: they run on one queue. */
static uint8_t env_buf[MESHTASTIC_ATTACHMENT_ENV_MAX];

static int status_send_locked(uint32_t brain)
{
	struct meshtastic_attachment_status st = {
		.preset = (uint8_t)mt.modem_preset,
		/* RX_ONLY only when transmit is compiled out of this image: since P3
		 * a head keys up what its brain hands it, and the brain refuses to
		 * hand anything to a head that says receive-only. */
#if defined(CONFIG_MESHTASTIC_SCANNER_RX_ONLY) || defined(CONFIG_MESHTASTIC_RELAY_EAR_RX_ONLY)
		.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD | MESHTASTIC_ATTACHMENT_ST_RX_ONLY,
#else
		.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD,
#endif
		.hwid = mt.node_id,
		.brain = brain,
		.rx_frames = head.stats.forwarded,
		.tx_frames = head.stats.tx_sent,
		.uptime_s = (uint32_t)(k_uptime_get() / 1000),
		.rx_dropped = head.stats.queue_full + head.stats.send_failed,
	};
	int len;

	if (mt.tx_enabled) {
		st.flags |= MESHTASTIC_ATTACHMENT_ST_TX_ENABLED;
	}
	if (mt.radio_held) {
		st.flags |= MESHTASTIC_ATTACHMENT_ST_RADIO_HELD;
	}
	len = meshtastic_attachment_encode_status(&st, env_buf, sizeof(env_buf));
	if (len < 0) {
		return len;
	}
	return meshtastic_attachment_head_send(brain, env_buf, (size_t)len);
}

/* Transmit what the brain handed us: the one funnel every transmit takes
 * (meshtastic_radio_send_wire_now: CAD/LBT, the scanner gate, tx_enabled,
 * radio_held), so a head keys up under exactly the rules its own image would.
 * An own frame (OWN_DELAY) waits its contention window first; DEFER is
 * retried, bounded as the brain's own worker bounds it. On its own thread:
 * the radio blocks for the airtime, and the forward path must keep draining. */
static void head_tx_thread_fn(void *p1, void *p2, void *p3)
{
	struct head_tx t;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		int ret;

		(void)k_msgq_get(&head_tx_q, &t, K_FOREVER);
		if (t.not_before != 0U) {
			int32_t left = (int32_t)(t.not_before - k_uptime_get_32());

			if (left > 0) {
				k_msleep(left);
			}
		}
		if ((t.flags & MESHTASTIC_ATTACHMENT_TXF_RELAY) != 0U) {
			/* The reference's cancel-on-duplicate / late clamp, on THIS
			 * radio's hearing and clock (ATTACHMENT-DESIGN §12): a copy we
			 * heard after the one this relay is of means a peer relayed it
			 * first. The brain's own withdrawal (a copy that reached it by
			 * another radio) arrives as TX_CANCEL. */
			bool withdrawn;
			bool again;

			k_mutex_lock(&head_lock, K_FOREVER);
			withdrawn = cancelled_take_locked(t.relay_src, t.relay_id);
			again = heard_again_locked(t.relay_src, t.relay_id, t.orig_rx_ms);
			k_mutex_unlock(&head_lock);
			if (withdrawn || (again && t.dupe == MESHTASTIC_ATTACHMENT_DUPE_CANCEL)) {
				k_mutex_lock(&head_lock, K_FOREVER);
				head.stats.tx_cancelled++;
				k_mutex_unlock(&head_lock);
				ret = -ECANCELED;
				goto report;
			}
			if (again && t.dupe == MESHTASTIC_ATTACHMENT_DUPE_LATE) {
				/* To the end of the worst-case window, from now, with this
				 * modem and the SNR we heard the original at. */
				uint32_t late = meshtastic_contention_delay_relay_worst_ms(
					t.snr, meshtastic_contention_effective_slot_ms(
						       mt.modem.spread_factor, mt.modem.bandwidth_hz,
						       false));

				k_mutex_lock(&head_lock, K_FOREVER);
				head.stats.tx_late++;
				k_mutex_unlock(&head_lock);
				k_msleep(late);
				k_mutex_lock(&head_lock, K_FOREVER);
				withdrawn = cancelled_take_locked(t.relay_src, t.relay_id);
				k_mutex_unlock(&head_lock);
				if (withdrawn) {
					ret = -ECANCELED;
					goto report;
				}
			}
		}
		{
			/* The regulatory gate, at THIS radio (ATTACHMENT-SCOPE A4): the
			 * brain's duty ledger prices its own channel, not this one.
			 * A stub (false) on an image without the airtime ledger. */
			uint8_t silent = 0U;

			if (meshtastic_duty_blocked(&silent)) {
				k_mutex_lock(&head_lock, K_FOREVER);
				head.stats.tx_duty_blocked++;
				k_mutex_unlock(&head_lock);
				LOG_WRN("head: tx seq %u refused by the duty gate (%u min silent)",
					t.tx_seq, silent);
				ret = -ECANCELED;
				goto report;
			}
		}
		ret = meshtastic_radio_send_wire_now(t.wire, t.len);
		while (ret == MESHTASTIC_TX_DEFER && t.defers < CONFIG_MESHTASTIC_TX_DEFER_MAX) {
			t.defers++;
			k_mutex_lock(&head_lock, K_FOREVER);
			head.stats.tx_deferred++;
			k_mutex_unlock(&head_lock);
			k_msleep(20);
			ret = meshtastic_radio_send_wire_now(t.wire, t.len);
		}

report:
		k_mutex_lock(&head_lock, K_FOREVER);
		if (ret == 0) {
			head.stats.tx_sent++;
		} else if (ret != -ECANCELED) {
			head.stats.tx_failed++;
		}
		if ((t.flags & MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT) != 0U && head.brain != 0U) {
			const struct meshtastic_attachment_tx_result r = {
				.tx_seq = t.tx_seq,
				.rc = (int8_t)CLAMP(ret, -127, 127),
				.defers = t.defers,
				.tx_ms = (uint32_t)k_uptime_get(),
			};
			int elen = meshtastic_attachment_encode_tx_result(&r, tx_env_buf,
									  sizeof(tx_env_buf));

			if (elen > 0) {
				(void)meshtastic_attachment_head_send(head.brain, tx_env_buf,
								      (size_t)elen);
			}
		}
		k_mutex_unlock(&head_lock);
		if (ret != 0) {
			LOG_DBG("head: tx seq %u failed (%d)", t.tx_seq, ret);
		}
	}
}

static void head_work_fn(struct k_work *work)
{
	static struct head_frame f; /* one worker: static keeps it off the stack */

	ARG_UNUSED(work);

	while (k_msgq_get(&head_q, &f, K_NO_WAIT) == 0) {
		const struct meshtastic_attachment_rx_frame m = {
			.preset = f.preset,
			.rssi = f.rssi,
			.snr = f.snr,
			.rx_ms = f.rx_ms,
			.flags = 0U,
			.wire = f.wire,
			.wire_len = f.len,
		};
		uint32_t brain;
		int ret;

		k_mutex_lock(&head_lock, K_FOREVER);
		brain = head.brain;
		if (brain == 0U) {
			ret = -EHOSTUNREACH;
		} else {
			ret = meshtastic_attachment_encode_rx_frame(&m, env_buf, sizeof(env_buf));
			if (ret > 0) {
				ret = meshtastic_attachment_head_send(brain, env_buf, (size_t)ret);
			}
		}
		if (ret == 0) {
			head.stats.forwarded++;
		} else {
			head.stats.send_failed++;
		}
		k_mutex_unlock(&head_lock);
		if (ret != 0) {
			LOG_DBG("head: forward to 0x%08x failed (%d)", brain, ret);
		}
	}
}

static void head_status_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&head_lock, K_FOREVER);
	if (head.brain != 0U && status_send_locked(head.brain) == 0) {
		head.stats.status_sent++;
	}
	k_mutex_unlock(&head_lock);
}

/* Is the peer link up to @p brain? If it is up to someone else (a target the
 * link restored from its node days, a stale intent), drop it; if it is not up,
 * dial. The tick asks this every period, so a head converges on its brain
 * without an operator (ATTACHMENT-SCOPE C7; bench 2026-09-28: kit1 booted
 * scanning for the node it last dialled as a node). */
static void head_assert_dial(uint32_t brain)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	struct meshtastic_ble_peer_link l;

	if (brain == 0U) {
		return;
	}
	meshtastic_ble_peer_link_get(&l);
	if (l.connected && l.node_num == brain) {
		return;
	}
	if (l.connected) {
		LOG_INF("head: peer link is to 0x%08x, not the brain 0x%08x: dropping it",
			l.node_num, brain);
		(void)meshtastic_ble_peer_disconnect();
	}
	if (meshtastic_ble_peer_scan_target() != brain || !meshtastic_ble_peer_scan_armed()) {
		head_dial_brain(brain);
	}
#else
	ARG_UNUSED(brain);
#endif
}

static void head_status_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	head_assert_dial(meshtastic_attachment_head_get_brain());
	head_submit(&head_status_work);
	(void)k_work_schedule(&head_status_timer,
			      K_SECONDS(CONFIG_MESHTASTIC_ATTACHMENT_HEAD_STATUS_PERIOD_SEC));
}

void meshtastic_attachment_head_link_up(uint32_t peer)
{
	uint32_t brain = meshtastic_attachment_head_get_brain();

	if (peer != 0U && peer == brain) {
		/* Introduce ourselves now, not at the next tick (review X4). */
		head_submit(&head_status_work);
	}
}

void meshtastic_attachment_head_on_rx(const uint8_t *wire, uint16_t len, int16_t rssi, int8_t snr,
				      uint8_t preset, uint32_t rx_ms)
{
	struct head_frame f;

	if (wire == NULL || len == 0U || len > sizeof(f.wire)) {
		return;
	}

	k_mutex_lock(&head_lock, K_FOREVER);
	head.stats.heard++;
	heard_note_locked(wire, len, snr, rx_ms);
	if (head.brain == 0U) {
		head.stats.no_brain++;
		k_mutex_unlock(&head_lock);
		return;
	}
	f.len = len;
	f.rssi = rssi;
	f.snr = snr;
	f.preset = preset;
	f.rx_ms = rx_ms;
	memcpy(f.wire, wire, len);
	if (k_msgq_put(&head_q, &f, K_NO_WAIT) != 0) {
		head.stats.queue_full++;
		k_mutex_unlock(&head_lock);
		return;
	}
	k_mutex_unlock(&head_lock);
	head_submit(&head_work);
}

int meshtastic_attachment_head_on_envelope(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_attachment_head_on_envelope_from(NULL, node, env, len);
}

int meshtastic_attachment_head_on_envelope_from(const struct meshtastic_attach_bearer *b,
						uint32_t node, const uint8_t *env, size_t len)
{
	struct meshtastic_attachment_msg msg;
	int ret;

	if (node == 0U || env == NULL) {
		return -EINVAL;
	}
	ret = meshtastic_attachment_decode(env, len, &msg);

	k_mutex_lock(&head_lock, K_FOREVER);
	if (ret < 0) {
		head.stats.rejected++;
		ret = -EBADMSG;
		goto out;
	}
	switch (msg.type) {
	case MESHTASTIC_ATTACHMENT_SET_PRESET: {
		struct meshtastic_attach_link_info info;
		bool trusted;

		/* Only the brain retunes this radio, and only over a link the
		 * bearer vouches for (bonded BLE, a trusted wire): a beat's node
		 * number alone is spoofable (ATTACHMENT-SCOPE C4). */
		if (node != head.brain) {
			head.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_attach_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_ATTACH_AUTH_NONE;
		if (!trusted) {
			head.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		if (msg.u.preset > (uint8_t)_meshtastic_Config_LoRaConfig_ModemPreset_MAX) {
			head.stats.rejected++;
			ret = -EBADMSG;
			break;
		}
		ret = meshtastic_preset_switch((meshtastic_Config_LoRaConfig_ModemPreset)msg.u.preset,
					       NULL);
		if (ret == 0) {
			head.stats.controls++;
			/* The brain learns the outcome from STATUS, not from an ack. */
			if (status_send_locked(node) == 0) {
				head.stats.status_sent++;
			}
		}
		break;
	}
	case MESHTASTIC_ATTACHMENT_TX_FRAME: {
		struct meshtastic_attach_link_info info;
		struct head_tx t;
		bool trusted;

		/* The same gate as SET_PRESET: the brain, over a link the bearer
		 * vouches for. Transmitting is the one thing a head does on the
		 * air, and only for its brain. */
		if (node != head.brain) {
			head.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_attach_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_ATTACH_AUTH_NONE;
		if (!trusted) {
			head.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		if (msg.u.tx.wire_len == 0U || msg.u.tx.wire_len > MESHTASTIC_PKT_MAX) {
			head.stats.rejected++;
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
		t.dupe = MESHTASTIC_ATTACHMENT_DUPE_KEEP;
		t.snr = 0;
		if ((t.flags & MESHTASTIC_ATTACHMENT_TXF_RELAY) != 0U) {
			/* The window runs from OUR reception of the original: the brain
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
			t.dupe = msg.u.tx.dupe;
			for (unsigned int i = 0U; i < HEARD_RING; i++) {
				if (heard_ring[i].count != 0U && heard_ring[i].src == t.relay_src &&
				    heard_ring[i].id == t.relay_id) {
					t.snr = heard_ring[i].snr;
					break;
				}
			}
		}
		if ((t.flags & MESHTASTIC_ATTACHMENT_TXF_OWN_DELAY) != 0U) {
			/* An own frame of the brain's: the reference's own-TX window
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
		if (k_msgq_put(&head_tx_q, &t, K_NO_WAIT) != 0) {
			head.stats.tx_queue_full++;
			ret = -ENOBUFS;
			break;
		}
		head.stats.controls++;
		ret = 0;
		break;
	}
	case MESHTASTIC_ATTACHMENT_SET_POLICY: {
		struct meshtastic_attach_link_info info;
		bool trusted;

		/* The brain configures its head's radio: the same gate as SET_PRESET. */
		if (node != head.brain) {
			head.stats.refused++;
			ret = -EPERM;
			break;
		}
		trusted = ((b != NULL) ? b->link_info(node, &info)
				       : meshtastic_attach_bearer_link_info(node, &info, NULL)) &&
			  info.up && info.auth != MESHTASTIC_ATTACH_AUTH_NONE;
		if (!trusted) {
			head.stats.untrusted++;
			ret = -EACCES;
			break;
		}
		ret = 0;
		if ((msg.u.policy.flags & MESHTASTIC_ATTACHMENT_POL_HAS_TX_POWER) != 0U) {
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
			head.stats.controls++;
			if (status_send_locked(node) == 0) {
				head.stats.status_sent++;
			}
		}
		break;
	}
	case MESHTASTIC_ATTACHMENT_TX_CANCEL: {
		unsigned int slot = ARRAY_SIZE(cancels);

		if (node != head.brain) {
			head.stats.refused++;
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
		head.stats.controls++;
		ret = 0;
		break;
	}
	default:
		/* RX_FRAME, STATUS, TX_RESULT are what a head SENDS. */
		head.stats.rejected++;
		ret = -EBADMSG;
		break;
	}
out:
	k_mutex_unlock(&head_lock);
	return ret;
}

/* ---- persistence: the brain survives a reboot (mtattach/brain) ---------------- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static void head_save(uint32_t brain)
{
	if (settings_save_one("mtattach/brain", &brain, sizeof(brain)) != 0) {
		LOG_WRN("head: settings save failed");
	}
}

static int head_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint32_t brain;

	if (strcmp(key, "brain") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(brain) || read_cb(cb_arg, &brain, len) != (ssize_t)len) {
		return -EINVAL;
	}
	k_mutex_lock(&head_lock, K_FOREVER);
	head.brain = brain;
	k_mutex_unlock(&head_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_attach, "mtattach", NULL, head_settings_set, NULL, NULL);

static void head_forget(void)
{
	(void)settings_delete("mtattach/brain");
}
#else
static void head_save(uint32_t brain)
{
	ARG_UNUSED(brain);
}

static void head_forget(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* A head dials its brain (ATTACHMENT-SCOPE C7): the peer link's central half
 * hunts for the brain's advert and reconnects on its own after either side
 * reboots, and the intent is persisted by the peer link itself. Without a BLE
 * stack (native_sim) there is nothing to dial. */
static void head_dial_brain(uint32_t brain)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	int ret = (brain != 0U) ? meshtastic_ble_peer_connect(brain) : meshtastic_ble_peer_scan_set(false);

	if (ret != 0 && ret != -EALREADY) {
		LOG_WRN("head: dialling brain 0x%08x failed (%d)", brain, ret);
	}
#else
	ARG_UNUSED(brain);
#endif
}

void meshtastic_attachment_head_set_brain(uint32_t node)
{
	k_mutex_lock(&head_lock, K_FOREVER);
	head.brain = node;
	k_mutex_unlock(&head_lock);
	head_save(node);
	head_dial_brain(node);
	if (node != 0U) {
		/* Introduce ourselves without waiting for the timer. */
		head_submit(&head_status_work);
	}
}

uint32_t meshtastic_attachment_head_get_brain(void)
{
	uint32_t brain;

	k_mutex_lock(&head_lock, K_FOREVER);
	brain = head.brain;
	k_mutex_unlock(&head_lock);
	return brain;
}

int meshtastic_attachment_head_status_send(void)
{
	int ret;

	k_mutex_lock(&head_lock, K_FOREVER);
	if (head.brain == 0U) {
		ret = -EHOSTUNREACH;
	} else {
		ret = status_send_locked(head.brain);
		if (ret == 0) {
			head.stats.status_sent++;
		}
	}
	k_mutex_unlock(&head_lock);
	return ret;
}

void meshtastic_attachment_head_stats_get(struct meshtastic_attachment_head_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&head_lock, K_FOREVER);
	*out = head.stats;
	k_mutex_unlock(&head_lock);
}

void meshtastic_attachment_head_reset(void)
{
	memset(heard_ring, 0, sizeof(heard_ring));
	heard_next = 0U;
	memset(cancels, 0, sizeof(cancels));
	k_msgq_purge(&head_q);
	k_mutex_lock(&head_lock, K_FOREVER);
	head.brain = CONFIG_MESHTASTIC_ATTACHMENT_HEAD_BRAIN_ID;
	memset(&head.stats, 0, sizeof(head.stats));
	k_mutex_unlock(&head_lock);
	head_forget();
}

void meshtastic_attachment_head_start(void)
{
	uint32_t brain = meshtastic_attachment_head_get_brain();

	if (brain != 0U) {
		head_dial_brain(brain);
	}
	/* The first tick comes soon: the peer link restores its own saved intent
	 * after this runs, and the tick's head_assert_dial() puts the brain back. */
	(void)k_work_schedule(&head_status_timer, K_SECONDS(5));
	k_thread_create(&head_tx_thread, head_tx_stack, K_THREAD_STACK_SIZEOF(head_tx_stack),
			head_tx_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&head_tx_thread, "head_tx");
}
