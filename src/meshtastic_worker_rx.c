/* SPDX-License-Identifier: GPL-3.0 */

/*
 * Cross-preset relay, the rx-only worker (agents-jbrq.12, hole H33).
 *
 * The BLE peer link used to carry only a node's OWN outbound unicasts (the TX
 * divert). Nothing passed on what a node HEARD. The rx-only worker does exactly that, for
 * the one kind of frame a relay can use: a broadcast text on a channel this
 * node decrypts. It forwards the wire bytes unchanged; the receiving half
 * (meshtastic_relay.c) reads them through its own copy of the channel and
 * decides what crosses.
 *
 * Forwarding never runs on the LoRa receive thread: frames are queued and sent
 * from the BLE module's work queue, because a GATT write can block on buffers.
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

#include "meshtastic_core.h"
#include "meshtastic_relay.h"
#include "meshtastic_link_codec.h"
#if defined(CONFIG_MESHTASTIC_LINK_BEARER)
#include "meshtastic_link_bearer.h"
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include "meshtastic_ble_peer.h" /* meshtastic_ble_work_submit, frame_send_to */
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

struct worker_rx_frame {
	uint16_t len;
	/* What this radio saw, for a peer that takes link envelopes
	 * (ATTACHMENT-DESIGN P0): the same frame, plus its signal and preset. */
	int16_t rssi;
	int8_t snr;
	uint8_t preset;
	uint32_t rx_ms;
	uint8_t wire[MESHTASTIC_PKT_MAX];
};

static struct {
	uint32_t peer;
	int64_t window_start_ms;
	uint16_t used;
	struct meshtastic_worker_rx_stats stats;
} worker_rx = {
	.peer = CONFIG_MESHTASTIC_WORKER_RX_PEER,
};

static K_MUTEX_DEFINE(worker_rx_lock);
K_MSGQ_DEFINE(worker_rx_q, sizeof(struct worker_rx_frame), CONFIG_MESHTASTIC_WORKER_RX_QUEUE_SIZE, 4);

static void worker_rx_work_fn(struct k_work *work);
static K_WORK_DEFINE(worker_rx_work, worker_rx_work_fn);

__weak int meshtastic_worker_rx_send(uint32_t peer, const uint8_t *wire, size_t wire_len)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	return meshtastic_ble_peer_frame_send_to(peer, wire, wire_len);
#else
	ARG_UNUSED(peer);
	ARG_UNUSED(wire);
	ARG_UNUSED(wire_len);
	return -ENOTSUP;
#endif
}

/* The same seam for a link envelope, over whichever bearer reaches the
 * peer (ATTACHMENT-SCOPE §4). */
__weak int meshtastic_worker_rx_send_env(uint32_t peer, const uint8_t *env, size_t env_len)
{
#if defined(CONFIG_MESHTASTIC_LINK_BEARER)
	return meshtastic_link_bearer_send(peer, env, env_len);
#else
	ARG_UNUSED(peer);
	ARG_UNUSED(env);
	ARG_UNUSED(env_len);
	return -ENOTSUP;
#endif
}

/* Does the peer take link envelopes? The bearer knows (on BLE, the
 * peer's beat flag). Without a bearer the test seam decides. */
__weak bool meshtastic_worker_rx_peer_takes_env(uint32_t peer)
{
#if defined(CONFIG_MESHTASTIC_LINK_BEARER)
	struct meshtastic_link_info info;

	return meshtastic_link_bearer_link_info(peer, &info, NULL) && info.up &&
	       info.takes_envelopes;
#else
	ARG_UNUSED(peer);
	return false;
#endif
}

static int worker_rx_forward(uint32_t peer, const struct worker_rx_frame *f)
{
	if (meshtastic_worker_rx_peer_takes_env(peer)) {
		/* A hub (ATTACHMENT-DESIGN P0): the frame with the signal this
		 * radio saw, so the hub treats it as heard on THIS preset by THIS
		 * radio -- an RF frame, not a bearer frame. */
		static uint8_t env[MESHTASTIC_LINK_ENV_MAX]; /* one worker */
		const struct meshtastic_link_rx_frame m = {
			.preset = f->preset,
			.rssi = f->rssi,
			.snr = f->snr,
			.rx_ms = f->rx_ms,
			.flags = 0U,
			.wire = f->wire,
			.wire_len = f->len,
		};
		int len = meshtastic_link_encode_rx_frame(&m, env, sizeof(env));

		if (len < 0) {
			return len;
		}
		return meshtastic_worker_rx_send_env(peer, env, (size_t)len);
	}
	/* The receiving half of a relay: the bare frame, as before. */
	return meshtastic_worker_rx_send(peer, f->wire, f->len);
}

static void worker_rx_work_fn(struct k_work *work)
{
	static struct worker_rx_frame f; /* one worker: static keeps it off the stack */
	uint32_t peer;

	ARG_UNUSED(work);

	while (k_msgq_get(&worker_rx_q, &f, K_NO_WAIT) == 0) {
		int ret;

		k_mutex_lock(&worker_rx_lock, K_FOREVER);
		peer = worker_rx.peer;
		k_mutex_unlock(&worker_rx_lock);

		ret = (peer == 0U) ? -EHOSTUNREACH : worker_rx_forward(peer, &f);

		k_mutex_lock(&worker_rx_lock, K_FOREVER);
		if (ret == 0) {
			worker_rx.stats.forwarded++;
		} else {
			worker_rx.stats.send_failed++;
		}
		k_mutex_unlock(&worker_rx_lock);
		if (ret != 0) {
			LOG_DBG("worker-rx: forward to 0x%08x failed (%d)", peer, ret);
		}
	}
}

static void worker_rx_submit(void)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	(void)meshtastic_ble_work_submit(&worker_rx_work);
#else
	/* No BLE stack (native_sim): nothing on this queue can block on GATT. */
	(void)k_work_submit(&worker_rx_work);
#endif
}

void meshtastic_worker_rx_on_rx(const struct meshtastic_packet *pkt, const uint8_t *wire,
				size_t wire_len)
{
	struct worker_rx_frame f;

	if (pkt == NULL || wire == NULL || wire_len == 0U || wire_len > sizeof(f.wire)) {
		return;
	}

	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	if (pkt->to != MESHTASTIC_NODE_BROADCAST || pkt->pki_encrypted) {
		worker_rx.stats.not_broadcast++;
		goto out;
	}
	worker_rx.stats.heard++;
	if (pkt->portnum != MESHTASTIC_PORT_TEXT_MESSAGE) {
		worker_rx.stats.not_text++;
		goto out;
	}
	if (worker_rx.peer == 0U) {
		worker_rx.stats.no_peer++;
		goto out;
	}
	/* A flood of text on this preset -- a broken or hostile node -- must not
	 * saturate the peer link or the receiving half. */
	{
		int64_t now = k_uptime_get();

		if (worker_rx.window_start_ms == 0 ||
		    now - worker_rx.window_start_ms >=
			    (int64_t)CONFIG_MESHTASTIC_WORKER_RX_RATE_WINDOW_SEC * 1000) {
			worker_rx.window_start_ms = (now == 0) ? 1 : now;
			worker_rx.used = 0U;
		}
		if (worker_rx.used >= CONFIG_MESHTASTIC_WORKER_RX_RATE_MAX) {
			worker_rx.stats.rate_dropped++;
			goto out;
		}
		worker_rx.used++;
	}
	f.len = (uint16_t)wire_len;
	f.rssi = pkt->rssi;
	f.snr = pkt->snr;
	f.preset = (uint8_t)mt.modem_preset;
	f.rx_ms = (uint32_t)k_uptime_get();
	memcpy(f.wire, wire, wire_len);
	if (k_msgq_put(&worker_rx_q, &f, K_NO_WAIT) != 0) {
		worker_rx.stats.queue_full++;
		goto out;
	}
	k_mutex_unlock(&worker_rx_lock);
	worker_rx_submit();
	return;

out:
	k_mutex_unlock(&worker_rx_lock);
}

/* ---- persistence: the peer survives a reboot (mtear/peer) --------------------- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static void worker_rx_save(uint32_t peer)
{
	if (settings_save_one("mtear/peer", &peer, sizeof(peer)) != 0) {
		LOG_WRN("worker-rx: settings save failed");
	}
}

static int worker_rx_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint32_t peer;

	if (strcmp(key, "peer") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(peer) || read_cb(cb_arg, &peer, len) != (ssize_t)len) {
		return -EINVAL;
	}
	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	worker_rx.peer = peer;
	k_mutex_unlock(&worker_rx_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_worker_rx, "mtear", NULL, worker_rx_settings_set, NULL, NULL);

/* The role intent: the receiving half this rx-only worker forwards to. */
static bool worker_rx_intent_is_set(char *detail, size_t len)
{
	uint32_t peer;

	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	peer = worker_rx.peer;
	k_mutex_unlock(&worker_rx_lock);
	if (peer != 0U) {
		snprintf(detail, len, "peer 0x%08x", peer);
	}
	return peer != 0U;
}

static int worker_rx_intent_clear(void)
{
	int ret;

	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	worker_rx.peer = 0U;
	k_mutex_unlock(&worker_rx_lock);
	ret = settings_delete("mtear/peer");
	return (ret == -ENOENT) ? 0 : ret;
}

MESHTASTIC_INTENT_DEFINE(worker_rx, "worker-rx", "mtear/peer", worker_rx_intent_is_set, worker_rx_intent_clear);

static void worker_rx_forget(void)
{
	(void)settings_delete("mtear/peer");
}
#else
static void worker_rx_save(uint32_t peer)
{
	ARG_UNUSED(peer);
}

static void worker_rx_forget(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

void meshtastic_worker_rx_set_peer(uint32_t node_id)
{
	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	worker_rx.peer = node_id;
	k_mutex_unlock(&worker_rx_lock);
	worker_rx_save(node_id);
}

uint32_t meshtastic_worker_rx_get_peer(void)
{
	uint32_t peer;

	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	peer = worker_rx.peer;
	k_mutex_unlock(&worker_rx_lock);
	return peer;
}

void meshtastic_worker_rx_stats_get(struct meshtastic_worker_rx_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	*out = worker_rx.stats;
	k_mutex_unlock(&worker_rx_lock);
}

void meshtastic_worker_rx_reset(void)
{
	k_msgq_purge(&worker_rx_q);
	k_mutex_lock(&worker_rx_lock, K_FOREVER);
	worker_rx.peer = CONFIG_MESHTASTIC_WORKER_RX_PEER;
	memset(&worker_rx.stats, 0, sizeof(worker_rx.stats));
	worker_rx.window_start_ms = 0;
	worker_rx.used = 0U;
	k_mutex_unlock(&worker_rx_lock);
	worker_rx_forget();
}
