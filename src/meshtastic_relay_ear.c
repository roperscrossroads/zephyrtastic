/* SPDX-License-Identifier: GPL-3.0 */

/*
 * Cross-preset relay, the ear (agents-jbrq.12, hole H33).
 *
 * The BLE peer link used to carry only a node's OWN outbound unicasts (the TX
 * divert). Nothing passed on what a node HEARD. The ear does exactly that, for
 * the one kind of frame a relay can use: a broadcast text on a channel this
 * node decrypts. It forwards the wire bytes unchanged; the receiving half
 * (meshtastic_relay.c) reads them through its own copy of the channel and
 * decides what crosses.
 *
 * Forwarding never runs on the LoRa receive thread: frames are queued and sent
 * from the BLE module's work queue, because a GATT write can block on buffers.
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

#include "meshtastic_core.h"
#include "meshtastic_relay.h"
#include "meshtastic_attachment_codec.h"
#if defined(CONFIG_MESHTASTIC_ATTACH_BEARER)
#include "meshtastic_attach_bearer.h"
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include "meshtastic_ble_peer.h" /* meshtastic_ble_work_submit, frame_send_to */
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

struct ear_frame {
	uint16_t len;
	/* What this radio saw, for a peer that takes attachment envelopes
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
	struct meshtastic_relay_ear_stats stats;
} ear = {
	.peer = CONFIG_MESHTASTIC_RELAY_EAR_PEER,
};

static K_MUTEX_DEFINE(ear_lock);
K_MSGQ_DEFINE(ear_q, sizeof(struct ear_frame), CONFIG_MESHTASTIC_RELAY_EAR_QUEUE_SIZE, 4);

static void ear_work_fn(struct k_work *work);
static K_WORK_DEFINE(ear_work, ear_work_fn);

__weak int meshtastic_relay_ear_send(uint32_t peer, const uint8_t *wire, size_t wire_len)
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

/* The same seam for an attachment envelope, over whichever bearer reaches the
 * peer (ATTACHMENT-SCOPE §4). */
__weak int meshtastic_relay_ear_send_env(uint32_t peer, const uint8_t *env, size_t env_len)
{
#if defined(CONFIG_MESHTASTIC_ATTACH_BEARER)
	return meshtastic_attach_bearer_send(peer, env, env_len);
#else
	ARG_UNUSED(peer);
	ARG_UNUSED(env);
	ARG_UNUSED(env_len);
	return -ENOTSUP;
#endif
}

/* Does the peer take attachment envelopes? The bearer knows (on BLE, the
 * peer's beat flag). Without a bearer the test seam decides. */
__weak bool meshtastic_relay_ear_peer_takes_env(uint32_t peer)
{
#if defined(CONFIG_MESHTASTIC_ATTACH_BEARER)
	struct meshtastic_attach_link_info info;

	return meshtastic_attach_bearer_link_info(peer, &info, NULL) && info.up &&
	       info.takes_envelopes;
#else
	ARG_UNUSED(peer);
	return false;
#endif
}

static int ear_forward(uint32_t peer, const struct ear_frame *f)
{
	if (meshtastic_relay_ear_peer_takes_env(peer)) {
		/* A brain (ATTACHMENT-DESIGN P0): the frame with the signal this
		 * radio saw, so the brain treats it as heard on THIS preset by THIS
		 * radio -- an RF frame, not a bearer frame. */
		static uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX]; /* one worker */
		const struct meshtastic_attachment_rx_frame m = {
			.preset = f->preset,
			.rssi = f->rssi,
			.snr = f->snr,
			.rx_ms = f->rx_ms,
			.flags = 0U,
			.wire = f->wire,
			.wire_len = f->len,
		};
		int len = meshtastic_attachment_encode_rx_frame(&m, env, sizeof(env));

		if (len < 0) {
			return len;
		}
		return meshtastic_relay_ear_send_env(peer, env, (size_t)len);
	}
	/* The receiving half of a relay: the bare frame, as before. */
	return meshtastic_relay_ear_send(peer, f->wire, f->len);
}

static void ear_work_fn(struct k_work *work)
{
	static struct ear_frame f; /* one worker: static keeps it off the stack */
	uint32_t peer;

	ARG_UNUSED(work);

	while (k_msgq_get(&ear_q, &f, K_NO_WAIT) == 0) {
		int ret;

		k_mutex_lock(&ear_lock, K_FOREVER);
		peer = ear.peer;
		k_mutex_unlock(&ear_lock);

		ret = (peer == 0U) ? -EHOSTUNREACH : ear_forward(peer, &f);

		k_mutex_lock(&ear_lock, K_FOREVER);
		if (ret == 0) {
			ear.stats.forwarded++;
		} else {
			ear.stats.send_failed++;
		}
		k_mutex_unlock(&ear_lock);
		if (ret != 0) {
			LOG_DBG("ear: forward to 0x%08x failed (%d)", peer, ret);
		}
	}
}

static void ear_submit(void)
{
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	(void)meshtastic_ble_work_submit(&ear_work);
#else
	/* No BLE stack (native_sim): nothing on this queue can block on GATT. */
	(void)k_work_submit(&ear_work);
#endif
}

void meshtastic_relay_ear_on_rx(const struct meshtastic_packet *pkt, const uint8_t *wire,
				size_t wire_len)
{
	struct ear_frame f;

	if (pkt == NULL || wire == NULL || wire_len == 0U || wire_len > sizeof(f.wire)) {
		return;
	}

	k_mutex_lock(&ear_lock, K_FOREVER);
	if (pkt->to != MESHTASTIC_NODE_BROADCAST || pkt->pki_encrypted) {
		ear.stats.not_broadcast++;
		goto out;
	}
	ear.stats.heard++;
	if (pkt->portnum != MESHTASTIC_PORT_TEXT_MESSAGE) {
		ear.stats.not_text++;
		goto out;
	}
	if (ear.peer == 0U) {
		ear.stats.no_peer++;
		goto out;
	}
	/* A flood of text on this preset -- a broken or hostile node -- must not
	 * saturate the peer link or the receiving half. */
	{
		int64_t now = k_uptime_get();

		if (ear.window_start_ms == 0 ||
		    now - ear.window_start_ms >=
			    (int64_t)CONFIG_MESHTASTIC_RELAY_EAR_RATE_WINDOW_SEC * 1000) {
			ear.window_start_ms = (now == 0) ? 1 : now;
			ear.used = 0U;
		}
		if (ear.used >= CONFIG_MESHTASTIC_RELAY_EAR_RATE_MAX) {
			ear.stats.rate_dropped++;
			goto out;
		}
		ear.used++;
	}
	f.len = (uint16_t)wire_len;
	f.rssi = pkt->rssi;
	f.snr = pkt->snr;
	f.preset = (uint8_t)mt.modem_preset;
	f.rx_ms = (uint32_t)k_uptime_get();
	memcpy(f.wire, wire, wire_len);
	if (k_msgq_put(&ear_q, &f, K_NO_WAIT) != 0) {
		ear.stats.queue_full++;
		goto out;
	}
	k_mutex_unlock(&ear_lock);
	ear_submit();
	return;

out:
	k_mutex_unlock(&ear_lock);
}

/* ---- persistence: the peer survives a reboot (mtear/peer) --------------------- */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static void ear_save(uint32_t peer)
{
	if (settings_save_one("mtear/peer", &peer, sizeof(peer)) != 0) {
		LOG_WRN("ear: settings save failed");
	}
}

static int ear_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint32_t peer;

	if (strcmp(key, "peer") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(peer) || read_cb(cb_arg, &peer, len) != (ssize_t)len) {
		return -EINVAL;
	}
	k_mutex_lock(&ear_lock, K_FOREVER);
	ear.peer = peer;
	k_mutex_unlock(&ear_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_ear, "mtear", NULL, ear_settings_set, NULL, NULL);

static void ear_forget(void)
{
	(void)settings_delete("mtear/peer");
}
#else
static void ear_save(uint32_t peer)
{
	ARG_UNUSED(peer);
}

static void ear_forget(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

void meshtastic_relay_ear_set_peer(uint32_t node_id)
{
	k_mutex_lock(&ear_lock, K_FOREVER);
	ear.peer = node_id;
	k_mutex_unlock(&ear_lock);
	ear_save(node_id);
}

uint32_t meshtastic_relay_ear_get_peer(void)
{
	uint32_t peer;

	k_mutex_lock(&ear_lock, K_FOREVER);
	peer = ear.peer;
	k_mutex_unlock(&ear_lock);
	return peer;
}

void meshtastic_relay_ear_stats_get(struct meshtastic_relay_ear_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&ear_lock, K_FOREVER);
	*out = ear.stats;
	k_mutex_unlock(&ear_lock);
}

void meshtastic_relay_ear_reset(void)
{
	k_msgq_purge(&ear_q);
	k_mutex_lock(&ear_lock, K_FOREVER);
	ear.peer = CONFIG_MESHTASTIC_RELAY_EAR_PEER;
	memset(&ear.stats, 0, sizeof(ear.stats));
	ear.window_start_ms = 0;
	ear.used = 0U;
	k_mutex_unlock(&ear_lock);
	ear_forget();
}
