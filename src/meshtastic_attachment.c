/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The brain's attachment table and ingest (meshtastic_attachment.h;
 * ATTACHMENT-DESIGN S6). A head's RX_FRAME becomes an RF frame on the router's
 * queue with the head's preset and signal attached -- from there dedup, decrypt,
 * NodeDB and the modules see it exactly as they would a frame from this board's
 * own radio, because it is one: it crossed the air, on the head's preset.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>
#include "meshtastic_core.h"
#include "meshtastic_attachment.h"
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include "meshtastic_ble_peer_codec.h" /* the ENV_MAX cross-check only */
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
BUILD_ASSERT(MESHTASTIC_ATTACHMENT_ENV_MAX == MESHTASTIC_BLE_PEER_ENV_MAX,
	     "the envelope must fit the peer link's kind-1 frame");
#endif

/* [0] is the local radio and never admitted or forgotten. */
static struct meshtastic_attachment_info tab[CONFIG_MESHTASTIC_ATTACHMENT_MAX + 1U];
static bool used[CONFIG_MESHTASTIC_ATTACHMENT_MAX + 1U] = { true };
static K_MUTEX_DEFINE(tab_lock);

static struct meshtastic_attachment_info *find_locked(uint32_t node)
{
	for (unsigned int i = 1U; i < ARRAY_SIZE(tab); i++) {
		if (used[i] && tab[i].node == node) {
			return &tab[i];
		}
	}
	return NULL;
}

static struct meshtastic_attachment_info *admit_locked(uint32_t node)
{
	for (unsigned int i = 1U; i < ARRAY_SIZE(tab); i++) {
		if (!used[i]) {
			memset(&tab[i], 0, sizeof(tab[i]));
			tab[i].id = (uint8_t)i;
			tab[i].node = node;
			tab[i].preset = MESHTASTIC_PRESET_UNKNOWN;
			tab[i].rssi_min = INT16_MAX;
			tab[i].rssi_max = INT16_MIN;
			used[i] = true;
			LOG_INF("attach: head 0x%08x admitted as attachment %u", node, i);
			return &tab[i];
		}
	}
	return NULL;
}

/* The send seam: whichever registered bearer has a live link to the head. A
 * test overrides this, or registers a bearer of its own. */
__weak int meshtastic_attachment_send(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_attach_bearer_send(node, env, len);
}

int meshtastic_attachment_ingest(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_attachment_ingest_from(NULL, node, env, len);
}

void meshtastic_attachment_link_down(const struct meshtastic_attach_bearer *b, uint32_t node)
{
	struct meshtastic_attachment_info *a;

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a != NULL && (a->bearer == b || a->bearer == NULL)) {
		a->link_up = false;
		a->down_ms = k_uptime_get();
		LOG_INF("attach: head 0x%08x (attachment %u) link down", node, a->id);
	}
	k_mutex_unlock(&tab_lock);
}

int meshtastic_attachment_ingest_from(const struct meshtastic_attach_bearer *b, uint32_t node,
				      const uint8_t *env, size_t len)
{
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_info *a;
	int ret;

	if (node == 0U || env == NULL) {
		return -EINVAL;
	}

	ret = meshtastic_attachment_decode(env, len, &msg);

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a == NULL) {
		a = admit_locked(node);
		if (a == NULL) {
			k_mutex_unlock(&tab_lock);
			LOG_WRN("attach: no slot for head 0x%08x", node);
			return -ENOSPC;
		}
	}
	a->bearer = b;
	a->link_up = true;
	a->last_ms = k_uptime_get();
	if (ret < 0) {
		a->rejected++;
		k_mutex_unlock(&tab_lock);
		return -EBADMSG;
	}

	switch (msg.type) {
	case MESHTASTIC_ATTACHMENT_RX_FRAME: {
		const struct meshtastic_rx_meta meta = {
			.bearer = MESHTASTIC_BEARER_ATTACHMENT,
			.attach = a->id,
			.preset = msg.u.rx.preset,
			.rssi = msg.u.rx.rssi,
			.snr = msg.u.rx.snr,
			.rx_ms = msg.u.rx.rx_ms,
		};

		a->preset = msg.u.rx.preset;
		a->last_rssi = msg.u.rx.rssi;
		a->last_snr = msg.u.rx.snr;
		a->rssi_min = MIN(a->rssi_min, msg.u.rx.rssi);
		a->rssi_max = MAX(a->rssi_max, msg.u.rx.rssi);
		ret = meshtastic_radio_rx_inject_meta(msg.u.rx.wire, msg.u.rx.wire_len, &meta);
		if (ret == 0) {
			a->rx_frames++;
		} else {
			a->rx_dropped++;
		}
		break;
	}
	case MESHTASTIC_ATTACHMENT_STATUS:
		a->status = msg.u.status;
		a->have_status = true;
		a->preset = msg.u.status.preset;
		ret = 0;
		break;
	case MESHTASTIC_ATTACHMENT_TX_RESULT:
		a->tx_results++;
		ret = 0;
		break;
	default:
		/* TX_FRAME and SET_PRESET are controls a HEAD accepts; a brain
		 * receiving one is talking to something that is not a head. */
		a->rejected++;
		ret = -EBADMSG;
		break;
	}
	k_mutex_unlock(&tab_lock);
	return ret;
}

unsigned int meshtastic_attachment_count(void)
{
	unsigned int n = 0U;

	k_mutex_lock(&tab_lock, K_FOREVER);
	for (unsigned int i = 0U; i < ARRAY_SIZE(tab); i++) {
		n += used[i] ? 1U : 0U;
	}
	k_mutex_unlock(&tab_lock);
	return n;
}

bool meshtastic_attachment_get(uint8_t id, struct meshtastic_attachment_info *out)
{
	bool ok = false;

	if (id >= ARRAY_SIZE(tab) || out == NULL) {
		return false;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (used[id]) {
		*out = tab[id];
		if (id == 0U) {
			out->preset = (uint8_t)mt.modem_preset;
			out->last_rssi = mt.status.last_rssi;
			out->last_snr = mt.status.last_snr;
		}
		ok = true;
	}
	k_mutex_unlock(&tab_lock);
	return ok;
}

uint8_t meshtastic_attachment_id_for_node(uint32_t node)
{
	struct meshtastic_attachment_info *a;
	uint8_t id = 0U;

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a != NULL) {
		id = a->id;
	}
	k_mutex_unlock(&tab_lock);
	return id;
}

int meshtastic_attachment_forget(uint8_t id)
{
	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	used[id] = false;
	memset(&tab[id], 0, sizeof(tab[id]));
	k_mutex_unlock(&tab_lock);
	return 0;
}

int meshtastic_attachment_set_preset(uint8_t id, uint8_t preset)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_SET_PRESET_LEN];
	uint32_t node;
	int len;

	if (id == 0U) {
		return -EINVAL;
	}
	if (id >= ARRAY_SIZE(tab)) {
		return -ENOENT;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (!used[id]) {
		k_mutex_unlock(&tab_lock);
		return -ENOENT;
	}
	node = tab[id].node;
	k_mutex_unlock(&tab_lock);

	len = meshtastic_attachment_encode_set_preset(preset, env, sizeof(env));
	if (len < 0) {
		return len;
	}
	return meshtastic_attachment_send(node, env, (size_t)len);
}
