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
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
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

static K_MUTEX_DEFINE(head_lock);
K_MSGQ_DEFINE(head_q, sizeof(struct head_frame), CONFIG_MESHTASTIC_ATTACHMENT_HEAD_QUEUE_SIZE, 4);

static void head_work_fn(struct k_work *work);
static K_WORK_DEFINE(head_work, head_work_fn);
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
		.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD | MESHTASTIC_ATTACHMENT_ST_RX_ONLY,
		.hwid = mt.node_id,
		.brain = brain,
		.rx_frames = head.stats.forwarded,
		.tx_frames = 0U,
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

static void head_status_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	head_submit(&head_status_work);
	(void)k_work_schedule(&head_status_timer,
			      K_SECONDS(CONFIG_MESHTASTIC_ATTACHMENT_HEAD_STATUS_PERIOD_SEC));
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
	case MESHTASTIC_ATTACHMENT_TX_FRAME:
		/* Phase 3. Counted as refused so the brain's TX_RESULT-less send
		 * is visible on both ends. */
		head.stats.refused++;
		ret = -EPERM;
		break;
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

void meshtastic_attachment_head_set_brain(uint32_t node)
{
	k_mutex_lock(&head_lock, K_FOREVER);
	head.brain = node;
	k_mutex_unlock(&head_lock);
	head_save(node);
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
	k_msgq_purge(&head_q);
	k_mutex_lock(&head_lock, K_FOREVER);
	head.brain = CONFIG_MESHTASTIC_ATTACHMENT_HEAD_BRAIN_ID;
	memset(&head.stats, 0, sizeof(head.stats));
	k_mutex_unlock(&head_lock);
	head_forget();
}

void meshtastic_attachment_head_start(void)
{
	(void)k_work_schedule(&head_status_timer,
			      K_SECONDS(CONFIG_MESHTASTIC_ATTACHMENT_HEAD_STATUS_PERIOD_SEC));
}
