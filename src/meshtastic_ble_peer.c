/* SPDX-License-Identifier: GPL-3.0
 *
 * Node-to-node BLE peer link, peripheral half (agents-a4it.3): a GATT service
 * carrying a 16-byte heartbeat (meshtastic_ble_peer_codec.h) in both
 * directions — a peer central WRITES its beats to us, and reads ours by
 * subscribing to NOTIFY on the same characteristic.
 *
 * This is deliberately NOT the Meshtastic phone service: that one is a
 * single-client stateful session (one queue, one from_num, one frame cursor)
 * whose FromRadio frames a peer would consume, and connecting to it triggers a
 * full want_config sync storm. The peer link starts as a heartbeat and stays
 * separable from config sync by construction.
 *
 * SECURITY: none, deliberately. Two no-IO nodes can only pair Just Works —
 * encryption with zero authentication — protecting a heartbeat whose only
 * content (the node number) is already broadcast in the advertisement. Plain
 * BT_GATT_PERM_* here, NOT the phone service's MT_GATT_PERM_* (which picks up
 * _AUTHEN under MESHTASTIC_BLE_FIXED_PASSKEY and would demand MITM pairing
 * two no-IO nodes cannot perform).
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#include "meshtastic_core.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* A peer link only exists next to a phone link; one connection slot cannot
 * hold both. Fail the build rather than silently refusing the second
 * connection at runtime. */
BUILD_ASSERT(CONFIG_BT_MAX_CONN >= 2,
	     "MESHTASTIC_BLE_PEER needs CONFIG_BT_MAX_CONN >= 2 (phone + peer)");

/*
 * Fresh 128-bit UUIDs in this port's own namespace — deliberately NOT upstream
 * Meshtastic UUIDs. The phone app filters on the Meshtastic service UUID and
 * must never discover a peer service it would try to speak PhoneAPI to; and
 * nobody should mistake these for protocol shared with stock firmware.
 */
#define BT_UUID_MESHTASTIC_PEER_SERVICE_VAL                                                        \
	BT_UUID_128_ENCODE(0xf3f1a2c0, 0x8e5e, 0x4d8b, 0x9f2a, 0x6c1d3b7e5a10)
#define BT_UUID_MESHTASTIC_PEER_BEAT_VAL                                                           \
	BT_UUID_128_ENCODE(0xf3f1a2c1, 0x8e5e, 0x4d8b, 0x9f2a, 0x6c1d3b7e5a10)

static struct bt_uuid_128 peer_service_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_PEER_SERVICE_VAL);
static struct bt_uuid_128 peer_beat_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_PEER_BEAT_VAL);

enum {
	MESHTASTIC_PEER_ATTR_SERVICE = 0,
	MESHTASTIC_PEER_ATTR_BEAT_CHRC,
	MESHTASTIC_PEER_ATTR_BEAT_VALUE,
	MESHTASTIC_PEER_ATTR_BEAT_CCC,
};

/* All mutated from thread context (BT RX thread, system workqueue, shell);
 * never from ISR. One mutex covers both directions' state. */
static K_MUTEX_DEFINE(peer_lock);

static struct {
	/* TX (notify) side: seq restarts from 0 on every subscribe edge and the
	 * first beat then carries HELLO, so the receiver's accounting resyncs.
	 * Single-subscriber by design for now (CONFIG_BT_MAX_CONN=2 leaves one
	 * non-phone slot); revisit per-conn TX state before raising that. */
	bool notify_enabled;
	bool hello_pending;
	uint32_t tx_seq;
	uint32_t tx_beats;
	/* RX (write) side, per connection registry slot. */
	struct meshtastic_ble_peer_rx rx[MESHTASTIC_BLE_REG_SLOTS];
	uint32_t rx_bad; /* frames that failed decode, all links */
} peer;

static void beat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	k_mutex_lock(&peer_lock, K_FOREVER);
	peer.notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	if (peer.notify_enabled) {
		peer.tx_seq = 0U;
		peer.hello_pending = true;
	}
	k_mutex_unlock(&peer_lock);

	LOG_INF("BLE peer beat notify %s", value == BT_GATT_CCC_NOTIFY ? "on" : "off");
}

static ssize_t write_beat(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	struct meshtastic_ble_peer_beat beat;
	unsigned int index = bt_conn_index(conn);
	int ret;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	ret = meshtastic_ble_peer_beat_decode(buf, len, &beat);
	if (ret < 0) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.rx_bad++;
		k_mutex_unlock(&peer_lock);
		LOG_WRN("BLE peer beat rejected (%d, len=%u)", ret, len);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (index < MESHTASTIC_BLE_REG_SLOTS) {
		meshtastic_ble_peer_rx_account(&peer.rx[index], &beat);
	}
	k_mutex_unlock(&peer_lock);

	LOG_DBG("BLE peer beat from 0x%08x seq=%u up=%us%s", beat.node_num, beat.seq,
		beat.uptime_s, (beat.flags & MESHTASTIC_BLE_PEER_FLAG_HELLO) ? " HELLO" : "");
	return len;
}

BT_GATT_SERVICE_DEFINE(meshtastic_peer_svc,
		       BT_GATT_PRIMARY_SERVICE(&peer_service_uuid.uuid),
		       BT_GATT_CHARACTERISTIC(&peer_beat_uuid.uuid,
					      BT_GATT_CHRC_WRITE |
						      BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						      BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_WRITE, NULL, write_beat, NULL),
		       BT_GATT_CCC(beat_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

int meshtastic_ble_peer_send_beat(void)
{
	uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN];
	struct meshtastic_ble_peer_beat beat;
	int ret;

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (!peer.notify_enabled) {
		k_mutex_unlock(&peer_lock);
		return -ENOTCONN;
	}
	beat.flags = peer.hello_pending ? MESHTASTIC_BLE_PEER_FLAG_HELLO : 0U;
	beat.node_num = mt.node_id;
	beat.seq = peer.tx_seq;
	beat.uptime_s = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
	k_mutex_unlock(&peer_lock);

	meshtastic_ble_peer_beat_encode(&beat, buf);

	ret = bt_gatt_notify(NULL, &meshtastic_peer_svc.attrs[MESHTASTIC_PEER_ATTR_BEAT_VALUE],
			     buf, sizeof(buf));
	if (ret == 0) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.tx_seq++;
		peer.tx_beats++;
		peer.hello_pending = false;
		k_mutex_unlock(&peer_lock);
	}
	return ret;
}

bool meshtastic_ble_peer_notify_ready(void)
{
	return peer.notify_enabled;
}

bool meshtastic_ble_peer_rx_get(unsigned int index, struct meshtastic_ble_peer_rx *out)
{
	bool valid = false;

	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return false;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (peer.rx[index].beats > 0U) {
		*out = peer.rx[index];
		valid = true;
	}
	k_mutex_unlock(&peer_lock);
	return valid;
}

void meshtastic_ble_peer_conn_down(unsigned int index)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	meshtastic_ble_peer_rx_reset(&peer.rx[index]);
	k_mutex_unlock(&peer_lock);
}
