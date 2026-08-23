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
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
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

/* Central-half work (defined with the central code below): the scan restarter
 * and the beat engine, both on the SYSTEM workqueue. */
static void scan_work_fn(struct k_work *work);
static K_WORK_DEFINE(scan_work, scan_work_fn);
static void beat_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(beat_work, beat_work_fn);

static struct {
	/* TX (notify) side: seq restarts from 0 on every subscribe edge and the
	 * first beat then carries HELLO, so the receiver's accounting resyncs.
	 * Single-subscriber by design for now (CONFIG_BT_MAX_CONN=2 leaves one
	 * non-phone slot); revisit per-conn TX state before raising that. */
	bool notify_enabled;
	bool hello_pending;
	uint32_t tx_seq;
	/* RX (write) side, per connection registry slot. */
	struct meshtastic_ble_peer_rx rx[MESHTASTIC_BLE_REG_SLOTS];
	struct meshtastic_ble_peer_stats stats;
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

	if (value == BT_GATT_CCC_NOTIFY) {
		/* Subscribe edge: start beating toward the new central. */
		(void)k_work_schedule(&beat_work, K_NO_WAIT);
	}
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
		peer.stats.hello_malformed++;
		k_mutex_unlock(&peer_lock);
		LOG_WRN("BLE peer beat rejected (%d, len=%u)", ret, len);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	/* A valid beat IS the peer evidence for an unclassified incoming
	 * connection (HELLO or not — a phone never writes this char). */
	ret = meshtastic_ble_classify_peer_evidence(index);
	if (ret == 0) {
		LOG_INF("BLE conn %u classified peer (beat from 0x%08x)", index, beat.node_num);
	} else if (ret == -EBUSY) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.hello_rejected_late++;
		k_mutex_unlock(&peer_lock);
		LOG_WRN("BLE conn %u: peer beat on a phone-classified link, ignored", index);
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
		peer.stats.notify_tx_beats++;
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

/* ============================ central half (a4it.5) ========================
 *
 * Follows zephyr/samples/bluetooth/central_ht: scan -> filter on AD ->
 * bt_le_scan_stop() -> bt_conn_le_create() -> discover PRIMARY then
 * CHARACTERISTIC then DESCRIPTOR -> bt_gatt_subscribe(); the scan resumes
 * after a connection completes or fails.
 *
 * PASSIVE scan, deliberately: the discriminator (service UUID + mfg blob) is
 * in the advertisement, not the scan response, so passive costs no TX — which
 * matters for power and for ESP32 radio contention with advertising and the
 * phone link. Scanning is default-off and armed from the shell, so a first
 * bench session can attribute any instability to it.
 *
 * Threading: scan/conn/GATT callbacks run on the BT RX thread; the scan
 * restarter and beat engine run on the SYSTEM workqueue (not ble.work_q — a
 * PhoneAPI config-sync build there would stretch the classification window
 * unboundedly). peer_lock guards the shared scalars; BT API calls are made
 * outside the lock with a ref held where a teardown could race.
 */

static struct {
	bool scan_on;   /* user intent (shell) */
	bool scanning;  /* bt_le_scan actually running */
	uint32_t target_node; /* 0 = any peer */
	struct bt_conn *conn; /* the one outbound link */
	uint32_t conn_node;
	bool link_ready; /* discovery + subscribe complete */
	uint16_t value_handle;
	uint32_t tx_seq;
	bool hello_pending;
	struct meshtastic_ble_peer_seen seen[CONFIG_MESHTASTIC_BLE_PEER_SEEN_MAX];
} central;

static struct bt_gatt_discover_params central_disc;
static struct bt_gatt_subscribe_params central_sub;
static struct bt_uuid_128 central_disc_uuid128;
static struct bt_uuid_16 central_disc_uuid16;

static void seen_note(uint32_t node_num, int8_t rssi)
{
	unsigned int oldest = 0U;
	int64_t oldest_ms = INT64_MAX;

	k_mutex_lock(&peer_lock, K_FOREVER);
	for (unsigned int i = 0U; i < ARRAY_SIZE(central.seen); i++) {
		if (central.seen[i].node_num == node_num) {
			oldest = i;
			goto store;
		}
		if (central.seen[i].last_ms < oldest_ms) {
			oldest_ms = central.seen[i].last_ms;
			oldest = i;
		}
	}
store:
	central.seen[oldest].node_num = node_num;
	central.seen[oldest].rssi = rssi;
	central.seen[oldest].last_ms = k_uptime_get();
	k_mutex_unlock(&peer_lock);
}

struct peer_ad_match {
	bool service_uuid;
	bool blob;
	uint32_t node_num;
};

static bool peer_ad_cb(struct bt_data *data, void *user_data)
{
	struct peer_ad_match *m = user_data;

	switch (data->type) {
	case BT_DATA_UUID128_SOME:
	case BT_DATA_UUID128_ALL:
		for (uint16_t off = 0U; off + 16U <= data->data_len; off += 16U) {
			if (memcmp(&data->data[off], meshtastic_ble_service_uuid128(), 16U) ==
			    0) {
				m->service_uuid = true;
			}
		}
		return true;
	case BT_DATA_MANUFACTURER_DATA:
		/* BOTH discriminators are required: an unrelated 0xFFFF user
		 * is not a peer, and a Meshtastic phone advert has no blob. */
		if (meshtastic_ble_peer_adv_decode(data->data, data->data_len, &m->node_num) ==
		    0) {
			m->blob = true;
		}
		return true;
	default:
		return true;
	}
}

static void peer_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
			 struct net_buf_simple *buf)
{
	struct peer_ad_match m = {0};
	bool connect_now = false;
	int err;

	if (adv_type != BT_GAP_ADV_TYPE_ADV_IND) {
		return;
	}

	bt_data_parse(buf, peer_ad_cb, &m);
	if (!m.service_uuid || !m.blob) {
		return;
	}

	/* Reflection guard: our own advert is not a peer. */
	if (m.node_num == mt.node_id) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.reflections++;
		k_mutex_unlock(&peer_lock);
		return;
	}

	seen_note(m.node_num, rssi);

	k_mutex_lock(&peer_lock, K_FOREVER);
	peer.stats.adverts_matched++;
	if (central.conn == NULL && central.scanning &&
	    (central.target_node == 0U || central.target_node == m.node_num)) {
		connect_now = true;
	}
	k_mutex_unlock(&peer_lock);

	if (!connect_now) {
		return;
	}

	err = bt_le_scan_stop();
	if (err != 0) {
		LOG_WRN("BLE peer scan stop failed (%d)", err);
		return;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	central.scanning = false;
	central.conn_node = m.node_num;
	peer.stats.connects_attempted++;
	k_mutex_unlock(&peer_lock);

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&central.conn);
	if (err != 0) {
		LOG_WRN("BLE peer connect to 0x%08x failed to start (%d)", m.node_num, err);
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.connects_failed++;
		k_mutex_unlock(&peer_lock);
		(void)k_work_submit(&scan_work);
		return;
	}

	LOG_INF("BLE peer connecting to 0x%08x (RSSI %d)", m.node_num, rssi);
}

static void scan_work_fn(struct k_work *work)
{
	bool start = false;
	int err;

	ARG_UNUSED(work);

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (central.scan_on && !central.scanning && central.conn == NULL) {
		start = true;
	}
	k_mutex_unlock(&peer_lock);

	if (!start) {
		return;
	}

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, peer_scan_cb);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("BLE peer scan start failed (%d)", err);
		return;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	central.scanning = true;
	k_mutex_unlock(&peer_lock);
	LOG_INF("BLE peer scan running (passive)");
}

static uint8_t central_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				 const void *data, uint16_t length)
{
	struct meshtastic_ble_peer_beat beat;
	unsigned int index;

	if (data == NULL) {
		/* Unsubscribed (link going down); the disconnect path resets
		 * the rest of the state. */
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	if (meshtastic_ble_peer_beat_decode(data, length, &beat) < 0) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.hello_malformed++;
		k_mutex_unlock(&peer_lock);
		return BT_GATT_ITER_CONTINUE;
	}

	index = bt_conn_index(conn);
	k_mutex_lock(&peer_lock, K_FOREVER);
	if (index < MESHTASTIC_BLE_REG_SLOTS) {
		meshtastic_ble_peer_rx_account(&peer.rx[index], &beat);
	}
	k_mutex_unlock(&peer_lock);
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t central_discover_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		LOG_WRN("BLE peer discovery: attribute not found (stage %u)", params->type);
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.discovery_failures++;
		k_mutex_unlock(&peer_lock);
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		memcpy(&central_disc_uuid128, &peer_beat_uuid, sizeof(central_disc_uuid128));
		central_disc.uuid = &central_disc_uuid128.uuid;
		central_disc.start_handle = attr->handle + 1U;
		central_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &central_disc);
		if (err != 0) {
			LOG_WRN("BLE peer char discover failed (%d)", err);
		}
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		memcpy(&central_disc_uuid16, BT_UUID_GATT_CCC, sizeof(central_disc_uuid16));
		central_disc.uuid = &central_disc_uuid16.uuid;
		central_disc.start_handle = attr->handle + 2U;
		central_disc.type = BT_GATT_DISCOVER_DESCRIPTOR;
		central_sub.value_handle = bt_gatt_attr_value_handle(attr);
		err = bt_gatt_discover(conn, &central_disc);
		if (err != 0) {
			LOG_WRN("BLE peer CCC discover failed (%d)", err);
		}
	} else {
		central_sub.notify = central_notify_cb;
		central_sub.value = BT_GATT_CCC_NOTIFY;
		central_sub.ccc_handle = attr->handle;
		err = bt_gatt_subscribe(conn, &central_sub);
		if (err != 0 && err != -EALREADY) {
			LOG_WRN("BLE peer subscribe failed (%d)", err);
			k_mutex_lock(&peer_lock, K_FOREVER);
			peer.stats.discovery_failures++;
			k_mutex_unlock(&peer_lock);
		} else {
			k_mutex_lock(&peer_lock, K_FOREVER);
			central.value_handle = central_sub.value_handle;
			central.link_ready = true;
			central.tx_seq = 0U;
			central.hello_pending = true;
			k_mutex_unlock(&peer_lock);
			LOG_INF("BLE peer link to 0x%08x ready (beats every %u ms)",
				central.conn_node, CONFIG_MESHTASTIC_BLE_PEER_BEAT_PERIOD_MS);
			(void)k_work_schedule(&beat_work, K_NO_WAIT);
		}
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static int central_send_beat(void)
{
	uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN];
	struct meshtastic_ble_peer_beat beat;
	struct bt_conn *conn;
	uint16_t handle;
	int ret;

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (!central.link_ready || central.conn == NULL) {
		k_mutex_unlock(&peer_lock);
		return -ENOTCONN;
	}
	conn = bt_conn_ref(central.conn);
	handle = central.value_handle;
	beat.flags = central.hello_pending ? MESHTASTIC_BLE_PEER_FLAG_HELLO : 0U;
	beat.node_num = mt.node_id;
	beat.seq = central.tx_seq;
	beat.uptime_s = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
	k_mutex_unlock(&peer_lock);

	meshtastic_ble_peer_beat_encode(&beat, buf);
	ret = bt_gatt_write_without_response(conn, handle, buf, sizeof(buf), false);
	bt_conn_unref(conn);

	if (ret == 0) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		central.tx_seq++;
		central.hello_pending = false;
		peer.stats.write_tx_beats++;
		k_mutex_unlock(&peer_lock);
	}
	return ret;
}

/* One engine, both roles: notify toward a subscribed central, write toward a
 * connected peripheral. Parks itself when neither link is active and is
 * re-armed by the subscribe edges. */
static void beat_work_fn(struct k_work *work)
{
	bool any = false;

	ARG_UNUSED(work);

	if (meshtastic_ble_peer_notify_ready()) {
		(void)meshtastic_ble_peer_send_beat();
		any = true;
	}
	if (central_send_beat() != -ENOTCONN) {
		any = true;
	}

	if (any) {
		(void)k_work_reschedule(&beat_work,
					K_MSEC(CONFIG_MESHTASTIC_BLE_PEER_BEAT_PERIOD_MS));
	}
}

static void peer_central_connected(struct bt_conn *conn, uint8_t err)
{
	int ret;

	if (conn != central.conn) {
		return;
	}

	if (err != 0U) {
		LOG_WRN("BLE peer connect to 0x%08x failed (0x%02x)", central.conn_node, err);
		bt_conn_unref(central.conn);
		k_mutex_lock(&peer_lock, K_FOREVER);
		central.conn = NULL;
		peer.stats.connects_failed++;
		k_mutex_unlock(&peer_lock);
		(void)k_work_submit(&scan_work);
		return;
	}

	/* Outbound role CENTRAL: meshtastic_ble.c's connected() has already
	 * registered this slot as a peer. Start the discovery chain. */
	memcpy(&central_disc_uuid128, &peer_service_uuid, sizeof(central_disc_uuid128));
	central_disc.uuid = &central_disc_uuid128.uuid;
	central_disc.func = central_discover_cb;
	central_disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	central_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	central_disc.type = BT_GATT_DISCOVER_PRIMARY;

	ret = bt_gatt_discover(conn, &central_disc);
	if (ret != 0) {
		LOG_WRN("BLE peer discovery failed to start (%d)", ret);
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.discovery_failures++;
		k_mutex_unlock(&peer_lock);
	}
}

static void peer_central_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != central.conn) {
		return;
	}

	LOG_INF("BLE peer link to 0x%08x down (0x%02x)", central.conn_node, reason);
	bt_conn_unref(central.conn);
	k_mutex_lock(&peer_lock, K_FOREVER);
	central.conn = NULL;
	central.link_ready = false;
	central.value_handle = 0U;
	central.conn_node = 0U;
	k_mutex_unlock(&peer_lock);

	/* Resume the hunt off the BT callback context. */
	(void)k_work_submit(&scan_work);
}

BT_CONN_CB_DEFINE(peer_conn_callbacks) = {
	.connected = peer_central_connected,
	.disconnected = peer_central_disconnected,
};

int meshtastic_ble_peer_scan_set(bool on)
{
	int err = 0;

	k_mutex_lock(&peer_lock, K_FOREVER);
	central.scan_on = on;
	if (!on) {
		central.target_node = 0U;
	}
	k_mutex_unlock(&peer_lock);

	if (on) {
		(void)k_work_submit(&scan_work);
	} else {
		err = bt_le_scan_stop();
		if (err == -EALREADY) {
			err = 0;
		}
		k_mutex_lock(&peer_lock, K_FOREVER);
		central.scanning = false;
		k_mutex_unlock(&peer_lock);
	}
	return err;
}

bool meshtastic_ble_peer_scan_armed(void)
{
	return central.scan_on;
}

int meshtastic_ble_peer_connect(uint32_t node_num)
{
	if (node_num == mt.node_id) {
		return -EINVAL;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (central.conn != NULL) {
		k_mutex_unlock(&peer_lock);
		return -EALREADY;
	}
	central.target_node = node_num;
	central.scan_on = true;
	k_mutex_unlock(&peer_lock);

	(void)k_work_submit(&scan_work);
	return 0;
}

int meshtastic_ble_peer_disconnect(void)
{
	struct bt_conn *conn = NULL;
	int err = -ENOTCONN;

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (central.conn != NULL) {
		conn = bt_conn_ref(central.conn);
	}
	k_mutex_unlock(&peer_lock);

	if (conn != NULL) {
		err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		bt_conn_unref(conn);
	}
	return err;
}

bool meshtastic_ble_peer_seen_get(unsigned int i, struct meshtastic_ble_peer_seen *out)
{
	bool valid = false;

	if (i >= ARRAY_SIZE(central.seen)) {
		return false;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (central.seen[i].node_num != 0U) {
		*out = central.seen[i];
		valid = true;
	}
	k_mutex_unlock(&peer_lock);
	return valid;
}

void meshtastic_ble_peer_link_get(struct meshtastic_ble_peer_link *out)
{
	k_mutex_lock(&peer_lock, K_FOREVER);
	out->connected = (central.conn != NULL);
	out->ready = central.link_ready;
	out->node_num = central.conn_node;
	out->index = (central.conn != NULL) ? bt_conn_index(central.conn) : 0U;
	out->tx_beats = peer.stats.write_tx_beats;
	k_mutex_unlock(&peer_lock);
}

void meshtastic_ble_peer_stats_get(struct meshtastic_ble_peer_stats *out)
{
	k_mutex_lock(&peer_lock, K_FOREVER);
	*out = peer.stats;
	k_mutex_unlock(&peer_lock);
}
