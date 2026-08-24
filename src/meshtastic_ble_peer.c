/* SPDX-License-Identifier: GPL-3.0
 *
 * Node-to-node BLE peer link, peripheral half (agents-a4it.3): a GATT service
 * carrying a 16-byte heartbeat (meshtastic_ble_peer_codec.h) in both
 * directions — a peer central WRITES its beats to us, and reads ours by
 * subscribing to NOTIFY on the same characteristic. A second characteristic,
 * the frame channel (agents-xhli.1), carries whole Meshtastic wire frames the
 * same way, chunked so the default 23-byte ATT MTU always suffices.
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
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <zephyr/sys/byteorder.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"

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
#define BT_UUID_MESHTASTIC_PEER_FRAME_VAL                                                          \
	BT_UUID_128_ENCODE(0xf3f1a2c2, 0x8e5e, 0x4d8b, 0x9f2a, 0x6c1d3b7e5a10)

static struct bt_uuid_128 peer_service_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_PEER_SERVICE_VAL);
static struct bt_uuid_128 peer_beat_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_PEER_BEAT_VAL);
static struct bt_uuid_128 peer_frame_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_PEER_FRAME_VAL);

enum {
	MESHTASTIC_PEER_ATTR_SERVICE = 0,
	MESHTASTIC_PEER_ATTR_BEAT_CHRC,
	MESHTASTIC_PEER_ATTR_BEAT_VALUE,
	MESHTASTIC_PEER_ATTR_BEAT_CCC,
	MESHTASTIC_PEER_ATTR_FRAME_CHRC,
	MESHTASTIC_PEER_ATTR_FRAME_VALUE,
	MESHTASTIC_PEER_ATTR_FRAME_CCC,
};

/* The frame channel carries exactly what meshtastic_build_wire_packet()
 * produces; the codec's mirror of the limit must not drift. */
BUILD_ASSERT(MESHTASTIC_BLE_PEER_FRAME_MAX == MESHTASTIC_PKT_MAX,
	     "frame-channel FRAME_MAX must track MESHTASTIC_PKT_MAX");

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
	int64_t rx_last_ms[MESHTASTIC_BLE_REG_SLOTS]; /* k_uptime at last beat */
	/* Frame channel: notify gate + one reassembler per slot (one frame in
	 * flight per direction is the codec's contract). */
	bool frame_notify_enabled;
	struct meshtastic_ble_peer_reasm frame_rx[MESHTASTIC_BLE_REG_SLOTS];
	meshtastic_ble_peer_frame_cb_t frame_cb;
	struct meshtastic_ble_peer_stats stats;
} peer;

void meshtastic_ble_peer_frame_rx_register(meshtastic_ble_peer_frame_cb_t cb)
{
	k_mutex_lock(&peer_lock, K_FOREVER);
	peer.frame_cb = cb;
	k_mutex_unlock(&peer_lock);
}

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
		peer.rx_last_ms[index] = k_uptime_get();
	}
	k_mutex_unlock(&peer_lock);

	LOG_DBG("BLE peer beat from 0x%08x seq=%u up=%us%s", beat.node_num, beat.seq,
		beat.uptime_s, (beat.flags & MESHTASTIC_BLE_PEER_FLAG_HELLO) ? " HELLO" : "");
	return len;
}

static void frame_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	k_mutex_lock(&peer_lock, K_FOREVER);
	peer.frame_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	k_mutex_unlock(&peer_lock);
	LOG_INF("BLE peer frame notify %s", value == BT_GATT_CCC_NOTIFY ? "on" : "off");
}

/* Shared RX tail for both directions of the frame channel: peripheral-side
 * chunk-writes and central-side notifications land here. Returns the
 * reassembler's verdict (1 frame complete, 0 accepted, <0 refused). */
static int frame_chunk_ingest(unsigned int index, const void *buf, uint16_t len)
{
	size_t frame_len = 0U;
	int ret;

	k_mutex_lock(&peer_lock, K_FOREVER);
	ret = meshtastic_ble_peer_reasm_ingest(&peer.frame_rx[index], buf, len, &frame_len);
	if (ret == 1) {
		peer.stats.frame_rx_frames++;
		LOG_DBG("BLE peer frame rx: %zu bytes (conn %u)", frame_len, index);
		if (peer.frame_cb != NULL) {
			/* Contract (meshtastic_ble_peer.h): cb runs here with
			 * the peer lock held and must only copy/queue. */
			peer.frame_cb(index, peer.frame_rx[index].frame, frame_len);
		}
	} else if (ret < 0) {
		peer.stats.frame_rx_rejected++;
		LOG_WRN("BLE peer frame chunk rejected (%d, len=%u, conn %u)", ret, len, index);
	}
	k_mutex_unlock(&peer_lock);
	return ret;
}

static ssize_t write_frame(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			   uint16_t len, uint16_t offset, uint8_t flags)
{
	unsigned int index = bt_conn_index(conn);
	int ret;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Like a beat, a frame chunk is peer evidence — a phone never touches
	 * this characteristic. */
	ret = meshtastic_ble_classify_peer_evidence(index);
	if (ret == -EBUSY) {
		k_mutex_lock(&peer_lock, K_FOREVER);
		peer.stats.hello_rejected_late++;
		k_mutex_unlock(&peer_lock);
		LOG_WRN("BLE conn %u: frame chunk on a phone-classified link, ignored", index);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	} else if (ret == 0) {
		LOG_INF("BLE conn %u classified peer (frame chunk)", index);
	}

	if (frame_chunk_ingest(index, buf, len) < 0) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	return len;
}

BT_GATT_SERVICE_DEFINE(meshtastic_peer_svc,
		       BT_GATT_PRIMARY_SERVICE(&peer_service_uuid.uuid),
		       BT_GATT_CHARACTERISTIC(&peer_beat_uuid.uuid,
					      BT_GATT_CHRC_WRITE |
						      BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						      BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_WRITE, NULL, write_beat, NULL),
		       BT_GATT_CCC(beat_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
		       BT_GATT_CHARACTERISTIC(&peer_frame_uuid.uuid,
					      BT_GATT_CHRC_WRITE |
						      BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						      BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_WRITE, NULL, write_frame, NULL),
		       BT_GATT_CCC(frame_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

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

bool meshtastic_ble_peer_frame_notify_ready(void)
{
	return peer.frame_notify_enabled;
}

int meshtastic_ble_peer_frame_notify(const uint8_t *frame, size_t len)
{
	/* Chunk at the guaranteed minimum ATT payload rather than the live
	 * MTU: correct on every link by construction (the codec's whole
	 * point), merely more chunks when an MTU exchange has succeeded.
	 * Per-conn MTU lookup is an optimization to measure at M3. */
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	struct meshtastic_ble_peer_chunker ck;
	int n;
	int ret;

	if (!peer.frame_notify_enabled) {
		return -ENOTCONN;
	}

	ret = meshtastic_ble_peer_chunker_start(&ck, frame, len);
	if (ret < 0) {
		return ret;
	}

	while ((n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf))) > 0) {
		ret = bt_gatt_notify(NULL,
				     &meshtastic_peer_svc.attrs[MESHTASTIC_PEER_ATTR_FRAME_VALUE],
				     buf, (uint16_t)n);
		if (ret != 0) {
			/* Frame abandoned mid-flight; the receiver's partial
			 * dies on our next FIRST chunk. */
			k_mutex_lock(&peer_lock, K_FOREVER);
			peer.stats.frame_tx_failed++;
			k_mutex_unlock(&peer_lock);
			return ret;
		}
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	peer.stats.frame_tx_frames++;
	k_mutex_unlock(&peer_lock);
	return 0;
}

bool meshtastic_ble_peer_rx_get(unsigned int index, struct meshtastic_ble_peer_rx *out,
				int64_t *last_ms)
{
	bool valid = false;

	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return false;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (peer.rx[index].beats > 0U) {
		*out = peer.rx[index];
		if (last_ms != NULL) {
			*last_ms = peer.rx_last_ms[index];
		}
		valid = true;
	}
	k_mutex_unlock(&peer_lock);
	return valid;
}

/* Poke the beat engine: one immediate beat on every active link. */
void meshtastic_ble_peer_beat_now(void)
{
	(void)k_work_schedule(&beat_work, K_NO_WAIT);
}

void meshtastic_ble_peer_conn_down(unsigned int index)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	meshtastic_ble_peer_rx_reset(&peer.rx[index]);
	meshtastic_ble_peer_reasm_reset(&peer.frame_rx[index]);
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
	bool link_ready; /* beat discovery + subscribe complete */
	uint16_t value_handle;
	/* Frame channel on the outbound link (agents-xhli.2): discovered after
	 * the beat chain. A peer without the characteristic (older image)
	 * downgrades to a beat-only link — frame_ready simply stays false. */
	bool frame_ready;
	uint16_t frame_value_handle;
	uint32_t tx_seq;
	bool hello_pending;
	struct meshtastic_ble_peer_seen seen[CONFIG_MESHTASTIC_BLE_PEER_SEEN_MAX];
} central;

static struct bt_gatt_discover_params central_disc;
static struct bt_gatt_subscribe_params central_sub;
static struct bt_gatt_subscribe_params central_frame_sub;
static struct bt_uuid_128 central_disc_uuid128;
static struct bt_uuid_16 central_disc_uuid16;
/* Which characteristic the CHARACTERISTIC/DESCRIPTOR discovery stages are
 * for: false = beat, true = frame. Only touched on the BT RX thread (the
 * discovery callback chain) and at connect, before discovery starts. */
static bool central_disc_frame;

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
		/* Already-linked guard: a node whose beats we are receiving on an
		 * INBOUND link keeps advertising (the re-arm is deliberate — it must
		 * stay visible to phones and other peers), so its adverts are not
		 * connect candidates. Without this, a >2-node bench busy-loops on its
		 * own inbound peer at advertising rate: matched -> bt_conn_le_create
		 * -EINVAL ("found valid connection") -> rescan -> matched again — 174
		 * attempts in 50 s on the first 3-node topology (2026-08-24). */
		for (unsigned int i = 0; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
			if (peer.rx[i].beats > 0U && peer.rx[i].last.node_num == m.node_num) {
				connect_now = false;
				break;
			}
		}
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

	/* BT_LE_SCAN_PASSIVE minus BT_LE_SCAN_OPT_FILTER_DUPLICATE: the ESP32-S3
	 * controller rejects LE Set Scan Enable with the duplicate filter on
	 * (opcode 0x200c, status 0x12 -> -EINVAL; found on the a4it.8 bench node,
	 * where the Nordic controller had accepted it). Controller-side dedup was
	 * only ever an optimization here — the seen table dedups matched adverts
	 * itself — so scanning without it costs extra callbacks, not correctness. */
	static const struct bt_le_scan_param peer_scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&peer_scan_param, peer_scan_cb);
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
		peer.rx_last_ms[index] = k_uptime_get();
	}
	k_mutex_unlock(&peer_lock);
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t central_frame_notify_cb(struct bt_conn *conn,
				       struct bt_gatt_subscribe_params *params, const void *data,
				       uint16_t length)
{
	unsigned int index;

	if (data == NULL) {
		/* Unsubscribed (link going down); the disconnect path resets
		 * the rest of the state. */
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	index = bt_conn_index(conn);
	if (index < MESHTASTIC_BLE_REG_SLOTS) {
		(void)frame_chunk_ingest(index, data, length);
	}
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t central_discover_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (central_disc_frame) {
			/* No frame channel on this peer (older image): the beat
			 * link stays up and useful; frames just cannot ride this
			 * hop. A downgrade, not a discovery failure. */
			LOG_WRN("BLE peer 0x%08x has no frame channel (stage %u)",
				central.conn_node, params->type);
		} else {
			LOG_WRN("BLE peer discovery: attribute not found (stage %u)",
				params->type);
			k_mutex_lock(&peer_lock, K_FOREVER);
			peer.stats.discovery_failures++;
			k_mutex_unlock(&peer_lock);
		}
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
		if (central_disc_frame) {
			central_frame_sub.value_handle = bt_gatt_attr_value_handle(attr);
		} else {
			central_sub.value_handle = bt_gatt_attr_value_handle(attr);
		}
		err = bt_gatt_discover(conn, &central_disc);
		if (err != 0) {
			LOG_WRN("BLE peer CCC discover failed (%d)", err);
		}
	} else if (!central_disc_frame) {
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

			/* Beat chain done — continue into the frame channel,
			 * which sits after the beat CCC in the service table
			 * (BT_GATT_SERVICE_DEFINE order, ours on both ends). */
			central_disc_frame = true;
			memcpy(&central_disc_uuid128, &peer_frame_uuid,
			       sizeof(central_disc_uuid128));
			central_disc.uuid = &central_disc_uuid128.uuid;
			central_disc.start_handle = attr->handle + 1U;
			central_disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
			central_disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
			err = bt_gatt_discover(conn, &central_disc);
			if (err != 0) {
				LOG_WRN("BLE peer frame char discover failed (%d)", err);
			}
		}
		return BT_GATT_ITER_STOP;
	} else {
		central_frame_sub.notify = central_frame_notify_cb;
		central_frame_sub.value = BT_GATT_CCC_NOTIFY;
		central_frame_sub.ccc_handle = attr->handle;
		err = bt_gatt_subscribe(conn, &central_frame_sub);
		if (err != 0 && err != -EALREADY) {
			LOG_WRN("BLE peer frame subscribe failed (%d)", err);
		} else {
			k_mutex_lock(&peer_lock, K_FOREVER);
			central.frame_value_handle = central_frame_sub.value_handle;
			central.frame_ready = true;
			k_mutex_unlock(&peer_lock);
			LOG_INF("BLE peer frame channel to 0x%08x ready", central.conn_node);
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
	central_disc_frame = false;
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
	central.frame_ready = false;
	central.frame_value_handle = 0U;
	central.conn_node = 0U;
	k_mutex_unlock(&peer_lock);

	/* Resume the hunt off the BT callback context. */
	(void)k_work_submit(&scan_work);
}

BT_CONN_CB_DEFINE(peer_conn_callbacks) = {
	.connected = peer_central_connected,
	.disconnected = peer_central_disconnected,
};

#if defined(CONFIG_MESHTASTIC_SETTINGS)
/*
 * Persisted central intent (agents-xhli.9): scan-arm + connect target survive
 * a reboot, so a chain re-forms unattended — before this, every reset silently
 * disarmed the scanner and a human had to re-arm it from the shell, and a
 * rebooted node's empty link accounting let a plain any-scan grab the wrong
 * peer. Only INTENT is stored (what the operator armed), never link state.
 * For deterministic chain shapes, arm links with a targeted
 * `blepeer connect <node>` rather than a bare `scan on`.
 */
struct peer_intent_rec {
	uint8_t scan_on;
	uint32_t target;
} __packed;

static void peer_intent_save(void)
{
	struct peer_intent_rec rec;

	k_mutex_lock(&peer_lock, K_FOREVER);
	rec.scan_on = central.scan_on ? 1U : 0U;
	rec.target = central.target_node;
	k_mutex_unlock(&peer_lock);

	if (settings_save_one("blepeer/central", &rec, sizeof(rec)) != 0) {
		LOG_WRN("BLE peer: intent save failed");
	}
}

static int peer_intent_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				    void *cb_arg)
{
	struct peer_intent_rec rec;

	if (strcmp(key, "central") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(rec) || read_cb(cb_arg, &rec, sizeof(rec)) != (ssize_t)sizeof(rec)) {
		return -EINVAL;
	}

	k_mutex_lock(&peer_lock, K_FOREVER);
	central.scan_on = (rec.scan_on != 0U);
	central.target_node = rec.target;
	k_mutex_unlock(&peer_lock);
	return 0;
}

static int peer_intent_settings_commit(void)
{
	/* meshtastic_ble_init()'s settings_load() runs after bt_enable(), so a
	 * restored arm can start immediately; a subtree load from anywhere
	 * earlier just leaves the intent staged for that later commit. */
	if (!bt_is_ready()) {
		return 0;
	}
	if (central.scan_on) {
		if (central.target_node != 0U) {
			LOG_INF("BLE peer: restored scan intent, target 0x%08x",
				central.target_node);
		} else {
			LOG_INF("BLE peer: restored scan intent (any peer)");
		}
		(void)k_work_submit(&scan_work);
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_blepeer, "blepeer", NULL, peer_intent_settings_set,
			       peer_intent_settings_commit, NULL);
#else
static void peer_intent_save(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

int meshtastic_ble_peer_scan_set(bool on)
{
	int err = 0;

	k_mutex_lock(&peer_lock, K_FOREVER);
	central.scan_on = on;
	if (!on) {
		central.target_node = 0U;
	}
	k_mutex_unlock(&peer_lock);
	peer_intent_save();

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

uint32_t meshtastic_ble_peer_scan_target(void)
{
	uint32_t t;

	k_mutex_lock(&peer_lock, K_FOREVER);
	t = central.target_node;
	k_mutex_unlock(&peer_lock);
	return t;
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
	peer_intent_save();

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

/* Chunk-write one frame up the outbound link. The target check has already
 * passed; conn/handle are re-read under the lock per chunk-loop entry. */
static int central_send_frame(const uint8_t *frame, size_t len)
{
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	struct meshtastic_ble_peer_chunker ck;
	struct bt_conn *conn;
	uint16_t handle;
	int n;
	int ret;

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (!central.frame_ready || central.conn == NULL) {
		k_mutex_unlock(&peer_lock);
		return -ENOTCONN;
	}
	conn = bt_conn_ref(central.conn);
	handle = central.frame_value_handle;
	k_mutex_unlock(&peer_lock);

	ret = meshtastic_ble_peer_chunker_start(&ck, frame, len);
	if (ret < 0) {
		bt_conn_unref(conn);
		return ret;
	}

	while ((n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf))) > 0) {
		ret = bt_gatt_write_without_response(conn, handle, buf, (uint16_t)n, false);
		if (ret != 0) {
			break;
		}
	}
	bt_conn_unref(conn);

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (ret == 0) {
		peer.stats.frame_tx_frames++;
	} else {
		peer.stats.frame_tx_failed++;
	}
	k_mutex_unlock(&peer_lock);
	return ret;
}

int meshtastic_ble_peer_frame_send_to(uint32_t node_num, const uint8_t *frame, size_t len)
{
	bool via_central = false;
	bool via_notify = false;

	k_mutex_lock(&peer_lock, K_FOREVER);
	if (central.frame_ready && central.conn != NULL && central.conn_node == node_num) {
		via_central = true;
	} else if (peer.frame_notify_enabled) {
		/* Inbound link: the peer is OUR central. The notify targets
		 * every frame-subscribed central (single-subscriber by design,
		 * like the beat channel); requiring live beats from the target
		 * keeps a stale subscription from swallowing frames. The
		 * outbound conn's slot is excluded: its rx accounting holds the
		 * node at the far end of OUR central link, which a notify on
		 * our peripheral characteristic can never reach. */
		unsigned int out_idx = (central.conn != NULL)
					       ? bt_conn_index(central.conn)
					       : MESHTASTIC_BLE_REG_SLOTS;

		for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
			if (i != out_idx && peer.rx[i].beats > 0U &&
			    peer.rx[i].last.node_num == node_num) {
				via_notify = true;
				break;
			}
		}
	}
	k_mutex_unlock(&peer_lock);

	if (via_central) {
		return central_send_frame(frame, len);
	}
	if (via_notify) {
		return meshtastic_ble_peer_frame_notify(frame, len);
	}
	return -EHOSTUNREACH;
}

int meshtastic_ble_peer_tx_try_divert(const uint8_t *wire, uint32_t len)
{
	const struct meshtastic_wire_header *hdr;
	uint32_t dest;

	if (wire == NULL || len < MESHTASTIC_HDR_LEN) {
		return -EINVAL;
	}

	hdr = (const struct meshtastic_wire_header *)wire;
	dest = sys_le32_to_cpu(hdr->dest);

	/* Only frames THIS node originates may leave over the peer link: a
	 * relayed frame keeps its originator's src, and diverting it would
	 * bridge LoRa onto BLE — the other half of the v1 link-local rule.
	 * Broadcasts stay on the air: v1 is a point-to-point lane. */
	if (sys_le32_to_cpu(hdr->src) != mt.node_id || dest == MESHTASTIC_NODE_BROADCAST) {
		return -EHOSTUNREACH;
	}

	return meshtastic_ble_peer_frame_send_to(dest, wire, len);
}

void meshtastic_ble_peer_link_get(struct meshtastic_ble_peer_link *out)
{
	k_mutex_lock(&peer_lock, K_FOREVER);
	out->connected = (central.conn != NULL);
	out->ready = central.link_ready;
	out->frame_ready = central.frame_ready;
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
