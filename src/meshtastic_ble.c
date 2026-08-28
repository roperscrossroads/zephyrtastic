/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "meshtastic_ble_name.h"
#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#include "meshtastic_config_store.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_phoneapi.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define BT_UUID_MESHTASTIC_SERVICE_VAL                                                             \
	BT_UUID_128_ENCODE(0x6ba1b218, 0x15a8, 0x461f, 0x9fa8, 0x5dcae273eafd)
#define BT_UUID_MESHTASTIC_TORADIO_VAL                                                             \
	BT_UUID_128_ENCODE(0xf75c76d2, 0x129e, 0x4dad, 0xa1dd, 0x7866124401e7)
#define BT_UUID_MESHTASTIC_FROMRADIO_VAL                                                           \
	BT_UUID_128_ENCODE(0x2c55e69e, 0x4993, 0x11ed, 0xb878, 0x0242ac120002)
#define BT_UUID_MESHTASTIC_FROMNUM_VAL                                                             \
	BT_UUID_128_ENCODE(0xed9da18c, 0xa800, 0x4f66, 0xa670, 0xaa7547e34453)
#define BT_UUID_MESHTASTIC_LOGRADIO_VAL                                                            \
	BT_UUID_128_ENCODE(0x5a3d6e49, 0x06e6, 0x4423, 0x9944, 0xe9de8cdf9547)

static struct bt_uuid_128 meshtastic_service_uuid =
	BT_UUID_INIT_128(BT_UUID_MESHTASTIC_SERVICE_VAL);
static struct bt_uuid_128 toradio_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_TORADIO_VAL);
static struct bt_uuid_128 fromradio_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_FROMRADIO_VAL);
static struct bt_uuid_128 fromnum_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_FROMNUM_VAL);
static struct bt_uuid_128 logradio_uuid = BT_UUID_INIT_128(BT_UUID_MESHTASTIC_LOGRADIO_VAL);

extern const struct bt_gatt_attr attr_meshtastic_svc[];

/* PSRAM on V4 (no-op on V3): the FromRadio staging queue (~4 KB) off internal
 * DRAM. Pulled OUT of the ble struct so its kernel objects (mutex, work-queue,
 * work items) stay in internal RAM — only the plain frame buffers relocate. Safe
 * per meshtastic_ext_ram.h: CPU-only, work-queue-serialised, and copied out before
 * bt_gatt_notify, so it is never a DMA/flash source. */
static MESHTASTIC_EXT_RAM_BSS_ATTR
	struct meshtastic_phoneapi_frame ble_queue[CONFIG_MESHTASTIC_BLE_FROMRADIO_QUEUE_SIZE];

/* ToRadio/FromRadio decode-encode scratch (508 B / 768 B) for
 * meshtastic_phoneapi_handle_toradio() and friends — off PSRAM for the same
 * reason as ble_queue above. Only the ble_work_stack work-queue thread ever
 * touches these, so no lock is needed. */
static MESHTASTIC_EXT_RAM_BSS_ATTR meshtastic_ToRadio ble_to_scratch;
static MESHTASTIC_EXT_RAM_BSS_ATTR meshtastic_FromRadio ble_from_scratch;

static struct {
	struct meshtastic_phoneapi api;
	struct k_mutex lock;
	bool fromnum_notify_enabled;
	bool log_notify_enabled;
	struct bt_conn *conn;
	uint8_t to_radio_buf[MESHTASTIC_API_FRAME_MAX];
	uint16_t to_radio_len;
	struct k_work_q work_q;
	struct k_work to_radio_work;
	struct k_work fromradio_work;
	/* Advertiser bookkeeping (a4it.2): legacy connectable advertising stops on
	 * connect and this Zephyr does not auto-resume it, so "is it advertising
	 * right now" must be tracked, not inferred. adv_starts counts successful
	 * (re)starts so a flapping advertiser is visible from the shell. */
	bool adv_active;
	uint32_t adv_starts;
	/* Every live connection, by bt_conn_index() — the BT-side twin of the
	 * registry slot. ble.conn above stays the PHONE session's connection;
	 * these refs exist so classification (which may happen seconds after
	 * connect) can still reach the conn object. */
	struct bt_conn *conns[MESHTASTIC_BLE_REG_SLOTS];
	int64_t conn_ms[MESHTASTIC_BLE_REG_SLOTS]; /* connect time, for the classify window */
} ble;

/* Statically reserved. NB: this stack must NOT be heap-allocated at bring-up: the
 * ESP32 BT controller allocates from the same system heap (CONFIG_ESP_BT_HEAP_SYSTEM),
 * and a k_malloc here starves it into an OOM abort — a hard fault, before the UI
 * draws — in the unified BLE+WiFi image. (Cost the whole first unified bring-up on
 * hardware; see git history / docs/KNOWN-ISSUES.md.) Internal RAM only. */
static K_THREAD_STACK_DEFINE(ble_work_stack, CONFIG_MESHTASTIC_BLE_WORK_STACK_SIZE);

static struct meshtastic_phoneapi_frame fromradio_staged;
static uint8_t fromradio_buf[MESHTASTIC_API_FRAME_MAX];
static uint16_t fromradio_len;
static bool fromradio_ready;

#if defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
/*
 * Match official Meshtastic NimBLE (PROPERTY_*_ENC / *_AUTHEN) and nrf54l15
 * MESH_PERM_*_AUTHEN: MITM passkey pairing before any mesh GATT access.
 */
#define MT_GATT_PERM_READ  (BT_GATT_PERM_READ | BT_GATT_PERM_READ_AUTHEN)
#define MT_GATT_PERM_WRITE (BT_GATT_PERM_WRITE | BT_GATT_PERM_WRITE_AUTHEN)
#elif defined(CONFIG_MESHTASTIC_BLE_PAIRING)
/* Bonding with Just Works; no MITM (official NO_PIN uses open GATT). */
#define MT_GATT_PERM_READ  BT_GATT_PERM_READ
#define MT_GATT_PERM_WRITE BT_GATT_PERM_WRITE
#else
#define MT_GATT_PERM_READ  BT_GATT_PERM_READ
#define MT_GATT_PERM_WRITE BT_GATT_PERM_WRITE
#endif

enum {
	MESHTASTIC_ATTR_SERVICE = 0,
	MESHTASTIC_ATTR_TORADIO_CHRC,
	MESHTASTIC_ATTR_TORADIO_VALUE,
	MESHTASTIC_ATTR_FROMRADIO_CHRC,
	MESHTASTIC_ATTR_FROMRADIO_VALUE,
	MESHTASTIC_ATTR_FROMNUM_CHRC,
	MESHTASTIC_ATTR_FROMNUM_VALUE,
	MESHTASTIC_ATTR_FROMNUM_CCC,
	MESHTASTIC_ATTR_LOGRADIO_CHRC,
	MESHTASTIC_ATTR_LOGRADIO_VALUE,
	MESHTASTIC_ATTR_LOGRADIO_CCC,
};

static void notify_fromnum(void);
static void schedule_fromradio_prepare(void);
static void ble_note_phone_traffic(struct bt_conn *conn);

static void ble_invalidate_delivery(struct meshtastic_phoneapi *api)
{
	ARG_UNUSED(api);

	fromradio_ready = false;
	fromradio_len = 0U;
	meshtastic_phoneapi_current_frame_reset(&ble.api);
}

static void ble_data_ready(struct meshtastic_phoneapi *api)
{
	ARG_UNUSED(api);

	schedule_fromradio_prepare();
}

/*
 * Synchronously stage the next FromRadio frame for delivery, if one is
 * available. Returns true if a frame is now ready. Called both from the read
 * handler (so a phone draining FromRadio in a tight loop gets back-to-back
 * frames, like stock Meshtastic firmware) and from the work queue (initial
 * kick / re-notify).
 */
static bool stage_fromradio(void)
{
	if (fromradio_ready) {
		return true;
	}

	if (!meshtastic_phoneapi_current_frame(&ble.api, &fromradio_staged)) {
		return false;
	}

	k_mutex_lock(&ble.api.lock, K_FOREVER);
	ble.api.from_num++;
	k_mutex_unlock(&ble.api.lock);

	fromradio_ready = true;
	LOG_DBG("BLE FromRadio staged len=%u from_num=%u", fromradio_staged.len,
		meshtastic_phoneapi_from_num(&ble.api));
	return true;
}

static void fromradio_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (stage_fromradio()) {
		notify_fromnum();
	}
}

static void ble_disconnect(struct meshtastic_phoneapi *api)
{
	ARG_UNUSED(api);

	if (ble.conn != NULL) {
		(void)bt_conn_disconnect(ble.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void schedule_fromradio_prepare(void)
{
	(void)k_work_submit_to_queue(&ble.work_q, &ble.fromradio_work);
}

static void notify_fromnum(void)
{
	uint8_t value[4];
	struct bt_conn *conn;

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (!ble.fromnum_notify_enabled || ble.conn == NULL) {
		k_mutex_unlock(&ble.lock);
		return;
	}
	conn = bt_conn_ref(ble.conn);
	sys_put_le32(meshtastic_phoneapi_from_num(&ble.api), value);
	k_mutex_unlock(&ble.lock);

	(void)bt_gatt_notify(conn, &attr_meshtastic_svc[MESHTASTIC_ATTR_FROMNUM_VALUE], value,
			     sizeof(value));
	bt_conn_unref(conn);
}

static ssize_t read_fromnum(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			    uint16_t len, uint16_t offset)
{
	uint8_t value[4];

	ARG_UNUSED(attr);

	ble_note_phone_traffic(conn);
	sys_put_le32(meshtastic_phoneapi_from_num(&ble.api), value);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static ssize_t read_fromradio(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	ssize_t ret;

	ARG_UNUSED(attr);

	ble_note_phone_traffic(conn);

	if (offset == 0U) {
		/*
		 * Stage synchronously here so a phone draining FromRadio in a tight
		 * loop gets the next frame immediately, instead of racing the work
		 * queue and getting a premature empty read — which the app treats as
		 * "config paused" and stalls on (the "loading module config" hang).
		 */
		if (!fromradio_ready) {
			(void)stage_fromradio();
		}
		if (fromradio_ready) {
			memcpy(fromradio_buf, fromradio_staged.data, fromradio_staged.len);
			fromradio_len = fromradio_staged.len;
		} else {
			fromradio_len = 0U;
		}
	}

	ret = bt_gatt_attr_read(conn, attr, buf, len, offset, fromradio_buf, fromradio_len);
	if (ret >= 0 && fromradio_len > 0U && offset + ret >= fromradio_len) {
		meshtastic_phoneapi_release_current_frame(&ble.api);
		fromradio_ready = false;
		/*
		 * Proactively stage the next frame and notify, so the phone keeps
		 * draining even if it waits on a FromNum notification between reads.
		 * A dropped notify is now self-healing: the next read re-stages.
		 */
		if (stage_fromradio()) {
			notify_fromnum();
		}
	}

	return ret;
}

static ssize_t read_logradio(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	static const char msg[] = "Zephyr Meshtastic\n";

	return bt_gatt_attr_read(conn, attr, buf, len, offset, msg, sizeof(msg) - 1U);
}

static void fromnum_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	k_mutex_lock(&ble.lock, K_FOREVER);
	ble.fromnum_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	k_mutex_unlock(&ble.lock);
}

static void logradio_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	k_mutex_lock(&ble.lock, K_FOREVER);
	ble.log_notify_enabled = (value & (BT_GATT_CCC_NOTIFY | BT_GATT_CCC_INDICATE)) != 0U;
	k_mutex_unlock(&ble.lock);
}

static void to_radio_work_handler(struct k_work *work)
{
	uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	uint16_t buf_len;

	ARG_UNUSED(work);

	k_mutex_lock(&ble.lock, K_FOREVER);
	buf_len = ble.to_radio_len;
	memcpy(buf, ble.to_radio_buf, buf_len);
	ble.to_radio_len = 0U;
	k_mutex_unlock(&ble.lock);

	meshtastic_phoneapi_handle_toradio(&ble.api, buf, buf_len);
}

static ssize_t write_toradio(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			     uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(attr);

	ble_note_phone_traffic(conn);

	if (offset + len > sizeof(ble.to_radio_buf)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (offset == 0U && (flags & BT_GATT_WRITE_FLAG_PREPARE) == 0U) {
		ble.to_radio_len = 0U;
	}
	memcpy(ble.to_radio_buf + offset, buf, len);
	ble.to_radio_len = MAX(ble.to_radio_len, offset + len);
	k_mutex_unlock(&ble.lock);

	if ((flags & BT_GATT_WRITE_FLAG_PREPARE) == 0U) {
		k_work_submit_to_queue(&ble.work_q, &ble.to_radio_work);
	}

	return len;
}

BT_GATT_SERVICE_DEFINE(meshtastic_svc, BT_GATT_PRIMARY_SERVICE(&meshtastic_service_uuid.uuid),
		       BT_GATT_CHARACTERISTIC(&toradio_uuid.uuid,
					      BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
					      MT_GATT_PERM_WRITE, NULL, write_toradio, NULL),
		       BT_GATT_CHARACTERISTIC(&fromradio_uuid.uuid, BT_GATT_CHRC_READ,
					      MT_GATT_PERM_READ, read_fromradio, NULL, NULL),
		       BT_GATT_CHARACTERISTIC(&fromnum_uuid.uuid,
					      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					      MT_GATT_PERM_READ, read_fromnum, NULL, NULL),
		       BT_GATT_CCC(fromnum_ccc_changed, MT_GATT_PERM_READ | MT_GATT_PERM_WRITE),
		       BT_GATT_CHARACTERISTIC(&logradio_uuid.uuid,
					      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY |
						      BT_GATT_CHRC_INDICATE,
					      MT_GATT_PERM_READ, read_logradio, NULL, NULL),
		       BT_GATT_CCC(logradio_ccc_changed, MT_GATT_PERM_READ | MT_GATT_PERM_WRITE));

/*
 * Connectable advertising (BT_LE_ADV_CONN_FAST_1) stops the instant a central
 * connects, and this Zephyr (4.4.99) does NOT auto-resume it on disconnect — so
 * without an explicit restart the node becomes undiscoverable after the first
 * disconnect until reboot. disconnected() re-arms it via this work item:
 * bt_le_adv_start() must not run inside the BT disconnected-callback context, and
 * deferring also avoids racing the connection teardown.
 */
static int start_advertising(void);

static void adv_restart_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!IS_ENABLED(CONFIG_MESHTASTIC_BLE_ADV)) {
		return;
	}

	int ret = start_advertising();

	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("BLE re-advertise after disconnect failed (%d)", ret);
	}
}
static K_WORK_DEFINE(adv_restart_work, adv_restart_fn);

/*
 * Classification (a4it.5): whether the phone PM inhibitor was charged is
 * recorded per-connection in the registry, and the disconnect path keys off
 * that record — never re-derived — so the refcount stays balanced for any
 * interleaving. A connection WE created is a peer immediately (this node never
 * acts as central toward a phone). An incoming connection starts UNCLASSIFIED
 * and becomes the phone on first PhoneAPI traffic, a peer on a valid beat, or
 * the phone by default when the classify window expires. In a phone-only
 * build (MESHTASTIC_BLE_PEER=n) incoming connections classify PHONE at
 * connect, preserving the proven instant-session behaviour exactly.
 */
static bool ble_try_classify_phone(unsigned int index, enum meshtastic_ble_classify_reason r)
{
	bool ok = false;

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (meshtastic_ble_reg_classify(index, MESHTASTIC_BLE_CONN_PHONE, r) == 0) {
		/* The registry granted the phone note (exactly once per slot);
		 * we charge it below. First phone also owns the session conn. */
		if (ble.conn == NULL && ble.conns[index] != NULL) {
			ble.conn = bt_conn_ref(ble.conns[index]);
		}
		ok = true;
	}
	k_mutex_unlock(&ble.lock);

	if (ok) {
		meshtastic_set_ble_connected(true);
		meshtastic_power_note_phone_connected();
		meshtastic_emit_event(MESHTASTIC_EVENT_BLE_CONNECTED, 0, NULL);
	}
	return ok;
}

/* PhoneAPI traffic on an unclassified incoming connection is phone evidence.
 * Called from the GATT handlers (BT RX thread); the unlocked pre-check keeps
 * the steady-state cost of every ToRadio/FromRadio access to one enum read. */
static void ble_note_phone_traffic(struct bt_conn *conn)
{
	unsigned int index;

	if (!IS_ENABLED(CONFIG_MESHTASTIC_BLE_PEER) || conn == NULL) {
		return;
	}
	index = bt_conn_index(conn);
	if (meshtastic_ble_reg_kind(index) != MESHTASTIC_BLE_CONN_UNCLASSIFIED) {
		return;
	}
	(void)ble_try_classify_phone(index, MESHTASTIC_BLE_CLASSIFY_TRAFFIC);
}

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
/*
 * Bounded backstop: an incoming connection that produced no evidence inside
 * the window defaults to PHONE. The STATE is the guard — the sweep acts only
 * on slots still UNCLASSIFIED — so this work item is never cancelled from BT
 * callback context (which is what would deadlock).
 */
static void classify_backstop_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(classify_backstop_work, classify_backstop_fn);

static void classify_backstop_fn(struct k_work *work)
{
	int64_t now = k_uptime_get();
	int64_t next = -1;

	ARG_UNUSED(work);

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		bool expired = false;

		k_mutex_lock(&ble.lock, K_FOREVER);
		if (meshtastic_ble_reg_kind(i) == MESHTASTIC_BLE_CONN_UNCLASSIFIED) {
			int64_t deadline =
				ble.conn_ms[i] + CONFIG_MESHTASTIC_BLE_PEER_CLASSIFY_WINDOW_MS;

			if (now >= deadline) {
				expired = true;
			} else if (next < 0 || deadline < next) {
				next = deadline;
			}
		}
		k_mutex_unlock(&ble.lock);

		if (expired) {
			LOG_INF("BLE conn %u: no evidence in %u ms, defaulting to phone", i,
				CONFIG_MESHTASTIC_BLE_PEER_CLASSIFY_WINDOW_MS);
			(void)ble_try_classify_phone(i, MESHTASTIC_BLE_CLASSIFY_TIMER);
		}
	}

	if (next >= 0) {
		(void)k_work_reschedule(&classify_backstop_work, K_MSEC(next - now));
	}
}

/* Peer evidence from meshtastic_ble_peer.c (a valid beat arrived). Returns 0
 * on the transition, 1 if the slot is already a peer (duplicate beats are
 * normal), -EBUSY if it is already the phone (evidence came too late), or
 * -EINVAL for a dead slot. */
int meshtastic_ble_classify_peer_evidence(unsigned int index)
{
	int ret;

	k_mutex_lock(&ble.lock, K_FOREVER);
	switch (meshtastic_ble_reg_kind(index)) {
	case MESHTASTIC_BLE_CONN_PEER:
		ret = 1;
		break;
	case MESHTASTIC_BLE_CONN_PHONE:
		ret = -EBUSY;
		break;
	case MESHTASTIC_BLE_CONN_UNCLASSIFIED:
		ret = meshtastic_ble_reg_classify(index, MESHTASTIC_BLE_CONN_PEER,
						  MESHTASTIC_BLE_CLASSIFY_HELLO);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	k_mutex_unlock(&ble.lock);
	return ret;
}

const uint8_t *meshtastic_ble_service_uuid128(void)
{
	return meshtastic_service_uuid.val;
}

/* Peer address + connection age for the blepeer shell. */
bool meshtastic_ble_slot_info(unsigned int index, char *addr, size_t addr_len, int64_t *age_ms)
{
	bool ok = false;

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (index < MESHTASTIC_BLE_REG_SLOTS && ble.conns[index] != NULL) {
		bt_addr_le_to_str(bt_conn_get_dst(ble.conns[index]), addr, addr_len);
		*age_ms = k_uptime_get() - ble.conn_ms[index];
		ok = true;
	}
	k_mutex_unlock(&ble.lock);
	return ok;
}

bool meshtastic_ble_slot_addr(unsigned int index, bt_addr_le_t *out)
{
	bool ok = false;

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (index < MESHTASTIC_BLE_REG_SLOTS && ble.conns[index] != NULL) {
		bt_addr_le_copy(out, bt_conn_get_dst(ble.conns[index]));
		ok = true;
	}
	k_mutex_unlock(&ble.lock);
	return ok;
}

struct bt_conn *meshtastic_ble_slot_conn(unsigned int index)
{
	struct bt_conn *conn = NULL;

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (index < MESHTASTIC_BLE_REG_SLOTS && ble.conns[index] != NULL) {
		conn = bt_conn_ref(ble.conns[index]);
	}
	k_mutex_unlock(&ble.lock);
	return conn;
}
#endif /* CONFIG_MESHTASTIC_BLE_PEER */

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	unsigned int index;
	bool outbound;
	bool slots_free;
	int ret;

	if (err != 0U) {
		LOG_WRN("BLE connection failed: 0x%02x", err);
		return;
	}

	(void)bt_conn_get_info(conn, &info);
	outbound = (info.role == BT_CONN_ROLE_CENTRAL);
	index = bt_conn_index(conn);

	k_mutex_lock(&ble.lock, K_FOREVER);
	ret = meshtastic_ble_reg_connect(index, outbound ? MESHTASTIC_BLE_CONN_PEER
							 : MESHTASTIC_BLE_CONN_UNCLASSIFIED);
	if (ret == 0) {
		ble.conns[index] = bt_conn_ref(conn);
		ble.conn_ms[index] = k_uptime_get();
	}
	k_mutex_unlock(&ble.lock);

	if (ret < 0) {
		/* A live conn's index cannot collide; refuse rather than let a
		 * stale slot duplicate or absorb a PM note. */
		LOG_ERR("BLE conn registry rejected slot %u (%d)", index, ret);
		return;
	}

	LOG_INF("BLE connected: slot %u %s", index, outbound ? "(outbound peer)" : "(incoming)");

	if (!outbound) {
		if (IS_ENABLED(CONFIG_MESHTASTIC_BLE_PEER)) {
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
			/* schedule (not reschedule): an earlier pending
			 * deadline must not be pushed out; the sweep re-arms
			 * for whatever is still unclassified. */
			(void)k_work_schedule(
				&classify_backstop_work,
				K_MSEC(CONFIG_MESHTASTIC_BLE_PEER_CLASSIFY_WINDOW_MS));
#endif
		} else {
			/* Phone-only build: no peers exist to wait for, so the
			 * "default to phone" outcome applies immediately —
			 * today's proven instant-session behaviour. */
			(void)ble_try_classify_phone(index, MESHTASTIC_BLE_CLASSIFY_TIMER);
		}
	}

	/*
	 * Legacy connectable advertising stopped the moment this connection was
	 * accepted, and Zephyr does not auto-resume it (see adv_restart_fn). While
	 * a conn slot remains free, re-arm it so the node stays discoverable to
	 * peers — otherwise the phone connecting makes the node invisible and the
	 * symptom ("peer never appears") points at the scanner, the wrong end
	 * entirely (agents-a4it.2). With every slot taken, start_advertising()'s
	 * own guard defers the restart to the next disconnect.
	 */
	k_mutex_lock(&ble.lock, K_FOREVER);
	ble.adv_active = false;
	slots_free = meshtastic_ble_reg_active() < MESHTASTIC_BLE_REG_SLOTS;
	k_mutex_unlock(&ble.lock);
	if (slots_free && IS_ENABLED(CONFIG_MESHTASTIC_BLE_ADV)) {
		(void)k_work_submit_to_queue(&ble.work_q, &adv_restart_work);
	}

	/*
	 * Do not call bt_conn_set_security() here. Official Meshtastic firmware
	 * lets the phone initiate SMP when it needs encrypted GATT access.
	 * A peripheral-initiated pairing races the app and often ends with MIC
	 * failure (0x3d) or "pairing has been deleted".
	 */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	unsigned int index = bt_conn_index(conn);
	bool was_phone;

	k_mutex_lock(&ble.lock, K_FOREVER);
	was_phone = meshtastic_ble_reg_disconnect(index);
	if (ble.conn == conn) {
		bt_conn_unref(ble.conn);
		ble.conn = NULL;
	}
	if (ble.conns[index] != NULL) {
		bt_conn_unref(ble.conns[index]);
		ble.conns[index] = NULL;
	}
	k_mutex_unlock(&ble.lock);

	/* Drop any peer-beat accounting for this slot before the index can be
	 * recycled (no-op stub when the peer link is compiled out). */
	meshtastic_ble_peer_conn_down(index);

	/* Tear down the phone session and release its PM inhibitor only when it
	 * was the phone's own connection that went away. */
	if (was_phone) {
		ble_invalidate_delivery(&ble.api);
		meshtastic_phoneapi_reset(&ble.api);
		(void)k_work_cancel(&ble.fromradio_work);

		meshtastic_set_ble_connected(false);
		meshtastic_power_note_phone_disconnected();
		meshtastic_emit_event(MESHTASTIC_EVENT_BLE_DISCONNECTED, 0, NULL);
	}
	LOG_INF("BLE disconnected: 0x%02x%s", reason, was_phone ? " (phone)" : "");

	/* Re-arm connectable advertising so the phone can reconnect without a reboot
	 * (see adv_restart_fn). Deferred off the BT callback context. */
	(void)k_work_submit_to_queue(&ble.work_q, &adv_restart_work);
}

#if defined(CONFIG_BT_SMP)
static struct bt_gatt_exchange_params mtu_exchange;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);

	if (err != 0U) {
		LOG_WRN("BLE MTU exchange failed: 0x%02x", err);
		return;
	}

	LOG_INF("BLE MTU %u", bt_gatt_get_mtu(conn));
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err != BT_SECURITY_ERR_SUCCESS) {
		LOG_WRN("BLE security failed: level %u err %u (%s)", (unsigned int)level,
			(unsigned int)err, bt_security_err_to_str(err));

		if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING) {
			/*
			 * Stale phone bond after reflash (official nrf54l15 path).
			 * Do not bt_unpair on generic failures — that races SMP and
			 * triggers "The in-progress pairing has been deleted!".
			 */
			LOG_WRN("BLE stale bond (key missing); forget device in Meshtastic app");
			(void)bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
		}
		return;
	}

	LOG_INF("BLE security level %u", (unsigned int)level);

	mtu_exchange.func = mtu_exchange_cb;
	if (bt_gatt_exchange_mtu(conn, &mtu_exchange) < 0) {
		LOG_WRN("BLE MTU exchange request failed");
	}
}

#if defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	ARG_UNUSED(conn);

	LOG_INF("BLE pairing passkey %06u (enter in Meshtastic app)", passkey);
}

static uint32_t auth_app_passkey(struct bt_conn *conn)
{
	ARG_UNUSED(conn);

	return CONFIG_MESHTASTIC_BLE_PASSKEY;
}
#endif

#if defined(CONFIG_MESHTASTIC_BLE_PAIRING) && !defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
static void pairing_confirm(struct bt_conn *conn)
{
	LOG_INF("BLE pairing confirm (Just Works)");
	(void)bt_conn_auth_pairing_confirm(conn);
}
#endif

static void pairing_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);

	LOG_INF("BLE pairing cancelled");
}

#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	ARG_UNUSED(conn);

	LOG_DBG("BLE pairing request auth_req=0x%02x io=0x%02x", feat->auth_req,
		feat->io_capability);

	return BT_SECURITY_ERR_SUCCESS;
}
#endif

static const struct bt_conn_auth_cb auth_callbacks = {
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
	.pairing_accept = pairing_accept,
#endif
#if defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
	.passkey_display = auth_passkey_display,
	.app_passkey = auth_app_passkey,
#elif defined(CONFIG_MESHTASTIC_BLE_PAIRING)
	.pairing_confirm = pairing_confirm,
#endif
	.cancel = pairing_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);

	LOG_INF("BLE pairing complete, bonded=%u", bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("BLE pairing failed: err %u (%s)", (unsigned int)reason,
		bt_security_err_to_str(reason));

	if (reason == BT_SECURITY_ERR_PIN_OR_KEY_MISSING || reason == BT_SECURITY_ERR_AUTH_FAIL) {
		LOG_WRN("Forget this device in the Meshtastic app, then reconnect");
	}
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed,
#endif
};

/*
 * THE ADVERTISED NAME — composed, because a constant one is a bug.
 *
 * This was CONFIG_MESHTASTIC_BLE_DEVICE_NAME verbatim, so every node built from
 * this tree advertised the identical name and a phone offered a list of
 * indistinguishable entries (agents-xhli.15 — found with four bench nodes up,
 * one of them deliberately parked out of USB reach). The owner short name is the
 * node's identity everywhere else on the mesh; the advert now carries it too.
 *
 * The rule itself lives in meshtastic_ble_name.c, which is BT-free so it can be
 * unit-tested without a controller; this only supplies the two runtime values
 * and owns the buffer the advert points at.
 *
 * Length is bounded by construction: the scan response has 31 bytes and carries
 * only this field (2 header + name), the prefix is a build constant and the
 * short name is 4 characters. The composer truncates rather than overruns if
 * either ever grows.
 */
static char adv_name[24];

static uint8_t adv_name_compose(void)
{
	return (uint8_t)meshtastic_ble_adv_name_compose(adv_name, sizeof(adv_name),
							meshtastic_config_store_short_name(),
							mt.node_id);
}

const char *meshtastic_ble_adv_name(void)
{
	(void)adv_name_compose();
	return adv_name;
}

static int start_advertising(void)
{
	static const uint8_t flags[] = {
		BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR,
	};
	static const uint8_t service_uuid[] = {
		BT_UUID_MESHTASTIC_SERVICE_VAL,
	};
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	/*
	 * Peer builds add the node number as manufacturer-specific data so two
	 * nodes running the same image can tell each other apart from the
	 * ADVERT alone, without waiting for a scan response. File-static
	 * mutable storage: the node num is a runtime value and the blob must
	 * outlive bt_le_adv_start() and survive re-arms.
	 *
	 * Budget: 3 (flags) + 18 (128-bit UUID) + 2+7 (mfg blob) = 30 of 31
	 * bytes. ONE byte spare — the next AD field overflows the advert and
	 * bt_le_adv_start() returns -EINVAL, hence the assert.
	 */
	BUILD_ASSERT(3U + 18U + 2U + MESHTASTIC_BLE_PEER_ADV_LEN <= 31U,
		     "BLE advertisement overflows 31 bytes");
	static uint8_t peer_mfg_blob[MESHTASTIC_BLE_PEER_ADV_LEN];
	static const struct bt_data ad[] = {
		BT_DATA(BT_DATA_FLAGS, flags, sizeof(flags)),
		BT_DATA(BT_DATA_UUID128_ALL, service_uuid, sizeof(service_uuid)),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, peer_mfg_blob, sizeof(peer_mfg_blob)),
	};
#else
	/* Gated on MESHTASTIC_BLE_PEER so a phone-only build's advert stays
	 * BYTE-IDENTICAL: the Meshtastic app filters on the service UUID and
	 * this exact advert is proven against it. */
	static const struct bt_data ad[] = {
		BT_DATA(BT_DATA_FLAGS, flags, sizeof(flags)),
		BT_DATA(BT_DATA_UUID128_ALL, service_uuid, sizeof(service_uuid)),
	};
#endif
	/* Composed, not constant — see adv_name_compose(). Non-const because
	 * data_len is only known once the name exists; the buffer it points at is
	 * file-static so it outlives bt_le_adv_start() and survives re-arms,
	 * exactly like the peer blob above. */
	static struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, 0),
	};

	int ret;

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	/* Refreshed on every (re)start: node_id is settled long before BLE init,
	 * but a re-arm after any later change keeps the advert truthful. */
	meshtastic_ble_peer_adv_encode(peer_mfg_blob, mt.node_id);
#endif
	/* Same reason, and the same limit: a rename lands in the advert at the
	 * next re-arm (a disconnect, or a reboot), not the instant it is written.
	 * Nothing here is worth forcing an advertising restart for — a phone that
	 * is already connected does not read the advert, and one that is not will
	 * see the new name on its next scan. */
	sd[0].data_len = adv_name_compose();

	k_mutex_lock(&ble.lock, K_FOREVER);
	if (meshtastic_ble_reg_active() >= MESHTASTIC_BLE_REG_SLOTS) {
		/* Every conn slot is taken: a connectable advert could only be
		 * accepted into -ENOMEM. The next disconnect re-arms. */
		ble.adv_active = false;
		k_mutex_unlock(&ble.lock);
		return 0;
	}
	k_mutex_unlock(&ble.lock);

	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret == 0 || ret == -EALREADY) {
		k_mutex_lock(&ble.lock, K_FOREVER);
		ble.adv_active = true;
		if (ret == 0) {
			ble.adv_starts++;
		}
		k_mutex_unlock(&ble.lock);
	}
	return ret;
}

bool meshtastic_ble_is_connected(void)
{
	return ble.conn != NULL;
}

/* Advertiser observability for the (future) `meshtastic blepeer status` shell
 * surface: tracked state, not inference (agents-a4it.2 / .6). */
bool meshtastic_ble_adv_active(void)
{
	return ble.adv_active;
}

uint32_t meshtastic_ble_adv_starts(void)
{
	return ble.adv_starts;
}

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
/* Bearer ingest (agents-xhli.2): a wire frame completed by the peer frame
 * channel enters the same RX pipeline as an airframe, tagged with its bearer
 * so the router applies the link-local rule (delivered locally, never relayed
 * onto LoRa). Runs under the peer lock on the BT RX thread — the inject is a
 * non-blocking queue put, exactly what the M1 callback contract allows. */
static void ble_peer_frame_ingest(unsigned int index, const uint8_t *frame, size_t len)
{
	int ret = meshtastic_radio_rx_inject(frame, (uint16_t)len, MESHTASTIC_BEARER_BLE_PEER);

	if (ret < 0) {
		LOG_WRN("BLE peer frame (conn %u, %zu B) not ingested (%d)", index, len, ret);
	}
}
#endif

int meshtastic_ble_init(void)
{
	int ret;

	k_mutex_init(&ble.lock);

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
	meshtastic_ble_peer_frame_rx_register(ble_peer_frame_ingest);
#endif

	meshtastic_phoneapi_init(&ble.api, "ble", ble_queue, ARRAY_SIZE(ble_queue), ble_data_ready,
				 ble_disconnect, ble_invalidate_delivery, NULL, &ble_to_scratch,
				 &ble_from_scratch);
	meshtastic_phoneapi_register(&ble.api);

	k_work_queue_start(&ble.work_q, ble_work_stack, CONFIG_MESHTASTIC_BLE_WORK_STACK_SIZE,
			   CONFIG_MESHTASTIC_BLE_WORK_PRIORITY, NULL);
	k_work_init(&ble.to_radio_work, to_radio_work_handler);
	k_work_init(&ble.fromradio_work, fromradio_work_handler);

	if (!bt_is_ready()) {
		ret = bt_enable(NULL);
		if (ret < 0) {
			LOG_ERR("bt_enable failed (%d)", ret);
			return ret;
		}
	}

#if defined(CONFIG_SETTINGS)
	settings_load();
#endif

#if defined(CONFIG_BT_SMP)
	if (IS_ENABLED(CONFIG_MESHTASTIC_BLE_PAIRING)) {
		ret = bt_conn_auth_cb_register(&auth_callbacks);
		if (ret < 0 && ret != -EALREADY) {
			LOG_WRN("BLE auth callback registration failed (%d)", ret);
		}

		ret = bt_conn_auth_info_cb_register(&auth_info_callbacks);
		if (ret < 0 && ret != -EALREADY) {
			LOG_WRN("BLE auth info callback registration failed (%d)", ret);
		}
	}
#endif

	// #if defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
	// 	LOG_INF("BLE fixed PIN %06u", (unsigned int)CONFIG_MESHTASTIC_BLE_PASSKEY);
	// #endif

	if (IS_ENABLED(CONFIG_MESHTASTIC_BLE_ADV)) {
		ret = start_advertising();
		if (ret < 0 && ret != -EALREADY) {
			LOG_ERR("Meshtastic BLE advertising failed (%d)", ret);
			return ret;
		}
	}

	LOG_INF("Meshtastic BLE service ready");
	return 0;
}
