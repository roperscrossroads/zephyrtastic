/* SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * meshtastic_smp_central.c — this node as an mcumgr (SMP) CLIENT over BLE.
 *
 * Zephyr ships the SMP *server* over BLE (subsys/mgmt/mcumgr/transport/src/
 * smp_bt.c: one GATT service, one characteristic, write-without-response in,
 * notifications out) and the SMP *client* core (smp_client + img_mgmt_client +
 * os_mgmt_client) — but no client transport that talks to that server from
 * another Zephyr node. Its client transports are UART and UDP. This file is the
 * missing piece: a BLE-central SMP transport. It is the mirror image of
 * smp_bt.c — connect as a central, discover the SMP service, subscribe, then:
 *
 *   output(nb)        -> the SMP frame written to the characteristic in ATT
 *                        MTU-3 sized chunks (write-without-response, paced by
 *                        the TX-complete callback)
 *   notification(...) -> chunks reassembled by the SMP header's length field
 *                        (the server fragments notifications the same way),
 *                        then handed to smp_client_single_response()
 *
 * Nothing above the transport is new: img_mgmt_client does upload / state
 * write / state read, os_mgmt_client does echo / reset — the same SMP
 * commands `smpmgr --ble` sends, so a node updated this way sees no difference
 * from an update from a laptop. The target needs NO new firmware: any node
 * built with overlay-ota-ble.conf is a target.
 *
 * The "courier" source: the image this node pushes is whatever is staged in
 * its OWN slot1 (a signed MCUboot image uploaded there and never marked for
 * test — MCUboot ignores an unmarked secondary slot). That makes an ESP32 with
 * 5.9 MB of slot1 a store-and-forward depot for nRF images: upload the nRF
 * image into the ESP32 over any SMP path, then `smpc push` it on to the nRF
 * over BLE. It never boots on the courier — wrong architecture — and MCUboot's
 * signature check on the target is what decides whether it runs.
 *
 * Single link, single job: one target at a time, one operation in flight.
 * That is deliberate for the spike (compare with BT Mesh DFU's one->many).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>
#include <zephyr/storage/flash_map.h>
#if defined(CONFIG_MESHTASTIC_DEPOT)
#include <zephyr/fs/fs.h>
#endif

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/smp/smp_client.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt_client.h>
#include <zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt_client.h>
#include <mgmt/mcumgr/transport/smp_internal.h>

LOG_MODULE_REGISTER(mt_smpc, CONFIG_MESHTASTIC_LOG_LEVEL);

/* ------------------------------------------------------------------------ */
/* Tunables                                                                  */

/* Connect + MTU exchange + 3-stage discovery + subscribe, end to end. */
#define SMPC_CONNECT_TIMEOUT_MS 15000
/* One ATT chunk's TX-complete; a connection interval at worst, so generous. */
#define SMPC_TX_CHUNK_TIMEOUT_MS 3000
/* bt_gatt_write_without_response_cb() -> -ENOMEM means the host's TX buffers
 * are all in flight; back off and retry rather than fail the frame. */
#define SMPC_TX_ENOMEM_RETRIES 400
#define SMPC_TX_ENOMEM_BACKOFF_MS 5
/* Largest SMP response we will reassemble: bounded by the net_buf we put it
 * in, which is the same pool the server-side transports use. */
#define SMPC_RX_MAX CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE
/* How long a target may be away for the swap+reboot leg of `smpc update`. */
#define SMPC_UPDATE_REBOOT_WAIT_MS 150000
#define SMPC_UPDATE_RECONNECT_GAP_MS 3000

/* Connection parameters this central asks for. The target (a peripheral that
 * carries BT_GAP_AUTO_UPDATE_CONN_PARAMS=n since the A.2 investigation) keeps
 * whatever the central chose. 30-50 ms is BT_LE_CONN_PARAM_DEFAULT's interval;
 * the 6 s supervision timeout is what the nodes advertise as preferred
 * (upstream Meshtastic's NimBLE value) — a target that is erasing QSPI flash
 * between chunks wants the slack, and the 420 ms default is what the A.2 link
 * dropped on. Latency 0 so the peripheral answers every event. */
#define SMPC_CONN_INT_MIN 24
#define SMPC_CONN_INT_MAX 40
#define SMPC_CONN_LATENCY 0
#define SMPC_CONN_TIMEOUT 600

/* The courier's read buffer. Internal RAM on purpose: flash_area_read() goes
 * through the flash driver, and on the ESP32 that must not land in PSRAM. Two
 * SMP chunks' worth per flash read. */
#define SMPC_READ_CHUNK 2048
/* The ATT default MTU (BT_ATT_DEFAULT_LE_MTU is host-internal). */
#define SMPC_ATT_MIN_MTU 23

/* ------------------------------------------------------------------------ */
/* State                                                                     */

static struct bt_uuid_128 smpc_svc_uuid = BT_UUID_INIT_128(SMP_BT_SVC_UUID_VAL);
static struct bt_uuid_128 smpc_chr_uuid = BT_UUID_INIT_128(SMP_BT_CHR_UUID_VAL);
static struct bt_uuid_16 smpc_ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

/* The SMP client stack: one transport, one client object, one of each group. */
static struct smp_transport smpc_transport;
static struct smp_client_transport_entry smpc_entry;
static struct smp_client_object smpc_client;
static struct img_mgmt_client smpc_img;
static struct os_mgmt_client smpc_os;
static struct mcumgr_image_data smpc_images[2];
static bool smpc_client_ready;

enum smpc_job_kind {
	SMPC_JOB_NONE = 0,
	SMPC_JOB_PUSH,
	SMPC_JOB_UPDATE,
};

static struct {
	struct k_mutex lock;

	/* Link */
	struct bt_conn *conn;
	bt_addr_le_t addr;
	uint16_t chr_handle;
	uint16_t ccc_handle;
	bool ready;		/* subscribed: SMP frames can flow */
	bool adopted;		/* the link was already up (peer link, phone...) — not ours to drop */
	bool connect_failed;	/* set by a callback so a waiter wakes with a verdict */
	uint8_t last_disconnect;
	struct k_sem ready_sem;
	struct k_sem tx_sem;
	struct bt_gatt_discover_params disc;
	struct bt_gatt_subscribe_params sub;
	struct bt_gatt_exchange_params mtu_params;

	/* RX reassembly (touched only from the BT RX thread) */
	struct net_buf *rx;
	uint16_t rx_expected;

	/* Job (one at a time, on its own work queue) */
	struct k_work job_work;
	enum smpc_job_kind job_kind;
	bool job_running;
	int job_rc;
	const char *job_stage;
	uint32_t job_off;
	uint32_t job_total;
	int64_t job_t0;
	int64_t job_t_upload;	/* upload leg wall time, ms (0 until done) */
	char job_path[64];	/* "" = slot1 */

	/* Counters */
	uint32_t tx_frames, tx_chunks, tx_errors;
	uint32_t rx_frames, rx_chunks, rx_dropped, rx_unmatched;
} smpc = {
	.lock = Z_MUTEX_INITIALIZER(smpc.lock),
	.ready_sem = Z_SEM_INITIALIZER(smpc.ready_sem, 0, 1),
	.tx_sem = Z_SEM_INITIALIZER(smpc.tx_sem, 0, 1),
};

static K_THREAD_STACK_DEFINE(smpc_job_stack, CONFIG_MESHTASTIC_SMP_CENTRAL_JOB_STACK_SIZE);
static struct k_work_q smpc_job_q;
static uint8_t smpc_read_buf[SMPC_READ_CHUNK];

/* ------------------------------------------------------------------------ */
/* Transport: TX                                                             */

static void smpc_tx_done(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);
	k_sem_give(&smpc.tx_sem);
}

/* smp_client hands us a whole SMP frame (header + CBOR) on the SMP work queue.
 * We own the reference we are given; the client keeps its own for retries. */
static int smpc_output(struct net_buf *nb)
{
	struct bt_conn *conn = NULL;
	uint16_t handle;
	uint16_t mtu;
	size_t off = 0;
	int rc = 0;

	k_mutex_lock(&smpc.lock, K_FOREVER);
	if (smpc.ready && smpc.conn != NULL) {
		conn = bt_conn_ref(smpc.conn);
	}
	handle = smpc.chr_handle;
	k_mutex_unlock(&smpc.lock);

	if (conn == NULL) {
		smpc.tx_errors++;
		smp_packet_free(nb);
		return -ENOTCONN;
	}

	mtu = bt_gatt_get_mtu(conn);
	if (mtu < SMPC_ATT_MIN_MTU) {
		mtu = SMPC_ATT_MIN_MTU;
	}
	mtu -= 3U; /* ATT write-without-response opcode + handle */

	while (off < nb->len) {
		uint16_t n = MIN(mtu, nb->len - off);
		int tries = SMPC_TX_ENOMEM_RETRIES;

		k_sem_reset(&smpc.tx_sem);
		do {
			rc = bt_gatt_write_without_response_cb(conn, handle, nb->data + off, n,
							       false, smpc_tx_done, NULL);
			if (rc == -ENOMEM && tries-- > 0) {
				k_sleep(K_MSEC(SMPC_TX_ENOMEM_BACKOFF_MS));
			}
		} while (rc == -ENOMEM && tries > 0);

		if (rc != 0) {
			break;
		}
		smpc.tx_chunks++;
		off += n;
		if (k_sem_take(&smpc.tx_sem, K_MSEC(SMPC_TX_CHUNK_TIMEOUT_MS)) != 0) {
			rc = -ETIMEDOUT;
			break;
		}
	}

	if (rc == 0) {
		smpc.tx_frames++;
	} else {
		smpc.tx_errors++;
		LOG_WRN("SMPC tx failed at %u/%u (%d)", (unsigned int)off, nb->len, rc);
	}

	bt_conn_unref(conn);
	smp_packet_free(nb);
	return rc;
}

static uint16_t smpc_get_mtu(const struct net_buf *nb)
{
	ARG_UNUSED(nb);
	/* The client sizes upload chunks from NETBUF_SIZE, not from this; it is
	 * here for completeness of the transport contract. */
	return CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE;
}

/* ------------------------------------------------------------------------ */
/* Transport: RX (notifications from the target's SMP characteristic)        */

static void smpc_rx_drop(void)
{
	if (smpc.rx != NULL) {
		smp_packet_free(smpc.rx);
		smpc.rx = NULL;
	}
	smpc.rx_expected = 0U;
	smpc.rx_dropped++;
}

static uint8_t smpc_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			      const void *data, uint16_t len)
{
	struct smp_hdr hdr;

	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (data == NULL) {
		/* Unsubscribed (link torn down). */
		return BT_GATT_ITER_STOP;
	}

	smpc.rx_chunks++;

	if (smpc.rx == NULL) {
		/* First chunk of a frame. The server's first fragment is at least
		 * MTU-3 >= 20 bytes, so the 8-byte header is always whole. */
		if (len < sizeof(hdr)) {
			LOG_WRN("SMPC rx: runt first chunk (%u)", len);
			smpc.rx_dropped++;
			return BT_GATT_ITER_CONTINUE;
		}
		memcpy(&hdr, data, sizeof(hdr));
		smpc.rx_expected = sizeof(hdr) + sys_be16_to_cpu(hdr.nh_len);
		if (smpc.rx_expected > SMPC_RX_MAX) {
			LOG_WRN("SMPC rx: frame %u > %u", smpc.rx_expected, SMPC_RX_MAX);
			smpc.rx_dropped++;
			smpc.rx_expected = 0U;
			return BT_GATT_ITER_CONTINUE;
		}
		smpc.rx = smp_packet_alloc();
		if (smpc.rx == NULL) {
			LOG_WRN("SMPC rx: no net_buf");
			smpc.rx_dropped++;
			smpc.rx_expected = 0U;
			return BT_GATT_ITER_CONTINUE;
		}
	}

	if (len > net_buf_tailroom(smpc.rx)) {
		LOG_WRN("SMPC rx: overflow");
		smpc_rx_drop();
		return BT_GATT_ITER_CONTINUE;
	}
	net_buf_add_mem(smpc.rx, data, len);

	if (smpc.rx->len >= smpc.rx_expected) {
		int rc;

		memcpy(&hdr, smpc.rx->data, sizeof(hdr));
		hdr.nh_len = sys_be16_to_cpu(hdr.nh_len);
		hdr.nh_group = sys_be16_to_cpu(hdr.nh_group);
		smpc.rx_frames++;

		/* The client's group callbacks decode CBOR from nb->data: the SMP
		 * core strips the header before dispatching a response
		 * (smp_process_request_packet), so strip it here too. Ownership of
		 * the buffer stays with us; the callback only reads it. */
		net_buf_pull(smpc.rx, sizeof(hdr));
		rc = smp_client_single_response(smpc.rx, &hdr);
		if (rc != MGMT_ERR_EOK) {
			smpc.rx_unmatched++;
			LOG_WRN("SMPC rx: unmatched response seq %u group %u id %u", hdr.nh_seq,
				hdr.nh_group, hdr.nh_id);
		}
		smp_packet_free(smpc.rx);
		smpc.rx = NULL;
		smpc.rx_expected = 0U;
	}

	return BT_GATT_ITER_CONTINUE;
}

/* ------------------------------------------------------------------------ */
/* Link bring-up: connect -> MTU -> discover service/chr/CCC -> subscribe     */

static void smpc_link_reset_locked(void)
{
	smpc.ready = false;
	smpc.adopted = false;
	smpc.chr_handle = 0U;
	smpc.ccc_handle = 0U;
	memset(&smpc.sub, 0, sizeof(smpc.sub));
}

static void smpc_subscribed_cb(struct bt_conn *conn, uint8_t err,
			       struct bt_gatt_subscribe_params *params)
{
	ARG_UNUSED(params);

	if (err != 0U) {
		LOG_WRN("SMPC subscribe failed (0x%02x)", err);
		k_mutex_lock(&smpc.lock, K_FOREVER);
		smpc.connect_failed = true;
		k_mutex_unlock(&smpc.lock);
		k_sem_give(&smpc.ready_sem);
		return;
	}

	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.ready = true;
	k_mutex_unlock(&smpc.lock);
	LOG_INF("SMPC link ready: chr 0x%04x ccc 0x%04x ATT MTU %u", smpc.chr_handle,
		smpc.ccc_handle, bt_gatt_get_mtu(conn));
	k_sem_give(&smpc.ready_sem);
}

static void smpc_discover_fail(struct bt_conn *conn, const char *what)
{
	LOG_WRN("SMPC discovery: %s", what);
	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.connect_failed = true;
	k_mutex_unlock(&smpc.lock);
	k_sem_give(&smpc.ready_sem);
	(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static uint8_t smpc_discover_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params)
{
	int rc;

	if (attr == NULL) {
		smpc_discover_fail(conn, params->type == BT_GATT_DISCOVER_PRIMARY
						 ? "no SMP service on target"
						 : "SMP service incomplete");
		return BT_GATT_ITER_STOP;
	}

	switch (params->type) {
	case BT_GATT_DISCOVER_PRIMARY: {
		const struct bt_gatt_service_val *svc = attr->user_data;

		params->uuid = &smpc_chr_uuid.uuid;
		params->start_handle = attr->handle + 1U;
		params->end_handle = svc->end_handle;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
		rc = bt_gatt_discover(conn, params);
		if (rc != 0) {
			smpc_discover_fail(conn, "chr discovery failed to start");
		}
		return BT_GATT_ITER_STOP;
	}
	case BT_GATT_DISCOVER_CHARACTERISTIC:
		smpc.chr_handle = bt_gatt_attr_value_handle(attr);
		params->uuid = &smpc_ccc_uuid.uuid;
		params->start_handle = attr->handle + 2U;
		params->type = BT_GATT_DISCOVER_DESCRIPTOR;
		rc = bt_gatt_discover(conn, params);
		if (rc != 0) {
			smpc_discover_fail(conn, "CCC discovery failed to start");
		}
		return BT_GATT_ITER_STOP;
	case BT_GATT_DISCOVER_DESCRIPTOR:
		smpc.ccc_handle = attr->handle;
		smpc.sub.notify = smpc_notify_cb;
		smpc.sub.subscribe = smpc_subscribed_cb;
		smpc.sub.value = BT_GATT_CCC_NOTIFY;
		smpc.sub.value_handle = smpc.chr_handle;
		smpc.sub.ccc_handle = smpc.ccc_handle;
		rc = bt_gatt_subscribe(conn, &smpc.sub);
		if (rc != 0) {
			smpc_discover_fail(conn, "subscribe failed to start");
		}
		return BT_GATT_ITER_STOP;
	default:
		return BT_GATT_ITER_STOP;
	}
}

static void smpc_discover_start(struct bt_conn *conn)
{
	int rc;

	memset(&smpc.disc, 0, sizeof(smpc.disc));
	smpc.disc.uuid = &smpc_svc_uuid.uuid;
	smpc.disc.func = smpc_discover_cb;
	smpc.disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	smpc.disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	smpc.disc.type = BT_GATT_DISCOVER_PRIMARY;

	rc = bt_gatt_discover(conn, &smpc.disc);
	if (rc != 0) {
		smpc_discover_fail(conn, "service discovery failed to start");
	}
}

static void smpc_mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);
	LOG_INF("SMPC MTU exchange %s, ATT MTU %u", err != 0U ? "refused" : "ok",
		bt_gatt_get_mtu(conn));
	/* Either way the link is usable; a refused exchange just means 23. */
	smpc_discover_start(conn);
}

static void smpc_connected(struct bt_conn *conn, uint8_t err)
{
	int rc;

	if (conn != smpc.conn) {
		return;
	}

	if (err != 0U) {
		LOG_WRN("SMPC connect failed (0x%02x)", err);
		k_mutex_lock(&smpc.lock, K_FOREVER);
		bt_conn_unref(smpc.conn);
		smpc.conn = NULL;
		smpc.connect_failed = true;
		k_mutex_unlock(&smpc.lock);
		k_sem_give(&smpc.ready_sem);
		return;
	}

	LOG_INF("SMPC connected");
	smpc.mtu_params.func = smpc_mtu_cb;
	rc = bt_gatt_exchange_mtu(conn, &smpc.mtu_params);
	if (rc != 0) {
		LOG_WRN("SMPC MTU exchange failed to start (%d)", rc);
		smpc_discover_start(conn);
	}
}

static void smpc_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != smpc.conn) {
		return;
	}

	LOG_INF("SMPC link down (0x%02x)", reason);
	k_mutex_lock(&smpc.lock, K_FOREVER);
	bt_conn_unref(smpc.conn);
	smpc.conn = NULL;
	smpc.last_disconnect = reason;
	smpc_link_reset_locked();
	smpc.connect_failed = true;
	k_mutex_unlock(&smpc.lock);
	/* A waiter in connect() wakes with a verdict; a stale give is harmless
	 * (the semaphore's limit is 1 and connect() resets it first). */
	k_sem_give(&smpc.ready_sem);
}

BT_CONN_CB_DEFINE(smpc_conn_cb) = {
	.connected = smpc_connected,
	.disconnected = smpc_disconnected,
};

/* ------------------------------------------------------------------------ */
/* Client stack init (lazy: the first connect)                               */

static int smpc_client_init(void)
{
	int rc;

	if (smpc_client_ready) {
		return 0;
	}

	smpc_transport.functions.output = smpc_output;
	smpc_transport.functions.get_mtu = smpc_get_mtu;
	rc = smp_transport_init(&smpc_transport);
	if (rc != 0) {
		return rc;
	}
	/* NOT SMP_BLUETOOTH_TRANSPORT: with SMP_CLIENT on, Zephyr's own smp_bt.c
	 * registers ITSELF under that type at init — its client mode sends
	 * requests as notifications over a link where this node is the GATT
	 * server, which is not what a plain SMP target answers. A second
	 * registration of the same type is silently ignored ("already in
	 * list"), and every request then dies in smp_bt_tx_pkt() with no conn
	 * attached — no log, just timeouts (found the hard way, 2026-08-26). */
	smpc_entry.smpt = &smpc_transport;
	smpc_entry.smpt_type = SMP_USER_DEFINED_TRANSPORT;
	smp_client_transport_register(&smpc_entry);

	rc = smp_client_object_init(&smpc_client, SMP_USER_DEFINED_TRANSPORT);
	if (rc != MGMT_ERR_EOK) {
		return -EIO;
	}
	img_mgmt_client_init(&smpc_img, &smpc_client, ARRAY_SIZE(smpc_images), smpc_images);
	os_mgmt_client_init(&smpc_os, &smpc_client);

	k_work_queue_start(&smpc_job_q, smpc_job_stack, K_THREAD_STACK_SIZEOF(smpc_job_stack),
			   CONFIG_MESHTASTIC_SMP_CENTRAL_JOB_PRIORITY, NULL);
	k_thread_name_set(&smpc_job_q.thread, "smpc_job");

	smpc_client_ready = true;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Public-ish operations (shell + job)                                       */

int meshtastic_smpc_connect(const bt_addr_le_t *addr, k_timeout_t timeout)
{
	int rc;

	rc = smpc_client_init();
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&smpc.lock, K_FOREVER);
	if (smpc.conn != NULL) {
		k_mutex_unlock(&smpc.lock);
		return -EALREADY;
	}
	smpc_link_reset_locked();
	smpc.connect_failed = false;
	bt_addr_le_copy(&smpc.addr, addr);
	k_sem_reset(&smpc.ready_sem);

	/* Zephyr keeps ONE connection per peer address, whichever side opened
	 * it. In a BLE pocket the target may already be linked to us — its peer
	 * link connects to every Meshtastic node it sees, a phone-side link may
	 * be up — and bt_conn_le_create() then fails with "found valid
	 * connection" (-EINVAL). GATT client procedures do not care who is
	 * central, so adopt the link that exists and run SMP over it. */
	smpc.conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
	if (smpc.conn != NULL) {
		smpc.adopted = true;
		k_mutex_unlock(&smpc.lock);
		LOG_INF("SMPC adopting the existing link to the target");
		smpc_connected(smpc.conn, 0U);
	} else {
		rc = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				       BT_LE_CONN_PARAM(SMPC_CONN_INT_MIN, SMPC_CONN_INT_MAX,
							SMPC_CONN_LATENCY, SMPC_CONN_TIMEOUT),
				       &smpc.conn);
		k_mutex_unlock(&smpc.lock);
		if (rc != 0) {
			LOG_WRN("SMPC connect failed to start (%d)", rc);
			return rc;
		}
	}

	if (k_sem_take(&smpc.ready_sem, timeout) != 0) {
		LOG_WRN("SMPC connect timed out");
		k_mutex_lock(&smpc.lock, K_FOREVER);
		if (smpc.conn != NULL) {
			(void)bt_conn_disconnect(smpc.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		k_mutex_unlock(&smpc.lock);
		return -ETIMEDOUT;
	}

	return smpc.ready ? 0 : -EIO;
}

int meshtastic_smpc_disconnect(void)
{
	int rc = -ENOTCONN;

	k_mutex_lock(&smpc.lock, K_FOREVER);
	if (smpc.conn != NULL) {
		if (smpc.adopted) {
			/* Not our link to drop: stop listening and let go of it. */
			struct bt_conn *conn = smpc.conn;
			bool was_ready = smpc.ready;

			smpc.conn = NULL;
			smpc.ready = false;
			k_mutex_unlock(&smpc.lock);
			/* Unsubscribe needs the very params that were subscribed, so
			 * it goes before the reset that wipes them. */
			if (was_ready) {
				(void)bt_gatt_unsubscribe(conn, &smpc.sub);
			}
			k_mutex_lock(&smpc.lock, K_FOREVER);
			smpc_link_reset_locked();
			k_mutex_unlock(&smpc.lock);
			bt_conn_unref(conn);
			LOG_INF("SMPC released the adopted link");
			return 0;
		}
		rc = bt_conn_disconnect(smpc.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	k_mutex_unlock(&smpc.lock);
	return rc;
}

bool meshtastic_smpc_ready(void)
{
	bool ready;

	k_mutex_lock(&smpc.lock, K_FOREVER);
	ready = smpc.ready;
	k_mutex_unlock(&smpc.lock);
	return ready;
}

/* --- the courier's image source: this node's slot1, or a file in /depot --- */

/* Where the bytes come from. slot1 is the fallback every MCUboot board has; a
 * depot file (CONFIG_MESHTASTIC_DEPOT) is what a real courier uses, because
 * slot1 is also where its OWN next image lands. */
struct smpc_src {
	const struct flash_area *fa;
#if defined(CONFIG_MESHTASTIC_DEPOT)
	struct fs_file_t file;
	bool is_file;
#endif
	uint32_t size_cap;	/* partition size, or file size */
};

static int smpc_src_open(struct smpc_src *src, const char *path)
{
	memset(src, 0, sizeof(*src));
#if defined(CONFIG_MESHTASTIC_DEPOT)
	if (path != NULL && path[0] != '\0') {
		struct fs_dirent ent;
		int rc;

		rc = fs_stat(path, &ent);
		if (rc != 0) {
			return rc;
		}
		if (ent.type != FS_DIR_ENTRY_FILE) {
			return -EISDIR;
		}
		fs_file_t_init(&src->file);
		rc = fs_open(&src->file, path, FS_O_READ);
		if (rc != 0) {
			return rc;
		}
		src->is_file = true;
		src->size_cap = ent.size;
		return 0;
	}
#else
	if (path != NULL && path[0] != '\0') {
		return -ENOTSUP;
	}
#endif
	{
		int rc = flash_area_open(FIXED_PARTITION_ID(slot1_partition), &src->fa);

		if (rc != 0) {
			return rc;
		}
		src->size_cap = src->fa->fa_size;
		return 0;
	}
}

static int smpc_src_read(struct smpc_src *src, uint32_t off, void *buf, size_t len)
{
#if defined(CONFIG_MESHTASTIC_DEPOT)
	if (src->is_file) {
		ssize_t n;

		if (fs_seek(&src->file, off, FS_SEEK_SET) != 0) {
			return -EIO;
		}
		n = fs_read(&src->file, buf, len);
		return n == (ssize_t)len ? 0 : -EIO;
	}
#endif
	return flash_area_read(src->fa, off, buf, len);
}

static void smpc_src_close(struct smpc_src *src)
{
#if defined(CONFIG_MESHTASTIC_DEPOT)
	if (src->is_file) {
		(void)fs_close(&src->file);
		return;
	}
#endif
	if (src->fa != NULL) {
		flash_area_close(src->fa);
	}
}


struct smpc_local_image {
	uint32_t size;		/* header + body + (protected) TLVs: the bytes to send */
	uint8_t major, minor;
	uint16_t revision;
	uint32_t build;
};

/* MCUboot's image layout, spelled out locally so this file has no bootutil
 * dependency (the courier need not be MCUboot-managed itself). */
struct smpc_mcuboot_hdr {
	uint32_t magic;
	uint32_t load_addr;
	uint16_t hdr_size;
	uint16_t protect_tlv_size;
	uint32_t img_size;
	uint32_t flags;
	uint8_t major, minor;
	uint16_t revision;
	uint32_t build;
	uint32_t pad;
} __packed;

struct smpc_mcuboot_tlv_info {
	uint16_t magic;
	uint16_t tot;
} __packed;

#define SMPC_MCUBOOT_MAGIC 0x96f3b83dU
#define SMPC_MCUBOOT_TLV_INFO_MAGIC 0x6907U
#define SMPC_MCUBOOT_TLV_PROT_INFO_MAGIC 0x6908U

static int smpc_local_image_inspect(struct smpc_src *src, struct smpc_local_image *out)
{
	struct smpc_mcuboot_hdr hdr;
	struct smpc_mcuboot_tlv_info tlv;
	uint32_t off;
	int rc;

	rc = smpc_src_read(src, 0, &hdr, sizeof(hdr));
	if (rc != 0) {
		return rc;
	}
	if (hdr.magic != SMPC_MCUBOOT_MAGIC) {
		return -ENOENT;
	}

	off = (uint32_t)hdr.hdr_size + hdr.img_size;
	rc = smpc_src_read(src, off, &tlv, sizeof(tlv));
	if (rc != 0) {
		return rc;
	}
	if (tlv.magic == SMPC_MCUBOOT_TLV_PROT_INFO_MAGIC) {
		/* Protected TLVs first, then the plain TLV block behind them. */
		off += tlv.tot;
		rc = smpc_src_read(src, off, &tlv, sizeof(tlv));
		if (rc != 0) {
			return rc;
		}
	}
	if (tlv.magic != SMPC_MCUBOOT_TLV_INFO_MAGIC) {
		return -EBADMSG;
	}
	off += tlv.tot;
	if (off > src->size_cap) {
		return -EOVERFLOW;
	}

	out->size = off;
	out->major = hdr.major;
	out->minor = hdr.minor;
	out->revision = hdr.revision;
	out->build = hdr.build;
	return 0;
}

int meshtastic_smpc_local_image(const char *path, struct smpc_local_image *out)
{
	struct smpc_src src;
	int rc;

	rc = smpc_src_open(&src, path);
	if (rc != 0) {
		return rc;
	}
	rc = smpc_local_image_inspect(&src, out);
	smpc_src_close(&src);
	return rc;
}

/* --- SMP operations against the connected target ------------------------- */

static const char *mgmt_err_str(int rc)
{
	switch (rc) {
	case MGMT_ERR_EOK:
		return "ok";
	case MGMT_ERR_EUNKNOWN:
		return "unknown";
	case MGMT_ERR_ENOMEM:
		return "nomem";
	case MGMT_ERR_EINVAL:
		return "inval";
	case MGMT_ERR_ETIMEOUT:
		return "timeout";
	case MGMT_ERR_ENOENT:
		return "noent";
	case MGMT_ERR_EBADSTATE:
		return "badstate";
	case MGMT_ERR_EMSGSIZE:
		return "msgsize";
	case MGMT_ERR_ENOTSUP:
		return "notsup";
	case MGMT_ERR_ECORRUPT:
		return "corrupt";
	case MGMT_ERR_EBUSY:
		return "busy";
	case MGMT_ERR_EACCESSDENIED:
		return "denied";
	default:
		return "err";
	}
}

/* Upload this node's slot1 image to the target's slot1. Progress lands in the
 * job fields so `smpc status` can watch. Returns an MGMT_ERR_* (0 = ok). */
static int smpc_do_push(const char *path, struct smpc_local_image *img)
{
	struct smpc_src src;
	struct mcumgr_image_upload up;
	int rc;

	rc = smpc_src_open(&src, path);
	if (rc != 0) {
		LOG_ERR("SMPC push: source '%s' open failed (%d)", path != NULL ? path : "slot1", rc);
		return MGMT_ERR_ENOENT;
	}
	rc = smpc_local_image_inspect(&src, img);
	if (rc != 0) {
		LOG_ERR("SMPC push: nothing pushable in '%s' (%d)", path != NULL ? path : "slot1", rc);
		smpc_src_close(&src);
		return MGMT_ERR_ENOENT;
	}

	LOG_INF("SMPC push: %u.%u.%u+%u, %u bytes", img->major, img->minor, img->revision,
		img->build, img->size);

	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_total = img->size;
	smpc.job_off = 0U;
	smpc.job_stage = "upload";
	k_mutex_unlock(&smpc.lock);

	rc = img_mgmt_client_upload_init(&smpc_img, img->size, 0U, NULL);
	if (rc != MGMT_ERR_EOK) {
		smpc_src_close(&src);
		return rc;
	}

	/* Feed from wherever the target says it is: the client tracks the
	 * server-reported offset, which is how a resumed or re-sent chunk is
	 * absorbed instead of desynchronising the stream. */
	while (smpc_img.upload.offset < img->size) {
		uint32_t off = smpc_img.upload.offset;
		uint32_t n = MIN(sizeof(smpc_read_buf), img->size - off);

		rc = smpc_src_read(&src, off, smpc_read_buf, n);
		if (rc != 0) {
			LOG_ERR("SMPC push: source read @%u failed (%d)", off, rc);
			rc = MGMT_ERR_EUNKNOWN;
			break;
		}
		rc = img_mgmt_client_upload(&smpc_img, smpc_read_buf, n, &up);
		if (rc != MGMT_ERR_EOK) {
			LOG_ERR("SMPC push: upload @%u failed (%s)", off, mgmt_err_str(rc));
			break;
		}
		if (smpc_img.upload.offset <= off) {
			/* The target did not move; do not spin forever on it. */
			LOG_ERR("SMPC push: target stuck at %u", off);
			rc = MGMT_ERR_EBADSTATE;
			break;
		}
		k_mutex_lock(&smpc.lock, K_FOREVER);
		smpc.job_off = smpc_img.upload.offset;
		k_mutex_unlock(&smpc.lock);
	}

	smpc_src_close(&src);
	return rc;
}

/* Find the slot1 entry in a state read; NULL if the target has none. */
static const struct mcumgr_image_data *smpc_find_slot(const struct mcumgr_image_state *st,
						      uint32_t slot)
{
	for (int i = 0; i < st->image_list_length; i++) {
		if (st->image_list[i].slot_num == slot) {
			return &st->image_list[i];
		}
	}
	return NULL;
}

static void smpc_job_fn(struct k_work *work)
{
	struct smpc_local_image img = {0};
	struct mcumgr_image_state st;
	enum smpc_job_kind kind;
	int rc;

	ARG_UNUSED(work);

	k_mutex_lock(&smpc.lock, K_FOREVER);
	kind = smpc.job_kind;
	smpc.job_running = true;
	smpc.job_rc = 0;
	smpc.job_t0 = k_uptime_get();
	smpc.job_t_upload = 0;
	k_mutex_unlock(&smpc.lock);

	rc = smpc_do_push(smpc.job_path[0] != '\0' ? smpc.job_path : NULL, &img);
	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_t_upload = k_uptime_get() - smpc.job_t0;
	k_mutex_unlock(&smpc.lock);
	if (rc != MGMT_ERR_EOK) {
		goto out;
	}
	LOG_INF("SMPC push: %u bytes in %lld ms (%lld B/s)", img.size, (long long)smpc.job_t_upload,
		(long long)(smpc.job_t_upload > 0 ? (img.size * 1000LL) / smpc.job_t_upload : 0LL));

	if (kind != SMPC_JOB_UPDATE) {
		goto out;
	}

	/* --- the rest of the ladder: mark test -> reset -> reconnect -> confirm */
	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_stage = "mark-test";
	k_mutex_unlock(&smpc.lock);

	rc = img_mgmt_client_state_read(&smpc_img, &st);
	if (rc != MGMT_ERR_EOK) {
		LOG_ERR("SMPC update: state read failed (%s)", mgmt_err_str(rc));
		goto out;
	}
	{
		const struct mcumgr_image_data *s1 = smpc_find_slot(&st, 1U);
		char hash[IMG_MGMT_DATA_SHA_LEN];

		if (s1 == NULL) {
			LOG_ERR("SMPC update: target shows no slot1 image after upload");
			rc = MGMT_ERR_ENOENT;
			goto out;
		}
		memcpy(hash, s1->hash, sizeof(hash));
		rc = img_mgmt_client_state_write(&smpc_img, hash, false, &st);
		if (rc != MGMT_ERR_EOK) {
			LOG_ERR("SMPC update: mark-test failed (%s)", mgmt_err_str(rc));
			goto out;
		}
	}

	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_stage = "reset";
	k_mutex_unlock(&smpc.lock);
	rc = os_mgmt_client_reset(&smpc_os);
	if (rc != MGMT_ERR_EOK) {
		LOG_ERR("SMPC update: reset failed (%s)", mgmt_err_str(rc));
		goto out;
	}

	/* The target reboots into MCUboot, swaps, and comes back advertising.
	 * Our link drops on its own; wait it out, then reconnect until it is
	 * back or the budget is spent. */
	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_stage = "reboot-wait";
	k_mutex_unlock(&smpc.lock);
	{
		int64_t deadline = k_uptime_get() + SMPC_UPDATE_REBOOT_WAIT_MS;
		bt_addr_le_t addr;

		k_mutex_lock(&smpc.lock, K_FOREVER);
		bt_addr_le_copy(&addr, &smpc.addr);
		k_mutex_unlock(&smpc.lock);

		k_sleep(K_MSEC(SMPC_UPDATE_RECONNECT_GAP_MS));
		(void)meshtastic_smpc_disconnect();
		rc = -ENOTCONN;
		while (k_uptime_get() < deadline) {
			k_sleep(K_MSEC(SMPC_UPDATE_RECONNECT_GAP_MS));
			if (meshtastic_smpc_ready()) {
				rc = 0;
				break;
			}
			rc = meshtastic_smpc_connect(&addr, K_MSEC(SMPC_CONNECT_TIMEOUT_MS));
			if (rc == 0) {
				break;
			}
		}
		if (rc != 0) {
			LOG_ERR("SMPC update: target did not come back (%d)", rc);
			rc = MGMT_ERR_ETIMEOUT;
			goto out;
		}
	}

	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_stage = "verify";
	k_mutex_unlock(&smpc.lock);
	rc = img_mgmt_client_state_read(&smpc_img, &st);
	if (rc != MGMT_ERR_EOK) {
		LOG_ERR("SMPC update: post-swap state read failed (%s)", mgmt_err_str(rc));
		goto out;
	}
	{
		const struct mcumgr_image_data *s0 = smpc_find_slot(&st, 0U);
		char want[IMG_MGMT_VER_MAX_STR_LEN + 1];

		snprintf(want, sizeof(want), "%u.%u.%u", img.major, img.minor, img.revision);
		if (s0 == NULL || !s0->flags.active || strncmp(s0->version, want, strlen(want)) != 0) {
			LOG_ERR("SMPC update: running '%s', wanted %s — not confirming (MCUboot will revert)",
				s0 != NULL ? s0->version : "?", want);
			rc = MGMT_ERR_EBADSTATE;
			goto out;
		}
	}

	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_stage = "confirm";
	k_mutex_unlock(&smpc.lock);
	rc = img_mgmt_client_state_write(&smpc_img, NULL, true, &st);
	if (rc != MGMT_ERR_EOK) {
		LOG_ERR("SMPC update: confirm failed (%s)", mgmt_err_str(rc));
		goto out;
	}
	LOG_INF("SMPC update: target now runs %u.%u.%u+%u, confirmed, %lld ms end to end",
		img.major, img.minor, img.revision, img.build, (long long)(k_uptime_get() - smpc.job_t0));

out:
	k_mutex_lock(&smpc.lock, K_FOREVER);
	smpc.job_rc = rc;
	smpc.job_running = false;
	smpc.job_stage = rc == 0 ? "done" : "failed";
	k_mutex_unlock(&smpc.lock);
	LOG_INF("SMPC job %s: %s", kind == SMPC_JOB_UPDATE ? "update" : "push",
		rc == 0 ? "OK" : mgmt_err_str(rc));
}

static int smpc_job_start(enum smpc_job_kind kind, const char *path)
{
	int rc = 0;

	if (!meshtastic_smpc_ready()) {
		return -ENOTCONN;
	}
	k_mutex_lock(&smpc.lock, K_FOREVER);
	if (smpc.job_running) {
		rc = -EBUSY;
	} else {
		smpc.job_kind = kind;
		smpc.job_running = true;
		smpc.job_stage = "queued";
		smpc.job_off = 0U;
		smpc.job_total = 0U;
		strncpy(smpc.job_path, path != NULL ? path : "", sizeof(smpc.job_path) - 1);
		smpc.job_path[sizeof(smpc.job_path) - 1] = '\0';
		k_work_init(&smpc.job_work, smpc_job_fn);
		(void)k_work_submit_to_queue(&smpc_job_q, &smpc.job_work);
	}
	k_mutex_unlock(&smpc.lock);
	return rc;
}

/* ------------------------------------------------------------------------ */
/* Shell: `smpc ...`                                                          */

#if defined(CONFIG_SHELL)

static int cmd_connect(const struct shell *sh, size_t argc, char **argv)
{
	bt_addr_le_t addr;
	const char *type = argc > 2 ? argv[2] : "random";
	int rc;

	rc = bt_addr_le_from_str(argv[1], type, &addr);
	if (rc != 0) {
		shell_error(sh, "bad address (want XX:XX:XX:XX:XX:XX [public|random])");
		return -EINVAL;
	}
	shell_print(sh, "connecting to %s (%s)...", argv[1], type);
	rc = meshtastic_smpc_connect(&addr, K_MSEC(SMPC_CONNECT_TIMEOUT_MS));
	if (rc != 0) {
		shell_error(sh, "connect failed (%d)", rc);
		return rc;
	}
	shell_print(sh, "SMP link ready, ATT MTU %u", bt_gatt_get_mtu(smpc.conn));
	return 0;
}

static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	int rc = meshtastic_smpc_disconnect();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (rc != 0) {
		shell_error(sh, "disconnect: %d", rc);
	}
	return rc;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	char addr[BT_ADDR_LE_STR_LEN] = "-";

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_mutex_lock(&smpc.lock, K_FOREVER);
	if (smpc.conn != NULL) {
		bt_addr_le_to_str(&smpc.addr, addr, sizeof(addr));
	}
	shell_print(sh, "link: %s %s%s%s",
		    smpc.conn == NULL ? "down" : smpc.ready ? "connected" : "connecting", addr,
		    smpc.ready ? " (SMP ready)" : "", smpc.adopted ? " (adopted)" : "");
	if (smpc.conn != NULL) {
		shell_print(sh, "  ATT MTU %u chr 0x%04x", bt_gatt_get_mtu(smpc.conn),
			    smpc.chr_handle);
	} else {
		shell_print(sh, "  last disconnect 0x%02x", smpc.last_disconnect);
	}
	shell_print(sh, "tx: %u frames %u chunks %u errors", smpc.tx_frames, smpc.tx_chunks,
		    smpc.tx_errors);
	shell_print(sh, "rx: %u frames %u chunks %u dropped %u unmatched", smpc.rx_frames,
		    smpc.rx_chunks, smpc.rx_dropped, smpc.rx_unmatched);
	if (smpc.job_kind != SMPC_JOB_NONE) {
		int64_t el = k_uptime_get() - smpc.job_t0;

		shell_print(sh, "job: %s %s %s", smpc.job_kind == SMPC_JOB_UPDATE ? "update" : "push",
			    smpc.job_running ? "running" : "idle", smpc.job_stage);
		if (smpc.job_total != 0U) {
			shell_print(sh, "  %u/%u bytes (%u%%) %s%lld ms%s", smpc.job_off,
				    smpc.job_total, (smpc.job_off * 100U) / smpc.job_total,
				    smpc.job_t_upload != 0 ? "upload " : "",
				    (long long)(smpc.job_t_upload != 0 ? smpc.job_t_upload : el),
				    smpc.job_t_upload != 0 ? "" : " elapsed");
		}
		if (!smpc.job_running) {
			shell_print(sh, "  result: %s", mgmt_err_str(smpc.job_rc));
		}
	}
	k_mutex_unlock(&smpc.lock);
	return 0;
}

static int cmd_image(const struct shell *sh, size_t argc, char **argv)
{
	struct smpc_local_image img;
	int rc;

	rc = meshtastic_smpc_local_image(argc > 1 ? argv[1] : NULL, &img);
	if (rc != 0) {
		shell_error(sh, "%s: nothing pushable (%d)", argc > 1 ? argv[1] : "slot1", rc);
		return rc;
	}
	shell_print(sh, "%s (courier image): %u.%u.%u+%u, %u bytes", argc > 1 ? argv[1] : "slot1",
		    img.major, img.minor, img.revision, img.build, img.size);
	return 0;
}

static int cmd_echo(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	if (!meshtastic_smpc_ready()) {
		shell_error(sh, "not connected");
		return -ENOTCONN;
	}
	rc = os_mgmt_client_echo(&smpc_os, argc > 1 ? argv[1] : "zephyrtastic", 64);
	shell_print(sh, "echo: %s", mgmt_err_str(rc));
	return rc == MGMT_ERR_EOK ? 0 : -EIO;
}

static void print_state(const struct shell *sh, const struct mcumgr_image_state *st)
{
	for (int i = 0; i < st->image_list_length; i++) {
		const struct mcumgr_image_data *d = &st->image_list[i];
		char hash[2 * IMG_MGMT_DATA_SHA_LEN + 1];

		for (int j = 0; j < IMG_MGMT_DATA_SHA_LEN; j++) {
			snprintf(&hash[2 * j], 3, "%02x", (uint8_t)d->hash[j]);
		}
		shell_print(sh, "  image %u slot %u: %s%s%s%s%s%s", d->img_num, d->slot_num,
			    d->version, d->flags.active ? " active" : "",
			    d->flags.confirmed ? " confirmed" : "", d->flags.pending ? " pending" : "",
			    d->flags.permanent ? " permanent" : "",
			    d->flags.bootable ? " bootable" : "");
		shell_print(sh, "    hash %s", hash);
	}
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	struct mcumgr_image_state st;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_smpc_ready()) {
		shell_error(sh, "not connected");
		return -ENOTCONN;
	}
	rc = img_mgmt_client_state_read(&smpc_img, &st);
	if (rc != MGMT_ERR_EOK) {
		shell_error(sh, "state read: %s", mgmt_err_str(rc));
		return -EIO;
	}
	shell_print(sh, "target images (%d):", st.image_list_length);
	print_state(sh, &st);
	return 0;
}

static int cmd_push(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	rc = smpc_job_start(SMPC_JOB_PUSH, argc > 1 ? argv[1] : NULL);
	if (rc != 0) {
		shell_error(sh, "push: %s", rc == -EBUSY ? "a job is running" : "not connected");
		return rc;
	}
	shell_print(sh, "push started — `smpc status` for progress");
	return 0;
}

static int cmd_update(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	rc = smpc_job_start(SMPC_JOB_UPDATE, argc > 1 ? argv[1] : NULL);
	if (rc != 0) {
		shell_error(sh, "update: %s", rc == -EBUSY ? "a job is running" : "not connected");
		return rc;
	}
	shell_print(sh, "update started (push -> test -> reset -> reconnect -> confirm) — `smpc status`");
	return 0;
}

/* `smpc test` marks the target's slot1 for a one-shot boot; the hash comes
 * from a fresh state read so there is nothing to type. */
static int cmd_test(const struct shell *sh, size_t argc, char **argv)
{
	struct mcumgr_image_state st;
	const struct mcumgr_image_data *s1;
	char hash[IMG_MGMT_DATA_SHA_LEN];
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_smpc_ready()) {
		shell_error(sh, "not connected");
		return -ENOTCONN;
	}
	rc = img_mgmt_client_state_read(&smpc_img, &st);
	if (rc != MGMT_ERR_EOK) {
		shell_error(sh, "state read: %s", mgmt_err_str(rc));
		return -EIO;
	}
	s1 = smpc_find_slot(&st, 1U);
	if (s1 == NULL) {
		shell_error(sh, "target has no slot1 image");
		return -ENOENT;
	}
	memcpy(hash, s1->hash, sizeof(hash));
	rc = img_mgmt_client_state_write(&smpc_img, hash, false, &st);
	if (rc != MGMT_ERR_EOK) {
		shell_error(sh, "mark test: %s", mgmt_err_str(rc));
		return -EIO;
	}
	shell_print(sh, "slot1 marked for test:");
	print_state(sh, &st);
	return 0;
}

static int cmd_confirm(const struct shell *sh, size_t argc, char **argv)
{
	struct mcumgr_image_state st;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_smpc_ready()) {
		shell_error(sh, "not connected");
		return -ENOTCONN;
	}
	rc = img_mgmt_client_state_write(&smpc_img, NULL, true, &st);
	if (rc != MGMT_ERR_EOK) {
		shell_error(sh, "confirm: %s", mgmt_err_str(rc));
		return -EIO;
	}
	shell_print(sh, "running image confirmed:");
	print_state(sh, &st);
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_smpc_ready()) {
		shell_error(sh, "not connected");
		return -ENOTCONN;
	}
	rc = os_mgmt_client_reset(&smpc_os);
	shell_print(sh, "reset: %s", mgmt_err_str(rc));
	return rc == MGMT_ERR_EOK ? 0 : -EIO;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	smpc_cmds,
	SHELL_CMD_ARG(connect, NULL, "<addr> [public|random]  open an SMP link to a target",
		      cmd_connect, 2, 1),
	SHELL_CMD(disconnect, NULL, "drop the SMP link", cmd_disconnect),
	SHELL_CMD(status, NULL, "link, counters, job progress", cmd_status),
	SHELL_CMD_ARG(image, NULL, "[path]  what slot1 (or a depot file) holds", cmd_image, 1, 1),
	SHELL_CMD_ARG(echo, NULL, "[text]  os_mgmt echo round-trip", cmd_echo, 1, 1),
	SHELL_CMD(list, NULL, "target's image slots (img_mgmt state read)", cmd_list),
	SHELL_CMD_ARG(push, NULL, "[path]  upload slot1 (or a depot file) into the target's slot1",
		      cmd_push, 1, 1),
	SHELL_CMD(test, NULL, "mark the target's slot1 for a test boot", cmd_test),
	SHELL_CMD(reset, NULL, "reset the target (os_mgmt)", cmd_reset),
	SHELL_CMD(confirm, NULL, "confirm the target's running image", cmd_confirm),
	SHELL_CMD_ARG(update, NULL,
		      "[path]  the whole ladder: push, test, reset, reconnect, verify, confirm",
		      cmd_update, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(smpc, &smpc_cmds, "SMP (mcumgr) client over BLE: manage a peer node", NULL);

#endif /* CONFIG_SHELL */
