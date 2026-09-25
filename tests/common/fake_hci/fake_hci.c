/* SPDX-License-Identifier: GPL-3.0
 *
 * Fake Bluetooth LE controller for native_sim tests — see fake_hci.h.
 *
 * Shape borrowed from zephyr/tests/bluetooth/host_long_adv_recv (a
 * zephyr,bt-hci driver whose send() answers commands inline), extended with
 * a state model, connections, ACL/ATT and an event thread for everything a
 * real controller sends unprompted.
 */

#include <string.h>

#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "fake_hci.h"

#define DT_DRV_COMPAT zephyr_bt_hci_fake

#define ATT_CID 0x0004U
#define SMP_CID 0x0006U

struct fake_conn {
	bool in_use;
	uint16_t handle;
	uint8_t role;
	bt_addr_le_t peer;
};

struct fake_fail {
	uint16_t opcode;
	uint8_t status;
	uint32_t count;
};

static struct k_spinlock lock;
static struct fake_hci_state st;
static bt_addr_t public_addr;
static struct fake_conn conns[FAKE_HCI_MAX_CONN];
static uint16_t next_handle = 1U;
static struct fake_fail fails[8];
static bool auto_connect;
static uint32_t auto_connect_delay_ms;
static bool att_silent;
static uint8_t rand_seed;

static K_FIFO_DEFINE(evt_fifo);

static const struct device *fake_dev(void)
{
	return DEVICE_DT_GET(DT_DRV_INST(0));
}

/* ------------------------------------------------------------------ events */

static struct net_buf *evt_new(uint8_t evt, uint8_t len)
{
	struct net_buf *buf = bt_buf_get_evt(evt, false, K_FOREVER);
	struct bt_hci_evt_hdr *hdr = net_buf_add(buf, sizeof(*hdr));

	hdr->evt = evt;
	hdr->len = len;
	return buf;
}

static void *le_meta_new(struct net_buf **out, uint8_t subevent, uint8_t len)
{
	struct bt_hci_evt_le_meta_event *meta;

	*out = evt_new(BT_HCI_EVT_LE_META_EVENT, sizeof(*meta) + len);
	meta = net_buf_add(*out, sizeof(*meta));
	meta->subevent = subevent;
	return net_buf_add(*out, len);
}

/* Unprompted events go through the event thread, in order, never from the
 * caller's context: a real controller's events race the host's commands. */
static void evt_queue(struct net_buf *buf)
{
	k_fifo_put(&evt_fifo, buf);
}

static void evt_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct net_buf *buf = k_fifo_get(&evt_fifo, K_FOREVER);

		bt_hci_recv(fake_dev(), buf);
	}
}

K_THREAD_DEFINE(fake_hci_evt, 4096, evt_thread, NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

static void *cmd_complete(struct net_buf **buf, uint16_t opcode, uint8_t plen)
{
	struct bt_hci_evt_cmd_complete *cc;

	*buf = evt_new(BT_HCI_EVT_CMD_COMPLETE, sizeof(*cc) + plen);
	cc = net_buf_add(*buf, sizeof(*cc));
	cc->ncmd = 1U;
	cc->opcode = sys_cpu_to_le16(opcode);
	return memset(net_buf_add(*buf, plen), 0, plen);
}

static void reply_status_only(uint16_t opcode, uint8_t status)
{
	struct net_buf *buf;
	struct bt_hci_evt_cc_status *cc = cmd_complete(&buf, opcode, sizeof(*cc));

	cc->status = status;
	bt_hci_recv(fake_dev(), buf);
}

static void reply_cmd_status(uint16_t opcode, uint8_t status)
{
	struct net_buf *buf = evt_new(BT_HCI_EVT_CMD_STATUS, sizeof(struct bt_hci_evt_cmd_status));
	struct bt_hci_evt_cmd_status *cs = net_buf_add(buf, sizeof(*cs));

	cs->status = status;
	cs->ncmd = 1U;
	cs->opcode = sys_cpu_to_le16(opcode);
	bt_hci_recv(fake_dev(), buf);
}

static void queue_conn_complete(uint8_t status, uint16_t handle, uint8_t role,
				const bt_addr_le_t *peer)
{
	struct net_buf *buf;
	struct bt_hci_evt_le_conn_complete *cc =
		le_meta_new(&buf, BT_HCI_EVT_LE_CONN_COMPLETE, sizeof(*cc));

	memset(cc, 0, sizeof(*cc));
	cc->status = status;
	cc->handle = sys_cpu_to_le16(handle);
	cc->role = role;
	bt_addr_le_copy(&cc->peer_addr, peer);
	cc->interval = sys_cpu_to_le16(24U);
	cc->supv_timeout = sys_cpu_to_le16(400U);
	evt_queue(buf);
}

static void queue_disconn_complete(uint16_t handle, uint8_t reason)
{
	struct net_buf *buf =
		evt_new(BT_HCI_EVT_DISCONN_COMPLETE, sizeof(struct bt_hci_evt_disconn_complete));
	struct bt_hci_evt_disconn_complete *dc = net_buf_add(buf, sizeof(*dc));

	dc->status = 0U;
	dc->handle = sys_cpu_to_le16(handle);
	dc->reason = reason;
	evt_queue(buf);
}

static void queue_nocp(uint16_t handle)
{
	struct net_buf *buf = evt_new(BT_HCI_EVT_NUM_COMPLETED_PACKETS,
				      sizeof(struct bt_hci_evt_num_completed_packets) +
					      sizeof(struct bt_hci_handle_count));
	struct bt_hci_evt_num_completed_packets *ev = net_buf_add(buf, sizeof(*ev));
	struct bt_hci_handle_count *hc = net_buf_add(buf, sizeof(*hc));

	ev->num_handles = 1U;
	hc->handle = sys_cpu_to_le16(handle);
	hc->count = sys_cpu_to_le16(1U);
	evt_queue(buf);
}

static void queue_acl(uint16_t handle, uint16_t cid, const uint8_t *pdu, uint16_t len)
{
	struct net_buf *buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
	struct bt_hci_acl_hdr *acl = net_buf_add(buf, sizeof(*acl));

	acl->handle = sys_cpu_to_le16(bt_acl_handle_pack(handle, BT_ACL_START));
	acl->len = sys_cpu_to_le16(4U + len);
	net_buf_add_le16(buf, len);
	net_buf_add_le16(buf, cid);
	net_buf_add_mem(buf, pdu, len);
	evt_queue(buf);
}

/* ------------------------------------------------------------------- state */

static bool own_addr_needs_random(uint8_t own_addr_type)
{
	return own_addr_type == BT_HCI_OWN_ADDR_RANDOM ||
	       own_addr_type == BT_HCI_OWN_ADDR_RPA_OR_RANDOM;
}

static struct fake_conn *conn_find(uint16_t handle)
{
	for (size_t i = 0; i < ARRAY_SIZE(conns); i++) {
		if (conns[i].in_use && conns[i].handle == handle) {
			return &conns[i];
		}
	}
	return NULL;
}

/* Under lock. */
static int conn_new(uint8_t role, const bt_addr_le_t *peer)
{
	for (size_t i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!conns[i].in_use) {
			conns[i].in_use = true;
			conns[i].handle = next_handle++;
			conns[i].role = role;
			bt_addr_le_copy(&conns[i].peer, peer);
			st.conns_up++;
			return conns[i].handle;
		}
	}
	return -1;
}

/* Under lock: an injected failure for this opcode, or 0. */
static uint8_t injected_status(uint16_t opcode)
{
	for (size_t i = 0; i < ARRAY_SIZE(fails); i++) {
		if (fails[i].count > 0U && fails[i].opcode == opcode) {
			fails[i].count--;
			return fails[i].status;
		}
	}
	return 0U;
}

static bool is_status_command(uint16_t opcode)
{
	switch (opcode) {
	case BT_HCI_OP_LE_CREATE_CONN:
	case BT_HCI_OP_DISCONNECT:
	case BT_HCI_OP_LE_READ_REMOTE_FEATURES:
	case BT_HCI_OP_READ_REMOTE_VERSION_INFO:
	case BT_HCI_OP_LE_CONN_UPDATE:
		return true;
	default:
		return false;
	}
}

/* ---------------------------------------------------------------- commands */

static void cmd_enable(uint16_t opcode, bool enable)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool adv = (opcode == BT_HCI_OP_LE_SET_ADV_ENABLE);
	uint8_t own = adv ? st.adv_own_addr_type : st.scan_own_addr_type;
	uint8_t status = 0U;

	if (enable && own_addr_needs_random(own) && !st.random_addr_set) {
		/* Core Vol 4 Part E 7.8.9 / 7.8.11: own address type random
		 * and no random address set. */
		status = BT_HCI_ERR_INVALID_PARAM;
	}
	if (adv) {
		st.adv_enable_last_status = status;
		if (status == 0U) {
			st.adv_enabled = enable;
			st.adv_enable_ok += enable ? 1U : 0U;
		} else {
			st.adv_enable_rejected++;
		}
	} else {
		st.scan_enable_last_status = status;
		if (status == 0U) {
			st.scan_enabled = enable;
			st.scan_enable_ok += enable ? 1U : 0U;
		} else {
			st.scan_enable_rejected++;
		}
	}
	k_spin_unlock(&lock, key);

	reply_status_only(opcode, status);
}

static void auto_connect_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)fake_hci_complete_create(0U);
}

static K_WORK_DELAYABLE_DEFINE(auto_connect_work, auto_connect_fn);

static void handle_cmd(uint16_t opcode, struct net_buf *cp)
{
	struct net_buf *rsp;
	k_spinlock_key_t key;
	uint8_t status;

	key = k_spin_lock(&lock);
	status = injected_status(opcode);
	k_spin_unlock(&lock, key);
	if (status != 0U) {
		if (is_status_command(opcode)) {
			reply_cmd_status(opcode, status);
		} else {
			reply_status_only(opcode, status);
		}
		return;
	}

	switch (opcode) {
	case BT_HCI_OP_READ_LOCAL_VERSION_INFO: {
		struct bt_hci_rp_read_local_version_info *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		rp->hci_version = BT_HCI_VERSION_5_0;
		rp->lmp_version = BT_HCI_VERSION_5_0;
		rp->manufacturer = sys_cpu_to_le16(0x05F1U);
		break;
	}
	case BT_HCI_OP_READ_SUPPORTED_COMMANDS: {
		struct bt_hci_rp_read_supported_commands *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		memset(rp->commands, 0xFF, sizeof(rp->commands));
		rp->status = 0U;
		break;
	}
	case BT_HCI_OP_READ_LOCAL_FEATURES: {
		struct bt_hci_rp_read_local_features *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		/* LE supported (controller), BR/EDR not supported. */
		rp->features[4] = BIT(5) | BIT(6);
		break;
	}
	case BT_HCI_OP_READ_BD_ADDR: {
		struct bt_hci_rp_read_bd_addr *rp = cmd_complete(&rsp, opcode, sizeof(*rp));

		bt_addr_copy(&rp->bdaddr, &public_addr);
		break;
	}
	case BT_HCI_OP_LE_READ_LOCAL_FEATURES: {
		struct bt_hci_rp_le_read_local_features *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		/* Encryption only: no DLE, no privacy, no extended advertising
		 * (the ESP32-S3's legacy-only controller shape). */
		rp->features[0] = BIT(0);
		break;
	}
	case BT_HCI_OP_LE_READ_BUFFER_SIZE: {
		struct bt_hci_rp_le_read_buffer_size *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		rp->le_max_len = sys_cpu_to_le16(27U);
		rp->le_max_num = 6U;
		break;
	}
	case BT_HCI_OP_LE_READ_SUPP_STATES: {
		struct bt_hci_rp_le_read_supp_states *rp =
			cmd_complete(&rsp, opcode, sizeof(*rp));

		memset(&rp->le_states, 0xFF, sizeof(rp->le_states));
		break;
	}
	case BT_HCI_OP_LE_RAND: {
		struct bt_hci_rp_le_rand *rp = cmd_complete(&rsp, opcode, sizeof(*rp));

		for (size_t i = 0; i < sizeof(rp->rand); i++) {
			rp->rand[i] = (uint8_t)(0x5AU + rand_seed++ * 37U);
		}
		break;
	}
	case BT_HCI_OP_LE_READ_FAL_SIZE: {
		struct bt_hci_rp_le_read_fal_size *rp = cmd_complete(&rsp, opcode, sizeof(*rp));

		rp->fal_size = 8U;
		break;
	}
	case BT_HCI_OP_LE_READ_ADV_CHAN_TX_POWER:
		(void)cmd_complete(&rsp, opcode, sizeof(struct bt_hci_rp_le_read_chan_tx_power));
		break;
	case BT_HCI_OP_LE_SET_RANDOM_ADDRESS: {
		const struct bt_hci_cp_le_set_random_address *c = (const void *)cp->data;

		key = k_spin_lock(&lock);
		if (st.adv_enabled || st.scan_enabled || st.initiating) {
			status = BT_HCI_ERR_CMD_DISALLOWED; /* 7.8.4 */
		} else {
			bt_addr_copy(&st.random_addr, &c->bdaddr);
			st.random_addr_set = true;
			st.random_addr_writes++;
		}
		k_spin_unlock(&lock, key);
		reply_status_only(opcode, status);
		return;
	}
	case BT_HCI_OP_LE_SET_ADV_PARAM: {
		const struct bt_hci_cp_le_set_adv_param *c = (const void *)cp->data;

		key = k_spin_lock(&lock);
		if (st.adv_enabled) {
			status = BT_HCI_ERR_CMD_DISALLOWED;
		} else {
			st.adv_type = c->type;
			st.adv_own_addr_type = c->own_addr_type;
		}
		k_spin_unlock(&lock, key);
		reply_status_only(opcode, status);
		return;
	}
	case BT_HCI_OP_LE_SET_SCAN_PARAM: {
		const struct bt_hci_cp_le_set_scan_param *c = (const void *)cp->data;

		key = k_spin_lock(&lock);
		if (st.scan_enabled) {
			status = BT_HCI_ERR_CMD_DISALLOWED;
		} else {
			st.scan_type = c->scan_type;
			st.scan_own_addr_type = c->addr_type;
		}
		k_spin_unlock(&lock, key);
		reply_status_only(opcode, status);
		return;
	}
	case BT_HCI_OP_LE_SET_ADV_ENABLE:
	case BT_HCI_OP_LE_SET_SCAN_ENABLE:
		cmd_enable(opcode, cp->data[0] != 0U);
		return;
	case BT_HCI_OP_LE_CREATE_CONN: {
		const struct bt_hci_cp_le_create_conn *c = (const void *)cp->data;
		bool autoc;

		key = k_spin_lock(&lock);
		if (st.initiating) {
			status = BT_HCI_ERR_CMD_DISALLOWED;
		} else if (own_addr_needs_random(c->own_addr_type) && !st.random_addr_set) {
			status = BT_HCI_ERR_INVALID_PARAM;
		} else {
			st.initiating = true;
			bt_addr_le_copy(&st.init_peer, &c->peer_addr);
			st.create_conn_count++;
		}
		autoc = auto_connect;
		k_spin_unlock(&lock, key);
		reply_cmd_status(opcode, status);
		if (status == 0U && autoc) {
			(void)k_work_reschedule(&auto_connect_work, K_MSEC(auto_connect_delay_ms));
		}
		return;
	}
	case BT_HCI_OP_LE_CREATE_CONN_CANCEL: {
		bool was;

		key = k_spin_lock(&lock);
		was = st.initiating;
		st.initiating = false;
		if (was) {
			st.create_conn_cancel_count++;
		}
		k_spin_unlock(&lock, key);
		(void)k_work_cancel_delayable(&auto_connect_work);
		reply_status_only(opcode, was ? 0U : BT_HCI_ERR_CMD_DISALLOWED);
		if (was) {
			queue_conn_complete(BT_HCI_ERR_UNKNOWN_CONN_ID, 0U, BT_HCI_ROLE_CENTRAL,
					    &st.init_peer);
		}
		return;
	}
	case BT_HCI_OP_DISCONNECT: {
		const struct bt_hci_cp_disconnect *c = (const void *)cp->data;
		uint16_t handle = sys_le16_to_cpu(c->handle);
		struct fake_conn *fc;

		key = k_spin_lock(&lock);
		fc = conn_find(handle);
		if (fc != NULL) {
			fc->in_use = false;
			st.disconnect_cmds++;
		}
		k_spin_unlock(&lock, key);
		reply_cmd_status(opcode, fc != NULL ? 0U : BT_HCI_ERR_UNKNOWN_CONN_ID);
		if (fc != NULL) {
			queue_disconn_complete(handle, BT_HCI_ERR_LOCALHOST_TERM_CONN);
		}
		return;
	}
	case BT_HCI_OP_LE_READ_REMOTE_FEATURES: {
		const struct bt_hci_cp_le_read_remote_features *c = (const void *)cp->data;
		struct bt_hci_evt_le_remote_feat_complete *ev;
		struct net_buf *buf;

		reply_cmd_status(opcode, 0U);
		ev = le_meta_new(&buf, BT_HCI_EVT_LE_REMOTE_FEAT_COMPLETE, sizeof(*ev));
		memset(ev, 0, sizeof(*ev));
		ev->handle = c->handle;
		ev->features[0] = BIT(0);
		evt_queue(buf);
		return;
	}
	case BT_HCI_OP_READ_REMOTE_VERSION_INFO: {
		const struct bt_hci_cp_read_remote_version_info *c = (const void *)cp->data;
		struct net_buf *buf = evt_new(BT_HCI_EVT_REMOTE_VERSION_INFO,
					      sizeof(struct bt_hci_evt_remote_version_info));
		struct bt_hci_evt_remote_version_info *ev = net_buf_add(buf, sizeof(*ev));

		reply_cmd_status(opcode, 0U);
		memset(ev, 0, sizeof(*ev));
		ev->handle = c->handle;
		ev->version = BT_HCI_VERSION_5_0;
		evt_queue(buf);
		return;
	}
	case BT_HCI_OP_LE_CONN_UPDATE: {
		const struct hci_cp_le_conn_update *c = (const void *)cp->data;
		struct bt_hci_evt_le_conn_update_complete *ev;
		struct net_buf *buf;

		reply_cmd_status(opcode, 0U);
		ev = le_meta_new(&buf, BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE, sizeof(*ev));
		ev->status = 0U;
		ev->handle = c->handle;
		ev->interval = c->conn_interval_max;
		ev->latency = c->conn_latency;
		ev->supv_timeout = c->supervision_timeout;
		evt_queue(buf);
		return;
	}
	case BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS:
		/* Host-to-controller flow control credits: no response. */
		return;
	case BT_HCI_OP_RESET:
	case BT_HCI_OP_SET_CTL_TO_HOST_FLOW:
	case BT_HCI_OP_HOST_BUFFER_SIZE:
	case BT_HCI_OP_SET_EVENT_MASK:
	case BT_HCI_OP_SET_EVENT_MASK_PAGE_2:
	case BT_HCI_OP_LE_SET_EVENT_MASK:
	case BT_HCI_OP_LE_WRITE_LE_HOST_SUPP:
	case BT_HCI_OP_LE_SET_ADV_DATA:
	case BT_HCI_OP_LE_SET_SCAN_RSP_DATA:
	case BT_HCI_OP_LE_CLEAR_FAL:
	case BT_HCI_OP_LE_ADD_DEV_TO_FAL:
	case BT_HCI_OP_LE_REM_DEV_FROM_FAL:
		reply_status_only(opcode, 0U);
		return;
	default:
		printk("fake_hci: unhandled opcode 0x%04x\n", opcode);
		reply_status_only(opcode, BT_HCI_ERR_UNKNOWN_CMD);
		return;
	}

	bt_hci_recv(fake_dev(), rsp);
}

/* ------------------------------------------------------------------- ACL */

static void handle_att(uint16_t handle, const uint8_t *pdu, uint16_t len)
{
	uint8_t op = pdu[0];

	if (op == 0x02U) { /* Exchange MTU Request -> Response, 23 */
		const uint8_t rsp[] = {0x03U, 23U, 0U};

		queue_acl(handle, ATT_CID, rsp, sizeof(rsp));
		return;
	}
	/* Requests have bit 6 (command) clear, and are even-numbered;
	 * responses, notifications, indications, commands get no answer. */
	switch (op) {
	case 0x04: /* Find Information */
	case 0x06: /* Find By Type Value */
	case 0x08: /* Read By Type */
	case 0x0A: /* Read */
	case 0x0C: /* Read Blob */
	case 0x10: /* Read By Group Type */
	case 0x12: /* Write */
	case 0x16: /* Prepare Write */
	case 0x18: /* Execute Write */ {
		const uint8_t rsp[] = {0x01U, op, len >= 3U ? pdu[1] : 0U,
				       len >= 3U ? pdu[2] : 0U, 0x0AU /* not found */};

		queue_acl(handle, ATT_CID, rsp, sizeof(rsp));
		return;
	}
	default:
		return;
	}
}

static void handle_acl(struct net_buf *buf)
{
	struct bt_hci_acl_hdr *acl = net_buf_pull_mem(buf, sizeof(*acl));
	uint16_t hf = sys_le16_to_cpu(acl->handle);
	uint16_t handle = bt_acl_handle(hf);
	bool start = bt_acl_flags_pb(bt_acl_flags(hf)) != BT_ACL_CONT;
	bool silent;

	queue_nocp(handle);
	if (!start || buf->len < 4U) {
		return;
	}

	uint16_t l2len = net_buf_pull_le16(buf);
	uint16_t cid = net_buf_pull_le16(buf);

	if (buf->len < 1U || l2len < 1U) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	silent = att_silent;
	if (cid == ATT_CID) {
		st.att_requests++;
	}
	k_spin_unlock(&lock, key);

	if (cid == ATT_CID && !silent) {
		handle_att(handle, buf->data, MIN(buf->len, l2len));
	} else if (cid == SMP_CID && buf->data[0] == 0x01U) {
		/* Pairing Request -> Pairing Failed, "pairing not supported". */
		const uint8_t rsp[] = {0x05U, 0x05U};

		queue_acl(handle, SMP_CID, rsp, sizeof(rsp));
	}
}

/* ---------------------------------------------------------------- driver */

static int driver_open(const struct device *dev)
{
	/* bt_hci_open() has already stored the host's recv callback. */
	ARG_UNUSED(dev);
	return 0;
}

static int driver_send(const struct device *dev, struct net_buf *buf)
{
	uint8_t type = net_buf_pull_u8(buf);

	ARG_UNUSED(dev);

	if (type == BT_HCI_H4_CMD) {
		struct bt_hci_cmd_hdr *hdr = net_buf_pull_mem(buf, sizeof(*hdr));

		handle_cmd(sys_le16_to_cpu(hdr->opcode), buf);
	} else if (type == BT_HCI_H4_ACL) {
		handle_acl(buf);
	}
	net_buf_unref(buf);
	return 0;
}

static DEVICE_API(bt_hci, driver_api) = {
	.open = driver_open,
	.send = driver_send,
};

#define FAKE_HCI_INIT(inst)                                                                        \
	static struct bt_hci_driver_data driver_data_##inst;                                       \
	static const struct bt_hci_driver_config driver_config_##inst =                            \
		BT_DT_HCI_DRIVER_CONFIG_INST_GET(inst);                                            \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &driver_data_##inst, &driver_config_##inst,        \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &driver_api)

DT_INST_FOREACH_STATUS_OKAY(FAKE_HCI_INIT)

/* ------------------------------------------------------------ test hooks */

void fake_hci_set_public_addr(const bt_addr_t *addr)
{
	if (addr == NULL) {
		memset(&public_addr, 0, sizeof(public_addr));
	} else {
		bt_addr_copy(&public_addr, addr);
	}
}

void fake_hci_get_state(struct fake_hci_state *out)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*out = st;
	k_spin_unlock(&lock, key);
}

void fake_hci_fail(uint16_t opcode, uint8_t status, uint32_t count)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (size_t i = 0; i < ARRAY_SIZE(fails); i++) {
		if (fails[i].count == 0U || fails[i].opcode == opcode) {
			fails[i] = (struct fake_fail){opcode, status, count};
			break;
		}
	}
	k_spin_unlock(&lock, key);
}

void fake_hci_set_auto_connect(bool on, uint32_t delay_ms)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	auto_connect = on;
	auto_connect_delay_ms = delay_ms;
	k_spin_unlock(&lock, key);
}

int fake_hci_complete_create(uint8_t status)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bt_addr_le_t peer;
	int handle = 0;

	if (!st.initiating) {
		k_spin_unlock(&lock, key);
		return -1;
	}
	st.initiating = false;
	bt_addr_le_copy(&peer, &st.init_peer);
	if (status == 0U) {
		handle = conn_new(BT_HCI_ROLE_CENTRAL, &peer);
		if (handle < 0) {
			status = BT_HCI_ERR_CONN_LIMIT_EXCEEDED;
			handle = 0;
		}
	}
	k_spin_unlock(&lock, key);

	queue_conn_complete(status, (uint16_t)handle, BT_HCI_ROLE_CENTRAL, &peer);
	return status == 0U ? handle : -1;
}

int fake_hci_incoming_conn(const bt_addr_le_t *peer)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	int handle;

	if (!st.adv_enabled || (st.adv_type != BT_HCI_ADV_IND &&
				st.adv_type != BT_HCI_ADV_DIRECT_IND)) {
		k_spin_unlock(&lock, key);
		return -1;
	}
	handle = conn_new(BT_HCI_ROLE_PERIPHERAL, peer);
	if (handle >= 0) {
		st.adv_enabled = false; /* connectable advertising ends */
	}
	k_spin_unlock(&lock, key);

	if (handle >= 0) {
		queue_conn_complete(0U, (uint16_t)handle, BT_HCI_ROLE_PERIPHERAL, peer);
	}
	return handle;
}

void fake_hci_remote_disconnect(uint16_t handle, uint8_t reason)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	struct fake_conn *fc = conn_find(handle);

	if (fc != NULL) {
		fc->in_use = false;
	}
	k_spin_unlock(&lock, key);

	if (fc != NULL) {
		queue_disconn_complete(handle, reason);
	}
}

bool fake_hci_adv_report(const bt_addr_le_t *addr, uint8_t evt_type, const uint8_t *ad,
			 uint8_t ad_len, int8_t rssi)
{
	struct bt_hci_evt_le_advertising_info *info;
	struct net_buf *buf;
	uint8_t *p;
	bool scanning;

	k_spinlock_key_t key = k_spin_lock(&lock);

	scanning = st.scan_enabled;
	k_spin_unlock(&lock, key);
	if (!scanning) {
		return false;
	}

	p = le_meta_new(&buf, BT_HCI_EVT_LE_ADVERTISING_REPORT,
			1U + sizeof(*info) + ad_len + 1U);
	p[0] = 1U; /* num_reports */
	info = (void *)&p[1];
	info->evt_type = evt_type;
	bt_addr_le_copy(&info->addr, addr);
	info->length = ad_len;
	memcpy(info->data, ad, ad_len);
	info->data[ad_len] = (uint8_t)rssi;
	evt_queue(buf);
	return true;
}

void fake_hci_set_att_silent(bool silent)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	att_silent = silent;
	k_spin_unlock(&lock, key);
}
