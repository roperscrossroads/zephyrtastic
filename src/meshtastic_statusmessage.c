/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_statusmessage.h -- and read it first: this module is the
 * template for the rest of the admin/config sprint's modules. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/module_config.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_modules.h"
#include "meshtastic_statusmessage.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define STATUS_MAX (sizeof(((meshtastic_StatusMessage *)0)->status))

BUILD_ASSERT(sizeof(((meshtastic_ModuleConfig_StatusMessageConfig *)0)->node_status) ==
		     STATUS_MAX,
	     "the stored status and the on-air status must be the same width");

/* ---- own status ----------------------------------------------------------- */

size_t meshtastic_statusmessage_get(char *buf, size_t cap)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	const char *src = "";

	if (buf == NULL || cap == 0U) {
		return 0U;
	}

	/* Read from the store every time, never a cached copy: what the admin
	 * channel persisted is the status, full stop. */
	if (meshtastic_config_store_get_module(meshtastic_ModuleConfig_statusmessage_tag, &mod) ==
	    0) {
		src = mod.payload_variant.statusmessage.node_status;
	}

	strncpy(buf, src, cap - 1U);
	buf[cap - 1U] = '\0';
	return strlen(buf);
}

static bool status_is_set(void)
{
	char status[STATUS_MAX];

	return meshtastic_statusmessage_get(status, sizeof(status)) != 0U;
}

int meshtastic_statusmessage_send(void)
{
	meshtastic_StatusMessage msg = meshtastic_StatusMessage_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_packet pkt = {0};
	pb_ostream_t stream;
	int ret;

	if (meshtastic_statusmessage_get(msg.status, sizeof(msg.status)) == 0U) {
		return -ENODATA;
	}

	stream = pb_ostream_from_buffer(payload, sizeof(payload));
	if (!pb_encode(&stream, meshtastic_StatusMessage_fields, &msg)) {
		LOG_ERR("StatusMessage encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	/* Reference: broadcast, primary channel, priority BACKGROUND, no
	 * want_response. The BG tier comes from the port number
	 * (meshtastic_sched_tier_for); K_NO_WAIT makes it droppable under
	 * congestion, which is what puts it behind the airtime gate. */
	pkt.to = MESHTASTIC_NODE_BROADCAST;
	pkt.portnum = MESHTASTIC_PORT_NODE_STATUS;
	pkt.payload = payload;
	pkt.payload_len = stream.bytes_written;

	ret = meshtastic_send_packet(&pkt, K_NO_WAIT);
	if (ret == 0) {
		LOG_INF("Status announced: \"%s\"", msg.status);
	} else {
		LOG_WRN("Status announce not queued (%d)", ret);
	}
	return ret;
}

/* ---- unprompted announce ---------------------------------------------------- */

#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)
static struct k_work_delayable announce_work;

static void announce_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* A status cleared between arming and firing: the cancel in
	 * config_changed() normally wins, but the work may already be running. */
	if (meshtastic_statusmessage_send() == -ENODATA) {
		return;
	}
	(void)k_work_reschedule(&announce_work,
				K_SECONDS(CONFIG_MESHTASTIC_STATUSMESSAGE_INTERVAL_SEC));
}
#endif

void meshtastic_statusmessage_config_changed(void)
{
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)
	if (status_is_set()) {
		/* Re-armed from the change, not from boot: an app user editing
		 * the text gets one announce when they stop, and a status set at
		 * runtime does not wait for the running interval (the reference's
		 * own TODO). */
		(void)k_work_reschedule(&announce_work,
					K_SECONDS(CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC));
		LOG_DBG("Status set; announce in %d s",
			CONFIG_MESHTASTIC_STATUSMESSAGE_START_DELAY_SEC);
	} else {
		(void)k_work_cancel_delayable(&announce_work);
		LOG_DBG("Status cleared; announce cancelled");
	}
#else
	LOG_DBG("Status %s (auto-send off)", status_is_set() ? "set" : "cleared");
#endif
}

int meshtastic_statusmessage_set(const char *status)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	char *dst = mod.payload_variant.statusmessage.node_status;
	int ret;

	mod.which_payload_variant = meshtastic_ModuleConfig_statusmessage_tag;
	if (status != NULL) {
		strncpy(dst, status, STATUS_MAX - 1U);
		dst[STATUS_MAX - 1U] = '\0';
	}

	ret = meshtastic_config_store_set_module(&mod);
	if (ret < 0) {
		return ret;
	}

	meshtastic_statusmessage_config_changed();
	return 0;
}

/* ---- peers ------------------------------------------------------------------ */

struct status_peer {
	uint32_t node;      /* 0: empty slot */
	int64_t heard_ms;
	char status[STATUS_MAX];
};

static K_MUTEX_DEFINE(peers_lock);
static struct status_peer peers[CONFIG_MESHTASTIC_STATUSMESSAGE_CACHE_SIZE];

/* Same shape as nodeinfo's peer_get_locked: the node's own slot, else an empty
 * one, else evict the least recently heard. */
static struct status_peer *peer_slot_locked(uint32_t node)
{
	struct status_peer *oldest = &peers[0];

	for (size_t i = 0; i < ARRAY_SIZE(peers); i++) {
		if (peers[i].node == node) {
			return &peers[i];
		}
		if (peers[i].node == 0U) {
			return &peers[i];
		}
		if (peers[i].heard_ms < oldest->heard_ms) {
			oldest = &peers[i];
		}
	}
	return oldest;
}

static void peer_remember(uint32_t node, const char *status, int64_t now_ms)
{
	struct status_peer *slot;

	k_mutex_lock(&peers_lock, K_FOREVER);
	slot = peer_slot_locked(node);
	slot->node = node;
	slot->heard_ms = now_ms;
	strncpy(slot->status, status, sizeof(slot->status) - 1U);
	slot->status[sizeof(slot->status) - 1U] = '\0';
	k_mutex_unlock(&peers_lock);
}

bool meshtastic_statusmessage_peer_get(uint32_t node, char *buf, size_t cap)
{
	bool found = false;

	if (node == 0U || buf == NULL || cap == 0U) {
		return false;
	}

	k_mutex_lock(&peers_lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(peers); i++) {
		if (peers[i].node == node) {
			strncpy(buf, peers[i].status, cap - 1U);
			buf[cap - 1U] = '\0';
			found = true;
			break;
		}
	}
	k_mutex_unlock(&peers_lock);

	return found;
}

bool meshtastic_statusmessage_peer_at(size_t index, uint32_t *node, char *buf, size_t cap,
				      int64_t *age_ms)
{
	bool present;

	if (index >= ARRAY_SIZE(peers) || node == NULL || buf == NULL || cap == 0U) {
		return false;
	}

	k_mutex_lock(&peers_lock, K_FOREVER);
	present = peers[index].node != 0U;
	if (present) {
		*node = peers[index].node;
		strncpy(buf, peers[index].status, cap - 1U);
		buf[cap - 1U] = '\0';
		if (age_ms != NULL) {
			*age_ms = k_uptime_get() - peers[index].heard_ms;
		}
	}
	k_mutex_unlock(&peers_lock);

	return present;
}

void meshtastic_statusmessage_reset(void)
{
	k_mutex_lock(&peers_lock, K_FOREVER);
	memset(peers, 0, sizeof(peers));
	k_mutex_unlock(&peers_lock);
}

static void statusmessage_on_packet(const struct meshtastic_packet *packet,
				    const meshtastic_MeshPacket *mesh)
{
	meshtastic_StatusMessage msg = meshtastic_StatusMessage_init_zero;
	pb_istream_t stream;
	uint32_t from;
	const uint8_t *payload;
	size_t payload_len;

	if (packet == NULL) {
		return;
	}

	/* Dual-rep read (see meshtastic_modules.h): the decoded MeshPacket when
	 * the RF path supplied one, else the flat struct. */
	from = mesh ? mesh->from : packet->from;
	payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;

	if (from == 0U || from == meshtastic_get_node_id() || payload == NULL ||
	    payload_len == 0U) {
		return;
	}

	stream = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&stream, meshtastic_StatusMessage_fields, &msg)) {
		LOG_DBG("StatusMessage from 0x%08x undecodable: %s", from, PB_GET_ERROR(&stream));
		return;
	}

	LOG_INF("Status from 0x%08x: \"%s\"", from, msg.status);
	peer_remember(from, msg.status, k_uptime_get());
}

MESHTASTIC_MODULE_DEFINE(statusmessage, MESHTASTIC_PORT_NODE_STATUS, 0, statusmessage_on_packet,
			 NULL);

int meshtastic_statusmessage_init(void)
{
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE_AUTO_SEND)
	k_work_init_delayable(&announce_work, announce_work_fn);
#endif
	/* Boot is a "change" from nothing: arms the first announce if a status
	 * was persisted, exactly as a runtime set would. */
	meshtastic_statusmessage_config_changed();
	return 0;
}
