/* SPDX-License-Identifier: GPL-3.0
 *
 * Remote-admin client (agents-xhli.3) — see meshtastic_admin_client.h for the
 * contract. Deliberately small: one pending request, one cached passkey, and
 * every frame is built by the same machinery the rest of the stack uses
 * (meshtastic_send_packet → PKC for a unicast ADMIN_APP → whatever bearer the
 * TX path picks).
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic/admin.pb.h"

#include "meshtastic_admin_client.h"
#include "meshtastic_admin_session.h"
#include "meshtastic_core.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* A cached passkey older than this is not worth trying: the target reissues at
 * 150 s and refuses at 300 s; staying well inside the window means a set that
 * follows a get never races the expiry. */
#define CLIENT_PASSKEY_MAX_AGE_MS (240 * MSEC_PER_SEC)

static K_MUTEX_DEFINE(client_lock);

static struct {
	uint32_t target;   /* node of the pending/last request */
	uint32_t req_id;   /* our request's packet id (response request_id) */
	bool have_passkey; /* passkey[] holds a key for `target` */
	uint8_t passkey[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	int64_t passkey_ms; /* k_uptime when the passkey arrived */
} client;

static int client_send(uint32_t node, meshtastic_AdminMessage *msg, bool want_response,
		       bool want_ack)
{
	uint8_t buf[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};
	int ret;

	if (node == 0U || node == MESHTASTIC_NODE_BROADCAST || node == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	if (!pb_encode(&stream, meshtastic_AdminMessage_fields, msg)) {
		LOG_ERR("admin client: encode failed (variant %u): %s",
			(unsigned int)msg->which_payload_variant, PB_GET_ERROR(&stream));
		return -EINVAL;
	}

	pkt.portnum = meshtastic_PortNum_ADMIN_APP;
	pkt.from = meshtastic_get_node_id();
	pkt.to = node;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.channel_index = 0U;
	pkt.payload = buf;
	pkt.payload_len = stream.bytes_written;
	pkt.want_response = want_response;
	pkt.want_ack = want_ack;

	/* Track before the send: on a shared-medium bearer the response can
	 * arrive before meshtastic_send_packet() returns. */
	k_mutex_lock(&client_lock, K_FOREVER);
	if (client.target != node) {
		client.have_passkey = false; /* passkeys are per-node */
	}
	client.target = node;
	client.req_id = pkt.id;
	k_mutex_unlock(&client_lock);

	ret = meshtastic_send_packet(&pkt, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN("admin client: send to 0x%08x failed (%d)%s", node, ret,
			ret == -EACCES ? " — no public key for the target in the NodeDB" : "");
	} else {
		LOG_INF("admin client: variant %u -> 0x%08x id=0x%08x",
			(unsigned int)msg->which_payload_variant, node, pkt.id);
	}
	return ret;
}

int meshtastic_admin_client_get_owner(uint32_t node)
{
	meshtastic_AdminMessage msg = meshtastic_AdminMessage_init_zero;

	msg.which_payload_variant = meshtastic_AdminMessage_get_owner_request_tag;
	msg.payload_variant.get_owner_request = true;
	return client_send(node, &msg, true, false);
}

int meshtastic_admin_client_get_config(uint32_t node, uint32_t config_type)
{
	meshtastic_AdminMessage msg = meshtastic_AdminMessage_init_zero;

	msg.which_payload_variant = meshtastic_AdminMessage_get_config_request_tag;
	msg.payload_variant.get_config_request = (meshtastic_AdminMessage_ConfigType)config_type;
	return client_send(node, &msg, true, false);
}

int meshtastic_admin_client_set_owner(uint32_t node, const char *long_name,
				      const char *short_name)
{
	meshtastic_AdminMessage msg = meshtastic_AdminMessage_init_zero;
	meshtastic_User *user = &msg.payload_variant.set_owner;

	k_mutex_lock(&client_lock, K_FOREVER);
	if (!client.have_passkey || client.target != node ||
	    (k_uptime_get() - client.passkey_ms) > CLIENT_PASSKEY_MAX_AGE_MS) {
		k_mutex_unlock(&client_lock);
		LOG_WRN("admin client: no fresh passkey for 0x%08x — run a get first", node);
		return -EACCES;
	}
	msg.session_passkey.size = MESHTASTIC_ADMIN_SESSION_KEY_LEN;
	memcpy(msg.session_passkey.bytes, client.passkey, MESHTASTIC_ADMIN_SESSION_KEY_LEN);
	k_mutex_unlock(&client_lock);

	msg.which_payload_variant = meshtastic_AdminMessage_set_owner_tag;
	if (long_name != NULL && long_name[0] != '\0') {
		strncpy(user->long_name, long_name, sizeof(user->long_name) - 1U);
	}
	if (short_name != NULL && short_name[0] != '\0') {
		strncpy(user->short_name, short_name, sizeof(user->short_name) - 1U);
	}

	return client_send(node, &msg, false, true);
}

bool meshtastic_admin_client_have_passkey(uint32_t node)
{
	bool ok;

	k_mutex_lock(&client_lock, K_FOREVER);
	ok = client.have_passkey && client.target == node &&
	     (k_uptime_get() - client.passkey_ms) <= CLIENT_PASSKEY_MAX_AGE_MS;
	k_mutex_unlock(&client_lock);
	return ok;
}

bool meshtastic_admin_client_on_admin(uint32_t from, uint32_t request_id, bool pki_encrypted,
				      const uint8_t *payload, size_t payload_len)
{
	meshtastic_AdminMessage msg = meshtastic_AdminMessage_init_zero;
	pb_istream_t stream;

	/* Identity is the wire crypto: only a PKC frame proves `from` held the
	 * matching private key. A plaintext/channel frame never matches a
	 * pending request. */
	if (!pki_encrypted || request_id == 0U) {
		return false;
	}

	k_mutex_lock(&client_lock, K_FOREVER);
	if (from != client.target || request_id != client.req_id) {
		k_mutex_unlock(&client_lock);
		return false;
	}

	stream = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&stream, meshtastic_AdminMessage_fields, &msg)) {
		k_mutex_unlock(&client_lock);
		LOG_WRN("admin client: response decode failed: %s", PB_GET_ERROR(&stream));
		return true; /* it was addressed to our request; don't let the
			      * server path NAK our own correlation id */
	}

	/* Every response carries the target's live session passkey — cache it
	 * for the mutating op that typically follows. */
	if (msg.session_passkey.size == MESHTASTIC_ADMIN_SESSION_KEY_LEN) {
		memcpy(client.passkey, msg.session_passkey.bytes,
		       MESHTASTIC_ADMIN_SESSION_KEY_LEN);
		client.have_passkey = true;
		client.passkey_ms = k_uptime_get();
	}
	k_mutex_unlock(&client_lock);

	switch (msg.which_payload_variant) {
	case meshtastic_AdminMessage_get_owner_response_tag:
		LOG_INF("admin client: 0x%08x owner long=\"%s\" short=\"%s\"", from,
			msg.payload_variant.get_owner_response.long_name,
			msg.payload_variant.get_owner_response.short_name);
		break;
	case meshtastic_AdminMessage_get_config_response_tag:
		LOG_INF("admin client: 0x%08x config response section tag %u", from,
			(unsigned int)msg.payload_variant.get_config_response
				.which_payload_variant);
		break;
	default:
		LOG_INF("admin client: 0x%08x response variant %u", from,
			(unsigned int)msg.which_payload_variant);
		break;
	}
	return true;
}
