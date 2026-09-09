/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_keyverify.h. Reference: firmware/src/modules/KeyVerificationModule.cpp. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include <psa/crypto.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_core.h"
#include "meshtastic_keyverify.h"
#include "meshtastic_modules.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_pki.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Reference: KEY_VERIFICATION_TIMEOUT_SECS / MAX_SESSION_MS / REMOTE_COOLDOWN_MS. */
#define KV_IDLE_TIMEOUT_MS   (60 * MSEC_PER_SEC)
#define KV_MAX_SESSION_MS    (3 * 60 * MSEC_PER_SEC)
#define KV_REMOTE_COOLDOWN_MS (60 * MSEC_PER_SEC)
#define KV_HASH_LEN          32U

static K_MUTEX_DEFINE(kv_lock);
static struct {
	enum meshtastic_keyverify_state state;
	uint32_t remote;
	uint64_t nonce;
	uint32_t security_number;
	int64_t progressed_ms;     /* last protocol advance (idle timeout) */
	int64_t session_started_ms;
	int64_t last_remote_end_ms; /* 0: never */
	bool session_from_remote;
	uint8_t hash1[KV_HASH_LEN];
	uint8_t hash2[KV_HASH_LEN];
} kv;

/* ---- helpers ----------------------------------------------------------------- */

static int sha256(const uint8_t *in, size_t len, uint8_t out[KV_HASH_LEN])
{
	size_t olen = 0U;

	return (psa_hash_compute(PSA_ALG_SHA_256, in, len, out, KV_HASH_LEN, &olen) ==
			PSA_SUCCESS && olen == KV_HASH_LEN)
		       ? 0
		       : -EIO;
}

/* H1 = SHA256(number || nonce || initiator || responder || PK_init || PK_resp),
 * integers little-endian as the reference hashes its raw memory. */
static int compute_hash1(uint32_t number, uint64_t nonce, uint32_t initiator, uint32_t responder,
			 const uint8_t *pk_init, const uint8_t *pk_resp, uint8_t out[KV_HASH_LEN])
{
	uint8_t buf[4 + 8 + 4 + 4 + KV_HASH_LEN + KV_HASH_LEN];
	uint8_t *p = buf;

	sys_put_le32(number, p);
	p += 4;
	sys_put_le64(nonce, p);
	p += 8;
	sys_put_le32(initiator, p);
	p += 4;
	sys_put_le32(responder, p);
	p += 4;
	memcpy(p, pk_init, KV_HASH_LEN);
	p += KV_HASH_LEN;
	memcpy(p, pk_resp, KV_HASH_LEN);
	return sha256(buf, sizeof(buf), out);
}

/* hash2 = SHA256(nonce || H1). */
static int compute_hash2(uint64_t nonce, const uint8_t h1[KV_HASH_LEN], uint8_t out[KV_HASH_LEN])
{
	uint8_t buf[8 + KV_HASH_LEN];

	sys_put_le64(nonce, buf);
	memcpy(buf + 8, h1, KV_HASH_LEN);
	return sha256(buf, sizeof(buf), out);
}

/* Reference generateVerificationCode: 8 characters from hash1, each byte >> 2
 * plus '0', with a space in the middle ("not a standardized base64"). */
static void verification_code(const uint8_t h1[KV_HASH_LEN], char out[10])
{
	for (int i = 0; i < 4; i++) {
		out[i] = (char)((h1[i] >> 2) + 48);
	}
	out[4] = ' ';
	for (int i = 5; i < 9; i++) {
		out[i] = (char)((h1[i] >> 2) + 48);
	}
	out[9] = '\0';
}

static bool nodedb_key(uint32_t node, uint8_t out[KV_HASH_LEN])
{
	return meshtastic_nodedb_copy_pubkey(node, out) == 0;
}

static void remote_long_name(uint32_t node, char *out, size_t cap)
{
	struct meshtastic_nodedb_node n;

	if (meshtastic_nodedb_get(node, &n) == 0 && n.has_user && n.long_name[0] != '\0') {
		strncpy(out, n.long_name, cap - 1U);
		out[cap - 1U] = '\0';
	} else {
		strncpy(out, "Unknown", cap - 1U);
		out[cap - 1U] = '\0';
	}
}

static void notify(meshtastic_ClientNotification *cn)
{
	cn->level = meshtastic_LogRecord_Level_WARNING;
	meshtastic_phoneapi_enqueue_client_notification(cn);
}

static void reset_session_locked(void)
{
	int64_t keep = kv.last_remote_end_ms;
	bool from_remote = kv.session_from_remote;

	memset(&kv, 0, sizeof(kv));
	kv.last_remote_end_ms = from_remote ? k_uptime_get() : keep;
	/* Discard an unverified key learned during this handshake: on reject or
	 * timeout it is never trusted. */
	meshtastic_pki_clear_pending_key();
}

/* Reference updateState: the absolute cap first, then the idle timeout;
 * @p refresh extends the idle deadline (a protocol step just happened). */
static void update_state_locked(bool refresh)
{
	int64_t now = k_uptime_get();

	if (kv.state == MESHTASTIC_KEYVERIFY_IDLE) {
		return;
	}
	if (now - kv.session_started_ms >= KV_MAX_SESSION_MS ||
	    now - kv.progressed_ms >= KV_IDLE_TIMEOUT_MS) {
		LOG_INF("KeyVerify: session with 0x%08x timed out", kv.remote);
		reset_session_locked();
	} else if (refresh) {
		kv.progressed_ms = now;
	}
}

static int send_kv(uint32_t to, uint64_t nonce, const uint8_t *hash1, const uint8_t *hash2,
		   bool no_pkc)
{
	meshtastic_KeyVerification msg = meshtastic_KeyVerification_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));
	struct meshtastic_packet pkt;

	msg.nonce = nonce;
	if (hash1 != NULL) {
		msg.hash1.size = KV_HASH_LEN;
		memcpy(msg.hash1.bytes, hash1, KV_HASH_LEN);
	}
	if (hash2 != NULL) {
		msg.hash2.size = KV_HASH_LEN;
		memcpy(msg.hash2.bytes, hash2, KV_HASH_LEN);
	}
	if (!pb_encode(&os, meshtastic_KeyVerification_fields, &msg)) {
		return -ENOMEM;
	}
	pkt = (struct meshtastic_packet){
		.to = to,
		.portnum = MESHTASTIC_PORT_KEY_VERIFICATION,
		.payload = payload,
		.payload_len = os.bytes_written,
		.want_response = true,
		.no_pkc = no_pkc,
	};
	return meshtastic_send_packet(&pkt, K_NO_WAIT);
}

/* ---- initiator (A) ------------------------------------------------------------ */

int meshtastic_keyverify_start(uint32_t remote_node)
{
	uint8_t our_pk[KV_HASH_LEN];
	uint8_t peer_pk[KV_HASH_LEN];
	bool peer_known;
	int ret;

	if (remote_node == 0U || remote_node == MESHTASTIC_NODE_BROADCAST ||
	    remote_node == meshtastic_get_node_id()) {
		return -EINVAL;
	}
	if (meshtastic_pki_get_public_key(our_pk) != KV_HASH_LEN) {
		return -EACCES;
	}

	k_mutex_lock(&kv_lock, K_FOREVER);
	update_state_locked(false);
	if (kv.state != MESHTASTIC_KEYVERIFY_IDLE) {
		k_mutex_unlock(&kv_lock);
		return -EBUSY;
	}

	/* The nonce binds the handshake: CSPRNG, all 64 bits. */
	if (psa_generate_random((uint8_t *)&kv.nonce, sizeof(kv.nonce)) != PSA_SUCCESS ||
	    kv.nonce == 0U) {
		reset_session_locked();
		k_mutex_unlock(&kv_lock);
		return -EIO;
	}
	kv.remote = remote_node;
	kv.session_started_ms = k_uptime_get();
	kv.progressed_ms = kv.session_started_ms;
	kv.session_from_remote = false;
	kv.state = MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED;

	/* M1: our key rides in hash1 so a peer that lacks it can bootstrap. PKC
	 * only when we already hold the peer's key; else channel-encrypted (the
	 * sanctioned no_pkc case: the payload is the key the peer is learning). */
	peer_known = nodedb_key(remote_node, peer_pk);
	ret = send_kv(remote_node, kv.nonce, our_pk, NULL, !peer_known);
	if (ret < 0) {
		LOG_WRN("KeyVerify: M1 to 0x%08x not sent (%d)", remote_node, ret);
		reset_session_locked();
		k_mutex_unlock(&kv_lock);
		return ret;
	}
	LOG_INF("KeyVerify: started with 0x%08x (%s)", remote_node, peer_known ? "PKC" : "bootstrap");
	k_mutex_unlock(&kv_lock);
	return 0;
}

int meshtastic_keyverify_provide_number(uint64_t nonce, uint32_t number)
{
	uint8_t our_pk[KV_HASH_LEN];
	uint8_t peer_pk[KV_HASH_LEN];
	uint8_t h1[KV_HASH_LEN];
	uint8_t h2[KV_HASH_LEN];
	meshtastic_ClientNotification cn = meshtastic_ClientNotification_init_zero;
	int ret;

	k_mutex_lock(&kv_lock, K_FOREVER);
	update_state_locked(false);
	if (kv.state != MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER || nonce != kv.nonce) {
		k_mutex_unlock(&kv_lock);
		return -EINVAL;
	}
	if (meshtastic_pki_get_public_key(our_pk) != KV_HASH_LEN ||
	    (!nodedb_key(kv.remote, peer_pk) && !meshtastic_pki_get_pending_key(kv.remote, peer_pk))) {
		LOG_WRN("KeyVerify: no key for 0x%08x, aborting", kv.remote);
		reset_session_locked();
		k_mutex_unlock(&kv_lock);
		return -EACCES;
	}

	if (compute_hash1(number, kv.nonce, meshtastic_get_node_id(), kv.remote, our_pk, peer_pk,
			  h1) < 0 ||
	    compute_hash2(kv.nonce, h1, h2) < 0) {
		k_mutex_unlock(&kv_lock);
		return -EIO;
	}
	if (memcmp(h2, kv.hash2, KV_HASH_LEN) != 0) {
		/* Reference: "Hash2 did not match" -- nothing is sent, the session
		 * stays open for another try until it times out. */
		LOG_WRN("KeyVerify: security number does not reproduce the peer's hash2");
		k_mutex_unlock(&kv_lock);
		return -EACCES;
	}

	memcpy(kv.hash1, h1, KV_HASH_LEN);
	kv.security_number = number;
	/* M3: H1 back to the peer, PKC (the pending key serves if the NodeDB
	 * lacks it): proves we hold the private key for PK_A. */
	ret = send_kv(kv.remote, kv.nonce, h1, NULL, false);
	if (ret < 0) {
		LOG_WRN("KeyVerify: M3 to 0x%08x not sent (%d)", kv.remote, ret);
		k_mutex_unlock(&kv_lock);
		return ret;
	}
	kv.state = MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER;
	kv.progressed_ms = k_uptime_get();

	cn.which_payload_variant = meshtastic_ClientNotification_key_verification_final_tag;
	cn.payload_variant.key_verification_final.nonce = kv.nonce;
	cn.payload_variant.key_verification_final.isSender = true;
	remote_long_name(kv.remote, cn.payload_variant.key_verification_final.remote_longname,
			 sizeof(cn.payload_variant.key_verification_final.remote_longname));
	verification_code(h1, cn.payload_variant.key_verification_final.verification_characters);
	(void)snprintk(cn.message, sizeof(cn.message),
		       "Final confirmation for outgoing manual key verification %s",
		       cn.payload_variant.key_verification_final.verification_characters);
	notify(&cn);
	LOG_INF("KeyVerify: code %s -- confirm it matches the peer's",
		cn.payload_variant.key_verification_final.verification_characters);
	k_mutex_unlock(&kv_lock);
	return 0;
}

/* ---- both sides ------------------------------------------------------------------ */

int meshtastic_keyverify_accept(uint64_t nonce)
{
	uint8_t pending[KV_HASH_LEN];
	uint8_t key[KV_HASH_LEN];
	uint32_t remote;
	int ret;

	k_mutex_lock(&kv_lock, K_FOREVER);
	update_state_locked(false);
	if ((kv.state != MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER &&
	     kv.state != MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER) ||
	    nonce != kv.nonce) {
		k_mutex_unlock(&kv_lock);
		return -EINVAL;
	}
	remote = kv.remote;

	/* Reference commitVerifiedRemoteNode: a key only held as pending is
	 * committed now that the user has confirmed; then the flag. */
	if (nodedb_key(remote, key)) {
		ret = meshtastic_nodedb_commit_pubkey(remote, key);
	} else if (meshtastic_pki_get_pending_key(remote, pending)) {
		ret = meshtastic_nodedb_commit_pubkey(remote, pending);
	} else {
		ret = -EACCES;
	}
	if (ret == 0) {
		ret = meshtastic_nodedb_set_key_verified(remote, true);
	}
	if (ret < 0) {
		LOG_WRN("KeyVerify: commit for 0x%08x failed (%d)", remote, ret);
		k_mutex_unlock(&kv_lock);
		return ret;
	}
	LOG_INF("KeyVerify: 0x%08x manually verified (security number %u)", remote,
		kv.security_number);
	reset_session_locked();
	k_mutex_unlock(&kv_lock);

	/* Reference: tell the peer who we are, now that it trusts our key. */
	(void)meshtastic_send_node_info(remote);
	return 0;
}

void meshtastic_keyverify_reject(void)
{
	k_mutex_lock(&kv_lock, K_FOREVER);
	if (kv.state != MESHTASTIC_KEYVERIFY_IDLE) {
		LOG_INF("KeyVerify: session with 0x%08x rejected", kv.remote);
	}
	reset_session_locked();
	k_mutex_unlock(&kv_lock);
}

void meshtastic_keyverify_status(struct meshtastic_keyverify_status *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&kv_lock, K_FOREVER);
	update_state_locked(false);
	memset(out, 0, sizeof(*out));
	out->state = kv.state;
	out->remote_node = kv.remote;
	out->nonce = kv.nonce;
	out->security_number = kv.security_number;
	if (kv.state == MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER ||
	    kv.state == MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER ||
	    kv.state == MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1) {
		verification_code(kv.hash1, out->code);
	}
	k_mutex_unlock(&kv_lock);
}

void meshtastic_keyverify_reset(void)
{
	k_mutex_lock(&kv_lock, K_FOREVER);
	memset(&kv, 0, sizeof(kv));
	meshtastic_pki_clear_pending_key();
	k_mutex_unlock(&kv_lock);
}

/* ---- receiving -------------------------------------------------------------------- */

static bool decode_kv(const struct meshtastic_packet *packet, const meshtastic_MeshPacket *mesh,
		      meshtastic_KeyVerification *out)
{
	const uint8_t *payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	size_t len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	pb_istream_t is = pb_istream_from_buffer(payload, len);

	*out = (meshtastic_KeyVerification)meshtastic_KeyVerification_init_zero;
	return payload != NULL && pb_decode(&is, meshtastic_KeyVerification_fields, out);
}

/* M2 (initiator side) and M3 (responder side). Reference handleReceivedProtobuf. */
static void keyverify_on_packet(const struct meshtastic_packet *packet,
				const meshtastic_MeshPacket *mesh)
{
	meshtastic_KeyVerification r;
	uint32_t from;
	uint32_t to;
	bool pki;

	if (packet == NULL) {
		return;
	}
	from = mesh ? mesh->from : packet->from;
	to = mesh ? mesh->to : packet->to;
	pki = mesh ? mesh->pki_encrypted : packet->pki_encrypted;
	if (to != meshtastic_get_node_id() || from == 0U || from == meshtastic_get_node_id()) {
		return;
	}

	k_mutex_lock(&kv_lock, K_FOREVER);
	/* No refresh: this runs before the sender is checked, so any node could
	 * otherwise hold the session open. */
	update_state_locked(false);
	if (kv.state == MESHTASTIC_KEYVERIFY_IDLE || from != kv.remote || !decode_kv(packet, mesh, &r) ||
	    r.nonce != kv.nonce) {
		k_mutex_unlock(&kv_lock);
		return;
	}

	if (kv.state == MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED && r.hash2.size == KV_HASH_LEN &&
	    r.hash1.size == KV_HASH_LEN) {
		meshtastic_ClientNotification cn = meshtastic_ClientNotification_init_zero;
		uint8_t known[KV_HASH_LEN];

		/* M2: hash2 to check the number against later; the responder's key in
		 * hash1, held as pending until the user accepts. */
		memcpy(kv.hash2, r.hash2.bytes, KV_HASH_LEN);
		if (!nodedb_key(from, known)) {
			meshtastic_pki_set_pending_key(from, r.hash1.bytes);
		}
		kv.state = MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER;
		kv.progressed_ms = k_uptime_get();

		cn.which_payload_variant = meshtastic_ClientNotification_key_verification_number_request_tag;
		cn.payload_variant.key_verification_number_request.nonce = kv.nonce;
		remote_long_name(from, cn.payload_variant.key_verification_number_request.remote_longname,
				 sizeof(cn.payload_variant.key_verification_number_request.remote_longname));
		(void)snprintk(cn.message, sizeof(cn.message),
			       "Enter Security Number for Key Verification");
		notify(&cn);
		LOG_INF("KeyVerify: 0x%08x answered; enter the security number it displays", from);
	} else if (kv.state == MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1 && pki &&
		   r.hash1.size == KV_HASH_LEN) {
		/* M3: must be PKC (proves the initiator holds its private key), and
		 * must carry OUR H1. */
		if (memcmp(kv.hash1, r.hash1.bytes, KV_HASH_LEN) == 0) {
			meshtastic_ClientNotification cn = meshtastic_ClientNotification_init_zero;

			kv.state = MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER;
			kv.progressed_ms = k_uptime_get();
			cn.which_payload_variant = meshtastic_ClientNotification_key_verification_final_tag;
			cn.payload_variant.key_verification_final.nonce = kv.nonce;
			cn.payload_variant.key_verification_final.isSender = false;
			remote_long_name(from, cn.payload_variant.key_verification_final.remote_longname,
					 sizeof(cn.payload_variant.key_verification_final.remote_longname));
			verification_code(kv.hash1,
					  cn.payload_variant.key_verification_final.verification_characters);
			(void)snprintk(cn.message, sizeof(cn.message),
				       "Final confirmation for incoming manual key verification %s",
				       cn.payload_variant.key_verification_final.verification_characters);
			notify(&cn);
			LOG_INF("KeyVerify: hash1 from 0x%08x matches; code %s", from,
				cn.payload_variant.key_verification_final.verification_characters);
		} else {
			LOG_WRN("KeyVerify: hash1 from 0x%08x does not match", from);
		}
	}
	k_mutex_unlock(&kv_lock);
}

/* M1 -> M2. Reference allocReply: the dispatcher only calls this for a
 * want_response unicast to us on this port. */
static int keyverify_alloc_reply(const struct meshtastic_packet *req,
				 const meshtastic_MeshPacket *mesh, struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_KeyVerification r;
	meshtastic_KeyVerification resp = meshtastic_KeyVerification_init_zero;
	meshtastic_ClientNotification cn = meshtastic_ClientNotification_init_zero;
	uint8_t our_pk[KV_HASH_LEN];
	uint8_t sender_pk[KV_HASH_LEN];
	bool sender_in_nodedb;
	uint32_t entropy = 0U;
	uint32_t from;
	pb_ostream_t os;
	int64_t now;

	if (req == NULL || reply == NULL) {
		return -EINVAL;
	}
	from = mesh ? mesh->from : req->from;
	if (from == 0U || from == meshtastic_get_node_id() || !decode_kv(req, mesh, &r)) {
		return -ENOENT;
	}
	/* Only an INIT (no hash2) opens a session; M3 carries hash1 alone and is
	 * handled by on_packet. */
	if (r.hash2.size != 0U || r.hash1.size != KV_HASH_LEN) {
		return -ENOENT;
	}
	if (meshtastic_pki_get_public_key(our_pk) != KV_HASH_LEN) {
		return -ENOENT;
	}

	k_mutex_lock(&kv_lock, K_FOREVER);
	update_state_locked(false);
	now = k_uptime_get();
	if (kv.state != MESHTASTIC_KEYVERIFY_IDLE) {
		LOG_WRN("KeyVerify: request from 0x%08x while a session is open", from);
		k_mutex_unlock(&kv_lock);
		return -ENOENT;
	}
	if (kv.last_remote_end_ms != 0 && (now - kv.last_remote_end_ms) < KV_REMOTE_COOLDOWN_MS) {
		LOG_WRN("KeyVerify: request from 0x%08x within the cooldown", from);
		k_mutex_unlock(&kv_lock);
		return -ENOENT;
	}

	/* The requester's key: from the NodeDB, else the one it carried (bootstrap),
	 * held as pending. */
	sender_in_nodedb = nodedb_key(from, sender_pk);
	if (!sender_in_nodedb) {
		memcpy(sender_pk, r.hash1.bytes, KV_HASH_LEN);
		meshtastic_pki_set_pending_key(from, sender_pk);
	}

	/* The security number is the handshake's MitM-resistance entropy: CSPRNG. */
	if (psa_generate_random((uint8_t *)&entropy, sizeof(entropy)) != PSA_SUCCESS) {
		meshtastic_pki_clear_pending_key();
		k_mutex_unlock(&kv_lock);
		return -EIO;
	}
	kv.security_number = (entropy % 999999U) + 1U;
	kv.nonce = r.nonce;
	kv.remote = from;
	kv.session_started_ms = now;
	kv.progressed_ms = now;
	kv.session_from_remote = true;

	if (compute_hash1(kv.security_number, kv.nonce, from, meshtastic_get_node_id(), sender_pk,
			  our_pk, kv.hash1) < 0 ||
	    compute_hash2(kv.nonce, kv.hash1, kv.hash2) < 0) {
		reset_session_locked();
		k_mutex_unlock(&kv_lock);
		return -EIO;
	}
	kv.state = MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1;

	/* M2: our key in hash1 (so the requester can bootstrap ours), hash2. PKC
	 * only if we already held the requester's key; in the bootstrap case it goes
	 * channel-encrypted so a requester that lacks our key can read it. */
	resp.nonce = kv.nonce;
	resp.hash1.size = KV_HASH_LEN;
	memcpy(resp.hash1.bytes, our_pk, KV_HASH_LEN);
	resp.hash2.size = KV_HASH_LEN;
	memcpy(resp.hash2.bytes, kv.hash2, KV_HASH_LEN);
	os = pb_ostream_from_buffer(payload, sizeof(payload));
	if (!pb_encode(&os, meshtastic_KeyVerification_fields, &resp)) {
		reset_session_locked();
		k_mutex_unlock(&kv_lock);
		return -ENOMEM;
	}
	*reply = (struct meshtastic_packet){
		.to = from,
		.portnum = MESHTASTIC_PORT_KEY_VERIFICATION,
		.payload = payload,
		.payload_len = os.bytes_written,
		.request_id = req->id,
		.no_pkc = !sender_in_nodedb,
	};

	cn.which_payload_variant = meshtastic_ClientNotification_key_verification_number_inform_tag;
	cn.payload_variant.key_verification_number_inform.nonce = kv.nonce;
	cn.payload_variant.key_verification_number_inform.security_number = kv.security_number;
	remote_long_name(from, cn.payload_variant.key_verification_number_inform.remote_longname,
			 sizeof(cn.payload_variant.key_verification_number_inform.remote_longname));
	(void)snprintk(cn.message, sizeof(cn.message),
		       "Incoming Key Verification.\nSecurity Number\n%03u %03u",
		       kv.security_number / 1000U, kv.security_number % 1000U);
	notify(&cn);
	LOG_INF("KeyVerify: request from 0x%08x (%s); security number %03u %03u", from,
		sender_in_nodedb ? "PKC" : "bootstrap", kv.security_number / 1000U,
		kv.security_number % 1000U);
	k_mutex_unlock(&kv_lock);
	return 0;
}

MESHTASTIC_MODULE_DEFINE(keyverify, MESHTASTIC_PORT_KEY_VERIFICATION, 0, keyverify_on_packet,
			 keyverify_alloc_reply);

int meshtastic_keyverify_init(void)
{
	meshtastic_keyverify_reset();
	return 0;
}
