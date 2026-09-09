/* SPDX-License-Identifier: GPL-3.0
 *
 * PKI-enabled remote-admin tests: exercise the REAL X25519 + AES-CCM PKC path
 * end to end. A peer PKC-encrypts an AdminMessage to us; the stack decrypts it
 * (setting pki_encrypted + recovering the sender key), and remote admin is
 * authorized only when that key is in SecurityConfig.admin_key.
 *
 * Forging the "peer -> us" ciphertext uses ECDH symmetry: the shared secret
 * X25519(peer_priv, our_pub) == X25519(our_priv, peer_pub), so our own
 * meshtastic_pki_encrypt(to=PEER, from=PEER, ...) — which keys off our private
 * key and the peer's public key in the NodeDB, with the nonce bound to from=PEER
 * — produces a frame our RX path decrypts exactly as if the peer had sent it.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <psa/crypto.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic_admin.h"
#include "meshtastic_admin_session.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_mqtt_config.h"
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
#include "meshtastic_statusmessage.h"
#endif
#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
#include "meshtastic_neighborinfo.h"
#endif
#if defined(CONFIG_MESHTASTIC_MESHBEACON)
#include "meshtastic_meshbeacon.h"
#endif
#if defined(CONFIG_MESHTASTIC_TRAFFIC)
#include "meshtastic_traffic.h"
#endif
#if defined(CONFIG_MESHTASTIC_EXTNOTIFY)
#include "meshtastic_extnotify.h"
#endif
#include "meshtastic_preset.h"
#include "meshtastic_admin_client.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_packet.h"
#include "meshtastic_pki.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"
#include "vectors/meshtastic_vectors.h"

#define TEST_NODE_ID 0x12345678U
#define PEER_NODE_ID 0x87654321U

/* ---- Minimal mock LoRa driver (send counter + RX injection) --------------- */

struct mock_lora_state {
	struct k_mutex lock;
	lora_recv_cb rx_cb;
	void *rx_user_data;
	uint32_t send_count;
	uint8_t last_tx[MESHTASTIC_PKT_MAX];
	uint32_t last_tx_len;
	/* The last few frames, newest at (ring_head - 1): a reply and the NodeInfo
	 * request the same inbound frame triggers leave the queue in whichever
	 * order the queue chooses, so "the last frame" is not enough to find one. */
#define MOCK_TX_RING 4U
	uint8_t ring[MOCK_TX_RING][MESHTASTIC_PKT_MAX];
	uint32_t ring_len[MOCK_TX_RING];
	uint32_t ring_head;
};

static struct mock_lora_state mock_lora;

static int mock_lora_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_mutex_init(&mock_lora.lock);
	return 0;
}

static int mock_lora_config(const struct device *dev, const struct lora_modem_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);
	return 0;
}

static uint32_t mock_lora_airtime(const struct device *dev, uint32_t data_len)
{
	ARG_UNUSED(dev);
	return data_len;
}

static int mock_lora_send(const struct device *dev, uint8_t *data, uint32_t data_len)
{
	ARG_UNUSED(dev);
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	if (data_len <= sizeof(mock_lora.last_tx)) {
		uint32_t slot = mock_lora.ring_head % MOCK_TX_RING;

		memcpy(mock_lora.last_tx, data, data_len);
		mock_lora.last_tx_len = data_len;
		memcpy(mock_lora.ring[slot], data, data_len);
		mock_lora.ring_len[slot] = data_len;
		mock_lora.ring_head++;
	}
	mock_lora.send_count++;
	k_mutex_unlock(&mock_lora.lock);
	return 0;
}

static int mock_lora_send_async(const struct device *dev, uint8_t *data, uint32_t data_len,
				struct k_poll_signal *async)
{
	int ret = mock_lora_send(dev, data, data_len);

	if (async != NULL) {
		k_poll_signal_raise(async, ret);
	}
	return ret;
}

static int mock_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
			  k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	return -ENOTSUP;
}

static int mock_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	ARG_UNUSED(dev);
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.rx_cb = cb;
	mock_lora.rx_user_data = user_data;
	k_mutex_unlock(&mock_lora.lock);
	return 0;
}

static DEVICE_API(lora, mock_lora_api) = {
	.config = mock_lora_config,
	.airtime = mock_lora_airtime,
	.send = mock_lora_send,
	.send_async = mock_lora_send_async,
	.recv = mock_lora_recv,
	.recv_async = mock_lora_recv_async,
};

DEVICE_DEFINE(mock_lora, "mock_lora", mock_lora_init, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mock_lora_api);

static const struct device *const lora_dev = DEVICE_GET(mock_lora);

/* ---- Test fixture --------------------------------------------------------- */

static uint8_t peer_pubkey[MESHTASTIC_PKI_KEY_LEN];

/* A registered PhoneAPI transport so LOCAL admin replies (which go straight to
 * meshtastic_phoneapi_on_packet(), never onto the mock LoRa device — see
 * admin_emit_reply()'s !admin_cur.remote branch) have somewhere to land. Local
 * admin requests need no PKC/passkey at all (meshtastic_admin_handle_local()
 * is the phone's trusted entry point), so this is the simplest way to test
 * any getter or app-originated op — dramatically less setup than inject_pkc_admin(). */
#define PHONE_Q_SIZE 4U
static struct meshtastic_phoneapi_frame phone_q_storage[PHONE_Q_SIZE];
static struct meshtastic_phoneapi phone_api;
static meshtastic_ToRadio phone_to_scratch;
static meshtastic_FromRadio phone_from_scratch;

static void inject_rx_frame(const uint8_t *wire, uint32_t wire_len)
{
	uint8_t frame[MESHTASTIC_PKT_MAX];
	lora_recv_cb cb;
	void *user_data;

	zassert_true(wire_len <= sizeof(frame), "rx len %u too large", wire_len);
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	cb = mock_lora.rx_cb;
	user_data = mock_lora.rx_user_data;
	k_mutex_unlock(&mock_lora.lock);
	zassert_not_null(cb, "rx callback not armed");

	memcpy(frame, wire, wire_len);
	cb(lora_dev, frame, wire_len, -20, 4, user_data);
}

/* Generate a valid X25519 keypair and return its public key. */
static void gen_x25519_pubkey(uint8_t pub[MESHTASTIC_PKI_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	size_t olen = 0;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	zassert_equal(psa_generate_key(&attr, &kid), PSA_SUCCESS, "peer keygen failed");
	zassert_equal(psa_export_public_key(kid, pub, MESHTASTIC_PKI_KEY_LEN, &olen), PSA_SUCCESS,
		      "peer pubkey export failed");
	zassert_equal(olen, (size_t)MESHTASTIC_PKI_KEY_LEN, "unexpected pubkey length");
	(void)psa_destroy_key(kid);
}

/* Seed PEER's public key into the NodeDB via a NodeInfo (apply_user), the way
 * the stack learns a peer's key on the air. Required both for the admin_key
 * match and so PKC decrypt can look up the sender's key. */
static void seed_peer_pubkey(const uint8_t key[MESHTASTIC_PKI_KEY_LEN])
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = PEER_NODE_ID,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	user.public_key.size = MESHTASTIC_PKI_KEY_LEN;
	memcpy(user.public_key.bytes, key, MESHTASTIC_PKI_KEY_LEN);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

static void set_admin_key(const uint8_t *key, size_t len)
{
	meshtastic_Config sec = meshtastic_Config_init_zero;

	sec.which_payload_variant = meshtastic_Config_security_tag;
	if (key != NULL && len > 0U) {
		sec.payload_variant.security.admin_key_count = 1U;
		sec.payload_variant.security.admin_key[0].size = (pb_size_t)len;
		memcpy(sec.payload_variant.security.admin_key[0].bytes, key, len);
	}
	zassert_ok(meshtastic_config_store_set_config(&sec), "set admin_key failed");
}

static void force_device_role(meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_Config dev = meshtastic_Config_init_zero;

	dev.which_payload_variant = meshtastic_Config_device_tag;
	dev.payload_variant.device.role = role;
	zassert_ok(meshtastic_config_store_set_config(&dev), "force device role failed");
}

static meshtastic_Config_DeviceConfig_Role current_role(void)
{
	meshtastic_Config dev;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &dev),
		   "device config read failed");
	return dev.payload_variant.device.role;
}

/* Encode an AdminMessage set_config(device.role) carrying the given passkey. */
static size_t encode_admin_set_role(meshtastic_Config_DeviceConfig_Role role,
				    const uint8_t *passkey, size_t passkey_len, uint8_t *buf,
				    size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_device_tag;
	am.payload_variant.set_config.payload_variant.device.role = role;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Encode an AdminMessage set_config(lora) carrying the given passkey. Changing the
 * modem preset is a LoRa change, which now applies live (agents-k8oe, F-1 fixed
 * 2026-09-01) via meshtastic_config_store_set_config() -> _preset_apply_stored(). */
static size_t encode_admin_set_lora_preset(meshtastic_Config_LoRaConfig_ModemPreset preset,
					   const uint8_t *passkey, size_t passkey_len, uint8_t *buf,
					   size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_Config_LoRaConfig *lora;

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_lora_tag;
	lora = &am.payload_variant.set_config.payload_variant.lora;
	/* A real client always get-modify-sets, so every LoRaConfig on the wire
	 * carries tx_enabled. Built from zero here, so set it explicitly — a
	 * proto3 bool defaults to false, which now genuinely means receive-only. */
	lora->tx_enabled = true;
	lora->use_preset = true;
	lora->modem_preset = preset;
	lora->region = meshtastic_Config_LoRaConfig_RegionCode_US;
	lora->hop_limit = 3;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Encode an AdminMessage set_config(device) carrying node_info_broadcast_secs
 * (agents-t2hb.1) -- proves the admin path reaches this field with NO admin.c
 * changes needed: set_config_tag already routes every Config section through
 * meshtastic_config_store_set_config() generically. */
static size_t encode_admin_set_nodeinfo_interval(uint32_t secs, const uint8_t *passkey,
						 size_t passkey_len, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_device_tag;
	am.payload_variant.set_config.payload_variant.device.node_info_broadcast_secs = secs;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Encode an AdminMessage begin_edit_settings/commit_edit_settings carrying the
 * given passkey. */
static size_t encode_admin_edit_settings(bool commit, const uint8_t *passkey, size_t passkey_len,
					 uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	if (commit) {
		am.which_payload_variant = meshtastic_AdminMessage_commit_edit_settings_tag;
		am.payload_variant.commit_edit_settings = true;
	} else {
		am.which_payload_variant = meshtastic_AdminMessage_begin_edit_settings_tag;
		am.payload_variant.begin_edit_settings = true;
	}
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Encode an AdminMessage factory_reset_config carrying the given passkey. */
static size_t encode_admin_factory_reset_config(const uint8_t *passkey, size_t passkey_len,
						uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_factory_reset_config_tag;
	am.payload_variant.factory_reset_config = 1;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Encode an AdminMessage remove_by_nodenum(target) carrying the given passkey. */
static size_t encode_admin_remove_by_nodenum(uint32_t target, const uint8_t *passkey,
					     size_t passkey_len, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_remove_by_nodenum_tag;
	am.payload_variant.remove_by_nodenum = target;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_get_device_metadata_request(uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_get_device_metadata_request_tag;
	am.payload_variant.get_device_metadata_request = true;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_get_device_connection_status_request(uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_get_device_connection_status_request_tag;
	am.payload_variant.get_device_connection_status_request = true;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_get_module_config(uint32_t module_type, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_get_module_config_request_tag;
	am.payload_variant.get_module_config_request = (meshtastic_AdminMessage_ModuleConfigType)module_type;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_set_module_config_mqtt(bool enabled, const char *address, uint8_t *buf,
						  size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant = meshtastic_ModuleConfig_mqtt_tag;
	am.payload_variant.set_module_config.payload_variant.mqtt.enabled = enabled;
	strncpy(am.payload_variant.set_module_config.payload_variant.mqtt.address, address,
		sizeof(am.payload_variant.set_module_config.payload_variant.mqtt.address) - 1U);
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_remove_ignored_node(uint32_t target, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_remove_ignored_node_tag;
	am.payload_variant.remove_ignored_node = target;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

static size_t encode_admin_reboot_seconds(int32_t seconds, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_reboot_seconds_tag;
	am.payload_variant.reboot_seconds = seconds;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* factory_reset_device is int32 (not bool, unlike factory_reset_config's
 * sibling declaration two lines away in admin.proto -- easy to get wrong by
 * copy-paste, which is exactly why this has its own encoder rather than
 * reusing encode_admin_bool_variant(). */
static size_t encode_admin_factory_reset_device(uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_factory_reset_device_tag;
	am.payload_variant.factory_reset_device = 1;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Build a genuine PKC-encrypted ADMIN_APP wire frame "from PEER to us" and feed
 * it through the LoRa RX path. Uses the ECDH-symmetry trick described up top. */
static void inject_pkc_admin(const uint8_t *admin_bytes, size_t admin_len, uint32_t id)
{
	uint8_t data[MESHTASTIC_MAX_PAYLOAD_LEN];
	size_t data_len = 0;
	uint8_t enc[MESHTASTIC_MAX_PAYLOAD_LEN + MESHTASTIC_PKI_OVERHEAD];
	size_t enc_len = 0;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *hdr = (struct meshtastic_wire_header *)wire;

	/* The PKC plaintext is a Data protobuf (portnum + payload), not the raw
	 * AdminMessage — that is what the RX path decodes after decryption. */
	zassert_ok(meshtastic_encode_data(MESHTASTIC_PORT_ADMIN, admin_bytes, admin_len, data,
					  sizeof(data), &data_len),
		   "Data encode failed");
	zassert_ok(meshtastic_pki_encrypt(PEER_NODE_ID, PEER_NODE_ID, id, data, data_len, enc,
					  sizeof(enc), &enc_len),
		   "PKC encrypt (forged peer frame) failed");

	hdr->dest = sys_cpu_to_le32(TEST_NODE_ID);
	hdr->src = sys_cpu_to_le32(PEER_NODE_ID);
	hdr->id = sys_cpu_to_le32(id);
	/* hop_limit 3, hop_start 3, no want_ack (keeps the RX thread off the
	 * blocking transport-ACK path; the admin apply is what we assert). */
	hdr->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	hdr->channel = 0x00U; /* PKC marker channel-hash */
	hdr->next_hop = 0U;
	hdr->relay_node = 0U;
	memcpy(wire + MESHTASTIC_HDR_LEN, enc, enc_len);

	inject_rx_frame(wire, MESHTASTIC_HDR_LEN + (uint32_t)enc_len);
}

static void *admin_pki_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa_crypto_init failed");
	zassert_true(device_is_ready(lora_dev), "mock lora not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	zassert_true(meshtastic_pki_have_key(), "our X25519 keypair should be ready");

	gen_x25519_pubkey(peer_pubkey);
	seed_peer_pubkey(peer_pubkey);

	meshtastic_phoneapi_init(&phone_api, "test", phone_q_storage, PHONE_Q_SIZE, NULL, NULL,
				 NULL, NULL, &phone_to_scratch, &phone_from_scratch);
	meshtastic_phoneapi_register(&phone_api);
	return NULL;
}

static void admin_pki_before(void *fixture)
{
	ARG_UNUSED(fixture);
	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	mock_lora.send_count = 0U;
	meshtastic_phoneapi_reset(&phone_api);
}

/* Send an AdminMessage as if the locally-connected phone sent it (no PKC, no
 * passkey — meshtastic_admin_handle_local() trusts its caller unconditionally
 * unless the node is_managed), then pop and decode whatever admin_emit_reply()
 * queued back. Returns false if no reply was queued (a getter that failed, or
 * an op that intentionally emits no AdminMessage response). */
static bool send_local_admin_and_pop_reply(const uint8_t *admin_bytes, size_t admin_len,
					   meshtastic_AdminMessage *resp)
{
	meshtastic_MeshPacket pkt = meshtastic_MeshPacket_init_zero;
	struct meshtastic_phoneapi_frame frame;
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
	pb_istream_t is;

	pkt.from = TEST_NODE_ID;
	pkt.id = 0x0AD00001U;
	pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	pkt.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	memcpy(pkt.decoded.payload.bytes, admin_bytes, admin_len);
	pkt.decoded.payload.size = (pb_size_t)admin_len;

	zassert_true(meshtastic_admin_handle_local(&pkt), "admin_handle_local must consume it");

	/* Skip anything already queued that ISN'T our reply -- a caller that just
	 * seeded a peer via inject_rx_frame()/seed_named_peer() also queues that
	 * peer's NodeInfo to the phone (the router forwards RX traffic there too),
	 * ahead of the reply this function is actually looking for. */
	while (meshtastic_phoneapi_pop_frame(&phone_api, &frame)) {
		from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
		is = pb_istream_from_buffer(frame.data, frame.len);
		zassert_true(pb_decode(&is, meshtastic_FromRadio_fields, &from),
			     "FromRadio decode failed");
		if (from.which_payload_variant != meshtastic_FromRadio_packet_tag ||
		    from.packet.decoded.portnum != meshtastic_PortNum_ADMIN_APP) {
			continue;
		}
		is = pb_istream_from_buffer(from.packet.decoded.payload.bytes,
					    from.packet.decoded.payload.size);
		zassert_true(pb_decode(&is, meshtastic_AdminMessage_fields, resp),
			     "AdminMessage decode failed");
		return true;
	}
	return false;
}

ZTEST_SUITE(admin_pki, NULL, admin_pki_setup, admin_pki_before, NULL, NULL);

/* --- DM encryption parity with upstream (build_wire_packet PKC decision) ------
 * PKI is enabled in this suite, so these exercise the real PKC-vs-channel path. */

/* A DM to a peer whose public key we don't have is REFUSED, not silently
 * channel-encrypted (parity: upstream PKI_SEND_FAIL_PUBLIC_KEY) — a private
 * message must never leak to every node on the channel. 0x0BADF00D is never
 * seeded into the NodeDB, so we hold no key for it regardless of test order. */
ZTEST(admin_pki, test_dm_without_peer_key_refused)
{
	zassert_true(meshtastic_pki_have_key(), "our X25519 key must be ready");
	zassert_true(meshtastic_send_text(0x0BADF00DU, "secret") < 0,
		     "a DM to a keyless peer must be refused, not channel-encrypted");
}

/* DM-2 regression: a persisted SecurityConfig whose public_key has desynced from
 * its private_key (a partial NVS write, or an admin set_config that touched one
 * field) is SELF-HEALED on pki_init — the advertised/stored pub is re-derived from
 * the private key, not trusted from NVS. Without this the node advertises a key
 * that does not match its private key, so every peer's PKC DM to it decrypts to
 * garbage and is NAKed NO_CHANNEL forever. Mirrors upstream ensurePkiKeys(). */
ZTEST(admin_pki, test_pki_init_rederives_public_key_from_private)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	uint8_t priv[MESHTASTIC_PKI_KEY_LEN];
	uint8_t correct_pub[MESHTASTIC_PKI_KEY_LEN];
	uint8_t wrong_pub[MESHTASTIC_PKI_KEY_LEN];
	uint8_t got_pub[MESHTASTIC_PKI_KEY_LEN];
	meshtastic_Config sec = meshtastic_Config_init_zero;
	meshtastic_Config saved;
	bool had_saved;
	size_t olen = 0;

	/* Preserve the suite's real keypair so this test is order-independent. */
	had_saved = (meshtastic_config_store_get_config(meshtastic_Config_security_tag, &saved) == 0);

	/* A genuine keypair: we hold both the private scalar and its true public key. */
	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	zassert_equal(psa_generate_key(&attr, &kid), PSA_SUCCESS, "keygen failed");
	zassert_equal(psa_export_key(kid, priv, sizeof(priv), &olen), PSA_SUCCESS, "priv export");
	zassert_equal(psa_export_public_key(kid, correct_pub, sizeof(correct_pub), &olen),
		      PSA_SUCCESS, "pub export");
	(void)psa_destroy_key(kid);

	/* A DIFFERENT valid pubkey stands in for the desynced/stored one. */
	gen_x25519_pubkey(wrong_pub);
	zassert_true(memcmp(wrong_pub, correct_pub, sizeof(correct_pub)) != 0, "keys collided");

	/* Persist priv + WRONG pub, as a corrupted/partial NVS write would leave it. */
	sec.which_payload_variant = meshtastic_Config_security_tag;
	sec.payload_variant.security.private_key.size = MESHTASTIC_PKI_KEY_LEN;
	memcpy(sec.payload_variant.security.private_key.bytes, priv, MESHTASTIC_PKI_KEY_LEN);
	sec.payload_variant.security.public_key.size = MESHTASTIC_PKI_KEY_LEN;
	memcpy(sec.payload_variant.security.public_key.bytes, wrong_pub, MESHTASTIC_PKI_KEY_LEN);
	zassert_ok(meshtastic_config_store_set_config(&sec), "seed desynced security cfg");

	/* Re-init: with the fix the pub is re-derived from priv, not trusted from NVS. */
	zassert_ok(meshtastic_pki_init(), "pki_init failed");

	/* The advertised key now matches the private key (the true derived pub). */
	zassert_equal(meshtastic_pki_get_public_key(got_pub), (size_t)MESHTASTIC_PKI_KEY_LEN,
		      "get pub len");
	zassert_mem_equal(got_pub, correct_pub, MESHTASTIC_PKI_KEY_LEN,
			  "advertised pub must be re-derived from the private key");

	/* ...and the NVS was self-healed (stored pub corrected, not left wrong). */
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_security_tag, &sec),
		   "reread sec");
	zassert_mem_equal(sec.payload_variant.security.public_key.bytes, correct_pub,
			  MESHTASTIC_PKI_KEY_LEN, "stored pub must be corrected in NVS");

	/* Restore the suite's original keypair for any later test. */
	if (had_saved) {
		zassert_ok(meshtastic_config_store_set_config(&saved), "restore sec");
		zassert_ok(meshtastic_pki_init(), "re-init restore");
	}
}

/* A DM to a peer whose public key we DO hold is PKC-encrypted: dest is the peer,
 * and the wire channel-hash byte is the 0x00 PKC marker (not a channel hash). */
ZTEST(admin_pki, test_pkc_dm_uses_zero_wire_marker)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];
	const struct meshtastic_wire_header *hdr =
		(const struct meshtastic_wire_header *)mock_lora.last_tx;

	gen_x25519_pubkey(pub);
	seed_peer_pubkey(pub);

	zassert_ok(meshtastic_send_text(PEER_NODE_ID, "hi peer"), "pkc dm send failed");

	zassert_equal(sys_le32_to_cpu(hdr->dest), PEER_NODE_ID, "PKC DM dest must be the peer");
	zassert_equal(hdr->channel, 0x00U,
		      "a PKC DM must carry the 0x00 PKC wire marker, not a channel hash");
}

/* A DIRECTED position — a portnum upstream excludes from PKC — stays on the
 * channel even when we hold the peer's key (parity with perhapsEncode's portnum
 * carve-outs). Wire byte is the primary channel hash, not the 0x00 PKC marker. */
ZTEST(admin_pki, test_directed_position_stays_on_channel)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];
	uint8_t payload[8] = {0};
	uint8_t primary_hash = meshtastic_channels_get_hash(meshtastic_channels_primary_index());
	const struct meshtastic_wire_header *hdr =
		(const struct meshtastic_wire_header *)mock_lora.last_tx;

	gen_x25519_pubkey(pub);
	seed_peer_pubkey(pub);

	zassert_ok(meshtastic_send_data(PEER_NODE_ID, MESHTASTIC_PORT_POSITION, payload,
					sizeof(payload), K_FOREVER),
		   "directed position send failed");

	zassert_equal(hdr->channel, primary_hash,
		      "a directed POSITION must stay channel-encrypted (excluded from PKC)");
}

/* On-air RX PKC-first (change-pointer 7, docs/parity/crypto-channels.md): the
 * decode entry meshtastic_try_decode_wire_packet tries PKC before the channel
 * loop, so a PKC DM to us is never shadowed by a channel whose hash also lands
 * on the 0x00 PKC marker — and, symmetrically, a genuine channel frame on that
 * colliding channel still decodes as channel traffic because the PKC attempt
 * fails CCM authentication and falls through. Both directions are asserted with
 * a SECONDARY channel deliberately provisioned to hash to 0x00. */
ZTEST(admin_pki, test_rx_pkc_first_beats_colliding_channel)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];
	uint8_t plain[MESHTASTIC_MAX_PAYLOAD_LEN];
	size_t plain_len = 0;
	uint8_t enc[MESHTASTIC_MAX_PAYLOAD_LEN + MESHTASTIC_PKI_OVERHEAD];
	size_t enc_len = 0;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *hdr = (struct meshtastic_wire_header *)wire;
	struct meshtastic_packet pkt = {0};
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool decoded = false;
	enum meshtastic_decode_fail fail = MESHTASTIC_DECODE_FAIL_NONE;
	int ret;

	gen_x25519_pubkey(pub);
	seed_peer_pubkey(pub);

	/* Provision a SECONDARY at index 1 that hashes to the 0x00 PKC marker.
	 * hash = xorHash(name) ^ xorHash(key); "aa" folds to 0 and a 16-byte key of
	 * {0xAB,0xAB,0,...} folds to 0 while staying non-zero (a real key), so the
	 * hash is 0 ^ 0 = 0x00. The assert both validates the arithmetic and guards
	 * against secondary-PSK inheritance silently replacing our key. */
	{
		meshtastic_Channel ch = meshtastic_Channel_init_zero;

		ch.role = meshtastic_Channel_Role_SECONDARY;
		ch.has_settings = true;
		strncpy(ch.settings.name, "aa", sizeof(ch.settings.name) - 1U);
		ch.settings.psk.size = 16U;
		ch.settings.psk.bytes[0] = 0xABU;
		ch.settings.psk.bytes[1] = 0xABU;
		zassert_ok(meshtastic_channels_set_slot(1U, &ch), "colliding channel set failed");
		zassert_equal(meshtastic_channels_get_hash(1U), 0x00U,
			      "test setup: channel 1 must hash to the 0x00 PKC marker");
	}

	/* Case 1 — a genuine PKC DM (wire byte 0x00, to us). Must decode as PKC and
	 * NOT be captured by the colliding channel. Uses the same ECDH-symmetry
	 * forge as inject_pkc_admin (encrypt "to PEER" so our decrypt "from PEER"
	 * reproduces the shared secret). */
	zassert_ok(meshtastic_encode_data(MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)"hi", 2U,
					  plain, sizeof(plain), &plain_len),
		   "Data encode failed");
	zassert_ok(meshtastic_pki_encrypt(PEER_NODE_ID, PEER_NODE_ID, 0x0C0FFEE1U, plain, plain_len,
					  enc, sizeof(enc), &enc_len),
		   "PKC encrypt failed");

	memset(hdr, 0, MESHTASTIC_HDR_LEN);
	hdr->dest = sys_cpu_to_le32(TEST_NODE_ID);
	hdr->src = sys_cpu_to_le32(PEER_NODE_ID);
	hdr->id = sys_cpu_to_le32(0x0C0FFEE1U);
	hdr->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	hdr->channel = 0x00U; /* PKC marker — also the colliding channel's hash */
	memcpy(wire + MESHTASTIC_HDR_LEN, enc, enc_len);

	ret = meshtastic_try_decode_wire_packet(wire, (int)(MESHTASTIC_HDR_LEN + enc_len), -20, 4,
						&pkt, payload, sizeof(payload), &decoded, &fail, NULL);
	zassert_ok(ret, "PKC DM decode returned %d", ret);
	zassert_true(decoded, "PKC DM must decode");
	zassert_true(pkt.pki_encrypted,
		     "PKC DM must be flagged pki_encrypted, not captured by the 0x00 channel");
	zassert_equal(pkt.channel_index, 0U, "PKC DM must land on the PKC pseudo-channel 0");
	zassert_equal(pkt.portnum, (uint32_t)MESHTASTIC_PORT_TEXT_MESSAGE, "wrong portnum");
	zassert_equal(pkt.payload_len, 2U, "wrong PKC payload len");
	zassert_mem_equal(pkt.payload, "hi", 2U, "PKC DM payload mismatch");

	/* Case 2 — a real channel frame on the colliding channel, addressed to us
	 * (wire byte 0x00). The PKC-first attempt must fail CCM auth cleanly and
	 * fall through to channel decode. Built as a broadcast on channel 1 (so the
	 * TX PKC decision is skipped), then re-addressed to us; the AES-CTR nonce is
	 * id+from, so re-addressing does not affect decryptability. */
	{
		struct meshtastic_packet tx = {
			.from = PEER_NODE_ID,
			.to = MESHTASTIC_NODE_BROADCAST,
			.id = 0x0C0FFEE2U,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = (const uint8_t *)"ch",
			.payload_len = 2U,
			.channel_index = 1U,
		};
		uint8_t built[MESHTASTIC_PKT_MAX];
		uint32_t built_len = 0;
		struct meshtastic_wire_header *bh = (struct meshtastic_wire_header *)built;
		struct meshtastic_packet cpkt = {0};

		zassert_ok(meshtastic_build_wire_packet(&tx, built, &built_len),
			   "colliding channel frame build failed");
		zassert_equal(bh->channel, 0x00U,
			      "the colliding channel frame must carry wire byte 0x00");
		bh->dest = sys_cpu_to_le32(TEST_NODE_ID); /* re-address to us */

		decoded = false;
		fail = MESHTASTIC_DECODE_FAIL_NONE;
		ret = meshtastic_try_decode_wire_packet(built, (int)built_len, -20, 4, &cpkt, payload,
							sizeof(payload), &decoded, &fail, NULL);
		zassert_ok(ret, "channel-frame decode returned %d", ret);
		zassert_true(decoded, "channel frame on the colliding channel must decode");
		zassert_false(cpkt.pki_encrypted,
			      "a channel frame must NOT be flagged pki_encrypted");
		zassert_equal(cpkt.channel_index, 1U, "channel frame must land on channel 1");
		zassert_equal(cpkt.payload_len, 2U, "wrong channel payload len");
		zassert_mem_equal(cpkt.payload, "ch", 2U, "channel frame payload mismatch");
	}

	/* Tear down: disable channel 1 so later tests in the suite see a clean table
	 * (there is no per-test channel reset). */
	{
		meshtastic_Channel off = meshtastic_Channel_init_zero;

		off.role = meshtastic_Channel_Role_DISABLED;
		off.has_settings = true;
		zassert_ok(meshtastic_channels_set_slot(1U, &off), "colliding channel teardown failed");
	}
}

/* A real PKC admin whose sender key is configured in admin_key[] is authorized;
 * with a valid session passkey the mutating op applies. */
ZTEST(admin_pki, test_pkc_admin_key_authorized_applies)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key), buf,
				    sizeof(buf));

	inject_pkc_admin(buf, len, 0x0AD10001U);
	k_sleep(K_MSEC(50));

	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "authorized PKC admin must decrypt, authorize, and apply the config");

	set_admin_key(NULL, 0U);
}

/* F-1: a LoRa config change over admin must SCHEDULE A REBOOT (the SX1262 is only
 * reconfigured at radio init, so a live config write alone leaves the radio on the
 * old preset/frequency), whereas a live-applied section (device role) must NOT.
 * Upstream reboots on any LoRaConfig change; the port previously excluded lora and
 * silently diverged. */
ZTEST(admin_pki, test_lora_config_change_applies_live)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;
	uint32_t generation_before;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	/* Device role applies live via apply_core -> must NOT schedule a reboot. */
	meshtastic_admin_cancel_reboot();
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0AD1F001U);
	k_sleep(K_MSEC(50));
	zassert_false(meshtastic_admin_reboot_scheduled(),
		      "device-role change applies live and must NOT schedule a reboot");

	/* LoRa preset change now reaches the radio live too (agents-k8oe): the
	 * reference never reboots for a LoRaConfig change
	 * (AdminModule.cpp:1108-1110, requiresReboot = false unconditionally for
	 * lora), and neither should we. Proven two ways: no reboot scheduled, and
	 * the preset generation counter -- bumped only by a successful
	 * meshtastic_preset_switch()/_apply_stored() retune -- actually advanced,
	 * so this isn't just "didn't schedule a reboot" but "really reconfigured
	 * the radio". */
	generation_before = meshtastic_preset_generation();
	meshtastic_admin_cancel_reboot();
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_lora_preset(meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST, key,
					   sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0AD1F002U);
	k_sleep(K_MSEC(50));
	zassert_false(meshtastic_admin_reboot_scheduled(),
		      "a LoRa config change applies live and must NOT schedule a reboot");
	zassert_true(meshtastic_preset_generation() != generation_before,
		     "a LoRa config change must actually retune the radio, not just skip "
		     "the reboot");

	/* Cleanup: cancel the scheduled reboot (so the sim doesn't reboot, in case
	 * anything above did schedule one) and restore the default preset for any
	 * later test. */
	meshtastic_admin_cancel_reboot();
	{
		meshtastic_Config lora = meshtastic_Config_init_zero;

		lora.which_payload_variant = meshtastic_Config_lora_tag;
		lora.payload_variant.lora.tx_enabled = true;
		lora.payload_variant.lora.use_preset = true;
		lora.payload_variant.lora.modem_preset =
			meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
		lora.payload_variant.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
		lora.payload_variant.lora.hop_limit = 3;
		(void)meshtastic_config_store_set_config(&lora);
	}
	set_admin_key(NULL, 0U);
}

/*
 * agents-dnr4.5: the test above proves a LoRa config change applies LIVE; it
 * says nothing about whether the value would still be there after a real
 * reboot. This is the missing half -- a genuine settings_save_subtree()/
 * settings_load_subtree("meshtastic") round trip through the real NVS
 * backend, not the RAM-only struct these config_store setters mutate
 * directly. Same harness as test_edit_transaction_suppresses_a_save_already_
 * queued and test_fixed_position_survives_real_reboot; see
 * docs/ADMIN-CONFIG-CAPABILITY-AUDIT.md §3.
 */
ZTEST(admin_pki, test_lora_config_survives_real_reboot)
{
	meshtastic_Config lora = meshtastic_Config_init_zero;
	meshtastic_Config got = meshtastic_Config_init_zero;

	lora.which_payload_variant = meshtastic_Config_lora_tag;
	lora.payload_variant.lora.tx_enabled = true;
	lora.payload_variant.lora.use_preset = true;
	lora.payload_variant.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST;
	lora.payload_variant.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
	lora.payload_variant.lora.hop_limit = 5;
	zassert_ok(meshtastic_config_store_set_config(&lora), "set lora failed");
	zassert_ok(settings_save_subtree("meshtastic"), "lora flush failed");

	/* Change it again, but do NOT flush -- a real reboot must not see this. */
	lora.payload_variant.lora.hop_limit = 2;
	lora.payload_variant.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	zassert_ok(meshtastic_config_store_set_config(&lora), "set lora (unflushed) failed");

	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &got),
		   "lora reread failed");
	zassert_equal(got.payload_variant.lora.hop_limit, 5,
		      "hop_limit must survive a real reboot at its FLUSHED value, not the "
		      "unflushed edit");
	zassert_equal(got.payload_variant.lora.modem_preset,
		      meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,
		      "modem_preset must survive a real reboot at its FLUSHED value");

	/* Restore the suite default for later tests. */
	lora.payload_variant.lora.tx_enabled = true;
	lora.payload_variant.lora.use_preset = true;
	lora.payload_variant.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	lora.payload_variant.lora.hop_limit = 3;
	(void)meshtastic_config_store_set_config(&lora);
	(void)settings_save_subtree("meshtastic");
}

/*
 * agents-dnr4.1: a save already queued (within the debounce window) before
 * begin_edit_settings opened must NOT be allowed to fire mid-transaction --
 * that would export a partially-edited store to flash and defeat the whole
 * point of the transaction. Regression test for the race fixed in
 * meshtastic_settings.c's save_work_handler(), which previously called
 * settings_save_subtree() unconditionally regardless of admin_edit_open.
 */
ZTEST(admin_pki, test_edit_transaction_suppresses_a_save_already_queued)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;
	uint32_t id = 0x0AD1E000U;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	/* Clean baseline, flushed to real NVS so the probe below has a known
	 * pre-transaction value to check against. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));
	zassert_ok(settings_save_subtree("meshtastic"), "baseline flush failed");

	/* Queue a debounced save (default 1000 ms) by changing the role... */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	/* ...then immediately open an edit transaction, BEFORE that save fires. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_edit_settings(false, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	/* A further edit inside the transaction. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_TRACKER, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	/* Let the originally-queued save's debounce window elapse (comfortably past
	 * the default 1000ms, comfortably under the suite's 1500ms idle-timeout so
	 * THAT mechanism doesn't also fire and confuse what this test is isolating).
	 * Before the fix, save_work_handler() would export the live (mid-transaction)
	 * store here. */
	k_sleep(K_MSEC(1100));

	/* Probe NVS directly by reloading the subtree. If nothing was flushed since
	 * the baseline, the role must come back CLIENT -- not ROUTER (the queued
	 * write) and definitely not TRACKER (the in-transaction edit). */
	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "a save fired mid-transaction: NVS should still hold the pre-transaction "
		      "role, not an in-progress edit");

	/* The reload just clobbered RAM back to CLIENT -- that's a side effect of
	 * probing flash, not a real edit. Prove a real commit still works: re-apply
	 * the edit and commit it. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_TRACKER, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_edit_settings(true, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	zassert_ok(settings_load_subtree("meshtastic"), "settings reload after commit failed");
	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_TRACKER,
		      "commit must persist the transaction's final value");

	/* Clean up: leave the shared suite state at its expected default. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));
	(void)settings_save_subtree("meshtastic");

	set_admin_key(NULL, 0U);
}

/*
 * agents-dnr4.2: an edit transaction left open with no commit AND no further
 * admin activity must not suppress saves forever. Regression test for the
 * missing idle-timeout in meshtastic_admin.c -- CONFIG_MESHTASTIC_ADMIN_EDIT_
 * IDLE_MS is lowered to 200 in this suite's prj.conf so the test stays fast.
 */
ZTEST(admin_pki, test_edit_transaction_auto_commits_after_idle_timeout)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;
	uint32_t id = 0x0ED10000U;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	/* Clean baseline, flushed. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));
	zassert_ok(settings_save_subtree("meshtastic"), "baseline flush failed");

	/* Open a transaction... */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_edit_settings(false, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	/* ...make an edit inside it... */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));

	/* ...and then go silent. Deliberately never send commit_edit_settings. */
	k_sleep(K_MSEC(CONFIG_MESHTASTIC_ADMIN_EDIT_IDLE_MS + 100));

	/* The idle timeout must have auto-committed by now: a real reload proves
	 * it (not just that admin_edit_open happens to read false). */
	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "an edit transaction idle with no commit must auto-commit, not suppress "
		      "saves forever");

	/* Clean up: leave the shared suite state at its expected default. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, id++);
	k_sleep(K_MSEC(50));
	(void)settings_save_subtree("meshtastic");

	set_admin_key(NULL, 0U);
}

/*
 * agents-dnr4.3: a commit_edit_settings with NO transaction open (no prior
 * begin_edit_settings) must be a no-op, not an unconditional un-suppress +
 * flush -- otherwise a stray/errant commit arriving while something ELSE has
 * deliberately suppressed saves (a factory reset mid-wipe, in the few seconds
 * before its scheduled reboot -- see factory_reset_config/_device in
 * meshtastic_admin.c) would undo that suppression and resurrect whatever the
 * live RAM store currently holds, right after the reset went out of its way
 * not to persist it. This is this port's equivalent of upstream calling
 * disableBluetooth() around destructive/commit ops: rather than silencing the
 * transport a stray message could arrive on, refuse to act on a commit that
 * doesn't correspond to a real open transaction.
 */
ZTEST(admin_pki, test_stray_commit_edit_settings_is_a_noop)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t peer_key_now[MESHTASTIC_PKI_KEY_LEN];
	size_t len;

	/* PEER's pinned NodeDB key is not suite-stable (see the comment in
	 * test_remove_by_nodenum_purges_the_warm_pinned_key) -- admin_key[] must
	 * match whatever it ACTUALLY is right now, not the suite's original
	 * peer_pubkey, or this admin message silently fails authorization and the
	 * test passes vacuously without ever exercising the guard. */
	zassert_ok(meshtastic_nodedb_copy_pubkey(PEER_NODE_ID, peer_key_now),
		   "PEER must have SOME pinned key by this point in the suite");
	set_admin_key(peer_key_now, sizeof(peer_key_now));

	/* Simulate something else (e.g. an in-flight factory reset) having
	 * deliberately suppressed saves, with no edit transaction actually open. */
	meshtastic_config_store_set_save_suppressed(true);

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_edit_settings(true, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0C3D1000U);
	k_sleep(K_MSEC(50));

	zassert_true(meshtastic_config_store_save_suppressed(),
		     "a stray commit_edit_settings (no matching begin) must not un-suppress "
		     "saves someone else deliberately suppressed");

	/* Clean up so later tests see normal save scheduling. */
	meshtastic_config_store_set_save_suppressed(false);
	set_admin_key(NULL, 0U);
}

/*
 * agents-dnr4.3 (end-to-end): a stray commit_edit_settings arriving in the
 * few seconds between factory_reset_config wiping NVS and its scheduled
 * reboot must not resurrect the wiped config/device key. Complements the
 * isolated guard test above with the real factory_reset_config path.
 */
ZTEST(admin_pki, test_factory_reset_config_survives_a_stray_commit)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;
	meshtastic_Config got;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	/* Baseline: a real, flushed, non-default value. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0F0000A1U);
	k_sleep(K_MSEC(50));
	zassert_ok(settings_save_subtree("meshtastic"), "baseline flush failed");

	/* Factory-reset-config: wipes NVS (config/device key gone) but the reset
	 * intentionally leaves RAM untouched and saves suppressed until reboot. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_factory_reset_config(key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0F0000A2U);
	k_sleep(K_MSEC(50));
	meshtastic_admin_cancel_reboot(); /* don't actually reboot the test binary */

	/* A stray commit_edit_settings arrives in the vulnerable window (no
	 * begin_edit_settings ever sent -- exactly the scenario the guard above
	 * protects). Before the fix this would flush RAM (still ROUTER) straight
	 * back into the just-wiped key. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_edit_settings(true, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0F0000A3U);
	k_sleep(K_MSEC(50));

	/* Probe: overwrite RAM with a sentinel the wiped key can't legitimately
	 * produce, then reload. If config/device still doesn't exist in NVS (the
	 * wipe held), the sentinel survives; if the stray commit resurrected it,
	 * reload overwrites the sentinel back to ROUTER. */
	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE);
	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &got),
		   "device reread failed");
	zassert_equal(got.payload_variant.device.role,
		      meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE,
		      "factory_reset_config's wipe must survive a stray commit_edit_settings "
		      "arriving before the scheduled reboot -- role should stay at the "
		      "sentinel (key still absent from NVS), not resurrect as ROUTER");

	/* Clean up: restore a real baseline for later tests. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_CLIENT, key, sizeof(key),
				    buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0F0000A4U);
	k_sleep(K_MSEC(50));
	(void)settings_save_subtree("meshtastic");

	set_admin_key(NULL, 0U);
}

/*
 * agents-t2hb.1: the admin path needs NO per-field code to reach
 * device.node_info_broadcast_secs -- set_config_tag already routes every
 * Config section through meshtastic_config_store_set_config() generically.
 * Proven by driving it through the exact same PKC-encrypted AdminMessage
 * path a real remote admin client uses, not just the local shell.
 */
ZTEST(admin_pki, test_admin_sets_nodeinfo_interval)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_nodeinfo_interval(120U, key, sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0AD1F003U);
	k_sleep(K_MSEC(50));

	zassert_equal(meshtastic_nodeinfo_interval_secs(), 120U,
		     "an admin set_config(device) write must reach node_info_broadcast_secs "
		     "with no admin.c changes");
	zassert_false(meshtastic_admin_reboot_scheduled(),
		      "a device-section change applies live and must NOT schedule a reboot");

	/* Restore, for any later test. */
	(void)meshtastic_config_store_set_node_info_interval(0U);
	set_admin_key(NULL, 0U);
}

/* Create/refresh a *key-bearing* hot NodeDB entry for @p num via a NodeInfo.
 * The fillers must be key-verified: the eviction picker spares key-verified nodes
 * while any keyless ("boring") node is still evictable, so to push the admin's own
 * keyed record out of the hot store the store has to be saturated with other keyed
 * nodes (a keyless flood would never touch it). */
static void hot_fill_node(uint32_t num)
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = num,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = num ^ 0x5EEDU,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	/* A distinct non-empty filler key per node — the contents are irrelevant,
	 * only that the entry is key-verified rather than "boring". */
	user.public_key.size = MESHTASTIC_PKI_KEY_LEN;
	memset(user.public_key.bytes, (uint8_t)num, MESHTASTIC_PKI_KEY_LEN);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "filler User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

/* A-1: PKC admin authorization must not depend on hot-store residency. Evict
 * the admin's hot record (its key survives only in the warm ring) and replay a
 * genuine PKC admin frame: it must still authorize and apply. */
ZTEST(admin_pki, test_pkc_admin_key_authorized_after_hot_eviction)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t warm_key[MESHTASTIC_PKI_KEY_LEN];
	struct meshtastic_nodedb_node snap;
	size_t len;

	set_admin_key(peer_pubkey, sizeof(peer_pubkey));

	/* Saturate the hot store with fresh key-bearing nodes until PEER's record is
	 * evicted — a keyless flood would never displace PEER's keyed record. Capped
	 * well above MAX_NODES so a tie-break change can't hang the loop; the warm
	 * ring is sized above the hot store (prj.conf) so these fillers do not also
	 * push PEER's key out of the warm tier. */
	for (uint32_t i = 0U; i < 64U && meshtastic_nodedb_get(PEER_NODE_ID, &snap) == 0; i++) {
		hot_fill_node(0x5A000001U + i);
	}
	zassert_equal(meshtastic_nodedb_get(PEER_NODE_ID, &snap), -ENOENT,
		      "PEER should be evicted from the hot store");
	zassert_ok(meshtastic_nodedb_copy_pubkey(PEER_NODE_ID, warm_key),
		   "PEER's key must survive in the warm ring");
	zassert_mem_equal(warm_key, peer_pubkey, sizeof(peer_pubkey),
			  "warm ring must hold PEER's real key");

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key), buf,
				    sizeof(buf));

	inject_pkc_admin(buf, len, 0x0AD10003U);
	k_sleep(K_MSEC(50));

	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "hot-store eviction must not lock out an authorized PKC admin");

	set_admin_key(NULL, 0U);
}

/* B-5: a keyed peer dropped from the hot store keeps its role when re-admitted via a
 * NON-NodeInfo packet — the fresh hot entry is seeded from the warm tier, not CLIENT.
 * Uses the warm ring (this suite has MESHTASTIC_SETTINGS/PERSIST_KEYS on). */
ZTEST(admin_pki, test_warm_tier_carries_role_on_readmit)
{
	const uint32_t router = 0x00B10000U;
	struct meshtastic_nodedb_node snap;
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	uint8_t text[] = {'h', 'i'};
	struct meshtastic_packet ni = {
		.from = router,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = 0xE4000000U,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};
	struct meshtastic_packet txt = {
		.from = router,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = 0xE4000001U,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.channel_index = meshtastic_channels_primary_index(),
		.payload = text,
		.payload_len = sizeof(text),
	};

	/* Learn a keyed peer advertising the ROUTER role (warm ring records key + role). */
	user.role = meshtastic_Config_DeviceConfig_Role_ROUTER;
	user.public_key.size = MESHTASTIC_PKI_KEY_LEN;
	memset(user.public_key.bytes, 0x5AU, MESHTASTIC_PKI_KEY_LEN);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);

	zassert_ok(meshtastic_nodedb_get(router, &snap), "peer should be learned");
	zassert_equal(snap.role, meshtastic_Config_DeviceConfig_Role_ROUTER, "role should be ROUTER");

	/* Drop it from the hot store; the warm ring keeps its key + role. */
	zassert_ok(meshtastic_nodedb_remove(router), "remove failed");
	zassert_equal(meshtastic_nodedb_get(router, &snap), -ENOENT, "gone from hot store");

	/* Re-hear via a TEXT packet (apply_user does NOT run): the fresh entry must take its role
	 * from the warm tier rather than default to CLIENT. */
	meshtastic_handle_inbound_packet(&txt, NULL, 0U, true);

	zassert_ok(meshtastic_nodedb_get(router, &snap), "peer should be re-admitted");
	zassert_equal(snap.role, meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "re-admitted peer must keep its ROUTER role from the warm tier (B-5)");

	(void)meshtastic_nodedb_remove(router);
}

/* A real PKC admin decrypts (pki_encrypted set), but its sender key is NOT in
 * admin_key[] -> refused, config unchanged. */
ZTEST(admin_pki, test_pkc_non_admin_key_refused)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;

	set_admin_key(NULL, 0U); /* no admin keys configured */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key), buf,
				    sizeof(buf));

	inject_pkc_admin(buf, len, 0x0AD10002U);
	k_sleep(K_MSEC(50));

	zassert_equal(current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "PKC admin whose key is not authorized must be refused");
}

/* H2: a peer whose public key we do not hold makes the PKC decrypt path ask for
 * its NodeInfo. That request must be throttled per peer and must not block the
 * RX thread — otherwise a stream of cheap junk frames with rolling ids (each
 * decode-failing with -ENOENT, each passing dedup) is an amplifier: one small
 * inbound frame in, one larger NodeInfo TX out, unbounded, from a single
 * spoofed id. Feed exactly that flood and assert the cooldown holds. */
ZTEST(admin_pki, test_unknown_key_nodeinfo_request_is_throttled)
{
	/* A sender we hold no key for: PEER's key is seeded, this one is not. */
	const uint32_t stranger = 0x0BADF00DU;
	uint8_t wire[MESHTASTIC_HDR_LEN + 32U];
	struct meshtastic_wire_header *hdr = (struct meshtastic_wire_header *)wire;
	uint32_t tx_after;

	/* Junk ciphertext: long enough to clear the PKC overhead check so the path
	 * reaches the key lookup, but it will never authenticate. */
	memset(wire + MESHTASTIC_HDR_LEN, 0xA5, 32U);

	for (uint32_t i = 0U; i < 8U; i++) {
		hdr->dest = sys_cpu_to_le32(TEST_NODE_ID);
		hdr->src = sys_cpu_to_le32(stranger);
		/* Rolling id: every frame is unique, so dedup never suppresses it
		 * and each one reaches the decrypt attempt. */
		hdr->id = sys_cpu_to_le32(0x0BAD0100U + i);
		hdr->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
		hdr->channel = 0x00U; /* PKC marker channel-hash */
		hdr->next_hop = 0U;
		hdr->relay_node = 0U;

		inject_rx_frame(wire, sizeof(wire));
	}

	k_sleep(K_MSEC(100));

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	tx_after = mock_lora.send_count;
	k_mutex_unlock(&mock_lora.lock);

	/* 8 junk frames in, at most 1 NodeInfo request out. Before the fix this
	 * was 8 — one unthrottled K_FOREVER send per inbound frame. */
	zassert_true(tx_after <= 1U,
		     "unknown-key NodeInfo request must be throttled: 8 junk frames "
		     "produced %u transmissions",
		     tx_after);
}

/* ---- Persistence: admin-set fixed position survives a save/reload ---------- */

/* The fixed-position coordinates persist through the same NVS record path the
 * settings backend uses (setting_get to save, setting_set to load). Round-trip
 * through those two entry points, wiping the in-RAM copy between, to prove the
 * record encodes and restores the coordinates — and that "no fixed position"
 * round-trips as absent rather than as a (0,0) fix. */
ZTEST(admin_pki, test_fixed_position_record_roundtrip)
{
	uint8_t buf[MESHTASTIC_STORE_VALUE_MAX];
	meshtastic_Position pos = meshtastic_Position_init_zero;
	meshtastic_Position got = meshtastic_Position_init_zero;
	int len;

	pos.has_latitude_i = true;
	pos.latitude_i = 375000000;    /* 37.5000000 deg */
	pos.has_longitude_i = true;
	pos.longitude_i = -1223000000; /* -122.3000000 deg */
	pos.has_altitude = true;
	pos.altitude = 42;
	pos.precision_bits = 16U;

	zassert_ok(meshtastic_config_store_set_fixed_position(&pos));

	len = meshtastic_config_store_setting_get("position_fixed", buf, sizeof(buf));
	zassert_true(len > 0, "encoding the fixed-position record failed (%d)", len);

	/* Wipe the in-RAM copy, as a reboot would. */
	zassert_ok(meshtastic_config_store_clear_fixed_position());
	zassert_equal(meshtastic_config_store_get_fixed_position(&got), -ENOENT,
		      "fixed position should be gone after clear");

	/* Reload from the encoded record. */
	zassert_ok(meshtastic_config_store_setting_set("position_fixed", buf, (size_t)len));

	zassert_ok(meshtastic_config_store_get_fixed_position(&got),
		   "fixed position should be restored from its record");
	zassert_true(got.has_latitude_i && got.has_longitude_i && got.has_altitude,
		     "restored position should keep its has-flags");
	zassert_equal(got.latitude_i, pos.latitude_i, "latitude not restored");
	zassert_equal(got.longitude_i, pos.longitude_i, "longitude not restored");
	zassert_equal(got.altitude, pos.altitude, "altitude not restored");
	zassert_equal(got.precision_bits, pos.precision_bits, "precision not restored");

	/* "No fixed position" must round-trip as absent, not as a (0,0) fix. */
	zassert_ok(meshtastic_config_store_clear_fixed_position());
	len = meshtastic_config_store_setting_get("position_fixed", buf, sizeof(buf));
	zassert_true(len > 0, "encoding the cleared record failed (%d)", len);
	zassert_ok(meshtastic_config_store_setting_set("position_fixed", buf, (size_t)len));
	zassert_equal(meshtastic_config_store_get_fixed_position(&got), -ENOENT,
		      "cleared fixed position must not resurrect as a (0,0) fix");
}

/*
 * agents-dnr4.5: the test above proves the record's encode/decode framing
 * round-trips, but calls meshtastic_config_store_setting_get/set() directly --
 * it never touches the real Zephyr settings/NVS subsystem, so it cannot tell
 * apart "persists" from "would survive a reboot". This is that missing case:
 * a real settings_save_subtree()/settings_load_subtree("meshtastic") round
 * trip, the pattern every other Config-section reboot-survival question
 * should follow (see docs/ADMIN-CONFIG-CAPABILITY-AUDIT.md §3).
 */
ZTEST(admin_pki, test_fixed_position_survives_real_reboot)
{
	meshtastic_Position pos = meshtastic_Position_init_zero;
	meshtastic_Position got = meshtastic_Position_init_zero;

	pos.has_latitude_i = true;
	pos.latitude_i = 400000000; /* 40.0000000 deg */
	pos.has_longitude_i = true;
	pos.longitude_i = -1050000000; /* -105.0000000 deg */
	pos.has_altitude = true;
	pos.altitude = 1600;
	pos.precision_bits = 16U;

	zassert_ok(meshtastic_config_store_set_fixed_position(&pos), "set fixed position failed");
	zassert_ok(settings_save_subtree("meshtastic"), "position flush failed");

	/* Clear it in RAM WITHOUT flushing -- a real reboot must still see the
	 * flushed fix, not this cleared state. */
	zassert_ok(meshtastic_config_store_clear_fixed_position(), "clear (unflushed) failed");
	zassert_equal(meshtastic_config_store_get_fixed_position(&got), -ENOENT,
		      "should read as cleared in RAM before the reload");

	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_ok(meshtastic_config_store_get_fixed_position(&got),
		   "fixed position must survive a real reboot, not stay cleared");
	zassert_equal(got.latitude_i, pos.latitude_i, "latitude not restored from real NVS");
	zassert_equal(got.longitude_i, pos.longitude_i, "longitude not restored from real NVS");
	zassert_equal(got.altitude, pos.altitude, "altitude not restored from real NVS");

	/* Clean up so later tests see no fixed position. */
	zassert_ok(meshtastic_config_store_clear_fixed_position());
	(void)settings_save_subtree("meshtastic");
}

/* ---- Persistence: curated (favorite) node identity survives reboot -------- */

/* Inject a NodeInfo for @num carrying a name + 32-byte key, the way the stack
 * learns a peer on the air (apply_user), so it lands in the hot NodeDB. */
static void seed_named_peer(uint32_t num, const char *long_name, const uint8_t *key)
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = num,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	strncpy(user.long_name, long_name, sizeof(user.long_name) - 1U);
	strcpy(user.short_name, "CT");
	user.public_key.size = MESHTASTIC_PKI_KEY_LEN;
	memcpy(user.public_key.bytes, key, MESHTASTIC_PKI_KEY_LEN);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

/* A favorited node's full identity is written to NVS (the mtrec subtree) and
 * restored into the hot store — even into a slot that no longer holds it. Uses a
 * dedicated node id and cleans up its own NVS record so the shared suite state
 * is untouched. */
ZTEST(admin_pki, test_favorite_node_record_persists)
{
	const uint32_t fav = 0x0FA00001U;
	uint8_t key[MESHTASTIC_PKI_KEY_LEN];
	struct meshtastic_nodedb_node snap;

	memset(key, 0xA5, sizeof(key)); /* a distinct, real key-verified peer */

	seed_named_peer(fav, "FavAlpha", key);
	zassert_ok(meshtastic_nodedb_set_favorite(fav, true), "favorite failed");
	zassert_ok(meshtastic_nodedb_get(fav, &snap), "seeded favorite should be present");
	zassert_true(snap.is_favorite, "node should be favorite");

	/* Persist the curated record now (bypassing the coalesced save timer). */
	zassert_ok(settings_save_subtree("mtrec"), "mtrec save failed");

	/* Drop it from the hot store. The NVS record survives — the prune is
	 * deferred and this test never yields long enough for it to run. */
	zassert_ok(meshtastic_nodedb_remove(fav), "remove failed");
	zassert_equal(meshtastic_nodedb_get(fav, &snap), -ENOENT, "node should be gone from RAM");

	/* Reload the subtree: the record must recreate the node with its identity. */
	zassert_ok(settings_load_subtree("mtrec"), "mtrec load failed");
	zassert_ok(meshtastic_nodedb_get(fav, &snap), "favorite must be restored from NVS");
	zassert_true(snap.is_favorite, "restored node should still be favorite");
	zassert_equal(strcmp(snap.long_name, "FavAlpha"), 0, "restored name mismatch: '%s'",
		      snap.long_name);
	zassert_equal(snap.public_key_len, MESHTASTIC_PKI_KEY_LEN, "restored key length mismatch");
	zassert_equal(memcmp(snap.public_key, key, sizeof(key)), 0, "restored key mismatch");

	/* Clean up: drop the node and delete its NVS record so no phantom favorite
	 * leaks into other tests or a later run sharing this NVS partition. */
	(void)meshtastic_nodedb_remove(fav);
	(void)settings_delete("mtrec/0fa00001");
}

/*
 * agents-dnr4.4: admin remove_by_nodenum must purge the target's pinned PKC key
 * from the warm tier too, not just drop the hot NodeDB entry -- otherwise a node
 * re-admitted with the same id (e.g. a re-flashed/re-keyed board) is silently
 * re-trusted under its OLD key (meshtastic_nodedb_copy_pubkey() falls back to the
 * warm ring). Regression test for routing remove_by_nodenum through
 * meshtastic_nodedb_forget() instead of the bare meshtastic_nodedb_remove().
 */
ZTEST(admin_pki, test_remove_by_nodenum_purges_the_warm_pinned_key)
{
	const uint32_t victim = 0x0FB00001U;
	uint8_t key[MESHTASTIC_PKI_KEY_LEN];
	uint8_t out[MESHTASTIC_PKI_KEY_LEN];
	uint8_t admin_key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t buf[256];
	size_t len;

	memset(key, 0x7EU, sizeof(key)); /* the victim's pinned identity */

	seed_named_peer(victim, "Victim", key);
	zassert_ok(meshtastic_nodedb_copy_pubkey(victim, out), "seeded key must be findable");
	zassert_mem_equal(out, key, sizeof(key), "seeded key mismatch");

	/* Admin-remove it (real PKC-authorized AdminMessage path, same as every
	 * other mutating op in this suite). PEER's *pinned* NodeDB key is not
	 * stable across the whole suite -- other tests (e.g.
	 * test_pkc_dm_uses_zero_wire_marker, test_directed_position_stays_on_
	 * channel) legitimately re-key PEER with a fresh random key for their own
	 * unrelated scenarios and never restore peer_pubkey, and the impersonation
	 * guard means a plain NodeInfo can never re-pin it back once changed. That
	 * is harmless for inject_pkc_admin() itself (meshtastic_pki_encrypt() looks
	 * up PEER's key fresh at encrypt time and the RX path looks it up fresh at
	 * decrypt time -- always self-consistent, whatever the value is) but it
	 * means admin_key[] must be set to whatever PEER's key ACTUALLY is right
	 * now, not the suite's original peer_pubkey. */
	{
		uint8_t peer_key_now[MESHTASTIC_PKI_KEY_LEN];

		zassert_ok(meshtastic_nodedb_copy_pubkey(PEER_NODE_ID, peer_key_now),
			   "PEER must have SOME pinned key by this point in the suite");
		set_admin_key(peer_key_now, sizeof(peer_key_now));
	}
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(admin_key);
	len = encode_admin_remove_by_nodenum(victim, admin_key, sizeof(admin_key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0FB00002U);
	k_sleep(K_MSEC(50));
	set_admin_key(NULL, 0U);

	/* Gone from the hot store either way -- the discriminating assertion is next. */
	{
		struct meshtastic_nodedb_node snap;

		zassert_equal(meshtastic_nodedb_get(victim, &snap), -ENOENT,
			      "victim should be gone from the hot store");
	}

	/* The real proof: no trace of the key anywhere, hot OR warm. Before routing
	 * through meshtastic_nodedb_forget(), this would still succeed via the warm
	 * fallback -- the bug this test catches. */
	zassert_not_equal(meshtastic_nodedb_copy_pubkey(victim, out), 0,
			  "admin remove_by_nodenum must purge the warm-tier key too, not just "
			  "the hot NodeDB entry");
}

/* ---- Upgrade safety: config record version window ------------------------ */

/* decode_record accepts a version WINDOW, not an exact match, so bumping the
 * record version never silently drops older records back to compile-time
 * defaults (which for config/security would regenerate the node identity). A
 * record stamped within [MIN, CUR] loads; one stamped outside is refused
 * without disturbing the value already held. */
ZTEST(admin_pki, test_config_record_version_window)
{
	uint8_t buf[MESHTASTIC_STORE_VALUE_MAX];
	meshtastic_Config pos = meshtastic_Config_init_zero;
	meshtastic_Config got = meshtastic_Config_init_zero;
	int len;

	pos.which_payload_variant = meshtastic_Config_position_tag;
	pos.payload_variant.position.position_broadcast_secs = 1234U;
	zassert_ok(meshtastic_config_store_set_config(&pos), "set position config failed");

	/* Encode the record as NVS would. Byte 0 is the record version. */
	len = meshtastic_config_store_setting_get("config/position", buf, sizeof(buf));
	zassert_true(len > 0, "encode failed (%d)", len);

	/* The current version loads and restores the value. */
	pos.payload_variant.position.position_broadcast_secs = 0U;
	zassert_ok(meshtastic_config_store_set_config(&pos), "reset value failed");
	zassert_ok(meshtastic_config_store_setting_set("config/position", buf, (size_t)len),
		   "current-version record must load");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &got));
	zassert_equal(got.payload_variant.position.position_broadcast_secs, 1234U,
		      "value not restored from current-version record");

	/* A version above the accepted window is refused, and the refusal leaves the
	 * in-RAM value intact (no revert-to-default). */
	buf[0] = 0xFFU;
	zassert_equal(meshtastic_config_store_setting_set("config/position", buf, (size_t)len),
		      -EINVAL, "a future-version record must be refused");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_position_tag, &got));
	zassert_equal(got.payload_variant.position.position_broadcast_secs, 1234U,
		      "a refused record must not wipe the held value");

	/* A version below the window is likewise refused. */
	buf[0] = 0x00U;
	zassert_equal(meshtastic_config_store_setting_set("config/position", buf, (size_t)len),
		      -EINVAL, "a too-old-version record must be refused");
}

/* ADMIN-1: set_owner with only a long name must NOT wipe the short name, and
 * set_owner with only a short name must NOT wipe the long name — an empty field
 * means "leave unchanged" (upstream AdminModule::handleSetOwner guards each with
 * `if (*o.long_name)` / `if (*o.short_name)`). Before the fix, `--set-owner "X"`
 * blanked the short name on air. */
ZTEST(admin_pki, test_set_owner_preserves_empty_name_fields)
{
	meshtastic_User seed = meshtastic_User_init_zero;
	meshtastic_User probe;
	meshtastic_User rb;

	/* Establish a known long+short. */
	strcpy(seed.long_name, "Base Long Name");
	strcpy(seed.short_name, "BASE");
	zassert_ok(meshtastic_config_store_set_owner(&seed), "seed owner failed");

	/* Update only the long name (empty short) — the short name must survive. */
	probe = (meshtastic_User)meshtastic_User_init_zero;
	strcpy(probe.long_name, "Renamed Long");
	zassert_ok(meshtastic_config_store_set_owner(&probe), "set long-only failed");
	meshtastic_fill_user(&rb);
	zassert_str_equal(rb.long_name, "Renamed Long", "long name must update");
	zassert_str_equal(rb.short_name, "BASE", "short name must be preserved on empty short");

	/* Update only the short name (empty long) — the long name must survive. */
	probe = (meshtastic_User)meshtastic_User_init_zero;
	strcpy(probe.short_name, "NEW");
	zassert_ok(meshtastic_config_store_set_owner(&probe), "set short-only failed");
	meshtastic_fill_user(&rb);
	zassert_str_equal(rb.short_name, "NEW", "short name must update");
	zassert_str_equal(rb.long_name, "Renamed Long", "long name must be preserved on empty long");
}

/* Pin the PKC (X25519+AES-CCM) nonce builder against a vector harvested by
 * running upstream's actual CryptoEngine::initNonce (tests/vectors, tool
 * tools/vectors/harvest.py) — not a hand-reimplemented copy of the formula.
 * A self-loopback encrypt/decrypt test can't catch a layout error here: both
 * sides use the same function, so they'd agree with each other even if the
 * bytes are in the wrong place relative to a real stock node. */
ZTEST(admin_pki, test_pki_nonce_matches_reference_layout)
{
	uint8_t nonce[MESHTASTIC_PKI_NONCE_LEN];

	/* pkc_extra_aabbccdd: id=1, from=0xdeadbeef, extraNonce=0xaabbccdd. */
	meshtastic_pki_nonce_build(nonce, 1U, 0xdeadbeefU, 0xaabbccddU);
	zassert_mem_equal(nonce,
			  ((const uint8_t[MESHTASTIC_PKI_NONCE_LEN]){ 0x01, 0x00, 0x00, 0x00, 0xdd, 0xcc,
								      0xbb, 0xaa, 0xef, 0xbe, 0xad, 0xde,
								      0x00 }),
			  sizeof(nonce), "PKC nonce layout must match reference initNonce(extraNonce!=0)");
}

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
/* The remote-admin CLIENT end to end (agents-xhli.3): our get request leaves
 * as a real PKC unicast; the peer's (forged, genuinely PKC-encrypted) response
 * comes back through the full RX path — router, decrypt, admin handle_remote —
 * where the client consumes it by sender + request_id BEFORE the admin_key
 * gate (our admin_key list is empty here: the target we query is not required
 * to hold our key) and caches the session passkey the mutating op needs. */
ZTEST(admin_pki, test_admin_client_get_then_set_roundtrip)
{
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	meshtastic_Data data = meshtastic_Data_init_zero;
	uint8_t enc[MESHTASTIC_MAX_PAYLOAD_LEN + MESHTASTIC_PKI_OVERHEAD];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *whdr = (struct meshtastic_wire_header *)wire;
	const struct meshtastic_wire_header *txh;
	pb_ostream_t os;
	uint32_t req_id;
	size_t enc_len = 0;
	const uint32_t resp_id = 0x0AD1C101U;

	set_admin_key(NULL, 0U);

	/* Client-side gate: no passkey, no mutating op. */
	zassert_false(meshtastic_admin_client_have_passkey(PEER_NODE_ID));
	zassert_equal(meshtastic_admin_client_set_owner(PEER_NODE_ID, "X", "Y"), -EACCES,
		      "mutating op without a passkey must be refused client-side");

	/* The get leaves as PKC (0x00 wire marker) addressed to the peer; its
	 * packet id — plaintext in the header — is the response correlation. */
	mock_lora.send_count = 0U;
	zassert_ok(meshtastic_admin_client_get_owner(PEER_NODE_ID), "get_owner send failed");
	k_sleep(K_MSEC(50));
	zassert_equal(mock_lora.send_count, 1U, "expected exactly the request on air");
	txh = (const struct meshtastic_wire_header *)mock_lora.last_tx;
	zassert_equal(sys_le32_to_cpu(txh->dest), PEER_NODE_ID, "request must target the peer");
	zassert_equal(txh->channel, 0x00U, "client request must be PKC");
	req_id = sys_le32_to_cpu(txh->id);

	/* The peer's response: owner + its session passkey, request_id set,
	 * genuinely PKC-encrypted via the ECDH-symmetry trick. */
	resp.which_payload_variant = meshtastic_AdminMessage_get_owner_response_tag;
	strcpy(resp.payload_variant.get_owner_response.long_name, "Peer Node");
	strcpy(resp.payload_variant.get_owner_response.short_name, "PEER");
	resp.session_passkey.size = MESHTASTIC_ADMIN_SESSION_KEY_LEN;
	memset(resp.session_passkey.bytes, 0x5a, MESHTASTIC_ADMIN_SESSION_KEY_LEN);

	data.portnum = (meshtastic_PortNum)MESHTASTIC_PORT_ADMIN;
	data.request_id = req_id;
	os = pb_ostream_from_buffer(data.payload.bytes, sizeof(data.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &resp), "admin encode failed");
	data.payload.size = (pb_size_t)os.bytes_written;

	{
		uint8_t plain[MESHTASTIC_MAX_PAYLOAD_LEN];
		pb_ostream_t dos = pb_ostream_from_buffer(plain, sizeof(plain));

		zassert_true(pb_encode(&dos, meshtastic_Data_fields, &data), "Data encode failed");
		zassert_ok(meshtastic_pki_encrypt(PEER_NODE_ID, PEER_NODE_ID, resp_id, plain,
						  dos.bytes_written, enc, sizeof(enc), &enc_len),
			   "PKC encrypt (forged response) failed");
	}

	whdr->dest = sys_cpu_to_le32(TEST_NODE_ID);
	whdr->src = sys_cpu_to_le32(PEER_NODE_ID);
	whdr->id = sys_cpu_to_le32(resp_id);
	whdr->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	whdr->channel = 0x00U;
	whdr->next_hop = 0U;
	whdr->relay_node = 0U;
	memcpy(wire + MESHTASTIC_HDR_LEN, enc, enc_len);

	mock_lora.send_count = 0U;
	inject_rx_frame(wire, MESHTASTIC_HDR_LEN + (uint32_t)enc_len);
	k_sleep(K_MSEC(50));

	/* Consumed by the client: passkey cached — and NOT refused by the
	 * server gate (an unauthorized-admin NAK would have gone on air). */
	zassert_true(meshtastic_admin_client_have_passkey(PEER_NODE_ID),
		     "response must cache the target's session passkey");
	zassert_equal(mock_lora.send_count, 0U,
		      "a client response must not be NAK'd by the server admin gate");

	/* The mutating op now leaves, carrying the cached passkey. */
	mock_lora.send_count = 0U;
	zassert_ok(meshtastic_admin_client_set_owner(PEER_NODE_ID, "New Name", "NEW"),
		   "set_owner send failed");
	k_sleep(K_MSEC(50));
	zassert_true(mock_lora.send_count >= 1U, "set_owner never reached the radio");
	txh = (const struct meshtastic_wire_header *)mock_lora.last_tx;
	zassert_equal(txh->channel, 0x00U, "set_owner must be PKC too");

	/* set_owner is want_ack: drop the reliable tracking so its retries
	 * cannot pollute a later test's send counts. */
	meshtastic_reliable_reset();
}
#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */

/* ---- agents-dnr4.6: tests for previously-untested-but-implemented admin ops */

ZTEST(admin_pki, test_get_device_metadata_returns_real_metadata)
{
	uint8_t buf[64];
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	size_t len = encode_admin_get_device_metadata_request(buf, sizeof(buf));

	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp),
		     "get_device_metadata must reply");
	zassert_equal(resp.which_payload_variant,
		      meshtastic_AdminMessage_get_device_metadata_response_tag,
		      "wrong response variant");
	zassert_true(strlen(resp.payload_variant.get_device_metadata_response.firmware_version) > 0,
		     "firmware_version must not be empty");
	zassert_true(resp.payload_variant.get_device_metadata_response.hasPKC,
		     "this suite has a real PKI key -- hasPKC must reflect it");
}

ZTEST(admin_pki, test_get_device_connection_status_returns_response)
{
	uint8_t buf[64];
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	size_t len = encode_admin_get_device_connection_status_request(buf, sizeof(buf));

	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp),
		     "get_device_connection_status must reply");
	zassert_equal(resp.which_payload_variant,
		      meshtastic_AdminMessage_get_device_connection_status_response_tag,
		      "wrong response variant");
	/* native_sim builds this suite with neither CONFIG_WIFI nor a WiFi iface,
	 * so has_wifi/has_bluetooth report whatever this Kconfig actually enables
	 * -- the real assertion is that the getter round-trips at all, not a
	 * specific transport's state. */
}

/* Real NVS persistence for ModuleConfig, not just RAM -- §2 of the audit
 * flagged this as untested (the round-trip existed, the durability claim
 * didn't). Mirrors test_lora_config_survives_real_reboot's flush/reload
 * pattern exactly. */
ZTEST(admin_pki, test_set_module_config_mqtt_roundtrips_and_survives_real_reboot)
{
	uint8_t buf[128];
	size_t len;
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	meshtastic_ModuleConfig got = meshtastic_ModuleConfig_init_zero;

	len = encode_admin_set_module_config_mqtt(true, "mqtt.example.org", buf, sizeof(buf));
	zassert_false(send_local_admin_and_pop_reply(buf, len, &resp),
		     "set_module_config emits no AdminMessage response, only a ROUTING ack");
	zassert_ok(settings_save_subtree("meshtastic"), "mqtt flush failed");

	/* Change it again, unflushed -- a real reboot must not see this either. */
	len = encode_admin_set_module_config_mqtt(false, "unflushed.example.org", buf, sizeof(buf));
	zassert_false(send_local_admin_and_pop_reply(buf, len, &resp), "unexpected response");

	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mqtt_tag, &got),
		   "mqtt module reread failed");
	zassert_true(got.payload_variant.mqtt.enabled,
		     "enabled must survive at its FLUSHED value (true), not the unflushed edit");
	zassert_mem_equal(got.payload_variant.mqtt.address, "mqtt.example.org",
			  strlen("mqtt.example.org"),
			  "address must survive a real reboot at its FLUSHED value");

	/* Also prove the getter path reflects the same persisted value an app
	 * would see over the admin channel, not just the config-store internals. */
	len = encode_admin_get_module_config(0U /* MQTT_CONFIG */, buf, sizeof(buf));
	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp), "get_module_config must reply");
	zassert_equal(resp.which_payload_variant,
		      meshtastic_AdminMessage_get_module_config_response_tag,
		      "wrong response variant");
	zassert_true(resp.payload_variant.get_module_config_response.payload_variant.mqtt.enabled,
		     "get_module_config must reflect the persisted value");
}

/* ---- agents-dnr4.8: ModuleConfig.mqtt is wired to the MQTT bridge -----------
 *
 * The bridge itself needs a network stack and is not compiled here (the variants
 * sweep builds it; nothing runs it in sim). What IS testable without a socket is
 * the whole mapping between the persisted section and what the bridge would
 * connect with — meshtastic_mqtt_config.c is pure for exactly that reason — plus
 * the two admin-side rules that guard the credential: redaction on the way out
 * over the mesh, and "sekrit" meaning keep-what-you-have on the way in. */

static size_t encode_admin_set_module_config_mqtt_full(
	const meshtastic_ModuleConfig_MQTTConfig *mqtt, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant = meshtastic_ModuleConfig_mqtt_tag;
	am.payload_variant.set_module_config.payload_variant.mqtt = *mqtt;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* Send a local admin op with want_ack and return the ROUTING error the node
 * answered with (NONE for a clean ACK), skipping unrelated frames queued ahead.
 * Also cancels the reboot every set_module_config schedules, so the suite does
 * not restart under a later test. */
static meshtastic_Routing_Error send_local_admin_and_pop_routing_ex(const uint8_t *admin_bytes,
								    size_t admin_len,
								    bool *reboot_was_scheduled)
{
	meshtastic_MeshPacket pkt = meshtastic_MeshPacket_init_zero;
	struct meshtastic_phoneapi_frame frame;
	meshtastic_FromRadio from;
	meshtastic_Routing routing;
	pb_istream_t is;

	pkt.from = TEST_NODE_ID;
	pkt.id = 0x0AD00002U;
	pkt.want_ack = true;
	pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	pkt.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	memcpy(pkt.decoded.payload.bytes, admin_bytes, admin_len);
	pkt.decoded.payload.size = (pb_size_t)admin_len;

	zassert_true(meshtastic_admin_handle_local(&pkt), "admin_handle_local must consume it");
	if (reboot_was_scheduled != NULL) {
		*reboot_was_scheduled = meshtastic_admin_reboot_scheduled();
	}
	meshtastic_admin_cancel_reboot();

	while (meshtastic_phoneapi_pop_frame(&phone_api, &frame)) {
		from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
		is = pb_istream_from_buffer(frame.data, frame.len);
		zassert_true(pb_decode(&is, meshtastic_FromRadio_fields, &from),
			     "FromRadio decode failed");
		if (from.which_payload_variant != meshtastic_FromRadio_packet_tag ||
		    from.packet.decoded.portnum != meshtastic_PortNum_ROUTING_APP) {
			continue;
		}
		routing = (meshtastic_Routing)meshtastic_Routing_init_zero;
		is = pb_istream_from_buffer(from.packet.decoded.payload.bytes,
					    from.packet.decoded.payload.size);
		zassert_true(pb_decode(&is, meshtastic_Routing_fields, &routing),
			     "Routing decode failed");
		return routing.error_reason;
	}
	zassert_unreachable("a want_ack setter must be answered with a ROUTING frame");
	return meshtastic_Routing_Error_NONE;
}

static meshtastic_Routing_Error send_local_admin_and_pop_routing(const uint8_t *admin_bytes,
								 size_t admin_len)
{
	return send_local_admin_and_pop_routing_ex(admin_bytes, admin_len, NULL);
}

static void read_stored_mqtt(meshtastic_ModuleConfig_MQTTConfig *out)
{
	meshtastic_ModuleConfig got = meshtastic_ModuleConfig_init_zero;

	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mqtt_tag, &got),
		   "mqtt module read failed");
	*out = got.payload_variant.mqtt;
}

ZTEST(admin_pki, test_mqtt_settings_resolve_empty_address_uses_build_defaults)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	struct meshtastic_mqtt_settings s;
	struct meshtastic_mqtt_settings z;

	cfg.enabled = true;
	meshtastic_mqtt_settings_resolve(&cfg, &s);

	zassert_true(s.enabled, "enabled passes through");
	zassert_false(s.address_is_custom, "an empty address is the build default, not custom");
	zassert_str_equal(s.host, MESHTASTIC_MQTT_FALLBACK_HOST, "host falls back to Kconfig");
	zassert_equal(s.port, MESHTASTIC_MQTT_FALLBACK_PORT, "port falls back to Kconfig");
	zassert_str_equal(s.username, MESHTASTIC_MQTT_FALLBACK_USERNAME, "username fallback");
	zassert_str_equal(s.password, MESHTASTIC_MQTT_FALLBACK_PASSWORD, "password fallback");
	zassert_str_equal(s.root, MESHTASTIC_MQTT_FALLBACK_ROOT, "root falls back to Kconfig");
	zassert_equal(s.default_broker,
		      strcmp(MESHTASTIC_MQTT_FALLBACK_HOST, MESHTASTIC_MQTT_DEFAULT_BROKER) == 0,
		      "default_broker reflects whether the fallback IS the public broker");
	zassert_equal(s.map_publish_interval_secs,
		      MAX(MESHTASTIC_MQTT_FALLBACK_MAP_INTERVAL_SEC,
			  MESHTASTIC_MQTT_MAP_INTERVAL_MIN_SEC),
		      "map interval falls back to Kconfig");
	zassert_equal(s.map_position_precision, MESHTASTIC_MQTT_FALLBACK_MAP_PRECISION,
		      "map precision falls back to Kconfig");

	/* A NULL section is the all-zero section: same fallbacks, just disabled. */
	meshtastic_mqtt_settings_resolve(NULL, &z);
	zassert_false(z.enabled, "NULL section resolves disabled");
	zassert_str_equal(z.host, s.host, "NULL section takes the same host fallback");
	zassert_equal(z.port, s.port, "NULL section takes the same port fallback");
}

ZTEST(admin_pki, test_mqtt_settings_resolve_custom_address_honours_creds_and_port)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	struct meshtastic_mqtt_settings s;

	cfg.enabled = true;
	strcpy(cfg.address, "broker.lan:8884");
	strcpy(cfg.username, "anon");
	/* password deliberately left empty: a custom broker's creds are taken AS
	 * GIVEN (reference PubSubConfig), never back-filled from Kconfig. */
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_true(s.address_is_custom, "a non-empty address is custom");
	zassert_str_equal(s.host, "broker.lan", "host is the part before the colon");
	zassert_equal(s.port, 8884U, "port is the part after the colon");
	zassert_str_equal(s.username, "anon", "custom-broker username is honoured");
	zassert_str_equal(s.password, "", "custom-broker EMPTY password is honoured, not defaulted");
	zassert_false(s.default_broker, "broker.lan is not the public broker");
	zassert_str_equal(s.tls_hostname, "broker.lan", "SNI names the custom host");

	/* No port in the address: 1883, or 8883 once TLS is asked for. */
	strcpy(cfg.address, "broker.lan");
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.port, MESHTASTIC_MQTT_PORT_PLAIN, "plain default port");
	cfg.tls_enabled = true;
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.port, MESHTASTIC_MQTT_PORT_TLS, "TLS default port");
	zassert_true(s.tls_enabled, "tls_enabled passes through");

	/* A junk port suffix keeps the default (reference: toInt() out of range). */
	strcpy(cfg.address, "broker.lan:99999");
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.port, MESHTASTIC_MQTT_PORT_TLS, "out-of-range port is ignored");
	zassert_str_equal(s.host, "broker.lan", "host still split off a bad port");
	strcpy(cfg.address, "broker.lan:");
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.port, MESHTASTIC_MQTT_PORT_TLS, "empty port is ignored");
	strcpy(cfg.address, "broker.lan:12x");
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.port, MESHTASTIC_MQTT_PORT_TLS, "non-numeric port is ignored");

	/* The public broker named explicitly is still "the default broker" for the
	 * gates keyed on it (portnum skip list, OK_TO_MQTT consent). */
	cfg.tls_enabled = false;
	strcpy(cfg.address, MESHTASTIC_MQTT_DEFAULT_BROKER);
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_true(s.address_is_custom, "explicit address is custom");
	zassert_true(s.default_broker, "...but it IS the public broker");

	strcpy(cfg.root, "msh/US");
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_str_equal(s.root, "msh/US", "a set root wins over the Kconfig root");
}

ZTEST(admin_pki, test_mqtt_settings_resolve_map_report_clamps_like_upstream)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	struct meshtastic_mqtt_settings s;

	cfg.map_reporting_enabled = true;
	cfg.has_map_report_settings = true;
	cfg.map_report_settings.publish_interval_secs = 10U;
	cfg.map_report_settings.position_precision = 3U;
	cfg.map_report_settings.should_report_location = true;
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_true(s.map_reporting_enabled, "map flag passes through");
	zassert_true(s.map_should_report_location, "location opt-in passes through");
	zassert_equal(s.map_publish_interval_secs, MESHTASTIC_MQTT_MAP_INTERVAL_MIN_SEC,
		      "interval below the floor is raised to it");
	zassert_equal(s.map_position_precision, MESHTASTIC_MQTT_FALLBACK_MAP_PRECISION,
		      "precision outside 12..15 falls back (reference clamps to default)");

	cfg.map_report_settings.publish_interval_secs = 3600U;
	cfg.map_report_settings.position_precision = 15U;
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.map_publish_interval_secs, 3600U, "in-range interval kept");
	zassert_equal(s.map_position_precision, 15U, "in-range precision kept");

	/* No sub-message at all: interval/precision fall back, and the location
	 * opt-in inherits the build's own map-report opt-in (what the store seeds). */
	cfg.has_map_report_settings = false;
	meshtastic_mqtt_settings_resolve(&cfg, &s);
	zassert_equal(s.map_publish_interval_secs,
		      MAX(MESHTASTIC_MQTT_FALLBACK_MAP_INTERVAL_SEC,
			  MESHTASTIC_MQTT_MAP_INTERVAL_MIN_SEC),
		      "absent sub-message: interval fallback");
	zassert_equal(s.map_should_report_location, IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT),
		      "absent sub-message: location opt-in follows the build");
}

ZTEST(admin_pki, test_mqtt_config_validate_refuses_tls_without_transport)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;

	zassert_equal(meshtastic_mqtt_config_validate(NULL), -EINVAL, "NULL section");
	zassert_ok(meshtastic_mqtt_config_validate(&cfg), "a plaintext section is always fine");

	cfg.tls_enabled = true;
	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_TLS)) {
		zassert_ok(meshtastic_mqtt_config_validate(&cfg), "TLS compiled in: accepted");
	} else {
		zassert_equal(meshtastic_mqtt_config_validate(&cfg), -ENOTSUP,
			      "no TLS transport in this image: refused, never downgraded");
	}
}

ZTEST(admin_pki, test_module_config_password_is_redacted_for_mesh_not_for_phone)
{
	meshtastic_ModuleConfig m = meshtastic_ModuleConfig_init_zero;
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	uint8_t buf[256];
	size_t len;

	/* The substitution the remote get path applies (reference secretReserved). */
	m.which_payload_variant = meshtastic_ModuleConfig_mqtt_tag;
	strcpy(m.payload_variant.mqtt.address, "broker.lan");
	strcpy(m.payload_variant.mqtt.password, "hunter2");
	meshtastic_admin_redact_module_config_for_mesh(&m);
	zassert_str_equal(m.payload_variant.mqtt.password, MESHTASTIC_MQTT_SECRET_RESERVED,
			  "mesh-bound mqtt password is replaced by the reserved word");
	zassert_str_equal(m.payload_variant.mqtt.address, "broker.lan",
			  "nothing but the password is touched");

	/* A section with no secret passes through untouched. */
	m = (meshtastic_ModuleConfig)meshtastic_ModuleConfig_init_zero;
	m.which_payload_variant = meshtastic_ModuleConfig_serial_tag;
	m.payload_variant.serial.enabled = true;
	meshtastic_admin_redact_module_config_for_mesh(&m);
	zassert_true(m.payload_variant.serial.enabled, "serial section untouched");

	/* The LOCAL path — the app that edits the credential — sees the real value
	 * end to end: set it, then get it back over the PhoneAPI. */
	cfg.enabled = true;
	strcpy(cfg.address, "broker.lan");
	strcpy(cfg.password, "hunter2");
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "set must ACK clean");
	len = encode_admin_get_module_config(0U /* MQTT_CONFIG */, buf, sizeof(buf));
	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp), "get_module_config must reply");
	zassert_str_equal(resp.payload_variant.get_module_config_response.payload_variant.mqtt.password,
			  "hunter2", "the phone gets the real password, not the reserved word");
}

ZTEST(admin_pki, test_set_module_config_mqtt_sekrit_keeps_stored_password)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	meshtastic_ModuleConfig_MQTTConfig stored;
	uint8_t buf[256];
	size_t len;

	cfg.enabled = true;
	strcpy(cfg.address, "broker.lan");
	strcpy(cfg.password, "hunter2");
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "initial set must ACK clean");

	/* An app round-tripping a redacted get writes "sekrit" back with its edits:
	 * the edit lands, the credential it never saw survives (reference writeSecret). */
	strcpy(cfg.address, "other.lan:8884");
	strcpy(cfg.password, MESHTASTIC_MQTT_SECRET_RESERVED);
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "sekrit set must ACK clean");
	read_stored_mqtt(&stored);
	zassert_str_equal(stored.address, "other.lan:8884", "the non-secret edit landed");
	zassert_str_equal(stored.password, "hunter2", "sekrit means keep the stored password");

	/* A real new password does replace it. */
	strcpy(cfg.password, "newpass");
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "new-password set must ACK clean");
	read_stored_mqtt(&stored);
	zassert_str_equal(stored.password, "newpass", "a real password replaces the stored one");
}

ZTEST(admin_pki, test_set_module_config_mqtt_tls_refused_without_transport)
{
	meshtastic_ModuleConfig_MQTTConfig cfg = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	meshtastic_ModuleConfig_MQTTConfig stored;
	uint8_t buf[256];
	size_t len;

	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_TLS)) {
		ztest_test_skip();
		return;
	}

	cfg.enabled = true;
	strcpy(cfg.address, "keep.lan");
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "plaintext set must ACK clean");

	/* Reference MQTT::isValidConfig: "tls_enabled unsupported on this node" is a
	 * refusal, and a refusal must leave the previous section intact — silently
	 * storing it would have the bridge connect in plaintext to a broker the
	 * operator asked to reach over TLS. */
	strcpy(cfg.address, "tls.lan");
	cfg.tls_enabled = true;
	len = encode_admin_set_module_config_mqtt_full(&cfg, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len),
		      meshtastic_Routing_Error_BAD_REQUEST, "TLS on a no-TLS image must NAK");
	read_stored_mqtt(&stored);
	zassert_str_equal(stored.address, "keep.lan", "refused set must not touch the store");
	zassert_false(stored.tls_enabled, "refused set must not touch the store");
}

/* ---- agents-dnr4.26: ModuleConfig.statusmessage applies live, no reboot ----- */

#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
static size_t encode_admin_set_module_config_status(const char *status, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_ModuleConfig *mc = &am.payload_variant.set_module_config;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	mc->which_payload_variant = meshtastic_ModuleConfig_statusmessage_tag;
	strncpy(mc->payload_variant.statusmessage.node_status, status,
		sizeof(mc->payload_variant.statusmessage.node_status) - 1U);
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

ZTEST(admin_pki, test_set_module_config_statusmessage_applies_live_without_reboot)
{
	meshtastic_ModuleConfig_MQTTConfig mqtt = meshtastic_ModuleConfig_MQTTConfig_init_zero;
	char status[80];
	uint8_t buf[256];
	size_t len;
	bool rebooting = true;

	/* Reference AdminModule: shouldReboot = false for this section alone
	 * among the module configs -- the module re-reads it live. */
	len = encode_admin_set_module_config_status("on call", buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing_ex(buf, len, &rebooting),
		      meshtastic_Routing_Error_NONE, "set must ACK clean");
	zassert_false(rebooting, "a statusmessage set must NOT schedule a reboot");
	zassert_equal(meshtastic_statusmessage_get(status, sizeof(status)), strlen("on call"),
		      "the module sees the new status immediately");
	zassert_str_equal(status, "on call", "");

	/* Contrast: a section nobody applies live still reboots. */
	mqtt.enabled = false;
	strcpy(mqtt.address, "contrast.lan");
	len = encode_admin_set_module_config_mqtt_full(&mqtt, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing_ex(buf, len, &rebooting),
		      meshtastic_Routing_Error_NONE, "mqtt set must ACK clean");
	zassert_true(rebooting, "an mqtt set still schedules the reboot it needs");

	/* Clear it so the announce this armed cannot fire into a later test. */
	len = encode_admin_set_module_config_status("", buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "clear must ACK clean");
	zassert_equal(meshtastic_statusmessage_get(status, sizeof(status)), 0U, "cleared live");
}
#endif /* CONFIG_MESHTASTIC_STATUSMESSAGE */

/* ---- agents-dnr4.19: ModuleConfig.neighbor_info applies live, no reboot ---- */

#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
static size_t encode_admin_set_module_config_neighbor(bool enabled, uint32_t interval, bool lora,
						      uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_ModuleConfig *mc = &am.payload_variant.set_module_config;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	mc->which_payload_variant = meshtastic_ModuleConfig_neighbor_info_tag;
	mc->payload_variant.neighbor_info.enabled = enabled;
	mc->payload_variant.neighbor_info.update_interval = interval;
	mc->payload_variant.neighbor_info.transmit_over_lora = lora;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

ZTEST(admin_pki, test_set_module_config_neighbor_info_applies_live_without_reboot)
{
	struct meshtastic_neighborinfo_settings s;
	uint8_t buf[256];
	size_t len;
	bool rebooting = true;

	len = encode_admin_set_module_config_neighbor(true, 0U, false, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing_ex(buf, len, &rebooting),
		      meshtastic_Routing_Error_NONE, "set must ACK clean");
	zassert_false(rebooting, "a neighbor_info set must NOT schedule a reboot");
	meshtastic_neighborinfo_settings(&s);
	zassert_true(s.enabled, "the module sees it immediately");
	zassert_false(s.transmit_over_lora, "");
	zassert_equal(s.interval_secs, CONFIG_MESHTASTIC_NEIGHBORINFO_INTERVAL_SEC,
		      "0 resolves to the compiled default");

	/* Disable again so its cycle cannot fire into a later test. */
	len = encode_admin_set_module_config_neighbor(false, 0U, false, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "disable must ACK clean");
	meshtastic_neighborinfo_settings(&s);
	zassert_false(s.enabled, "disabled live");
}
#endif /* CONFIG_MESHTASTIC_NEIGHBORINFO */

/* ---- agents-dnr4.13: manual key verification, both roles ----------------------- */

#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
#include "meshtastic_keyverify.h"

#define KV_PEER 0x0B00C0DEU

static void kv_sha256(const uint8_t *in, size_t len, uint8_t out[32])
{
	size_t olen = 0;

	zassert_equal(psa_hash_compute(PSA_ALG_SHA_256, in, len, out, 32, &olen), PSA_SUCCESS, "");
	zassert_equal(olen, 32U, "");
}

/* The reference's H1 = SHA256(number || nonce || initiator || responder || PK_i || PK_r),
 * little-endian integers, and hash2 = SHA256(nonce || H1). */
static void kv_hashes(uint32_t number, uint64_t nonce, uint32_t initiator, uint32_t responder,
		      const uint8_t *pk_i, const uint8_t *pk_r, uint8_t h1[32], uint8_t h2[32])
{
	uint8_t buf[84];
	uint8_t buf2[40];

	sys_put_le32(number, buf);
	sys_put_le64(nonce, buf + 4);
	sys_put_le32(initiator, buf + 12);
	sys_put_le32(responder, buf + 16);
	memcpy(buf + 20, pk_i, 32);
	memcpy(buf + 52, pk_r, 32);
	kv_sha256(buf, sizeof(buf), h1);
	sys_put_le64(nonce, buf2);
	memcpy(buf2 + 8, h1, 32);
	kv_sha256(buf2, sizeof(buf2), h2);
}

static void kv_code(const uint8_t h1[32], char out[10])
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

static size_t kv_encode(uint64_t nonce, const uint8_t *hash1, const uint8_t *hash2, uint8_t *buf,
			size_t cap)
{
	meshtastic_KeyVerification m = meshtastic_KeyVerification_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	m.nonce = nonce;
	if (hash1 != NULL) {
		m.hash1.size = 32;
		memcpy(m.hash1.bytes, hash1, 32);
	}
	if (hash2 != NULL) {
		m.hash2.size = 32;
		memcpy(m.hash2.bytes, hash2, 32);
	}
	zassert_true(pb_encode(&os, meshtastic_KeyVerification_fields, &m), "kv encode failed");
	return os.bytes_written;
}

/* A KeyVerification from KV_PEER to us on the primary channel (the bootstrap
 * envelope). */
static void kv_inject_channel(uint32_t id, uint64_t nonce, const uint8_t *hash1,
			      const uint8_t *hash2, bool want_response)
{
	uint8_t payload[128];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet pkt = {
		.from = KV_PEER,
		.to = TEST_NODE_ID,
		.id = id,
		.portnum = MESHTASTIC_PORT_KEY_VERIFICATION,
		.want_response = want_response,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
		/* The destination is our own node, whose key the NodeDB holds, so
		 * without this the builder would PKC-encrypt the frame under OUR
		 * key pair and the receiver, trying KV_PEER's shared secret, would
		 * drop it before any log line. no_pkc is the same flag the module
		 * uses for its own bootstrap reply. */
		.no_pkc = true,
	};

	pkt.payload = payload;
	pkt.payload_len = kv_encode(nonce, hash1, hash2, payload, sizeof(payload));
	zassert_ok(meshtastic_build_wire_packet(&pkt, wire, &wire_len), "build failed");
	inject_rx_frame(wire, wire_len);
	k_sleep(K_MSEC(50));
}

/* The same, PKC-encrypted "from" KV_PEER via the ECDH-symmetry trick (see
 * inject_pkc_admin): the node decrypts a packet from KV_PEER with the same
 * shared key our own encrypt-to-KV_PEER produces. */
static void kv_inject_pkc(uint32_t id, uint64_t nonce, const uint8_t *hash1, const uint8_t *hash2,
			  bool want_response)
{
	meshtastic_Data data = meshtastic_Data_init_zero;
	uint8_t plain[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t enc[MESHTASTIC_MAX_PAYLOAD_LEN + MESHTASTIC_PKI_OVERHEAD];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	struct meshtastic_wire_header *whdr = (struct meshtastic_wire_header *)wire;
	pb_ostream_t os;
	size_t enc_len = 0;

	data.portnum = (meshtastic_PortNum)MESHTASTIC_PORT_KEY_VERIFICATION;
	data.want_response = want_response;
	data.payload.size = (pb_size_t)kv_encode(nonce, hash1, hash2, data.payload.bytes,
						  sizeof(data.payload.bytes));
	os = pb_ostream_from_buffer(plain, sizeof(plain));
	zassert_true(pb_encode(&os, meshtastic_Data_fields, &data), "Data encode failed");
	zassert_ok(meshtastic_pki_encrypt(KV_PEER, KV_PEER, id, plain, os.bytes_written, enc,
					  sizeof(enc), &enc_len),
		   "PKC encrypt failed");
	whdr->dest = sys_cpu_to_le32(TEST_NODE_ID);
	whdr->src = sys_cpu_to_le32(KV_PEER);
	whdr->id = sys_cpu_to_le32(id);
	whdr->flags = 3U | (3U << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	whdr->channel = 0x00U;
	whdr->next_hop = 0U;
	whdr->relay_node = 0U;
	memcpy(wire + MESHTASTIC_HDR_LEN, enc, enc_len);
	inject_rx_frame(wire, MESHTASTIC_HDR_LEN + (uint32_t)enc_len);
	k_sleep(K_MSEC(50));
}

/* The next ClientNotification the phone would see, skipping other frames. */
static bool kv_pop_notification(meshtastic_ClientNotification *cn)
{
	struct meshtastic_phoneapi_frame frame;
	meshtastic_FromRadio from;

	while (meshtastic_phoneapi_pop_frame(&phone_api, &frame)) {
		pb_istream_t is = pb_istream_from_buffer(frame.data, frame.len);

		from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
		zassert_true(pb_decode(&is, meshtastic_FromRadio_fields, &from), "FromRadio decode");
		if (from.which_payload_variant == meshtastic_FromRadio_clientNotification_tag) {
			*cn = from.clientNotification;
			return true;
		}
	}
	return false;
}

/* Decode the last frame the node transmitted as a KeyVerification; reports
 * whether it went PKC (channel byte 0x00). Only a channel-encrypted frame can
 * be decoded here. */
static bool kv_last_tx(meshtastic_KeyVerification *out, bool *pkc, uint32_t *to)
{
	const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)mock_lora.last_tx;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_istream_t is;

	*pkc = (h->channel == 0x00U);
	*to = sys_le32_to_cpu(h->dest);
	if (*pkc) {
		return false;
	}
	if (meshtastic_decode_wire_packet(mock_lora.last_tx, mock_lora.last_tx_len, 0, 0, &decoded,
					  payload, sizeof(payload)) != 0) {
		return false;
	}
	*out = (meshtastic_KeyVerification)meshtastic_KeyVerification_init_zero;
	is = pb_istream_from_buffer(decoded.payload, decoded.payload_len);
	return decoded.portnum == MESHTASTIC_PORT_KEY_VERIFICATION &&
	       pb_decode(&is, meshtastic_KeyVerification_fields, out);
}

/* Any of the mock's recent frames a channel-encrypted port-12 message carrying
 * this nonce? (A PKC frame cannot be a bootstrap reply and is skipped.) */
static bool kv_find_tx(uint64_t nonce, meshtastic_KeyVerification *out)
{
	uint32_t n = MIN(mock_lora.ring_head, MOCK_TX_RING);

	for (uint32_t i = 1U; i <= n; i++) {
		uint32_t slot = (mock_lora.ring_head - i) % MOCK_TX_RING;
		const struct meshtastic_wire_header *h =
			(const struct meshtastic_wire_header *)mock_lora.ring[slot];
		struct meshtastic_packet decoded;
		uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
		pb_istream_t is;

		if (h->channel == 0x00U ||
		    meshtastic_decode_wire_packet(mock_lora.ring[slot], mock_lora.ring_len[slot], 0,
						  0, &decoded, payload, sizeof(payload)) != 0 ||
		    decoded.portnum != MESHTASTIC_PORT_KEY_VERIFICATION) {
			continue;
		}
		*out = (meshtastic_KeyVerification)meshtastic_KeyVerification_init_zero;
		is = pb_istream_from_buffer(decoded.payload, decoded.payload_len);
		if (pb_decode(&is, meshtastic_KeyVerification_fields, out) && out->nonce == nonce) {
			return true;
		}
	}
	return false;
}

static void kv_fresh(void)
{
	meshtastic_keyverify_reset();
	(void)meshtastic_nodedb_forget(KV_PEER);
	meshtastic_phoneapi_reset(&phone_api);
	mock_lora.send_count = 0U;
}

ZTEST(admin_pki, test_keyverify_responder_bootstrap_handshake)
{
	uint8_t peer_pub[32];
	uint8_t our_pub[32];
	uint8_t h1[32], h2[32], bad[32];
	char code[10];
	meshtastic_KeyVerification m2;
	meshtastic_ClientNotification cn;
	struct meshtastic_keyverify_status st;
	struct meshtastic_nodedb_node node;
	const uint64_t nonce = 0x1122334455667788ULL;
	bool pkc;
	uint32_t to;

	kv_fresh();
	gen_x25519_pubkey(peer_pub);
	zassert_equal(meshtastic_pki_get_public_key(our_pub), 32U, "");
	zassert_not_equal(meshtastic_nodedb_get(KV_PEER, &node), 0, "bootstrap: the peer is unknown");

	/* M1, channel-encrypted, carrying the peer's key in hash1. */
	kv_inject_channel(0x4B000001U, nonce, peer_pub, NULL, true);
	zassert_equal(mock_lora.send_count, 1U, "M2 went out");
	zassert_true(kv_last_tx(&m2, &pkc, &to), "M2 is channel-encrypted (the peer lacks our key)");
	zassert_false(pkc, "");
	zassert_equal(to, KV_PEER, "");
	zassert_equal(m2.nonce, nonce, "same nonce");
	zassert_equal(m2.hash1.size, 32U, "");
	zassert_mem_equal(m2.hash1.bytes, our_pub, 32, "M2 carries OUR key for the peer to bootstrap");
	zassert_equal(m2.hash2.size, 32U, "");

	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1, "");
	zassert_true(st.security_number >= 1U && st.security_number <= 999999U, "6-digit number");
	zassert_true(kv_pop_notification(&cn), "the phone is told the number to show");
	zassert_equal(cn.which_payload_variant,
		      meshtastic_ClientNotification_key_verification_number_inform_tag, "");
	zassert_equal(cn.payload_variant.key_verification_number_inform.security_number,
		      st.security_number, "");
	zassert_equal(cn.payload_variant.key_verification_number_inform.nonce, nonce, "");

	/* The peer (this test) is told the number out of band and reproduces H1. */
	kv_hashes(st.security_number, nonce, KV_PEER, TEST_NODE_ID, peer_pub, our_pub, h1, h2);
	zassert_mem_equal(m2.hash2.bytes, h2, 32, "hash2 = SHA256(nonce || H1) as the reference");

	/* A wrong H1 (wrong number) is ignored; a channel-encrypted M3 too. */
	memcpy(bad, h1, 32);
	bad[0] ^= 0xFFU;
	kv_inject_pkc(0x4B000002U, nonce, bad, NULL, true);
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1, "wrong H1: no advance");
	kv_inject_channel(0x4B000003U, nonce, h1, NULL, true);
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1,
		      "a channel-encrypted M3 proves nothing: no advance");

	/* M3, PKC (the pending key makes the node able to decrypt it). */
	kv_inject_pkc(0x4B000004U, nonce, h1, NULL, true);
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER, "H1 matched");
	kv_code(h1, code);
	zassert_str_equal(st.code, code, "the 8-character code the user compares");
	zassert_true(kv_pop_notification(&cn), "final prompt");
	zassert_equal(cn.which_payload_variant, meshtastic_ClientNotification_key_verification_final_tag,
		      "");
	zassert_false(cn.payload_variant.key_verification_final.isSender, "");
	zassert_str_equal(cn.payload_variant.key_verification_final.verification_characters, code, "");

	/* Accept: the pending key is committed, flagged verified, and our NodeInfo
	 * goes to the peer. */
	mock_lora.send_count = 0U;
	zassert_ok(meshtastic_keyverify_accept(nonce), "");
	zassert_ok(meshtastic_nodedb_get(KV_PEER, &node), "the peer is in the NodeDB now");
	zassert_true(node.is_key_manually_verified, "flagged verified");
	zassert_equal(node.public_key_len, 32U, "");
	zassert_mem_equal(node.public_key, peer_pub, 32, "with the key learned in the handshake");
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_IDLE, "");
	zassert_true(mock_lora.send_count >= 1U, "our NodeInfo went to the peer");
	kv_fresh();
}

ZTEST(admin_pki, test_keyverify_initiator_handshake_with_a_known_peer)
{
	uint8_t peer_pub[32];
	uint8_t our_pub[32];
	uint8_t h1[32], h2[32];
	char code[10];
	meshtastic_KeyVerification m;
	meshtastic_ClientNotification cn;
	struct meshtastic_keyverify_status st;
	struct meshtastic_nodedb_node node;
	const uint32_t number = 424242U;
	bool pkc;
	uint32_t to;

	kv_fresh();
	gen_x25519_pubkey(peer_pub);
	zassert_equal(meshtastic_pki_get_public_key(our_pub), 32U, "");
	/* Known peer: its key came in a NodeInfo. */
	{
		meshtastic_User user = meshtastic_User_init_zero;
		uint8_t buf[128];
		pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
		struct meshtastic_packet ni = {
			.from = KV_PEER,
			.to = MESHTASTIC_NODE_BROADCAST,
			.portnum = MESHTASTIC_PORT_NODEINFO,
			.channel_index = meshtastic_channels_primary_index(),
		};

		user.public_key.size = 32;
		memcpy(user.public_key.bytes, peer_pub, 32);
		zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "");
		ni.payload = buf;
		ni.payload_len = os.bytes_written;
		meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
	}
	mock_lora.send_count = 0U;

	zassert_ok(meshtastic_keyverify_start(KV_PEER), "");
	zassert_equal(meshtastic_keyverify_start(KV_PEER), -EBUSY, "one session at a time");
	k_sleep(K_MSEC(50));
	zassert_equal(mock_lora.send_count, 1U, "M1 went out");
	(void)kv_last_tx(&m, &pkc, &to);
	zassert_true(pkc, "the peer's key is known: M1 is PKC");
	zassert_equal(to, KV_PEER, "");
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED, "");
	zassert_not_equal(st.nonce, 0ULL, "a fresh nonce");

	/* The peer (this test) answers M2: its number, its key, hash2. */
	kv_hashes(number, st.nonce, TEST_NODE_ID, KV_PEER, our_pub, peer_pub, h1, h2);
	kv_inject_pkc(0x4B000011U, st.nonce, peer_pub, h2, true);
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER, "");
	zassert_true(kv_pop_notification(&cn), "the phone is asked for the number");
	zassert_equal(cn.which_payload_variant,
		      meshtastic_ClientNotification_key_verification_number_request_tag, "");

	/* A wrong number does not reproduce hash2: nothing leaves the node. */
	mock_lora.send_count = 0U;
	zassert_equal(meshtastic_keyverify_provide_number(st.nonce, number + 1U), -EACCES, "");
	zassert_equal(meshtastic_keyverify_provide_number(st.nonce + 1U, number), -EINVAL, "stale nonce");
	zassert_equal(mock_lora.send_count, 0U, "nothing sent on a wrong number");
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER, "still waiting");

	/* The right number: M3 (H1) goes out PKC, the final prompt shows the code. */
	zassert_ok(meshtastic_keyverify_provide_number(st.nonce, number), "");
	k_sleep(K_MSEC(50));
	zassert_equal(mock_lora.send_count, 1U, "M3 went out");
	(void)kv_last_tx(&m, &pkc, &to);
	zassert_true(pkc, "M3 is PKC: proves we hold our private key");
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER, "");
	kv_code(h1, code);
	zassert_str_equal(st.code, code, "same code the peer derives");
	zassert_true(kv_pop_notification(&cn), "");
	zassert_equal(cn.which_payload_variant, meshtastic_ClientNotification_key_verification_final_tag,
		      "");
	zassert_true(cn.payload_variant.key_verification_final.isSender, "");

	zassert_ok(meshtastic_keyverify_accept(st.nonce), "");
	zassert_ok(meshtastic_nodedb_get(KV_PEER, &node), "");
	zassert_true(node.is_key_manually_verified, "flagged verified");
	zassert_mem_equal(node.public_key, peer_pub, 32, "the known key, kept");
	kv_fresh();
}

ZTEST(admin_pki, test_keyverify_admin_path_timeout_and_cooldown)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	struct meshtastic_keyverify_status st;
	uint8_t peer_pub[32];
	uint8_t buf[256];
	pb_ostream_t os;

	kv_fresh();
	gen_x25519_pubkey(peer_pub);

	/* The app initiates over admin (local only). */
	am.which_payload_variant = meshtastic_AdminMessage_key_verification_tag;
	am.payload_variant.key_verification.message_type =
		meshtastic_KeyVerificationAdmin_MessageType_INITIATE_VERIFICATION;
	am.payload_variant.key_verification.remote_nodenum = KV_PEER;
	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "");
	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      meshtastic_Routing_Error_NONE, "");
	k_sleep(K_MSEC(50));
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED, "admin started it");
	zassert_true(mock_lora.send_count >= 1U, "M1 (bootstrap, channel-encrypted) went out");

	/* DO_NOT_VERIFY drops it. */
	am.payload_variant.key_verification.message_type =
		meshtastic_KeyVerificationAdmin_MessageType_DO_NOT_VERIFY;
	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "");
	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      meshtastic_Routing_Error_NONE, "");
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_IDLE, "rejected");

	/* Idle timeout: 60 s with no progress. */
	zassert_ok(meshtastic_keyverify_start(KV_PEER), "");
	k_sleep(K_SECONDS(61));
	meshtastic_keyverify_status(&st);
	zassert_equal(st.state, MESHTASTIC_KEYVERIFY_IDLE, "timed out");

	/* Remote cooldown: a session opened by a peer, then ended, blocks another
	 * remote request for a minute.
	 * A request is judged answered by a port-12 reply carrying THAT request's
	 * nonce among the mock's recent frames, not by a TX count or the last frame:
	 * the first frame from a never-heard peer also triggers a NodeInfo request
	 * (whether it does depends on the request throttle's state left by earlier
	 * tests), and the queue is free to put it on the air after the reply. Two
	 * back-to-back originated frames also need more than the inject helper's
	 * 50 ms to both reach the mock, hence the settle after each inject. */
	{
		meshtastic_KeyVerification m2;

		kv_inject_channel(0x4B000021U, 0x99ULL, peer_pub, NULL, true);
		k_sleep(K_MSEC(600));
		zassert_true(kv_find_tx(0x99ULL, &m2), "first request answered");
		meshtastic_keyverify_reject();
		kv_inject_channel(0x4B000022U, 0x9AULL, peer_pub, NULL, true);
		k_sleep(K_MSEC(600));
		zassert_false(kv_find_tx(0x9AULL, &m2), "inside the cooldown: not answered");
		k_sleep(K_SECONDS(61));
		kv_inject_channel(0x4B000023U, 0x9BULL, peer_pub, NULL, true);
		k_sleep(K_MSEC(600));
		zassert_true(kv_find_tx(0x9BULL, &m2), "after the cooldown: answered");
	}
	kv_fresh();
}
#endif /* CONFIG_MESHTASTIC_KEYVERIFY */

/* ---- agents-dnr4.12: enter_dfu_mode_request ---- */

/* native_sim has no bootloader to enter (like the ESP32-S3, where the ROM
 * download mode needs a GPIO0 strap). The request is refused with a NAK, not
 * dropped in silence: the phone must learn nothing happened. The nRF52 arm --
 * the deferred hand-off to meshtastic_dfu_enter() -- only builds on that SoC
 * and is compile-checked on the XIAO image. */
ZTEST(admin_pki, test_enter_dfu_mode_is_refused_where_there_is_no_bootloader_path)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	uint8_t buf[32];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	am.which_payload_variant = meshtastic_AdminMessage_enter_dfu_mode_request_tag;
	am.payload_variant.enter_dfu_mode_request = true;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "");
	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      IS_ENABLED(CONFIG_MESHTASTIC_DFU_TRIGGER) ? meshtastic_Routing_Error_NONE
							       : meshtastic_Routing_Error_BAD_REQUEST,
		      "no software DFU path here: NAK");
}

/* ---- agents-dnr4.9: ModuleConfig.serial -- no SerialModule on this port ---- */

static size_t encode_admin_set_serial(bool enabled, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_ModuleConfig_SerialConfig *c =
		&am.payload_variant.set_module_config.payload_variant.serial;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant =
		meshtastic_ModuleConfig_serial_tag;
	c->enabled = enabled;
	c->mode = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG;
	c->baud = meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

/* meshtastic_serial.c is the PhoneAPI-over-UART transport (the reference's
 * SerialConsole), not the reference's SerialModule this section configures. A
 * write that would turn that module on is refused, not stored as if it worked;
 * a disabled section is inert and accepted. */
ZTEST(admin_pki, test_set_module_config_serial_enabled_is_refused)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	uint8_t buf[256];
	size_t len;

	len = encode_admin_set_serial(true, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_BAD_REQUEST,
		      "serial.enabled on a port with no SerialModule: NAK");
	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_serial_tag, &mod), "");
	zassert_false(mod.payload_variant.serial.enabled, "the refused set left the store alone");

	len = encode_admin_set_serial(false, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "a disabled section is inert and accepted");
}

/* ---- agents-dnr4.17: external_notification applies live; buzzer/ringtone refused ---- */

#if defined(CONFIG_MESHTASTIC_EXTNOTIFY)
static size_t encode_admin_set_extnotify(bool enabled, bool buzzer, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_ModuleConfig_ExternalNotificationConfig *c =
		&am.payload_variant.set_module_config.payload_variant.external_notification;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant =
		meshtastic_ModuleConfig_external_notification_tag;
	c->enabled = enabled;
	c->active = true;
	c->alert_message = true;
	c->output_ms = 500U;
	c->alert_message_buzzer = buzzer;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

ZTEST(admin_pki, test_set_module_config_extnotify_applies_live_and_refuses_buzzer)
{
	struct meshtastic_extnotify_settings s;
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	uint8_t buf[512];
	pb_ostream_t os;
	size_t len;
	bool rebooting = true;

	len = encode_admin_set_extnotify(true, false, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing_ex(buf, len, &rebooting),
		      meshtastic_Routing_Error_NONE, "LED-only config: accepted");
	zassert_false(rebooting, "applied live");
	meshtastic_extnotify_settings(&s);
	zassert_true(s.enabled, "");
	zassert_equal(s.output_ms, 500U, "");

	len = encode_admin_set_extnotify(true, true, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_BAD_REQUEST,
		      "a buzzer alert on a board with no buzzer: NAK, not store-and-ignore");
	meshtastic_extnotify_settings(&s);
	zassert_equal(s.output_ms, 500U, "the refused set left the store alone");

	/* A ringtone plays on a buzzer: refused the same way. */
	am.which_payload_variant = meshtastic_AdminMessage_set_ringtone_message_tag;
	strcpy(am.payload_variant.set_ringtone_message, "a:d=8,o=5,b=125:4e6,4e6");
	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "");
	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      meshtastic_Routing_Error_BAD_REQUEST, "set_ringtone: no buzzer, NAK");

	/* Back off so no cycle outlives this test. */
	len = encode_admin_set_extnotify(false, false, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE, "");
	zassert_false(meshtastic_extnotify_nagging(), "disabling stops any cycle");
}
#endif /* CONFIG_MESHTASTIC_EXTNOTIFY */

/* ---- agents-dnr4.18: the canned-message list is stored, served and survives a reboot */

static size_t encode_admin_canned(bool set, const char *messages, uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	if (set) {
		am.which_payload_variant =
			meshtastic_AdminMessage_set_canned_message_module_messages_tag;
		strncpy(am.payload_variant.set_canned_message_module_messages, messages,
			sizeof(am.payload_variant.set_canned_message_module_messages) - 1U);
	} else {
		am.which_payload_variant =
			meshtastic_AdminMessage_get_canned_message_module_messages_request_tag;
		am.payload_variant.get_canned_message_module_messages_request = true;
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

ZTEST(admin_pki, test_canned_messages_round_trip_and_survive_a_real_reboot)
{
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	char stored[MESHTASTIC_CANNED_MESSAGES_LEN];
	uint8_t buf[512];
	size_t len;

	len = encode_admin_canned(false, NULL, buf, sizeof(buf));
	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp), "get must reply");
	zassert_equal(resp.which_payload_variant,
		      meshtastic_AdminMessage_get_canned_message_module_messages_response_tag, "");
	zassert_equal(resp.payload_variant.get_canned_message_module_messages_response[0], '\0',
		      "nothing stored yet: the empty string (reference)");

	len = encode_admin_canned(true, "On my way|Yes|No|Where are you?", buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE,
		      "set must ACK clean");
	zassert_ok(settings_save_subtree("meshtastic"), "flush failed");

	/* Reference: an empty set never clears the list. */
	len = encode_admin_canned(true, "", buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE, "");
	zassert_equal(meshtastic_config_store_get_canned_messages(stored, sizeof(stored)),
		      strlen("On my way|Yes|No|Where are you?"), "an empty set is a no-op");

	/* A real reboot: reload from NVS. */
	zassert_ok(meshtastic_config_store_set_canned_messages("unflushed"), "");
	zassert_ok(settings_load_subtree("meshtastic"), "settings reload failed");
	len = encode_admin_canned(false, NULL, buf, sizeof(buf));
	zassert_true(send_local_admin_and_pop_reply(buf, len, &resp), "get must reply");
	zassert_str_equal(resp.payload_variant.get_canned_message_module_messages_response,
			  "On my way|Yes|No|Where are you?",
			  "the FLUSHED list comes back, not the unflushed edit");
}

/* ---- agents-dnr4.20: ModuleConfig.traffic_management applies live; direct response refused */

#if defined(CONFIG_MESHTASTIC_TRAFFIC)
static size_t encode_admin_set_module_config_traffic(uint32_t dedup, uint32_t direct_hops,
						     uint8_t *buf, size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
	meshtastic_ModuleConfig *mc = &am.payload_variant.set_module_config;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	mc->which_payload_variant = meshtastic_ModuleConfig_traffic_management_tag;
	mc->payload_variant.traffic_management.position_min_interval_secs = dedup;
	mc->payload_variant.traffic_management.nodeinfo_direct_response_max_hops = direct_hops;
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	return os.bytes_written;
}

ZTEST(admin_pki, test_set_module_config_traffic_applies_live_and_refuses_direct_response)
{
	struct meshtastic_traffic_settings s;
	uint8_t buf[256];
	size_t len;
	bool rebooting = true;

	len = encode_admin_set_module_config_traffic(1234U, 0U, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing_ex(buf, len, &rebooting),
		      meshtastic_Routing_Error_NONE, "set must ACK clean");
	zassert_false(rebooting, "read per packet: no reboot");
	meshtastic_traffic_settings(&s);
	zassert_equal(s.position_min_interval_secs, 1234U, "the gate sees it immediately");

	/* Enabling the NodeInfo direct response is refused, and the previous
	 * section is untouched. */
	len = encode_admin_set_module_config_traffic(99U, 3U, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len),
		      meshtastic_Routing_Error_BAD_REQUEST,
		      "direct response is not supported: NAK, not store-and-ignore");
	meshtastic_traffic_settings(&s);
	zassert_equal(s.position_min_interval_secs, 1234U, "refused set left the store alone");

	/* Restore the compiled default so later tests see the seed value. */
	len = encode_admin_set_module_config_traffic(CONFIG_MESHTASTIC_TRAFFIC_POSITION_MIN_INTERVAL_SEC,
						     0U, buf, sizeof(buf));
	zassert_equal(send_local_admin_and_pop_routing(buf, len), meshtastic_Routing_Error_NONE, "");
}
#endif /* CONFIG_MESHTASTIC_TRAFFIC */

/* ---- agents-dnr4.25: ModuleConfig.mesh_beacon is sanitised on write, no reboot */

#if defined(CONFIG_MESHTASTIC_MESHBEACON)
ZTEST(admin_pki, test_set_module_config_mesh_beacon_is_sanitised_and_applies_live)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	meshtastic_ModuleConfig_MeshBeaconConfig *bc =
		&am.payload_variant.set_module_config.payload_variant.mesh_beacon;
	meshtastic_ModuleConfig got = meshtastic_ModuleConfig_init_zero;
	uint8_t buf[512];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	bool rebooting = true;

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant =
		meshtastic_ModuleConfig_mesh_beacon_tag;
	bc->flags = MESHTASTIC_MESHBEACON_FLAG_LISTEN;
	bc->broadcast_interval_secs = 10U; /* below the floor */
	bc->broadcast_offer_region = (meshtastic_Config_LoRaConfig_RegionCode)250; /* unknown */
	strcpy(bc->broadcast_message, "welcome");
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");

	zassert_equal(send_local_admin_and_pop_routing_ex(buf, os.bytes_written, &rebooting),
		      meshtastic_Routing_Error_NONE, "set must ACK clean");
	zassert_false(rebooting, "reference: shouldReboot = false for mesh_beacon");

	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mesh_beacon_tag, &got),
		   "");
	zassert_equal(got.payload_variant.mesh_beacon.broadcast_interval_secs,
		      CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC,
		      "the stored interval was floored on write (reference)");
	zassert_equal(got.payload_variant.mesh_beacon.broadcast_offer_region,
		      meshtastic_Config_LoRaConfig_RegionCode_UNSET,
		      "an unknown offer region was cleared on write (reference)");
	zassert_str_equal(got.payload_variant.mesh_beacon.broadcast_message, "welcome", "");

	/* Back to off so nothing is armed for a later test. */
	bc->flags = 0U;
	os = pb_ostream_from_buffer(buf, sizeof(buf));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "");
	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      meshtastic_Routing_Error_NONE, "");
}
#else /* !CONFIG_MESHTASTIC_MESHBEACON -- the default image */

/* Reference MESHTASTIC_EXCLUDE_BEACON: a write to an excluded module is NAKed,
 * not stored. The module is off by default here (partial vs. the reference),
 * so the default image must tell the app so rather than accept the section. */
ZTEST(admin_pki, test_set_module_config_mesh_beacon_is_refused_when_not_built)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	meshtastic_ModuleConfig got = meshtastic_ModuleConfig_init_zero;
	uint8_t buf[512];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	am.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
	am.payload_variant.set_module_config.which_payload_variant =
		meshtastic_ModuleConfig_mesh_beacon_tag;
	am.payload_variant.set_module_config.payload_variant.mesh_beacon.flags = 1U;
	strcpy(am.payload_variant.set_module_config.payload_variant.mesh_beacon.broadcast_message,
	       "welcome");
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");

	zassert_equal(send_local_admin_and_pop_routing(buf, os.bytes_written),
		      meshtastic_Routing_Error_BAD_REQUEST,
		      "mesh_beacon not built: the write must be NAKed, as the reference does");
	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mesh_beacon_tag, &got),
		   "");
	zassert_equal(got.payload_variant.mesh_beacon.broadcast_message[0], '\0',
		      "and nothing was stored");
}

#endif /* CONFIG_MESHTASTIC_MESHBEACON */

ZTEST(admin_pki, test_remove_ignored_node_unignores)
{
	const uint32_t target = 0x0FB00003U;
	uint8_t key[MESHTASTIC_PKI_KEY_LEN];
	uint8_t buf[32];
	size_t len;

	memset(key, 0x22U, sizeof(key));
	seed_named_peer(target, "Ignored", key);

	zassert_ok(meshtastic_nodedb_set_ignored(target, true), "seed: ignore failed");
	zassert_true(meshtastic_nodedb_is_ignored(target), "seed: must read as ignored");

	len = encode_admin_remove_ignored_node(target, buf, sizeof(buf));
	{
		meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;

		zassert_false(send_local_admin_and_pop_reply(buf, len, &resp),
			     "remove_ignored_node emits no AdminMessage response");
	}
	zassert_false(meshtastic_nodedb_is_ignored(target),
		      "remove_ignored_node must clear the ignored flag");
}

ZTEST(admin_pki, test_reboot_seconds_cancel_on_negative)
{
	uint8_t buf[32];
	size_t len;
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;

	len = encode_admin_reboot_seconds(30, buf, sizeof(buf));
	zassert_false(send_local_admin_and_pop_reply(buf, len, &resp), "unexpected response");
	zassert_true(meshtastic_admin_reboot_scheduled(), "a positive value must schedule a reboot");

	len = encode_admin_reboot_seconds(-1, buf, sizeof(buf));
	zassert_false(send_local_admin_and_pop_reply(buf, len, &resp), "unexpected response");
	zassert_false(meshtastic_admin_reboot_scheduled(),
		      "a negative value must cancel the pending reboot");
}

/* factory_reset_device clears the whole NodeDB (meshtastic_nodedb_reset) as
 * well as config -- factory_reset_config touches config only and leaves peers
 * alone. That's the safe, clearly-observable discriminator to test: unlike
 * the security identity (shared, mutating it risks corrupting every later
 * test in this suite if restoration goes wrong), a seeded peer is this test's
 * own disposable fixture. Only factory_reset_config had a test before this
 * (agents-dnr4.3's stray-commit regression test). */
ZTEST(admin_pki, test_factory_reset_device_also_clears_the_nodedb)
{
	const uint32_t peer = 0x0FB00004U;
	uint8_t key[MESHTASTIC_PKI_KEY_LEN];
	uint8_t buf[32];
	size_t len;
	meshtastic_AdminMessage resp = meshtastic_AdminMessage_init_zero;
	struct meshtastic_nodedb_node snap;

	memset(key, 0x11U, sizeof(key));
	seed_named_peer(peer, "Doomed", key);
	zassert_ok(meshtastic_nodedb_get(peer, &snap), "seed: peer must be present");

	len = encode_admin_factory_reset_device(buf, sizeof(buf));
	zassert_false(send_local_admin_and_pop_reply(buf, len, &resp),
		     "factory_reset_device emits no AdminMessage response, only a delayed reboot");
	k_sleep(K_MSEC(50));
	meshtastic_admin_cancel_reboot(); /* don't actually reboot the test binary */

	zassert_equal(meshtastic_nodedb_get(peer, &snap), -ENOENT,
		      "factory_reset_device must clear the NodeDB too, not just config -- "
		      "contrast with factory_reset_config, which leaves peers alone");

	/* nodedb_reset() just wiped PEER_NODE_ID's pinned key too (it's not a
	 * favorite) -- restore it, since every PKC test after this one in the
	 * suite depends on PEER's identity still being pinned. */
	seed_peer_pubkey(peer_pubkey);
}
