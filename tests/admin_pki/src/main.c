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
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <psa/crypto.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic_admin.h"
#include "meshtastic_admin_session.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_pki.h"
#include "meshtastic_router.h"

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
		memcpy(mock_lora.last_tx, data, data_len);
		mock_lora.last_tx_len = data_len;
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
	return NULL;
}

static void admin_pki_before(void *fixture)
{
	ARG_UNUSED(fixture);
	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	mock_lora.send_count = 0U;
}

ZTEST_SUITE(admin_pki, NULL, admin_pki_setup, admin_pki_before, NULL, NULL);

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
