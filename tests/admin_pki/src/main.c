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

/* Encode an AdminMessage set_config(lora) carrying the given passkey. Changing the
 * modem preset is a LoRa change, which F-1 must schedule a reboot for (the SX1262
 * is only reconfigured at radio init). */
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
						&pkt, payload, sizeof(payload), &decoded, &fail);
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
							sizeof(payload), &decoded, &fail);
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
ZTEST(admin_pki, test_lora_config_change_schedules_reboot)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	size_t len;

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

	/* LoRa preset change needs a radio re-init -> must schedule a reboot (F-1). */
	meshtastic_admin_cancel_reboot();
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_lora_preset(meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST, key,
					   sizeof(key), buf, sizeof(buf));
	inject_pkc_admin(buf, len, 0x0AD1F002U);
	k_sleep(K_MSEC(50));
	zassert_true(meshtastic_admin_reboot_scheduled(),
		     "a LoRa config change must schedule a reboot (F-1)");

	/* Cleanup: cancel the scheduled reboot (so the sim doesn't reboot) and restore
	 * the default preset for any later test. */
	meshtastic_admin_cancel_reboot();
	{
		meshtastic_Config lora = meshtastic_Config_init_zero;

		lora.which_payload_variant = meshtastic_Config_lora_tag;
		lora.payload_variant.lora.use_preset = true;
		lora.payload_variant.lora.modem_preset =
			meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
		lora.payload_variant.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
		lora.payload_variant.lora.hop_limit = 3;
		(void)meshtastic_config_store_set_config(&lora);
	}
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
