/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * CONFIG_MESHTASTIC_NODE_ID_FROM_KEY: the node id is crc32 of the X25519 public key, as in the
 * reference firmware since 2.8, with the configured id demoted to a provisional value.
 *
 * What is proven here, and what is not:
 *  - The CRC is pinned with the reference library's own known-answer vectors, so "our crc32
 *    agrees with ourselves" is not all that is being tested.
 *  - The booted stack actually runs under the derived id: MyNodeInfo, the NodeDB self entry and
 *    the wire header all carry it, and nothing is left under the provisional id.
 *  - Every branch of the decision (key / stored / provisional, and refusal of a reserved or
 *    broadcast derivation) is driven directly, because a locked boot and an unusable key cannot
 *    be staged inside one process.
 *  - A reboot with a DIFFERENT persisted id is staged by writing the record and re-running the
 *    boot step: the key wins and the record is corrected.
 * Not proven here: the lockdown unlock path end to end (the key_reloaded hook). Its decision is
 * the same choose() driven below.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <psa/crypto.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_node_identity.h"
#include "meshtastic_packet.h"
#include "meshtastic_pki.h"

/* The configured id: with the key-derived identity on, it is only the provisional value. */
#define PROVISIONAL_ID 0x12345678U

/* ---- a minimal LoRa device that records what was transmitted ------------------------- */

static struct {
	struct k_mutex lock;
	uint32_t send_count;
	uint8_t last_tx[MESHTASTIC_PKT_MAX];
	uint32_t last_tx_len;
} mock_lora;

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
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
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

/* ---- vectors --------------------------------------------------------------------------- */

/* RFC 7748 section 6.1, Alice's X25519 public key: a published, neutral 32-byte key. Its crc32
 * (0x8a62558c) was computed with Python's zlib.crc32 -- a second, independent implementation of
 * the same standard CRC-32 -- not with the code under test. */
static const uint8_t rfc7748_alice_pub[32] = {
	0x85, 0x20, 0xf0, 0x09, 0x89, 0x30, 0xa7, 0x54, 0x74, 0x8b, 0x7d, 0xdc, 0xb4, 0x3e, 0xf7, 0x5a,
	0x0d, 0xbf, 0x3a, 0x0d, 0x26, 0x38, 0x1a, 0xf4, 0xeb, 0xa4, 0xa9, 0x8e, 0xaa, 0x9b, 0x4e, 0x6a,
};

/* 32-byte inputs forced to a chosen crc32: bytes 0..27, then four bytes solved by running the
 * reflected CRC-32 register backwards (CRC is linear, so any 4-byte suffix can hit any target).
 * Each was checked with zlib.crc32. They are not real keys -- only the derivation's refusal of
 * an unusable result is under test, and a real keypair cannot be steered to one. */
static const uint8_t key_crc_zero[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0xc0, 0x02, 0xd1, 0xba,
};
static const uint8_t key_crc_three[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x42, 0x0e, 0x33, 0x0c,
};
static const uint8_t key_crc_broadcast[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0xa2, 0xf7, 0xf7, 0x28,
};
static const uint8_t key_crc_four[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x46, 0x17, 0xf7, 0x61,
};

/* ---- helpers ----------------------------------------------------------------------------- */

static uint32_t our_derived_id(void)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];

	zassert_equal(meshtastic_pki_get_public_key(pub), MESHTASTIC_PKI_KEY_LEN,
		      "no public key to derive from");
	return meshtastic_node_id_from_public_key(pub, sizeof(pub));
}

struct stored_read {
	bool found;
	uint32_t id;
};

static int read_stored_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			  void *param)
{
	struct stored_read *out = param;
	uint8_t buf[sizeof(uint32_t)];

	if (key == NULL || strcmp(key, "id") != 0 || len != sizeof(buf)) {
		return 0;
	}
	if (read_cb(cb_arg, buf, len) == (ssize_t)len) {
		out->id = sys_get_le32(buf);
		out->found = true;
	}
	return 0;
}

static struct stored_read read_stored(void)
{
	struct stored_read r = {0};

	(void)settings_load_subtree_direct(MESHTASTIC_NODE_ID_SUBTREE, read_stored_cb, &r);
	return r;
}

/* What the BOOT persisted, captured in setup before any test runs. Reading the store inside the
 * test instead would be order-dependent: ztest runs cases alphabetically, and the stale-record
 * test calls adopt() itself -- so a boot that persisted nothing still passed (found by the
 * negative check that removed the boot-time adopt). */
static struct stored_read boot_stored;

static void *node_identity_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = PROVISIONAL_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa_crypto_init failed");
	zassert_true(device_is_ready(lora_dev), "mock lora not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	zassert_true(meshtastic_pki_have_key(), "our X25519 keypair should be ready");
	boot_stored = read_stored();
	return NULL;
}

ZTEST_SUITE(node_identity, NULL, node_identity_setup, NULL, NULL, NULL);

/* ---- the CRC is the reference's --------------------------------------------------------- */

/* Verbatim from the library the reference links for crc32Buffer: ErriezCRC32 1.0.1,
 * tests/CRC32UnitTest/CRC32UnitTest.ino:50-53. If Zephyr's CRC ever differed in init, final xor
 * or reflection, one of these would fail -- a symmetric error cannot hide from a vector that
 * came from the other implementation. */
ZTEST(node_identity, test_crc_matches_the_reference_library_vectors)
{
	zassert_equal(meshtastic_node_id_from_public_key((const uint8_t[]){0x00}, 1), 0xD202EF8DU);
	zassert_equal(meshtastic_node_id_from_public_key((const uint8_t[]){0x01}, 1), 0xA505DF1BU);
	zassert_equal(meshtastic_node_id_from_public_key((const uint8_t[]){0xFF}, 1), 0xFF000000U);
}

/* A whole 32-byte key, in index order and with no byte swap on the result -- the two places a
 * port could quietly diverge that one-byte vectors cannot see. */
ZTEST(node_identity, test_a_32_byte_key_derives_in_index_order_without_a_swap)
{
	zassert_equal(meshtastic_node_id_from_public_key(rfc7748_alice_pub, 32), 0x8a62558cU);
}

/* ---- the booted stack runs under the derived id ----------------------------------------- */

ZTEST(node_identity, test_booted_id_is_the_crc32_of_our_public_key)
{
	uint32_t derived = our_derived_id();

	zassert_equal(meshtastic_get_node_id(), derived,
		      "node id 0x%08x is not crc32 of our key (0x%08x)", meshtastic_get_node_id(),
		      derived);
	zassert_not_equal(meshtastic_get_node_id(), PROVISIONAL_ID,
			  "the configured id must only be provisional once a key exists");
}

/* The NodeDB self entry is created after the id is adopted, so it must be under the derived id
 * -- and nothing may be left answering to the provisional one, which would be exactly the
 * "ghost of ourselves" the reference had to remove from its own migration. */
ZTEST(node_identity, test_self_entry_is_under_the_derived_id_and_nothing_under_the_old)
{
	struct meshtastic_nodedb_node node;

	zassert_ok(meshtastic_nodedb_get(our_derived_id(), &node), "no self entry under the id");
	zassert_not_equal(meshtastic_nodedb_get(PROVISIONAL_ID, &node), 0,
			  "an entry is still answering to the provisional id");
}

/* The id actually reaches the air. Checked on the wire header, not an accessor. */
ZTEST(node_identity, test_transmitted_frames_carry_the_derived_id)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	const struct meshtastic_wire_header *hdr;
	uint32_t before;
	uint32_t src;

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	before = mock_lora.send_count;
	k_mutex_unlock(&mock_lora.lock);

	mesh.to = MESHTASTIC_NODE_BROADCAST;
	mesh.id = 0x1D1DU;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
	mesh.decoded.payload.size = 2U;
	memcpy(mesh.decoded.payload.bytes, "id", 2U);
	zassert_ok(meshtastic_send_mesh_pb(&mesh), "send failed");

	for (int i = 0; i < 200; i++) {
		k_mutex_lock(&mock_lora.lock, K_FOREVER);
		bool sent = mock_lora.send_count > before;
		k_mutex_unlock(&mock_lora.lock);
		if (sent) {
			break;
		}
		k_msleep(10);
	}
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_true(mock_lora.send_count > before, "nothing was transmitted");
	hdr = (const struct meshtastic_wire_header *)mock_lora.last_tx;
	src = sys_le32_to_cpu(hdr->src);
	k_mutex_unlock(&mock_lora.lock);

	zassert_equal(src, our_derived_id(), "frame src 0x%08x is not the derived id", src);
}

/* ---- persistence ------------------------------------------------------------------------ */

ZTEST(node_identity, test_the_adopted_id_is_persisted_for_a_locked_boot)
{
	struct stored_read r = boot_stored;

	zassert_true(r.found, "the boot wrote no " MESHTASTIC_NODE_ID_SUBTREE " record");
	zassert_equal(r.id, our_derived_id(), "stored 0x%08x, expected the derived id", r.id);
}

/* A reboot finds a stored id that disagrees with the key (a restored backup, an admin re-key
 * applied at the reboot, a corrupted record). The key wins and the record is corrected -- the
 * stored id is only ever a stand-in for a key that cannot be read. */
ZTEST(node_identity, test_a_stale_stored_id_loses_to_the_key_and_is_rewritten)
{
	uint8_t buf[sizeof(uint32_t)];
	uint32_t id;

	sys_put_le32(0x0BADF00DU, buf);
	zassert_ok(settings_save_one(MESHTASTIC_NODE_ID_SUBTREE "/id", buf, sizeof(buf)));

	id = meshtastic_node_identity_adopt(PROVISIONAL_ID);

	zassert_equal(id, our_derived_id(), "the stale stored id won over the key");
	zassert_equal(read_stored().id, our_derived_id(), "the stale record was not corrected");
}

/* ---- the decision, branch by branch ----------------------------------------------------- */

ZTEST(node_identity, test_choose_prefers_the_key_over_anything_stored)
{
	enum meshtastic_node_id_origin o;
	uint32_t id = meshtastic_node_id_choose(key_crc_four, true, 0x0BADF00DU, PROVISIONAL_ID, &o);

	zassert_equal(id, 4U);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_KEY);
}

/* A locked boot: no readable key, but the last adopted id is stored in the clear. */
ZTEST(node_identity, test_choose_uses_the_stored_id_when_no_key_is_readable)
{
	enum meshtastic_node_id_origin o;
	uint32_t id = meshtastic_node_id_choose(NULL, true, 0x0BADF00DU, PROVISIONAL_ID, &o);

	zassert_equal(id, 0x0BADF00DU);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_STORE);
}

ZTEST(node_identity, test_choose_falls_back_to_provisional_with_no_key_and_no_store)
{
	enum meshtastic_node_id_origin o;
	uint32_t id = meshtastic_node_id_choose(NULL, false, 0U, PROVISIONAL_ID, &o);

	zassert_equal(id, PROVISIONAL_ID);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL);
}

/* A key whose crc32 is reserved or broadcast is refused, and the no-key rules decide instead.
 * Deliberately stricter than the reference, which adopts any crc32: a node whose own id is the
 * broadcast address is unreachable, while one on its hardware id only loses the binding. The
 * boundary (4, the first usable id) is checked on the accepting side too. */
ZTEST(node_identity, test_choose_refuses_a_reserved_or_broadcast_derivation)
{
	enum meshtastic_node_id_origin o;

	zassert_equal(meshtastic_node_id_from_public_key(key_crc_zero, 32), 0U, "bad vector");
	zassert_equal(meshtastic_node_id_from_public_key(key_crc_three, 32), 3U, "bad vector");
	zassert_equal(meshtastic_node_id_from_public_key(key_crc_broadcast, 32),
		      MESHTASTIC_NODE_BROADCAST, "bad vector");

	zassert_equal(meshtastic_node_id_choose(key_crc_zero, false, 0U, PROVISIONAL_ID, &o),
		      PROVISIONAL_ID);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL);

	zassert_equal(meshtastic_node_id_choose(key_crc_three, true, 0x0BADF00DU, PROVISIONAL_ID, &o),
		      0x0BADF00DU);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_STORE);

	zassert_equal(meshtastic_node_id_choose(key_crc_broadcast, false, 0U, PROVISIONAL_ID, &o),
		      PROVISIONAL_ID);

	zassert_equal(meshtastic_node_id_choose(key_crc_four, false, 0U, PROVISIONAL_ID, &o), 4U);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_KEY, "4 is the first usable id");
}

/* A stored id is only trusted if it is itself usable -- a corrupted record reading as the
 * broadcast address must not be adopted on a locked boot. */
ZTEST(node_identity, test_choose_ignores_an_unusable_stored_id)
{
	enum meshtastic_node_id_origin o;

	zassert_equal(meshtastic_node_id_choose(NULL, true, MESHTASTIC_NODE_BROADCAST,
						PROVISIONAL_ID, &o),
		      PROVISIONAL_ID);
	zassert_equal(o, MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL);
	zassert_equal(meshtastic_node_id_choose(NULL, true, 2U, PROVISIONAL_ID, &o), PROVISIONAL_ID);
}
