/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Who this node is: its node id at boot (from the hardware id), the packet and
 * FromRadio id allocators, its names, its hardware model, and the User and
 * DeviceMetadata records built from them for peers and the phone.
 *
 * Split out of meshtastic.c (agents-ls39.3.2); the state these read and write
 * is still the one `mt` (meshtastic_state.c).
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#if IS_ENABLED(CONFIG_MESHTASTIC_NODE_ID_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif

#include <psa/crypto.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_pki.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* CONFIG_MESHTASTIC_HW_MODEL's defaults are bare numbers (Kconfig cannot see the
 * generated enum): pin each one, so a renumbered proto fails the build here
 * instead of shipping a board under another board's model. */
BUILD_ASSERT(meshtastic_HardwareModel_HELTEC_V4_R8 == 132);
BUILD_ASSERT(meshtastic_HardwareModel_HELTEC_V4 == 110);
BUILD_ASSERT(meshtastic_HardwareModel_XIAO_NRF52_KIT == 88);
BUILD_ASSERT(meshtastic_HardwareModel_HELTEC_MESH_NODE_T114 == 69);
BUILD_ASSERT(meshtastic_HardwareModel_PRIVATE_HW == 255);


#if IS_ENABLED(CONFIG_MESHTASTIC_NODE_ID_HWINFO)
/* Last 4 bytes big-endian; for len>=6 uses bytes [2..5] (Meshtastic MAC rule). */
static uint32_t node_id_from_bytes(const uint8_t *data, size_t len)
{
	uint8_t buf[4] = {0};

	if (len >= 6) {
		return sys_get_be32(&data[2]);
	}

	if (len >= 4) {
		return sys_get_be32(&data[len - 4]);
	}

	if (len == 0) {
		return 0;
	}

	memcpy(&buf[4 - len], data, len);
	return sys_get_be32(buf);
}

int meshtastic_node_id_from_hwinfo(uint32_t *node_id)
{
	uint8_t hwid[16];
	ssize_t hwlen;

	hwlen = hwinfo_get_device_id(hwid, sizeof(hwid));
	if (hwlen <= 0) {
		return -ENODEV;
	}

	*node_id = node_id_from_bytes(hwid, (size_t)hwlen);
	if (*node_id == 0U) {
		return -EINVAL;
	}

	if (*node_id <= 3U) {
		LOG_WRN("Derived node ID 0x%08x is reserved (1-3)", *node_id);
	}

	return 0;
}
#endif /* CONFIG_MESHTASTIC_NODE_ID_HWINFO */

/*
 * Packet-id generation.
 *
 * The AES-CTR channel nonce is built from (packet id, source node id) and
 * nothing else. The source id is fixed for the life of the node, so the id is
 * the only thing keeping the (key, nonce) pair unique — and a counter that
 * restarts at a fixed value every boot re-emits packets under nonces already
 * used before the reboot. Two ciphertexts under one keystream leak their
 * plaintexts' XOR, which on a secret-PSK channel is a real confidentiality
 * break for anyone logging across an induced restart.
 *
 * So: the counter is seeded from the CSPRNG at boot, and each id carries fresh
 * random high bits on top of it. The low bits stay a rolling counter, which
 * keeps consecutive ids distinct even if the RNG were to repeat itself.
 *
 * Wire format is unchanged — the id is still an opaque 32-bit value.
 *
 * MESHTASTIC_PKT_ID_COUNTER_BITS (meshtastic_core.h) mirrors upstream
 * Router.cpp generatePacketId()'s split exactly: a 10-bit rolling counter in
 * the low bits, 22 fresh random bits refreshed on every call in the high bits
 * ("rollingPacketId &= ID_COUNTER_MASK; id = rollingPacketId | random() << 10").
 */
#define PKT_ID_COUNTER_BITS MESHTASTIC_PKT_ID_COUNTER_BITS
#define PKT_ID_COUNTER_MASK BIT_MASK(PKT_ID_COUNTER_BITS)

int meshtastic_seed_packet_id(void)
{
	uint32_t seed = 0U;
	psa_status_t st = psa_generate_random((uint8_t *)&seed, sizeof(seed));

	if (st != PSA_SUCCESS) {
		/* Refuse rather than silently fall back to a predictable counter:
		 * a node that cannot seed this cannot encrypt safely, and the
		 * failure would otherwise be invisible until someone captured
		 * traffic across a reboot. */
		LOG_ERR("packet-id seed failed (%d) — refusing to start with a "
			"predictable AES-CTR nonce",
			(int)st);
		return -EIO;
	}

	atomic_set(&mt.next_pkt_id, (atomic_val_t)seed);
	return 0;
}

uint32_t meshtastic_allocate_packet_id(void)
{
	uint32_t id;

	do {
		uint32_t high = 0U;
		/* One atomic step per id: two threads can no longer read the
		 * same counter value (atomic_inc returns the value before). */
		uint32_t ctr = (uint32_t)atomic_inc(&mt.next_pkt_id) + 1U;

		if (psa_generate_random((uint8_t *)&high, sizeof(high)) != PSA_SUCCESS) {
			/* Degrade to the counter alone. Still safe for the property
			 * that matters here: the counter was seeded randomly at
			 * boot, so ids do not repeat across a restart. */
			high = ctr >> PKT_ID_COUNTER_BITS;
		}

		id = (ctr & PKT_ID_COUNTER_MASK) | (high << PKT_ID_COUNTER_BITS);
	} while (id == 0U); /* id 0 is reserved (see the id==0 broadcast guard) */

	return id;
}

uint32_t meshtastic_next_fromradio_id(void)
{
	/* The low 32 bits of an atomic counter: wraps like the uint32_t it was,
	 * and skips NONE the same way, without a lost update between threads. */
	uint32_t id = (uint32_t)atomic_inc(&mt.next_fromradio_id);

	if (id == MESHTASTIC_FROMRADIO_NONE) {
		id = (uint32_t)atomic_inc(&mt.next_fromradio_id);
	}

	return id;
}

const char *meshtastic_long_name(void)
{
	return mt.long_name;
}

const char *meshtastic_short_name(void)
{
	return mt.short_name;
}

meshtastic_HardwareModel meshtastic_hw_model(void)
{
	return (meshtastic_HardwareModel)CONFIG_MESHTASTIC_HW_MODEL;
}

void meshtastic_fill_user(meshtastic_User *user)
{
	uint32_t node = mt.node_id;

	*user = (meshtastic_User)meshtastic_User_init_zero;
	snprintk(user->id, sizeof(user->id), "!%08x", node);
	strncpy(user->long_name, mt.long_name, sizeof(user->long_name) - 1U);
	strncpy(user->short_name, mt.short_name, sizeof(user->short_name) - 1U);

	/* Meshtastic derives the node number from the low 4 bytes of macaddr. */
	user->macaddr[0] = 0x02U;
	user->macaddr[1] = 0x00U;
	user->macaddr[2] = (uint8_t)(node >> 24);
	user->macaddr[3] = (uint8_t)(node >> 16);
	user->macaddr[4] = (uint8_t)(node >> 8);
	user->macaddr[5] = (uint8_t)node;

	user->hw_model = meshtastic_hw_model();
	user->role = meshtastic_device_role();

	meshtastic_config_store_get_owner_flags(&user->is_licensed, &user->is_unmessagable);
	user->has_is_unmessagable = true;

#if defined(CONFIG_MESHTASTIC_PKI)
	{
		uint8_t pub[MESHTASTIC_PKI_KEY_LEN];

		if (meshtastic_pki_get_public_key(pub) == MESHTASTIC_PKI_KEY_LEN) {
			memcpy(user->public_key.bytes, pub, MESHTASTIC_PKI_KEY_LEN);
			user->public_key.size = MESHTASTIC_PKI_KEY_LEN;
		}
	}
#endif
}

void meshtastic_fill_device_metadata(meshtastic_DeviceMetadata *md)
{
	*md = (meshtastic_DeviceMetadata)meshtastic_DeviceMetadata_init_zero;
	strncpy(md->firmware_version, MESHTASTIC_FIRMWARE_VERSION,
		sizeof(md->firmware_version) - 1U);
	md->hasBluetooth = IS_ENABLED(CONFIG_MESHTASTIC_BLE);
	md->hasWifi = IS_ENABLED(CONFIG_WIFI);
	md->hasEthernet = IS_ENABLED(CONFIG_NET_L2_ETHERNET);
	md->canShutdown = IS_ENABLED(CONFIG_POWEROFF); /* sys_poweroff() -> deep sleep */
	md->hasRemoteHardware = false; /* no RemoteHardware module */
#if defined(CONFIG_MESHTASTIC_PKI)
	md->hasPKC = meshtastic_pki_have_key();
#else
	md->hasPKC = false;            /* no ECDH/PKC key agreement */
#endif
	md->role = meshtastic_device_role();
	md->hw_model = meshtastic_hw_model();

	/*
	 * Tell the app which module config editors to hide. Unlike the reference
	 * firmware (which only excludes its platform-absent set), we report this
	 * port's TRUE module availability: modules with no handler here are marked
	 * excluded so the app doesn't offer no-op config screens for them. Gate the
	 * ones the port can build; exclude the rest unconditionally.
	 */
	md->excluded_modules =
		(IS_ENABLED(CONFIG_MESHTASTIC_MQTT) ? 0U
						    : meshtastic_ExcludedModules_MQTT_CONFIG) |
		((IS_ENABLED(CONFIG_MESHTASTIC_DEVICE_METRICS) ||
		  IS_ENABLED(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS))
			 ? 0U
			 : meshtastic_ExcludedModules_TELEMETRY_CONFIG) |
		(IS_ENABLED(CONFIG_MESHTASTIC_BLE) ? 0U
						   : meshtastic_ExcludedModules_BLUETOOTH_CONFIG) |
		((IS_ENABLED(CONFIG_WIFI) || IS_ENABLED(CONFIG_NET_L2_ETHERNET))
			 ? 0U
			 : meshtastic_ExcludedModules_NETWORK_CONFIG) |
		(IS_ENABLED(CONFIG_MESHTASTIC_EXTNOTIFY) ? 0U
							: meshtastic_ExcludedModules_EXTNOTIF_CONFIG) |
		(IS_ENABLED(CONFIG_MESHTASTIC_NEIGHBORINFO)
			 ? 0U
			 : meshtastic_ExcludedModules_NEIGHBORINFO_CONFIG) |
		/* No handler in the port — always excluded. Building one of these
		 * means moving its bit up into the gated group above; the protocol
		 * suite's test_excluded_modules_track_the_build holds both lists.
		 * Note MESHTASTIC_SERIAL is the PhoneAPI transport, not the on-mesh
		 * Serial module. */
		meshtastic_ExcludedModules_SERIAL_CONFIG |
		meshtastic_ExcludedModules_STOREFORWARD_CONFIG |
		meshtastic_ExcludedModules_RANGETEST_CONFIG |
		meshtastic_ExcludedModules_CANNEDMSG_CONFIG |
		meshtastic_ExcludedModules_AUDIO_CONFIG |
		meshtastic_ExcludedModules_REMOTEHARDWARE_CONFIG |
		meshtastic_ExcludedModules_AMBIENTLIGHTING_CONFIG |
		meshtastic_ExcludedModules_DETECTIONSENSOR_CONFIG |
		meshtastic_ExcludedModules_PAXCOUNTER_CONFIG;
}
