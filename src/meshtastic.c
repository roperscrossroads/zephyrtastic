/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic mesh radio stack public API and runtime state.
 */

#include <string.h>

#include <zephyr/device.h>
#if IS_ENABLED(CONFIG_MESHTASTIC_NODE_ID_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <psa/crypto.h>

#include <zephyr/meshtastic/gnss.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include <zephyr/meshtastic/telemetry.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_reliable.h"
#include "meshtastic_sched.h"

#include "meshtastic_settings.h"
#include "meshtastic_gnss.h"
#include "meshtastic_mqtt.h"
#if defined(CONFIG_MESHTASTIC_AIRTIME)
#include "meshtastic_airtime.h"
#endif

#if defined(CONFIG_MESHTASTIC_BLE)
int meshtastic_ble_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_SERIAL)
int meshtastic_serial_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_TCP)
int meshtastic_tcp_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
int meshtastic_metrics_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
int meshtastic_environment_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_MESSAGE)
int meshtastic_message_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB)
int meshtastic_nodedb_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_NODEINFO)
int meshtastic_nodeinfo_init(void);
#endif

#if defined(CONFIG_BOARD_TWATCH_S3)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_T_WATCH_S3
#elif defined(CONFIG_BOARD_TTGO_TBEAM)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_TBEAM
#elif defined(CONFIG_BOARD_TTGO_LORA32)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_TLORA_V2_1_1P8
#elif defined(CONFIG_BOARD_HELTEC_WIFI_LORA32_V3)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_HELTEC_V3
#elif defined(CONFIG_BOARD_HELTEC_WIFI_LORA32_V2)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_HELTEC_V2_1
#elif defined(CONFIG_BOARD_HELTEC_WIRELESS_TRACKER)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_HELTEC_WIRELESS_TRACKER
#elif defined(CONFIG_BOARD_HELTEC_WIRELESS_STICK_LITE_V3)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_HELTEC_WSL_V3
#elif defined(CONFIG_BOARD_HELTEC_T114_V2)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_HELTEC_MESH_NODE_T114
#elif defined(CONFIG_BOARD_RAK4631)
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_RAK4631
#else
#define MESHTASTIC_BOARD_HW_MODEL meshtastic_HardwareModel_PRIVATE_HW
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

const uint8_t meshtastic_default_psk[16] = {
	0xd4U, 0xf1U, 0xbbU, 0x3aU, 0x20U, 0x29U, 0x07U, 0x59U,
	0xf0U, 0xbcU, 0xffU, 0xabU, 0xcfU, 0x4eU, 0x69U, 0x01U,
};

struct meshtastic_context mt;
struct meshtastic_workspace mt_ws;

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

static int node_id_from_hwinfo(uint32_t *node_id)
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

uint32_t meshtastic_allocate_packet_id(void)
{
	uint32_t id = mt.next_pkt_id++;

	if (id == 0U) {
		id = mt.next_pkt_id++;
	}

	return id;
}

uint32_t meshtastic_next_fromradio_id(void)
{
	uint32_t id = mt.next_fromradio_id++;

	if (id == MESHTASTIC_FROMRADIO_NONE) {
		id = mt.next_fromradio_id++;
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
	return MESHTASTIC_BOARD_HW_MODEL;
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
}

void meshtastic_fill_device_metadata(meshtastic_DeviceMetadata *md)
{
	*md = (meshtastic_DeviceMetadata)meshtastic_DeviceMetadata_init_zero;
	/* Parses as 2.7.4 — keeps the app above its minimum-version gate. */
	strncpy(md->firmware_version, "2.7.4.zephyr", sizeof(md->firmware_version) - 1U);
	md->hasBluetooth = IS_ENABLED(CONFIG_MESHTASTIC_BLE);
	md->hasWifi = IS_ENABLED(CONFIG_WIFI);
	md->hasEthernet = IS_ENABLED(CONFIG_NET_L2_ETHERNET);
	md->canShutdown = false;       /* no PM/poweroff path in the port */
	md->hasRemoteHardware = false; /* no RemoteHardware module */
	md->hasPKC = false;            /* no ECDH/PKC key agreement yet */
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
		/* No handler in the port — always excluded. Note MESHTASTIC_SERIAL is
		 * the PhoneAPI transport, not the on-mesh Serial module. */
		meshtastic_ExcludedModules_SERIAL_CONFIG |
		meshtastic_ExcludedModules_EXTNOTIF_CONFIG |
		meshtastic_ExcludedModules_STOREFORWARD_CONFIG |
		meshtastic_ExcludedModules_RANGETEST_CONFIG |
		meshtastic_ExcludedModules_CANNEDMSG_CONFIG |
		meshtastic_ExcludedModules_AUDIO_CONFIG |
		meshtastic_ExcludedModules_REMOTEHARDWARE_CONFIG |
		meshtastic_ExcludedModules_NEIGHBORINFO_CONFIG |
		meshtastic_ExcludedModules_AMBIENTLIGHTING_CONFIG |
		meshtastic_ExcludedModules_DETECTIONSENSOR_CONFIG |
		meshtastic_ExcludedModules_PAXCOUNTER_CONFIG;
}

void meshtastic_set_ble_connected(bool connected)
{
	mt.status.ble_connected = connected;
}

void meshtastic_emit_event(enum meshtastic_event_type type, int err,
			   const struct meshtastic_packet *packet)
{
	struct meshtastic_event event = {
		.type = type,
		.err = err,
		.packet = packet,
	};

	if (mt.event_cb != NULL) {
		mt.event_cb(&event, mt.event_user_data);
	}
}

int meshtastic_init(const struct meshtastic_config *cfg)
{
	psa_status_t psa_st;
	int ret;

	if (cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->lora_dev == NULL || !device_is_ready(cfg->lora_dev)) {
		LOG_ERR("LoRa device not ready");
		return -ENODEV;
	}

	if (cfg->psk == NULL || (cfg->psk_len != 16U && cfg->psk_len != 32U)) {
		LOG_ERR("Invalid PSK (len=%zu)", cfg->psk_len);
		return -EINVAL;
	}

	if (cfg->channel_name == NULL || cfg->frequency == 0U) {
		return -EINVAL;
	}

	mt.lora_dev = cfg->lora_dev;
#if IS_ENABLED(CONFIG_MESHTASTIC_NODE_ID_CUSTOM)
	if (cfg->node_id != 0U) {
		mt.node_id = cfg->node_id;
	} else {
		mt.node_id = CONFIG_MESHTASTIC_NODE_ID_DEFAULT;
	}
#else /* CONFIG_MESHTASTIC_NODE_ID_HWINFO */
	ret = node_id_from_hwinfo(&mt.node_id);
	if (ret < 0) {
		return ret;
	}

	if (cfg->node_id != 0U && cfg->node_id != mt.node_id) {
		LOG_WRN("Ignoring config node_id 0x%08x (using derived 0x%08x)", cfg->node_id,
			mt.node_id);
	}
#endif
	ret = meshtastic_channels_init_from_config(cfg);
	if (ret < 0) {
		return ret;
	}

	mt.frequency = cfg->frequency;
	mt.hop_limit = (cfg->hop_limit == 0U) ? (uint8_t)CONFIG_MESHTASTIC_DEFAULT_HOP_LIMIT
					      : cfg->hop_limit;
	mt.tx_power = (cfg->tx_power == 0) ? (int8_t)CONFIG_MESHTASTIC_TX_POWER : cfg->tx_power;
	mt.long_name = (cfg->long_name != NULL) ? cfg->long_name : CONFIG_MESHTASTIC_NODE_LONG_NAME;
	mt.short_name =
		(cfg->short_name != NULL) ? cfg->short_name : CONFIG_MESHTASTIC_NODE_SHORT_NAME;
	mt.next_pkt_id = 1U;
	mt.next_fromradio_id = 1U;
	mt.dup_head = 0U;
	mt.radio_rx_armed = false;
	memset(mt.dup_cache, 0, sizeof(mt.dup_cache));
	memset(&mt.status, 0, sizeof(mt.status));
	mt.status.node_id = mt.node_id;

	psa_st = psa_crypto_init();
	if (psa_st != PSA_SUCCESS) {
		LOG_ERR("psa_crypto_init failed (%d)", (int)psa_st);
		return -EIO;
	}

	k_mutex_init(&mt.lock);
	k_mutex_init(&mt_ws.lock);

	mt.initialized = true;
	mt.status.initialized = true;

	ret = meshtastic_config_store_seed(cfg);
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	ret = meshtastic_settings_init();
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}
#endif

	ret = meshtastic_config_store_apply_core();
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}

	ret = meshtastic_settings_apply_all();
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}

	ret = meshtastic_radio_init();
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}

#if defined(CONFIG_MESHTASTIC_GNSS)
	ret = meshtastic_gnss_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
	ret = meshtastic_metrics_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
	ret = meshtastic_environment_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_NODEDB)
	ret = meshtastic_nodedb_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_NODEINFO)
	ret = meshtastic_nodeinfo_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_MESSAGE)
	ret = meshtastic_message_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_BLE)
	ret = meshtastic_ble_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_SERIAL)
	ret = meshtastic_serial_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_TCP)
	ret = meshtastic_tcp_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_MQTT)
	ret = meshtastic_mqtt_init();
	if (ret < 0) {
		return ret;
	}
#endif

	LOG_INF("Meshtastic init: node=0x%08x ch_hash=0x%02x freq=%uHz", mt.node_id, mt.ch_hash,
		mt.frequency);

	return 0;
}

static int send_packet_prepare(const struct meshtastic_packet *packet,
			       struct meshtastic_packet *local, uint8_t *wire, uint32_t *pkt_len)
{
	int ret;

	if (!mt.initialized) {
		return -EINVAL;
	}

	if (packet == NULL || packet->to == 0U ||
	    (packet->payload == NULL && packet->payload_len != 0U) ||
	    packet->payload_len > MESHTASTIC_MAX_PAYLOAD_LEN) {
		LOG_ERR("Invalid packet");
		return -EINVAL;
	}

	*local = *packet;
	if (local->from == 0U) {
		local->from = mt.node_id;
	}
	if (local->id == 0U) {
		local->id = meshtastic_allocate_packet_id();
	}
	if (local->hop_limit == 0U) {
		local->hop_limit = mt.hop_limit;
	}
	if (local->hop_start == 0U) {
		local->hop_start = local->hop_limit;
	}
	if (local->channel_index == MESHTASTIC_CHANNEL_INDEX_INVALID) {
		local->channel_index = meshtastic_channels_primary_index();
	}

	k_mutex_lock(&mt_ws.lock, K_FOREVER);
	ret = meshtastic_build_wire_packet(local, wire, pkt_len);
	k_mutex_unlock(&mt_ws.lock);

	return ret;
}

static int send_packet_complete(const struct meshtastic_packet *local, const uint8_t *wire,
				uint32_t pkt_len, int tx_ret, bool emit_done)
{
	if (tx_ret < 0) {
		LOG_ERR("Packet send failed (%d)", tx_ret);
		return tx_ret;
	}

	if (!emit_done) {
		return 0;
	}

	LOG_INF("TX to 0x%08x port=%u len=%u", local->to, (unsigned int)local->portnum,
		(unsigned int)pkt_len);
	meshtastic_emit_event(MESHTASTIC_EVENT_TX_DONE, 0, local);
#if defined(CONFIG_MESHTASTIC_MQTT)
	meshtastic_mqtt_on_tx(local, wire, pkt_len);
#endif

	return 0;
}

int meshtastic_send_packet(const struct meshtastic_packet *packet, k_timeout_t wait)
{
	struct meshtastic_packet local;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t pkt_len = 0U;
	int ret;

	ret = send_packet_prepare(packet, &local, wire, &pkt_len);
	if (ret < 0) {
		return ret;
	}

	uint8_t tier = meshtastic_sched_tier_for(local.portnum);

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	/* Airtime gate: throttle only the node's own periodic background beacons
	 * (broadcast, fire-and-forget). Requested replies (unicast and/or a caller
	 * that waits) and every higher tier are never gated. */
	if (tier == MT_SCHED_TIER_BG && K_TIMEOUT_EQ(wait, K_NO_WAIT) &&
	    local.to == MESHTASTIC_NODE_BROADCAST) {
		/* Single scalar, captured once — a direct atomic read is sufficient
		 * (see the concurrency note in meshtastic_sched.h). */
		uint8_t max_util = meshtastic_sched_get()->airtime_max_util;

		if (max_util != 0U &&
		    meshtastic_airtime_channel_util_percent() >= (float)max_util) {
			meshtastic_sched_stat_airtime_drop();
			LOG_DBG("airtime gate: suppressed BG broadcast port=%u (chan util >= %u%%)",
				(unsigned int)local.portnum, max_util);
			return 0;
		}
	}
#endif

	if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
		ret = meshtastic_radio_send_wire_prio(wire, pkt_len, tier);
	} else {
		ret = meshtastic_radio_send_wire_wait_prio(wire, pkt_len, tier, wait);
	}

	if (ret >= 0) {
		/* Track for retransmission if it is a want_ack unicast we originate
		 * (the hook self-filters everything else). */
		meshtastic_reliable_on_tx(&local, wire, pkt_len);
	}

	return send_packet_complete(&local, wire, pkt_len, ret, K_TIMEOUT_EQ(wait, K_FOREVER));
}

int meshtastic_send_data(uint32_t dest, uint32_t portnum, const uint8_t *payload,
			 size_t payload_len, k_timeout_t wait)
{
	struct meshtastic_packet packet = {
		.to = dest,
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
	};

	return meshtastic_send_packet(&packet, wait);
}

int meshtastic_send_text(uint32_t dest, const char *text)
{
	size_t len;

	if (text == NULL) {
		return -EINVAL;
	}

	len = strlen(text);
	if (len == 0U || len > MESHTASTIC_MAX_TEXT_LEN) {
		return -EINVAL;
	}

	return meshtastic_send_data(dest, MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)text, len,
				    K_FOREVER);
}

void meshtastic_set_recv_cb(meshtastic_recv_cb_t cb)
{
	mt.recv_cb = cb;
}

void meshtastic_set_event_cb(meshtastic_event_cb_t cb, void *user_data)
{
	mt.event_cb = cb;
	mt.event_user_data = user_data;
}

int meshtastic_get_status(struct meshtastic_status *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	*status = mt.status;
	status->initialized = mt.initialized;
	status->node_id = mt.node_id;

	return 0;
}

uint32_t meshtastic_get_node_id(void)
{
	return mt.node_id;
}

uint32_t meshtastic_runtime_frequency(void)
{
	return mt.frequency;
}

const char *meshtastic_runtime_channel_name(void)
{
	return meshtastic_channels_primary_name();
}

const char *meshtastic_get_channel_name(uint8_t index)
{
	return meshtastic_channels_get_name(index);
}

const uint8_t *meshtastic_runtime_psk(size_t *psk_len)
{
	static uint8_t psk_buf[32];

	if (psk_len != NULL) {
		*psk_len = mt.psk_len;
	}

	if (mt.psk_len > 0U) {
		memcpy(psk_buf, mt.psk, mt.psk_len);
		return psk_buf;
	}

	return meshtastic_default_psk;
}

uint8_t meshtastic_runtime_hop_limit(void)
{
	return mt.hop_limit;
}

#if !defined(CONFIG_MESHTASTIC_POSITION)
int meshtastic_send_position(uint32_t dest)
{
	ARG_UNUSED(dest);

	return -ENOTSUP;
}
#endif

#if !defined(CONFIG_MESHTASTIC_DEVICE_METRICS)
int meshtastic_send_device_metrics(uint32_t dest, k_timeout_t wait)
{
	ARG_UNUSED(dest);
	ARG_UNUSED(wait);

	return -ENOTSUP;
}
#endif

#if !defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS)
int meshtastic_send_environment(uint32_t dest, k_timeout_t wait)
{
	ARG_UNUSED(dest);
	ARG_UNUSED(wait);

	return -ENOTSUP;
}
#endif

#if !defined(CONFIG_MESHTASTIC_NODEDB)
size_t meshtastic_nodedb_count(void)
{
	return 0U;
}

int meshtastic_nodedb_get(uint32_t node_num, struct meshtastic_nodedb_node *out)
{
	ARG_UNUSED(node_num);
	ARG_UNUSED(out);

	return -ENOTSUP;
}

int meshtastic_nodedb_get_by_index(size_t index, struct meshtastic_nodedb_node *out)
{
	ARG_UNUSED(index);
	ARG_UNUSED(out);

	return -ENOTSUP;
}

int meshtastic_nodedb_set_favorite(uint32_t node_num, bool favorite)
{
	ARG_UNUSED(node_num);
	ARG_UNUSED(favorite);

	return -ENOTSUP;
}

int meshtastic_nodedb_set_ignored(uint32_t node_num, bool ignored)
{
	ARG_UNUSED(node_num);
	ARG_UNUSED(ignored);

	return -ENOTSUP;
}

int meshtastic_nodedb_remove(uint32_t node_num)
{
	ARG_UNUSED(node_num);

	return -ENOTSUP;
}
#endif

#if !defined(CONFIG_MESHTASTIC_NODEINFO)
int meshtastic_send_node_info(uint32_t dest)
{
	ARG_UNUSED(dest);

	return -ENOTSUP;
}
#endif
