/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_settings.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define STORE_RECORD_VERSION     1U
#define STORE_RECORD_VERSION_MIN 1U /* oldest record version still accepted on load */
#define STORE_RECORD_HEADER_LEN  4U
#define OWNER_LONG_NAME_LEN     40U
#define OWNER_SHORT_NAME_LEN    5U

struct store_owner_record {
	uint8_t version;
	char long_name[OWNER_LONG_NAME_LEN];
	char short_name[OWNER_SHORT_NAME_LEN];
	/* Appended after the initial release; loaded only if the stored blob is
	 * large enough (see setting_set_owner), so older records still load. */
	uint8_t is_licensed;
	uint8_t is_unmessagable;
} __packed;

/* Size of an owner record written before the licensed/unmessagable flags were
 * appended — used to accept older blobs without the trailing flags. */
#define STORE_OWNER_RECORD_V1_LEN offsetof(struct store_owner_record, is_licensed)

/* Persisted admin-set fixed position. A flat record (like the owner record)
 * rather than an encoded Position: only the handful of fields a fixed location
 * needs, plus an explicit valid byte so "no fixed position" round-trips without
 * relying on a sentinel coordinate. */
struct store_position_fixed_record {
	uint8_t version;
	uint8_t valid; /* 0 == no fixed position set */
	uint8_t has_latitude_i;
	uint8_t has_longitude_i;
	uint8_t has_altitude;
	uint8_t precision_bits;
	int32_t latitude_i;
	int32_t longitude_i;
	int32_t altitude;
} __packed;

struct named_config {
	const char *name;
	pb_size_t tag;
};

static const struct named_config config_names[] = {
	{"device", meshtastic_Config_device_tag},
	{"position", meshtastic_Config_position_tag},
	{"power", meshtastic_Config_power_tag},
	{"network", meshtastic_Config_network_tag},
	{"display", meshtastic_Config_display_tag},
	{"lora", meshtastic_Config_lora_tag},
	{"bluetooth", meshtastic_Config_bluetooth_tag},
	{"security", meshtastic_Config_security_tag},
	{"sessionkey", meshtastic_Config_sessionkey_tag},
	{"device_ui", meshtastic_Config_device_ui_tag},
};

static const struct named_config module_names[] = {
	{"mqtt", meshtastic_ModuleConfig_mqtt_tag},
	{"serial", meshtastic_ModuleConfig_serial_tag},
	{"external_notification", meshtastic_ModuleConfig_external_notification_tag},
	{"store_forward", meshtastic_ModuleConfig_store_forward_tag},
	{"range_test", meshtastic_ModuleConfig_range_test_tag},
	{"telemetry", meshtastic_ModuleConfig_telemetry_tag},
	{"canned_message", meshtastic_ModuleConfig_canned_message_tag},
	{"audio", meshtastic_ModuleConfig_audio_tag},
	{"remote_hardware", meshtastic_ModuleConfig_remote_hardware_tag},
	{"neighbor_info", meshtastic_ModuleConfig_neighbor_info_tag},
	{"ambient_lighting", meshtastic_ModuleConfig_ambient_lighting_tag},
	{"detection_sensor", meshtastic_ModuleConfig_detection_sensor_tag},
	{"paxcounter", meshtastic_ModuleConfig_paxcounter_tag},
	{"statusmessage", meshtastic_ModuleConfig_statusmessage_tag},
	{"traffic_management", meshtastic_ModuleConfig_traffic_management_tag},
	{"tak", meshtastic_ModuleConfig_tak_tag},
	{"mesh_beacon", meshtastic_ModuleConfig_mesh_beacon_tag},
};

static struct {
	struct k_mutex lock;
	bool lock_ready;
	char long_name[OWNER_LONG_NAME_LEN];
	char short_name[OWNER_SHORT_NAME_LEN];
	bool is_licensed;
	bool is_unmessagable;
	meshtastic_Channel channels[MESHTASTIC_MAX_CHANNELS];
	meshtastic_Config configs[ARRAY_SIZE(config_names)];
	meshtastic_ModuleConfig modules[ARRAY_SIZE(module_names)];
	bool has_fixed_position;
	meshtastic_Position fixed_position;
} store;

static void store_lock(void)
{
	if (!store.lock_ready) {
		k_mutex_init(&store.lock);
		store.lock_ready = true;
	}

	k_mutex_lock(&store.lock, K_FOREVER);
}

static void store_unlock(void)
{
	k_mutex_unlock(&store.lock);
}

static void copy_string(char *dst, size_t dst_len, const char *src)
{
	if (dst_len == 0U) {
		return;
	}

	dst[0] = '\0';
	if (src != NULL) {
		strncpy(dst, src, dst_len - 1U);
		dst[dst_len - 1U] = '\0';
	}
}

static void copy_fixed_string(char *dst, size_t dst_len, const char *src, size_t src_len)
{
	size_t copy_len = 0U;

	if (dst_len == 0U) {
		return;
	}

	while (copy_len < src_len && src[copy_len] != '\0') {
		copy_len++;
	}

	if (copy_len >= dst_len) {
		copy_len = dst_len - 1U;
	}

	memcpy(dst, src, copy_len);
	dst[copy_len] = '\0';
}

static int index_for_config_tag(pb_size_t tag)
{
	for (size_t i = 0; i < ARRAY_SIZE(config_names); i++) {
		if (config_names[i].tag == tag) {
			return (int)i;
		}
	}

	return -ENOENT;
}

static int index_for_module_tag(pb_size_t tag)
{
	for (size_t i = 0; i < ARRAY_SIZE(module_names); i++) {
		if (module_names[i].tag == tag) {
			return (int)i;
		}
	}

	return -ENOENT;
}

static int index_for_config_name(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(config_names); i++) {
		if (strcmp(config_names[i].name, name) == 0) {
			return (int)i;
		}
	}

	return -ENOENT;
}

static int index_for_module_name(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(module_names); i++) {
		if (strcmp(module_names[i].name, name) == 0) {
			return (int)i;
		}
	}

	return -ENOENT;
}

static int encode_record(const pb_msgdesc_t *fields, const void *msg, void *buf, size_t buf_len)
{
	pb_ostream_t stream;

	if (buf == NULL || buf_len < STORE_RECORD_HEADER_LEN) {
		return -EINVAL;
	}

	stream = pb_ostream_from_buffer((uint8_t *)buf + STORE_RECORD_HEADER_LEN,
					buf_len - STORE_RECORD_HEADER_LEN);
	if (!pb_encode(&stream, fields, msg)) {
		LOG_WRN("Meshtastic settings encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	((uint8_t *)buf)[0] = STORE_RECORD_VERSION;
	((uint8_t *)buf)[1] = 0U;
	sys_put_le16((uint16_t)stream.bytes_written, (uint8_t *)buf + 2);

	return (int)(stream.bytes_written + STORE_RECORD_HEADER_LEN);
}

static int decode_record(const pb_msgdesc_t *fields, const void *buf, size_t len, void *msg)
{
	pb_istream_t stream;
	uint16_t payload_len;

	if (buf == NULL || msg == NULL || len < STORE_RECORD_HEADER_LEN) {
		return -EINVAL;
	}

	/* Accept any record version in [MIN, CUR] rather than an exact match. The
	 * record framing (this header + a length-prefixed nanopb payload) is
	 * self-describing, and nanopb already absorbs proto field evolution (unknown
	 * fields ignored, missing fields defaulted), so adding a config field needs
	 * no version bump. Bump STORE_RECORD_VERSION only for a change to the framing
	 * itself, and advance STORE_RECORD_VERSION_MIN only to drop a format that can
	 * no longer be read. Because a bump keeps loading every version back to MIN,
	 * it never silently reverts stored config to compile-time defaults — which
	 * for config/security would regenerate the X25519 identity and break PKC with
	 * every peer that cached the old key. */
	if (((const uint8_t *)buf)[0] < STORE_RECORD_VERSION_MIN ||
	    ((const uint8_t *)buf)[0] > STORE_RECORD_VERSION) {
		return -EINVAL;
	}

	payload_len = sys_get_le16((const uint8_t *)buf + 2);
	if ((size_t)payload_len != len - STORE_RECORD_HEADER_LEN) {
		return -EINVAL;
	}

	stream =
		pb_istream_from_buffer((const uint8_t *)buf + STORE_RECORD_HEADER_LEN, payload_len);
	if (!pb_decode(&stream, fields, msg)) {
		LOG_WRN("Meshtastic settings decode failed: %s", PB_GET_ERROR(&stream));
		return -EINVAL;
	}

	return 0;
}

static int parse_channel_index(const char *name, uint8_t *index)
{
	unsigned long parsed;
	char *end;

	if (name == NULL || index == NULL || *name == '\0') {
		return -EINVAL;
	}

	parsed = strtoul(name, &end, 10);
	if (*end != '\0' || parsed >= MESHTASTIC_MAX_CHANNELS) {
		return -EINVAL;
	}

	*index = (uint8_t)parsed;
	return 0;
}

static bool channel_is_valid(const meshtastic_Channel *channel)
{
	if (channel == NULL) {
		return false;
	}

	if (channel->role > meshtastic_Channel_Role_SECONDARY) {
		return false;
	}

	if (channel->settings.psk.size != 0U && channel->settings.psk.size != 1U &&
	    channel->settings.psk.size != 16U && channel->settings.psk.size != 32U) {
		return false;
	}

	return true;
}

static bool config_is_valid(const meshtastic_Config *config)
{
	if (config == NULL || index_for_config_tag(config->which_payload_variant) < 0) {
		return false;
	}

	switch (config->which_payload_variant) {
	case meshtastic_Config_device_tag:
		return config->payload_variant.device.role <=
			       meshtastic_Config_DeviceConfig_Role_CLIENT_BASE &&
		       config->payload_variant.device.rebroadcast_mode <=
			       meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY;
	case meshtastic_Config_lora_tag:
		/* Reject the 2.4 GHz region: the port has no wide-LoRa-capable radio
		 * abstraction, so LORA_24 cannot be honored (firmware gates this on
		 * RadioLibInterface::wideLora(); AdminModule.cpp). */
		if (config->payload_variant.lora.region ==
		    meshtastic_Config_LoRaConfig_RegionCode_LORA_24) {
			return false;
		}
		/* RF-compliance gate. Any region the reference defines is accepted,
		 * but the band it implies is then enforced: this replaces an earlier
		 * hard lock to US, and the guards below are what make that safe.
		 * Applies to both the admin-set and NVS-load paths (every
		 * config_is_valid caller), so a rejected config can neither be stored
		 * nor loaded. */
		{
			meshtastic_Config_LoRaConfig_RegionCode region =
				config->payload_variant.lora.region;
			struct meshtastic_region_info info;

			/* UNSET keeps the compile-time default and needs no band check. */
			if (region != meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
				if (meshtastic_region_info(region, &info) < 0) {
					/* Obsolete or unknown to the reference table: there
					 * is no band to validate against, so refuse rather
					 * than transmit somewhere unverified. */
					return false;
				}

				/* Amateur (ITU) allocations are rejected outright, not
				 * merely gated on the licensed flag, for two independent
				 * reasons.
				 *
				 * Regulatory: the reference disables encryption in
				 * licensed mode — it suppresses PKI keygen and forces
				 * rebroadcast_mode to LOCAL_ONLY (NodeDB.cpp:577, :3862)
				 * — because amateur service forbids transmissions
				 * "encoded for the purpose of obscuring their meaning"
				 * (FCC Part 97.113(a)(4) and equivalents). This port does
				 * neither: keygen is unconditional and rebroadcast_mode is
				 * never forced. Honouring the licensed flag without those
				 * obligations would put encrypted traffic on amateur
				 * spectrum, so the flag alone is not enough to permit it.
				 *
				 * Hardware: the 2 m allocations are 144-146 MHz, below the
				 * SX1262's 150 MHz lower limit — it cannot tune there at
				 * all. The 125 cm and 70 cm allocations are within the
				 * chip's range, but the boards' front-end module and
				 * matching network are built for 863-928 MHz, so they
				 * would be severely mismatched.
				 *
				 * Supporting these needs the encryption obligations
				 * implemented AND hardware that can reach the band. Until
				 * both hold, refusing is the honest answer.
				 */
				if (info.licensed_only) {
					LOG_WRN("region %d is an amateur allocation; "
						"unsupported (see config_is_valid)",
						(int)region);
					return false;
				}

				if (config->payload_variant.lora.override_frequency > 0.0f) {
					uint32_t hz = (uint32_t)(
						config->payload_variant.lora
							.override_frequency *
						1000000.0f);

					/* An override must stay inside the selected
					 * region's allocation — the point of allowing
					 * other regions is not to allow arbitrary
					 * frequencies. */
					if (hz < info.freq_start_hz ||
					    hz > info.freq_end_hz) {
						LOG_WRN("override %u Hz outside region %d "
							"(%u-%u Hz)",
							hz, (int)region,
							info.freq_start_hz,
							info.freq_end_hz);
						return false;
					}
				}
			} else if (config->payload_variant.lora.override_frequency > 0.0f) {
				/* No region to bound the override against. */
				return false;
			}
		}
		return config->payload_variant.lora.hop_limit <= 7U;
	case meshtastic_Config_bluetooth_tag:
		return config->payload_variant.bluetooth.mode <=
		       meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN;
	default:
		return true;
	}
}

static bool module_is_valid(const meshtastic_ModuleConfig *module)
{
	return module != NULL && index_for_module_tag(module->which_payload_variant) >= 0;
}

static void init_config_entry(size_t index)
{
	store.configs[index] = (meshtastic_Config)meshtastic_Config_init_zero;
	store.configs[index].which_payload_variant = config_names[index].tag;
}

static void init_module_entry(size_t index)
{
	store.modules[index] = (meshtastic_ModuleConfig)meshtastic_ModuleConfig_init_zero;
	store.modules[index].which_payload_variant = module_names[index].tag;
}

static void seed_config_defaults(const struct meshtastic_config *cfg)
{
	int idx;

	for (size_t i = 0; i < ARRAY_SIZE(config_names); i++) {
		init_config_entry(i);
	}

	idx = index_for_config_tag(meshtastic_Config_device_tag);
	store.configs[idx].payload_variant.device.role = meshtastic_device_role();
	store.configs[idx].payload_variant.device.rebroadcast_mode = meshtastic_rebroadcast_mode();
	store.configs[idx].payload_variant.device.serial_enabled =
		IS_ENABLED(CONFIG_MESHTASTIC_SERIAL);

	idx = index_for_config_tag(meshtastic_Config_position_tag);
	store.configs[idx].payload_variant.position.gps_mode =
		IS_ENABLED(CONFIG_MESHTASTIC_GNSS)
			? meshtastic_Config_PositionConfig_GpsMode_ENABLED
			: meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT;

	idx = index_for_config_tag(meshtastic_Config_lora_tag);
	store.configs[idx].payload_variant.lora.use_preset = true;
	store.configs[idx].payload_variant.lora.config_ok_to_mqtt =
		IS_ENABLED(CONFIG_MESHTASTIC_DEFAULT_OK_TO_MQTT);
	store.configs[idx].payload_variant.lora.modem_preset =
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	/* Declared, not inferred. The region used to be derived from the
	 * compile-time frequency, which became circular once the frequency
	 * started being derived from the region and channel name. */
	store.configs[idx].payload_variant.lora.region =
		(meshtastic_Config_LoRaConfig_RegionCode)CONFIG_MESHTASTIC_DEFAULT_REGION;
	store.configs[idx].payload_variant.lora.hop_limit = mt.hop_limit;
	store.configs[idx].payload_variant.lora.tx_enabled = true;
	store.configs[idx].payload_variant.lora.tx_power = mt.tx_power;

	idx = index_for_config_tag(meshtastic_Config_bluetooth_tag);
	store.configs[idx].payload_variant.bluetooth.enabled = IS_ENABLED(CONFIG_MESHTASTIC_BLE);
#if defined(CONFIG_MESHTASTIC_BLE_FIXED_PASSKEY)
	store.configs[idx].payload_variant.bluetooth.mode =
		meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN;
	store.configs[idx].payload_variant.bluetooth.fixed_pin = CONFIG_MESHTASTIC_BLE_PASSKEY;
#else
	store.configs[idx].payload_variant.bluetooth.mode =
		meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN;
#endif
}

static void seed_module_defaults(void)
{
	int idx;

	for (size_t i = 0; i < ARRAY_SIZE(module_names); i++) {
		init_module_entry(i);
	}

	idx = index_for_module_tag(meshtastic_ModuleConfig_mqtt_tag);
	store.modules[idx].payload_variant.mqtt.enabled = IS_ENABLED(CONFIG_MESHTASTIC_MQTT);
#if defined(CONFIG_MESHTASTIC_MQTT)
	copy_string(store.modules[idx].payload_variant.mqtt.address,
		    sizeof(store.modules[idx].payload_variant.mqtt.address),
		    CONFIG_MESHTASTIC_MQTT_BROKER_HOST);
	copy_string(store.modules[idx].payload_variant.mqtt.username,
		    sizeof(store.modules[idx].payload_variant.mqtt.username),
		    CONFIG_MESHTASTIC_MQTT_USERNAME);
	copy_string(store.modules[idx].payload_variant.mqtt.password,
		    sizeof(store.modules[idx].payload_variant.mqtt.password),
		    CONFIG_MESHTASTIC_MQTT_PASSWORD);
	copy_string(store.modules[idx].payload_variant.mqtt.root,
		    sizeof(store.modules[idx].payload_variant.mqtt.root),
		    CONFIG_MESHTASTIC_MQTT_ROOT);
	store.modules[idx].payload_variant.mqtt.encryption_enabled =
		IS_ENABLED(CONFIG_MESHTASTIC_MQTT_ENCRYPTION_ENABLED);
	store.modules[idx].payload_variant.mqtt.map_reporting_enabled =
		IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT);
#if defined(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)
	store.modules[idx].payload_variant.mqtt.has_map_report_settings = true;
	store.modules[idx].payload_variant.mqtt.map_report_settings.publish_interval_secs =
		CONFIG_MESHTASTIC_MQTT_MAP_REPORT_INTERVAL_SEC;
	store.modules[idx].payload_variant.mqtt.map_report_settings.position_precision =
		CONFIG_MESHTASTIC_MQTT_MAP_REPORT_POSITION_PRECISION;
	store.modules[idx].payload_variant.mqtt.map_report_settings.should_report_location = true;
#endif
#endif
}

int meshtastic_config_store_seed(const struct meshtastic_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	store_lock();

	copy_string(store.long_name, sizeof(store.long_name),
		    (cfg->long_name != NULL) ? cfg->long_name : CONFIG_MESHTASTIC_NODE_LONG_NAME);
	copy_string(store.short_name, sizeof(store.short_name),
		    (cfg->short_name != NULL) ? cfg->short_name
					      : CONFIG_MESHTASTIC_NODE_SHORT_NAME);

	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *channel = meshtastic_channels_get(i);

		store.channels[i] = (channel != NULL)
					    ? *channel
					    : (meshtastic_Channel)meshtastic_Channel_init_zero;
		store.channels[i].index = (int8_t)i;
	}

	seed_config_defaults(cfg);
	seed_module_defaults();

	store_unlock();
	return 0;
}

int meshtastic_config_store_apply_core(void)
{
	meshtastic_Config_DeviceConfig device;
	meshtastic_Config_LoRaConfig lora;
	meshtastic_Channel channels[MESHTASTIC_MAX_CHANNELS];
	int device_idx;
	int lora_idx;
	int ret;

	store_lock();

	mt.long_name = store.long_name;
	mt.short_name = store.short_name;
	memcpy(channels, store.channels, sizeof(channels));

	device_idx = index_for_config_tag(meshtastic_Config_device_tag);
	lora_idx = index_for_config_tag(meshtastic_Config_lora_tag);
	device = store.configs[device_idx].payload_variant.device;
	lora = store.configs[lora_idx].payload_variant.lora;

	store_unlock();

	/* Must precede the channel loop: an unnamed channel takes the active
	 * preset's display name, and set_slot hashes that name as it stores each
	 * slot. Applying the preset afterwards would hash every channel against
	 * the previous preset. */
	mt.modem_preset = lora.modem_preset;
	mt.use_preset = lora.use_preset;
	mt.config_ok_to_mqtt = lora.config_ok_to_mqtt;

	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		ret = meshtastic_channels_set_slot(i, &channels[i]);
		if (ret < 0) {
			return ret;
		}
	}

	meshtastic_set_device_role(device.role);
	meshtastic_set_rebroadcast_mode(device.rebroadcast_mode);

	if (lora.hop_limit >= 1U && lora.hop_limit <= 7U) {
		mt.hop_limit = (uint8_t)lora.hop_limit;
	}

	if (lora.tx_power != 0) {
		mt.tx_power = lora.tx_power;
	}

	/* Clamp TX power to the region's limit unless the operator is licensed,
	 * mirroring the reference (RadioInterface.cpp: power > powerLimit &&
	 * !is_licensed). This matters more now that regions other than US are
	 * selectable: their limits are markedly lower — 30 dBm in the US against
	 * 27 in EU_868, 14 in ANZ_433, 10 in EU_433.
	 */
	{
		struct meshtastic_region_info info;
		meshtastic_Config_LoRaConfig_RegionCode region = lora.region;

		if (region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
			region = (meshtastic_Config_LoRaConfig_RegionCode)
				CONFIG_MESHTASTIC_DEFAULT_REGION;
		}
		if (meshtastic_region_info(region, &info) == 0 &&
		    info.power_limit_dbm > 0 && mt.tx_power > info.power_limit_dbm &&
		    !store.is_licensed) {
			LOG_WRN("TX power %d dBm exceeds region %d limit; clamping to %d",
				mt.tx_power, (int)region, info.power_limit_dbm);
			mt.tx_power = info.power_limit_dbm;
		}
	}

	/* Resolve the modem preset. wide_lora is hardcoded false: it is a
	 * per-region property, and the only region that sets it is the 2.4 GHz
	 * band, which no supported board targets. It becomes a real lookup when
	 * the region table lands (parity: radio D3).
	 */
	if (lora.use_preset) {
		(void)meshtastic_preset_to_params(lora.modem_preset, false, &mt.modem);
	} else {
		/* Custom SF/BW/CR is not honoured yet: the bandwidth field carries
		 * "special" codes needing the reference's bwCodeToKHz decode, which
		 * is not ported. Say so rather than silently running the preset
		 * config while the app displays custom values.
		 */
		LOG_WRN("lora.use_preset=false is not supported; keeping preset %d",
			(int)lora.modem_preset);
		(void)meshtastic_preset_to_params(lora.modem_preset, false, &mt.modem);
	}

	if (lora.override_frequency > 0.0f) {
		uint32_t hz = (uint32_t)(lora.override_frequency * 1000000.0f);

		/* Defense in depth (US-only firmware): ignore an out-of-band override
		 * even if one reached here without config_is_valid gating it. */
		if (hz >= 902000000U && hz <= 928000000U) {
			mt.frequency = hz;
		}
	} else {
		struct meshtastic_freq_plan plan;
		meshtastic_Config_LoRaConfig_RegionCode region = lora.region;

		/* UNSET means "keep the compile-time default". */
		if (region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
			region = (meshtastic_Config_LoRaConfig_RegionCode)
				CONFIG_MESHTASTIC_DEFAULT_REGION;
		}

		/* The frequency follows the PRIMARY channel's name, which is why this
		 * runs after the channel loop. Pinning one frequency per region was
		 * wrong even within a single region: stock firmware puts a node named
		 * "MyMesh" on slot djb2("MyMesh") % 104, not on LongFast's slot, so a
		 * renamed channel could not hear stock radios at all.
		 */
		ret = meshtastic_region_freq_plan(region, mt.modem_preset,
						  meshtastic_channels_primary_name(),
						  mt.use_preset, &plan);
		if (ret == 0) {
			mt.frequency = plan.frequency_hz;
			LOG_DBG("region %d: slot %u/%u -> %u Hz (duty %d%%)", (int)region,
				plan.slot, plan.num_slots, plan.frequency_hz,
				(int)plan.duty_cycle_pct);
		} else {
			/* No plan means no band data for this region (obsolete or
			 * unknown). Keep the previous frequency rather than invent one;
			 * transmitting on a guess is worse than not moving. */
			LOG_WRN("no frequency plan for region %d (%d); keeping %u Hz",
				(int)region, ret, mt.frequency);
		}
	}

	return 0;
}

const char *meshtastic_config_store_long_name(void)
{
	return store.long_name;
}

const char *meshtastic_config_store_short_name(void)
{
	return store.short_name;
}

int meshtastic_config_store_get_channel(uint8_t index, meshtastic_Channel *channel)
{
	if (index >= MESHTASTIC_MAX_CHANNELS || channel == NULL) {
		return -EINVAL;
	}

	store_lock();
	*channel = store.channels[index];
	store_unlock();

	return 0;
}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static bool save_suppressed;
#endif

void meshtastic_config_store_set_save_suppressed(bool suppressed)
{
#if defined(CONFIG_MESHTASTIC_SETTINGS)
	save_suppressed = suppressed;
#else
	(void)suppressed;
#endif
}

/*
 * Route setter persistence through here so an open admin edit transaction can
 * defer the coalesced flash write until commit
 * (see meshtastic_config_store_set_save_suppressed()).
 */
static void store_schedule_save(void)
{
#if defined(CONFIG_MESHTASTIC_SETTINGS)
	if (!save_suppressed) {
		meshtastic_settings_schedule_save();
	}
#endif
}

int meshtastic_config_store_set_channel(uint8_t index, const meshtastic_Channel *channel)
{
	int ret;

	if (index >= MESHTASTIC_MAX_CHANNELS || !channel_is_valid(channel)) {
		return -EINVAL;
	}

	store_lock();
	store.channels[index] = *channel;
	store.channels[index].index = (int8_t)index;
	store_unlock();

	ret = meshtastic_channels_set_slot(index, channel);
	if (ret < 0) {
		return ret;
	}

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_get_config(pb_size_t tag, meshtastic_Config *config)
{
	int idx = index_for_config_tag(tag);

	if (idx < 0 || config == NULL) {
		return -EINVAL;
	}

	store_lock();
	*config = store.configs[idx];
	store_unlock();

	return 0;
}

int meshtastic_config_store_set_config(const meshtastic_Config *config)
{
	int idx;
	int ret;

	if (!config_is_valid(config)) {
		return -EINVAL;
	}

	idx = index_for_config_tag(config->which_payload_variant);

	store_lock();
	store.configs[idx] = *config;
	store_unlock();

	ret = meshtastic_config_store_apply_core();
	if (ret < 0) {
		return ret;
	}

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_set_position_fixed(bool fixed)
{
	int idx = index_for_config_tag(meshtastic_Config_position_tag);

	if (idx < 0) {
		return -EINVAL;
	}

	store_lock();
	store.configs[idx].payload_variant.position.fixed_position = fixed;
	store_unlock();

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_set_fixed_position(const meshtastic_Position *pos)
{
	if (pos == NULL) {
		return -EINVAL;
	}

	store_lock();
	store.fixed_position = *pos;
	store.has_fixed_position = true;
	store_unlock();

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_clear_fixed_position(void)
{
	store_lock();
	store.fixed_position = (meshtastic_Position)meshtastic_Position_init_zero;
	store.has_fixed_position = false;
	store_unlock();

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_get_fixed_position(meshtastic_Position *pos)
{
	int ret = -ENOENT;

	if (pos == NULL) {
		return -EINVAL;
	}

	store_lock();
	if (store.has_fixed_position) {
		*pos = store.fixed_position;
		ret = 0;
	}
	store_unlock();

	return ret;
}

int meshtastic_config_store_get_module(pb_size_t tag, meshtastic_ModuleConfig *module)
{
	int idx = index_for_module_tag(tag);

	if (idx < 0 || module == NULL) {
		return -EINVAL;
	}

	store_lock();
	*module = store.modules[idx];
	store_unlock();

	return 0;
}

int meshtastic_config_store_set_module(const meshtastic_ModuleConfig *module)
{
	int idx;

	if (!module_is_valid(module)) {
		return -EINVAL;
	}

	idx = index_for_module_tag(module->which_payload_variant);

	store_lock();
	store.modules[idx] = *module;
	store_unlock();

	store_schedule_save();
	return 0;
}

int meshtastic_config_store_get_device_ui(meshtastic_DeviceUIConfig *device_ui)
{
	meshtastic_Config config;
	int ret;

	if (device_ui == NULL) {
		return -EINVAL;
	}

	ret = meshtastic_config_store_get_config(meshtastic_Config_device_ui_tag, &config);
	if (ret < 0) {
		return ret;
	}

	*device_ui = config.payload_variant.device_ui;
	return 0;
}

int meshtastic_config_store_set_device_role(meshtastic_Config_DeviceConfig_Role role)
{
	int idx = index_for_config_tag(meshtastic_Config_device_tag);

	if (role > meshtastic_Config_DeviceConfig_Role_CLIENT_BASE) {
		return -EINVAL;
	}

	store_lock();
	store.configs[idx].payload_variant.device.role = role;
	store_unlock();

	meshtastic_set_device_role(role);
	store_schedule_save();
	return 0;
}

int meshtastic_config_store_set_rebroadcast_mode(
	meshtastic_Config_DeviceConfig_RebroadcastMode mode)
{
	int idx = index_for_config_tag(meshtastic_Config_device_tag);

	if (mode > meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY) {
		return -EINVAL;
	}

	store_lock();
	store.configs[idx].payload_variant.device.rebroadcast_mode = mode;
	store_unlock();

	meshtastic_set_rebroadcast_mode(mode);
	store_schedule_save();
	return 0;
}

int meshtastic_config_store_set_owner(const meshtastic_User *user)
{
	if (user == NULL) {
		return -EINVAL;
	}

	store_lock();
	copy_string(store.long_name, sizeof(store.long_name), user->long_name);
	copy_string(store.short_name, sizeof(store.short_name), user->short_name);
	store.is_licensed = user->is_licensed;
	store.is_unmessagable = user->has_is_unmessagable && user->is_unmessagable;
	store_unlock();

	/* mt.long_name / mt.short_name already point at store.* (apply_core). */
	store_schedule_save();
	return 0;
}

void meshtastic_config_store_get_owner_flags(bool *is_licensed, bool *is_unmessagable)
{
	store_lock();
	if (is_licensed != NULL) {
		*is_licensed = store.is_licensed;
	}
	if (is_unmessagable != NULL) {
		*is_unmessagable = store.is_unmessagable;
	}
	store_unlock();
}

static int setting_get_fixed_position(void *buf, size_t buf_len)
{
	struct store_position_fixed_record rec = {.version = STORE_RECORD_VERSION};

	if (buf_len < sizeof(rec)) {
		return -ENOMEM;
	}

	store_lock();
	if (store.has_fixed_position) {
		rec.valid = 1U;
		rec.has_latitude_i = store.fixed_position.has_latitude_i ? 1U : 0U;
		rec.has_longitude_i = store.fixed_position.has_longitude_i ? 1U : 0U;
		rec.has_altitude = store.fixed_position.has_altitude ? 1U : 0U;
		rec.precision_bits = (uint8_t)store.fixed_position.precision_bits;
		rec.latitude_i = store.fixed_position.latitude_i;
		rec.longitude_i = store.fixed_position.longitude_i;
		rec.altitude = store.fixed_position.altitude;
	}
	store_unlock();

	memcpy(buf, &rec, sizeof(rec));
	return sizeof(rec);
}

static int setting_set_fixed_position(const void *buf, size_t len)
{
	const struct store_position_fixed_record *rec = buf;

	if (len < sizeof(*rec) || rec->version != STORE_RECORD_VERSION) {
		return -EINVAL;
	}

	store_lock();
	store.fixed_position = (meshtastic_Position)meshtastic_Position_init_zero;
	if (rec->valid) {
		store.fixed_position.has_latitude_i = (rec->has_latitude_i != 0U);
		store.fixed_position.latitude_i = rec->latitude_i;
		store.fixed_position.has_longitude_i = (rec->has_longitude_i != 0U);
		store.fixed_position.longitude_i = rec->longitude_i;
		store.fixed_position.has_altitude = (rec->has_altitude != 0U);
		store.fixed_position.altitude = rec->altitude;
		store.fixed_position.precision_bits = rec->precision_bits;
		store.fixed_position.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
		store.has_fixed_position = true;
	} else {
		store.has_fixed_position = false;
	}
	store_unlock();

	return 0;
}

static int setting_get_owner(void *buf, size_t buf_len)
{
	struct store_owner_record owner = {.version = STORE_RECORD_VERSION};

	if (buf_len < sizeof(owner)) {
		return -ENOMEM;
	}

	store_lock();
	copy_string(owner.long_name, sizeof(owner.long_name), store.long_name);
	copy_string(owner.short_name, sizeof(owner.short_name), store.short_name);
	owner.is_licensed = store.is_licensed ? 1U : 0U;
	owner.is_unmessagable = store.is_unmessagable ? 1U : 0U;
	store_unlock();

	memcpy(buf, &owner, sizeof(owner));
	return sizeof(owner);
}

static int setting_set_owner(const void *buf, size_t len)
{
	const struct store_owner_record *owner = buf;

	/* Accept the pre-flags record (names only) as well as the current one, so
	 * an upgrade keeps the stored owner instead of resetting it. */
	if (len < STORE_OWNER_RECORD_V1_LEN || owner->version != STORE_RECORD_VERSION) {
		return -EINVAL;
	}

	store_lock();
	copy_fixed_string(store.long_name, sizeof(store.long_name), owner->long_name,
			  sizeof(owner->long_name));
	copy_fixed_string(store.short_name, sizeof(store.short_name), owner->short_name,
			  sizeof(owner->short_name));
	if (len >= sizeof(*owner)) {
		store.is_licensed = (owner->is_licensed != 0U);
		store.is_unmessagable = (owner->is_unmessagable != 0U);
	} else {
		store.is_licensed = false;
		store.is_unmessagable = false;
	}
	store_unlock();

	return 0;
}

int meshtastic_config_store_setting_get(const char *key, void *buf, size_t buf_len)
{
	uint8_t index;
	int idx;
	int ret;

	if (key == NULL || buf == NULL) {
		return -EINVAL;
	}

	if (strcmp(key, "owner") == 0) {
		return setting_get_owner(buf, buf_len);
	}

	if (strcmp(key, "position_fixed") == 0) {
		return setting_get_fixed_position(buf, buf_len);
	}

	if (strncmp(key, "channel/", strlen("channel/")) == 0) {
		ret = parse_channel_index(key + strlen("channel/"), &index);
		if (ret < 0) {
			return ret;
		}

		store_lock();
		ret = encode_record(meshtastic_Channel_fields, &store.channels[index], buf,
				    buf_len);
		store_unlock();
		return ret;
	}

	if (strncmp(key, "config/", strlen("config/")) == 0) {
		idx = index_for_config_name(key + strlen("config/"));
		if (idx < 0) {
			return idx;
		}

		store_lock();
		ret = encode_record(meshtastic_Config_fields, &store.configs[idx], buf, buf_len);
		store_unlock();
		return ret;
	}

	if (strncmp(key, "module/", strlen("module/")) == 0) {
		idx = index_for_module_name(key + strlen("module/"));
		if (idx < 0) {
			return idx;
		}

		store_lock();
		ret = encode_record(meshtastic_ModuleConfig_fields, &store.modules[idx], buf,
				    buf_len);
		store_unlock();
		return ret;
	}

	return -ENOENT;
}

int meshtastic_config_store_setting_set(const char *key, const void *buf, size_t len)
{
	meshtastic_Channel channel = meshtastic_Channel_init_zero;
	meshtastic_Config config = meshtastic_Config_init_zero;
	meshtastic_ModuleConfig module = meshtastic_ModuleConfig_init_zero;
	uint8_t index;
	int idx;
	int ret;

	if (key == NULL || buf == NULL) {
		return -EINVAL;
	}

	if (strcmp(key, "owner") == 0) {
		return setting_set_owner(buf, len);
	}

	if (strcmp(key, "position_fixed") == 0) {
		return setting_set_fixed_position(buf, len);
	}

	if (strncmp(key, "channel/", strlen("channel/")) == 0) {
		ret = parse_channel_index(key + strlen("channel/"), &index);
		if (ret < 0) {
			return ret;
		}

		ret = decode_record(meshtastic_Channel_fields, buf, len, &channel);
		if (ret < 0 || !channel_is_valid(&channel)) {
			return -EINVAL;
		}

		channel.index = (int8_t)index;
		store_lock();
		store.channels[index] = channel;
		store_unlock();
		return 0;
	}

	if (strncmp(key, "config/", strlen("config/")) == 0) {
		idx = index_for_config_name(key + strlen("config/"));
		if (idx < 0) {
			return idx;
		}

		ret = decode_record(meshtastic_Config_fields, buf, len, &config);
		if (ret < 0 || config.which_payload_variant != config_names[idx].tag ||
		    !config_is_valid(&config)) {
			return -EINVAL;
		}

		store_lock();
		store.configs[idx] = config;
		store_unlock();
		return 0;
	}

	if (strncmp(key, "module/", strlen("module/")) == 0) {
		idx = index_for_module_name(key + strlen("module/"));
		if (idx < 0) {
			return idx;
		}

		ret = decode_record(meshtastic_ModuleConfig_fields, buf, len, &module);
		if (ret < 0 || module.which_payload_variant != module_names[idx].tag ||
		    !module_is_valid(&module)) {
			return -EINVAL;
		}

		store_lock();
		store.modules[idx] = module;
		store_unlock();
		return 0;
	}

	return -ENOENT;
}

static int export_one(int (*export_func)(const char *name, const void *val, size_t val_len),
		      const char *name)
{
	uint8_t buf[MESHTASTIC_STORE_VALUE_MAX];
	int len;

	len = meshtastic_config_store_setting_get(name, buf, sizeof(buf));
	if (len < 0) {
		return len;
	}

	return export_func(name, buf, (size_t)len);
}

int meshtastic_config_store_export(int (*export_func)(const char *name, const void *val,
						      size_t val_len))
{
	char name[40];
	int ret;

	if (export_func == NULL) {
		return -EINVAL;
	}

	ret = export_one(export_func, "owner");
	if (ret < 0) {
		return ret;
	}

	ret = export_one(export_func, "position_fixed");
	if (ret < 0) {
		return ret;
	}

	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		snprintk(name, sizeof(name), "channel/%u", i);
		ret = export_one(export_func, name);
		if (ret < 0) {
			return ret;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(config_names); i++) {
		snprintk(name, sizeof(name), "config/%s", config_names[i].name);
		ret = export_one(export_func, name);
		if (ret < 0) {
			return ret;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(module_names); i++) {
		snprintk(name, sizeof(name), "module/%s", module_names[i].name);
		ret = export_one(export_func, name);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
