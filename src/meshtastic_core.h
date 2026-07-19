/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_CORE_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_CORE_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/util.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/telemetry.pb.h"

#include "meshtastic_region_presets.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESHTASTIC_FROMRADIO_NONE 0U

#define MESHTASTIC_PKT_MAX     255U
#define MESHTASTIC_HDR_LEN     16U
#define MESHTASTIC_PAYLOAD_MAX (MESHTASTIC_PKT_MAX - MESHTASTIC_HDR_LEN)

struct meshtastic_dup_entry {
	uint32_t src;
	uint32_t id;
	uint32_t ms;       /* k_uptime_get_32() when recorded, for TTL expiry */
	uint8_t hop_limit; /* highest hop budget seen for this (src,id) */
};

struct meshtastic_context {
	const struct device *lora_dev;
	uint32_t node_id;
	uint8_t psk[32];
	size_t psk_len;
	uint8_t ch_hash;
	uint8_t hop_limit;
	int8_t tx_power;
	uint32_t frequency;
	struct meshtastic_modem_params modem; /* resolved from the active preset */
	/* The active preset is kept alongside its resolved parameters because the
	 * preset's display name is itself protocol data: it stands in for an empty
	 * channel name when hashing the channel and when picking the frequency
	 * slot. */
	meshtastic_Config_LoRaConfig_ModemPreset modem_preset;
	bool use_preset;
	/* config.lora.config_ok_to_mqtt: our consent to having our own traffic
	 * republished to MQTT by any gateway that hears it. Stamped into every
	 * packet we originate. */
	bool config_ok_to_mqtt;
	const char *channel_name;
	const char *long_name;
	const char *short_name;
	meshtastic_recv_cb_t recv_cb;
	meshtastic_event_cb_t event_cb;
	void *event_user_data;
	uint32_t next_pkt_id;
	uint32_t next_fromradio_id;
	struct meshtastic_dup_entry dup_cache[CONFIG_MESHTASTIC_DUP_CACHE_SIZE];
	uint8_t dup_head;
	struct k_mutex lock;
	struct meshtastic_status status;
	bool initialized;
	bool radio_rx_armed;
};

struct meshtastic_workspace {
	struct k_mutex lock;
	uint8_t pb_buf[MESHTASTIC_PAYLOAD_MAX];
	uint8_t enc_buf[MESHTASTIC_PAYLOAD_MAX + 16U];
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint8_t rx_dec[MESHTASTIC_PAYLOAD_MAX + 16U];
};

extern struct meshtastic_context mt;
extern struct meshtastic_workspace mt_ws;

uint32_t meshtastic_allocate_packet_id(void);
uint32_t meshtastic_next_fromradio_id(void);
void meshtastic_emit_event(enum meshtastic_event_type type, int err,
			   const struct meshtastic_packet *packet);
const char *meshtastic_long_name(void);
const char *meshtastic_short_name(void);
meshtastic_HardwareModel meshtastic_hw_model(void);
void meshtastic_fill_user(meshtastic_User *user);
void meshtastic_fill_device_metadata(meshtastic_DeviceMetadata *md);
uint32_t meshtastic_runtime_frequency(void);
const char *meshtastic_runtime_channel_name(void);
const uint8_t *meshtastic_runtime_psk(size_t *psk_len);
uint8_t meshtastic_runtime_hop_limit(void);
void meshtastic_set_ble_connected(bool connected);

int meshtastic_radio_init(void);

/**
 * @brief Convert a bandwidth in Hz to the LoRa driver's bandwidth code.
 *
 * The driver's codes are kHz-labelled and lossy — @c BW_62_KHZ is 62.5 kHz and
 * @c BW_1600_KHZ is 1625 kHz — so this is a lookup, not arithmetic.
 *
 * @return the driver code, or -EINVAL if no code represents @p hz.
 */
int meshtastic_radio_bw_hz_to_code(uint32_t bandwidth_hz);

/**
 * @brief Convert a 4/5..4/8 coding rate to the LoRa driver's coding-rate code.
 *
 * The reference firmware carries coding rate as 5..8; the driver's enum is
 * 1..4. Getting this wrong is silent — the radio configures happily at the
 * wrong rate — so the conversion is exposed to be asserted directly.
 *
 * @return the driver code, or -EINVAL if @p coding_rate is outside 5..8.
 */
int meshtastic_radio_cr_to_code(uint8_t coding_rate);

struct meshtastic_settings_apply {
	const char *name;
	int (*apply)(void);
};

#define MESHTASTIC_SETTINGS_APPLY_DEFINE(_name, _apply)                                            \
	static const STRUCT_SECTION_ITERABLE(meshtastic_settings_apply,                            \
					     _meshtastic_settings_apply_##_name) = {               \
		.name = STRINGIFY(_name), .apply = (_apply),                                       \
	}

int meshtastic_settings_apply_all(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_CORE_H_ */
