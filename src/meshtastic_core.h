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

/* The phone-protocol firmware version this port advertises — parses as 2.7.4 so
 * the app stays above its minimum-version gate. Single source of truth for the
 * DeviceMetadata field AND the MQTT map-report. Distinct from meshtastic_build_id()
 * (the git-describe build tag). */
#define MESHTASTIC_FIRMWARE_VERSION "2.7.4.zephyr"

struct meshtastic_dup_entry {
	uint32_t src;
	uint32_t id;
	uint32_t ms;        /* k_uptime_get_32() when recorded, for TTL expiry */
	uint32_t relayed_ms; /* when WE relayed it; valid only if relayed */
	uint8_t hop_limit;  /* highest hop budget seen for this (src,id) */
	bool relayed;       /* we transmitted a relay of this (src,id) */
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

/*
 * Which phone transport to bring up at boot. In a unified image (both
 * CONFIG_MESHTASTIC_BLE and CONFIG_MESHTASTIC_TCP compiled) this honors the
 * persisted network.wifi_enabled (default false = BLE, matching upstream). In a
 * single-transport image the compiled-in transport always wins, ignoring the
 * flag. Read at init and by the WiFi auto-connect thread so both agree; switching
 * transport is a config write + reboot (the admin path already reboots on a
 * network-config change).
 */
bool meshtastic_transport_prefer_wifi(void);

int meshtastic_radio_init(void);

/* Disarm the SX1262 DIO1 EXT1 light-sleep/deep-sleep wake before sys_poweroff(), so an
 * incoming frame does not wake an admin-shut-down node (it wakes only on reset, as
 * canShutdown advertises). The DIO1 wake is armed via GPIO_INT_WAKEUP on the board's
 * dio1-gpios cell, which the ESP32-S3 honours whenever CONFIG_PM || CONFIG_POWEROFF.
 * No-op without an SX126x radio (see meshtastic_radio.c). Call from the shutdown path. */
#if defined(CONFIG_LORA_SX126X)
void meshtastic_radio_disarm_dio1_wake(void);
#else
static inline void meshtastic_radio_disarm_dio1_wake(void)
{
}
#endif

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

/* Re-read the persisted PowerConfig and apply it to the PM subsystem
 * (is_power_saving -> light-sleep pm_policy lock). Called from the settings-apply
 * hook at boot and from the admin config-write path for a live phone toggle.
 * A no-op without CONFIG_PM (there is no light sleep to gate; see
 * meshtastic_power.c). */
#if defined(CONFIG_PM)
void meshtastic_power_config_apply(void);

/* Light-sleep governor activity signals (see meshtastic_power.c). Called directly
 * from the RX, BLE, TCP and input contexts (ISR-safe). The phone notes ref-count
 * the STANDBY inhibitor across BLE + TCP clients; the activity note refreshes the
 * min_wake_secs wake window. No-ops without CONFIG_PM, so every hook site (and a
 * disabled CONFIG_MESHTASTIC_BLE/_TCP) compiles away. */
void meshtastic_power_note_phone_connected(void);
void meshtastic_power_note_phone_disconnected(void);
void meshtastic_power_note_activity(void);

/* Network-link (WiFi) up/down signal. While the link is up the governor holds the
 * STANDBY inhibitor so the SoC does not light-sleep out from under an associated
 * WiFi station: the Zephyr esp32 WiFi path has no DTIM/beacon-wakeup coordination,
 * so a CPU-domain-down light sleep drops the association (the AP deauths on the
 * missed keepalives). This reproduces upstream's !isWifiAvailable() light-sleep
 * gate. Driven from the IPv4 addr add/del net_mgmt events inside meshtastic_power.c
 * (an IPv4 lease is the proxy for a usable link); idempotent, so duplicate events
 * are safe. No-ops without CONFIG_PM. */
void meshtastic_power_note_wifi_up(void);
void meshtastic_power_note_wifi_down(void);

/* Bench diagnostic accessors for the light-sleep inhibitor mask (see
 * meshtastic_power.c). meshtastic_power_inhibitors() returns the live mask; STANDBY
 * is blocked while it is nonzero, so 0 means the node is free to light-sleep. The
 * _str() helper decodes the set bits into a space-separated name list. Used by the
 * "meshtastic power" shell command to report why the SoC is (not) sleeping. */
uint32_t meshtastic_power_inhibitors(void);
void meshtastic_power_inhibitors_str(uint32_t mask, char *buf, size_t n);

/* Volatile bench/debug override for the light-sleep governor. hold==true pins the
 * node awake (a dedicated inhibitor), independent of every other inhibitor and of
 * the persisted PowerConfig; hold==false releases it. Not persisted (cleared on
 * reboot) and not a config write, so it neither fights a phone nor is refused on a
 * managed node — an observability lever, driven by the "meshtastic pm [on|off]"
 * shell command to rule light sleep in or out during a bench session. No-op without
 * CONFIG_PM. */
void meshtastic_power_set_manual_inhibit(bool hold);
bool meshtastic_power_manual_inhibit(void);
#else
static inline void meshtastic_power_config_apply(void)
{
}
static inline void meshtastic_power_note_phone_connected(void)
{
}
static inline void meshtastic_power_note_phone_disconnected(void)
{
}
static inline void meshtastic_power_note_activity(void)
{
}
static inline void meshtastic_power_note_wifi_up(void)
{
}
static inline void meshtastic_power_note_wifi_down(void)
{
}
static inline uint32_t meshtastic_power_inhibitors(void)
{
	return 0U;
}
static inline void meshtastic_power_inhibitors_str(uint32_t mask, char *buf, size_t n)
{
	(void)mask;
	if (n > 0U) {
		buf[0] = '\0';
	}
}
static inline void meshtastic_power_set_manual_inhibit(bool hold)
{
	(void)hold;
}
static inline bool meshtastic_power_manual_inhibit(void)
{
	return false;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_CORE_H_ */
