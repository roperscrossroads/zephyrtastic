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

/*
 * Which bearer carried a received wire frame (agents-xhli.2). The frame bytes
 * are identical on every bearer; the tag exists for the v1 link-local rule:
 * a frame from a non-LoRa bearer is delivered locally but NEVER relayed onto
 * LoRa (bridging bearers creates a multi-bearer flooding graph this design
 * refuses to enter -- PEER-TRANSPORT-DESIGN.md par.2). Route learning, the
 * MQTT uplink, airtime accounting and RF signal bookkeeping are LoRa-only for
 * the same reason: they describe RF topology and RF traffic.
 */
enum meshtastic_bearer {
	MESHTASTIC_BEARER_LORA = 0,
	MESHTASTIC_BEARER_BLE_PEER,
};

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
	/* config.lora.sx126x_rx_boosted_gain: staged into the radio driver at boot
	 * (meshtastic_radio_init) and again on every retune (meshtastic_radio_retune),
	 * because the driver only stages it — it reaches the chip on the next
	 * lora_config(). The retune site was missing until 2026-09-04; see the note
	 * in meshtastic_radio.c for how the stale "needs a reboot anyway" comment
	 * outlived its premise and became the bug (agents-znit). */
	bool rx_boosted_gain;
	/* config.lora.tx_enabled: false makes the node receive-only. Enforced at
	 * the single TX choke point in meshtastic_radio.c. Defaults true. */
	bool tx_enabled;
	/* owner.is_licensed, cached from the config store.
	 *
	 * Cached rather than read on demand because the transmit path needs it on
	 * every frame, and meshtastic_config_store_get_owner_flags() takes the
	 * store mutex — calling that while already holding mt_radio_sem and
	 * mt.lock would introduce a new lock ordering for no benefit. The value
	 * only changes on a config write, which is exactly when this is refreshed.
	 *
	 * It gates two things, both mirroring the reference: the region power
	 * clamp, and whether the board FEM's gain is subtracted at all
	 * (RadioInterface::limitPower). */
	bool licensed;
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
	/* C3 Phase 6: the outgoing MeshPacket the mesh-native wire build works from, kept
	 * off the (right-sized) app-thread send stacks. Guarded by @ref lock alongside the
	 * pb_buf/enc_buf scratch the build already serialises on. */
	meshtastic_MeshPacket tx_mesh;
};

extern struct meshtastic_context mt;
extern struct meshtastic_workspace mt_ws;

/* Packet-id low-bit counter width, mirroring upstream Router.cpp
 * generatePacketId()'s ID_COUNTER_MASK (10-bit rolling counter, 22 random high
 * bits refreshed per call). Exposed so the bit-width itself -- not just "looks
 * random" -- can be pinned against upstream in a test. */
#define MESHTASTIC_PKT_ID_COUNTER_BITS 10U

uint32_t meshtastic_allocate_packet_id(void);
uint32_t meshtastic_next_fromradio_id(void);
/* AES-CTR channel nonce byte-packing -- pure, no crypto. Exposed for the
 * upstream-layout pin test (test_channel_nonce_matches_reference_layout). */
void meshtastic_channel_nonce_build(uint8_t out[16], uint32_t id, uint32_t from);
void meshtastic_emit_event(enum meshtastic_event_type type, int err,
			   const struct meshtastic_packet *packet);
/*
 * Originate a phone/PKC-decoded MeshPacket directly (C3 Phase 6c): the decoded Data is the
 * authoritative outgoing payload, so every field the flat struct never models -- Data.emoji
 * and any field upstream adds next -- survives to the wire by construction. Shares the
 * mesh-native send engine with meshtastic_send_packet() (the struct originator entry).
 */
int meshtastic_send_mesh_decoded(const meshtastic_MeshPacket *mesh, k_timeout_t wait);
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

/*
 * Hand one received wire frame from a non-LoRa bearer to the RX pipeline
 * (agents-xhli.2). Posts into the same queue the LoRa driver callback feeds,
 * so the frame is processed on the mesh RX thread with the full router path —
 * dedup, decode, local delivery — under the bearer's link-local gates. Safe
 * from any thread context (non-blocking put; the frame is copied). Returns 0,
 * -EINVAL on a bad length, -ENOBUFS when the RX queue is full (dropped and
 * counted, same as an RF frame in that state).
 */
int meshtastic_radio_rx_inject(const uint8_t *buf, uint16_t len, enum meshtastic_bearer bearer);

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

/*
 * CAD (listen-before-talk) and periodic AGC-reset diagnostic counters,
 * surfaced on `meshtastic sched stats`. Chip-agnostic wrapper over the
 * sx126x driver's own counters (see sx126x.c) -- callers never need their
 * own CONFIG_LORA_SX126X guard; on a build without that radio these read as
 * a flat 0 rather than needing to be skipped.
 */
#if defined(CONFIG_LORA_SX126X)
uint32_t meshtastic_radio_cad_clear_count(void);
uint32_t meshtastic_radio_cad_busy_count(void);
uint32_t meshtastic_radio_cad_timeout_count(void);
uint32_t meshtastic_radio_cad_error_count(void);
uint32_t meshtastic_radio_agc_reset_ok_count(void);
uint32_t meshtastic_radio_agc_reset_fail_count(void);
uint32_t meshtastic_radio_agc_reset_skipped_count(void);
uint32_t meshtastic_radio_agc_patch_fail_count(void);
void meshtastic_radio_cad_agc_stats_reset(void);
#else
static inline uint32_t meshtastic_radio_cad_clear_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_cad_busy_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_cad_timeout_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_cad_error_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_agc_reset_ok_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_agc_reset_fail_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_agc_reset_skipped_count(void)
{
	return 0U;
}
static inline uint32_t meshtastic_radio_agc_patch_fail_count(void)
{
	return 0U;
}
static inline void meshtastic_radio_cad_agc_stats_reset(void)
{
}
#endif

/**
 * @brief Is a packet being demodulated right now?
 *
 * Parity: upstream Meshtastic's RadioLibInterface::isActivelyReceiving() /
 * receiveDetected(). The radio driver reports the chip's latched
 * preamble-detected and header-valid flags; this layer applies the timing
 * rules that separate a real reception from a stale flag: a preamble that has
 * not become a header within twice the preamble time was noise, and a header
 * that has not become a packet within the airtime of a maximum-length frame
 * was a false header. Either is retired (the flag is cleared) rather than left
 * to oscillate.
 *
 * Three decisions gate on this, exactly as upstream: a transmit must not key up
 * over a frame that is arriving ("not only would we drop the packet that was
 * on the way in, we almost certainly guarantee no one outside will like the
 * packet we are sending"), the periodic AGC reset must not tear RX down
 * mid-packet, and the noise floor must not be sampled while a signal is
 * present.
 *
 * Call with @c mt_radio_sem held: the detection timer it keeps is shared by the
 * transmit path and the AGC reset, and the semaphore is what serialises those.
 *
 * @return true while a reception is believed to be in progress.
 */
bool meshtastic_radio_actively_receiving(void);

/** @brief Counters behind meshtastic_radio_actively_receiving(), for `meshtastic rf`. */
struct meshtastic_radio_rx_activity_stats {
	uint32_t busy_rx;        /**< answers of "yes, a packet is arriving" */
	uint32_t false_preamble; /**< preamble flags retired by the 2x-preamble rule */
	uint32_t false_header;   /**< header flags retired by the max-packet rule */
	uint32_t agc_deferred;   /**< AGC resets postponed because a packet was arriving */
	uint32_t agc_radio_busy; /**< AGC resets postponed because the radio was held (a TX in flight) */
	uint32_t preamble_ms;    /**< the current modem's preamble time */
	uint32_t max_packet_ms;  /**< airtime of a maximum-length frame on it */
};

void meshtastic_radio_rx_activity_stats_get(struct meshtastic_radio_rx_activity_stats *out);
void meshtastic_radio_rx_activity_stats_reset(void);

/**
 * @brief Counters for the transmit defer path (MESHTASTIC_TX_DEFER), for `meshtastic rf`.
 *
 * A defer is a frame the radio refused to key up "right now" and the outbound
 * queue re-scheduled behind a contention delay; a drop is a frame that used up
 * CONFIG_MESHTASTIC_TX_DEFER_MAX of them on a channel that never went quiet.
 */
struct meshtastic_radio_tx_defer_stats {
	uint32_t busy_rx;  /**< refused because a packet was being demodulated */
	uint32_t cad_busy; /**< refused because CAD heard activity */
	uint32_t requeued; /**< frames put back in the queue behind a fresh delay */
	uint32_t dropped;  /**< frames abandoned: dropped_cap + dropped_qfull */
	uint32_t dropped_cap;   /**< abandoned at the defer cap: the channel never went quiet */
	uint32_t dropped_qfull; /**< abandoned because the outbound queue was full on re-insert */
};

void meshtastic_radio_tx_defer_stats_get(struct meshtastic_radio_tx_defer_stats *out);
void meshtastic_radio_tx_defer_stats_reset(void);

/** @brief Outbound drain -> radio: a deferred frame went back in the queue. Counter only. */
void meshtastic_radio_tx_defer_requeued(void);

/**
 * @brief Outbound drain -> radio: a frame was dropped at the defer cap.
 *
 * Accounts exactly as any other failed transmit did before the defer path
 * existed: tx_failures, and MESHTASTIC_EVENT_TX_FAILED with -EBUSY.
 */
void meshtastic_radio_tx_dropped_busy(void);
/** @brief A deferred frame was abandoned because the outbound queue was full.
 *  Distinct from the defer cap above -- see meshtastic_radio.c (agents-q0c6). */
void meshtastic_radio_tx_dropped_queue_full(void);

/**
 * @brief The chip's raw activity flags right now, untimed. For the report only.
 *
 * @return 0 with both flags filled, -ENOTSUP when the driver cannot say.
 */
int meshtastic_radio_rx_activity_now(bool *preamble, bool *header);

/**
 * @brief Three-state answer for a setting the radio may not be able to report.
 *
 * UNKNOWN is a first-class value, not a placeholder. A driver that cannot tell
 * us what the chip is doing must say so — reporting UNKNOWN as OFF would turn
 * "we cannot see" into "it is disabled", which is the exact confusion a
 * diagnostic exists to remove.
 */
enum meshtastic_radio_tristate {
	MESHTASTIC_RADIO_TRI_UNKNOWN = 0,
	MESHTASTIC_RADIO_TRI_OFF,
	MESHTASTIC_RADIO_TRI_ON,
};

/**
 * @brief What was last actually handed to the radio driver, and whether it took it.
 *
 * The point of this struct is that it is written at the lora_config() call
 * sites, so it records what the radio was really told rather than what the
 * configuration believes. The two diverge in ways that are otherwise invisible:
 * a failed config leaves the chip on its previous settings while every stored
 * value still reads new.
 */
struct meshtastic_radio_effective {
	uint32_t frequency;   /**< Hz, as programmed. */
	uint32_t bandwidth_hz;
	uint8_t spread_factor;
	uint8_t coding_rate;
	/*
	 * Two transmit-power fields, not one, because the drive level is
	 * programmed per transmission and reverted immediately afterwards. A
	 * single "current tx power" would report whichever config ran last —
	 * in practice the RX-side one — and be quietly wrong about what the
	 * node actually transmits at.
	 */
	int8_t tx_power_rx_cfg; /**< drive programmed by a receive-side config */
	int8_t tx_power_tx_cfg; /**< drive programmed for the last transmit */
	int last_rc;            /**< return code of the most recent lora_config() */
	uint32_t generation;    /**< increments per SUCCESSFUL config; 0 = never configured */
};

/**
 * @brief Read back the last configuration actually pushed to the radio.
 *
 * @param out Filled on success.
 * @return 0, or -EINVAL for a NULL @p out.
 */
int meshtastic_radio_effective_get(struct meshtastic_radio_effective *out);

/**
 * @brief RX gain boost as STAGED by the driver, and as APPLIED to the chip.
 *
 * These differ, and the gap is a real failure mode rather than a theoretical
 * one: sx126x_set_rx_boosted_gain() only records the request, which reaches the
 * silicon on the next lora_config(). If that config failed, the chip is still
 * on the old gain while the stored configuration claims the new one — a silent
 * loss of 2-3 dB of receive sensitivity. Reporting them separately is what makes
 * that visible.
 *
 * Both read UNKNOWN on a radio whose driver cannot report it — which includes
 * every non-SX126x build, hence the fallbacks below.
 */
#if defined(CONFIG_LORA_SX126X)
enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_staged(void);
enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_applied(void);

/** @brief Consecutive SPI BUSY-line timeouts; a wiring/driver-health signal. */
uint32_t meshtastic_radio_busy_timeout_streak(void);
#else
static inline enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_staged(void)
{
	return MESHTASTIC_RADIO_TRI_UNKNOWN;
}
static inline enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_applied(void)
{
	return MESHTASTIC_RADIO_TRI_UNKNOWN;
}
static inline uint32_t meshtastic_radio_busy_timeout_streak(void)
{
	return 0U;
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
