/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic mesh radio stack public API and runtime state.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <pb_encode.h>
#include <psa/crypto.h>

#include <zephyr/meshtastic/gnss.h>
#include <zephyr/meshtastic/nodedb.h>
#include <zephyr/meshtastic/nodeinfo.h>
#include <zephyr/meshtastic/telemetry.h>

#include "meshtastic_lockdown.h"
#include "meshtastic_build.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_contention.h"
#include "meshtastic_duty.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_tx_power.h"
#include "meshtastic_core.h"
#if defined(CONFIG_MESHTASTIC_WORKER)
#include "meshtastic_worker.h"
#endif
#if defined(CONFIG_MESHTASTIC_HUB)
#include "meshtastic_hub.h"
#endif
#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_scanner.h"
#endif
#include "meshtastic_modules.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"
#include "meshtastic_sched.h"
#include "meshtastic_health.h"
#include "meshtastic_heap_trace.h"
#include "meshtastic_watchdog.h"

#include "meshtastic_settings.h"
#include "meshtastic_pki.h"
#if defined(CONFIG_MESHTASTIC_NODE_ID_FROM_KEY)
#include "meshtastic_node_identity.h"
#endif
#include "meshtastic_gnss.h"
#include "meshtastic_mqtt.h"
#if defined(CONFIG_MESHTASTIC_CLUSTER)
#include "meshtastic_cluster.h"
#endif
#if defined(CONFIG_MESHTASTIC_AIRTIME)
#include "meshtastic_airtime.h"
#endif

#if defined(CONFIG_MESHTASTIC_BLE)
int meshtastic_ble_init(void);
int meshtastic_ble_radio_enable(void);
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
#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
int meshtastic_statusmessage_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
int meshtastic_neighborinfo_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_MESHBEACON)
int meshtastic_meshbeacon_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_TRAFFIC)
int meshtastic_traffic_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_EXTNOTIFY)
int meshtastic_extnotify_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_LED_STATUS)
int meshtastic_led_status_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
int meshtastic_keyverify_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_MESSAGE)
int meshtastic_message_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB)
int meshtastic_nodedb_init(void);
#if defined(CONFIG_MESHTASTIC_BULK_STORE)
#include "meshtastic_bulk.h"
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS) || defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
#define NODEDB_IN_BULK 1
int meshtastic_nodedb_bulk_reload(void);
#endif
#endif
#endif
#if defined(CONFIG_MESHTASTIC_NODEINFO)
int meshtastic_nodeinfo_init(void);
#endif
#if defined(CONFIG_MESHTASTIC_DISPLAY)
int meshtastic_display_init(void);
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
/* The reference's reloadFromDisk() after a deferred unlock, on the system
 * workqueue: every wrapped subtree re-read now that the DEK is in RAM, the
 * core config re-applied, the keypair re-read (a locked boot generated a
 * throwaway one over the defaults and could not persist it), and the radio
 * released. Modules read their sections from the store live, so they follow. */
static void lockdown_reload(void)
{
	int ret;

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	ret = settings_load_subtree("meshtastic");
	if (ret < 0) {
		LOG_WRN("Lockdown reload: settings load failed (%d)", ret);
	}
#endif
	ret = meshtastic_config_store_apply_core();
	if (ret < 0) {
		LOG_WRN("Lockdown reload: apply_core failed (%d)", ret);
	}
	(void)meshtastic_settings_apply_all();
#if defined(CONFIG_MESHTASTIC_PKI)
	(void)meshtastic_pki_init();
#endif
#if defined(CONFIG_MESHTASTIC_NODE_ID_FROM_KEY)
	/* The key is readable for the first time this boot. Record the id it implies; it is
	 * not switched at runtime, because the radio, NodeDB and cluster took the id at init. */
	meshtastic_node_identity_key_reloaded(mt.node_id);
#endif
#if defined(NODEDB_IN_BULK)
	/* The NodeDB tables in the bulk store restore from their pages; the settings
	 * subtrees are only read when the store is not mounted. */
	if (meshtastic_nodedb_bulk_reload() == -ENODEV) {
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
		(void)settings_load_subtree("mtnode");
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
		(void)settings_load_subtree("mtrec");
#endif
	}
#else
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	(void)settings_load_subtree("mtnode");
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	(void)settings_load_subtree("mtrec");
#endif
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
	(void)settings_load_subtree("mtclus/scope");
	(void)settings_load_subtree("mtclus");
#endif
	meshtastic_radio_release();
}
#endif

int meshtastic_init(const struct meshtastic_config *cfg)
{
	psa_status_t psa_st;
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	bool rf_entropy_up = false;
	bool rf_in_build = false; /* BT or WiFi is compiled in, so there is something to check */
#endif
	int ret;

	/* Banner: the build id (git describe + "-dirty") + build time, so a boot-log
	 * capture identifies the running image — separate from the phone-protocol
	 * firmware_version. */
	LOG_INF("meshtastic build %s (built %s UTC)", meshtastic_build_id(),
		meshtastic_build_time());

	/* Arm heap tracing before anything else allocates, so boot-time draws
	 * (WiFi HAL init, our dynamic TCP thread stack) are captured too, not
	 * just steady-state churn. No-op unless CONFIG_MESHTASTIC_HEAP_TRACE. */
	meshtastic_heap_trace_init();

	/* Arm early: a hang during the rest of init is exactly the kind of thing
	 * this exists to catch. meshtastic_watchdog_checkin() lands well before
	 * the timeout as soon as the radio starts its normal power-state cycling
	 * below. */
	meshtastic_watchdog_init();

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
	ret = meshtastic_node_id_from_hwinfo(&mt.node_id);
	if (ret < 0) {
		return ret;
	}

	if (cfg->node_id != 0U && cfg->node_id != mt.node_id) {
		LOG_WRN("Ignoring config node_id 0x%08x (using derived 0x%08x)", cfg->node_id,
			mt.node_id);
	}
#endif
#if defined(CONFIG_MESHTASTIC_WORKER)
	/* A keyless worker has no channel table: no PSK enters RAM from the app's
	 * config or from flash, and with every slot unnamed the frequency slot is
	 * the active preset's public one -- a worker sits where a stock node on that
	 * preset sits (ATTACHMENT-SCOPE B6, §7). */
	ARG_UNUSED(cfg->psk);
#else
	ret = meshtastic_channels_init_from_config(cfg);
	if (ret < 0) {
		return ret;
	}
#endif

	mt.frequency = cfg->frequency;

	/* Default the modem to LONG_FAST. Stored config replaces this during
	 * settings load; without it mt.modem would be zeroed and every
	 * lora_config() would fall back with a warning. The preset is kept too,
	 * because channel-name resolution needs it before any config is loaded.
	 */
	mt.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	mt.use_preset = true;
	(void)meshtastic_preset_to_params(mt.modem_preset, false, &mt.modem);

	mt.hop_limit = (cfg->hop_limit == 0U) ? (uint8_t)CONFIG_MESHTASTIC_DEFAULT_HOP_LIMIT
					      : cfg->hop_limit;
	/* Radiated dBm, reference semantics: 0 means "as much as the region
	 * allows". The region is not known until the config store loads, so this
	 * seeds from the build default and meshtastic_config_store_apply() redoes
	 * it against the stored region. */
	/* Transmitting is the default; only an explicit stored tx_enabled=false
	 * turns it off (config_store applies that). Seeded here so a node with no
	 * stored lora config is not silently mute. */
	mt.tx_enabled = true;
	/* Licensed operation is OPT-IN and must never be inherited by accident: it
	 * lifts the regional power clamp and skips the front-end backoff, so a
	 * spurious true transmits over the legal limit. Static storage already
	 * zeroes this; stating it here makes the default a decision rather than a
	 * property of where the struct happens to live, and it is only ever raised
	 * from a stored owner record that explicitly carries the flag. */
	mt.licensed = false;
	mt.tx_power = meshtastic_tx_power_resolve(
		cfg->tx_power, meshtastic_Config_LoRaConfig_RegionCode_UNSET, false);
	mt.long_name = (cfg->long_name != NULL) ? cfg->long_name : CONFIG_MESHTASTIC_NODE_LONG_NAME;
	mt.short_name =
		(cfg->short_name != NULL) ? cfg->short_name : CONFIG_MESHTASTIC_NODE_SHORT_NAME;
	/* Seeded from the CSPRNG once PSA is up (below) — a fixed start would make
	 * the AES-CTR nonce repeat after every reboot. */
	atomic_set(&mt.next_pkt_id, 1);
	atomic_set(&mt.next_fromradio_id, 1);
	mt.dup_head = 0U;
	mt.radio_rx_armed = false;
	memset(mt.dup_cache, 0, sizeof(mt.dup_cache));
	memset(&mt.status, 0, sizeof(mt.status));
	mt.status.node_id = mt.node_id;

#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	/* Warm up the BT controller BEFORE any entropy gets drawn below (agents-0lzm.10):
	 * Espressif's own esp_random.h says WDEV_RND "returns true random numbers" only
	 * "if Wi-Fi or Bluetooth are enabled" -- and psa_crypto_init() right after this
	 * seeds its DRBG from exactly that register, with meshtastic_pki_init() minting
	 * (and persisting, for life) a fresh X25519 identity from it on a node's first
	 * boot. The rest of meshtastic_ble_init() (advertising, PhoneAPI, GATT) still
	 * runs at its usual, much later spot -- this call is deliberately narrowed to
	 * just bt_enable(), and it is idempotent (bt_is_ready() guarded) so that later
	 * call is a no-op. ESP32-only: the nRF52840 XIAOs have a true hardware RNG,
	 * independent of any radio, and do not need this. */
	rf_in_build = true;
	if (meshtastic_ble_radio_enable() == 0) {
		LOG_INF("BT radio warmed up ahead of PKI init (agents-0lzm.10)");
		rf_entropy_up = true;
	} else {
		LOG_WRN("BT radio warm-up failed ahead of PKI init");
	}
#endif
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32) && defined(CONFIG_WIFI) && \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(wifi))
	/* A WiFi build needs no warm-up of ours: the Espressif driver starts the radio
	 * in its own device init ("Start Wi-Fi early", esp32_wifi_dev_init, POST_KERNEL),
	 * long before this. What can be asked is whether it did: a device that is not
	 * ready did not start the radio, and the RNG is then not fed. */
	rf_in_build = true;
	if (device_is_ready(DEVICE_DT_GET(DT_NODELABEL(wifi)))) {
		rf_entropy_up = true;
	} else {
		LOG_WRN("WiFi device not ready ahead of PKI init");
	}
#endif
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32) && defined(CONFIG_MESHTASTIC_PKI)
	/* A radio is in the build but did not come up: do not mint an identity from that (a stored
	 * key still loads). Other families have an RNG that needs no radio. */
	if (rf_in_build) {
		meshtastic_pki_set_entropy_ready(rf_entropy_up);
	} else {
		LOG_WRN("No RF in this ESP32 build: the identity key comes from an unfed RNG");
	}
#endif
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	ARG_UNUSED(rf_entropy_up);
	ARG_UNUSED(rf_in_build);
#endif

	psa_st = psa_crypto_init();
	if (psa_st != PSA_SUCCESS) {
		LOG_ERR("psa_crypto_init failed (%d)", (int)psa_st);
		return -EIO;
	}

	/* Must follow psa_crypto_init(): the seed comes from the PSA DRBG. */
	ret = meshtastic_seed_packet_id();
	if (ret < 0) {
		return ret;
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

#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
	/* Before any subtree loads: the artifacts decide whether the loads that
	 * follow read the real records or keep their defaults (a locked boot).
	 * settings_subsys_init() is idempotent; meshtastic_settings_init() calls it
	 * again harmlessly. */
	(void)settings_subsys_init();
	meshtastic_lockdown_init();
	meshtastic_lockdown_set_reload_hook(lockdown_reload);
	meshtastic_phoneapi_lockdown_init();
	mt.radio_held = meshtastic_lockdown_locked();
	if (mt.radio_held) {
		LOG_WRN("Lockdown: locked boot -- defaults loaded, radio held until unlock");
	}
#endif

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

#if defined(CONFIG_MESHTASTIC_PKI)
	/* Load-or-generate our X25519 keypair now that settings are applied (so the
	 * persisted key is visible). Non-fatal: PSK channels still work if this
	 * fails, so don't abort init. */
	ret = meshtastic_pki_init();
	if (ret < 0) {
		LOG_WRN("PKI init failed (%d); continuing with PSK channels only", ret);
	}
#endif

#if defined(CONFIG_MESHTASTIC_NODE_ID_FROM_KEY)
	/* The id so far is PROVISIONAL (the source choice above), because the key it should
	 * be derived from only exists from here on. Adopt the key-derived id now, before the
	 * radio, NodeDB, cluster and BLE capture it -- everything after this line sees the
	 * final id. What ran before it only took the id cosmetically: a never-configured
	 * owner's default short name, and the HLC tie-break author of a first-boot security
	 * write. The reference has the same shape: a MAC-derived provisional number, then
	 * createNewIdentity() once the key exists. */
	mt.node_id = meshtastic_node_identity_adopt(mt.node_id);
	mt.status.node_id = mt.node_id;
#endif

	ret = meshtastic_radio_init();
	if (ret < 0) {
		mt.initialized = false;
		mt.status.initialized = false;
		return ret;
	}
#if defined(CONFIG_MESHTASTIC_WORKER)
	meshtastic_worker_start();
#endif
#if defined(CONFIG_MESHTASTIC_HUB)
	meshtastic_hub_start();
#endif

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

#if defined(CONFIG_MESHTASTIC_BULK_STORE) && defined(CONFIG_MESHTASTIC_NODEDB)
	/* Before the NodeDB, which restores its records from it. A store that will
	 * not mount is not fatal: the NodeDB falls back to its settings records.
	 * (Its header is included under NODEDB above: without a NodeDB there is
	 * nothing to mount it for -- the keyless worker is the first such image.) */
	ret = meshtastic_bulk_init();
	if (ret < 0) {
		LOG_WRN("Bulk store unavailable (%d); node records stay in settings", ret);
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

#if defined(CONFIG_MESHTASTIC_STATUSMESSAGE)
	/* After the config store is seeded/loaded (above): init reads the
	 * persisted status to arm its first announce. */
	ret = meshtastic_statusmessage_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_NEIGHBORINFO)
	ret = meshtastic_neighborinfo_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_MESHBEACON)
	ret = meshtastic_meshbeacon_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_TRAFFIC)
	ret = meshtastic_traffic_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_EXTNOTIFY)
	ret = meshtastic_extnotify_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_LED_STATUS)
	ret = meshtastic_led_status_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_KEYVERIFY)
	ret = meshtastic_keyverify_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_CLUSTER)
	/* After the settings init: the channel table and persisted doc entries
	 * must both be loaded before the first digest describes them. */
	ret = meshtastic_cluster_init();
	if (ret < 0) {
		return ret;
	}
#endif

	/*
	 * Phone transport selection. In a unified image (BLE + WiFi/net both
	 * compiled) exactly one is brought up per boot, chosen by the persisted
	 * network.wifi_enabled; switching is a config write + reboot. Bringing up
	 * only one means only that stack's radio + runtime heap goes live, so the
	 * two never contend for the ESP32 radio (no BT/WiFi coexistence needed). In
	 * a single-transport image the sole transport always wins.
	 */
#if defined(CONFIG_MESHTASTIC_BLE) || defined(CONFIG_MESHTASTIC_TCP) ||                             \
	defined(CONFIG_MESHTASTIC_MQTT)
	const bool use_wifi = meshtastic_transport_prefer_wifi();

	LOG_INF("phone transport: %s", use_wifi ? "WiFi/net" : "BLE");
#endif

#if defined(CONFIG_MESHTASTIC_BLE)
	if (!use_wifi) {
		ret = meshtastic_ble_init();
		if (ret < 0) {
			/* Non-fatal, symmetric to TCP: keep the node on the mesh rather
			 * than failing boot — e.g. if the (optionally dynamic) work-thread
			 * stack can't allocate, or bt_enable fails. */
			LOG_WRN("BLE PhoneAPI init failed (%d); continuing without it", ret);
		}
	}
#endif

#if defined(CONFIG_MESHTASTIC_SERIAL)
	ret = meshtastic_serial_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_TCP)
	if (use_wifi) {
		ret = meshtastic_tcp_init();
		if (ret < 0) {
			/* Non-fatal: keep the node on the mesh (and BLE, if also active)
			 * rather than failing boot. The notable case is the dynamic
			 * serve-thread stack failing to allocate under heap pressure — a
			 * lost WiFi PhoneAPI beats a node that won't come up. */
			LOG_WRN("TCP PhoneAPI init failed (%d); continuing without it", ret);
		}
	}
#endif

#if defined(CONFIG_MESHTASTIC_MQTT)
	/* Always: a proxying bridge needs no network, and init leaves a broker bridge
	 * idle on a boot without one. */
	ret = meshtastic_mqtt_init(use_wifi);
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_DISPLAY)
	/* Last: the screen renders from the subsystems brought up above. A
	 * missing/failed panel is non-fatal — the node still runs headless. */
	ret = meshtastic_display_init();
	if (ret < 0) {
		return ret;
	}
#endif

	LOG_INF("Meshtastic init: node=0x%08x ch_hash=0x%02x freq=%uHz", mt.node_id, mt.ch_hash,
		mt.frequency);

#if defined(CONFIG_MESHTASTIC_SCANNER)
	/* LAST, and deliberately after the radio is up: a scanner image parks the
	 * SX1262 on a survey frequency and closes the TX gate, so it must not run
	 * until the stack it is displacing has finished claiming the radio. Doing
	 * this from the scanner's own SYS_INIT (before main()) had the two fighting
	 * for the chip for ~30 s of every boot — agents-0lzm.11. No-op unless the
	 * image was built with MESHTASTIC_SCANNER_AUTOSTART. */
	meshtastic_scanner_autostart();
#endif

	/* After everything above: needs the radio and the channel table both
	 * up (it resolves a channel by name and sends over the same path any
	 * other originated traffic uses). No-op unless this boot has a
	 * pending crash breadcrumb — see meshtastic_health.h. */
	meshtastic_health_init();

	return 0;
}
