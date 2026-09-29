/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The brain's attachment table and ingest (meshtastic_attachment.h;
 * ATTACHMENT-DESIGN S6). A head's RX_FRAME becomes an RF frame on the router's
 * queue with the head's preset and signal attached -- from there dedup, decrypt,
 * NodeDB and the modules see it exactly as they would a frame from this board's
 * own radio, because it is one: it crossed the air, on the head's preset.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>
#include "meshtastic_core.h"
#include "meshtastic_attachment.h"
#include "meshtastic_contention.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_channels.h"
#include "meshtastic_region_presets.h"
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
#include "meshtastic_ble_peer_codec.h" /* the ENV_MAX cross-check only */
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_MESHTASTIC_BLE_PEER)
BUILD_ASSERT(MESHTASTIC_ATTACHMENT_ENV_MAX == MESHTASTIC_BLE_PEER_ENV_MAX,
	     "the envelope must fit the peer link's kind-1 frame");
#endif

#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

/* [0] is the local radio and never admitted or forgotten. */
/* PSRAM on the V4 family (a no-op elsewhere): mutex-guarded, CPU-only, never
 * touched from an ISR or DMA -- the rules in meshtastic_ext_ram.h. The R8
 * brain sits at 99.7 % of internal RAM; this is where a table lives. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_attachment_info tab[CONFIG_MESHTASTIC_ATTACHMENT_MAX + 1U];
static bool used[CONFIG_MESHTASTIC_ATTACHMENT_MAX + 1U] = { true };
static struct meshtastic_attachment_stats stats;
static uint32_t allow[CONFIG_MESHTASTIC_ATTACHMENT_ALLOW_MAX];
static K_MUTEX_DEFINE(tab_lock);

static void evict_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(evict_work, evict_fn);

/* ---- the allow-list (mtattach/allow) --------------------------------------- */

static bool allowed_locked(uint32_t node)
{
	for (unsigned int i = 0U; i < ARRAY_SIZE(allow); i++) {
		if (allow[i] == node) {
			return true;
		}
	}
	return false;
}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
static void allow_save_locked(void)
{
	if (settings_save_one("mtattach/allow", allow, sizeof(allow)) != 0) {
		LOG_WRN("attach: allow-list save failed");
	}
}

static int attach_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint32_t tmp[CONFIG_MESHTASTIC_ATTACHMENT_ALLOW_MAX] = { 0 };

	if (strcmp(key, "allow") != 0) {
		return -ENOENT;
	}
	if (len > sizeof(tmp) || read_cb(cb_arg, tmp, len) != (ssize_t)len) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	memcpy(allow, tmp, sizeof(allow));
	k_mutex_unlock(&tab_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_attach_brain, "mtattach", NULL, attach_settings_set, NULL, NULL);
#else
static void allow_save_locked(void)
{
}
#endif

int meshtastic_attachment_allow_add(uint32_t node)
{
	int ret = -ENOSPC;

	if (node == 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (allowed_locked(node)) {
		ret = 0;
	} else {
		for (unsigned int i = 0U; i < ARRAY_SIZE(allow); i++) {
			if (allow[i] == 0U) {
				allow[i] = node;
				allow_save_locked();
				ret = 0;
				break;
			}
		}
	}
	k_mutex_unlock(&tab_lock);
	return ret;
}

void meshtastic_attachment_allow_clear(void)
{
	k_mutex_lock(&tab_lock, K_FOREVER);
	memset(allow, 0, sizeof(allow));
	allow_save_locked();
	k_mutex_unlock(&tab_lock);
}

bool meshtastic_attachment_allow_get(unsigned int i, uint32_t *node)
{
	bool ok = false;

	if (i >= ARRAY_SIZE(allow) || node == NULL) {
		return false;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (allow[i] != 0U) {
		*node = allow[i];
		ok = true;
	}
	k_mutex_unlock(&tab_lock);
	return ok;
}

bool meshtastic_attachment_is_allowed(uint32_t node)
{
	bool ok;

	k_mutex_lock(&tab_lock, K_FOREVER);
	ok = allowed_locked(node);
	k_mutex_unlock(&tab_lock);
	return ok;
}

void meshtastic_attachment_stats_get(struct meshtastic_attachment_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	*out = stats;
	k_mutex_unlock(&tab_lock);
}

/* Is the link this envelope came over one we may extend RF trust to? The
 * bearer answers; with no bearer (the test seam) the registry is asked. */
static bool link_trusted(const struct meshtastic_attach_bearer *b, uint32_t node)
{
	struct meshtastic_attach_link_info info;
	bool known;

	known = (b != NULL) ? b->link_info(node, &info)
			    : meshtastic_attach_bearer_link_info(node, &info, NULL);
	return known && info.up && info.auth != MESHTASTIC_ATTACH_AUTH_NONE;
}

/* ---- eviction: a dead link's slot is freed after the grace ------------------- */

static void evict_fn(struct k_work *work)
{
	int64_t now = k_uptime_get();

	ARG_UNUSED(work);
	k_mutex_lock(&tab_lock, K_FOREVER);
	for (unsigned int i = 1U; i < ARRAY_SIZE(tab); i++) {
		if (used[i] && !tab[i].link_up && tab[i].down_ms != 0 &&
		    now - tab[i].down_ms >= (int64_t)CONFIG_MESHTASTIC_ATTACHMENT_EVICT_SEC * 1000) {
			LOG_INF("attach: head 0x%08x (attachment %u) evicted after %d s down",
				tab[i].node, i, CONFIG_MESHTASTIC_ATTACHMENT_EVICT_SEC);
			used[i] = false;
			memset(&tab[i], 0, sizeof(tab[i]));
			stats.evicted++;
		}
	}
	k_mutex_unlock(&tab_lock);
	(void)k_work_schedule(&evict_work,
			      K_MSEC(CONFIG_MESHTASTIC_ATTACHMENT_EVICT_SEC * 1000 / 2));
}

void meshtastic_attachment_start(void)
{
	(void)k_work_schedule(&evict_work,
			      K_MSEC(CONFIG_MESHTASTIC_ATTACHMENT_EVICT_SEC * 1000 / 2));
}

static struct meshtastic_attachment_info *find_locked(uint32_t node)
{
	for (unsigned int i = 1U; i < ARRAY_SIZE(tab); i++) {
		if (used[i] && tab[i].node == node) {
			return &tab[i];
		}
	}
	return NULL;
}


/* DESIGN §13: slot time and ROUTER window of a preset, on this board's modem
 * rules (CW 3, the tightest case: (2*CW - 1) slots). 0 when unknown. */
static uint32_t preset_slot_ms(uint8_t preset)
{
	struct meshtastic_modem_params p;

	if (preset > (uint8_t)_meshtastic_Config_LoRaConfig_ModemPreset_MAX ||
	    meshtastic_preset_to_params((meshtastic_Config_LoRaConfig_ModemPreset)preset, false, &p) != 0) {
		return 0U;
	}
	return meshtastic_contention_effective_slot_ms(p.spread_factor, p.bandwidth_hz, false);
}

static uint32_t router_window_ms(uint8_t preset)
{
	return (2U * 3U - 1U) * preset_slot_ms(preset);
}

static bool role_relays_early(void)
{
	meshtastic_Config_DeviceConfig_Role r = meshtastic_device_role();

	return r == meshtastic_Config_DeviceConfig_Role_ROUTER ||
	       r == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE;
}

/* Recompute the ring's p50/p90/max and the LAG guard, tab_lock held. */
static void lag_recompute_locked(struct meshtastic_attachment_info *a)
{
	uint16_t v[ARRAY_SIZE(a->lag)];
	unsigned int n = a->lag_n;

	if (n == 0U) {
		return;
	}
	memcpy(v, a->lag, sizeof(v));
	/* Insertion sort: n <= 16. */
	for (unsigned int i = 1U; i < n; i++) {
		uint16_t x = v[i];
		unsigned int j = i;

		while (j > 0U && v[j - 1U] > x) {
			v[j] = v[j - 1U];
			j--;
		}
		v[j] = x;
	}
	a->lag_p50_ms = v[(n - 1U) / 2U];
	a->lag_p90_ms = v[((n * 9U + 9U) / 10U) - 1U]; /* ceil(0.9 n) - 1: the 2nd largest of 16 */
	a->lag_max_ms = v[n - 1U];
	a->warn_lag = role_relays_early() && a->preset != MESHTASTIC_PRESET_UNKNOWN &&
		      (uint32_t)a->lag_p90_ms > router_window_ms(a->preset) / 2U;
	if (a->warn_lag && !a->warned_lag) {
		a->warned_lag = true;
		LOG_WRN("attach %u: link lag p90 %u ms is over half the preset-%u ROUTER window (%u ms) "
			"-- a ROUTER's relays through this head land late; a fast preset belongs on "
			"the brain's radio (DESIGN §13)",
			a->id, a->lag_p90_ms, a->preset, router_window_ms(a->preset));
	}
}

/* The FAST-HEAD guard, tab_lock held: the head's slot time against our own radio's. */
static void fast_head_check_locked(struct meshtastic_attachment_info *a)
{
	uint32_t ours = meshtastic_contention_effective_slot_ms(mt.modem.spread_factor,
								 mt.modem.bandwidth_hz, false);
	uint32_t theirs = preset_slot_ms(a->preset);

	a->warn_fast_head = (theirs != 0U && ours != 0U && theirs < ours);
	if (a->warn_fast_head && !a->warned_fast_head) {
		a->warned_fast_head = true;
		LOG_WRN("attach %u: head preset %u (slot %u ms) is faster than our own (slot %u ms) -- "
			"the fastest preset an identity serves belongs on the brain's radio (DESIGN §13)",
			a->id, a->preset, theirs, ours);
	}
}

static struct meshtastic_attachment_info *admit_locked(uint32_t node)
{
	for (unsigned int i = 1U; i < ARRAY_SIZE(tab); i++) {
		if (!used[i]) {
			memset(&tab[i], 0, sizeof(tab[i]));
			tab[i].id = (uint8_t)i;
			tab[i].node = node;
			tab[i].preset = MESHTASTIC_PRESET_UNKNOWN;
			tab[i].rssi_min = INT16_MAX;
			tab[i].rssi_max = INT16_MIN;
			used[i] = true;
			LOG_INF("attach: head 0x%08x admitted as attachment %u", node, i);
			return &tab[i];
		}
	}
	return NULL;
}

/* The send seam: whichever registered bearer has a live link to the head. A
 * test overrides this, or registers a bearer of its own. */
__weak int meshtastic_attachment_send(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_attach_bearer_send(node, env, len);
}

int meshtastic_attachment_ingest(uint32_t node, const uint8_t *env, size_t len)
{
	return meshtastic_attachment_ingest_from(NULL, node, env, len);
}

void meshtastic_attachment_link_down(const struct meshtastic_attach_bearer *b, uint32_t node)
{
	struct meshtastic_attachment_info *a;

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a != NULL && (a->bearer == b || a->bearer == NULL)) {
		a->link_up = false;
		a->down_ms = k_uptime_get();
		LOG_INF("attach: head 0x%08x (attachment %u) link down", node, a->id);
	}
	k_mutex_unlock(&tab_lock);
}

int meshtastic_attachment_ingest_from(const struct meshtastic_attach_bearer *b, uint32_t node,
				      const uint8_t *env, size_t len)
{
	struct meshtastic_attachment_msg msg;
	struct meshtastic_attachment_info *a;
	int ret;

	if (node == 0U || env == NULL) {
		return -EINVAL;
	}

	ret = meshtastic_attachment_decode(env, len, &msg);

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a == NULL) {
		/* Admission (ATTACHMENT-SCOPE C1): decode first, trust second. */
		if (ret < 0) {
			stats.not_admitted_malformed++;
			k_mutex_unlock(&tab_lock);
			return -EBADMSG;
		}
		if (!allowed_locked(node) && !link_trusted(b, node)) {
			stats.admission_refused++;
			k_mutex_unlock(&tab_lock);
			LOG_WRN("attach: 0x%08x refused: link not trusted and not allowed", node);
			return -EACCES;
		}
		a = admit_locked(node);
		if (a == NULL) {
			k_mutex_unlock(&tab_lock);
			LOG_WRN("attach: no slot for head 0x%08x", node);
			return -ENOSPC;
		}
	}
	a->bearer = b;
	a->link_up = true;
	a->down_ms = 0;
	a->last_ms = k_uptime_get();
	if (ret < 0) {
		a->rejected++;
		k_mutex_unlock(&tab_lock);
		return -EBADMSG;
	}

	switch (msg.type) {
	case MESHTASTIC_ATTACHMENT_RX_FRAME: {
		const struct meshtastic_rx_meta meta = {
			.bearer = MESHTASTIC_BEARER_ATTACHMENT,
			.attach = a->id,
			.preset = msg.u.rx.preset,
			.rssi = msg.u.rx.rssi,
			.snr = msg.u.rx.snr,
			.rx_ms = msg.u.rx.rx_ms,
		};

		a->preset = msg.u.rx.preset;
		a->last_rssi = msg.u.rx.rssi;
		a->last_snr = msg.u.rx.snr;
		a->rssi_min = MIN(a->rssi_min, msg.u.rx.rssi);
		a->rssi_max = MAX(a->rssi_max, msg.u.rx.rssi);
		ret = meshtastic_radio_rx_inject_meta(msg.u.rx.wire, msg.u.rx.wire_len, &meta);
		if (ret == 0) {
			a->rx_frames++;
		} else {
			a->rx_dropped++;
		}
		break;
	}
	case MESHTASTIC_ATTACHMENT_STATUS:
		a->status = msg.u.status;
		a->have_status = true;
		a->preset = msg.u.status.preset;
		fast_head_check_locked(a);
		ret = 0;
		break;
	case MESHTASTIC_ATTACHMENT_TX_RESULT:
		a->tx_results++;
		a->last_tx_rc = msg.u.result.rc;
		a->last_tx_defers = msg.u.result.defers;
		if (msg.u.result.rc != 0) {
			a->tx_failed++;
		}
		if (msg.u.result.has_lag) {
			a->lag[a->lag_next] = msg.u.result.lag_ms;
			a->lag_next = (uint8_t)((a->lag_next + 1U) % ARRAY_SIZE(a->lag));
			if (a->lag_n < ARRAY_SIZE(a->lag)) {
				a->lag_n++;
			}
			lag_recompute_locked(a);
		}
		ret = 0;
		break;
	default:
		/* TX_FRAME and SET_PRESET are controls a HEAD accepts; a brain
		 * receiving one is talking to something that is not a head. */
		a->rejected++;
		ret = -EBADMSG;
		break;
	}
	k_mutex_unlock(&tab_lock);
	return ret;
}

unsigned int meshtastic_attachment_count(void)
{
	unsigned int n = 0U;

	k_mutex_lock(&tab_lock, K_FOREVER);
	for (unsigned int i = 0U; i < ARRAY_SIZE(tab); i++) {
		n += used[i] ? 1U : 0U;
	}
	k_mutex_unlock(&tab_lock);
	return n;
}

void meshtastic_attachment_note_latency(uint8_t id, uint32_t ms)
{
	struct meshtastic_attachment_info *a;

	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	a = &tab[id];
	if (used[id]) {
		if (a->lat_n == 0U || ms < a->lat_min_ms) {
			a->lat_min_ms = ms;
		}
		if (ms > a->lat_max_ms) {
			a->lat_max_ms = ms;
		}
		a->lat_sum_ms += ms;
		a->lat_n++;
	}
	k_mutex_unlock(&tab_lock);
}

bool meshtastic_attachment_get(uint8_t id, struct meshtastic_attachment_info *out)
{
	bool ok = false;

	if (id >= ARRAY_SIZE(tab) || out == NULL) {
		return false;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (used[id]) {
		*out = tab[id];
		if (id == 0U) {
			out->preset = (uint8_t)mt.modem_preset;
			out->last_rssi = mt.status.last_rssi;
			out->last_snr = mt.status.last_snr;
		}
		ok = true;
	}
	k_mutex_unlock(&tab_lock);
	return ok;
}

uint8_t meshtastic_attachment_id_for_node(uint32_t node)
{
	struct meshtastic_attachment_info *a;
	uint8_t id = 0U;

	k_mutex_lock(&tab_lock, K_FOREVER);
	a = find_locked(node);
	if (a != NULL) {
		id = a->id;
	}
	k_mutex_unlock(&tab_lock);
	return id;
}

int meshtastic_attachment_forget(uint8_t id)
{
	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	used[id] = false;
	memset(&tab[id], 0, sizeof(tab[id]));
	k_mutex_unlock(&tab_lock);
	return 0;
}

bool meshtastic_attachment_tx_ready(uint8_t id)
{
	bool ready = false;

	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return false;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (used[id] && tab[id].link_up) {
		const struct meshtastic_attachment_info *a = &tab[id];

		ready = !a->have_status ||
			((a->status.flags & MESHTASTIC_ATTACHMENT_ST_RX_ONLY) == 0U &&
			 (a->status.flags & MESHTASTIC_ATTACHMENT_ST_TX_ENABLED) != 0U);
	}
	k_mutex_unlock(&tab_lock);
	return ready;
}

uint8_t meshtastic_attachment_tx_hash(uint8_t id, uint8_t index)
{
	uint8_t preset = MESHTASTIC_PRESET_UNKNOWN;

	if (id != 0U && id < ARRAY_SIZE(tab)) {
		k_mutex_lock(&tab_lock, K_FOREVER);
		if (used[id]) {
			preset = tab[id].preset;
		}
		k_mutex_unlock(&tab_lock);
	}
	if (preset == MESHTASTIC_PRESET_UNKNOWN) {
		return meshtastic_channels_get_hash(index);
	}
	return meshtastic_channels_hash_for_preset(index, preset);
}

int meshtastic_attachment_tx(uint8_t id, const uint8_t *wire, size_t len, uint8_t flags)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	struct meshtastic_attachment_tx_frame tx = {
		.flags = flags,
		.wire = wire,
		.wire_len = (uint16_t)len,
	};
	uint32_t node;
	int elen;
	int ret;

	if (wire == NULL || len == 0U || len > MESHTASTIC_ATTACHMENT_WIRE_MAX) {
		return -EINVAL;
	}
	if (!meshtastic_attachment_tx_ready(id)) {
		return -EHOSTUNREACH;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	node = tab[id].node;
	tx.preset = tab[id].preset;
	tx.tx_seq = ++tab[id].tx_seq;
	k_mutex_unlock(&tab_lock);

	elen = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
	if (elen < 0) {
		return elen;
	}
	ret = meshtastic_attachment_send(node, env, (size_t)elen);
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (used[id]) {
		if (ret == 0) {
			tab[id].tx_frames++;
		} else {
			tab[id].tx_failed++;
		}
	}
	k_mutex_unlock(&tab_lock);
	return ret;
}

static int attachment_send_env(uint8_t id, const uint8_t *env, size_t len, bool count_tx)
{
	uint32_t node;
	int ret;

	k_mutex_lock(&tab_lock, K_FOREVER);
	node = tab[id].node;
	k_mutex_unlock(&tab_lock);
	ret = meshtastic_attachment_send(node, env, len);
	if (count_tx) {
		k_mutex_lock(&tab_lock, K_FOREVER);
		if (used[id]) {
			if (ret == 0) {
				tab[id].tx_frames++;
			} else {
				tab[id].tx_failed++;
			}
		}
		k_mutex_unlock(&tab_lock);
	}
	return ret;
}

int meshtastic_attachment_relay(uint8_t id, const uint8_t *wire, size_t len, uint32_t src,
				uint32_t pkt_id, uint32_t rx_ms, uint32_t not_before_ms,
				uint8_t dupe)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	struct meshtastic_attachment_tx_frame tx = {
		.flags = MESHTASTIC_ATTACHMENT_TXF_WANT_RESULT | MESHTASTIC_ATTACHMENT_TXF_RELAY,
		.relay_src = src,
		.relay_id = pkt_id,
		.rx_ms = rx_ms,
		.not_before_ms = not_before_ms,
		.dupe = dupe,
		.wire = wire,
		.wire_len = (uint16_t)len,
	};
	int elen;

	if (wire == NULL || len == 0U || len > MESHTASTIC_ATTACHMENT_WIRE_MAX) {
		return -EINVAL;
	}
	if (!meshtastic_attachment_tx_ready(id)) {
		return -EHOSTUNREACH;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	tx.preset = tab[id].preset;
	tx.tx_seq = ++tab[id].tx_seq;
	k_mutex_unlock(&tab_lock);
	elen = meshtastic_attachment_encode_tx_frame(&tx, env, sizeof(env));
	if (elen < 0) {
		return elen;
	}
	return attachment_send_env(id, env, (size_t)elen, true);
}

int meshtastic_attachment_set_tx_power(uint8_t id, int8_t dbm)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_SET_POLICY_LEN];
	const struct meshtastic_attachment_policy pol = {
		.flags = MESHTASTIC_ATTACHMENT_POL_HAS_TX_POWER,
		.tx_power = dbm,
	};
	int elen;

	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (!used[id]) {
		k_mutex_unlock(&tab_lock);
		return -ENOENT;
	}
	k_mutex_unlock(&tab_lock);
	elen = meshtastic_attachment_encode_set_policy(&pol, env, sizeof(env));
	if (elen < 0) {
		return elen;
	}
	return attachment_send_env(id, env, (size_t)elen, false);
}

int meshtastic_attachment_cancel(uint8_t id, uint32_t src, uint32_t pkt_id)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_TX_CANCEL_LEN];
	int elen;

	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return -EINVAL;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (!used[id] || !tab[id].link_up) {
		k_mutex_unlock(&tab_lock);
		return -ENOTCONN;
	}
	k_mutex_unlock(&tab_lock);
	elen = meshtastic_attachment_encode_tx_cancel(src, pkt_id, env, sizeof(env));
	if (elen < 0) {
		return elen;
	}
	return attachment_send_env(id, env, (size_t)elen, false);
}

bool meshtastic_attachment_modem(uint8_t id, uint8_t *spread_factor, uint32_t *bandwidth_hz)
{
	struct meshtastic_modem_params p;
	uint8_t preset = MESHTASTIC_PRESET_UNKNOWN;

	if (id == 0U || id >= ARRAY_SIZE(tab)) {
		return false;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (used[id]) {
		preset = tab[id].preset;
	}
	k_mutex_unlock(&tab_lock);
	if (preset == MESHTASTIC_PRESET_UNKNOWN ||
	    meshtastic_preset_to_params((meshtastic_Config_LoRaConfig_ModemPreset)preset, false, &p) != 0) {
		return false;
	}
	*spread_factor = p.spread_factor;
	*bandwidth_hz = p.bandwidth_hz;
	return true;
}

int meshtastic_attachment_set_preset(uint8_t id, uint8_t preset)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_SET_PRESET_LEN];
	uint32_t node;
	int len;

	if (id == 0U) {
		return -EINVAL;
	}
	if (id >= ARRAY_SIZE(tab)) {
		return -ENOENT;
	}
	k_mutex_lock(&tab_lock, K_FOREVER);
	if (!used[id]) {
		k_mutex_unlock(&tab_lock);
		return -ENOENT;
	}
	node = tab[id].node;
	k_mutex_unlock(&tab_lock);

	len = meshtastic_attachment_encode_set_preset(preset, env, sizeof(env));
	if (len < 0) {
		return len;
	}
	return meshtastic_attachment_send(node, env, (size_t)len);
}
