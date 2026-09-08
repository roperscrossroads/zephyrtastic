/* SPDX-License-Identifier: GPL-3.0 */

/* See meshtastic_meshbeacon.h for the design and where this stops short of the
 * reference. Reference: firmware/src/modules/MeshBeaconModule.cpp. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh_beacon.pb.h"
#include "meshtastic/module_config.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_meshbeacon.h"
#include "meshtastic_modules.h"
#include "meshtastic_preset.h"
#include "meshtastic_region_presets.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

static struct k_work_delayable beacon_work;
static K_MUTEX_DEFINE(offer_lock);
static struct meshtastic_meshbeacon_offer last_offer;
static struct meshtastic_meshbeacon_stats stats;

/* ---- config ------------------------------------------------------------------ */

static void read_cfg(meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	(void)meshtastic_config_store_get_module(meshtastic_ModuleConfig_mesh_beacon_tag, &mod);
	*cfg = mod.payload_variant.mesh_beacon;
}

static bool region_known(meshtastic_Config_LoRaConfig_RegionCode region)
{
	struct meshtastic_region_info info;

	return meshtastic_region_info(region, &info) == 0;
}

static bool preset_known(meshtastic_Config_LoRaConfig_ModemPreset preset)
{
	struct meshtastic_modem_params p;

	/* preset_to_params() maps an unknown value to a default rather than
	 * refusing it, so bound the enum first. */
	return preset <= _meshtastic_Config_LoRaConfig_ModemPreset_MAX &&
	       meshtastic_preset_to_params(preset, false, &p) == 0;
}

void meshtastic_meshbeacon_sanitise(meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	if (cfg == NULL) {
		return;
	}

	/* Reference: hard cap at 100 chars. */
	cfg->broadcast_message[MESHTASTIC_MESHBEACON_MESSAGE_MAX] = '\0';

	/* Reference: 0 means unset (-> default); a value below the minimum is
	 * raised to it. */
	if (cfg->broadcast_interval_secs != 0U &&
	    cfg->broadcast_interval_secs < CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC) {
		cfg->broadcast_interval_secs = CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC;
	}

	if (cfg->has_broadcast_offer_preset && !preset_known(cfg->broadcast_offer_preset)) {
		LOG_WRN("Beacon: broadcast_offer_preset %d unknown, clearing",
			(int)cfg->broadcast_offer_preset);
		cfg->has_broadcast_offer_preset = false;
	}
	if (cfg->broadcast_offer_region != meshtastic_Config_LoRaConfig_RegionCode_UNSET &&
	    !region_known(cfg->broadcast_offer_region)) {
		LOG_WRN("Beacon: broadcast_offer_region %d unknown, clearing",
			(int)cfg->broadcast_offer_region);
		cfg->broadcast_offer_region = meshtastic_Config_LoRaConfig_RegionCode_UNSET;
	}

	for (pb_size_t i = 0; i < cfg->broadcast_targets_count; i++) {
		meshtastic_ModuleConfig_MeshBeaconConfig_BroadcastTarget *t =
			&cfg->broadcast_targets[i];

		if (t->region != meshtastic_Config_LoRaConfig_RegionCode_UNSET &&
		    !region_known(t->region)) {
			LOG_WRN("Beacon: broadcast_targets[%u] region %d unknown, clearing", i,
				(int)t->region);
			t->region = meshtastic_Config_LoRaConfig_RegionCode_UNSET;
		}
		if (t->has_preset && !preset_known(t->preset)) {
			LOG_WRN("Beacon: broadcast_targets[%u] preset %d unknown, clearing", i,
				(int)t->preset);
			t->has_preset = false;
			t->has_channel_index = false;
		}
		if (t->has_channel_index && t->channel_index >= MESHTASTIC_MAX_CHANNELS) {
			LOG_WRN("Beacon: broadcast_targets[%u] channel_index %u out of range, "
				"clearing",
				i, (unsigned int)t->channel_index);
			t->has_channel_index = false;
		}
	}
}

uint32_t meshtastic_meshbeacon_interval_secs(void)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;

	read_cfg(&cfg);
	if (cfg.broadcast_interval_secs == 0U ||
	    cfg.broadcast_interval_secs < CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC) {
		return CONFIG_MESHTASTIC_MESHBEACON_MIN_INTERVAL_SEC;
	}
	return cfg.broadcast_interval_secs;
}

int meshtastic_meshbeacon_set(const meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;
	int ret;

	if (cfg == NULL) {
		return -EINVAL;
	}

	mod.which_payload_variant = meshtastic_ModuleConfig_mesh_beacon_tag;
	mod.payload_variant.mesh_beacon = *cfg;
	meshtastic_meshbeacon_sanitise(&mod.payload_variant.mesh_beacon);

	ret = meshtastic_config_store_set_module(&mod);
	if (ret < 0) {
		return ret;
	}
	meshtastic_meshbeacon_config_changed();
	return 0;
}

/* ---- broadcasting ------------------------------------------------------------ */

static bool cfg_has_text(const meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	return cfg->broadcast_message[0] != '\0';
}

static bool cfg_has_offer(const meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	return cfg->has_broadcast_offer_preset || cfg->has_broadcast_offer_channel ||
	       cfg->broadcast_offer_region != meshtastic_Config_LoRaConfig_RegionCode_UNSET;
}

static void fill_offer(meshtastic_MeshBeacon *b, const meshtastic_ModuleConfig_MeshBeaconConfig *cfg)
{
	if (cfg->has_broadcast_offer_channel) {
		/* PSK included on purpose: a public join-invitation, not a secret
		 * (reference rebuildCache). */
		b->has_offer_channel = true;
		b->offer_channel = cfg->broadcast_offer_channel;
	}
	b->has_offer_preset = cfg->has_broadcast_offer_preset;
	b->offer_preset = cfg->broadcast_offer_preset;
	b->offer_region = cfg->broadcast_offer_region;
}

static int encode_beacon(const meshtastic_MeshBeacon *b, uint8_t *buf, size_t cap, size_t *len)
{
	pb_ostream_t stream = pb_ostream_from_buffer(buf, cap);

	if (!pb_encode(&stream, meshtastic_MeshBeacon_fields, b)) {
		LOG_ERR("MeshBeacon encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}
	*len = stream.bytes_written;
	return 0;
}

/* One zero-hop frame on @p channel_index. Reference stampPacket: broadcast,
 * hop_limit 0, priority BACKGROUND (the port's tier comes from the port number),
 * no want_ack; hop_start 1 under the legacy flag. */
static int send_zero_hop(uint32_t portnum, const uint8_t *payload, size_t len,
			 uint8_t channel_index, bool legacy)
{
	struct meshtastic_packet pkt = {
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = portnum,
		.payload = payload,
		.payload_len = len,
		.channel_index = channel_index,
		.hop_limit = 0U,
		.hop_start = legacy ? 1U : 0U,
		.zero_hop = true,
	};
	int ret = meshtastic_send_packet(&pkt, K_NO_WAIT);

	if (ret == 0) {
		stats.frames_sent++;
	} else {
		LOG_WRN("Beacon: frame on port %u not queued (%d)", (unsigned int)portnum, ret);
	}
	return ret;
}

static bool slot_configured(uint8_t index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(index);

	return ch != NULL && ch->role != meshtastic_Channel_Role_DISABLED && ch->has_settings &&
	       (ch->settings.name[0] != '\0' || ch->settings.psk.size > 0U);
}

/* An inline channel (name + PSK) is honoured only as the table slot it matches. */
static bool find_slot_for(const meshtastic_ChannelSettings *want, uint8_t *index)
{
	for (uint8_t i = 0; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *ch = meshtastic_channels_get(i);

		if (ch == NULL || ch->role == meshtastic_Channel_Role_DISABLED ||
		    !ch->has_settings) {
			continue;
		}
		if (strncmp(ch->settings.name, want->name, sizeof(want->name)) == 0 &&
		    ch->settings.psk.size == want->psk.size &&
		    memcmp(ch->settings.psk.bytes, want->psk.bytes, want->psk.size) == 0) {
			*index = i;
			return true;
		}
	}
	return false;
}

static bool radio_matches(bool has_preset, meshtastic_Config_LoRaConfig_ModemPreset preset,
			  meshtastic_Config_LoRaConfig_RegionCode region)
{
	if (has_preset && preset != mt.modem_preset) {
		return false;
	}
	if (region != meshtastic_Config_LoRaConfig_RegionCode_UNSET &&
	    region != meshtastic_preset_region()) {
		return false;
	}
	return true;
}

/* Resolve target @p i of the section to a channel-table slot on the running
 * radio config. false: skipped (counted, logged). */
static bool resolve_target(const meshtastic_ModuleConfig_MeshBeaconConfig *cfg, int i,
			   uint8_t *channel_index)
{
	*channel_index = meshtastic_channels_primary_index();

	if (cfg->broadcast_targets_count == 0U) {
		/* Single-target path: the inline fields. */
		if (!radio_matches(cfg->has_broadcast_on_preset, cfg->broadcast_on_preset,
				   cfg->broadcast_on_region)) {
			LOG_WRN("Beacon: broadcast_on_preset/region differ from the running "
				"radio; radio switching is not supported, skipped");
			stats.targets_radio_switch++;
			return false;
		}
		if (cfg->has_broadcast_on_channel &&
		    (cfg->broadcast_on_channel.name[0] != '\0' ||
		     cfg->broadcast_on_channel.psk.size > 0U)) {
			if (!find_slot_for(&cfg->broadcast_on_channel, channel_index)) {
				LOG_WRN("Beacon: broadcast_on_channel \"%s\" is not a channel-table "
					"slot; skipped",
					cfg->broadcast_on_channel.name);
				stats.targets_no_slot++;
				return false;
			}
		}
		return true;
	}

	const meshtastic_ModuleConfig_MeshBeaconConfig_BroadcastTarget *bt =
		&cfg->broadcast_targets[i];

	if (!radio_matches(bt->has_preset, bt->preset, bt->region)) {
		LOG_WRN("Beacon: target %d needs preset %d/region %d, running %d/%d; radio "
			"switching is not supported, skipped",
			i, (int)bt->preset, (int)bt->region, (int)mt.modem_preset,
			(int)meshtastic_preset_region());
		stats.targets_radio_switch++;
		return false;
	}
	if (bt->has_channel_index) {
		if (bt->channel_index < MESHTASTIC_MAX_CHANNELS &&
		    slot_configured((uint8_t)bt->channel_index)) {
			*channel_index = (uint8_t)bt->channel_index;
		} else {
			/* Reference: out of range or blank -> the preset's default
			 * channel, which is the primary here. */
			LOG_DBG("Beacon: target %d channel_index %u unusable, primary", i,
				(unsigned int)bt->channel_index);
		}
	}
	return true;
}

int meshtastic_meshbeacon_send(void)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	uint8_t offer_buf[meshtastic_MeshBeacon_size];
	uint8_t combined_buf[meshtastic_MeshBeacon_size];
	size_t offer_len = 0U;
	size_t combined_len = 0U;
	uint8_t sent_on[MESHTASTIC_MAX_CHANNELS];
	int sent_on_count = 0;
	int targets;
	int queued = 0;
	int last_err = 0;

	if (meshtastic_device_role() == meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN) {
		return -EPERM; /* reference runOnce: a hidden client never beacons */
	}

	read_cfg(&cfg);
	const bool has_text = cfg_has_text(&cfg);
	const bool has_offer = cfg_has_offer(&cfg);

	if (!has_text && !has_offer) {
		stats.cycles_empty++;
		return -ENODATA;
	}

	/* Reference sendBeacon: three independent decisions, not an if/else chain. */
	const bool legacy = (cfg.flags & MESHTASTIC_MESHBEACON_FLAG_LEGACY_SPLIT) != 0U;
	const bool split_both = legacy && has_offer && has_text;
	const bool send_offer_only = split_both || (has_offer && !has_text);
	const bool send_text_only = split_both || (!has_offer && has_text);
	const bool send_combined = !legacy && has_offer && has_text;

	if (send_offer_only) {
		meshtastic_MeshBeacon b = meshtastic_MeshBeacon_init_zero;

		fill_offer(&b, &cfg);
		if (encode_beacon(&b, offer_buf, sizeof(offer_buf), &offer_len) < 0) {
			return -ENOMEM;
		}
	}
	if (send_combined) {
		meshtastic_MeshBeacon b = meshtastic_MeshBeacon_init_zero;

		strncpy(b.message, cfg.broadcast_message, sizeof(b.message) - 1U);
		fill_offer(&b, &cfg);
		if (encode_beacon(&b, combined_buf, sizeof(combined_buf), &combined_len) < 0) {
			return -ENOMEM;
		}
	}

	/* Reference: an empty target list still beacons once, on the running
	 * config over the primary channel. */
	targets = (cfg.broadcast_targets_count > 0U) ? (int)cfg.broadcast_targets_count : 1;

	for (int i = 0; i < targets; i++) {
		uint8_t ch;
		bool dup = false;

		if (!resolve_target(&cfg, i, &ch)) {
			continue;
		}
		/* Reference dedup: two targets that resolve to the same radio config
		 * would just repeat the same frame. */
		for (int s = 0; s < sent_on_count; s++) {
			if (sent_on[s] == ch) {
				dup = true;
				break;
			}
		}
		if (dup) {
			LOG_DBG("Beacon: target %d duplicates an earlier one, skipped", i);
			continue;
		}
		sent_on[sent_on_count++] = ch;

		if (send_offer_only && offer_len > 0U) {
			int ret = send_zero_hop(MESHTASTIC_PORT_MESH_BEACON, offer_buf, offer_len,
						ch, legacy);

			if (ret == 0) {
				queued++;
				LOG_INF("Beacon: offer-only MESH_BEACON_APP on channel %u", ch);
			} else {
				last_err = ret;
			}
		}
		if (send_text_only) {
			/* NUL-terminated by construction: nanopb terminates decoded
			 * strings and sanitise() caps the field. */
			size_t len = strlen(cfg.broadcast_message);
			int ret = send_zero_hop(MESHTASTIC_PORT_TEXT_MESSAGE,
						(const uint8_t *)cfg.broadcast_message, len, ch,
						legacy);

			if (ret == 0) {
				queued++;
				LOG_INF("Beacon: TEXT_MESSAGE_APP \"%.40s\" on channel %u",
					cfg.broadcast_message, ch);
			} else {
				last_err = ret;
			}
		}
		if (send_combined && combined_len > 0U) {
			int ret = send_zero_hop(MESHTASTIC_PORT_MESH_BEACON, combined_buf,
						combined_len, ch, legacy);

			if (ret == 0) {
				queued++;
				LOG_INF("Beacon: MESH_BEACON_APP offer+\"%.40s\" on channel %u",
					cfg.broadcast_message, ch);
			} else {
				last_err = ret;
			}
		}
	}

	if (queued > 0) {
		return 0;
	}
	return (last_err != 0) ? last_err : -ENOENT;
}

static void beacon_work_fn(struct k_work *work)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;

	ARG_UNUSED(work);

	read_cfg(&cfg);
	if ((cfg.flags & MESHTASTIC_MESHBEACON_FLAG_BROADCAST) == 0U) {
		return;
	}
	(void)meshtastic_meshbeacon_send();
	(void)k_work_reschedule(&beacon_work, K_SECONDS(meshtastic_meshbeacon_interval_secs()));
}

void meshtastic_meshbeacon_config_changed(void)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;

	read_cfg(&cfg);
	if ((cfg.flags & MESHTASTIC_MESHBEACON_FLAG_BROADCAST) != 0U) {
		(void)k_work_reschedule(&beacon_work,
					K_SECONDS(CONFIG_MESHTASTIC_MESHBEACON_START_DELAY_SEC));
		LOG_DBG("Beacon: broadcast armed, first in %d s then every %u s",
			CONFIG_MESHTASTIC_MESHBEACON_START_DELAY_SEC,
			meshtastic_meshbeacon_interval_secs());
	} else {
		(void)k_work_cancel_delayable(&beacon_work);
		LOG_DBG("Beacon: broadcast off");
	}
}

/* ---- listening ---------------------------------------------------------------- */

bool meshtastic_meshbeacon_last_offer(struct meshtastic_meshbeacon_offer *out)
{
	bool valid;

	if (out == NULL) {
		return false;
	}
	k_mutex_lock(&offer_lock, K_FOREVER);
	*out = last_offer;
	valid = last_offer.valid;
	k_mutex_unlock(&offer_lock);
	return valid;
}

void meshtastic_meshbeacon_clear_offer(void)
{
	k_mutex_lock(&offer_lock, K_FOREVER);
	memset(&last_offer, 0, sizeof(last_offer));
	k_mutex_unlock(&offer_lock);
}

void meshtastic_meshbeacon_stats(struct meshtastic_meshbeacon_stats *out)
{
	if (out != NULL) {
		*out = stats;
	}
}

void meshtastic_meshbeacon_reset(void)
{
	meshtastic_meshbeacon_clear_offer();
	memset(&stats, 0, sizeof(stats));
}

static void meshbeacon_on_packet(const struct meshtastic_packet *packet,
				 const meshtastic_MeshPacket *mesh)
{
	meshtastic_ModuleConfig_MeshBeaconConfig cfg;
	meshtastic_MeshBeacon b = meshtastic_MeshBeacon_init_zero;
	pb_istream_t is;
	uint32_t from;
	const uint8_t *payload;
	size_t payload_len;
	bool has_offer;
	bool has_text;

	if (packet == NULL) {
		return;
	}

	read_cfg(&cfg);
	if ((cfg.flags & MESHTASTIC_MESHBEACON_FLAG_LISTEN) == 0U) {
		return; /* reference wantPacket: not listening, not ours */
	}

	from = mesh ? mesh->from : packet->from;
	payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	if (from == 0U || from == meshtastic_get_node_id() || payload == NULL) {
		return;
	}

	is = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&is, meshtastic_MeshBeacon_fields, &b)) {
		LOG_DBG("MeshBeacon from 0x%08x undecodable: %s", from, PB_GET_ERROR(&is));
		return;
	}

	has_offer = b.has_offer_channel || b.has_offer_preset ||
		    b.offer_region != meshtastic_Config_LoRaConfig_RegionCode_UNSET;
	has_text = b.message[0] != '\0';
	if (!has_text && !has_offer) {
		return;
	}
	stats.beacons_heard++;

	/* The text stays in the beacon: the packet reaches the phone as-is, and
	 * an aware client renders it from there (reference: no synthesised
	 * TEXT_MESSAGE_APP, no wake-the-device event). */
	if (has_text) {
		LOG_INF("Beacon from 0x%08x: \"%.40s\"", from, b.message);
	}

	if (has_offer) {
		k_mutex_lock(&offer_lock, K_FOREVER);
		last_offer.valid = true;
		last_offer.sender = from;
		last_offer.has_channel = b.has_offer_channel;
		if (b.has_offer_channel) {
			last_offer.channel = b.offer_channel;
		} else {
			memset(&last_offer.channel, 0, sizeof(last_offer.channel));
		}
		last_offer.region = b.offer_region;
		last_offer.has_preset = b.has_offer_preset;
		last_offer.preset = b.offer_preset;
		last_offer.heard_uptime_ms = k_uptime_get();
		k_mutex_unlock(&offer_lock);
		stats.offers_cached++;
		LOG_INF("Beacon: offer from 0x%08x cached (channel \"%s\", preset %d, region %d) "
			"-- not applied",
			from, b.has_offer_channel ? b.offer_channel.name : "-",
			b.has_offer_preset ? (int)b.offer_preset : -1, (int)b.offer_region);
	}
}

MESHTASTIC_MODULE_DEFINE(meshbeacon, MESHTASTIC_PORT_MESH_BEACON, 0, meshbeacon_on_packet, NULL);

int meshtastic_meshbeacon_init(void)
{
	k_work_init_delayable(&beacon_work, beacon_work_fn);
	meshtastic_meshbeacon_config_changed();
	return 0;
}
