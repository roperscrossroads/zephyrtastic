/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic PhoneAPI configuration handshake for phone clients.
 *
 * The official Meshtastic mobile apps expect the same FromRadio sequence as
 * meshtastic_firmware/src/mesh/PhoneAPI.cpp after a ToRadio want_config_id.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_region_presets.h"
#if defined(CONFIG_MESHTASTIC_POSITION)
#include "meshtastic_position.h"
#endif

#include "meshtastic/device_ui.pb.h"
#include "meshtastic/module_config.pb.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Companion apps drive a two-request config handshake via magic nonces (see
 * meshtastic_firmware PhoneAPI.h): ONLY_NODES asks for the node DB only,
 * skipping my_info/metadata/config. Any other nonce = the full config dump. */
#define MESHTASTIC_PHONEAPI_NONCE_ONLY_CONFIG 69420U
#define MESHTASTIC_PHONEAPI_NONCE_ONLY_NODES  69421U

static void fill_my_info(meshtastic_FromRadio *from)
{
	uint8_t node_id[4];

	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_my_info_tag;
	from->my_info.my_node_num = meshtastic_get_node_id();
	from->my_info.min_app_version = 0U;
	from->my_info.nodedb_count = (uint32_t)meshtastic_nodedb_count();
	from->my_info.device_id.size = sizeof(node_id);
	sys_put_le32(meshtastic_get_node_id(), node_id);
	memcpy(from->my_info.device_id.bytes, node_id, sizeof(node_id));
	strncpy(from->my_info.pio_env, "zephyr", sizeof(from->my_info.pio_env) - 1U);
}

static void fill_node_info(meshtastic_FromRadio *from)
{
	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_node_info_tag;
	from->node_info.num = meshtastic_get_node_id();
	from->node_info.last_heard = meshtastic_clock_now_epoch(); /* 0 until clock seeded */
	from->node_info.has_user = true;
	meshtastic_fill_user(&from->node_info.user);
#if defined(CONFIG_MESHTASTIC_POSITION)
	/* Advertise our own position (admin-set fixed, or a live GNSS fix) so the
	 * app shows this node on the map. */
	if (meshtastic_position_get_current(&from->node_info.position) == 0) {
		from->node_info.has_position = true;
	}
#endif
}

/* Map an in-RAM NodeDB entry to a NodeInfo FromRadio frame (peer nodes streamed
 * during the want_config handshake, firmware STATE_SEND_OTHER_NODEINFOS). */
static void fill_other_node_info(meshtastic_FromRadio *from,
				 const struct meshtastic_nodedb_node *node)
{
	meshtastic_NodeInfo *ni = &from->node_info;

	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_node_info_tag;

	ni->num = node->num;
	ni->snr = node->snr;
	ni->channel = node->channel;
	ni->via_mqtt = node->via_mqtt;
	/* Convert the stored uptime-relative last-heard to epoch (0 until the clock
	 * is seeded from GNSS or the phone's set_time_only). */
	ni->last_heard = meshtastic_clock_uptime_to_epoch(node->last_heard_uptime_sec);
	if (node->has_hops_away) {
		ni->has_hops_away = true;
		ni->hops_away = node->hops_away;
	}
	ni->is_favorite = node->is_favorite;
	ni->is_ignored = node->is_ignored;

	if (node->has_user) {
		meshtastic_User *u = &ni->user;

		ni->has_user = true;
		snprintk(u->id, sizeof(u->id), "!%08x", node->num);
		strncpy(u->long_name, node->long_name, sizeof(u->long_name) - 1U);
		strncpy(u->short_name, node->short_name, sizeof(u->short_name) - 1U);
		/* Meshtastic derives the node number from the low 4 bytes of macaddr. */
		u->macaddr[0] = 0x02U;
		u->macaddr[1] = 0x00U;
		u->macaddr[2] = (uint8_t)(node->num >> 24);
		u->macaddr[3] = (uint8_t)(node->num >> 16);
		u->macaddr[4] = (uint8_t)(node->num >> 8);
		u->macaddr[5] = (uint8_t)node->num;
		u->hw_model = node->hw_model;
		u->role = node->role;
		u->is_licensed = node->is_licensed;
		if (node->has_is_unmessagable) {
			u->has_is_unmessagable = true;
			u->is_unmessagable = node->is_unmessagable;
		}
		if (node->public_key_len > 0U) {
			u->public_key.size =
				(pb_size_t)MIN(node->public_key_len, sizeof(u->public_key.bytes));
			memcpy(u->public_key.bytes, node->public_key, u->public_key.size);
		}
	}

	/* Position and device metrics are not retained per node (full-lean NodeDB),
	 * so peer NodeInfo streamed to the phone carries identity + pubkey only; the
	 * app fills position/metrics from packets it receives directly. */
}

static void fill_metadata_frame(meshtastic_FromRadio *from)
{
	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_metadata_tag;
	meshtastic_fill_device_metadata(&from->metadata);
}

static void fill_channel(meshtastic_FromRadio *from, int index)
{
	meshtastic_Channel slot = meshtastic_Channel_init_zero;

	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_channel_tag;
	from->channel.index = index;

	if (meshtastic_config_store_get_channel((uint8_t)index, &slot) == 0 &&
	    slot.role != meshtastic_Channel_Role_DISABLED) {
		from->channel = slot;
		from->channel.index = index;
	} else {
		from->channel.role = meshtastic_Channel_Role_DISABLED;
		from->channel.has_settings = true;
	}
}

static int fill_config_variant(meshtastic_FromRadio *from, pb_size_t which_tag)
{
	int ret;

	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_config_tag;

	ret = meshtastic_config_store_get_config(which_tag, &from->config);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int fill_module_variant(meshtastic_FromRadio *from, pb_size_t which_tag)
{
	int ret;

	from->id = meshtastic_next_fromradio_id();
	from->which_payload_variant = meshtastic_FromRadio_moduleConfig_tag;

	ret = meshtastic_config_store_get_module(which_tag, &from->moduleConfig);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int emit_frame(struct meshtastic_phoneapi *api, const meshtastic_FromRadio *from,
		      enum meshtastic_phoneapi_config_state next_state, uint8_t next_index,
		      struct meshtastic_phoneapi_frame *frame)
{
	int ret = meshtastic_phoneapi_encode_fromradio_frame(from, frame);

	if (ret < 0) {
		return ret;
	}

	api->config_state = next_state;
	api->config_index = next_index;

	return 0;
}

int meshtastic_phoneapi_next_config_frame(struct meshtastic_phoneapi *api,
					  struct meshtastic_phoneapi_frame *frame)
{
	static const pb_size_t config_tags[] = {
		meshtastic_Config_device_tag,     meshtastic_Config_position_tag,
		meshtastic_Config_power_tag,      meshtastic_Config_network_tag,
		meshtastic_Config_display_tag,    meshtastic_Config_lora_tag,
		meshtastic_Config_bluetooth_tag,  meshtastic_Config_security_tag,
		meshtastic_Config_sessionkey_tag, meshtastic_Config_device_ui_tag,
	};
	static const pb_size_t module_tags[] = {
		meshtastic_ModuleConfig_mqtt_tag,
		meshtastic_ModuleConfig_serial_tag,
		meshtastic_ModuleConfig_external_notification_tag,
		meshtastic_ModuleConfig_store_forward_tag,
		meshtastic_ModuleConfig_range_test_tag,
		meshtastic_ModuleConfig_telemetry_tag,
		meshtastic_ModuleConfig_canned_message_tag,
		meshtastic_ModuleConfig_audio_tag,
		meshtastic_ModuleConfig_remote_hardware_tag,
		meshtastic_ModuleConfig_neighbor_info_tag,
		meshtastic_ModuleConfig_ambient_lighting_tag,
		meshtastic_ModuleConfig_detection_sensor_tag,
		meshtastic_ModuleConfig_paxcounter_tag,
		meshtastic_ModuleConfig_statusmessage_tag,
		meshtastic_ModuleConfig_traffic_management_tag,
		meshtastic_ModuleConfig_tak_tag,
		meshtastic_ModuleConfig_mesh_beacon_tag,
	};
	meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
	int ret;

	while (true) {
		switch (api->config_state) {
		case MESHTASTIC_PHONEAPI_CONFIG_IDLE:
			return -ENOENT;
		case MESHTASTIC_PHONEAPI_CONFIG_MY_INFO:
			fill_my_info(&from);
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_DEVICE_UI, 0U,
					  frame);
		case MESHTASTIC_PHONEAPI_CONFIG_DEVICE_UI:
			from.id = meshtastic_next_fromradio_id();
			from.which_payload_variant = meshtastic_FromRadio_deviceuiConfig_tag;
			(void)meshtastic_config_store_get_device_ui(&from.deviceuiConfig);
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_NODE_INFO, 0U,
					  frame);
		case MESHTASTIC_PHONEAPI_CONFIG_NODE_INFO:
			fill_node_info(&from);
			/* ONLY_NODES (app Stage 2): own node info then straight to the
			 * node DB, skipping metadata/config -- mirrors firmware
			 * STATE_SEND_OWN_NODEINFO under SPECIAL_NONCE_ONLY_NODES. */
			return emit_frame(
				api, &from,
				(api->config_request_id == MESHTASTIC_PHONEAPI_NONCE_ONLY_NODES)
					? MESHTASTIC_PHONEAPI_CONFIG_OTHER_NODEINFOS
					: MESHTASTIC_PHONEAPI_CONFIG_METADATA,
				0U, frame);
		case MESHTASTIC_PHONEAPI_CONFIG_METADATA:
			fill_metadata_frame(&from);
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_REGION_PRESETS,
					  0U, frame);
		case MESHTASTIC_PHONEAPI_CONFIG_REGION_PRESETS:
			/* region -> valid-modem-preset compatibility map. The official
			 * apps ASSUME this stage arrives after metadata and before channels
			 * (see firmware PhoneAPI.cpp STATE_SEND_REGION_PRESETS); omitting it
			 * stalls the config handshake. Built from the reference region table
			 * so the app can constrain region+preset selections. */
			from.id = meshtastic_next_fromradio_id();
			from.which_payload_variant = meshtastic_FromRadio_region_presets_tag;
			meshtastic_build_region_preset_map(&from.region_presets);
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_CHANNELS, 0U,
					  frame);
		case MESHTASTIC_PHONEAPI_CONFIG_CHANNELS:
			fill_channel(&from, api->config_index);
			return emit_frame(api, &from,
					  (api->config_index + 1U >= MESHTASTIC_MAX_CHANNELS)
						  ? MESHTASTIC_PHONEAPI_CONFIG_CONFIGS
						  : MESHTASTIC_PHONEAPI_CONFIG_CHANNELS,
					  (api->config_index + 1U >= MESHTASTIC_MAX_CHANNELS)
						  ? 0U
						  : (uint8_t)(api->config_index + 1U),
					  frame);
		case MESHTASTIC_PHONEAPI_CONFIG_CONFIGS:
			while (api->config_index < ARRAY_SIZE(config_tags)) {
				from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
				ret = fill_config_variant(&from, config_tags[api->config_index]);
				if (ret == 0) {
					return emit_frame(
						api, &from,
						(api->config_index + 1U >= ARRAY_SIZE(config_tags))
							? MESHTASTIC_PHONEAPI_CONFIG_MODULES
							: MESHTASTIC_PHONEAPI_CONFIG_CONFIGS,
						(api->config_index + 1U >= ARRAY_SIZE(config_tags))
							? 0U
							: (uint8_t)(api->config_index + 1U),
						frame);
				}
				api->config_index++;
			}
			api->config_state = MESHTASTIC_PHONEAPI_CONFIG_MODULES;
			api->config_index = 0U;
			break;
		case MESHTASTIC_PHONEAPI_CONFIG_MODULES: {
			/* ONLY_CONFIG (app Stage 1) wants config only — skip the node DB
			 * and go straight to the file manifest, mirroring firmware
			 * PhoneAPI.cpp STATE_SEND_MODULECONFIG -> STATE_SEND_FILEMANIFEST
			 * under SPECIAL_NONCE_ONLY_CONFIG. Otherwise stream peers next. */
			enum meshtastic_phoneapi_config_state after_modules =
				(api->config_request_id ==
				 MESHTASTIC_PHONEAPI_NONCE_ONLY_CONFIG)
					? MESHTASTIC_PHONEAPI_CONFIG_FILEMANIFEST
					: MESHTASTIC_PHONEAPI_CONFIG_OTHER_NODEINFOS;

			while (api->config_index < ARRAY_SIZE(module_tags)) {
				from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
				ret = fill_module_variant(&from, module_tags[api->config_index]);
				if (ret == 0) {
					return emit_frame(
						api, &from,
						(api->config_index + 1U >= ARRAY_SIZE(module_tags))
							? after_modules
							: MESHTASTIC_PHONEAPI_CONFIG_MODULES,
						(api->config_index + 1U >= ARRAY_SIZE(module_tags))
							? 0U
							: (uint8_t)(api->config_index + 1U),
						frame);
				}
				api->config_index++;
			}
			api->config_state = after_modules;
			api->config_index = 0U;
			break;
		}
		case MESHTASTIC_PHONEAPI_CONFIG_OTHER_NODEINFOS:
			/* Stream the NodeDB (peers) so the app shows a populated node list;
			 * mirrors firmware STATE_SEND_OTHER_NODEINFOS. Own node already went
			 * out in the NODE_INFO stage, so skip it if the DB holds it. */
			while (api->config_index < meshtastic_nodedb_count()) {
				struct meshtastic_nodedb_node node;

				from = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
				if (meshtastic_nodedb_get_by_index(api->config_index, &node) == 0 &&
				    node.num != meshtastic_get_node_id()) {
					fill_other_node_info(&from, &node);
					return emit_frame(
						api, &from,
						MESHTASTIC_PHONEAPI_CONFIG_OTHER_NODEINFOS,
						(uint8_t)(api->config_index + 1U), frame);
				}
				api->config_index++;
			}
			api->config_state = MESHTASTIC_PHONEAPI_CONFIG_FILEMANIFEST;
			api->config_index = 0U;
			break;
		case MESHTASTIC_PHONEAPI_CONFIG_FILEMANIFEST:
			/* No on-device file manifest yet; emit nothing and advance, matching
			 * firmware's empty-manifest -> config_complete path. ONLY_NODES skips
			 * the manifest AND queue status, going straight to complete. */
			api->config_state =
				(api->config_request_id == MESHTASTIC_PHONEAPI_NONCE_ONLY_NODES)
					? MESHTASTIC_PHONEAPI_CONFIG_COMPLETE
					: MESHTASTIC_PHONEAPI_CONFIG_QUEUE_STATUS;
			api->config_index = 0U;
			break;
		case MESHTASTIC_PHONEAPI_CONFIG_QUEUE_STATUS:
			from.id = meshtastic_next_fromradio_id();
			from.which_payload_variant = meshtastic_FromRadio_queueStatus_tag;
			from.queueStatus.res = 0;
			from.queueStatus.free = (api->count >= api->queue_size)
							? 0U
							: (api->queue_size - api->count);
			from.queueStatus.maxlen = api->queue_size;
			from.queueStatus.mesh_packet_id = 0U;
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_COMPLETE, 0U,
					  frame);
		case MESHTASTIC_PHONEAPI_CONFIG_COMPLETE:
			LOG_INF("%s config_complete_id=%u emitted", api->name, api->config_request_id);
			from.id = meshtastic_next_fromradio_id();
			from.which_payload_variant = meshtastic_FromRadio_config_complete_id_tag;
			from.config_complete_id = api->config_request_id;
			return emit_frame(api, &from, MESHTASTIC_PHONEAPI_CONFIG_IDLE, 0U, frame);
		default:
			api->config_state = MESHTASTIC_PHONEAPI_CONFIG_IDLE;
			api->config_index = 0U;
			return -EINVAL;
		}
	}
}

void meshtastic_phoneapi_enqueue_phone_config(struct meshtastic_phoneapi *api, uint32_t request_id)
{
	LOG_INF("%s want_config nonce=%u", api->name, request_id);

	k_mutex_lock(&api->lock, K_FOREVER);
	api->head = 0U;
	api->tail = 0U;
	api->count = 0U;
	api->current_valid = false;
	/* ONLY_NODES (app Stage 2) starts at own node info; every other nonce
	 * (incl. ONLY_CONFIG and legacy full) starts at my_info. */
	api->config_state = (request_id == MESHTASTIC_PHONEAPI_NONCE_ONLY_NODES)
				    ? MESHTASTIC_PHONEAPI_CONFIG_NODE_INFO
				    : MESHTASTIC_PHONEAPI_CONFIG_MY_INFO;
	api->config_index = 0U;
	api->config_request_id = request_id;
	k_mutex_unlock(&api->lock);

	if (api->invalidate_delivery != NULL) {
		api->invalidate_delivery(api);
	}

	meshtastic_phoneapi_notify_data_ready(api);
}

int meshtastic_phoneapi_set_channel(uint8_t index, const meshtastic_Channel *channel)
{
	return meshtastic_config_store_set_channel(index, channel);
}
