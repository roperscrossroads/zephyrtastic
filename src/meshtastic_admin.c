/* SPDX-License-Identifier: GPL-3.0
 *
 * Local AdminMessage handling — the config-write engine. Intercepts ADMIN_APP
 * packets the connected app addresses to this node, dispatches get_/set_ config,
 * module config, channel and owner operations into the config store, and
 * synthesizes the ROUTING ACK / AdminMessage responses the app waits on.
 *
 * Scope: LOCAL transport only (from == this node). The session passkey is
 * returned in responses but not validated, mirroring the reference firmware's
 * behavior for directly-connected (from==0) clients.
 */

#include "meshtastic_admin.h"

#include <errno.h>
#include <string.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_clock.h"
#if defined(CONFIG_MESHTASTIC_POSITION)
#include "meshtastic_position.h"
#endif
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_settings.h"

#include "meshtastic/admin.pb.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

BUILD_ASSERT(sizeof(meshtastic_AdminMessage) < 1024,
	     "meshtastic_AdminMessage larger than expected — check admin.options bounds");

/*
 * Module-scoped protobuf scratch, guarded by admin_lock, so the large
 * AdminMessage (embeds a full Config/ModuleConfig union) never lands on the
 * PhoneAPI RX thread stack.
 */
K_MUTEX_DEFINE(admin_lock);
static meshtastic_AdminMessage admin_req;
static meshtastic_AdminMessage admin_resp;

/* begin_edit_settings ... commit_edit_settings transaction state (single local
 * session; the app opens/closes serially). */
static bool admin_edit_open;

/* Set when a write lands a section the port applies only on reboot (module
 * config, and non-core config sections apply_core doesn't apply live). Fired
 * outside a txn immediately, or deferred to commit — mirrors the reference
 * firmware's requiresReboot + defer-to-commit behavior. */
static bool reboot_pending;

/* Post-commit / post-set reboot delay (firmware DEFAULT_REBOOT_SECONDS). */
#define ADMIN_REBOOT_SECONDS 7

static void admin_reboot_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(admin_reboot_work, admin_reboot_work_fn);

static void admin_reboot_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	meshtastic_settings_flush(); /* persist pending/committed writes first */
#endif
	LOG_WRN("admin: rebooting now");
	sys_reboot(SYS_REBOOT_COLD);
}

/* Schedule the deferred reboot for a config change that only takes effect on
 * restart, and clear the pending flag. */
static void admin_schedule_reboot(void)
{
	LOG_INF("admin: config change needs reboot in %d s", ADMIN_REBOOT_SECONDS);
	k_work_reschedule(&admin_reboot_work, K_SECONDS(ADMIN_REBOOT_SECONDS));
	reboot_pending = false;
}

/* ------------------------------------------------------------------------- */
/* Emit helpers — both build a struct meshtastic_packet and hand it to        */
/* meshtastic_phoneapi_on_packet(), which wraps it as a FromRadio.packet and  */
/* fans it out to every connected transport (no radio TX).                    */
/* ------------------------------------------------------------------------- */

static void fill_session_passkey(meshtastic_AdminMessage *resp)
{
	uint32_t nid = meshtastic_get_node_id();

	/* Local MVP: the app only requires a non-empty 8-byte key (never
	 * validated for from==self). Derive a stable value from the node id. */
	resp->session_passkey.size = 8U;
	for (int i = 0; i < 8; i++) {
		resp->session_passkey.bytes[i] =
			(uint8_t)((nid >> ((i % 4) * 8)) & 0xFFU) ^ (uint8_t)(0xA5U + i);
	}
}

/* Emit an ADMIN_APP FromRadio.packet response carrying resp, correlated to the
 * request via request_id. resp is the module scratch (caller holds admin_lock). */
static void admin_emit_reply(const meshtastic_MeshPacket *req, meshtastic_AdminMessage *resp)
{
	uint8_t buf[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};

	fill_session_passkey(resp);

	if (!pb_encode(&stream, meshtastic_AdminMessage_fields, resp)) {
		LOG_ERR("admin: response encode failed (variant %u): %s",
			(unsigned int)resp->which_payload_variant, PB_GET_ERROR(&stream));
		return;
	}

	pkt.portnum = meshtastic_PortNum_ADMIN_APP;
	pkt.from = meshtastic_get_node_id();
	pkt.to = req->from;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.request_id = req->id;
	pkt.channel_index = 0U;
	pkt.payload = buf;
	pkt.payload_len = stream.bytes_written;

	meshtastic_phoneapi_on_packet(&pkt);
}

/* Emit a ROUTING_APP ACK the app matches to its want_ack request_id. */
static void admin_ack_write(const meshtastic_MeshPacket *req, meshtastic_Routing_Error err)
{
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	uint8_t buf[16];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};

	routing.which_variant = meshtastic_Routing_error_reason_tag;
	routing.error_reason = err;

	if (!pb_encode(&stream, meshtastic_Routing_fields, &routing)) {
		LOG_ERR("admin: routing ack encode failed");
		return;
	}

	pkt.portnum = meshtastic_PortNum_ROUTING_APP;
	pkt.from = meshtastic_get_node_id();
	pkt.to = req->from;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.request_id = req->id;
	pkt.channel_index = 0U;
	pkt.payload = buf;
	pkt.payload_len = stream.bytes_written;

	meshtastic_phoneapi_on_packet(&pkt);
}

/* ------------------------------------------------------------------------- */
/* Getters (Stage 3) — reply with the matching *_response arm + passkey.      */
/* ------------------------------------------------------------------------- */

/* Getters return true iff a response was emitted; the dispatcher uses this to
 * skip the redundant ROUTING ACK on success (firmware sends only the response),
 * while still ACKing a failed getter so the app's request doesn't time out. */
static bool handle_get_device_metadata(const meshtastic_MeshPacket *req)
{
	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant =
		meshtastic_AdminMessage_get_device_metadata_response_tag;
	meshtastic_fill_device_metadata(&admin_resp.payload_variant.get_device_metadata_response);
	admin_emit_reply(req, &admin_resp);
	return true;
}

static bool handle_get_config(const meshtastic_MeshPacket *req, uint32_t config_type)
{
	/* admin ConfigType N maps to Config oneof tag N+1 (verified). */
	pb_size_t tag = (pb_size_t)(config_type + 1U);

	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant = meshtastic_AdminMessage_get_config_response_tag;
	if (meshtastic_config_store_get_config(
		    tag, &admin_resp.payload_variant.get_config_response) < 0) {
		LOG_WRN("admin: get_config unknown type %u", (unsigned int)config_type);
		return false;
	}
	admin_emit_reply(req, &admin_resp);
	return true;
}

static bool handle_get_module_config(const meshtastic_MeshPacket *req, uint32_t module_type)
{
	pb_size_t tag = (pb_size_t)(module_type + 1U);

	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant =
		meshtastic_AdminMessage_get_module_config_response_tag;
	if (meshtastic_config_store_get_module(
		    tag, &admin_resp.payload_variant.get_module_config_response) < 0) {
		LOG_WRN("admin: get_module unknown type %u", (unsigned int)module_type);
		return false;
	}
	admin_emit_reply(req, &admin_resp);
	return true;
}

static bool handle_get_channel(const meshtastic_MeshPacket *req, uint32_t one_based)
{
	uint8_t index;

	/* get_channel_request is 1-based (0 means "not present" in protobuf). */
	if (one_based == 0U) {
		LOG_WRN("admin: get_channel index 0 (expected 1-based)");
		return false;
	}
	index = (uint8_t)(one_based - 1U);

	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant = meshtastic_AdminMessage_get_channel_response_tag;
	if (meshtastic_config_store_get_channel(
		    index, &admin_resp.payload_variant.get_channel_response) < 0) {
		LOG_WRN("admin: get_channel bad index %u", (unsigned int)index);
		return false;
	}
	admin_resp.payload_variant.get_channel_response.index = (int8_t)index;
	admin_emit_reply(req, &admin_resp);
	return true;
}

static bool handle_get_owner(const meshtastic_MeshPacket *req)
{
	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant = meshtastic_AdminMessage_get_owner_response_tag;
	meshtastic_fill_user(&admin_resp.payload_variant.get_owner_response);
	admin_emit_reply(req, &admin_resp);
	return true;
}

/* True if name is non-empty but contains only whitespace (rejected for owner). */
static bool owner_name_all_whitespace(const char *name)
{
	bool any = false;

	for (const char *p = name; *p != '\0'; p++) {
		if (*p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
			return false;
		}
		any = true;
	}
	return any;
}

/* ------------------------------------------------------------------------- */
/* Dispatcher                                                                 */
/* ------------------------------------------------------------------------- */

bool meshtastic_admin_handle_local(const meshtastic_MeshPacket *pkt)
{
	pb_istream_t stream;
	meshtastic_Routing_Error ack_err = meshtastic_Routing_Error_NONE;
	bool response_sent = false;
	int ret;

	if (pkt == NULL) {
		return false;
	}

	k_mutex_lock(&admin_lock, K_FOREVER);

	admin_req = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	stream = pb_istream_from_buffer(pkt->decoded.payload.bytes, pkt->decoded.payload.size);
	if (!pb_decode(&stream, meshtastic_AdminMessage_fields, &admin_req)) {
		LOG_WRN("admin: AdminMessage decode failed: %s", PB_GET_ERROR(&stream));
		k_mutex_unlock(&admin_lock);
		return true; /* consumed — never forward admin onto the mesh */
	}

	LOG_DBG("admin: variant %u from 0x%08x id=0x%08x",
		(unsigned int)admin_req.which_payload_variant, pkt->from, pkt->id);

	switch (admin_req.which_payload_variant) {
	/* Getters — a response already carries the passkey, so the redundant
	 * ROUTING ACK is suppressed below when response_sent (firmware behavior). */
	case meshtastic_AdminMessage_get_device_metadata_request_tag:
		response_sent = handle_get_device_metadata(pkt);
		break;
	case meshtastic_AdminMessage_get_config_request_tag:
		response_sent = handle_get_config(pkt, admin_req.payload_variant.get_config_request);
		break;
	case meshtastic_AdminMessage_get_module_config_request_tag:
		response_sent = handle_get_module_config(
			pkt, admin_req.payload_variant.get_module_config_request);
		break;
	case meshtastic_AdminMessage_get_channel_request_tag:
		response_sent = handle_get_channel(pkt, admin_req.payload_variant.get_channel_request);
		break;
	case meshtastic_AdminMessage_get_owner_request_tag:
		response_sent = handle_get_owner(pkt);
		break;

	/* Setters — apply + persist (persistence deferred if a txn is open). */
	case meshtastic_AdminMessage_set_config_tag:
		ret = meshtastic_config_store_set_config(&admin_req.payload_variant.set_config);
		if (ret < 0) {
			LOG_WRN("admin: set_config failed (%d)", ret);
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
		} else {
			/* apply_core applies device/lora core fields live; every other
			 * section is persisted and applied on reboot. */
			pb_size_t which = admin_req.payload_variant.set_config.which_payload_variant;

			if (which != meshtastic_Config_device_tag &&
			    which != meshtastic_Config_lora_tag) {
				reboot_pending = true;
			}
		}
		break;
	case meshtastic_AdminMessage_set_module_config_tag:
		ret = meshtastic_config_store_set_module(
			&admin_req.payload_variant.set_module_config);
		if (ret < 0) {
			LOG_WRN("admin: set_module_config failed (%d)", ret);
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
		} else {
			/* No module runs its config live; effect requires a reboot. */
			reboot_pending = true;
		}
		break;
	case meshtastic_AdminMessage_set_channel_tag:
		ret = meshtastic_config_store_set_channel(
			(uint8_t)admin_req.payload_variant.set_channel.index,
			&admin_req.payload_variant.set_channel);
		if (ret < 0) {
			LOG_WRN("admin: set_channel failed (%d)", ret);
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
		}
		break;
	case meshtastic_AdminMessage_set_owner_tag:
		/* Reject a non-empty long_name that is only whitespace (firmware
		 * AdminModule::handleSetOwner does the same). */
		if (owner_name_all_whitespace(admin_req.payload_variant.set_owner.long_name)) {
			LOG_WRN("admin: set_owner rejected all-whitespace long_name");
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
			break;
		}
		ret = meshtastic_config_store_set_owner(&admin_req.payload_variant.set_owner);
		if (ret < 0) {
			LOG_WRN("admin: set_owner failed (%d)", ret);
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
		}
		break;

	/* NodeDB mutations (in-RAM). Best-effort like the reference firmware:
	 * acting on a node not in the DB is a benign no-op (ack NONE), so the app
	 * doesn't surface a spurious error. Changes become visible to the app on
	 * its next want_config node stream. */
	case meshtastic_AdminMessage_set_favorite_node_tag:
		ret = meshtastic_nodedb_set_favorite(
			admin_req.payload_variant.set_favorite_node, true);
		LOG_INF("admin: favorite 0x%08x (%d)",
			admin_req.payload_variant.set_favorite_node, ret);
		break;
	case meshtastic_AdminMessage_remove_favorite_node_tag:
		ret = meshtastic_nodedb_set_favorite(
			admin_req.payload_variant.remove_favorite_node, false);
		LOG_INF("admin: unfavorite 0x%08x (%d)",
			admin_req.payload_variant.remove_favorite_node, ret);
		break;
	case meshtastic_AdminMessage_set_ignored_node_tag:
		ret = meshtastic_nodedb_set_ignored(
			admin_req.payload_variant.set_ignored_node, true);
		LOG_INF("admin: ignore 0x%08x (%d)",
			admin_req.payload_variant.set_ignored_node, ret);
		break;
	case meshtastic_AdminMessage_remove_ignored_node_tag:
		ret = meshtastic_nodedb_set_ignored(
			admin_req.payload_variant.remove_ignored_node, false);
		LOG_INF("admin: unignore 0x%08x (%d)",
			admin_req.payload_variant.remove_ignored_node, ret);
		break;
	case meshtastic_AdminMessage_remove_by_nodenum_tag:
		ret = meshtastic_nodedb_remove(admin_req.payload_variant.remove_by_nodenum);
		if (ret == -EINVAL) {
			/* Refusing to remove the local node is a real client error. */
			LOG_WRN("admin: remove_by_nodenum rejected (self)");
			ack_err = meshtastic_Routing_Error_BAD_REQUEST;
		} else {
			LOG_INF("admin: remove_by_nodenum 0x%08x (%d)",
				admin_req.payload_variant.remove_by_nodenum, ret);
		}
		break;

	/* Fixed position: store the manual coordinates in the position module
	 * (which broadcasts them + answers Position requests) and flip the
	 * position config flag the app reads back. */
#if defined(CONFIG_MESHTASTIC_POSITION)
	case meshtastic_AdminMessage_set_fixed_position_tag:
		meshtastic_position_set_fixed(&admin_req.payload_variant.set_fixed_position);
		(void)meshtastic_config_store_set_position_fixed(true);
		LOG_INF("admin: set_fixed_position");
		break;
	case meshtastic_AdminMessage_remove_fixed_position_tag:
		meshtastic_position_clear_fixed();
		(void)meshtastic_config_store_set_position_fixed(false);
		LOG_INF("admin: remove_fixed_position");
		break;
#endif

	/* Edit transaction (Stage 5). */
	case meshtastic_AdminMessage_begin_edit_settings_tag:
		meshtastic_config_store_set_save_suppressed(true);
		admin_edit_open = true;
		LOG_DBG("admin: begin edit transaction");
		break;
	case meshtastic_AdminMessage_commit_edit_settings_tag:
		meshtastic_config_store_set_save_suppressed(false);
#if defined(CONFIG_MESHTASTIC_SETTINGS)
		meshtastic_settings_flush();
#endif
		admin_edit_open = false;
		LOG_DBG("admin: commit edit transaction");
		break;

	/* Reboot (Stage 6) — ACK is emitted below, before the delayed reboot. */
	case meshtastic_AdminMessage_reboot_seconds_tag:
		if (admin_req.payload_variant.reboot_seconds < 0) {
			(void)k_work_cancel_delayable(&admin_reboot_work);
			LOG_INF("admin: reboot cancelled");
		} else {
			LOG_INF("admin: reboot in %d s",
				admin_req.payload_variant.reboot_seconds);
			k_work_reschedule(&admin_reboot_work,
					  K_SECONDS(admin_req.payload_variant.reboot_seconds));
		}
		break;

	/* Phone-supplied wall clock (epoch seconds). Seeds the clock so peer
	 * last_heard / own position time can be reported as epoch. */
	case meshtastic_AdminMessage_set_time_only_tag:
		LOG_INF("admin: set_time_only epoch=%u",
			admin_req.payload_variant.set_time_only);
		meshtastic_clock_set_epoch(admin_req.payload_variant.set_time_only);
		break;

	/* Reset ops — deferred in the MVP (ack so the app doesn't hang). */
	case meshtastic_AdminMessage_factory_reset_config_tag:
	case meshtastic_AdminMessage_factory_reset_device_tag:
	case meshtastic_AdminMessage_nodedb_reset_tag:
		LOG_WRN("admin: reset op (variant %u) not implemented in MVP — acked only",
			(unsigned int)admin_req.which_payload_variant);
		break;

	default:
		LOG_WRN("admin: unhandled variant %u (consumed, not forwarded)",
			(unsigned int)admin_req.which_payload_variant);
		break;
	}

	/* The app's 30s write timeout waits on a ROUTING ACK. A getter already
	 * emitted an AdminMessage response (carrying the passkey), so suppress the
	 * redundant ACK there — matching firmware, which sends only the response —
	 * but still ACK a failed getter and every setter. */
	if (pkt->want_ack && !response_sent) {
		admin_ack_write(pkt, ack_err);
	}

	/* Fire a deferred reboot once the write that needs it is not inside an open
	 * edit transaction: immediately for a lone set, or after commit clears
	 * admin_edit_open. ACK above goes out first; the reboot is 7s delayed. */
	if (reboot_pending && !admin_edit_open) {
		admin_schedule_reboot();
	}

	k_mutex_unlock(&admin_lock);
	return true;
}

void meshtastic_admin_reset(void)
{
	/* Lock-free by design: clears an open edit transaction on (re)connect or
	 * disconnect so a dropped batch can't leave saves suppressed. Benign
	 * races only (a concurrent commit clears the same state). */
	if (admin_edit_open) {
		admin_edit_open = false;
		meshtastic_config_store_set_save_suppressed(false);
	}
	/* Don't let a dropped edit batch strand a reboot that never got committed. */
	reboot_pending = false;
}
