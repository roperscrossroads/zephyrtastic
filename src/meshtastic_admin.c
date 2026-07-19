/* SPDX-License-Identifier: GPL-3.0
 *
 * Local AdminMessage handling — the config-write engine. Intercepts ADMIN_APP
 * packets the connected app addresses to this node, dispatches get_/set_ config,
 * module config, channel and owner operations into the config store, and
 * synthesizes the ROUTING ACK / AdminMessage responses the app waits on.
 *
 * Two transports share one dispatcher (admin_dispatch):
 *  - LOCAL (from == self), the directly-connected app: trusted, no passkey
 *    (the reference never gates local admin), replies fan out via the PhoneAPI.
 *    Refused outright when the node is is_managed.
 *  - REMOTE (from != self) over the mesh: the sender is authorized (PKC key in
 *    SecurityConfig.admin_key, or the legacy admin channel) and mutating ops
 *    must carry a valid session passkey; replies (and ACK/error) are sent back
 *    over the mesh, PKC-encrypted to the admin when we hold its key.
 */

#include "meshtastic_admin.h"

#include <errno.h>
#include <string.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#if defined(CONFIG_POWEROFF)
#include <zephyr/sys/poweroff.h>
#endif

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_admin_session.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#if defined(CONFIG_MESHTASTIC_POSITION)
#include "meshtastic_position.h"
#endif
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_router.h"
#include "meshtastic_settings.h"

#include "meshtastic/admin.pb.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_WIFI)
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#endif

/* Cross-module connection-state accessors (get_device_connection_status). */
#if defined(CONFIG_MESHTASTIC_MQTT)
bool meshtastic_mqtt_is_connected(void);
#endif
#if defined(CONFIG_MESHTASTIC_BLE)
bool meshtastic_ble_is_connected(void);
#endif

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

/* Per-dispatch reply context, valid only while admin_lock is held. Set once by
 * admin_dispatch() so the emit helpers know where and how to reply — the local
 * path fans out via the PhoneAPI; the remote path transmits back over the mesh
 * to the requester. Serialized by admin_lock (single dispatch at a time). */
static struct admin_ctx {
	uint32_t from;			   /* requester node id (reply destination) */
	uint32_t id;			   /* request packet id (reply request_id) */
	bool want_ack;			   /* requester asked for a ROUTING ACK */
	uint8_t channel_index;		   /* channel the request arrived on */
	bool remote;			   /* true: reply over the mesh, not PhoneAPI */
	const struct meshtastic_packet *rx; /* remote only: the received packet */
} admin_cur;

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

/* Reset variant of the reboot work: deliberately does NOT flush. A factory reset
 * has already deleted the persisted config keys; flushing here would re-export the
 * still-populated in-RAM store and resurrect everything just wiped. The next boot
 * finds an empty store and re-seeds defaults. */
static void admin_reset_reboot_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(admin_reset_reboot_work, admin_reset_reboot_work_fn);

static void admin_reset_reboot_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_WRN("admin: rebooting now (post-reset, no flush)");
	sys_reboot(SYS_REBOOT_COLD);
}

#if defined(CONFIG_POWEROFF)
/* Shutdown work: flush like the reboot path, then power off. On the ESP32-S3
 * sys_poweroff() enters deep sleep (esp_deep_sleep_start), so the device wakes on
 * the next reset — as advertised to the app via canShutdown. */
static void admin_shutdown_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	meshtastic_settings_flush();
#endif
	LOG_WRN("admin: powering off now");
	sys_poweroff();
}
static K_WORK_DELAYABLE_DEFINE(admin_shutdown_work, admin_shutdown_work_fn);
#endif /* CONFIG_POWEROFF */

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
	/* Issue the live session key (rotated/generated as needed). A remote
	 * client must echo it back with any mutating op; the local from==self
	 * path never validates it. See meshtastic_admin_session.[ch]. */
	BUILD_ASSERT(sizeof(resp->session_passkey.bytes) >= MESHTASTIC_ADMIN_SESSION_KEY_LEN,
		     "AdminMessage.session_passkey too small for the session key");
	resp->session_passkey.size = MESHTASTIC_ADMIN_SESSION_KEY_LEN;
	meshtastic_admin_session_current(resp->session_passkey.bytes);
}

/* True when this node is administratively managed — config may be changed only
 * by an authorized remote admin, so local (directly-connected app) admin is
 * refused. Mirrors AdminModule's `mp.from == 0 && is_managed` guard. */
static bool admin_is_managed(void)
{
	meshtastic_Config cfg;

	if (meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg) != 0) {
		return false;
	}
	return cfg.which_payload_variant == meshtastic_Config_security_tag &&
	       cfg.payload_variant.security.is_managed;
}

/* Emit an ADMIN_APP response carrying resp, correlated to the request via
 * request_id + the live session passkey. resp is the module scratch (caller
 * holds admin_lock). Routes by admin_cur.remote: local fans out via the
 * PhoneAPI; remote transmits back to the requester over the mesh (a unicast DM,
 * so meshtastic_build_wire_packet PKC-encrypts it when we hold the admin's key,
 * fire-and-forget with K_NO_WAIT so the RX thread never blocks). */
static void admin_emit_reply(meshtastic_AdminMessage *resp)
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
	pkt.to = admin_cur.from;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.request_id = admin_cur.id;
	pkt.payload = buf;
	pkt.payload_len = stream.bytes_written;

	if (admin_cur.remote) {
		pkt.channel_index = admin_cur.channel_index;
		(void)meshtastic_send_packet(&pkt, K_NO_WAIT);
	} else {
		pkt.channel_index = 0U;
		meshtastic_phoneapi_on_packet(&pkt);
	}
}

/* Emit a ROUTING_APP ACK/NAK the client matches to its want_ack request_id.
 * Routes like admin_emit_reply: local via the PhoneAPI, remote back over the
 * mesh via the router (K_NO_WAIT, primary channel — matching NAK behaviour). */
static void admin_ack_write(meshtastic_Routing_Error err)
{
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	uint8_t buf[16];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};

	if (admin_cur.remote) {
		meshtastic_routing_send_error(admin_cur.rx, err);
		return;
	}

	routing.which_variant = meshtastic_Routing_error_reason_tag;
	routing.error_reason = err;

	if (!pb_encode(&stream, meshtastic_Routing_fields, &routing)) {
		LOG_ERR("admin: routing ack encode failed");
		return;
	}

	pkt.portnum = meshtastic_PortNum_ROUTING_APP;
	pkt.from = meshtastic_get_node_id();
	pkt.to = admin_cur.from;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.request_id = admin_cur.id;
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
static bool handle_get_device_metadata(void)
{
	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant =
		meshtastic_AdminMessage_get_device_metadata_response_tag;
	meshtastic_fill_device_metadata(&admin_resp.payload_variant.get_device_metadata_response);
	admin_emit_reply(&admin_resp);
	return true;
}

static bool handle_get_config(uint32_t config_type)
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
	admin_emit_reply(&admin_resp);
	return true;
}

static bool handle_get_module_config(uint32_t module_type)
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
	admin_emit_reply(&admin_resp);
	return true;
}

static bool handle_get_channel(uint32_t one_based)
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
	admin_emit_reply(&admin_resp);
	return true;
}

static bool handle_get_owner(void)
{
	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant = meshtastic_AdminMessage_get_owner_response_tag;
	meshtastic_fill_user(&admin_resp.payload_variant.get_owner_response);
	admin_emit_reply(&admin_resp);
	return true;
}

/* Fill the app-facing device connection status from whatever transports this
 * build actually has. Only WiFi/BT are meaningfully live on this port; each arm
 * is compiled out when its transport isn't. */
static void admin_fill_connection_status(meshtastic_DeviceConnectionStatus *st)
{
#if defined(CONFIG_WIFI)
	struct net_if *iface = net_if_get_default();
	struct wifi_iface_status wstat = {0};
	struct net_in_addr *ip;

	st->has_wifi = true;
	if (iface != NULL) {
		if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &wstat, sizeof(wstat)) == 0 &&
		    wstat.state >= WIFI_STATE_ASSOCIATED) {
			size_t n = MIN((size_t)wstat.ssid_len, sizeof(st->wifi.ssid) - 1U);

			memcpy(st->wifi.ssid, wstat.ssid, n);
			st->wifi.ssid[n] = '\0';
			st->wifi.rssi = wstat.rssi;
		}
		/* Report connected + IP off the actual assigned address, not just link
		 * state — an associated-but-no-DHCP iface isn't usable yet. */
		ip = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
		if (ip != NULL) {
			st->wifi.has_status = true;
			st->wifi.status.is_connected = true;
			st->wifi.status.ip_address = ip->s_addr;
#if defined(CONFIG_MESHTASTIC_MQTT)
			st->wifi.status.is_mqtt_connected = meshtastic_mqtt_is_connected();
#endif
		}
	}
#endif /* CONFIG_WIFI */

#if defined(CONFIG_MESHTASTIC_BLE)
	st->has_bluetooth = true;
	st->bluetooth.is_connected = meshtastic_ble_is_connected();
#endif
}

static bool handle_get_device_connection_status(void)
{
	admin_resp = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	admin_resp.which_payload_variant =
		meshtastic_AdminMessage_get_device_connection_status_response_tag;
	admin_fill_connection_status(
		&admin_resp.payload_variant.get_device_connection_status_response);
	admin_emit_reply(&admin_resp);
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
/* Remote authorization (mesh path)                                           */
/* ------------------------------------------------------------------------- */

/* X25519 public-key length; SecurityConfig.admin_key entries and NodeDB peer
 * keys are both this size. */
#define ADMIN_PUBKEY_LEN 32U

/* Legacy insecure admin channel — a plaintext admin packet is accepted only if
 * it arrived on a channel literally named "admin". Mirrors Channels::adminChannel. */
#define ADMIN_LEGACY_CHANNEL_NAME "admin"

/* True if the sender's stored public key matches a configured admin key
 * (SecurityConfig.admin_key[0..2]) — the modern, secure remote-admin gate.
 * Uses the hot→warm key lookup (not the hot-only nodedb_get): an admin whose
 * hot record was evicted, its key surviving only in the warm ring, must not be
 * locked out of remote admin (A-1). */
static bool admin_key_matches_sender(uint32_t from)
{
	uint8_t sender_key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
	meshtastic_Config cfg;

	BUILD_ASSERT(sizeof(sender_key) == ADMIN_PUBKEY_LEN, "key length mismatch");

	if (meshtastic_nodedb_copy_pubkey(from, sender_key) != 0) {
		return false;
	}
	if (meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg) != 0 ||
	    cfg.which_payload_variant != meshtastic_Config_security_tag) {
		return false;
	}
	for (pb_size_t i = 0; i < cfg.payload_variant.security.admin_key_count; i++) {
		const meshtastic_Config_SecurityConfig_admin_key_t *k =
			&cfg.payload_variant.security.admin_key[i];

		if (k->size == ADMIN_PUBKEY_LEN &&
		    memcmp(k->bytes, sender_key, ADMIN_PUBKEY_LEN) == 0) {
			return true;
		}
	}
	return false;
}

/* True if a plaintext admin packet is allowed via the legacy admin channel:
 * admin_channel_enabled AND the arrival channel is named "admin". */
static bool admin_channel_authorized(uint8_t channel_index)
{
	meshtastic_Config cfg;
	const char *name;

	if (meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg) != 0 ||
	    cfg.which_payload_variant != meshtastic_Config_security_tag ||
	    !cfg.payload_variant.security.admin_channel_enabled) {
		return false;
	}
	name = meshtastic_channels_get_name(channel_index);
	return name != NULL && strcmp(name, ADMIN_LEGACY_CHANNEL_NAME) == 0;
}

/* Decide whether a remote admin packet is authorized. On refusal, sets *err to
 * the ROUTING error to return. Mirrors AdminModule: PKC admin_key first, then
 * the legacy admin channel, else NOT_AUTHORIZED. */
static bool admin_remote_authorized(const struct meshtastic_packet *pkt,
				    meshtastic_Routing_Error *err)
{
	if (pkt->pki_encrypted) {
		if (admin_key_matches_sender(pkt->from)) {
			return true;
		}
		*err = meshtastic_Routing_Error_ADMIN_PUBLIC_KEY_UNAUTHORIZED;
		return false;
	}
	/* The legacy channel gate authorizes by channel *name* with no identity,
	 * so it is only ever safe for a packet that actually reached us over the
	 * mesh. A packet that traversed MQTT has no such provenance: the broker is
	 * not a mesh peer, an injected packet's channel is forced to primary, and
	 * on a plaintext or bridged broker any internet peer can produce one. The
	 * downlink path already rejects ADMIN_APP outright; this refuses the
	 * identity-less gate for anything that gets here claiming via_mqtt,
	 * including a frame that arrived over RF with the wire flag set. PKC admin
	 * is unaffected — its authorization is a key match, which MQTT cannot
	 * forge. */
	if (pkt->via_mqtt) {
		LOG_WRN("admin: refusing legacy-channel authorization for an MQTT-borne "
			"packet from 0x%08x",
			pkt->from);
		*err = meshtastic_Routing_Error_NOT_AUTHORIZED;
		return false;
	}

	if (admin_channel_authorized(pkt->channel_index)) {
		return true;
	}
	*err = meshtastic_Routing_Error_NOT_AUTHORIZED;
	return false;
}

/* Mutating admin ops from a remote node require a valid session passkey;
 * get_*_request reads are exempt (their response issues a fresh key). Mirrors
 * AdminModule messageIsRequest. Responses fall through to "needs" but are
 * no-ops in the dispatcher default arm. */
static bool admin_variant_needs_passkey(pb_size_t variant)
{
	switch (variant) {
	case meshtastic_AdminMessage_get_config_request_tag:
	case meshtastic_AdminMessage_get_module_config_request_tag:
	case meshtastic_AdminMessage_get_channel_request_tag:
	case meshtastic_AdminMessage_get_owner_request_tag:
	case meshtastic_AdminMessage_get_device_metadata_request_tag:
	case meshtastic_AdminMessage_get_device_connection_status_request_tag:
	case meshtastic_AdminMessage_get_ringtone_request_tag:
	case meshtastic_AdminMessage_get_canned_message_module_messages_request_tag:
	case meshtastic_AdminMessage_get_ui_config_request_tag:
	case meshtastic_AdminMessage_get_node_remote_hardware_pins_request_tag:
		return false;
	default:
		return true;
	}
}

/* ------------------------------------------------------------------------- */
/* Dispatcher                                                                 */
/* ------------------------------------------------------------------------- */

/* Shared decode + gate + dispatch + reply. @p ctx (copied into admin_cur under
 * the lock) tells the emit helpers where to reply. Local and remote both flow
 * through here; the differences are the is_managed gate (local) and the passkey
 * gate (remote). */
static void admin_dispatch(struct admin_ctx ctx, const uint8_t *payload, size_t payload_len)
{
	pb_istream_t stream;
	meshtastic_Routing_Error ack_err = meshtastic_Routing_Error_NONE;
	bool response_sent = false;
	int ret;

	k_mutex_lock(&admin_lock, K_FOREVER);
	admin_cur = ctx;

	admin_req = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	stream = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&stream, meshtastic_AdminMessage_fields, &admin_req)) {
		LOG_WRN("admin: AdminMessage decode failed: %s", PB_GET_ERROR(&stream));
		k_mutex_unlock(&admin_lock);
		return;
	}

	LOG_DBG("admin: variant %u from 0x%08x id=0x%08x remote=%d",
		(unsigned int)admin_req.which_payload_variant, admin_cur.from, admin_cur.id,
		(int)admin_cur.remote);

	/* Managed node: the directly-connected app may not administer it — only an
	 * authorized remote admin can. Consume without applying; the app already
	 * knows the node is managed from SecurityConfig and greys its settings out.
	 * Config still reaches the app through the normal want_config stream. */
	if (!admin_cur.remote && admin_is_managed()) {
		LOG_INF("admin: ignoring local admin variant %u — node is_managed",
			(unsigned int)admin_req.which_payload_variant);
		k_mutex_unlock(&admin_lock);
		return;
	}

	/* Remote mutating ops must carry a live session passkey (replay protection);
	 * the sender was already authorized (PKC/admin channel) by handle_remote. */
	if (admin_cur.remote &&
	    admin_variant_needs_passkey(admin_req.which_payload_variant) &&
	    !meshtastic_admin_session_valid(admin_req.session_passkey.bytes,
					    admin_req.session_passkey.size)) {
		LOG_WRN("admin: remote variant %u rejected — bad/absent session passkey",
			(unsigned int)admin_req.which_payload_variant);
		admin_ack_write(meshtastic_Routing_Error_ADMIN_BAD_SESSION_KEY);
		k_mutex_unlock(&admin_lock);
		return;
	}

	switch (admin_req.which_payload_variant) {
	/* Getters — a response already carries the passkey, so the redundant
	 * ROUTING ACK is suppressed below when response_sent (firmware behavior). */
	case meshtastic_AdminMessage_get_device_metadata_request_tag:
		response_sent = handle_get_device_metadata();
		break;
	case meshtastic_AdminMessage_get_config_request_tag:
		response_sent = handle_get_config(admin_req.payload_variant.get_config_request);
		break;
	case meshtastic_AdminMessage_get_module_config_request_tag:
		response_sent = handle_get_module_config(
			admin_req.payload_variant.get_module_config_request);
		break;
	case meshtastic_AdminMessage_get_channel_request_tag:
		response_sent = handle_get_channel(admin_req.payload_variant.get_channel_request);
		break;
	case meshtastic_AdminMessage_get_owner_request_tag:
		response_sent = handle_get_owner();
		break;
	case meshtastic_AdminMessage_get_device_connection_status_request_tag:
		response_sent = handle_get_device_connection_status();
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

	/* Reset ops. The ACK below goes out first; the reset-reboot fires 7 s later
	 * on a no-flush work item so the wiped store isn't re-persisted. Mirrors
	 * AdminModule: factoryReset() keeps the private key; factoryReset(true) erases
	 * everything; resetNodes(keepFavorites) clears only the peer database. */
	case meshtastic_AdminMessage_factory_reset_config_tag:
		LOG_INF("admin: factory_reset_config (preserving security identity)");
		meshtastic_config_store_set_save_suppressed(true);
#if defined(CONFIG_MESHTASTIC_SETTINGS)
		(void)meshtastic_settings_wipe(true); /* keep config/security */
#endif
		k_work_reschedule(&admin_reset_reboot_work, K_SECONDS(ADMIN_REBOOT_SECONDS));
		break;

	case meshtastic_AdminMessage_factory_reset_device_tag:
		LOG_INF("admin: factory_reset_device (full wipe)");
		meshtastic_config_store_set_save_suppressed(true);
#if defined(CONFIG_MESHTASTIC_SETTINGS)
		(void)meshtastic_settings_wipe(false); /* wipe everything incl. identity */
#endif
		meshtastic_nodedb_reset(false);
		k_work_reschedule(&admin_reset_reboot_work, K_SECONDS(ADMIN_REBOOT_SECONDS));
		break;

	case meshtastic_AdminMessage_nodedb_reset_tag: {
		/* CLIENT_BASE / ROUTER / ROUTER_LATE preserve hop count when relaying
		 * via a favorited node, so keep their favorites on reset — otherwise
		 * honor the request's own flag (matches AdminModule.cpp). */
		meshtastic_Config cfg;
		bool role_pref = false;
		bool keep_favorites;

		if (meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg) == 0) {
			meshtastic_Config_DeviceConfig_Role role = cfg.payload_variant.device.role;

			role_pref = (role == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE) ||
				    (role == meshtastic_Config_DeviceConfig_Role_ROUTER) ||
				    (role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
		}
		keep_favorites = role_pref ? true : admin_req.payload_variant.nodedb_reset;

		LOG_INF("admin: nodedb_reset (keep_favorites=%d)", (int)keep_favorites);
		meshtastic_nodedb_reset(keep_favorites);
		/* A node-db reset leaves config untouched, so a normal (flushing)
		 * reboot is fine — the peer store was already persisted synchronously. */
		k_work_reschedule(&admin_reboot_work, K_SECONDS(ADMIN_REBOOT_SECONDS));
		break;
	}

	/* Shutdown: a negative value cancels a pending shutdown, else schedule one
	 * (mirrors AdminModule shutdownAtMsec). On the ESP32-S3 this deep-sleeps; the
	 * device wakes on reset. Only wired when the platform has a poweroff path. */
	case meshtastic_AdminMessage_shutdown_seconds_tag: {
		int32_t s = admin_req.payload_variant.shutdown_seconds;

#if defined(CONFIG_POWEROFF)
		if (s < 0) {
			(void)k_work_cancel_delayable(&admin_shutdown_work);
			LOG_INF("admin: shutdown cancelled");
		} else {
			LOG_INF("admin: shutdown in %d s", s);
			k_work_reschedule(&admin_shutdown_work, K_SECONDS(s));
		}
#else
		LOG_WRN("admin: shutdown requested (%d s) but no poweroff path — ignored", s);
#endif
		break;
	}

	/* DFU: the ESP32-S3 ROM download mode needs a GPIO0 strap at hardware reset,
	 * so there is no software path to enter it (unlike nRF52/RP2040/STM32). The
	 * port's real update route is OTA via mcumgr. Log and drop, don't fake it. */
	case meshtastic_AdminMessage_enter_dfu_mode_request_tag:
		LOG_WRN("admin: enter_dfu_mode unsupported on this platform (use OTA) — ignored");
		break;

	default:
		LOG_WRN("admin: unhandled variant %u (consumed, not forwarded)",
			(unsigned int)admin_req.which_payload_variant);
		break;
	}

	/* The client's write timeout waits on a ROUTING ACK. A getter already
	 * emitted an AdminMessage response (carrying the passkey), so suppress the
	 * redundant ACK there — matching firmware, which sends only the response —
	 * but still ACK a failed getter and every setter. */
	if (admin_cur.want_ack && !response_sent) {
		admin_ack_write(ack_err);
	}

	/* Fire a deferred reboot once the write that needs it is not inside an open
	 * edit transaction: immediately for a lone set, or after commit clears
	 * admin_edit_open. ACK above goes out first; the reboot is 7s delayed. */
	if (reboot_pending && !admin_edit_open) {
		admin_schedule_reboot();
	}

	k_mutex_unlock(&admin_lock);
}

bool meshtastic_admin_handle_local(const meshtastic_MeshPacket *pkt)
{
	if (pkt == NULL) {
		return false;
	}

	admin_dispatch((struct admin_ctx){
			       .from = pkt->from,
			       .id = pkt->id,
			       .want_ack = pkt->want_ack,
			       .channel_index = 0U,
			       .remote = false,
			       .rx = NULL,
		       },
		       pkt->decoded.payload.bytes, pkt->decoded.payload.size);
	return true; /* consumed — never forward admin onto the mesh */
}

void meshtastic_admin_handle_remote(const struct meshtastic_packet *pkt)
{
	meshtastic_Routing_Error auth_err = meshtastic_Routing_Error_NOT_AUTHORIZED;

	if (pkt == NULL || (pkt->payload == NULL && pkt->payload_len != 0U)) {
		return;
	}

	/* Packet-level authorization before touching the AdminMessage contents. On
	 * refusal, NAK the requester with the specific reason and drop it. */
	if (!admin_remote_authorized(pkt, &auth_err)) {
		LOG_WRN("admin: remote admin from 0x%08x unauthorized (err=%d)", pkt->from,
			(int)auth_err);
		meshtastic_routing_send_error(pkt, auth_err);
		return;
	}

	admin_dispatch((struct admin_ctx){
			       .from = pkt->from,
			       .id = pkt->id,
			       .want_ack = pkt->want_ack,
			       .channel_index = pkt->channel_index,
			       .remote = true,
			       .rx = pkt,
		       },
		       pkt->payload, pkt->payload_len);
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
