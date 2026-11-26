/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * The core's shared state and its plain accessors: the one `mt` and `mt_ws`,
 * the application's receive and event callbacks, status, and the runtime
 * values other modules read (node id, frequency, hop limit, channel names,
 * which phone transport boots).
 *
 * Split out of meshtastic.c (agents-ls39.3.2). Who may write which field of
 * `mt` is not settled here; that is agents-ls39.3.3.
 */

#include <zephyr/kernel.h>

#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

const uint8_t meshtastic_default_psk[16] = {
	0xd4U, 0xf1U, 0xbbU, 0x3aU, 0x20U, 0x29U, 0x07U, 0x59U,
	0xf0U, 0xbcU, 0xffU, 0xabU, 0xcfU, 0x4eU, 0x69U, 0x01U,
};

struct meshtastic_context mt;
struct meshtastic_workspace mt_ws;

void meshtastic_set_ble_connected(bool connected)
{
	mt.status.ble_connected = connected;
}

void meshtastic_emit_event(enum meshtastic_event_type type, int err,
			   const struct meshtastic_packet *packet)
{
	struct meshtastic_event event = {
		.type = type,
		.err = err,
		.packet = packet,
	};

	if (mt.event_cb != NULL) {
		mt.event_cb(&event, mt.event_user_data);
	}
}

bool meshtastic_transport_prefer_wifi(void)
{
#if defined(CONFIG_MESHTASTIC_BLE) && defined(CONFIG_MESHTASTIC_TCP)
	/*
	 * Unified image: both transports are compiled in, so the persisted
	 * network config picks which one boots. Mirrors upstream, where the phone
	 * app's "WiFi enabled" toggle drives NetworkConfig.wifi_enabled.
	 */
	meshtastic_Config config;

	if (meshtastic_config_store_get_config(meshtastic_Config_network_tag, &config) == 0 &&
	    config.which_payload_variant == meshtastic_Config_network_tag) {
		return config.payload_variant.network.wifi_enabled;
	}

	return false; /* fresh device / no network config persisted: default to BLE */
#elif defined(CONFIG_MESHTASTIC_TCP)
	return true;  /* net-only image: WiFi/net is the only phone transport */
#else
	return false; /* BLE-only image (or no phone transport built) */
#endif
}

void meshtastic_set_recv_cb(meshtastic_recv_cb_t cb)
{
	mt.recv_cb = cb;
}

void meshtastic_set_event_cb(meshtastic_event_cb_t cb, void *user_data)
{
	mt.event_cb = cb;
	mt.event_user_data = user_data;
}

int meshtastic_get_status(struct meshtastic_status *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	*status = mt.status;
	status->initialized = mt.initialized;
	status->node_id = mt.node_id;

	return 0;
}

uint32_t meshtastic_get_node_id(void)
{
	return mt.node_id;
}

uint32_t meshtastic_runtime_frequency(void)
{
	return mt.frequency;
}

const char *meshtastic_runtime_channel_name(void)
{
	return meshtastic_channels_primary_name();
}

const char *meshtastic_get_channel_name(uint8_t index)
{
	return meshtastic_channels_get_name(index);
}

uint8_t meshtastic_runtime_hop_limit(void)
{
	return mt.hop_limit;
}
