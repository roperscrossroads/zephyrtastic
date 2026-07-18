/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic sample application.
 *
 * Demonstrates basic usage of the Meshtastic subsystem:
 *  - Initialise the stack with the default LongFast channel.
 *  - Print every received text message to the console.
 *  - Broadcast "Hello from Zephyr!" every 30 seconds.
 *
 * The LoRa device is obtained from the DT alias "lora0".
 * The local node ID is derived from HWINFO by default; see
 * CONFIG_MESHTASTIC_NODE_ID_SOURCE.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <zephyr/meshtastic/meshtastic.h>

LOG_MODULE_REGISTER(meshtastic_sample, LOG_LEVEL_INF);

/* Obtain LoRa device from the "lora0" devicetree alias. */
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static const char *packet_channel_name(const struct meshtastic_packet *packet)
{
	if (packet->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
		return meshtastic_get_channel_name(packet->channel_index);
	}

	return "unknown";
}

static void on_event(const struct meshtastic_event *event, void *user_data)
{
	const struct meshtastic_packet *packet;

	ARG_UNUSED(user_data);

	if (event == NULL) {
		return;
	}

	switch (event->type) {
	case MESHTASTIC_EVENT_TEXT_MESSAGE:
		packet = event->packet;
		if (packet == NULL || packet->payload == NULL) {
			break;
		}

		LOG_INF("MSG from 0x%08x on \"%s\": %.*s  (RSSI %d dBm, SNR %d)", packet->from,
			packet_channel_name(packet), (int)packet->payload_len,
			(const char *)packet->payload, (int)packet->rssi, (int)packet->snr);
		break;
	default:
		break;
	}
}

int main(void)
{
	struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = 0,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
#if defined(CONFIG_MESHTASTIC_TEST_LORA_SEND_PARAMS)
		.frequency = 865100000U,
		.tx_power = CONFIG_MESHTASTIC_TX_POWER,
#else
		.frequency = MESHTASTIC_FREQ_US,
		/* hop_limit and tx_power: 0 → use Kconfig defaults */
#endif
	};
	int ret;

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa device not ready");
		return -ENODEV;
	}

	ret = meshtastic_init(&cfg);
	if (ret < 0) {
		LOG_ERR("meshtastic_init failed (%d)", ret);
		return ret;
	}

	meshtastic_set_event_cb(on_event, NULL);

	LOG_INF("Meshtastic sample started, node ID 0x%08x",
		meshtastic_get_node_id());

	/*
	 * NodeInfo (name, MAC address, hardware model) is announced to the
	 * mesh automatically by the subsystem; see CONFIG_MESHTASTIC_NODEINFO.
	 */

	/* Automatic periodic text broadcast disabled: the node still
	 * announces NodeInfo and relays mesh traffic; it just no longer spams
	 * "Hello from ...". Park main so the app thread stays alive. */
	(void)ret;
	k_sleep(K_FOREVER);

	return 0;
}
