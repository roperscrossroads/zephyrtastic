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
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <zephyr/meshtastic/meshtastic.h>

LOG_MODULE_REGISTER(meshtastic_sample, LOG_LEVEL_INF);

/* Obtain LoRa device from the "lora0" devicetree alias. */
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/*
 * Reset-cause logging (wedge/reboot investigation, 2026-07-17): hwinfo's reset-
 * cause register is readable the instant we boot, but a log emitted that early
 * is silently lost — NETLOG has no route until the WiFi interface actually has
 * an IPv4 lease (confirmed: the pre-existing "Meshtastic sample started" boot
 * line, logged after meshtastic_init() completes, has *never* once reached the
 * collector). So: read+clear the cause immediately (the value doesn't survive
 * past the next reset), but defer the LOG_INF to the first NET_EVENT_IPV4_ADDR_ADD,
 * by which point NETLOG can actually deliver it.
 */
static uint32_t boot_reset_cause;
static struct net_mgmt_event_callback ipv4_ready_cb;

static void log_boot_reset_cause(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				  struct net_if *iface)
{
	static bool logged;

	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD || logged) {
		return;
	}
	logged = true;

	LOG_INF("Reset cause 0x%08x:%s%s%s%s%s%s", boot_reset_cause,
		(boot_reset_cause & RESET_POR) ? " POR" : "",
		(boot_reset_cause & RESET_PIN) ? " PIN" : "",
		(boot_reset_cause & RESET_SOFTWARE) ? " SOFTWARE" : "",
		(boot_reset_cause & RESET_WATCHDOG) ? " WATCHDOG" : "",
		(boot_reset_cause & RESET_CPU_LOCKUP) ? " PANIC" : "",
		(boot_reset_cause & RESET_BROWNOUT) ? " BROWNOUT" : "");
}

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

	/* Read+clear now (value doesn't survive past the next reset); the
	 * matching LOG_INF fires later, once IPv4 is up — see log_boot_reset_cause(). */
	(void)hwinfo_get_reset_cause(&boot_reset_cause);
	(void)hwinfo_clear_reset_cause();
	net_mgmt_init_event_callback(&ipv4_ready_cb, log_boot_reset_cause, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_ready_cb);

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
