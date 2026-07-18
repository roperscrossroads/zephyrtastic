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

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <esp_attr.h>

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
 *
 * 2026-07-18: a single last-cause reading isn't enough — several wedge cycles
 * this session never reached NETLOG at all before the *next* reset (the crash's
 * own boot attempt failed to reconnect in time). RTC_NOINIT_ATTR memory survives
 * warm resets (software/watchdog/panic — the interesting cases) even though the
 * app's own RAM gets reinitialized, so a small ring of {boot_count, reset_cause}
 * kept there accumulates across cycles and gets logged in full on ANY later boot
 * that does reach the network — even if several intermediate crashes never got
 * a chance to report themselves individually. (A genuine POR — or any reset that
 * cuts the RTC power domain, which V3's EN-based resets appear to do — clears
 * this memory too; the magic-value check below just starts a fresh history in
 * that case rather than trusting garbage.)
 */
#define RESET_HISTORY_LEN   8U
#define RESET_HISTORY_MAGIC 0x4D455348U /* "MESH" */

struct reset_history_entry {
	uint32_t boot_count;
	uint32_t reset_cause;
};

static RTC_NOINIT_ATTR uint32_t rtc_magic;
static RTC_NOINIT_ATTR uint32_t rtc_boot_count;
static RTC_NOINIT_ATTR uint32_t rtc_history_next;
static RTC_NOINIT_ATTR struct reset_history_entry rtc_history[RESET_HISTORY_LEN];

static struct net_mgmt_event_callback ipv4_ready_cb;

static void log_reset_cause_line(const char *prefix, uint32_t boot_num, uint32_t cause)
{
	LOG_INF("%sboot #%u: cause 0x%08x:%s%s%s%s%s%s", prefix, boot_num, cause,
		(cause & RESET_POR) ? " POR" : "",
		(cause & RESET_PIN) ? " PIN" : "",
		(cause & RESET_SOFTWARE) ? " SOFTWARE" : "",
		(cause & RESET_WATCHDOG) ? " WATCHDOG" : "",
		(cause & RESET_CPU_LOCKUP) ? " PANIC" : "",
		(cause & RESET_BROWNOUT) ? " BROWNOUT" : "");
}

static void log_boot_reset_cause(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				  struct net_if *iface)
{
	static bool logged;
	uint32_t i;

	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD || logged) {
		return;
	}
	logged = true;

	LOG_INF("Reset-cause history (RTC-persistent across warm resets, oldest first):");
	for (i = 0; i < RESET_HISTORY_LEN; i++) {
		uint32_t idx = (rtc_history_next + i) % RESET_HISTORY_LEN;

		if (rtc_history[idx].boot_count == 0U) {
			continue; /* slot never used yet */
		}
		log_reset_cause_line("  ", rtc_history[idx].boot_count, rtc_history[idx].reset_cause);
	}
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
	uint32_t cause = 0;

	/* Read+clear now (the hwinfo register doesn't survive past the next reset);
	 * append to the RTC-persistent ring immediately too, so it's captured even if
	 * this boot never reaches NETLOG. The matching LOG_INF of the full history
	 * fires later, once IPv4 is up — see log_boot_reset_cause(). */
	(void)hwinfo_get_reset_cause(&cause);
	(void)hwinfo_clear_reset_cause();

	if (rtc_magic != RESET_HISTORY_MAGIC) {
		/* Uninitialized RTC memory: first boot ever, or the RTC power domain
		 * itself got cut (genuine POR, or an EN-based reset that cuts it too
		 * on this board) — start a fresh history rather than trust garbage. */
		rtc_magic = RESET_HISTORY_MAGIC;
		rtc_boot_count = 0;
		rtc_history_next = 0;
		memset(rtc_history, 0, sizeof(rtc_history));
	}
	rtc_boot_count++;
	rtc_history[rtc_history_next].boot_count = rtc_boot_count;
	rtc_history[rtc_history_next].reset_cause = cause;
	rtc_history_next = (rtc_history_next + 1U) % RESET_HISTORY_LEN;

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
