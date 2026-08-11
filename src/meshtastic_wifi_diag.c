/* SPDX-License-Identifier: GPL-3.0
 *
 * WiFi/network event logging + periodic link snapshot.
 *
 * Exists because of the 2026-08-11T14:45Z event: all three bench nodes
 * hardware-watchdog reset inside a 20-second window, with no preceding anomaly
 * in any log, and -- the sharp part -- NOT ONE of them left a stage-0
 * breadcrumb, which means the IRAM-resident watchdog ISR never ran, i.e.
 * interrupts were masked for the whole ~40 s window. All three are associated
 * to the SAME BSSID, so the AP is the one shared dependency that can plausibly
 * hit all three at once. See docs/KNOWN-ISSUES.md.
 *
 * The bench was close to blind here: CONFIG_WIFI_LOG_LEVEL was ERR, so the
 * ESP32 driver's own `LOG_DBG("Disconnect reason: %d")`
 * (drivers/wifi/esp32/src/esp_wifi_drv.c) never emitted, and there were no
 * net statistics of any kind. Every disconnect this bench ever had was
 * invisible.
 *
 * IMPORTANT, so nobody expects too much of this: it CANNOT capture the stall
 * itself. If interrupts really are masked for ~40 s, nothing logs during the
 * window. What it captures is the LEAD-UP -- a disconnect storm, an RSSI
 * collapse, a BSSID roam, a reconnect loop in the minutes before. If the
 * lead-up turns out clean on all three, that is itself a strong result and
 * points away from WiFi.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>

LOG_MODULE_REGISTER(mt_wifi_diag, CONFIG_MESHTASTIC_WIFI_DIAG_LOG_LEVEL);

/* Deliberately NOT in meshtastic_netlog_filter.c's restricted list: this is
 * low-volume by construction (events are rare; the snapshot is periodic and
 * one line) and it is the whole point of the exercise. */

static const char *disconn_reason_str(int reason)
{
	switch (reason) {
	case WIFI_REASON_DISCONN_SUCCESS:
		return "success/none";
	case WIFI_REASON_DISCONN_UNSPECIFIED:
		return "unspecified";
	case WIFI_REASON_DISCONN_USER_REQUEST:
		return "user request";
	case WIFI_REASON_DISCONN_AP_LEAVING:
		return "AP leaving";
	case WIFI_REASON_DISCONN_INACTIVITY:
		return "inactivity (AP aged us out)";
	default:
		return "?";
	}
}

static void log_link_snapshot(const char *prefix)
{
	struct net_if *iface = net_if_get_first_wifi();
	struct wifi_iface_status status = {0};

	if (iface == NULL) {
		return;
	}

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
		     sizeof(struct wifi_iface_status))) {
		LOG_WRN("%s: iface status query failed", prefix);
		return;
	}

	/* BSSID included deliberately: a roam between APs (or a single AP
	 * changing BSSID on restart) is exactly the kind of event that would hit
	 * every node at once, and it is invisible in RSSI alone. */
	LOG_INF("%s: state=%d rssi=%d ch=%u band=%d link=%d "
		"bssid=%02X:%02X:%02X:%02X:%02X:%02X",
		prefix, status.state, status.rssi, status.channel, status.band,
		status.link_mode, (uint8_t)status.bssid[0], (uint8_t)status.bssid[1],
		(uint8_t)status.bssid[2], (uint8_t)status.bssid[3], (uint8_t)status.bssid[4],
		(uint8_t)status.bssid[5]);
}

static void wifi_event_handler(uint64_t mgmt_event, struct net_if *iface, void *info,
			       size_t info_length, void *user_data)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(info_length);
	ARG_UNUSED(user_data);

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		const struct wifi_status *st = (const struct wifi_status *)info;

		LOG_INF("WiFi CONNECT result status=%d", st ? st->status : -1);
		log_link_snapshot("WiFi link after connect");
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		const struct wifi_status *st = (const struct wifi_status *)info;
		int reason = st ? st->disconn_reason : -1;

		/* WRN, not INF: a disconnect is the event this module exists for,
		 * and it must survive any future per-backend log throttling. */
		LOG_WRN("WiFi DISCONNECT status=%d reason=%d (%s)", st ? st->status : -1,
			reason, disconn_reason_str(reason));
		break;
	}
	case NET_EVENT_WIFI_SIGNAL_CHANGE:
		log_link_snapshot("WiFi signal change");
		break;
	default:
		break;
	}
}

NET_MGMT_REGISTER_EVENT_HANDLER(mt_wifi_diag_events,
				 NET_EVENT_WIFI_CONNECT_RESULT |
					 NET_EVENT_WIFI_DISCONNECT_RESULT |
					 NET_EVENT_WIFI_SIGNAL_CHANGE,
				 &wifi_event_handler, NULL);

#if CONFIG_MESHTASTIC_WIFI_DIAG_SNAPSHOT_SEC > 0
static void snapshot_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		k_sleep(K_SECONDS(CONFIG_MESHTASTIC_WIFI_DIAG_SNAPSHOT_SEC));
		log_link_snapshot("WiFi link");
	}
}

K_THREAD_DEFINE(mt_wifi_diag, CONFIG_MESHTASTIC_WIFI_DIAG_STACK_SIZE, snapshot_thread_fn, NULL,
		 NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
#endif
