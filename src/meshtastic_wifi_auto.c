/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * Boot-time WiFi auto-connect. Issues NET_REQUEST_WIFI_CONNECT_STORED so the
 * station comes up from credentials persisted in NVS, with no shell interaction
 * (needed once the console UART is repurposed and only telnet remains). Runs in
 * its own thread because connect-stored can block while walking the stored list.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>

LOG_MODULE_REGISTER(meshtastic_wifi_auto, CONFIG_MESHTASTIC_LOG_LEVEL);

#define AUTO_DELAY   K_SECONDS(CONFIG_MESHTASTIC_WIFI_AUTOCONNECT_DELAY_SEC)
#define AUTO_RETRIES CONFIG_MESHTASTIC_WIFI_AUTOCONNECT_RETRIES

static void wifi_auto_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct net_if *iface = NULL;

	/* Let the WiFi driver bring up the STA interface first. */
	k_sleep(AUTO_DELAY);

	for (int i = 0; i < 10 && iface == NULL; i++) {
		iface = net_if_get_wifi_sta();
		if (iface == NULL) {
			k_sleep(K_SECONDS(1));
		}
	}

	if (iface == NULL) {
		LOG_WRN("No WiFi STA interface; auto-connect disabled");
		return;
	}

	for (int attempt = 1; attempt <= AUTO_RETRIES; attempt++) {
		int rc = net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0);

		if (rc == 0) {
			LOG_INF("WiFi auto-connect from stored credentials (attempt %d)",
				attempt);
			return;
		}

		LOG_WRN("connect-stored attempt %d/%d failed (%d)",
			attempt, AUTO_RETRIES, rc);
		k_sleep(K_SECONDS(5));
	}

	LOG_WRN("WiFi auto-connect gave up; provision via `wifi cred add`");
}

K_THREAD_DEFINE(meshtastic_wifi_auto_tid, 4096, wifi_auto_thread,
		NULL, NULL, NULL, 12, 0, 0);
