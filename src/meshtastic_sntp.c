/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * SNTP wall-clock sync over IP. This board has no GNSS and no battery-backed
 * RTC, so meshtastic_clock is otherwise only seeded by the phone's
 * set_time_only admin message — which never arrives when driving via the CLI,
 * leaving timestamps at the epoch floor. On a WiFi build we query an SNTP
 * server whenever the interface gets an IPv4 lease (boot or reconnect) and
 * re-sync periodically to correct crystal drift, seeding
 * meshtastic_clock_set_epoch() so NodeInfo.last_heard, Position.time and
 * message timestamps report real Unix epoch seconds.
 *
 * The query runs on a dedicated workqueue: sntp_simple() blocks on DNS + UDP
 * for up to the configured timeout, and the system workqueue must stay free to
 * complete SX126x DIO (LoRa RX/TX) work.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/sntp.h>

#include "meshtastic_clock.h"

LOG_MODULE_REGISTER(meshtastic_sntp, CONFIG_MESHTASTIC_LOG_LEVEL);

#define SNTP_SERVER     CONFIG_MESHTASTIC_SNTP_SERVER
#define SNTP_TIMEOUT_MS CONFIG_MESHTASTIC_SNTP_TIMEOUT_MS
#define RETRY_DELAY     K_SECONDS(30)

static K_THREAD_STACK_DEFINE(sntp_stack, CONFIG_MESHTASTIC_SNTP_STACK_SIZE);
static struct k_work_q sntp_wq;
static struct net_mgmt_event_callback ipv4_cb;
static struct k_work sync_work;
static struct k_work_delayable resync_work;

static void sntp_do_sync(struct k_work *work)
{
	ARG_UNUSED(work);
	struct sntp_time ts;
	int rc = sntp_simple(SNTP_SERVER, SNTP_TIMEOUT_MS, &ts);

	if (rc == 0 && ts.seconds != 0U) {
		meshtastic_clock_set_epoch((uint32_t)ts.seconds);
		LOG_INF("SNTP sync ok: epoch=%u", (uint32_t)ts.seconds);
#if CONFIG_MESHTASTIC_SNTP_RESYNC_HOURS > 0
		k_work_schedule_for_queue(&sntp_wq, &resync_work,
					  K_HOURS(CONFIG_MESHTASTIC_SNTP_RESYNC_HOURS));
#endif
	} else {
		LOG_WRN("SNTP query failed (rc=%d); retry in 30s", rc);
		k_work_schedule_for_queue(&sntp_wq, &resync_work, RETRY_DELAY);
	}
}

static void sntp_resync_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_work_submit_to_queue(&sntp_wq, &sync_work);
}

static void sntp_ipv4_event(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			    struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		/* Fresh lease (boot or reconnect): (re)sync the clock. */
		k_work_submit_to_queue(&sntp_wq, &sync_work);
	}
}

static const struct k_work_queue_config sntp_wq_cfg = {
	.name = "mt_sntp",
};

static int meshtastic_sntp_init(void)
{
	k_work_queue_init(&sntp_wq);
	k_work_queue_start(&sntp_wq, sntp_stack, K_THREAD_STACK_SIZEOF(sntp_stack),
			   CONFIG_MESHTASTIC_SNTP_PRIORITY, &sntp_wq_cfg);

	k_work_init(&sync_work, sntp_do_sync);
	k_work_init_delayable(&resync_work, sntp_resync_fn);

	net_mgmt_init_event_callback(&ipv4_cb, sntp_ipv4_event, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	LOG_DBG("SNTP time sync armed (server=%s)", SNTP_SERVER);
	return 0;
}

SYS_INIT(meshtastic_sntp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
