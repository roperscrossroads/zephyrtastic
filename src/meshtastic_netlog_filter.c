/* SPDX-License-Identifier: GPL-3.0
 *
 * Per-backend log filtering: restricts a curated list of noisy modules to
 * WRN-or-worse on log_backend_net specifically, leaving every other backend
 * (the shell/console) at whatever each module is compiled at. See
 * Kconfig.netlog_filter for the rationale and docs/KNOWN-ISSUES.md for the
 * incident history this exists to get ahead of.
 *
 * Applied from a NET_EVENT_L4_CONNECTED handler, not at early boot -- an
 * earlier version called this from meshtastic_init() directly and it
 * silently had no effect. Root cause: CONFIG_LOG_PROCESS_THREAD (the normal
 * deferred-logging config) makes the log core's own SYS_INIT
 * (POST_KERNEL/CONFIG_LOG_CORE_INIT_PRIORITY, enable_logger() in
 * zephyr/subsys/logging/log_core.c) only start the logging thread -- the
 * thread's FIRST tick, delayed by CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS,
 * is what actually calls z_log_init() and assigns every backend's real
 * cb->id. Before that tick runs, every backend's id still reads its
 * LOG_BACKEND_DEFINE default (0), so log_filter_set() -- which resolves the
 * backend id internally via log_backend_id_get() at call time -- silently
 * wrote into whatever backend happens to hold slot 0, not necessarily
 * log_backend_net; the real z_log_init() pass, whenever it ran, then reset
 * that slot to its normal default, erasing the change. Confirmed via a raw
 * (non-log-routed) printk trace live on rzr3, 2026-08-09.
 *
 * log_backend_net itself only ever starts sending on NET_EVENT_L4_CONNECTED
 * (log_backend_net_start(), see zephyr/subsys/logging/backends/
 * log_backend_net.c) -- by definition well after the logging thread's first
 * tick (WiFi association + DHCP takes far longer than that startup delay),
 * so hooking the same event both guarantees backend ids are real by the
 * time this runs AND means the filter is in place before the backend could
 * possibly send anything anyway. Mirrors log_backend_net.c's own
 * l4_event_handler / NET_MGMT_REGISTER_EVENT_HANDLER pattern exactly.
 *
 * Deliberately excluded from the restricted list -- these are exactly what
 * the bench's passive netlog tooling (docs/monitor_bench.py, docs/mlog.py)
 * reads:
 *   meshtastic          -- heartbeat, TX/RX summary, boot banner
 *   meshtastic_sample    -- boot-cause history + crash breadcrumbs (one-shot
 *                            per boot, not a volume concern)
 *   meshtastic_sntp       -- once per sync
 *   thread_analyzer       -- the ~5 min dump the CPU-usage monitor parses
 *
 * powermon (meshtastic_powermon.c -- registered as the short name "powermon",
 * see that file) is a real judgment call, included in the restricted list
 * below: it fires on every RX/TX power-state transition (real volume on a
 * busy mesh) but was also the main "is this node radio-active" signal
 * relied on all session. Move it to the keep list above if that visibility
 * turns out to matter more than the bandwidth.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend_net.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>

LOG_MODULE_REGISTER(mt_netlog_filter, LOG_LEVEL_INF);

static const char *const restricted_modules[] = {
	/* LoRa driver family -- radio-internals chatter, low value remote,
	 * real volume risk (this exact class of module caused the resolved
	 * net_pkt-exhaustion bug at DEBUG level). */
	"sx126x",
	"sx126x_hal",
	"sx126x_hal_common",
	"sx12xx_common",
	"lbm_driver",
	"sx127x",
	"lr11xx",
	"lr11xx_hal",
	"rylr",

	/* Protocol/subsystem internals -- low value remote. */
	"meshtastic_mqtt",
	"meshtastic_pki",
	"meshtastic_display",
	"meshtastic_wifi_auto",
	"heltec_v4_fem",

	/* Fires on every RX/TX power-state transition -- see file header. */
	"powermon",

	/* Defense in depth: this is the tool that flooded the log badly
	 * enough to lose ~85% of its own output on 2026-08-09 -- restricted
	 * here too in case CONFIG_MESHTASTIC_HEAP_TRACE is ever turned back
	 * on without also remembering this. */
	"mt_heap_trace",
};

static void apply_netlog_filter_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	const struct log_backend *net_backend = log_backend_net_get();
	size_t applied = 0;

	for (size_t i = 0; i < ARRAY_SIZE(restricted_modules); i++) {
		int source_id = log_source_id_get(restricted_modules[i]);

		if (source_id < 0) {
			/* Not compiled into this build (e.g. a LoRa driver
			 * variant not selected for this board) -- fine. */
			continue;
		}

		(void)log_filter_set(net_backend, 0 /* Z_LOG_LOCAL_DOMAIN_ID */,
				      (int16_t)source_id, LOG_LEVEL_WRN);
		applied++;
	}

	LOG_INF("Netlog filter: %u/%u module(s) restricted to WRN+ on log_backend_net",
		(unsigned int)applied, (unsigned int)ARRAY_SIZE(restricted_modules));
}

static K_WORK_DELAYABLE_DEFINE(apply_netlog_filter_work, apply_netlog_filter_work_fn);

static void l4_event_handler(uint64_t mgmt_event, struct net_if *iface, void *info,
			      size_t info_length, void *user_data)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(info);
	ARG_UNUSED(info_length);
	ARG_UNUSED(user_data);

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		/* Deferred rather than applied inline: log_backend_net has its
		 * own handler for this same event (log_backend_net.c,
		 * l4_event_handler -> log_backend_net_start() ->
		 * log_backend_activate()), and NET_MGMT_REGISTER_EVENT_HANDLER
		 * doesn't guarantee handler ordering between the two. A short
		 * delay guarantees that activation has completed before this
		 * runs, removing that race as a variable. */
		(void)k_work_schedule(&apply_netlog_filter_work, K_SECONDS(2));
	}
}

NET_MGMT_REGISTER_EVENT_HANDLER(mt_netlog_filter_event_handler, NET_EVENT_L4_CONNECTED,
				 &l4_event_handler, NULL);
