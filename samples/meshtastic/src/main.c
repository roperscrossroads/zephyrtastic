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

#include <zephyr/meshtastic/bootlog.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#endif

#include <zephyr/meshtastic/diagnostics.h>
#include <zephyr/meshtastic/logring.h>
#include <zephyr/meshtastic/meshtastic.h>

LOG_MODULE_REGISTER(meshtastic_sample, LOG_LEVEL_INF);

/* Obtain LoRa device from the "lora0" devicetree alias. */
/* _OR_NULL so a radio-less bring-up probe (the radio DRIVER disabled, e.g.
 * -DCONFIG_LORA_SX126X=n, while the DT node stays) still compiles: with no
 * driver the device is NULL, device_is_ready(NULL) is false, and main exits
 * early — USB console, shell and BLE still come up. Exists because a hang in
 * a radio driver's init happens BEFORE the USB console and looks like a dead
 * board; this probe build is how it gets diagnosed. */
static const struct device *lora_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(lora0));

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
 *
 * ESP32-only: needs RTC memory that survives a warm reset, which is an
 * Espressif-specific concept (RTC_NOINIT_ATTR) with no equivalent used here on
 * other chip families. On a board without it, boot-history simply isn't kept
 * across resets -- the watchdog/fatal/logring crash reporting below this block
 * is unaffected and still runs on every board.
 */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
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
#endif /* CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 */

#if defined(CONFIG_NET_MGMT_EVENT)
/* On a networked build the report is deferred to the first IPv4 lease so it
 * lands in the remote syslog, not just a serial console nobody is watching. */
static struct net_mgmt_event_callback ipv4_ready_cb;

static void log_boot_reset_cause(void);

static void ipv4_ready_report(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			      struct net_if *iface)
{
	static bool logged;

	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD || logged) {
		return;
	}
	logged = true;
	log_boot_reset_cause();
}
#endif /* CONFIG_NET_MGMT_EVENT */

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
/* Decodes the Espressif reset-cause bits for the RTC history ring above --
 * both are ESP32-only. */
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
#endif /* CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 */

static const char *fatal_reason_name(uint32_t reason)
{
	switch (reason) {
	case K_ERR_CPU_EXCEPTION:
		return "CPU exception";
	case K_ERR_SPURIOUS_IRQ:
		return "unhandled interrupt";
	case K_ERR_STACK_CHK_FAIL:
		return "stack overflow";
	case K_ERR_KERNEL_OOPS:
		return "kernel oops";
	case K_ERR_KERNEL_PANIC:
		return "kernel panic";
	default:
		return "unknown";
	}
}

static void log_boot_reset_cause(void)
{
	struct meshtastic_watchdog_crash_info wdt_crash;
	struct meshtastic_hw_watchdog_crash_info hw_wdt_crash;
	struct meshtastic_fatal_crash_info fatal_crash;

#if defined(CONFIG_MESHTASTIC_BOOTLOG)
	/* First, and on every target: warm-vs-cold plus the boot counter. The
	 * breadcrumbs below say what the software was doing when it died; this says
	 * whether the RAM rail held, which is the half that distinguishes a hang
	 * from a power event — and the half nRF targets had no way to answer. */
	meshtastic_bootlog_report();
#endif

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	LOG_INF("Reset-cause history (RTC-persistent across warm resets, oldest first):");
	for (uint32_t i = 0; i < RESET_HISTORY_LEN; i++) {
		uint32_t idx = (rtc_history_next + i) % RESET_HISTORY_LEN;

		if (rtc_history[idx].boot_count == 0U) {
			continue; /* slot never used yet */
		}
		log_reset_cause_line("  ", rtc_history[idx].boot_count, rtc_history[idx].reset_cause);
	}
#endif /* CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 */

	/* Companion to the ring above: if the *last* reset was watchdog-forced,
	 * this fills in what the bare cause code above can't -- see
	 * meshtastic_watchdog_take_last_crash() for why RTC-persistent memory
	 * rather than the coredump/log path. One-shot: reported here, then gone. */
	if (meshtastic_watchdog_take_last_crash(&wdt_crash)) {
		LOG_INF("Watchdog crash info: channel \"%s\" timed out, heap free=%u "
			"allocated=%u max_allocated=%u, running thread \"%s\"",
			wdt_crash.channel, wdt_crash.heap_free, wdt_crash.heap_allocated,
			wdt_crash.heap_max_allocated, wdt_crash.thread_name);
	}

	/* Same idea, for the last-resort hardware-ISR path: fires independently
	 * of the scheduler, so a breadcrumb here with NO corresponding entry
	 * above means task_wdt's own software channel timeout never got a
	 * chance to run at all -- the scheduler itself was unresponsive, a
	 * strictly more severe condition than any single channel going unfed.
	 * See meshtastic_watchdog.c's hw_wdt_stage0_callback(). */
	if (meshtastic_hw_watchdog_take_last_crash(&hw_wdt_crash)) {
		LOG_INF("HW watchdog crash info (scheduler-independent path): uptime=%u ms, "
			"running thread \"%s\"",
			hw_wdt_crash.uptime_ms, hw_wdt_crash.thread_name);
	}

	/* Same idea, for the other crash path: a genuine software-detected fault
	 * (assert, stack-canary failure, CPU exception) reaching
	 * k_sys_fatal_error_handler() -- see meshtastic_fatal.c. */
	if (meshtastic_fatal_take_last_crash(&fatal_crash)) {
		LOG_INF("Fatal error crash info: %s (%u), heap free=%u allocated=%u "
			"max_allocated=%u, faulting thread \"%s\"",
			fatal_reason_name(fatal_crash.reason), fatal_crash.reason,
			fatal_crash.heap_free, fatal_crash.heap_allocated,
			fatal_crash.heap_max_allocated, fatal_crash.thread_name);
	}

	/* Last, and deliberately so: the breadcrumbs above are one-line summaries
	 * of machine state, and this is the raw log tail leading up to whatever
	 * happened. Reading the summary first and then the narrative is the useful
	 * order. Also one-shot -- see meshtastic_logring_dump(). */
	meshtastic_logring_dump();
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

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	uint32_t cause = 0;

	/* The cause comes from bootlog, NOT from hwinfo directly. bootlog reads and
	 * clears the register at PRE_KERNEL_1, long before this runs, so a second
	 * hwinfo_get_reset_cause() here would read a register that has already been
	 * consumed and quietly record 0 for every boot — turning this history into a
	 * column of zeroes with no error to notice.
	 *
	 * Still appended to the RTC ring immediately, so it is captured even if this
	 * boot never reaches a log backend. The matching LOG_INF of the full history
	 * fires later — see log_boot_reset_cause(). */
#if defined(CONFIG_MESHTASTIC_BOOTLOG)
	{
		struct meshtastic_boot_record rec;

		meshtastic_bootlog_this_boot(&rec);
		cause = rec.cause;
	}
#else
	(void)hwinfo_get_reset_cause(&cause);
	(void)hwinfo_clear_reset_cause();
#endif

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
#endif /* CONFIG_SOC_FAMILY_ESPRESSIF_ESP32 */

	/* The crash-info report runs on every board, RTC ring or not. On a
	 * networked build it is deferred to the first IPv4 lease so it lands in
	 * the remote syslog; a board with no IP network (LoRa+BLE only) would
	 * otherwise NEVER report -- the earlier "accepted limitation" left every
	 * nRF target blind to its own crashes (and made these three symbols
	 * -Wunused on non-net builds, agents-sgs7.1) -- so there it reports
	 * directly at boot, to the console/BLE log. */
#if defined(CONFIG_NET_MGMT_EVENT)
	net_mgmt_init_event_callback(&ipv4_ready_cb, ipv4_ready_report, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_ready_cb);
#else
	log_boot_reset_cause();
#endif

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
