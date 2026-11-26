/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic sched`: the scheduler / QoS policy. Built with the shell
 * (src/CMakeLists.txt); moved from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_mqtt.h"
#include "meshtastic_pki.h"
#include "meshtastic_sched.h"
#include "meshtastic_shell_internal.h"

static int cmd_sched_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_sched_config c;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_sched_snapshot(&c);

	shell_print(sh, "TX egress:");
	shell_print(sh, "  tx.order     %-9s [fifo|priority]",
		    meshtastic_sched_order_name(c.tx_order));
	shell_print(sh, "  tx.overflow  %-9s [drop-newest|drop-lowest]",
		    meshtastic_sched_overflow_name(c.tx_overflow));
	shell_print(sh, "  tx.depth     %u / %u max", c.tx_depth,
		    CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX);
	shell_print(sh, "Phone queue:");
	shell_print(sh, "  phone.evict  %-9s [drop-oldest|protect]",
		    meshtastic_sched_phone_evict_name(c.phone_evict));
	shell_print(sh, "Airtime / dedup:");
	if (c.airtime_max_util != 0U) {
		shell_print(sh, "  airtime.max  %u%%        [0=off, else max chan-util for BG TX]",
			    c.airtime_max_util);
	} else {
		shell_print(sh, "  airtime.max  off       [0=off, else max chan-util for BG TX]");
	}
	if (c.dedup_ttl_sec != 0U) {
		shell_print(sh, "  dedup.ttl    %us       [0=never expire] (cache %u entries)",
			    c.dedup_ttl_sec, CONFIG_MESHTASTIC_DUP_CACHE_SIZE);
	} else {
		shell_print(sh, "  dedup.ttl    off       [0=never expire] (cache %u entries)",
			    CONFIG_MESHTASTIC_DUP_CACHE_SIZE);
	}
	shell_print(sh, "Reliable delivery:");
	if (c.reliable_retries != 0U) {
		shell_print(sh, "  reliable.retries %u     [0=off]", c.reliable_retries);
		shell_print(sh, "  reliable.timeout %ums", c.reliable_timeout_ms);
	} else {
		shell_print(sh, "  reliable.retries off   [0=off]");
	}
	if (c.route_ttl_sec != 0U) {
		shell_print(sh, "  route.ttl    %us       [0=never expire]", c.route_ttl_sec);
	} else {
		shell_print(sh, "  route.ttl    off       [0=never expire]");
	}

	shell_print(sh, "Contention window:");
	if (c.cw_max != 0U) {
		shell_print(sh, "  cw.min/max   %u/%u       [pool = 1<<cw slots]", c.cw_min,
			    c.cw_max);
		shell_print(sh, "  cw.offset    %u         [client waits offset*cw.max slots]",
			    c.cw_relay_offset);
	} else {
		shell_print(sh, "  cw.max       off       [0 = transmit without waiting]");
	}
	if (c.cw_slot_ms != 0U) {
		shell_print(sh, "  cw.slot      %ums      [override; 0 = derive from preset]",
			    c.cw_slot_ms);
	} else {
		shell_print(sh, "  cw.slot      derived   [from the active modem preset]");
	}

	shell_fprintf(sh, SHELL_NORMAL, "presets:");
	for (int i = 0; meshtastic_sched_preset_name(i) != NULL; i++) {
		shell_fprintf(sh, SHELL_NORMAL, " %s", meshtastic_sched_preset_name(i));
	}
	shell_print(sh, "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_sched_show)

static int cmd_sched_policy(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic sched policy <name>");
		return -EINVAL;
	}

	ret = meshtastic_sched_apply_preset(argv[1]);
	if (ret == -ENOENT) {
		shell_error(sh, "unknown policy '%s'", argv[1]);
		return ret;
	}

	shell_print(sh, "policy '%s' applied (stats reset)", argv[1]);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_sched_policy)

static int cmd_sched_set(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc < 3U) {
		shell_error(sh, "usage: meshtastic sched set <key> <value>");
		shell_print(sh, "keys: tx.order tx.overflow tx.depth phone.evict "
				"airtime.max dedup.ttl reliable.retries reliable.timeout "
				"route.ttl cw.min cw.max cw.offset cw.slot");
		return -EINVAL;
	}

	ret = meshtastic_sched_set(argv[1], argv[2]);
	if (ret == -ENOENT) {
		shell_error(sh, "unknown key '%s'", argv[1]);
	} else if (ret == -EINVAL) {
		shell_error(sh, "bad value '%s' for %s", argv[2], argv[1]);
	} else {
		shell_print(sh, "%s = %s (stats reset)", argv[1], argv[2]);
	}
	return ret;
}
MTSH_DEFINE_ON_WORK(cmd_sched_set)

static int cmd_sched_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_sched_stats st;
	struct meshtastic_status status;

	if (argc >= 2U && strcmp(argv[1], "reset") == 0) {
		meshtastic_sched_stats_reset();
		meshtastic_radio_cad_agc_stats_reset();
		meshtastic_radio_rx_activity_stats_reset();
		meshtastic_radio_rx_queue_peak_reset();
		meshtastic_radio_tx_defer_stats_reset();
#if defined(CONFIG_MESHTASTIC_PKI)
		meshtastic_pki_stats_reset();
#endif
		shell_print(sh, "sched stats reset");
		return 0;
	}

	meshtastic_sched_stats_get(&st);
	(void)meshtastic_get_status(&status);

	shell_print(sh, "RX   pkts %u  drop(queue) %u (injected %u)  peak %u/%u  dup %u  decode-fail %u",
		    status.rx_packets, status.rx_dropped, status.rx_dropped_injected,
		    status.rx_queue_peak, (unsigned int)CONFIG_MESHTASTIC_RX_QUEUE_DEPTH,
		    status.duplicate_packets, status.decode_failures);
	shell_print(sh, "TX   pkts %u  failed %u  relayed %u", status.tx_packets,
		    status.tx_failures, status.relayed_packets);
#if defined(CONFIG_MESHTASTIC_PKI)
	{
		struct meshtastic_pki_stats p;

		meshtastic_pki_stats_get(&p);
		shell_print(sh, "PKI  x25519 %u (avg %u us, max %u us)  cache hits %u of %u slots  "
				"failed %u",
			    p.derived, p.us_avg, p.us_max, p.hits, p.cache_size, p.failed);
	}
#endif
	shell_print(sh, "TX egress by tier (enqueued / dropped):");
	for (uint8_t t = 0; t < MT_SCHED_TIER_COUNT; t++) {
		shell_print(sh, "  %-6s %u / %u", meshtastic_sched_tier_name(t), st.tx_enq[t],
			    st.tx_drop[t]);
	}
	shell_print(sh, "outbound hi-water: %u / %u", st.ob_hiwater,
		    CONFIG_MESHTASTIC_OUTBOUND_QUEUE_MAX);
	shell_print(sh, "phone FromRadio drops with no client reading: %u", st.phone_drop_idle);
	shell_print(sh, "phone FromRadio drops: %u (%u protected)", st.phone_drop,
		    st.phone_drop_protected);
	shell_print(sh, "airtime-gated BG broadcasts: %u", st.tx_airtime_drop);
	shell_print(sh, "dedup TTL expiries: %u", st.dedup_expired);
	shell_print(sh, "reliable delivery: %u acked, %u failed", st.reliable_acked,
		    st.reliable_failed);

	/* CAD (listen-before-talk) and periodic AGC-reset outcomes -- driver-owned
	 * counters (sx126x.c), zero on a build without CONFIG_LORA_SX126X. A
	 * nonzero "timeout" here specifically is worth watching: it is the exact
	 * symptom of the DIO1/irq_work race G-3 fixed (2026-08-06) -- see
	 * meshtastic_radio.c and zephyr/patches/0005-*. */
	shell_print(sh, "CAD: clear %u  busy %u  timeout %u  error %u",
		    meshtastic_radio_cad_clear_count(), meshtastic_radio_cad_busy_count(),
		    meshtastic_radio_cad_timeout_count(), meshtastic_radio_cad_error_count());
	shell_print(sh, "AGC reset: ok %u  fail %u", meshtastic_radio_agc_reset_ok_count(),
		    meshtastic_radio_agc_reset_fail_count());
	if (meshtastic_radio_agc_reset_skipped_count() > 0U ||
	    meshtastic_radio_agc_patch_fail_count() > 0U) {
		shell_print(sh, "  skipped (TX in flight) %u  sensitivity-patch re-apply failed %u",
			    meshtastic_radio_agc_reset_skipped_count(),
			    meshtastic_radio_agc_patch_fail_count());
	}

	/* Flood redundancy: how many of our relays a peer also relayed, and how
	 * soon after ours theirs arrived. Gaps inside a plausible contention window
	 * are the transmissions a delay + overhear-cancel would have saved. */
	if (st.relay_deferred_late > 0U) {
		shell_print(sh, "relays deferred to the late window: %u", st.relay_deferred_late);
	}
	shell_print(sh, "relays cancelled on overhear: %u (transmitted %u of %u queued)",
		    st.relay_cancelled, st.relay_sent - MIN(st.relay_cancelled, st.relay_sent),
		    st.relay_sent);
	shell_print(sh, "relays sent: %u  also relayed by a peer: %u (%u%%)", st.relay_sent,
		    st.relay_redundant,
		    st.relay_sent ? (unsigned int)((st.relay_redundant * 100U) / st.relay_sent) : 0U);
	if (st.relay_redundant > 0U) {
		shell_fprintf(sh, SHELL_NORMAL, "  peer-relay gap after ours:");
		for (int i = 0; i < MT_RELAY_GAP_BUCKETS; i++) {
			if (i == MT_RELAY_GAP_BUCKETS - 1) {
				shell_fprintf(sh, SHELL_NORMAL, " >=%ums:%u",
					      meshtastic_relay_gap_bounds[i - 1], st.relay_gap[i]);
			} else {
				shell_fprintf(sh, SHELL_NORMAL, " <%ums:%u",
					      meshtastic_relay_gap_bounds[i], st.relay_gap[i]);
			}
		}
		shell_print(sh, "");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_sched_stats)

static int cmd_sched_defaults(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_sched_defaults();
	shell_print(sh, "sched reverted to compiled defaults");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_sched_defaults)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_sched_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("Show scheduler policy and knobs.", NULL), cmd_sched_show_on_work),
	SHELL_CMD(policy, NULL, SHELL_HELP("Apply a named policy preset.", "<name>"),
		  cmd_sched_policy_on_work),
	SHELL_CMD(set, NULL, SHELL_HELP("Set one knob.", "<key> <value>"), cmd_sched_set_on_work),
	SHELL_CMD(stats, NULL, SHELL_HELP("Show or reset live counters.", "[reset]"),
		  cmd_sched_stats_on_work),
	SHELL_CMD(defaults, NULL, SHELL_HELP("Revert to compiled defaults.", NULL),
		  cmd_sched_defaults_on_work),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), sched, &meshtastic_sched_cmds,
		 SHELL_HELP("Scheduler / QoS policy commands.", NULL),
		 cmd_sched_show_on_work, 0, 0);
