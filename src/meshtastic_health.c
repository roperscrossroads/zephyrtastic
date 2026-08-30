/* SPDX-License-Identifier: GPL-3.0
 *
 * Mesh-facing health/crash-loop announcement. See Kconfig.health for the
 * full rationale: rung 4 of meshtastic_supervisor.c's escalation ladder, the
 * only rung that costs airtime, so it only ever fires on an abnormal boot
 * (a pending crash breadcrumb -- zephyr/meshtastic/diagnostics.h) and backs
 * off the more it happens.
 *
 * Phase 4 of the diagnostics-baseline plan.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/stats/stats.h>

#include <zephyr/meshtastic/bootlog.h>
#include <zephyr/meshtastic/diagnostics.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/config.pb.h"

#include "meshtastic_channels.h"
#include "meshtastic_core.h"

LOG_MODULE_REGISTER(mt_health, CONFIG_MESHTASTIC_LOG_LEVEL);

STATS_SECT_START(mt_health)
STATS_SECT_ENTRY(announced)
STATS_SECT_ENTRY(suppressed)
STATS_SECT_ENTRY(channel_absent)
STATS_SECT_END;

STATS_NAME_START(mt_health)
STATS_NAME(mt_health, announced)
STATS_NAME(mt_health, suppressed)
STATS_NAME(mt_health, channel_absent)
STATS_NAME_END(mt_health);

static STATS_SECT_DECL(mt_health) mt_health;

/* Which crash breadcrumb (if any) is pending for this boot -- part of the
 * fixed wire struct below, so its numbering is an ABI, not an implementation
 * detail: do not renumber without bumping every consumer, phone included. */
enum meshtastic_health_breadcrumb {
	MESHTASTIC_HEALTH_BREADCRUMB_NONE = 0,
	MESHTASTIC_HEALTH_BREADCRUMB_WATCHDOG = 1,
	MESHTASTIC_HEALTH_BREADCRUMB_HW_WATCHDOG = 2,
	MESHTASTIC_HEALTH_BREADCRUMB_FATAL = 3,
};

/* The allowlist, in full. Boot number, cause bits, flags, previous uptime,
 * which breadcrumb fired -- nothing else. No thread names, no free text, no
 * config values: OBSERVABILITY-AND-COURIER.md §6 is explicit that a
 * free-form event channel is a way past the secret boundary the cluster
 * enforces, and this is the discipline that keeps it from becoming one. */
struct meshtastic_health_report {
	uint32_t boot_num;
	uint32_t cause;
	uint16_t flags;
	uint16_t prev_uptime_s;
	uint8_t breadcrumb;
	uint8_t reserved[3];
} __packed;

static struct k_work_delayable announce_work;

/* The health channel is found by NAME at send time, never cached -- same
 * pattern and same reasoning as meshtastic_cluster.c's cluster_channel_index():
 * channels are operator-editable at runtime, and a stale index would aim
 * frames at whatever channel got renumbered into the slot. If no such
 * channel is provisioned this returns false and the caller idles -- no
 * primary fallback, ever, which is what keeps this off the public mesh by
 * construction rather than by configuration discipline. */
static bool health_channel_index(uint8_t *out)
{
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *ch = meshtastic_channels_get(i);
		const char *name = meshtastic_channels_get_name(i);

		if (ch == NULL || !ch->has_settings ||
		    ch->role == meshtastic_Channel_Role_DISABLED) {
			continue;
		}
		if (name != NULL && strcmp(name, CONFIG_MESHTASTIC_HEALTH_CHANNEL_NAME) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

/* Non-destructive: meshtastic_shell.c's `meshtastic crashinfo` and
 * samples/meshtastic/src/main.c's boot-time report both still get to
 * consume (and clear) the real thing via the _take_ variants. Fatal takes
 * priority as the most specific diagnosis; hw-watchdog next, since reaching
 * it at all means the scheduler itself was unresponsive -- a strictly more
 * severe condition than a single task_wdt channel timing out. */
static enum meshtastic_health_breadcrumb pending_breadcrumb(void)
{
	struct meshtastic_fatal_crash_info fatal;
	struct meshtastic_hw_watchdog_crash_info hw_wdt;
	struct meshtastic_watchdog_crash_info wdt;

	if (meshtastic_fatal_peek_last_crash(&fatal)) {
		return MESHTASTIC_HEALTH_BREADCRUMB_FATAL;
	}
	if (meshtastic_hw_watchdog_peek_last_crash(&hw_wdt)) {
		return MESHTASTIC_HEALTH_BREADCRUMB_HW_WATCHDOG;
	}
	if (meshtastic_watchdog_peek_last_crash(&wdt)) {
		return MESHTASTIC_HEALTH_BREADCRUMB_WATCHDOG;
	}
	return MESHTASTIC_HEALTH_BREADCRUMB_NONE;
}

/* Trailing run of consecutive short-lived boots, most recent first, straight
 * out of meshtastic_bootlog.c's already-retained history -- nothing new is
 * stored for this. Breaks on the first entry that ran
 * CONFIG_MESHTASTIC_HEALTH_SHORT_UPTIME_S or longer, or whose prev_uptime_s
 * is 0 (unknown): ambiguous data does not extend the streak either way. */
static unsigned int trailing_short_boot_streak(void)
{
	struct meshtastic_boot_record hist[CONFIG_MESHTASTIC_BOOTLOG_ENTRIES];
	size_t n = meshtastic_bootlog_history(hist, ARRAY_SIZE(hist));
	unsigned int streak = 0U;

	for (size_t i = n; i > 0U; i--) {
		const struct meshtastic_boot_record *r = &hist[i - 1U];

		if (r->prev_uptime_s == 0U ||
		    r->prev_uptime_s >= (uint16_t)CONFIG_MESHTASTIC_HEALTH_SHORT_UPTIME_S) {
			break;
		}
		streak++;
	}
	return streak;
}

static void announce_work_fn(struct k_work *work)
{
	struct meshtastic_health_report report = {0};
	struct meshtastic_boot_record rec = {0};
	struct meshtastic_packet pkt = {0};
	uint8_t ch_index;

	ARG_UNUSED(work);

	if (!health_channel_index(&ch_index)) {
		STATS_INC(mt_health, channel_absent);
		return;
	}

	meshtastic_bootlog_this_boot(&rec);

	report.boot_num = rec.boot_num;
	report.cause = rec.cause;
	report.flags = rec.flags;
	report.prev_uptime_s = rec.prev_uptime_s;
	report.breadcrumb = (uint8_t)pending_breadcrumb();

	pkt.to = MESHTASTIC_NODE_BROADCAST;
	pkt.channel_index = ch_index;
	pkt.portnum = MESHTASTIC_PORT_HEALTH;
	pkt.payload = (const uint8_t *)&report;
	pkt.payload_len = (uint16_t)sizeof(report);

	/* K_NO_WAIT: fire-and-forget, droppable under congestion -- required for
	 * this to actually reach the BG airtime gate in meshtastic.c's
	 * send_wire_tail() (MT_SCHED_TIER_BG + K_NO_WAIT + broadcast, all three
	 * or the gate does not apply). */
	if (meshtastic_send_packet(&pkt, K_NO_WAIT) == 0) {
		STATS_INC(mt_health, announced);
		LOG_INF("Health announced: boot #%u cause 0x%08x breadcrumb=%u",
			report.boot_num, report.cause, report.breadcrumb);
	}
}

void meshtastic_health_init(void)
{
	unsigned int streak;
	uint32_t delay_s;

	(void)STATS_INIT_AND_REG(mt_health, STATS_SIZE_32, "mt_health");

	if (pending_breadcrumb() == MESHTASTIC_HEALTH_BREADCRUMB_NONE) {
		/* Nothing abnormal about this boot: this feature only ever
		 * speaks about a fault, never as a routine "I'm here" beacon --
		 * that is already NodeInfo's job, over the primary channel. */
		return;
	}

	streak = trailing_short_boot_streak();

	if (streak == 0U) {
		delay_s = CONFIG_MESHTASTIC_HEALTH_ANNOUNCE_DELAY_1ST_S;
	} else if (streak == 1U) {
		delay_s = CONFIG_MESHTASTIC_HEALTH_ANNOUNCE_DELAY_2ND_S;
	} else {
		/* 3rd and beyond: stay silent. A node stuck in a boot loop must
		 * produce LESS traffic over time, not more -- same shape as the
		 * courier's re-arm health window, "so a boot loop cannot become
		 * a push loop" (DECLARATIVE-FLEET.md §18.3). */
		STATS_INC(mt_health, suppressed);
		return;
	}

	k_work_init_delayable(&announce_work, announce_work_fn);
	(void)k_work_schedule(&announce_work, K_SECONDS(delay_s));
}
