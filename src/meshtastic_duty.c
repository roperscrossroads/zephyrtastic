/* SPDX-License-Identifier: GPL-3.0
 *
 * Regulatory duty-cycle enforcement.
 *
 * WHY THIS EXISTS
 * ---------------
 * Several regions cap how much of an hour a node may spend transmitting:
 * EU_868 10 %, EU_866 2.5 % (10 % for a router), EU_433 and TH 10 %, UA_868
 * 1 %. The port has carried `duty_cycle_pct` per region in
 * meshtastic_region_presets.c for a long time, correctly populated -- and its
 * only consumer was a LOG_DBG. A node in any of those regions transmitted past
 * the legal ceiling without limit. That is docs/parity/OPEN-DIVERGENCES.md
 * `DUTY`, the last Tier-1 (correctness/legal) item in that set.
 *
 * A REFUSAL, NOT A THROTTLE
 * -------------------------
 * The gate does not delay, queue or reschedule anything. It refuses the send and
 * says how long until sending is allowed again. Three reasons, and they are the
 * whole design:
 *
 *   - A regulatory ceiling is not a fairness knob. Deferring a frame by twenty
 *     minutes and transmitting it then is not the same as declining to transmit
 *     it, and a queue of deferred frames would burst the instant the window
 *     reopened -- straight back over the ceiling.
 *   - It must apply to RELAYS as well as to our own traffic. Airtime is airtime;
 *     the regulator does not care whose packet it was. Upstream blocks all
 *     egress in Router::send for exactly this reason, and this port's existing
 *     airtime gate does NOT -- that one throttles only our own background
 *     beacons, which is a politeness measure and a different thing entirely.
 *     Both now run: the politeness gate first, this one at the egress funnel.
 *   - The caller has to learn. A silently dropped text message is worse than a
 *     refused one, so a locally-originated frame that gets refused produces a
 *     DUTY_CYCLE_LIMIT NAK to the phone (see send_wire_tail in meshtastic.c).
 *
 * WHERE IT SITS
 * -------------
 * At outbound_enqueue(), the single funnel every egress path reaches --
 * originated sends, relays, reliable retransmits, routing replies. Deliberately
 * NOT at the TX worker's dequeue: by then the frame is queued and there is
 * nobody left to tell.
 */

#include <errno.h>

#include <zephyr/kernel.h>

#include "meshtastic_airtime.h"
#include "meshtastic_channels.h"
#include "meshtastic_duty.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* EU_866's allocation is split by role rather than being a flat regional
 * number. Values from upstream getEffectiveDutyCycle(). */
#define DUTY_EU866_ROUTER_PCT 10.0f
#define DUTY_EU866_OTHER_PCT  2.5f

#define DUTY_UNRESTRICTED_PCT 100.0f

static struct {
	float region_pct;
	meshtastic_Config_LoRaConfig_RegionCode region;
	bool override_on;
	struct meshtastic_duty_stats stats;
} duty = {
	/* Until a region is applied, assume unrestricted. A node that has not
	 * resolved a frequency plan has not transmitted either, and refusing
	 * everything on an unknown region would brick a fresh node rather than
	 * protect anyone. */
	.region_pct = DUTY_UNRESTRICTED_PCT,
};

void meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode region, float pct)
{
	if (pct <= 0.0f || pct > DUTY_UNRESTRICTED_PCT) {
		/* A zero or nonsense ceiling would mean "never transmit". Treat a
		 * bad table entry as unrestricted and say so, rather than silently
		 * muting the radio. */
		LOG_WRN("duty: ignoring implausible ceiling %d%% for region %d",
			(int)pct, (int)region);
		pct = DUTY_UNRESTRICTED_PCT;
	}

	duty.region = region;
	duty.region_pct = pct;
	LOG_DBG("duty: region %d ceiling %d%% (effective %d%%)", (int)region, (int)pct,
		(int)meshtastic_duty_effective_pct());
}

void meshtastic_duty_set_override(bool override_on)
{
	if (override_on && !duty.override_on) {
		/* Worth a warning rather than a debug line: this is an operator
		 * choosing to transmit past a legal ceiling, and the log is the only
		 * record that it was a choice. */
		LOG_WRN("duty: override_duty_cycle SET — the regulatory ceiling is "
			"no longer enforced on this node");
	}
	duty.override_on = override_on;
}

float meshtastic_duty_effective_pct(void)
{
	if (duty.region == meshtastic_Config_LoRaConfig_RegionCode_EU_866) {
		meshtastic_Config_DeviceConfig_Role role = meshtastic_device_role();

		return (role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
			role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE)
			       ? DUTY_EU866_ROUTER_PCT
			       : DUTY_EU866_OTHER_PCT;
	}

	return duty.region_pct;
}

bool meshtastic_duty_blocked(uint8_t *silent_minutes_out)
{
	float ceiling = meshtastic_duty_effective_pct();
	float used;

	if (duty.override_on || ceiling >= DUTY_UNRESTRICTED_PCT) {
		return false;
	}

	used = meshtastic_airtime_tx_util_percent();
	if (used <= ceiling) {
		return false;
	}

	if (silent_minutes_out != NULL) {
		*silent_minutes_out = meshtastic_airtime_silent_minutes(used, ceiling);
	}
	return true;
}

void meshtastic_duty_note_blocked(bool is_relay, uint8_t silent_minutes)
{
	static int64_t last_log_ms;
	int64_t now = k_uptime_get();

	duty.stats.blocked++;
	if (is_relay) {
		duty.stats.blocked_relay++;
	}
	duty.stats.last_silent_minutes = silent_minutes;

	/* Throttled: once the ceiling is hit, EVERY frame is refused, and a line
	 * per refusal would turn the safety measure into the flood. One line a
	 * minute is enough to see it happening; `meshtastic duty` has the counts. */
	if (duty.stats.blocked == 1U || (now - last_log_ms) >= (int64_t)MSEC_PER_SEC * 60) {
		last_log_ms = now;
		LOG_WRN("duty cycle limit reached (%d%% ceiling, %d%% used) — refusing egress, "
			"%u min to go; %u refused so far (%u of them relays)",
			(int)meshtastic_duty_effective_pct(),
			(int)meshtastic_airtime_tx_util_percent(), silent_minutes,
			duty.stats.blocked, duty.stats.blocked_relay);
	}
}

void meshtastic_duty_stats_get(struct meshtastic_duty_stats *out)
{
	if (out != NULL) {
		*out = duty.stats;
	}
}

void meshtastic_duty_stats_reset(void)
{
	duty.stats = (struct meshtastic_duty_stats){0};
}
