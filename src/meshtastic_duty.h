/* SPDX-License-Identifier: GPL-3.0
 *
 * Regulatory duty-cycle enforcement. See meshtastic_duty.c for why this is a
 * refusal at egress rather than a throttle.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_DUTY_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_DUTY_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include "meshtastic/config.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_duty_stats {
	uint32_t blocked;       /**< frames refused at egress */
	uint32_t blocked_relay; /**< of those, ones that were somebody else's traffic */
	uint8_t last_silent_minutes;
};

#if defined(CONFIG_MESHTASTIC_DUTY_CYCLE)

/**
 * @brief Record the regulatory ceiling for the region now in force.
 *
 * Called from the config-apply path with the value the frequency plan resolved.
 * 100 means unrestricted. Cached rather than re-derived per send: resolving a
 * frequency plan walks the band tables, and the egress path must stay cheap.
 */
void meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode region, float pct);

/** @brief Record @c config.lora.override_duty_cycle. */
void meshtastic_duty_set_override(bool override_on);

/**
 * @brief The ceiling that actually applies to this node, in percent.
 *
 * The region ceiling, except in EU_866 where the allocation is role-dependent:
 * 10 % for a ROUTER or ROUTER_LATE, 2.5 % otherwise (upstream
 * RadioInterface.cpp getEffectiveDutyCycle). 100 means unrestricted.
 */
float meshtastic_duty_effective_pct(void);

/**
 * @brief True when a transmission right now would exceed the ceiling.
 *
 * @param silent_minutes_out optional; set to how long until sending is allowed
 *                           again when the answer is true.
 */
bool meshtastic_duty_blocked(uint8_t *silent_minutes_out);

void meshtastic_duty_stats_get(struct meshtastic_duty_stats *out);
void meshtastic_duty_stats_reset(void);

/** @brief Count a refusal (called from the egress gate). */
void meshtastic_duty_note_blocked(bool is_relay, uint8_t silent_minutes);

#else

static inline void meshtastic_duty_set_region(meshtastic_Config_LoRaConfig_RegionCode region,
					      float pct)
{
	ARG_UNUSED(region);
	ARG_UNUSED(pct);
}

static inline void meshtastic_duty_set_override(bool override_on)
{
	ARG_UNUSED(override_on);
}

static inline float meshtastic_duty_effective_pct(void)
{
	return 100.0f;
}

static inline bool meshtastic_duty_blocked(uint8_t *silent_minutes_out)
{
	ARG_UNUSED(silent_minutes_out);
	return false;
}

static inline void meshtastic_duty_stats_get(struct meshtastic_duty_stats *out)
{
	if (out != NULL) {
		*out = (struct meshtastic_duty_stats){0};
	}
}

static inline void meshtastic_duty_stats_reset(void)
{
}

static inline void meshtastic_duty_note_blocked(bool is_relay, uint8_t silent_minutes)
{
	ARG_UNUSED(is_relay);
	ARG_UNUSED(silent_minutes);
}

#endif /* CONFIG_MESHTASTIC_DUTY_CYCLE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_DUTY_H_ */
