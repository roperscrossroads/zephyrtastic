/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_EXTNOTIFY_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_EXTNOTIFY_H_

/*
 * External notification (agents-dnr4.17) -- the reference's
 * ExternalNotificationModule, on the statusmessage template, for the one output
 * this port's boards have: the LED behind the led0 alias.
 *
 * Mirrored (the reference's "generic" output, index 0, rule for rule):
 *   - a text payload (TEXT_MESSAGE_APP / ALERT_APP / DETECTION_SENSOR_APP) from
 *     another node alerts when alert_message is set and the message is not
 *     muted, or when alert_bell is set and it carries an ASCII BEL;
 *   - muted: a broadcast on a channel whose module_settings.is_muted is set.
 *     (The reference also honours a per-node mute for DMs; this port's NodeDB
 *     has no such flag, so a DM is never muted here.)
 *   - an alert arms the nag cycle: nag_timeout seconds (or output_ms when 0),
 *     the output toggling every output_ms until the cycle ends, then off;
 *   - `active` chooses the drive polarity on top of the devicetree's own
 *     (true = "on" drives the LED's active level, as the reference's
 *     digitalWrite(output, active ? on : !on)).
 *
 * Refused at write time (validate -> -ENOTSUP -> admin NAK), never stored and
 * ignored: output != 0 (a raw pin number the port cannot map), output_vibra /
 * output_buzzer, alert_*_vibra / alert_*_buzzer, use_pwm, use_i2s_as_buzzer,
 * and `enabled` on a board with no led0. set_ringtone is refused the same way:
 * there is no buzzer to play it on.
 */

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic/module_config.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_extnotify_settings {
	bool enabled;
	bool active;
	bool alert_message;
	bool alert_bell;
	uint32_t output_ms;    /* 0 resolved to the reference's 1000 */
	uint32_t nag_timeout;  /* seconds; 0: one output_ms pulse */
	bool output_present;   /* the board has an led0 */
};

struct meshtastic_extnotify_stats {
	uint32_t alerts;       /* nag cycles armed */
	uint32_t bells;        /* alerts that carried a BEL */
	uint32_t muted;        /* messages skipped as muted */
};

int meshtastic_extnotify_init(void);
void meshtastic_extnotify_settings(struct meshtastic_extnotify_settings *out);

/** @brief Can this port honour the section? 0 or -ENOTSUP. */
int meshtastic_extnotify_validate(const meshtastic_ModuleConfig_ExternalNotificationConfig *cfg);

/** @brief Validate, persist, re-read. */
int meshtastic_extnotify_set(const meshtastic_ModuleConfig_ExternalNotificationConfig *cfg);

/** @brief ModuleConfig.external_notification was written by someone else. */
void meshtastic_extnotify_config_changed(void);

/** @brief Start a nag cycle now, as a received alert would (shell `notify test`). */
int meshtastic_extnotify_trigger(void);

/** @brief End the cycle and switch the output off (reference stopNow). */
void meshtastic_extnotify_stop(void);

/** @brief A cycle is in progress (reference nagging()). */
bool meshtastic_extnotify_nagging(void);

/** @brief The output is currently driven "on". */
bool meshtastic_extnotify_output_on(void);

void meshtastic_extnotify_stats(struct meshtastic_extnotify_stats *out);
void meshtastic_extnotify_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_EXTNOTIFY_H_ */
