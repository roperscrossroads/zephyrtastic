/* SPDX-License-Identifier: GPL-3.0
 *
 * Mesh-facing health/crash-loop announcement -- internal-only init call.
 * See Kconfig.health and meshtastic_health.c for the full design.
 */

#ifndef MESHTASTIC_HEALTH_H_
#define MESHTASTIC_HEALTH_H_

#if defined(CONFIG_MESHTASTIC_HEALTH_REPORT)

/**
 * If this boot has a pending crash breadcrumb, schedule (with an escalating
 * startup delay, per the crash-loop guard) a single announcement on the
 * named health channel -- or stay silent, on a routine boot or a 3rd-and-
 * beyond consecutive short-lived one. Safe to call once at the end of
 * meshtastic_init(), after the radio and channel table are both up.
 */
void meshtastic_health_init(void);

#else /* !CONFIG_MESHTASTIC_HEALTH_REPORT */

static inline void meshtastic_health_init(void)
{
}

#endif /* CONFIG_MESHTASTIC_HEALTH_REPORT */

#endif /* MESHTASTIC_HEALTH_H_ */
