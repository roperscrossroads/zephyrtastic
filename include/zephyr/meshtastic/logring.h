/* SPDX-License-Identifier: GPL-3.0
 *
 * Log ring in RTC-persistent memory -- see meshtastic_logring.c for why this
 * exists (short version: every other diagnostic path this project has depends
 * on the network, and log_backend_net no-ops in panic mode).
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_LOGRING_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_LOGRING_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_MESHTASTIC_LOGRING)

/** True if log content from before the last reset was recovered at boot. */
bool meshtastic_logring_have_previous(void);

/**
 * Print the recovered pre-reset log tail, once.
 *
 * One-shot, like the crash breadcrumbs: after this the snapshot is discarded so
 * a later, unrelated boot cannot re-report a stale tail as if it were fresh.
 * Call once the log path can actually deliver (i.e. after the network is up, if
 * netlog is the backend that matters).
 */
void meshtastic_logring_dump(void);

#else

static inline bool meshtastic_logring_have_previous(void)
{
	return false;
}

static inline void meshtastic_logring_dump(void)
{
}

#endif /* CONFIG_MESHTASTIC_LOGRING */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_LOGRING_H_ */
