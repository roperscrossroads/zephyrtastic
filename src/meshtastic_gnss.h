/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_GNSS_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_GNSS_H_

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

int meshtastic_gnss_init(void);

#if defined(CONFIG_ZTEST)
/**
 * @brief Test-only: forget when the last fix was sent/attempted.
 *
 * The send gate is a pair of statics (last_sent_ms / last_attempt_ms) with no
 * production reason to be cleared — a node only ever moves forward. But a test
 * that has just exercised the interval gate leaves the module holding a recent
 * "sent" stamp, which then silently suppresses the NEXT test's first fix. So
 * this returns the gate to its post-init state (both stamps one full interval
 * in the past, i.e. a fix is immediately due), exactly as
 * meshtastic_clock_test_reset() does for the wall clock.
 */
void meshtastic_gnss_test_reset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_GNSS_H_ */
