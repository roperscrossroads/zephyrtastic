/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Developer-facing build identity, deliberately SEPARATE from the phone-protocol
 * firmware_version ("2.7.4.zephyr" in meshtastic.c) that the app negotiates with.
 * This exists to answer "is the bench node running the old or new image?" at a
 * glance — it is surfaced by the `meshtastic version` shell command, on the OLED
 * device page, and in the boot log.
 *
 *   meshtastic_build_id()   -> `git describe --always --dirty` (short SHA, with a
 *                              "-dirty" suffix for an uncommitted tree)
 *   meshtastic_build_time() -> UTC build time, "MMDD-HHMM"
 *
 * Both come from the generated meshtastic_build_id.h (cmake/gen_build_id.cmake),
 * isolated in meshtastic_build.c so only that one file recompiles per build.
 */
const char *meshtastic_build_id(void);
const char *meshtastic_build_time(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_ */
