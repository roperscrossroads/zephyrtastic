/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_

#include <stdint.h>

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

/*
 * The THIRD version a node carries, and the one the fleet orders by: the
 * MCUboot image header's semantic version (CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION
 * at build, e.g. "0.2.4+0"), read back from the running image in slot0.
 * Orderable, unlike the build id; and it is what a peer's SMP client verifies
 * a pushed image against, so a node advertising it advertises something the
 * courier can act on. Order by (major << 24) | (minor << 16) | revision; the
 * build number is informational.
 *
 * Returns 0, or -ENOTSUP when this image is not MCUboot-managed (no image
 * manager in the build — native_sim, the legacy XIAO layout) and *out is left
 * untouched, or the negative error from the header read.
 */
struct meshtastic_image_version {
	uint8_t major;
	uint8_t minor;
	uint16_t revision;
	uint32_t build;
};

int meshtastic_image_version(struct meshtastic_image_version *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_BUILD_H_ */
