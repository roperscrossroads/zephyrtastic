/* SPDX-License-Identifier: GPL-3.0
 *
 * Build identity accessors. The values come from the generated
 * meshtastic_build_id.h (cmake/gen_build_id.cmake regenerates it each build);
 * keeping the include in this one small file means only it recompiles when the
 * build id changes, not every consumer. See meshtastic_build.h.
 */

#include <errno.h>

#include "meshtastic_build.h"

#include "meshtastic_build_id.h"

#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#endif

const char *meshtastic_build_id(void)
{
	return MESHTASTIC_BUILD_ID;
}

const char *meshtastic_build_time(void)
{
	return MESHTASTIC_BUILD_TIME;
}

int meshtastic_image_version(struct meshtastic_image_version *out)
{
#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
	/* The running image is always slot0 in a swap layout: after a test
	 * boot MCUboot has already moved it there, and a revert moves it back. */
	struct mcuboot_img_header hdr;
	int rc = boot_read_bank_header(PARTITION_ID(slot0_partition), &hdr, sizeof(hdr));

	if (rc < 0) {
		return rc;
	}
	if (hdr.mcuboot_version != 1U) {
		return -ENOTSUP;
	}
	out->major = hdr.h.v1.sem_ver.major;
	out->minor = hdr.h.v1.sem_ver.minor;
	out->revision = hdr.h.v1.sem_ver.revision;
	out->build = hdr.h.v1.sem_ver.build_num;
	return 0;
#else
	(void)out;
	return -ENOTSUP;
#endif
}
