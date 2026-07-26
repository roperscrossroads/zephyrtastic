/* SPDX-License-Identifier: GPL-3.0
 *
 * Build identity accessors. The values come from the generated
 * meshtastic_build_id.h (cmake/gen_build_id.cmake regenerates it each build);
 * keeping the include in this one small file means only it recompiles when the
 * build id changes, not every consumer. See meshtastic_build.h.
 */

#include "meshtastic_build.h"

#include "meshtastic_build_id.h"

const char *meshtastic_build_id(void)
{
	return MESHTASTIC_BUILD_ID;
}

const char *meshtastic_build_time(void)
{
	return MESHTASTIC_BUILD_TIME;
}
