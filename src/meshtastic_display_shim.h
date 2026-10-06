/*
 * The display shim: a display device between CFB and the panel that forwards a frame only
 * when it differs from the one on the glass (src/meshtastic_display_shim.c). The UI draws
 * on DEVICE_GET(mt_display_shim); the panel is whatever `zephyr,display` chose.
 */
#ifndef MESHTASTIC_DISPLAY_SHIM_H_
#define MESHTASTIC_DISPLAY_SHIM_H_

#include <stdint.h>
#include <zephyr/device.h>

#define MESHTASTIC_DISPLAY_SHIM_NAME "mt_display_shim"

/** The shim: the display device the UI draws on. */
const struct device *meshtastic_display_shim_device(void);

struct meshtastic_display_shim_stats {
	uint32_t written;   /* frames forwarded to the panel */
	uint32_t unchanged; /* frames identical to the one on the glass: not forwarded */
	uint32_t held;      /* differing frames inside MIN_REFRESH_MS of the last: not forwarded */
	uint32_t errors;    /* the panel refused a write */
};

/** The panel behind the shim. */
const struct device *meshtastic_display_shim_panel(void);

void meshtastic_display_shim_stats(struct meshtastic_display_shim_stats *out);

#endif /* MESHTASTIC_DISPLAY_SHIM_H_ */
