/* SPDX-License-Identifier: GPL-3.0
 *
 * Which oscillator drives the nRF 32 kHz clock (LFCLK), asked of the hardware.
 *
 * Why it is worth a line in the boot report and in `meshtastic state` (agents-xf7q): the XIAO
 * runs its LFCLK from the internal RC oscillator because one kit's 32 kHz crystal never
 * started, and Zephyr waits for the crystal forever before main (a dark board). Zephyr 4.5
 * moved the source choice from Kconfig (CONFIG_CLOCK_CONTROL_NRF_K32SRC_*) to the lfclk
 * devicetree node's `k32src`; the old Kconfig lines stayed in the board conf, were dropped
 * with only a Kconfig warning, and every XIAO quietly went back to the crystal. Reading the
 * running source from the CLOCK peripheral, beside what the build asked for, is what says
 * so on a board.
 *
 * Header-only: the sample's boot report and the shell both use it, and on a target without
 * the nRF lfclk driver it compiles to "not available".
 */
#ifndef ZEPHYR_MESHTASTIC_LFCLK_H_
#define ZEPHYR_MESHTASTIC_LFCLK_H_

#include <stdbool.h>
#include <string.h>

#include <zephyr/devicetree.h>

#if defined(CONFIG_CLOCK_CONTROL_NRF_LFCLK) && DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_clock_lfclk)
#define MESHTASTIC_LFCLK_AVAILABLE 1
#include <hal/nrf_clock.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#define MESHTASTIC_LFCLK_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_nrf_clock_lfclk)
#else
#define MESHTASTIC_LFCLK_AVAILABLE 0
#endif

struct meshtastic_lfclk_info {
	/** What the build asked for: the lfclk node's k32src ("rc", "xtal", ...). */
	const char *want;
	/** What the CLOCK peripheral says is running: "rc", "xtal", "synth" or "off". */
	const char *running;
	/** RC calibrations done and skipped since boot; -1 when not counted
	 * (CONFIG_CLOCK_CONTROL_NRF_CALIBRATION_DEBUG off, or not an RC build). */
	int cal;
	int cal_skip;
	/** The running source is the one asked for (an external swing reads back as "xtal"). */
	bool as_asked;
};

/** Fill @p out. False on a target with no nRF LFCLK (nothing to report). */
static inline bool meshtastic_lfclk_get(struct meshtastic_lfclk_info *out)
{
#if MESHTASTIC_LFCLK_AVAILABLE
	nrf_clock_lfclk_t src;

	out->want = DT_PROP(MESHTASTIC_LFCLK_NODE, k32src);
	if (!nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &src)) {
		out->running = "off";
	} else if (src == NRF_CLOCK_LFCLK_RC) {
		out->running = "rc";
	} else if (src == NRF_CLOCK_LFCLK_SYNTH) {
		out->running = "synth";
	} else {
		out->running = "xtal"; /* the HAL reports both external swings as XTAL */
	}
	out->as_asked = (strcmp(out->want, out->running) == 0) ||
			(strncmp(out->want, "ext_", 4) == 0 && strcmp(out->running, "xtal") == 0);
#if defined(CONFIG_CLOCK_CONTROL_NRF_DRIVER_CALIBRATION)
	out->cal = z_nrf_clock_calibration_count();
	out->cal_skip = z_nrf_clock_calibration_skips_count();
#else
	out->cal = -1;
	out->cal_skip = -1;
#endif
	return true;
#else
	(void)out;
	return false;
#endif
}

#endif /* ZEPHYR_MESHTASTIC_LFCLK_H_ */
