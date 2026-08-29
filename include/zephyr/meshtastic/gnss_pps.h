/* SPDX-License-Identifier: GPL-3.0
 *
 * GNSS 1PPS capture — the board's pulse-per-second input, as a time reference.
 *
 * WHY THIS EXISTS. A GNSS receiver knows the time to nanoseconds and tells us in
 * two very different ways. The NMEA sentence carries the digits but arrives late
 * — measured on this bench at 853 ms, being the receiver's fix latency plus the
 * whole sentence's UART time, and one-sided, so it cannot be averaged away. The
 * 1PPS pin carries no digits at all but its rising edge IS the second boundary,
 * to microseconds. Neither is usable alone; together they are a clock.
 *
 * This module owns the edge. It lives in portable code rather than in a board
 * file because the pin is described in devicetree (a meshtastic,gnss-pps node),
 * so any board that routes the pulse gets this for free and one that does not
 * compiles it out.
 */
#ifndef ZEPHYR_INCLUDE_MESHTASTIC_GNSS_PPS_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_GNSS_PPS_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/** Snapshot of the pulse train, for the shell and for health reporting. */
struct meshtastic_gnss_pps_stats {
	uint32_t count;       /**< edges seen since arming */
	uint32_t intervals;   /**< intervals measured (count - 1, saturating) */
	uint32_t min_cyc;     /**< shortest interval, cycles */
	uint32_t max_cyc;     /**< longest interval, cycles */
	uint64_t sum_cyc;     /**< total of all intervals, cycles (never wraps) */
	uint32_t last_cyc;    /**< k_cycle_get_32() at the last edge */
	int64_t last_uptime_ms; /**< k_uptime_get() at the last edge */
	bool armed;           /**< the interrupt is configured */
	bool locked;          /**< edges are arriving at a plausible 1 Hz */
};

#if defined(CONFIG_MESHTASTIC_GNSS_PPS)

/** Arm the edge capture. @p log_edges logs one line per pulse (bench use). */
int meshtastic_gnss_pps_start(bool log_edges);

/** Disarm the capture. */
int meshtastic_gnss_pps_stop(void);

/** Copy out the current statistics. */
void meshtastic_gnss_pps_get_stats(struct meshtastic_gnss_pps_stats *out);

/**
 * @brief Uptime of the most recent edge, if it can be trusted as a time source.
 *
 * @param uptime_ms_out k_uptime_get() sampled inside the edge ISR.
 * @param age_ms_out    optional; how long ago that edge was.
 *
 * @return true only when the train is LOCKED (see below) and the last edge is
 *         younger than one second. Both conditions are load-bearing:
 *
 *         - Unlocked means we have not established that these edges are a 1 Hz
 *           pulse train at all. A floating pin picks up noise, and noise
 *           timestamped as a second boundary is worse than no reference.
 *         - An edge a second or more old means we MISSED one, and pairing a
 *           sentence with the wrong edge is an error of exactly one second —
 *           silent, plausible, and the worst possible size. Refusing is cheap;
 *           the caller falls back to the sentence's own arrival.
 */
bool meshtastic_gnss_pps_last_edge(int64_t *uptime_ms_out, int64_t *age_ms_out);

#else

static inline int meshtastic_gnss_pps_start(bool log_edges)
{
	ARG_UNUSED(log_edges);
	return -ENOTSUP;
}

static inline int meshtastic_gnss_pps_stop(void)
{
	return -ENOTSUP;
}

static inline void meshtastic_gnss_pps_get_stats(struct meshtastic_gnss_pps_stats *out)
{
	if (out != NULL) {
		*out = (struct meshtastic_gnss_pps_stats){0};
	}
}

static inline bool meshtastic_gnss_pps_last_edge(int64_t *uptime_ms_out, int64_t *age_ms_out)
{
	ARG_UNUSED(uptime_ms_out);
	ARG_UNUSED(age_ms_out);
	return false;
}

#endif /* CONFIG_MESHTASTIC_GNSS_PPS */

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_GNSS_PPS_H_ */
