/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Position portnum module — caches the node position (from GNSS and/or an
 * admin-set fixed position), answers Position requests, and broadcasts.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_

#include <stdint.h>

#include "meshtastic_core.h"

#include <zephyr/meshtastic/gnss.h> /* meshtastic_send_position() */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Max on-wire position precision (bits) on a publicly-decryptable channel.
 *
 * Mirrors upstream @c MAX_POSITION_PRECISION_PUBLIC_KEY: a precise location must
 * never leak on a channel anyone can decrypt. 15 bits keeps the latitude cell
 * ~700 m worldwide and matches the MQTT map-report public ceiling.
 */
#define MESHTASTIC_MAX_POSITION_PRECISION_PUBLIC_KEY 15U

/**
 * @brief Bit-truncate a latitude/longitude pair to @p precision significant bits.
 *
 * Masks off the low bits and re-centers the result in the middle of the resulting
 * grid cell (stable under GPS jitter), mirroring upstream @c truncateCoordinate.
 * A @p precision of 0 or >= 32 leaves the coordinates unchanged (full resolution);
 * the "0 means do not share" policy is the caller's to enforce.
 *
 * @param latitude_i  In/out latitude_i (Meshtastic 1e-7 deg fixed point).
 * @param longitude_i In/out longitude_i.
 * @param precision   Significant bits to keep (0 or >= 32 leaves both unchanged).
 */
static inline void meshtastic_position_truncate_latlon(int32_t *latitude_i,
							int32_t *longitude_i,
							uint32_t precision)
{
	uint32_t mask;
	uint32_t center;

	if (precision == 0U || precision >= 32U) {
		return;
	}

	mask = UINT32_MAX << (32U - precision);
	center = 1U << (31U - precision);
	*latitude_i = (int32_t)(((uint32_t)*latitude_i & mask) + center);
	*longitude_i = (int32_t)(((uint32_t)*longitude_i & mask) + center);
}

/**
 * @brief Copy the position the node would currently advertise.
 *
 * Returns the admin-set fixed position if one is set, else the latest
 * source-supplied (GNSS) position.
 *
 * @retval 0        Position copied.
 * @retval -EINVAL  @p position is NULL.
 * @retval -ENODATA No position is available.
 */
int meshtastic_position_get_current(meshtastic_Position *position);

/**
 * @brief Feed a fresh source-derived position (called by the GNSS driver).
 *
 * Ignored for send purposes while a fixed position is set (fixed wins), but
 * still cached so clearing the fixed position falls back to live GNSS.
 */
void meshtastic_position_set_current(const meshtastic_Position *position);

/**
 * @brief Set a fixed position (admin set_fixed_position).
 *
 * Overrides any live source, broadcasts immediately, and re-broadcasts
 * periodically so a GNSS-less node still appears on the map.
 */
void meshtastic_position_set_fixed(const meshtastic_Position *position);

/** @brief Clear the fixed position (admin remove_fixed_position). */
void meshtastic_position_clear_fixed(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_ */
