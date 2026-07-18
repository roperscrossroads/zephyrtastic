/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Position portnum module — caches the node position (from GNSS and/or an
 * admin-set fixed position), answers Position requests, and broadcasts.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_

#include "meshtastic_core.h"

#include <zephyr/meshtastic/gnss.h> /* meshtastic_send_position() */

#ifdef __cplusplus
extern "C" {
#endif

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
