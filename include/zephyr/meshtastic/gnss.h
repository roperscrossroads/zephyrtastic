/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/**
 * @file
 * @brief Meshtastic GNSS position public API.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_GNSS_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_GNSS_H_

#include <zephyr/meshtastic/meshtastic.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send the latest GNSS position if one is available.
 *
 * @param dest Destination node ID, or @ref MESHTASTIC_NODE_BROADCAST.
 *
 * @retval 0        Success.
 * @retval -ENODATA No GNSS fix is available yet.
 * @retval -ENOMEM  Protobuf encoding failed.
 * @retval -EIO     Crypto or radio transmission failed.
 * @retval -ENOTSUP GNSS support is not compiled in.
 */
/**
 * @brief What the GNSS receiver is doing, for `meshtastic gnss status`.
 *
 * Read-only. Ages are milliseconds since the event, or -1 if it never
 * happened. `sats`, `fix_status` (enum gnss_fix_status) and `fix_quality`
 * (enum gnss_fix_quality) are the last data callback's values, fix or not —
 * a receiver tracking satellites without a fix is visible here, which is
 * what an indoor bench needs.
 */
struct meshtastic_gnss_status {
	bool present;            /**< a gnss alias exists in the devicetree */
	bool ready;              /**< and its device is ready */
	const char *dev_name;    /**< the device's name, or NULL */
	bool has_fix;            /**< the position module holds a fix */
	uint32_t callbacks;      /**< data callbacks seen since boot */
	uint32_t fixes;          /**< of which carried a fix */
	uint32_t sends;          /**< automatic position sends handed to the queue */
	uint8_t sats;            /**< satellites in the last callback */
	uint8_t fix_status;      /**< enum gnss_fix_status, last callback */
	uint8_t fix_quality;     /**< enum gnss_fix_quality, last callback */
	uint16_t hdop_centi;     /**< last HDOP in hundredths (0 when unknown) */
	int64_t last_data_age_ms;
	int64_t last_fix_age_ms;
	int64_t last_send_age_ms;
};

/**
 * @brief Fill @p out. -ENODEV when the build has no GNSS device at all.
 */
int meshtastic_gnss_status_get(struct meshtastic_gnss_status *out);

int meshtastic_send_position(uint32_t dest);

/**
 * @brief Broadcast the latest position as a background beacon (G-5).
 *
 * Like @ref meshtastic_send_position to @ref MESHTASTIC_NODE_BROADCAST, but
 * fire-and-forget (K_NO_WAIT) so it is subject to the airtime/channel-util gate.
 * Intended for the periodic auto-send timer; a manual/user-initiated send should
 * use @ref meshtastic_send_position (blocking, ungated).
 *
 * @retval 0        Success (or gate-suppressed under congestion).
 * @retval -ENODATA No GNSS fix / position is available yet.
 * @retval <0       Encode or transmit failure.
 */
int meshtastic_send_position_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_GNSS_H_ */
