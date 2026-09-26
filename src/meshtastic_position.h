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

/**
 * @brief TX-side precision mask for an outbound Position packet (POS-1).
 *
 * Masks a Position we are about to transmit to the sharing precision of the channel
 * it will go out on, mirroring the self-generated path. Called by
 * meshtastic_modules_sanitise_tx() for POSITION packets we originate. The Position in
 * @p mesh->decoded.payload is re-encoded in place (truncated + precision stamped).
 *
 * @param mesh  Outbound MeshPacket; @p mesh->channel must be the resolved send index.
 *              @p mesh->decoded.payload is rewritten in place.
 * @retval 0        Sanitised (or a no-op for a non-POSITION / undecodable payload).
 * @retval -ENODATA The channel shares no position (precision 0) — suppress the send.
 * @retval -ENOMEM  The re-encoded Position did not fit the payload buffer.
 */
int meshtastic_position_sanitise_tx(meshtastic_MeshPacket *mesh);

/**
 * @brief The position broadcast interval in seconds (agents-t2hb.2).
 *
 * PositionConfig.position_broadcast_secs, or
 * CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC when that is 0. Read from the
 * store on every call; both senders (the GNSS gate and the beacon) use it.
 */
uint32_t meshtastic_position_broadcast_secs(void);

/**
 * @brief Should the GNSS source broadcast now? (agents-t2hb.3)
 *
 * The reference's cadence decision: periodic (interval, held to the stationary
 * floor while the node has not left its precision cell), else smart (moved at
 * least broadcast_smart_minimum_distance, at least
 * broadcast_smart_minimum_interval_secs since the last broadcast). False while
 * the beacon owns the position (a fixed one is set).
 *
 * @param on_fix true when called for a fresh GNSS fix: the smart test then runs
 *               even with position_broadcast_smart_enabled off, as the
 *               reference's handleNewPosition() does.
 */
bool meshtastic_position_broadcast_due(bool on_fix);

/**
 * @brief Distance in metres between two 1e-7-degree points, computed as the
 *        reference's GeoCoord::latLongToMeter (MESHTASTIC_TRIG_APPROX).
 */
float meshtastic_position_distance_m(int32_t lat_a_i, int32_t lon_a_i, int32_t lat_b_i,
				     int32_t lon_b_i);

/**
 * @brief Re-apply the position section after a write (admin set_config, shell).
 *
 * Re-arms the beacon on the current interval and applies gps_mode to the GNSS
 * receiver. The GNSS gate needs nothing: it reads the interval at each fix.
 */
void meshtastic_position_config_changed(void);

/** @brief Persist position_broadcast_secs (0 = compiled default) and apply it. */
int meshtastic_position_set_broadcast_secs(uint32_t secs);

/** @brief Persist gps_mode and apply it. -EINVAL for an unknown mode. */
int meshtastic_position_set_gps_mode(meshtastic_Config_PositionConfig_GpsMode mode);

/** @brief The stored gps_mode. */
meshtastic_Config_PositionConfig_GpsMode meshtastic_position_gps_mode(void);

/**
 * @brief Drop the current source position if it came from @p source.
 *
 * The GNSS source calls this with LOC_INTERNAL when the receiver is turned off,
 * so a fix from before the switch-off is not advertised forever (reference:
 * AdminModule clears the local position when GPS leaves ENABLED).
 */
void meshtastic_position_forget_source(meshtastic_Position_LocSource source);

/**
 * @brief Take a POSITION_APP packet the phone handed us (agents-t2hb.13).
 *
 * Addressed to us (Android's "provide phone location") or broadcast, it is the
 * node's own position: adopted as the current position unless a fixed position
 * is set (then only its time is used), and its time sets the clock at NTP
 * quality when it came on the primary channel. A unicast to a peer is ignored.
 *
 * @return true when the packet was addressed to us and is now consumed -- the
 *         caller must NOT transmit it; false when it should go out as usual
 *         (including a broadcast, which is adopted AND sent).
 */
bool meshtastic_position_handle_from_phone(const meshtastic_MeshPacket *mesh);

#if defined(CONFIG_ZTEST)
/** @brief Test hook: forget the source position and the reply throttle. */
void meshtastic_position_test_reset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_POSITION_H_ */
