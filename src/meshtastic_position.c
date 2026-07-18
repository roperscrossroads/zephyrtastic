/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * Position portnum module — decoupled from any position *source*. It caches the
 * node's current position (fed by the GNSS driver, if compiled in) and/or an
 * admin-set fixed position, answers incoming Position requests, and broadcasts.
 *
 * This lives here rather than in meshtastic_gnss.c so a node with no GNSS
 * hardware can still advertise a manually-set fixed position (admin
 * set_fixed_position), mirroring the reference firmware where the Position
 * module and the GPS driver are separate.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_clock.h"
#include "meshtastic_modules.h"
#include "meshtastic_position.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

static K_MUTEX_DEFINE(pos_lock);
static struct {
	bool has_current;            /* a source (GNSS) has supplied a position */
	meshtastic_Position current; /* latest source position */
	bool fixed_valid;            /* an admin fixed position is set */
	meshtastic_Position fixed;   /* the fixed position (takes priority) */
	uint32_t seq;
	int64_t last_reply_ms;
	bool reply_time_valid;
} pos_state;

/* Pick the position to send: the fixed position wins over any live source,
 * matching firmware where fixed_position overrides the GPS. */
static bool select_position_locked(meshtastic_Position *out)
{
	if (pos_state.fixed_valid) {
		*out = pos_state.fixed;
		return true;
	}
	if (pos_state.has_current) {
		*out = pos_state.current;
		return true;
	}
	return false;
}

static int position_build_packet(uint32_t dest, bool want_response, uint8_t channel,
				 uint32_t response_to_id, uint8_t *payload,
				 struct meshtastic_packet *packet)
{
	meshtastic_Position position;
	pb_ostream_t stream;
	uint32_t seq;

	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!select_position_locked(&position)) {
		k_mutex_unlock(&pos_lock);
		return -ENODATA;
	}
	pos_state.seq++;
	seq = pos_state.seq;
	k_mutex_unlock(&pos_lock);

	position.seq_number = seq;
	/* Refresh the timestamp on every emission so a static fixed position still
	 * carries a current time (0 until the clock is seeded). */
	position.time = meshtastic_clock_now_epoch();

	stream = pb_ostream_from_buffer(payload, MESHTASTIC_MAX_PAYLOAD_LEN);
	if (!pb_encode(&stream, meshtastic_Position_fields, &position)) {
		const char *err = PB_GET_ERROR(&stream);

		LOG_ERR("Position encode failed: %s", err);
		return -ENOMEM;
	}

	*packet = (struct meshtastic_packet){
		.to = dest,
		.portnum = MESHTASTIC_PORT_POSITION,
		.payload = payload,
		.payload_len = stream.bytes_written,
		.channel = channel,
		.want_response = want_response,
		.request_id = response_to_id,
	};

	return 0;
}

static int position_send(uint32_t dest, k_timeout_t wait)
{
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_packet packet;
	int ret;

	ret = position_build_packet(dest, false, 0U, 0U, payload, &packet);
	if (ret < 0) {
		return ret;
	}

	return meshtastic_send_packet(&packet, wait);
}

int meshtastic_send_position(uint32_t dest)
{
	return position_send(dest, K_FOREVER);
}

int meshtastic_position_get_current(meshtastic_Position *position)
{
	bool ok;

	if (position == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	ok = select_position_locked(position);
	k_mutex_unlock(&pos_lock);

	return ok ? 0 : -ENODATA;
}

void meshtastic_position_set_current(const meshtastic_Position *position)
{
	if (position == NULL) {
		return;
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.current = *position;
	pos_state.has_current = true;
	k_mutex_unlock(&pos_lock);
}

/* Periodic re-broadcast of a fixed position. A GNSS-less node has no data
 * callback to drive sends, so a static fixed position needs its own timer. It
 * self-reschedules only while a fixed position remains set. Runs on the system
 * workqueue with a non-blocking send (drop-if-busy is fine for a periodic
 * beacon). */
static void fixed_broadcast_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(fixed_broadcast_work, fixed_broadcast_work_fn);

static void fixed_broadcast_work_fn(struct k_work *work)
{
	bool valid;

	ARG_UNUSED(work);

	k_mutex_lock(&pos_lock, K_FOREVER);
	valid = pos_state.fixed_valid;
	k_mutex_unlock(&pos_lock);

	if (!valid) {
		return;
	}

	(void)position_send(MESHTASTIC_NODE_BROADCAST, K_NO_WAIT);
	k_work_reschedule(&fixed_broadcast_work,
			  K_SECONDS(CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC));
}

void meshtastic_position_set_fixed(const meshtastic_Position *position)
{
	meshtastic_Position pos;

	if (position == NULL) {
		return;
	}

	/* Take the app-supplied coordinates and stamp the source as manual so
	 * peers and the app render it as a fixed location. */
	pos = *position;
	pos.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
	if (pos.precision_bits == 0U) {
		pos.precision_bits = 32U;
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.fixed = pos;
	pos_state.fixed_valid = true;
	k_mutex_unlock(&pos_lock);

	LOG_INF("Position fixed at lat=%d lon=%d alt=%d",
		pos.has_latitude_i ? pos.latitude_i : 0,
		pos.has_longitude_i ? pos.longitude_i : 0, pos.has_altitude ? pos.altitude : 0);

	/* Announce immediately, then let the work re-arm the periodic beacon. */
	k_work_reschedule(&fixed_broadcast_work, K_NO_WAIT);
}

void meshtastic_position_clear_fixed(void)
{
	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.fixed_valid = false;
	pos_state.fixed = (meshtastic_Position)meshtastic_Position_init_zero;
	k_mutex_unlock(&pos_lock);

	(void)k_work_cancel_delayable(&fixed_broadcast_work);
	LOG_INF("Position fixed cleared");
}

static bool packet_decode_position(const struct meshtastic_packet *packet,
				   meshtastic_Position *position)
{
	pb_istream_t stream;

	if (packet->payload == NULL || packet->payload_len == 0U) {
		return false;
	}

	stream = pb_istream_from_buffer(packet->payload, packet->payload_len);
	if (!pb_decode(&stream, meshtastic_Position_fields, position)) {
		const char *err = PB_GET_ERROR(&stream);

		LOG_DBG("Position decode failed: %s", err);
		return false;
	}

	return true;
}

static void log_position(uint32_t from, uint32_t request_id, const meshtastic_Position *position)
{
	if (request_id != 0U) {
		LOG_INF("Position reply from 0x%08x (request_id 0x%08x): lat=%d lon=%d alt=%d "
			"sats=%u",
			from, request_id, position->has_latitude_i ? position->latitude_i : 0,
			position->has_longitude_i ? position->longitude_i : 0,
			position->has_altitude ? position->altitude : 0, position->sats_in_view);
	} else {
		LOG_INF("Position from 0x%08x: lat=%d lon=%d alt=%d sats=%u", from,
			position->has_latitude_i ? position->latitude_i : 0,
			position->has_longitude_i ? position->longitude_i : 0,
			position->has_altitude ? position->altitude : 0, position->sats_in_view);
	}
}

static void meshtastic_module_position_on_packet(const struct meshtastic_packet *packet)
{
	meshtastic_Position position = meshtastic_Position_init_zero;

	if (packet == NULL || packet->from == 0U || packet->from == meshtastic_get_node_id()) {
		return;
	}

	/*
	 * want_response Position packets are requests (often empty payload), not
	 * position updates — stock clients must not treat them as lat/lon data.
	 */
	if (packet->want_response) {
		return;
	}

	if (!packet_decode_position(packet, &position)) {
		if (packet->request_id != 0U) {
			LOG_WRN("Position reply from 0x%08x (request_id 0x%08x) decode failed",
				packet->from, packet->request_id);
		}
		return;
	}

	log_position(packet->from, packet->request_id, &position);
}

static bool interval_elapsed(bool valid, int64_t last_ms, int64_t now_ms, int64_t interval_ms)
{
	return !valid || (now_ms - last_ms) >= interval_ms;
}

static int meshtastic_module_position_alloc_reply(const struct meshtastic_packet *req,
						  struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	int64_t now_ms;
	int ret;

	if (req == NULL || reply == NULL || req->from == 0U ||
	    req->from == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	now_ms = k_uptime_get();
	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!interval_elapsed(pos_state.reply_time_valid, pos_state.last_reply_ms, now_ms,
			      (int64_t)CONFIG_MESHTASTIC_POSITION_REPLY_SUPPRESS_SEC *
				      MSEC_PER_SEC)) {
		k_mutex_unlock(&pos_lock);
		return -ENOENT;
	}
	pos_state.reply_time_valid = true;
	pos_state.last_reply_ms = now_ms;
	k_mutex_unlock(&pos_lock);

	ret = position_build_packet(req->from, false, req->channel, req->id, payload, reply);
	if (ret == -ENODATA) {
		LOG_DBG("Position request from 0x%08x ignored (no position)", req->from);
		return -ENOENT;
	}

	if (ret == 0) {
		LOG_INF("Position request from 0x%08x, sending response", req->from);
	}

	return ret;
}

MESHTASTIC_MODULE_DEFINE(position, MESHTASTIC_PORT_POSITION, 0,
			 meshtastic_module_position_on_packet,
			 meshtastic_module_position_alloc_reply);
