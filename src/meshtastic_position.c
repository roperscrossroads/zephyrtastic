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
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_config_store.h"
#include "meshtastic_modules.h"
#include "meshtastic_position.h"
#if defined(CONFIG_MESHTASTIC_GNSS)
#include "meshtastic_gnss.h"
#endif

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
	/* The last periodic/smart broadcast of our own position (agents-t2hb.3):
	 * when, and where we were -- at full precision, the reference's
	 * lastGpsSend / lastGpsLatitude / lastGpsLongitude. Manual sends and
	 * request replies do not stamp it, as in the reference. */
	bool sent_valid;
	int64_t last_sent_ms;
	int32_t last_sent_lat;
	int32_t last_sent_lon;
	bool announce_now;           /* set_fixed: send on the next beacon run */
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

/* Who announces the position, by where it came from. The reference has one
 * PositionModule::runOnce loop for every source; this port has two senders, and
 * each position has exactly one owner so the two never both announce it:
 *
 *   - a live GNSS fix (LOC_INTERNAL): the GNSS source's own fix-driven gate
 *     (meshtastic_gnss.c), which knows when the receiver is producing;
 *   - anything else -- a fixed position, or one the phone supplied
 *     (agents-t2hb.13) -- this module's beacon timer, since nothing else would
 *     ever announce it.
 *
 * A fixed position outranks a live fix (select_position_locked), so while one
 * is set the beacon owns the announcement and the GNSS gate stands down --
 * which also ends the double cadence a GNSS node with a fixed position used to
 * have (GNSS every 300 s plus the fixed timer every 900 s). */
static bool beacon_owns_locked(void)
{
	if (pos_state.fixed_valid) {
		return true;
	}
	return pos_state.has_current &&
	       pos_state.current.location_source != meshtastic_Position_LocSource_LOC_INTERNAL;
}

/* agents-t2hb.2: PositionConfig.position_broadcast_secs, 0 = the compiled
 * default. Read from the store at every use (the Phase A shape, see
 * meshtastic_nodeinfo_interval_secs), so a write through admin, the shell or a
 * cluster document reaches the next broadcast with nothing cached to go stale. */
uint32_t meshtastic_position_broadcast_secs(void)
{
	meshtastic_Config cfg;

	if (meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_position_tag &&
	    cfg.payload_variant.position.position_broadcast_secs != 0U) {
		return cfg.payload_variant.position.position_broadcast_secs;
	}

	return CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC;
}

static void beacon_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(beacon_work, beacon_work_fn);

/* G-1: mirror upstream getPositionPrecisionForChannel + channelFileUsesPublicKey
 * so a broadcast never leaks a more precise location than the channel it goes out
 * on permits.
 *
 * A channel is "publicly decryptable" when anyone can read it: no PSK, a single
 * byte short-PSK alias (the well-known defaultpsk family), or the full 16-byte
 * default PSK (whose last byte varies per default channel). A SECONDARY channel
 * with no PSK inherits the PRIMARY key, so resolve it against the primary. */
static bool channel_uses_public_key(uint8_t index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(index);

	if (ch == NULL || !ch->has_settings ||
	    ch->role == meshtastic_Channel_Role_DISABLED) {
		return false;
	}

	if (ch->settings.psk.size == 0U) {
		if (ch->role == meshtastic_Channel_Role_SECONDARY) {
			uint8_t primary = meshtastic_channels_primary_index();

			return (primary == index) ? true : channel_uses_public_key(primary);
		}
		/* PRIMARY with no PSK: cleartext — readable by anyone. */
		return true;
	}
	if (ch->settings.psk.size == 1U) {
		/* Short-PSK alias: 0 disables encryption, 1..255 select the public
		 * defaultpsk family — either way, publicly decryptable. */
		return true;
	}
	/* Full-length key: public only when it is the default-PSK family (its last
	 * byte is bumped per default channel, so compare the leading bytes). */
	return (ch->settings.psk.size == sizeof(meshtastic_default_psk)) &&
	       (memcmp(ch->settings.psk.bytes, meshtastic_default_psk,
		       sizeof(meshtastic_default_psk) - 1U) == 0);
}

/* On-wire position precision (bits) for a channel slot: the channel's configured
 * precision, clamped to MESHTASTIC_MAX_POSITION_PRECISION_PUBLIC_KEY on a publicly
 * decryptable channel, and 0 ("do not share") when the slot is disabled or carries
 * no module settings. Fail-closed: a channel with no module_settings must not emit
 * a precise position (upstream #10509). */
static uint32_t position_precision_for_channel(uint8_t index)
{
	const meshtastic_Channel *ch = meshtastic_channels_get(index);
	uint32_t precision;

	if (ch == NULL || !ch->has_settings ||
	    ch->role == meshtastic_Channel_Role_DISABLED ||
	    !ch->settings.has_module_settings) {
		return 0U;
	}

	precision = ch->settings.module_settings.position_precision;

	if (precision > MESHTASTIC_MAX_POSITION_PRECISION_PUBLIC_KEY &&
	    channel_uses_public_key(index)) {
		precision = MESHTASTIC_MAX_POSITION_PRECISION_PUBLIC_KEY;
	}

	return precision;
}

/* TX-side sanitisation (the C1 send-path hook, for POSITION): mask a Position we are
 * about to transmit to the sharing precision of the channel it will actually go out
 * on — exactly what position_build_packet() does for a self-generated position. The
 * self-generated path already masks; this closes POS-1, where a phone-injected
 * position reaches meshtastic_send_mesh_pb() and radiates at FULL precision onto a
 * channel configured for coarse or no sharing, because the phone->mesh path never ran
 * it through the position module.
 *
 * C3 Phase 6c: mesh-native. Operates on the outbound MeshPacket the send engine builds
 * the wire from — @p mesh->channel is the already-resolved send index the engine will
 * key the frame with, so precision reads off it directly (no re-resolution). Decodes
 * mesh->decoded.payload, truncates the coordinates, stamps the applied precision, and
 * re-encodes IN PLACE back into the payload buffer (the decode has already extracted
 * every field into @p position, so overwriting the source bytes is safe) — the whole
 * message round-trips, no other Position field is disturbed. Idempotent on an
 * already-masked payload. Returns 0 (payload rewritten), -ENODATA when the channel
 * shares no position (precision 0 — the caller suppresses the send, matching upstream's
 * fail-closed), or -ENOMEM on a re-encode overflow. A payload that does not decode as a
 * Position is left untouched. */
int meshtastic_position_sanitise_tx(meshtastic_MeshPacket *mesh)
{
	meshtastic_Position position = meshtastic_Position_init_zero;
	pb_istream_t istream;
	pb_ostream_t ostream;
	uint8_t send_index;
	uint32_t precision;

	/* mesh->decoded.portnum is the nanopb meshtastic_PortNum enum; compare through
	 * uint32_t against the public port constant (avoids -Werror=enum-compare). */
	if ((uint32_t)mesh->decoded.portnum != MESHTASTIC_PORT_POSITION) {
		return 0;
	}

	/* mesh->channel is the resolved send index (the engine ran
	 * meshtastic_channels_resolve_send_index before calling us). */
	send_index = (uint8_t)mesh->channel;
	precision = position_precision_for_channel(send_index);
	if (precision == 0U) {
		return -ENODATA;
	}
	if (precision > 32U) {
		precision = 32U;
	}

	istream = pb_istream_from_buffer(mesh->decoded.payload.bytes, mesh->decoded.payload.size);
	if (!pb_decode(&istream, meshtastic_Position_fields, &position)) {
		return 0;
	}

	if (precision < 32U && position.has_latitude_i && position.has_longitude_i) {
		meshtastic_position_truncate_latlon(&position.latitude_i, &position.longitude_i,
						    precision);
	}
	position.precision_bits = precision;

	ostream = pb_ostream_from_buffer(mesh->decoded.payload.bytes,
					 sizeof(mesh->decoded.payload.bytes));
	if (!pb_encode(&ostream, meshtastic_Position_fields, &position)) {
		LOG_ERR("position TX sanitise re-encode failed: %s", PB_GET_ERROR(&ostream));
		return -ENOMEM;
	}

	mesh->decoded.payload.size = (pb_size_t)ostream.bytes_written;
	return 0;
}

static int position_build_packet(uint32_t dest, uint8_t channel_index, bool want_response,
				 uint32_t response_to_id, uint8_t *payload,
				 struct meshtastic_packet *packet)
{
	meshtastic_Position position;
	pb_ostream_t stream;
	uint32_t seq;
	uint8_t send_index;
	uint32_t precision;

	/* G-1: clamp the position to the precision of the channel it will actually
	 * be transmitted on. The send path resolves the channel from (to, channel_index), so
	 * mirror that exact resolution here: @p channel_index is the slot the packet will
	 * carry -- MESHTASTIC_CHANNEL_INDEX_INVALID for the send default (primary), or the
	 * request's slot for a reply, which set_reply_to installs at dispatch (POS-3: the
	 * reply used to be masked for slot 0 and then relabelled by sanitise_tx with the
	 * request channel's precision, claiming more precision than it carried). Computed
	 * before touching pos_state so a sharing-disabled channel neither burns a sequence
	 * number nor leaks. */
	send_index = meshtastic_channels_resolve_send_index(
		dest, (channel_index == MESHTASTIC_CHANNEL_INDEX_INVALID) ? 0U : channel_index, 0U);
	precision = position_precision_for_channel(send_index);
	if (precision == 0U) {
		/* Sharing disabled / fail-closed on this channel: emit no position. */
		return -ENODATA;
	}
	if (precision > 32U) {
		precision = 32U;
	}

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

	/* Truncate the on-wire coordinates and stamp the precision actually applied
	 * so peers/apps render the coarsened location correctly. The cached position
	 * is untouched — only this outbound copy is masked. */
	if (precision < 32U && position.has_latitude_i && position.has_longitude_i) {
		meshtastic_position_truncate_latlon(&position.latitude_i,
						    &position.longitude_i, precision);
	}
	position.precision_bits = precision;

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

	ret = position_build_packet(dest, MESHTASTIC_CHANNEL_INDEX_INVALID, false, 0U, payload,
				    &packet);
	if (ret < 0) {
		return ret;
	}

	return meshtastic_send_packet(&packet, wait);
}

int meshtastic_send_position(uint32_t dest)
{
	return position_send(dest, K_FOREVER);
}

/* ---- when to broadcast: the reference's PositionModule::runOnce +
 * handleNewPosition, as one decision both senders consult (agents-t2hb.3) ---- */

/* GeoCoord::latLongToMeter under MESHTASTIC_TRIG_APPROX (the reference's
 * default): an equirectangular projection with the longitude difference wrapped
 * across the antimeridian, cos() replaced by the same minimax polynomial, and the
 * same Earth radius -- so a threshold decision lands where the reference's does.
 * Inputs in 1e-7 degrees. */
static double cos_latitude_approx(double lat_rad)
{
	const double c1 = 0.9999932946, c2 = -0.4999124376, c3 = 0.0414877472,
		     c4 = -0.0012712095;
	double x2 = lat_rad * lat_rad;

	return c1 + x2 * (c2 + x2 * (c3 + c4 * x2));
}

float meshtastic_position_distance_m(int32_t lat_a_i, int32_t lon_a_i, int32_t lat_b_i,
				     int32_t lon_b_i)
{
	const double deg_to_rad = 3.14159265358979323846 / 180.0;
	const double pi = 3.14159265358979323846;
	double a1, a2, b1, b2, d_lng, x, y;

	if (lat_a_i == lat_b_i && lon_a_i == lon_b_i) {
		return 0.0f;
	}

	a1 = (lat_a_i * 1e-7) * deg_to_rad;
	a2 = (lon_a_i * 1e-7) * deg_to_rad;
	b1 = (lat_b_i * 1e-7) * deg_to_rad;
	b2 = (lon_b_i * 1e-7) * deg_to_rad;

	d_lng = b2 - a2;
	if (d_lng > pi) {
		d_lng -= 2.0 * pi;
	} else if (d_lng < -pi) {
		d_lng += 2.0 * pi;
	}
	x = d_lng * cos_latitude_approx((a1 + b1) / 2.0);
	y = b1 - a1;

	return (float)(6366000.0 * sqrt(x * x + y * y));
}

/* Everything the decision reads, resolved once per decision from the store:
 * 0 means "the reference's default" for each numeric field
 * (Default::getConfiguredOrDefault). */
struct bcast_cfg {
	int64_t interval_ms;
	bool smart_enabled;
	uint32_t smart_distance_m;   /* broadcast_smart_minimum_distance, 0 -> 100 */
	int64_t smart_min_ms;        /* broadcast_smart_minimum_interval_secs, 0 -> 300 */
	meshtastic_Config_DeviceConfig_Role role;
	uint32_t wire_precision;     /* on-wire (public-clamped) bits of the broadcast channel */
	uint32_t configured_precision; /* the same channel, unclamped (trackers) */
};

static void bcast_cfg_load(struct bcast_cfg *c)
{
	meshtastic_Config cfg;
	const meshtastic_Channel *ch;
	uint8_t send_index = meshtastic_channels_resolve_send_index(MESHTASTIC_NODE_BROADCAST,
								      0U, 0U);

	*c = (struct bcast_cfg){
		.interval_ms = (int64_t)meshtastic_position_broadcast_secs() * MSEC_PER_SEC,
		.smart_distance_m = 100U,
		.smart_min_ms = 300 * MSEC_PER_SEC,
		.role = meshtastic_Config_DeviceConfig_Role_CLIENT,
	};

	if (meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_position_tag) {
		const meshtastic_Config_PositionConfig *pc = &cfg.payload_variant.position;

		c->smart_enabled = pc->position_broadcast_smart_enabled;
		if (pc->broadcast_smart_minimum_distance != 0U) {
			c->smart_distance_m = pc->broadcast_smart_minimum_distance;
		}
		if (pc->broadcast_smart_minimum_interval_secs != 0U) {
			c->smart_min_ms =
				(int64_t)pc->broadcast_smart_minimum_interval_secs * MSEC_PER_SEC;
		}
	}
	if (meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_device_tag) {
		c->role = cfg.payload_variant.device.role;
	}

	c->wire_precision = position_precision_for_channel(send_index);
	ch = meshtastic_channels_get(send_index);
	c->configured_precision = (ch != NULL && ch->has_settings &&
				   ch->settings.has_module_settings)
					  ? ch->settings.module_settings.position_precision
					  : 0U;
}

enum bcast_decision {
	BCAST_NONE,
	BCAST_PERIODIC,
	BCAST_SMART,
};

/* PositionModule::positionUnchangedSinceLastSend: still inside the precision
 * cell we last broadcast from. Never true before a first broadcast, nor at
 * precision 0 (not shared) or >= 32 (no coarse cell to hold within). */
static bool unchanged_since_last_send_locked(const meshtastic_Position *pos, uint32_t precision)
{
	int32_t a_lat = pos->latitude_i, a_lon = pos->longitude_i;
	int32_t b_lat = pos_state.last_sent_lat, b_lon = pos_state.last_sent_lon;

	if (!pos_state.sent_valid || precision == 0U || precision >= 32U) {
		return false;
	}
	meshtastic_position_truncate_latlon(&a_lat, &a_lon, precision);
	meshtastic_position_truncate_latlon(&b_lat, &b_lon, precision);
	return a_lat == b_lat && a_lon == b_lon;
}

/*
 * The reference's decision, in its order:
 *
 *  1. Periodic: never sent, or the interval has elapsed. The interval is held to
 *     the stationary floor (6 h) while the node is stationary -- a fixed
 *     position, or still in the precision cell of the last broadcast -- except
 *     for LOST_AND_FOUND; a TRACKER judges "same cell" at its configured,
 *     unclamped precision.
 *  2. Otherwise smart: moved at least smart_distance_m since the last broadcast,
 *     both points first snapped to the on-wire precision grid (so a move inside
 *     one cell is no move), and smart_min_ms since the last broadcast.
 *
 * @p smart_allowed is position_broadcast_smart_enabled for the periodic path --
 * and true on a fresh GNSS fix, because the reference's handleNewPosition()
 * runs the distance test without looking at that flag (GPS.cpp calls it on
 * every published fix while gps_mode is ENABLED). Replicated, not endorsed.
 *
 * @p next_ms: the uptime at which the answer could next change without a new
 * position arriving -- the periodic deadline, or when a pending smart move
 * clears its time throttle. The beacon sleeps until then instead of polling on
 * the reference's 5 s tick.
 */
static enum bcast_decision bcast_decide_locked(const struct bcast_cfg *c,
					       const meshtastic_Position *pos, int64_t now,
					       bool smart_allowed, int64_t *next_ms)
{
	bool stationary;
	int64_t effective_ms = c->interval_ms;
	int64_t since;

	*next_ms = INT64_MAX;
	if (!pos_state.sent_valid) {
		return BCAST_PERIODIC;
	}

	if (pos_state.fixed_valid) {
		stationary = true;
	} else if (c->role == meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND) {
		stationary = false;
	} else {
		bool tracker = c->role == meshtastic_Config_DeviceConfig_Role_TRACKER ||
			       c->role == meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;

		stationary = unchanged_since_last_send_locked(
			pos, tracker ? c->configured_precision : c->wire_precision);
	}
	if (stationary &&
	    (int64_t)CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC * MSEC_PER_SEC > effective_ms) {
		effective_ms = (int64_t)CONFIG_MESHTASTIC_POSITION_STATIONARY_FLOOR_SEC * MSEC_PER_SEC;
	}

	since = now - pos_state.last_sent_ms;
	if (since >= effective_ms) {
		return BCAST_PERIODIC;
	}
	*next_ms = pos_state.last_sent_ms + effective_ms;

	if (smart_allowed) {
		int32_t a_lat = pos_state.last_sent_lat, a_lon = pos_state.last_sent_lon;
		int32_t b_lat = pos->latitude_i, b_lon = pos->longitude_i;

		meshtastic_position_truncate_latlon(&a_lat, &a_lon, c->wire_precision);
		meshtastic_position_truncate_latlon(&b_lat, &b_lon, c->wire_precision);
		if (meshtastic_position_distance_m(a_lat, a_lon, b_lat, b_lon) >=
		    (float)c->smart_distance_m) {
			if (since >= c->smart_min_ms) {
				return BCAST_SMART;
			}
			*next_ms = MIN(*next_ms, pos_state.last_sent_ms + c->smart_min_ms);
		}
	}
	return BCAST_NONE;
}

/* Broadcast our position now and stamp it as the last broadcast. A send the
 * channel refuses outright (precision 0, -ENODATA) is not a broadcast and does
 * not stamp; one the airtime gate drops under congestion does -- the reference
 * stamps lastGpsSend before it sends, too. */
static int position_broadcast_stamped(void)
{
	meshtastic_Position pos;
	int ret;

	ret = position_send(MESHTASTIC_NODE_BROADCAST, K_NO_WAIT);
	if (ret == -ENODATA) {
		return ret;
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	if (select_position_locked(&pos)) {
		pos_state.sent_valid = true;
		pos_state.last_sent_ms = k_uptime_get();
		pos_state.last_sent_lat = pos.latitude_i;
		pos_state.last_sent_lon = pos.longitude_i;
	}
	k_mutex_unlock(&pos_lock);
	return ret;
}

bool meshtastic_position_broadcast_due(bool on_fix)
{
	struct bcast_cfg c;
	meshtastic_Position pos;
	enum bcast_decision d = BCAST_NONE;
	int64_t next_ms;

	bcast_cfg_load(&c);
	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!beacon_owns_locked() && select_position_locked(&pos)) {
		d = bcast_decide_locked(&c, &pos, k_uptime_get(), on_fix || c.smart_enabled,
					&next_ms);
	}
	k_mutex_unlock(&pos_lock);
	if (d == BCAST_SMART) {
		LOG_DBG("Position: smart broadcast (moved)");
	}
	return d != BCAST_NONE;
}

int meshtastic_send_position_periodic(void)
{
	/* G-5: a periodic auto-broadcast is a background beacon — fire-and-forget
	 * (K_NO_WAIT) so the airtime/channel-util gate can drop it under congestion,
	 * matching the fixed-beacon path and the reference PositionModule's BG cadence.
	 * A manual meshtastic_send_position() stays K_FOREVER/ungated: the user (or
	 * shell) asked for it explicitly. */
	bool beacon_owned;

	k_mutex_lock(&pos_lock, K_FOREVER);
	beacon_owned = beacon_owns_locked();
	k_mutex_unlock(&pos_lock);

	/* The GNSS gate's entry point: stand down while the beacon owns the
	 * announcement (a fixed position is set). */
	if (beacon_owned) {
		return -ENODATA;
	}

	return position_broadcast_stamped();
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

#if defined(CONFIG_ZTEST)
void meshtastic_position_test_reset(void)
{
	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.has_current = false;
	pos_state.current = (meshtastic_Position)meshtastic_Position_init_zero;
	pos_state.reply_time_valid = false;
	pos_state.sent_valid = false;
	pos_state.announce_now = false;
	k_mutex_unlock(&pos_lock);
	(void)k_work_cancel_delayable(&beacon_work);
}
#endif

/* The beacon: periodic re-broadcast of a position no fix callback drives -- a
 * fixed one, or one the phone supplied (beacon_owns_locked). A GNSS-less node
 * has no other sender, so without this timer such a position would never go out.
 * Self-reschedules only while it still owns a position, re-reading the interval
 * each time; runs on the system workqueue with a non-blocking send (drop-if-busy
 * is fine for a periodic beacon). */
static void beacon_work_fn(struct k_work *work)
{
	struct bcast_cfg c;
	meshtastic_Position pos;
	enum bcast_decision d;
	int64_t now, next_ms;
	bool announce;

	ARG_UNUSED(work);

	bcast_cfg_load(&c);
	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!beacon_owns_locked() || !select_position_locked(&pos)) {
		k_mutex_unlock(&pos_lock);
		return;
	}
	announce = pos_state.announce_now;
	pos_state.announce_now = false;
	now = k_uptime_get();
	d = bcast_decide_locked(&c, &pos, now, c.smart_enabled, &next_ms);
	k_mutex_unlock(&pos_lock);

	if (announce || d != BCAST_NONE) {
		if (d == BCAST_SMART) {
			LOG_DBG("Position: smart broadcast (moved)");
		}
		(void)position_broadcast_stamped();
		/* Re-decide against the new stamp for the next deadline. Still due
		 * means the send did not stamp -- the channel shares no position right
		 * now -- so look again in one interval rather than stopping for good
		 * or spinning on it. */
		k_mutex_lock(&pos_lock, K_FOREVER);
		now = k_uptime_get();
		if (bcast_decide_locked(&c, &pos, now, c.smart_enabled, &next_ms) != BCAST_NONE) {
			next_ms = now + c.interval_ms;
		}
		k_mutex_unlock(&pos_lock);
	}

	/* Never closer than a second: a deadline that is somehow already past
	 * would otherwise re-run this at once, forever, on the system workqueue. */
	if (next_ms != INT64_MAX) {
		k_work_reschedule(&beacon_work, K_MSEC(MAX(next_ms - now, (int64_t)MSEC_PER_SEC)));
	}
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

	/* Persist the coordinates so the fixed position survives a reboot. */
	(void)meshtastic_config_store_set_fixed_position(&pos);

	/* Announce immediately -- the reference's set_fixed_position sends at once
	 * whatever the cadence says -- then let the work re-arm the beacon. */
	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.announce_now = true;
	k_mutex_unlock(&pos_lock);
	k_work_reschedule(&beacon_work, K_NO_WAIT);
}

void meshtastic_position_clear_fixed(void)
{
	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.fixed_valid = false;
	pos_state.fixed = (meshtastic_Position)meshtastic_Position_init_zero;
	k_mutex_unlock(&pos_lock);

	(void)k_work_cancel_delayable(&beacon_work);
	(void)meshtastic_config_store_clear_fixed_position();
	LOG_INF("Position fixed cleared");

	/* A phone-supplied position underneath is the beacon's again. */
	meshtastic_position_config_changed();
}

/* Restore a persisted fixed position after settings load. This hook runs before
 * the radio is up, so the first beacon is scheduled at the normal interval
 * rather than sent immediately. No-op when nothing is persisted. */
static int position_restore_fixed(void)
{
	meshtastic_Position pos;

	if (meshtastic_config_store_get_fixed_position(&pos) < 0) {
		return 0;
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	pos_state.fixed = pos;
	pos_state.fixed_valid = true;
	k_mutex_unlock(&pos_lock);

	LOG_INF("Restored fixed position lat=%d lon=%d alt=%d",
		pos.has_latitude_i ? pos.latitude_i : 0,
		pos.has_longitude_i ? pos.longitude_i : 0, pos.has_altitude ? pos.altitude : 0);

	k_work_reschedule(&beacon_work, K_SECONDS(meshtastic_position_broadcast_secs()));
	return 0;
}

MESHTASTIC_SETTINGS_APPLY_DEFINE(position_fixed, position_restore_fixed);

static bool packet_decode_position(const uint8_t *payload, size_t payload_len,
				   meshtastic_Position *position)
{
	pb_istream_t stream;

	if (payload == NULL || payload_len == 0U) {
		return false;
	}

	stream = pb_istream_from_buffer(payload, payload_len);
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

static void meshtastic_module_position_on_packet(const struct meshtastic_packet *packet,
						 const meshtastic_MeshPacket *mesh)
{
	meshtastic_Position position = meshtastic_Position_init_zero;
	uint32_t from;
	uint32_t request_id;
	bool want_response;

	if (packet == NULL) {
		return;
	}

	/*
	 * Phase 5b: read identity + the decoded payload from the MeshPacket (the C3
	 * currency) on the RF path; fall back to the flat struct on the NULL-mesh
	 * public-inject / test boundary. Byte-identical on the RF path (the struct
	 * is materialised from the same MeshPacket), so this is parity-preserving.
	 */
	from = mesh ? mesh->from : packet->from;
	request_id = mesh ? mesh->decoded.request_id : packet->request_id;
	want_response = mesh ? mesh->decoded.want_response : packet->want_response;

	if (from == 0U || from == meshtastic_get_node_id()) {
		return;
	}

	/* POS-13: a want_response Position is a request, and it is ALSO a
	 * position -- Android's "request position" carries the requester's own
	 * fix and time. The reference's handleReceivedProtobuf takes both; the
	 * reply is the module framework's business (alloc_reply). An empty request
	 * (the Python CLI's) decodes to no time and is inert below. */
	ARG_UNUSED(want_response);

	if (!packet_decode_position(mesh ? mesh->decoded.payload.bytes : packet->payload,
				    mesh ? mesh->decoded.payload.size : packet->payload_len,
				    &position)) {
		if (request_id != 0U) {
			LOG_WRN("Position reply from 0x%08x (request_id 0x%08x) decode failed",
				from, request_id);
		}
		return;
	}

	log_position(from, request_id, &position);

	/*
	 * Mesh time (reference: PositionModule::trySetRtc): a remote position on
	 * the PRIMARY channel carrying a nonzero, device-derived time is offered
	 * to the wall clock at NET quality. The quality ladder does the real
	 * gating — NET seeds an unset (or device-RTC) clock and can never
	 * displace phone/operator (NTP) or GPS time. Deliberate divergences from
	 * the reference, see docs/parity/module-parity notes: no
	 * hasQualityTimesource() pre-gate (no port target has an I2C RTC, and
	 * "GNSS fitted but fixless" is exactly the situation where mesh time
	 * helps — a later fix outranks NET on the ladder anyway), and no T-Watch
	 * force path (no such board).
	 */
	if (position.time != 0U &&
	    packet->channel_index == meshtastic_channels_primary_index() &&
	    position.location_source >= meshtastic_Position_LocSource_LOC_INTERNAL) {
		enum meshtastic_clock_quality before = meshtastic_clock_get_quality();

		meshtastic_clock_set_epoch(position.time, MESHTASTIC_CLOCK_QUALITY_NET);
		if (before < MESHTASTIC_CLOCK_QUALITY_NET &&
		    meshtastic_clock_get_quality() == MESHTASTIC_CLOCK_QUALITY_NET) {
			LOG_INF("Clock set from mesh: epoch %u (position from 0x%08x)",
				position.time, from);
		}
	}
}

/* agents-t2hb.13: a position the phone hands us about ourselves. Reference:
 * PositionModule::handleReceivedProtobuf's isFromUs branch -- the phone's fix
 * becomes the node's own (only its time while a fixed position is set), and its
 * time sets the clock at NTP quality when it came on the primary channel
 * (trySetRtc, isLocal). The reference reaches that branch by loopback through
 * Router::sendLocal, for a packet addressed to us and for a broadcast; a unicast
 * to a peer is not looped back, so it says nothing about where WE are.
 *
 * One deliberate divergence: a packet with no coordinates updates the time only.
 * Upstream would store its (0,0), and this port would then advertise it; the
 * Android app applies the same filter on its side (NodeManagerImpl). */
bool meshtastic_position_handle_from_phone(const meshtastic_MeshPacket *mesh)
{
	meshtastic_Position position = meshtastic_Position_init_zero;
	uint32_t me = meshtastic_get_node_id();
	bool adopted = false;
	bool to_self;

	if (mesh == NULL || mesh->which_payload_variant != meshtastic_MeshPacket_decoded_tag ||
	    (uint32_t)mesh->decoded.portnum != MESHTASTIC_PORT_POSITION) {
		return false;
	}

	to_self = (mesh->to == me);
	if (!to_self && mesh->to != MESHTASTIC_NODE_BROADCAST && mesh->to != 0U) {
		return false;
	}

	if (!packet_decode_position(mesh->decoded.payload.bytes, mesh->decoded.payload.size,
				    &position)) {
		/* Still ours to consume: upstream's local delivery would drop it too,
		 * and it must not go on the air addressed to ourselves. */
		return to_self;
	}

	if (position.time != 0U && mesh->channel == meshtastic_channels_primary_index()) {
		meshtastic_clock_set_epoch(position.time, MESHTASTIC_CLOCK_QUALITY_NTP);
	}

	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!pos_state.fixed_valid &&
	    ((position.has_latitude_i && position.latitude_i != 0) ||
	     (position.has_longitude_i && position.longitude_i != 0))) {
		pos_state.current = position;
		pos_state.has_current = true;
		adopted = true;
		/* A phone BROADCAST is on its way out right now: that is our
		 * broadcast, from here, so stamp it rather than repeat it. */
		if (!to_self) {
			pos_state.sent_valid = true;
			pos_state.last_sent_ms = k_uptime_get();
			pos_state.last_sent_lat = position.latitude_i;
			pos_state.last_sent_lon = position.longitude_i;
		}
	}
	k_mutex_unlock(&pos_lock);

	/* Let the beacon decide on the new position now. It sends only if the
	 * cadence says so -- the first position ever (the reference's runOnce sends
	 * once hasLocalPositionSinceBoot() turns true), a due periodic, or a smart
	 * move -- and otherwise re-arms for its existing deadline, so the phone's
	 * 30 s refreshes never push a broadcast back. */
	if (adopted) {
		k_work_reschedule(&beacon_work, K_NO_WAIT);
	}

	LOG_DBG("Position from phone (to %s): lat=%d lon=%d time=%u",
		to_self ? "self" : "broadcast", position.latitude_i, position.longitude_i,
		position.time);
	return to_self;
}

void meshtastic_position_config_changed(void)
{
	bool owned;

	/* A new interval reaches the beacon now rather than after the old one runs
	 * out; the GNSS gate reads it at its next fix anyway. */
	k_mutex_lock(&pos_lock, K_FOREVER);
	owned = beacon_owns_locked();
	k_mutex_unlock(&pos_lock);

	if (owned) {
		k_work_reschedule(&beacon_work, K_NO_WAIT);
	}

#if defined(CONFIG_MESHTASTIC_GNSS)
	(void)meshtastic_gnss_apply_mode();
#endif
}

void meshtastic_position_forget_source(meshtastic_Position_LocSource source)
{
	k_mutex_lock(&pos_lock, K_FOREVER);
	if (pos_state.has_current && pos_state.current.location_source == source) {
		pos_state.has_current = false;
		pos_state.current = (meshtastic_Position)meshtastic_Position_init_zero;
	}
	k_mutex_unlock(&pos_lock);
}

/* Read-modify-write of one PositionConfig field, then the same live-apply the
 * admin path runs. */
static int position_config_write(uint32_t *broadcast_secs,
				 const meshtastic_Config_PositionConfig_GpsMode *gps_mode)
{
	meshtastic_Config cfg = meshtastic_Config_init_zero;
	int ret;

	ret = meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg);
	if (ret < 0) {
		return ret;
	}
	cfg.which_payload_variant = meshtastic_Config_position_tag;
	if (broadcast_secs != NULL) {
		cfg.payload_variant.position.position_broadcast_secs = *broadcast_secs;
	}
	if (gps_mode != NULL) {
		cfg.payload_variant.position.gps_mode = *gps_mode;
	}
	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		return ret;
	}
	meshtastic_position_config_changed();
	return 0;
}

int meshtastic_position_set_broadcast_secs(uint32_t secs)
{
	return position_config_write(&secs, NULL);
}

int meshtastic_position_set_gps_mode(meshtastic_Config_PositionConfig_GpsMode mode)
{
	if (mode != meshtastic_Config_PositionConfig_GpsMode_DISABLED &&
	    mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
	    mode != meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT) {
		return -EINVAL;
	}
	return position_config_write(NULL, &mode);
}

meshtastic_Config_PositionConfig_GpsMode meshtastic_position_gps_mode(void)
{
	meshtastic_Config cfg;

	if (meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_position_tag) {
		return cfg.payload_variant.position.gps_mode;
	}
	return meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT;
}

static bool interval_elapsed(bool valid, int64_t last_ms, int64_t now_ms, int64_t interval_ms)
{
	return !valid || (now_ms - last_ms) >= interval_ms;
}

static int meshtastic_module_position_alloc_reply(const struct meshtastic_packet *req,
						  const meshtastic_MeshPacket *mesh,
						  struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_Config cfg;
	bool lost_and_found = false;
	int64_t now_ms;
	int ret;

	ARG_UNUSED(mesh);

	if (req == NULL || reply == NULL || req->from == 0U ||
	    req->from == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	if (meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg) == 0 &&
	    cfg.which_payload_variant == meshtastic_Config_device_tag) {
		lost_and_found = cfg.payload_variant.device.role ==
				 meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND;
	}

	/* Reference PositionModule::allocReply: one reply per window, except a
	 * LOST_AND_FOUND node, which answers everyone. A throttled request is
	 * IGNORED -- -ENOENT, no NAK (the reference sets ignoreRequest). */
	now_ms = k_uptime_get();
	k_mutex_lock(&pos_lock, K_FOREVER);
	if (!lost_and_found &&
	    !interval_elapsed(pos_state.reply_time_valid, pos_state.last_reply_ms, now_ms,
			      (int64_t)CONFIG_MESHTASTIC_POSITION_REPLY_SUPPRESS_SEC *
				      MSEC_PER_SEC)) {
		k_mutex_unlock(&pos_lock);
		LOG_DBG("Position request from 0x%08x throttled", req->from);
		return -ENOENT;
	}
	k_mutex_unlock(&pos_lock);

	/* Built for the channel the reply goes out on -- the request's (POS-3). */
	ret = position_build_packet(req->from, req->channel_index, false, req->id, payload,
				    reply);
	if (ret == -ENODATA) {
		/* No position, or the channel shares none: the requester is told
		 * NO_RESPONSE (reference MeshModule: no module replied and none asked
		 * to ignore), and the window is NOT consumed -- it is stamped only for
		 * a reply actually built (POS-11). */
		LOG_DBG("Position request from 0x%08x: nothing to share", req->from);
		return -ENODATA;
	}
	if (ret == 0) {
		k_mutex_lock(&pos_lock, K_FOREVER);
		pos_state.reply_time_valid = true;
		pos_state.last_reply_ms = now_ms;
		k_mutex_unlock(&pos_lock);
		LOG_INF("Position request from 0x%08x, sending response", req->from);
	}

	return ret;
}

MESHTASTIC_MODULE_DEFINE(position, MESHTASTIC_PORT_POSITION, 0,
			 meshtastic_module_position_on_packet,
			 meshtastic_module_position_alloc_reply);
