/* SPDX-License-Identifier: GPL-3.0
 *
 * TraceRoute responder (portnum TRACEROUTE_APP). Replies to RouteDiscovery
 * requests addressed to this node so the app / `meshtastic --traceroute` can
 * map the path to us. Mirrors the reference firmware's destination behavior:
 * the request's route[] holds the intermediate hops it traversed (empty for a
 * direct request), and the destination appends only the SNR of the hop that
 * reached it to snr_towards[] — not its own id (it is the implied endpoint).
 *
 * Scope: responder only. This flood-router port does not alter the payload of
 * packets it rebroadcasts, so it does not insert itself into the route[] of
 * traces passing THROUGH it, and it has no local trace initiator (no UI).
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_modules.h"
#include "meshtastic_packet.h"

#include "meshtastic/mesh.pb.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Convert a raw dB SNR to the wire encoding (dB scaled by 4). INT8_MIN is the
 * reference firmware's "unknown" sentinel, so clamp just above it. */
static int8_t snr_to_q4(int8_t snr_db)
{
	int scaled = (int)snr_db * 4;

	return (int8_t)CLAMP(scaled, INT8_MIN + 1, INT8_MAX);
}

static int traceroute_alloc_reply(const struct meshtastic_packet *req,
				  struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_RouteDiscovery rd = meshtastic_RouteDiscovery_init_zero;
	pb_istream_t istream;
	pb_ostream_t ostream;

	if (req == NULL || reply == NULL) {
		return -EINVAL;
	}

	/* alloc_reply is only dispatched for want_response packets addressed to this
	 * node (see meshtastic_dispatch_modules), so req->to is always our node id
	 * here — a broadcast/relayed trace never reaches this responder. */

	/* Decode the accumulated request (route[] = intermediate hops toward us). */
	istream = pb_istream_from_buffer(req->payload, req->payload_len);
	if (!pb_decode(&istream, meshtastic_RouteDiscovery_fields, &rd)) {
		LOG_WRN("traceroute: RouteDiscovery decode failed");
		return -EINVAL;
	}

	/* Destination appends only the SNR of the last hop (to us), not its id. */
	if (rd.snr_towards_count < (pb_size_t)ARRAY_SIZE(rd.snr_towards)) {
		rd.snr_towards[rd.snr_towards_count++] = snr_to_q4(req->snr);
	}

	ostream = pb_ostream_from_buffer(payload, sizeof(payload));
	if (!pb_encode(&ostream, meshtastic_RouteDiscovery_fields, &rd)) {
		LOG_ERR("traceroute: RouteDiscovery encode failed");
		return -ENOMEM;
	}

	/* setReplyTo: to=req->from, correlate via request_id, preserve channel. */
	*reply = (struct meshtastic_packet){0};
	meshtastic_packet_set_reply_to(reply, req);
	reply->portnum = MESHTASTIC_PORT_TRACEROUTE;
	reply->payload = payload;
	reply->payload_len = ostream.bytes_written;

	LOG_INF("traceroute: reply to 0x%08x (%u hops toward, snr %ddB)", req->from,
		(unsigned int)rd.route_count, req->snr);
	return 0;
}

MESHTASTIC_MODULE_DEFINE(traceroute, MESHTASTIC_PORT_TRACEROUTE, 0, NULL,
			 traceroute_alloc_reply);
