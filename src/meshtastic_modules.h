/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_MODULES_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_MODULES_H_

#include <zephyr/sys/iterable_sections.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Module receives every packet when @ref MESHTASTIC_MODULE_ALL_PACKETS is set. */
#define MESHTASTIC_MODULE_ALL_PACKETS BIT(0)

struct meshtastic_module {
	const char *name;
	uint32_t portnum;
	uint8_t flags;
	/**
	 * Handle a received packet. @p mesh is the decoded MeshPacket on the RF
	 * receive path (the C3 currency) and NULL on the public-inject / test
	 * boundary; read it when non-NULL, else fall back to @p packet. The two
	 * are byte-identical when both are present (@p packet is materialised
	 * from @p mesh), so the fallback is parity-preserving.
	 */
	void (*on_packet)(const struct meshtastic_packet *packet,
			  const meshtastic_MeshPacket *mesh);
	/**
	 * Build a reply to @p req. Return 0 when @p reply is ready to send,
	 * -ENOENT when this request should be ignored (no reply), or another
	 * negative errno on failure. @p mesh is the decoded MeshPacket (dual-rep,
	 * see @ref on_packet); NULL on the public-inject / test boundary.
	 */
	int (*alloc_reply)(const struct meshtastic_packet *req,
			   const meshtastic_MeshPacket *mesh, struct meshtastic_packet *reply);
};

/**
 * @brief Register a Meshtastic port handler in the iterable module section.
 */
#define MESHTASTIC_MODULE_DEFINE(_name, _portnum, _flags, _on_packet, _alloc_reply)                \
	static const STRUCT_SECTION_ITERABLE(meshtastic_module, _meshtastic_module_##_name) = {    \
		.name = STRINGIFY(_name), .portnum = (_portnum), .flags = (_flags),                \
				  .on_packet = _on_packet, .alloc_reply = _alloc_reply,            \
	}

/**
 * @brief Dispatch a received packet to the registered per-portnum modules.
 *
 * @param packet Materialised flat-struct view of the packet (always non-NULL).
 * @param mesh   Decoded MeshPacket for the RF receive path (the C3 currency),
 *               or NULL on the public-inject / test boundary. Threaded to each
 *               module's on_packet / alloc_reply for dual-rep reads.
 */
void meshtastic_dispatch_modules(const struct meshtastic_packet *packet,
				 const meshtastic_MeshPacket *mesh);

/**
 * @brief TX-side sanitisation hook for packets we originate — the send-path mirror of
 * meshtastic_dispatch_modules() (upstream's alterReceived / callModules-on-send).
 *
 * C3 Phase 6c: mesh-native. Dispatches per-portnum on @p mesh->decoded.portnum; a
 * handler may rewrite @p mesh->decoded.payload in place to alter what actually goes on
 * air — e.g. masking an outbound Position to the sharing precision of its channel
 * (POS-1). Called from the send engine for locally-originated packets only, with
 * @p mesh->channel already resolved to the send index; relays and MQTT-injected
 * third-party frames do not pass through it.
 *
 * @param mesh Outbound MeshPacket, rewritten in place by a matching handler.
 * @retval 0        Send the (possibly rewritten) packet.
 * @retval -ENODATA Suppress the send (e.g. a channel that shares no position).
 * @retval <0       Other errno from a handler.
 */
int meshtastic_modules_sanitise_tx(meshtastic_MeshPacket *mesh);

/**
 * @brief Fill standard reply header fields per Meshtastic setReplyTo().
 *
 * Correlates the reply with @p req via Data.request_id (not reply_id).
 */
static inline void meshtastic_packet_set_reply_to(struct meshtastic_packet *reply,
						  const struct meshtastic_packet *req)
{
	reply->to = req->from;
	if (req->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
		reply->channel_index = req->channel_index;
	} else if (req->channel != 0U) {
		reply->channel = req->channel;
	}
	reply->request_id = req->id;
	reply->reply_id = 0U;
	reply->want_response = false;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_MODULES_H_ */
