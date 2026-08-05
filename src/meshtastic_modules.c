/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>

#include "meshtastic_modules.h"
#if defined(CONFIG_MESHTASTIC_POSITION)
#include "meshtastic_position.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

int meshtastic_modules_sanitise_tx(meshtastic_MeshPacket *mesh)
{
	if (mesh == NULL) {
		return 0;
	}

#if defined(CONFIG_MESHTASTIC_POSITION)
	/* portnum is the nanopb enum; compare through uint32_t (-Werror=enum-compare). */
	if ((uint32_t)mesh->decoded.portnum == MESHTASTIC_PORT_POSITION) {
		return meshtastic_position_sanitise_tx(mesh);
	}
#endif
	return 0;
}

static bool packet_is_to_us(const struct meshtastic_packet *packet)
{
	return packet != NULL && packet->to == meshtastic_get_node_id();
}

void meshtastic_dispatch_modules(const struct meshtastic_packet *packet,
				 const meshtastic_MeshPacket *mesh)
{
	struct meshtastic_packet reply = {0};
	const struct meshtastic_module *handler = NULL;
	int ret;

	if (packet == NULL) {
		return;
	}

	STRUCT_SECTION_FOREACH(meshtastic_module, mod) {
		if (mod->on_packet == NULL) {
			continue;
		}

		if ((mod->flags & MESHTASTIC_MODULE_ALL_PACKETS) != 0U ||
		    mod->portnum == packet->portnum) {
			mod->on_packet(packet, mesh);
		}
	}

	if (!packet_is_to_us(packet) || !packet->want_response) {
		return;
	}

	STRUCT_SECTION_FOREACH(meshtastic_module, mod) {
		if (mod->portnum != packet->portnum || mod->alloc_reply == NULL) {
			continue;
		}

		ret = mod->alloc_reply(packet, mesh, &reply);
		if (ret == -ENOENT) {
			continue;
		}

		handler = mod;
		break;
	}

	if (handler == NULL) {
		LOG_DBG("want_response on port %u but no handler", (unsigned int)packet->portnum);
		return;
	}

	if (ret < 0) {
		LOG_WRN("Module '%s' alloc_reply failed (%d)", handler->name, ret);
		return;
	}

	meshtastic_packet_set_reply_to(&reply, packet);

	LOG_INF("Module '%s' sending want_response reply", handler->name);
	ret = meshtastic_send_packet(&reply, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN("Reply send failed (%d)", ret);
	}
}
