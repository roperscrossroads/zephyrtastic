/* SPDX-License-Identifier: GPL-3.0 */

/* The bearer registry (ATTACHMENT-SCOPE §4). See meshtastic_attach_bearer.h. */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "meshtastic_attach_bearer.h"
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
#include "meshtastic_attachment.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
#include "meshtastic_attachment_head.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define BEARERS_MAX 3U /* ble + a wire + a test pipe */

static const struct meshtastic_attach_bearer *bearers[BEARERS_MAX];
static K_MUTEX_DEFINE(reg_lock);

int meshtastic_attach_bearer_register(const struct meshtastic_attach_bearer *b)
{
	int ret = -ENOSPC;

	if (b == NULL || b->send == NULL || b->link_info == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&reg_lock, K_FOREVER);
	for (unsigned int i = 0U; i < BEARERS_MAX; i++) {
		if (bearers[i] == b) {
			ret = 0;
			break;
		}
		if (bearers[i] == NULL) {
			bearers[i] = b;
			ret = 0;
			break;
		}
	}
	k_mutex_unlock(&reg_lock);
	if (ret == 0) {
		LOG_INF("attach: bearer %s registered", b->name);
	}
	return ret;
}

void meshtastic_attach_bearer_unregister(const struct meshtastic_attach_bearer *b)
{
	k_mutex_lock(&reg_lock, K_FOREVER);
	for (unsigned int i = 0U; i < BEARERS_MAX; i++) {
		if (bearers[i] == b) {
			bearers[i] = NULL;
		}
	}
	k_mutex_unlock(&reg_lock);
}

bool meshtastic_attach_bearer_link_info(uint32_t peer, struct meshtastic_attach_link_info *out,
					const struct meshtastic_attach_bearer **which)
{
	struct meshtastic_attach_link_info info;
	bool known = false;

	if (out != NULL) {
		memset(out, 0, sizeof(*out));
	}
	if (which != NULL) {
		*which = NULL;
	}
	k_mutex_lock(&reg_lock, K_FOREVER);
	for (unsigned int i = 0U; i < BEARERS_MAX; i++) {
		const struct meshtastic_attach_bearer *b = bearers[i];

		if (b == NULL || !b->link_info(peer, &info)) {
			continue;
		}
		/* The first bearer with the link UP wins; a bearer that merely
		 * remembers the peer only counts if nobody has it up. */
		if (!known || (info.up && out != NULL && !out->up)) {
			if (out != NULL) {
				*out = info;
			}
			if (which != NULL) {
				*which = b;
			}
			known = true;
		}
		if (info.up) {
			break;
		}
	}
	k_mutex_unlock(&reg_lock);
	return known;
}

int meshtastic_attach_bearer_send(uint32_t peer, const uint8_t *env, size_t len)
{
	const struct meshtastic_attach_bearer *b;
	struct meshtastic_attach_link_info info;

	if (!meshtastic_attach_bearer_link_info(peer, &info, &b) || !info.up) {
		return -EHOSTUNREACH;
	}
	return b->send(peer, env, len);
}

int meshtastic_attach_bearer_rx(const struct meshtastic_attach_bearer *b, uint32_t peer,
				const uint8_t *env, size_t len)
{
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
	return meshtastic_attachment_ingest_from(b, peer, env, len);
#elif defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
	return meshtastic_attachment_head_on_envelope_from(b, peer, env, len);
#else
	ARG_UNUSED(b);
	ARG_UNUSED(peer);
	ARG_UNUSED(env);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

void meshtastic_attach_bearer_link_down(const struct meshtastic_attach_bearer *b, uint32_t peer)
{
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
	meshtastic_attachment_link_down(b, peer);
#else
	ARG_UNUSED(b);
	ARG_UNUSED(peer);
#endif
}
