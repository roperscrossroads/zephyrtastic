/* SPDX-License-Identifier: GPL-3.0
 *
 * Node identity derived from the public key — see meshtastic_node_identity.h.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_node_identity.h"
#include "meshtastic_pki.h"

LOG_MODULE_REGISTER(mt_node_id, CONFIG_MESHTASTIC_LOG_LEVEL);

#define NODE_ID_RESERVED_MAX 3U
#define NODE_ID_KEY          "id"

static struct {
	bool have;
	uint32_t id;
} stored;

uint32_t meshtastic_node_id_from_public_key(const uint8_t *key, size_t len)
{
	/* Zephyr's crc32_ieee is the same CRC-32 as the reference's crc32Buffer; the suite pins
	 * that with the reference library's own known-answer vectors. */
	return crc32_ieee(key, len);
}

bool meshtastic_node_id_usable(uint32_t id)
{
	return id > NODE_ID_RESERVED_MAX && id != MESHTASTIC_NODE_BROADCAST;
}

uint32_t meshtastic_node_id_choose(const uint8_t *key, bool have_stored, uint32_t stored_id,
				   uint32_t provisional, enum meshtastic_node_id_origin *origin)
{
	enum meshtastic_node_id_origin o;
	uint32_t id;

	if (key != NULL) {
		id = meshtastic_node_id_from_public_key(key, MESHTASTIC_PKI_KEY_LEN);
		if (meshtastic_node_id_usable(id)) {
			o = MESHTASTIC_NODE_ID_ORIGIN_KEY;
			goto out;
		}
		LOG_ERR("public key implies unusable node id 0x%08x; not adopting it", id);
	}
	if (have_stored && meshtastic_node_id_usable(stored_id)) {
		id = stored_id;
		o = MESHTASTIC_NODE_ID_ORIGIN_STORE;
	} else {
		id = provisional;
		o = MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL;
	}
out:
	if (origin != NULL) {
		*origin = o;
	}
	return id;
}

static int node_id_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				void *cb_arg)
{
	uint8_t buf[sizeof(uint32_t)];

	if (strcmp(key, NODE_ID_KEY) != 0) {
		return -ENOENT;
	}
	if (len != sizeof(buf) || read_cb(cb_arg, buf, len) != (ssize_t)len) {
		return -EINVAL;
	}
	stored.id = sys_get_le32(buf);
	stored.have = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_node_id, MESHTASTIC_NODE_ID_SUBTREE, NULL,
			       node_id_settings_set, NULL, NULL);

static void persist(uint32_t id)
{
	uint8_t buf[sizeof(uint32_t)];
	int ret;

	sys_put_le32(id, buf);
	ret = settings_save_one(MESHTASTIC_NODE_ID_SUBTREE "/" NODE_ID_KEY, buf, sizeof(buf));
	if (ret != 0) {
		/* Not fatal: the derivation is deterministic, so the next boot with a readable key
		 * arrives at the same id. Only a locked boot needs the record. */
		LOG_ERR("could not persist node id 0x%08x (%d)", id, ret);
		return;
	}
	stored.id = id;
	stored.have = true;
}

/* Our public key, or NULL when there is no usable one (no key yet, a failed persist, or a
 * locked boot, where pki_init generated a throwaway key it could not store). */
static const uint8_t *our_key(uint8_t buf[MESHTASTIC_PKI_KEY_LEN])
{
	if (!meshtastic_pki_have_key() ||
	    meshtastic_pki_get_public_key(buf) != MESHTASTIC_PKI_KEY_LEN) {
		return NULL;
	}
	return buf;
}

uint32_t meshtastic_node_identity_adopt(uint32_t provisional)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];
	enum meshtastic_node_id_origin origin;
	bool had = false;
	uint32_t prev = 0U;
	uint32_t id;

	stored.have = false;
	(void)settings_load_subtree(MESHTASTIC_NODE_ID_SUBTREE);
	had = stored.have;
	prev = stored.id;

	id = meshtastic_node_id_choose(our_key(pub), had, prev, provisional, &origin);

	switch (origin) {
	case MESHTASTIC_NODE_ID_ORIGIN_KEY:
		if (!had) {
			/* The migration event: first boot under this scheme. Worded to be grepped
			 * for in bench captures. */
			LOG_WRN("node id adopted from public key: 0x%08x -> 0x%08x", provisional,
				id);
			persist(id);
		} else if (prev != id) {
			LOG_WRN("node id changed with the key: 0x%08x -> 0x%08x", prev, id);
			persist(id);
		} else {
			LOG_INF("node id 0x%08x (from public key)", id);
		}
		break;
	case MESHTASTIC_NODE_ID_ORIGIN_STORE:
		LOG_WRN("node id 0x%08x from store: no identity key readable (locked boot?)", id);
		break;
	case MESHTASTIC_NODE_ID_ORIGIN_PROVISIONAL:
		LOG_WRN("node id 0x%08x provisional: no identity key and none stored", id);
		break;
	}
	return id;
}

void meshtastic_node_identity_key_reloaded(uint32_t current)
{
	uint8_t pub[MESHTASTIC_PKI_KEY_LEN];
	enum meshtastic_node_id_origin origin;
	uint32_t id;

	id = meshtastic_node_id_choose(our_key(pub), stored.have, stored.id, current, &origin);
	if (origin != MESHTASTIC_NODE_ID_ORIGIN_KEY) {
		return;
	}
	if (!stored.have || stored.id != id) {
		persist(id);
	}
	if (id != current) {
		LOG_WRN("node id: key implies 0x%08x, running as 0x%08x until the next boot", id,
			current);
	}
}
