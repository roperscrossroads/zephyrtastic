/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_modules.h"

#include "meshtastic/deviceonly.pb.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* NodeInfoLite.bitfield bit indices (Meshtastic upstream layout). Only peer
 * public keys are persisted (see MESHTASTIC_NODEDB_PERSIST_KEYS); the bitfield
 * and everything else stays in RAM, so these indices are internal to this
 * firmware. */
#define NODEINFO_BITFIELD_IS_FAVORITE_BIT         0
#define NODEINFO_BITFIELD_IS_IGNORED_BIT          1
#define NODEINFO_BITFIELD_VIA_MQTT_BIT            2
#define NODEINFO_BITFIELD_HAS_USER_BIT            5
#define NODEINFO_BITFIELD_IS_LICENSED_BIT         6
#define NODEINFO_BITFIELD_IS_UNMESSAGABLE_BIT     7
#define NODEINFO_BITFIELD_HAS_IS_UNMESSAGABLE_BIT 8

/* Full-lean: the DB retains only the NodeInfoLite core (identity + pubkey).
 * Position / device+environment telemetry / status are report-and-forget (as in
 * the reference firmware) — not retained per node. */
struct nodedb_entry {
	bool used;
	meshtastic_NodeInfoLite node;
};

static K_MUTEX_DEFINE(nodedb_lock);
static struct nodedb_entry nodedb_entries[CONFIG_MESHTASTIC_NODEDB_MAX_NODES];
static size_t nodedb_entry_count;

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
static void nodekeys_schedule_save(void);
static void warm_upsert_locked(uint32_t num, const uint8_t *pub);
static bool warm_copy_key_locked(uint32_t num, uint8_t out[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN]);
#endif

static uint32_t uptime_seconds(void)
{
	return (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
}

static void copy_string(char *dst, size_t dst_len, const char *src)
{
	if (dst_len == 0U) {
		return;
	}

	dst[0] = '\0';
	if (src != NULL) {
		strncpy(dst, src, dst_len - 1U);
		dst[dst_len - 1U] = '\0';
	}
}

static void apply_user(struct nodedb_entry *entry, const meshtastic_User *user)
{
	meshtastic_NodeInfoLite *node = &entry->node;
	size_t key_len;

	copy_string(node->long_name, sizeof(node->long_name), user->long_name);
	copy_string(node->short_name, sizeof(node->short_name), user->short_name);
	node->hw_model = (uint8_t)user->hw_model;
	node->role = (uint8_t)user->role;

	key_len = MIN((size_t)user->public_key.size, sizeof(node->public_key.bytes));
	{
		bool key_changed = (node->public_key.size != (pb_size_t)key_len) ||
				   (key_len > 0U && memcmp(node->public_key.bytes,
							  user->public_key.bytes, key_len) != 0);

		node->public_key.size = (pb_size_t)key_len;
		if (key_len > 0U) {
			memcpy(node->public_key.bytes, user->public_key.bytes, key_len);
		}
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
		/* Mirror a peer's key into the warm tier + NVS, only when it first
		 * appears or changes, so a known peer re-broadcasting its NodeInfo
		 * does not rewrite NVS. Our own key is excluded (SecurityConfig).
		 * Caller (apply_user) holds nodedb_lock, as warm_upsert requires. */
		if (key_changed && key_len == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN &&
		    node->num != meshtastic_get_node_id()) {
			warm_upsert_locked(node->num, node->public_key.bytes);
			nodekeys_schedule_save();
		}
#else
		ARG_UNUSED(key_changed);
#endif
	}

	WRITE_BIT(node->bitfield, NODEINFO_BITFIELD_HAS_USER_BIT, true);
	WRITE_BIT(node->bitfield, NODEINFO_BITFIELD_IS_LICENSED_BIT, user->is_licensed);
	WRITE_BIT(node->bitfield, NODEINFO_BITFIELD_HAS_IS_UNMESSAGABLE_BIT,
		  user->has_is_unmessagable);
	WRITE_BIT(node->bitfield, NODEINFO_BITFIELD_IS_UNMESSAGABLE_BIT,
		  user->has_is_unmessagable && user->is_unmessagable);
}

static struct nodedb_entry *find_entry_locked(uint32_t node_num)
{
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		if (nodedb_entries[i].used && nodedb_entries[i].node.num == node_num) {
			return &nodedb_entries[i];
		}
	}

	return NULL;
}

static size_t oldest_evictable_index_locked(void)
{
	uint32_t local = meshtastic_get_node_id();
	uint32_t oldest = UINT32_MAX;
	size_t oldest_index = SIZE_MAX;

	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		if (!nodedb_entries[i].used || nodedb_entries[i].node.num == local ||
		    IS_BIT_SET(nodedb_entries[i].node.bitfield,
			       NODEINFO_BITFIELD_IS_FAVORITE_BIT)) {
			continue;
		}

		if (nodedb_entries[i].node.last_heard < oldest) {
			oldest = nodedb_entries[i].node.last_heard;
			oldest_index = i;
		}
	}

	return oldest_index;
}

static struct nodedb_entry *get_or_create_entry_locked(uint32_t node_num)
{
	struct nodedb_entry *entry;
	size_t index;

	if (node_num == 0U) {
		return NULL;
	}

	entry = find_entry_locked(node_num);
	if (entry != NULL) {
		return entry;
	}

	if (nodedb_entry_count < ARRAY_SIZE(nodedb_entries)) {
		entry = &nodedb_entries[nodedb_entry_count++];
	} else {
		index = oldest_evictable_index_locked();
		if (index == SIZE_MAX) {
			return NULL;
		}

		entry = &nodedb_entries[index];
		LOG_DBG("NodeDB evicting 0x%08x", entry->node.num);
	}

	*entry = (struct nodedb_entry){0};
	entry->used = true;
	entry->node = (meshtastic_NodeInfoLite)meshtastic_NodeInfoLite_init_zero;
	entry->node.num = node_num;

	return entry;
}

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
/*
 * Warm key tier + peer public-key persistence. The warm tier is an in-RAM
 * {node id -> 32-byte X25519 pubkey} cache, sized larger than the hot store and
 * mirrored to NVS ("mtnode/<id>"). It decouples "peers we can PKC-encrypt to"
 * from the hot record cap: a key here stays usable after the peer's full record
 * is evicted from the hot store. Persisted keys are restored into this tier at
 * boot (not the hot store), so a large key set never thrashes the hot store.
 * Our own key is not stored here (it lives in the SecurityConfig). Guarded by
 * nodedb_lock: the "_locked" helpers require the caller to hold it.
 */
#define MTNODE_SUBTREE "mtnode"

struct warm_key {
	uint32_t num;       /* 0 == empty slot */
	uint32_t last_seen; /* uptime seconds; least-recently-seen evicted first */
	uint8_t pub[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
};

static struct warm_key warm_keys[CONFIG_MESHTASTIC_NODEDB_WARM_KEYS];

static struct warm_key *warm_find_locked(uint32_t num)
{
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == num) {
			return &warm_keys[i];
		}
	}
	return NULL;
}

static void warm_upsert_locked(uint32_t num, const uint8_t *pub)
{
	struct warm_key *slot;

	if (num == 0U) {
		return;
	}

	slot = warm_find_locked(num);
	if (slot == NULL) {
		/* Prefer an empty slot; otherwise evict the least-recently-seen. */
		slot = &warm_keys[0];
		for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
			if (warm_keys[i].num == 0U) {
				slot = &warm_keys[i];
				break;
			}
			if (warm_keys[i].last_seen < slot->last_seen) {
				slot = &warm_keys[i];
			}
		}
	}

	slot->num = num;
	slot->last_seen = uptime_seconds();
	memcpy(slot->pub, pub, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
}

static bool warm_copy_key_locked(uint32_t num, uint8_t out[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN])
{
	struct warm_key *slot = warm_find_locked(num);

	if (slot == NULL) {
		return false;
	}
	memcpy(out, slot->pub, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
	return true;
}

static void nodekeys_save_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	ret = settings_save_subtree(MTNODE_SUBTREE);
	if (ret < 0) {
		LOG_WRN("NodeDB key save failed (%d)", ret);
	}
}

static K_WORK_DELAYABLE_DEFINE(nodekeys_save_work, nodekeys_save_work_handler);

static void nodekeys_schedule_save(void)
{
	(void)k_work_reschedule(&nodekeys_save_work,
				K_MSEC(CONFIG_MESHTASTIC_SETTINGS_SAVE_DELAY_MS));
}

static int nodekeys_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
	uint32_t node_num;
	char *endptr;
	ssize_t read;

	if (len != sizeof(buf)) {
		LOG_WRN("Ignoring persisted node key '%s' with unexpected size %zu", key, len);
		return 0;
	}

	node_num = (uint32_t)strtoul(key, &endptr, 16);
	if (*endptr != '\0' || node_num == 0U) {
		return 0;
	}

	read = read_cb(cb_arg, buf, sizeof(buf));
	if (read != (ssize_t)sizeof(buf)) {
		return 0;
	}

	/* Restore into the warm tier, not the hot store: keeps the key reachable
	 * for PKC without occupying a hot record slot (avoids restore thrash). */
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	warm_upsert_locked(node_num, buf);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

static int nodekeys_export(int (*export_func)(const char *name, const void *val, size_t val_len))
{
	char name[SETTINGS_MAX_NAME_LEN + 1];
	uint32_t self = meshtastic_get_node_id();
	int ret = 0;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == 0U || warm_keys[i].num == self) {
			continue;
		}

		(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x", warm_keys[i].num);
		ret = export_func(name, warm_keys[i].pub, sizeof(warm_keys[i].pub));
		if (ret < 0) {
			break;
		}
	}
	k_mutex_unlock(&nodedb_lock);

	return ret;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtnode, MTNODE_SUBTREE, NULL, nodekeys_set, NULL, nodekeys_export);
#endif /* CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS */

static bool decode_user_payload(const struct meshtastic_packet *packet, meshtastic_User *user)
{
	pb_istream_t stream;

	if (packet->payload == NULL || packet->payload_len == 0U) {
		return false;
	}

	stream = pb_istream_from_buffer(packet->payload, packet->payload_len);
	if (!pb_decode(&stream, meshtastic_User_fields, user)) {
		LOG_DBG("NodeDB User decode failed: %s", PB_GET_ERROR(&stream));
		return false;
	}

	return true;
}

static bool packet_hops_away(const struct meshtastic_packet *packet, uint8_t *hops_away)
{
	if (packet->hop_start == 0U || packet->hop_start < packet->hop_limit) {
		return false;
	}

	*hops_away = packet->hop_start - packet->hop_limit;
	return true;
}

static void apply_basic_packet(struct nodedb_entry *entry, const struct meshtastic_packet *packet,
			       uint32_t now_sec)
{
	uint8_t hops_away;

	entry->node.last_heard = now_sec;
	entry->node.snr = (float)packet->snr;
	entry->node.channel = (packet->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID)
				      ? packet->channel_index
				      : 0U;
	entry->node.next_hop = packet->next_hop;
	WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_VIA_MQTT_BIT, packet->via_mqtt);

	if (packet_hops_away(packet, &hops_away)) {
		entry->node.has_hops_away = true;
		entry->node.hops_away = hops_away;
	}
}

static void meshtastic_module_nodedb_on_packet(const struct meshtastic_packet *packet)
{
	meshtastic_User user = meshtastic_User_init_zero;
	bool has_user = false;
	struct nodedb_entry *entry;
	uint32_t now_sec;

	if (packet == NULL || packet->from == 0U || packet->from == meshtastic_get_node_id()) {
		return;
	}

	/* Every packet refreshes the basic record (last_heard / snr / hops); only
	 * NodeInfo carries identity + pubkey. Position/telemetry/status are not
	 * retained (full-lean) — hearing them still updates last_heard via the
	 * basic-packet path below. */
	if (packet->portnum == MESHTASTIC_PORT_NODEINFO) {
		has_user = decode_user_payload(packet, &user);
	}

	now_sec = uptime_seconds();

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = get_or_create_entry_locked(packet->from);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return;
	}

	apply_basic_packet(entry, packet, now_sec);

	if (has_user) {
		apply_user(entry, &user);
	}

	k_mutex_unlock(&nodedb_lock);
}

MESHTASTIC_MODULE_DEFINE(nodedb, 0, MESHTASTIC_MODULE_ALL_PACKETS,
			 meshtastic_module_nodedb_on_packet, NULL);

static void fill_snapshot(const struct nodedb_entry *entry, struct meshtastic_nodedb_node *out)
{
	const meshtastic_NodeInfoLite *node = &entry->node;
	size_t key_len;

	*out = (struct meshtastic_nodedb_node){0};
	out->num = node->num;
	out->last_heard_uptime_sec = node->last_heard;
	out->snr = node->snr;
	out->channel = node->channel;
	out->next_hop = node->next_hop;
	out->via_mqtt = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_VIA_MQTT_BIT);
	out->has_hops_away = node->has_hops_away;
	out->hops_away = node->hops_away;
	out->is_favorite = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT);
	out->is_ignored = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_IGNORED_BIT);

	out->has_user = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_HAS_USER_BIT);
	copy_string(out->long_name, sizeof(out->long_name), node->long_name);
	copy_string(out->short_name, sizeof(out->short_name), node->short_name);
	out->hw_model = node->hw_model;
	out->role = node->role;
	out->is_licensed = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_LICENSED_BIT);
	out->has_is_unmessagable =
		IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_HAS_IS_UNMESSAGABLE_BIT);
	out->is_unmessagable = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_UNMESSAGABLE_BIT);

	key_len = MIN((size_t)node->public_key.size, sizeof(out->public_key));
	out->public_key_len = key_len;
	if (key_len > 0U) {
		memcpy(out->public_key, node->public_key.bytes, key_len);
	}
}

size_t meshtastic_nodedb_count(void)
{
	size_t count;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	count = nodedb_entry_count;
	k_mutex_unlock(&nodedb_lock);

	return count;
}

int meshtastic_nodedb_get(uint32_t node_num, struct meshtastic_nodedb_node *out)
{
	struct nodedb_entry *entry;

	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}

	fill_snapshot(entry, out);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

int meshtastic_nodedb_copy_pubkey(uint32_t node_num,
				  uint8_t out[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN])
{
	struct nodedb_entry *entry;
	int ret = -ENOENT;

	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry != NULL &&
	    entry->node.public_key.size == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN) {
		memcpy(out, entry->node.public_key.bytes,
		       MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
		ret = 0;
	}
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	else if (warm_copy_key_locked(node_num, out)) {
		ret = 0;
	}
#endif
	k_mutex_unlock(&nodedb_lock);

	return ret;
}

int meshtastic_nodedb_get_by_index(size_t index, struct meshtastic_nodedb_node *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	if (index >= nodedb_entry_count || !nodedb_entries[index].used) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}

	fill_snapshot(&nodedb_entries[index], out);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

static int nodedb_set_bit(uint32_t node_num, int bit, bool value)
{
	struct nodedb_entry *entry;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}

	WRITE_BIT(entry->node.bitfield, bit, value);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

int meshtastic_nodedb_set_favorite(uint32_t node_num, bool favorite)
{
	return nodedb_set_bit(node_num, NODEINFO_BITFIELD_IS_FAVORITE_BIT, favorite);
}

int meshtastic_nodedb_set_ignored(uint32_t node_num, bool ignored)
{
	return nodedb_set_bit(node_num, NODEINFO_BITFIELD_IS_IGNORED_BIT, ignored);
}

int meshtastic_nodedb_remove(uint32_t node_num)
{
	/* The local node is always present and must never be evicted or removed. */
	if (node_num == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		if (!nodedb_entries[i].used || nodedb_entries[i].node.num != node_num) {
			continue;
		}

		/* Preserve the "entries [0, count) are all used" invariant by
		 * swapping the last entry into the hole, then shrinking. */
		size_t last = nodedb_entry_count - 1U;

		if (i != last) {
			nodedb_entries[i] = nodedb_entries[last];
		}
		nodedb_entries[last] = (struct nodedb_entry){0};
		nodedb_entry_count--;
		k_mutex_unlock(&nodedb_lock);
		LOG_DBG("NodeDB removed 0x%08x", node_num);
		return 0;
	}
	k_mutex_unlock(&nodedb_lock);

	return -ENOENT;
}

int meshtastic_nodedb_init(void)
{
	meshtastic_User user;
	struct nodedb_entry *entry;

	meshtastic_fill_user(&user);

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	memset(nodedb_entries, 0, sizeof(nodedb_entries));
	nodedb_entry_count = 0U;

	entry = get_or_create_entry_locked(meshtastic_get_node_id());
	if (entry != NULL) {
		entry->node.last_heard = uptime_seconds();
		apply_user(entry, &user);
	}
	k_mutex_unlock(&nodedb_lock);

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	/* Restore persisted peer public keys now the array is initialised and the
	 * settings subsystem is up (settings_subsys_init ran earlier in
	 * meshtastic_init). Runs outside the lock: nodekeys_set() takes it. */
	(void)settings_load_subtree(MTNODE_SUBTREE);
#endif

	return (entry == NULL) ? -ENOMEM : 0;
}
