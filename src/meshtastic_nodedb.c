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
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>

#include <esp_attr.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_clock.h"
#include "meshtastic_modules.h"
#include "meshtastic_sched.h"
#if defined(CONFIG_MESHTASTIC_PKI)
#include "meshtastic_pki.h"
#endif

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
/* EXT_RAM_BSS_ATTR: no-op unless CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY (V4-only) —
 * places this table in PSRAM instead of internal DRAM. See PSRAM-NEXT-STEPS.md. */
static EXT_RAM_BSS_ATTR struct nodedb_entry nodedb_entries[CONFIG_MESHTASTIC_NODEDB_MAX_NODES];
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
	size_t key_len = MIN((size_t)user->public_key.size, sizeof(node->public_key.bytes));
	bool pinned = (node->public_key.size == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
	bool incoming_full = (key_len == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);

	/* Public-key pinning: once a peer's 32-byte key is known, a NodeInfo
	 * carrying a DIFFERENT key is treated as impersonation — refuse the whole
	 * identity update so a spoofer can neither replace the key nor rename the
	 * node (upstream NodeDB mismatch-drop). RX metadata already applied by
	 * apply_basic_packet (snr/last_heard/...) is unaffected. A legitimately
	 * re-keyed peer needs an operator remove (admin remove_by_nodenum) first. */
	if (pinned && incoming_full &&
	    memcmp(node->public_key.bytes, user->public_key.bytes, key_len) != 0) {
		LOG_WRN("NodeInfo for 0x%08x carries a different public key — dropped "
			"(possible impersonation)",
			(unsigned int)node->num);
		return;
	}

#if defined(CONFIG_MESHTASTIC_PKI)
	/* Someone advertising OUR node id with a key that is not our real public
	 * key is impersonating this node: never store it, and say so loudly. */
	if (node->num == meshtastic_get_node_id() && incoming_full) {
		uint8_t own[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];

		if (meshtastic_pki_get_public_key(own) == sizeof(own) &&
		    memcmp(own, user->public_key.bytes, sizeof(own)) != 0) {
			LOG_WRN("NodeInfo advertises our node id 0x%08x with a foreign "
				"public key — dropped (possible impersonation)",
				(unsigned int)node->num);
			return;
		}
	}
#endif

	copy_string(node->long_name, sizeof(node->long_name), user->long_name);
	copy_string(node->short_name, sizeof(node->short_name), user->short_name);
	node->hw_model = (uint8_t)user->hw_model;
	node->role = (uint8_t)user->role;

	/* Key write: an absent/short incoming key never wipes a pinned one (a
	 * keyless NodeInfo would otherwise downgrade the peer back to PSK DMs). */
	if (incoming_full || !pinned) {
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
	uint32_t last_seen; /* recency for LRU; wall-clock epoch once seeded (see warm_now) */
	uint8_t pub[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
};

/* Persisted record: last_seen (LE32) + public key. Restoring recency keeps warm
 * LRU meaningful across reboots. Legacy records are key-only (32 B) — see set. */
#define MTNODE_REC_LEN (sizeof(uint32_t) + MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN)

static EXT_RAM_BSS_ATTR struct warm_key warm_keys[CONFIG_MESHTASTIC_NODEDB_WARM_KEYS];

/* Set when a warm eviction (or boot) may have orphaned an NVS record. The
 * save-work handler then prunes any persisted mtnode/<id> whose node is no
 * longer in the warm ring, so the durable store stays == the RAM ring
 * (bounded; no unbounded NVS growth, no boot-load thrash). */
static bool nodekeys_reconcile;

/* Recency stamp for warm entries: wall-clock epoch when the SNTP/GNSS clock is
 * seeded, else a small uptime-relative value. Epoch values (post-sync, and those
 * restored from NVS) always outrank pre-sync uptime stamps, so a persisted key's
 * true recency wins the LRU over a freshly-heard-but-unsynced boot window. */
static uint32_t warm_now(void)
{
	uint32_t epoch = meshtastic_clock_now_epoch();

	return (epoch != 0U) ? epoch : uptime_seconds();
}

static struct warm_key *warm_find_locked(uint32_t num)
{
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == num) {
			return &warm_keys[i];
		}
	}
	return NULL;
}

/* Slot to (re)write for @num: its existing slot, else an empty slot, else a
 * least-recently-seen entry to evict — but keys whose node is still active in
 * the hot store (favorites are always hot-resident) are protected, so an active
 * conversation never loses its PKC key to a burst of new nodes. Only if every
 * entry is protected (warm smaller than the hot store) do we fall back to the
 * global LRU, so a store always succeeds. */
static struct warm_key *warm_slot_for_locked(uint32_t num)
{
	struct warm_key *slot = warm_find_locked(num);
	struct warm_key *victim = NULL;   /* LRU among evictable (hot-absent) keys */
	struct warm_key *fallback = NULL; /* global LRU, used only if all protected */

	if (slot != NULL) {
		return slot;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == 0U) {
			return &warm_keys[i];
		}
		if (fallback == NULL || warm_keys[i].last_seen < fallback->last_seen) {
			fallback = &warm_keys[i];
		}
		if (find_entry_locked(warm_keys[i].num) != NULL) {
			continue; /* protected: node still active in the hot store */
		}
		if (victim == NULL || warm_keys[i].last_seen < victim->last_seen) {
			victim = &warm_keys[i];
		}
	}

	return (victim != NULL) ? victim : fallback;
}

static void warm_place_locked(uint32_t num, const uint8_t *pub, uint32_t last_seen)
{
	struct warm_key *slot;

	if (num == 0U) {
		return;
	}

	slot = warm_slot_for_locked(num);
	if (slot->num != 0U && slot->num != num) {
		/* Overwriting a different node evicts it; its NVS record is now an
		 * orphan. Flag a reconcile so the next save prunes it. */
		nodekeys_reconcile = true;
	}
	slot->num = num;
	slot->last_seen = last_seen;
	memcpy(slot->pub, pub, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
}

static void warm_upsert_locked(uint32_t num, const uint8_t *pub)
{
	warm_place_locked(num, pub, warm_now());
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

/* Max orphaned NVS records pruned per reconcile pass. A larger backlog (e.g. a
 * device upgrading from the pre-bounded store) is drained across successive
 * saves — see the overflow reschedule below. */
#define WARM_RECONCILE_BATCH 32U

struct warm_reconcile_ctx {
	uint32_t orphans[WARM_RECONCILE_BATCH];
	size_t count;
	bool overflow;
};

/* Direct-load callback over the mtnode subtree: collect ids that are NOT in the
 * warm ring (orphans to prune). Robust to the key arriving as "mtnode/<id>" or
 * bare "<id>" — parse the last path component. */
static int warm_reconcile_cb(const char *key, size_t len, settings_read_cb read_cb,
			     void *cb_arg, void *param)
{
	struct warm_reconcile_ctx *ctx = param;
	const char *id = strrchr(key, '/');
	uint32_t num;
	char *endptr;
	bool orphan;

	ARG_UNUSED(len);
	ARG_UNUSED(read_cb);
	ARG_UNUSED(cb_arg);

	id = (id != NULL) ? id + 1 : key;
	num = (uint32_t)strtoul(id, &endptr, 16);
	if (*endptr != '\0' || num == 0U) {
		return 0;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	orphan = (warm_find_locked(num) == NULL);
	k_mutex_unlock(&nodedb_lock);

	if (!orphan) {
		return 0;
	}
	if (ctx->count < ARRAY_SIZE(ctx->orphans)) {
		ctx->orphans[ctx->count++] = num;
	} else {
		ctx->overflow = true;
	}
	return 0;
}

static void nodekeys_schedule_save(void);

/* Prune orphaned mtnode/<id> NVS records (any not in the warm ring) then persist
 * the ring. Shared by the delayed save-work and the synchronous reset path. */
static void nodekeys_do_persist(void)
{
	int ret;

	if (nodekeys_reconcile) {
		struct warm_reconcile_ctx ctx = {.count = 0U, .overflow = false};
		char name[SETTINGS_MAX_NAME_LEN + 1];

		nodekeys_reconcile = false;
		(void)settings_load_subtree_direct(MTNODE_SUBTREE, warm_reconcile_cb, &ctx);

		for (size_t i = 0U; i < ctx.count; i++) {
			(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x",
				       ctx.orphans[i]);
			(void)settings_delete(name);
		}
		if (ctx.count > 0U) {
			LOG_DBG("NodeDB pruned %zu orphaned key(s) from NVS", ctx.count);
		}
		if (ctx.overflow) {
			nodekeys_reconcile = true; /* more to prune next pass */
		}
	}

	ret = settings_save_subtree(MTNODE_SUBTREE);
	if (ret < 0) {
		LOG_WRN("NodeDB key save failed (%d)", ret);
	}

	if (nodekeys_reconcile) {
		nodekeys_schedule_save();
	}
}

static void nodekeys_save_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	nodekeys_do_persist();
}

static K_WORK_DELAYABLE_DEFINE(nodekeys_save_work, nodekeys_save_work_handler);

static void nodekeys_schedule_save(void)
{
	(void)k_work_reschedule(&nodekeys_save_work,
				K_MSEC(CONFIG_MESHTASTIC_SETTINGS_SAVE_DELAY_MS));
}

static int nodekeys_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MTNODE_REC_LEN];
	uint32_t node_num;
	uint32_t last_seen;
	const uint8_t *pub;
	char *endptr;
	ssize_t read;

	if (len == MTNODE_REC_LEN) {
		/* Current format: last_seen (LE32) + public key. */
		read = read_cb(cb_arg, buf, MTNODE_REC_LEN);
		if (read != (ssize_t)MTNODE_REC_LEN) {
			return 0;
		}
		last_seen = sys_get_le32(buf);
		pub = buf + sizeof(uint32_t);
	} else if (len == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN) {
		/* Legacy key-only record (pre-recency): restore the key, but rank it
		 * oldest (last_seen 0) so it's evicted first and re-stamped on save. */
		read = read_cb(cb_arg, buf, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
		if (read != (ssize_t)MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN) {
			return 0;
		}
		last_seen = 0U;
		pub = buf;
	} else {
		LOG_WRN("Ignoring persisted node key '%s' with unexpected size %zu", key, len);
		return 0;
	}

	node_num = (uint32_t)strtoul(key, &endptr, 16);
	if (*endptr != '\0' || node_num == 0U) {
		return 0;
	}

	/* Restore into the warm tier, not the hot store: keeps the key reachable
	 * for PKC without occupying a hot record slot (avoids restore thrash). */
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	warm_place_locked(node_num, pub, last_seen);
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

		uint8_t rec[MTNODE_REC_LEN];

		sys_put_le32(warm_keys[i].last_seen, rec);
		memcpy(rec + sizeof(uint32_t), warm_keys[i].pub,
		       MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);

		(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x", warm_keys[i].num);
		ret = export_func(name, rec, sizeof(rec));
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
	/* node.next_hop is the *learned route to reach this node* (next-hop router),
	 * set from ACK/relay correlation — NOT the packet's outbound next_hop hint.
	 * Don't overwrite it with the incoming wire field. */
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

/* Read-time route health (M4, upstream RouteHealth): freshness + failure
 * tracking for learned next hops, consulted by get_next_hop so a stale or
 * repeatedly failing route decays back to flood instead of being trusted on
 * the first (slowest) attempt of each DM. RAM-only, advisory: a route without
 * a record (ring-evicted, or planted before tracking) stays trusted and only
 * decays through failures. All access under nodedb_lock. */
#define ROUTE_HEALTH_SIZE     16U
#define ROUTE_HEALTH_MAX_FAIL 3U

struct route_health {
	uint32_t dest;       /* 0 = empty slot */
	uint32_t learned_at; /* uptime seconds at learn / last confirmed delivery */
	uint8_t fail_count;  /* consecutive reliable-exhaustion strikes */
};

static struct route_health route_health[ROUTE_HEALTH_SIZE];

static struct route_health *route_health_find_locked(uint32_t dest)
{
	for (size_t i = 0U; i < ARRAY_SIZE(route_health); i++) {
		if (route_health[i].dest == dest) {
			return &route_health[i];
		}
	}

	return NULL;
}

static void route_health_upsert_locked(uint32_t dest)
{
	struct route_health *rh = route_health_find_locked(dest);

	if (rh == NULL) {
		/* Prefer an empty slot, else evict the oldest record. */
		rh = &route_health[0];
		for (size_t i = 0U; i < ARRAY_SIZE(route_health); i++) {
			if (route_health[i].dest == 0U) {
				rh = &route_health[i];
				break;
			}
			if (route_health[i].learned_at < rh->learned_at) {
				rh = &route_health[i];
			}
		}
	}

	rh->dest = dest;
	rh->learned_at = uptime_seconds();
	rh->fail_count = 0U;
}

static void route_health_drop_locked(uint32_t dest)
{
	struct route_health *rh = route_health_find_locked(dest);

	if (rh != NULL) {
		*rh = (struct route_health){0};
	}
}

void meshtastic_nodedb_note_route_failure(uint32_t dest)
{
	struct nodedb_entry *entry;
	struct route_health *rh;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	rh = route_health_find_locked(dest);
	if (rh != NULL && rh->fail_count < UINT8_MAX) {
		rh->fail_count++;
	}
	if (rh == NULL || rh->fail_count >= ROUTE_HEALTH_MAX_FAIL) {
		/* Untracked route, or three strikes: back to flood so the next
		 * send rediscovers a working path (self-healing). */
		entry = find_entry_locked(dest);
		if (entry != NULL && entry->node.next_hop != 0U) {
			LOG_DBG("route health: next_hop(0x%08x) decayed (failures)",
				(unsigned int)dest);
			entry->node.next_hop = 0U;
		}
		route_health_drop_locked(dest);
	}
	k_mutex_unlock(&nodedb_lock);
}

void meshtastic_nodedb_note_route_success(uint32_t dest)
{
	struct route_health *rh;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	rh = route_health_find_locked(dest);
	if (rh != NULL) {
		rh->fail_count = 0U;
		rh->learned_at = uptime_seconds();
	}
	k_mutex_unlock(&nodedb_lock);
}

/* Next-hop routing support (Increment 1: foundation). The on-wire next_hop /
 * relay_node fields are only the *last byte* of a node number, so a byte can be
 * ambiguous on a large mesh — resolve one back to a node only when exactly one
 * known node (never self) matches, else return 0 (caller falls back to flood). */
uint32_t meshtastic_nodedb_resolve_unique_last_byte(uint8_t last_byte)
{
	uint32_t local = meshtastic_get_node_id();
	uint32_t match = 0U;
	size_t count = 0U;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		uint32_t num;

		if (!nodedb_entries[i].used) {
			continue;
		}
		num = nodedb_entries[i].node.num;
		if (num == local) {
			continue;
		}
		if ((uint8_t)(num & 0xFFU) == last_byte) {
			match = num;
			if (++count > 1U) {
				break;
			}
		}
	}
	k_mutex_unlock(&nodedb_lock);

	return (count == 1U) ? match : 0U;
}

uint8_t meshtastic_nodedb_get_next_hop(uint32_t dest)
{
	struct nodedb_entry *entry;
	uint8_t next_hop = 0U;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(dest);
	if (entry != NULL) {
		next_hop = entry->node.next_hop;
	}

	/* Read-time decay (M4): a tracked route that is stale (past route.ttl)
	 * or has struck out is cleared and the caller floods. A single aligned
	 * scalar read of the TTL is atomic (see meshtastic_sched.h). */
	if (next_hop != 0U) {
		struct route_health *rh = route_health_find_locked(dest);
		uint16_t ttl = meshtastic_sched_get()->route_ttl_sec;

		if (rh != NULL &&
		    (rh->fail_count >= ROUTE_HEALTH_MAX_FAIL ||
		     (ttl != 0U && (uptime_seconds() - rh->learned_at) > (uint32_t)ttl))) {
			LOG_DBG("route health: next_hop(0x%08x) decayed (stale)",
				(unsigned int)dest);
			entry->node.next_hop = 0U;
			route_health_drop_locked(dest);
			next_hop = 0U;
		}
	}
	k_mutex_unlock(&nodedb_lock);

	return next_hop;
}

int meshtastic_nodedb_set_next_hop(uint32_t dest, uint8_t next_hop)
{
	struct nodedb_entry *entry;
	int ret = -ENOENT;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(dest);
	if (entry != NULL) {
		entry->node.next_hop = next_hop;
		if (next_hop != 0U) {
			route_health_upsert_locked(dest);
		} else {
			route_health_drop_locked(dest);
		}
		ret = 0;
	}
	k_mutex_unlock(&nodedb_lock);

	return ret;
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

size_t meshtastic_nodedb_warm_count(void)
{
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	size_t n = 0U;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num != 0U) {
			n++;
		}
	}
	k_mutex_unlock(&nodedb_lock);
	return n;
#else
	return 0U;
#endif
}

int meshtastic_nodedb_warm_get(size_t index, uint32_t *num, uint32_t *last_seen)
{
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	int ret = -ENOENT;
	size_t seen = 0U;

	if (num == NULL || last_seen == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == 0U) {
			continue;
		}
		if (seen == index) {
			*num = warm_keys[i].num;
			*last_seen = warm_keys[i].last_seen;
			ret = 0;
			break;
		}
		seen++;
	}
	k_mutex_unlock(&nodedb_lock);
	return ret;
#else
	ARG_UNUSED(index);
	ARG_UNUSED(num);
	ARG_UNUSED(last_seen);
	return -ENOTSUP;
#endif
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

bool meshtastic_nodedb_is_ignored(uint32_t node_num)
{
	struct nodedb_entry *entry;
	bool ignored = false;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry != NULL) {
		ignored = IS_BIT_SET(entry->node.bitfield, NODEINFO_BITFIELD_IS_IGNORED_BIT);
	}
	k_mutex_unlock(&nodedb_lock);

	return ignored;
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
		route_health_drop_locked(node_num);
		k_mutex_unlock(&nodedb_lock);
		LOG_DBG("NodeDB removed 0x%08x", node_num);
		return 0;
	}
	k_mutex_unlock(&nodedb_lock);

	return -ENOENT;
}

void meshtastic_nodedb_reset(bool keep_favorites)
{
	uint32_t self = meshtastic_get_node_id();

	k_mutex_lock(&nodedb_lock, K_FOREVER);

	/* Compact the hot store in place, keeping self and (optionally) favorites.
	 * Mirrors NodeDB::resetNodes: self is never removed; keep_favorites spares
	 * favorited peers, otherwise every peer goes. */
	size_t kept = 0U;
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		struct nodedb_entry *e = &nodedb_entries[i];
		bool keep;

		if (!e->used) {
			continue;
		}
		keep = (e->node.num == self) ||
		       (keep_favorites &&
			IS_BIT_SET(e->node.bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT));
		if (keep) {
			if (kept != i) {
				nodedb_entries[kept] = *e;
			}
			kept++;
		}
	}
	for (size_t i = kept; i < nodedb_entry_count; i++) {
		nodedb_entries[i] = (struct nodedb_entry){0};
	}
	nodedb_entry_count = kept;

	/* Route-health records are advisory and cheap to relearn; drop them all.
	 * A kept favorite's route simply becomes untracked (still trusted). */
	memset(route_health, 0, sizeof(route_health));

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	/* Warm keys are never favorites (self's key lives in config/security), so a
	 * DB reset clears them wholesale and flags a reconcile to prune the persisted
	 * mtnode records. */
	memset(warm_keys, 0, sizeof(warm_keys));
	nodekeys_reconcile = true;
#endif

	k_mutex_unlock(&nodedb_lock);

	LOG_INF("NodeDB reset (%s favorites): %zu node(s) retained",
		keep_favorites ? "keeping" : "removing", kept);

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	/* Persist synchronously: the reset reboot path does not flush the mtnode
	 * subtree, so prune the now-orphaned records and save the empty ring here.
	 * Runs outside the lock (the reconcile callback takes it). */
	(void)k_work_cancel_delayable(&nodekeys_save_work);
	nodekeys_do_persist();
#endif
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

	/* Prune any NVS records that didn't fit the warm ring on restore (or that a
	 * pre-bounded build left behind), so the durable store converges to the RAM
	 * ring. Deferred to the save-work so the flash writes happen off the boot
	 * path. */
	nodekeys_reconcile = true;
	nodekeys_schedule_save();
#endif

	return (entry == NULL) ? -ENOMEM : 0;
}
