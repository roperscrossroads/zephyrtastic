/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_lockdown.h"
#include "meshtastic_ext_ram.h"

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_modules.h"
#include "meshtastic_sched.h"
#include "meshtastic_storage.h" /* declarations only; safe unconditionally */
#if defined(CONFIG_MESHTASTIC_BULK_STORE) && defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
/* The warm key tier lives in the bulk store too (agents-2dk3 Phase 3d). */
#define NODEDB_WBULK 1
#endif
#if defined(CONFIG_MESHTASTIC_BULK_STORE) && defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
/* Node records live in the bulk store (agents-2dk3 Phase 3), not one settings
 * record per node. The settings path stays for images without the partition, and
 * to import the records an older image left behind. */
#define NODEDB_BULK 1
#endif
#if defined(NODEDB_BULK) || defined(NODEDB_WBULK)
#include <zephyr/sys/crc.h>
#include "meshtastic_bulk.h"
#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
#include <zephyr/dfu/mcuboot.h>
#endif
/* One page buffer for both bulk tables (node records and warm keys), because on the
 * XIAO (no PSRAM) a second 2 KB buffer is real RAM. What makes sharing it safe is
 * that both tables take nodedb_page_lock for the whole build-and-write or
 * read-and-apply of a page. The mutex is recursive, so a future path that held it
 * mid-page and called into the OTHER table would not deadlock -- it would silently
 * clobber the page. Keep each table's page work self-contained. PSRAM on ESP32:
 * CPU-only; the bulk engine copies a page into its own internal-RAM frame before
 * any flash write. */
static MESHTASTIC_EXT_RAM_BSS_ATTR uint8_t nodedb_page_buf[MESHTASTIC_BULK_BLOB_MAX];
static K_MUTEX_DEFINE(nodedb_page_lock); /* the buffer and both tables' state; nodedb_lock nests inside */
#endif
#if defined(CONFIG_MESHTASTIC_PKI)
#include "meshtastic_pki.h"
#include "meshtastic_phoneapi.h"
#endif

#include "meshtastic/deviceonly.pb.h"
#include "meshtastic/mesh.pb.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/*
 * NodeInfoLite.bitfield bit indices — the reference firmware's layout
 * (firmware/src/mesh/NodeDB.h, NODEINFO_BITFIELD_*_SHIFT), reproduced exactly.
 *
 * Corrected 2026-08-19: IS_FAVORITE and IS_IGNORED used to sit at bits 0 and 1
 * here, matching an older reference revision. The reference has since inserted
 * IS_KEY_MANUALLY_VERIFIED and IS_MUTED at 0/1 and moved favorite/ignored to
 * 3/4, so this firmware's "favorite" was the reference's "key manually
 * verified" and its "ignored" was "muted". Bits 2 and 5-8 always agreed.
 *
 * The bitfield IS persisted — mtrec_durable_copy() keeps it (clearing only
 * VIA_MQTT), so stored records written under the old layout would read back
 * with "favorite" meaning "key manually verified" and "ignored" meaning
 * "muted". MTREC_RECORD_VERSION is bumped alongside this change so those
 * records are rejected at load and re-learned, rather than silently
 * reinterpreted. (Do not confuse the two node subtrees: mtnode holds only
 * last_seen + role + public key and is unaffected; mtrec holds the full
 * NodeInfoLite.)
 *
 * The unused-here bits are still defined, so nobody claims one for something
 * else and reintroduces exactly the divergence this comment records.
 */
#define NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT 0 /* key verification (dnr4.13) */
#define NODEINFO_BITFIELD_IS_MUTED_BIT                 1 /* admin toggle_muted_node */
#define NODEINFO_BITFIELD_VIA_MQTT_BIT                 2
#define NODEINFO_BITFIELD_IS_FAVORITE_BIT              3
#define NODEINFO_BITFIELD_IS_IGNORED_BIT               4
#define NODEINFO_BITFIELD_HAS_USER_BIT                 5
#define NODEINFO_BITFIELD_IS_LICENSED_BIT              6
#define NODEINFO_BITFIELD_IS_UNMESSAGABLE_BIT          7
#define NODEINFO_BITFIELD_HAS_IS_UNMESSAGABLE_BIT      8
#define NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_BIT        9  /* agents-ooma.32; set only via
							    * meshtastic_nodedb_note_xeddsa_signer(),
							    * never trusted from inbound data */
#define NODEINFO_BITFIELD_HAS_SNR_BIT                  10 /* not yet implemented */

/* Mostly-lean: the DB retains the NodeInfoLite core (identity + pubkey) plus a
 * last-known position per node (agents-ooma.39). Device+environment telemetry
 * / status are still report-and-forget for now -- NOT "as in the reference
 * firmware" (that claim was checked against upstream and found wrong: NodeDB.cpp's
 * nodePositions/nodeTelemetry/nodeEnvironment/nodeStatus maps are all cached by
 * default except on ARCH_STM32WL, which this port never targets). Position
 * shipped first because a freshly-connected phone's map view is the visible
 * symptom; telemetry/status caching is a natural follow-up, not done here. */
struct nodedb_entry {
	bool used;
	meshtastic_NodeInfoLite node;
	/* Persisted last-heard epoch, carried across reboot for a restored node that
	 * has not yet been re-heard this boot (node.last_heard is uptime-relative and
	 * resets to 0 on restore). 0 for nodes heard this boot — their epoch derives
	 * from node.last_heard via the clock. */
	uint32_t last_heard_epoch;
	/* Last-known position, mirroring upstream's nodePositions cache (see the
	 * comment above). Capped 1:1 with nodedb_entries -- a node's position lives
	 * and dies with its NodeDB entry, unlike upstream's independently-sized and
	 * independently-evicted satellite map. */
	bool has_position;
	meshtastic_PositionLite position;
};

static K_MUTEX_DEFINE(nodedb_lock);
/* MESHTASTIC_EXT_RAM_BSS_ATTR: no-op unless CONFIG_ESP_SPIRAM (V4 family only) — places this
 * table in PSRAM instead of internal DRAM. Rules for what may live there:
 * src/meshtastic_ext_ram.h; budget and remaining levers: docs/memory-savings.md. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct nodedb_entry nodedb_entries[CONFIG_MESHTASTIC_NODEDB_MAX_NODES];
static size_t nodedb_entry_count;

/* Node-list sort (B-8): the hot store is re-sorted lazily on read, throttled, and only when
 * something that affects order changed — so a read burst (a phone config handshake or one
 * display frame) sees a stable order, and a busy mesh doesn't sort on every packet. */
static bool nodedb_dirty;
static int64_t nodedb_last_sort_ms;

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
static void nodekeys_schedule_save(void);
#if defined(NODEDB_WBULK)
static bool wrec_active(void);
static void wrec_do_persist(void);
static int wrec_boot_restore(void);
static void wrec_rewrite_hook(void);
#endif
static void warm_upsert_locked(uint32_t num, const uint8_t *pub, uint8_t role);
static bool warm_copy_key_locked(uint32_t num, uint8_t out[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN]);
static bool warm_get_role_locked(uint32_t num, uint8_t *role);
static bool warm_get_known_signer_locked(uint32_t num);
static void warm_set_known_signer_locked(uint32_t num, bool signer);
static void warm_absorb_locked(const struct nodedb_entry *entry);
#endif

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
static void mtrec_schedule_save(void);
static void mtrec_note_evicted(void);
#if defined(NODEDB_BULK)
static bool nrec_active(void);
static void nrec_do_persist(void);
#endif
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

#if defined(CONFIG_MESHTASTIC_PKI)
/* agents-ooma.21 -- a peer advertising OUR public key under a different node id.
 *
 * Nobody but us should hold our key. A NodeInfo carrying it from another id means the key
 * was copied (a cloned flash, a restored backup on a second board), or two boards minted the
 * same key -- which is exactly the failure the ESP32-S3 first-boot entropy question
 * (agents-0lzm.10) is about, so this is the detector for it. It is also what an echo of our
 * OWN old NodeInfo looks like after a node-id migration, which is why it matters now.
 *
 * Mirrors the reference (NodeDB::updateUser): refuse the identity update, and warn the user
 * ONCE per boot -- a latch that is never reset -- with the sender's name made safe to embed.
 *
 * The notification is raised under nodedb_lock but delivered after it is released: the
 * PhoneAPI's config stream walks the NodeDB, so enqueueing to the phone while holding this
 * lock would invert that order. The alert is taken once, so a static buffer cannot race. */
static struct {
	bool warned;
	bool pending;
	char name[sizeof(((meshtastic_User *)0)->long_name)];
} own_key_alert;

/* The reference's sanitizeUtf8 (meshUtils.cpp), ported: replace every byte that does not begin
 * a valid UTF-8 sequence -- a stray continuation byte, a sequence cut short by truncation, an
 * overlong form, a surrogate half, anything past U+10FFFF -- with '?'. A name truncated to fit
 * a buffer can end mid-character, and a phone-side protobuf decoder that validates UTF-8 would
 * then reject the whole frame carrying the warning. */
static void sanitize_utf8(char *buf, size_t size)
{
	size_t i = 0U;
	size_t len;

	if (buf == NULL || size == 0U) {
		return;
	}
	buf[size - 1U] = '\0';
	len = strlen(buf);

	while (i < len) {
		uint8_t b = (uint8_t)buf[i];
		size_t seq;
		uint32_t min_cp;
		uint32_t cp;
		bool valid = true;

		if (b <= 0x7FU) {
			i++;
			continue;
		} else if ((b & 0xE0U) == 0xC0U) {
			seq = 2U;
			min_cp = 0x80U;
			cp = b & 0x1FU;
		} else if ((b & 0xF0U) == 0xE0U) {
			seq = 3U;
			min_cp = 0x800U;
			cp = b & 0x0FU;
		} else if ((b & 0xF8U) == 0xF0U) {
			seq = 4U;
			min_cp = 0x10000U;
			cp = b & 0x07U;
		} else {
			buf[i++] = '?';
			continue;
		}

		if (i + seq > len) {
			for (size_t j = i; j < len; j++) {
				buf[j] = '?';
			}
			break;
		}
		for (size_t j = 1U; j < seq; j++) {
			uint8_t c = (uint8_t)buf[i + j];

			if ((c & 0xC0U) != 0x80U) {
				valid = false;
				break;
			}
			cp = (cp << 6) | (c & 0x3FU);
		}
		if (valid && (cp < min_cp || cp > 0x10FFFFU || (cp >= 0xD800U && cp <= 0xDFFFU))) {
			valid = false;
		}
		if (valid) {
			i += seq;
		} else {
			/* Only the lead byte; its continuations are caught on the next pass. */
			buf[i++] = '?';
		}
	}
}

/* Take a raised alert, under nodedb_lock. */
static bool own_key_alert_take_locked(char *name, size_t cap)
{
	if (!own_key_alert.pending) {
		return false;
	}
	own_key_alert.pending = false;
	copy_string(name, cap, own_key_alert.name);
	return true;
}

/* Deliver a taken alert, WITHOUT nodedb_lock held. Same wording as the reference. */
static void own_key_alert_deliver(const char *name)
{
	static meshtastic_ClientNotification cn;

	LOG_WRN("Remote device %s has advertised your public key. This may indicate a "
		"compromised key. You may need to regenerate your public keys.", name);
	cn = (meshtastic_ClientNotification)meshtastic_ClientNotification_init_zero;
	cn.level = meshtastic_LogRecord_Level_WARNING;
	(void)snprintf(cn.message, sizeof(cn.message),
		       "Remote device %s has advertised your public key. This may indicate a "
		       "compromised key. You may need to regenerate your public keys.",
		       name);
	(void)meshtastic_phoneapi_enqueue_client_notification(&cn);
}
#endif /* CONFIG_MESHTASTIC_PKI */

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
	if (incoming_full) {
		uint8_t own[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
		bool have_own = (meshtastic_pki_get_public_key(own) == sizeof(own));
		bool ours = have_own && memcmp(own, user->public_key.bytes, sizeof(own)) == 0;

		/* Someone advertising OUR node id with a key that is not our real public
		 * key is impersonating this node: never store it, and say so loudly. */
		if (node->num == meshtastic_get_node_id() && have_own && !ours) {
			LOG_WRN("NodeInfo advertises our node id 0x%08x with a foreign "
				"public key — dropped (possible impersonation)",
				(unsigned int)node->num);
			return;
		}

		/* The converse: OUR key under someone else's id (see own_key_alert). */
		if (node->num != meshtastic_get_node_id() && ours) {
			if (!own_key_alert.warned) {
				own_key_alert.warned = true;
				own_key_alert.pending = true;
				copy_string(own_key_alert.name, sizeof(own_key_alert.name),
					    user->long_name);
				sanitize_utf8(own_key_alert.name, sizeof(own_key_alert.name));
			}
			LOG_DBG("NodeInfo from 0x%08x carries our public key — dropped",
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
			warm_upsert_locked(node->num, node->public_key.bytes, (uint8_t)node->role);
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

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* A curated (favorite/ignored) node's identity changed — re-persist its
	 * record. Volatile fields are excluded from the record, so this rewrites
	 * NVS only on a genuine identity change, not on every heard packet. */
	if (IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT) ||
	    IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_IGNORED_BIT)) {
		mtrec_schedule_save();
	}
#endif
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

/* Reserve at least this many evictable (non-protected) slots so a NodeDB
 * saturated with favorites/ignored nodes can still learn new peers.
 * Mirrors the reference firmware's MAX_NUM_NODES-2 protected cap. */
#define NODEDB_PROTECTED_RESERVE 2U

/* Protected from eviction: favorited (operator-pinned) or ignored (a block must
 * outlast churn). The reference NodeDB skips both; the local node is handled
 * separately by callers. */
static bool node_is_protected(const meshtastic_NodeInfoLite *node)
{
	return IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT) ||
	       IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_IGNORED_BIT);
}

static size_t protected_count_locked(void)
{
	size_t n = 0U;

	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		if (nodedb_entries[i].used && node_is_protected(&nodedb_entries[i].node)) {
			n++;
		}
	}

	return n;
}

static size_t oldest_evictable_index_locked(void)
{
	uint32_t local = meshtastic_get_node_id();
	uint32_t oldest = UINT32_MAX; /* oldest keyed (PKC-capable) candidate */
	size_t oldest_index = SIZE_MAX;
	uint32_t oldest_boring = UINT32_MAX; /* oldest keyless ("boring") candidate */
	size_t oldest_boring_index = SIZE_MAX;
	uint32_t oldest_verified = UINT32_MAX; /* oldest manually verified candidate */
	size_t oldest_verified_index = SIZE_MAX;

	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		const meshtastic_NodeInfoLite *node = &nodedb_entries[i].node;

		/* Skip favorites *and* ignored nodes — both are protected from eviction. */
		if (!nodedb_entries[i].used || node->num == local || node_is_protected(node)) {
			continue;
		}

		/* Three tiers, oldest first within each: keyless "boring" nodes, then
		 * keyed peers, then manually verified peers. A keyed peer goes only when
		 * no keyless victim exists, so PKC reach survives churn (the reference
		 * oldestBoring split); a verified peer goes last of all (the reference
		 * protects key_manually_verified outright; here it is the final tier, so
		 * a table of verified peers can still learn a new node). An evicted keyed
		 * peer's key moves to the warm tier (get_or_create_entry_locked). */
		if (node->public_key.size == 0U) {
			if (node->last_heard < oldest_boring) {
				oldest_boring = node->last_heard;
				oldest_boring_index = i;
			}
		} else if (IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT)) {
			if (node->last_heard < oldest_verified) {
				oldest_verified = node->last_heard;
				oldest_verified_index = i;
			}
		} else if (node->last_heard < oldest) {
			oldest = node->last_heard;
			oldest_index = i;
		}
	}

	if (oldest_boring_index != SIZE_MAX) {
		return oldest_boring_index;
	}
	return (oldest_index != SIZE_MAX) ? oldest_index : oldest_verified_index;
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
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
		warm_absorb_locked(entry);
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
		/* The evicted node's persisted record (if any) is now an orphan —
		 * flag a reconcile so the next snapshot prunes it (NVS == hot store). */
		mtrec_note_evicted();
#endif
	}

	*entry = (struct nodedb_entry){0};
	entry->used = true;
	entry->node = (meshtastic_NodeInfoLite)meshtastic_NodeInfoLite_init_zero;
	entry->node.num = node_num;
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	{
		/* B-5: a peer evicted to the warm tier and now re-admitted keeps its role
		 * instead of dropping to CLIENT (a later NodeInfo still overwrites it with the
		 * authoritative value). */
		uint8_t warm_role;

		if (warm_get_role_locked(node_num, &warm_role)) {
			entry->node.role = warm_role;
		}
		/* agents-ooma.32: restore the XEdDSA-signed bit too -- it is learned from
		 * verified traffic, not from NodeInfo, so a round trip through the warm tier
		 * must not relearn it from zero (that is the exact trap this bead closes). */
		if (warm_get_known_signer_locked(node_num)) {
			WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_BIT, 1);
		}
	}
#endif

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
	uint8_t role;       /* NodeInfoLite role — carried so an evicted->readmitted peer keeps it (B-5) */
	uint8_t known_signer; /* 0/1 — learned from a verified XEdDSA signature, carried the same way
			       * so BALANCED's downgrade protection is not forgotten on eviction
			       * (agents-ooma.32). Reset to 0 whenever the key itself is (re)placed —
			       * a rotated key has proven nothing yet under its new identity. */
	uint8_t pub[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
};

/* Persisted record: last_seen (LE32) + role (u8) + known_signer (u8) + public key. Restoring
 * recency keeps warm LRU meaningful across reboots; role and known_signer let an
 * evicted->readmitted peer keep them (B-5; known_signer is agents-ooma.32). Only the
 * current 38 B format is read; a differently-sized record is a stale format left by an older
 * build. Pre-1.0 policy is wipe-and-re-learn, not a migration branch — the one-shot
 * CONFIG_MESHTASTIC_NODEDB_PURGE_FOREIGN_KEYS deletes any such record from NVS. */
#define MTNODE_REC_LEN (sizeof(uint32_t) + 1U + 1U + MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN) /* 38 */

static MESHTASTIC_EXT_RAM_BSS_ATTR struct warm_key warm_keys[CONFIG_MESHTASTIC_NODEDB_WARM_KEYS];

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

/* B-5: copy a warm-tier peer's carried role, if present. Caller holds nodedb_lock. */
static bool warm_get_role_locked(uint32_t num, uint8_t *role)
{
	struct warm_key *slot = warm_find_locked(num);

	if (slot == NULL) {
		return false;
	}
	*role = slot->role;
	return true;
}

/* agents-ooma.32: whether the warm tier remembers @p num as a known XEdDSA signer. False
 * (not just "unknown") when no warm slot exists -- the hot store is checked first by the
 * caller, so reaching here already means neither tier knows this node either way. */
static bool warm_get_known_signer_locked(uint32_t num)
{
	struct warm_key *slot = warm_find_locked(num);

	return slot != NULL && slot->known_signer != 0U;
}

/* Sets the known-signer bit on an EXISTING warm slot only -- a signature only verifies
 * against a key we already hold, so by the time this is called the node's key (and thus its
 * warm slot, if it isn't hot-resident) already exists. A no-op otherwise is deliberate: this
 * must never itself create a slot with no key attached. */
static void warm_set_known_signer_locked(uint32_t num, bool signer)
{
	struct warm_key *slot = warm_find_locked(num);
	uint8_t value = signer ? 1U : 0U;

	if (slot != NULL && slot->known_signer != value) {
		slot->known_signer = value;
		nodekeys_schedule_save();
	}
}

/* Slot to (re)write for @num: its existing slot, else an empty slot, else a
 * least-recently-seen entry to evict — but keys whose node is still active in
 * the hot store (favorites are always hot-resident) are protected, so an active
 * conversation never loses its PKC key to a burst of new nodes. Only if every
 * entry is protected (warm smaller than the hot store) do we fall back to the
 * global LRU, so a store always succeeds. */
/* Where a key goes: its own slot, else an empty one, else the least recent key
 * whose node is still in the hot store (a redundant copy: the hot record holds the
 * key), else the least recent key of an evicted node. The warm tier's job is the
 * evicted nodes, as upstream's is (agents-2dk3 Phase 2); hot keys are only cached
 * while there is room. The hot-store lookup is O(hot) and is done only for a slot
 * that could still win, so a large ring does not cost warm x hot on every place. */
static struct warm_key *warm_slot_for_locked(uint32_t num)
{
	struct warm_key *slot = warm_find_locked(num);
	struct warm_key *hot_lru = NULL;  /* LRU among keys still held by the hot store */
	struct warm_key *cold_lru = NULL; /* LRU among evicted nodes' keys */

	if (slot != NULL) {
		return slot;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		struct warm_key *w = &warm_keys[i];

		if (w->num == 0U) {
			return w;
		}
		if (hot_lru != NULL && cold_lru != NULL && w->last_seen >= hot_lru->last_seen &&
		    w->last_seen >= cold_lru->last_seen) {
			continue; /* newer than both candidates: cannot win either */
		}
		if (find_entry_locked(w->num) != NULL) {
			if (hot_lru == NULL || w->last_seen < hot_lru->last_seen) {
				hot_lru = w;
			}
		} else if (cold_lru == NULL || w->last_seen < cold_lru->last_seen) {
			cold_lru = w;
		}
	}

	return (hot_lru != NULL) ? hot_lru : cold_lru;
}

static void warm_place_locked(uint32_t num, const uint8_t *pub, uint8_t role, uint8_t known_signer,
			      uint32_t last_seen)
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
	slot->role = role;
	slot->known_signer = known_signer;
	memcpy(slot->pub, pub, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
}

static void warm_upsert_locked(uint32_t num, const uint8_t *pub, uint8_t role)
{
	/* known_signer resets to 0 here: this path is a key learned/changed via NodeInfo, and a
	 * (re)established key has proven nothing under its new identity yet (agents-ooma.32). A
	 * first-contact NodeInfo that itself arrived signed re-marks it true right afterward, in
	 * the same synchronous RX handling -- see meshtastic_nodedb_note_xeddsa_signer's callers. */
	warm_place_locked(num, pub, role, 0U, warm_now());
}

/* A keyed node leaving the hot store: keep its key, role, signer bit and recency
 * in the warm tier, so PKC to it keeps working and a readmission restores them
 * (upstream warmStore.absorb). Without this a key was lost whenever the warm ring
 * was smaller than the hot store, as it is on the XIAO (100 warm, 120 hot). */
static void warm_absorb_locked(const struct nodedb_entry *entry)
{
	const meshtastic_NodeInfoLite *n = &entry->node;
	uint32_t last_seen;

	if (!entry->used || n->num == 0U || n->num == meshtastic_get_node_id() ||
	    n->public_key.size != MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN) {
		return;
	}
	if (n->last_heard != 0U) {
		/* Heard this boot: its age in the warm tier's own time base. */
		uint32_t age = uptime_seconds() - n->last_heard;
		uint32_t now = warm_now();

		last_seen = (now > age) ? now - age : 0U;
	} else {
		last_seen = entry->last_heard_epoch; /* restored, not re-heard */
	}
	/* Take the slot BEFORE the entry is overwritten: the caller reuses it. */
	warm_place_locked(n->num, n->public_key.bytes, (uint8_t)n->role,
			  IS_BIT_SET(n->bitfield, NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_BIT) ? 1U : 0U,
			  last_seen);
	nodekeys_schedule_save();
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
	bool all; /* collect every record (the bulk migration's cleanup), not just orphans */
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
	orphan = ctx->all || (warm_find_locked(num) == NULL);
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

	/* Lockdown (phase 2): on a locked boot RAM holds nothing because the sealed
	 * records could not be opened -- pruning "what RAM does not hold" would
	 * delete every persisted key. Bench 2026-09-10: it did. Leave the flag set
	 * so the prune runs after the unlock reload has put the records back. */
	if (!meshtastic_lockdown_store_ready()) {
		return;
	}
#if defined(NODEDB_WBULK)
	if (wrec_active()) {
		nodekeys_reconcile = false; /* the pages are the whole ring: nothing to prune */
		wrec_do_persist();
		return;
	}
#endif

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

#if defined(CONFIG_MESHTASTIC_NODEDB_PURGE_FOREIGN_KEYS)
struct nodekeys_purge_ctx {
	uint32_t ids[WARM_RECONCILE_BATCH];
	size_t count;
	bool more;
};

/* Direct-load callback over the mtnode subtree: collect ids whose record length is
 * NOT the current MTNODE_REC_LEN (a stale pre-role/key-only format to purge). */
static int nodekeys_purge_cb(const char *key, size_t len, settings_read_cb read_cb,
			     void *cb_arg, void *param)
{
	struct nodekeys_purge_ctx *ctx = param;
	const char *id = strrchr(key, '/');
	uint32_t num;
	char *endptr;

	ARG_UNUSED(read_cb);
	ARG_UNUSED(cb_arg);

	if (len == MTNODE_REC_LEN) {
		return 0; /* current format — keep */
	}

	id = (id != NULL) ? id + 1 : key;
	num = (uint32_t)strtoul(id, &endptr, 16);
	if (*endptr != '\0' || num == 0U) {
		return 0;
	}
	if (ctx->count < ARRAY_SIZE(ctx->ids)) {
		ctx->ids[ctx->count++] = num;
	} else {
		ctx->more = true;
	}
	return 0;
}

/* One-shot dev cleanup (temp build): delete every persisted mtnode/<id> record whose
 * length is not the current 37 B format, so a device that ran an older build converges
 * to a clean, single-format key store without a full NVS erase. Scoped strictly to the
 * mtnode subtree — the device private key (meshtastic config subtree) and WiFi creds
 * (wifi_cred subtree) are never touched. Batched with a pass cap so a large backlog is
 * fully drained but a persistently-failing delete can't spin forever. */
static void nodekeys_purge_foreign(void)
{
	char name[SETTINGS_MAX_NAME_LEN + 1];
	size_t total = 0U;
	int passes = 0;

	do {
		struct nodekeys_purge_ctx ctx = {.count = 0U, .more = false};

		(void)settings_load_subtree_direct(MTNODE_SUBTREE, nodekeys_purge_cb, &ctx);
		for (size_t i = 0U; i < ctx.count; i++) {
			(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x", ctx.ids[i]);
			(void)settings_delete(name);
		}
		total += ctx.count;
		if (!ctx.more || ++passes >= 64) {
			break;
		}
	} while (true);

	if (total > 0U) {
		LOG_WRN("NodeDB purged %zu foreign (non-%u B) key record(s) from NVS",
			total, (unsigned int)MTNODE_REC_LEN);
	} else {
		LOG_INF("NodeDB purge: no foreign key records in NVS");
	}
}
#endif /* CONFIG_MESHTASTIC_NODEDB_PURGE_FOREIGN_KEYS */

#if defined(NODEDB_WBULK)
/* ---- the warm key tier in the bulk store ----------------------------------------
 *
 * Same scheme as the node-record pages (NREC, below): fixed-slot pages, written only
 * when a page's plaintext CRC changes. Simpler, because the warm ring is already a
 * fixed array: ring slot s is slot (s % per-page) of page (s / per-page), and a
 * restore puts each key back in its own slot, so an unchanged ring rereads to
 * identical pages and costs no writes.
 *
 * Page: [ver][slots per page][used count][0], then per slot [num LE32] followed by
 * the 38 B mtnode record (last_seen LE32, role, known_signer, public key).
 */
#define WREC_SLOT_LEN   (4U + MTNODE_REC_LEN)
#define WREC_PAGE_HDR   4U
#define WREC_PER_PAGE   ((MESHTASTIC_BULK_BLOB_MAX - WREC_PAGE_HDR) / WREC_SLOT_LEN)
#define WREC_PAGES      DIV_ROUND_UP(CONFIG_MESHTASTIC_NODEDB_WARM_KEYS, WREC_PER_PAGE)
#define WREC_PAGE_LEN   (WREC_PAGE_HDR + WREC_PER_PAGE * WREC_SLOT_LEN)
#define WREC_PAGE_ID(p) ((uint16_t)(0x0200U + (p)))
#define WREC_META_ID    0x00F1U
#define WREC_META_MAGIC 0x434E4B57U /* "WKNC" */
#define WREC_PAGE_VER   1U

BUILD_ASSERT(WREC_PER_PAGE <= UINT8_MAX, "slot count is stored in a byte");
BUILD_ASSERT(WREC_PAGES <= 0x100U, "warm pages must stay inside their id range");

static uint32_t wrec_crc[WREC_PAGES];
static bool wrec_crc_valid[WREC_PAGES];
static bool wrec_loaded;
static bool wrec_legacy_pending;
#define wrec_page nodedb_page_buf
#define wrec_lock nodedb_page_lock
BUILD_ASSERT(WREC_PAGE_LEN <= MESHTASTIC_BULK_BLOB_MAX, "warm page too large for a bulk blob");

static bool wrec_active(void)
{
	return meshtastic_bulk_ready();
}

static uint32_t wrec_build_page_locked(size_t p, uint8_t *used_out)
{
	uint32_t self = meshtastic_get_node_id();
	uint8_t used = 0U;

	memset(wrec_page, 0, WREC_PAGE_LEN);
	wrec_page[0] = WREC_PAGE_VER;
	wrec_page[1] = (uint8_t)WREC_PER_PAGE;
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < WREC_PER_PAGE; i++) {
		size_t s = p * WREC_PER_PAGE + i;
		uint8_t *slot = wrec_page + WREC_PAGE_HDR + i * WREC_SLOT_LEN;
		const struct warm_key *w;

		if (s >= ARRAY_SIZE(warm_keys)) {
			break;
		}
		w = &warm_keys[s];
		if (w->num == 0U || w->num == self) {
			continue;
		}
		sys_put_le32(w->num, slot);
		sys_put_le32(w->last_seen, slot + 4);
		slot[8] = w->role;
		slot[9] = w->known_signer;
		memcpy(slot + 10, w->pub, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
		used++;
	}
	k_mutex_unlock(&nodedb_lock);
	wrec_page[2] = used;
	*used_out = used;
	return crc32_ieee(wrec_page, WREC_PAGE_LEN);
}

static void wrec_persist(bool force)
{
	if (!wrec_active() || !wrec_loaded || !meshtastic_lockdown_store_ready()) {
		return;
	}
	k_mutex_lock(&wrec_lock, K_FOREVER);
	for (size_t p = 0U; p < WREC_PAGES; p++) {
		uint8_t used;
		uint32_t crc = wrec_build_page_locked(p, &used);
		int ret;

		if (!force && wrec_crc_valid[p] && wrec_crc[p] == crc) {
			continue;
		}
		ret = (used > 0U) ? meshtastic_bulk_write(WREC_PAGE_ID(p), wrec_page,
							  WREC_PAGE_LEN)
				  : meshtastic_bulk_delete(WREC_PAGE_ID(p));
		if (ret == 0 || ret == -ENOENT) {
			wrec_crc[p] = crc;
			wrec_crc_valid[p] = true;
		} else {
			wrec_crc_valid[p] = false;
			LOG_WRN("NodeDB: bulk warm page %u write failed (%d)", (unsigned int)p, ret);
		}
	}
	k_mutex_unlock(&wrec_lock);
}

/* The restore is authoritative: the ring is cleared and each key goes back into
 * its own slot. Returns the number restored, or -EACCES on a locked boot (the
 * ring is left as it was). */
static int wrec_load(void)
{
	int restored = 0;
	bool cleared = false;

	k_mutex_lock(&wrec_lock, K_FOREVER);
	for (size_t p = 0U; p < WREC_PAGES; p++) {
		ssize_t n = meshtastic_bulk_read(WREC_PAGE_ID(p), wrec_page, WREC_PAGE_LEN);

		wrec_crc_valid[p] = false;
		if (n == -EACCES) {
			k_mutex_unlock(&wrec_lock);
			return -EACCES;
		}
		if (!cleared) {
			k_mutex_lock(&nodedb_lock, K_FOREVER);
			memset(warm_keys, 0, sizeof(warm_keys));
			k_mutex_unlock(&nodedb_lock);
			cleared = true;
		}
		if (n != (ssize_t)WREC_PAGE_LEN || wrec_page[0] != WREC_PAGE_VER ||
		    wrec_page[1] != (uint8_t)WREC_PER_PAGE) {
			continue; /* absent, damaged or another layout: rebuilt at the next save */
		}
		k_mutex_lock(&nodedb_lock, K_FOREVER);
		for (size_t i = 0U; i < WREC_PER_PAGE; i++) {
			const uint8_t *slot = wrec_page + WREC_PAGE_HDR + i * WREC_SLOT_LEN;
			size_t s = p * WREC_PER_PAGE + i;
			uint32_t num = sys_get_le32(slot);

			if (num == 0U || num == meshtastic_get_node_id()) {
				continue;
			}
			if (s < ARRAY_SIZE(warm_keys) && warm_keys[s].num == 0U) {
				struct warm_key *w = &warm_keys[s];

				w->num = num;
				w->last_seen = sys_get_le32(slot + 4);
				w->role = slot[8];
				w->known_signer = slot[9];
				memcpy(w->pub, slot + 10, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
			} else {
				/* A ring built smaller than the pages: LRU placement. */
				warm_place_locked(num, slot + 10, slot[8], slot[9],
						  sys_get_le32(slot + 4));
			}
			restored++;
		}
		k_mutex_unlock(&nodedb_lock);
		wrec_crc[p] = crc32_ieee(wrec_page, WREC_PAGE_LEN);
		wrec_crc_valid[p] = true;
	}
	wrec_loaded = true;
	k_mutex_unlock(&wrec_lock);
	return restored;
}

static int wrec_count(void)
{
	uint32_t self = meshtastic_get_node_id();
	int n = 0;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num != 0U && warm_keys[i].num != self) {
			n++;
		}
	}
	k_mutex_unlock(&nodedb_lock);
	return n;
}

/* Boot restore plus the one-time import of settings mtnode/ records, with the
 * node records' commit-after-verify rule. */
static int wrec_boot_restore(void)
{
	uint8_t meta[8];
	ssize_t n = meshtastic_bulk_read(WREC_META_ID, meta, sizeof(meta));
	bool migrated = (n == (ssize_t)sizeof(meta) && sys_get_le32(meta) == WREC_META_MAGIC &&
			 meta[4] == 1U);
	int restored;

	wrec_loaded = false;
	if (n == -EACCES) {
		return -EACCES; /* locked: the unlock's reload runs this again */
	}
	if (migrated) {
		restored = wrec_load();
		wrec_legacy_pending = true;
		return restored;
	}

	/* First boot with a bulk store: bring the settings keys across. */
	(void)settings_load_subtree(MTNODE_SUBTREE);
	{
		int want = wrec_count();

		k_mutex_lock(&wrec_lock, K_FOREVER);
		memset(wrec_crc_valid, 0, sizeof(wrec_crc_valid));
		k_mutex_unlock(&wrec_lock);
		wrec_loaded = true;
		wrec_persist(true);
		restored = wrec_load();
		if (restored < 0 || restored != want) {
			LOG_WRN("NodeDB: bulk key import read back %d of %d; the settings keys "
				"stay authoritative until a later boot imports them",
				restored, want);
			return restored;
		}
	}
	sys_put_le32(WREC_META_MAGIC, meta);
	meta[4] = 1U;
	memset(meta + 5, 0, 3);
	if (meshtastic_bulk_write(WREC_META_ID, meta, sizeof(meta)) == 0) {
		wrec_legacy_pending = true;
		LOG_INF("NodeDB: %d warm key(s) moved to the bulk store", restored);
	}
	return restored;
}

static bool wrec_image_confirmed(void)
{
#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
	/* An MCUboot revert must still find the settings keys the older image reads. */
	return boot_is_img_confirmed();
#else
	return true;
#endif
}

/* The save path when the bulk store is up: delete leftover settings keys (in
 * batches, once confirmed), then write the pages that changed. */
static void wrec_do_persist(void)
{
	if (wrec_legacy_pending && wrec_loaded && wrec_image_confirmed()) {
		struct warm_reconcile_ctx ctx = {.count = 0U, .overflow = false, .all = true};
		char name[SETTINGS_MAX_NAME_LEN + 1];

		(void)settings_load_subtree_direct(MTNODE_SUBTREE, warm_reconcile_cb, &ctx);
		for (size_t i = 0U; i < ctx.count; i++) {
			(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x", ctx.orphans[i]);
			(void)settings_delete(name);
		}
		if (ctx.count > 0U) {
			LOG_INF("NodeDB: deleted %zu settings key(s) now in the bulk store",
				ctx.count);
		}
		wrec_legacy_pending = ctx.overflow;
		if (ctx.overflow) {
			nodekeys_schedule_save();
		}
	}
	wrec_persist(false);
}

/* Lockdown rewrote the store: rewrite every page in the mode now in force. */
static void wrec_rewrite_hook(void)
{
	wrec_persist(true);
}
#endif /* NODEDB_WBULK */

static int nodekeys_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MTNODE_REC_LEN];
	char full_name[SETTINGS_MAX_NAME_LEN + 1];
	uint32_t node_num;
	uint32_t last_seen;
	uint8_t role;
	uint8_t known_signer;
	const uint8_t *pub;
	char *endptr;
	ssize_t read;

#if defined(NODEDB_WBULK)
	/* Once the bulk pages are loaded, settings keys are leftovers awaiting
	 * deletion; a later full settings_load() must not re-place them. */
	if (wrec_active() && wrec_loaded) {
		return 0;
	}
#endif

	/* Only the current 38 B format is accepted (plus lockdown's seal when the
	 * record is sealed). A stale-format record (a pre-signer 37 B, pre-role 36 B,
	 * or key-only 32 B one) is ignored here and removed from NVS by the one-shot
	 * CONFIG_MESHTASTIC_NODEDB_PURGE_FOREIGN_KEYS pass. */
	if (len != MTNODE_REC_LEN && len != MTNODE_REC_LEN + MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD) {
		LOG_WRN("Ignoring persisted node key '%s' with unexpected size %zu", key, len);
		return 0;
	}

	(void)snprintk(full_name, sizeof(full_name), MTNODE_SUBTREE "/%s", key);
	read = meshtastic_lockdown_read(full_name, len, read_cb, cb_arg, buf, sizeof(buf));
	if (read != (ssize_t)MTNODE_REC_LEN) {
		return 0; /* unreadable, or sealed on a locked boot: not restored */
	}
	last_seen = sys_get_le32(buf);
	role = buf[sizeof(uint32_t)];
	known_signer = buf[sizeof(uint32_t) + 1U];
	pub = buf + sizeof(uint32_t) + 2U;

	node_num = (uint32_t)strtoul(key, &endptr, 16);
	if (*endptr != '\0' || node_num == 0U) {
		return 0;
	}

	/* Restore into the warm tier, not the hot store: keeps the key reachable
	 * for PKC without occupying a hot record slot (avoids restore thrash). */
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	warm_place_locked(node_num, pub, role, known_signer, last_seen);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

static int nodekeys_export(int (*export_func)(const char *name, const void *val, size_t val_len))
{
	char name[SETTINGS_MAX_NAME_LEN + 1];
	uint32_t self = meshtastic_get_node_id();
	int ret = 0;

#if defined(NODEDB_WBULK)
	if (wrec_active()) {
		return 0; /* the ring lives in the bulk store */
	}
#endif

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < ARRAY_SIZE(warm_keys); i++) {
		if (warm_keys[i].num == 0U || warm_keys[i].num == self) {
			continue;
		}

		uint8_t rec[MTNODE_REC_LEN];

		sys_put_le32(warm_keys[i].last_seen, rec);
		rec[sizeof(uint32_t)] = warm_keys[i].role;
		rec[sizeof(uint32_t) + 1U] = warm_keys[i].known_signer;
		memcpy(rec + sizeof(uint32_t) + 2U, warm_keys[i].pub,
		       MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);

		(void)snprintk(name, sizeof(name), MTNODE_SUBTREE "/%08x", warm_keys[i].num);
		ret = meshtastic_lockdown_export(export_func, name, rec, sizeof(rec));
		if (ret < 0) {
			break;
		}
	}
	k_mutex_unlock(&nodedb_lock);

	return ret;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtnode, MTNODE_SUBTREE, NULL, nodekeys_set, NULL, nodekeys_export);
#endif /* CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS */

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
/*
 * Record tier. Persists the full identity (name, hw model, role, flags, public
 * key) of every node in the hot store, keyed "mtrec/<id>", and restores them at
 * boot so the seen-node list survives a reboot. Distinct from the warm-key tier
 * above, which persists only public keys for PKC reach: a record here is
 * self-contained and carries the key too. The set is bounded by the hot store
 * (MESHTASTIC_NODEDB_MAX_NODES), and NVS is kept == the hot store by pruning a
 * node's record when it is evicted or removed.
 *
 * Volatile per-hearing fields (snr / channel / next_hop / hops / via-mqtt) are
 * NOT persisted — they are re-learned from the next packet. last_heard IS
 * persisted, but as a durable wall-clock epoch (see mtrec_encode), not the
 * volatile uptime, so a node's recency survives a reboot. Identity changes and
 * favorite/ignore toggles persist promptly; last_heard is refreshed by a
 * periodic snapshot (MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC) that only rewrites
 * records whose epoch actually changed. Guarded by nodedb_lock like the rest of
 * the store.
 */
#define MTREC_SUBTREE         "mtrec"
/* 2 (2026-08-19): the NodeInfoLite bitfield layout changed to match the
 * reference — IS_FAVORITE 0->3, IS_IGNORED 1->4 — and the bitfield is part of
 * this record. A v1 record decodes cleanly but means something different, which
 * is the worst kind of stale data, so the version gates it out: mtrec_decode()
 * rejects any record whose first byte is not this value, the entry is simply
 * re-learned from the air, and the reconcile pass prunes the orphan. */
#define MTREC_RECORD_VERSION  2U
#define MTREC_HEADER_LEN      4U /* version, reserved, LE16 payload length */
#define MTREC_RECONCILE_BATCH 32U
#define MTREC_BUF_LEN         (meshtastic_NodeInfoLite_size + MTREC_HEADER_LEN)

/* Set when a node leaves the hot store (evicted / removed / un-curated on reset),
 * so the next persist pass prunes its now-orphaned NVS record. */
static bool mtrec_reconcile;

/* Flag an eviction orphan for pruning. Called from the (locked) eviction path,
 * so it only sets the flag; the next snapshot/save does the prune. */
static void mtrec_note_evicted(void)
{
	mtrec_reconcile = true;
}

/* Copy @in, zeroing the volatile per-hearing fields that must not be persisted.
 * last_heard is set separately by mtrec_encode, as a durable wall-clock epoch.
 *
 * snr and hops_away are NOT in that volatile set, despite having been treated as
 * such until 2026-08-19. They are what the phone shows as a peer's link quality
 * and distance, and dropping them meant a reboot blanked both columns until
 * every node happened to be heard again — hours on a quiet mesh. The reference
 * persists both. SNR travels as the Q4 integer rather than the float (1-2 bytes
 * against 5, and no float in the on-disk record), with a presence bit so a
 * genuine 0 dB reading is distinguishable from "never measured".
 *
 * channel and next_hop stay volatile here, deliberately: next_hop is a learned
 * route that self-corrects and is actively misleading while stale, and channel
 * is re-derived from the next packet at no cost.
 */
static void mtrec_durable_copy(const meshtastic_NodeInfoLite *in, meshtastic_NodeInfoLite *out)
{
	*out = *in;
	out->channel = 0U;
	out->next_hop = 0U;
	WRITE_BIT(out->bitfield, NODEINFO_BITFIELD_VIA_MQTT_BIT, 0);

	/* Q4 on disk; the float is in-memory only (reference: NodeInfoLite.snr is
	 * "always zeroed before encode"). */
	out->snr = 0.0f;
	if (IS_BIT_SET(in->bitfield, NODEINFO_BITFIELD_HAS_SNR_BIT)) {
		out->snr_q4 = meshtastic_snr_to_q4(in->snr);
	} else {
		out->snr_q4 = 0;
	}
}

static int mtrec_encode(const struct nodedb_entry *e, uint8_t *buf, size_t buf_len)
{
	meshtastic_NodeInfoLite durable;
	pb_ostream_t stream;

	if (buf_len < MTREC_HEADER_LEN) {
		return -EINVAL;
	}

	mtrec_durable_copy(&e->node, &durable);
	/* Persist last-heard as a durable wall-clock epoch, not the volatile uptime:
	 * derive it from the uptime for a node heard this boot, else carry forward the
	 * epoch restored at the last boot. 0 when no clock has ever been seeded. */
	durable.last_heard = (e->node.last_heard > 0U)
				     ? meshtastic_clock_uptime_to_epoch(e->node.last_heard)
				     : e->last_heard_epoch;
	stream = pb_ostream_from_buffer(buf + MTREC_HEADER_LEN, buf_len - MTREC_HEADER_LEN);
	if (!pb_encode(&stream, meshtastic_NodeInfoLite_fields, &durable)) {
		LOG_WRN("NodeDB record encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	buf[0] = MTREC_RECORD_VERSION;
	buf[1] = 0U;
	sys_put_le16((uint16_t)stream.bytes_written, buf + 2);
	return (int)(stream.bytes_written + MTREC_HEADER_LEN);
}

static int mtrec_decode(const uint8_t *buf, size_t len, meshtastic_NodeInfoLite *node)
{
	pb_istream_t stream;
	uint16_t payload_len;

	if (len < MTREC_HEADER_LEN || buf[0] != MTREC_RECORD_VERSION) {
		return -EINVAL;
	}

	payload_len = sys_get_le16(buf + 2);
	if ((size_t)payload_len != len - MTREC_HEADER_LEN) {
		return -EINVAL;
	}

	*node = (meshtastic_NodeInfoLite)meshtastic_NodeInfoLite_init_zero;
	stream = pb_istream_from_buffer(buf + MTREC_HEADER_LEN, payload_len);
	if (!pb_decode(&stream, meshtastic_NodeInfoLite_fields, node)) {
		LOG_WRN("NodeDB record decode failed: %s", PB_GET_ERROR(&stream));
		return -EINVAL;
	}

	/* Rehydrate the in-memory float from the Q4 integer that was persisted. Only
	 * when the presence bit says a reading was actually taken — otherwise leave
	 * snr at 0 with the bit clear, which every consumer reads as "not heard yet"
	 * rather than as a 0 dB link. */
	if (IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_HAS_SNR_BIT)) {
		node->snr = meshtastic_snr_from_q4(node->snr_q4);
	}
	node->snr_q4 = 0;

	return 0;
}

/* True if @num is still in the hot store — i.e. its NVS record should be kept
 * rather than pruned as an orphan (left by an eviction, removal, or reset).
 * Takes nodedb_lock. */
static bool mtrec_is_current(uint32_t num)
{
	bool current;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	current = (find_entry_locked(num) != NULL);
	k_mutex_unlock(&nodedb_lock);

	return current;
}

struct mtrec_reconcile_ctx {
	uint32_t orphans[MTREC_RECONCILE_BATCH];
	size_t count;
	bool overflow;
	bool all; /* collect every record, not just orphans (the bulk migration's cleanup) */
};

/* Direct-load callback over the mtrec subtree: collect ids whose node is no
 * longer curated (orphans to prune). */
static int mtrec_reconcile_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			      void *param)
{
	struct mtrec_reconcile_ctx *ctx = param;
	const char *id = strrchr(key, '/');
	uint32_t num;
	char *endptr;

	ARG_UNUSED(len);
	ARG_UNUSED(read_cb);
	ARG_UNUSED(cb_arg);

	id = (id != NULL) ? id + 1 : key;
	num = (uint32_t)strtoul(id, &endptr, 16);
	if (*endptr != '\0' || num == 0U) {
		return 0;
	}

	if (!ctx->all && mtrec_is_current(num)) {
		return 0;
	}
	if (ctx->count < ARRAY_SIZE(ctx->orphans)) {
		ctx->orphans[ctx->count++] = num;
	} else {
		ctx->overflow = true;
	}
	return 0;
}

/* Prune orphaned mtrec/<id> records (nodes no longer curated) then persist the
 * current curated set. Shared by the delayed save-work and the reset path. */
static void mtrec_do_persist(void)
{
	int ret;

	if (!meshtastic_lockdown_store_ready()) {
		return; /* as nodekeys_do_persist: never prune against placeholders */
	}
#if defined(NODEDB_BULK)
	if (nrec_active()) {
		nrec_do_persist();
		return;
	}
#endif

	if (mtrec_reconcile) {
		struct mtrec_reconcile_ctx ctx = {.count = 0U, .overflow = false};
		char name[SETTINGS_MAX_NAME_LEN + 1];

		mtrec_reconcile = false;
		(void)settings_load_subtree_direct(MTREC_SUBTREE, mtrec_reconcile_cb, &ctx);

		for (size_t i = 0U; i < ctx.count; i++) {
			(void)snprintk(name, sizeof(name), MTREC_SUBTREE "/%08x", ctx.orphans[i]);
			(void)settings_delete(name);
		}
		if (ctx.count > 0U) {
			LOG_DBG("NodeDB pruned %zu orphaned record(s) from NVS", ctx.count);
		}
		if (ctx.overflow) {
			mtrec_reconcile = true; /* more to prune next pass */
		}
	}

	ret = settings_save_subtree(MTREC_SUBTREE);
	if (ret < 0) {
		LOG_WRN("NodeDB record save failed (%d)", ret);
	}

	if (mtrec_reconcile) {
		mtrec_schedule_save();
	}
}

static void mtrec_save_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	mtrec_do_persist();
}

static K_WORK_DELAYABLE_DEFINE(mtrec_save_work, mtrec_save_work_handler);

static void mtrec_schedule_save(void)
{
	(void)k_work_reschedule(&mtrec_save_work,
				K_MSEC(CONFIG_MESHTASTIC_SETTINGS_SAVE_DELAY_MS));
}

#if CONFIG_MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC > 0
/* Periodic full-store snapshot: refreshes each node's persisted last-heard epoch
 * (and captures newly-seen nodes) so recency survives a reboot, and prunes any
 * eviction orphans flagged since the last pass. The settings backend skips
 * unchanged records, so a snapshot only writes nodes heard since the previous
 * one. Self-reschedules. */
static void mtrec_snapshot_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(mtrec_snapshot_work, mtrec_snapshot_work_handler);

static void mtrec_snapshot_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	mtrec_do_persist();
	(void)k_work_reschedule(&mtrec_snapshot_work,
				K_SECONDS(CONFIG_MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC));
}
#endif /* CONFIG_MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC > 0 */

/* Put a persisted record into the hot store. Takes nodedb_lock. */
static void mtrec_apply(uint32_t num, meshtastic_NodeInfoLite *node)
{
	struct nodedb_entry *entry;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = get_or_create_entry_locked(num);
	if (entry != NULL) {
		node->num = num; /* the key is authoritative */
		/* The persisted last_heard is a durable wall-clock epoch; move it to
		 * the entry's epoch field and reset the uptime one — this node has not
		 * been heard this boot, so its age resolves from the epoch until it is
		 * re-heard. */
		entry->last_heard_epoch = node->last_heard;
		node->last_heard = 0U;
		entry->node = *node;
		entry->used = true;
	}
	k_mutex_unlock(&nodedb_lock);
}

#if defined(NODEDB_BULK)
/* ---- node records in the bulk store ----------------------------------------------
 *
 * The hot store is persisted as NREC_PAGES fixed-slot pages, one bulk blob each.
 * A node keeps its slot for as long as it stays in the hot store, so a change to
 * one node rewrites one page, and a page whose plaintext has not changed since it
 * was last written or read is not written at all (compared by CRC32). The CRC
 * compare is what keeps lockdown cheap: a sealed page gets a fresh nonce on every
 * write, so "write it and let NVS dedup it" would rewrite every page every
 * snapshot on a sealed device (agents-2dk3.5).
 *
 * Page: [ver][slots per page][used count][0], then per slot
 *       [num LE32][record length][mtrec_encode() output, zero-padded].
 */
#define NREC_SLOT_LEN   (4U + 1U + MTREC_BUF_LEN)
#define NREC_PAGE_HDR   4U
#define NREC_PER_PAGE   ((MESHTASTIC_BULK_BLOB_MAX - NREC_PAGE_HDR) / NREC_SLOT_LEN)
#define NREC_PAGES      DIV_ROUND_UP(CONFIG_MESHTASTIC_NODEDB_MAX_NODES, NREC_PER_PAGE)
#define NREC_SLOTS      (NREC_PAGES * NREC_PER_PAGE)
#define NREC_PAGE_LEN   (NREC_PAGE_HDR + NREC_PER_PAGE * NREC_SLOT_LEN)
#define NREC_PAGE_ID(p) ((uint16_t)(0x0100U + (p)))
#define NREC_META_ID    0x00F0U
#define NREC_META_MAGIC 0x4345524EU /* "NREC" */
#define NREC_PAGE_VER   1U

BUILD_ASSERT(NREC_PER_PAGE >= 1U, "a node record must fit a bulk blob");
BUILD_ASSERT(NREC_PER_PAGE <= UINT8_MAX, "slot count is stored in a byte");
BUILD_ASSERT(NREC_PAGE_LEN <= MESHTASTIC_BULK_BLOB_MAX, "page too large for a bulk blob");
BUILD_ASSERT(MTREC_BUF_LEN <= UINT8_MAX, "record length is stored in a byte");

/* The slot map and page buffer are CPU-only and mutex-guarded, so on ESP32 they go
 * to PSRAM: the bulk engine copies a page into its own internal-RAM frame before
 * any flash write (flash writes cannot source from PSRAM). The V4 courier has
 * ~5 KB of internal DRAM to spare; these two would have taken 3 KB of it. */
static MESHTASTIC_EXT_RAM_BSS_ATTR uint32_t nrec_slot[NREC_SLOTS]; /* node per slot, 0 = free */
static uint32_t nrec_crc[NREC_PAGES];   /* plaintext CRC last written or read */
static bool nrec_crc_valid[NREC_PAGES];
static bool nrec_loaded;                /* pages read (or migrated) this boot */
static bool nrec_legacy_pending;        /* settings mtrec/ records still to delete */
#define nrec_page nodedb_page_buf /* shared with the warm pages; see nodedb_page_lock */
#define nrec_lock nodedb_page_lock

static bool nrec_active(void)
{
	return meshtastic_bulk_ready();
}

static int nrec_slot_of_locked(uint32_t num)
{
	for (size_t s = 0U; s < NREC_SLOTS; s++) {
		if (nrec_slot[s] == num) {
			return (int)s;
		}
	}
	return -1;
}

/* Free the slots of nodes that left the hot store, then give every hot node
 * without a slot the first free one. Caller holds nrec_lock. */
static void nrec_assign_slots_locked(void)
{
	uint32_t self = meshtastic_get_node_id();

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t s = 0U; s < NREC_SLOTS; s++) {
		if (nrec_slot[s] != 0U && find_entry_locked(nrec_slot[s]) == NULL) {
			nrec_slot[s] = 0U;
		}
	}
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		const struct nodedb_entry *e = &nodedb_entries[i];

		if (!e->used || e->node.num == self || nrec_slot_of_locked(e->node.num) >= 0) {
			continue;
		}
		for (size_t s = 0U; s < NREC_SLOTS; s++) {
			if (nrec_slot[s] == 0U) {
				nrec_slot[s] = e->node.num;
				break;
			}
		}
	}
	k_mutex_unlock(&nodedb_lock);
}

/* Build page p's plaintext from the hot store into nrec_page and return its CRC.
 * Caller holds nrec_lock. */
static uint32_t nrec_build_page_locked(size_t p, uint8_t *used_out)
{
	uint8_t used = 0U;

	memset(nrec_page, 0, NREC_PAGE_LEN);
	nrec_page[0] = NREC_PAGE_VER;
	nrec_page[1] = (uint8_t)NREC_PER_PAGE;
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < NREC_PER_PAGE; i++) {
		uint32_t num = nrec_slot[p * NREC_PER_PAGE + i];
		uint8_t *slot = nrec_page + NREC_PAGE_HDR + i * NREC_SLOT_LEN;
		const struct nodedb_entry *e;
		int len;

		if (num == 0U) {
			continue;
		}
		e = find_entry_locked(num);
		if (e == NULL) {
			continue;
		}
		len = mtrec_encode(e, slot + 5, MTREC_BUF_LEN);
		if (len <= 0) {
			continue;
		}
		sys_put_le32(num, slot);
		slot[4] = (uint8_t)len;
		used++;
	}
	k_mutex_unlock(&nodedb_lock);
	nrec_page[2] = used;
	*used_out = used;
	return crc32_ieee(nrec_page, NREC_PAGE_LEN);
}

/* Write every page whose content changed (every page when `force`). */
static void nrec_persist(bool force)
{
	if (!nrec_active() || !nrec_loaded || !meshtastic_lockdown_store_ready()) {
		return;
	}
	k_mutex_lock(&nrec_lock, K_FOREVER);
	nrec_assign_slots_locked();
	for (size_t p = 0U; p < NREC_PAGES; p++) {
		uint8_t used;
		uint32_t crc = nrec_build_page_locked(p, &used);
		int ret;

		if (!force && nrec_crc_valid[p] && nrec_crc[p] == crc) {
			continue;
		}
		ret = (used > 0U) ? meshtastic_bulk_write(NREC_PAGE_ID(p), nrec_page,
							  NREC_PAGE_LEN)
				  : meshtastic_bulk_delete(NREC_PAGE_ID(p));
		if (ret == 0 || ret == -ENOENT) {
			nrec_crc[p] = crc;
			nrec_crc_valid[p] = true;
		} else {
			nrec_crc_valid[p] = false;
			LOG_WRN("NodeDB: bulk page %u write failed (%d)", (unsigned int)p, ret);
		}
	}
	k_mutex_unlock(&nrec_lock);
}

/* Read every page into the hot store. Returns the number of records restored, or
 * -EACCES on a locked boot (nothing restored, nothing may be written). */
static int nrec_load(void)
{
	int restored = 0;

	k_mutex_lock(&nrec_lock, K_FOREVER);
	memset(nrec_slot, 0, sizeof(nrec_slot));
	for (size_t p = 0U; p < NREC_PAGES; p++) {
		ssize_t n = meshtastic_bulk_read(NREC_PAGE_ID(p), nrec_page, NREC_PAGE_LEN);

		nrec_crc_valid[p] = false;
		if (n == -EACCES) {
			k_mutex_unlock(&nrec_lock);
			return -EACCES;
		}
		if (n != (ssize_t)NREC_PAGE_LEN || nrec_page[0] != NREC_PAGE_VER ||
		    nrec_page[1] != (uint8_t)NREC_PER_PAGE) {
			continue; /* absent, damaged or another layout: rebuilt at the next save */
		}
		for (size_t i = 0U; i < NREC_PER_PAGE; i++) {
			const uint8_t *slot = nrec_page + NREC_PAGE_HDR + i * NREC_SLOT_LEN;
			uint32_t num = sys_get_le32(slot);
			uint8_t len = slot[4];
			meshtastic_NodeInfoLite node;

			if (num == 0U || len == 0U || len > MTREC_BUF_LEN ||
			    num == meshtastic_get_node_id() ||
			    mtrec_decode(slot + 5, len, &node) < 0) {
				continue;
			}
			mtrec_apply(num, &node);
			nrec_slot[p * NREC_PER_PAGE + i] = num;
			restored++;
		}
		nrec_crc[p] = crc32_ieee(nrec_page, NREC_PAGE_LEN);
		nrec_crc_valid[p] = true;
	}
	nrec_loaded = true;
	k_mutex_unlock(&nrec_lock);
	return restored;
}

static int nrec_hot_peer_count(void)
{
	uint32_t self = meshtastic_get_node_id();
	int n = 0;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		if (nodedb_entries[i].used && nodedb_entries[i].node.num != self) {
			n++;
		}
	}
	k_mutex_unlock(&nodedb_lock);
	return n;
}

/* Boot restore, and the one-time import of the settings records an older image
 * wrote. The import is committed only after the pages read back complete; until
 * the meta record says so, the settings records stay authoritative, so a crash
 * mid-import just imports again. Returns -EACCES on a locked boot. */
static int nrec_boot_restore(void)
{
	uint8_t meta[8];
	ssize_t n = meshtastic_bulk_read(NREC_META_ID, meta, sizeof(meta));
	bool migrated = (n == (ssize_t)sizeof(meta) && sys_get_le32(meta) == NREC_META_MAGIC &&
			 meta[4] == 1U);
	int restored;

	/* A restore starts from nothing loaded: until the pages (or the import) are
	 * in, nothing may be written over them, and the settings records are still
	 * readable for the import. The unlock's reload re-enters here too. */
	nrec_loaded = false;
	if (n == -EACCES) {
		return -EACCES; /* locked: the unlock's reload runs this again */
	}
	if (migrated) {
		restored = nrec_load();
		nrec_legacy_pending = true; /* delete any leftovers once confirmed */
		return restored;
	}

	/* First boot with a bulk store: bring the settings records across. */
	(void)settings_load_subtree(MTREC_SUBTREE);
	{
		int want = nrec_hot_peer_count();

		k_mutex_lock(&nrec_lock, K_FOREVER);
		memset(nrec_crc_valid, 0, sizeof(nrec_crc_valid));
		k_mutex_unlock(&nrec_lock);
		nrec_loaded = true;
		nrec_persist(true);
		restored = nrec_load();
		if (restored < 0 || restored != want) {
			LOG_WRN("NodeDB: bulk import read back %d of %d records; the settings "
				"records stay authoritative until a later boot imports them",
				restored, want);
			return restored;
		}
	}
	sys_put_le32(NREC_META_MAGIC, meta);
	meta[4] = 1U;
	memset(meta + 5, 0, 3);
	if (meshtastic_bulk_write(NREC_META_ID, meta, sizeof(meta)) == 0) {
		nrec_legacy_pending = true;
		LOG_INF("NodeDB: %d node record(s) moved to the bulk store", restored);
	}
	return restored;
}

static bool nrec_image_confirmed(void)
{
#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
	/* An MCUboot revert must still find the settings records the older image
	 * reads, so they are deleted only once this image is confirmed. */
	return boot_is_img_confirmed();
#else
	return true;
#endif
}

/* The save path when the bulk store is up: delete leftover settings records (in
 * batches, once confirmed), then write the pages that changed. */
static void nrec_do_persist(void)
{
	if (nrec_legacy_pending && nrec_loaded && nrec_image_confirmed()) {
		struct mtrec_reconcile_ctx ctx = {.count = 0U, .overflow = false, .all = true};
		char name[SETTINGS_MAX_NAME_LEN + 1];

		(void)settings_load_subtree_direct(MTREC_SUBTREE, mtrec_reconcile_cb, &ctx);
		for (size_t i = 0U; i < ctx.count; i++) {
			(void)snprintk(name, sizeof(name), MTREC_SUBTREE "/%08x", ctx.orphans[i]);
			(void)settings_delete(name);
		}
		if (ctx.count > 0U) {
			LOG_INF("NodeDB: deleted %zu settings record(s) now in the bulk store",
				ctx.count);
		}
		nrec_legacy_pending = ctx.overflow;
		if (ctx.overflow) {
			mtrec_schedule_save();
		}
	}
	nrec_persist(false);
}

/* Lockdown rewrote the store (provision seals, disable writes in the clear):
 * rewrite every page in the new mode. */
static void nrec_rewrite_hook(void)
{
	nrec_persist(true);
}

#endif /* NODEDB_BULK */



static int mtrec_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t buf[MTREC_BUF_LEN];
	char full_name[SETTINGS_MAX_NAME_LEN + 1];
	meshtastic_NodeInfoLite node;
	uint32_t num;
	char *endptr;
	ssize_t read;

#if defined(NODEDB_BULK)
	/* Once the bulk store has loaded, settings records are leftovers awaiting
	 * deletion: a later full settings_load() (the BLE bring-up does one) must
	 * not lay them over the newer bulk copies. */
	if (nrec_active() && nrec_loaded) {
		return 0;
	}
#endif

	if (len > sizeof(buf) + MESHTASTIC_LOCKDOWN_SEAL_OVERHEAD) {
		LOG_WRN("Ignoring oversized persisted node record '%s' (%zu)", key, len);
		return 0;
	}

	(void)snprintk(full_name, sizeof(full_name), MTREC_SUBTREE "/%s", key);
	read = meshtastic_lockdown_read(full_name, len, read_cb, cb_arg, buf, sizeof(buf));
	if (read <= 0) {
		return 0; /* unreadable, or sealed on a locked boot: not restored */
	}
	len = (size_t)read;

	if (mtrec_decode(buf, len, &node) < 0) {
		LOG_WRN("Ignoring malformed persisted node record '%s'", key);
		return 0;
	}

	num = (uint32_t)strtoul(key, &endptr, 16);
	if (*endptr != '\0' || num == 0U || num == meshtastic_get_node_id()) {
		return 0;
	}

	mtrec_apply(num, &node);
	return 0;
}

static int mtrec_export(int (*export_func)(const char *name, const void *val, size_t val_len))
{
	char name[SETTINGS_MAX_NAME_LEN + 1];
	uint8_t buf[MTREC_BUF_LEN];
	uint32_t self = meshtastic_get_node_id();
	int ret = 0;

#if defined(NODEDB_BULK)
	if (nrec_active()) {
		return 0; /* the records live in the bulk store */
	}
#endif

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		struct nodedb_entry *e = &nodedb_entries[i];
		int len;

		if (!e->used || e->node.num == self) {
			continue;
		}

		len = mtrec_encode(e, buf, sizeof(buf));
		if (len < 0) {
			continue; /* skip a record that will not encode, keep the rest */
		}

		(void)snprintk(name, sizeof(name), MTREC_SUBTREE "/%08x", e->node.num);
		ret = meshtastic_lockdown_export(export_func, name, buf, (size_t)len);
		if (ret < 0) {
			break;
		}
	}
	k_mutex_unlock(&nodedb_lock);

	return ret;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtrec, MTREC_SUBTREE, NULL, mtrec_set, NULL, mtrec_export);
#endif /* CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS */

static bool decode_user_payload(const uint8_t *payload, size_t payload_len, meshtastic_User *user)
{
	pb_istream_t stream;

	if (payload == NULL || payload_len == 0U) {
		return false;
	}

	stream = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&stream, meshtastic_User_fields, user)) {
		LOG_DBG("NodeDB User decode failed: %s", PB_GET_ERROR(&stream));
		return false;
	}

	return true;
}

#if defined(CONFIG_MESHTASTIC_POSITION)
static bool decode_position_payload(const uint8_t *payload, size_t payload_len,
				    meshtastic_Position *pos)
{
	pb_istream_t stream;

	if (payload == NULL || payload_len == 0U) {
		return false;
	}

	stream = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&stream, meshtastic_Position_fields, pos) || !pos->has_latitude_i ||
	    !pos->has_longitude_i) {
		return false;
	}

	return true;
}

/* Cache the last-known position, mirroring upstream's NodeDB::updatePosition()
 * for a REMOTE report (the LOCAL-position and EUD-time-stamp special cases
 * there don't apply to a peer cache). One nuance kept: a sparse remote report
 * with no timestamp must not blow away a previously-known one -- upstream's
 * own comment calls this out explicitly (`if (!slot.time) slot.time = tmp_time`). */
static void apply_position(struct nodedb_entry *entry, const meshtastic_Position *pos)
{
	uint32_t prior_time = entry->has_position ? entry->position.time : 0U;

	entry->position = (meshtastic_PositionLite){
		.latitude_i = pos->latitude_i,
		.longitude_i = pos->longitude_i,
		.altitude = pos->has_altitude ? pos->altitude : 0,
		.time = pos->time,
		.location_source = pos->location_source,
		.precision_bits = pos->precision_bits,
	};
	if (entry->position.time == 0U) {
		entry->position.time = prior_time;
	}
	entry->has_position = true;
}
#endif /* CONFIG_MESHTASTIC_POSITION */

/* C3 Phase 8d: dual-rep — read the hop fields from the decoded MeshPacket when the RF path
 * supplied one, else the flat struct (NULL-mesh public inject / test boundary). */
static bool packet_hops_away(const struct meshtastic_packet *packet,
			     const meshtastic_MeshPacket *mesh, uint8_t *hops_away)
{
	uint8_t hop_start = mesh ? (uint8_t)mesh->hop_start : packet->hop_start;
	uint8_t hop_limit = mesh ? (uint8_t)mesh->hop_limit : packet->hop_limit;

	if (hop_start == 0U || hop_start < hop_limit) {
		return false;
	}

	*hops_away = hop_start - hop_limit;
	return true;
}

static void apply_basic_packet(struct nodedb_entry *entry, const struct meshtastic_packet *packet,
			       const meshtastic_MeshPacket *mesh, uint32_t now_sec)
{
	uint8_t hops_away;
	/* C3 Phase 8d dual-rep. mesh->channel is the resolved index; snr is the float
	 * MeshPacket.rx_snr (the plan sanctions the float round-trip for this consumer —
	 * byte-identical here, rx_snr is (float)(int8) on the RF path). */
	uint8_t channel_index = mesh ? ((mesh->channel < MESHTASTIC_MAX_CHANNELS)
						? (uint8_t)mesh->channel
						: MESHTASTIC_CHANNEL_INDEX_INVALID)
				     : packet->channel_index;

	entry->node.last_heard = now_sec;
	entry->node.snr = mesh ? mesh->rx_snr : (float)packet->snr;
	/* Mark the reading present so a persisted 0 dB — a real and unremarkable
	 * value on a short link — is not indistinguishable from "never heard". */
	WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_HAS_SNR_BIT, 1);
	entry->node.channel =
		(channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) ? channel_index : 0U;
	/* node.next_hop is the *learned route to reach this node* (next-hop router),
	 * set from ACK/relay correlation — NOT the packet's outbound next_hop hint.
	 * Don't overwrite it with the incoming wire field. */
	WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_VIA_MQTT_BIT,
		  mesh ? mesh->via_mqtt : packet->via_mqtt);

	if (packet_hops_away(packet, mesh, &hops_away)) {
		entry->node.has_hops_away = true;
		entry->node.hops_away = hops_away;
	}

	nodedb_dirty = true; /* last_heard (and any new node) changed the sort order */
}

static void meshtastic_module_nodedb_on_packet(const struct meshtastic_packet *packet,
					       const meshtastic_MeshPacket *mesh)
{
	meshtastic_User user = meshtastic_User_init_zero;
	bool has_user = false;
	struct nodedb_entry *entry;
	uint32_t now_sec;
	uint32_t from;
	uint32_t portnum;
#if defined(CONFIG_MESHTASTIC_POSITION)
	meshtastic_Position pos = meshtastic_Position_init_zero;
	bool has_pos = false;
#endif

	if (packet == NULL && mesh == NULL) {
		return;
	}

	/* C3 Phase 8d: identity + portnum from the decoded MeshPacket on the RF path, flat
	 * struct on the NULL-mesh boundary. NodeDB is now MeshPacket-native like every other
	 * RX consumer — the struct is only the fallback. */
	from = mesh ? mesh->from : packet->from;
	if (from == 0U || from == meshtastic_get_node_id()) {
		return;
	}

	/* Every packet refreshes the basic record (last_heard / snr / hops); NodeInfo
	 * carries identity + pubkey, Position carries a last-known fix
	 * (agents-ooma.39). Telemetry/status are still not retained — hearing them
	 * still updates last_heard via the basic-packet path below. */
	portnum = mesh ? (uint32_t)mesh->decoded.portnum : packet->portnum;
	if (portnum == MESHTASTIC_PORT_NODEINFO) {
		has_user = decode_user_payload(mesh ? mesh->decoded.payload.bytes : packet->payload,
					       mesh ? mesh->decoded.payload.size : packet->payload_len,
					       &user);
	}
#if defined(CONFIG_MESHTASTIC_POSITION)
	else if (portnum == MESHTASTIC_PORT_POSITION) {
		has_pos = decode_position_payload(
			mesh ? mesh->decoded.payload.bytes : packet->payload,
			mesh ? mesh->decoded.payload.size : packet->payload_len, &pos);
	}
#endif

	now_sec = uptime_seconds();

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = get_or_create_entry_locked(from);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return;
	}

	apply_basic_packet(entry, packet, mesh, now_sec);

	if (has_user) {
		apply_user(entry, &user);
	}
#if defined(CONFIG_MESHTASTIC_POSITION)
	if (has_pos) {
		apply_position(entry, &pos);
	}
#endif

#if defined(CONFIG_MESHTASTIC_PKI)
	char alert_name[sizeof(own_key_alert.name)];
	bool alert = own_key_alert_take_locked(alert_name, sizeof(alert_name));
#endif
	k_mutex_unlock(&nodedb_lock);
#if defined(CONFIG_MESHTASTIC_PKI)
	if (alert) {
		own_key_alert_deliver(alert_name);
	}
#endif
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
	/* Resolve the durable epoch: heard this boot -> derive from uptime (0 if the
	 * clock is unseeded); otherwise the value carried across reboot on restore. */
	out->last_heard_epoch = (node->last_heard > 0U)
					? meshtastic_clock_uptime_to_epoch(node->last_heard)
					: entry->last_heard_epoch;
	out->snr = node->snr;
	out->channel = node->channel;
	out->next_hop = node->next_hop;
	out->via_mqtt = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_VIA_MQTT_BIT);
	out->has_hops_away = node->has_hops_away;
	out->hops_away = node->hops_away;
	out->is_favorite = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT);
	out->is_key_manually_verified =
		IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT);
	out->is_ignored = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_IGNORED_BIT);
	out->is_muted = IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_MUTED_BIT);

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

#if defined(CONFIG_MESHTASTIC_POSITION)
	out->has_position = entry->has_position;
	if (entry->has_position) {
		out->position_latitude_i = entry->position.latitude_i;
		out->position_longitude_i = entry->position.longitude_i;
		out->position_altitude = entry->position.altitude;
		out->position_time = entry->position.time;
		out->position_location_source = (uint8_t)entry->position.location_source;
		out->position_precision_bits = entry->position.precision_bits;
	}
#endif
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

/* B-8 node-list sort — mirrors upstream NodeDB::sortMeshDB: self first, then favorites, then
 * most-recently-heard first. Nodes heard this boot (uptime last_heard > 0) rank above ones
 * restored from NVS but not yet re-heard; each group is ordered by its own recency (uptime for
 * this-boot, the durable epoch for restored). */
static int nodedb_sort_cmp(const void *pa, const void *pb)
{
	const struct nodedb_entry *a = pa;
	const struct nodedb_entry *b = pb;
	uint32_t self = meshtastic_get_node_id();
	bool af, bf, ab, bb;

	if (a->used != b->used) {
		return a->used ? -1 : 1; /* unused slots sink to the end (shouldn't occur in range) */
	}
	if (a->node.num == self) {
		return -1;
	}
	if (b->node.num == self) {
		return 1;
	}
	af = IS_BIT_SET(a->node.bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT);
	bf = IS_BIT_SET(b->node.bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT);
	if (af != bf) {
		return af ? -1 : 1;
	}
	ab = (a->node.last_heard > 0U);
	bb = (b->node.last_heard > 0U);
	if (ab != bb) {
		return ab ? -1 : 1; /* heard this boot ranks above restored-not-yet-reheard */
	}
	if (ab) {
		if (a->node.last_heard != b->node.last_heard) {
			return (a->node.last_heard > b->node.last_heard) ? -1 : 1;
		}
	} else if (a->last_heard_epoch != b->last_heard_epoch) {
		return (a->last_heard_epoch > b->last_heard_epoch) ? -1 : 1;
	}
	return 0;
}

/* Throttled in-place sort (caller holds nodedb_lock). Re-sorts at most every
 * CONFIG_MESHTASTIC_NODEDB_SORT_THROTTLE_MS and only when the order is dirty, so a read burst
 * sees a stable order and a busy mesh doesn't sort on every packet. */
static void nodedb_maybe_sort_locked(void)
{
	int64_t now = k_uptime_get();

	if (!nodedb_dirty) {
		return;
	}
	if (nodedb_last_sort_ms != 0 &&
	    (now - nodedb_last_sort_ms) < CONFIG_MESHTASTIC_NODEDB_SORT_THROTTLE_MS) {
		return;
	}
	qsort(nodedb_entries, nodedb_entry_count, sizeof(nodedb_entries[0]), nodedb_sort_cmp);
	nodedb_dirty = false;
	nodedb_last_sort_ms = now;
}

int meshtastic_nodedb_get_by_index(size_t index, struct meshtastic_nodedb_node *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	nodedb_maybe_sort_locked();
	if (index >= nodedb_entry_count || !nodedb_entries[index].used) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}

	fill_snapshot(&nodedb_entries[index], out);
	k_mutex_unlock(&nodedb_lock);

	return 0;
}

int meshtastic_nodedb_get_next_after(uint32_t after_num, struct meshtastic_nodedb_node *out)
{
	const struct nodedb_entry *best = NULL;

	if (out == NULL) {
		return -EINVAL;
	}

	/* No sort: the walk's order is the node number, which nothing reorders.
	 * O(N) per call, O(N^2) for a whole stream -- ~62k comparisons at 250
	 * nodes, spread over as many frames. */
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	for (size_t i = 0U; i < nodedb_entry_count; i++) {
		const struct nodedb_entry *e = &nodedb_entries[i];

		if (e->used && e->node.num > after_num &&
		    (best == NULL || e->node.num < best->node.num)) {
			best = e;
		}
	}
	if (best == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}
	fill_snapshot(best, out);
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

	/* Protecting a node (favorite/ignored) takes it out of the eviction
	 * pool. Refuse if that would leave fewer than NODEDB_PROTECTED_RESERVE
	 * evictable slots, so a protection-saturated DB can still learn new peers.
	 * Un-protecting, non-protection bits, and re-setting an already-protected
	 * node stay unconditional. */
	if (value &&
	    (bit == NODEINFO_BITFIELD_IS_FAVORITE_BIT || bit == NODEINFO_BITFIELD_IS_IGNORED_BIT) &&
	    !node_is_protected(&entry->node) &&
	    protected_count_locked() + 1U > ARRAY_SIZE(nodedb_entries) - NODEDB_PROTECTED_RESERVE) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOSPC;
	}

	WRITE_BIT(entry->node.bitfield, bit, value);
	nodedb_dirty = true; /* favorite/ignored change reorders the list */
	k_mutex_unlock(&nodedb_lock);

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	if (bit == NODEINFO_BITFIELD_IS_FAVORITE_BIT || bit == NODEINFO_BITFIELD_IS_IGNORED_BIT ||
	    bit == NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT) {
		/* The curated set changed (favorite/ignored), or a key was verified: save
		 * promptly rather than at the next snapshot, so a verification made just
		 * before a reboot is not lost (agents-2dk3.4). */
		mtrec_reconcile = true;
		mtrec_schedule_save();
	}
#endif
	return 0;
}

int meshtastic_nodedb_set_favorite(uint32_t node_num, bool favorite)
{
	return nodedb_set_bit(node_num, NODEINFO_BITFIELD_IS_FAVORITE_BIT, favorite);
}

int meshtastic_nodedb_set_key_verified(uint32_t node_num, bool verified)
{
	return nodedb_set_bit(node_num, NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT, verified);
}

int meshtastic_nodedb_commit_pubkey(uint32_t node_num,
				    const uint8_t key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN])
{
	struct nodedb_entry *entry;
	meshtastic_NodeInfoLite *node;

	if (key == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}
	node = &entry->node;
	node->public_key.size = MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN;
	memcpy(node->public_key.bytes, key, MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	if (node_num != meshtastic_get_node_id()) {
		warm_upsert_locked(node_num, node->public_key.bytes, (uint8_t)node->role);
		nodekeys_schedule_save();
	}
#endif
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* Was guarded by CONFIG_MESHTASTIC_NODEDB_PERSIST, which does not exist, so
	 * this record save never ran (agents-2dk3.4). */
	mtrec_reconcile = true;
	mtrec_schedule_save();
#endif
	k_mutex_unlock(&nodedb_lock);
	return 0;
}

void meshtastic_nodedb_note_xeddsa_signer(uint32_t node_num)
{
	struct nodedb_entry *entry;

	if (node_num == 0U || node_num == meshtastic_get_node_id()) {
		return;
	}

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = get_or_create_entry_locked(node_num);
	if (entry != NULL) {
		WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_BIT, 1);
	}
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	warm_set_known_signer_locked(node_num, true);
#endif
	k_mutex_unlock(&nodedb_lock);
}

bool meshtastic_nodedb_is_xeddsa_signer(uint32_t node_num)
{
	struct nodedb_entry *entry;
	bool result;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry != NULL) {
		result = IS_BIT_SET(entry->node.bitfield, NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_BIT);
	} else {
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
		result = warm_get_known_signer_locked(node_num);
#else
		result = false;
#endif
	}
	k_mutex_unlock(&nodedb_lock);
	return result;
}

bool meshtastic_nodedb_is_from_or_to_favorite(uint32_t from, uint32_t to)
{
	struct nodedb_entry *e;
	bool favorite = false;

	k_mutex_lock(&nodedb_lock, K_FOREVER);

	e = find_entry_locked(from);
	if (e != NULL && IS_BIT_SET(e->node.bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT)) {
		favorite = true;
	}

	/* The broadcast address is never a stored node, so only the sender can
	 * make a broadcast favorite-relevant. Checking it would always miss. */
	if (!favorite && to != MESHTASTIC_NODE_BROADCAST) {
		e = find_entry_locked(to);
		if (e != NULL && IS_BIT_SET(e->node.bitfield, NODEINFO_BITFIELD_IS_FAVORITE_BIT)) {
			favorite = true;
		}
	}

	k_mutex_unlock(&nodedb_lock);

	return favorite;
}

int meshtastic_nodedb_set_ignored(uint32_t node_num, bool ignored)
{
	return nodedb_set_bit(node_num, NODEINFO_BITFIELD_IS_IGNORED_BIT, ignored);
}

int meshtastic_nodedb_add_contact(uint32_t node_num, const meshtastic_User *user,
				  bool manually_verified, bool should_ignore)
{
	struct nodedb_entry *entry;
	meshtastic_NodeInfoLite *node;
	size_t key_len;
	bool differs;
	int ret = 0;

	if (user == NULL || node_num == 0U || node_num == MESHTASTIC_NODE_BROADCAST ||
	    node_num == meshtastic_get_node_id()) {
		return -EINVAL;
	}
	key_len = MIN((size_t)user->public_key.size, (size_t)MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN);

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = get_or_create_entry_locked(node_num);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOMEM;
	}
	node = &entry->node;
	differs = node->public_key.size != (pb_size_t)key_len ||
		  (key_len > 0U && memcmp(node->public_key.bytes, user->public_key.bytes, key_len) != 0);

	/* A verified key is changed only by a contact that is itself verified. */
	if (IS_BIT_SET(node->bitfield, NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_BIT) &&
	    !manually_verified && node->public_key.size == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN &&
	    differs) {
		k_mutex_unlock(&nodedb_lock);
		LOG_WRN("add_contact 0x%08x refused: would replace a manually verified key",
			(unsigned int)node_num);
		return -EPERM;
	}
	/* The phone, unlike a NodeInfo off the air, may replace a pinned key:
	 * clear the pin so apply_user takes the new one (and mirrors it to the
	 * warm tier). A keyless contact leaves the stored key alone there. */
	if (key_len == MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN && differs) {
		node->public_key.size = 0;
	}
	apply_user(entry, user);
	if (!should_ignore &&
	    meshtastic_device_role() == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE) {
		/* favorite means something else to a CLIENT_BASE: stamp it heard so
		 * it is not the first eviction victim instead (reference). */
		node->last_heard = uptime_seconds();
	}
	nodedb_dirty = true;
#if defined(CONFIG_MESHTASTIC_PKI)
	char alert_name[sizeof(own_key_alert.name)];
	bool alert = own_key_alert_take_locked(alert_name, sizeof(alert_name));
#endif
	k_mutex_unlock(&nodedb_lock);
#if defined(CONFIG_MESHTASTIC_PKI)
	if (alert) {
		own_key_alert_deliver(alert_name);
	}
#endif

	if (should_ignore) {
		ret = meshtastic_nodedb_set_ignored(node_num, true);
		(void)meshtastic_nodedb_set_favorite(node_num, false);
	} else {
		if (meshtastic_device_role() != meshtastic_Config_DeviceConfig_Role_CLIENT_BASE &&
		    meshtastic_nodedb_set_favorite(node_num, true) == -ENOSPC) {
			LOG_WRN("add_contact 0x%08x: protected-node cap, not favorited",
				(unsigned int)node_num);
		}
		if (manually_verified) {
			ret = meshtastic_nodedb_set_key_verified(node_num, true);
		}
	}
	return ret == -ENOSPC ? 0 : ret;
}

int meshtastic_nodedb_toggle_muted(uint32_t node_num)
{
	struct nodedb_entry *entry;
	bool muted;

	k_mutex_lock(&nodedb_lock, K_FOREVER);
	entry = find_entry_locked(node_num);
	if (entry == NULL) {
		k_mutex_unlock(&nodedb_lock);
		return -ENOENT;
	}
	muted = !IS_BIT_SET(entry->node.bitfield, NODEINFO_BITFIELD_IS_MUTED_BIT);
	WRITE_BIT(entry->node.bitfield, NODEINFO_BITFIELD_IS_MUTED_BIT, muted);
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* The bit rides in the persisted record of a curated node. */
	if (node_is_protected(&entry->node)) {
		k_mutex_unlock(&nodedb_lock);
		mtrec_schedule_save();
		return muted ? 1 : 0;
	}
#endif
	k_mutex_unlock(&nodedb_lock);
	return muted ? 1 : 0;
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
		nodedb_dirty = true; /* swap-into-hole broke sort order */
		route_health_drop_locked(node_num);
		k_mutex_unlock(&nodedb_lock);
#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
		/* Every node is persisted now — prune the removed node's record. */
		mtrec_reconcile = true;
		mtrec_schedule_save();
#endif
		LOG_DBG("NodeDB removed 0x%08x", node_num);
		return 0;
	}
	k_mutex_unlock(&nodedb_lock);

	return -ENOENT;
}

/* An operator forget is NOT an eviction: eviction parks a peer's key and role
 * in the warm tier so a re-admitted peer keeps them (B-5, and the test
 * test_warm_tier_carries_role_on_readmit); a forget must ALSO purge that tier,
 * or a re-keyed peer (a re-flashed kit) is re-admitted from the warm slot with
 * the old, pinned key within a second and every PKC frame to it keeps failing
 * (bench, 2026-08-27). The save then prunes its NVS record. */
int meshtastic_nodedb_forget(uint32_t node_num)
{
	int ret = meshtastic_nodedb_remove(node_num);

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	k_mutex_lock(&nodedb_lock, K_FOREVER);
	{
		struct warm_key *w = warm_find_locked(node_num);

		if (w != NULL) {
			*w = (struct warm_key){0};
			nodekeys_schedule_save();
			if (ret == -ENOENT) {
				ret = 0; /* it was only warm; that is gone too */
			}
		}
	}
	k_mutex_unlock(&nodedb_lock);
#endif
	return ret;
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
	nodedb_dirty = true; /* the DB changed — force a re-sort on the next read */
	nodedb_last_sort_ms = 0;

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_KEYS)
	/* Warm keys are never favorites (self's key lives in config/security), so a
	 * DB reset clears them wholesale and flags a reconcile to prune the persisted
	 * mtnode records. */
	memset(warm_keys, 0, sizeof(warm_keys));
	nodekeys_reconcile = true;
#endif

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* Any curated node dropped by the reset must have its record pruned. */
	mtrec_reconcile = true;
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

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* Same synchronous prune+save for curated records: the reset reboot path
	 * does not flush the mtrec subtree either. Runs outside the lock. */
	(void)k_work_cancel_delayable(&mtrec_save_work);
	mtrec_do_persist();
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
#if defined(CONFIG_MESHTASTIC_NODEDB_PURGE_FOREIGN_KEYS)
	/* One-shot dev cleanup: drop any stale-format mtnode records before restoring,
	 * so the load below only ever sees the current 37 B format. */
	nodekeys_purge_foreign();
#endif

	/* Restore persisted peer public keys now the array is initialised and the
	 * settings subsystem is up (settings_subsys_init ran earlier in
	 * meshtastic_init). Runs outside the lock: nodekeys_set() takes it. */
	{
		uint32_t t0 = k_cycle_get_32();

#if defined(NODEDB_WBULK)
		if (wrec_active()) {
			(void)wrec_boot_restore();
			meshtastic_lockdown_add_rewrite_hook(wrec_rewrite_hook);
		} else {
			(void)settings_load_subtree(MTNODE_SUBTREE);
		}
#else
		(void)settings_load_subtree(MTNODE_SUBTREE);
#endif
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
		meshtastic_storage_note_load(MESHTASTIC_STORAGE_LOAD_NODE_KEYS,
					     k_cycle_get_32() - t0);
#else
		ARG_UNUSED(t0);
#endif
	}

	/* Prune any NVS records that didn't fit the warm ring on restore (or that a
	 * pre-bounded build left behind), so the durable store converges to the RAM
	 * ring. Deferred to the save-work so the flash writes happen off the boot
	 * path. */
	nodekeys_reconcile = true;
	nodekeys_schedule_save();
#endif

#if defined(CONFIG_MESHTASTIC_NODEDB_PERSIST_RECORDS)
	/* Restore persisted node records into the hot store now the settings
	 * subsystem is up. Then reconcile so any record whose node no longer fits the
	 * hot store (e.g. a shrunk MAX_NODES) is pruned off the boot path. */
	{
		uint32_t t0 = k_cycle_get_32();

#if defined(NODEDB_BULK)
		if (nrec_active()) {
			(void)nrec_boot_restore();
			meshtastic_lockdown_add_rewrite_hook(nrec_rewrite_hook);
		} else {
			(void)settings_load_subtree(MTREC_SUBTREE);
		}
#else
		(void)settings_load_subtree(MTREC_SUBTREE);
#endif
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
		meshtastic_storage_note_load(MESHTASTIC_STORAGE_LOAD_NODE_RECS,
					     k_cycle_get_32() - t0);
#else
		ARG_UNUSED(t0);
#endif
	}
	mtrec_reconcile = true;
	mtrec_schedule_save();
#if CONFIG_MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC > 0
	/* Start the periodic last-heard snapshot so recency survives future reboots. */
	(void)k_work_reschedule(&mtrec_snapshot_work,
				K_SECONDS(CONFIG_MESHTASTIC_NODEDB_PERSIST_INTERVAL_SEC));
#endif
#endif

	return (entry == NULL) ? -ENOMEM : 0;
}

#if defined(NODEDB_BULK) || defined(NODEDB_WBULK)
/* The unlock's reload: restore whichever NodeDB tables live in the bulk store,
 * keys first, as at boot. -ENODEV when the store is not mounted (the caller then
 * reloads the settings subtrees). */
int meshtastic_nodedb_bulk_reload(void)
{
	int ret = 0;

	if (!meshtastic_bulk_ready()) {
		return -ENODEV;
	}
#if defined(NODEDB_WBULK)
	ret = wrec_boot_restore();
#endif
#if defined(NODEDB_BULK)
	ret = nrec_boot_restore();
#endif
	return ret;
}
#endif
