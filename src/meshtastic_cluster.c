/* SPDX-License-Identifier: GPL-3.0
 *
 * Cluster config sync, module half (agents-xhli.4, docs/CLUSTER-SYNC-M4.md).
 *
 * Follows the same shape upstream builds every core feature with — a
 * per-portnum module (their SinglePortModule/ProtobufModule, our
 * MESHTASTIC_MODULE_DEFINE with the protobuf decoded in on_packet). The port
 * number is PRIVATE_APP (256): the range upstream reserves for private
 * applications, and the dedicated cluster channel — not the portnum — is what
 * scopes the traffic, so ingest is double-gated on both.
 *
 * M4a: digests out at a cadence, digests in compared against the local doc.
 * M4b (here): what a mismatch now DOES — the anti-entropy walk of §3.3
 * (ClusterVectorReq → stamp rows → ClusterEntryReq → entries → LWW merge) and
 * the reconciler that turns a changed document into applied configuration.
 *
 * Two properties are worth naming because they are why this needs no delivery
 * tracking, no acknowledgements and no retry queues:
 *
 *  - It is LEVEL-triggered, not edge-triggered. Nothing here has to succeed.
 *    A lost frame, a preempted responder, a peer that reboots mid-walk — all
 *    leave the two documents still differing, so the next digest states the
 *    difference again and the walk simply reruns. The only cost of failure is
 *    latency, bounded by the digest period.
 *  - Nobody ever pushes uninvited. A node that finds it is AHEAD of a peer
 *    does nothing at all; its own digest is the invitation for the peer to
 *    pull. That is what keeps a mute node (radio TX off) a full participant:
 *    it pulls over its BLE peer link, which carries unicasts (M2's divert).
 *
 * The document logic lives in meshtastic_cluster_doc.c where it is
 * unit-testable without any of this.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic/config.pb.h"
#include "zephyrtastic/cluster.pb.h"

#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_channels.h"
#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_hlc.h"
#include "meshtastic_modules.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* The wire cap (cluster.options) and the table cap must agree, and the whole
 * envelope must fit a frame's Data payload with margin for the Data wrapper. */
BUILD_ASSERT(MESHTASTIC_CLUSTER_PAYLOAD_MAX == 128U, "doc cap must match cluster.options");
BUILD_ASSERT(zephyrtastic_ClusterMessage_size + 16U <= MESHTASTIC_MAX_PAYLOAD_LEN,
	     "ClusterMessage must fit one frame");
/* The v1 allowlist's largest section must fit the payload cap (wrapper ~6 B). */
BUILD_ASSERT(meshtastic_Config_DeviceConfig_size + 8U <= MESHTASTIC_CLUSTER_PAYLOAD_MAX,
	     "DeviceConfig outgrew the cluster payload cap");

/* How many stamp rows ride in one ClusterVector, and how many keys in one
 * ClusterEntryReq. Both are wire facts (cluster.options max_count), read back
 * off the generated structs so a change there cannot silently desync the
 * buffers sized against them here. */
#define CLUSTER_VECTOR_ROWS ARRAY_SIZE(((zephyrtastic_ClusterVector *)NULL)->entries)
#define CLUSTER_PULL_KEYS   ARRAY_SIZE(((zephyrtastic_ClusterEntryReq *)NULL)->keys)

static K_MUTEX_DEFINE(cluster_lock);

/* PSRAM on the V4 family (no-op elsewhere): CPU-only, mutex-guarded, never a
 * DMA source — passes the meshtastic_ext_ram.h placement rules. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_cluster_entry
	cluster_storage[CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES];

/*
 * One exchange in each direction at a time (§3.3's rate limit), which is all
 * the state the walk needs: as the REQUESTER we are pulling from one peer, and
 * as the RESPONDER we are serving one. A second peer asking mid-serve is told
 * nothing and simply retries after its next digest — refusing is cheaper and
 * more predictable than queueing, and costs only latency.
 */
enum cluster_pull_state {
	PULL_IDLE = 0,
	PULL_SEND_VECTOR_REQ, /* mismatch seen; owe pull_peer a ClusterVectorReq */
	PULL_AWAIT_VECTOR,    /* asked; waiting for stamp rows */
	PULL_SEND_ENTRY_REQ,  /* rows diffed; owe pull_peer a ClusterEntryReq */
};

static struct {
	struct meshtastic_cluster_doc doc;
	struct meshtastic_hlc hlc;
	struct meshtastic_cluster_stats stats;
	bool channel_missing_logged;

	/* Requester half. */
	enum cluster_pull_state pull_state;
	uint32_t pull_peer;
	int64_t pull_deadline_ms;
	struct meshtastic_cluster_key pull_keys[CLUSTER_PULL_KEYS];
	uint8_t pull_key_count;

	/* Responder half: a vector walk (resumable by offset) and a burst of
	 * entry replies, each aimed at one peer. */
	uint32_t vec_dest;
	uint16_t vec_offset;
	uint32_t ent_dest;
	struct meshtastic_cluster_key ent_keys[CLUSTER_PULL_KEYS];
	uint8_t ent_count;
	uint8_t ent_next;
} cluster;

static void digest_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(digest_work, digest_work_fn);
static void tx_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tx_work, tx_work_fn);
static void reconcile_work_fn(struct k_work *work);
static K_WORK_DEFINE(reconcile_work, reconcile_work_fn);

/* ---- the v1 allowlist (D9) ------------------------------------------------ */

/* security (private key) and network (WiFi PSK) are the named bans; everything
 * unlisted is banned by default. Enforced on BOTH sides — a doc build never
 * emits one, an ingest always refuses one, and the reconciler only ever walks
 * this list — so no single mistake can carry a secret onto the channel. */
static const uint16_t shareable_sections[] = {
	meshtastic_Config_device_tag,	 meshtastic_Config_position_tag,
	meshtastic_Config_power_tag,	 meshtastic_Config_display_tag,
	meshtastic_Config_lora_tag, /* replicable; promote refuses it separately */
	meshtastic_Config_bluetooth_tag,
};

static bool section_shareable(uint16_t section)
{
	for (size_t i = 0; i < ARRAY_SIZE(shareable_sections); i++) {
		if (shareable_sections[i] == section) {
			return true;
		}
	}
	return false;
}

/*
 * The payload must BE the section its key claims.
 *
 * The document is deliberately payload-agnostic — pure table logic, no protobuf
 * — so if this check does not happen at the module boundary it happens nowhere,
 * and the allowlist above is tested against key.section alone. A well-formed key
 * could then carry anything: undecodable bytes, or a banned section wearing a
 * permitted key.
 *
 * Refusing at APPLY time is not enough, which is the whole point. An entry the
 * reconciler will never apply still replicates to every member, changes every
 * document hash, and at the BASE layer can never be withdrawn — base has no
 * tombstone by design (§2.1), so "no fleet default" is the key's ABSENCE and
 * there is no way to express its removal. One frame, permanent fleet-wide junk.
 *
 * Both doors use this: frames off the air, and records off flash at boot. Flash
 * is more trusted, but it is also where an entry accepted by an OLDER, laxer
 * build is still sitting — so validating on load is what makes the upgrade
 * clean out what the previous rules let in.
 */
static bool payload_is_its_section(uint16_t section, const uint8_t *payload, size_t len,
				   bool tombstone)
{
	/* Static: the callers are the RX thread (single-threaded module
	 * dispatch) and the one-shot settings load, never concurrently, and a
	 * decoded Config is a union of every section — too big for either
	 * stack. */
	static meshtastic_Config probe;
	pb_istream_t is;

	if (tombstone) {
		return len == 0U; /* nothing to check; accept() enforces the rest */
	}
	is = pb_istream_from_buffer(payload, len);
	probe = (meshtastic_Config)meshtastic_Config_init_zero;
	return pb_decode(&is, meshtastic_Config_fields, &probe) &&
	       probe.which_payload_variant == section;
}

/* ---- authorship (D4) ------------------------------------------------------ */

static bool author_is_master(uint32_t node_id)
{
#if defined(CONFIG_MESHTASTIC_ADMIN)
	return meshtastic_admin_node_is_trusted(node_id);
#else
	/* No admin module means no admin_key[] list to consult, so base
	 * authorship is unverifiable. Fail CLOSED: the channel PSK still bounds
	 * replication, but "anyone on the channel may rewrite fleet defaults"
	 * is not a default worth shipping. A build that wants fleet base config
	 * wants MESHTASTIC_ADMIN (default y wherever the phone API exists).
	 * Per-node entries authored by their own owner still replicate. */
	ARG_UNUSED(node_id);
	return false;
#endif
}

/*
 * Who may author what (§4):
 *
 *   base/<sec>      — a master only: the stamp's author key is in MY admin_key[].
 *   nodes/<X>/<sec> — X itself, or a master.
 *
 * Plus, at either layer, MY OWN past writes. A node restored from a factory
 * reset pulls its own entries back off its neighbours (§2.3's self-healing
 * property), and it has no reason to distrust a version it minted itself.
 *
 * What this is NOT: proof. Meshtastic PKC is X25519 ECDH *encryption* with no
 * signing primitive, so a channel member can mint a stamp claiming any author
 * and no receiver can refute it (§4 says so in as many words; ClusterEntry
 * field 15 is reserved for the signature that would fix it). The gate stops
 * accidents and honest misconfiguration, not an adversary holding the PSK.
 */
static bool entry_authorized(const struct meshtastic_cluster_key *key,
			     const struct meshtastic_hlc_stamp *stamp)
{
	uint32_t author = stamp->node_id;

	if (author == meshtastic_get_node_id()) {
		return true;
	}
	if (key->layer == MESHTASTIC_CLUSTER_LAYER_NODE && author == key->node_id) {
		return true;
	}
	return author_is_master(author);
}

/*
 * THE TABLE-EXHAUSTION GATE, and why it is separate from authorship.
 *
 * A per-node key names its owner, and nothing about the owner has to be real:
 * authorship (above) is satisfied by an author claiming to BE the owner, and a
 * channel member can claim anything (§4). So without this, one member can mint
 * entries for invented node ids until the table is full — and that damage does
 * not heal. There is no eviction (§6 defers it) and BASE entries cannot be
 * tombstoned at all, so a full table stays full and the fleet's real config can
 * no longer be stored. Rewriting config — the worse-sounding attack §4 already
 * concedes — is repairable by a master writing again. This is not.
 *
 * The gate is EXISTENCE, not permission: a per-node entry is accepted only for
 * a node this one has actually heard of, or for itself (so a wiped node can
 * still pull its own entries back — §2.3). That bounds the table by the size of
 * the real mesh instead of by an attacker's imagination. It is not a complete
 * answer — a large mesh can still outgrow a small table, and the eviction
 * policy that would finish the job is still deferred — it removes the part an
 * attacker gets for free.
 *
 * Hot tier OR warm key tier, deliberately: a node evicted from the hot store
 * whose key survives is still a node we have met, and refusing its pin would be
 * a silent, self-inflicted divergence.
 */
static bool node_owner_is_known(uint32_t node_id)
{
	struct meshtastic_nodedb_node node;
	uint8_t key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];

	if (node_id == meshtastic_get_node_id()) {
		return true;
	}
	if (meshtastic_nodedb_get(node_id, &node) == 0) {
		return true;
	}
	return meshtastic_nodedb_copy_pubkey(node_id, key) == 0;
}

/* ---- channel binding ------------------------------------------------------ */

/* The cluster channel is found by NAME at use time, never cached across
 * config writes: channels are operator-editable at runtime and a stale index
 * would aim frames at whatever channel got renumbered into the slot. */
static bool cluster_channel_index(uint8_t *out)
{
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *ch = meshtastic_channels_get(i);
		const char *name = meshtastic_channels_get_name(i);

		if (ch == NULL || !ch->has_settings ||
		    ch->role == meshtastic_Channel_Role_DISABLED) {
			continue;
		}
		if (name != NULL && strcmp(name, CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

bool meshtastic_cluster_channel_resolved(uint8_t *ch_index)
{
	uint8_t idx;
	bool ok = cluster_channel_index(&idx);

	if (ok && ch_index != NULL) {
		*ch_index = idx;
	}
	return ok;
}

/* ---- NVS persistence ------------------------------------------------------ */

#if defined(CONFIG_MESHTASTIC_SETTINGS)

/* One settings record per entry under mtclus/<l><node-hex>/<sec>; the value
 * is stamp + tombstone + payload, packed. Mirrors the hlc/config parallel-
 * subtree pattern: nothing existing is touched. */

struct cluster_rec {
	int64_t physical_ms;
	uint32_t counter;
	uint32_t author;
	uint8_t tombstone;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
} __packed;

static void rec_name(const struct meshtastic_cluster_key *key, char *buf, size_t len)
{
	snprintf(buf, len, "mtclus/%c%08x/%u",
		 key->layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n', key->node_id,
		 (unsigned int)key->section);
}

static void persist_entry(const struct meshtastic_cluster_entry *e)
{
	struct cluster_rec rec = {
		.physical_ms = e->stamp.physical_ms,
		.counter = e->stamp.counter,
		.author = e->stamp.node_id,
		.tombstone = e->tombstone ? 1U : 0U,
	};
	char name[40];
	size_t len = offsetof(struct cluster_rec, payload) + e->payload_len;

	memcpy(rec.payload, e->payload, e->payload_len);
	rec_name(&e->key, name, sizeof(name));
	if (settings_save_one(name, &rec, len) != 0) {
		LOG_WRN("cluster: persist of %s failed", name);
	}
}

static int cluster_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				void *cb_arg)
{
	struct cluster_rec rec;
	struct meshtastic_cluster_key k;
	struct meshtastic_hlc_stamp stamp;
	unsigned int node_id;
	unsigned int section;
	char layer;
	size_t plen;

	/* key is the remainder after "mtclus/": "<l><node-hex>/<sec>" */
	if (sscanf(key, "%c%8x/%u", &layer, &node_id, &section) != 3) {
		return -ENOENT;
	}
	if (len < offsetof(struct cluster_rec, payload) || len > sizeof(rec)) {
		return -EINVAL;
	}
	if (read_cb(cb_arg, &rec, len) != (ssize_t)len) {
		return -EINVAL;
	}

	k.layer = (layer == 'b') ? MESHTASTIC_CLUSTER_LAYER_BASE : MESHTASTIC_CLUSTER_LAYER_NODE;
	k.node_id = node_id;
	k.section = (uint16_t)section;
	stamp.physical_ms = rec.physical_ms;
	stamp.counter = rec.counter;
	stamp.node_id = rec.author;
	plen = len - offsetof(struct cluster_rec, payload);

	/* Flash is more trusted than the air, but not exempt: these same records
	 * may have been accepted by an older build with laxer rules, so the load
	 * path runs the static checks too and an upgrade quietly drops what it
	 * should never have stored. Authorship is deliberately NOT re-checked
	 * here — it depends on the NodeDB, and refusing a persisted entry because
	 * we have not re-learned a master's key yet would throw away this node's
	 * own configuration on every cold boot. */
	if (!section_shareable(k.section) ||
	    !payload_is_its_section(k.section, rec.payload, plen, rec.tombstone != 0U)) {
		LOG_WRN("cluster: dropping persisted %s — it does not pass the current "
			"ingest rules", key);
		return 0;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	(void)meshtastic_cluster_doc_accept(&cluster.doc, &k, &stamp, rec.tombstone != 0U,
					    rec.payload, plen);
	(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
	k_mutex_unlock(&cluster_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mtclus, "mtclus", NULL, cluster_settings_set, NULL, NULL);

#else /* !CONFIG_MESHTASTIC_SETTINGS */

/* No settings backend: the doc is RAM-only and re-converges from the fleet
 * after a reboot — slower, never wrong (level-triggered). */
static void persist_entry(const struct meshtastic_cluster_entry *e)
{
	ARG_UNUSED(e);
}

#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* ---- the reconciler (D10) -------------------------------------------------- */

/*
 * THE ORIGIN MARKER.
 *
 * D10 requires that a store write the document caused is distinguishable from a
 * human's local edit, so the former can never be lifted back into the document
 * (every applied entry would otherwise mint a fresh stamp and gossip forever).
 * The marker is not a side flag: a doc-derived write copies the DOCUMENT's
 * stamp into the store slot verbatim — merge_config takes the stamp — so
 * "this section's value came from the document" is a fact the two stamps state
 * themselves.
 *
 * That is worth more than a bool. There is nothing to keep in sync, nothing to
 * lose across a reboot (both stamps already persist), and it cannot be forged
 * by accident: a local edit mints a fresh stamp carrying THIS node's id and its
 * own counter, which can never equal a stamp minted on another node.
 */
static bool section_is_doc_derived(uint16_t section, const struct meshtastic_hlc_stamp *doc_stamp)
{
	struct meshtastic_hlc_stamp store_stamp;

	if (meshtastic_config_store_get_config_stamp((pb_size_t)section, &store_stamp) != 0) {
		return false;
	}
	return meshtastic_hlc_compare(&store_stamp, doc_stamp) == 0;
}

static void reconcile_section(uint16_t section)
{
	/* Static scratch, not stack: this runs on the system workqueue (the one
	 * whose 2 KB nRF default the M4a digest already overflowed once), the
	 * decoded Config is a union of every section, and merge_config's apply
	 * path puts a full channel table on the stack below us. The workqueue is
	 * a single executor, so one copy is all that is ever needed. */
	static uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	static meshtastic_Config cfg;
	const struct meshtastic_cluster_entry *e;
	struct meshtastic_hlc_stamp doc_stamp;
	uint16_t payload_len;
	pb_istream_t is;
	int ret;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	e = meshtastic_cluster_doc_effective(&cluster.doc, meshtastic_get_node_id(), section);
	if (e == NULL) {
		/* The document holds no opinion about this section — not the
		 * same as holding an empty one. Local config stands untouched. */
		k_mutex_unlock(&cluster_lock);
		return;
	}
	doc_stamp = e->stamp;
	payload_len = e->payload_len;
	memcpy(payload, e->payload, payload_len);
	k_mutex_unlock(&cluster_lock);

	if (section_is_doc_derived(section, &doc_stamp)) {
		return; /* already applied; re-writing would only churn */
	}

	/*
	 * THE SECTION THIS NODE REPLICATES BUT REFUSES TO ACT ON.
	 *
	 * `promote` will not CREATE a base/lora entry, because a fleet preset
	 * change permanently orphans any node that misses it and the straggler
	 * sweep (CONFIG-CONVERGENCE.md §7.9) does not exist yet. Nothing stops
	 * one ARRIVING, though, and applying it re-keys the radio: new preset,
	 * new channel hashes, node gone from the mesh. One frame from anyone
	 * holding the cluster PSK would take the whole fleet off the air, and
	 * the nodes that missed the frame could not be told.
	 *
	 * So the two halves are kept apart. The document is replicated state
	 * and this entry stays in it — refusing it at ingest instead would
	 * strand this node's document from the fleet's forever, trading a
	 * recoverable problem for a permanent one. The reconciler is the
	 * actuator, and this is where the trigger does not get pulled.
	 *
	 * When the straggler sweep lands, this is the gate that opens.
	 */
	if (section == meshtastic_Config_lora_tag) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		if (cluster.stats.sections_held == 0U) {
			k_mutex_unlock(&cluster_lock);
			LOG_WRN("cluster: HOLDING a fleet LoRa section — replicated, "
				"deliberately not applied (a missed preset change orphans a "
				"node until the straggler sweep exists)");
			k_mutex_lock(&cluster_lock, K_FOREVER);
		}
		cluster.stats.sections_held++;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	cfg = (meshtastic_Config)meshtastic_Config_init_zero;
	is = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&is, meshtastic_Config_fields, &cfg) ||
	    cfg.which_payload_variant != section) {
		LOG_WRN("cluster: entry for section %u does not decode as that section",
			(unsigned int)section);
		return;
	}

	ret = meshtastic_config_store_merge_config(&cfg, &doc_stamp);
	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (ret == 1) {
		cluster.stats.sections_applied++;
	} else if (ret == 0) {
		cluster.stats.sections_kept_local++;
	}
	k_mutex_unlock(&cluster_lock);

	if (ret == 1) {
		LOG_INF("cluster: applied section %u from the document (stamp %lld.%u by "
			"0x%08x)",
			(unsigned int)section, (long long)doc_stamp.physical_ms,
			doc_stamp.counter, doc_stamp.node_id);
	} else if (ret == 0) {
		/* Not a failure and not a retry loop: LWW said our own version
		 * is newer, so ours stands and our digest invites the fleet to
		 * take it. The operator's route to make it fleet-wide is an
		 * explicit promote (or, in M4c, a pin). */
		LOG_DBG("cluster: section %u kept (local version is newer than the document)",
			(unsigned int)section);
	} else {
		LOG_WRN("cluster: section %u merge failed (%d)", (unsigned int)section, ret);
	}
}

/* Recompute effective(me) across the whole allowlist and write what changed.
 * Cheap enough to run wholesale on any document change: six decodes, and every
 * one of them short-circuits on the stamp comparison once applied. */
static void reconcile_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	for (size_t i = 0; i < ARRAY_SIZE(shareable_sections); i++) {
		reconcile_section(shareable_sections[i]);
	}
}

/* ---- TX ------------------------------------------------------------------- */

static int send_cluster_message(const zephyrtastic_ClusterMessage *msg, uint32_t to,
				uint8_t ch_index)
{
	/* Static, not stack: every caller runs on the single system workqueue
	 * (the digest timer and the anti-entropy walk), which serialises them by
	 * construction — and this path continues into channel encryption, whose
	 * PSA stack appetite is exactly what ejected the first bench digest on a
	 * 2 KB sysworkq. Keep the frame off the stack. */
	static uint8_t buf[zephyrtastic_ClusterMessage_size];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet pkt = {0};

	if (!pb_encode(&os, zephyrtastic_ClusterMessage_fields, msg)) {
		LOG_ERR("cluster: encode failed: %s", PB_GET_ERROR(&os));
		return -EINVAL;
	}

	pkt.portnum = MESHTASTIC_PORT_PRIVATE;
	pkt.from = meshtastic_get_node_id();
	pkt.to = to;
	pkt.channel_index = ch_index;
	pkt.payload = buf;
	pkt.payload_len = os.bytes_written;
	return meshtastic_send_packet(&pkt, K_NO_WAIT);
}

static void stamp_to_pb(const struct meshtastic_hlc_stamp *in, zephyrtastic_HlcStamp *out)
{
	out->physical_ms = in->physical_ms;
	out->counter = in->counter;
	out->node_id = in->node_id;
}

static void stamp_from_pb(const zephyrtastic_HlcStamp *in, struct meshtastic_hlc_stamp *out)
{
	out->physical_ms = in->physical_ms;
	out->counter = in->counter;
	out->node_id = in->node_id;
}

static void key_to_pb(const struct meshtastic_cluster_key *in, zephyrtastic_ClusterKey *out)
{
	out->layer = (in->layer == MESHTASTIC_CLUSTER_LAYER_BASE) ? zephyrtastic_ClusterLayer_BASE
								  : zephyrtastic_ClusterLayer_NODE;
	out->node_id = in->node_id;
	out->section = in->section;
}

static void key_from_pb(const zephyrtastic_ClusterKey *in, struct meshtastic_cluster_key *out)
{
	out->layer = (in->layer == zephyrtastic_ClusterLayer_BASE)
			     ? MESHTASTIC_CLUSTER_LAYER_BASE
			     : MESHTASTIC_CLUSTER_LAYER_NODE;
	out->node_id = in->node_id;
	/* A section tag wider than the doc's key field could alias a different
	 * section; refuse it by clamping to something the allowlist rejects. */
	out->section = (in->section <= UINT16_MAX) ? (uint16_t)in->section : 0U;
}

static void pull_reset_locked(void)
{
	cluster.pull_state = PULL_IDLE;
	cluster.pull_peer = 0U;
	cluster.pull_key_count = 0U;
	cluster.pull_deadline_ms = 0;
}

/*
 * Pick ONE outbound job and describe it in @p msg; returns the destination, or
 * 0 when there is nothing to send. Called with cluster_lock held.
 *
 * Replies outrank our own requests. A peer blocked on a reply we owe it loses a
 * whole digest period if we make it wait, whereas deferring our own pull by one
 * frame gap costs nothing — we are already behind and staying behind a moment
 * longer changes nothing.
 */
static uint32_t tx_next_locked(zephyrtastic_ClusterMessage *msg)
{
	*msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;

	/* 1. An entry a peer asked for. */
	while (cluster.ent_dest != 0U && cluster.ent_next < cluster.ent_count) {
		const struct meshtastic_cluster_entry *e = meshtastic_cluster_doc_find(
			&cluster.doc, &cluster.ent_keys[cluster.ent_next]);
		uint32_t dest = cluster.ent_dest;

		cluster.ent_next++;
		if (cluster.ent_next >= cluster.ent_count) {
			cluster.ent_dest = 0U;
		}
		if (e == NULL) {
			/* Gone between request and reply. Not sending it is the
			 * whole recovery: the requester's next digest asks again. */
			continue;
		}

		msg->which_variant = zephyrtastic_ClusterMessage_entry_tag;
		key_to_pb(&e->key, &msg->variant.entry.key);
		msg->variant.entry.has_key = true;
		stamp_to_pb(&e->stamp, &msg->variant.entry.stamp);
		msg->variant.entry.has_stamp = true;
		msg->variant.entry.tombstone = e->tombstone;
		msg->variant.entry.payload.size = e->payload_len;
		memcpy(msg->variant.entry.payload.bytes, e->payload, e->payload_len);
		cluster.stats.entry_tx++;
		return dest;
	}
	cluster.ent_dest = 0U;

	/* 2. The next chunk of a stamp vector a peer asked for. Sent even when
	 * the document is empty (total = 0): the requester needs the reply to
	 * know there is nothing to pull, not a silence to time out on.
	 *
	 * The walk resumes by INDEX, so an entry arriving mid-walk (the table is
	 * sorted, so an insert shifts everything after it) can make one chunk
	 * skip or repeat a row. That costs a round of latency and nothing else:
	 * a repeated row is diffed to "already have it", a skipped one still
	 * differs and the next digest says so. Freezing a snapshot to avoid it
	 * would cost a second copy of the document to defend against a
	 * self-correcting inaccuracy. */
	if (cluster.vec_dest != 0U) {
		zephyrtastic_ClusterVector *v = &msg->variant.vector;
		uint32_t dest = cluster.vec_dest;

		msg->which_variant = zephyrtastic_ClusterMessage_vector_tag;
		v->offset = cluster.vec_offset;
		v->total = cluster.doc.count;
		while (v->entries_count < CLUSTER_VECTOR_ROWS &&
		       cluster.vec_offset < cluster.doc.count) {
			const struct meshtastic_cluster_entry *e =
				&cluster.doc.entries[cluster.vec_offset];
			zephyrtastic_KeyStamp *row = &v->entries[v->entries_count];

			key_to_pb(&e->key, &row->key);
			row->has_key = true;
			stamp_to_pb(&e->stamp, &row->stamp);
			row->has_stamp = true;
			v->entries_count++;
			cluster.vec_offset++;
		}
		if (cluster.vec_offset >= cluster.doc.count) {
			cluster.vec_dest = 0U;
		}
		cluster.stats.vector_tx++;
		return dest;
	}

	/* 3. Our own entry request, once the vector told us which keys to want. */
	if (cluster.pull_state == PULL_SEND_ENTRY_REQ) {
		zephyrtastic_ClusterEntryReq *r = &msg->variant.entry_req;
		uint32_t dest = cluster.pull_peer;

		msg->which_variant = zephyrtastic_ClusterMessage_entry_req_tag;
		for (uint8_t i = 0U; i < cluster.pull_key_count; i++) {
			key_to_pb(&cluster.pull_keys[i], &r->keys[i]);
			r->keys_count++;
		}
		/* The exchange ends here from our side. Entries arrive when they
		 * arrive; nothing waits on them, and whatever does not turn up
		 * still differs at the next digest. */
		pull_reset_locked();
		return dest;
	}

	/* 4. Our own vector request. */
	if (cluster.pull_state == PULL_SEND_VECTOR_REQ) {
		msg->which_variant = zephyrtastic_ClusterMessage_vector_req_tag;
		cluster.pull_state = PULL_AWAIT_VECTOR;
		return cluster.pull_peer;
	}

	return 0U;
}

static void tx_work_fn(struct k_work *work)
{
	/* Static for the same sysworkq-stack reason as send_cluster_message's
	 * buffer (single executor serialises access). */
	static zephyrtastic_ClusterMessage msg;
	uint8_t ch_index;
	uint32_t dest;
	int64_t now = k_uptime_get();
	k_timeout_t again = K_NO_WAIT;
	bool reschedule = false;

	ARG_UNUSED(work);

	if (!cluster_channel_index(&ch_index)) {
		/* The channel was renamed or disabled mid-walk. Drop everything
		 * pending rather than aim frames at whatever now sits in the
		 * slot; the digest timer re-announces when a channel reappears. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		pull_reset_locked();
		cluster.vec_dest = 0U;
		cluster.ent_dest = 0U;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (cluster.pull_state == PULL_AWAIT_VECTOR && now > cluster.pull_deadline_ms) {
		LOG_INF("cluster: no vector from 0x%08x — giving up this round",
			cluster.pull_peer);
		cluster.stats.pull_timed_out++;
		pull_reset_locked();
	}
	dest = tx_next_locked(&msg);
	if (dest != 0U) {
		/* Something else may still be queued behind this frame; space
		 * them so a multi-frame reply does not seize the airtime the
		 * peer needs to answer. */
		reschedule = true;
		again = K_MSEC(CONFIG_MESHTASTIC_CLUSTER_TX_GAP_MS);
	} else if (cluster.pull_state == PULL_AWAIT_VECTOR) {
		/* Nothing to send, but a deadline to enforce. */
		reschedule = true;
		again = K_MSEC(MAX(cluster.pull_deadline_ms - now, 100));
	}
	k_mutex_unlock(&cluster_lock);

	if (dest != 0U) {
		int ret = send_cluster_message(&msg, dest, ch_index);

		if (ret == -EACCES) {
			/* The send path refuses to downgrade a unicast to a
			 * node whose public key we lack into channel traffic.
			 * Nothing to work around here: NodeInfo exchange
			 * supplies the key, and the walk reruns on the next
			 * digest. */
			LOG_INF("cluster: no public key for 0x%08x yet — the walk resumes once "
				"its NodeInfo arrives", dest);
		} else if (ret != 0) {
			LOG_WRN("cluster: send to 0x%08x failed (%d)", dest, ret);
		}
	}
	if (reschedule) {
		(void)k_work_reschedule(&tx_work, again);
	}
}

/* ---- digest TX ------------------------------------------------------------ */

static uint32_t digest_period_ms(void)
{
	/* ± ~12% jitter so a fleet booted together does not sync its digests
	 * into one contention slot forever. */
	uint32_t base = CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC * MSEC_PER_SEC;
	uint32_t jitter = base / 8U;

	return base - jitter / 2U + (sys_rand32_get() % (jitter + 1U));
}

static void digest_work_fn(struct k_work *work)
{
	/* Static for the same sysworkq-stack reason as send_cluster_message's
	 * buffer (single executor serialises access). */
	static zephyrtastic_ClusterMessage msg;
	struct meshtastic_hlc_stamp max;
	uint8_t ch_index;
	int ret;

	ARG_UNUSED(work);
	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;

	if (!cluster_channel_index(&ch_index)) {
		if (!cluster.channel_missing_logged) {
			LOG_INF("cluster: no channel named \"%s\" — idle until one exists",
				CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
			cluster.channel_missing_logged = true;
		}
		(void)k_work_reschedule(&digest_work, K_MSEC(digest_period_ms()));
		return;
	}
	cluster.channel_missing_logged = false;

	msg.which_variant = zephyrtastic_ClusterMessage_digest_tag;
	k_mutex_lock(&cluster_lock, K_FOREVER);
	msg.variant.digest.doc_hash = meshtastic_cluster_doc_hash(&cluster.doc);
	msg.variant.digest.entry_count = cluster.doc.count;
	meshtastic_cluster_doc_max_stamp(&cluster.doc, &max);
	k_mutex_unlock(&cluster_lock);
	msg.variant.digest.has_max_stamp = true;
	stamp_to_pb(&max, &msg.variant.digest.max_stamp);

	ret = send_cluster_message(&msg, MESHTASTIC_NODE_BROADCAST, ch_index);
	if (ret == 0) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.digest_tx++;
		k_mutex_unlock(&cluster_lock);
	} else {
		LOG_WRN("cluster: digest send failed (%d)", ret);
	}

	(void)k_work_reschedule(&digest_work, K_MSEC(digest_period_ms()));
}

/* ---- RX ------------------------------------------------------------------- */

/* Open an anti-entropy exchange with @p peer after @p delay_ms. Called with the
 * lock held. Returns false if one is already in flight (§3.3: one per node). */
static bool pull_start_locked(uint32_t peer, uint32_t delay_ms)
{
	if (cluster.pull_state != PULL_IDLE || peer == 0U ||
	    peer == meshtastic_get_node_id()) {
		return false;
	}
	cluster.pull_state = PULL_SEND_VECTOR_REQ;
	cluster.pull_peer = peer;
	cluster.pull_key_count = 0U;
	cluster.pull_deadline_ms = k_uptime_get() + (int64_t)delay_ms +
				   (int64_t)CONFIG_MESHTASTIC_CLUSTER_SYNC_TIMEOUT_SEC *
					   MSEC_PER_SEC;
	cluster.stats.pull_started++;
	return true;
}

static void on_digest(uint32_t from, const zephyrtastic_ClusterDigest *d)
{
	struct meshtastic_hlc_stamp max;
	struct meshtastic_hlc_stamp theirs;
	uint32_t hash;
	uint16_t count;
	uint32_t delay = 0U;
	bool match;
	bool pulling = false;

	stamp_from_pb(&d->max_stamp, &theirs);

	k_mutex_lock(&cluster_lock, K_FOREVER);
	hash = meshtastic_cluster_doc_hash(&cluster.doc);
	count = cluster.doc.count;
	meshtastic_cluster_doc_max_stamp(&cluster.doc, &max);
	(void)meshtastic_hlc_observe(&cluster.hlc, &theirs);

	match = (hash == d->doc_hash) && (count == d->entry_count) &&
		(meshtastic_hlc_compare(&max, &theirs) == 0);
	if (match) {
		cluster.stats.digest_rx_match++;
	} else {
		cluster.stats.digest_rx_mismatch++;
		/* Jitter before answering: a digest is a broadcast, so every
		 * diverged member hears the same one and would otherwise pull
		 * from the same sender in the same slot. */
		delay = sys_rand32_get() % (CONFIG_MESHTASTIC_CLUSTER_PULL_DELAY_MS + 1U);
		pulling = pull_start_locked(from, delay);
	}
	k_mutex_unlock(&cluster_lock);

	if (!match) {
		LOG_INF("cluster: DIVERGED from 0x%08x — hash %08x/%u vs local %08x/%u%s", from,
			(unsigned int)d->doc_hash, (unsigned int)d->entry_count,
			(unsigned int)hash, (unsigned int)count,
			pulling ? " — pulling" : " (an exchange is already in flight)");
	} else {
		LOG_DBG("cluster: digest from 0x%08x matches (%u entries)", from, count);
	}

	if (pulling && !k_work_delayable_is_pending(&tx_work)) {
		/* Only when the worker is idle — never reschedule a pending one
		 * out to our jitter, which would push back a reply we already
		 * owe some other peer. Nothing is lost by leaving it: a pull can
		 * only start from PULL_IDLE, and the sole delay reachable from
		 * that state is the inter-frame gap, so a pending worker fires
		 * within CONFIG_MESHTASTIC_CLUSTER_TX_GAP_MS and picks the
		 * request up then. */
		(void)k_work_reschedule(&tx_work, K_MSEC(delay));
	}
}

static void on_vector_req(uint32_t from)
{
	bool serve;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	/*
	 * Only when idle — NOT when the requester is the peer we are already
	 * serving. Serving a stamp vector is the most expensive thing a peer
	 * can ask for: one ~30 byte request, up to a whole document in reply.
	 * If a repeat request restarted the walk, a peer could reset our offset
	 * to zero forever for the cost of one small frame each time, and the
	 * walk would never finish. Refusing costs the requester one digest
	 * period; the walk in flight is already answering it.
	 */
	serve = (cluster.vec_dest == 0U);
	if (serve) {
		cluster.vec_dest = from;
		cluster.vec_offset = 0U;
	} else {
		cluster.stats.tx_busy++;
	}
	k_mutex_unlock(&cluster_lock);

	if (serve) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_vector(uint32_t from, const zephyrtastic_ClusterVector *v)
{
	bool fire = false;
	bool last;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (cluster.pull_state != PULL_AWAIT_VECTOR || from != cluster.pull_peer) {
		/* A later chunk from the peer we WERE walking is not unsolicited,
		 * it is late: a full batch of keys ends the vector phase early, so
		 * the tail of a long vector arrives after we have moved on. Ignore
		 * it quietly; counting it as unsolicited would make an ordinary
		 * multi-chunk walk look like someone was talking out of turn. */
		if (from != cluster.pull_peer) {
			cluster.stats.rx_unsolicited++;
		}
		k_mutex_unlock(&cluster_lock);
		return;
	}
	cluster.stats.vector_rx++;

	/* Chunks are diffed INDEPENDENTLY — nothing is assembled. A lost chunk
	 * therefore costs only the keys it carried, and the next digest asks
	 * for them again; there is no reassembly buffer to size, stall on, or
	 * get wrong. */
	for (pb_size_t i = 0; i < v->entries_count && cluster.pull_key_count < CLUSTER_PULL_KEYS;
	     i++) {
		const zephyrtastic_KeyStamp *row = &v->entries[i];
		struct meshtastic_cluster_key key;
		struct meshtastic_hlc_stamp stamp;

		if (!row->has_key || !row->has_stamp) {
			continue;
		}
		key_from_pb(&row->key, &key);
		stamp_from_pb(&row->stamp, &stamp);
		if (!section_shareable(key.section)) {
			continue; /* never spend a round trip fetching a banned section */
		}
		(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
		if (meshtastic_cluster_doc_wants(&cluster.doc, &key, &stamp)) {
			cluster.pull_keys[cluster.pull_key_count++] = key;
		}
	}

	last = (v->total == 0U) || ((uint32_t)v->offset + v->entries_count >= v->total);
	if (cluster.pull_key_count >= CLUSTER_PULL_KEYS || last) {
		if (cluster.pull_key_count > 0U) {
			/* More than one batch of keys behind? Ask for this one;
			 * the rest still differ, so the next digest re-opens the
			 * walk. Convergence takes another round, never a hang. */
			cluster.pull_state = PULL_SEND_ENTRY_REQ;
			fire = true;
		} else {
			/* Their document holds nothing we lack. If they are the
			 * ones behind, our digest is already inviting them. */
			pull_reset_locked();
		}
	}
	k_mutex_unlock(&cluster_lock);

	if (fire) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_entry_req(uint32_t from, const zephyrtastic_ClusterEntryReq *r)
{
	bool serve;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	/* Idle only, for the same reason as the vector walk above: a repeat
	 * request must not rewind a burst that is already being served. */
	serve = (cluster.ent_dest == 0U);
	if (serve) {
		cluster.ent_dest = from;
		cluster.ent_count = 0U;
		cluster.ent_next = 0U;
		for (pb_size_t i = 0; i < r->keys_count && cluster.ent_count < CLUSTER_PULL_KEYS;
		     i++) {
			key_from_pb(&r->keys[i], &cluster.ent_keys[cluster.ent_count]);
			cluster.ent_count++;
		}
		if (cluster.ent_count == 0U) {
			cluster.ent_dest = 0U;
			serve = false;
		}
	} else {
		cluster.stats.tx_busy++;
	}
	k_mutex_unlock(&cluster_lock);

	if (serve) {
		(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	}
}

static void on_entry(uint32_t from, const zephyrtastic_ClusterEntry *e)
{
	struct meshtastic_cluster_key key;
	struct meshtastic_hlc_stamp stamp;
	bool changed = false;
	int ret;

	if (!e->has_key || !e->has_stamp) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_undecodable++;
		k_mutex_unlock(&cluster_lock);
		return;
	}
	key_from_pb(&e->key, &key);
	stamp_from_pb(&e->stamp, &stamp);

	/* The secret boundary, ingest side (D9). Refused even when well-formed
	 * and master-authored: this is the half of the ban that does not depend
	 * on every peer in the fleet running correct code. */
	if (!section_shareable(key.section)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.entry_rx_refused++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: refused section %u from 0x%08x (not shareable)",
			(unsigned int)key.section, from);
		return;
	}
	if (!entry_authorized(&key, &stamp)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.entry_rx_refused++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: refused %s entry authored by 0x%08x (not a master)",
			key.layer == MESHTASTIC_CLUSTER_LAYER_BASE ? "base" : "node",
			stamp.node_id);
		return;
	}
	if (key.layer == MESHTASTIC_CLUSTER_LAYER_NODE && !node_owner_is_known(key.node_id)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.entry_rx_refused++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: refused an entry for unknown node 0x%08x — the table is "
			"finite and never evicts, so entries for nodes that do not exist "
			"would fill it permanently", key.node_id);
		return;
	}
	/*
	 * The payload must BE the section its key claims.
	 *
	 * The document is deliberately payload-agnostic — pure table logic, no
	 * protobuf — so if this check does not happen here it happens nowhere,
	 * and the allowlist above is checked against key.section alone. A
	 * well-formed key could then carry anything: undecodable bytes, or a
	 * banned section wearing a permitted key.
	 *
	 * Refusing at APPLY time is not enough, which is the whole point. An
	 * entry the reconciler will never apply still replicates to every
	 * member, changes every document hash, and at the BASE layer can never
	 * be withdrawn — base has no tombstone by design (§2.1), so "no fleet
	 * default" is the key's absence and there is no way to express its
	 * removal. One frame, permanent fleet-wide junk. Ingest is the only
	 * place that can still say no.
	 */
	if (!payload_is_its_section(key.section, e->payload.bytes, e->payload.size,
				    e->tombstone)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.entry_rx_refused++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: refused entry for section %u from 0x%08x — payload is not "
			"that section", (unsigned int)key.section, from);
		return;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	(void)meshtastic_hlc_observe(&cluster.hlc, &stamp);
	ret = meshtastic_cluster_doc_accept(&cluster.doc, &key, &stamp, e->tombstone,
					    e->payload.bytes, e->payload.size);
	if (ret == 1) {
		persist_entry(meshtastic_cluster_doc_find(&cluster.doc, &key));
		cluster.stats.entry_rx_applied++;
		changed = true;
	} else if (ret == 0) {
		/* Replay defence, for free: a re-sent old entry carries its old
		 * stamp and loses LWW like any other stale write (§4). */
		cluster.stats.entry_rx_stale++;
	} else if (ret == -ENOSPC) {
		/* Counted apart from malformed input because it means something
		 * completely different and needs a different response: the frame
		 * was fine, this node has simply run out of table. The walk will
		 * keep asking for this key every round and keep failing — the
		 * diff predicate is not capacity-aware (tests/cluster pins that)
		 * — so a climbing number here is the fleet outgrowing
		 * MESHTASTIC_CLUSTER_MAX_ENTRIES, not an attack. */
		cluster.stats.entry_rx_no_space++;
	} else {
		cluster.stats.rx_undecodable++;
	}
	k_mutex_unlock(&cluster_lock);

	if (changed) {
		LOG_INF("cluster: merged %c/%08x/%u from 0x%08x (stamp %lld.%u by 0x%08x)",
			key.layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n', key.node_id,
			(unsigned int)key.section, from, (long long)stamp.physical_ms,
			stamp.counter, stamp.node_id);
		(void)k_work_submit(&reconcile_work);
	}
}

static void cluster_on_packet(const struct meshtastic_packet *packet,
			      const meshtastic_MeshPacket *mesh)
{
	/* Static, guarded by cluster_lock only for the doc it touches — but the
	 * decode itself is on the RX thread, and a ClusterMessage embeds a
	 * 128-byte payload plus four stamp rows. Keeping it off that stack is
	 * the same discipline the TX side follows; the module dispatch is
	 * single-threaded, so one copy suffices. */
	static zephyrtastic_ClusterMessage msg;
	const uint8_t *payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	size_t payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	uint8_t ch_index;
	pb_istream_t is;

	/*
	 * Double gate (D1): port 256 got us dispatched; only the cluster
	 * channel's frames are ours to interpret — with one exception that is
	 * not a loosening.
	 *
	 * The walk's pulls are UNICASTS, and the send path encrypts a unicast to
	 * a peer whose public key we hold with PKC rather than the channel key
	 * (meshtastic_packet.c, mirroring upstream). A PKC frame has no channel:
	 * it decrypts to channel_index 0, the PKC pseudo-channel, with
	 * pki_encrypted set. Requiring the cluster channel index would therefore
	 * refuse every vector and entry exchanged between two nodes that know
	 * each other's keys — which is every node pair the M3 admin trust
	 * already set up, i.e. exactly the fleet this feature is for. The walk
	 * would open, be answered, and have the answer thrown away.
	 *
	 * Accepting them does not widen the trust boundary, it narrows it: a PKC
	 * frame that decrypted here was addressed to THIS node and authenticated
	 * (X25519 + AES-CCM) to a sender whose key we already hold — a stronger
	 * claim than "knows the cluster PSK", which is one fleet-wide shared
	 * secret with no per-node authentication in it at all. What an accepted
	 * frame may then DO is governed the same way either way: the D9 section
	 * allowlist and the D4 authorship gate, both downstream of here.
	 *
	 * Broadcasts are never PKC, so digests still require the channel — the
	 * scoping property D1 is actually about is untouched.
	 */
	if (!(mesh ? mesh->pki_encrypted : packet->pki_encrypted)) {
		if (!cluster_channel_index(&ch_index) || packet->channel_index != ch_index) {
			k_mutex_lock(&cluster_lock, K_FOREVER);
			cluster.stats.rx_wrong_channel++;
			k_mutex_unlock(&cluster_lock);
			return;
		}
	} else if (!cluster_channel_index(&ch_index)) {
		/* Not a member: no cluster channel provisioned means this node
		 * is not participating, whatever a peer sends it. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_wrong_channel++;
		k_mutex_unlock(&cluster_lock);
		return;
	}

	msg = (zephyrtastic_ClusterMessage)zephyrtastic_ClusterMessage_init_zero;
	is = pb_istream_from_buffer(payload, payload_len);
	if (!pb_decode(&is, zephyrtastic_ClusterMessage_fields, &msg)) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_undecodable++;
		k_mutex_unlock(&cluster_lock);
		LOG_WRN("cluster: undecodable frame from 0x%08x", packet->from);
		return;
	}

	switch (msg.which_variant) {
	case zephyrtastic_ClusterMessage_digest_tag:
		on_digest(packet->from, &msg.variant.digest);
		break;
	case zephyrtastic_ClusterMessage_vector_req_tag:
		on_vector_req(packet->from);
		break;
	case zephyrtastic_ClusterMessage_vector_tag:
		on_vector(packet->from, &msg.variant.vector);
		break;
	case zephyrtastic_ClusterMessage_entry_req_tag:
		on_entry_req(packet->from, &msg.variant.entry_req);
		break;
	case zephyrtastic_ClusterMessage_entry_tag:
		on_entry(packet->from, &msg.variant.entry);
		break;
	default:
		/* M4c verbs land here; count them so a mixed-version bench is
		 * visible rather than silent. */
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_not_implemented++;
		k_mutex_unlock(&cluster_lock);
		break;
	}
}

MESHTASTIC_MODULE_DEFINE(cluster, MESHTASTIC_PORT_PRIVATE, 0, cluster_on_packet, NULL);

/* ---- promote (the only v1 writer) ---------------------------------------- */

int meshtastic_cluster_promote(uint16_t section)
{
	meshtastic_Config cfg;
	uint8_t payload[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	pb_ostream_t os = pb_ostream_from_buffer(payload, sizeof(payload));
	struct meshtastic_cluster_key key = {
		.layer = MESHTASTIC_CLUSTER_LAYER_BASE,
		.node_id = 0U,
		.section = section,
	};
	struct meshtastic_hlc_stamp stamp;
	struct meshtastic_hlc_stamp store_stamp;
	const struct meshtastic_cluster_entry *existing;
	int ret;

	if (!section_shareable(section)) {
		return -EPERM;
	}
	if (section == meshtastic_Config_lora_tag) {
		/* A fleet preset change orphans any node that misses it, and
		 * the straggler sweep (§7.9) does not exist yet. Refused, not
		 * warned: the failure is silent and permanent. */
		LOG_WRN("cluster: refusing to promote lora — a missed preset change orphans "
			"nodes until the straggler sweep exists");
		return -EPERM;
	}

	ret = meshtastic_config_store_get_config((pb_size_t)section, &cfg);
	if (ret < 0 || cfg.which_payload_variant != section) {
		return -ENOENT;
	}
	if (meshtastic_config_store_get_config_stamp((pb_size_t)section, &store_stamp) != 0) {
		return -ENOENT;
	}
	if (!pb_encode(&os, meshtastic_Config_fields, &cfg)) {
		LOG_ERR("cluster: promote encode failed: %s", PB_GET_ERROR(&os));
		return -EINVAL;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);

	/* The origin marker, enforced (D10). This section's stored value IS the
	 * fleet base already — promoting it would mint a second stamp for bytes
	 * the fleet has, and every node would then churn through an apply that
	 * changes nothing. A local edit moves the store's stamp and lifts the
	 * refusal, which is exactly the intended workflow. */
	existing = meshtastic_cluster_doc_find(&cluster.doc, &key);
	if (existing != NULL && meshtastic_hlc_compare(&store_stamp, &existing->stamp) == 0) {
		k_mutex_unlock(&cluster_lock);
		LOG_INF("cluster: section %u is already the fleet base", (unsigned int)section);
		return -EALREADY;
	}

	/* Fold the store's version for this section into the cluster clock
	 * before minting. The two clocks are separate instances, so without
	 * this a freshly promoted entry could compare OLDER than the very value
	 * it was made from and the reconciler would decline to apply it. */
	meshtastic_hlc_observe(&cluster.hlc, &store_stamp);
	meshtastic_hlc_local(&cluster.hlc, meshtastic_get_node_id(), &stamp);
	ret = meshtastic_cluster_doc_accept(&cluster.doc, &key, &stamp, false, payload,
					    os.bytes_written);
	if (ret == 1) {
		persist_entry(meshtastic_cluster_doc_find(&cluster.doc, &key));
		ret = 0;
	} else if (ret == 0) {
		/* Freshly minted stamps are strictly newer than anything seen;
		 * a stale verdict here means a logic error, not a race. */
		ret = -EINVAL;
	}
	k_mutex_unlock(&cluster_lock);

	if (ret == 0) {
		LOG_INF("cluster: promoted section %u to base (stamp %lld.%u by 0x%08x)",
			(unsigned int)section, (long long)stamp.physical_ms, stamp.counter,
			stamp.node_id);
		/* Reconcile so this node's own store records the base stamp —
		 * which is what makes the section doc-derived here too, and the
		 * refusal above meaningful. Push-on-change is M4c; until then
		 * the next digest advertises it. */
		(void)k_work_submit(&reconcile_work);
	}
	return ret;
}

int meshtastic_cluster_pull(uint32_t node_id)
{
	bool started;
	uint8_t ch_index;

	if (node_id == 0U || node_id == meshtastic_get_node_id()) {
		return -EINVAL;
	}
	if (!cluster_channel_index(&ch_index)) {
		return -ENOTCONN;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
	started = pull_start_locked(node_id, 0U);
	k_mutex_unlock(&cluster_lock);
	if (!started) {
		return -EBUSY;
	}
	(void)k_work_reschedule(&tx_work, K_NO_WAIT);
	return 0;
}

void meshtastic_cluster_digest_now(void)
{
	(void)k_work_reschedule(&digest_work, K_NO_WAIT);
}

/* ---- introspection -------------------------------------------------------- */

void meshtastic_cluster_stats_get(struct meshtastic_cluster_stats *out)
{
	k_mutex_lock(&cluster_lock, K_FOREVER);
	*out = cluster.stats;
	k_mutex_unlock(&cluster_lock);
}

const char *meshtastic_cluster_sync_state(uint32_t *peer)
{
	static const char *const names[] = {
		[PULL_IDLE] = "idle",
		[PULL_SEND_VECTOR_REQ] = "sending vector request",
		[PULL_AWAIT_VECTOR] = "awaiting vector",
		[PULL_SEND_ENTRY_REQ] = "sending entry request",
	};
	const char *s;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	s = names[cluster.pull_state];
	if (peer != NULL) {
		*peer = cluster.pull_peer;
	}
	k_mutex_unlock(&cluster_lock);
	return s;
}

uint16_t meshtastic_cluster_entry_count(void)
{
	uint16_t n;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	n = cluster.doc.count;
	k_mutex_unlock(&cluster_lock);
	return n;
}

uint32_t meshtastic_cluster_doc_hash_now(void)
{
	uint32_t h;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	h = meshtastic_cluster_doc_hash(&cluster.doc);
	k_mutex_unlock(&cluster_lock);
	return h;
}

bool meshtastic_cluster_entry_get(uint16_t idx, struct meshtastic_cluster_entry *out)
{
	bool ok = false;

	k_mutex_lock(&cluster_lock, K_FOREVER);
	if (idx < cluster.doc.count) {
		*out = cluster.doc.entries[idx];
		ok = true;
	}
	k_mutex_unlock(&cluster_lock);
	return ok;
}

/* ---- init ----------------------------------------------------------------- */

int meshtastic_cluster_init(void)
{
	meshtastic_cluster_doc_init(&cluster.doc, cluster_storage,
				    ARRAY_SIZE(cluster_storage));

#if defined(CONFIG_MESHTASTIC_SETTINGS)
	/* Persisted entries load through the settings handler; the config
	 * store's subtree load has already run by the time we are called, so
	 * pull ours explicitly rather than depending on a global load. */
	(void)settings_load_subtree("mtclus");
#endif

	(void)k_work_schedule(&digest_work, K_MSEC(digest_period_ms()));
	if (cluster.doc.count > 0U) {
		/* A document restored from NVS describes configuration this
		 * node may not currently be running (it was wiped, or the doc
		 * moved on while it was off). Reconcile before the first
		 * digest so what we advertise is what we are actually running. */
		(void)k_work_submit(&reconcile_work);
	}
	LOG_INF("cluster: up — %u entr%s, digest every ~%u s on channel \"%s\"",
		cluster.doc.count, cluster.doc.count == 1U ? "y" : "ies",
		(unsigned int)CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC,
		CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
	return 0;
}
