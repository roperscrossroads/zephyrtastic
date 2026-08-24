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
 * M4a: digests out at a cadence, digests in compared against the local doc
 * (mismatch = logged + counted, not yet acted on), promote-to-base as the
 * only writer, NVS persistence. The doc logic lives in
 * meshtastic_cluster_doc.c where it is unit-testable without any of this.
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

static K_MUTEX_DEFINE(cluster_lock);

/* PSRAM on the V4 family (no-op elsewhere): CPU-only, mutex-guarded, never a
 * DMA source — passes the meshtastic_ext_ram.h placement rules. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_cluster_entry
	cluster_storage[CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES];

static struct {
	struct meshtastic_cluster_doc doc;
	struct meshtastic_hlc hlc;
	struct meshtastic_cluster_stats stats;
	bool channel_missing_logged;
} cluster;

static void digest_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(digest_work, digest_work_fn);

/* ---- the v1 allowlist (D9) ------------------------------------------------ */

static bool section_shareable(uint16_t section)
{
	switch (section) {
	case meshtastic_Config_device_tag:
	case meshtastic_Config_position_tag:
	case meshtastic_Config_power_tag:
	case meshtastic_Config_display_tag:
	case meshtastic_Config_lora_tag: /* replicable; promote refuses it separately */
	case meshtastic_Config_bluetooth_tag:
		return true;
	default:
		/* security (private key) and network (WiFi PSK) are the named
		 * bans; everything unlisted is banned by default. */
		return false;
	}
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

/* ---- digest TX ------------------------------------------------------------ */

static int send_cluster_message(const zephyrtastic_ClusterMessage *msg, uint32_t to,
				uint8_t ch_index)
{
	/* Static, not stack: every caller runs on the single system workqueue
	 * (the digest timer today, M4b's pull replies later), which serialises
	 * them by construction — and this path continues into channel
	 * encryption, whose PSA stack appetite is exactly what ejected the
	 * first bench digest on a 2 KB sysworkq. Keep the frame off the stack. */
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
	msg.variant.digest.max_stamp.physical_ms = max.physical_ms;
	msg.variant.digest.max_stamp.counter = max.counter;
	msg.variant.digest.max_stamp.node_id = max.node_id;

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

static void on_digest(uint32_t from, const zephyrtastic_ClusterDigest *d)
{
	struct meshtastic_hlc_stamp max;
	struct meshtastic_hlc_stamp theirs = {
		.physical_ms = d->max_stamp.physical_ms,
		.counter = d->max_stamp.counter,
		.node_id = d->max_stamp.node_id,
	};
	uint32_t hash;
	uint16_t count;
	bool match;

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
	}
	k_mutex_unlock(&cluster_lock);

	if (!match) {
		/* M4a stops here on purpose: notice and count, so the bench
		 * measures real divergence before the sync machinery (M4b)
		 * exists to hide it. */
		LOG_INF("cluster: DIVERGED from 0x%08x — hash %08x/%u vs local %08x/%u", from,
			(unsigned int)d->doc_hash, (unsigned int)d->entry_count,
			(unsigned int)hash, (unsigned int)count);
	} else {
		LOG_DBG("cluster: digest from 0x%08x matches (%u entries)", from, count);
	}
}

static void cluster_on_packet(const struct meshtastic_packet *packet,
			      const meshtastic_MeshPacket *mesh)
{
	zephyrtastic_ClusterMessage msg = zephyrtastic_ClusterMessage_init_zero;
	const uint8_t *payload = mesh ? mesh->decoded.payload.bytes : packet->payload;
	size_t payload_len = mesh ? mesh->decoded.payload.size : packet->payload_len;
	uint8_t ch_index;
	pb_istream_t is;

	/* Double gate (D1): port 256 got us dispatched; only the cluster
	 * channel's frames are ours to interpret. */
	if (!cluster_channel_index(&ch_index) || packet->channel_index != ch_index) {
		k_mutex_lock(&cluster_lock, K_FOREVER);
		cluster.stats.rx_wrong_channel++;
		k_mutex_unlock(&cluster_lock);
		return;
	}

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
	default:
		/* Vector/entry traffic lands in M4b/M4c; count it so a
		 * mixed-version bench is visible rather than silent. */
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
	if (!pb_encode(&os, meshtastic_Config_fields, &cfg)) {
		LOG_ERR("cluster: promote encode failed: %s", PB_GET_ERROR(&os));
		return -EINVAL;
	}

	k_mutex_lock(&cluster_lock, K_FOREVER);
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
		/* Push-on-change is M4c; the next digest advertises this. */
	}
	return ret;
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
	LOG_INF("cluster: up — %u entr%s, digest every ~%u s on channel \"%s\"",
		cluster.doc.count, cluster.doc.count == 1U ? "y" : "ies",
		(unsigned int)CONFIG_MESHTASTIC_CLUSTER_DIGEST_PERIOD_SEC,
		CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
	return 0;
}
