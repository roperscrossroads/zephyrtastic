/* SPDX-License-Identifier: GPL-3.0 */

/*
 * Cross-preset text relay, the receiving half (agents-jbrq.12).
 *
 * An ear on another preset forwards what it hears over the BLE peer link. The
 * router delivers those frames locally and, by the link-local rule, never
 * floods them onto our air. This module is the one exception it makes on
 * purpose: for broadcast TEXT, it sends a NEW packet from this node, on this
 * node's preset, carrying "[xxxx] " + the origin's text.
 *
 * Why re-originate and not forward: a public channel is named after its
 * preset, so a LongFast frame carries a different channel hash from a
 * MediumFast one and a verbatim copy is dropped on arrival (hole H1). A new
 * packet is minted under the destination's own channel. That is also why only
 * text crosses: a position or NodeInfo sent in the relay's name would describe
 * the relay.
 *
 * The receive path only DECIDES. Sending happens on the system workqueue
 * (the statusmessage module's pattern), so the RX thread never builds a frame.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_relay.h"
#include "meshtastic_utf8.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* "[abcd] " */
#define RELAY_PREFIX_LEN 7U

struct seen_entry {
	uint32_t origin;
	uint32_t text_hash;
	int64_t at_ms; /* 0 = free */
};

struct rate_bucket {
	int64_t window_start_ms;
	uint16_t used;
};

struct relay_job {
	uint8_t channel_index;
	uint8_t len;
	char text[MESHTASTIC_MAX_TEXT_LEN];
};

static struct {
	enum meshtastic_relay_dir dir;
	struct seen_entry seen[CONFIG_MESHTASTIC_RELAY_SEEN_SIZE];
	uint8_t seen_next;
	struct rate_bucket inbound;
	struct {
		uint32_t origin;
		struct rate_bucket b;
	} origins[CONFIG_MESHTASTIC_RELAY_ORIGIN_SLOTS];
	uint32_t ignore[CONFIG_MESHTASTIC_RELAY_IGNORE_MAX];
	uint8_t ignore_count;
	struct meshtastic_relay_stats stats;
} relay = {
	.dir = IS_ENABLED(CONFIG_MESHTASTIC_RELAY_INBOUND_AT_BOOT) ? MESHTASTIC_RELAY_INBOUND
								  : MESHTASTIC_RELAY_OFF,
};

static K_MUTEX_DEFINE(relay_lock);

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_job), CONFIG_MESHTASTIC_RELAY_QUEUE_SIZE, 4);

static void relay_work_fn(struct k_work *work);
static K_WORK_DEFINE(relay_work, relay_work_fn);

static enum meshtastic_relay_dir boot_direction(void)
{
	return IS_ENABLED(CONFIG_MESHTASTIC_RELAY_INBOUND_AT_BOOT) ? MESHTASTIC_RELAY_INBOUND
								   : MESHTASTIC_RELAY_OFF;
}

/* ---- pure helpers ----------------------------------------------------------- */

bool meshtastic_relay_has_prefix(const uint8_t *text, size_t len)
{
	size_t i;

	if (text == NULL || len < 4U || text[0] != '[') {
		return false;
	}
	/* 1..8 printable, non-space ASCII characters, then "] ". */
	for (i = 1U; i < len && i <= 9U; i++) {
		if (text[i] == ']') {
			return i >= 2U && i + 1U < len && text[i + 1U] == ' ';
		}
		if (text[i] <= ' ' || text[i] > '~') {
			return false;
		}
	}
	return false;
}

/* The text the relay would send, and whether it is worth sending.
 *
 * Sanitizing is the reference's own: meshtastic_utf8_sanitize() (its
 * sanitizeUtf8) cuts at the first NUL and turns every invalid UTF-8 lead byte
 * into '?'. Control characters, CR included, pass through as the reference
 * passes them.
 *
 * Refusing is ours, because a relay MULTIPLIES what it admits onto a second
 * tier: a text with nothing visible left (empty or whitespace only after the
 * NUL cut) or one that is more than a quarter replacement characters is what a
 * wrong-key decrypt that happened to parse, or a binary payload on the text
 * port, looks like (H30/R21; seen on air 2026-09-27). */
static bool text_sanitize(const uint8_t *in, size_t in_len, char *out, size_t *out_len,
			  bool *changed)
{
	size_t len;
	size_t replaced = 0U;
	bool visible = false;

	memcpy(out, in, in_len);
	out[in_len] = '\0';
	*changed = meshtastic_utf8_sanitize(out, in_len + 1U);
	len = strlen(out);
	if (len != in_len) {
		*changed = true; /* cut at a NUL */
	}
	for (size_t i = 0U; i < len; i++) {
		if (out[i] == '?' && in[i] != '?') {
			replaced++;
		}
		if ((uint8_t)out[i] > ' ') {
			visible = true;
		}
	}
	*out_len = len;
	return visible && replaced * 4U <= len;
}

/* Refusals worth a line in the log, at most one per
 * MESHTASTIC_RELAY_LOG_INTERVAL_SEC so a flooding or broken node cannot turn
 * the log into the attack. The first bytes go in hex: the only way to tell a
 * key collision from a client sending binary on the text port. */
static void log_refusal(const char *why, uint32_t from, const uint8_t *payload, size_t len,
			int64_t now)
{
	static int64_t last_ms;
	static uint32_t suppressed;
	uint32_t was_suppressed;

	k_mutex_lock(&relay_lock, K_FOREVER);
	if (last_ms != 0 && now - last_ms < (int64_t)CONFIG_MESHTASTIC_RELAY_LOG_INTERVAL_SEC * 1000) {
		suppressed++;
		relay.stats.log_suppressed++;
		k_mutex_unlock(&relay_lock);
		return;
	}
	last_ms = (now == 0) ? 1 : now;
	was_suppressed = suppressed;
	suppressed = 0U;
	k_mutex_unlock(&relay_lock);

	LOG_INF("relay: refused %s from 0x%08x (%u B)%s", why, from, (unsigned int)len,
		was_suppressed ? " (+ earlier refusals suppressed)" : "");
	if (payload != NULL && len > 0U) {
		LOG_HEXDUMP_INF(payload, MIN(len, 16U), "relay: first bytes");
	}
}

static uint32_t fnv1a(const uint8_t *s, size_t len)
{
	uint32_t h = 2166136261U;

	for (size_t i = 0U; i < len; i++) {
		h ^= s[i];
		h *= 16777619U;
	}
	return h;
}

/* Where a frame read through slot @p src goes on this preset, or
 * MESHTASTIC_CHANNEL_INDEX_INVALID. */
static uint8_t map_channel(uint8_t src)
{
	const char *here =
		meshtastic_preset_display_name(mt.modem_preset, mt.use_preset);

	if (src >= MESHTASTIC_MAX_CHANNELS) {
		return MESHTASTIC_CHANNEL_INDEX_INVALID;
	}

	if (meshtastic_channels_is_public_default(src)) {
		/* Already this preset's public channel: the ear is on our own
		 * preset, and there is nothing to cross. */
		if (strcmp(meshtastic_channels_get_name(src), here) == 0) {
			return MESHTASTIC_CHANNEL_INDEX_INVALID;
		}
		for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
			if (meshtastic_channels_is_default(i)) {
				return i;
			}
		}
		return MESHTASTIC_CHANNEL_INDEX_INVALID;
	}

	/* A preset-named channel on a simple2..9 key is another public mesh,
	 * not ours to join to anything (the operator said default keys). */
	if (meshtastic_channels_is_well_known(src)) {
		return MESHTASTIC_CHANNEL_INDEX_INVALID;
	}

	/* A custom channel hashes the same on every preset (the hash does not
	 * include it), so the frame decrypted in the very slot we use here. */
	return src;
}

/* ---- state, under relay_lock ------------------------------------------------ */

static bool seen_hit_locked(uint32_t origin, uint32_t text_hash, int64_t now)
{
	for (size_t i = 0U; i < ARRAY_SIZE(relay.seen); i++) {
		const struct seen_entry *e = &relay.seen[i];

		if (e->at_ms != 0 && e->origin == origin && e->text_hash == text_hash &&
		    now - e->at_ms < (int64_t)CONFIG_MESHTASTIC_RELAY_SEEN_TTL_SEC * 1000) {
			return true;
		}
	}
	return false;
}

static void seen_add_locked(uint32_t origin, uint32_t text_hash, int64_t now)
{
	struct seen_entry *e = &relay.seen[relay.seen_next];

	e->origin = origin;
	e->text_hash = text_hash;
	e->at_ms = (now == 0) ? 1 : now;
	relay.seen_next = (uint8_t)((relay.seen_next + 1U) % ARRAY_SIZE(relay.seen));
}

static bool rate_take_locked(struct rate_bucket *b, int64_t now)
{
	if (b->window_start_ms == 0 ||
	    now - b->window_start_ms >= (int64_t)CONFIG_MESHTASTIC_RELAY_RATE_WINDOW_SEC * 1000) {
		b->window_start_ms = (now == 0) ? 1 : now;
		b->used = 0U;
	}
	if (b->used >= CONFIG_MESHTASTIC_RELAY_RATE_MAX) {
		return false;
	}
	b->used++;
	return true;
}

/* Per origin, so one node -- misbehaving, buggy, or hostile -- cannot spend the
 * whole direction's cap and starve everyone else's text. A small table: an
 * origin not in it takes the slot whose window started longest ago. */
static bool origin_take_locked(uint32_t origin, int64_t now)
{
	size_t pick = 0U;
	struct rate_bucket *b;

	for (size_t i = 0U; i < ARRAY_SIZE(relay.origins); i++) {
		if (relay.origins[i].origin == origin && relay.origins[i].b.window_start_ms != 0) {
			pick = i;
			goto found;
		}
		if (relay.origins[i].b.window_start_ms < relay.origins[pick].b.window_start_ms) {
			pick = i;
		}
	}
	relay.origins[pick].origin = origin;
	relay.origins[pick].b.window_start_ms = 0;
found:
	b = &relay.origins[pick].b;
	if (b->window_start_ms == 0 ||
	    now - b->window_start_ms >= (int64_t)CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_WINDOW_SEC * 1000) {
		b->window_start_ms = (now == 0) ? 1 : now;
		b->used = 0U;
	}
	if (b->used >= CONFIG_MESHTASTIC_RELAY_PER_ORIGIN_MAX) {
		return false;
	}
	b->used++;
	return true;
}

static void origin_give_back_locked(uint32_t origin)
{
	for (size_t i = 0U; i < ARRAY_SIZE(relay.origins); i++) {
		if (relay.origins[i].origin == origin && relay.origins[i].b.used > 0U) {
			relay.origins[i].b.used--;
			return;
		}
	}
}

static bool ignored_locked(uint32_t from)
{
	if (from == mt.node_id) {
		return true;
	}
	for (uint8_t i = 0U; i < relay.ignore_count; i++) {
		if (relay.ignore[i] == from) {
			return true;
		}
	}
	return false;
}

/* ---- the send side ---------------------------------------------------------- */

static void relay_work_fn(struct k_work *work)
{
	struct relay_job job;

	ARG_UNUSED(work);

	while (k_msgq_get(&relay_q, &job, K_NO_WAIT) == 0) {
		struct meshtastic_packet pkt = {
			.to = MESHTASTIC_NODE_BROADCAST,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = (const uint8_t *)job.text,
			.payload_len = job.len,
			.channel_index = job.channel_index,
		};
		int ret = meshtastic_send_packet(&pkt, K_NO_WAIT);

		k_mutex_lock(&relay_lock, K_FOREVER);
		if (ret == 0) {
			relay.stats.sent++;
		} else {
			relay.stats.tx_failed++;
		}
		k_mutex_unlock(&relay_lock);
		if (ret != 0) {
			LOG_WRN("relay: send on ch %u failed (%d)", job.channel_index, ret);
		}
	}
}

/* ---- the receive side ------------------------------------------------------- */

void meshtastic_relay_on_rx(const struct meshtastic_packet *pkt, const meshtastic_MeshPacket *mesh,
			   enum meshtastic_bearer bearer)
{
	struct relay_job job;
	char text[MESHTASTIC_MAX_TEXT_LEN + 1];
	size_t text_len = 0U;
	bool changed = false;
	const char *refused = NULL;
	uint32_t text_hash;
	uint8_t dest;
	int64_t now;

	/* Only what the ear forwards. Our own air is not re-sent anywhere in v1:
	 * that is the outbound direction, which needs a transmitting ear. */
	if (pkt == NULL || bearer != MESHTASTIC_BEARER_BLE_PEER) {
		return;
	}

	now = k_uptime_get();
	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.stats.considered++;

	if (relay.dir != MESHTASTIC_RELAY_INBOUND && relay.dir != MESHTASTIC_RELAY_BOTH) {
		relay.stats.dir_off++;
		goto out;
	}
	if (pkt->to != MESHTASTIC_NODE_BROADCAST || pkt->pki_encrypted) {
		relay.stats.not_broadcast++;
		goto out;
	}
	if (pkt->portnum != MESHTASTIC_PORT_TEXT_MESSAGE) {
		relay.stats.not_text++;
		goto out;
	}
	/* A reaction (tapback) is a TEXT_MESSAGE_APP packet with Data.emoji set
	 * and reply_id naming the message it reacts to. Re-originated, it would
	 * arrive on the other tier as a bare emoji reacting to nothing there, so
	 * it does not cross (meshtastic-matrix-relay excludes them too, or renders
	 * them as a sentence). Plain replies do cross: their text stands alone. */
	if (mesh != NULL && mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
	    mesh->decoded.emoji != 0U) {
		relay.stats.reaction++;
		goto out;
	}
	if (ignored_locked(pkt->from)) {
		relay.stats.ignored++;
		goto out;
	}
	if (pkt->payload == NULL || pkt->payload_len == 0U ||
	    pkt->payload_len > MESHTASTIC_MAX_TEXT_LEN ||
	    !text_sanitize(pkt->payload, pkt->payload_len, text, &text_len, &changed)) {
		relay.stats.bad_text++;
		refused = "bad text";
		goto out;
	}
	/* Any relay's prefix, not only ours: a second relay we do not know about
	 * (someone else's code) would otherwise loop with us forever (H26b). */
	if (meshtastic_relay_has_prefix((const uint8_t *)text, text_len)) {
		relay.stats.prefixed++;
		refused = "relay prefix";
		goto out;
	}

	dest = map_channel(pkt->channel_index);
	if (dest == MESHTASTIC_CHANNEL_INDEX_INVALID) {
		relay.stats.no_mapping++;
		goto out;
	}

	/* Seen before any cap, so a repeat does not spend a token; per origin
	 * before the direction's cap, so one node cannot spend everyone's. */
	text_hash = fnv1a((const uint8_t *)text, text_len);
	if (seen_hit_locked(pkt->from, text_hash, now)) {
		relay.stats.seen++;
		goto out;
	}
	if (!origin_take_locked(pkt->from, now)) {
		relay.stats.origin_limited++;
		refused = "per-origin limit";
		goto out;
	}
	if (!rate_take_locked(&relay.inbound, now)) {
		relay.stats.rate_dropped++;
		refused = "direction cap";
		goto out;
	}

	job.channel_index = dest;
	if (text_len + RELAY_PREFIX_LEN <= MESHTASTIC_MAX_TEXT_LEN) {
		/* The origin's low 16 bits: the same four hex digits the apps show
		 * as a node's default short name. */
		snprintk(job.text, sizeof(job.text), "[%04x] ", (unsigned int)(pkt->from & 0xFFFFU));
		memcpy(job.text + RELAY_PREFIX_LEN, text, text_len);
		job.len = (uint8_t)(text_len + RELAY_PREFIX_LEN);
	} else {
		/* Drop the annotation, never the operator's text. */
		memcpy(job.text, text, text_len);
		job.len = (uint8_t)text_len;
		relay.stats.unprefixed++;
	}

	if (k_msgq_put(&relay_q, &job, K_NO_WAIT) != 0) {
		/* Nothing was sent, so nothing is spent: give back the tokens the two
		 * caps took above (agents-jbrq.12.9). */
		origin_give_back_locked(pkt->from);
		if (relay.inbound.used > 0U) {
			relay.inbound.used--;
		}
		relay.stats.queue_full++;
		refused = "send queue full";
		goto out;
	}
	seen_add_locked(pkt->from, text_hash, now);
	relay.stats.relayed++;
	if (changed) {
		relay.stats.sanitized++;
	}
	k_mutex_unlock(&relay_lock);

	(void)k_work_submit(&relay_work);
	LOG_INF("relay: 0x%08x ch %u -> ch %u (%u B%s)", pkt->from, pkt->channel_index, dest,
		job.len, changed ? ", sanitized" : "");
	return;

out:
	k_mutex_unlock(&relay_lock);
	if (refused != NULL) {
		log_refusal(refused, pkt->from, pkt->payload, pkt->payload_len, now);
	}
}

/* ---- persistence ------------------------------------------------------------ */

#if defined(CONFIG_MESHTASTIC_SETTINGS)
/*
 * The direction and the ignore list survive a reboot (mtrelay/relay). Before
 * this, a relay left running as a soak stopped silently at the first reset and
 * someone had to notice and type `relay dir in` again. Only what the operator
 * set is stored; caches and counters are not.
 */
#define RELAY_REC_VER 1U

struct relay_rec {
	uint8_t ver;
	uint8_t dir;
	uint8_t n_ignore;
	uint32_t ignore[CONFIG_MESHTASTIC_RELAY_IGNORE_MAX];
} __packed;

#define RELAY_REC_HDR_LEN 3U

/* Snapshot under the lock, write outside it. */
static void relay_save(void)
{
	struct relay_rec rec;
	size_t len;

	k_mutex_lock(&relay_lock, K_FOREVER);
	rec.ver = RELAY_REC_VER;
	rec.dir = (uint8_t)relay.dir;
	rec.n_ignore = relay.ignore_count;
	memcpy(rec.ignore, relay.ignore, sizeof(rec.ignore));
	len = RELAY_REC_HDR_LEN + (size_t)rec.n_ignore * sizeof(uint32_t);
	k_mutex_unlock(&relay_lock);

	if (settings_save_one("mtrelay/relay", &rec, len) != 0) {
		LOG_WRN("relay: settings save failed");
	}
}

static int relay_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	struct relay_rec rec;

	if (strcmp(key, "relay") != 0) {
		return -ENOENT;
	}
	if (len < RELAY_REC_HDR_LEN || len > sizeof(rec) ||
	    read_cb(cb_arg, &rec, len) != (ssize_t)len || rec.ver != RELAY_REC_VER ||
	    rec.n_ignore > CONFIG_MESHTASTIC_RELAY_IGNORE_MAX ||
	    len != RELAY_REC_HDR_LEN + (size_t)rec.n_ignore * sizeof(uint32_t)) {
		return -EINVAL;
	}
	/* Only what this image can do: a record written by a build that had the
	 * outbound direction must not switch it on here. */
	if (rec.dir != MESHTASTIC_RELAY_OFF && rec.dir != MESHTASTIC_RELAY_INBOUND) {
		LOG_WRN("relay: stored direction %u not supported, staying off", rec.dir);
		return -EINVAL;
	}

	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.dir = (enum meshtastic_relay_dir)rec.dir;
	relay.ignore_count = rec.n_ignore;
	memcpy(relay.ignore, rec.ignore, (size_t)rec.n_ignore * sizeof(uint32_t));
	k_mutex_unlock(&relay_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_relay, "mtrelay", NULL, relay_settings_set, NULL, NULL);

static void relay_forget(void)
{
	(void)settings_delete("mtrelay/relay");
}
#else
static void relay_save(void)
{
}

static void relay_forget(void)
{
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */

/* ---- control ---------------------------------------------------------------- */

int meshtastic_relay_set_direction(enum meshtastic_relay_dir dir)
{
	if (dir == MESHTASTIC_RELAY_OUTBOUND || dir == MESHTASTIC_RELAY_BOTH) {
		/* Needs an ear that transmits on its preset; v1's ear is RX-only. */
		return -ENOTSUP;
	}
	if (dir != MESHTASTIC_RELAY_OFF && dir != MESHTASTIC_RELAY_INBOUND) {
		return -EINVAL;
	}
	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.dir = dir;
	k_mutex_unlock(&relay_lock);
	relay_save();
	return 0;
}

enum meshtastic_relay_dir meshtastic_relay_get_direction(void)
{
	enum meshtastic_relay_dir dir;

	k_mutex_lock(&relay_lock, K_FOREVER);
	dir = relay.dir;
	k_mutex_unlock(&relay_lock);
	return dir;
}

int meshtastic_relay_ignore_add(uint32_t node_id)
{
	int ret = 0;

	k_mutex_lock(&relay_lock, K_FOREVER);
	for (uint8_t i = 0U; i < relay.ignore_count; i++) {
		if (relay.ignore[i] == node_id) {
			goto out;
		}
	}
	if (relay.ignore_count >= ARRAY_SIZE(relay.ignore)) {
		ret = -ENOMEM;
		goto out;
	}
	relay.ignore[relay.ignore_count++] = node_id;
	k_mutex_unlock(&relay_lock);
	relay_save();
	return 0;
out:
	k_mutex_unlock(&relay_lock);
	return ret;
}

void meshtastic_relay_ignore_clear(void)
{
	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.ignore_count = 0U;
	k_mutex_unlock(&relay_lock);
	relay_save();
}

void meshtastic_relay_stats_get(struct meshtastic_relay_stats *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&relay_lock, K_FOREVER);
	*out = relay.stats;
	k_mutex_unlock(&relay_lock);
}

void meshtastic_relay_reset(void)
{
	k_msgq_purge(&relay_q);
	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.dir = boot_direction();
	memset(relay.seen, 0, sizeof(relay.seen));
	relay.seen_next = 0U;
	memset(&relay.inbound, 0, sizeof(relay.inbound));
	memset(relay.origins, 0, sizeof(relay.origins));
	relay.ignore_count = 0U;
	memset(&relay.stats, 0, sizeof(relay.stats));
	k_mutex_unlock(&relay_lock);
	relay_forget();
}
