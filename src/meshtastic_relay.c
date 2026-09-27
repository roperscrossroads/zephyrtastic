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
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_relay.h"

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

/* Strict enough to refuse what a wrong-key decrypt produces: well-formed UTF-8
 * with no C0 control characters other than tab and newline. The router's
 * parse-as-Data is the first admission test; this one stops a rare garbage
 * frame that parsed from being MULTIPLIED onto a second tier (H30/R21). */
static bool text_is_valid(const uint8_t *s, size_t len)
{
	size_t i = 0U;

	if (len == 0U) {
		return false;
	}
	while (i < len) {
		uint8_t c = s[i];
		size_t n;

		if (c < 0x80U) {
			if (c < 0x20U && c != '\t' && c != '\n') {
				return false;
			}
			i++;
			continue;
		}
		if ((c & 0xE0U) == 0xC0U && c >= 0xC2U) {
			n = 1U;
		} else if ((c & 0xF0U) == 0xE0U) {
			n = 2U;
		} else if ((c & 0xF8U) == 0xF0U && c <= 0xF4U) {
			n = 3U;
		} else {
			return false;
		}
		if (i + n >= len) {
			return false; /* truncated sequence */
		}
		for (size_t k = 1U; k <= n; k++) {
			if ((s[i + k] & 0xC0U) != 0x80U) {
				return false;
			}
		}
		i += n + 1U;
	}
	return true;
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

void meshtastic_relay_on_rx(const struct meshtastic_packet *pkt, enum meshtastic_bearer bearer)
{
	struct relay_job job;
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
	if (ignored_locked(pkt->from)) {
		relay.stats.ignored++;
		goto out;
	}
	if (pkt->payload == NULL || pkt->payload_len > MESHTASTIC_MAX_TEXT_LEN ||
	    !text_is_valid(pkt->payload, pkt->payload_len)) {
		relay.stats.bad_text++;
		goto out;
	}
	/* Any relay's prefix, not only ours: a second relay we do not know about
	 * (someone else's code) would otherwise loop with us forever (H26b). */
	if (meshtastic_relay_has_prefix(pkt->payload, pkt->payload_len)) {
		relay.stats.prefixed++;
		goto out;
	}

	dest = map_channel(pkt->channel_index);
	if (dest == MESHTASTIC_CHANNEL_INDEX_INVALID) {
		relay.stats.no_mapping++;
		goto out;
	}

	/* Seen before the rate cap, so a repeat does not spend a token. */
	text_hash = fnv1a(pkt->payload, pkt->payload_len);
	if (seen_hit_locked(pkt->from, text_hash, now)) {
		relay.stats.seen++;
		goto out;
	}
	if (!rate_take_locked(&relay.inbound, now)) {
		relay.stats.rate_dropped++;
		goto out;
	}

	job.channel_index = dest;
	if (pkt->payload_len + RELAY_PREFIX_LEN <= MESHTASTIC_MAX_TEXT_LEN) {
		/* The origin's low 16 bits: the same four hex digits the apps show
		 * as a node's default short name. */
		snprintk(job.text, sizeof(job.text), "[%04x] ", (unsigned int)(pkt->from & 0xFFFFU));
		memcpy(job.text + RELAY_PREFIX_LEN, pkt->payload, pkt->payload_len);
		job.len = (uint8_t)(pkt->payload_len + RELAY_PREFIX_LEN);
	} else {
		/* Drop the annotation, never the operator's text. */
		memcpy(job.text, pkt->payload, pkt->payload_len);
		job.len = (uint8_t)pkt->payload_len;
		relay.stats.unprefixed++;
	}

	if (k_msgq_put(&relay_q, &job, K_NO_WAIT) != 0) {
		relay.stats.queue_full++;
		goto out;
	}
	seen_add_locked(pkt->from, text_hash, now);
	relay.stats.relayed++;
	k_mutex_unlock(&relay_lock);

	(void)k_work_submit(&relay_work);
	LOG_INF("relay: 0x%08x ch %u -> ch %u (%u B)", pkt->from, pkt->channel_index, dest,
		job.len);
	return;

out:
	k_mutex_unlock(&relay_lock);
}

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
	return 0;
}

enum meshtastic_relay_dir meshtastic_relay_get_direction(void)
{
	return relay.dir;
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
out:
	k_mutex_unlock(&relay_lock);
	return ret;
}

void meshtastic_relay_ignore_clear(void)
{
	k_mutex_lock(&relay_lock, K_FOREVER);
	relay.ignore_count = 0U;
	k_mutex_unlock(&relay_lock);
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
	relay.ignore_count = 0U;
	memset(&relay.stats, 0, sizeof(relay.stats));
	k_mutex_unlock(&relay_lock);
}
