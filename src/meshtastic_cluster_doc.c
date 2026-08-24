/* SPDX-License-Identifier: GPL-3.0
 *
 * The cluster document — pure table + merge + digest logic. See the header
 * for the contract and docs/CLUSTER-SYNC-M4.md for the design.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/crc.h>

#include "meshtastic_cluster_doc.h"

void meshtastic_cluster_doc_init(struct meshtastic_cluster_doc *doc,
				 struct meshtastic_cluster_entry *storage, uint16_t cap)
{
	doc->entries = storage;
	doc->cap = cap;
	doc->count = 0U;
	memset(storage, 0, (size_t)cap * sizeof(*storage));
}

int meshtastic_cluster_key_cmp(const struct meshtastic_cluster_key *a,
			       const struct meshtastic_cluster_key *b)
{
	if (a->layer != b->layer) {
		return (int)a->layer - (int)b->layer;
	}
	if (a->node_id != b->node_id) {
		return (a->node_id < b->node_id) ? -1 : 1;
	}
	return (int)a->section - (int)b->section;
}

/* Sorted-insert position for @p key; *found tells whether it is already there. */
static uint16_t doc_pos(const struct meshtastic_cluster_doc *doc,
			const struct meshtastic_cluster_key *key, bool *found)
{
	uint16_t i;

	*found = false;
	for (i = 0U; i < doc->count; i++) {
		int c = meshtastic_cluster_key_cmp(key, &doc->entries[i].key);

		if (c == 0) {
			*found = true;
			break;
		}
		if (c < 0) {
			break;
		}
	}
	return i;
}

const struct meshtastic_cluster_entry *
meshtastic_cluster_doc_find(const struct meshtastic_cluster_doc *doc,
			    const struct meshtastic_cluster_key *key)
{
	bool found;
	uint16_t i = doc_pos(doc, key, &found);

	return found ? &doc->entries[i] : NULL;
}

/* Structural validity of a key on its own: BASE has no owner, NODE must name
 * one, no other layer exists. Shared by accept() and wants() so the two can
 * never disagree about which rows are worth handling. */
static bool key_is_valid(const struct meshtastic_cluster_key *key)
{
	if (key->layer == MESHTASTIC_CLUSTER_LAYER_BASE) {
		return key->node_id == 0U;
	}
	if (key->layer == MESHTASTIC_CLUSTER_LAYER_NODE) {
		return key->node_id != 0U;
	}
	return false;
}

int meshtastic_cluster_doc_accept(struct meshtastic_cluster_doc *doc,
				  const struct meshtastic_cluster_key *key,
				  const struct meshtastic_hlc_stamp *stamp, bool tombstone,
				  const uint8_t *payload, size_t payload_len)
{
	struct meshtastic_cluster_entry *e;
	bool found;
	uint16_t i;

	if (payload_len > MESHTASTIC_CLUSTER_PAYLOAD_MAX) {
		return -EINVAL;
	}
	if (tombstone && payload_len != 0U) {
		return -EINVAL;
	}
	if (!tombstone && payload_len == 0U) {
		return -EINVAL;
	}
	if (!key_is_valid(key)) {
		return -EINVAL;
	}
	/* BASE never tombstones — "no fleet default" is simply the key's
	 * absence, so a BASE tombstone would be an unremovable hole. */
	if (key->layer == MESHTASTIC_CLUSTER_LAYER_BASE && tombstone) {
		return -EINVAL;
	}
	if (meshtastic_hlc_stamp_is_unset(stamp)) {
		return -EINVAL; /* an unversioned write can never win */
	}

	i = doc_pos(doc, key, &found);
	if (found) {
		if (!meshtastic_hlc_newer(stamp, &doc->entries[i].stamp)) {
			return 0; /* stale or equal — ours stands */
		}
	} else {
		if (doc->count >= doc->cap) {
			return -ENOSPC;
		}
		memmove(&doc->entries[i + 1U], &doc->entries[i],
			(size_t)(doc->count - i) * sizeof(doc->entries[0]));
		doc->count++;
	}

	e = &doc->entries[i];
	e->used = true;
	e->key = *key;
	e->stamp = *stamp;
	e->tombstone = tombstone;
	e->payload_len = (uint16_t)payload_len;
	if (payload_len > 0U) {
		memcpy(e->payload, payload, payload_len);
	}
	return 1;
}

uint32_t meshtastic_cluster_doc_hash(const struct meshtastic_cluster_doc *doc)
{
	/* Canonical row bytes, little-endian, in sorted order — two docs with
	 * the same rows hash identically regardless of arrival order (the
	 * table is arrival-order-independent by construction: sorted). */
	uint32_t crc = 0U;

	for (uint16_t i = 0U; i < doc->count; i++) {
		const struct meshtastic_cluster_entry *e = &doc->entries[i];
		uint8_t row[1 + 4 + 2 + 8 + 4 + 4 + 1];
		uint8_t *p = row;

		*p++ = e->key.layer;
		for (int b = 0; b < 4; b++) {
			*p++ = (uint8_t)(e->key.node_id >> (8 * b));
		}
		*p++ = (uint8_t)(e->key.section & 0xFFU);
		*p++ = (uint8_t)(e->key.section >> 8);
		for (int b = 0; b < 8; b++) {
			*p++ = (uint8_t)((uint64_t)e->stamp.physical_ms >> (8 * b));
		}
		for (int b = 0; b < 4; b++) {
			*p++ = (uint8_t)(e->stamp.counter >> (8 * b));
		}
		for (int b = 0; b < 4; b++) {
			*p++ = (uint8_t)(e->stamp.node_id >> (8 * b));
		}
		*p++ = e->tombstone ? 1U : 0U;

		crc = crc32_ieee_update(crc, row, sizeof(row));
	}
	return crc;
}

bool meshtastic_cluster_doc_wants(const struct meshtastic_cluster_doc *doc,
				  const struct meshtastic_cluster_key *key,
				  const struct meshtastic_hlc_stamp *stamp)
{
	const struct meshtastic_cluster_entry *mine;

	if (!key_is_valid(key) || meshtastic_hlc_stamp_is_unset(stamp)) {
		return false;
	}
	mine = meshtastic_cluster_doc_find(doc, key);
	if (mine != NULL) {
		/* An update needs no free slot, so capacity never blocks it. */
		return meshtastic_hlc_newer(stamp, &mine->stamp);
	}
	/* A new key is the "newer than unset" case accept() takes — but only if
	 * there is somewhere to put it. Asking for a row a full table must
	 * refuse turns the walk into a request loop that never terminates and
	 * never achieves anything. */
	return doc->count < doc->cap;
}

void meshtastic_cluster_doc_max_stamp(const struct meshtastic_cluster_doc *doc,
				      struct meshtastic_hlc_stamp *out)
{
	memset(out, 0, sizeof(*out));
	for (uint16_t i = 0U; i < doc->count; i++) {
		if (meshtastic_hlc_newer(&doc->entries[i].stamp, out)) {
			*out = doc->entries[i].stamp;
		}
	}
}

const struct meshtastic_cluster_entry *
meshtastic_cluster_doc_effective(const struct meshtastic_cluster_doc *doc, uint32_t node_id,
				 uint16_t section)
{
	struct meshtastic_cluster_key k = {
		.layer = MESHTASTIC_CLUSTER_LAYER_NODE,
		.node_id = node_id,
		.section = section,
	};
	const struct meshtastic_cluster_entry *e = meshtastic_cluster_doc_find(doc, &k);

	if (e != NULL && !e->tombstone) {
		return e;
	}

	k.layer = MESHTASTIC_CLUSTER_LAYER_BASE;
	k.node_id = 0U;
	e = meshtastic_cluster_doc_find(doc, &k);
	return (e != NULL && !e->tombstone) ? e : NULL;
}
