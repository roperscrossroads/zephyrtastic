/* SPDX-License-Identifier: GPL-3.0
 *
 * Firmware intent on the cluster document — see meshtastic_fleet.h.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "zephyrtastic/cluster.pb.h"
#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_core.h"
#include "meshtastic_ble_peer_codec.h"
#include "meshtastic_fleet.h"

LOG_MODULE_REGISTER(mt_fleet, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Decode the FW payload of @p e. False if it is not one (it always is, past
 * the ingest validator — but the document is payload-agnostic, so say so). */
static bool intent_decode(const struct meshtastic_cluster_entry *e,
			  zephyrtastic_FleetIntent *out)
{
	pb_istream_t is = pb_istream_from_buffer(e->payload, e->payload_len);

	*out = (zephyrtastic_FleetIntent)zephyrtastic_FleetIntent_init_zero;
	return !e->tombstone && pb_decode(&is, zephyrtastic_FleetIntent_fields, out);
}

static int intent_publish(uint8_t layer, uint32_t node_id, const zephyrtastic_FleetIntent *in)
{
	uint8_t buf[MESHTASTIC_CLUSTER_PAYLOAD_MAX];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	if (!pb_encode(&os, zephyrtastic_FleetIntent_fields, in)) {
		LOG_ERR("fleet: intent encode failed: %s", PB_GET_ERROR(&os));
		return -EINVAL;
	}
	return meshtastic_cluster_publish(MESHTASTIC_CLUSTER_SECTION_FW, layer, node_id, buf,
					  os.bytes_written);
}

static void row_fill(zephyrtastic_ClassIntent *r, uint8_t class_id, uint32_t version,
		     const uint8_t *hash4, uint32_t flags, uint16_t hw_model)
{
	*r = (zephyrtastic_ClassIntent)zephyrtastic_ClassIntent_init_zero;
	r->class_id = class_id;
	r->version = version;
	r->flags = flags;
	r->hw_model = hw_model;
	if (hash4 != NULL) {
		r->hash_prefix.size = 4U;
		memcpy(r->hash_prefix.bytes, hash4, 4U);
	}
}

int meshtastic_fleet_desire(uint8_t class_id, uint32_t version, const uint8_t *hash4,
			    uint32_t flags, uint16_t hw_model)
{
	struct meshtastic_cluster_entry base;
	zephyrtastic_FleetIntent intent = zephyrtastic_FleetIntent_init_zero;
	pb_size_t slot;

	if (class_id == 0U || version == 0U) {
		return -EINVAL;
	}
	/* Start from the fleet's current rows so a desire for one class never
	 * erases another's. effective(0, FW) is BASE (no node is 0). */
	if (meshtastic_cluster_effective_copy(0U, MESHTASTIC_CLUSTER_SECTION_FW, &base) &&
	    !intent_decode(&base, &intent)) {
		intent = (zephyrtastic_FleetIntent)zephyrtastic_FleetIntent_init_zero;
	}
	for (slot = 0U; slot < intent.rows_count; slot++) {
		if (intent.rows[slot].class_id == class_id) {
			break;
		}
	}
	if (slot == intent.rows_count) {
		if (slot >= ARRAY_SIZE(intent.rows)) {
			return -ENOSPC;
		}
		intent.rows_count++;
	}
	row_fill(&intent.rows[slot], class_id, version, hash4, flags, hw_model);
	return intent_publish(MESHTASTIC_CLUSTER_LAYER_BASE, 0U, &intent);
}

int meshtastic_fleet_pin(uint32_t node_id, uint32_t version, const uint8_t *hash4,
			 uint32_t flags)
{
	zephyrtastic_FleetIntent intent = zephyrtastic_FleetIntent_init_zero;

	if (node_id == 0U) {
		return -EINVAL;
	}
	if (version == 0U) {
		/* Withdraw: an empty payload at the NODE layer is a tombstone. */
		return meshtastic_cluster_publish(MESHTASTIC_CLUSTER_SECTION_FW,
						  MESHTASTIC_CLUSTER_LAYER_NODE, node_id, NULL,
						  0U);
	}
	intent.rows_count = 1U;
	row_fill(&intent.rows[0], 0U /* whatever that node is */, version, hash4, flags,
		 0U /* a pin is per node: its board is not in question */);
	return intent_publish(MESHTASTIC_CLUSTER_LAYER_NODE, node_id, &intent);
}

bool meshtastic_fleet_desired_for(uint32_t node_id, uint8_t class_id,
				  struct meshtastic_fleet_intent *out)
{
	struct meshtastic_cluster_entry e;
	zephyrtastic_FleetIntent intent;
	bool pinned;

	if (!meshtastic_cluster_effective_copy(node_id, MESHTASTIC_CLUSTER_SECTION_FW, &e) ||
	    !intent_decode(&e, &intent)) {
		return false;
	}
	pinned = (e.key.layer == MESHTASTIC_CLUSTER_LAYER_NODE);
	for (pb_size_t i = 0U; i < intent.rows_count; i++) {
		const zephyrtastic_ClassIntent *r = &intent.rows[i];

		/* A pin's wildcard row (class 0) answers for the node whatever
		 * it is; a BASE row answers only for its own class. */
		if (r->class_id == class_id || (pinned && r->class_id == 0U)) {
			memset(out, 0, sizeof(*out));
			out->class_id = (uint8_t)r->class_id;
			out->version = r->version;
			out->flags = r->flags;
			out->has_hash = (r->hash_prefix.size == 4U);
			if (out->has_hash) {
				memcpy(out->hash_prefix, r->hash_prefix.bytes, 4U);
			}
			out->hw_model = (uint16_t)r->hw_model;
			out->pinned = pinned;
			out->stamp = e.stamp;
			return true;
		}
	}
	return false;
}

/*
 * The reconciler calls this after ANY document change (it cannot tell which
 * key moved without decoding, and it should not decode intent). Fingerprint
 * the FW rows — newest stamp and count — and stay quiet unless that moved,
 * so a config change elsewhere in the document is not reported as new
 * orders. F2: log it. F3: wake the courier loop on the same edge.
 */
void meshtastic_fleet_intent_changed(void)
{
	/* This runs on the SYSTEM WORKQUEUE (from the cluster reconciler), whose
	 * 2 KB nRF default the cluster module has overflowed once already —
	 * hence its static scratch. An entry copy is ~170 bytes; keep it off
	 * that stack too. The workqueue is a single executor, so one copy is
	 * all that is ever needed. */
	static struct meshtastic_cluster_entry e;
	static struct meshtastic_hlc_stamp seen_max;
	static uint16_t seen_rows;
	struct meshtastic_hlc_stamp max = {0};
	uint16_t rows = 0U;

	for (uint16_t i = 0U; meshtastic_cluster_entry_get(i, &e); i++) {
		if (e.key.section != MESHTASTIC_CLUSTER_SECTION_FW) {
			continue;
		}
		rows++;
		if (meshtastic_hlc_newer(&e.stamp, &max)) {
			max = e.stamp;
		}
	}
	if (rows == seen_rows && meshtastic_hlc_compare(&max, &seen_max) == 0) {
		return;
	}
	seen_rows = rows;
	seen_max = max;
	LOG_INF("fleet: intent changed — %u FW row%s, newest stamp %lld.%u by 0x%08x", rows,
		rows == 1U ? "" : "s", (long long)max.physical_ms, max.counter, max.node_id);
}

/* ---- the courier's decision, pure ---------------------------------------- */

const char *meshtastic_fleet_verdict_str(enum meshtastic_fleet_verdict v)
{
	switch (v) {
	case MESHTASTIC_FLEET_PUSH:               return "push";
	case MESHTASTIC_FLEET_SKIP_NO_INTENT:     return "no-intent";
	case MESHTASTIC_FLEET_SKIP_UP_TO_DATE:    return "up-to-date";
	case MESHTASTIC_FLEET_SKIP_UNKNOWN_VER:   return "unknown-version";
	case MESHTASTIC_FLEET_SKIP_HOLD:          return "hold";
	case MESHTASTIC_FLEET_SKIP_TESTBOOT:      return "testboot";
	case MESHTASTIC_FLEET_SKIP_PAUSED:        return "paused";
	case MESHTASTIC_FLEET_SKIP_DOWNGRADE:     return "downgrade-refused";
	case MESHTASTIC_FLEET_SKIP_HW_MISMATCH:   return "wrong-board";
	default:                                  return "?";
	}
}

enum meshtastic_fleet_verdict
meshtastic_fleet_evaluate(uint32_t running, bool running_known, uint8_t beat_flags,
			  uint16_t nbr_hw_model, const struct meshtastic_fleet_intent *want)
{
	if (want == NULL) {
		return MESHTASTIC_FLEET_SKIP_NO_INTENT;
	}
	/* The class byte says what an image is for; the row's hw_model says what
	 * board that means. A row that names a board is never served to a node
	 * we KNOW to be another board — rxru's transmit-disabled V4 must not be
	 * handed a V4-R8 courier image because someone mistyped a class. Either
	 * side unstated (0) means no cross-check, not a refusal. */
	if (want->hw_model != 0U && nbr_hw_model != 0U && want->hw_model != nbr_hw_model) {
		return MESHTASTIC_FLEET_SKIP_HW_MISMATCH;
	}
	/* Consent first: a node that says HOLD or is already mid-swap is off
	 * limits whatever the intent says. */
	if (beat_flags & MESHTASTIC_BLE_PEER_FLAG_HOLD) {
		return MESHTASTIC_FLEET_SKIP_HOLD;
	}
	if (beat_flags & MESHTASTIC_BLE_PEER_FLAG_TESTBOOT) {
		return MESHTASTIC_FLEET_SKIP_TESTBOOT;
	}
	if (want->flags & MESHTASTIC_FLEET_FLAG_PAUSE) {
		return MESHTASTIC_FLEET_SKIP_PAUSED;
	}
	if (!running_known) {
		/* Cannot compare, so cannot safely act — a beat with no version
		 * is a pre-fleet node or one whose image MCUboot does not manage. */
		return MESHTASTIC_FLEET_SKIP_UNKNOWN_VER;
	}
	if (want->version == running) {
		return MESHTASTIC_FLEET_SKIP_UP_TO_DATE;
	}
	if (want->version < running) {
		/* R2: never downgrade unless the row explicitly allows it. */
		return (want->flags & MESHTASTIC_FLEET_FLAG_ALLOW_DOWNGRADE)
			       ? MESHTASTIC_FLEET_PUSH
			       : MESHTASTIC_FLEET_SKIP_DOWNGRADE;
	}
	return MESHTASTIC_FLEET_PUSH;
}

/* ==========================================================================
 * The courier loop (CONFIG_MESHTASTIC_FLEET_COURIER).
 * ========================================================================== */
