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
#if defined(CONFIG_MESHTASTIC_FLEET_COURIER)

#include <zephyr/bluetooth/addr.h>

#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_registry.h"
#include "meshtastic_ble_peer_codec.h"
#include "meshtastic_smp_central.h"

#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
#include <zephyr/dfu/mcuboot.h>
#endif

#define COURIER_TICK        K_SECONDS(CONFIG_MESHTASTIC_FLEET_COURIER_TICK_SEC)
#define COURIER_STALE_MS    3000  /* ~3 beats: older than this, the node is not "here" */
#define COURIER_CONNECT_TO  K_SECONDS(15)
/* WAIT_CONFIRM budget: the target's confirm delay plus slack for the swap and
 * the link to re-form. */
#define COURIER_CONFIRM_MS  ((CONFIG_MESHTASTIC_OTA_CONFIRM_DELAY_SEC + 90) * 1000)
#define COURIER_BACKOFF_MIN_MS 60000
#define COURIER_BACKOFF_MAX_MS 3600000

enum courier_state {
	CS_IDLE = 0,
	CS_UPDATING,
	CS_WAIT_CONFIRM,
	CS_DONE,
	CS_REVERTED,
};

struct courier_nbr {
	bool present;
	uint32_t node;
	uint8_t class_id;
	uint32_t running;   /* packed; 0 = unknown */
	bool running_known;
	uint8_t flags;
	int64_t last_beat_ms;
	enum courier_state state;
	uint32_t attempts;
	int64_t next_try_ms;
	uint32_t target_version; /* the version we are/were driving it to */
	int64_t confirm_deadline_ms;
	uint32_t nocargo_version; /* last wanted version logged as "no classed cargo" */
	uint16_t hw_model;       /* from our NodeDB, 0 if we hold no NodeInfo for it */
	bool hw_logged;          /* the wrong-board refusal was logged once */
};

static struct {
	struct k_work_delayable tick;
	struct k_work_q wq;
	bool armed;
	int active_slot; /* the slot with a job in flight, or -1 */
	struct courier_nbr nbr[MESHTASTIC_BLE_REG_SLOTS];
} courier = { .active_slot = -1 };

/* 4096, not 2048: the tick thread does the blocking work — a depot directory
 * scan that parses each image's MCUboot header, then the SMP connect into the
 * BT host — and 2048 overflowed on the ESP32 (Xtensa register windows) the
 * instant the loop was armed. Matches the smpc job stack. */
static K_THREAD_STACK_DEFINE(courier_stack, 4096);
static K_MUTEX_DEFINE(courier_lock);

static const char *courier_state_str(enum courier_state s)
{
	switch (s) {
	case CS_IDLE:         return "idle";
	case CS_UPDATING:     return "updating";
	case CS_WAIT_CONFIRM: return "wait-confirm";
	case CS_DONE:         return "done";
	case CS_REVERTED:     return "reverted";
	default:              return "?";
	}
}

static uint32_t beat_version_packed(const struct meshtastic_ble_peer_beat *b, bool *known)
{
	uint32_t v = ((uint32_t)b->fw_major << 24) | ((uint32_t)b->fw_minor << 16) | b->fw_revision;

	*known = (v != 0U);
	return v;
}

/* Refresh the per-slot view from the beats each slot last stored. */
static void courier_refresh(void)
{
	int64_t now = k_uptime_get();

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		struct courier_nbr *n = &courier.nbr[i];
		struct meshtastic_ble_peer_rx rx;
		int64_t last_ms = 0;
		bool fresh;

		if (meshtastic_ble_reg_kind(i) != MESHTASTIC_BLE_CONN_PEER ||
		    !meshtastic_ble_peer_rx_get(i, &rx, &last_ms)) {
			n->present = false;
			continue;
		}
		fresh = (now - last_ms) <= COURIER_STALE_MS;
		if (!fresh || rx.last.version < 2U) {
			/* A pre-fleet (v1) neighbour carries no version/flags; it
			 * is present on the link but not a courier candidate. */
			n->present = false;
			continue;
		}
		if (n->node != rx.last.node_num) {
			/* A different node in this slot. A node that re-links after
			 * its swap usually lands in a DIFFERENT slot (seen on the
			 * bench: slot 1 -> conn 0), so first look for its record
			 * elsewhere and carry it here — state, attempts, the version
			 * being driven, the confirm deadline — or WAIT_CONFIRM would
			 * sit in a slot that never sees its beats again while a clean
			 * IDLE row for the same node appears beside it. */
			int from = -1;

			for (unsigned int k = 0U; k < MESHTASTIC_BLE_REG_SLOTS; k++) {
				if (k != i && courier.nbr[k].node == rx.last.node_num) {
					from = (int)k;
					break;
				}
			}
			if (from >= 0) {
				*n = courier.nbr[from];
				memset(&courier.nbr[from], 0, sizeof(courier.nbr[from]));
				if (courier.active_slot == from) {
					courier.active_slot = (int)i;
				}
			} else {
				memset(n, 0, sizeof(*n));
			}
		}
		n->present = true;
		n->node = rx.last.node_num;
		n->class_id = rx.last.class_id;
		{
			/* What board it is, from its NodeInfo — the cross-check for a
			 * row that names one. Missing NodeInfo -> 0 -> no check. */
			struct meshtastic_nodedb_node nd;

			n->hw_model = (meshtastic_nodedb_get(n->node, &nd) == 0) ? nd.hw_model : 0U;
		}
		n->running = beat_version_packed(&rx.last, &n->running_known);
		n->flags = rx.last.flags;
		n->last_beat_ms = last_ms;
	}
}

/* Advance a neighbour we have a completed/failed job for, and any waiting for
 * confirmation. Returns with active_slot cleared if the job ended. */
static void courier_poll_active(void)
{
	struct meshtastic_smpc_job job;
	struct courier_nbr *n;

	if (courier.active_slot < 0) {
		return;
	}
	n = &courier.nbr[courier.active_slot];
	meshtastic_smpc_job_get(&job);
	if (job.running) {
		return; /* still working */
	}

	/* The job ended: give the link back. It is the peer link — the target
	 * opened it and the peer-link module owns it; smpc only adopted it for
	 * the job — and while smpc holds it subscribed, the target's beats stop
	 * reaching us, which would leave WAIT_CONFIRM blind. The return is an
	 * unsubscribe plus dropping smpc's ref, never a disconnect. (Bug #2,
	 * 2026-08-27: the return crashed the BT RX thread because smpc wiped its
	 * subscribe params while the unsubscribe was still in flight; fixed in
	 * smpc, see smpc_link_reset_locked.) */
	(void)meshtastic_smpc_disconnect();
	if (job.rc == 0) {
		n->state = CS_WAIT_CONFIRM;
		n->confirm_deadline_ms = k_uptime_get() + COURIER_CONFIRM_MS;
		LOG_INF("courier: 0x%08x delivered, awaiting its self-confirm", n->node);
	} else if (strcmp(job.fail_stage, "reboot-wait") == 0) {
		/* We reset it into the delivered image and it never came back
		 * within the budget. That is a failed test boot as far as this
		 * courier can know (bench 2026-08-27: a faulting image reverted
		 * and the kit then sat in its UF2 bootloader) — and retrying
		 * would knock it over again every backoff. Latch, like a revert
		 * seen through the beats; `fleet clear` is the operator's retry. */
		n->state = CS_REVERTED;
		LOG_WRN("courier: 0x%08x never came back after the swap to %u.%u.%u — latched as REVERTED, `fleet clear` to retry",
			n->node, (unsigned int)(n->target_version >> 24),
			(unsigned int)((n->target_version >> 16) & 0xFFU),
			(unsigned int)(n->target_version & 0xFFFFU));
	} else {
		n->state = CS_IDLE;
		n->attempts++;
		n->next_try_ms = k_uptime_get() +
				 MIN(COURIER_BACKOFF_MIN_MS << MIN(n->attempts, 6U),
				     COURIER_BACKOFF_MAX_MS);
		LOG_WRN("courier: 0x%08x job failed (mgmt %d, in %s), backing off", n->node, job.rc,
			job.fail_stage);
	}
	courier.active_slot = -1;
}

/* WAIT_CONFIRM and revert bookkeeping, on the refreshed view. */
static void courier_track(void)
{
	int64_t now = k_uptime_get();

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		struct courier_nbr *n = &courier.nbr[i];

		if (n->state == CS_WAIT_CONFIRM && n->present) {
			bool testboot = (n->flags & MESHTASTIC_BLE_PEER_FLAG_TESTBOOT) != 0U;

			if (n->running_known && n->running == n->target_version && !testboot) {
				n->state = CS_DONE;
				n->attempts = 0U;
				LOG_INF("courier: 0x%08x confirmed on the delivered version", n->node);
			} else if (n->running_known && n->running != n->target_version && !testboot) {
				/* The job verified it RUNNING the delivered version; now
				 * it beats another one, confirmed: MCUboot swapped back
				 * after a bad boot. Say so now, not at the deadline. */
				n->state = CS_REVERTED;
				LOG_WRN("courier: 0x%08x REVERTED to %u.%u.%u — latched, `fleet clear` to retry",
					n->node, (unsigned int)(n->running >> 24),
					(unsigned int)((n->running >> 16) & 0xFFU),
					(unsigned int)(n->running & 0xFFFFU));
			} else if (now > n->confirm_deadline_ms) {
				n->state = CS_REVERTED;
				LOG_WRN("courier: 0x%08x never confirmed — treating as reverted", n->node);
			}
		} else if (n->state == CS_DONE && n->present && n->running_known &&
			   n->running < n->target_version) {
			/* The delivered version was live and now an older one
			 * beats back: MCUboot reverted it. Stop pushing that
			 * version to that node until an operator clears it (R4). */
			n->state = CS_REVERTED;
			LOG_WRN("courier: 0x%08x reverted after confirming — latched, `fleet clear` to retry",
				n->node);
		}
	}
}

/* Is this node allowed to be a courier right now? R7. */
static bool courier_self_ready(void)
{
#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
	if (!boot_is_img_confirmed()) {
		return false; /* an unconfirmed courier must not push itself onward */
	}
#endif
	return !meshtastic_ble_peer_hold_get();
}

/* Choose and start one job. Assumes no job is active and the node is ready. */
static void courier_maybe_start(void)
{
	int64_t now = k_uptime_get();
	int best = -1;
	char path[64];
	uint32_t want_version = 0U;

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		struct courier_nbr *n = &courier.nbr[i];
		struct meshtastic_fleet_intent want;
		bool have;
		enum meshtastic_fleet_verdict v;

		if (!n->present || n->state == CS_REVERTED || now < n->next_try_ms) {
			continue;
		}
		have = meshtastic_fleet_desired_for(n->node, n->class_id, &want);
		v = meshtastic_fleet_evaluate(n->running, n->running_known, n->flags, n->hw_model,
					      have ? &want : NULL);
		if (v == MESHTASTIC_FLEET_SKIP_HW_MISMATCH && !n->hw_logged) {
			n->hw_logged = true;
			LOG_WRN("courier: 0x%08x is hw_model %u but the class-%u row is for %u — refusing",
				n->node, n->hw_model, n->class_id, want.hw_model);
		}
		if (v != MESHTASTIC_FLEET_PUSH) {
			continue;
		}
		/* Have the bytes for that version, FOR THAT CLASS? The image's
		 * signed class TLV decides (F4); an unclassed image matches nothing. */
		if (meshtastic_smpc_depot_find(n->class_id, want.version, path, sizeof(path)) != 0) {
			/* Say so once per wanted version, not once per tick: an
			 * idle row with a want is otherwise silent about why. */
			if (n->nocargo_version != want.version) {
				n->nocargo_version = want.version;
				LOG_INF("courier: 0x%08x wants %u.%u.%u but /depot has no class-%u image of it",
					n->node, (unsigned int)(want.version >> 24),
					(unsigned int)((want.version >> 16) & 0xFFU),
					(unsigned int)(want.version & 0xFFFFU), n->class_id);
			}
			continue;
		}
		/* Sort: fewest attempts, then freshest beat, then lowest id. */
		if (best < 0 || n->attempts < courier.nbr[best].attempts ||
		    (n->attempts == courier.nbr[best].attempts &&
		     n->last_beat_ms > courier.nbr[best].last_beat_ms) ||
		    (n->attempts == courier.nbr[best].attempts &&
		     n->last_beat_ms == courier.nbr[best].last_beat_ms &&
		     n->node < courier.nbr[best].node)) {
			best = (int)i;
			want_version = want.version;
		}
	}
	if (best < 0) {
		return;
	}

	{
		struct courier_nbr *n = &courier.nbr[best];
		bt_addr_le_t addr;
		int rc;

		/* Re-resolve the path for the winner (the loop reused `path`). */
		if (meshtastic_smpc_depot_find(n->class_id, want_version, path, sizeof(path)) != 0 ||
		    !meshtastic_ble_slot_addr((unsigned int)best, &addr)) {
			return;
		}
		rc = meshtastic_smpc_connect(&addr, COURIER_CONNECT_TO);
		if (rc != 0) {
			n->attempts++;
			n->next_try_ms = now + COURIER_BACKOFF_MIN_MS;
			LOG_WRN("courier: 0x%08x connect failed (%d)", n->node, rc);
			return;
		}
		rc = meshtastic_smpc_job_start(MESHTASTIC_SMPC_JOB_UPDATE_NOCONFIRM, path);
		if (rc != 0) {
			(void)meshtastic_smpc_disconnect();
			LOG_WRN("courier: 0x%08x job start failed (%d)", n->node, rc);
			return;
		}
		n->state = CS_UPDATING;
		n->target_version = want_version;
		n->attempts++;
		courier.active_slot = best;
		LOG_INF("courier: pushing %u.%u.%u to 0x%08x (%s)",
			(unsigned int)(want_version >> 24), (unsigned int)((want_version >> 16) & 0xFFU),
			(unsigned int)(want_version & 0xFFFFU), n->node, path);
	}
}

static void courier_tick_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&courier_lock, K_FOREVER);
	courier_refresh();
	courier_poll_active();
	courier_track();
	if (courier.armed && courier.active_slot < 0 && courier_self_ready()) {
		courier_maybe_start();
	}
	k_mutex_unlock(&courier_lock);

	(void)k_work_reschedule_for_queue(&courier.wq, &courier.tick, COURIER_TICK);
}

void meshtastic_fleet_courier_arm(bool on)
{
	k_mutex_lock(&courier_lock, K_FOREVER);
	courier.armed = on;
	k_mutex_unlock(&courier_lock);
	LOG_INF("courier: %s", on ? "ARMED" : "disarmed");
	(void)k_work_reschedule_for_queue(&courier.wq, &courier.tick, K_NO_WAIT);
}

bool meshtastic_fleet_courier_armed(void)
{
	return courier.armed;
}

void meshtastic_fleet_courier_clear(uint32_t node_id)
{
	k_mutex_lock(&courier_lock, K_FOREVER);
	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		if (courier.nbr[i].node == node_id) {
			courier.nbr[i].state = CS_IDLE;
			courier.nbr[i].attempts = 0U;
			courier.nbr[i].next_try_ms = 0;
		}
	}
	k_mutex_unlock(&courier_lock);
}

uint16_t meshtastic_fleet_courier_rows(struct meshtastic_fleet_courier_row *out, uint16_t max)
{
	uint16_t n = 0U;

	k_mutex_lock(&courier_lock, K_FOREVER);
	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS && n < max; i++) {
		struct courier_nbr *c = &courier.nbr[i];
		struct meshtastic_fleet_intent want;

		if (!c->present && c->state == CS_IDLE) {
			continue;
		}
		out[n].node_id = c->node;
		out[n].class_id = c->class_id;
		out[n].running = c->running_known ? c->running : 0U;
		out[n].desired = meshtastic_fleet_desired_for(c->node, c->class_id, &want)
					 ? want.version : 0U;
		out[n].flags = c->flags;
		out[n].state = courier_state_str(c->state);
		out[n].attempts = c->attempts;
		n++;
	}
	k_mutex_unlock(&courier_lock);
	return n;
}

static int courier_init(void)
{
	k_work_queue_init(&courier.wq);
	k_work_queue_start(&courier.wq, courier_stack, K_THREAD_STACK_SIZEOF(courier_stack),
			   K_PRIO_PREEMPT(10), NULL);
	k_work_init_delayable(&courier.tick, courier_tick_fn);
	(void)k_work_reschedule_for_queue(&courier.wq, &courier.tick, COURIER_TICK);
	return 0;
}

SYS_INIT(courier_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* CONFIG_MESHTASTIC_FLEET_COURIER */
