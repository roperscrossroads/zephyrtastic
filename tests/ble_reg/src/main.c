/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the BLE connection registry (src/meshtastic_ble_registry.c).
 *
 * Two properties under test:
 *
 * 1. THE PM LEDGER (agents-a4it.1): disconnected() used to release the phone
 *    light-sleep inhibitor unconditionally for EVERY connection, while the
 *    connect path charged it only for the first — balanced at
 *    CONFIG_BT_MAX_CONN=1, broken at 2. The registry grants the phone note
 *    exactly once (on classification to PHONE) and returns it exactly once
 *    (on disconnect), so for ANY interleaving phone_notes == phone_unnotes
 *    once every connection is down. The final test drives a deterministic
 *    pseudo-random interleaving against a shadow model.
 *
 * 2. CLASSIFICATION (agents-a4it.5): outbound connections are peers at
 *    connect; incoming ones start UNCLASSIFIED and transition exactly once,
 *    with every transition counted by reason.
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "meshtastic_ble_registry.h"

static void reg_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_ble_reg_reset();
}

ZTEST_SUITE(ble_reg, NULL, NULL, reg_before, NULL, NULL);

#define KIND_UNCLASSIFIED MESHTASTIC_BLE_CONN_UNCLASSIFIED
#define KIND_PHONE        MESHTASTIC_BLE_CONN_PHONE
#define KIND_PEER         MESHTASTIC_BLE_CONN_PEER

static struct meshtastic_ble_reg_stats stats_now(void)
{
	struct meshtastic_ble_reg_stats s;

	meshtastic_ble_reg_stats(&s);
	return s;
}

ZTEST(ble_reg, test_phone_note_taken_and_returned_once)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_equal(meshtastic_ble_reg_kind(0U), KIND_UNCLASSIFIED);
	zassert_false(meshtastic_ble_reg_phone_noted(0U), "no note before classification");

	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TRAFFIC));
	zassert_true(meshtastic_ble_reg_phone_noted(0U));
	zassert_equal(stats_now().phone_notes, 1U);

	zassert_true(meshtastic_ble_reg_disconnect(0U), "the phone's disconnect returns its note");
	zassert_false(meshtastic_ble_reg_disconnect(0U), "a second disconnect must not repeat it");
	zassert_equal(stats_now().phone_unnotes, 1U);
	zassert_equal(meshtastic_ble_reg_active(), 0U);
}

/* The exact interleaving from the a4it.1 bug report: phone up, a peer comes
 * and goes — the phone's inhibit must survive untouched. */
ZTEST(ble_reg, test_peer_disconnect_does_not_release_phone_note)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TRAFFIC));
	zassert_ok(meshtastic_ble_reg_connect(1U, KIND_PEER)); /* outbound peer */
	zassert_equal(meshtastic_ble_reg_active(), 2U);

	zassert_false(meshtastic_ble_reg_disconnect(1U),
		      "a peer disconnect must not release the phone's note");
	zassert_true(meshtastic_ble_reg_phone_noted(0U));
	zassert_false(meshtastic_ble_reg_disconnect(1U),
		      "a spurious repeat disconnect must not release it either");

	zassert_true(meshtastic_ble_reg_disconnect(0U));
	zassert_equal(stats_now().phone_notes, stats_now().phone_unnotes);
}

ZTEST(ble_reg, test_outbound_is_peer_immediately)
{
	zassert_ok(meshtastic_ble_reg_connect(2U, KIND_PEER));
	zassert_equal(meshtastic_ble_reg_kind(2U), KIND_PEER);
	zassert_equal(stats_now().classified_peer_by_role, 1U);
	zassert_false(meshtastic_ble_reg_disconnect(2U), "a peer never returns a phone note");
}

ZTEST(ble_reg, test_classification_transitions_exactly_once)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));

	/* HELLO evidence wins the race... */
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PEER, MESHTASTIC_BLE_CLASSIFY_HELLO));
	zassert_equal(meshtastic_ble_reg_kind(0U), KIND_PEER);
	zassert_equal(stats_now().classified_peer_by_hello, 1U);

	/* ...and the late timer default must bounce off the state. */
	zassert_equal(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TIMER),
		      -EALREADY);
	zassert_equal(meshtastic_ble_reg_kind(0U), KIND_PEER, "late evidence must not reclassify");
	zassert_false(meshtastic_ble_reg_phone_noted(0U));
	zassert_equal(stats_now().phone_notes, 0U);

	zassert_false(meshtastic_ble_reg_disconnect(0U));
}

ZTEST(ble_reg, test_timer_default_counts_separately_from_traffic)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TIMER));
	zassert_equal(stats_now().classified_phone_default, 1U);
	zassert_equal(stats_now().classified_phone_by_traffic, 0U);
	zassert_true(meshtastic_ble_reg_disconnect(0U));
}

ZTEST(ble_reg, test_invalid_inputs_are_rejected_and_counted)
{
	/* Out-of-range indexes are counted: nonzero means the bt_conn_index()
	 * invariant is broken. */
	zassert_equal(meshtastic_ble_reg_connect(MESHTASTIC_BLE_REG_SLOTS, KIND_UNCLASSIFIED),
		      -EINVAL);
	zassert_false(meshtastic_ble_reg_disconnect(MESHTASTIC_BLE_REG_SLOTS));
	zassert_equal(meshtastic_ble_reg_classify(MESHTASTIC_BLE_REG_SLOTS, KIND_PHONE,
						  MESHTASTIC_BLE_CLASSIFY_TIMER),
		      -EINVAL);
	zassert_equal(stats_now().slot_index_out_of_range, 3U);

	/* PHONE is never an initial kind: the note is granted only through
	 * classify, keeping the ledger single-path. */
	zassert_equal(meshtastic_ble_reg_connect(0U, KIND_PHONE), -EINVAL);

	/* Classifying a free slot, or to a non-terminal kind, is an error. */
	zassert_equal(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TIMER),
		      -EINVAL);
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_equal(meshtastic_ble_reg_classify(0U, KIND_UNCLASSIFIED,
						  MESHTASTIC_BLE_CLASSIFY_TIMER),
		      -EINVAL);
}

/* A connect on a still-claimed slot (a missed disconnect — cannot happen with
 * a live host, but the ledger must not depend on that) is refused without
 * disturbing the resident note. */
ZTEST(ble_reg, test_stale_slot_refused_without_clobbering_note)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TRAFFIC));
	zassert_equal(meshtastic_ble_reg_connect(0U, KIND_PEER), -EALREADY);
	zassert_true(meshtastic_ble_reg_phone_noted(0U), "refused connect must not clear the note");
	zassert_true(meshtastic_ble_reg_disconnect(0U));
	zassert_false(meshtastic_ble_reg_disconnect(0U));
}

/* bt_conn_index values recycle once a connection is freed; a slot must carry
 * only the CURRENT connection's classification. */
ZTEST(ble_reg, test_slot_reuse_carries_only_current_state)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TRAFFIC));
	zassert_true(meshtastic_ble_reg_disconnect(0U));

	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_PEER));
	zassert_equal(meshtastic_ble_reg_kind(0U), KIND_PEER);
	zassert_false(meshtastic_ble_reg_disconnect(0U));

	zassert_ok(meshtastic_ble_reg_connect(0U, KIND_UNCLASSIFIED));
	zassert_equal(meshtastic_ble_reg_unclassified(), 1U);
	zassert_ok(meshtastic_ble_reg_classify(0U, KIND_PHONE, MESHTASTIC_BLE_CLASSIFY_TIMER));
	zassert_true(meshtastic_ble_reg_disconnect(0U));
}

/* Deterministic LCG (no libc rand: reproducibility across platforms). */
static uint32_t lcg_state = 0x12345678U;
static uint32_t lcg_next(void)
{
	lcg_state = lcg_state * 1664525U + 1013904223U;
	return lcg_state;
}

ZTEST(ble_reg, test_ledger_closes_under_random_interleaving)
{
	enum meshtastic_ble_conn_kind model[MESHTASTIC_BLE_REG_SLOTS] = {0};
	bool model_noted[MESHTASTIC_BLE_REG_SLOTS] = {false};
	uint32_t notes = 0U;
	uint32_t unnotes = 0U;
	struct meshtastic_ble_reg_stats s;

	for (int i = 0; i < 20000; i++) {
		uint32_t r = lcg_next();
		unsigned int slot = r % MESHTASTIC_BLE_REG_SLOTS;
		uint32_t op = (r >> 8) % 4U;

		switch (op) {
		case 0U: { /* connect, random initial kind */
			bool outbound = (r & 0x10000U) != 0U;
			enum meshtastic_ble_conn_kind initial =
				outbound ? KIND_PEER : KIND_UNCLASSIFIED;
			int ret = meshtastic_ble_reg_connect(slot, initial);

			if (model[slot] != MESHTASTIC_BLE_CONN_NONE) {
				zassert_equal(ret, -EALREADY);
			} else {
				zassert_ok(ret);
				model[slot] = initial;
				model_noted[slot] = false;
			}
			break;
		}
		case 1U: { /* classify, random target kind + reason */
			bool to_phone = (r & 0x10000U) != 0U;
			enum meshtastic_ble_conn_kind kind = to_phone ? KIND_PHONE : KIND_PEER;
			enum meshtastic_ble_classify_reason reason =
				to_phone ? ((r & 0x20000U) ? MESHTASTIC_BLE_CLASSIFY_TIMER
							   : MESHTASTIC_BLE_CLASSIFY_TRAFFIC)
					 : MESHTASTIC_BLE_CLASSIFY_HELLO;
			int ret = meshtastic_ble_reg_classify(slot, kind, reason);

			if (model[slot] == KIND_UNCLASSIFIED) {
				zassert_ok(ret);
				model[slot] = kind;
				if (to_phone) {
					model_noted[slot] = true;
					notes++;
				}
			} else if (model[slot] == MESHTASTIC_BLE_CONN_NONE) {
				zassert_equal(ret, -EINVAL);
			} else {
				zassert_equal(ret, -EALREADY);
			}
			break;
		}
		default: { /* disconnect (weighted 2x so slots drain) */
			bool ret = meshtastic_ble_reg_disconnect(slot);

			zassert_equal(ret, model_noted[slot],
				      "disconnect must return exactly the recorded note");
			if (ret) {
				unnotes++;
			}
			model[slot] = MESHTASTIC_BLE_CONN_NONE;
			model_noted[slot] = false;
			break;
		}
		}

		zassert_equal(meshtastic_ble_reg_kind(slot), model[slot]);
		zassert_equal(meshtastic_ble_reg_phone_noted(slot),
			      model[slot] != MESHTASTIC_BLE_CONN_NONE && model_noted[slot]);
	}

	/* Drain whatever is still connected, then the ledger must close. */
	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		if (meshtastic_ble_reg_disconnect(i)) {
			unnotes++;
		}
	}
	zassert_equal(meshtastic_ble_reg_active(), 0U);
	zassert_equal(notes, unnotes,
		      "every phone note taken must be returned exactly once (got %u vs %u)",
		      notes, unnotes);

	meshtastic_ble_reg_stats(&s);
	zassert_equal(s.phone_notes, notes);
	zassert_equal(s.phone_unnotes, unnotes);
	zassert_equal(s.slot_index_out_of_range, 0U, "all indexes in range, counter must be 0");
}
