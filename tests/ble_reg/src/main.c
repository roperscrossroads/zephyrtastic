/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the BLE connection registry (src/meshtastic_ble_registry.c).
 *
 * The registry exists because of agents-a4it.1: disconnected() used to release
 * the phone light-sleep inhibitor unconditionally for EVERY connection, while
 * connected() only charged it for the first. Balanced at CONFIG_BT_MAX_CONN=1,
 * broken at 2 — a peer disconnect would half-release the phone's inhibit while
 * the phone was still connected. The fix records "this connection charged the
 * phone note" per slot at connect time, and the disconnect path consumes that
 * record instead of re-deriving anything.
 *
 * The property under test is the ledger: for ANY interleaving of connects and
 * disconnects, the number of times connect records a phone note equals the
 * number of times disconnect returns one, once all connections are down. The
 * directed tests pin the exact interleaving from the bug report; the final
 * test drives a deterministic pseudo-random interleaving against a shadow
 * model and checks the ledger closes.
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

ZTEST(ble_reg, test_phone_note_taken_and_returned_once)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, true));
	zassert_true(meshtastic_ble_reg_phone_noted(0U));
	zassert_equal(meshtastic_ble_reg_active(), 1U);

	zassert_true(meshtastic_ble_reg_disconnect(0U), "the phone's own disconnect returns its note");
	zassert_false(meshtastic_ble_reg_disconnect(0U), "a second disconnect must not return a note again");
	zassert_false(meshtastic_ble_reg_phone_noted(0U));
	zassert_equal(meshtastic_ble_reg_active(), 0U);
}

/* The exact interleaving from the bug report: phone connects, a peer connects
 * and disconnects — the phone's inhibit must survive untouched. */
ZTEST(ble_reg, test_peer_disconnect_does_not_release_phone_note)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, true));  /* phone */
	zassert_ok(meshtastic_ble_reg_connect(1U, false)); /* peer */
	zassert_equal(meshtastic_ble_reg_active(), 2U);

	zassert_false(meshtastic_ble_reg_disconnect(1U),
		      "a peer disconnect must not release the phone's note");
	zassert_true(meshtastic_ble_reg_phone_noted(0U), "phone note survives the peer's departure");
	zassert_false(meshtastic_ble_reg_disconnect(1U),
		      "a spurious repeat disconnect must not release it either");

	zassert_true(meshtastic_ble_reg_disconnect(0U), "only the phone's disconnect returns the note");
	zassert_equal(meshtastic_ble_reg_active(), 0U);
}

ZTEST(ble_reg, test_nonphone_slot_never_returns_a_note)
{
	zassert_ok(meshtastic_ble_reg_connect(2U, false));
	zassert_false(meshtastic_ble_reg_phone_noted(2U));
	zassert_false(meshtastic_ble_reg_disconnect(2U));
}

ZTEST(ble_reg, test_out_of_range_is_rejected)
{
	zassert_equal(meshtastic_ble_reg_connect(MESHTASTIC_BLE_REG_SLOTS, true), -EINVAL);
	zassert_false(meshtastic_ble_reg_disconnect(MESHTASTIC_BLE_REG_SLOTS));
	zassert_false(meshtastic_ble_reg_phone_noted(MESHTASTIC_BLE_REG_SLOTS));
	zassert_equal(meshtastic_ble_reg_active(), 0U);
}

/* bt_conn_index values are recycled once a connection is freed; a slot must
 * carry only the CURRENT connection's flag, never a previous one's. */
ZTEST(ble_reg, test_slot_reuse_carries_only_current_flag)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, true));
	zassert_true(meshtastic_ble_reg_disconnect(0U));

	zassert_ok(meshtastic_ble_reg_connect(0U, false));
	zassert_false(meshtastic_ble_reg_disconnect(0U));

	zassert_ok(meshtastic_ble_reg_connect(0U, true));
	zassert_true(meshtastic_ble_reg_disconnect(0U));
}

/* A connect on a still-claimed slot (a missed disconnect — cannot happen with
 * a live host, but the ledger must not depend on that) is refused without
 * disturbing the resident note. */
ZTEST(ble_reg, test_stale_slot_refused_without_clobbering_note)
{
	zassert_ok(meshtastic_ble_reg_connect(0U, true));
	zassert_equal(meshtastic_ble_reg_connect(0U, false), -EALREADY);
	zassert_true(meshtastic_ble_reg_phone_noted(0U), "refused connect must not clear the note");
	zassert_equal(meshtastic_ble_reg_active(), 1U);
	zassert_true(meshtastic_ble_reg_disconnect(0U));
	zassert_false(meshtastic_ble_reg_disconnect(0U));
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
	bool model_in_use[MESHTASTIC_BLE_REG_SLOTS] = {false};
	bool model_noted[MESHTASTIC_BLE_REG_SLOTS] = {false};
	unsigned int notes = 0U;
	unsigned int unnotes = 0U;

	for (int i = 0; i < 10000; i++) {
		uint32_t r = lcg_next();
		unsigned int slot = r % MESHTASTIC_BLE_REG_SLOTS;
		bool do_connect = (r & 0x100U) != 0U;
		bool as_phone = (r & 0x200U) != 0U;

		if (do_connect) {
			int ret = meshtastic_ble_reg_connect(slot, as_phone);

			if (model_in_use[slot]) {
				zassert_equal(ret, -EALREADY);
			} else {
				zassert_ok(ret);
				model_in_use[slot] = true;
				model_noted[slot] = as_phone;
				if (as_phone) {
					notes++;
				}
			}
		} else {
			bool ret = meshtastic_ble_reg_disconnect(slot);

			zassert_equal(ret, model_in_use[slot] && model_noted[slot],
				      "disconnect must return exactly the recorded note");
			if (ret) {
				unnotes++;
			}
			model_in_use[slot] = false;
			model_noted[slot] = false;
		}

		zassert_equal(meshtastic_ble_reg_phone_noted(slot),
			      model_in_use[slot] && model_noted[slot]);
	}

	/* Drain whatever is still connected, then the ledger must close. */
	for (unsigned int s = 0U; s < MESHTASTIC_BLE_REG_SLOTS; s++) {
		if (meshtastic_ble_reg_disconnect(s)) {
			unnotes++;
		}
	}
	zassert_equal(meshtastic_ble_reg_active(), 0U);
	zassert_equal(notes, unnotes,
		      "every phone note taken must be returned exactly once (got %u vs %u)",
		      notes, unnotes);
}
