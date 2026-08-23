/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_REGISTRY_H_
#define MESHTASTIC_BLE_REGISTRY_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * BT-free BLE connection registry: one slot per host connection object, keyed
 * by bt_conn_index() (Zephyr guarantees the index is < CONFIG_BT_MAX_CONN for
 * the lifetime of the connection).
 *
 * Two things live here, both deliberately outside the Bluetooth stack so they
 * are unit-testable on native_sim (no BLE controller exists there):
 *
 * 1. THE PM LEDGER (agents-a4it.1). Each slot records whether the phone
 *    light-sleep inhibitor was charged for THAT connection (`phone_noted`).
 *    The flag is granted exactly once — when the slot is classified PHONE —
 *    and consumed exactly once — by disconnect — so the refcount is balanced
 *    by construction for any interleaving of connections. The disconnect path
 *    keys off the flag and never re-derives the kind.
 *
 * 2. CLASSIFICATION (agents-a4it.5). A connection this node created is a PEER
 *    immediately and unambiguously (we never act as central toward a phone).
 *    An incoming connection starts UNCLASSIFIED and is resolved by evidence —
 *    a peer beat (HELLO), PhoneAPI traffic — or by the bounded classify timer
 *    defaulting it to PHONE. Transitions only ever leave UNCLASSIFIED, and
 *    every transition is counted by reason: counters over assumptions.
 *
 * The caller serialises access (in meshtastic_ble.c every call sits under
 * ble.lock; meshtastic_ble_peer.c's calls run on the BT RX thread or the
 * system workqueue, never in ISR context).
 */

#if defined(CONFIG_BT_MAX_CONN)
#define MESHTASTIC_BLE_REG_SLOTS CONFIG_BT_MAX_CONN
#else
/* Unit-test builds have no BT host; any small capacity exercises the logic. */
#define MESHTASTIC_BLE_REG_SLOTS 4
#endif

enum meshtastic_ble_conn_kind {
	MESHTASTIC_BLE_CONN_NONE = 0, /* slot not in use */
	MESHTASTIC_BLE_CONN_UNCLASSIFIED,
	MESHTASTIC_BLE_CONN_PHONE,
	MESHTASTIC_BLE_CONN_PEER,
};

enum meshtastic_ble_classify_reason {
	/* PEER because we created the connection (role CENTRAL at connect). */
	MESHTASTIC_BLE_CLASSIFY_ROLE = 0,
	/* PEER because a valid peer beat arrived on the link. */
	MESHTASTIC_BLE_CLASSIFY_HELLO,
	/* PHONE because PhoneAPI traffic arrived on the link. */
	MESHTASTIC_BLE_CLASSIFY_TRAFFIC,
	/* PHONE because the classify window expired with no evidence. */
	MESHTASTIC_BLE_CLASSIFY_TIMER,
};

struct meshtastic_ble_reg_stats {
	/* THE ledger read-out: these two converging to equal on an idle bench
	 * is the direct, debugger-free proof of the a4it.1 fix. */
	uint32_t phone_notes;
	uint32_t phone_unnotes;
	/* Classification, by reason. */
	uint32_t classified_peer_by_role;
	uint32_t classified_peer_by_hello;
	uint32_t classified_phone_by_traffic;
	uint32_t classified_phone_default;
	/* Must stay 0: nonzero means the bt_conn_index() < CONFIG_BT_MAX_CONN
	 * invariant is broken and every per-slot structure is suspect. */
	uint32_t slot_index_out_of_range;
};

/* Drop every slot and zero the counters. Test support. */
void meshtastic_ble_reg_reset(void);

/*
 * Claim the slot for a new connection. `initial` is either
 * MESHTASTIC_BLE_CONN_PEER (we created the connection — counted as
 * classified_peer_by_role) or MESHTASTIC_BLE_CONN_UNCLASSIFIED (incoming).
 * PHONE cannot be an initial kind: the phone note is granted only through
 * meshtastic_ble_reg_classify(), keeping the ledger single-path.
 *
 * Returns 0, -EINVAL on a bad index (counted) or bad initial kind, or
 * -EALREADY if the slot is still claimed (a missed disconnect — refused so a
 * stale slot can never absorb or duplicate a live connection's note).
 */
int meshtastic_ble_reg_connect(unsigned int index, enum meshtastic_ble_conn_kind initial);

/*
 * Resolve an UNCLASSIFIED slot to PHONE or PEER. On the transition to PHONE
 * the slot's phone_noted flag is set and the CALLER MUST charge the phone PM
 * inhibitor exactly once (the two call sites in meshtastic_ble.c do so
 * immediately, under the same lock).
 *
 * Returns 0 exactly when the transition happened; -EALREADY if the slot is
 * already classified (evidence arriving late is normal — count it, don't act
 * on it); -EINVAL on a bad index (counted), a slot not in use, or an attempt
 * to classify to UNCLASSIFIED/NONE.
 */
int meshtastic_ble_reg_classify(unsigned int index, enum meshtastic_ble_conn_kind kind,
				enum meshtastic_ble_classify_reason reason);

/*
 * Release the slot. Returns true iff this connection's phone_noted flag was
 * set — i.e. the caller must now release the phone PM inhibitor exactly once.
 * The flag is consumed: a second call for the same index returns false, as
 * does releasing a never-claimed or out-of-range slot.
 */
bool meshtastic_ble_reg_disconnect(unsigned int index);

/* Current kind of the slot (NONE when free or out of range). */
enum meshtastic_ble_conn_kind meshtastic_ble_reg_kind(unsigned int index);

/* True iff the slot is claimed with phone_noted set. */
bool meshtastic_ble_reg_phone_noted(unsigned int index);

/* Number of currently claimed slots. */
unsigned int meshtastic_ble_reg_active(void);

/* Number of claimed slots still UNCLASSIFIED (the classify timer's workload). */
unsigned int meshtastic_ble_reg_unclassified(void);

/* Copy the counters. */
void meshtastic_ble_reg_stats(struct meshtastic_ble_reg_stats *out);

#endif /* MESHTASTIC_BLE_REGISTRY_H_ */
