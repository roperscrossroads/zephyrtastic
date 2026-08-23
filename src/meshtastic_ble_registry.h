/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_BLE_REGISTRY_H_
#define MESHTASTIC_BLE_REGISTRY_H_

#include <stdbool.h>

/*
 * BT-free BLE connection registry.
 *
 * One slot per host connection object, keyed by bt_conn_index() (Zephyr
 * guarantees the index is < CONFIG_BT_MAX_CONN for the lifetime of the
 * connection). Each slot carries a `phone_noted` flag recording whether the
 * connect path charged the phone light-sleep inhibitor for THAT connection;
 * the disconnect path releases the inhibitor iff the flag was set, and the
 * flag is cleared in the same operation. The flag can therefore be taken at
 * most once and returned at most once per connection, which makes the PM
 * refcount balanced by construction for any interleaving of connections —
 * the property that broke when disconnected() charged/released
 * unconditionally (agents-a4it.1).
 *
 * The registry deliberately has no Bluetooth or kernel dependency so the
 * ledger property is unit-testable on native_sim, where no BLE controller
 * exists. The caller serialises access (in meshtastic_ble.c every call sits
 * under ble.lock).
 */

#if defined(CONFIG_BT_MAX_CONN)
#define MESHTASTIC_BLE_REG_SLOTS CONFIG_BT_MAX_CONN
#else
/* Unit-test builds have no BT host; any small capacity exercises the logic. */
#define MESHTASTIC_BLE_REG_SLOTS 4
#endif

/* Drop every slot. Test support and (defensively) bt_disable paths. */
void meshtastic_ble_reg_reset(void);

/*
 * Claim the slot for a new connection. `phone_noted` records whether the
 * caller charged the phone PM inhibitor for this connection.
 *
 * Returns 0, -EINVAL if the index is out of range, or -EALREADY if the slot
 * is still claimed (a missed disconnect — refused so a stale slot can never
 * absorb or duplicate a live connection's note).
 */
int meshtastic_ble_reg_connect(unsigned int index, bool phone_noted);

/*
 * Release the slot. Returns true iff this connection's `phone_noted` flag was
 * set — i.e. the caller must now release the phone PM inhibitor exactly once.
 * The flag is consumed: a second call for the same index returns false, as
 * does releasing a never-claimed or out-of-range slot.
 */
bool meshtastic_ble_reg_disconnect(unsigned int index);

/* True iff the slot is claimed with `phone_noted` set. */
bool meshtastic_ble_reg_phone_noted(unsigned int index);

/* Number of currently claimed slots. */
unsigned int meshtastic_ble_reg_active(void);

#endif /* MESHTASTIC_BLE_REGISTRY_H_ */
