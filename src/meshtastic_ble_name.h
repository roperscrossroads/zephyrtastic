/* SPDX-License-Identifier: GPL-3.0
 *
 * The Bluetooth advertising name, composed from the node's own identity.
 *
 * Deliberately BT-free — like meshtastic_ble_registry.h and
 * meshtastic_ble_peer_codec.h beside it — so the rule can be unit-tested with no
 * Bluetooth stack, no kernel and no config store. The caller supplies the short
 * name and node id; this decides only what the name should read.
 *
 * Why it exists at all: the advertised name used to be one compile-time
 * constant, so every node built from this tree advertised the SAME name and a
 * phone was offered a list of indistinguishable entries (agents-xhli.15).
 */
#ifndef MESHTASTIC_BLE_NAME_H_
#define MESHTASTIC_BLE_NAME_H_

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Compose this node's advertising name into @p out.
 *
 * The name is `<CONFIG_MESHTASTIC_BLE_NAME_PREFIX><short_name>` — e.g.
 * "zeph-rzr3". When @p short_name is NULL or empty the node id's low three
 * bytes are used instead (`zeph-04e14b`): an unnamed node is exactly the case
 * that used to collide, so the fallback must still be unique per board.
 *
 * Always NUL-terminates when @p out_len is non-zero, and truncates rather than
 * overruns.
 *
 * @return the length written, excluding the terminator.
 */
size_t meshtastic_ble_adv_name_compose(char *out, size_t out_len, const char *short_name,
				       uint32_t node_id);

#endif /* MESHTASTIC_BLE_NAME_H_ */
