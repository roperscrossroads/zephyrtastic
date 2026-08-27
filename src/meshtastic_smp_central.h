/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_SMP_CENTRAL_H_
#define MESHTASTIC_SMP_CENTRAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>

/*
 * The SMP (mcumgr) client over BLE — this node manages a peer node's images.
 * Mechanism and shell (`smpc`) in meshtastic_smp_central.c. This header is the
 * programmatic surface: what a caller that is not the shell (a fleet
 * reconciler acting on the cluster document) needs in order to drive the same
 * ladder the shell does. Single link, single job, by design.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Connect to (or ADOPT — Zephyr keeps one bt_conn per address, so if the peer
 * already holds a link to us, this rides it) the peer's SMP GATT service and
 * subscribe. Blocks up to `timeout` for the link to become SMP-ready.
 * -EBUSY if a link is already held; -ETIMEDOUT / the connect error otherwise.
 */
int meshtastic_smpc_connect(const bt_addr_le_t *addr, k_timeout_t timeout);

/* Drop the link — or, for an adopted link, only our subscription on it. */
int meshtastic_smpc_disconnect(void);

/* True when SMP frames can flow (subscribed). */
bool meshtastic_smpc_ready(void);

/*
 * What a local image is, read from its MCUboot header: the byte count to send
 * (header + body + protected TLVs) and the semantic version. `path` names a
 * file under the depot (CONFIG_MESHTASTIC_DEPOT), or NULL/"" for this node's
 * own slot1.
 */
struct meshtastic_smpc_image {
	uint32_t size;
	uint8_t major, minor;
	uint16_t revision;
	uint32_t build;
};

int meshtastic_smpc_local_image(const char *path, struct meshtastic_smpc_image *out);

#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_SMP_CENTRAL_H_ */
