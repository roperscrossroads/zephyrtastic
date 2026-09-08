/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_STATUSMESSAGE_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_STATUSMESSAGE_H_

/*
 * Node status message (agents-dnr4.26) -- the reference's StatusMessageModule.
 *
 * An away-message: ModuleConfig.statusmessage.node_status, broadcast on
 * NODE_STATUS_APP (port 36) as a StatusMessage protobuf so peers can show it,
 * and the last status heard from each peer kept in a small cache.
 *
 * This is the first of the ten modules the admin/config sprint builds from
 * zero, so it also sets the wiring pattern the rest reuse:
 *
 *   - the module READS ITS SECTION FROM THE CONFIG STORE, never a copy: what the
 *     admin channel persisted is what goes on air (C4 in module-parity.md is the
 *     failure this avoids -- "stored, persisted, echoed to the phone, and never
 *     read");
 *   - a section the reference applies without a reboot gets a
 *     *_config_changed() hook that the admin set path calls instead of
 *     scheduling one, and the module re-arms itself from it;
 *   - unprompted sends ride a k_work_delayable on the system workqueue with
 *     K_NO_WAIT (meshtastic_health.c's pattern), not a dedicated thread: a
 *     beacon that fires twice a day does not earn a 3.5 KB stack;
 *   - a Kconfig bool for the module and one for AUTO_SEND, each with a scenario
 *     in tests/variants, plus a suite of its own driven through the sim radio.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int meshtastic_statusmessage_init(void);

/**
 * @brief Copy this node's current status (from ModuleConfig.statusmessage).
 *
 * @return Length of the status; 0 when none is set (@p buf is then "").
 */
size_t meshtastic_statusmessage_get(char *buf, size_t cap);

/**
 * @brief Persist a new status and re-arm the announce. "" (or NULL) clears it,
 *        which also cancels any pending announce. Truncated to the protobuf
 *        field width. Goes through the config store, so it is what an admin
 *        get_module_config reads back.
 */
int meshtastic_statusmessage_set(const char *status);

/**
 * @brief ModuleConfig.statusmessage was written by someone else (the admin
 *        path). Re-reads the store and re-arms: a status set -> announce
 *        START_DELAY_SEC from now, then every INTERVAL_SEC; cleared -> silence.
 */
void meshtastic_statusmessage_config_changed(void);

/**
 * @brief Broadcast the status now.
 *
 * @return 0 queued; -ENODATA when no status is set; other errno from the send.
 */
int meshtastic_statusmessage_send(void);

/** @brief Most recent status heard from @p node. false when none cached. */
bool meshtastic_statusmessage_peer_get(uint32_t node, char *buf, size_t cap);

/**
 * @brief Walk the peer cache (shell). Slots are returned in storage order, not
 *        recency; an empty slot yields false. @p age_ms may be NULL.
 */
bool meshtastic_statusmessage_peer_at(size_t index, uint32_t *node, char *buf, size_t cap,
				      int64_t *age_ms);

/** @brief Forget every cached peer status (tests). */
void meshtastic_statusmessage_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_STATUSMESSAGE_H_ */
