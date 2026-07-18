/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_SETTINGS_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_SETTINGS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int meshtastic_settings_init(void);
void meshtastic_settings_schedule_save(void);
int meshtastic_settings_flush(void);

/**
 * @brief Delete the persisted Meshtastic config/module/channel/owner from NVS.
 *
 * Removes every key in the config subtree. When @p preserve_security is true the
 * config/security record (the once-only X25519 identity) is kept — the config-only
 * factory reset. The in-RAM store is left untouched, so the caller MUST reboot
 * without flushing (otherwise the old values are re-persisted). Cancels any pending
 * save. No-op benefit only with CONFIG_MESHTASTIC_SETTINGS.
 *
 * @return 0 on success, negative errno on the first delete/format failure.
 */
int meshtastic_settings_wipe(bool preserve_security);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_SETTINGS_H_ */
