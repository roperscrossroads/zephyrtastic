/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_LED_STATUS_H_
#define MESHTASTIC_LED_STATUS_H_

/*
 * Bench diagnostic: led0 driven straight from CONFIG_MESHTASTIC_BLE_PEER's
 * live link state, independent of the app/config-driven external
 * notification module (Kconfig.led_status explains the split and why the two
 * are mutually exclusive).
 *
 * Solid: powered on, no active peer link.
 * 1 Hz blink (on CONFIG_MESHTASTIC_LED_STATUS_TICK_MS): at least one peer
 * link — inbound or outbound — is up.
 */

/* Starts the periodic tick. Idempotent-safe to call once at boot, same as
 * every other module's _init(). Returns 0, or a negative errno if led0 is
 * declared but not ready. */
int meshtastic_led_status_init(void);

#endif /* MESHTASTIC_LED_STATUS_H_ */
