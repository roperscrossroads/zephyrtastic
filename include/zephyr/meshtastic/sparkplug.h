/* SPDX-License-Identifier: GPL-3.0
 *
 * The gateway's Sparkplug B Edge Node, seen from the rest of the firmware.
 */
#ifndef ZEPHYR_MESHTASTIC_SPARKPLUG_H_
#define ZEPHYR_MESHTASTIC_SPARKPLUG_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

/**
 * End the Sparkplug session on purpose, before a planned reboot: publish the session's NDEATH,
 * then send MQTT DISCONNECT (spec §5.5 / §7.3: an Edge Node that disconnects intentionally
 * publishes NDEATH first, and SHOULD send DISCONNECT after it). Without this the broker only
 * learns of the reboot when the keepalive runs out, and publishes the Will (the same NDEATH) up
 * to 1.5 keepalives later.
 *
 * Waits up to @p timeout for the Sparkplug thread to do it; the session stays stopped (no
 * reconnect) until meshtastic_sparkplug_start(). Safe to call when not connected.
 *
 * @return 0 once stopped, -EAGAIN on timeout, -EDEADLK from the Sparkplug thread itself.
 *
 * Both are no-ops when CONFIG_MESHTASTIC_SPARKPLUG is off, so reboot paths need no guard.
 */
/** The Edge Node's uplink as `meshtastic state` reports it. The broker's host name only: never
 * the port, the login, the group or the edge id. */
struct meshtastic_sparkplug_uplink {
	bool on;            /**< not stopped (`sparkplug stop`, a planned reboot) */
	bool connected;     /**< a live session with the broker */
	bool tls;           /**< every connection is TLS, the broker verified */
	uint32_t errors;    /**< failed connects (refused or not answered), publishes and encodes */
	char host[64];
};

#if defined(CONFIG_MESHTASTIC_SPARKPLUG)
int meshtastic_sparkplug_stop(k_timeout_t timeout);
void meshtastic_sparkplug_start(void);
void meshtastic_sparkplug_uplink_get(struct meshtastic_sparkplug_uplink *out);
#else
static inline int meshtastic_sparkplug_stop(k_timeout_t timeout)
{
	(void)timeout;
	return 0;
}
static inline void meshtastic_sparkplug_start(void)
{
}
#endif

/** How long a planned reboot waits for the NDEATH and DISCONNECT to leave. */
#define MESHTASTIC_SPARKPLUG_STOP_TIMEOUT K_MSEC(2000)

#endif /* ZEPHYR_MESHTASTIC_SPARKPLUG_H_ */
