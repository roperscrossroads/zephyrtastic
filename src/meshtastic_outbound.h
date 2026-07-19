/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

int meshtastic_outbound_init(void);

/*
 * Queue a wire frame for transmission.  Returns 0 when queued, -ENOMSG if the
 * queue is full.  Does not block on the radio driver.  The NORMAL-tier default;
 * use the _prio variant to classify.
 */
int meshtastic_radio_send_wire(uint8_t *pkt, uint32_t pkt_len);

/*
 * Queue a wire frame and block until the outbound worker completes the
 * transmission or @p timeout expires (-EAGAIN).  NORMAL-tier default.
 */
int meshtastic_radio_send_wire_wait(const uint8_t *pkt, uint32_t pkt_len, k_timeout_t timeout);

/*
 * Priority-aware variants.  @p tier is a meshtastic_sched_tier value governing
 * ordering and overflow protection.  Fire-and-forget (_prio) frames may be
 * dropped per the active scheduler overflow policy; the blocking (_wait_prio)
 * variant is never silently dropped — it waits for space up to @p timeout.
 */
int meshtastic_radio_send_wire_prio(uint8_t *pkt, uint32_t pkt_len, uint8_t tier);
int meshtastic_radio_send_wire_wait_prio(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
					 k_timeout_t timeout);

/* Driver-level TX; only called from the outbound worker thread. */
int meshtastic_radio_send_wire_now(uint8_t *pkt, uint32_t pkt_len);

/**
 * @brief Queue a frame that must not go out for at least @p delay_ms.
 *
 * The contention window (see meshtastic_contention.h). The frame occupies a
 * queue slot immediately — so it is subject to the usual depth and overflow
 * policy — but the worker will not transmit it until the delay elapses, and
 * higher-tier traffic queued behind it still goes first. @p delay_ms of 0 is
 * identical to meshtastic_radio_send_wire_prio().
 *
 * Deferring is what makes a relay cancellable: until the deadline passes there
 * is a queued frame to remove.
 */
int meshtastic_radio_send_wire_after(uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
				     uint32_t delay_ms);

/**
 * @brief Drop any queued frame matching @p src / @p id that has not gone out.
 *
 * Overhear-cancel: hearing a peer relay a packet we still have queued means our
 * copy would be pure duplicate airtime, so it is removed before transmitting.
 * Only reachable because relays are deferred — with no contention window there
 * is no interval in which a queued relay exists to cancel.
 *
 * Frames a caller is blocked on are never cancelled.
 *
 * @return number of frames removed (0 if ours already went out, which is the
 *         common case when the window is short or the peer was slow).
 */
int meshtastic_outbound_cancel(uint32_t src, uint32_t id);

/**
 * @brief Push a queued frame into the late-rebroadcast window.
 *
 * Mirrors the reference clampToLateRebroadcastWindow(). Used by roles that
 * should relay even after hearing a peer cover the frame, but should go last:
 * ROUTER_LATE always, and CLIENT_BASE for traffic touching a favourite. The
 * relay still happens — it just stops competing with everyone else's.
 *
 * The new deadline is measured from now rather than from the original one, and
 * a frame already in the late window is left alone: re-clamping on every
 * further copy heard would keep pushing the deadline out and the frame would
 * never transmit at all.
 *
 * @return number of frames moved (0 if ours already went out, or was already late).
 */
int meshtastic_outbound_defer_late(uint32_t src, uint32_t id, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_ */
