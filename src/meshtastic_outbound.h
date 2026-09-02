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
 * @brief "Not now" — meshtastic_radio_send_wire_now()'s answer when the air is in use.
 *
 * Returned when a packet is being demodulated (transmitting would destroy it,
 * and nobody would decode ours either) or when CAD heard one on the channel.
 * In both cases the radio has been left LISTENING, so the packet in question is
 * being received, not lost. The outbound drain re-queues the frame behind a
 * fresh contention delay (parity: RadioLibInterface re-rolls its transmit
 * delay and keeps the packet at the head of its queue). Private to the drain
 * and the radio layer; never surfaces to a caller.
 */
#define MESHTASTIC_TX_DEFER (-EINPROGRESS)

/**
 * @brief Push mt.frequency + mt.modem to the radio and re-arm RX.
 *
 * The live-reconfigure step of a preset switch (meshtastic_preset.c). Tears down
 * continuous async RX first — the SX126x driver rejects lora_config() while it is
 * running — and re-arms afterwards even if the config failed, since a deaf radio
 * is worse than a misconfigured one.
 *
 * @return 0 on success, negative errno from lora_config().
 */
int meshtastic_radio_retune(void);

/**
 * @brief Tune to exact parameters, deriving nothing.
 *
 * The scanner's entry point, and deliberately NOT meshtastic_preset_switch():
 * that resolves the frequency through the primary channel's name, which is right
 * for operating and wrong for scanning. A scanner has to visit the frequency a
 * *foreign* mesh is on, and a named local channel would keep dragging it back to
 * that name's slot (docs/MULTI-PRESET-OPERATION.md §3.1a).
 *
 * Sets no preset, touches no channel, re-derives no hash — it only moves the
 * radio. That also means it leaves mt.modem_preset stale, so a caller that
 * intends to resume normal operation must go back through
 * meshtastic_preset_switch() rather than tuning "back" by hand.
 *
 * @return 0 on success, -EINVAL for a bandwidth or coding rate the driver cannot
 *         represent, or a negative errno from the radio.
 */
int meshtastic_radio_tune_explicit(uint32_t frequency_hz, uint8_t spread_factor,
				   uint32_t bandwidth_hz, uint8_t coding_rate);

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
/**
 * @brief Frames queued for transmission, plus one for a frame already handed to
 *        the radio and not yet finished.
 *
 * The second half is the point. The worker dequeues before it transmits, so a
 * bare queue count reads zero while a frame is being keyed — and a caller using
 * this to decide "nothing of mine is still going out" (the preset-hop drain
 * interlock, docs/MULTI-PRESET-OPERATION.md §4.4) would retune underneath it.
 *
 * Zero therefore means the radio owes this node nothing.
 */
uint8_t meshtastic_outbound_pending(void);

int meshtastic_outbound_defer_late(uint32_t src, uint32_t id, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_ */
