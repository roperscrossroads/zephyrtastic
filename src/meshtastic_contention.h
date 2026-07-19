/* SPDX-License-Identifier: GPL-3.0
 *
 * LoRa contention window — the timing maths behind "wait a random moment before
 * transmitting, so two nodes that heard the same packet do not answer together".
 *
 * Ports the reference firmware's RadioInterface delay computation
 * (computeSlotTimeMsec, getCWsize, getTxDelayMsec, getTxDelayMsecWeighted). Pure
 * functions only: nothing here touches the radio, the queue or the clock, so it
 * can be asserted directly against values taken from the reference.
 *
 * Why this matters for this port specifically: we already do listen-before-talk
 * (LORA_CAD_MODE_LBT) and back off 8-40 ms when CAD reports the channel busy,
 * but on a *clear* channel we transmit immediately. The reference waits even
 * then. The observable consequence on a live mesh is that our relays consistently
 * pre-empt well-behaved peers rather than colliding with them — measured at
 * 12/12 redundant relays arriving >= 1 s after ours.
 */

#ifndef MESHTASTIC_CONTENTION_H_
#define MESHTASTIC_CONTENTION_H_

#include <stdbool.h>
#include <stdint.h>

/* Contention-window bounds, as exponents: the pool of slots is 1 << CWsize.
 * Reference RadioInterface.h (CWmin/CWmax). */
#define MESHTASTIC_CW_MIN 3U
#define MESHTASTIC_CW_MAX 8U

/**
 * @brief Slot time in ms for a modem configuration.
 *
 * Reference computeSlotTimeMsec(): CAD duration + propagation + transceiver
 * turnaround + MAC processing, where the fixed part is 0.2 + 0.4 + 7 = 7.6 ms
 * and the symbol time is 2^sf / bandwidth(kHz).
 *
 * LongFast (sf 11, 250 kHz) works out at 28 ms.
 *
 * @param spread_factor  5..12
 * @param bandwidth_hz   modem bandwidth
 * @param wide_lora      true for the 2.4 GHz CAD figure (4 symbols + a
 *                       spreading-factor term) instead of the sub-GHz one
 * @return slot time in milliseconds (0 if the inputs are unusable)
 */
uint32_t meshtastic_contention_slot_ms(uint8_t spread_factor, uint32_t bandwidth_hz,
				       bool wide_lora);

/**
 * @brief Contention-window exponent for a received SNR.
 *
 * Reference getCWsize(): map(snr, -20, 10, CWmin, CWmax).
 *
 * Note the direction, which is deliberate and easy to get backwards: a *high*
 * SNR yields a *large* window and therefore a *longer* wait. The node that heard
 * the packet loudest is the closest one, and so the one whose relay adds least
 * coverage; making it defer lets the weak-signal node — the one that reaches
 * furthest — relay first.
 *
 * @param snr  received SNR in dB
 * @return exponent clamped to [MESHTASTIC_CW_MIN, MESHTASTIC_CW_MAX]
 */
uint8_t meshtastic_contention_cw_from_snr(int8_t snr);

/**
 * @brief Contention-window exponent for the current channel utilisation.
 *
 * Reference getTxDelayMsec(): map(channelUtil, 0, 100, CWmin, CWmax) — a busier
 * channel widens the window.
 *
 * @param util_pct  channel utilisation, 0..100
 * @return exponent clamped to [MESHTASTIC_CW_MIN, MESHTASTIC_CW_MAX]
 */
uint8_t meshtastic_contention_cw_from_util(uint8_t util_pct);

/**
 * @brief Delay before transmitting a packet we originated.
 *
 * Reference getTxDelayMsec(): random(0, 1 << CWsize) * slot, sized by channel
 * utilisation. Draws from the CSPRNG.
 */
uint32_t meshtastic_contention_delay_own_ms(uint8_t util_pct, uint32_t slot_ms);

/**
 * @brief Delay before relaying a packet we received.
 *
 * Reference getTxDelayMsecWeighted(). Routers go early — random(0, 2*CWsize)
 * slots — while everyone else waits out a fixed 2*CWmax offset first, so a
 * router's relay reliably beats a client's. Draws from the CSPRNG.
 *
 * @param snr                SNR the packet was received at
 * @param early_like_router  true if this node should relay early
 * @param slot_ms            slot time from meshtastic_contention_slot_ms()
 */
uint32_t meshtastic_contention_delay_relay_ms(int8_t snr, bool early_like_router,
					      uint32_t slot_ms);

/**
 * @brief Worst-case relay delay for an SNR — the upper bound of the window.
 *
 * Reference getTxDelayMsecWeightedWorst(), used to clamp accumulated delay.
 */
uint32_t meshtastic_contention_delay_relay_worst_ms(int8_t snr, uint32_t slot_ms);

#endif /* MESHTASTIC_CONTENTION_H_ */
