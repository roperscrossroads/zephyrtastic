/* SPDX-License-Identifier: GPL-3.0
 *
 * LoRa contention window timing. See meshtastic_contention.h.
 */

#include "meshtastic_contention.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <psa/crypto.h>

#include "meshtastic_sched.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Symbols of CAD before transmitting. Reference RadioInterface.h: 2 for sub-GHz
 * (RadioLib default per AN1200.48), 4 at 2.4 GHz (AN1200.22 of the SX1280). */
#define NUM_SYM_CAD       2U
#define NUM_SYM_CAD_24GHZ 4U

/* Propagation (~30 km) + transceiver turnaround + MAC processing, in HUNDREDTHS
 * of a millisecond so the slot computation stays integral: 0.2 + 0.4 + 7 = 7.6.
 *
 * Hundredths, not tenths, because the symbol time is where the precision is
 * needed: LongFast's 2^11/250 = 8.192 ms truncates to 8.1 at tenths, which drags
 * the computed slot from 28 ms down to 27 and scales every contention delay
 * with it. The harvested vectors caught exactly that. */
#define FIXED_TIME_HUNDREDTHS 760U

/* Arduino map() as the reference uses it: integer arithmetic, truncating, and
 * notably *not* clamped. Reproduced exactly so the mapping matches; callers
 * clamp the result themselves (see the note in cw_clamp). */
static int32_t map_i32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Live window bounds. Defaults reproduce the reference (3, 8); the "legacy"
 * policy zeroes them to restore transmit-immediately. A max below min is
 * operator error rather than a state worth honouring — collapse to min, which
 * degrades to "one slot" instead of inverting the window. */
static void cw_bounds(uint8_t *lo, uint8_t *hi)
{
	const struct meshtastic_sched_config *c = meshtastic_sched_get();

	*lo = c->cw_min;
	*hi = (c->cw_max < c->cw_min) ? c->cw_min : c->cw_max;
}

/*
 * Deliberate divergence from the reference: it does not clamp getCWsize(), so an
 * SNR outside [-20, 10] produces an exponent outside the window bounds. That is
 * harmless there because the SNR handed in is a real radio measurement, but our
 * snr is an int8 carried in the packet struct and an out-of-range value would
 * become a shift count — snr 127 maps to 27, and 1 << 27 slots of 28 ms is about
 * an hour. Clamping keeps a malformed or hostile value from turning into an
 * unbounded delay.
 */
static uint8_t cw_clamp(int32_t cw, uint8_t lo, uint8_t hi)
{
	if (cw < (int32_t)lo) {
		return lo;
	}
	if (cw > (int32_t)hi) {
		return hi;
	}
	return (uint8_t)cw;
}

/* Uniform in [0, n). Falls back to a monotonic-clock draw if the DRBG fails, so
 * a transient RNG error degrades the randomness rather than collapsing the delay
 * to a constant — every node picking the same slot is the exact failure this
 * whole mechanism exists to prevent. */
static uint32_t random_below(uint32_t n)
{
	uint32_t r = 0U;

	if (n == 0U) {
		return 0U;
	}

	if (psa_generate_random((uint8_t *)&r, sizeof(r)) != PSA_SUCCESS) {
		r = k_cycle_get_32();
	}

	return r % n;
}

uint32_t meshtastic_contention_slot_ms(uint8_t spread_factor, uint32_t bandwidth_hz, bool wide_lora)
{
	uint32_t bw_khz;
	uint32_t cad_hundredths; /* the CAD symbol multiplier, x100 */
	uint32_t slot_hundredths;

	if (spread_factor == 0U || spread_factor > 20U || bandwidth_hz == 0U) {
		return 0U;
	}

	bw_khz = bandwidth_hz / 1000U;
	if (bw_khz == 0U) {
		return 0U;
	}

	if (wide_lora) {
		/* (NUM_SYM_CAD_24GHZ + (2*sf + 3)/32) * symbolTime. That inner
		 * division is integer in the reference too — both operands are ints —
		 * so it contributes 0 for every spreading factor a real radio uses,
		 * and the multiplier is simply NUM_SYM_CAD_24GHZ. Kept in full so it
		 * still tracks if the reference ever widens sf. */
		cad_hundredths = (NUM_SYM_CAD_24GHZ + ((2U * spread_factor + 3U) / 32U)) * 100U;
	} else {
		/* max(2.25, NUM_SYM_CAD + 0.5) * symbolTime. */
		cad_hundredths = MAX(225U, (NUM_SYM_CAD * 100U) + 50U);
	}

	/* One division at the end: multiplying 2^sf by the x100 multiplier before
	 * dividing by the bandwidth keeps the symbol time's fractional part, which
	 * is where the accuracy lives. */
	slot_hundredths = ((uint32_t)BIT(spread_factor) * cad_hundredths) / bw_khz;

	return (slot_hundredths + FIXED_TIME_HUNDREDTHS) / 100U;
}

uint8_t meshtastic_contention_cw_from_snr(int8_t snr)
{
	uint8_t lo, hi;

	cw_bounds(&lo, &hi);
	/* Reference getCWsize(): SNR_MIN -20, SNR_MAX 10. */
	return cw_clamp(map_i32((int32_t)snr, -20, 10, (int32_t)lo, (int32_t)hi), lo, hi);
}

uint8_t meshtastic_contention_cw_from_util(uint8_t util_pct)
{
	uint8_t util = MIN(util_pct, 100U);
	uint8_t lo, hi;

	cw_bounds(&lo, &hi);
	return cw_clamp(map_i32((int32_t)util, 0, 100, (int32_t)lo, (int32_t)hi), lo, hi);
}

/* Slots a client waits before its own random window opens. */
static uint32_t relay_offset_slots(void)
{
	uint8_t lo, hi;

	cw_bounds(&lo, &hi);
	return (uint32_t)meshtastic_sched_get()->cw_relay_offset * (uint32_t)hi;
}

uint32_t meshtastic_contention_effective_slot_ms(uint8_t spread_factor, uint32_t bandwidth_hz,
						 bool wide_lora)
{
	uint16_t override_ms = meshtastic_sched_get()->cw_slot_ms;

	if (override_ms != 0U) {
		return override_ms;
	}
	return meshtastic_contention_slot_ms(spread_factor, bandwidth_hz, wide_lora);
}

uint32_t meshtastic_contention_delay_own_ms(uint8_t util_pct, uint32_t slot_ms)
{
	uint8_t cw = meshtastic_contention_cw_from_util(util_pct);

	return random_below(BIT(cw)) * slot_ms;
}

uint32_t meshtastic_contention_delay_relay_ms(int8_t snr, bool early_like_router, uint32_t slot_ms)
{
	uint8_t cw = meshtastic_contention_cw_from_snr(snr);

	if (early_like_router) {
		return random_below(2U * (uint32_t)cw) * slot_ms;
	}

	/* Offset past the whole router window first, so a router's relay reliably
	 * precedes a client's rather than merely tending to. */
	return (relay_offset_slots() * slot_ms) + (random_below(BIT(cw)) * slot_ms);
}

uint32_t meshtastic_contention_delay_relay_worst_ms(int8_t snr, uint32_t slot_ms)
{
	uint8_t cw = meshtastic_contention_cw_from_snr(snr);

	return (relay_offset_slots() * slot_ms) + (BIT(cw) * slot_ms);
}
