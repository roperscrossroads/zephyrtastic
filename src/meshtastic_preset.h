/* SPDX-License-Identifier: GPL-3.0
 *
 * Runtime modem-preset switching. See meshtastic_preset.c for why a preset
 * change moves the frequency and the channel hash as well as the modem.
 */
#ifndef MESHTASTIC_PRESET_H_
#define MESHTASTIC_PRESET_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic/config.pb.h"

/** What a switch actually resolved to. Every field is a thing that must change
 *  together; a caller (or test) asserting on only one of them cannot tell a
 *  complete switch from a half-applied one. */
struct meshtastic_preset_result {
	meshtastic_Config_LoRaConfig_ModemPreset preset;
	uint32_t frequency_hz;
	uint32_t bandwidth_hz;
	uint8_t  spread_factor;
	uint8_t  channel_hash;
};

/**
 * @brief Move the radio to a different modem preset, live, without a reboot.
 *
 * Applies the modem params, re-derives every channel hash, resolves the new
 * frequency slot, pushes it all to the radio and re-arms RX — in that order,
 * because the frequency depends on the refreshed channel name.
 *
 * @warning This makes the node deaf to every peer still on the old preset, and
 * inaudible to them, immediately. Presets are doubly orthogonal (different
 * frequency AND different spreading factor), so there is no partial overlap to
 * fall back on. Switching a whole fleet is a partition-inducing operation —
 * see docs/MULTI-PRESET-OPERATION.md §1.1.
 *
 * Atomicity: an unknown preset changes nothing. A preset with no frequency plan
 * for the current region is rolled back. Only a radio-level failure can leave
 * the node mid-switch, and that is already fatal for TX either way.
 *
 * @param preset Target preset.
 * @param out    Optional; receives what the switch resolved to.
 *
 * @return 0 on success, -EINVAL for an unknown preset, -ENOTSUP when the region
 *         has no frequency plan (rolled back), or a negative errno from the
 *         radio.
 */
/**
 * @brief The region the frequency plan should be resolved against.
 *
 * Read from the stored LoRaConfig on each call rather than cached: an admin can
 * change the region, and a stale copy would put the node on a frequency that is
 * legal somewhere else. UNSET keeps the compile-time default.
 */
meshtastic_Config_LoRaConfig_RegionCode meshtastic_preset_region(void);

int meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset preset,
			     struct meshtastic_preset_result *out);

/* --- Interlocks: docs/MULTI-PRESET-OPERATION.md §4.4 -----------------------
 *
 * meshtastic_preset_switch() above is the primitive: it does what it is told,
 * immediately, and is what an operator or an admin config change should reach
 * for. meshtastic_preset_hop() below is the same switch performed on a
 * SCHEDULE — a slot boundary rather than a human — and a schedule has no
 * judgement to apply, so the judgement has to be encoded.
 *
 * The distinction is the whole point of this half of the file. "Change preset
 * because I said so" and "change preset because the clock says it is 15 s past"
 * want opposite failure behaviour: the first should obey and report, the second
 * should refuse and try again at the next boundary.
 */

/**
 * @brief Why a scheduled hop is being held off.
 *
 * Ordered by decreasing permanence, and reported first-match, so a caller that
 * shows one reason shows the one least likely to clear on its own: a node with
 * no clock must not slice at all until it finds a time source, whereas a queued
 * frame clears in milliseconds.
 */
enum meshtastic_preset_hold {
	MESHTASTIC_PRESET_HOLD_NONE = 0,
	/** No wall clock. slot = f(epoch) is meaningless without an epoch, and a
	 *  node hopping on a guessed schedule is on nobody else's schedule. */
	MESHTASTIC_PRESET_HOLD_NO_CLOCK,
	/** A survey owns the radio; it is already off the operating preset. */
	MESHTASTIC_PRESET_HOLD_SCANNING,
	/** An originated want_ack unicast is still awaiting its ACK. */
	MESHTASTIC_PRESET_HOLD_RELIABLE,
	/** Frames still queued (or one being keyed) for the preset we are on. */
	MESHTASTIC_PRESET_HOLD_TX_QUEUED,
	/** Another hop is already mid-flight. Two schedules driving one radio is
	 *  a caller bug; the guard exists so it stays a refusal rather than a
	 *  half-applied switch, and it gets its own counter so it reads as one. */
	MESHTASTIC_PRESET_HOLD_SWITCHING,
	MESHTASTIC_PRESET_HOLD_COUNT,
};

/** @brief Short lowercase name for @p reason, for logs and the shell. */
const char *meshtastic_preset_hold_name(enum meshtastic_preset_hold reason);

/**
 * @brief What would hold a scheduled hop off right now, if anything.
 *
 * A snapshot, not a reservation — everything it reports can change in the next
 * millisecond. Use it to explain, not to decide; meshtastic_preset_hop() makes
 * the decision itself under the freeze it takes out.
 */
enum meshtastic_preset_hold meshtastic_preset_hold_check(void);

/**
 * @brief Switch preset on a schedule, honouring the interlocks.
 *
 * Refuses rather than waits for the first three conditions above: a hop missed
 * at one slot boundary costs nothing, because slot = f(epoch) means the next
 * boundary arrives on its own and puts the node back on the fleet's schedule
 * with no negotiation. Waiting, by contrast, would drag the node into the next
 * slot late and out of step with every other slicer.
 *
 * The TX queue is the exception: it is drained rather than refused on, for up to
 * CONFIG_MESHTASTIC_PRESET_DRAIN_MS, because it is the one condition guaranteed
 * to clear on its own and usually within a frame time. Draining is not a freeze
 * — the queued frames genuinely transmit, on the preset they were composed for,
 * which is the whole point of waiting for them rather than discarding them.
 *
 * What the drain cannot cover is a frame composed in the moment between the
 * queue reporting empty and the retune finishing. That one is caught afterwards
 * by the generation stamp (meshtastic_preset_generation()) and dropped.
 *
 * @param preset Target preset.
 * @param out    Optional; receives what the switch resolved to.
 *
 * @return 0 on success, -EBUSY when an interlock held it off (the node has not
 *         moved), or any error from meshtastic_preset_switch().
 */
int meshtastic_preset_hop(meshtastic_Config_LoRaConfig_ModemPreset preset,
			  struct meshtastic_preset_result *out);

/**
 * @brief Bumped once per successful preset switch. Never reused, never reset.
 *
 * The stamp that makes "this frame belongs to the preset we are on" decidable
 * rather than merely likely. A queued frame records the generation it was
 * composed under; the outbound worker compares that with this before keying the
 * PA and drops the frame if they differ.
 *
 * A drain alone cannot close the hole. It empties the queue, but the instant it
 * reports empty a frame can still be composed against the old channel hash and
 * land in the queue while the retune is happening — and for a default
 * (empty-name) channel that hash is derived from the preset's display name, so
 * the frame would go out on the new frequency addressed to a channel nobody on
 * it is listening for. The drain is what makes stale frames rare; this is what
 * makes them impossible.
 *
 * Bumped AFTER the switch completes, deliberately. That errs toward dropping a
 * frame composed mid-switch, which is the safe direction — the alternative
 * lets one through whose hash may be from either side.
 */
uint32_t meshtastic_preset_generation(void);

/** @brief Count a frame dropped because it was composed for an older preset. */
void meshtastic_preset_note_tx_stale(void);

/**
 * @brief Milliseconds still owed to the post-switch settle window, or 0.
 *
 * The TX path sleeps this out before keying the PA. See
 * CONFIG_MESHTASTIC_PRESET_SETTLE_MS for why a retuned radio is not immediately
 * ready to judge whether the channel is busy.
 */
uint32_t meshtastic_preset_settle_remaining_ms(void);

/** @brief Count a transmit that had to wait out the settle window. */
void meshtastic_preset_note_settle_wait(void);

/* --- Counters. Refusals are counted rather than silent for the same reason the
 * scanner counts its own: "nothing tried to transmit mid-switch" should be an
 * assertion, not an assumption. --- */

/** @brief Hops completed since the last reset. */
uint32_t meshtastic_preset_hops(void);
/** @brief Hops refused for @p reason since the last reset. */
uint32_t meshtastic_preset_holds(enum meshtastic_preset_hold reason);
/** @brief Frames dropped for carrying an older preset's generation. */
uint32_t meshtastic_preset_tx_stale(void);
/** @brief Transmits that waited out a settle window. */
uint32_t meshtastic_preset_settle_waits(void);
/** @brief Clear every counter above (test and shell use). */
void meshtastic_preset_stats_reset(void);

#endif /* MESHTASTIC_PRESET_H_ */
