/* SPDX-License-Identifier: GPL-3.0
 *
 * Hybrid Logical Clock (HLC) — versions for last-writer-wins config merge.
 *
 * The problem this solves: two nodes out of contact both change a setting. When
 * they meet, which value is correct? LWW answers "the higher version", which only
 * works if versions are comparable and meaningful. Neither obvious source is:
 *
 *   - Wall-clock time alone breaks under skew. A node whose clock runs fast wins
 *     every conflict forever, including against changes genuinely made later.
 *   - A logical counter (Lamport) captures causality but the numbers mean nothing
 *     to a human, which is miserable when reading a mesh log after the fact.
 *
 * HLC keeps a pair — physical milliseconds plus a counter that only advances when
 * two events land in the same millisecond. The result reads as a real timestamp,
 * never runs backwards, and respects causality (if A influenced B, B compares
 * higher). See Kulkarni et al., "Logical Physical Clocks" (OPODIS 2014).
 *
 * Degradation is deliberate: a node whose wall clock has never been seeded gets
 * now_ms == 0, the physical component stops advancing, and the counter carries the
 * ordering on its own — i.e. it silently becomes a Lamport clock and still
 * converges. Its writes lose to any clock-bearing node, which is the behaviour we
 * want from a node that cannot say when anything happened.
 *
 * A side effect of that path is worth knowing: because physical never advances from
 * its initial 0, the FIRST unseeded write takes the "physical did not advance" branch
 * and its counter starts at 1, not 0. So an unseeded stamp is {0, 1, node_id} and can
 * never collide with the all-zero "never set" sentinel that
 * meshtastic_hlc_stamp_is_unset() tests for. Convenient, and load-bearing — do not
 * "fix" the counter to start at 0.
 *
 * Concurrency: these functions are pure operations on caller-owned state. They take
 * no locks. A shared clock must be serialised by its owner (the config store
 * already holds store_lock() across its writes).
 */
#ifndef MESHTASTIC_HLC_H_
#define MESHTASTIC_HLC_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief How far ahead of our own clock a peer's timestamp may be before we stop
 *        letting it drag ours forward.
 *
 * Legitimate skew between SNTP/GNSS-synced nodes is milliseconds (see
 * docs/MULTI-PRESET-OPERATION.md §5.5), so five minutes is enormously generous.
 * The point is only to stop one node with a broken clock — a GPS week-rollover, a
 * bad NTP answer — from permanently dragging the whole fleet's clock into the
 * future, which a naive max() would do and never recover from.
 */
#define MESHTASTIC_HLC_MAX_DRIFT_MS (5 * 60 * 1000)

/** Clock state for one node. Zero-initialised is a valid starting state. */
struct meshtastic_hlc {
	int64_t physical_ms; /**< wall-clock ms at the last event */
	uint32_t counter;    /**< tiebreak within one millisecond */
};

/**
 * @brief A version stamped onto a value.
 *
 * The clock pair plus the node that minted it. @c node_id is identity rather than
 * time: it exists so two nodes can never produce an identical version, which would
 * leave a conflict with no deterministic winner.
 */
struct meshtastic_hlc_stamp {
	int64_t physical_ms;
	uint32_t counter;
	uint32_t node_id;
};

/**
 * @brief Mint a version for a local write. Pure form — @p now_ms is supplied.
 *
 * @param hlc     Clock state, advanced in place.
 * @param node_id This node's id, copied into @p out.
 * @param now_ms  Current wall clock in epoch ms, or 0 if unseeded (see file header).
 * @param out     Receives the minted stamp. May be NULL to advance the clock only.
 */
void meshtastic_hlc_local_at(struct meshtastic_hlc *hlc, uint32_t node_id, int64_t now_ms,
			     struct meshtastic_hlc_stamp *out);

/**
 * @brief Fold an observed remote stamp into our clock. Pure form.
 *
 * Call on receiving any stamped value, whether or not the value is accepted — the
 * point is to stay causally ahead of anything we have seen.
 *
 * A @p remote more than @ref MESHTASTIC_HLC_MAX_DRIFT_MS beyond @p now_ms is not
 * allowed to advance our physical component (our clock is protected), but this
 * function deliberately does NOT reject the stamp: whether such a write *wins* is
 * an LWW-layer policy decision, not a clock one. @ref meshtastic_hlc_compare stays
 * a pure total order either way.
 *
 * @return true if @p remote was beyond the drift window and was clamped.
 */
bool meshtastic_hlc_observe_at(struct meshtastic_hlc *hlc,
			       const struct meshtastic_hlc_stamp *remote, int64_t now_ms);

/**
 * @brief Total order over stamps: physical, then counter, then node_id.
 *
 * A total order (never 0 for two distinct stamps) is what lets every node pick the
 * same winner independently, with no coordination.
 *
 * @return <0 if @p a orders before @p b, >0 if after, 0 only if identical.
 */
int meshtastic_hlc_compare(const struct meshtastic_hlc_stamp *a,
			   const struct meshtastic_hlc_stamp *b);

/** True if @p a is strictly newer than @p b — the LWW "does this write win?" test. */
static inline bool meshtastic_hlc_newer(const struct meshtastic_hlc_stamp *a,
					const struct meshtastic_hlc_stamp *b)
{
	return meshtastic_hlc_compare(a, b) > 0;
}

/** True if @p s has never been set (an unversioned value; loses to everything). */
static inline bool meshtastic_hlc_stamp_is_unset(const struct meshtastic_hlc_stamp *s)
{
	return s->physical_ms == 0 && s->counter == 0U && s->node_id == 0U;
}

/* Convenience wrappers that read the node's wall clock (meshtastic_clock). Kept
 * separate from the _at forms so tests can drive time deterministically. */

/** @brief Mint a version for a local write, using the node wall clock. */
void meshtastic_hlc_local(struct meshtastic_hlc *hlc, uint32_t node_id,
			  struct meshtastic_hlc_stamp *out);

/** @brief Fold in a remote stamp, using the node wall clock. */
bool meshtastic_hlc_observe(struct meshtastic_hlc *hlc,
			    const struct meshtastic_hlc_stamp *remote);

#endif /* MESHTASTIC_HLC_H_ */
