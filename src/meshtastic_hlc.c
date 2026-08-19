/* SPDX-License-Identifier: GPL-3.0
 *
 * Hybrid Logical Clock — see meshtastic_hlc.h.
 */

#include "meshtastic_hlc.h"

#include "meshtastic_clock.h"

#include <zephyr/sys/util.h>

/* Counter saturation. A uint32 counter cannot overflow from honest use — it would
 * need 4 billion events inside one millisecond. It CAN overflow from a hostile or
 * corrupt peer sending counter == UINT32_MAX, where a naive `remote->counter + 1`
 * wraps to 0 and silently moves our clock BACKWARDS, breaking the one invariant
 * the whole scheme rests on. Spend a millisecond of physical time instead: strictly
 * forward, and it costs nothing real. */
static void bump_counter(int64_t *physical_ms, uint32_t *counter, uint32_t from)
{
	if (from == UINT32_MAX) {
		*physical_ms += 1;
		*counter = 0U;
		return;
	}

	*counter = from + 1U;
}

void meshtastic_hlc_local_at(struct meshtastic_hlc *hlc, uint32_t node_id, int64_t now_ms,
			     struct meshtastic_hlc_stamp *out)
{
	int64_t prev;

	if (hlc == NULL) {
		return;
	}

	/* A negative now_ms is nonsense; treat it as unseeded rather than letting it
	 * drag the clock backwards. */
	if (now_ms < 0) {
		now_ms = 0;
	}

	prev = hlc->physical_ms;
	hlc->physical_ms = MAX(prev, now_ms);

	if (hlc->physical_ms == prev) {
		/* Physical time did not advance: either two writes inside one
		 * millisecond, or an unseeded clock (now_ms == 0) where the counter is
		 * the only thing carrying order. Same handling for both. */
		bump_counter(&hlc->physical_ms, &hlc->counter, hlc->counter);
	} else {
		hlc->counter = 0U;
	}

	if (out != NULL) {
		out->physical_ms = hlc->physical_ms;
		out->counter = hlc->counter;
		out->node_id = node_id;
	}
}

bool meshtastic_hlc_observe_at(struct meshtastic_hlc *hlc,
			       const struct meshtastic_hlc_stamp *remote, int64_t now_ms)
{
	int64_t prev;
	int64_t remote_ms;
	int64_t l;
	bool clamped = false;

	if (hlc == NULL || remote == NULL) {
		return false;
	}

	if (now_ms < 0) {
		now_ms = 0;
	}

	remote_ms = remote->physical_ms;

	/* Drift guard. Only meaningful when our own clock is seeded (now_ms > 0) — an
	 * unseeded node has no standing to judge anyone else's sense of time, and
	 * clamping on a zero reference would reject every legitimate peer. */
	if (now_ms > 0 && remote_ms > now_ms + MESHTASTIC_HLC_MAX_DRIFT_MS) {
		remote_ms = now_ms;
		clamped = true;
	}

	prev = hlc->physical_ms;
	l = MAX(prev, MAX(remote_ms, now_ms));

	if (l == prev && l == remote_ms) {
		/* Tied with the remote in the same millisecond: step past the higher of
		 * the two counters, so we end strictly ahead of both. */
		bump_counter(&l, &hlc->counter, MAX(hlc->counter, remote->counter));
	} else if (l == prev) {
		bump_counter(&l, &hlc->counter, hlc->counter);
	} else if (l == remote_ms) {
		bump_counter(&l, &hlc->counter, remote->counter);
	} else {
		/* Our own physical clock is the maximum: it has genuinely advanced past
		 * everything seen, so the counter resets. */
		hlc->counter = 0U;
	}

	hlc->physical_ms = l;

	return clamped;
}

int meshtastic_hlc_compare(const struct meshtastic_hlc_stamp *a,
			   const struct meshtastic_hlc_stamp *b)
{
	if (a->physical_ms != b->physical_ms) {
		return (a->physical_ms > b->physical_ms) ? 1 : -1;
	}

	if (a->counter != b->counter) {
		return (a->counter > b->counter) ? 1 : -1;
	}

	if (a->node_id != b->node_id) {
		return (a->node_id > b->node_id) ? 1 : -1;
	}

	return 0;
}

void meshtastic_hlc_local(struct meshtastic_hlc *hlc, uint32_t node_id,
			  struct meshtastic_hlc_stamp *out)
{
	meshtastic_hlc_local_at(hlc, node_id, meshtastic_clock_now_epoch_ms(), out);
}

bool meshtastic_hlc_observe(struct meshtastic_hlc *hlc,
			    const struct meshtastic_hlc_stamp *remote)
{
	return meshtastic_hlc_observe_at(hlc, remote, meshtastic_clock_now_epoch_ms());
}
