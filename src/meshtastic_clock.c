/* SPDX-License-Identifier: GPL-3.0
 *
 * Wall-clock helper — see meshtastic_clock.h.
 */

#include "meshtastic_clock.h"

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>

/*
 * THE ANCHOR IS A TUPLE, AND THAT IS WHY THERE IS A SEQLOCK HERE.
 *
 * This module used to hold a single int64_t — epoch ms at k_uptime == 0 — and
 * argue, correctly, that lock-free access was fine: a torn read of one word
 * mis-stamps one display value once, and the next seed repairs it.
 *
 * That argument does not survive the anchor becoming (epoch_ms, uptime_ms).
 * The two are written together and are only meaningful together: a reader that
 * catches a NEW epoch beside an OLD anchor uptime is not off by a rounding
 * error, it is off by the whole size of the step — and, if the step was
 * backwards, it reports a time BEFORE one it already reported. A clock that can
 * run backwards under a race is worse than one that is merely coarse, because
 * everything downstream (the HLC, last_heard, the scan tap's resolver) is
 * written on the assumption that it does not.
 *
 * A mutex is not available: meshtastic_log_timestamp() below is called from ISR
 * context, and so — via meshtastic_radio.c's RX callback — is the scanner's
 * resolver. So: a seqlock. Writers are rare (a GNSS fix, an SNTP reply, an admin
 * set_time, the shell) and take a spinlock against each other; readers take
 * nothing, copy the tuple, and retry if the sequence moved underneath them.
 *
 * The retry is BOUNDED, and that bound is the point rather than a detail. An
 * unbounded reader spinning in an ISR behind a preempted writer is a hang, which
 * would be a far worse failure than the one being fixed. Past the bound a reader
 * reports "not seeded" — the same answer it gives before first sync, which every
 * caller already handles.
 *
 * Note the representation change is exactly value-neutral: storing
 * (epoch_ms, anchor_uptime_ms) and computing epoch_ms + (now - anchor_uptime_ms)
 * is the same integer arithmetic as the old boot_epoch_ms + now, with
 * boot_epoch_ms == epoch_ms - anchor_uptime_ms. It is stored this way because
 * rate correction needs the elapsed time SINCE the anchor to multiply by, and
 * "since k_uptime == 0" is the wrong interval.
 *
 * Held in ms rather than seconds so a source carrying a sub-second part (SNTP)
 * is not quantised on the way in. The old seconds anchor also truncated the
 * *uptime* side (k_uptime_seconds()), so the stored offset carried an error of
 * frac(epoch) - frac(uptime) in (-1 s, +1 s) — frozen at sync time, different on
 * every node, and re-randomised at each resync.
 */
/* Reapply window (T-A) for a same-quality NTP-class source: mirrors the
 * reference's 30-minute NTP "slam" so SNTP / phone time can still re-discipline
 * the clock for drift without a higher-quality source present, while a flood of
 * low-trust writes in between is refused. */
#define MESHTASTIC_CLOCK_NTP_REAPPLY_MS (30 * 60 * 1000)

/* Four is not a tuning parameter. On this SoC (CONFIG_SMP=n) k_spin_lock masks
 * interrupts, so a reader cannot interleave with a writer at all and the first
 * attempt always succeeds; the loop exists for correctness under SMP, where one
 * retry per contending writer is the real bound. */
#define MESHTASTIC_CLOCK_READ_RETRIES 4

struct clock_anchor {
	/** Epoch milliseconds AT anchor_uptime_ms (not at k_uptime == 0). */
	int64_t epoch_ms;
	/** k_uptime_get() when this anchor was installed. */
	int64_t anchor_uptime_ms;
	/** Trust level of the source that installed it. */
	enum meshtastic_clock_quality quality;
	/** False until a source has ever been accepted. */
	bool valid;
};

static struct clock_anchor anchor = {
	.quality = MESHTASTIC_CLOCK_QUALITY_NONE,
};

/* Even = stable, odd = a write is in flight. */
static atomic_t anchor_seq;

/* Writers only. Readers are lock-free; see the seqlock note above. */
static struct k_spinlock anchor_write_lock;

/**
 * Publish a new anchor. Callers must hold anchor_write_lock.
 */
static void anchor_write(const struct clock_anchor *new_anchor)
{
	atomic_inc(&anchor_seq); /* -> odd: readers back off */
	barrier_dmem_fence_full();
	anchor = *new_anchor;
	barrier_dmem_fence_full();
	atomic_inc(&anchor_seq); /* -> even: readers may proceed */
}

/**
 * Copy the anchor out consistently.
 *
 * @return true on a clean read. false means either "never seeded" or "gave up
 *         after MESHTASTIC_CLOCK_READ_RETRIES"; both are reported to callers as
 *         an unseeded clock, which is the pre-sync state they already handle.
 */
static bool anchor_read(struct clock_anchor *out)
{
	for (int i = 0; i < MESHTASTIC_CLOCK_READ_RETRIES; i++) {
		atomic_val_t before = atomic_get(&anchor_seq);

		if ((before & 1) != 0) {
			continue; /* a write is in flight */
		}
		barrier_dmem_fence_full();
		*out = anchor;
		barrier_dmem_fence_full();
		if (atomic_get(&anchor_seq) == before) {
			return out->valid;
		}
	}

	return false;
}

/* Gate a clock write by source quality (T-A), mirroring the reference
 * perhapsSetRTC ladder: accept a strictly higher quality, always re-apply GPS
 * (top of the ladder), re-apply an NTP-class source only past the drift window,
 * and otherwise refuse — a lower-trust source must not rewind or advance a clock
 * a better source already set.
 *
 * Reads the anchor directly rather than through anchor_read(): the caller holds
 * the write lock, so the tuple is stable and a retry loop would be noise.
 */
static bool clock_should_set(enum meshtastic_clock_quality quality, int64_t now_ms)
{
	if (quality > anchor.quality) {
		return true;
	}
	if (quality == MESHTASTIC_CLOCK_QUALITY_GPS) {
		return true;
	}
	/* anchor_uptime_ms doubles as "when the clock was last set", because today
	 * every accepted write installs an anchor and nothing else does. If a future
	 * change re-anchors WITHOUT that counting as a fresh sync — slewing a small
	 * correction rather than stepping it, say — this window has to become its own
	 * field, or a slew every few minutes would hold the NTP re-apply gate shut
	 * forever. */
	if (quality == MESHTASTIC_CLOCK_QUALITY_NTP &&
	    (now_ms - anchor.anchor_uptime_ms) >= MESHTASTIC_CLOCK_NTP_REAPPLY_MS) {
		return true;
	}
	return false;
}

void meshtastic_clock_set_epoch_ms(int64_t epoch_ms, enum meshtastic_clock_quality quality)
{
	struct clock_anchor next;
	k_spinlock_key_t key;
	int64_t epoch_sec;
	int64_t now_ms;

	if (epoch_ms < 0) {
		return;
	}

	/* Range-check on the derived second so the window stays exactly the one the
	 * seconds entry point has always enforced. */
	epoch_sec = epoch_ms / 1000;
	if (epoch_sec < (int64_t)MESHTASTIC_EPOCH_MIN || epoch_sec > (int64_t)MESHTASTIC_EPOCH_MAX) {
		return;
	}

	/* One k_uptime_get() for both the drift window and the anchor, so the two
	 * cannot disagree about when this write happened. Taken before the lock so
	 * the spinlock's interrupt-masked section stays as short as it can be. */
	now_ms = k_uptime_get();

	key = k_spin_lock(&anchor_write_lock);

	/* The ladder decision and the write it authorises must be one atomic step.
	 * Split, two concurrent sources could both pass the gate against the same
	 * old quality and the loser's value would land last. */
	if (clock_should_set(quality, now_ms)) {
		next.epoch_ms = epoch_ms;
		next.anchor_uptime_ms = now_ms;
		next.quality = quality;
		next.valid = true;
		anchor_write(&next);
	}

	k_spin_unlock(&anchor_write_lock, key);
}

#if defined(CONFIG_ZTEST)
/* Test seam: the clock is process-global with no production reset (nothing in
 * the firmware ever un-learns time), but tests need a known starting state --
 * several suites set GPS-quality time, which would otherwise leak into any
 * later test of a lower-quality source (the reference has the same seam,
 * PIO_UNIT_TESTING in RTC.cpp). */
void meshtastic_clock_test_reset(void)
{
	struct clock_anchor cleared = {
		.quality = MESHTASTIC_CLOCK_QUALITY_NONE,
	};
	k_spinlock_key_t key = k_spin_lock(&anchor_write_lock);

	/* Normalise the sequence FIRST. anchor_write() bumps it twice, which
	 * preserves parity — so a test that left it odd via
	 * meshtastic_clock_test_hold_write() would still be odd after a reset, and
	 * every subsequent test in the file would read an unseeded clock for reasons
	 * having nothing to do with what it was testing. A reset that does not reset
	 * everything is worse than no reset. */
	atomic_set(&anchor_seq, 0);

	/* Goes through anchor_write() rather than assigning the struct, so a reader
	 * that raced the reset still gets a consistent tuple. */
	anchor_write(&cleared);
	k_spin_unlock(&anchor_write_lock, key);
}

void meshtastic_clock_test_hold_write(bool held)
{
	k_spinlock_key_t key = k_spin_lock(&anchor_write_lock);

	if (held != ((atomic_get(&anchor_seq) & 1) != 0)) {
		atomic_inc(&anchor_seq);
	}
	k_spin_unlock(&anchor_write_lock, key);
}
#endif

void meshtastic_clock_set_epoch(uint32_t epoch_now, enum meshtastic_clock_quality quality)
{
	meshtastic_clock_set_epoch_ms((int64_t)epoch_now * 1000, quality);
}

enum meshtastic_clock_quality meshtastic_clock_get_quality(void)
{
	struct clock_anchor a;

	if (!anchor_read(&a)) {
		return MESHTASTIC_CLOCK_QUALITY_NONE;
	}

	return a.quality;
}

bool meshtastic_clock_valid(void)
{
	struct clock_anchor a;

	return anchor_read(&a);
}

/* The one place the anchor is turned into a time. Everything public below is a
 * thin unit conversion on top of this, so there is exactly one expression to
 * change when rate correction arrives (it becomes elapsed + elapsed*ppb/1e9). */
static int64_t epoch_ms_at_uptime(const struct clock_anchor *a, int64_t uptime_ms)
{
	return a->epoch_ms + (uptime_ms - a->anchor_uptime_ms);
}

uint32_t meshtastic_clock_now_epoch(void)
{
	struct clock_anchor a;

	if (!anchor_read(&a)) {
		return 0U;
	}

	return (uint32_t)(epoch_ms_at_uptime(&a, k_uptime_get()) / 1000);
}

int64_t meshtastic_clock_now_epoch_ms(void)
{
	struct clock_anchor a;

	if (!anchor_read(&a)) {
		return 0;
	}

	return epoch_ms_at_uptime(&a, k_uptime_get());
}

int64_t meshtastic_clock_uptime_ms_to_epoch_ms(int64_t uptime_ms)
{
	struct clock_anchor a;

	if (!anchor_read(&a)) {
		return 0;
	}

	return epoch_ms_at_uptime(&a, uptime_ms);
}

uint32_t meshtastic_clock_uptime_to_epoch(uint32_t uptime_sec)
{
	struct clock_anchor a;

	if (!anchor_read(&a)) {
		return 0U;
	}

	return (uint32_t)(epoch_ms_at_uptime(&a, (int64_t)uptime_sec * 1000) / 1000);
}

#if defined(CONFIG_MESHTASTIC_LOG_WALLCLOCK)

#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>

/*
 * Feed the logging subsystem wall-clock time once the clock is seeded, so syslog
 * and console lines carry real time (2026-..) instead of 1970+uptime. log_output
 * renders "1970 + timestamp/freq", so returning epoch milliseconds with freq=1000
 * yields the real date to the millisecond — note the resolution is ms but the
 * accuracy is the seeding source's. Before the clock is valid, fall back to
 * k_uptime (the familiar 1970+uptime) so early-boot lines stay ordered; the
 * source re-reads the anchor on every call and switches to wall time the instant
 * SNTP/GPS/phone seeds the clock, with no re-registration.
 */
static log_timestamp_t meshtastic_log_timestamp(void)
{
	int64_t up_ms = k_uptime_get();
	struct clock_anchor a;

	/* Called from ISR context, which is the reason the anchor read below is
	 * lock-free and bounded rather than mutex-guarded. If the read gives up, this
	 * falls back to 1970+uptime for that one line — the same rendering every line
	 * gets before first sync, so the log stays ordered and nothing lies. */
	if (anchor_read(&a)) {
		return (log_timestamp_t)epoch_ms_at_uptime(&a, up_ms);
	}

	return (log_timestamp_t)up_ms;
}

static int meshtastic_log_wallclock_init(void)
{
	(void)log_set_timestamp_func(meshtastic_log_timestamp, 1000U);
	return 0;
}

/* APPLICATION level: the log subsystem is up by now; the clock may not be seeded
 * yet, which the source handles per-call (see above). */
SYS_INIT(meshtastic_log_wallclock_init, APPLICATION, 0);

#endif /* CONFIG_MESHTASTIC_LOG_WALLCLOCK */
