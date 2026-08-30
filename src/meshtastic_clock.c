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
#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
	/** Rate correction in parts per billion; see the skew note below. Rides in
	 *  the anchor so the hot path reads it under the same seqlock, and so a
	 *  reader can never pair a new anchor with a stale correction. */
	int32_t skew_ppb;
	/** False until a window long enough to measure has closed. Distinct from
	 *  skew_ppb == 0, which is a perfectly good estimate. */
	bool skew_valid;
#endif
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

#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
/*
 * RATE CORRECTION — the clock is disciplined, not merely stepped.
 *
 * Everything above establishes WHAT time it is at one instant. Between those
 * instants the clock free-runs on k_uptime, and k_uptime runs on a crystal that
 * is not exactly right — measured on this board at -2 ppm, which is 173 ms of
 * error per day. Acquisition cannot fix that; only measuring the rate can.
 *
 * The measurement is a two-point one and deliberately nothing cleverer. Hold the
 * (epoch, uptime) pair of one trusted sync as a base; when a later trusted sync
 * arrives, compare how much real time passed against how much local time passed.
 * The difference over the interval IS the rate error:
 *
 *     skew_ppb = 1e9 * (real elapsed - local elapsed) / local elapsed
 *
 * so a local clock running FAST gives local > real, a negative ppb, and a
 * correction that subtracts. All integer: this value is consumed by
 * epoch_ms_at_uptime() below, which runs in the LoRa RX callback and in
 * meshtastic_log_timestamp(), i.e. in ISR context, where float has no business.
 *
 * WHY THIS CANNOT STEP THE CLOCK. A new estimate changes the reported time by
 * (elapsed since anchor) * (change in ppb). skew_estimate() is only ever called
 * from inside an ACCEPTED clock write — the same write that re-anchors — so at
 * the moment a new value lands, elapsed is zero, or ~853 ms for a PPS anchor
 * sitting slightly in the past. At 2 ppm that is 1.7 microseconds. A correction
 * that arrives with the anchor cannot produce the backwards step the seqlock
 * above exists to prevent, and the test asserts it.
 *
 * WHAT IT DOES NOT DO. A node holding a GNSS fix re-anchors every second, so the
 * elapsed time this multiplies is ~1 s and the correction is nanoseconds. This
 * is worth nothing to a node that can see the sky and a great deal to one that
 * cannot — which is the honest framing, and the one in the Kconfig help.
 */

/** The sync that opened the current measurement window. */
static int64_t skew_base_epoch_ms;
static int64_t skew_base_uptime_ms;
/** The most recent sync folded in, for reporting the window length. */
static int64_t skew_last_uptime_ms;
static bool skew_base_valid;

#define SKEW_PPB_PER_PPM 1000LL
#define SKEW_MAX_PPB ((int64_t)CONFIG_MESHTASTIC_CLOCK_SKEW_MAX_PPM * SKEW_PPB_PER_PPM)
#define SKEW_MIN_WINDOW_MS ((int64_t)CONFIG_MESHTASTIC_CLOCK_SKEW_MIN_WINDOW_S * 1000LL)

/**
 * The whole estimator, as a pure function of the two intervals.
 *
 * Pure so it can be tested exactly rather than approximately: every guard below
 * is a branch a test can hit by choosing two numbers, with no clock, no waiting
 * and no hardware.
 *
 * @return true if the window produced a usable estimate.
 */
static bool skew_estimate(int64_t d_ref_ms, int64_t d_local_ms, int32_t *out_ppb)
{
	int64_t ppb;

	/* Too short a window divides the source's own error by too small a number
	 * and reports the result as a rate. See MIN_WINDOW_S's help: the bound is
	 * set by how accurate the SOURCE is, not by how patient we are. */
	if (d_local_ms < SKEW_MIN_WINDOW_MS) {
		return false;
	}

	ppb = ((d_ref_ms - d_local_ms) * 1000000000LL) / d_local_ms;

	/* Beyond the sanity bound this is not a rate, it is a bad pair of syncs.
	 * Refusing leaves the previous correction standing, and having none at all
	 * is exactly the behaviour before this existed — so the failure is safe in
	 * both directions. */
	if (ppb > SKEW_MAX_PPB || ppb < -SKEW_MAX_PPB) {
		return false;
	}

	*out_ppb = (int32_t)ppb;
	return true;
}

/**
 * Fold one accepted sync into the estimate. Callers must hold
 * anchor_write_lock; @p next is the anchor about to be published.
 *
 * @return true if THIS call installed an estimate (as opposed to opening the
 *         window, being refused, or closing a window still too short).
 */
static bool skew_feed(int64_t epoch_ms, int64_t uptime_ms,
		      enum meshtastic_clock_quality quality,
		      enum meshtastic_clock_precision precision, struct clock_anchor *next)
{
	int32_t ppb;

	/*
	 * Only a source that independently knows the time can measure our rate.
	 * NET is another node's opinion, carrying that node's own error; DEVICE is
	 * a clock we restored from our own retained RAM, so feeding it would have
	 * the estimator measure itself and conclude, with confidence, zero.
	 */
	if (quality < MESHTASTIC_CLOCK_QUALITY_NTP) {
		return false;
	}

	/*
	 * AND it must know the INSTANT to better than a whole second, which is a
	 * different property from trust and one quality cannot express. The phone's
	 * admin set_time_only is NTP-class and the phone knows the time perfectly
	 * well, but the protobuf field carries whole seconds, so what arrives is
	 * +/-1 s; an operator typing at a console is the same. Over a ten-hour
	 * window that reads as 28 ppm — inside the plausibility bound, and more than
	 * ten times the error being corrected. A correction that large is worse
	 * than none.
	 *
	 * The source states this rather than having it inferred. It used to be
	 * inferred from which setter was called, which was true of every caller at
	 * the time and would have silently stopped being true as soon as a
	 * millisecond-resolution console command existed.
	 */
	if (precision != MESHTASTIC_CLOCK_PRECISION_SUBSECOND) {
		return false;
	}

	if (!skew_base_valid) {
		skew_base_epoch_ms = epoch_ms;
		skew_base_uptime_ms = uptime_ms;
		skew_last_uptime_ms = uptime_ms;
		skew_base_valid = true;
		return false;
	}

	skew_last_uptime_ms = uptime_ms;

	/*
	 * The base is NOT replaced on success. A two-point secant has no filtering,
	 * so its only defence against noise is a long baseline — re-basing on every
	 * sync would reset the window to zero and make the estimate permanently as
	 * bad as the shortest interval. The window therefore grows without bound,
	 * which also means the estimate stops tracking a rate that changes with
	 * temperature. Accepted for now: a crystal's tempco is far below the error
	 * this removes, and a node that reboots re-bases anyway.
	 */
	if (!skew_estimate(epoch_ms - skew_base_epoch_ms, uptime_ms - skew_base_uptime_ms,
			   &ppb)) {
		return false;
	}

	next->skew_ppb = ppb;
	next->skew_valid = true;
	return true;
}
#endif /* CONFIG_MESHTASTIC_CLOCK_SKEW */

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
	/* anchor_uptime_ms doubles as "when the clock was last set". Since PPS
	 * anchoring it can sit slightly in the past of the write that installed it —
	 * under one second, because an older edge is refused — which is three orders
	 * inside this thirty-minute window and so cannot change the decision.
	 *
	 * It would still have to become its own field if something ever re-anchored
	 * WITHOUT that counting as a fresh sync: slewing a small correction rather
	 * than stepping it, say, where a slew every few minutes would hold this gate
	 * shut forever. Every write today is a real sync. */
	if (quality == MESHTASTIC_CLOCK_QUALITY_NTP &&
	    (now_ms - anchor.anchor_uptime_ms) >= MESHTASTIC_CLOCK_NTP_REAPPLY_MS) {
		return true;
	}
	return false;
}

void meshtastic_clock_set_epoch_ms(int64_t epoch_ms, enum meshtastic_clock_quality quality,
				   enum meshtastic_clock_precision precision)
{
	/* "Now" is the ordinary assumption: the value became true when it arrived. */
	meshtastic_clock_set_epoch_ms_at(epoch_ms, k_uptime_get(), quality, precision);
}

/**
 * The one implementation. @p precision is the source's own claim about how
 * tightly it pins the instant; skew_feed() decides what that is good for.
 */
static void clock_set(int64_t epoch_ms, int64_t at_uptime_ms,
		      enum meshtastic_clock_quality quality,
		      enum meshtastic_clock_precision precision)
{
	struct clock_anchor next;
	k_spinlock_key_t key;
	int64_t epoch_sec;
	int64_t now_ms;

#if !defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
	ARG_UNUSED(precision);
#endif

	if (epoch_ms < 0) {
		return;
	}

	/* Range-check on the derived second so the window stays exactly the one the
	 * seconds entry point has always enforced. */
	epoch_sec = epoch_ms / 1000;
	if (epoch_sec < (int64_t)MESHTASTIC_EPOCH_MIN || epoch_sec > (int64_t)MESHTASTIC_EPOCH_MAX) {
		return;
	}

	now_ms = k_uptime_get();

	/* The anchor may sit slightly in the PAST of this decision — a PPS edge is
	 * ~853 ms before the sentence that names it. Clamp rather than trust: an
	 * anchor in the future would make the clock run backwards until uptime
	 * caught up, which is the one outcome the tuple representation exists to
	 * prevent. */
	if (at_uptime_ms > now_ms || at_uptime_ms < 0) {
		at_uptime_ms = now_ms;
	}

	key = k_spin_lock(&anchor_write_lock);

	/* The ladder decision and the write it authorises must be one atomic step.
	 * Split, two concurrent sources could both pass the gate against the same
	 * old quality and the loser's value would land last. */
	if (clock_should_set(quality, now_ms)) {
		next.epoch_ms = epoch_ms;
		next.anchor_uptime_ms = at_uptime_ms;
		next.quality = quality;
		next.valid = true;
#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
		/* The learned rate is a property of the board, not of this sync, so it
		 * survives the re-anchor. Then let this sync refine it — inside the
		 * same write, which is what keeps a new estimate from stepping the
		 * clock (see the skew note above). */
		next.skew_ppb = anchor.skew_ppb;
		next.skew_valid = anchor.skew_valid;
		(void)skew_feed(epoch_ms, at_uptime_ms, quality, precision, &next);
#endif
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

#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
	/* The estimator's base lives OUTSIDE the anchor, so clearing the anchor does
	 * not clear it. Leaving it behind would let one test's base pair with the
	 * next test's sync and produce an estimate over a window that never existed
	 * — and ztest runs in name order, so which tests collide would change every
	 * time a test is renamed. */
	skew_base_valid = false;
	skew_base_epoch_ms = 0;
	skew_base_uptime_ms = 0;
	skew_last_uptime_ms = 0;
#endif

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

void meshtastic_clock_set_epoch_ms_at(int64_t epoch_ms, int64_t at_uptime_ms,
				      enum meshtastic_clock_quality quality,
				      enum meshtastic_clock_precision precision)
{
	clock_set(epoch_ms, at_uptime_ms, quality, precision);
}

void meshtastic_clock_set_epoch(uint32_t epoch_now, enum meshtastic_clock_quality quality)
{
	/* Whole seconds in, so a whole-second instant out, however good the clock
	 * behind it. Sets the clock exactly as before; never disciplines the rate. */
	clock_set((int64_t)epoch_now * 1000, k_uptime_get(), quality,
		  MESHTASTIC_CLOCK_PRECISION_SECOND);
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
	int64_t elapsed_ms = uptime_ms - a->anchor_uptime_ms;

#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
	/* Integer throughout: this runs in ISR context. elapsed_ms is bounded by
	 * uptime and skew_ppb by SKEW_MAX_PPB, so the product cannot overflow
	 * int64 for any uptime this hardware will ever reach (1e9 ms x 1e5 ppb is
	 * 1e14, against a 9.2e18 ceiling).
	 *
	 * elapsed_ms is negative when resolving a PAST uptime — the scanner's
	 * records, last_heard — and C truncates division toward zero, so the
	 * correction is asymmetric about the anchor by less than one millisecond.
	 * Below the resolution of everything that consumes it. */
	return a->epoch_ms + elapsed_ms + (elapsed_ms * a->skew_ppb) / 1000000000LL;
#else
	return a->epoch_ms + elapsed_ms;
#endif
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

#if defined(CONFIG_MESHTASTIC_CLOCK_SKEW)
bool meshtastic_clock_skew(int32_t *ppb, uint32_t *window_s)
{
	k_spinlock_key_t key = k_spin_lock(&anchor_write_lock);
	bool valid = anchor.skew_valid;

	/* Not the hot path — this is a reporting call, so it takes the writers'
	 * lock rather than the seqlock and reads the base statics beside the
	 * anchor consistently. */
	if (ppb != NULL) {
		*ppb = anchor.skew_ppb;
	}
	if (window_s != NULL) {
		/* Filled in even when no estimate exists yet: "N seconds into a window
		 * that needs M" is the answer to the question someone typing
		 * `meshtastic time` is actually asking, and zero would read as
		 * "nothing is happening". */
		*window_s = skew_base_valid
				    ? (uint32_t)((skew_last_uptime_ms - skew_base_uptime_ms) / 1000)
				    : 0U;
	}
	k_spin_unlock(&anchor_write_lock, key);

	return valid;
}

#if defined(CONFIG_ZTEST)
bool meshtastic_clock_test_skew_estimate(int64_t d_ref_ms, int64_t d_local_ms, int32_t *ppb)
{
	return skew_estimate(d_ref_ms, d_local_ms, ppb);
}

bool meshtastic_clock_test_skew_feed(int64_t epoch_ms, int64_t uptime_ms,
				     enum meshtastic_clock_quality quality,
				     enum meshtastic_clock_precision precision)
{
	struct clock_anchor next;
	k_spinlock_key_t key = k_spin_lock(&anchor_write_lock);
	bool produced;

	/* Drives the SAME skew_feed() the accepted-write path calls, with a
	 * synthetic (epoch, uptime) pair — because the real path anchors against
	 * k_uptime_get() and the windows worth testing are hours long. */
	next = anchor;
	produced = skew_feed(epoch_ms, uptime_ms, quality, precision, &next);
	anchor_write(&next);
	k_spin_unlock(&anchor_write_lock, key);

	return produced;
}

void meshtastic_clock_test_set_skew(int32_t ppb)
{
	struct clock_anchor next;
	k_spinlock_key_t key = k_spin_lock(&anchor_write_lock);

	next = anchor;
	next.skew_ppb = ppb;
	next.skew_valid = true;
	anchor_write(&next);
	k_spin_unlock(&anchor_write_lock, key);
}
#endif /* CONFIG_ZTEST */
#endif /* CONFIG_MESHTASTIC_CLOCK_SKEW */

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
