/* SPDX-License-Identifier: GPL-3.0
 *
 * Signal-quality aggregation — see meshtastic_rf_measure.h and
 * main/docs/rf-measurement.md.
 *
 * Two decisions carry most of the weight here.
 *
 * WHERE THE HOOK SITS. In the radio RX callback, after the scanner's
 * off-preset gate and before the packet reaches the software queue:
 *
 *   - After the scanner gate, because a sweeping node is parked on other
 *     people's frequencies and spreading factors. Those frames say nothing about
 *     the chain this node actually operates on, and letting them into the same
 *     distribution would quietly poison every comparison.
 *
 *   - Before the queue, because a frame the queue drops was still demodulated.
 *     A full queue is a limit of our software, not of the RF chain, and omitting
 *     those frames would bias the sample downward exactly during the bursts an
 *     experiment most wants to capture.
 *
 * It is deliberately NOT in the router: the router discards ignored, duplicate
 * and undecodable frames before RSSI is ever read, and weak undecodable frames
 * are precisely the population an antenna or LNA change moves.
 *
 * LOCKING. A spinlock, not a mutex. The writer runs on the driver workqueue and
 * must not be delayed; the readers are the shell. The discipline that makes that
 * safe is: snapshot under the lock into caller-provided storage, format outside
 * it. Holding a lock across forty lines of shell output at 115200 baud would
 * stall the radio path for tens of milliseconds.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_rf_measure.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define PEER_SLOTS CONFIG_MESHTASTIC_RF_PEERS

static struct k_spinlock rf_lock;

static struct {
	struct meshtastic_rf_hist hist;
	struct meshtastic_rf_peer peers[PEER_SLOTS];
	uint32_t started_ms;
	uint32_t lifetime;
	uint32_t evicted;
	uint8_t preset;
	bool preset_seen;
	bool preset_mixed;
	uint8_t mru; /* last peer slot touched — most frames arrive in runs */
} rf;

/*
 * Bin edges.
 *
 * RSSI in 5 dB steps: the SX126x quantises to 0.5 dB but is only accurate to
 * about +/-2 dB, so anything finer bins measurement noise. A change worth acting
 * on — an LNA, a real antenna swap — is 3 to 6 dB, which lands in the next bin.
 *
 * The rail bin is separate and load-bearing. RssiPkt is an unsigned -dBm/2 byte,
 * so the scale runs 0 to -127.5 dBm and ZERO IS THE TOP, not a missing reading.
 * A bench node a metre from its peer pins there, and a pinned receiver cannot
 * demonstrate any improvement — so the condition that invalidates a whole
 * comparison has to be visible rather than folded into the top ordinary bin.
 */
uint8_t meshtastic_rf_rssi_bin(int16_t rssi)
{
	int idx;

	if (rssi >= 0) {
		return MESHTASTIC_RF_RSSI_RAIL_BIN;
	}
	if (rssi < MESHTASTIC_RF_RSSI_FLOOR) {
		return 0U;
	}

	/* Pre-clamped to [-130, 0), so this division never sees a negative. The
	 * bound below is nevertheless explicit rather than reasoned: this index
	 * subscripts an array from the receive path, and an arithmetic edge case
	 * here would be an out-of-bounds write on every frame. Cheap insurance
	 * against a future edit to the floor, the step, or the bin count. */
	idx = 1 + ((rssi - MESHTASTIC_RF_RSSI_FLOOR) / MESHTASTIC_RF_RSSI_STEP);

	return (uint8_t)CLAMP(idx, 0, MESHTASTIC_RF_RSSI_RAIL_BIN - 1);
}

/*
 * SNR in 2 dB steps: unlike RSSI its interesting range sits near the
 * demodulation floor, and 2 dB resolves a 3 dB effect into distinct bins.
 *
 * Note the top edge is >= +14, not > +14: bins 1..18 cover -22..+13, and +14
 * shares the overflow bin. That is a consequence of 18 two-dB bins spanning
 * exactly 36 dB from -22, and it is stated here because the labels printed by
 * `rf hist` have to agree with it — an off-by-one in a bin LABEL is invisible
 * in testing and quietly misreports every reading at the edge.
 */
uint8_t meshtastic_rf_snr_bin(int8_t snr)
{
	int idx;

	if (snr < MESHTASTIC_RF_SNR_FLOOR) {
		return 0U;
	}

	idx = 1 + ((snr - MESHTASTIC_RF_SNR_FLOOR) / MESHTASTIC_RF_SNR_STEP);

	return (uint8_t)CLAMP(idx, 0, MESHTASTIC_RF_SNR_BINS - 1);
}

static void hist_add(struct meshtastic_rf_hist *h, int16_t rssi, int8_t snr)
{
	if (h->frames == 0U) {
		h->rssi_min = rssi;
		h->rssi_max = rssi;
		h->snr_min = snr;
		h->snr_max = snr;
	} else {
		h->rssi_min = MIN(h->rssi_min, rssi);
		h->rssi_max = MAX(h->rssi_max, rssi);
		h->snr_min = MIN(h->snr_min, snr);
		h->snr_max = MAX(h->snr_max, snr);
	}

	h->rssi[meshtastic_rf_rssi_bin(rssi)]++;
	h->snr[meshtastic_rf_snr_bin(snr)]++;
	h->rssi_sum += rssi;
	h->snr_sum += snr;
	h->rssi_sumsq += (int64_t)rssi * (int64_t)rssi;
	h->snr_sumsq += (int64_t)snr * (int64_t)snr;
	h->frames++;
}

/*
 * Find or claim a slot for this peer. Linear scan with an MRU fast path, which
 * is the right shape here: frames arrive in runs from the same node, and the
 * table is small enough (24 by default) that a scan is cheaper than a hash.
 *
 * Eviction is least-recently-heard, and it is COUNTED. A fixed table that
 * silently drops peers is a lie about coverage — the count is what lets a
 * reader know the peer list is partial.
 */
static struct meshtastic_rf_peer *peer_slot(uint32_t num, uint32_t now)
{
	uint8_t oldest = 0U;
	uint32_t oldest_ms = UINT32_MAX;

	if (rf.peers[rf.mru].used && rf.peers[rf.mru].num == num) {
		return &rf.peers[rf.mru];
	}

	for (uint8_t i = 0; i < PEER_SLOTS; i++) {
		if (rf.peers[i].used && rf.peers[i].num == num) {
			rf.mru = i;
			return &rf.peers[i];
		}
		if (!rf.peers[i].used) {
			rf.mru = i;
			rf.peers[i].used = true;
			rf.peers[i].num = num;
			rf.peers[i].first_ms = now;
			return &rf.peers[i];
		}
		if (rf.peers[i].last_ms < oldest_ms) {
			oldest_ms = rf.peers[i].last_ms;
			oldest = i;
		}
	}

	rf.evicted++;
	memset(&rf.peers[oldest], 0, sizeof(rf.peers[oldest]));
	rf.peers[oldest].used = true;
	rf.peers[oldest].num = num;
	rf.peers[oldest].first_ms = now;
	rf.mru = oldest;
	return &rf.peers[oldest];
}

void meshtastic_rf_on_rx(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr)
{
	const struct meshtastic_wire_header *hdr;
	struct meshtastic_rf_peer *p;
	k_spinlock_key_t key;
	uint32_t now;
	bool direct;

	if (buf == NULL || len < MESHTASTIC_HDR_LEN) {
		return;
	}

	hdr = (const struct meshtastic_wire_header *)buf;
	now = k_uptime_get_32();

	/*
	 * One hop means hop_start still equals hop_limit — nothing has decremented
	 * it. That is the only case where hdr->src is the node whose transmitter we
	 * actually heard, which is the only case that says anything about our own
	 * receive chain.
	 */
	direct = (((hdr->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >>
		   MESHTASTIC_FLAGS_HOP_START_SHIFT) ==
		  (hdr->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK));

	key = k_spin_lock(&rf_lock);

	if (rf.started_ms == 0U) {
		rf.started_ms = now;
	}

	/* Preset mixing invalidates a comparison outright — spreading factor moves
	 * sensitivity by far more than any gain setting here. Sampled rather than
	 * hooked into the preset switch, so this module stays independent of it. */
	if (!rf.preset_seen) {
		rf.preset = (uint8_t)mt.modem_preset;
		rf.preset_seen = true;
	} else if (rf.preset != (uint8_t)mt.modem_preset) {
		rf.preset_mixed = true;
	}

	hist_add(&rf.hist, rssi, snr);
	rf.lifetime++;

	p = peer_slot(hdr->src, now);
	p->last_ms = now;
	if (direct) {
		if (p->frames_direct == 0U) {
			p->rssi_best = rssi;
			p->snr_best = snr;
		} else {
			p->rssi_best = MAX(p->rssi_best, rssi);
			p->snr_best = MAX(p->snr_best, snr);
		}
		p->frames_direct++;
		p->rssi_sum += rssi;
		p->snr_sum += snr;
	} else {
		p->frames_relayed++;
	}

	k_spin_unlock(&rf_lock, key);
}

void meshtastic_rf_window_get(struct meshtastic_rf_window *out)
{
	k_spinlock_key_t key;

	if (out == NULL) {
		return;
	}

	key = k_spin_lock(&rf_lock);
	out->hist = rf.hist;
	out->lifetime = rf.lifetime;
	out->evicted = rf.evicted;
	out->preset = rf.preset;
	out->preset_mixed = rf.preset_mixed;
	out->window_ms = (rf.started_ms == 0U) ? 0U : (k_uptime_get_32() - rf.started_ms);
	out->rail_frames = rf.hist.rssi[MESHTASTIC_RF_RSSI_RAIL_BIN];
	k_spin_unlock(&rf_lock, key);
}

int meshtastic_rf_peers_get(struct meshtastic_rf_peer *out, size_t max)
{
	k_spinlock_key_t key;
	size_t n = 0;

	if (out == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&rf_lock);
	for (uint8_t i = 0; i < PEER_SLOTS && n < max; i++) {
		if (rf.peers[i].used) {
			out[n++] = rf.peers[i];
		}
	}
	k_spin_unlock(&rf_lock, key);

	return (int)n;
}

void meshtastic_rf_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&rf_lock);

	memset(&rf.hist, 0, sizeof(rf.hist));
	memset(rf.peers, 0, sizeof(rf.peers));
	rf.started_ms = 0U;
	rf.evicted = 0U;
	rf.preset_seen = false;
	rf.preset_mixed = false;
	rf.mru = 0U;
	/* rf.lifetime deliberately survives — see the header. */

	k_spin_unlock(&rf_lock, key);
}
