/* SPDX-License-Identifier: GPL-3.0
 *
 * Signal-quality aggregation: RSSI/SNR distributions and per-peer frame rates.
 *
 * The gap this fills is narrow and specific. The firmware already counts plenty
 * — CAD outcomes, AGC resets, TX failures — and the NodeDB keeps each peer's
 * LAST SNR. What nothing does is accumulate: no distribution, no frame count per
 * peer, no window. A single last-SNR reading cannot answer "did that antenna
 * help?", because one sample of a noisy quantity is not evidence.
 *
 * See main/docs/rf-measurement.md for why the bins are the widths they are.
 */
#ifndef MESHTASTIC_RF_MEASURE_H_
#define MESHTASTIC_RF_MEASURE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Bin layout. Both scales carry an explicit out-of-range bin at each end rather
 * than clamping into the neighbouring one, because "off the scale" and "at the
 * edge of the scale" mean different things and only one of them invalidates a
 * comparison.
 */
#define MESHTASTIC_RF_RSSI_BINS 28 /* [0] < -130 | [1..26] -130..-1 by 5 dB | [27] >= 0 rail */
#define MESHTASTIC_RF_SNR_BINS  20 /* [0] < -22  | [1..18] -22..+13 by 2 dB | [19] >= +14  */

#define MESHTASTIC_RF_RSSI_FLOOR (-130)
#define MESHTASTIC_RF_RSSI_STEP  5
#define MESHTASTIC_RF_SNR_FLOOR  (-22)
#define MESHTASTIC_RF_SNR_STEP   2

/** @brief Index of the RSSI rail bin — signals at or above 0 dBm. */
#define MESHTASTIC_RF_RSSI_RAIL_BIN (MESHTASTIC_RF_RSSI_BINS - 1)

/**
 * @brief One accumulated distribution.
 *
 * Sums and sums-of-squares are kept alongside the bins so mean and standard
 * deviation need no second pass over anything — the bins are for showing a
 * shape, the moments are for comparing two of them.
 */
struct meshtastic_rf_hist {
	uint32_t rssi[MESHTASTIC_RF_RSSI_BINS];
	uint32_t snr[MESHTASTIC_RF_SNR_BINS];
	int64_t rssi_sumsq;
	int64_t snr_sumsq;
	/* 64-bit, not 32. At ~-130 per frame an int32_t sum wraps after roughly
	 * 16.5 million frames — reachable in a long soak — and a wrapped sum
	 * yields a confidently wrong mean with nothing to indicate it. The extra
	 * 8 bytes per window buys an accumulator that cannot lie. */
	int64_t rssi_sum;
	int64_t snr_sum;
	uint32_t frames;
	int16_t rssi_min, rssi_max;
	int8_t snr_min, snr_max;
};

/** @brief What one peer's link has looked like this window. */
struct meshtastic_rf_peer {
	uint32_t num;
	/*
	 * Split deliberately. hdr->src is only OUR RF peer when the frame arrived
	 * in one hop; a relayed frame measures the RELAY's link to us, and its
	 * originator may be kilometres away. Averaging both together produces a
	 * per-peer number that moves when an unrelated third node moves, which is
	 * worse than useless in a comparison.
	 */
	uint32_t frames_direct;
	uint32_t frames_relayed;
	int64_t rssi_sum; /* direct frames only; 64-bit for the same reason as above */
	int64_t snr_sum;  /* direct frames only */
	uint32_t first_ms, last_ms;
	int16_t rssi_best;
	int8_t snr_best;
	bool used;
};

/** @brief Snapshot of the whole measurement window. */
struct meshtastic_rf_window {
	struct meshtastic_rf_hist hist;
	uint32_t window_ms;    /**< since the last reset */
	uint32_t lifetime;     /**< frames ever seen; survives a reset */
	uint32_t evicted;      /**< peers dropped from a full table */
	uint32_t rail_frames;  /**< convenience: hist.rssi[RAIL_BIN] */
	uint8_t preset;        /**< preset at the first frame of the window */
	bool preset_mixed;     /**< the preset changed mid-window: bins not comparable */
};

/**
 * @brief Record one received frame. Called from the radio RX callback.
 *
 * Must stay cheap and must not block: the driver reuses its receive buffer as
 * soon as the callback returns.
 */
void meshtastic_rf_on_rx(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr);

/** @brief Copy out the current window. */
void meshtastic_rf_window_get(struct meshtastic_rf_window *out);

/**
 * @brief Copy out up to @p max peers, most recently heard first.
 * @return the number written.
 */
int meshtastic_rf_peers_get(struct meshtastic_rf_peer *out, size_t max);

/**
 * @brief Start a new window.
 *
 * Clears the distribution and the peer table. Deliberately does NOT clear the
 * lifetime frame count — a reset must not destroy the ability to say how much
 * has ever been observed.
 */
void meshtastic_rf_reset(void);

/** @brief Bin index for an RSSI reading; exposed so tests can pin the edges. */
uint8_t meshtastic_rf_rssi_bin(int16_t rssi);
/** @brief Bin index for an SNR reading. */
uint8_t meshtastic_rf_snr_bin(int8_t snr);

#endif /* MESHTASTIC_RF_MEASURE_H_ */
