/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Test-facing control surface for the simulated LoRa driver (drivers/lora_sim).
 *
 * A native_sim test drives the real Meshtastic stack against this fake radio:
 * transmitted frames are captured (lora_sim_take_tx) and frames are injected as
 * if received over the air (lora_sim_inject). The driver models per-frame airtime
 * (identical to the SX126x driver) and channel-busy, so contention / duty-cycle /
 * dedup behaviour is exercised under twister with no hardware.
 */
#ifndef MESHTASTIC_LORA_SIM_H_
#define MESHTASTIC_LORA_SIM_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A single captured transmission. */
struct lora_sim_frame {
	uint8_t  data[256];
	uint8_t  len;
	uint32_t air_ms; /**< modelled airtime of this frame (ms) */
	int64_t  t_ms;   /**< k_uptime_get() at TX */
};

/**
 * @brief Deliver a frame to the node as if received over the air, on whatever
 *        the node is currently tuned to.
 *
 * Radio-agnostic: the frame always reaches the receiver. Correct for the large
 * majority of tests, which model one channel and never retune. Use
 * @ref lora_sim_inject_on when the point of the test IS that two radios are on
 * different settings.
 *
 * Calls the stack's registered receive callback synchronously.
 * @return 0 on success, -EAGAIN if no receiver is armed, -EMSGSIZE if too long.
 */
int lora_sim_inject(const struct device *dev, const uint8_t *data, uint8_t len,
		    int16_t rssi, int8_t snr);

/**
 * @brief Deliver a frame transmitted with specific modem settings.
 *
 * Models the property that makes multi-preset work possible at all: LoRa
 * transmissions are only receivable by a radio tuned to the SAME frequency,
 * spreading factor and bandwidth. A node on LongFast (906.875 MHz, SF11) is not
 * "weakly hearing" a ShortTurbo node (926.750 MHz, SF7) — it is completely deaf
 * to it. Without this, a sim test cannot tell a working preset switch from one
 * that forgot to retune, which is precisely the silent failure mode.
 *
 * @param freq_hz Transmitter's centre frequency.
 * @param sf      Transmitter's spreading factor (enum lora_datarate).
 * @param bw      Transmitter's bandwidth (enum lora_signal_bandwidth).
 *
 * @return 0 if delivered, -ENOTCONN if the receiver is tuned elsewhere (the
 *         interesting negative result), -EAGAIN if no receiver is armed,
 *         -EMSGSIZE if too long.
 */
int lora_sim_inject_on(const struct device *dev, uint32_t freq_hz, uint8_t sf, uint8_t bw,
		       const uint8_t *data, uint8_t len, int16_t rssi, int8_t snr);

/**
 * @brief Read back what the node is currently tuned to.
 *
 * Lets a test assert the *whole* switch happened — a preset change that updates
 * SF/BW but forgets the frequency (or vice versa) is invisible on hardware until
 * a second node fails to hear it.
 *
 * @return 0 on success, -EAGAIN if lora_config() has not been called yet.
 */
int lora_sim_get_tuning(const struct device *dev, uint32_t *freq_hz, uint8_t *sf, uint8_t *bw);

/**
 * @brief Read back the drive level most recently programmed into the radio.
 *
 * This is what actually reached `lora_config()` -- i.e. the value after the
 * stack's region-limit resolution (meshtastic_tx_power_resolve) and FEM gain
 * conversion (meshtastic_tx_power_chip_drive) have both been applied. Lets a
 * test pin the *whole* tx_power pipeline against the driver, not just its
 * pieces in isolation.
 *
 * @return 0 on success, -EAGAIN if lora_config() has not been called yet.
 */
int lora_sim_get_tx_power(const struct device *dev, int8_t *tx_power_dbm);

/**
 * @brief Pop the next captured TX frame.
 * @return 0 on success; -ENOMSG (K_NO_WAIT) or -EAGAIN (timeout) if none.
 */
int lora_sim_take_tx(const struct device *dev, struct lora_sim_frame *out,
		     k_timeout_t timeout);

/** @brief Number of frames currently waiting in the TX capture queue. */
int lora_sim_tx_pending(const struct device *dev);

/**
 * @brief True if the stack currently has a receive callback armed.
 *
 * The mesh stack cancels RX while transmitting (half-duplex), so a test should
 * wait for this before injecting a frame the node must actually hear.
 */
bool lora_sim_rx_armed(const struct device *dev);

/** @brief Force the modelled channel busy for @p ms from now (LBT/contention). */
void lora_sim_set_busy(const struct device *dev, uint32_t ms);

/**
 * @brief Model the SX126x's latched PREAMBLE_DETECTED / HEADER_VALID flags.
 *
 * What meshtastic_radio_actively_receiving() reads on hardware. A test sets
 * them to stand in for "a frame is being demodulated"; they clear the way the
 * real flags do — on a re-arm, on an injected (completed) frame, on an
 * explicit clear from the stack, and on lora_sim_reset().
 */
void lora_sim_set_rx_activity(const struct device *dev, bool preamble, bool header);
/** @brief The flags as the stack sees them (false/false while not listening). */
int lora_sim_rx_activity(const struct device *dev, bool *preamble, bool *header);
/** @brief The stack retiring a detection it judged stale. */
int lora_sim_rx_activity_clear(const struct device *dev);

/** @brief Silently drop the next @p n transmissions (retransmit/reliability). */
void lora_sim_drop_next(const struct device *dev, unsigned int n);

/** @brief Clear the capture queue, drop counter and busy state (between tests). */
void lora_sim_reset(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_LORA_SIM_H_ */
