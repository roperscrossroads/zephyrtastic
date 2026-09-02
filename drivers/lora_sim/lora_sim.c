/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Simulated LoRa transceiver for native_sim.
 *
 * Registers a `struct lora_driver_api` device behind DT_ALIAS(lora0) so the real
 * Meshtastic stack (router, dedup, reliable retransmit, traceroute, airtime) runs
 * on the host with no radio. Transmitted frames are captured in a queue a test
 * drains (lora_sim_take_tx); tests inject frames as if received (lora_sim_inject).
 *
 * Only a local, in-process backend is implemented (Tier 1). The TX capture / RX
 * injection is deliberately the only chip-specific surface, so a future Tier 3
 * host-side RF-hub backend can replace it without touching the ops here.
 */

#define DT_DRV_COMPAT meshtastic_lora_sim

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <meshtastic/lora_sim.h>

LOG_MODULE_REGISTER(lora_sim, CONFIG_LORA_LOG_LEVEL);

struct lora_sim_data {
	struct lora_modem_config cfg;
	bool cfg_valid;

	lora_recv_cb rx_cb;
	void        *rx_user;

	int64_t      busy_until_ms; /* channel modelled busy until this uptime */
	unsigned int drop_next;     /* silently drop this many upcoming TX */

	/* The SX126x's latched PREAMBLE_DETECTED / HEADER_VALID flags, as a test
	 * sets them. Cleared by a re-arm (as the real driver's SetRx does), by an
	 * injected frame (RX_DONE clears them on hardware), by an explicit clear
	 * from the stack (a detection judged stale) and by lora_sim_reset(). */
	bool act_preamble;
	bool act_header;

	struct k_msgq txq;
	char          txq_buf[CONFIG_LORA_SIM_TX_QUEUE_DEPTH *
			      sizeof(struct lora_sim_frame)];

	struct k_mutex lock;
};

/* ---- airtime: MUST match zephyr/drivers/lora/native/sx126x/sx126x.c ---------
 * meshtastic_airtime.c delegates to lora_airtime() (the driver op), so any drift
 * here silently changes every duty-cycle and contention decision the stack makes.
 * The body below is lifted verbatim from sx126x_lora_airtime()/bandwidth_to_hz()/
 * should_enable_ldro(); the only omission is the SX126x hal `force_ldro` override,
 * which a simulated radio has no reason to set (== false).
 */

static int lora_sim_bandwidth_to_hz(enum lora_signal_bandwidth bw, uint32_t *bw_hz)
{
	switch (bw) {
	case BW_7_KHZ:   *bw_hz = 7810;   return 0;
	case BW_10_KHZ:  *bw_hz = 10420;  return 0;
	case BW_15_KHZ:  *bw_hz = 15630;  return 0;
	case BW_20_KHZ:  *bw_hz = 20830;  return 0;
	case BW_31_KHZ:  *bw_hz = 31250;  return 0;
	case BW_41_KHZ:  *bw_hz = 41670;  return 0;
	case BW_62_KHZ:  *bw_hz = 62500;  return 0;
	case BW_125_KHZ: *bw_hz = 125000; return 0;
	case BW_250_KHZ: *bw_hz = 250000; return 0;
	case BW_500_KHZ: *bw_hz = 500000; return 0;
	default:         return -EINVAL;
	}
}

static bool lora_sim_should_enable_ldro(enum lora_datarate sf, enum lora_signal_bandwidth bw)
{
	uint32_t bw_hz;

	if (lora_sim_bandwidth_to_hz(bw, &bw_hz) != 0) {
		return false;
	}
	/* Symbol time > 16.38 ms -> low-data-rate optimize on. */
	return (((1UL << sf) * 1000000UL) / bw_hz) > 16380;
}

static uint32_t lora_sim_airtime(const struct device *dev, uint32_t data_len)
{
	struct lora_sim_data *d = dev->data;
	uint32_t t_preamble_us, t_payload_us, t_sym_us, n_payload, bw_hz;
	uint8_t sf, cr;
	int32_t tmp;
	bool de, crc;

	if (!d->cfg_valid) {
		return 0;
	}
	if (lora_sim_bandwidth_to_hz(d->cfg.bandwidth, &bw_hz) != 0) {
		return 0;
	}
	sf = d->cfg.datarate;

	/* Symbol time (us) = 2^SF * 1e6 / BW */
	t_sym_us = ((1UL << sf) * 1000000UL) / bw_hz;

	/* Preamble time (4.25 extra symbols) */
	t_preamble_us = (d->cfg.preamble_len + 4) * t_sym_us + (t_sym_us / 4);

	de  = lora_sim_should_enable_ldro(sf, d->cfg.bandwidth);
	crc = !d->cfg.packet_crc_disable;
	cr  = d->cfg.coding_rate;

	tmp = 8 * data_len - 4 * sf + 28 + 16 * crc;
	if (tmp < 0) {
		tmp = 0;
	}
	n_payload = 8 + (((tmp + 4 * (sf - 2 * de) - 1) /
			  (4 * (sf - 2 * de))) * (cr + 4));
	t_payload_us = n_payload * t_sym_us;

	return (t_preamble_us + t_payload_us + 500) / 1000;
}

/* ---- driver ops ----------------------------------------------------------- */

static int lora_sim_config(const struct device *dev, const struct lora_modem_config *cfg)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->cfg = *cfg;
	d->cfg_valid = true;
	k_mutex_unlock(&d->lock);
	return 0;
}

static int lora_sim_send(const struct device *dev, uint8_t *data, uint32_t data_len)
{
	struct lora_sim_data *d = dev->data;
	struct lora_sim_frame frame;
	uint32_t air_ms = lora_sim_airtime(dev, data_len);
	bool drop;

	if (data_len > sizeof(frame.data)) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&d->lock, K_FOREVER);
	/* Listen-Before-Talk: refuse if the modelled channel is busy, matching
	 * lora_send()'s contract that meshtastic_radio.c relies on when it sets
	 * cad.mode = LORA_CAD_MODE_LBT. */
	if (d->cfg.cad.mode == LORA_CAD_MODE_LBT && k_uptime_get() < d->busy_until_ms) {
		k_mutex_unlock(&d->lock);
		return -EBUSY;
	}

	drop = (d->drop_next > 0U);
	if (drop) {
		d->drop_next--;
	} else {
		frame.len    = (uint8_t)data_len;
		frame.air_ms = air_ms;
		frame.t_ms   = k_uptime_get();
		memcpy(frame.data, data, data_len);
		if (k_msgq_put(&d->txq, &frame, K_NO_WAIT) != 0) {
			LOG_WRN("TX capture queue full, frame dropped");
		}
	}
	/* Model the channel occupied for this frame's airtime regardless of drop. */
	d->busy_until_ms = k_uptime_get() + air_ms;
	k_mutex_unlock(&d->lock);

	/* lora_send() blocks until TX completes. Sleeping the modelled airtime is
	 * what makes contention tests meaningful; native_sim's virtual clock makes
	 * it free in wall-clock terms. */
	k_msleep(air_ms);
	return 0;
}

static int lora_sim_send_async(const struct device *dev, uint8_t *data, uint32_t data_len,
			       struct k_poll_signal *async)
{
	int ret = lora_sim_send(dev, data, data_len);

	if (async != NULL) {
		k_poll_signal_raise(async, ret);
	}
	return ret;
}

static int lora_sim_recv(const struct device *dev, uint8_t *data, uint8_t size,
			 k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	/* Synchronous receive is unused by the mesh stack (it uses recv_async). */
	return -ENOSYS;
}

static int lora_sim_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->rx_cb   = cb; /* cb == NULL cancels, per the API */
	d->rx_user = user_data;
	if (cb != NULL) {
		/* A fresh receive starts with no activity on record (sx126x_set_rx). */
		d->act_preamble = false;
		d->act_header = false;
	}
	k_mutex_unlock(&d->lock);
	return 0;
}

static int lora_sim_cad(const struct device *dev, k_timeout_t timeout)
{
	struct lora_sim_data *d = dev->data;
	int detected;

	ARG_UNUSED(timeout);
	k_mutex_lock(&d->lock, K_FOREVER);
	detected = (k_uptime_get() < d->busy_until_ms) ? 1 : 0;
	k_mutex_unlock(&d->lock);
	return detected; /* 1 = activity detected, 0 = clear */
}

/* ---- test control surface (drivers/../include/meshtastic/lora_sim.h) ------- */

/* Shared body. @p match_tuning selects whether the transmitter's settings have to
 * agree with what this receiver is tuned to.
 *
 * The check is exact equality on all three of frequency, SF and bandwidth. Real
 * LoRa is not quite that binary — an adjacent-channel signal is attenuated, not
 * annihilated — but for the question these tests ask (did the switch retune
 * everything, or only some of it?) a hard gate is the right model, and a partial
 * switch shows up as a clean -ENOTCONN rather than a flaky near-miss. */
static int lora_sim_inject_common(const struct device *dev, bool match_tuning, uint32_t freq_hz,
				  uint8_t sf, uint8_t bw, const uint8_t *data, uint8_t len,
				  int16_t rssi, int8_t snr)
{
	struct lora_sim_data *d = dev->data;
	lora_recv_cb cb;
	void *user;
	uint8_t frame[256];
	bool tuned_here = true;

	if (len > sizeof(frame)) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&d->lock, K_FOREVER);
	cb   = d->rx_cb;
	user = d->rx_user;
	if (match_tuning) {
		/* An unconfigured radio hears nothing: lora_config() has not run, so
		 * there is no frequency to be on. */
		tuned_here = d->cfg_valid && d->cfg.frequency == freq_hz &&
			     (uint8_t)d->cfg.datarate == sf &&
			     (uint8_t)d->cfg.bandwidth == bw;
	}
	k_mutex_unlock(&d->lock);

	if (cb == NULL) {
		return -EAGAIN; /* nobody listening */
	}

	if (!tuned_here) {
		return -ENOTCONN; /* on the air, but not on OUR air */
	}

	/* Hand the callback a private, mutable copy, exactly as the real driver
	 * hands over its RX buffer. Do not hold the lock across the callback: the
	 * stack may synchronously turn around and transmit a relay/ACK. */
	memcpy(frame, data, len);
	cb(dev, frame, len, rssi, snr, user);

	/* RX_DONE: the driver's IRQ handler clears every flag it read, the
	 * preamble/header pair included. A completed frame ends the detection. */
	k_mutex_lock(&d->lock, K_FOREVER);
	d->act_preamble = false;
	d->act_header = false;
	k_mutex_unlock(&d->lock);
	return 0;
}

void lora_sim_set_rx_activity(const struct device *dev, bool preamble, bool header)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->act_preamble = preamble;
	d->act_header = header;
	k_mutex_unlock(&d->lock);
}

int lora_sim_rx_activity(const struct device *dev, bool *preamble, bool *header)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	/* Only meaningful while listening, as on the SX126x (sx126x_rx_activity
	 * answers "nothing" in every state but RX). */
	*preamble = (d->rx_cb != NULL) && d->act_preamble;
	*header = (d->rx_cb != NULL) && d->act_header;
	k_mutex_unlock(&d->lock);
	return 0;
}

int lora_sim_rx_activity_clear(const struct device *dev)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->act_preamble = false;
	d->act_header = false;
	k_mutex_unlock(&d->lock);
	return 0;
}

int lora_sim_inject(const struct device *dev, const uint8_t *data, uint8_t len,
		    int16_t rssi, int8_t snr)
{
	return lora_sim_inject_common(dev, false, 0U, 0U, 0U, data, len, rssi, snr);
}

int lora_sim_inject_on(const struct device *dev, uint32_t freq_hz, uint8_t sf, uint8_t bw,
		       const uint8_t *data, uint8_t len, int16_t rssi, int8_t snr)
{
	return lora_sim_inject_common(dev, true, freq_hz, sf, bw, data, len, rssi, snr);
}

int lora_sim_get_tuning(const struct device *dev, uint32_t *freq_hz, uint8_t *sf, uint8_t *bw)
{
	struct lora_sim_data *d = dev->data;
	int ret = 0;

	k_mutex_lock(&d->lock, K_FOREVER);
	if (!d->cfg_valid) {
		ret = -EAGAIN;
	} else {
		if (freq_hz != NULL) {
			*freq_hz = d->cfg.frequency;
		}
		if (sf != NULL) {
			*sf = (uint8_t)d->cfg.datarate;
		}
		if (bw != NULL) {
			*bw = (uint8_t)d->cfg.bandwidth;
		}
	}
	k_mutex_unlock(&d->lock);

	return ret;
}

int lora_sim_get_tx_power(const struct device *dev, int8_t *tx_power_dbm)
{
	struct lora_sim_data *d = dev->data;
	int ret = 0;

	k_mutex_lock(&d->lock, K_FOREVER);
	if (!d->cfg_valid) {
		ret = -EAGAIN;
	} else if (tx_power_dbm != NULL) {
		*tx_power_dbm = d->cfg.tx_power;
	}
	k_mutex_unlock(&d->lock);

	return ret;
}

int lora_sim_take_tx(const struct device *dev, struct lora_sim_frame *out, k_timeout_t timeout)
{
	struct lora_sim_data *d = dev->data;

	return k_msgq_get(&d->txq, out, timeout);
}

int lora_sim_tx_pending(const struct device *dev)
{
	struct lora_sim_data *d = dev->data;

	return (int)k_msgq_num_used_get(&d->txq);
}

bool lora_sim_rx_armed(const struct device *dev)
{
	struct lora_sim_data *d = dev->data;
	bool armed;

	k_mutex_lock(&d->lock, K_FOREVER);
	armed = (d->rx_cb != NULL);
	k_mutex_unlock(&d->lock);
	return armed;
}

void lora_sim_set_busy(const struct device *dev, uint32_t ms)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->busy_until_ms = k_uptime_get() + ms;
	k_mutex_unlock(&d->lock);
}

void lora_sim_drop_next(const struct device *dev, unsigned int n)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->drop_next = n;
	k_mutex_unlock(&d->lock);
}

void lora_sim_reset(const struct device *dev)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	k_msgq_purge(&d->txq);
	d->drop_next     = 0U;
	d->busy_until_ms = 0;
	d->act_preamble  = false;
	d->act_header    = false;
	k_mutex_unlock(&d->lock);
}

/* ---- init / instantiation ------------------------------------------------- */

static int lora_sim_init(const struct device *dev)
{
	struct lora_sim_data *d = dev->data;

	k_mutex_init(&d->lock);
	k_msgq_init(&d->txq, d->txq_buf, sizeof(struct lora_sim_frame),
		    CONFIG_LORA_SIM_TX_QUEUE_DEPTH);
	return 0;
}

static DEVICE_API(lora, lora_sim_api) = {
	.config     = lora_sim_config,
	.airtime    = lora_sim_airtime,
	.send       = lora_sim_send,
	.send_async = lora_sim_send_async,
	.recv       = lora_sim_recv,
	.recv_async = lora_sim_recv_async,
	.cad        = lora_sim_cad,
};

#define LORA_SIM_DEFINE(inst)                                                  \
	static struct lora_sim_data lora_sim_data_##inst;                      \
	DEVICE_DT_INST_DEFINE(inst, lora_sim_init, NULL,                       \
			      &lora_sim_data_##inst, NULL,                     \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,          \
			      &lora_sim_api);

DT_INST_FOREACH_STATUS_OKAY(LORA_SIM_DEFINE)
