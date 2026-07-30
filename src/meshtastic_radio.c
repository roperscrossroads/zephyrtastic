/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * LoRa radio access, RX handoff, and TX/RX state serialization.
 */

#include <string.h>

#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/fem.h>

#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_router.h"
#include "meshtastic_airtime.h"
#include "meshtastic_powermon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Bandwidth in Hz -> the driver's kHz-labelled code. The labels are rounded
 * (BW_62_KHZ is 62.5 kHz, BW_400_KHZ is 406.25, BW_800_KHZ is 812.5,
 * BW_1600_KHZ is 1625), so this must be a table rather than a division.
 */
int meshtastic_radio_bw_hz_to_code(uint32_t bandwidth_hz)
{
	switch (bandwidth_hz) {
	case 15600U:
		return BW_15_KHZ;
	case 62500U:
		return BW_62_KHZ;
	case 125000U:
		return BW_125_KHZ;
	case 250000U:
		return BW_250_KHZ;
	case 406250U:
		return BW_400_KHZ;
	case 500000U:
		return BW_500_KHZ;
	case 812500U:
		return BW_800_KHZ;
	case 1625000U:
		return BW_1600_KHZ;
	default:
		return -EINVAL;
	}
}

/* The reference carries coding rate as 5..8 meaning 4/5..4/8; the driver's
 * enum is CR_4_5=1 .. CR_4_8=4. An off-by-four here misconfigures the radio
 * without any error, so it is a named, tested conversion rather than an
 * inline subtraction.
 */
int meshtastic_radio_cr_to_code(uint8_t coding_rate)
{
	if (coding_rate < 5U || coding_rate > 8U) {
		return -EINVAL;
	}

	return (int)CR_4_5 + (int)(coding_rate - 5U);
}

/* Initialised to LongFast, the value mt.modem also defaults to. Overwritten
 * from mt.modem before every lora_config().
 */
static struct lora_modem_config mt_lora_cfg = {
	.bandwidth = BW_250_KHZ,
	.datarate = SF_11,
	.coding_rate = CR_4_5,
	.preamble_len = 16U,
	.iq_inverted = false,
	.public_network = false,
	.sync_word = 0x2b,
	.cad =
		{
			.mode = LORA_CAD_MODE_NONE,
			.symbol_num = 0,
		},
};

/* Push the resolved modem params into the driver config. Called under mt.lock
 * at both lora_config() sites.
 *
 * A value the driver cannot represent keeps whatever was configured before
 * rather than failing the operation: the config validator rejects illegal
 * combinations long before here, so reaching this path means a bug, and going
 * off the air is a worse response to it than staying on the previous config.
 * It is logged loudly either way.
 */
static void apply_modem_params(void)
{
	int bw = meshtastic_radio_bw_hz_to_code(mt.modem.bandwidth_hz);
	int cr = meshtastic_radio_cr_to_code(mt.modem.coding_rate);

	if (bw >= 0) {
		mt_lora_cfg.bandwidth = (enum lora_signal_bandwidth)bw;
	} else {
		LOG_WRN("bandwidth %u Hz has no driver code; keeping %d kHz",
			mt.modem.bandwidth_hz, (int)mt_lora_cfg.bandwidth);
	}

	if (cr >= 0) {
		mt_lora_cfg.coding_rate = (enum lora_coding_rate)cr;
	} else {
		LOG_WRN("coding rate 4/%u out of range; keeping 4/%d",
			mt.modem.coding_rate, (int)mt_lora_cfg.coding_rate + 4);
	}

	if (mt.modem.spread_factor >= (uint8_t)SF_5 &&
	    mt.modem.spread_factor <= (uint8_t)SF_12) {
		mt_lora_cfg.datarate = (enum lora_datarate)mt.modem.spread_factor;
	} else {
		LOG_WRN("spread factor %u out of range; keeping SF%d",
			mt.modem.spread_factor, (int)mt_lora_cfg.datarate);
	}
}

static K_THREAD_STACK_DEFINE(mt_stack, CONFIG_MESHTASTIC_THREAD_STACK_SIZE);
static struct k_thread mt_thread;

/*
 * Serialises radio state transitions.  Continuous async RX runs in the LoRa
 * driver; only TX and the surrounding stop/re-arm of async RX touch radio
 * state and must not race each other (the SX126x driver rejects lora_send()
 * /lora_config() with -EBUSY while async RX is active).
 */
static K_SEM_DEFINE(mt_radio_sem, 1, 1);

/* Raw frame handed from the driver RX callback to the processing thread. */
struct mt_rx_slot {
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t buf[MESHTASTIC_PKT_MAX];
};

K_MSGQ_DEFINE(mt_rx_msgq, sizeof(struct mt_rx_slot), CONFIG_MESHTASTIC_RX_QUEUE_DEPTH, 4);

static void mt_rx_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi,
		     int8_t snr, void *user_data);

#if defined(CONFIG_MESHTASTIC_PACKET_HEXDUMP)
static void log_wire_tx(const uint8_t *pkt, uint32_t pkt_len)
{
	const struct meshtastic_wire_header *hdr = (const struct meshtastic_wire_header *)pkt;

	LOG_DBG("LoRa TX %08x->%08x id=%08x ch=0x%02x len=%u",
		(unsigned int)sys_le32_to_cpu(hdr->src), (unsigned int)sys_le32_to_cpu(hdr->dest),
		(unsigned int)sys_le32_to_cpu(hdr->id), hdr->channel, (unsigned int)pkt_len);
	/* Deep log-stack formatter (~256 B) — gated + off by default; see the
	 * stack-overflow warning on CONFIG_MESHTASTIC_PACKET_HEXDUMP. */
	LOG_HEXDUMP_DBG(pkt, pkt_len, "LoRa TX");
}
#endif /* CONFIG_MESHTASTIC_PACKET_HEXDUMP */

static int mt_radio_arm_rx(void)
{
	int ret = lora_recv_async(mt.lora_dev, mt_rx_cb, NULL);

	if (ret < 0) {
		mt.radio_rx_armed = false;
		mt.status.rx_rearm_failures++;
		meshtastic_powermon_clear(MESHTASTIC_PM_LORA_RX);
		LOG_ERR("lora_recv_async arm failed (%d)", ret);
	} else {
		mt.radio_rx_armed = true;
		meshtastic_powermon_set(MESHTASTIC_PM_LORA_RX);
	}

	return ret;
}

static uint32_t mt_busy_backoff_ms(void)
{
	uint32_t min_ms = CONFIG_MESHTASTIC_TX_BUSY_BACKOFF_MIN_MS;
	uint32_t max_ms = CONFIG_MESHTASTIC_TX_BUSY_BACKOFF_MAX_MS;

	if (max_ms <= min_ms) {
		return min_ms;
	}

	return min_ms + (sys_rand32_get() % (max_ms - min_ms + 1U));
}

/*
 * Default RF front-end hook: no-op. Boards with an external PA/LNA front-end
 * whose mode pin must follow TX/RX (e.g. Heltec V4) override this with a strong
 * definition. See <zephyr/meshtastic/fem.h>.
 */
__weak void meshtastic_radio_fem_set_tx(bool tx)
{
	ARG_UNUSED(tx);
}

int meshtastic_radio_send_wire_now(uint8_t *pkt, uint32_t pkt_len)
{
	int ret;
	int retries;
	int busy_retries = 0;

#if defined(CONFIG_MESHTASTIC_PACKET_HEXDUMP)
	log_wire_tx(pkt, pkt_len);
#endif

	(void)k_sem_take(&mt_radio_sem, K_FOREVER);

	/*
	 * Continuous async RX must be stopped first: the SX126x driver
	 * rejects lora_config()/lora_send() with -EBUSY while it is active.
	 */
	(void)lora_recv_async(mt.lora_dev, NULL, NULL);
	mt.radio_rx_armed = false;
	meshtastic_powermon_clear(MESHTASTIC_PM_LORA_RX);
	meshtastic_powermon_set(MESHTASTIC_PM_LORA_TX);

	k_mutex_lock(&mt.lock, K_FOREVER);

	mt_lora_cfg.frequency = mt.frequency;
	apply_modem_params();
	mt_lora_cfg.tx_power = mt.tx_power;
	mt_lora_cfg.tx = true;
	mt_lora_cfg.cad.mode = LORA_CAD_MODE_LBT;
	mt_lora_cfg.cad.symbol_num = LORA_CAD_SYMB_2;

	/* Steer any RF front-end to its TX path before keying the transmitter. */
	meshtastic_radio_fem_set_tx(true);

	ret = lora_config(mt.lora_dev, &mt_lora_cfg);
	if (ret == 0) {
		retries = CONFIG_MESHTASTIC_TX_BUSY_RETRIES;
		for (;;) {
			ret = lora_send(mt.lora_dev, pkt, pkt_len);
			if (ret != -EBUSY || retries == 0) {
				break;
			}

			retries--;
			busy_retries++;
			k_sleep(K_MSEC(mt_busy_backoff_ms()));
		}
	}

	mt_lora_cfg.tx = false;
	mt_lora_cfg.cad.mode = LORA_CAD_MODE_NONE;
	(void)lora_config(mt.lora_dev, &mt_lora_cfg);

	/* Return the front-end to its RX path (e.g. re-enable the LNA). */
	meshtastic_radio_fem_set_tx(false);

	k_mutex_unlock(&mt.lock);

	(void)mt_radio_arm_rx();
	meshtastic_powermon_clear(MESHTASTIC_PM_LORA_TX);

	(void)k_sem_give(&mt_radio_sem);

	if (busy_retries > 0) {
		LOG_DBG("TX deferred by CAD busy channel (%d retries)", busy_retries);
	}

	if (ret < 0) {
		if (ret == -EBUSY) {
			LOG_DBG("TX failed: channel busy after retries exhausted");
		}
		mt.status.tx_failures++;
		meshtastic_emit_event(MESHTASTIC_EVENT_TX_FAILED, ret, NULL);
	} else {
		mt.status.tx_packets++;
#if defined(CONFIG_MESHTASTIC_AIRTIME)
		meshtastic_airtime_log(MESHTASTIC_AIRTIME_TX,
				       meshtastic_airtime_packet_ms(pkt_len));
#endif
	}

	return ret;
}

/*
 * LoRa driver receive callback.  Runs on the driver's system workqueue (not
 * an ISR), and the driver auto-restarts continuous RX as soon as this returns
 * - so do the minimum: copy the frame out (the driver reuses its RX buffer
 * immediately) and hand it to the processing thread.
 */
static void mt_rx_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi,
		     int8_t snr, void *user_data)
{
	struct mt_rx_slot slot;

	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (size == 0U || size > sizeof(slot.buf)) {
		return;
	}

	slot.len = size;
	slot.rssi = rssi;
	slot.snr = snr;
	memcpy(slot.buf, data, size);

	if (k_msgq_put(&mt_rx_msgq, &slot, K_NO_WAIT) != 0) {
		mt.status.rx_dropped++;
		LOG_DBG("RX queue full, dropped %u-byte frame", size);
	}
}

static void mt_thread_fn(void *p1, void *p2, void *p3)
{
	struct mt_rx_slot slot;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		/*
		 * Block long while RX is armed so the SoC can idle between
		 * frames — the wait is a liveness backstop, not a poll, and a
		 * frame arrival wakes this immediately via mt_rx_msgq.  Drop to
		 * the fast retry cadence only while the radio is un-armed, so a
		 * failed post-TX re-arm is still recovered promptly.  (A pure
		 * K_FOREVER would be wrong here: a deaf radio receives nothing,
		 * so nothing would wake the thread to notice the failed re-arm.)
		 */
		k_timeout_t wait = mt.radio_rx_armed
					   ? K_MSEC(CONFIG_MESHTASTIC_RX_IDLE_RECHECK_MS)
					   : K_MSEC(CONFIG_MESHTASTIC_RX_REARM_RETRY_MS);

		ret = k_msgq_get(&mt_rx_msgq, &slot, wait);
		if (ret == 0) {
			meshtastic_router_process_lora_rx(slot.buf, slot.len, slot.rssi, slot.snr);
			continue;
		}

		/*
		 * No packet within the retry window.  If a TX left async RX
		 * un-armed (re-arm failed), the radio is deaf - recover it.
		 */
		if (!mt.radio_rx_armed) {
			(void)k_sem_take(&mt_radio_sem, K_FOREVER);
			if (!mt.radio_rx_armed) {
				(void)mt_radio_arm_rx();
			}
			(void)k_sem_give(&mt_radio_sem);
		}
	}
}

#if defined(CONFIG_LORA_SX126X)
#include <zephyr/drivers/gpio.h>

/*
 * DIO1 light-sleep wake support (ESP32-S3). The board DT arms EXT1 on the DIO1 pin
 * via GPIO_INT_WAKEUP on its dio1-gpios cell, so a frame wakes the SoC from
 * PM_STATE_STANDBY. Two app-side pieces make that correct and useful; see
 * docs/light-sleep-governor.md and the board-DT dio1 note.
 */
static const struct gpio_dt_spec mt_dio1 = GPIO_DT_SPEC_GET(DT_NODELABEL(lora0), dio1_gpios);

/* Carried in zephyr/patches/0002-*: submit the driver's DIO1 work if DIO1 is asserted.
 * Takes the radio device because the native sx126x driver builds for two compatibles. */
extern void sx126x_poll_dio1(const struct device *dev);

/*
 * Disarm the DIO1 EXT1 wake before deep sleep. GPIO_INT_WAKEUP arms EXT1 whenever
 * CONFIG_PM || CONFIG_POWEROFF, so without this an incoming frame would also wake a
 * node the user has admin-shut-down; reconfiguring DIO1 as plain input (no WAKEUP)
 * clears the EXT1 arming (the esp32 gpio driver's wakeup_disable path) so shutdown
 * wakes only on reset — as canShutdown advertises. Called from the shutdown path
 * (meshtastic_admin.c) just before sys_poweroff(). Safe on non-esp targets (a plain
 * input reconfigure). Compiled whenever the SX126x radio is present, independent of
 * CONFIG_PM, because CONFIG_POWEROFF alone is enough to arm the wake.
 */
void meshtastic_radio_disarm_dio1_wake(void)
{
	(void)gpio_pin_configure_dt(&mt_dio1, GPIO_INPUT);
}

#if defined(CONFIG_PM)
#include <zephyr/pm/pm.h>

/*
 * On every wake from light sleep, poll DIO1. A frame that arrived while asleep left
 * DIO1 latched HIGH but produced no edge for the (powered-down) GPIO peripheral to
 * catch, so nothing else will read it. Runs in the PM-exit context (IRQs locked): it
 * does only a register read + k_work_submit (both safe there) — the SPI read happens
 * later in the driver's work handler on the system workqueue.
 */
static void mt_radio_pm_exit(enum pm_state state)
{
	if (state == PM_STATE_STANDBY) {
		sx126x_poll_dio1(mt.lora_dev);
	}
}

static struct pm_notifier mt_radio_pm_note = {
	.state_exit = mt_radio_pm_exit,
};

/*
 * Strong override of the sx126x driver's __weak sx126x_hal_busy_timeout_report()
 * (agents-qnpp). Enriches a BUSY timeout with the light-sleep correlation — the
 * wake sequence number and ms since the last PM_STATE_STANDBY exit — so timeouts
 * can be placed relative to wakes with NO wall-clock time (the node's clock may be
 * unset; log timestamps read 1970). If timeouts cluster in the first few ms after
 * a wake, the BUSY timeout is a light-sleep race, not a chip/SPI fault. PM-only:
 * without CONFIG_PM the driver's weak default (opcode only) applies.
 */
void sx126x_hal_busy_timeout_report(const struct device *dev, uint8_t opcode, uint32_t timeout_ms,
				    bool post, uint8_t prev_opcode)
{
	ARG_UNUSED(dev);
	LOG_WRN("Busy timeout after %u ms (op=0x%02x %s, prev=0x%02x) wake=#%u +%ums", timeout_ms,
		opcode, post ? "post" : "pre", prev_opcode, meshtastic_powermon_sleep_count(),
		meshtastic_powermon_ms_since_wake());
}
#endif /* CONFIG_PM */
#endif /* CONFIG_LORA_SX126X */

int meshtastic_radio_init(void)
{
	int ret;

	mt_lora_cfg.frequency = mt.frequency;
	apply_modem_params();
	mt_lora_cfg.tx_power = mt.tx_power;
	mt_lora_cfg.tx = false;

	ret = lora_config(mt.lora_dev, &mt_lora_cfg);
	if (ret < 0) {
		LOG_ERR("Initial lora_config failed (%d)", ret);
		return -EIO;
	}

	ret = meshtastic_outbound_init();
	if (ret < 0) {
		return ret;
	}

	k_thread_create(&mt_thread, mt_stack, K_THREAD_STACK_SIZEOF(mt_stack), mt_thread_fn, NULL,
			NULL, NULL, CONFIG_MESHTASTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mt_thread, "meshtastic");

#if defined(CONFIG_PM) && defined(CONFIG_LORA_SX126X)
	/* Read a frame that arrived during light sleep as soon as the SoC wakes (the
	 * DIO1 edge is missed while the GPIO peripheral is powered down). */
	pm_notifier_register(&mt_radio_pm_note);
#endif

	/*
	 * Arm continuous async RX.  A failure here is non-fatal: TX still
	 * works and the processing thread re-attempts the arm periodically.
	 */
	(void)k_sem_take(&mt_radio_sem, K_FOREVER);
	(void)mt_radio_arm_rx();
	(void)k_sem_give(&mt_radio_sem);

	return 0;
}
