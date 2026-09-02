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
#include "meshtastic_preset.h"
#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_scanner.h"
#endif
#if defined(CONFIG_MESHTASTIC_RF_HIST)
#include "meshtastic_rf_measure.h"
#endif
#include "meshtastic_router.h"
#include "meshtastic_airtime.h"
#include "meshtastic_tx_power.h"
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

/*
 * What was last actually handed to lora_config(), for `meshtastic rf`.
 *
 * Recorded HERE, at the call sites, rather than derived from mt.* on demand,
 * because the whole point is to be able to say when the two disagree. A config
 * that failed leaves the chip on its previous settings while every stored value
 * already reads new; asking mt.* would cheerfully report the new one.
 *
 * Guarded by mt_effective_lock rather than mt.lock: the readers are the shell
 * and (later) the RF report, and making them contend for mt.lock — held across
 * lora_config() itself — would let a diagnostic stall the radio path.
 */
static struct meshtastic_radio_effective mt_effective;
static struct k_spinlock mt_effective_lock;

/*
 * @param tx_side true when this config programmed a transmit (so the drive
 *                level belongs in tx_power_tx_cfg, not the RX-side field).
 * @param rc      lora_config()'s return code, recorded verbatim.
 *
 * Reads mt_lora_cfg, so it must be called while the caller still holds mt.lock,
 * before anything can overwrite the config it describes.
 */
static void mt_record_effective(bool tx_side, int rc)
{
	k_spinlock_key_t key = k_spin_lock(&mt_effective_lock);

	mt_effective.frequency = mt_lora_cfg.frequency;
	mt_effective.bandwidth_hz = mt.modem.bandwidth_hz;
	mt_effective.spread_factor = mt.modem.spread_factor;
	mt_effective.coding_rate = mt.modem.coding_rate;
	if (tx_side) {
		mt_effective.tx_power_tx_cfg = (int8_t)mt_lora_cfg.tx_power;
	} else {
		mt_effective.tx_power_rx_cfg = (int8_t)mt_lora_cfg.tx_power;
	}
	mt_effective.last_rc = rc;
	if (rc == 0) {
		/* Only successful configs advance the generation, so a stalled
		 * counter alongside a growing uptime is itself the symptom. */
		mt_effective.generation++;
	}

	k_spin_unlock(&mt_effective_lock, key);
}

int meshtastic_radio_effective_get(struct meshtastic_radio_effective *out)
{
	k_spinlock_key_t key;

	if (out == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&mt_effective_lock);
	*out = mt_effective;
	k_spin_unlock(&mt_effective_lock, key);

	return 0;
}

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

/* Raw frame handed from the driver RX callback — or a non-LoRa bearer via
 * meshtastic_radio_rx_inject() — to the processing thread. */
struct mt_rx_slot {
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t bearer; /* enum meshtastic_bearer */
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

/*
 * Default transmit-power conversion: identity. Correct for any board with no
 * transmit gain between the transceiver and the antenna — the configured dBm
 * IS the drive level there. A board with a PA front-end overrides this.
 */
__weak int8_t meshtastic_radio_fem_tx_power_conversion(int8_t radiated_dbm)
{
	return radiated_dbm;
}

/*
 * Default FEM LNA control: absent. A board whose front-end can switch its
 * receive path between the LNA and a bypass overrides both of these.
 */
__weak bool meshtastic_radio_fem_lna_can_control(void)
{
	return false;
}

__weak void meshtastic_radio_fem_lna_set(bool enable)
{
	ARG_UNUSED(enable);
}

/*
 * Default receive-path intent: bypass. Only meaningful where the board reports
 * meshtastic_radio_fem_lna_can_control(), so a diagnostic must gate on that
 * rather than treat this false as "the LNA is off" — on hardware with no
 * controllable path there is no choice being made at all.
 */
__weak bool meshtastic_radio_fem_lna_get(void)
{
	return false;
}

/* Default: no front-end fitted. A board that detects one at runtime returns what
 * it actually found. */
__weak const char *meshtastic_radio_fem_name(void)
{
	return NULL;
}

/* Default: no front-end, which is a settled answer rather than a missing one —
 * the weak identity power conversion above is exactly right for such a board.
 * A board that detects at runtime overrides this so a FAILED detection cannot
 * hide behind the same NULL name as legitimately absent hardware. */
__weak enum meshtastic_fem_state meshtastic_radio_fem_state(void)
{
	return MESHTASTIC_FEM_STATE_NONE;
}

int meshtastic_radio_tune_explicit(uint32_t frequency_hz, uint8_t spread_factor,
				   uint32_t bandwidth_hz, uint8_t coding_rate)
{
	if (meshtastic_radio_bw_hz_to_code(bandwidth_hz) < 0 ||
	    meshtastic_radio_cr_to_code(coding_rate) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&mt.lock, K_FOREVER);
	mt.frequency = frequency_hz;
	mt.modem.spread_factor = spread_factor;
	mt.modem.bandwidth_hz = bandwidth_hz;
	mt.modem.coding_rate = coding_rate;
	k_mutex_unlock(&mt.lock);

	return meshtastic_radio_retune();
}

int meshtastic_radio_retune(void)
{
	int ret;

	/* Same sequence, and the same lock order (radio sem then mt.lock), as the
	 * TX path below — the SX126x driver rejects lora_config() while continuous
	 * async RX is running, so RX must be torn down first and re-armed after. */
	(void)k_sem_take(&mt_radio_sem, K_FOREVER);

	(void)lora_recv_async(mt.lora_dev, NULL, NULL);
	mt.radio_rx_armed = false;
	meshtastic_powermon_clear(MESHTASTIC_PM_LORA_RX);

	k_mutex_lock(&mt.lock, K_FOREVER);
	mt_lora_cfg.frequency = mt.frequency;
	apply_modem_params();
	mt_lora_cfg.tx_power = meshtastic_tx_power_chip_drive(mt.tx_power, mt.licensed);
	mt_lora_cfg.tx = false;
	mt_lora_cfg.cad.mode = LORA_CAD_MODE_NONE;
	mt_lora_cfg.cad.symbol_num = 0;
	ret = lora_config(mt.lora_dev, &mt_lora_cfg);
	mt_record_effective(false, ret);
	k_mutex_unlock(&mt.lock);

	if (ret < 0) {
		LOG_ERR("retune: lora_config failed (%d)", ret);
	}

	/* Re-arm regardless: leaving RX down would make the node deaf until the
	 * processing thread's periodic re-arm notices, and a failed config is more
	 * recoverable with a listening radio than a silent one. */
	(void)mt_radio_arm_rx();
	(void)k_sem_give(&mt_radio_sem);

	return ret;
}

int meshtastic_radio_send_wire_now(uint8_t *pkt, uint32_t pkt_len)
{
	uint32_t settle;
	int ret;
	int retries;
	int busy_retries = 0;

#if defined(CONFIG_MESHTASTIC_SCANNER_RX_ONLY)
	/* Dedicated scanner build: no transmit path at all. */
	ARG_UNUSED(pkt);
	ARG_UNUSED(pkt_len);
	ARG_UNUSED(retries);
	ARG_UNUSED(busy_retries);
	return -EPERM;
#else

#if defined(CONFIG_MESHTASTIC_SCANNER)
	/* THE gate. While scanning, the radio is parked on a frequency this node
	 * was never configured for, carrying a channel hash it did not derive — a
	 * transmission there would be interference on someone else's channel, not
	 * merely a local bug.
	 *
	 * One check suffices because this function is the single choke point:
	 * lora_send() is called nowhere outside this file, and this function's only
	 * caller is the outbound queue drain. Relay, NodeInfo, position, telemetry,
	 * reliable retransmits, admin replies and phone-injected sends all arrive
	 * here. There is no second path to slip through.
	 *
	 * Refusal is counted rather than silent, so "nothing tried to transmit while
	 * scanning" is an assertion (see `meshtastic scan status`) instead of an
	 * assumption. The drain has already dequeued the frame and simply propagates
	 * this error to whoever enqueued it, so a refusal costs one dropped frame
	 * and never a retry storm. */
	if (meshtastic_scanner_active()) {
		const struct meshtastic_wire_header *h =
			(pkt_len >= MESHTASTIC_HDR_LEN)
				? (const struct meshtastic_wire_header *)pkt
				: NULL;

		if (h != NULL) {
			meshtastic_scanner_note_tx_blocked_frame(sys_le32_to_cpu(h->dest),
								 h->channel, (uint16_t)pkt_len);
			LOG_DBG("TX refused (scanning): to 0x%08x ch 0x%02x len %u",
				sys_le32_to_cpu(h->dest), h->channel, (unsigned int)pkt_len);
		} else {
			meshtastic_scanner_note_tx_blocked();
		}
		return -EPERM;
	}
#endif

	/* config.lora.tx_enabled. The reference exposes this to every config tool
	 * as "receive only" and honours it; we used to store it and transmit
	 * anyway, so a tool could set it, read it back set, and still be on air.
	 * Checked at the same choke point as the scanner gate, and for the same
	 * reason: every transmit in the firmware funnels through here. */
	if (!mt.tx_enabled) {
		LOG_DBG("TX refused: tx_enabled is off (receive-only)");
		return -EPERM;
	}

#if defined(CONFIG_MESHTASTIC_PACKET_HEXDUMP)
	log_wire_tx(pkt, pkt_len);
#endif

	(void)k_sem_take(&mt_radio_sem, K_FOREVER);

#if defined(CONFIG_MESHTASTIC_SCANNER)
	/* The gate again, now that the radio is actually ours.
	 *
	 * The check above runs OUTSIDE mt_radio_sem, so a sweep starting in the
	 * window between the two could take the semaphore first and retune. That is
	 * not merely a stale answer: meshtastic_radio_tune_explicit() writes
	 * mt.frequency and mt.modem, and the configuration below reads exactly those
	 * — so the frame would be configured onto the SCAN preset and transmitted
	 * there. Narrow, but it is the one outcome the gate exists to prevent, and
	 * the semaphore is what makes the answer stable, so the decisive check
	 * belongs on this side of it.
	 *
	 * The early check is kept as well: it is the common path while sweeping, and
	 * it refuses without touching the radio at all. */
	if (meshtastic_scanner_active()) {
		const struct meshtastic_wire_header *h =
			(pkt_len >= MESHTASTIC_HDR_LEN)
				? (const struct meshtastic_wire_header *)pkt
				: NULL;

		if (h != NULL) {
			meshtastic_scanner_note_tx_blocked_frame(sys_le32_to_cpu(h->dest),
								 h->channel, (uint16_t)pkt_len);
		} else {
			meshtastic_scanner_note_tx_blocked();
		}
		(void)k_sem_give(&mt_radio_sem);
		return -EPERM;
	}
#endif

	/* The settle window (docs/MULTI-PRESET-OPERATION.md §4.4, third interlock).
	 * The radio was retuned very recently, so the AGC has not settled on the new
	 * bandwidth — and the first thing this transmit does is ask the chip to
	 * listen before it talks (LORA_CAD_MODE_LBT, below). A CAD taken on an
	 * unsettled front end can read a busy channel as quiet, which turns
	 * listen-before-talk into a collision.
	 *
	 * A wait, not a refusal: the frame is unambiguously on the new preset (the
	 * freeze above refused everything composed while the switch was in flight),
	 * so nothing is wrong with it except its timing.
	 *
	 * Waited out while HOLDING mt_radio_sem, deliberately. Outside it, a retune
	 * could land between the wait and the transmit and put the frame on a radio
	 * that is unsettled all over again — the wait would have measured a tuning
	 * that no longer applies. It costs at most
	 * CONFIG_MESHTASTIC_PRESET_SETTLE_MS, once per switch. */
	settle = meshtastic_preset_settle_remaining_ms();
	if (settle > 0U) {
		meshtastic_preset_note_settle_wait();
		k_sleep(K_MSEC(settle));
	}

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
	mt_lora_cfg.tx_power = meshtastic_tx_power_chip_drive(mt.tx_power, mt.licensed);
	mt_lora_cfg.tx = true;
	mt_lora_cfg.cad.mode = LORA_CAD_MODE_LBT;
	/* 0 = "driver default" (documented in lora.h) -- the sx126x driver
	 * resolves this to RadioLib's own SX1262 CAD defaults (4 symbols,
	 * detPeak=SF+13, detMin=10), the same library upstream Meshtastic
	 * runs, rather than this being a second, possibly-diverging
	 * source of truth for those numbers. Previously hardcoded to
	 * LORA_CAD_SYMB_2, which didn't match either upstream's real
	 * default (4) or anything the driver used to honor at all. */
	mt_lora_cfg.cad.symbol_num = 0;

	/* Steer any RF front-end to its TX path before keying the transmitter. */
	meshtastic_radio_fem_set_tx(true);

	ret = lora_config(mt.lora_dev, &mt_lora_cfg);
	mt_record_effective(true, ret);
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
	{
		/* Deliberately NOT into `ret` — that still carries the send result,
		 * which the caller's status accounting below depends on. The revert
		 * is recorded as an RX-side config because that is what it is: it
		 * puts the radio back where it listens. */
		int revert_rc = lora_config(mt.lora_dev, &mt_lora_cfg);

		mt_record_effective(false, revert_rc);
	}

	/* Return the front-end to its RX path (e.g. re-enable the LNA). */
	meshtastic_radio_fem_set_tx(false);

	k_mutex_unlock(&mt.lock);

	(void)mt_radio_arm_rx();
	meshtastic_powermon_clear(MESHTASTIC_PM_LORA_TX);

	(void)k_sem_give(&mt_radio_sem);

	if (busy_retries > 0) {
		LOG_DBG("TX deferred by CAD busy channel (%d retries)", busy_retries);
	}
#endif /* CONFIG_MESHTASTIC_SCANNER_RX_ONLY */

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

#if defined(CONFIG_MESHTASTIC_SCANNER)
	/* Survey hook, BEFORE the queue. The scanner must see frames the normal
	 * stack discards — wrong channel, undecryptable, filtered — which is most of
	 * what a foreign mesh produces, and the whole point of the survey. It reads
	 * only the plaintext header and does not block. */
	meshtastic_scanner_on_frame(data, size, rssi, snr);

	/* ...and then the frame STOPS here. A scanning node is an observer, not a
	 * participant: it is parked on someone else's frequency, so anything it
	 * hears belongs to a mesh it is not a member of.
	 *
	 * Letting those frames continue was a real defect, caught on rzr3 on
	 * 2026-08-19. The nodedb module takes ALL_PACKETS, so a 10-minute sweep
	 * learned 43 foreign nodes and generated 4 NodeInfo requests for them —
	 * refused by the TX gate, but they should never have been produced. Without
	 * that gate the node would have announced itself onto two foreign meshes on
	 * a frequency it was never configured for.
	 *
	 * Gated on the same condition as TX, deliberately: while the node is off its
	 * operating preset it neither transmits nor ingests. One rule, both
	 * directions — and it covers the restore window, where a frame could belong
	 * to either tuning. */
	if (meshtastic_scanner_active()) {
		meshtastic_scanner_note_rx_dropped();
		return;
	}
#endif

#if defined(CONFIG_MESHTASTIC_RF_HIST)
	/* Measurement tap. The position is argued, not incidental — two pressures
	 * pull in opposite directions and both matter:
	 *
	 *  - AFTER the scanner gate above, so a sweeping node does not fold frames
	 *    from other people's presets into its own signal statistics. Those
	 *    frames are on a different frequency and spreading factor; they say
	 *    nothing about the chain this node operates on.
	 *
	 *  - BEFORE the msgq put below, so a frame the software queue drops is
	 *    still counted. The radio demodulated it; a full queue is our limit,
	 *    not the RF chain's, and dropping those silently would bias the sample
	 *    downward during exactly the bursts an experiment wants to see.
	 *
	 * Not in the router, deliberately: it discards ignored, duplicate and
	 * undecodable frames before RSSI is ever read, and weak undecodable frames
	 * are the population an antenna or LNA change actually moves. */
	meshtastic_rf_on_rx(data, size, rssi, snr);
#endif

	slot.len = size;
	slot.rssi = rssi;
	slot.snr = snr;
	slot.bearer = MESHTASTIC_BEARER_LORA;
	memcpy(slot.buf, data, size);

	if (k_msgq_put(&mt_rx_msgq, &slot, K_NO_WAIT) != 0) {
		mt.status.rx_dropped++;
		LOG_DBG("RX queue full, dropped %u-byte frame", size);
	}
}

int meshtastic_radio_rx_inject(const uint8_t *buf, uint16_t len, enum meshtastic_bearer bearer)
{
	struct mt_rx_slot slot;

	if (buf == NULL || len < MESHTASTIC_HDR_LEN || len > sizeof(slot.buf)) {
		return -EINVAL;
	}

	slot.len = len;
	/* No RF reception happened: rssi/snr carry no meaning on a wired-style
	 * bearer. Zeros are the explicit "no measurement" the router's bookkeeping
	 * gates on (it never folds non-LoRa frames into RF signal stats). */
	slot.rssi = 0;
	slot.snr = 0;
	slot.bearer = (uint8_t)bearer;
	memcpy(slot.buf, buf, len);

	if (k_msgq_put(&mt_rx_msgq, &slot, K_NO_WAIT) != 0) {
		mt.status.rx_dropped++;
		LOG_DBG("RX queue full, dropped %u-byte bearer frame", len);
		return -ENOBUFS;
	}
	return 0;
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
			meshtastic_router_process_rx(slot.buf, slot.len, slot.rssi, slot.snr,
						     (enum meshtastic_bearer)slot.bearer);
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

/* RX gain boost override (G-2) — data->rx_boosted, not the devicetree default,
 * is what sx126x_lora_config() applies on every call, so this survives every
 * subsequent TX/RX reconfigure rather than being clobbered back. Called once
 * at boot from meshtastic_radio_init(); a LoRaConfig change already requires
 * a reboot to take effect (F-1), so there is no live-apply path to wire. */
extern int sx126x_set_rx_boosted_gain(const struct device *dev, bool boosted);

/* Chip-level AGC reset (warm-sleep cycle + RX sensitivity register patch) —
 * parity: upstream Meshtastic RadioLibInterface::resetAGC(). The driver
 * function only does the chip-level cycle; pausing/resuming continuous RX
 * around it is this file's job, same as it already is for a TX. */
extern int sx126x_reset_agc(const struct device *dev);

/* CAD and AGC-reset diagnostic counters (driver-owned; see sx126x.c). */
extern uint32_t sx126x_cad_clear_count_get(void);
extern uint32_t sx126x_cad_busy_count_get(void);
extern uint32_t sx126x_cad_timeout_count_get(void);
extern uint32_t sx126x_cad_error_count_get(void);
extern uint32_t sx126x_agc_reset_ok_count_get(void);
extern uint32_t sx126x_agc_reset_fail_count_get(void);
extern uint32_t sx126x_agc_reset_skipped_count_get(void);
extern uint32_t sx126x_agc_patch_fail_count_get(void);
extern void sx126x_cad_agc_stats_reset(void);

uint32_t meshtastic_radio_cad_clear_count(void)
{
	return sx126x_cad_clear_count_get();
}

uint32_t meshtastic_radio_cad_busy_count(void)
{
	return sx126x_cad_busy_count_get();
}

uint32_t meshtastic_radio_cad_timeout_count(void)
{
	return sx126x_cad_timeout_count_get();
}

uint32_t meshtastic_radio_cad_error_count(void)
{
	return sx126x_cad_error_count_get();
}

uint32_t meshtastic_radio_agc_reset_ok_count(void)
{
	return sx126x_agc_reset_ok_count_get();
}

uint32_t meshtastic_radio_agc_reset_fail_count(void)
{
	return sx126x_agc_reset_fail_count_get();
}

uint32_t meshtastic_radio_agc_reset_skipped_count(void)
{
	return sx126x_agc_reset_skipped_count_get();
}

uint32_t meshtastic_radio_agc_patch_fail_count(void)
{
	return sx126x_agc_patch_fail_count_get();
}

void meshtastic_radio_cad_agc_stats_reset(void)
{
	sx126x_cad_agc_stats_reset();
}

/*
 * Readback of what the driver staged vs what it actually applied, plus the SPI
 * BUSY-timeout streak.
 *
 * These are declared __weak here rather than plain extern on purpose: they are
 * provided by carried patch 0011 (rx-boost readback) and by
 * sx126x_hal_common.c, and this firmware must still LINK against a Zephyr tree
 * that has not had the patch applied. A build that silently failed to link
 * would be an obvious problem; one that linked and reported a confident,
 * fabricated answer would be a much worse one. So the fallbacks return
 * UNKNOWN/0 and the report prints them as unknown.
 */
__weak bool sx126x_rx_boosted_staged_get(const struct device *dev, bool *out)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(out);
	return false;
}

__weak bool sx126x_rx_boosted_applied_get(const struct device *dev, bool *out)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(out);
	return false;
}

__weak uint32_t sx126x_hal_busy_timeout_streak(void)
{
	return 0U;
}

static enum meshtastic_radio_tristate mt_tri(bool got, bool value)
{
	if (!got) {
		return MESHTASTIC_RADIO_TRI_UNKNOWN;
	}
	return value ? MESHTASTIC_RADIO_TRI_ON : MESHTASTIC_RADIO_TRI_OFF;
}

enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_staged(void)
{
	bool v = false;

	return mt_tri(sx126x_rx_boosted_staged_get(mt.lora_dev, &v), v);
}

enum meshtastic_radio_tristate meshtastic_radio_rx_boosted_applied(void)
{
	bool v = false;

	return mt_tri(sx126x_rx_boosted_applied_get(mt.lora_dev, &v), v);
}

uint32_t meshtastic_radio_busy_timeout_streak(void)
{
	return sx126x_hal_busy_timeout_streak();
}

static void mt_agc_reset_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(mt_agc_reset_work, mt_agc_reset_work_fn);

/*
 * Periodic AGC reset (parity: upstream's AGC_RESET_INTERVAL_MS, default 60s —
 * see sx126x_reset_agc() for why this exists at all). Mirrors the TX-prep
 * pattern exactly: take mt_radio_sem, stop continuous async RX before
 * touching the radio, do the operation, re-arm RX, release the semaphore —
 * so this can never race a send.
 */
static void mt_agc_reset_work_fn(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	(void)k_sem_take(&mt_radio_sem, K_FOREVER);

	(void)lora_recv_async(mt.lora_dev, NULL, NULL);
	mt.radio_rx_armed = false;
	meshtastic_powermon_clear(MESHTASTIC_PM_LORA_RX);

	ret = sx126x_reset_agc(mt.lora_dev);
	if (ret < 0 && ret != -EBUSY) {
		LOG_WRN("Periodic AGC reset failed (%d)", ret);
	}

	(void)mt_radio_arm_rx();

	(void)k_sem_give(&mt_radio_sem);

	(void)k_work_reschedule(&mt_agc_reset_work,
				K_MSEC(CONFIG_MESHTASTIC_AGC_RESET_INTERVAL_MS));
}

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
 * Strong override of the sx126x driver's __weak sx126x_hal_busy_timeout_report().
 * Enriches a BUSY timeout with the light-sleep correlation — the
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
	mt_lora_cfg.tx_power = meshtastic_tx_power_chip_drive(mt.tx_power, mt.licensed);
	mt_lora_cfg.tx = false;

#if defined(CONFIG_LORA_SX126X)
	/* Stage the config-derived RX gain override before the first
	 * lora_config() call, which is what actually pushes it to the chip. */
	(void)sx126x_set_rx_boosted_gain(mt.lora_dev, mt.rx_boosted_gain);
#endif

	ret = lora_config(mt.lora_dev, &mt_lora_cfg);
	mt_record_effective(false, ret);
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

#if defined(CONFIG_LORA_SX126X)
	(void)k_work_schedule(&mt_agc_reset_work, K_MSEC(CONFIG_MESHTASTIC_AGC_RESET_INTERVAL_MS));
#endif

	return 0;
}
