/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Channel and TX airtime accounting (ported from Meshtastic firmware airtime.cpp).
 *
 * The reference firmware advances its rolling utilization windows from a 1 Hz
 * thread.  Here the windows decay LAZILY instead: airtime_advance() clears any
 * bucket that has been entered since the previous access, and it is called at
 * the top of every log/read.  The observable outputs (the two util percentages,
 * each a rolling sum over the same 60 s / 60 min windows) are identical, but an
 * idle node no longer carries a periodic wakeup here — which matters for letting
 * the SoC reach light sleep.  See ~/lab/meshprojects/POWER-MGMT-STATUS.md §3.
 */

#include <string.h>

#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>

#include "meshtastic_airtime.h"
#include "meshtastic_core.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

struct meshtastic_airtime_state {
	struct k_mutex lock;
	uint32_t channel_utilization[MESHTASTIC_CHANNEL_UTILIZATION_PERIODS];
	uint32_t utilization_tx[MESHTASTIC_MINUTES_IN_HOUR];
	/* Absolute bucket counts (not indices) at the last advance, so a gap of
	 * any length is handled correctly: channel-util counts 10 s buckets, TX
	 * counts 60 s buckets.
	 */
	uint32_t last_util_tick;
	uint32_t last_tx_tick;
	bool initialized;
};

static struct meshtastic_airtime_state airtime;

static uint32_t now_seconds(void)
{
	return (uint32_t)(k_uptime_get() / 1000);
}

static uint8_t period_util_minute(uint32_t sec)
{
	return (uint8_t)((sec / 10U) % MESHTASTIC_CHANNEL_UTILIZATION_PERIODS);
}

static uint8_t period_util_hour(uint32_t sec)
{
	return (uint8_t)((sec / 60U) % MESHTASTIC_MINUTES_IN_HOUR);
}

/*
 * Advance both rolling windows to `sec` (seconds of uptime), zeroing every
 * bucket newly entered since the previous call — the lazy equivalent of the
 * former 1 Hz tick, which cleared the just-entered bucket on each rollover.
 * Must be called with airtime.lock held, before any add or read.
 */
static void airtime_advance(uint32_t sec)
{
	uint32_t util_tick = sec / 10U; /* 10 s channel-util buckets */
	uint32_t tx_tick = sec / 60U;   /* 60 s TX-util buckets */
	uint32_t elapsed;

	if (!airtime.initialized) {
		airtime.initialized = true;
		airtime.last_util_tick = util_tick;
		airtime.last_tx_tick = tx_tick;
		return;
	}

	/* Channel utilization: 6 × 10 s buckets (60 s window). */
	elapsed = util_tick - airtime.last_util_tick;
	if (elapsed >= MESHTASTIC_CHANNEL_UTILIZATION_PERIODS) {
		memset(airtime.channel_utilization, 0, sizeof(airtime.channel_utilization));
	} else {
		for (uint32_t i = 1U; i <= elapsed; i++) {
			airtime.channel_utilization[(airtime.last_util_tick + i) %
						    MESHTASTIC_CHANNEL_UTILIZATION_PERIODS] = 0U;
		}
	}
	airtime.last_util_tick = util_tick;

	/* TX utilization: 60 × 60 s buckets (1 h window). */
	elapsed = tx_tick - airtime.last_tx_tick;
	if (elapsed >= MESHTASTIC_MINUTES_IN_HOUR) {
		memset(airtime.utilization_tx, 0, sizeof(airtime.utilization_tx));
	} else {
		for (uint32_t i = 1U; i <= elapsed; i++) {
			airtime.utilization_tx[(airtime.last_tx_tick + i) %
					       MESHTASTIC_MINUTES_IN_HOUR] = 0U;
		}
	}
	airtime.last_tx_tick = tx_tick;
}

void meshtastic_airtime_log(enum meshtastic_airtime_type type, uint32_t ms)
{
	uint32_t sec = now_seconds();

	k_mutex_lock(&airtime.lock, K_FOREVER);
	airtime_advance(sec);

	switch (type) {
	case MESHTASTIC_AIRTIME_TX:
		LOG_DBG("Packet TX: %u ms", ms);
		airtime.utilization_tx[period_util_hour(sec)] += ms;
		airtime.channel_utilization[period_util_minute(sec)] += ms;
		break;
	case MESHTASTIC_AIRTIME_RX:
		LOG_DBG("Packet RX: %u ms", ms);
		airtime.channel_utilization[period_util_minute(sec)] += ms;
		break;
	case MESHTASTIC_AIRTIME_RX_ALL:
		LOG_DBG("Packet RX (noise?): %u ms", ms);
		airtime.channel_utilization[period_util_minute(sec)] += ms;
		break;
	default:
		break;
	}

	k_mutex_unlock(&airtime.lock);
}

uint32_t meshtastic_airtime_packet_ms(uint32_t wire_len)
{
	if (mt.lora_dev == NULL || !device_is_ready(mt.lora_dev)) {
		return 0U;
	}

	return lora_airtime(mt.lora_dev, wire_len);
}

float meshtastic_airtime_channel_util_percent(void)
{
	uint32_t sum = 0U;
	float percent;

	k_mutex_lock(&airtime.lock, K_FOREVER);
	airtime_advance(now_seconds());
	for (size_t i = 0; i < MESHTASTIC_CHANNEL_UTILIZATION_PERIODS; i++) {
		sum += airtime.channel_utilization[i];
	}
	k_mutex_unlock(&airtime.lock);

	percent = ((float)sum / (float)(MESHTASTIC_CHANNEL_UTILIZATION_PERIODS * 10U * 1000U)) *
		  100.0f;

	return percent;
}

float meshtastic_airtime_tx_util_percent(void)
{
	uint32_t sum = 0U;
	float percent;

	k_mutex_lock(&airtime.lock, K_FOREVER);
	airtime_advance(now_seconds());
	for (size_t i = 0; i < MESHTASTIC_MINUTES_IN_HOUR; i++) {
		sum += airtime.utilization_tx[i];
	}
	k_mutex_unlock(&airtime.lock);

	percent = ((float)sum / (float)MESHTASTIC_MS_IN_HOUR) * 100.0f;

	return percent;
}

int meshtastic_airtime_init(void)
{
	memset(&airtime, 0, sizeof(airtime));
	k_mutex_init(&airtime.lock);
	/*
	 * No periodic timer: the rolling windows decay lazily on access
	 * (airtime_advance), so an idle node adds no wakeup here.  `initialized`
	 * is false until the first access seeds the bucket ticks.
	 */
	return 0;
}
