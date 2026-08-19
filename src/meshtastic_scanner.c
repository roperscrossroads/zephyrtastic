/* SPDX-License-Identifier: GPL-3.0
 *
 * Multi-preset RX-only survey — see meshtastic_scanner.h.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "meshtastic_airtime.h"
#include "meshtastic_clock.h"
#include "meshtastic_core.h"
#include "meshtastic_ext_ram.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_scanner.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define RING_CAP   CONFIG_MESHTASTIC_SCANNER_RING_RECORDS
#define DWELL_C_MS CONFIG_MESHTASTIC_SCANNER_DWELL_MS

/* The presets to visit, resolved against their own CANONICAL (empty-channel-name)
 * frequencies. A stock node never writes a literal channel name, so these ten
 * cover every default-configured mesh. Named meshes sit on other slots entirely
 * (docs/MULTI-PRESET-OPERATION.md §1.0a); finding those is a separate slot sweep
 * over ~364 landing spots rather than ten. */
static const meshtastic_Config_LoRaConfig_ModemPreset scan_presets[] = {
	meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
	meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW,
	meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE,
	meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO,
	meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW,
	meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
	meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_TURBO,
	meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW,
	meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,
	meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
};

#define SCAN_N ARRAY_SIZE(scan_presets)

static struct {
	struct k_mutex lock;
	bool lock_ready;
	bool active;
	uint8_t cur;
	uint32_t head;  /* next write slot; also the running count of writes */
	uint32_t total; /* ever captured; keeps counting past the ring */
	struct meshtastic_scan_stats stats[SCAN_N];
} scan;

/* The ring is the big allocation, so it goes to PSRAM where the board has one.
 * CPU-only and mutex-guarded, which is what MESHTASTIC_EXT_RAM_BSS_ATTR requires:
 * no DMA, no ISR access, never read during a flash-write cache window. */
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_scan_record scan_ring[RING_CAP];

static void scan_lock(void)
{
	if (!scan.lock_ready) {
		k_mutex_init(&scan.lock);
		scan.lock_ready = true;
	}
	k_mutex_lock(&scan.lock, K_FOREVER);
}

static void scan_unlock(void)
{
	k_mutex_unlock(&scan.lock);
}

void meshtastic_scanner_on_frame(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr)
{
	const struct meshtastic_wire_header *hdr;
	struct meshtastic_scan_record *rec;

	if (buf == NULL || len < MESHTASTIC_HDR_LEN) {
		return;
	}

	hdr = (const struct meshtastic_wire_header *)buf;

	scan_lock();
	if (!scan.active) {
		scan_unlock();
		return;
	}

	rec = &scan_ring[scan.head % RING_CAP];

	rec->epoch_sec = meshtastic_clock_now_epoch();
	rec->epoch_ms = (uint16_t)(meshtastic_clock_now_epoch_ms() % 1000);
	rec->from = hdr->src;
	rec->to = hdr->dest;
	rec->id = hdr->id;
	rec->preset = (uint8_t)scan_presets[scan.cur];
	rec->flags = hdr->flags;
	rec->chan_hash = hdr->channel;
	rec->next_hop = hdr->next_hop;
	rec->relay_node = hdr->relay_node;
	rec->rssi = rssi;
	rec->snr = snr;
	/* Length only. The payload is never copied — this is a header survey, and
	 * keeping it one is what makes it safe to run over other people's meshes. */
	rec->payload_len = (uint8_t)(len - MESHTASTIC_HDR_LEN);

	scan.head++;
	scan.total++;
	scan.stats[scan.cur].heard++;
	scan_unlock();
}

static void scan_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct meshtastic_modem_params modem;
		struct meshtastic_freq_plan plan;
		uint32_t dwell;
		uint8_t i;

		scan_lock();
		if (!scan.active) {
			scan_unlock();
			k_sleep(K_MSEC(250));
			continue;
		}
		i = scan.cur;
		scan_unlock();

		(void)meshtastic_preset_to_params(scan_presets[i], false, &modem);

		/* Resolve against the PRESET's own display name, not this node's channel
		 * name: a named local channel would otherwise drag every visit back to
		 * that one slot instead of where foreign meshes actually are (§3.1a). */
		if (meshtastic_region_freq_plan(
			    meshtastic_preset_region(), scan_presets[i],
			    meshtastic_preset_display_name(scan_presets[i], true), true,
			    &plan) < 0 ||
		    meshtastic_radio_tune_explicit(plan.frequency_hz, modem.spread_factor,
						   modem.bandwidth_hz, modem.coding_rate) < 0) {
			/* No plan for this preset in this region, or the driver cannot
			 * represent it: skip rather than listen on a guess and attribute
			 * the silence to the preset. */
			scan_lock();
			scan.cur = (uint8_t)((i + 1U) % SCAN_N);
			scan_unlock();
			continue;
		}

		/* Dwell = this preset's worst-case time-on-air + the constant. Computed
		 * AFTER tuning, because lora_airtime() reads the driver's current config
		 * — which keeps one formula rather than a second copy that can drift.
		 *
		 * The ToA term is not padding. A frame is only captured if its preamble
		 * starts while we are already listening, so a fixed dwell would silently
		 * make slow presets invisible: LongSlow's 9.28 s max packet cannot be
		 * caught AT ALL by a 5 s dwell, and the survey would confidently report
		 * "nobody uses LongSlow". Adding ToA equalises capture probability at
		 * c/T across every preset, which is what makes the rates comparable. */
		dwell = meshtastic_airtime_packet_ms(MESHTASTIC_PKT_MAX) + (uint32_t)DWELL_C_MS;

		scan_lock();
		scan.stats[i].preset = scan_presets[i];
		scan.stats[i].frequency_hz = plan.frequency_hz;
		scan.stats[i].visits++;
		scan.stats[i].dwell_ms_total += dwell;
		scan_unlock();

		LOG_DBG("scan: %s %u Hz SF%u BW%uk dwell %ums",
			meshtastic_preset_display_name(scan_presets[i], true), plan.frequency_hz,
			modem.spread_factor, modem.bandwidth_hz / 1000U, dwell);

		k_sleep(K_MSEC(dwell));

		scan_lock();
		scan.cur = (uint8_t)((i + 1U) % SCAN_N);
		scan_unlock();
	}
}

static K_THREAD_STACK_DEFINE(scan_stack, CONFIG_MESHTASTIC_SCANNER_STACK_SIZE);
static struct k_thread scan_thread;

int meshtastic_scanner_start(void)
{
	scan_lock();
	if (scan.active) {
		scan_unlock();
		return -EALREADY;
	}
	scan.active = true;
	scan.cur = 0U;
	scan_unlock();

	LOG_INF("scanner: sweeping %u presets, dwell = ToA + %ums", (unsigned int)SCAN_N,
		(unsigned int)DWELL_C_MS);
	return 0;
}

int meshtastic_scanner_stop(void)
{
	scan_lock();
	if (!scan.active) {
		scan_unlock();
		return -EALREADY;
	}
	scan.active = false;
	scan_unlock();

	/* Return through the preset path rather than a hand-rolled tune: it
	 * re-derives the channel hashes and keeps mt.modem_preset agreeing with the
	 * radio, which an explicit tune deliberately does not. */
	return meshtastic_preset_switch(mt.modem_preset, NULL);
}

bool meshtastic_scanner_active(void)
{
	bool a;

	scan_lock();
	a = scan.active;
	scan_unlock();
	return a;
}

uint32_t meshtastic_scanner_total(void)
{
	uint32_t t;

	scan_lock();
	t = scan.total;
	scan_unlock();
	return t;
}

void meshtastic_scanner_reset(void)
{
	scan_lock();
	scan.head = 0U;
	scan.total = 0U;
	memset(scan.stats, 0, sizeof(scan.stats));
	scan_unlock();
}

int meshtastic_scanner_stats(struct meshtastic_scan_stats *out, size_t max)
{
	size_t n;

	if (out == NULL) {
		return -EINVAL;
	}

	scan_lock();
	n = MIN(max, (size_t)SCAN_N);
	for (size_t i = 0; i < n; i++) {
		out[i] = scan.stats[i];
		out[i].preset = scan_presets[i];
	}
	scan_unlock();

	return (int)n;
}

int meshtastic_scanner_records(struct meshtastic_scan_record *out, size_t max, uint32_t from)
{
	uint32_t retained;
	uint32_t oldest;
	size_t n = 0;

	if (out == NULL) {
		return -EINVAL;
	}

	scan_lock();
	retained = MIN(scan.total, (uint32_t)RING_CAP);
	/* index 0 means "oldest still retained", which is not record 0 once the ring
	 * has wrapped — the caller asked for what exists, not what was dropped. */
	oldest = (scan.total > (uint32_t)RING_CAP) ? (scan.total - (uint32_t)RING_CAP) : 0U;

	for (uint32_t i = from; i < retained && n < max; i++, n++) {
		out[n] = scan_ring[(oldest + i) % RING_CAP];
	}
	scan_unlock();

	return (int)n;
}

static int scanner_init(void)
{
	k_thread_create(&scan_thread, scan_stack, K_THREAD_STACK_SIZEOF(scan_stack),
			scan_thread_fn, NULL, NULL, NULL, CONFIG_MESHTASTIC_SCANNER_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&scan_thread, "mt_scan");

#if defined(CONFIG_MESHTASTIC_SCANNER_AUTOSTART)
	(void)meshtastic_scanner_start();
#endif
	return 0;
}
SYS_INIT(scanner_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
