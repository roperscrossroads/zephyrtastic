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

#include <zephyr/drivers/lora.h>
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
static const meshtastic_Config_LoRaConfig_ModemPreset scan_presets_all[] = {
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

#define SCAN_N_ALL ARRAY_SIZE(scan_presets_all)

BUILD_ASSERT(SCAN_N_ALL <= MESHTASTIC_SCANNER_MAX_PRESETS, "preset list longer than the cap");

/* The live list — a copy of scan_presets_all unless narrowed at runtime. */
static meshtastic_Config_LoRaConfig_ModemPreset scan_list[MESHTASTIC_SCANNER_MAX_PRESETS];
static uint8_t scan_list_n;

static K_SEM_DEFINE(scan_wake, 0, 1);
static K_SEM_DEFINE(scan_idle, 0, 1);

static struct {
	struct k_mutex lock;
	bool lock_ready;
	/* Two flags, not one, and the distinction is load-bearing.
	 *
	 * `sweeping`  drives the thread.
	 * `tx_closed` drives the TX gate, and must stay set until the radio is back
	 *             on the operating preset. Clearing one flag for both would
	 *             reopen the gate while still parked on a scan frequency — the
	 *             exact "transmit on someone else's channel" outcome this whole
	 *             design exists to prevent.
	 */
	bool sweeping;
	bool tx_closed;
	uint8_t cur;
	uint32_t head;  /* next write slot; also the running count of writes */
	uint32_t total;      /* ever captured; keeps counting past the ring */
	uint32_t tx_blocked; /* transmissions refused while off our preset */
	uint32_t rx_dropped; /* frames surveyed but withheld from the stack */
	struct meshtastic_scan_stats stats[MESHTASTIC_SCANNER_MAX_PRESETS];
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
	if (!scan.sweeping) {
		scan_unlock();
		return;
	}

	rec = &scan_ring[scan.head % RING_CAP];

	rec->epoch_sec = meshtastic_clock_now_epoch();
	rec->epoch_ms = (uint16_t)(meshtastic_clock_now_epoch_ms() % 1000);
	rec->from = hdr->src;
	rec->to = hdr->dest;
	rec->id = hdr->id;
	rec->preset = (uint8_t)scan_list[scan.cur];
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
		if (!scan.sweeping) {
			scan_unlock();
			/* Parked. Tell stop() the radio is ours no longer, then wait to
			 * be woken rather than polling. */
			k_sem_give(&scan_idle);
			(void)k_sem_take(&scan_wake, K_FOREVER);
			continue;
		}
		i = scan.cur;
		scan_unlock();

		(void)meshtastic_preset_to_params(scan_list[i], false, &modem);

		/* Resolve against the PRESET's own display name, not this node's channel
		 * name: a named local channel would otherwise drag every visit back to
		 * that one slot instead of where foreign meshes actually are (§3.1a). */
		if (meshtastic_region_freq_plan(
			    meshtastic_preset_region(), scan_list[i],
			    meshtastic_preset_display_name(scan_list[i], true), true,
			    &plan) < 0 ||
		    meshtastic_radio_tune_explicit(plan.frequency_hz, modem.spread_factor,
						   modem.bandwidth_hz, modem.coding_rate) < 0) {
			/* No plan for this preset in this region, or the driver cannot
			 * represent it: skip rather than listen on a guess and attribute
			 * the silence to the preset. */
			scan_lock();
			scan.cur = (uint8_t)((i + 1U) % scan_list_n);
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
		dwell = lora_airtime(mt.lora_dev, MESHTASTIC_PKT_MAX) + (uint32_t)DWELL_C_MS;

		scan_lock();
		scan.stats[i].preset = scan_list[i];
		scan.stats[i].frequency_hz = plan.frequency_hz;
		scan.stats[i].visits++;
		scan.stats[i].dwell_ms_total += dwell;
		scan_unlock();

		LOG_DBG("scan: %s %u Hz SF%u BW%uk dwell %ums",
			meshtastic_preset_display_name(scan_list[i], true), plan.frequency_hz,
			modem.spread_factor, modem.bandwidth_hz / 1000U, dwell);

		/* Interruptible dwell: stop() gives scan_wake so a stop does not have
		 * to wait out a full dwell — which on LongSlow is ~19 s of the node
		 * being unable to transmit after the operator asked it to resume. */
		(void)k_sem_take(&scan_wake, K_MSEC(dwell));

		scan_lock();
		scan.cur = (uint8_t)((i + 1U) % scan_list_n);
		scan_unlock();
	}
}

static K_THREAD_STACK_DEFINE(scan_stack, CONFIG_MESHTASTIC_SCANNER_STACK_SIZE);
static struct k_thread scan_thread;

int meshtastic_scanner_start(void)
{
	scan_lock();
	if (scan.sweeping) {
		scan_unlock();
		return -EALREADY;
	}
	/* Close the TX gate BEFORE the sweep thread can retune, so no frame can be
	 * transmitted after the radio has left the operating preset. */
	scan.tx_closed = true;
	scan.sweeping = true;
	scan.cur = 0U;
	scan_unlock();

	k_sem_give(&scan_wake); /* un-park the thread */

	LOG_INF("scanner: sweeping %u presets, dwell = ToA + %ums — TX refused until stop",
		(unsigned int)scan_list_n, (unsigned int)DWELL_C_MS);
	return 0;
}

int meshtastic_scanner_stop(void)
{
	int ret;

	scan_lock();
	if (!scan.sweeping) {
		scan_unlock();
		return -EALREADY;
	}
	scan.sweeping = false;
	scan_unlock();

	/* Wake the thread out of its dwell and wait for it to park, so it cannot
	 * retune underneath the restore below. The timeout is a backstop: a missed
	 * ack costs a redundant retune, never a transmit, because tx_closed is still
	 * set until after the restore succeeds. */
	k_sem_give(&scan_wake);
	(void)k_sem_take(&scan_idle, K_MSEC(2000));

	/* Return through the preset path rather than a hand-rolled tune: it
	 * re-derives the channel hashes and keeps mt.modem_preset agreeing with the
	 * radio, which an explicit tune deliberately does not. */
	ret = meshtastic_preset_switch(mt.modem_preset, NULL);

	/* Reopen the TX gate ONLY once the radio is genuinely back on the operating
	 * preset. On failure it stays shut: a node that cannot retune must not
	 * transmit, because it is still sitting on a scan frequency. */
	if (ret == 0) {
		scan_lock();
		scan.tx_closed = false;
		scan_unlock();
	} else {
		LOG_ERR("scanner: restore failed (%d) — TX stays disabled, radio is not "
			"on the operating preset", ret);
	}

	return ret;
}

bool meshtastic_scanner_active(void)
{
	bool closed;

	/* Deliberately reports the TX-GATE state, not the sweep state: this is what
	 * the transmit path asks, and the honest answer to "may I transmit?" spans
	 * the restore window too. */
	scan_lock();
	closed = scan.tx_closed;
	scan_unlock();
	return closed;
}

bool meshtastic_scanner_sweeping(void)
{
	bool s;

	scan_lock();
	s = scan.sweeping;
	scan_unlock();
	return s;
}

void meshtastic_scanner_note_tx_blocked(void)
{
	scan_lock();
	scan.tx_blocked++;
	scan_unlock();
}

uint32_t meshtastic_scanner_tx_blocked(void)
{
	uint32_t n;

	scan_lock();
	n = scan.tx_blocked;
	scan_unlock();
	return n;
}

void meshtastic_scanner_note_rx_dropped(void)
{
	scan_lock();
	scan.rx_dropped++;
	scan_unlock();
}

uint32_t meshtastic_scanner_rx_dropped(void)
{
	uint32_t n;

	scan_lock();
	n = scan.rx_dropped;
	scan_unlock();
	return n;
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
	scan.tx_blocked = 0U;
	scan.rx_dropped = 0U;
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
	n = MIN(max, (size_t)scan_list_n);
	for (size_t i = 0; i < n; i++) {
		out[i] = scan.stats[i];
		out[i].preset = scan_list[i];
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

int meshtastic_scanner_set_presets(const meshtastic_Config_LoRaConfig_ModemPreset *list, size_t n)
{
	if (n > MESHTASTIC_SCANNER_MAX_PRESETS) {
		return -EINVAL;
	}

	/* Validate the WHOLE list before applying any of it: a half-applied list
	 * would leave the sweep visiting a mixture of old and new, and the per-preset
	 * stats indexed against the wrong presets. */
	for (size_t i = 0; i < n; i++) {
		if (strcmp(meshtastic_preset_display_name(list[i], true), "Invalid") == 0) {
			return -EINVAL;
		}
	}

	scan_lock();
	if (list == NULL || n == 0U) {
		memcpy(scan_list, scan_presets_all, sizeof(scan_presets_all));
		scan_list_n = (uint8_t)SCAN_N_ALL;
	} else {
		memcpy(scan_list, list, n * sizeof(*list));
		scan_list_n = (uint8_t)n;
	}
	/* Stats are indexed by position in the list, so a changed list invalidates
	 * them. Clearing is the honest option — carrying counts across would
	 * attribute one preset's captures to another. */
	memset(scan.stats, 0, sizeof(scan.stats));
	scan.cur = 0U;
	scan_unlock();

	return 0;
}

int meshtastic_scanner_get_presets(meshtastic_Config_LoRaConfig_ModemPreset *out, size_t max)
{
	size_t n;

	if (out == NULL) {
		return -EINVAL;
	}

	scan_lock();
	n = MIN(max, (size_t)scan_list_n);
	for (size_t i = 0; i < n; i++) {
		out[i] = scan_list[i];
	}
	scan_unlock();

	return (int)n;
}

static int scanner_init(void)
{
	(void)meshtastic_scanner_set_presets(NULL, 0U); /* full set by default */

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
