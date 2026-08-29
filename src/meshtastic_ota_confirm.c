/* SPDX-License-Identifier: GPL-3.0
 *
 * Auto-confirm the running image after a healthy window (MESHTASTIC_OTA_AUTOCONFIRM).
 *
 * The other half of a node-to-node update. A courier delivers an image and
 * leaves it UNCONFIRMED (meshtastic_smp_central.c, UPDATE_NOCONFIRM): the
 * courier can prove the right version is active, but only the target can prove
 * it is HEALTHY, and confirming on a peer's behalf would defeat MCUboot's
 * revert-on-bad-boot safety net. So the target confirms itself — but only after
 * it has stayed up long enough to trust the image.
 *
 * "Healthy" here is simply "still running after the delay". A boot that faults
 * resets before this fires, and an unconfirmed image reverts on that reset;
 * richer signals (radio up, no recorded crash) are a later refinement. The
 * node's beat carries TESTBOOT until this clears it (via boot_is_img_confirmed),
 * which is the edge a courier watches to call a round done.
 *
 * ⚠ nRF trap: MCUboot leaves a hardware watchdog running that the app inherits
 * (NODE-TO-NODE-SMP.md §7). This work runs on the system workqueue while the
 * node is actively beating and serving the radio, so the CPU is not deeply
 * idle during the window — but a future "quiet" health window must feed that
 * watchdog, not assume idle.
 */

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_MESHTASTIC_OTA_HOLD_DURING_DFU)
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt_callbacks.h>
#include "meshtastic_ble_peer.h"
#endif

LOG_MODULE_REGISTER(mt_otaconfirm, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_MESHTASTIC_OTA_HOLD_DURING_DFU)
/* Consent, said out loud: while a courier (or a host) is writing into our
 * slot1, our beats carry HOLD so nobody else starts a competing upload.
 * DFU_STARTED fires on the first chunk, DFU_STOPPED when the upload ends
 * (complete, aborted or erased); PENDING/CONFIRMED are the image-state
 * writes that follow and are not an upload. RAM-only: the reboot into the
 * new image clears it.
 *
 * STOPPED is not enough on its own. img_mgmt raises it only from inside a
 * request it is processing, so a writer that VANISHES mid-upload — no abort,
 * no last chunk — leaves the flag raised with nothing left to lower it, and
 * every courier from then on skips this node as busy. That is not theoretical:
 * on 2026-08-28 the courier's BLE controller faulted 41 s into a push and the
 * target sat in HOLD until an operator cleared it by hand. So the flag is also
 * on a dead-man timer, rearmed by each chunk written and cancelled by STOPPED.
 * `hold_is_ours` keeps it honest: a HOLD an operator raised with
 * `blepeer hold on` is never lowered by this module. */
static void dfu_hold_release(const char *why);

static bool hold_is_ours;

#if CONFIG_MESHTASTIC_OTA_HOLD_IDLE_TIMEOUT_S > 0
static void dfu_hold_idle_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	dfu_hold_release("no upload progress for "
			 STRINGIFY(CONFIG_MESHTASTIC_OTA_HOLD_IDLE_TIMEOUT_S)
			 " s — the writer is gone");
}
static K_WORK_DELAYABLE_DEFINE(dfu_hold_idle, dfu_hold_idle_fn);

static void dfu_hold_idle_rearm(void)
{
	(void)k_work_reschedule(&dfu_hold_idle,
				K_SECONDS(CONFIG_MESHTASTIC_OTA_HOLD_IDLE_TIMEOUT_S));
}

static void dfu_hold_idle_cancel(void)
{
	(void)k_work_cancel_delayable(&dfu_hold_idle);
}
#else
static void dfu_hold_idle_rearm(void) { }
static void dfu_hold_idle_cancel(void) { }
#endif

static void dfu_hold_release(const char *why)
{
	if (!hold_is_ours) {
		return; /* an operator's hold, or none — not ours to lower */
	}
	hold_is_ours = false;
	meshtastic_ble_peer_hold_set(false);
	LOG_INF("OTA: %s — HOLD released", why);
}

static enum mgmt_cb_return dfu_hold_cb(uint32_t event, enum mgmt_cb_return prev_status,
				       int32_t *rc, uint16_t *group, bool *abort_more, void *data,
				       size_t data_size)
{
	ARG_UNUSED(prev_status);
	ARG_UNUSED(rc);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);
	ARG_UNUSED(data);
	ARG_UNUSED(data_size);

	if (event == MGMT_EVT_OP_IMG_MGMT_DFU_STARTED) {
		if (!meshtastic_ble_peer_hold_get()) {
			LOG_INF("OTA: upload into slot1 started — beats say HOLD");
			hold_is_ours = true;
			meshtastic_ble_peer_hold_set(true);
		}
		dfu_hold_idle_rearm();
	} else if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK_WRITE_COMPLETE) {
		dfu_hold_idle_rearm();
	} else if (event == MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED) {
		dfu_hold_idle_cancel();
		dfu_hold_release("upload stopped");
	}
	return MGMT_CB_OK;
}

static struct mgmt_callback dfu_hold_callback = {
	.callback = dfu_hold_cb,
	.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STARTED | MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED |
		    MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK_WRITE_COMPLETE,
};
#endif /* CONFIG_MESHTASTIC_OTA_HOLD_DURING_DFU */

#if CONFIG_MESHTASTIC_OTA_FAULT_TEST_SEC > 0
/* TEST ONLY (overlay-fault-test.conf): a deliberately bad image. It comes up,
 * beats — so a courier sees it running the delivered version — and then
 * faults inside the confirm window. RESET_ON_FATAL reboots it, MCUboot finds
 * the image unconfirmed and swaps back, and the courier watching the beats
 * should call the round REVERTED and never push that version again. */
static void fault_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_ERR("OTA FAULT TEST: faulting on purpose (%d s after boot)",
		CONFIG_MESHTASTIC_OTA_FAULT_TEST_SEC);
	/* k_panic(), not k_oops(): an oops in a non-essential thread only aborts
	 * that thread (Zephyr's z_fatal_error) and the node limps on until the
	 * inherited hardware watchdog fires — a WDT reset, not the immediate
	 * software reset a real crash (an exception, a panic) produces under
	 * RESET_ON_FATAL_ERROR. The bench showed the two reset kinds behave
	 * differently on the kit's bootloader chain (2026-08-27). */
	k_panic();
}

static K_WORK_DELAYABLE_DEFINE(fault_work, fault_work_fn);
#endif

static void confirm_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (boot_is_img_confirmed()) {
		return; /* a flash/USB path, or an earlier confirm, already did it */
	}
	if (boot_write_img_confirmed() == 0) {
		LOG_INF("OTA: image confirmed after %d s healthy uptime",
			CONFIG_MESHTASTIC_OTA_CONFIRM_DELAY_SEC);
	} else {
		LOG_ERR("OTA: boot_write_img_confirmed failed — MCUboot may revert on next reset");
	}
}

static K_WORK_DELAYABLE_DEFINE(confirm_work, confirm_work_fn);

#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <hal/nrf_power.h>
/* Why did we boot? RESETREAS is sticky until written, so after a board comes
 * back from the dark (a bootloader drop, a DFU) this line names every reset
 * that happened meanwhile — SREQ (software), DOG (watchdog), RESETPIN, LOCKUP,
 * OFF… — then clears it so the next boot reports only its own. GPREGRET is the
 * UF2 bootloader's DFU request register (0x57 UF2, 0x4E serial, 0xA8 OTA) and
 * 0x20007F7C its double-reset word (bench 2026-08-27: the xhli.16 instrument).
 */
static void reset_reason_report(void)
{
	uint32_t reas = nrf_power_resetreas_get(NRF_POWER);
	uint32_t gpregret = NRF_POWER->GPREGRET;
	uint32_t dbl = *(volatile uint32_t *)0x20007F7CUL;

	LOG_INF("reset reason 0x%08x:%s%s%s%s%s%s%s  GPREGRET=0x%02x  dblreset=0x%08x", reas,
		(reas & NRF_POWER_RESETREAS_RESETPIN_MASK) ? " PIN" : "",
		(reas & NRF_POWER_RESETREAS_DOG_MASK) ? " DOG" : "",
		(reas & NRF_POWER_RESETREAS_SREQ_MASK) ? " SREQ" : "",
		(reas & NRF_POWER_RESETREAS_LOCKUP_MASK) ? " LOCKUP" : "",
		(reas & NRF_POWER_RESETREAS_OFF_MASK) ? " OFF" : "",
		(reas & NRF_POWER_RESETREAS_DIF_MASK) ? " DIF" : "",
		reas == 0U ? " (power-on/none)" : "", gpregret, dbl);
	nrf_power_resetreas_clear(NRF_POWER, 0xFFFFFFFFUL);
}
#endif

static int ota_confirm_init(void)
{
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
	reset_reason_report();
#endif
#if defined(CONFIG_MESHTASTIC_OTA_HOLD_DURING_DFU)
	mgmt_callback_register(&dfu_hold_callback);
#endif
#if CONFIG_MESHTASTIC_OTA_FAULT_TEST_SEC > 0
	LOG_WRN("OTA FAULT TEST image: will fault %d s after boot", CONFIG_MESHTASTIC_OTA_FAULT_TEST_SEC);
	k_work_schedule(&fault_work, K_SECONDS(CONFIG_MESHTASTIC_OTA_FAULT_TEST_SEC));
#endif
	if (boot_is_img_confirmed()) {
		return 0; /* nothing staged as a test — the common case */
	}
	LOG_INF("OTA: running an unconfirmed image; will self-confirm in %d s if healthy",
		CONFIG_MESHTASTIC_OTA_CONFIRM_DELAY_SEC);
	k_work_schedule(&confirm_work, K_SECONDS(CONFIG_MESHTASTIC_OTA_CONFIRM_DELAY_SEC));
	return 0;
}

SYS_INIT(ota_confirm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
