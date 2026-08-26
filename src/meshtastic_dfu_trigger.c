/* SPDX-License-Identifier: GPL-3.0
 *
 * Software re-entry into the Adafruit-fork nRF52 bootloader.
 *
 * The bootloader inspects the POWER GPREGRET retained register on reset (the
 * register survives a soft reset by design — it is how the stock Arduino
 * core's 1200-baud touch works) and enters the requested DFU mode:
 *
 *   0x57 DFU_MAGIC_UF2_RESET          UF2 mass-storage + serial DFU
 *   0x4E DFU_MAGIC_SERIAL_ONLY_RESET  serial DFU only (no MSC drive)
 *
 * Exists because the XIAO+Wio-SX1262 kits expose no reset button — only an
 * RST pad — so the hardware double-tap needs tweezers on a 1 mm pad
 * (notes/local/infra/hardware/xiao-kit-flashing.md tells that story). An
 * image that faults before USB init still needs the pad; every healthy image
 * carries this hatch instead.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#include <hal/nrf_power.h>
#if defined(CONFIG_MESHTASTIC_DFU_BOOT_GUARD)
#include <cmsis_core.h>
#endif

#include "meshtastic_bootlog.h"
#include "meshtastic_dfu_trigger.h"

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define DFU_MAGIC_UF2_RESET         0x57U
#define DFU_MAGIC_SERIAL_ONLY_RESET 0x4EU

void meshtastic_dfu_mark(bool serial_only)
{
	nrf_power_gpregret_set(NRF_POWER, 0U,
			       serial_only ? DFU_MAGIC_SERIAL_ONLY_RESET : DFU_MAGIC_UF2_RESET);
}

FUNC_NORETURN void meshtastic_dfu_enter(bool serial_only)
{
	LOG_WRN("Rebooting into the bootloader (DFU %s mode)", serial_only ? "serial" : "UF2");
	/* Flush the deferred log so the line above actually leaves the box. */
	while (log_process()) {
	}
	k_sleep(K_MSEC(50));

	meshtastic_dfu_mark(serial_only);
	sys_reboot(SYS_REBOOT_WARM);
	CODE_UNREACHABLE;
}

#if defined(CONFIG_MESHTASTIC_DFU_TOUCH)
/*
 * The 1200-baud touch: a host opening the CDC console at 1200 baud asks for
 * the bootloader (Arduino convention; `adafruit-nrfutil --touch 1200` and the
 * runbook's PowerShell snippet both send it). CDC line coding is pushed by
 * the host whenever it opens the port, so polling the last-seen value is
 * enough — no callback plumbing.
 */
static void dfu_touch_poll(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dfu_touch_work, dfu_touch_poll);

static void dfu_touch_poll(struct k_work *work)
{
	static const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t baud = 0U;
	int ret;

	ARG_UNUSED(work);

	ret = uart_line_ctrl_get(console, UART_LINE_CTRL_BAUD_RATE, &baud);
	if (ret == -ENOSYS || ret == -ENOTSUP) {
		/* Console is not a CDC ACM in this image; nothing to watch. */
		return;
	}
	if (ret == 0 && baud == 1200U) {
		meshtastic_dfu_enter(true);
	}

	(void)k_work_reschedule(&dfu_touch_work,
				K_MSEC(CONFIG_MESHTASTIC_DFU_TOUCH_POLL_MS));
}

static int dfu_touch_init(void)
{
	(void)k_work_schedule(&dfu_touch_work, K_MSEC(CONFIG_MESHTASTIC_DFU_TOUCH_POLL_MS));
	return 0;
}
SYS_INIT(dfu_touch_init, APPLICATION, 99);
#endif /* CONFIG_MESHTASTIC_DFU_TOUCH */

#if defined(CONFIG_MESHTASTIC_DFU_BOOT_GUARD)
/*
 * Hang insurance (faults are DFU_ON_FATAL's job): arm the hardware watchdog
 * at PRE_KERNEL_1 priority 0 — before any POST_KERNEL driver init can hang
 * (the concrete case that motivated this: the sx126x driver's
 * SX126xWaitOnBusy() spins forever with no timeout, at an init priority
 * BEFORE the USB console exists, leaving the board looking dead). If the WDT
 * fires, the next boot passes through here first, sees the DOG reset reason,
 * and lands in the bootloader instead of hanging again.
 *
 * Raw MDK register access, deliberately: the Zephyr WDT driver is a
 * POST_KERNEL device — too late to guard the other POST_KERNEL inits — and
 * this must not race a driver instance (hence `depends on !WATCHDOG`).
 */
static void boot_guard_feed(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(boot_guard_work, boot_guard_feed);

/* Consecutive-failed-boot counter in __noinit RAM. RAM survives every soft
 * reset (watchdog included), and — unlike RESETREAS — nothing else owns it:
 * the Adafruit bootloader runs BEFORE this app on every reset and consumes
 * RESETREAS for its own double-tap detection, which is why the first version
 * of this guard (keyed on the DOG bit) never fired and dead images just
 * boot-looped in the dark. The magic guards against power-on garbage. */
#define BOOT_GUARD_MAGIC 0x42475244U /* "BGRD" */
#define BOOT_GUARD_MAX_ATTEMPTS 3U
static __noinit uint32_t boot_guard_magic;
static __noinit uint32_t boot_guard_attempts;

static void boot_guard_feed(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Reaching the system workqueue IS the definition of a healthy boot. */
	boot_guard_attempts = 0U;
#if defined(CONFIG_MESHTASTIC_BOOTLOG)
	meshtastic_bootlog_heartbeat((uint32_t)k_uptime_seconds());
#endif
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	(void)k_work_reschedule(&boot_guard_work,
				K_SECONDS(CONFIG_MESHTASTIC_DFU_BOOT_GUARD_TIMEOUT_S / 3U));
}

static int boot_guard_arm(void)
{
	if (boot_guard_magic != BOOT_GUARD_MAGIC) {
		boot_guard_magic = BOOT_GUARD_MAGIC;
		boot_guard_attempts = 0U;
	}

	boot_guard_attempts++;
	if (boot_guard_attempts >= BOOT_GUARD_MAX_ATTEMPTS) {
		/* Same image died this many times in a row: stop trying and
		 * park in the bootloader, ready for a reflash. */
		boot_guard_attempts = 0U;
		nrf_power_gpregret_set(NRF_POWER, 0U, DFU_MAGIC_SERIAL_ONLY_RESET);
		NVIC_SystemReset();
	}

	/* Run during sleep, pause under the debugger. */
	NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = CONFIG_MESHTASTIC_DFU_BOOT_GUARD_TIMEOUT_S * 32768U;
	NRF_WDT->RREN = WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1U;
	return 0;
}
SYS_INIT(boot_guard_arm, PRE_KERNEL_1, 0);

static int boot_guard_feeder_start(void)
{
	(void)k_work_schedule(&boot_guard_work, K_NO_WAIT);
	return 0;
}
SYS_INIT(boot_guard_feeder_start, APPLICATION, 99);
#endif /* CONFIG_MESHTASTIC_DFU_BOOT_GUARD */
