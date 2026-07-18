/*
 * Heltec WiFi LoRa 32 V4 — RF front-end (FEM) bring-up with runtime detection.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One firmware image for both V4 PCB revisions (and R8). Mirrors the reference
 * firmware (firmware/src/mesh/LoRaFEMInterface.cpp): power the FEM LDO rail, then
 * read the CSD line as an input — the two front-ends bias it differently, so
 * HIGH => KCT8103L (rev 4.3 / R8) and LOW => GC1109 (rev 4.2) — then enable the
 * detected FEM and set its PA/LNA mode pin.
 *
 * The SX1262's DIO2 drives the FEM TX/RX *path*-select pin automatically
 * (dio2-tx-enable on lora0). This file owns only the pins DIO2 does not:
 *   GPIO7   VFEM power   (LDO enable, active-high)
 *   GPIO2   CSD          (chip enable; also the detect line)
 *   GPIO46  GC1109  CPS  (PA mode: HIGH = full PA; "don't care" in RX)
 *   GPIO5   KCT8103L CTX  (HIGH = TX full-PA / RX-bypass; LOW = RX-LNA)
 *
 * INTERIM: the mode pin is set statically HIGH after detection. That is fully
 * correct for the GC1109, and gives working full-PA TX on the KCT8103L, but the
 * KCT8103L's RX LNA stays inactive (RX runs in bypass) — enabling it needs CTX
 * driven LOW *only during RX*. Dynamic per-TX/RX control (a radio hook, like the
 * reference firmware's setTx/RxModeEnable) is the follow-up for that extra RX
 * sensitivity. See ./README.md.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(heltec_v4_fem, LOG_LEVEL_INF);

/* ESP32-S3 GPIO bank mapping: 0-31 -> gpio0, 32-63 -> gpio1 (pin = GPIO - 32). */
#define FEM_POWER_PIN       7  /* gpio0  — VFEM LDO enable                     */
#define FEM_CSD_PIN         2  /* gpio0  — CSD chip-enable / FEM-detect line   */
#define FEM_GC1109_CPS_PIN 14  /* gpio1  — GPIO46, GC1109 PA-mode select       */
#define FEM_KCT_CTX_PIN     5  /* gpio0  — GPIO5,  KCT8103L RX/TX mode select  */

static int heltec_v4_fem_init(void)
{
	const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	const struct device *const gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	int csd;

	if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) {
		LOG_ERR("GPIO controllers not ready; LoRa FEM left unconfigured");
		return -ENODEV;
	}

	/* 1. Power the FEM LDO rail; let it settle (5 ms, matching firmware). */
	gpio_pin_configure(gpio0, FEM_POWER_PIN, GPIO_OUTPUT_HIGH);
	k_busy_wait(5000);

	/* 2. Detect the fitted FEM: read CSD as a (bias-free) input. */
	gpio_pin_configure(gpio0, FEM_CSD_PIN, GPIO_INPUT);
	k_busy_wait(1000);
	csd = gpio_pin_get_raw(gpio0, FEM_CSD_PIN);
	if (csd < 0) {
		LOG_ERR("FEM detect read failed (%d); LoRa RF may be non-functional", csd);
		return csd;
	}

	/* 3. Enable the FEM (CSD high) and set the detected front-end's mode pin. */
	gpio_pin_configure(gpio0, FEM_CSD_PIN, GPIO_OUTPUT_HIGH);

	if (csd == 1) {
		LOG_INF("Detected KCT8103L LoRa FEM (V4 rev 4.3 / R8)");
		gpio_pin_configure(gpio0, FEM_KCT_CTX_PIN, GPIO_OUTPUT_HIGH);
		LOG_INF("LoRa FEM: full-PA TX enabled; RX in bypass (LNA inactive) — "
			"static CTX; dynamic RX-LNA is a future improvement");
	} else {
		LOG_INF("Detected GC1109 LoRa FEM (V4 rev 4.2)");
		gpio_pin_configure(gpio1, FEM_GC1109_CPS_PIN, GPIO_OUTPUT_HIGH);
		LOG_INF("LoRa FEM: full-PA TX path enabled");
	}

	return 0;
}

/* After the GPIO drivers (PRE_KERNEL) and before the LoRa driver
 * (POST_KERNEL, CONFIG_LORA_INIT_PRIORITY = 90) so the FEM rail is up and the
 * front-end enabled before the SX1262 is first used.
 */
SYS_INIT(heltec_v4_fem_init, POST_KERNEL, 80);
