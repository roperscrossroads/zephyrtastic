/*
 * Heltec WiFi LoRa 32 V4 — RF front-end (FEM) bring-up with runtime detection.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One firmware image for both V4 PCB revisions (and R8). Mirrors the reference
 * firmware (firmware/src/mesh/LoRaFEMInterface.cpp): power the FEM LDO rail, then
 * read the CSD line as an input — the two front-ends bias it differently, so
 * HIGH => KCT8103L (rev 4.3 / R8) and LOW => GC1109 (rev 4.2) — then enable the
 * detected FEM and take over its PA/LNA mode pin.
 *
 * The SX1262's DIO2 drives the FEM TX/RX *path*-select pin automatically
 * (dio2-tx-enable on lora0). This file owns the pins DIO2 does not:
 *   GPIO7   VFEM power   (LDO enable, active-high)
 *   GPIO2   CSD          (chip enable; also the detect line)
 *   GPIO46  GC1109  CPS  (PA mode)    | the "mode pin": HIGH on TX, LOW on RX
 *   GPIO5   KCT8103L CTX  (RX/TX mode) | (see meshtastic_radio_fem_set_tx below)
 *
 * The mode pin is driven dynamically with the transceiver via the radio layer's
 * FEM hook (<zephyr/meshtastic/fem.h>): HIGH just before TX, LOW on return to
 * RX. For the GC1109 the RX level is "don't care" (harmless); for the KCT8103L
 * the RX-LOW is what engages the ~1.9 dB-NF LNA. This matches the reference
 * firmware's setTx/RxModeEnable. See ./README.md.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/meshtastic/fem.h>

LOG_MODULE_REGISTER(heltec_v4_fem, LOG_LEVEL_INF);

/* ESP32-S3 GPIO bank mapping: 0-31 -> gpio0, 32-63 -> gpio1 (pin = GPIO - 32). */
#define FEM_POWER_PIN       7  /* gpio0  — VFEM LDO enable                     */
#define FEM_CSD_PIN         2  /* gpio0  — CSD chip-enable / FEM-detect line   */
#define FEM_GC1109_CPS_PIN 14  /* gpio1  — GPIO46, GC1109 PA-mode select       */
#define FEM_KCT_CTX_PIN     5  /* gpio0  — GPIO5,  KCT8103L RX/TX mode select  */

/*
 * The detected FEM's mode pin, captured at init and driven per TX/RX by the
 * radio hook. mode_port == NULL means "no FEM configured" (detect failed) —
 * the hook then does nothing.
 */
static const struct device *fem_mode_port;
static gpio_pin_t fem_mode_pin;

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

	/* 3. Enable the FEM (CSD high) and record the detected mode pin. It starts
	 *    in the RX state (LOW) — the KCT8103L LNA is then active by default;
	 *    the radio hook raises it only during TX.
	 */
	gpio_pin_configure(gpio0, FEM_CSD_PIN, GPIO_OUTPUT_HIGH);

	if (csd == 1) {
		LOG_INF("Detected KCT8103L LoRa FEM (V4 rev 4.3 / R8)");
		fem_mode_port = gpio0;
		fem_mode_pin = FEM_KCT_CTX_PIN;
	} else {
		LOG_INF("Detected GC1109 LoRa FEM (V4 rev 4.2)");
		fem_mode_port = gpio1;
		fem_mode_pin = FEM_GC1109_CPS_PIN;
	}

	gpio_pin_configure(fem_mode_port, fem_mode_pin, GPIO_OUTPUT_LOW);
	LOG_INF("LoRa FEM ready: mode pin follows TX/RX (RX-LNA active on KCT8103L)");

	return 0;
}

/*
 * Strong override of the radio layer's weak FEM hook. Drives the detected
 * front-end's mode pin with the transceiver direction: HIGH for full-PA TX,
 * LOW for RX (which enables the KCT8103L LNA; "don't care" for the GC1109).
 */
void meshtastic_radio_fem_set_tx(bool tx)
{
	if (fem_mode_port == NULL) {
		return;
	}

	gpio_pin_set_raw(fem_mode_port, fem_mode_pin, tx ? 1 : 0);
}

/* After the GPIO drivers (PRE_KERNEL) and before the LoRa driver
 * (POST_KERNEL, CONFIG_LORA_INIT_PRIORITY = 90) so the FEM rail is up and the
 * front-end enabled before the SX1262 is first used.
 */
SYS_INIT(heltec_v4_fem_init, POST_KERNEL, 80);
