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

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/meshtastic/fem.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/gnss_pps.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

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

/*
 * Transmit gain of each fitted front-end, in dB, indexed by the SX1262 drive
 * level in dBm (index 0 == drive 0 dBm). Straight from the reference firmware
 * (firmware/src/mesh/LoRaFEMInterface.cpp, powerConversion()) — the gain falls
 * off at the top because the PA compresses.
 *
 * These are what make `tx_power` mean power at the ANTENNA, as it does on stock
 * firmware: a request of 20 dBm on a KCT8103L board becomes drive level 7,
 * because 7 + 13 = 20. Programming the request straight into the transceiver
 * instead — which this port did until 2026-08-19 — radiates the FEM's gain on
 * top of it, i.e. ~11 dB hot at any setting below the maximum.
 */
static const uint8_t fem_gain_gc1109[] = {
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 7,
};
static const uint8_t fem_gain_kct8103l[] = {
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7,
};

/* The detected front-end's gain table, or NULL when detection failed (in which
 * case the conversion below degrades to the identity — the same behaviour as a
 * board with no FEM at all, and the safer direction to fail in: it under-drives
 * rather than over-radiates). */
static const uint8_t *fem_gain_table;
static size_t fem_gain_len;

/*
 * Whether the fitted part lets software choose the receive path, and which path
 * is currently selected.
 *
 * Only the KCT8103L (rev 4.3 / R8) exposes this: its CTX pin selects LNA vs
 * bypass on receive, which is exactly what config.lora.fem_lna_mode controls.
 * On the GC1109 (rev 4.2) the mode pin is a transmit PA-mode select and there
 * is no receive-path choice, so that board reports "cannot control" and the
 * config store normalizes the stored value to NOT_PRESENT — the reference does
 * the same rather than leave a setting that quietly does nothing.
 */
static bool fem_lna_controllable;
static bool fem_lna_enabled = true;

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
		fem_gain_table = fem_gain_kct8103l;
		fem_gain_len = ARRAY_SIZE(fem_gain_kct8103l);
		fem_lna_controllable = true; /* CTX selects RX LNA vs RX bypass */
	} else {
		LOG_INF("Detected GC1109 LoRa FEM (V4 rev 4.2)");
		fem_mode_port = gpio1;
		fem_mode_pin = FEM_GC1109_CPS_PIN;
		fem_gain_table = fem_gain_gc1109;
		fem_gain_len = ARRAY_SIZE(fem_gain_gc1109);
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

	/* HIGH for TX. On RX the level is the receive-path select: LOW engages the
	 * KCT8103L's LNA, HIGH bypasses it. Honour the configured mode there rather
	 * than always assuming the LNA, so fem_lna_mode is a real control and not
	 * decoration. ("Don't care" on the GC1109, which has no RX path choice.) */
	gpio_pin_set_raw(fem_mode_port, fem_mode_pin,
			 tx ? 1 : (fem_lna_enabled ? 0 : 1));
}

bool meshtastic_radio_fem_lna_can_control(void)
{
	return fem_lna_controllable;
}

/*
 * Report the INTENT, never the pin. meshtastic_radio_fem_set_tx() above drives
 * this one pin HIGH for transmit and back to the configured receive level after,
 * so sampling the pad during a TX would say "bypass" on a node whose LNA is
 * enabled. fem_lna_enabled is what the receive path actually returns to, which
 * is the answer that holds at every instant.
 */
bool meshtastic_radio_fem_lna_get(void)
{
	return fem_lna_enabled;
}

/*
 * The detected part, not a compile-time guess: one image serves both PCB
 * revisions and detection can fail (see heltec_v4_fem_init). NULL here means
 * detection did not complete, which matters — the gain table degrades to
 * identity in that case and transmit power is quietly under-driven.
 */
const char *meshtastic_radio_fem_name(void)
{
	if (fem_gain_table == fem_gain_kct8103l) {
		return "KCT8103L";
	}
	if (fem_gain_table == fem_gain_gc1109) {
		return "GC1109";
	}
	return NULL;
}

void meshtastic_radio_fem_lna_set(bool enable)
{
	fem_lna_enabled = enable;

	/* Apply immediately if we are sitting in receive, so the change takes
	 * effect without waiting for the next transmit to cycle the pin. */
	if (fem_mode_port != NULL) {
		gpio_pin_set_raw(fem_mode_port, fem_mode_pin, enable ? 0 : 1);
	}
	LOG_INF("LoRa FEM receive path: %s", enable ? "LNA" : "bypass");
}

/*
 * Strong override of the radio layer's weak power-conversion hook: turn the
 * operator's requested antenna power into an SX1262 drive level, using the
 * detected front-end's gain table. The caller clamps the result to the radio's
 * settable range.
 */
int8_t meshtastic_radio_fem_tx_power_conversion(int8_t radiated_dbm)
{
	return meshtastic_fem_gain_convert(fem_gain_table, fem_gain_len, radiated_dbm);
}

/* After the GPIO drivers (PRE_KERNEL) and before the LoRa driver
 * (POST_KERNEL, CONFIG_LORA_INIT_PRIORITY = 90) so the FEM rail is up and the
 * front-end enabled before the SX1262 is first used.
 */
SYS_INIT(heltec_v4_fem_init, POST_KERNEL, 80);

/*
 * ---------------------------------------------------------------------------
 * L76K GNSS control — board-aware pin map.
 *
 * Octal PSRAM on the R8 claims GPIO33-37 for the wide MSPI bus, so the whole GNSS
 * control map moved vs the plain V4, and two of the lines were dropped entirely:
 *
 *              V4                     V4 R8
 *   EN        GPIO34 (gpio1.2)        GPIO42 (gpio1.10)      active-low
 *   STANDBY   GPIO40 (gpio1.8)        — dropped (GPIO40 is Vext on the R8!)
 *   RESET     GPIO42 (gpio1.10)       — removed
 *
 * EN (and STANDBY, on the V4) are held at their run state by gpio-hogs in the board
 * .dts; this file only *pulses RESET at boot* where a reset line exists, plus the
 * `gps` bench CLI. On the R8 there is no reset line — and GPIO42 is now the ENABLE
 * line, so the V4's boot reset pulse would drive it and disturb the hog — and GPIO40
 * is the Vext power rail, so driving it as "standby" would power-cycle the OLED/GPS.
 * Both the reset pulse and the standby control are therefore compiled out for the R8.
 * ---------------------------------------------------------------------------
 */
#if defined(CONFIG_BOARD_HELTEC_WIFI_LORA32_V4_R8_ESP32S3_PROCPU)
#define GPS_EN_PIN      10 /* gpio1 — GPIO42, GPS_EN, active-low (0 = enabled) */
#define GPS_HAS_STANDBY  0 /* GPIO40 is Vext on the R8 — never drive it as standby */
#define GPS_HAS_RESET    0 /* R8 has no GNSS reset line (GPIO42 is now the enable)  */
#else /* plain V4 (rev 4.2 / 4.3) */
#define GPS_EN_PIN       2 /* gpio1 — GPIO34, GPS_EN, active-low (0 = enabled) */
#define GPS_STANDBY_PIN  8 /* gpio1 — GPIO40, high = force wake                */
#define GPS_RESET_PIN   10 /* gpio1 — GPIO42, L76K reset, active-low           */
#define GPS_HAS_STANDBY  1
#define GPS_HAS_RESET    1
#endif

#if GPS_HAS_RESET
/*
 * L76K GNSS reset pulse — mirrors firmware/src/gps/GPS.cpp, which resets the L76K
 * at boot for a clean, known start. The DT holds EN low (enabled) and STANDBY high
 * (awake) via gpio-hogs; RESET is active-low on GPIO42 (gpio1.10): assert low
 * >=100 ms, then release. The DT hog for this pin was removed so we can pulse it.
 * Runs after the GPIO drivers and before/around the GNSS driver reading uart1.
 */
static int heltec_v4_gps_reset(void)
{
	const struct device *const gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(gpio1)) {
		LOG_ERR("GPIO1 not ready; GPS reset pulse skipped");
		return -ENODEV;
	}

	gpio_pin_configure(gpio1, GPS_RESET_PIN, GPIO_OUTPUT);
	gpio_pin_set_raw(gpio1, GPS_RESET_PIN, 0); /* assert reset (active-low) */
	k_msleep(120);                             /* >=100 ms per L76K datasheet */
	gpio_pin_set_raw(gpio1, GPS_RESET_PIN, 1); /* release */
	k_msleep(250);                             /* let the module boot + start NMEA */
	LOG_INF("GPS: pulsed L76K reset (GPIO42)");

	return 0;
}

SYS_INIT(heltec_v4_gps_reset, POST_KERNEL, 85);
#endif /* GPS_HAS_RESET — on the R8 the gps_en_hog holds GPS enabled; no reset line */

/*
 * ---------------------------------------------------------------------------
 * `gps` bench CLI — live L76K control for GNSS bring-up debugging.
 *
 * A hands-on toolkit for bringing up / diagnosing the L76K on the bench: power-
 * cycle, reset, wake, retune the uart1 baud, inject raw $PCAS/$PMTK lines, and
 * (with `gps pinmux`) dump the decoded IO_MUX / GPIO-matrix / pad state. These
 * exist because getting this GNSS talking took a long hunt — the actual bug was
 * swapped uart1 RX/TX pins (see the pinctrl file), which `gps pinmux` + a stock-
 * firmware serial log finally exposed. Every L76K control pin lives on this board,
 * so the CLI lives here too — no coupling to the portable sample.
 *
 *   gps status            EN/STANDBY intent + uart1 baud
 *   gps on | off          EN (GPIO34, active-low): power the module on/off
 *   gps wake | sleep      STANDBY (GPIO40, high = force wake)
 *   gps reset             pulse RESET (GPIO42) low >=100 ms
 *   gps baud <rate>       retune uart1 to <rate> 8N1 (RX resumes at new rate)
 *   gps raw <text...>     send one CRLF-terminated line to the module
 * ---------------------------------------------------------------------------
 */
#if defined(CONFIG_SHELL)

/* GPS_EN_PIN / GPS_STANDBY_PIN / GPS_HAS_* come from the board-aware block above. */

/* Last-commanded physical levels, seeded to the gpio-hog boot defaults so
 * `gps status` is truthful before any command (reading back an output pin is
 * driver-dependent on ESP32, so intent is tracked rather than sampled). */
static int gps_en_level = 0;      /* enabled */
#if GPS_HAS_STANDBY
static int gps_standby_level = 1; /* awake   */
#endif

static const struct device *gps_gpio1(void)
{
	const struct device *const gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	return device_is_ready(gpio1) ? gpio1 : NULL;
}

static const struct device *gps_uart(void)
{
	const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

	return device_is_ready(uart) ? uart : NULL;
}

static int gps_drive(const struct shell *sh, unsigned int pin, int raw, int *track,
		     const char *msg)
{
	const struct device *gpio1 = gps_gpio1();

	if (gpio1 == NULL) {
		shell_error(sh, "gpio1 not ready");
		return -ENODEV;
	}

	gpio_pin_configure(gpio1, pin, GPIO_OUTPUT);
	gpio_pin_set_raw(gpio1, pin, raw);
	if (track != NULL) {
		*track = raw;
	}
	shell_print(sh, "%s", msg);
	return 0;
}

static int cmd_gps_on(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return gps_drive(sh, GPS_EN_PIN, 0, &gps_en_level, "EN low: module powered on");
}

static int cmd_gps_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return gps_drive(sh, GPS_EN_PIN, 1, &gps_en_level, "EN high: module powered off");
}

static int cmd_gps_wake(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
#if GPS_HAS_STANDBY
	return gps_drive(sh, GPS_STANDBY_PIN, 1, &gps_standby_level, "STANDBY high: awake");
#else
	shell_error(sh, "no STANDBY line on this board (GPIO40 is Vext on the R8)");
	return -ENOTSUP;
#endif
}

static int cmd_gps_sleep(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
#if GPS_HAS_STANDBY
	return gps_drive(sh, GPS_STANDBY_PIN, 0, &gps_standby_level, "STANDBY low: asleep");
#else
	shell_error(sh, "no STANDBY line on this board (GPIO40 is Vext on the R8)");
	return -ENOTSUP;
#endif
}

static int cmd_gps_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
#if GPS_HAS_RESET
	int ret = heltec_v4_gps_reset();

	if (ret < 0) {
		shell_error(sh, "reset failed: %d", ret);
		return ret;
	}
	shell_print(sh, "pulsed RESET (GPIO42)");
	return 0;
#else
	shell_error(sh, "no GNSS reset line on this board (R8: GPIO42 is the enable)");
	return -ENOTSUP;
#endif
}

static int cmd_gps_baud(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *uart = gps_uart();
	struct uart_config cfg;
	uint32_t rate;
	char *end;
	int ret;

	if (argc != 2) {
		shell_error(sh, "usage: gps baud <rate>");
		return -EINVAL;
	}
	if (uart == NULL) {
		shell_error(sh, "uart1 not ready");
		return -ENODEV;
	}

	rate = (uint32_t)strtoul(argv[1], &end, 10);
	if (*end != '\0' || rate == 0U) {
		shell_error(sh, "invalid rate: %s", argv[1]);
		return -EINVAL;
	}

	ret = uart_config_get(uart, &cfg);
	if (ret < 0) {
		shell_error(sh, "uart_config_get failed: %d", ret);
		return ret;
	}
	cfg.baudrate = rate;
	ret = uart_configure(uart, &cfg);
	if (ret < 0) {
		shell_error(sh, "uart_configure failed: %d", ret);
		return ret;
	}
	/* Reconfigure can drop the RX IRQ the modem backend armed at pipe-open;
	 * re-arm it so the GNSS driver keeps receiving at the new rate. */
	uart_irq_rx_enable(uart);
	shell_print(sh, "uart1 baud -> %u (watch for gnss_dump lines)", rate);
	return 0;
}

static int cmd_gps_raw(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *uart = gps_uart();

	if (argc < 2) {
		shell_error(sh, "usage: gps raw <text...>  (CRLF appended)");
		return -EINVAL;
	}
	if (uart == NULL) {
		shell_error(sh, "uart1 not ready");
		return -ENODEV;
	}

	for (size_t i = 1; i < argc; i++) {
		if (i > 1) {
			uart_poll_out(uart, ' ');
		}
		for (const char *p = argv[i]; *p != '\0'; p++) {
			uart_poll_out(uart, (unsigned char)*p);
		}
	}
	uart_poll_out(uart, '\r');
	uart_poll_out(uart, '\n');
	shell_print(sh, "sent");
	return 0;
}

/*
 * Send a checksummed NMEA/CASIC sentence built in firmware, so no '*' has to
 * survive the shell (Zephyr's SHELL_WILDCARD eats an argument containing '*',
 * which is exactly the NMEA checksum delimiter). Type the body only — the '$'
 * prefix and the '*<XX>' XOR checksum are added here:
 *   gps nmea PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0   -> $PCAS03,...*02\r\n
 */
static int cmd_gps_nmea(const struct shell *sh, size_t argc, char **argv)
{
	static const char hex[] = "0123456789ABCDEF";
	const struct device *uart = gps_uart();
	uint8_t csum = 0;

	if (argc < 2) {
		shell_error(sh, "usage: gps nmea <body>  ($ prefix + *checksum added)");
		return -EINVAL;
	}
	if (uart == NULL) {
		shell_error(sh, "uart1 not ready");
		return -ENODEV;
	}

	uart_poll_out(uart, '$');
	for (size_t i = 1; i < argc; i++) {
		const char *p = argv[i];

		if (i == 1 && *p == '$') {
			p++; /* tolerate a leading '$' in the typed body */
		}
		if (i > 1) {
			csum ^= (uint8_t)' ';
			uart_poll_out(uart, ' ');
		}
		for (; *p != '\0'; p++) {
			csum ^= (uint8_t)*p;
			uart_poll_out(uart, (unsigned char)*p);
		}
	}
	uart_poll_out(uart, '*');
	uart_poll_out(uart, hex[(csum >> 4) & 0x0F]);
	uart_poll_out(uart, hex[csum & 0x0F]);
	uart_poll_out(uart, '\r');
	uart_poll_out(uart, '\n');
	shell_print(sh, "sent $...*%02X", csum);
	return 0;
}

static int cmd_gps_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *uart = gps_uart();
	struct uart_config cfg;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if GPS_HAS_STANDBY
	shell_print(sh, "EN(GPIO34)=%d [0=on]  STANDBY(GPIO40)=%d [1=wake]  (last commanded)",
		    gps_en_level, gps_standby_level);
#else
	shell_print(sh, "EN(GPIO42)=%d [0=on]  (R8: no STANDBY/RESET lines)  (last commanded)",
		    gps_en_level);
#endif
	if (uart != NULL && uart_config_get(uart, &cfg) == 0) {
		shell_print(sh, "uart1 baud=%u", cfg.baudrate);
	} else {
		shell_print(sh, "uart1 not ready");
	}
	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * `gps pinmux` — low-level dump of every GPS pin's real hardware state, decoded.
 *
 * Reads the ESP32-S3 IO_MUX + GPIO-matrix + pad registers directly so we can
 * confirm the L76K pins are wired to UART1 exactly as intended (the config
 * layer, DT/pinctrl, can look right while the silicon disagrees). Key subtlety:
 * an OUTPUT pin has its input buffer (FUN_IE) off, so a plain GPIO_IN read of it
 * returns 0 regardless of the true pad level — so for the level column we
 * momentarily set FUN_IE, sample, and restore. Columns: IO_MUX func (GPIO vs a
 * peripheral/JTAG alt), pull, input-enable, drive; the GPIO-matrix out-signal
 * routed to the pad; and the true pad level.
 * ---------------------------------------------------------------------------
 */
#define GPS_GPIO_IN1_REG     0x60004040U           /* pad input, GPIO32..48 (bit = gpio-32) */
#define GPS_GPIO_OUTSEL_BASE 0x60004554U           /* + 4*gpio : matrix out-signal for a pad */
#define GPS_U1RXD_INSEL_REG  0x60004190U           /* GPIO_FUNC15_IN_SEL (U1RXD sig=15)      */
#define GPS_UART1_CONF0_REG  0x60010020U
#define GPS_SIG_GPIO_OUT     256U                  /* out_sel value meaning "plain GPIO out" */
#define GPS_U1TXD_SIG        15U

static uint32_t gps_iomux_reg(uint8_t gpio)
{
	switch (gpio) {
	case 34:
		return 0x6000908CU; /* EN      */
	case 38:
		return 0x6000909CU; /* ESP TX -> module RX */
	case 39:
		return 0x600090A0U; /* ESP RX <- module TX (MTCK) */
	case 40:
		return 0x600090A4U; /* STANDBY (MTDO) */
	case 41:
		return 0x600090A8U; /* PPS     (MTDI) — GNSS 1PPS, input only */
	case 42:
		return 0x600090ACU; /* RESET   (MTMS) */
	default:
		return 0U;
	}
}

static int gps_pad_level(uint8_t gpio, uint32_t iomux)
{
	uint32_t save = sys_read32(iomux);
	int lvl;

	sys_write32(save | BIT(9), iomux); /* FUN_IE: enable input buffer to read true level */
	lvl = (int)((sys_read32(GPS_GPIO_IN1_REG) >> (gpio - 32U)) & 1U);
	sys_write32(save, iomux);          /* restore */
	return lvl;
}

static void gps_dump_pin(const struct shell *sh, const char *name, uint8_t gpio)
{
	uint32_t m = sys_read32(gps_iomux_reg(gpio));
	uint32_t out = sys_read32(GPS_GPIO_OUTSEL_BASE + (4U * gpio)) & 0x1ffU;
	const char *func = (((m >> 12) & 0x7U) == 1U) ? "GPIO" : "ALT(jtag/periph)";
	const char *pull = ((m >> 8) & 1U) ? "up" : (((m >> 7) & 1U) ? "down" : "none");
	const char *outn = (out == GPS_SIG_GPIO_OUT) ? " (plain-GPIO)"
			   : (out == GPS_U1TXD_SIG)   ? " (U1TXD)"
						      : "";

	shell_print(sh,
		    "GPIO%u %-8s iomux=0x%04x func=%-16s pull=%-4s ie=%u drv=%u  out_sig=%u%s  pad=%d",
		    gpio, name, m, func, pull, (unsigned)((m >> 9) & 1U),
		    (unsigned)((m >> 10) & 3U), (unsigned)out, outn, gps_pad_level(gpio, gps_iomux_reg(gpio)));
}

static int cmd_gps_pinmux(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t insel = sys_read32(GPS_U1RXD_INSEL_REG);
	uint32_t c0 = sys_read32(GPS_UART1_CONF0_REG);
	unsigned databits = ((c0 >> 2) & 3U) + 5U;
	unsigned stopbits = (c0 >> 4) & 3U;
	bool parity = ((c0 >> 1) & 1U) != 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_BOARD_HELTEC_WIFI_LORA32_V4_R8_ESP32S3_PROCPU)
	gps_dump_pin(sh, "EN", 42);
	gps_dump_pin(sh, "TX", 38);
	gps_dump_pin(sh, "RX", 39);
	gps_dump_pin(sh, "PPS", 41);
#else
	gps_dump_pin(sh, "EN", 34);
	gps_dump_pin(sh, "TX", 38);
	gps_dump_pin(sh, "RX", 39);
	gps_dump_pin(sh, "STANDBY", 40);
	gps_dump_pin(sh, "RESET", 42);
	gps_dump_pin(sh, "PPS", 41);
#endif
	shell_print(sh, "U1RXD in : src_gpio=%u matrix_en=%u  (want gpio=39 en=1)",
		    (unsigned)(insel & 0x3fU), (unsigned)((insel >> 7) & 1U));
	shell_print(sh, "UART1    : conf0=0x%08x  %u%c%u  loopback=%u", c0, databits,
		    parity ? '?' : 'N', stopbits ? stopbits : 1U, (unsigned)((c0 >> 14) & 1U));
	shell_print(sh, "want: EN pad=0, TX/STBY/RESET pad=1, RX pad=1, TX out_sig=15, 8N1");
	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * `gps pps` — the bench view of the 1PPS capture.
 *
 * The capture itself is NOT here any more. It moved to meshtastic_gnss_pps.c
 * once the pin gained a devicetree description, because at that point nothing
 * about it is board-specific: any board with a meshtastic,gnss-pps node gets the
 * same capture, and one without compiles it out. What stays here is the bench
 * CLI, next to the rest of the L76K controls it is used alongside.
 *
 * The pulse is also a TIME SOURCE now, not only a diagnostic — the wall clock
 * anchors on its edge instead of on the NMEA sentence's arrival — so the capture
 * arms itself at init. `start` and `stop` remain for bench work; `stop` really
 * does stop disciplining the clock.
 *
 *   gps pps                 count, rate, min/max interval, jitter, lock state
 *   gps pps start [quiet]   re-arm and reset the statistics
 *   gps pps stop            disarm
 * ---------------------------------------------------------------------------
 */
#define GPS_PPS_GPIO 41 /* for the human-readable output only; the pin is in DT */

static int cmd_gps_pps_start(const struct shell *sh, size_t argc, char **argv)
{
	bool log_edges = !(argc == 2 && strcmp(argv[1], "quiet") == 0);
	int ret = meshtastic_gnss_pps_start(log_edges);

	if (ret < 0) {
		shell_error(sh, "arming the 1PPS capture failed: %d", ret);
		return ret;
	}
	shell_print(sh, "PPS capture armed on GPIO%u (rising edge)%s", GPS_PPS_GPIO,
		    log_edges ? "" : ", per-edge log off");
	shell_print(sh, "cycle clock = %u Hz; expect ~1 edge/s if the module drives PPS",
		    (unsigned int)sys_clock_hw_cycles_per_sec());
	return 0;
}

static int cmd_gps_pps_stop(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_gnss_pps_stats st;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_gnss_pps_get_stats(&st);
	ret = meshtastic_gnss_pps_stop();
	if (ret == -EALREADY) {
		shell_print(sh, "PPS capture not armed");
		return 0;
	}
	if (ret < 0) {
		shell_error(sh, "disarm failed: %d", ret);
		return ret;
	}
	shell_warn(sh, "PPS capture disarmed (%u edges) — the clock will fall back to "
		       "NMEA arrival", st.count);
	return 0;
}

static int cmd_gps_pps(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_gnss_pps_stats st;
	uint32_t hz = sys_clock_hw_cycles_per_sec();
	int64_t edge_ms, age_ms;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_gnss_pps_get_stats(&st);

	shell_print(sh, "GPIO%u (PPS): armed=%s  locked=%s  cycle clock=%u Hz", GPS_PPS_GPIO,
		    st.armed ? "yes" : "no", st.locked ? "yes" : "no", (unsigned int)hz);
	shell_print(sh, "edges: %u", st.count);

	if (!st.armed) {
		shell_print(sh, "arm it with `gps pps start` first");
		return 0;
	}
	if (st.intervals == 0U) {
		/* A zero count is NOT proof the pin is dead: most receivers gate PPS
		 * on a position fix. Say so, rather than let silence read as a verdict. */
		shell_warn(sh, "no interval yet — need >=2 edges");
		shell_print(sh, "0 edges is inconclusive without a fix: most receivers only");
		shell_print(sh, "drive PPS once they have one. The `gnsst` log line carries");
		shell_print(sh, "fix= and sats=; overlay-gnssdump.conf adds per-satellite SNR.");
		return 0;
	}

	{
		uint32_t mean = (uint32_t)(st.sum_cyc / st.intervals);

		shell_print(sh, "interval: mean %u us   min %u us   max %u us   (n=%u)",
			    (unsigned int)k_cyc_to_us_near32(mean),
			    (unsigned int)k_cyc_to_us_near32(st.min_cyc),
			    (unsigned int)k_cyc_to_us_near32(st.max_cyc), st.intervals);
		shell_print(sh, "jitter (max-min): %u us",
			    (unsigned int)k_cyc_to_us_near32(st.max_cyc - st.min_cyc));
		/* From the SUM, not last-first: those are 32-bit cycle stamps and their
		 * difference wraps every 17.9 s at 240 MHz. Caught on the bench
		 * reporting a 56-edge run as a 2.3 s window. */
		shell_print(sh, "window: %llu us over %u intervals",
			    (unsigned long long)(st.sum_cyc / (hz / 1000000U)), st.intervals);
	}

	if (meshtastic_gnss_pps_last_edge(&edge_ms, &age_ms)) {
		shell_print(sh, "last edge %lld ms ago — USABLE as a clock anchor",
			    (long long)age_ms);
	} else {
		shell_warn(sh, "no usable edge right now: the clock falls back to NMEA "
			       "arrival (%d ms assumed)",
			   CONFIG_MESHTASTIC_GNSS_FIX_LATENCY_MS);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(gps_pps_cmds,
			       SHELL_CMD(start, NULL,
					 "Re-arm the 1PPS capture: gps pps start [quiet].",
					 cmd_gps_pps_start),
			       SHELL_CMD(stop, NULL, "Disarm the 1PPS capture.",
					 cmd_gps_pps_stop),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	gps_cmds, SHELL_CMD(status, NULL, "Show EN/STANDBY intent + uart1 baud.", cmd_gps_status),
	SHELL_CMD(pinmux, NULL, "Dump decoded IO_MUX/matrix/pad state of the GPS pins.",
		  cmd_gps_pinmux),
	SHELL_CMD(on, NULL, "Power the module ON (EN low).", cmd_gps_on),
	SHELL_CMD(off, NULL, "Power the module OFF (EN high).", cmd_gps_off),
	SHELL_CMD(wake, NULL, "Force wake (STANDBY high).", cmd_gps_wake),
	SHELL_CMD(sleep, NULL, "Allow standby (STANDBY low).", cmd_gps_sleep),
	SHELL_CMD(reset, NULL, "Pulse RESET (GPIO42) low >=100 ms.", cmd_gps_reset),
	SHELL_CMD(baud, NULL, "Retune uart1: gps baud <rate>.", cmd_gps_baud),
	SHELL_CMD(raw, NULL, "Send a line to the module: gps raw <text...>.", cmd_gps_raw),
	SHELL_CMD(nmea, NULL, "Send $<body>*<cksum>: gps nmea <body> (no $/*).", cmd_gps_nmea),
	SHELL_CMD(pps, &gps_pps_cmds, "1PPS (GPIO41) edge counter: gps pps [start|stop].",
		  cmd_gps_pps),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(gps, &gps_cmds, "Heltec V4 L76K GNSS bench control", NULL);


#endif /* CONFIG_SHELL */
