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

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/meshtastic/fem.h>
#include <zephyr/shell/shell.h>
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

/*
 * L76K GNSS reset pulse — mirrors firmware/src/gps/GPS.cpp, which resets the L76K
 * at boot for a clean, known start. The DT holds EN low (enabled) and STANDBY high
 * (awake) via gpio-hogs; RESET is active-low on GPIO42 (gpio1.10): assert low
 * >=100 ms, then release. The DT hog for this pin was removed so we can pulse it.
 * Runs after the GPIO drivers and before/around the GNSS driver reading uart1.
 */
#define GPS_RESET_PIN 10 /* gpio1 — GPIO42, L76K reset, active-low */

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

#define GPS_EN_PIN      2 /* gpio1 — GPIO34, GPS_EN, active-low (0 = enabled) */
#define GPS_STANDBY_PIN 8 /* gpio1 — GPIO40, high = force wake                */

/* Last-commanded physical levels, seeded to the gpio-hog boot defaults so
 * `gps status` is truthful before any command (reading back an output pin is
 * driver-dependent on ESP32, so intent is tracked rather than sampled). */
static int gps_en_level = 0;      /* enabled */
static int gps_standby_level = 1; /* awake   */

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
	return gps_drive(sh, GPS_STANDBY_PIN, 1, &gps_standby_level, "STANDBY high: awake");
}

static int cmd_gps_sleep(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return gps_drive(sh, GPS_STANDBY_PIN, 0, &gps_standby_level, "STANDBY low: asleep");
}

static int cmd_gps_reset(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = heltec_v4_gps_reset();
	if (ret < 0) {
		shell_error(sh, "reset failed: %d", ret);
		return ret;
	}
	shell_print(sh, "pulsed RESET (GPIO42)");
	return 0;
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

	shell_print(sh, "EN(GPIO34)=%d [0=on]  STANDBY(GPIO40)=%d [1=wake]  (last commanded)",
		    gps_en_level, gps_standby_level);
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

	gps_dump_pin(sh, "EN", 34);
	gps_dump_pin(sh, "TX", 38);
	gps_dump_pin(sh, "RX", 39);
	gps_dump_pin(sh, "STANDBY", 40);
	gps_dump_pin(sh, "RESET", 42);
	shell_print(sh, "U1RXD in : src_gpio=%u matrix_en=%u  (want gpio=39 en=1)",
		    (unsigned)(insel & 0x3fU), (unsigned)((insel >> 7) & 1U));
	shell_print(sh, "UART1    : conf0=0x%08x  %u%c%u  loopback=%u", c0, databits,
		    parity ? '?' : 'N', stopbits ? stopbits : 1U, (unsigned)((c0 >> 14) & 1U));
	shell_print(sh, "want: EN pad=0, TX/STBY/RESET pad=1, RX pad=1, TX out_sig=15, 8N1");
	return 0;
}

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
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(gps, &gps_cmds, "Heltec V4 L76K GNSS bench control", NULL);

#endif /* CONFIG_SHELL */
