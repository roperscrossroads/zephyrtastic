# Heltec WiFi LoRa 32 V4 (rev 4.2) — hardware reference

A cross-checked reference for the board this firmware targets, gathered while
bringing up the on-device UI. Every pin below was confirmed against **three
independent sources**: Heltec's official documentation + V4.2 schematic, the
Heltec Arduino library (`Heltec_ESP32`), and the upstream Meshtastic firmware
(`firmware/variants/esp32s3/heltec_v4`). Where they disagree, it is called out.

> **Scope: plain WiFi LoRa 32 V4, revision 4.2** (ESP32-S3R2, GC1109 FEM). Rev
> 4.3 shares the digital wiring (different FEM); the **V4-R8** and **V3** differ
> in several places — see [Variant differences](#variant-differences). Battery
> specifics have their own sourced writeup in
> [`battery-provenance.md`](battery-provenance.md).

## Confirmed GPIO map

| Function | GPIO | Our devicetree | Cross-checked against |
|---|---|---|---|
| OLED I2C SDA / SCL | 17 / 18 | `i2c0` pinctrl | `pins_arduino.h` `SDA_OLED`/`SCL_OLED` |
| OLED reset | 21 | `ssd1306@3c` `reset-gpios` | `RST_OLED = 21` |
| OLED address / controller / size | 0x3c / SSD1306(-class) / 128×64 | `ssd1306@3c` | lib `SSD1306Wire(0x3c, …, GEOMETRY_128_64)` |
| **Vext** (OLED + peripheral rail) | **36, active-LOW** | `vext_ctrl` power-domain (`GPIO_ACTIVE_LOW`) | lib `VextON()` = `digitalWrite(36, LOW)` |
| USER / PRG button | 0 | `button0` / `sw0` | `Boot_Key 0`, `attachInterrupt(0, …)` |
| LED | 35 | `led0` | `pins_arduino.h` `LED = 35` |
| Battery sense ADC | 1 (ADC1 ch0) | `vbatt` `io-channels` | lib `analogRead(1)`; schematic `ADC_IN` |
| ADC_CTRL (battery-divider gate) | **37, active-HIGH** | `adc_ctrl` power-domain | lib `digitalWrite(37, HIGH)`; schematic `ADC_Ctrl` pin 37 |
| LoRa NSS / SCK / MOSI / MISO | 8 / 9 / 10 / 11 | `spi2` + `lora0` | lib `board-config.h` (identical V3↔V4) |
| LoRa RST / BUSY / DIO1 | 12 / 13 / 14 | `lora0` | lib `board-config.h` |
| FEM power / CSD / TX-mode (4.2 = GC1109) | 7 / 2 / 46 | board C init (`…_v4_fem.c`) | lib `USE_GC1109_PA`: 7 / 2 / 46 |
| GNSS UART (to the connector) | uart1 (RX 39 / TX 38) | `uart1` + `gnss` | lib GPS `Serial1`, RX39/TX38 |
| GNSS enable-standby / reset | 34 / 42 | `gps_standby` hog (per-board `.dts`) | Heltec `VGNSS_Ctrl`/`GNSS_RST` |
| USB serial | native USB-Serial-JTAG (43/44) | `usb_serial` chosen | CP2102 removed on V4 |

## Peripheral notes & bring-up gotchas

### OLED power — the #1 first-boot risk
The 0.96" OLED is driven over I2C, but its rail is switched by **Vext (GPIO36)**.
Our DT declares `vext_ctrl` as **active-LOW** (drive GPIO36 LOW to enable the
3.3 V/250 mA rail), matching Heltec's own V4 factory-test code.

- **Known inconsistency:** two Heltec files (a `VextControl.ino` comment and a
  `BME280basic.ino` example) claim V4 Vext is *active-HIGH*. Every V4 **factory
  test** contradicts them (drives LOW to enable), our DT uses active-LOW, and the
  UI rendered on a bench V4 — so active-LOW is almost certainly right. **If the
  screen stays dark, Vext polarity is suspect #1.**
- The `vext_ctrl` power-domain is inert without `PM_DEVICE`/`POWER_DOMAIN`; the
  hardware-verified render came from a build where the domain is enabled at boot.
  Screen power is coupled to that config, not just the pin.

### Battery
GPIO1 / ADC1 ch0 through a 390 kΩ/100 kΩ divider (×4.9), gated by ADC_CTRL
(GPIO37, high), read at 12-bit / `ADC_ATTEN_DB_12`, with a ×1.045 calibration and
an OCV curve for percent. Full sourcing and the `VERIFY(hardware)` checklist are
in [`battery-provenance.md`](battery-provenance.md). Charging is handled by a
dedicated hardware charge IC — the firmware only reads and displays.

### GNSS — optional pluggable module
GNSS is **not on the V4 board**. The board provides the SH1.25-8Pin GNSS
connector, an enable/standby control GPIO, and the NMEA UART above. A separate
GPS add-on board (**included in some V4 kits**) plugs into that connector —
typically a Quectel **L76K** or compatible NMEA receiver. Our `gnss` node is the
generic NMEA driver, so it works with whatever module is fitted; with no module
connected it simply never reports a fix, which is harmless (the UI's GPS page
shows "No GPS fix").

### RF front-end (FEM)
Rev 4.2 uses the **GC1109** PA/LNA (fixed RX-LNA path, no software control pin
broken out). Rev 4.3 switched to the **KCT8103L** with a control pin on GPIO5.
The board C init auto-detects which FEM is fitted, so one image covers both.
Not relevant to the display, but it owns GPIO7/2/46 on 4.2.

### PSRAM
Plain V4 = **ESP32-S3R2 → 2 MB quad (QSPI)** PSRAM. It is **not enabled by
default** in this port; `overlay-psram.conf` (opt-in) turns it on. See the PSRAM
note in [`cfb-display-ui.md`](cfb-display-ui.md). (The V4-R8 is 8 MB **octal**.)

### USB / flashing
Native USB-Serial-JTAG — the CP2102 bridge was removed on V4 (ignore any Heltec
"install CP210x driver" guidance; that is V3). Manual download mode: **hold PRG,
plug in USB-C, release** (or plug in, hold PRG, tap RST once, release). Relevant
only for a first flash over USB/esptool; subsequent updates go over mcumgr OTA.

## Variant differences

Only rev 4.2 is verified here. Do not assume these carry across the family.

| | V4 rev 4.2 ✅ | V4 rev 4.3 | V4-R8 | V3 |
|---|---|---|---|---|
| Vext | GPIO36, active-low | GPIO36, active-low | **GPIO40** | GPIO21 |
| ADC_CTRL | GPIO37, **HIGH** | GPIO37, HIGH | **removed** (always on) | GPIO37, **LOW** ⚠️ |
| Battery multiplier | 4.9 × **1.045** | 4.9 × 1.045 | 4.9 × **1.035** | 4.9 × 1.045 |
| FEM | GC1109 | KCT8103L (ctrl GPIO5) | KCT8103L | none (bare SX1262) |
| PSRAM | 2 MB quad | 2 MB quad | 8 MB octal | none |
| LED | GPIO35 | GPIO35 | **GPIO46** | GPIO35 |

Battery per-variant detail: [`battery-provenance.md` → Other Heltec variants](battery-provenance.md#other-heltec-variants).

## Pre-flash checklist (V4 rev 4.2)

1. Point `west`'s `manifest.path` at the target worktree before building
   (`./wt manifest ui-test`) — building from the wrong worktree silently
   compiles the *other* worktree's source.
2. First flash over USB/esptool uses download mode (hold PRG, plug USB); OTA
   updates use mcumgr thereafter.
3. **Screen dark?** Check Vext (GPIO36 active-low) and that `POWER_DOMAIN` is
   enabled so the rail comes up at boot.
4. **Battery number off?** Adjust `CONFIG_MESHTASTIC_DISPLAY_BATTERY_CAL_PERMILLE`
   against a multimeter — see the provenance doc's checklist.
5. **No GPS fix?** Expected unless a GPS module is plugged into the connector.

## Sources

- **Heltec official docs** — board pages, FAQs, hardware-update-logs, and the
  `heltec_V4` Arduino variant pin table (docs.heltec.org), plus the manufacturer
  schematic `WiFi_LoRa_32_V4.2.pdf`.
- **Heltec Arduino library** (`Heltec_ESP32`) — `src/heltec.{h,cpp}`,
  `src/driver/board-config.h`, and the `WiFi_LoRa_32_V4_FactoryTest` example.
- **Meshtastic firmware** — `firmware/variants/esp32s3/heltec_v4/variant.h`,
  `firmware/src/Power.{cpp,h}`.
- **This port** — `boards/heltec/heltec_wifi_lora32_v4/`.
