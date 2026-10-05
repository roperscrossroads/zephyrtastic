# Heltec Vision Master E290 (HT-VME290)

Zephyr board for the Heltec **Vision Master E290** — a 2.9" black/white E-Ink dev
board with ESP32-S3, SX1262 LoRa, Wi-Fi and BLE, Meshtastic-compatible.

> **Builds, not yet run.** The meshtastic sample builds for this board in both the net and the
> LoRa+BLE variants (fleet scenario `sample.meshtastic.fleet.e290_ble`), with the octal PSRAM
> enabled and the e-ink screen UI linked. Nothing has run on hardware. Pins are transcribed from the Meshtastic reference
> variant and the Heltec factory test (which agree), but **no E290 hardware has
> been on the bench**. Every `VERIFY(hardware)` in the `.dts` is a real
> unknown — especially the flash size, the e-ink orientation/offset, and the
> battery multiplier.

## What it is (and isn't)

It is a structural cross of the existing Heltec boards:

| Aspect | E290 | Closest sibling |
|---|---|---|
| SoC | ESP32-S3**R8** (8 MB octal PSRAM, enabled by this board's Kconfig.defconfig, as the V4-R8) | V4-R8 |
| Flash | **16 MB** (measured: Winbond W25Q128; stock Meshtastic only uses 8 MB) | V4-R8 |
| Radio | **bare SX1262, no FEM**, 21 dBm | V3 |
| Display | 2.9" SSD1680 e-ink, 296×128, on **SPI3** | (none — new) |
| GNSS | **none** (standard rev) | V3 |
| Console | native USB-Serial-JTAG (no CP2102) | V4 |

Not to be confused with the **E213** (HT-VME213): same silicon, a smaller 2.13"
panel. The **T190** is the colour-TFT sibling.

## Pin map (from `variant.h` + factory test, cross-checked)

| GPIO | Function | | GPIO | Function |
|---|---|---|---|---|
| 0  | Button 1 (PRG/wake) | | 13 | LoRa BUSY |
| 1  | E-Ink MOSI (SPI3)   | | 14 | LoRa DIO1/IRQ |
| 2  | E-Ink SCLK (SPI3)   | | 18 | **Vext EN (active-HIGH)** |
| 3  | E-Ink CS            | | 21 | Button 2 (aux) |
| 4  | E-Ink DC            | | 38 | I2C SCL (QuickLink) |
| 5  | E-Ink RES           | | 39 | I2C SDA (QuickLink) |
| 6  | E-Ink BUSY (act-HIGH)| | 43 | UART0 TX |
| 7  | Battery ADC (ADC1_CH6)| | 44 | UART0 RX |
| 8  | LoRa NSS/CS         | | 45 | LED (unpopulated on early units) |
| 9  | LoRa SCK (SPI2)     | | 46 | **ADC_CTRL (active-HIGH)** |
| 10 | LoRa MOSI           | | 19/20 | USB D-/D+ (native) |
| 11 | LoRa MISO           | | | |
| 12 | LoRa RESET          | | | |

**Vext is active-HIGH here** — the opposite of the V3/V4 (active-low). It powers
the e-ink panel and the QuickLink sensor rail, and is driven on at boot by the
GPIO power-domain (`CONFIG_POWER_DOMAIN[_GPIO]`, forced on).

## Build

```
west build -b heltec_vision_master_e290/esp32s3/procpu <app>
```

From this repo's meshtastic sample (net variant), point `BOARD` at
`heltec_vision_master_e290/esp32s3/procpu` and `BOARD_TAG` at
`heltec-vme290`. The e-ink display driver (`ssd16xx` over MIPI-DBI) is enabled by
the application, not the board defconfig.

## Open items (`VERIFY(hardware)`)

1. ~~Flash size~~ — **16 MB**, measured and on the schematic; the board uses the shared 16 MB map.
2. **E-Ink orientation** (`rotation`) and the **1-byte X RAM offset** the
   Meshtastic driver applies (`DEPG0290BNS800: bufferOffsetX=1`) — an 8-px shift
   is the symptom if it's needed and missing.
3. **Battery multiplier** — variant.h uses 4.9×1.03, the factory test 4.01;
   measure against a known voltage.
4. ~~Button 2 (GPIO21) pull~~ — external 10k pull-up to 3V3 on the schematic (R21).
5. The **SSD1680** compatible is the closest in-tree match to the DEPG0290BNS800;
   confirm full/partial refresh behave on the real panel (waveform is from OTP).

## Display refresh policy

The screen UI's defaults are an OLED's (redraw every second, blank after 30 s). The sample's board conf
sets a 10 s redraw and no blanking, because every redraw of an e-ink panel is a refresh. The Zephyr
`ssd16xx` driver has no X RAM offset, so if the real panel shows the 8-pixel shift (item 2 below) the
driver needs a small carried patch; do not guess it before the panel has been seen.
