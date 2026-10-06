# Heltec Vision Master E290 (HT-VME290)

Zephyr board for the Heltec **Vision Master E290** — a 2.9" black/white E-Ink dev
board with ESP32-S3, SX1262 LoRa, Wi-Fi and BLE, Meshtastic-compatible.

> **Runs on hardware since 2026-10-06.** Boots the meshtastic sample as a LoRa+BLE node (fleet
> scenario `sample.meshtastic.fleet.e290_ble`), with the octal PSRAM, the e-ink screen UI, the
> radio (TX, RX, relaying and PKI exchange proven against other nodes on the air), BLE controller,
> LED, NVS and the bulk store. Pins match the schematic (V0.3.1) on every pin used; three things the
> scaffold had wrong were found on the board and are fixed (see "Bring-up findings"). Still to check:
> battery reading, the button, BLE to a phone, octal PSRAM at 40 MHz over time, and one visual item
> on the panel's header bar.

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
the e-ink panel only (the QuickLink connector is on the always-on 3V3), and is driven on at boot by the
GPIO power-domain (`CONFIG_POWER_DOMAIN[_GPIO]`, forced on).

## Build

```
west build -b heltec_vision_master_e290/esp32s3/procpu <app>
```

From this repo's meshtastic sample (net variant), point `BOARD` at
`heltec_vision_master_e290/esp32s3/procpu` and `BOARD_TAG` at
`heltec-vme290`. The e-ink display driver (`ssd16xx` over MIPI-DBI) is enabled by
the application, not the board defconfig.

## Bring-up findings (2026-10-06)

Three things the scaffold had wrong, each visible only on hardware:

1. **Panel geometry.** The Zephyr `ssd16xx` driver counts `width` along the **gate** lines (296 is an
   SSD1680's maximum) and `height` along the **source** lines (176 max); the reel_board's 250x122
   panel is declared `width 250, height 122`. The DEPG0290BNS800 is 296 gates by 128 sources, so it
   is `width = <296>; height = <128>;` with no rotation. Declared 128 x 296 rotated 90, it was refused
   at init with `Display size out of range` -- and that message only shows with the display driver
   at DEBUG. The image is the right way up with this.
2. **Partial refresh.** With an empty `partial { }` profile every frame after the first goes through
   the partial profile with the controller's OTP waveform in display mode 2, and on this panel that
   left the screen blank right after the first frame had shown. The board declares a `full` profile
   only; a partial LUT that works for this panel is upstream Meshtastic's (its fast-refresh driver
   for the DEPG0290BNS800) and can be carried later.
3. **Refresh life.** The UI renders every `MESHTASTIC_DISPLAY_REFRESH_MS` and CFB writes the whole
   frame to the panel each time; on e-ink that was a full refresh every 10 s with unchanged content,
   and the controller is rated for about a million refreshes. The firmware now has a display shim
   (`src/meshtastic_display_shim.c`): a frame reaches the panel only when it differs from the one on
   the glass, and `MESHTASTIC_DISPLAY_MIN_REFRESH_MS` holds a floor between refreshes. This board asks
   for 30 s in the sample's board conf.

Also found: the VBAT divider was on the wrong ADC unit (`&adc0` channel 6 is hardware ADC1 / GPIO7;
`&adc1` is hardware ADC2), and ADC_CTRL was a power domain held on for the whole boot (about 3 mA);
it is the divider's own `power-gpios` now. The panel's power rail (Vext, active-HIGH) is a GPIO power
domain driven on at init.

### Seeing a driver's init on this board

The console is native USB-Serial/JTAG, which nothing reads while the kernel initializes its devices:
early lines are discarded by the USB driver with no host attached, and a reflash's own reset gives the
same silence. To see a driver's init: build with that driver at DEBUG and a log buffer of about 24 KB
(`CONFIG_LOG_BUFFER_SIZE`), then `kernel reboot warm` **with the console held open** -- the
USB-Serial/JTAG peripheral keeps its link across a CPU reset, and the early lines reach the host.
For the transceiver, `overlay-lora-debug.conf` is the ready-made overlay (a recipe feature, so an
image built with it has its own identity and reports `lora-debug` in its state line).

## Open items (`VERIFY(hardware)`)

1. ~~Flash size~~ -- **16 MB**, measured and on the schematic; the board uses the shared 16 MB map.
2. ~~E-Ink orientation~~ -- 296 x 128, rotation 0 is the right way up. **The 1-byte X RAM offset**
   the Meshtastic driver applies (`DEPG0290BNS800: bufferOffsetX=1`) is still open: the UI's header
   bar shows missing pixels; an 8-px strip at one end of the bar would be this, scattered pixels the
   waveform. The `ssd16xx` driver has no X offset, so if it is needed it is a small carried patch.
3. **Battery multiplier** -- variant.h uses 4.9x1.03, the factory test 4.01; measure against a known
   voltage. `MESHTASTIC_BATTERY` is off in the bring-up build.
4. ~~Button 2 (GPIO21) pull~~ -- external 10k pull-up to 3V3 on the schematic (R21).
5. ~~SSD1680 compatible~~ -- the full refresh from OTP works; the partial does not (finding 2).
6. **The 2x20 header.** Heltec's three documents disagree about header pins 7, 11, 22, 36 and 37
   (schematic: LoRa_RST, DIO1, BOOT, CHIP_PU, Vext; datasheet pin table and pin-map PNG: GPIO7 / NC).
   Firmware cannot settle it; a continuity check can. It matters before the board goes on a
   Raspberry Pi's header (the E290's header follows the Pi's layout: UART and I2C line up, and
   CHIP_PU / BOOT land on Pi GPIO16 / GPIO25).

## Display refresh policy

The screen UI's defaults are an OLED's (redraw every second, blank after 30 s). The sample's board conf
sets a 10 s render interval, no blanking, and a 30 s floor between panel refreshes; with the display
shim a render that changes nothing never reaches the panel, so the 10 s is how soon a change can show,
not how often the panel refreshes.
