# Seeed SenseCAP Indicator D1L — Zephyr board port (HEADLESS, radio-only)

Local (out-of-tree) Zephyr board port for the Seeed SenseCAP Indicator **D1L**.
Upstream Zephyr has no board for this product; it lives in this repo's `boards/`
tree, discovered the same way as the Heltec boards
(`zephyr/module.yml`, `settings.board_root: .`).

> **Status:** builds clean (Zephyr 4.4.99, meshtastic sample, headless — no
> overlay) as of 2026-08-23. **Never flashed to hardware.** Every pin here is
> sourced from vendor/upstream firmware (see [Pin sources](#pin-sources) below),
> not guessed — but "sourced, not guessed" and "bench-verified" are different
> claims. Everything marked `VERIFY(hardware)` in the devicetree needs
> confirming on a real board before this is trustworthy.

---

## Scope: headless, radio-only. What's NOT here.

This board also has an **ST7701 RGB touchscreen** (16 dedicated parallel-data
GPIOs plus VSYNC/HSYNC/DE/PCLK/backlight — effectively all of GPIO0–21), an
**RP2040 co-processor** (talked to over a plain UART, running its own separate
firmware for sensors/UI), an **audio codec**, and a **BMP390 sensor**. None of
that is wired here. This board dir covers exactly what's needed to boot and run
the mesh radio: the SoC, PSRAM/flash, the I2C GPIO expander, and the SX1262
behind it.

Deliberate choice, not an oversight — see
`~/zephyrtastic-wt/docs/FUTURE-BOARD-TARGETS.md` §4 for why the radio path was
prioritized: this is the piece a long-standing, never-actually-fixed upstream
report ("fades out, stops seeing nodes" — GitHub meshtastic/firmware#6672) is
most plausibly rooted in, and the piece worth proving correct on Zephyr's own
architecture before spending effort on the display.

## Pin sources

Nothing in this board's devicetree is guessed. Three independent, cross-checked
sources, all cited inline in the .dts/.dtsi comments:

1. **Seeed's own ESP-IDF reference firmware**
   (`seeed-solution/SenseCAP_Indicator_ESP32`, vendored at
   `~/zephyrtastic-wt/SenseCAP_Indicator_ESP32`) —
   `components/bsp/src/boards/sensecap_indicator_board.c` (I2C bus pins, TCA9535
   expander I2C address) and `components/lora/bsp_sx126x.h` +
   `sx126x_sensecap_board.c` (SPI pins, the expander's combined interrupt pin,
   the radio's NSS/RESET/BUSY/DIO1 expander pin indices, and — critically — the
   *mechanism*: a real edge ISR on the combined INT GPIO, deferred to a task,
   which reads the TCA9535's input-port register over I2C to determine which
   pin fired before dispatching. Not blind polling.).
2. **Upstream Meshtastic** `firmware/variants/esp32s3/seeed-sensecap-indicator/`
   (`variant.h`, `platformio.ini`) — agrees with (1) on every pin and the
   expander address.
3. **Seeed's product page and firmware examples**, for flash/PSRAM size:
   `seeedstudio.com/SenseCAP-Indicator-D1L-p-5646.html` states "D1L (SX1262
   LoRa)"; `examples/indicator_lora/sdkconfig.defaults` and
   `examples/indicator_basis/sdkconfig.defaults` in the vendored repo above both
   show `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y` + `CONFIG_SPIRAM_MODE_OCT=y`.

The one thing this board's files do NOT source from a SenseCAP-specific
document: the console. There's no confirmed evidence of a discrete USB-UART
bridge chip on this board, so it uses the ESP32-S3's native USB-CDC peripheral
(`&usb_serial`) — the same choice `heltec_wifi_lora32_v4` makes for the same
reason ("the V4 has no CP2102"), and one that needs no GPIO pin assignment at
all, which sidesteps the guessing problem entirely rather than resolving it.

## The radio path, briefly

```
ESP32-S3
 ├─ I2C0 (SDA=GPIO39, SCL=GPIO40) ── TCA9535 expander @ 0x20
 │                                    ├─ pin 0: SX1262 NSS  (SPI CS)
 │                                    ├─ pin 1: SX1262 RESET
 │                                    ├─ pin 2: SX1262 BUSY
 │                                    └─ pin 3: SX1262 DIO1 (RX/TX-done IRQ)
 │        combined IRQ ── GPIO42 (edge, real, confirmed routed)
 └─ SPI2  (SCLK=GPIO41, MISO=GPIO47, MOSI=GPIO48) ── SX1262 data
```

The TCA9535 is wired as `compatible = "nxp,pca9539"` — Zephyr's own binding
docs (`nxp,pca_series.yaml`) state that part is register/pin-compatible with
the PCA9535 family the TCA9535 belongs to, and there's already a real, in-tree
precedent for exactly this substitution
(`zephyr/boards/alientek/dnesp32s3b`, a TI/vendor-clone "XL9555" wired as
`nxp,pca9555`). `gpio_pca_series.c` is a complete, interrupt-capable driver
(edge/level triggering, deferred `k_work` dispatch, per-pin callbacks) — the
same shape of thing Seeed hand-rolls in `sx126x_sensecap_board.c`. Using it
means `zephyr/drivers/lora/native/sx126x/sx126x.c` (the same native SX126x
driver every other board in this project uses) works against this radio
**completely unmodified** — it just calls the generic Zephyr GPIO API, which
this expander driver implements identically to a native SoC GPIO bank. No
board-specific C glue was needed for the radio/expander integration at all;
everything is devicetree wiring. Confirmed by an actual build:
`CONFIG_GPIO_PCA_SERIES_INTERRUPT=y` and `CONFIG_LORA_SX126X_NATIVE=y` both
come out enabled from the `int-gpios`/`compatible = "semtech,sx1262"`
devicetree alone.

**Known cost, not a correctness problem:** SPI chip-select for this radio also
goes through the expander (`cs-gpios` on `&spi2`, pointing at the TCA9535).
Zephyr's SPI CS handling (`spi_context_cs_control()`) is a plain
`gpio_pin_set_dt()` call, generic across any GPIO controller — so this works —
but it means every single SPI transfer to the radio now pays an I2C
round-trip (roughly tens to a few hundred µs at typical I2C Fast-mode speeds)
before the clock even starts, on top of the transfer itself. Worth
characterising on the bench once this boots; if it turns out to matter, the
Zephyr-side fix is to stop using automatic `cs-gpios` and drive NSS manually
around a `spi_transceive()` call instead — same expander pin, more control over
when the I2C write actually happens.

## Build

```
source .venv/bin/activate   # from the workspace root
west build -b sensecap_indicator_d1l/esp32s3/procpu main/samples/meshtastic
```

No board-specific overlay yet (no `overlay-sensecap-...conf` — the plain
`prj.conf` is headless already: no display, no WiFi/BLE profile assumptions).
Builds clean; **not yet flashed or bench-tested.**

## Pending hardware verification

- **GPIO42 (expander INT) polarity/pull, and the whole interrupt path.** Sourced
  and cross-confirmed as a real edge-triggered line (not polled), but never
  exercised against real silicon by this port.
- **TCXO voltage: 2.4 V** (`SX126X_DIO3_TCXO_2V4`), taken directly from Seeed's
  `bsp_sx126x.h` (`TCXO_CTRL_2_4V`). Different from every Heltec board in this
  project (1.8 V) — deliberate, not copied over by habit, but still unverified
  on this specific hardware.
- **SPI CS-via-expander latency** noted above — real, not yet characterised.
- **No FEM, no `rx-boosted`.** Neither Seeed's BSP nor Meshtastic's variant.h
  mentions an external PA/LNA front-end for this SKU (unlike the Heltec V4
  family) — treated as a bare SX1262, so the weak
  `meshtastic_radio_fem_*()` identity/no-op defaults apply and no board
  `_fem.c` exists. `rx-boosted` (an SX1262-native capability, independent of
  any FEM) is left off because nothing sourced here confirms Seeed enables it
  — not assumed on.
- **PSRAM speed (40 MHz)** is a conservative default, not a SenseCAP-specific
  measurement — see the comment in `Kconfig.defconfig`. Seeed's own firmware
  runs octal PSRAM at 120 MHz, but only with vendor-supplied ESP-IDF patches
  this port hasn't investigated.
- **Console** (`&usb_serial`, native USB-CDC): confirmed to compile and enable
  the right driver (`CONFIG_SERIAL_ESP32_USB=y`), never confirmed against a
  real USB-C port on this hardware.

## Not attempted here

Display (ST7701 RGB, parallel), touch, RP2040 link, audio, BMP390 — all
deliberately out of scope for this pass. See
`~/zephyrtastic-wt/docs/FUTURE-BOARD-TARGETS.md` for the fuller hardware
picture and the reasoning behind bringing the radio up first.
