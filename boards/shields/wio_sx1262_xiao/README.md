# Wio-SX1262 for XIAO — Zephyr shield

Local (out-of-tree) Zephyr **shield** for Seeed's "Wio-SX1262 for XIAO" radio module
(standalone SKU 113010003 / nRF52840 kit SKU 102010710 — Meshtastic's
`XIAO_NRF52_KIT`/`SEEED_XIAO_NRF_KIT_DEFAULT`). Unlike the SenseCAP Indicator D1L, this
is a real shield rather than a full custom board: the base carrier
(`zephyr/boards/seeed/xiao_ble`, "XIAO BLE (Sense)") already exists upstream, so this
overlay plugs into it via Zephyr's standard `seeed,xiao-gpio` connector nexus.

> **Status:** builds clean (Zephyr 4.4.99, meshtastic sample) as of 2026-08-23. **Never
> flashed to hardware.**

## Scope

Radio (SX1262) + optional GPS (L76K over UART, matching the DEFAULT pinout variant).
**Not included:** battery ADC and the tri-color status LED — see the overlay's own
top-comment for why (they're base-carrier-board concerns with real sourcing gaps, not
shield concerns; not guessed in to fill out the scope).

## Pin sources

Every pin is sourced, not guessed:

1. **Upstream Meshtastic** `firmware/variants/nrf52840/seeed_xiao_nrf52840_kit/variant.h`
   — the radio's CS/RESET/BUSY/DIO1/RXEN pin assignments (D-labels), TCXO voltage, and
   the GPS UART pins/standby line. That file has three pinout blocks (legacy XIAO_BLE,
   the 30-pin board-to-board variant, and this shield's target — "Wio-SX1262 for XIAO
   (standalone SKU 113010003 or nRF52840 kit SKU 102010710)"); confirm which physical kit
   is in hand before assuming this shield is the right one.
2. **Zephyr's own `xiao_ble` board** — the D-label → physical nRF52840 GPIO translation
   (`seeed_xiao_connector.dtsi`'s `xiao_d` gpio-map) and confirmation that `xiao_spi`
   (SPI2) and `xiao_serial` (UART0) are already routed to D8/D9/D10 and D6/D7
   respectively by the base board's own pinctrl — this shield adds no new pin muxing,
   only enables buses the carrier board already wires to the header.

## Build

```
source .venv/bin/activate   # from the workspace root
west build -b xiao_ble/nrf52840/sense main/samples/meshtastic -- -DSHIELD=wio_sx1262_xiao
```

Console: this board's native USB-CDC ACM is wired as `zephyr,console`/`zephyr,shell-uart`
by `zephyr/boards/common/usb/cdc_acm_serial.dtsi`, included from the base `xiao_ble`
board itself — nothing to configure here, and it's why D6/D7 (UART0) are free to use for
GPS instead of a serial console.

## Pending hardware verification

- **`rx-enable-gpios` polarity** (`GPIO_ACTIVE_HIGH`, the conventional default for an
  enable pin) — nothing sourced here states it explicitly.
- **GPS standby polarity** ("drive high to force wake," inferred from
  `heltec_wifi_lora32_v4`'s handling of the same L76K module family, not from this
  board's own schematic).
- **Which physical variant is actually on hand** — this shield targets the DEFAULT
  pinout (SKU 113010003 / 102010710). The 30-pin board-to-board version from the
  ESP32-S3 kit uses different pins (`SEEED_XIAO_NRF_WIO_BTB` in Meshtastic's variant.h)
  and would need a separate shield.
- **Never bench-tested.**
