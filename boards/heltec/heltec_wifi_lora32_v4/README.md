# Heltec WiFi LoRa 32 V4 — Zephyr board port

Local (out-of-tree) Zephyr board port for the Heltec WiFi LoRa 32 **V4** family.
Upstream Zephyr has no V4 board, so this lives in the repo's `boards/` tree and
is discovered via `zephyr/module.yml` (`settings.board_root: .`).

> **Status:** all targets **build clean** (Zephyr 4.4.99, meshtastic sample) but
> are **bench-unverified**. Every pin/behaviour marked `VERIFY(hardware)` in the
> devicetree still needs confirming on a real board. See
> [Pending verification](#pending-hardware-verification).

---

## ⚠️ Which image goes on which board (read before flashing)

The V4 family is **not one board**. The two that matter most are distinguished
by their PSRAM, and Meshtastic gives them **different hardware model numbers**:

| Board | Meshtastic HW model | PSRAM | Zephyr target here | Cross-flash risk |
|---|---|---|---|---|
| **Heltec V4** (4.2 **and** 4.3) | **110** `HELTEC_V4` | 2 MB **QSPI** (quad) | `heltec_wifi_lora32_v4/esp32s3/procpu` (one image, FEM auto-detected) | — |
| **Heltec V4 R8** | **132** `HELTEC_V4_R8` | 8 MB **OPI** (octal) | `heltec_wifi_lora32_v4_r8/esp32s3/procpu` | ❌ **do not cross-flash** |

**The dangerous mismatch is PSRAM mode, not the FEM.** A quad-PSRAM image
(`_v4`) on an octal-PSRAM board (R8), or vice-versa, configures the SPI RAM
controller wrong — expect boot hangs / RAM corruption / crashes, not a clean
error. Keep model **110** and model **132** firmware strictly separate.

**The FEM (GC1109 vs KCT8103L) is *not* a separate board number, and this port
does not split on it.** Like the stock `HELTEC_V4` (110) image, this port
**auto-detects the FEM at runtime** (`heltec_wifi_lora32_v4_fem.c`) and runs a
single image on both 4.2 and 4.3 — no board revisions, no `@rev` suffix. See
[The FEM](#the-fem-the-real-v4-difference).

There is also a **Heltec V4 TFT** (model 110, ST7789 240×320 touch screen
instead of the OLED). This port targets the **OLED** variant; the TFT display is
out of scope (the radio/FEM/GPS wiring is the same, only the display differs).

---

## How to identify your board

You do **not** need to open the case or read chip markings. Easiest first:

### Best: read the stock Meshtastic boot log
Flash the official `HELTEC_V4` firmware (or use the web flasher's auto-detect),
open the serial console at 115200, and reboot. The log tells you everything:

- **FEM type** — one of:
  `Detected KCT8103L LoRa FEM` (→ rev **4.3**) or
  `Detected GC1109 LoRa FEM` (→ rev **4.2**).
- **PSRAM** — the ESP-IDF banner prints the SPIRAM size/mode early in boot.
  `2 MB` quad ⇒ plain **V4**; `8 MB` octal ⇒ **R8** (and you should have
  flashed model 132, not 110).

(If the flasher/device page reports the model directly: **110** = V4,
**132** = V4 R8.)

### Alternative: esptool
```
esptool.py --chip esp32s3 flash_id        # flash is 16 MB on all V4s
esptool.py --chip esp32s3 read_flash_status # (flash size doesn't distinguish R8)
```
Flash size (16 MB) is common to every V4, so it can't tell R8 apart — the PSRAM
does, and the boot-log method above is the reliable read of PSRAM.

### Last resort: visual
The FEM IC sits near the antenna connector; under magnification it is marked
**GC1109** or **KCT8103L**. The R8 carries a larger/octal PSRAM part. Treat
markings as confirmation, not the primary method — the boot log is definitive.

---

## The V4 family: variants and differences

All V4s share the **same core**: ESP32-S3 (WROOM, 16 MB flash), on-board SX1262
LoRa, SSD1315 OLED (SSD1306-compatible), L76K GNSS, and — new vs V3 — an **RF
front-end module (PA + LNA)** between the SX1262 and the antenna. What differs:

| | **V4 (4.2)** | **V4 (4.3)** | **V4 R8** |
|---|---|---|---|
| Meshtastic model | 110 `HELTEC_V4` | 110 `HELTEC_V4` | 132 `HELTEC_V4_R8` |
| PSRAM | 2 MB QSPI (quad) | 2 MB QSPI (quad) | 8 MB OPI (octal) |
| RF front-end | **GC1109** | **KCT8103L** | KCT8103L |
| FEM mode pin | CPS = **GPIO46** | CTX = **GPIO5** | CTX = **GPIO5** |
| Vext (OLED pwr) | GPIO36 | GPIO36 | **GPIO40** |
| ADC_CTRL (batt gate) | GPIO37 | GPIO37 | **none** |
| GPS enable | GPIO34 | GPIO34 | **GPIO42** |
| Radio SPI wiring | identical | identical | identical |

Shared FEM controls (all V4s): **VFEM power = GPIO7**, **CSD chip-enable = GPIO2**,
and the SX1262's **DIO2** drives the FEM TX/RX path-select pin.

Firmware envs in the reference tree (`firmware/variants/esp32s3/`):
`heltec-v4` (OLED), `heltec-v4-tft` (touch TFT), `heltec-v4-r8-oled`.

---

## The FEM — the real V4 difference

Unlike the bare-radio V3, the SX1262 feeds an integrated PA+LNA front-end. It
needs three controls the V3 never had:

1. **VFEM power** (GPIO7) — LDO enable; must be ON before any RX/TX.
2. **CSD chip-enable** (GPIO2) — HIGH = FEM active, LOW = shutdown.
3. **Mode select** — GC1109 **CPS/GPIO46** (HIGH = full PA) or KCT8103L
   **CTX/GPIO5** (LOW = RX-LNA, HIGH = TX / RX-bypass).

The **TX/RX path** select is done by the SX1262's own **DIO2** (`dio2-tx-enable`),
exactly as on V3 — DIO2 is hardwired to the FEM's path pin (CTX on GC1109, CPS on
KCT8103L).

### How the stock firmware handles the FEM
`firmware/src/mesh/LoRaFEMInterface.cpp` powers VFEM, then reads GPIO2 (CSD) as an
input: **HIGH ⇒ KCT8103L, LOW ⇒ GC1109** (the two boards pull that line
differently). It then drives the pins **dynamically** per mode — e.g. KCT8103L
CTX is HIGH on TX and LOW on RX-LNA. One image, both boards, correct RF in every
mode. It also applies a **different TX-gain table per FEM** (`powerConversion()`),
so the FEM type affects power calibration too.

### How this Zephyr port handles it
`heltec_wifi_lora32_v4_fem.c` runs at boot (`SYS_INIT`, POST_KERNEL/80 — after
the GPIO drivers, before the SX1262). It **mirrors the stock firmware's
detection**: power VFEM (GPIO7), read CSD (GPIO2) as an input — **HIGH ⇒
KCT8103L, LOW ⇒ GC1109** — then enable the FEM (CSD high) and set that
front-end's mode pin. DIO2 does the TX/RX path switching. It logs what it found
(see [Log messages](#log-messages)). One image, no revisions.

**Status of RF correctness:**

- **GC1109 (4.2) — fully correct.** CPS (GPIO46) held HIGH: full PA on TX,
  "don't care" on RX.
- **KCT8103L (4.3 / R8) — TX correct, RX-LNA not yet.** CTX (GPIO5) is held
  HIGH, which gives correct **full-PA TX** and a working (bypass) RX — but the
  ~1.9 dB-NF **LNA is inactive**, because engaging it needs CTX driven LOW *only
  during RX*. The stock Zephyr `semtech,sx126x` driver's `tx-enable-gpios` can't
  provide that: it is inhibited whenever `dio2-tx-enable` is set (and we need
  DIO2 for the path pin), so it can't toggle a *second* FEM pin.

> **Remaining follow-up (extra RX sensitivity on KCT8103L):** a small FEM hook
> that drives CTX LOW on RX / HIGH on TX, called from the radio layer's TX/RX
> transition (`meshtastic_radio.c`, `mt_radio_do_tx`), mirroring the reference
> firmware's `setTx/RxModeEnable`. Not needed for a functional node; it only
> recovers RX-LNA gain on the KCT8103L variant.

---

## Log messages

At boot the FEM init prints (LOG level INF, module `heltec_v4_fem`) exactly which
front-end it detected — useful both for confirming the board and for the RF
caveat above:

```
<inf> heltec_v4_fem: Detected KCT8103L LoRa FEM (V4 rev 4.3 / R8)
<inf> heltec_v4_fem: LoRa FEM: full-PA TX enabled; RX in bypass (LNA inactive) — static CTX; dynamic RX-LNA is a future improvement
```
or
```
<inf> heltec_v4_fem: Detected GC1109 LoRa FEM (V4 rev 4.2)
<inf> heltec_v4_fem: LoRa FEM: full-PA TX path enabled
```
An `<err> … FEM detect read failed` means the CSD read errored — RF will be
unreliable; check the board.

## Building & flashing

From the Zephyr workspace (`~/zephyrproject`, venv active). **One image per
PSRAM class** — the FEM (GC1109 vs KCT8103L) is auto-detected, so there is no
revision to choose:

```sh
# Heltec V4 (110) — 2 MB quad PSRAM. Runs on both 4.2 and 4.3.
west build -p always -b heltec_wifi_lora32_v4/esp32s3/procpu \
  ~/lab/meshprojects/meshtastic-zephyr/samples/meshtastic \
  -- -DZEPHYR_EXTRA_MODULES=~/lab/meshprojects/meshtastic-zephyr

# Heltec V4 R8 (132) — 8 MB octal PSRAM. SEPARATE image, do not cross-flash.
west build -p always -b heltec_wifi_lora32_v4_r8/esp32s3/procpu   ...

west flash        # or: west flash --esp-device /dev/ttyUSB0
```

The only choice that matters is `_v4` (model 110) vs `_v4_r8` (model 132). If
unsure, identify first (stock boot log / PSRAM size) — do **not** guess.

---

## Pin reference (V4 4.2 / 4.3)

Source: `firmware/variants/esp32s3/heltec_v4/variant.h`. ESP32-S3 GPIO→bank:
0–31 → `gpio0`, 32–63 → `gpio1` (pin = GPIO − 32).

| Function | GPIO | Notes |
|---|---|---|
| SX1262 CS | 8 | SPI2 |
| SX1262 SCK / MOSI / MISO | 9 / 10 / 11 | SPI2 |
| SX1262 RESET | 12 | open-drain, active-low |
| SX1262 BUSY | 13 | |
| SX1262 DIO1 (IRQ) | 14 | |
| SX1262 DIO2 | — | RF-switch → FEM path pin (`dio2-tx-enable`) |
| SX1262 DIO3 | — | 1.8 V TCXO supply |
| **FEM VFEM power** | **7** | LDO enable, active-high |
| **FEM CSD** (enable) | **2** | active-high; also the FEM-detect line |
| **GC1109 CPS** (PA mode, 4.2) | **46** | HIGH = full PA |
| **KCT8103L CTX** (RX mode, 4.3) | **5** | LOW = RX-LNA, HIGH = TX / RX-bypass |
| OLED I2C SDA / SCL | 17 / 18 | SSD1315 @ 0x3c |
| OLED reset | 21 | active-low |
| Vext (OLED power) | 36 | active-low (**R8: GPIO40**) |
| White LED | 35 | |
| USER button | 0 | active-low |
| Battery ADC | 1 | ADC1 ch0, divider |
| ADC_CTRL (batt gate) | 37 | active-high (**absent on R8**) |
| GPS RESET / EN / STANDBY / PPS | 42 / 34 / 40 / 41 | L76K (**R8 differs**) |
| GPS UART TX→CPU / RX→GPS | 38 / 39 | uart1, 9600 8N1 |

---

## Pending hardware verification

- [ ] Confirm the boot log FEM detection matches the actual board (GC1109 vs
      KCT8103L), and PSRAM/model (110 vs 132).
- [ ] OLED powers up (Vext polarity correct).
- [ ] FEM init runs before the SX1262 keys up (SYS_INIT POST_KERNEL/80 < LoRa 90).
- [ ] **GC1109 (4.2):** measure TX output — PA engaged, not ~1 dB bypass.
- [ ] **KCT8103L (4.3 / R8):** TX PA works (static CTX HIGH); RX runs in bypass
      (LNA off) until the dynamic-CTX follow-up — measure RX sensitivity.
- [ ] L76K GPS: 9600 baud + EN/STANDBY/RESET polarities; R8 GPS pins.
- [ ] R8 octal-PSRAM bring-up (SPIRAM not yet enabled in the defconfig).
- [ ] `CONFIG_MESHTASTIC_TX_POWER`: 20 dBm at the chip is high-20s dBm at the
      antenna through the FEM — set per region/antenna.

## Files

```
boards/heltec/heltec_wifi_lora32_v4/
  board.yml                         # single image, no revisions
  Kconfig* / board.cmake            # SoC select (WROOM_N16R2), boot, heap
  CMakeLists.txt                    # compiles the FEM init C file
  heltec_wifi_lora32_v4_fem.c       # runtime FEM detect + config + logs
  heltec_wifi_lora32_v4-pinctrl.dtsi   # radio SPI, OLED I2C, console + GPS uart1
  heltec_wifi_lora32_v4-common.dtsi    # ALL shared hardware (FEM pins owned by C)
  heltec_wifi_lora32_v4_procpu.dts     # SoC + partitions + Vext + GPS hogs
  heltec_wifi_lora32_v4_procpu_defconfig

boards/heltec/heltec_wifi_lora32_v4_r8/   # octal-PSRAM sibling (scaffold)
  CMakeLists.txt   # reuses ../heltec_wifi_lora32_v4/heltec_wifi_lora32_v4_fem.c
  ... reuses ../heltec_wifi_lora32_v4/*-common.dtsi + *-pinctrl.dtsi

samples/meshtastic/boards/heltec_wifi_lora32_v4_esp32s3_procpu.conf  # node tuning
```

Pin source of truth: `firmware/variants/esp32s3/heltec_v4{,_r8}/variant.h`.
