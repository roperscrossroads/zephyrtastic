# Heltec WiFi LoRa 32 — board specs

Reference for the Heltec WiFi LoRa 32 **V4** family (rev 4.2, rev 4.3, and V4-R8).

**Common to all:** Semtech **SX1262** LoRa radio; SX1262 **DIO2 drives the
FEM TX/RX path-select** (`dio2-tx-enable`); bootloader @ `0x0`, signed app @
slot0 `0x20000`; **MCUboot swap-using-move** (real A/B rollback). Default
radio plan: **US 906.875 MHz, SF11, BW250, CR 4/5**. The firmware ships a US
default (902–928 MHz) as a deliberate, safe out-of-the-box choice so a
freshly flashed board never transmits on an unintended band — the hardware
itself supports both 868 and 915 MHz, so the region is a firmware setting,
not a hardware limit. Set it with `CONFIG_MESHTASTIC_DEFAULT_REGION`.

**No licence is required for US operation:** 902–928 MHz is unlicensed ISM
spectrum. The amateur-radio "licensed operator" flag matters only for the ITU
amateur allocations and for exceeding a region's power limit.

### Bands these boards cannot use

The firmware rejects two groups of regions outright:

- **Amateur (ITU) allocations.** Partly regulatory — amateur service forbids
  obscuring the meaning of a transmission, and this port does not yet disable
  encryption in licensed mode the way the reference firmware does (it neither
  suppresses PKI keygen nor forces `LOCAL_ONLY` rebroadcast). Partly physical:
  the **2 m allocations are 144–146 MHz, below the SX1262's 150 MHz tuning
  floor**, so the radio cannot reach them at all. The 125 cm (220 MHz) and
  70 cm (430–440 MHz) allocations are within the chip's range but sit far
  outside these boards' 863–928 MHz front end and matching network, where the
  PA would be badly mismatched.

- **LORA_24 (2.4 GHz)**, which needs a wide-LoRa capable radio this port does
  not support.

The 433 MHz ISM regions (`EU_433`, `ANZ_433`, `UA_433`, …) are *accepted* by
the config validator but are equally outside these boards' front end. They are
meaningful only if this firmware is ever built for a 433 MHz Heltec variant.

## Variants

| Spec | **V4 rev 4.2** | **V4 rev 4.3** | **V4-R8** |
|---|---|---|---|
| MCU | ESP32-**S3R2** | ESP32-**S3R2** | ESP32-**S3R8** |
| PSRAM | 2 MB quad (QSPI) | 2 MB quad (QSPI) | 8 MB octal (OPI) |
| PSRAM mode @ speed | `SPIRAM_MODE_QUAD` @ 40 MHz | `SPIRAM_MODE_QUAD` @ 40 MHz | **`SPIRAM_MODE_OCT` @ 80 MHz** |
| PSRAM heap (`ESP_SPIRAM_HEAP_SIZE`) | 1 MB (Zephyr default) | 1 MB (Zephyr default) | **4 MB** |
| **GPIO33–37** | free | free | **bonded to the PSRAM bus — unusable** |
| Flash | 16 MB | 16 MB | 16 MB |
| MT hardware model | **110** | **110** | **132** |
| Zephyr target | `heltec_wifi_lora32_v4/esp32s3/procpu` | `heltec_wifi_lora32_v4/esp32s3/procpu` | `heltec_wifi_lora32_v4_r8/esp32s3/procpu` |
| BOARD_TAG (flasher) | `heltec-v4` | `heltec-v4` | `heltec-v4r8` |
| SoC dtsi | `esp32s3_wroom_n16r2` | `esp32s3_wroom_n16r2` | `esp32s3_wroom_n16r8` |
| Partition table | `..._16M.dtsi` | `..._16M.dtsi` | `..._16M.dtsi` |
| slot0 / slot1 | `0x20000` / `0x5F0000` | `0x20000` / `0x5F0000` | `0x20000` / `0x5F0000` |
| NVS/storage | **`0xFB0000`** | **`0xFB0000`** | **`0xFB0000`** |
| USB-serial | native USB-Serial-JTAG (`303a:1001`) | native (`303a:1001`) | native (`303a:1001`) |
| Serial port | `/dev/ttyACM0` | `/dev/ttyACM0` | `/dev/ttyACM0` |
| Console | usb_serial (native) | usb_serial (native) | usb_serial (native) |
| LoRa FEM (PA/LNA) | **GC1109** | **KCT8103L** | **KCT8103L** |
| FEM detect (CSD, GPIO2) | reads **LOW** | reads **HIGH** | reads **HIGH** |
| FEM mode pin | **GPIO46** (CPS) | **GPIO5** (CTX) | **GPIO5** (CTX) |
| FEM mode behavior | HIGH=TX/PA, LOW=RX | HIGH=TX, LOW=RX (LNA) | HIGH=TX, LOW=RX (LNA) |
| White LED | GPIO35 | GPIO35 | **GPIO46** |
| Vext (OLED power) | GPIO36 | GPIO36 | **GPIO40** |
| GNSS module | L76K | L76K | L76K |
| VGNSS_Ctrl | GPIO34 | GPIO34 | **GPIO42** |

## FEM notes (V4 family)

**Shared pins:** VFEM LDO enable = **GPIO7** (active-high); CSD
chip-enable/detect = **GPIO2**.

One firmware image per PSRAM class; the **FEM (GC1109 vs KCT8103L) is
auto-detected at boot** by reading CSD — it is *not* a build-time choice.
Only PSRAM size selects the target (`v4` = 2 MB, `v4_r8` = 8 MB).

## PSRAM notes (V4 family)

### GPIO33–37 are gone on the R8

An ESP32-S3 with **octal** PSRAM routes five extra MSPI lines — SPIIO4, SPIIO5,
SPIIO6, SPIIO7 and SPIDQS — to **GPIO33–37**, which the module bonds to the
PSRAM die. They are not pins a peripheral is merely "using": muxing any of them
away corrupts every PSRAM access, and since the NodeDB, config store, MQTT
context and PhoneAPI queues all live in `.ext_ram.bss`, the symptom is random
corruption of application state rather than an obviously broken peripheral.

The 2 MB **quad** V4 leaves all five free, which is exactly why Heltec used
GPIO34/35/36/37 there and had to relocate every one of them on the R8 (see the
table above). Two practical consequences:

- **Any new pin assignment in the shared `-common.dtsi` / `-pinctrl.dtsi` in the
  33–37 range must be overridden in `heltec_wifi_lora32_v4_r8_procpu.dts`.**
  Nothing in the build system will warn about it.
- Watch for *latent* assignments. `ledc0` is `status = "okay"` with the shared
  `ledc0_default` pinctrl, but the meshtastic sample builds `CONFIG_PWM=n`, so
  the driver never applies that state. Enabling PWM on the R8 without the
  GPIO46 pinctrl override would have muxed LEDC onto GPIO35 = SPIIO6 at driver
  init. That override is now in the R8 `.dts`.

### Where PSRAM settings live

Mode, type, speed and heap size are **board** properties and live in each
board's `Kconfig.defconfig`, *not* in an application overlay. An overlay shared
between a quad board and an octal board can only be right for one of them:
`overlay-v4-unified.conf` previously pinned `CONFIG_SPIRAM_MODE_QUAD=y`
unconditionally, and the R8 inherited it.

That mistake was invisible in every build artefact. `ESP_SPIRAM_SIZE` is derived
from the `psram0` devicetree node so it was correctly 8 MB, and the
`SPIRAM_TYPE` choice carries a `depends on (ESP_SPIRAM_SIZE = ...)` cross-check
so it correctly resolved to `ESPPSRAM64` — but **`SPIRAM_MODE` has no guard
against the devicetree at all**, so "octal part, quad mode" linked cleanly and
produced a normal-looking image.
