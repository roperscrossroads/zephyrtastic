# Heltec WiFi LoRa 32 — board specs

Reference for the Heltec WiFi LoRa 32 targets: **V3** and the **V4** family
(rev 4.2, rev 4.3, and V4-R8).

**Common to all:** Semtech **SX1262** LoRa radio; SX1262 **DIO2 drives the
FEM TX/RX path-select** (`dio2-tx-enable`); bootloader @ `0x0`, signed app @
slot0 `0x20000`; **MCUboot swap-using-move** (real A/B rollback). Default
radio plan: **US 906.875 MHz, SF11, BW250, CR 4/5**. The firmware ships a US
default (902–928 MHz) as a deliberate, safe out-of-the-box choice so a
freshly flashed board never transmits on an unintended band — the hardware
itself supports both 868 and 915 MHz, so the region is a firmware setting,
not a hardware limit.

## Variants

| Spec | **V3** | **V4 rev 4.2** | **V4 rev 4.3** | **V4-R8** |
|---|---|---|---|---|
| MCU | ESP32-S3FN8 | ESP32-**S3R2** | ESP32-**S3R2** | ESP32-**S3R8** |
| PSRAM | none | 2 MB quad (QSPI) | 2 MB quad (QSPI) | 8 MB octal (OPI) |
| Flash | **8 MB** | 16 MB | 16 MB | 16 MB |
| MT hardware model | HELTEC_V3 | **110** | **110** | **132** |
| Zephyr target | `heltec_wifi_lora32_v3/esp32s3/procpu` | `heltec_wifi_lora32_v4/esp32s3/procpu` | `heltec_wifi_lora32_v4/esp32s3/procpu` | `heltec_wifi_lora32_v4_r8/esp32s3/procpu` |
| BOARD_TAG (flasher) | `heltec-v3` | `heltec-v4` | `heltec-v4` | `heltec-v4r8` |
| SoC dtsi | (upstream) | `esp32s3_wroom_n16r2` | `esp32s3_wroom_n16r2` | `esp32s3_wroom_n16r8` |
| Partition table | `..._8M.dtsi` | `..._16M.dtsi` | `..._16M.dtsi` | `..._16M.dtsi` |
| slot0 / slot1 | `0x20000` / `0x2F0000` | `0x20000` / `0x5F0000` | `0x20000` / `0x5F0000` | `0x20000` / `0x5F0000` |
| NVS/storage | **`0x7B0000`** | **`0xFB0000`** | **`0xFB0000`** | **`0xFB0000`** |
| USB-serial | **CP2102** (`10c4:ea60`) | native USB-Serial-JTAG (`303a:1001`) | native (`303a:1001`) | native (`303a:1001`) |
| Serial port | `/dev/ttyUSB0` | `/dev/ttyACM0` | `/dev/ttyACM0` | `/dev/ttyACM0` |
| Console | uart0 (via CP2102) | usb_serial (native) | usb_serial (native) | usb_serial (native) |
| LoRa FEM (PA/LNA) | **none** (bare SX1262) | **GC1109** | **KCT8103L** | **KCT8103L** |
| FEM detect (CSD, GPIO2) | n/a | reads **LOW** | reads **HIGH** | reads **HIGH** |
| FEM mode pin | n/a | **GPIO46** (CPS) | **GPIO5** (CTX) | **GPIO5** (CTX) |
| FEM mode behavior | n/a | HIGH=TX/PA, LOW=RX | HIGH=TX, LOW=RX (LNA) | HIGH=TX, LOW=RX (LNA) |
| White LED | GPIO35 | GPIO35 | GPIO35 | **GPIO46** |
| Vext (OLED power) | (V3 pin) | GPIO36 | GPIO36 | **GPIO40** |
| GNSS module | none | L76K | L76K | L76K |
| VGNSS_Ctrl | n/a | GPIO34 | GPIO34 | **GPIO42** |

## FEM notes (V4 family)

**Shared pins:** VFEM LDO enable = **GPIO7** (active-high); CSD
chip-enable/detect = **GPIO2**.

One firmware image per PSRAM class; the **FEM (GC1109 vs KCT8103L) is
auto-detected at boot** by reading CSD — it is *not* a build-time choice.
Only PSRAM size selects the target (`v4` = 2 MB, `v4_r8` = 8 MB).
