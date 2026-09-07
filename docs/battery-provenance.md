# Battery reading — where every number comes from

The on-device UI's battery readout (`CONFIG_MESHTASTIC_DISPLAY_BATTERY`, see
[`cfb-display-ui.md`](cfb-display-ui.md#battery)) depends on a handful of
hardware constants and calibration choices. Because a wrong number here is
*plausible* rather than obviously broken, this file records the source for each
one and cross-checks it against independent references.

> **Scope: Heltec WiFi LoRa 32 V4, revision 4.2.** The manufacturer schematic
> consulted below is `WiFi_LoRa_32_V4.2.pdf`, and the firmware constants come
> from the `variants/esp32s3/heltec_v4` variant — which upstream Meshtastic and
> this Zephyr port apply as **one image to both rev 4.2 and rev 4.3** (the FEM is
> auto-detected; the digital wiring is shared). Only rev 4.2's schematic was
> verified here; rev 4.3 is *expected* to use the identical battery circuit but
> is not independently confirmed in this document. The **V4-R8** board differs
> materially (different gate, different calibration) — see
> [Other Heltec variants](#other-heltec-variants) before assuming any number
> here carries over.

## Safety first: we only *read*, we never *charge*

The Heltec V4 charges its cell with a **dedicated hardware charge IC** — visible
on the schematic as `U4` (`CHRG`/`DONE`/`ISET` pins) with `R13` setting the
constant-current limit (the schematic annotates `I = 1188/R13 = 540 mA`). That
IC, the protection circuitry, and the cell's own PCM handle all charge/discharge
safety **independently of this firmware**.

Our code does exactly one thing: sample a voltage divider through the ADC and
draw a number on the screen. A wrong calibration factor or OCV curve can only
make the *displayed percentage* inaccurate — it cannot affect charging, cannot
over-discharge the cell, and poses no battery-safety risk. This is a cosmetic
gauge, not a battery-management system.

## The sources

| # | Source | Independence | What it establishes |
|---|---|---|---|
| **S1** | **Heltec official schematic — rev 4.2** — `WiFi_LoRa_32_V4.2.pdf` ([resource.heltec.cn](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/Schematic/WiFi_LoRa_32_V4.2.pdf), linked from the [Heltec wiki V4 page](https://docs.heltec.org/)) | **Primary** (the manufacturer's own hardware design, revision 4.2) | The physical divider resistors, the ADC net, the ADC_CTRL gate, the charge IC |
| **S2** | **Heltec Arduino library** — `Heltec_ESP32` examples (`LoRaWAN/LoRaWAN_GHTV3_Battery.ino`, `VME290/weather_station.ino`) | **Heltec-side, but software** (their own reference code) | Which GPIO the divider reads, the ADC resolution, the ÷4.9 ratio, the ADC_CTRL-enable pattern |
| **S3** | **Meshtastic firmware** — `firmware/variants/esp32s3/heltec_v4/variant.h`, `firmware/src/Power.{cpp,h}` | **Independent project** (the upstream we port for parity) | Pin/channel, attenuation, the calibration multiplier, the SoC curve |
| **S4** | **This Zephyr port** — `boards/heltec/heltec_wifi_lora32_v4/heltec_wifi_lora32_v4-common.dtsi`, `src/meshtastic_display.c` | Derived from S1/S3 | How we encode the divider + gate + read |

> **Honest note on independence.** S1 and S3 are genuinely independent (Heltec
> designed the board; Meshtastic reverse-derived the read path). S2 is Heltec's
> own but for sibling boards. S4 (us) is *derived* — it corroborates faithful
> transcription, not an independent measurement. So "3 sources agree" is real for
> the **hardware facts** below, and honestly weaker for the **calibration**.

## Hardware facts — independently corroborated ✅

These are properties of the board, confirmed by the manufacturer schematic **and**
at least one other source. High confidence.

| Fact | Value | S1 (schematic) | S2 (Heltec SW) | S3 (Meshtastic) | S4 (our DT) |
|---|---|---|---|---|---|
| Divider top resistor | **390 kΩ** | one `390K` on the `ADC_IN` net | — | ratio only | `full-ohms = 100000 + 390000` |
| Divider bottom resistor | **100 kΩ** | `100K` on `ADC_IN` | — | ratio only | `output-ohms = 100000` |
| Divider ratio | **4.9** (= 490/100) | 490k/100k | `× 4.9` (`weather_station.ino:129`) | `ADC_MULTIPLIER 4.9 * …` (`variant.h:9`) | `full/output = 4.9` |
| ADC pin | **GPIO1 / ADC1 ch0** | divider → `ADC_IN` → SoC | `analogRead(1)` (`…GHTV3_Battery.ino:131`) | `BATTERY_PIN 1`, `ADC_CHANNEL_0` (`variant.h:6-7`) | `io-channels = <&adc0 0>` (was `&adc1 0` — see correction below) |
| ADC resolution | **12-bit** | — | `analogReadResolution(12)` (`…:168`) | 12-bit read | `zephyr,resolution = <12>` |
| ADC_CTRL gate | **GPIO37, active-high** | net `ADC_Ctrl` on pin 37, gated via Q6 | ADC_CTRL-enable pattern (`weather_station.ino:78`) | `ADC_CTRL 37`, `ADC_CTRL_ENABLED HIGH` (`variant.h:4-5`) | `adc_ctrl` `enable-gpios = <&gpio1 5>` (=GPIO37) `ACTIVE_HIGH` |
| Cell chemistry | **single-cell LiPo** | 1.25×2P LiPo connector, charge IC `U4` | — | `NUM_CELLS 1` (`Power.h:30`) | n/a |
| Charging | **hardware** (`U4`, R13→540 mA CC) | charge IC on schematic | — | — | n/a |

The `390K` + `100K` pair is the *only* such divider on the schematic, and it sits
directly on the battery-sense `ADC_IN` net — so there is no ambiguity about which
resistors form it.

> **Correction, 2026-09-06: the S4 row above was wrong for seven weeks and this
> table's own cross-check method could not catch it.** `io-channels = <&adc1 0>`
> did NOT mean "GPIO1" — it meant Zephyr devicetree node `adc1`, which
> `esp32s3_common.dtsi` defines as `unit = <2>` (hardware ADC2), channel 0 of
> which is `ADC2_CHANNEL_0_GPIO_NUM` = **GPIO11**, not GPIO1
> (`modules/hal/espressif/.../soc/esp32s3/include/soc/adc_channel.h`). GPIO11 is
> this board's SPI MISO line to the SX1262 (`LORA_MISO 11` in both
> `heltec_v4/variant.h` and `heltec_v4_r8/variant.h`, confirmed independently of
> Zephyr) — so the divider was sampling the radio's MISO pad instead of the
> battery. Found by cross-referencing upstream Zephyr commit `2662f84b83`
> ("boards: heltec: fix battery voltage divider adc unit", 2026-08-28), which hit
> and fixed the identical mistake on the in-tree `heltec_wifi_lora32_v3`,
> `heltec_wireless_tracker` and `heltec_wireless_stick_lite_v3` boards — all
> ESP32-S3, all with the same `&adc1` vs `&adc0` label/unit mismatch. Fixed here
> in `boards/heltec/heltec_wifi_lora32_v4/heltec_wifi_lora32_v4-common.dtsi`
> (shared by V4 and V4-R8).
>
> The lesson for this doc's own method: S1-S3 independently corroborate the
> *board fact* (GPIO1 is the right pin) perfectly well — the bug was entirely in
> S4's *encoding* of that fact, where a devicetree node label that reads like a
> 1:1 name for the SoC's own "ADC1"/"ADC2" units is actually an unrelated index.
> A `voltage-divider` binding pointed at the wrong pin still parses, builds and
> boots — nothing here would have failed loudly. Carried patch 0001 (see
> `zephyr/patches/0001-...`, the ADC-driver GPIO-disconnect workaround discovered
> 2026-07-21) was very likely masking exactly this bug rather than fixing an
> unrelated one: disconnecting GPIO11's digital buffer breaks all SPI reads from
> the radio (both TX-done and RX-done status), which fully accounts for the
> "both TX and RX dead" symptom that patch was written against, with no need for
> the "neighbouring RTC-IO pin" mechanism that patch's own comment proposed.
> **Not yet re-tested on hardware** — whether patch 0001 is now redundant (GPIO1
> has no other function on this board) is an open question for the next bench
> session, not a conclusion to act on without a real test.

## Calibration & curve — single upstream source ⚠️

These are **software choices made by upstream Meshtastic**, not hardware facts.
We copy them verbatim for behavioural parity. They are **not** independently
corroborated — and largely can't be, because they are tuning, not physics.

| Choice | Value | Source | Notes |
|---|---|---|---|
| ADC attenuation | `ADC_ATTEN_DB_12` (→ Zephyr `ADC_GAIN_1_4`) | `Power.cpp:100` | Heltec's `analogReadMilliVolts()` uses the same ~12 dB default, so loosely corroborated by S2 |
| Empirical multiplier | **× 1.045** (V4; × 1.035 on R8) | `variant.h:9` (`ADC_MULTIPLIER 4.9 * 1.045`) | **Explicitly a fudge factor.** Upstream's own guidance: *"If the calculated result shows a significant deviation from the actual battery level, please adjust the value of the coefficient."* Exposed as `…_BATTERY_CAL_PERMILLE`. |
| State-of-charge curve | OCV table `4190…3100 mV`, interpolated | `Power.h:24`, `Power.cpp:349-392` | A **generic single-cell LiPo** open-circuit-voltage curve (attributed in-code to G. Russo, 2024) — not board-specific or measured on a V4. Below 2600 mV = "no battery". |

## What this means for trust

- **Where the divider connects and what the resistors are:** solid. Three
  sources including the manufacturer schematic agree.
- **The exact voltage the screen shows:** trust it to ~±5%, no better, until
  bench-verified. The `× 1.045` factor is empirical and the ESP32-S3 ADC needs
  per-chip calibration; that is why the code and devicetree carry a
  `VERIFY(hardware)` marker and the factor is a tunable Kconfig.
- **The percentage:** a reasonable estimate from a generic LiPo curve, not a
  fuel-gauge reading. Good enough for a glance; do not treat it as precise.

## Other Heltec variants

Only **V4 rev 4.2** is verified above. The battery *reading* is **not** uniform
across the Heltec ESP32-S3 family — the table below is recorded from the upstream
firmware variants so a future port has a starting point, but **each row still
needs its own schematic check and bench pass** before its numbers are trusted.
Do not assume rev 4.2's constants carry over.

| Board | Divider | ADC_CTRL | Multiplier | PSRAM | Watch out for |
|---|---|---|---|---|---|
| **V4 rev 4.2** ✅ | 390k/100k (4.9) | GPIO37, **HIGH** | 4.9 × **1.045** | 2 MB quad | — (this document) |
| **V4 rev 4.3** | same *(expected)* | GPIO37, HIGH *(expected)* | 4.9 × 1.045 | 2 MB quad | Same Zephyr image as 4.2; wiring shared, only the FEM differs. Confirm against a 4.3 schematic if one is published. |
| **V4-R8** | same | **none** — divider always connected | 4.9 × **1.035** | 8 MB octal | `heltec_v4_r8` defines **no** `ADC_CTRL`. Our code already skips the gate when the `adc_ctrl` node is absent; set `…_BATTERY_CAL_PERMILLE=1035`. |

Source for the non-4.2 rows: `firmware/variants/esp32s3/heltec_v{4,4_r8}/variant.h`.
When you port one of these, re-run the
[bench checklist](#bench-verification-checklist-verifyhardware) and replace its
row's *(expected)* / firmware-only entries with a schematic citation for that
specific revision.

## Bench verification checklist (`VERIFY(hardware)`)

1. With a known battery voltage (measure at the JST with a multimeter), compare
   the on-screen `Bat x.yV`. If it is off by a fixed ratio, adjust
   `CONFIG_MESHTASTIC_DISPLAY_BATTERY_CAL_PERMILLE`.
2. Confirm the reading only appears with ADC_CTRL asserted (it is pulsed high per
   sample) and that a battery-less board reads low enough to show "no batt"
   rather than a phantom voltage.
3. Sanity-check the percentage at full charge (~4.2 V → ~100%) and a partly
   drained cell against the OCV curve.

## 2026-09-06/07: the `&adc0` fix confirmed on real hardware, and a second gap found

Bench-tested the `&adc1 0` → `&adc0 0` fix (see the correction above) on **rzr4**
(V4-R8, class 6). Built and flashed twice:

- **0.3.21** (the fix alone, on top of the in-flight sx126x wedge fix): boots
  clean, radio fully healthy (`meshtastic status`/`rf` all green) — but this
  build has no code that ever calls into the ADC at all (no battery feature is
  merged on `main`), so it only proves `&adc0 { status = "okay"; }` alone is
  harmless. It does not exercise `adc_channel_setup()`.
- **0.3.22** (0.3.21 + `CONFIG_SENSOR=y CONFIG_VOLTAGE_DIVIDER=y
  CONFIG_SENSOR_SHELL=y`, a config-only change, no app code): this makes
  Zephyr's generic `voltage-divider` sensor driver auto-init at boot and
  actually call `adc_channel_setup_dt()` on the (now-correct) GPIO1 channel.
  Radio stayed fully healthy afterward — `tx: 7 ok, 0 failed`, `rx: 16
  decoded`, `radio wedge-resets 0` — real confirmation that whatever the ADC
  setup call touches now, it is no longer GPIO11/MISO. rzr4 was reflashed back
  to plain 0.3.21 afterward; 0.3.22 was a throwaway diagnostic build, not
  registered as a fleet class.

**Second finding, unrelated to the pin fix:** the `vbatt` sensor device came up
`(DISABLED)` (`device list`) on 0.3.22 — `voltage_init()`
(`zephyr/drivers/sensor/voltage_divider/voltage.c`) failed, most likely because
our `vbatt` node has no `channel@0` child under `&adc0` carrying
`zephyr,gain`/`zephyr,reference`/`zephyr,acquisition-time`/`zephyr,resolution` —
properties `ADC_DT_SPEC_GET` expects on the channel node the `io-channels`
phandle points at (see `zephyr/include/zephyr/drivers/adc.h`'s own DT examples).
This is **independent of adc0 vs adc1** — it would have failed identically
under the old wrong pin, and the `battery-adc-v2` branch's custom
`meshtastic_battery.c` (not the generic sensor framework) may or may not hit
the same gap depending on how it calls the ADC API directly. **Whoever revives
`battery-adc-v2` needs to check this first**, before assuming the pin fix alone
unblocks it.

**Third finding: carried patch 0001 (the `adc_esp32` GPIO-disconnect skip) now
looks genuinely redundant.** Built **0.3.23** — 0.3.22's config, but with
`zephyr/patches/0001-adc-esp32-skip-disruptive-gpio-disconnect.patch` reverse-
applied, i.e. the original disruptive `gpio_pin_configure_dt(GPIO_DISCONNECTED)`
call restored in `drivers/adc/adc_esp32.c`. If patch 0001 was ever protecting
against a real "neighbouring RTC-IO pin" mechanism (as its own comment
theorizes) rather than just masking the wrong-pin bug, this build should have
broken the radio the moment `adc_channel_setup()` ran. It didn't: `tx: 12 ok, 0
failed`, `rx: 24 decoded, 0 decode failures`, `radio wedge-resets 0`, `SPI BUSY
streak 0`, sustained over several minutes on rzr4. (One single `rx armed: NO`
snapshot appeared mid-test and cleared itself 15s later on a re-check — a
normal CAD/TX-cycle timing artifact, not a wedge; confirmed by tx/rx counts
climbing normally across it.) This is real hardware evidence that patch 0001's
"neighbouring pin" theory was never the actual mechanism — the disruptive
disconnect call was always landing on whatever pin `io-channels` pointed
at, and now that it correctly points at GPIO1 (unused for anything else on
this board), disconnecting it costs nothing. **Not yet long enough to call
proven** (minutes, not hours) — before actually deleting patch 0001, let this
soak longer and confirm on a plain V4 too, not just the R8. rzr4 was left
running 0.3.23 (fix + sensor test + patch 0001 reverted) rather than reverted
back, specifically to keep gathering evidence on this question.

**Independent cross-check, 2026-09-07:** a maintained community fork,
[`Amoulier/meshtastic-heltec-v4-firmware`](https://github.com/Amoulier/meshtastic-heltec-v4-firmware)
(stock Meshtastic/PlatformIO, C++, not Zephyr — explicitly does not build for
the R8) confirms the same hardware facts independently: `BATTERY_PIN 1`,
`ADC_CHANNEL 0`, `ADC_CTRL 37` in its own `variant.h`. It reads via ESP-IDF's
native `adc_oneshot`/`ADC_UNIT_1`/`ADC_CHANNEL_0` directly, so it cannot hit
our `&adc0`-vs-`&adc1` bug at all — that's a Zephyr devicetree-label trap with
no ESP-IDF equivalent. Two things worth stealing when `battery-adc-v2` is
revived: it gates NVS/preference writes on a healthy recent battery reading or
an active USB host (skips the optional NodeDB save at its critical-voltage
shutdown path specifically to cut brownout-corruption risk during the exact
low-battery window this project's own docs already record as having cost two
swollen cells), and it rate-limits the displayed percentage to ≤1%/min to
smooth LoRa-TX voltage sag rather than showing every sample's jitter.
