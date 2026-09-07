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
| Empirical multiplier | **× 1.030** (V4; × 1.015 on R8) | corrected against a real cell, see below | **Started as a fudge factor**, not a hardware constant. Upstream's starting point was `4.9 * 1.045` (V4) / `4.9 * 1.035` (R8) (`variant.h:9`), copied here for parity but never checked against real hardware until a direct multimeter cross-check found both variants over-reading by 1.5-2% — corrected to the values above. Upstream's own guidance still applies: *"If the calculated result shows a significant deviation from the actual battery level, please adjust the value of the coefficient."* Exposed as `…_BATTERY_CAL_PERMILLE`. Each correction is a single sample per variant — the divider resistors carry their own tolerance (typically ±1%), so re-check against a meter on any board this matters for rather than assuming these generalize to every unit. |
| State-of-charge curve | OCV table `4190…3100 mV`, interpolated | `Power.h:24`, `Power.cpp:349-392` | A **generic single-cell LiPo** open-circuit-voltage curve (attributed in-code to G. Russo, 2024) — not board-specific or measured on a V4. Below 2600 mV = "no battery". |

## What this means for trust

- **Where the divider connects and what the resistors are:** solid. Three
  sources including the manufacturer schematic agree.
- **The exact voltage the screen shows:** the calibration factor has now been
  checked against a real cell on one board per variant and corrected (see
  above) — trust it to a couple of percent, not better, since that's a single
  sample and the ESP32-S3 ADC needs per-chip calibration. That is why the code
  and devicetree carry a `VERIFY(hardware)` marker and the factor is a tunable
  Kconfig.
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
| **V4 rev 4.2** ✅ | 390k/100k (4.9) | GPIO37, **HIGH** | 4.9 × **1.030** | 2 MB quad | — (this document) |
| **V4 rev 4.3** | same *(expected)* | GPIO37, HIGH *(expected)* | 4.9 × 1.030 | 2 MB quad | Same Zephyr image as 4.2; wiring shared, only the FEM differs. Confirm against a 4.3 schematic if one is published. |
| **V4-R8** | same | **none** — divider always connected | 4.9 × **1.015** | 8 MB octal | `heltec_v4_r8` defines **no** `ADC_CTRL`. Our code already skips the gate when the `adc_ctrl` node is absent; set `…_BATTERY_CAL_PERMILLE=1015`. |

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
4. If using Zephyr's generic `voltage-divider` sensor binding rather than a
   direct ADC read, the `vbatt` node needs a `channel@0` child under its ADC
   parent carrying `zephyr,gain`/`zephyr,reference`/`zephyr,acquisition-time`/
   `zephyr,resolution` — properties `ADC_DT_SPEC_GET` expects on the channel
   node the `io-channels` phandle points at (see
   `zephyr/include/zephyr/drivers/adc.h`'s own DT examples). Without it the
   sensor device comes up `(DISABLED)` at boot with no build-time warning.
5. Unplugging a charge source (USB) from a board running on battery+USB
   produces a real, physical settling transient — the reading stays elevated
   for up to about a minute before relaxing to the true rest voltage. That's
   Li-ion internal-resistance polarization relaxing once charge current stops,
   not a firmware artifact — don't treat a reading taken in that window as
   ground truth when calibrating.

Carried patch 0001 (the `adc_esp32` GPIO-disconnect skip, see the correction
above) was very likely only ever masking the `&adc1`/`&adc0` pin bug rather
than protecting against a real "neighbouring RTC-IO pin" mechanism — hardware
evidence suggests it may now be redundant, but this needs a longer soak and a
plain-V4 confirmation (not just R8) before it's safe to remove.
