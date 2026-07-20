# On-device screen UI (CFB) — `CONFIG_MESHTASTIC_DISPLAY`

An **opt-in**, read-only status UI for the board's small monochrome panel,
built on Zephyr's Character Framebuffer (CFB). It is pure C, needs no phone
app, shell, C++, or extra west modules, and renders entirely from the existing
Meshtastic C getters. Off by default — headless builds are unaffected.

It is **navigable with the board's user button** (the PRG/USER key, wired as the
`sw0` alias): a launcher menu lists the pages, and a single button drives it —
**short press = next** (the highlight cycles in a loop), **long press = confirm**.
On a page, a `‹Back  Next›` footer lets you return to the launcher or flip to the
next page. On a board with no `sw0` alias — or with `CONFIG_MESHTASTIC_DISPLAY_BUTTON=n`
— it falls back to auto-cycling the pages on a timer.

## Enabling / building

The board must expose a `chosen { zephyr,display }` node. The Heltec WiFi
LoRa 32 **V4** already wires a 128×64 SSD1306 there (`…-common.dtsi`), so only
the overlay is needed (board pin map, peripheral polarities, and a pre-flash
checklist are in [`heltec-v4-hardware.md`](heltec-v4-hardware.md)):

```
./wt manifest ui-test          # if building from a worktree — REQUIRED
cd ui-test
west build -b heltec_wifi_lora32_v4/esp32s3/procpu samples/meshtastic \
    -- -DEXTRA_CONF_FILE=overlay-display.conf
```

`MESHTASTIC_DISPLAY` `select`s `DISPLAY` + `CHARACTER_FRAMEBUFFER`; the SSD1306
driver (`CONFIG_SSD1306`) auto-enables from the devicetree node.
`MESHTASTIC_DISPLAY_BUTTON` (default y) `select`s `INPUT`; the gpio-keys driver
auto-enables from the `sw0` node. `MESHTASTIC_DISPLAY_BATTERY` (default y)
`select`s `ADC`; the SoC ADC driver auto-enables from the `vbatt` node.
Tunables: `MESHTASTIC_DISPLAY_REFRESH_MS` (1000), `…_LONGPRESS_MS` (600),
`…_TIMEOUT_SECONDS` (30, 0 = never blank), `…_BATTERY_CAL_PERMILLE` (1045),
`…_PAGE_SECONDS` (4, fallback only), `…_STACK_SIZE` (2048), `…_PRIORITY` (10).

## Navigation

One button, two gestures:

- **short press → next.** Moves the highlight to the next item, cycling in a
  loop — the next launcher entry, or the other footer choice on a page.
- **long press → confirm.** Selects the highlighted item. It fires at the
  `LONGPRESS_MS` threshold while the button is still held (immediate feedback).

The UI has three levels. The **launcher** is a list of pages with a `>` cursor;
long-press opens the highlighted page. A **page** shows its data plus a
`‹Back  Next›` footer: long-press on **Back** returns to the launcher,
long-press on **Next** advances to the following page.

The **Nodes** page is special: its focus ring runs through each node (a `>`
cursor scrolls the windowed list) and then the Back/Next footer. Long-press on a
node opens a **node-detail** view — long name, node id + role, SNR, hops, and
`M`(MQTT)/`K`(public-key) flags. There, long-press **Next** flips to the next
node and **Back** returns to the list with the cursor on the node you were
viewing. The cursor is re-clamped every frame against the live NodeDB, so nodes
appearing or being evicted under you never point it out of range.

The panel **blanks after `TIMEOUT_SECONDS`** of no button activity to save power;
the next press wakes it *without* also navigating.

## How it works

- `src/meshtastic_display.c` gets `DEVICE_DT_GET(DT_CHOSEN(zephyr_display))`,
  initialises CFB, picks the **smallest registered font** that fits, reads the
  panel geometry, and starts a dedicated refresh thread.
- An `INPUT_CALLBACK_DEFINE` handler turns `sw0` press/release into short/long
  events (a `k_timer` fires the long press at the threshold) and posts them to
  the thread over a `k_msgq`. The thread waits on that queue with a `REFRESH_MS`
  timeout, so it reacts to a press immediately but still redraws live counters
  (TX/RX, RSSI, uptime) on each timeout tick.
- `meshtastic_display_init()` is called **last** in `meshtastic_init()` (after
  NodeDB/NodeInfo), guarded by `CONFIG_MESHTASTIC_DISPLAY`. A missing/failed
  panel is **non-fatal** — the mesh stack keeps running headless.

Pages (all read-only):

| Page | Shows | Source |
|---|---|---|
| Device | `<short> H<hop>` · `ID xxxxxxxx` · `F<MHz> <chan>` | `meshtastic_short_name()`, `meshtastic_get_node_id()`, `meshtastic_runtime_frequency/channel_name/hop_limit()` |
| Nodes | count, then `<cursor><*fav><short> <±snr>` per node; long-press for detail | `meshtastic_nodedb_count()` / `…_get_by_index()` |
| ↳ node detail | long name · `<id> <role>` · `<±snr> h<hops> M K` | `meshtastic_nodedb_get_by_index()` (long_name, num, role, snr, hops, via_mqtt, public_key) |
| Status | battery `V`/`%` (or `RSSI` if no `vbatt`) · `TX` · `RX` · uptime + `BLE` | `meshtastic_get_status()`, ADC `vbatt`, `k_uptime_get()` |
| Radio | channel util % · TX util % · MQTT (or `F<MHz>`) | `meshtastic_airtime_channel_util_percent()` / `…_tx_util_percent()`, `meshtastic_mqtt_is_connected()` |
| GPS | `Lat`/`Lon` (4 dp) · `Alt`m · sats — or "No GPS fix" | `meshtastic_position_get_current()` |
| Time | `YYYY-MM-DD` · `HH:MM:SS UTC` · uptime — or "Clock unset" | `meshtastic_clock_valid()` / `…_now_epoch()` |

The `*` before a node name marks a **favourite**. The **Radio**, **GPS** and
**Time** pages read from optional subsystems: they are compile-guarded, so a
build with `CONFIG_MESHTASTIC_AIRTIME`, `…_POSITION` or MQTT off shows a short
"off"/fallback line instead of a link error (`AIRTIME`/`POSITION` default y;
the clock is always compiled). Epoch→date uses an integer civil-from-days
conversion — no `time.h`, no floating point.

## Battery

`MESHTASTIC_DISPLAY_BATTERY` reads the board's `vbatt` voltage-divider
(ADC1 ch0 / GPIO1 on the V4) and shows voltage + estimated charge on the Status
page. The read path mirrors the upstream firmware
(`firmware/src/Power.{h,cpp}`, `variants/esp32s3/heltec_v4/variant.h`):

- **ADC.** `channel@0` is added to `&adc1` at `ADC_GAIN_1_4` (12 dB attenuation,
  matching upstream `ADC_ATTEN_DB_12`) with `ADC_REF_INTERNAL` (the esp32 driver
  requires it) and 12-bit resolution. The esp32 driver stores a calibrated
  pseudo-raw, so `adc_raw_to_millivolts_dt()` returns true pin millivolts.
- **Scaling.** `voltage_divider_scale_dt()` applies the devicetree ratio
  (490 kΩ / 100 kΩ = 4.9), then `…_BATTERY_CAL_PERMILLE` applies the empirical
  factor upstream folds into `ADC_MULTIPLIER` (V4 = 4.9 × **1.045** → 1045;
  V4-R8 = 1035). **VERIFY(hardware):** confirm against a known voltage.
- **ADC_CTRL gate.** The V4 divider sits behind ADC_CTRL (GPIO37, active-high);
  the UI drives it high only around each sample. Absent on the V4-R8 (always
  connected). The `adc_ctrl` power-domain is inert without `PM_DEVICE`, so the
  UI drives the pin directly.
- **Charge %.** An open-circuit-voltage lookup (single-cell LiPo curve
  `4190…3100 mV`, integer interpolation) identical to upstream; below 2600 mV it
  reports "no batt".
- **Sampling.** Cached 5 s, so button traffic doesn't re-sample (or re-toggle
  ADC_CTRL) on every redraw. Any read failure shows `Bat: n/a` and the mesh
  stack is unaffected.

Boards without a `vbatt` node compile the feature out and the Status page keeps
its `RSSI` line.

Every constant above (divider resistors, ADC pin, ADC_CTRL gate, calibration
factor, OCV curve) is sourced and cross-checked against the Heltec **V4 rev 4.2**
schematic, the Heltec Arduino examples, and the upstream firmware in
[`battery-provenance.md`](battery-provenance.md) — including an honest split of
which facts are independently corroborated versus single-source upstream tuning.
That provenance is scoped to **rev 4.2** (shared with 4.3); the **V4-R8** (no
ADC_CTRL gate, ×1.035) and **V3** (inverted ADC_CTRL polarity) differ and are
tabulated there for when we port them. Note the board charges its cell in
**hardware** (a dedicated charge IC); this UI only reads and displays, so a wrong
number is cosmetic, never a safety issue.

## Constraints & runtime caveats

- **Font/rows.** Zephyr's built-in CFB fonts start at 10×16, so a 128×64 panel
  gives **12×4 characters**. With button nav one row is the `‹Back  Next›`
  footer, leaving **3 content rows** — enough for the abbreviated fields. A
  smaller custom ~6×8 font would give 8 rows later.
- **No bitmaps.** CFB draws text + vector primitives only (no arbitrary image
  blit). Icons/logos are a reason to move to LVGL later, not CFB.
- **Panel power (verify on hardware).** The V4 OLED sits behind a `vext_ctrl`
  power-domain; if the screen stays dark on first boot, that rail not being
  energised is suspect #1.
- **Pixel format.** Init tries `MONO10` then falls back to `MONO01`.

## Portability

The renderer reads width/height at runtime and lays out in character rows, so a
different small mono panel needs **no code change** — only a board that points
`chosen { zephyr,display }` at it. The **160×80 Heltec Tracker V2** would get
16×5 characters automatically once its board defines that chosen display.

## Where this sits

This is the lightweight, native path (see the UI feasibility discussion): CFB
for a functional text UI now. The heavier alternative is the upstream
`meshtastic/device-ui` (LVGL, C++), whose protocol/controller layer talks the
same client protobuf API this firmware already speaks — worth adopting only
when rendering to a colour TFT, where its mature 320×240 UI lives.

## Verified on hardware (Heltec V4, over OTA)

Confirmed rendering on a bench Heltec V4, flashed via mcumgr OTA. One wrinkle
when PSRAM is left disabled (this build omits `overlay-psram.conf`): the display
adds ~4 KB internal DRAM, which pushes the full **net** variant (WiFi +
MQTT-over-TLS + OTA) to ~98 % — too tight to trust on a remote node. `overlay-uitest.conf` drops MQTT/TLS/GNSS (none needed to see the
UI) and brings it back to ~89 %:

```
./wt manifest ui-test
west build -p always --sysbuild -b heltec_wifi_lora32_v4/esp32s3/procpu \
    samples/meshtastic -- -DEXTRA_CONF_FILE="overlay-shell.conf;overlay-wifi-shell.conf;overlay-mqtt.conf;overlay-net.conf;overlay-ota.conf;overlay-display.conf;overlay-uitest.conf"
# then: mcumgr image upload zephyr.signed.bin -> image test <hash> -> reset -> image confirm <hash>
```

The display code itself is untouched by the slimming. The V4 has 2 MB quad
PSRAM; enabling it (`overlay-psram.conf`) is the other way to reclaim internal
DRAM — though WiFi-heap-to-PSRAM routing is still unsolved (only the NodeDB is
relocated today), so the full net image can stay tight even with PSRAM on.

