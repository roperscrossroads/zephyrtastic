# On-device screen UI (CFB) — `CONFIG_MESHTASTIC_DISPLAY`

An **opt-in**, read-only status UI for the board's small monochrome panel,
built on Zephyr's Character Framebuffer (CFB). It is pure C, needs no phone
app, shell, C++, or extra west modules, and renders entirely from the existing
Meshtastic C getters. Off by default — headless builds are unaffected.

This is an early **prototype**: it auto-cycles a few pages on a timer (no button
input yet) and has only been built, not yet verified on hardware.

## Enabling / building

The board must expose a `chosen { zephyr,display }` node. The Heltec WiFi
LoRa 32 **V4** already wires a 128×64 SSD1306 there (`…-common.dtsi`), so only
the overlay is needed:

```
./wt manifest ui-test          # if building from a worktree — REQUIRED
cd ui-test
west build -b heltec_wifi_lora32_v4/esp32s3/procpu samples/meshtastic \
    -- -DEXTRA_CONF_FILE=overlay-display.conf
```

`MESHTASTIC_DISPLAY` `select`s `DISPLAY` + `CHARACTER_FRAMEBUFFER`; the SSD1306
driver (`CONFIG_SSD1306`) auto-enables from the devicetree node. Tunables:
`MESHTASTIC_DISPLAY_REFRESH_MS` (1000), `…_PAGE_SECONDS` (4), `…_STACK_SIZE`
(2048), `…_PRIORITY` (10).

## How it works

- `src/meshtastic_display.c` gets `DEVICE_DT_GET(DT_CHOSEN(zephyr_display))`,
  initialises CFB, picks the **smallest registered font** that fits, reads the
  panel geometry, and starts a dedicated refresh thread.
- The thread redraws the current page every `REFRESH_MS` and advances to the
  next page every `PAGE_SECONDS`.
- `meshtastic_display_init()` is called **last** in `meshtastic_init()` (after
  NodeDB/NodeInfo), guarded by `CONFIG_MESHTASTIC_DISPLAY`. A missing/failed
  panel is **non-fatal** — the mesh stack keeps running headless.

Pages (all read-only):

| Page | Shows | Source |
|---|---|---|
| Device | short name · `ID xxxxxxxx` · `F <MHz>` · `CH <name> H<hop>` | `meshtastic_short_name()`, `meshtastic_get_node_id()`, `meshtastic_runtime_frequency/channel_name/hop_limit()` |
| Nodes | count, then `<short> <±snr>dB h<hops>` per node | `meshtastic_nodedb_count()` / `…_get_by_index()` |
| Status | `TX` · `RX` · `RSSI` · uptime + `BLE` flag | `meshtastic_get_status()`, `k_uptime_get()` |

## Constraints & runtime caveats

- **Font/rows.** Zephyr's built-in CFB fonts start at 10×16, so a 128×64 panel
  gives **12×4 characters** — enough for the prototype's abbreviated fields. A
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

