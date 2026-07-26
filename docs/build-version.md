# Build version tag

A short, developer-facing **build identity** baked into every image so you can tell
*which build a bench node is running* — the recurring "is this the old or new
firmware?" question. It is deliberately **separate** from the phone-protocol
`firmware_version` string (`"2.7.4.zephyr"`, set in
[`src/meshtastic.c`](../src/meshtastic.c) `meshtastic_fill_device_metadata()`), which
the phone app negotiates against and should stay stable.

## What it is

Two values, captured at build time by
[`src/cmake/gen_build_id.cmake`](../src/cmake/gen_build_id.cmake):

| Macro / accessor | Value | Example |
|---|---|---|
| `MESHTASTIC_BUILD_ID` / `meshtastic_build_id()` | `git describe --always --dirty --abbrev=8` | `92fa2720` or `92fa2720-dirty` |
| `MESHTASTIC_BUILD_TIME` / `meshtastic_build_time()` | UTC build time `MMDD-HHMM` | `0726-1632` |

The **`-dirty`** suffix is the important part during development: it flags an image
built from an **uncommitted** tree (which is most bench builds), so you never confuse
a committed build with a working-tree experiment.

## Where it shows

- **CLI:** `meshtastic version` →
  ```
  build:  92fa2720-dirty (built 0726-1632 UTC)
  board:  heltec_wifi_lora32_v4/esp32s3/procpu
  zephyr: 4.x.y
  ```
- **UI:** a `b:<id>` row on the OLED **device page** (row 3). Visible at a glance;
  fits since the compact 6×9 font is in.
- **Boot log:** `meshtastic build <id> (built …)` at the top of `meshtastic_init()`,
  so a boot-log capture (e.g. over serial before the console sleeps) identifies the
  image.

## How it is wired (so a rebuild is always fresh)

`src/CMakeLists.txt` runs `gen_build_id.cmake` via an always-run custom target
(`meshtastic_build_id_gen`) that the `meshtastic` library depends on, writing
`meshtastic_build_id.h` into the module's generated include dir. The header is only
rewritten when its contents change, and the include is isolated in
[`src/meshtastic_build.c`](../src/meshtastic_build.c), so a rebuild recompiles **only
that one file** (plus a relink) rather than every consumer.

Because the id comes from `git describe`, a west checkout with no tags yields the bare
short SHA (no `g` prefix); a tagged release would show `<tag>-<n>-g<sha>`. If git is
unavailable at build time the id falls back to `nogit`.
