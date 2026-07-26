# Build profiles (canonical overlay targets)

The sample's Kconfig overlays are organised as **one self-contained overlay per
build target** rather than a pile of fragments composed by build knobs. This
replaced a 24-file / 11-knob / 13-stage composition (2026-07-26).

## The profiles

Each `samples/meshtastic/overlay-<profile>.conf` is the flattened, self-contained
config for one target. Built via the workspace justfile:

```
PROFILE=v4-net just dist    # Heltec V4: WiFi/net + OTA + netlog + display + PM, MQTT off  (deploy / rzr1)
PROFILE=v4-ble just dist    # Heltec V4: BLE PhoneAPI + display + PM                        (bench / rzr2)
PROFILE=v3-net just dist    # Heltec V3: WiFi/net + OTA + netlog + PM, no display/PSRAM
PROFILE=v4-netserial …      # Heltec V4: serial PhoneAPI test — BLOCKED (see below)
```

`PROFILE` presets the board + the config knobs; an unknown/empty `PROFILE` errors
with the valid list. Explicit env vars still override a preset (e.g.
`PROFILE=v4-net DISPLAY=0 just dist`).

## Layered overlays

- **Local infra** — `overlay-net-local.conf` (**gitignored**) carries the real
  broker/collector/NTP IPs and is layered last on the `*-net` profiles for local
  builds (`LOCAL=1`, default). It is slimmed to *only* IP/hostname strings, so it is
  order-independent; everything functional lives in the tracked profile. A public
  build (`LOCAL=0`) uses the sanitized `192.0.2.x` placeholders baked into the profile.
- **Diagnostics (opt-in `EXTRAS`)** — layer extra overlays on any profile:
  ```
  PROFILE=v4-net EXTRAS="overlay-threadanalyzer.conf" just dist
  ```
  Kept as separate overlays: `overlay-threadanalyzer.conf` (per-thread stack
  high-water logging), `overlay-pm-quiet.conf` (quiet PM-residency rig),
  `overlay-uitest.conf` (DRAM-slimming for display-without-PSRAM).

## Regenerating a profile

A profile is the ordered concatenation of the fragments it used to compose (see the
`# ===== overlay-*.conf =====` section markers inside each file). To change shared
config you now edit each profile that needs it — the trade the flattening makes
(readable duplication over an opaque composition chain).

## v4-netserial is blocked

`overlay-v4-netserial.conf` is captured but currently **fails to configure**: a
Zephyr Kconfig *"Dependency loop"* through `I2C → FDC2X1X → EXTERNAL_LIBC → libc`.
It is **pre-existing** (the same config as the old `netserial` variant), not caused
by the flattening — the fuller `v4-net` profile happens to avoid the cycle (its
display/I2C config resolves a choice), and `CONFIG_PICOLIBC=y` did not break it.
Needs a separate Kconfig fix before the profile builds.
