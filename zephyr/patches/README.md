# Carried Zephyr patches

Local patches applied on top of the upstream Zephyr tree (`zephyr/`, a west
project pinned to `main` in `../west.yml`). `west update` restores each project
to its manifest revision and **discards in-tree edits**, so anything we change in
`zephyr/` must be captured here or it is lost on the next update.

## Workflow

```bash
# from the workspace root, venv active (source .venv/bin/activate)
west patch list      # show carried patches (reads ../patches.yml)
west patch apply     # re-apply all patches after a `west update`
west patch clean     # revert patched projects to pristine
```

Manifest: [`../patches.yml`](../patches.yml). Patch files here are `git diff`
output with paths relative to the target module's root (`module: zephyr`).

## To add / refresh a patch

```bash
cd zephyr
git diff <files> > ../main/zephyr/patches/<NNNN-name>.patch
sha256sum ../main/zephyr/patches/<NNNN-name>.patch     # -> sha256sum in patches.yml
# add/update the entry in main/zephyr/patches.yml
```

The `sha256sum` in `patches.yml` must match the file, or `west patch apply`
refuses it. Verify a patch matches the current tree without a destructive
clean/apply cycle: `cd zephyr && git apply --reverse --check ../main/zephyr/patches/<file>`.

## Current patches

### 0001-adc-esp32-skip-disruptive-gpio-disconnect.patch

Drops the `gpio_pin_configure_dt(gpio0, io_num, GPIO_DISCONNECTED)` call in
`adc_esp32`'s channel-setup.

**Why:** on the ESP32-S3 that call runs `rtcio_hal_function_select()` +
interrupt-config touches on the **shared gpio0 / RTC-IO controller**, which
silently knocks out an unrelated **digital** GPIO interrupt on a neighbouring
RTC-capable pin (0–21) on the same controller. On the Heltec **V4** the neighbour
is the SX1262 **DIO1 (GPIO14)**, right next to the ADC1_CH0 battery pin
**(GPIO1)**: the first battery ADC channel-setup killed the radio's
TX-done/RX-done IRQ — **both TX and RX dead** the moment the display's battery
feature initialised. Upstream ESP-IDF `adc_oneshot` never touches the GPIO here;
the SAR ADC reads the undriven analog input without it.

Confirmed on hardware: with the call dropped, the V4 display+battery UI and the
LoRa radio coexist (`tx ok` / `rx decoding`, full UI enabled).

`upstreamable: true` — the side effect on neighbouring pins is a general
`adc_esp32`/`gpio_esp32` bug, not V4-specific.

Full diagnosis: `~/notes/local/infra/runbooks/friction-log.md`
(2026-07-20) and `TMP-BAT-FIX-OPTIONS.md` at the workspace root.

### 0002-sx126x-poll-dio1-on-light-sleep-wake.patch

Adds `void sx126x_poll_dio1(const struct device *dev)` to the **native** sx126x
driver (`drivers/lora/native/sx126x/sx126x.c`): if DIO1 is asserted, submit the
same work the edge ISR would have.

**Why:** DIO1 is an **edge** interrupt, but ESP32-S3 `PM_STATE_STANDBY` powers the
GPIO peripheral down, so a frame received while asleep leaves DIO1 latched **HIGH
with no fresh edge** — `dio1_isr()` never fires and the frame is never read. The
board DT arms an EXT1 level-HIGH wake on DIO1 (`GPIO_INT_WAKEUP` on the
`dio1-gpios` cell), so the SoC *wakes*; this helper lets it *read* the pending
frame. The app calls it from a PM wake hook (`pm_notifier .state_exit` in
`meshtastic_radio.c`, gated on `CONFIG_PM` + `CONFIG_LORA_SX126X`) with
`mt.lora_dev`. Idempotent; the work handler reads+clears the IRQ over SPI
regardless of the missed edge. See `docs/light-sleep-governor.md`.

> **LOAD-BEARING for PM builds.** Unlike 0001 (dormant until the battery ADC
> returns), this symbol is *referenced* by `meshtastic_radio.c` in any
> `CONFIG_PM` + `CONFIG_LORA_SX126X` build. Skip `west patch apply` after a
> `west update` and such a build fails to **link** (`undefined reference to
> sx126x_poll_dio1`). Non-PM builds and native_sim are unaffected (the reference
> is `#if`-compiled out).

`upstreamable: true` — a generic light-sleep-safety helper for the edge-IRQ
radio, no board coupling in the driver.
