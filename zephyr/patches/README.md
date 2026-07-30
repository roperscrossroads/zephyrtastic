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

### 0003-sx126x-busy-timeout-diagnostics.patch

Diagnostics for the recurring BUSY timeout (`agents-qnpp` / `agents-3ujx`).
`sx126x_hal_wait_busy()` gains three diagnostic tags so a timeout names exactly
what stalled: an **`opcode`** (splits **TX-path** `SET_TX_PARAMS`/`WRITE_BUFFER`
from **RX-path** `GET_IRQ_STATUS`/`READ_BUFFER` and config — also the `agents-la0m`
CAD/LBT discriminator); a **pre/post phase** (`pre` = the wait *before* the SPI
transfer, so a timeout means the *prior* op left BUSY high; `post` = after, so this
command did not complete); and **`prev_opcode`** (the last command actually issued —
on a `pre` timeout, the culprit that left BUSY high). The timeout `LOG_WRN` moves
into a **`__weak`** `sx126x_hal_busy_timeout_report(dev, opcode, timeout_ms, post,
prev_opcode)` (declared in `sx126x_hal.h`), so an application can override it with a
strong symbol.

**Why:** `meshtastic_radio.c` provides the strong override (PM builds), appending
the light-sleep **wake sequence number + ms-since-wake** (from `powermon`) so a
BUSY timeout can be placed relative to a `PM_STATE_STANDBY` wake **without
wall-clock time** — the node's log timestamps read 1970 until SNTP/GNSS seeds the
clock. If timeouts cluster in the first few ms after a wake, the stall is a
light-sleep race rather than a chip/SPI fault. Pure diagnostics — no behavioural
change to the radio.

> **App-coupled, unlike 0001/0002.** The `__weak` symbol has a default in the
> driver, so a build with no override (native_sim, non-PM) still links and logs the
> opcode. The wake correlation only appears when `meshtastic_radio.c` is linked in
> a `CONFIG_PM` build.

`upstreamable: false` as-is — the weak-hook contract is app-specific. Revisit /
trim once the root cause is known.

### 0004-sx126x-busy-timeout-fix-and-recovery.patch

Fix **and** recovery for the recurring 1 s BUSY timeout (`agents-3ujx`), pinned via the
0003 diagnostics (`op=0x80 pre, prev=0x84/SET_SLEEP`). Two parts:

**(1) The fix — `sx126x_set_standby()`.** With `CONFIG_LORA_SX126X_NATIVE_SLEEP` the chip
rests in **sleep** between operations, holding BUSY **high** until an NSS-edge wake — but
`SET_STANDBY`'s `write_cmd` waits for BUSY **low** before it can send, so any teardown/
transition path reaching `set_standby` without an intervening wake stalls a full second.
It now issues a raw NSS-edge wake first, gated on `state == SLEEP || sx126x_hal_is_busy()`.
The **state** check also covers the sleep-*entry* window right after `SET_SLEEP` where BUSY
reads transiently low (which `is_busy` alone missed — a residual `op=0x80 **post**`).
Skipped (no extra SPI) when genuinely awake. Fixes every `set_standby` caller at once.

**(2) The recovery (`agents-oieb`).** `sx126x_hal_wait_busy` keeps a module-scope count of
**consecutive** BUSY timeouts (cleared on any success); `sx126x_lora_config()` resets the
radio (`sx126x_chip_init`: RESET-pin pulse + re-init) once the streak crosses
`SX126X_BUSY_RECOVERY_THRESHOLD`, then re-applies the full modem config — so a *wedged*
radio (BUSY stuck) is bounded to a few seconds instead of hanging forever. The app re-runs
`lora_config` every TX, so a wedge clears within a TX cycle.

> **First attempt missed.** The fix initially targeted `sx126x_duty_cycle_stop()`, but the
> app uses continuous RX (`.recv_async`), not the duty-cycle path — a dead target. The 0003
> soak caught it (timeouts unchanged, same `prev=0x84`) before it was believed fixed; the
> fix moved to `set_standby`, the common choke point.

`upstreamable: true` — generic driver-correctness fixes, no board coupling.

`upstreamable: true` — a generic driver-correctness fix, no board coupling.
