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

**If any of `<files>` already has an earlier carried patch applied** (check
`grep -l <file> *.patch` in this dir first), a plain `git diff <files>` is
**wrong** — it captures the earlier patch's content *and* the new change
merged into one diff, which then double-applies (or conflicts) when
`west patch apply` tries to layer it on top of the already-applied earlier
patch. `zephyr/`'s working tree has no commit boundary between "pristine",
"pristine + earlier patches", and "+ your new edit" to diff against directly
— reconstruct the middle state instead (found the hard way writing 0005,
which touched files 0002/0003/0004 already covered):

```bash
# 1. Pristine copies of just the files you touched, at the pinned commit
#    (git -C zephyr log -1 -- <file>, or the version twister prints).
git -C zephyr show <pinned-sha>:<path> > /tmp/a/<path>   # per file

# 2. Apply every EARLIER patch touching those files, in patch-number order
#    (grep -l <file> main/zephyr/patches/*.patch to find which ones)
cd /tmp/a && patch -p1 < .../000N-earlier.patch   # repeat per earlier patch

# 3. Diff the reconstructed baseline against the CURRENT live file -- this
#    isolates exactly your new change, nothing from the earlier patches
git diff --no-index --src-prefix= --dst-prefix= /tmp/a/<path> zephyr/<path>

# 4. Sanity-check before trusting it: does the new patch apply cleanly on a
#    freshly-reconstructed pristine+earlier-patches tree, and does the
#    result match the live working tree byte-for-byte?
```
Files with no earlier patch (check the same `grep -l`) need none of this —
a plain `git diff <file>` is already exactly right for those.

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

### 0005-sx126x-cad-pa-clamp-agc-reset-rx-boost.patch

Four radio-reliability fixes, found auditing this driver against upstream
Meshtastic/RadioLib **source** rather than re-reading the datasheet in
isolation. Each closes a gap where the driver's config *looked* correct but
was silently a no-op or missing entirely.

**(1) CAD** (`sx126x_do_cad`, wired into `sx126x_lora_send_async` before
`SetTx`). The driver's `lora_modem_config.cad.mode = LORA_CAD_MODE_LBT`
contract ("`lora_send()` performs CAD before transmitting and returns
`-EBUSY` if the channel is busy") was accepted but never implemented — the
app (`meshtastic_radio.c`) had been setting `cad.mode = LBT` believing it got
real hardware listen-before-talk; every TX actually went out with **zero**
channel sensing. Implements the real `SetCadParams(0x88)`/`SetCad(0xC5)`
sequence with `CAD_DONE`/`CAD_ACTIVITY_DETECTED` IRQ handling — polling DIO1
directly (matching RadioLib's own `scanChannel()`) rather than the async
IRQ+workqueue pipeline, since this is a short, synchronous, inline check, not
an operation needing its own completion callback. Auto-derived parameters
(`symbol_num`/`detection_peak`/`detection_minimum = 0`) resolve to RadioLib's
own SX1262 defaults — confirmed against RadioLib source 2026-08-06, not
assumed: 4-symbol window, `detPeak = spreadingFactor + 13` (SF-dependent),
`detMin = 10`, always exits to `STDBY_RC`. Restores the driver's standard
TX/RX/timeout/CRC-err DIO1 mask afterward regardless of result — `SetCadParams`
own IRQ config otherwise displaces it, and the subsequent `SetTx`'s `TX_DONE`
IRQ depends on it.

**(2) PA-clamping errata fix** (`sx126x_hal_fix_pa_clamping`, `sx126x_hal.c`).
The SX1262/SX1268 datasheet (ch. 15 "Known Limitations" §15.2) documents an
overly-eager PA over-current clamp that silently caps actual TX power below
what's configured — worse at high power / antenna mismatch, no error raised
anywhere. RadioLib applies this unconditionally for every SX1262
(`fixPaClamping()`). This exact fix **already existed** in this same driver
tree's STM32WL HAL variant (`sx126x_hal_stm32wl.c`) but was never ported to
the discrete-chip path Heltec V4 and similar boards actually use.

**(3) Periodic AGC reset** (`sx126x_reset_agc` + a `meshtastic_radio.c`
timer, 60 s default via new `CONFIG_MESHTASTIC_AGC_RESET_INTERVAL_MS`).
Upstream Meshtastic runs this every `AGC_RESET_INTERVAL_MS` because — per
upstream's own commit history — the SX126x's internal AGC can get stuck and
RX sensitivity silently degrades, never recovering until reboot. A plain
standby cycle does not reset it; only a warm-sleep power-cycle does, which
this driver already had via `sx126x_set_sleep()`/`sx126x_ensure_ready()`
(`SLEEP_WARM_START` retains config, so this is cheap) — just never invoked
for this purpose. Also re-applies the RX-sensitivity register patch (`0x8B5`
bit 0) upstream sets alongside its own reset. The app owns pausing/resuming
continuous async RX around the chip-level cycle, the same pattern already
used around a TX.

**(4) RX-boosted-gain runtime control** (`sx126x_set_rx_boosted_gain`). The
driver only ever read the devicetree `rx-boosted` default
(`hal_config->rx_boosted`); `config.lora.sx126x_rx_boosted_gain` — a real,
phone-app-settable proto field, present in this port's `config.proto` but
never read anywhere in the app layer either — had no effect whatsoever.
Fixed at both ends: the app now reads it (`meshtastic_config_store.c`) and
pushes it at boot (`meshtastic_radio_init`; a LoRaConfig change already
requires a reboot to take effect, so no live-apply path is needed); the
driver now tracks it in persistent runtime state (`data->rx_boosted`) rather
than the devicetree constant, since `sx126x_lora_config()` re-applies
whichever one is the source of truth on **every** call (before every TX) — a
one-shot override without this would have been silently clobbered back on
the very next transmit.

> **Verified:** 220/220 native_sim protocol tests green (unaffected, as
> expected — native_sim doesn't compile this driver); real ESP32-S3 builds
> for both `heltec-v4` and `heltec-v4r8` (`v4-unified` profile) succeed
> cleanly with zero warnings in any changed file. **Not yet verified on-air**
> — CAD/AGC/PA-clamp/RX-boost all need real RF to prove out (`native_sim` has
> no PHY).

`upstreamable: true` — all four are generic driver-correctness/completeness
fixes citing upstream RadioLib/Meshtastic source and the Semtech datasheet,
no board-specific coupling.

### 0006-sx126x-cad-agc-diagnostic-counters.patch

Diagnostic counters for the CAD and periodic AGC-reset mechanisms 0005
added, requested for the live bench soak of 0005 itself — there was
previously no way to answer "how often does CAD find the channel busy, has
the DIO1/irq_work timeout race recurred, has AGC-reset ever failed" except
grepping node logs by hand.

Eight module-scope `uint32_t` counters (`sx126x.c`), matching the existing
`sx126x_busy_streak` precedent (`sx126x_hal_common.c`) rather than atomics —
every increment site already runs under `data->lock`, and these are
soak-test visibility, not synchronization state:

- `cad_clear` / `cad_busy` / `cad_timeout` / `cad_error` — `sx126x_do_cad()`'s
  four possible outcomes, incremented at its single classification point
  (channel clear, activity detected, DIO1 poll timeout — the exact symptom
  0005's own fix closed, kept counted specifically to watch for recurrence —
  or an SPI/comms error before CAD completed).
- `agc_reset_ok` / `agc_reset_fail` / `agc_reset_skipped` / `agc_patch_fail`
  — `sx126x_reset_agc()`'s outcomes: skipped means a TX was already in
  flight (not a failure, matching how the caller already treats it as
  benign); patch-fail is the sensitivity-register-patch soft-failure that
  0005 already logs a warning for and swallows — now also counted, since
  "ok" still increments alongside it (the core cycle did succeed).

Exported via plain accessor + reset functions (`sx126x_*_count_get()`,
`sx126x_cad_agc_stats_reset()`), in the same style as
`sx126x_hal_busy_timeout_streak()`/`_reset()`. Wired through a
`CONFIG_LORA_SX126X`-gated chip-agnostic wrapper in `meshtastic_radio.c`
(`meshtastic_radio_cad_clear_count()` etc. — a build without this radio
reads a flat 0 rather than needing its own guard), surfaced as two new
lines on `meshtastic sched stats` and reset by `sched stats reset`
alongside the existing counters.

> **Verified:** 220/220 native_sim protocol tests green (these counters are
> inert there — `CONFIG_LORA_SX126X` unset, wrapper reads back 0); real
> ESP32-S3 builds for both `heltec-v4` and `heltec-v4r8` succeed cleanly with
> zero warnings; live-queried via `meshtastic sched stats` over telnet on all
> three bench nodes post-flash — CAD/AGC counts increment correctly,
> `sched stats reset` zeroes them correctly.

`upstreamable: false` — these counters and their shell surface are specific
to 0005's own CAD/AGC-reset implementation, which itself isn't upstream
(upstream Meshtastic runs on RadioLib against a different driver entirely;
there's no equivalent counter surface to align with).

### 0011-sx126x-expose-rx-boosted-gain-staged-and-applied.patch

Records what was actually written to the chip's `RX_GAIN` register, and
exposes both that and the staged request:
`sx126x_rx_boosted_staged_get()` / `sx126x_rx_boosted_applied_get()`.

**Why:** `sx126x_set_rx_boosted_gain()` (added by 0005) deliberately only
*stages* its value — the register is valid to write in STANDBY only
(datasheet §9.6), so the request reaches the silicon on the next
`sx126x_lora_config()`. That is the right design, but it makes one failure
mode completely silent: a `lora_config()` that errored **before** the gain
step leaves the chip on the old gain while every layer above believes the
new one is in force. The cost is 2–3 dB of receive sensitivity, and nothing
anywhere reports it — the node simply hears less than it should.

`data->rx_boosted_applied` is assigned only **after** the write succeeds, so
a failed write leaves the previous true value standing rather than an
optimistic one. `data->rx_gain_valid` separates "never written" from
"wrote false", and both getters return `false` instead of inventing a value
when it is not knowable — "never configured" and "configured off" are
different states, and reporting the first as the second would be worse than
admitting the gap.

Consumed by `main/src/meshtastic_rf_path.c` (`meshtastic rf`), which declares
**weak fallbacks** for both getters so the firmware still links — and reports
the value as `unknown` — against a Zephyr tree that has not had this patch
applied. A build that silently failed to link would be obvious; one that
linked and reported a fabricated answer would be much worse.

> **Verified:** patch isolated via the reconstructed-baseline procedure above
> (pristine + 0002/0003/0004/0005/0006/0008/0009 + this patch reproduces both
> live files byte-for-byte); `git apply --reverse --check` clean against the
> live tree; native_sim suites green (inert there — `CONFIG_LORA_SX126X`
> unset, the weak fallbacks report unknown, which the shell tests assert).

`upstreamable: true` — the staged/applied distinction is a property of the
driver's own design, not of this project, and every user of
`sx126x_set_rx_boosted_gain()` has the same blind spot.

### 0012-nordic-qspi-nor-write-block-size.patch

Declares `write-block-size` and `erase-block-size` as optional properties on
`nordic,qspi-nor.yaml`, matching `soc-nv-flash.yaml`'s pair of the same name.
Neither this binding nor its `jedec,spi-nor-common.yaml` base declared them
at all.

**Why:** the XIAO nRF52840 coredump-to-flash work (Phase 5, diagnostics-
baseline plan) puts a `coredump_partition` on the XIAO's QSPI `p25q16h` chip
(`main/samples/meshtastic/mcuboot-xiao/partitions.dtsi`) — the only free
flash space on that board; all 1 MB of internal flash is already spoken for
by MCUboot's own layout. `CONFIG_DEBUG_COREDUMP_BACKEND_FLASH_PARTITION`
(`subsys/debug/coredump/coredump_backend_flash_partition.c`) reads
`write-block-size` off the coredump partition's **grandparent** via
`DT_PROP(FLASH_CONTROLLER, write_block_size)` — for a QSPI-backed partition
that grandparent is the flash chip node itself, `p25q16h`. Without the
property declared, devicetree validation refuses the build outright
(`'write-block-size' appears in ... but is not declared in 'properties:'`),
not merely a soft warning — no board in this Zephyr tree had combined QSPI
NOR with `coredump_partition` before, so nothing had ever needed it.

A second `"soc-nv-flash"` compatible on the `p25q16h` node was tried first
(that binding does declare both properties) and does **not** work: Zephyr
picks exactly one binding per node for property validation, not a union of
every listed compatible's schema — it only moved the identical error onto
`soc-nv-flash`'s much smaller property set instead of resolving it.

> **Verified:** both XIAO build modes succeed (`--sysbuild`, with
> `write-block-size = <1>` now set on `&p25q16h` and
> `DEBUG_COREDUMP_BACKEND_FLASH_PARTITION=y` active; plain/legacy, unaffected
> since it never sets the property at all) and `heltec_wifi_lora32_v4_r8`
> (unaffected — a different flash binding entirely) builds clean; native_sim
> 1014/1014 across 46 configs (this binding is nRF-only, not exercised there
> either way).

`upstreamable: true` — a generic MTD property gap, not specific to this
project or to QSPI NOR specifically: any board combining
`coredump_partition` with a flash whose binding omits `write-block-size`
hits the same wall.

### 0013-sx126x-latch-rx-activity-flags-for-actively-receiving.patch

Enables `PREAMBLE_DETECTED | HEADER_VALID | HEADER_ERR` in the SX126x IRQ
**enable** mask while leaving the DIO1 mask exactly as it was (the four
terminal events), so the receive-activity flags latch in the status register
without raising an interrupt; adds `sx126x_rx_activity()` /
`sx126x_rx_activity_clear()` to read and retire them; and clears them before
every `SET_RX` so a re-arm starts clean (RadioLib's `startReceiveCommon()`
does the same).

**Why:** this is the chip half of upstream Meshtastic's
`isActivelyReceiving()`, which gates three decisions — a transmit must not
key up over a packet being demodulated ("doubly bad: we drop the packet on
the way in, and no one outside will like the one we send"), the periodic AGC
reset must not tear RX down mid-packet, and the noise floor is not sampled
under a signal. RadioLib configures the chip with these flags in the IRQ mask
and `RX_DONE` alone on DIO1, then reads the status register. Our mask never
had them, so they never latched, so no layer could ask; both the AGC reset
and every transmit could land on an inbound frame. Found by the 2026-09-02
RF parity audit (`docs/RF-PARITY-PLAN.md` §1.1 in the tooling repo).

The timing that separates a real reception from a stale flag (twice the
preamble time without a header; the airtime of a maximum-length frame
without RX_DONE) lives in `main/src/meshtastic_radio.c`
(`meshtastic_radio_actively_receiving()`), because it needs the modem the
radio is on. One deliberate divergence from upstream there: a detection
judged false is *cleared* via `sx126x_rx_activity_clear()` rather than left
latched to be re-timed on every call.

Consumed through `__weak` fallbacks in `meshtastic_radio.c` (same pattern as
0011), so an unpatched tree still links and reports "unknown" / never
receiving. Verified on the V4 class-4 image; the `RX activity` row of
`meshtastic rf` shows the raw flags and the two timing rules' verdicts.

Built with the reconstruction recipe above (pristine `6072d4880d` +
0002/0003/0004/0005/0006/0008/0009/0011), and checked to reproduce the live
file byte-for-byte.

### 0014-sx126x-recover-a-lost-tx-done-edge-instead-of-stalling-15s.patch

`sx126x_lora_send()` waits for TX completion in 1 s slices and calls
`sx126x_poll_dio1()` between them, instead of a single 15 s `k_msgq_get()`.

**Why:** a TX_DONE edge can be missed (this driver documents two ways: a
`ClearIrqStatus` racing the ISR, and the GPIO block being down under light
sleep). The single wait turned every lost edge into a 15 s stall — the
frame already on the air, the chip in STDBY_RC, the caller holding the radio
semaphore — and then reported `-ETIMEDOUT` for a transmit that had
succeeded. On the bench (2026-09-02) that was the "1 failed TX" on two
Heltecs, and on the XIAO kits it was a **reboot**: the periodic AGC reset
runs on the system workqueue and blocked behind the held radio; the DFU
boot guard's watchdog feeder shares that workqueue and starved; the 15 s
hardware watchdog fired. Every XIAO watchdog reset that night sat on a 60 s
AGC tick (60, 120, 300, 3060 s).

`sx126x_poll_dio1()` is exactly the level check the edge missed; if DIO1 is
asserted it submits the IRQ work, which reads and clears the real status and
completes the send through the normal path inside a second. The 15 s cap is
kept as the last resort. The companion fix in `meshtastic_radio.c` makes the
AGC reset's semaphore wait bounded, so the workqueue can never be held
hostage by the radio again regardless.

Built on pristine `6072d4880d` + 0002/0003/0004/0005/0006/0008/0009/0011/0013
with the reconstruction recipe and checked byte-for-byte.

### 0015-stream-flash-erase-frontier-from-page-end-not-page-count.patch

`stream_flash_erase_to_append()`: after erasing a page, set `erased_up_to` from
the page's real end (`page.start_offset + page.size - ctx->offset`) instead of
`+= page.size`.

**Why:** `erased_up_to` is relative to `ctx->offset`, which need not be
page-aligned. For a stream that starts mid-page, adding a page size overstates
the erased frontier by `offset % page`: the write that straddles the next page
boundary passes the check without an erase, and the append after it erases that
page, wiping the tail the straddling write had just landed there. The coredump
flash backend starts its stream 16 B into its partition (after its header), so
on the bench (2026-09-03, ESP32-S3 members with the 512 KB coredump partition)
**every 4 KB page of a stored dump began with 16 erased bytes** — 84 of 84
boundaries in a 346 KB dump — and the header's checksum never verified, so
`coredump find` reported nothing while esptool could read a dump gdb walked
happily. Image slots are page-aligned, which is why `img_mgmt`'s progressive
erase (the other user of this option) never showed it.

**Upstreamable:** yes — pure logic fix, no behaviour change when `ctx->offset`
is page-aligned. Regression-tested with `tests/subsys/storage/stream` on
native_sim.

### 0016-xtensa-backtrace-on-every-espressif-xtensa-soc.patch

`arch/xtensa/Kconfig`, `arch/xtensa/core/xtensa_backtrace.c`. Upstream gates
`XTENSA_ENABLE_BACKTRACE` (default y) on `SOC_SERIES_ESP32`, so the ESP32-S3
never had it; the code is chip-generic. **Upstreamable as-is.** Found while
instrumenting the class-B hang (tooling `RF-PARITY-PLAN.md` §8.16).

### 0018-hal-espressif-esp32s3-window-spill-for-xtensa-backtrace.patch

**First patch on a module other than `zephyr/`** (`module: hal_espressif`, path
`modules/hal/espressif`). One `zephyr_sources_ifdef(CONFIG_XTENSA_ENABLE_BACKTRACE
...)` line in the S3 CMake, compiling the esp32 tree's `windowspill_asm.S` so
0016's backtrace helper links. Without it the S3 link fails on
`xthal_window_spill`. **Upstreamable with 0016.**

