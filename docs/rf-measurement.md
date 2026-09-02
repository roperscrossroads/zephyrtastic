# RF gain path: diagnostics and measurement

`CONFIG_MESHTASTIC_RF`. Two halves that answer two different questions:

- **`meshtastic rf`** — *what is the radio actually doing?* Pure readback, no state.
- **`meshtastic rf hist` / `peers`** — *is this setting helping?* Accumulates received
  frames into distributions a comparison can be run against.

The companion explainer for the RF concepts themselves (what an LNA is for, why the FEM
gain table exists, which of our boards has what) is the **Antenna to Bits** reference.
This document covers the firmware: what it measures, why the numbers are the shape they
are, and the traps that will produce confidently wrong answers if ignored.

---

## 1. Why "effective" is the whole point

Three layers can supply a value for the same setting, and they routinely disagree. The
report exists because nothing else in the firmware surfaces the disagreement.

**`sx126x_rx_boosted_gain`.** A board's devicetree `rx-boosted` property only decides what
the driver starts with *before config loads*. `seed_config_defaults()` seeds `true` into
the config store, and `meshtastic_config_store_apply()` pushes the stored value to the
driver at every boot — so on any provisioned node the board file is **not** the effective
setting. Worse, `sx126x_set_rx_boosted_gain()` only *stages* the value; it reaches the chip
on the next `lora_config()`. A config that failed leaves the radio on the old gain while
every stored value reads new — a silent 2–3 dB sensitivity loss.

That is why the row reports **staged** and **applied** separately, and why an
unreportable value prints `unknown` rather than `OFF`. Turning "we cannot see" into "it is
disabled" is exactly the confusion the command exists to remove.

**`tx_power`.** The stored value is power at the *antenna*. On a board with a PA the drive
level programmed into the transceiver is a different number, and it can additionally be
clamped to the radio's range with no log at all. Both reasons are reported separately,
because conflating them is how an operator ends up believing a bare SX1262 radiates
30 dBm:

```
  [ok] tx power         30 dBm requested at the antenna
                        region 1 limit 30 dBm, unlicensed
  [!!] drive level      22 dBm
       -> no front-end, so this IS the radiated power: 22 dBm, not the 30 dBm
          requested (radio range [-9..22])
```

That output is from a board with no front-end, and it is the correct, previously invisible
answer: the SX1262 tops out at 22 dBm, so an 8 dB shortfall against the region limit is
physics, not a bug — but nothing told anyone. Upstream behaves identically
(`RadioInterface::limitPower()` clamps to `SX126X_MAX_POWER` = 22 and logs `Final Tx power`);
what is new here is surfacing it on demand rather than in one boot log line.

**The front-end backoff is licence-gated, the clamp is not.** They exist for different
reasons and it matters:

- The FEM conversion keeps *radiated* power inside a **regulatory** limit. A licensed
  operator is not bound by that limit, so the reference skips the conversion entirely for
  them — `if (!devicestate.owner.is_licensed) power = powerConversion(power);` — and the
  extra front-end gain on top is precisely what the licence buys.
- The clamp is the transceiver's **electrical** range and applies to everyone.

This port applied the conversion unconditionally until 2026-08-23, which left a licensed
operator on a KCT8103L board radiating roughly **6 dB below vanilla** — quietly penalising
the one class of operator entitled to more, with nothing logged and nothing errored. Fixed,
and pinned by `test_licensed_operator_keeps_the_full_drive_level`. The licence flag is
cached in `mt.licensed` rather than read per frame, because the config store's accessor takes
a mutex the transmit path must not acquire while holding the radio semaphore.

**Row markers.** `[ok]` in effect · `[!!]` configured but not in effect · `[--]` not on
this hardware · `[??]` unknown. A row is never omitted when it does not apply, because an
omitted row cannot be told apart from a forgotten one.

---

## 2. Bin widths, and why these

**RSSI, 5 dB.** The SX126x quantises to 0.5 dB but is only accurate to about ±2 dB, so
anything finer bins measurement noise. A change worth acting on — an LNA, a real antenna
swap — is 3 to 6 dB, which lands in the adjacent bin: visible without being smeared.

**SNR, 2 dB.** Unlike RSSI, SNR's interesting range sits near the demodulation floor rather
than at the top of the scale, and it is what the rebroadcast contention window already
keys off. 2 dB resolves a 3 dB effect into distinct bins.

**The RSSI rail gets its own bin.** `RssiPkt` is an unsigned −dBm/2 byte, so the scale runs
0 to −127.5 dBm and **zero is the top of the scale, not a missing reading**. A bench node a
metre from its peer pins there. A pinned receiver *cannot demonstrate an improvement from
any gain change*, so the condition that invalidates a whole comparison has to be visible
rather than folded into the top ordinary bin. `rf hist` prints the rail percentage and
warns above ~10%.

Both scales carry an explicit out-of-range bin at each end rather than clamping into the
neighbour: "off the scale" and "at the edge of the scale" mean different things.

---

## 3. Direct vs relayed — the per-peer split

`hdr->src` is only the node whose transmitter we heard when the frame arrived in **one
hop** (`hop_start == hop_limit`). A relayed frame measures the *relay's* link to us; its
originator may be kilometres away.

So per peer, direct and relayed frames are counted separately and **only direct frames
contribute to the averages**. Folding both together produces a per-peer number that moves
when an unrelated third node moves, which is worse than useless in a comparison.

---

## 4. Where the RX hook sits, and why

In `mt_rx_cb()`, **after** the scanner's off-preset gate and **before** the software queue.
Two pressures, pulling opposite ways, both load-bearing:

- **After the scanner gate** — a sweeping node is parked on other people's frequencies and
  spreading factors. Those frames say nothing about the chain this node operates on, and
  letting them into the same distribution would quietly poison every comparison.
- **Before the queue** — a frame the queue drops was still demodulated. A full queue is a
  limit of our software, not of the RF chain, and omitting those frames biases the sample
  downward during exactly the bursts an experiment wants to capture.

**Not in the router**, deliberately: it discards ignored, duplicate and undecodable frames
before RSSI is ever read, and weak undecodable frames are precisely the population an
antenna or LNA change moves.

**Locking.** A spinlock, not a mutex, because the writer runs on the driver workqueue.
The discipline that makes it safe: *snapshot under the lock, format outside it*. Holding a
lock across forty lines of shell output at 115200 baud would stall the radio path for tens
of milliseconds.

---

## 5. Cost

Measured on `heltec_wifi_lora32_v4` with `CONFIG_MESHTASTIC_RF_PEERS=24`:

| | |
|---|---|
| `.bss` | **1016 B** (histogram 232 B + 24 peers + bookkeeping) |
| Flash | ~1.3 KB module + shell formatting |
| RX path | one spinlock, two branch-free bin indices, a few adds, an MRU-first scan of ≤24 keys |

That is cheaper than the 256-byte `memcpy` already on the same path, and it fits on a
256 KB nRF52840 alongside everything else. Nothing is placed in PSRAM: the counters are
touched on every frame, PSRAM is slower, and a flash-write cache window would make it
momentarily unreadable.

---

## 6. Traps

Four ways to get a confident wrong answer. The first is the one that catches people.

1. **Survivorship bias in mean SNR.** A *better* receive chain can *lower* mean SNR while
   raising the frame rate, because the frames it newly demodulates are all weak ones.
   Reporting mean SNR alone would score a genuinely better antenna as a regression. Always
   read the rate and the distribution together; when they disagree, the rate is the harder
   evidence.
2. **Receiver saturation.** Bench nodes metres apart pin the RSSI rail, and a saturated
   receiver cannot show an improvement from anything. Check the rail percentage before
   trusting any comparison.
3. **Preset mixing.** Spreading factor moves sensitivity by far more than any gain setting
   here, so a window spanning a preset change measures the preset. `rf hist` detects this
   and says so.
4. **Sample non-independence.** LoRa fading is correlated over seconds, so consecutive
   frames from one peer are not independent draws. `rf peers` shows the per-peer breakdown
   so a single chatty node dominating the sample is visible.

Peer-table eviction is **counted** and reported, for the same reason: a fixed table that
silently drops peers is a lie about coverage.

---

## 7. Commands

```
meshtastic rf              the gain path, in signal order, effective values
meshtastic rf hist         RSSI/SNR distribution for the current window
meshtastic rf peers        per-peer direct/relayed counts, rates and link quality
meshtastic rf reset        start a new window (the lifetime frame count survives)
```

`rf reset` deliberately keeps the lifetime count: a reset must not destroy the record of
how much has ever been observed.

### 7.1 The HEALTH block

The tail of `meshtastic rf` is the radio's *mechanisms*, not its gain. Each row is a
workaround or an interlock with its own counters, and each `[!!]` names the one number on
that row that should never move.

| row | counters | what a non-zero means |
|---|---|---|
| `CAD` | `clear` / `busy` / `timeout` / `error` | listen-before-talk before every transmit (2 symbols, as stock). `busy` climbs with traffic and is healthy; `timeout` or `error` is the chip not answering. |
| `AGC reset` | `ok` / `fail` / `skipped` / `deferred` / `patch-fail` | the 60 s warm-sleep cycle that keeps the SX126x AGC from latching. `skipped` = a transmit was in flight; `deferred` = a packet was being demodulated, so the reset stood back 1 s rather than lose it. `fail`/`patch-fail` are the bad ones. |
| `RX activity` | `now: preamble/header`, `busy-rx`, `false-preamble`, `false-header`, and the two `windows` | the chip's latched preamble-detected / header-valid flags this instant, and the verdicts of the two timing rules that turn them into "a packet is arriving": a preamble with no header after twice the preamble time, or a header with no packet after a maximum-length frame's airtime, is retired as noise. `busy-rx` counts the times the answer was yes. `unknown` on both flags means the driver cannot report them (an unpatched tree). |
| `TX defer` | `busy-rx` / `cad-busy` / `requeued` / `dropped` | transmits the radio refused *for now* — a packet arriving, or CAD hearing one — and the outbound queue re-rolled behind a fresh contention delay with the radio left listening. `dropped` is a frame that used up every defer on a channel that never went quiet (`CONFIG_MESHTASTIC_TX_DEFER_MAX`), and is the only bad number here. |
| `SPI BUSY streak` | consecutive BUSY-line timeouts | wiring / driver health; anything non-zero is a fault in progress. |

`meshtastic sched stats reset` zeroes every counter in this block together.

---

## 8. Status

Built and green on native_sim. Exercised fleet-wide on hardware since 2026-09-02 (the
front-end, LNA and staged-vs-applied rows read real values on all eight bench nodes); the
HEALTH block's `RX activity` and `TX defer` rows (2026-09-02, later) were first read on
rzr1/rzr2 the evening they shipped.

Still to build: the A/B alternation engine (`rf ab`), which needs carried patch 0011 for
real rx-boost readback before its verdicts mean anything.
