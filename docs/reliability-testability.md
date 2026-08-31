# Reliability & testability roadmap

Brainstormed 2026-07-27. A prioritized backlog for making this Zephyr Meshtastic
port more robust in the field and easier to validate before it ships. Grounded in
a survey of the current code/config, so it records what already exists as well as
the gaps. Companion to [`memory-savings.md`](memory-savings.md) and
[`unified-transport.md`](unified-transport.md).

## Current posture (what's already in place)

| Area | State |
|---|---|
| **CI** | `.github/workflows/sim.yml` runs `west twister -T tests -p native_sim` on every push + PR; `vectors-drift.yml` weekly cron. |
| **Stack overflow** | `CONFIG_STACK_SENTINEL=y` + `CONFIG_STACK_USAGE=y` in deploy profiles — overflow becomes a clean detected fault, not silent corruption (commits `0dd0004`/`a5b7a26`; two measured hardening rounds). |
| **Reset cause** | Logged to NETLOG at boot + `hwinfo reset_cause` on the shell on demand. |
| **OTA** | `MCUBOOT_IMG_MANAGER=y` + `MESHTASTIC_OTA_AUTOCONFIRM` (confirm after N s healthy uptime); A/B slots, health-gated `just ota`/`just confirm`. |
| **Radio** | Deaf-radio re-arm recovery in the RX thread (failed post-TX re-arm is retried). |
| **RX pipeline** | Non-blocking: phone fan-out + MQTT are async drop-oldest; decode/dispatch CPU-bounded (see ARCH-REVIEW #6 reassessment). |
| **Wire-crypto vectors** | The `vectors` suite (T4/T6's target) now pins actual production code, not just the harvested fixture: `meshtastic_channel_nonce_build`/`meshtastic_pki_nonce_build` were pulled out as pure functions specifically so `tests/protocol`/`tests/admin_pki` can call them and assert the output against `tests/vectors/meshtastic_vectors.h` (harvested by compiling and running upstream's `CryptoEngine.cpp`, not reimplemented) — closes the class of bug a self-loopback encrypt/decrypt round-trip can't catch (both sides agree with each other even if the wire layout is wrong). 2026-08-05. |

Gaps the backlog below closes: **no watchdog · zero BLE coverage in sim · no
coverage measurement · no hardware-in-the-loop automation.**

---

## Reliability

### R1 — Hardware watchdog  *(HIGH — the standout gap)*
No `CONFIG_WATCHDOG` / `CONFIG_TASK_WDT` today. A deploy-and-forget mesh node with
no watchdog needs a physical power-cycle to recover from any wedge — a deadlocked
thread, a latched SX1262, a hung net stack. Add the ESP32-S3 WDT + task WDT and
reset on hang.
- **Design decision (do first):** what is the *healthy* signal? Feeding the WDT
  from a thread that is alive while the **radio** is wedged buys nothing. Options:
  tie the feed to actual progress (RX thread advancing, a periodic health-check
  thread that verifies radio liveness + heap sanity before feeding). Pick the
  liveness definition before wiring.
- **Effort:** M. **Risk:** low if the feed condition is chosen well; a
  too-aggressive WDT causes spurious reboots (e.g. during long flash writes / OTA —
  must be paused or its window widened there).

### R2 — Confirm-gate + rollback hardening for OTA  *(HIGH — the sleeper)*
`AUTOCONFIRM` gates on **uptime only**, so a "boots but broken" image (radio dead,
never joins the mesh) still confirms itself permanently after N s. For a remote
node that is the dangerous case.
- **Design:** gate confirm on **functional** health (successful mesh RX, or a
  WiFi/BLE link established) not just uptime; and verify mcuboot actually **reverts**
  on a pre-confirm crash-loop (swap mode was not explicit in `sysbuild.conf` — a
  bad OTA should self-heal, not brick).
- **Effort:** S–M. **Risk:** low. High payoff for remote-node safety.

### R3 — Radio self-recovery  *(HIGH)*
Extend the deaf-radio re-arm: if there has been **no successful TX and no RX** for
a long window, do a full SX1262 SPI re-init (not just a re-arm). Catches a
latched/glitched radio the re-arm retry can't.
- **Design:** pick the window + what counts as "no activity" (a node in a quiet
  area legitimately hears nothing — don't reset a healthy-but-idle radio; couple
  the check to a failed self-TX or a SPI-health probe).
- **Effort:** M. **Risk:** medium — a wrong trigger resets a healthy radio.

### R4 — Field reliability observability  *(MEDIUM — force-multiplier)*
Counters exist scattered (sched/airtime). Unify a compact health readout —
RX-queue drops, phone evictions, dedup hits, TX failures, heap high-water, reset
cause, uptime — surfaced via telemetry/MQTT/telnet. Turns "seems flaky" into data
and makes R1–R3's effects measurable.
- **Effort:** S–M. **Risk:** low. Do alongside R1–R3 so their impact is visible.

### R5 — Dead-man timers on peer-visible state  *(MEDIUM — one instance fixed, audit the rest)*
Any state one node raises on another party's *start* event and lowers on its *stop*
event latches on forever if that party vanishes. Across a link there is no such
thing as a guaranteed stop event.

Shipped instance (2026-08-28): the peer-link **HOLD** flag was raised on
`MGMT_EVT_OP_IMG_MGMT_DFU_STARTED` and lowered on `DFU_STOPPED`. `img_mgmt` raises
STOPPED only from inside a request it is processing — complete, aborted, erased —
so a courier that died mid-push (a BLE-controller fault took one down 41 s into an
upload) left the target beating HOLD with nothing able to lower it. Every later
courier then skipped that node as busy: permanently unservable until an operator
ran `blepeer hold off`. Fixed by `MESHTASTIC_OTA_HOLD_IDLE_TIMEOUT_S` (default
120 s), rearmed on each `DFU_CHUNK_WRITE_COMPLETE`, cancelled by STOPPED, with a
`hold_is_ours` latch so an operator's own hold is never lowered by the machinery.

- **Design rule:** bound such a flag on *observed progress*, not on the promise of a
  final event. The document plane already works this way — it is level-triggered, so
  nothing has to succeed — and this is the same discipline applied to the session
  plane (SMP jobs, peer links) where it does not come for free.
- **Do:** audit the remaining start/stop pairs the same way — the courier's own job
  state (`job_running`, the `updating` row), the `unsub_pending` latch in
  `meshtastic_smp_central.c`, and anything else cleared only on a success path.
- **Effort:** S per instance. **Risk:** low.

---

## Testability

### T1 — BLE coverage via BabbleSim  *(HIGH-value, biggest lift)*
native_sim builds **zero** BLE, so the reconnect fix and the unified image's
**default** (BLE) transport have no automated coverage at all. Zephyr's `bsim`
simulates the BLE controller + RF, enabling connect → disconnect → **reconnect**
and pairing tests in CI.
- **Trade-off:** heavier harness (separate build flow), BLE integration tests are
  non-trivial. But it is the *only* path to automated BLE coverage, and BLE is now
  load-bearing.
- **Effort:** L.
- **Evidence it is under-rated (2026-08-28):** three bugs shipped to the bench in one
  arc, all in *reconnect-to-a-bonded-peer*, none reachable from native_sim:
  1. Zephyr keeps a GATT subscription across a disconnect for a **bonded** peer
     (`remove_subscriptions()`), so `bt_gatt_subscribe()` answers `-EALREADY` for
     reused params — a client with one static `bt_gatt_subscribe_params` works
     exactly once per boot. Symptom appeared one release after the change (bonding)
     that caused it.
  2. `bt_conn_set_security()` answers `-EBUSY` while the peer is already encrypting;
     treating that as failure ran a second MTU+discovery over the same static params.
  3. A bring-up that died left a link held but never ready, and every retry answered
     `-EALREADY` until the job's budget expired.

  The minimal test that catches all three is small and entirely bsim-shaped:
  **connect → subscribe → disconnect → reconnect → subscribe again, with a bond, and
  again with the peer initiating.** Each bug cost a build/flash/bench cycle to find
  and needed hardware to confirm; each would have failed that test in seconds. This
  is the concrete case for moving T1 up the sequence.

### T2 — Coverage measurement  *(DONE 2026-07-27)*
`west twister --coverage --coverage-tool gcovr` on the native_sim suites.

**Port-wide line coverage: 58 %** (5906 lines, the 32 of 43 `main/src/*.c` files
native_sim actually builds).

**Gotcha (important):** twister's built-in coverage report is rooted at the Zephyr
tree and **excludes the meshtastic module** — the default report shows *zero*
port files. Measure the port with gcovr rooted at the workspace:
```
gcovr --root "$PWD" --gcov-executable gcov-13 --filter 'main/src/.*\.c$' twister-cov
# HTML: add --html --html-details -o twister-cov/mesh-coverage.html
```
CI fix: pass `--coverage-basedir` (or a gcovr step like the above) so the module
is included; otherwise the coverage number is meaningless for the port.

**The gaps (where risk actually is):**
- **Entire transport/net/BLE/display layer is NOT built in native_sim → 0 possible
  coverage:** `meshtastic_ble.c`, `meshtastic_tcp.c`, `meshtastic_wifi_auto.c`,
  `meshtastic_mqtt.c`, `meshtastic_ota.c`, `meshtastic_sntp.c`,
  `meshtastic_display.c`, `meshtastic_serial.c` (11 of 43 files). **Every one of
  this session's uncommitted fixes lives here** (BLE reconnect, dynamic stacks,
  MQTT version, wifi_auto gating, unified toggle). → the case for **T1 (bsim)** and
  a net-logic test host.
- **Compiled but 0–2 % (sim-testable, just no test):** `meshtastic_phoneapi_config.c`
  (0 % — the `want_config_id` → config-stream handshake, core phone functionality),
  `meshtastic_metrics.c` (0 %), `meshtastic_telemetry_common.c` (0 %),
  `meshtastic_position.c` (2 %).
- **Low core:** `meshtastic_admin.c` 28 % (many handlers/config-setters untested,
  security-relevant), `meshtastic_shell.c` 18 %, `meshtastic_settings.c` 34 %,
  `meshtastic_phoneapi.c` 36 %.
- **Well-covered:** router 77 %, packet 76 %, routing 84 %, contention 94 %,
  admin_session 100 %, nodedb 69 %, pki 67 %.

**Follow-on work this exposes (all sim-provable, no hardware):** add tests for the
config-sync handshake (`phoneapi_config`, 0 %), the metrics/telemetry/position
emitters, and the untested `admin` handlers; and wire the gcovr-with-basedir step
into CI so coverage is tracked over time. Report artifact:
`twister-cov/mesh-coverage.html`.

**Follow-on DONE 2026-07-27 — config-sync handshake: `phoneapi_config.c` 0 % → 90 %.**
Two tests in the `protocol_stack` suite (`test_phone_config_handshake_full`,
`..._only_nodes`) pump `meshtastic_phoneapi_next_config_frame` to completion and
assert the FromRadio sequence invariants — full dump starts at `my_info` and
includes metadata/region_presets/channels; `ONLY_NODES` starts at own `node_info`
and skips metadata/config; both end with `config_complete_id` echoing the request
nonce. Full suite 175 → **177**. Remaining 10 % is edge branches (position, error
paths, `set_channel`). Next untested targets: metrics/telemetry/position (0–2 %),
`admin` handlers (28 %).

### T3 — Automated hardware-in-the-loop (HIL) bench  *(HIGH-value, bigger lift)*
The entire uncommitted batch is gated on **manual** rzr2 testing. A scripted
harness — flash both nodes, run scenarios (send/verify messages, toggle
transports, force a BLE reconnect, measure drops), assert pass/fail — reusing
`meshtest.py`/`tsh.py` + the runbox/ask mechanism. The missing "real" test tier;
directly de-risks everything waiting on the bench.
- **Effort:** L.

### T4 — Fuzz the wire parser  *(MEDIUM)*
`meshtastic_router_process_lora_rx` decodes untrusted on-air bytes (protobuf +
AES + PKI). A libFuzzer/property harness on native_sim (extending the `vectors`
suite) that feeds malformed frames and asserts no crash/OOB — reliability **and**
security hardening.
- **Effort:** M.

### T5 — Fault injection in sim  *(MEDIUM)*
Inject alloc failures, radio errors, dropped packets to exercise the error/recovery
paths (the non-fatal inits, deaf-radio recovery, reliable retry) that only trigger
on real faults today. A fault-injection hook makes them testable.
- **Effort:** M.

### T6 — Grow the on-air regression corpus  *(LOW, incremental)*
Capture real frames (`PACKET_HEXDUMP`) into the `vectors` suite as replay tests, so
real-world edge cases become permanent regression guards.
- **Effort:** S, ongoing.

---

## Suggested sequence

1. **T2 (coverage)** — an afternoon; tells us objectively where the risk is. *(in progress)*
2. **R1 (watchdog)** — highest-value reliability add; decide the liveness signal first.
3. **R2 (OTA confirm-gate)** — cheap, prevents a bad remote OTA from bricking a node.
4. **R3 (radio self-recovery)** + **R4 (observability)** — pair them so recovery is measurable.
5. **T1 (bsim BLE)** — biggest testability lift; justified because BLE is the default and uncovered.
6. **T4/T5/T6** — harden as capacity allows.

Operator priority (2026-07-27): **R1, R2, R3 high.** T2 started first as the
cheap objective-signal step.

Since then (2026-08-28): R1 is done (the watchdog landed, and the `wdt_esp32`
ms-vs-ticks bug behind it is fixed and carried as a patch); **R5** is new, with one
instance already fixed; and **T1 has earned a promotion** — the node-to-node OTA path
is now load-bearing, it is BLE end to end, and the three bugs listed under T1 are what
"no automated BLE coverage" costs in practice. Reconnect-with-a-bond is the single
highest-yield test to write first.
