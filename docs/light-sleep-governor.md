# Light-sleep governor — `CONFIG_PM`

Turns Meshtastic's phone-settable `PowerConfig` into light-sleep behaviour on
ESP32-S3, **without** porting upstream's manual state machine. It consumes three
fields — `is_power_saving`, `min_wake_secs`, `wait_bluetooth_secs` — and expresses
"reasons to stay awake" as constraints on one Zephyr `pm_policy` lock; the idle
thread does the actual sleeping. Compiled only with `CONFIG_PM` (there is no light
sleep to gate otherwise); every hook is a no-op inline without it. Lives in
[`src/meshtastic_power.c`](../src/meshtastic_power.c).

## Why not a ported `PowerFSM`

Upstream (`firmware/src/PowerFSM.cpp`) runs a 100 ms-ticked state machine that
calls `esp_light_sleep_start()` itself. Zephyr inverts this: the idle thread
sleeps automatically whenever every thread is blocked, and the policy picks the
deepest state that fits the next timeout. So instead of transliterating the FSM we
turn its *states* into *policy constraints* and let the idle path pick — no
periodic tick, no `esp_light_sleep_start()` call of our own. (This is the Tier-3
direction predicted in the `LOW-POWER.md` reference: "states become policy
constraints, and the idle path picks.")

## How it works — the inhibitor set

One `pm_policy` STANDBY lock is held while an inhibitor bitmask is nonzero; the
SoC light-sleeps only when every bit clears.

| Inhibitor | Set when | Cleared when |
|---|---|---|
| `POLICY` | `is_power_saving == false` | `is_power_saving == true` |
| `PHONE` | a BLE **or** TCP PhoneAPI client connects (ref-counted) | last client disconnects |
| `ACTIVITY` | RX-for-us or button press | `min_wake_secs` after the last activity |
| `BOOT_WAIT` | at boot (armed once) | `wait_bluetooth_secs` after boot |
| `WIFI` | the interface gains an IPv4 lease (`NET_EVENT_IPV4_ADDR_ADD`) | the lease is lost (`NET_EVENT_IPV4_ADDR_DEL`) |

`WIFI` is the light-sleep-vs-WiFi gate (see parity point 4): the Zephyr esp32 WiFi
path has **no** DTIM/beacon-wakeup coordination, so a CPU-domain-down light sleep
drops an associated station (the AP deauths on the missed keepalives). Holding the
inhibitor while an IPv4 lease exists reproduces upstream's `!isWifiAvailable()` gate
— a net node stays up while it has connectivity, and only light-sleeps once the link
is genuinely down. It is driven from the IPv4 add/del `net_mgmt` events (the same
signal `meshtastic_mqtt.c` / `meshtastic_sntp.c` already watch), is **not**
ref-counted (a single interface has one link state; `inhibit_update_locked()` is
idempotent, so duplicate add/del events are safe no-ops), and compiles out entirely
without `CONFIG_NETWORKING`.

`POLICY` is the only always-considered inhibitor and reproduces upstream's default
(power saving OFF → stay responsive, the right choice for a mains/WiFi node). When
no `PowerConfig` is stored it comes from `CONFIG_MESHTASTIC_POWER_SAVE_DEFAULT`
([`Kconfig.power`](../src/Kconfig.power)); the per-board profile overlays
(`overlay-v4-unified.conf`, `overlay-v4-{net,ble}.conf`, `overlay-v3-net.conf`) set
it `y` so a PM build sleeps out of the box, and the phone's stored value overrides
on the next boot. Screen-timeout stays display-owned; BT is not dynamically toggled, so
no ON/DARK/NB/LS state graph is needed.

`meshtastic_power_config_apply()` re-reads the stored `PowerConfig` and drives
`POLICY` + `min_wake_secs` + boot-wait. It runs at boot (the
`MESHTASTIC_SETTINGS_APPLY_DEFINE` hook) and live on every admin `PowerConfig`
write (`meshtastic_admin.c`), so a phone toggle of `is_power_saving` takes effect
immediately, no reboot.

## Concurrency model (the load-bearing decision)

One file-static `k_spinlock gov_lock`. The bitmask read-modify-write **and** the
paired `pm_policy_state_lock_get/put` happen inside one locked region, so the lock
toggles exactly on the mask 0↔nonzero edge — never a lost or double `put` (which
would trip the underflow `__ASSERT` in `zephyr/subsys/pm/policy/policy_state_lock.c`).

- A **spinlock**, not a mutex, so the activity notes stay ISR-safe. The pm_policy
  calls take their own leaf spinlock and never call back into us, so the nesting
  is a strict one-way order.
- Notes are called **directly** from the RX / BLE / TCP / input contexts (Zephyr
  input defaults to `INPUT_MODE_THREAD`, so the button path is not ISR context,
  but the code is ISR-safe regardless).
- Two one-shot `K_WORK_DELAYABLE`s (`activity_work`, `boot_wait_work`), coalesced
  via `k_work_reschedule` — **no unconditional periodic wakeup**. `activity_expiry`
  does a **deadline recheck**: if a stale timer fires in the gap between a note's
  unlock and its reschedule, it re-arms for the remaining time instead of clearing
  a still-live window.
- The locked region is kept free of immediate-mode logging (the one `LOG_INF` in
  `config_apply` runs after unlock).

## Signals and hook sites

Five sites, one call each, all compiling away when `!CONFIG_PM` via the no-op
`static inline` stubs in [`src/meshtastic_core.h`](../src/meshtastic_core.h) (which
also covers `CONFIG_MESHTASTIC_BLE`/`_TCP`/`NETWORKING` being off):

| Signal | Site | Note |
|---|---|---|
| BLE connect / disconnect | `meshtastic_ble.c` `connected()` / `disconnected()` | around `set_ble_connected()` |
| TCP connect / disconnect | `meshtastic_tcp.c` `tcp_accept_client()` / `tcp_close_client()` | **new** hooks — TCP emitted nothing before |
| RX-for-us | `meshtastic_router.c` `deliver_packet()` | the single delivery choke point |
| Button press | `meshtastic_display.c` `ui_input_cb()` | press branch only |
| WiFi link up / down | `meshtastic_power.c` `power_net_event()` (IPv4 add/del) | self-contained `net_mgmt` callback (`CONFIG_NETWORKING` only) |

`PHONE` is ref-counted across BLE + TCP, toggling the inhibitor only on the
0↔nonzero edge; an unmatched disconnect is a guarded no-op. The TCP "newest wins"
preempt path closes-then-accepts, so the count stays balanced. A failed BLE
`connected(err)` early-returns before its note, and Zephyr only calls
`disconnected()` for a connection that connected — so those stay balanced too.

## Value-0 semantics

`min_wake_secs == 0` → `ACTIVITY` is never engaged (the activity notes no-op);
`wait_bluetooth_secs == 0` → `BOOT_WAIT` is never armed. This **deliberately
diverges** from upstream's `getConfiguredOrDefaultMs` (0 = a compiled default), to
honour the port's "no wake the config didn't ask for" discipline (the same reason
the airtime 1 Hz tick was removed). If a nonzero default is ever wanted, map
`0 → CONFIG_…` in `meshtastic_power_config_apply()` only.

## Parity with upstream `PowerFSM` (deliberate divergences)

The field→behaviour mapping is faithful, but four differences are intentional and
worth knowing before comparing against `firmware/src/PowerFSM.cpp`:

1. **`min_wake_secs`** maps to upstream's `stateNB → stateLS` "Min wake timeout"
   (`PowerFSM.cpp:443`) — the wake window after activity. The `ACTIVITY` inhibitor
   holding for `min_wake_secs` is the direct analogue. ✅ faithful.
2. **What counts as activity is broader.** Upstream resets the wake window on
   `EVENT_PACKET_FOR_PHONE` (packets queued for an *attached phone*) and only
   full-wakes on `EVENT_RECEIVED_MSG`, labelled "Received **text**"
   (`PowerFSM.cpp:390`). Our `note_activity()` fires on **every** delivered packet
   (all portnums, broadcasts included) — more conservative (stays awake more). A
   tunable comment in `deliver_packet()` marks how to narrow to
   `packet->to == mt.node_id`.
3. **`wait_bluetooth_secs` is boot-only here.** Upstream re-applies it on every
   `stateDARK → stateLS` "Bluetooth timeout" (`PowerFSM.cpp:449`), i.e. after each
   screen-dark, not just boot. We arm `BOOT_WAIT` once — the port has no clean
   "screen went dark" edge to re-arm on, and the boot pairing window is the
   primary intent.
4. **WiFi gate — now reproduced (`WIFI` inhibitor).** Upstream only installs the
   light-sleep transitions when `!isWifiAvailable() && !isTrackerOrSensor &&
   (isRouter || is_power_saving)` (`PowerFSM.cpp:442`). An earlier revision of this
   port relied *only* on `is_power_saving` and dropped the WiFi gate — which broke
   connectivity: this tree's esp32 WiFi path has no DTIM/beacon-wakeup coordination,
   so light-sleeping with a station associated made the AP deauth it. The `WIFI`
   inhibitor restores the gate: while the interface holds an IPv4 lease the governor
   blocks STANDBY, so a net node behaves like upstream (no light sleep while WiFi is
   up) and only sleeps once the link is down. Restoring true battery savings *with*
   WiFi up would need modem-sleep/DTIM coordination at the SoC/HAL layer (esp-idf's
   enhanced-light-sleep), which is not wired into this tree — a separate follow-on.
   Related: upstream treats a **serial** API connection as a stay-awake state
   (`EVENT_SERIAL_CONNECTED`); our `PHONE` inhibitor covers only BLE + TCP, not
   serial (a serial link implies USB power anyway).

## The radio controllers block STANDBY independently of the governor

The governor releasing its STANDBY lock is **necessary but not sufficient**: on a
radio-active node the SoC still will not light-sleep, because the ESP-IDF radio
controllers hold their own PM lock that the governor never sees.

Both the WiFi driver (`esp_wifi/src/wifi_init.c` — an `ESP_PM_APB_FREQ_MAX` lock)
and the BLE controller (`bt/controller/esp32c3/bt.c` — the S3 uses the C3-family
BLE core) call `esp_pm_lock_acquire()` while active. The Zephyr HAL bridges that
through a **coarse stub** (`modules/hal/espressif/.../pm_impl_zephyr.c`):
`esp_pm_impl_switch_mode(MODE_LOCK)` → `pm_policy_state_all_lock_get()`, which
locks **every** PM state, `STANDBY` included. (The stub's own comment flags it as a
placeholder "until we port IDF pm_impl.") This lock is a separate
`pm_policy_state_lock_get` from the governor's, so it does **not** appear in the
governor's inhibitor mask.

Consequences, both observed on the bench:

- **A BLE node never light-sleeps while advertising/connectable.** With no
  `CONFIG_BT_CTRL_MODEM_SLEEP`, the controller holds the lock continuously —
  `is_power_saving=on`, governor mask `0x00`, yet `light-sleep entries` stays `0`.
- **A WiFi node is doubly gated.** The governor's `WIFI` inhibitor and the WiFi
  APB lock both drop together when the link goes down, which is why a `netpause`
  makes an otherwise-pinned WiFi node light-sleep hundreds of times per second.

Restoring true battery savings on a radio-up node is a HAL/controller bring-up
job, not a governor change — enable the controller's modem-sleep (`BT_CTRL_MODEM_SLEEP`
+ a low-power clock; note the Heltec V4 has no 32.768 kHz crystal) **and** refine
the coarse `pm_impl_zephyr` stub so a modem-sleep-capable controller does not take
an all-states lock. A scoped follow-on. The `meshtastic power` shell command
reports both the governor mask **and** the true `pm_policy_state_lock_is_active(STANDBY)`
so this distinction is visible at the console (`STANDBY lock: held by esp-idf radio
controller … NOT the governor`).

## Scope — light sleep only

Deep sleep is explicitly **out**. `sds_secs` / `ls_secs` escalation and
deep-sleep duty-cycling need a separate force-sleep + LoRa-wake mechanism and hit
the S3's ~18 s tickless / deep-sleep limits. A follow-on, not this governor. The
`MESHTASTIC_POWER_GOVERNOR` escape-hatch Kconfig from the design was skipped —
`CONFIG_PM` is the only gate.

## Wake sources on the ESP32-S3 (what actually wakes the CPU)

The governor decides *when the SoC may sleep*; this is *what brings it back*. From
`PM_STATE_STANDBY` (`esp_light_sleep_start()`, CPU domain powered down) two things
wake the ESP32-S3:

1. **The RTC timer — always.** Zephyr arms `esp_sleep_enable_timer_wakeup()` for the
   next scheduled timeout before every light sleep (`soc/espressif/common/power.c`);
   if it can't, sleep is *skipped* (logged), so a true never-wake hang is not
   reachable. This bounds worst-case latency for anything time-driven.
2. **LoRa RX on DIO1 — armed via the board DT.** The SX1262's DIO1 goes HIGH on
   RX/TX-done. `GPIO_INT_WAKEUP` on the board's `dio1-gpios` cell (V4 dtsi; V3
   overlay override) makes the sx126x driver's own `gpio_pin_configure_dt(dio1,
   GPIO_INPUT)` arm an **EXT1** level-HIGH wake on GPIO14. Without this the CPU
   would only ever wake on the timer, so a sleeping node adds up to a whole
   `MESHTASTIC_RX_IDLE_RECHECK_MS` (~30 s) of RX latency — enough to fall off the
   mesh. On WiFi builds the `WIFI` inhibitor means the node rarely sleeps at all;
   this matters for battery / non-WiFi nodes.

   **Read-on-wake caveat.** EXT1 *wakes* the CPU but the DIO1 edge was missed while
   the GPIO peripheral was powered down, so `dio1_isr()` does not fire — the frame
   would sit unread. A PM wake hook (`mt_radio_pm_exit`, a `pm_notifier .state_exit`
   in `meshtastic_radio.c`) polls DIO1 on every wake and, if still HIGH, submits the
   driver work via `sx126x_poll_dio1()` — a carried Zephyr patch
   (`zephyr/patches/0002-*`, **load-bearing**: PM builds won't link without it). The
   work handler then reads the frame over SPI as usual.

   **Button is not a wake source.** The S3's EXT1 has no per-pin trigger polarity, so
   it can't mix DIO1 (wake-on-HIGH) with the active-low USER button (wake-on-LOW);
   RX wins. Waking on the button would need a different arrangement.

   **Shutdown interaction.** EXT1 is shared with deep sleep, and `GPIO_INT_WAKEUP`
   arms it whenever `CONFIG_PM || CONFIG_POWEROFF`. So the admin shutdown path
   (`sys_poweroff()` → deep sleep) calls `meshtastic_radio_disarm_dio1_wake()` first,
   reconfiguring DIO1 to plain input to clear the EXT1 arming — a shut-down node
   still wakes only on reset, as `canShutdown` advertises.

### The console dies during light sleep (a "hang" that isn't)

On the **V4** the console/shell is the ESP32-S3 **USB-Serial-JTAG** (`usb_serial`),
not a UART. The S3 does not keep USB-Serial-JTAG alive across light sleep (the HAL
disables its PHY on every entry) and the driver holds no PM lock, so once the node
starts sleeping the CDC-ACM port drops and the console looks **hung** even though the
CPU is waking normally on the timer. This is expected, not a fault. There is **no
clean runtime "USB host connected" signal** on the S3 USJ to gate sleep on (only a
flaky SOF heartbeat), so it is left as a documented limitation rather than an
unreliable inhibitor. To observe residency, use the `overlay-pm-quiet.conf` rig + the
`meshtastic netpause <secs>` shell command (drive/read over serial while the network —
and the sleeps it would perturb — is genuinely quiet). The **V3** console is `uart0`
(real UART, via the CP2102), whose driver *does* hold a PM lock while TX is pending, so
it flushes before sleep and does not show this symptom.

## Tests

[`tests/power/`](../tests/power/) — a native_sim `ztest` suite (mirrors
`tests/airtime/`). The **critical** piece is `app.overlay`: it adds a real
`zephyr,power-state` `standby` node on `&cpu0`. Without it the entire
`pm_policy_state_lock_*` machinery compiles out
(`DT_HAS_COMPAT_STATUS_OKAY(zephyr_power_state)` gate) — get/put become no-ops and
`is_active()` always returns false, so the asserts would test nothing. `prj.conf`
adds `CONFIG_PM=y` + `CONFIG_ASSERT=y` (so an unbalanced put aborts, making the
ref-count/idempotency tests meaningful). Eight suites: phone ref-count (incl. a
guarded unmatched disconnect), policy toggle, activity expiry,
activity-disabled-when-zero, activity coalesce (exercises the deadline recheck),
boot-wait arm-once, WiFi inhibit (incl. idempotent duplicate up/down), and WiFi
composing with POLICY. No `meshtastic_init()` — the governor is driven directly;
`PowerConfig`s are seeded via the real `meshtastic_config_store_set_config()` path
(safe standalone).

## Building / running the tests

This repo is both the west manifest and the Zephyr module, so the module source
always resolves to `.west/config` `manifest.path` — a **shared** setting across
all worktrees. Point it at this worktree first (`./wt manifest <worktree>`), and
only build when no other session is mid-build (the manifest is global state). Then:

```
west twister -T <worktree>/tests/power -s meshtastic.power.native_sim -p native_sim/native/64
```

When you can't coordinate a shared-manifest window, build without touching it via
direct cmake with `-DZEPHYR_EXTRA_MODULES=<abs-path-to-worktree>` (see
`~/lab/meshprojects/POWER-MGMT-STATUS.md` §6). Bench validation (real light-sleep
residency vs the inhibitors) is per `~/lab/meshprojects/BENCH-RUNBOOK.md`.
