# Unified image — runtime BLE ⇄ WiFi transport toggle

One firmware image that carries **both** the BLE and the WiFi/net phone
transports and chooses **one at boot** from persisted config — mirroring
upstream Meshtastic, where the phone app's *WiFi enabled* switch drives
`NetworkConfig.wifi_enabled`. Built as `PROFILE=v4-unified` (Heltec V4).

This supersedes the "deferred R&D, two hard gates" framing in the older
`docs/BLE-MODEM-SLEEP-SCOPING.md` handoff notes: measured, only one gate was
real and it was small.

## Feasibility — both handoff "gates", measured

| Gate (as scoped) | Verdict | Evidence |
|---|---|---|
| **(a)** mbedTLS ECC-curve / crypto conflict with BT + WiFi both on | **Not a blocker** | The union configures, compiles, and links. BT crypto (`libsubsys__bluetooth__crypto.a`, controller blobs `-lbtdm_app -lbtbb`) and the WiFi PHY/MAC blobs (`-lnet80211 -lpp -lphy`) and PSA/mbedTLS (`libtfpsacrypto.a`, `libmbedtls.a`) all link into one ELF. Only a benign `PSA_WANT_ECC_SECP_R1_256` warning + an auto heap-bump (49152 → 55376). |
| **(b)** ~20–30 KB internal-DRAM shortfall | **Real, but ~10.7 KB** | The full union overran `dram0_0_seg` by **10,704 B**. Reclaimed to fit by dropping the unused TLS *data path* + right-sizing net buffers + dropping GNSS. |

Final fit: `dram0_0_seg` **345,112 / 380,672 B = 90.66 %** (~35 KB headroom).
Progression: first fit 99.23 % (2.9 KB) → drop `net` diagnostic shell + stats,
+3.4 KB → **dynamic StreamAPI (TCP) stack, +16 KB** → **`ob_items` + PhoneAPI queues
to PSRAM, +12.4 KB**. The 16 KB TCP serve stack moved from a static reservation to a
heap allocation made only in WiFi mode; its size was not cut (overflow-hardened). The
PSRAM moves relocate CPU-only *data* (see `memory-savings.md`).

> A symmetric dynamic **BLE** work stack (~6 KB) was tried and **REVERTED**: the ESP32
> BT controller draws from the same system heap (`CONFIG_ESP_BT_HEAP_SYSTEM`), so
> heap-allocating the BLE stack starved it into an OOM abort → BLE-mode boot hung
> (hardware-verified 2026-07-28: dynamic = hang, static = clean boot). The BLE stack
> stays **static** (+6 KB), which is why the final fit is 90.66 %, not 89.04 %.

## How the toggle works

Only **one** transport is brought up per boot, so only that stack's radio and
runtime heap go live — the two never contend for the ESP32 radio, and no BT/WiFi
coexistence layer is needed. Switching is a config write + reboot.

Boot-time selection lives in **`meshtastic_transport_prefer_wifi()`**
(`src/meshtastic.c`):

- Unified image (both `CONFIG_MESHTASTIC_BLE` and `CONFIG_MESHTASTIC_TCP`
  compiled): returns persisted `network.wifi_enabled` (default **false = BLE**
  when no NetworkConfig is stored).
- Single-transport image: the compiled-in transport always wins (the flag is
  ignored), so a BLE-only or WiFi-only build's behaviour is unchanged.

Wired at three sites:

1. **`meshtastic_init()`** (`src/meshtastic.c`) — reads the helper once, then
   gates the inits: `meshtastic_ble_init()` runs only when **not** WiFi;
   `meshtastic_tcp_init()` / `meshtastic_mqtt_init()` only when WiFi. In WiFi
   mode `bt_enable()` is never called; in BLE mode no TCP listener is opened.
2. **`meshtastic_wifi_auto.c`** — the auto-connect thread self-exits when the
   selection is BLE, so `esp_wifi` never starts and its ~25 KB runtime heap
   stays unclaimed (and a BLE node can light-sleep).
3. **`meshtastic_admin.c`** — *unchanged*: it already sets `reboot_pending` for
   any non-device/non-lora config section, so a phone writing `network` config
   already triggers the deferred cold reboot that applies the new transport.

Net services that only key off an IPv4 lease (SNTP, the power governor's `WIFI`
inhibitor) stay dormant in BLE mode with no extra gating — the lease event never
fires.

## Using it

```
PROFILE=v4-unified just dist          # -> firmware-out/heltec-v4/unified-ota-ui-nomqtt-pm/
# flash (guard first — two identical V4s):
devreg guard rzr2 /dev/ttyACM0 && \
  firmware-out/heltec-v4/unified-ota-ui-nomqtt-pm/flash.sh /dev/ttyACM0   # then tap RST
```

Switch transport from the **serial/telnet console** (no phone needed):

```
meshtastic transport            # show current + active-this-boot
meshtastic transport wifi       # persist wifi_enabled=true,  then: kernel reboot cold
meshtastic transport ble        # persist wifi_enabled=false, then: kernel reboot cold
```

…or from the **phone app**: toggle *WiFi enabled* → the admin path persists it
and schedules the reboot automatically.

**Local infra + diagnostic overlays.** The build layers two optional overlays on
top of `overlay-v4-unified.conf`:

- `overlay-net-local.conf` (**gitignored**) — the real broker/collector/NTP IPs,
  layered last for local builds (`LOCAL=1`, default). A public build (`LOCAL=0`)
  uses the sanitized `192.0.2.x` placeholders baked into the profile.
- Diagnostics via `EXTRAS` — layer `overlay-threadanalyzer.conf` (per-thread stack
  high-water logging), `overlay-pm-quiet.conf` (quiet PM-residency rig), or
  `overlay-uitest.conf` (DRAM-slimming for display-without-PSRAM):
  ```
  EXTRAS="overlay-threadanalyzer.conf" just dist
  ```

## What was trimmed to fit (and why it's safe)

The net side inherited MQTT-over-TLS buffer sizing it does not need in this
image — MQTT is off, OTA is mcumgr/UDP, and the phone link is telnet or BLE, so
there is **no TLS data path**. Baked into `overlay-v4-unified.conf`:

- `MQTT_LIB=n`, `MQTT_LIB_TLS=n`, `NET_SOCKETS_SOCKOPT_TLS=n`, `TLS_CREDENTIALS=n`
  — drop the TLS socket layer. **mbedTLS/PSA core stays on** for Meshtastic PKI
  (DM crypto) and BT_SMP.
- Net buffers to the proven `overlay-uitest.conf` sizing
  (`NET_PKT_RX/TX=8/10`, `NET_BUF_RX/TX=12/12`) + a small `NET_MAX_CONN=12` /
  `NET_MAX_CONTEXTS=10` trim.
- `GNSS=n` / `MESHTASTIC_GNSS=n` — orthogonal to the transport toggle; the bench
  nodes have no GPS wired. A GNSS-bearing unified image would need the ~1.7 KB
  back via PSRAM relocation.
- **Diagnostic trim (+3.4 KB, no function lost):** `NET_SHELL=n` +
  `NET_STATISTICS=n` (the `net` shell commands + per-protocol counters).
- **NOT trimmed — the 16 KB StreamAPI thread stack
  (`MESHTASTIC_TCP_THREAD_STACK_SIZE=16384`).** It is the single biggest
  internal-DRAM item and it is tempting, but 16384 is the result of *two*
  measured stack-hardening rounds (commits `0dd0004`, `a5b7a26`): the WiFi
  PhoneAPI serve thread — which decodes each framed `ToRadio` protobuf and
  dispatches it into the mesh — ran **86 % used at 4096** and still **≥70 % at
  8192** on live V4 boards, so it was doubled to 16384 to reach ≤34 %. That
  decode+dispatch chain is unchanged here, so a static cut re-introduces the
  overflow risk. The correct way to reclaim its 16 KB is a **dynamic
  (heap-allocated) stack created in `meshtastic_tcp_init()`**, which only runs
  when WiFi is the active transport — see the memory-ceiling section.

## Why both stacks are still allocated (memory efficiency ceiling)

Only one transport runs per boot, so in principle the other's memory is wasted —
in BLE mode ~15 KB of net stacks sit idle, in WiFi mode ~20 KB of BT stacks. It
is **not cleanly reclaimable in one image**: the stacks/pools are allocated at
*link time* (`K_THREAD_STACK_DEFINE` → `.bss`/`.noinit`) inside the Zephyr BT
host, net/TCP, mcumgr, and the ESP HCI/WiFi drivers, and Kconfig cannot make an
allocation depend on a runtime value. Options considered:

- **Dynamic stacks from the shared heap** (`k_thread_stack_alloc` inside the
  transport's init) — clean for threads *we* own, but only safe where the heap isn't
  already spoken for. **DONE for the TCP serve stack**
  (`CONFIG_MESHTASTIC_TCP_DYNAMIC_STACK`, 16 KB, WiFi-mode, unified-only): heap-allocated
  at WiFi bring-up, so it leaves the static image and BLE mode never allocates it. The
  **BLE** work stack was tried this way too and **REVERTED** — the ESP32 BT controller
  draws from the same system heap (`CONFIG_ESP_BT_HEAP_SYSTEM`), so the k_malloc starved
  it into an OOM abort → BLE-mode boot hung (hardware-verified 2026-07-28). The BLE stack
  stays static (+6 KB). Framework stacks (BT host, net-TCP, mcumgr) would need Zephyr
  subsystem patches — not pursued.
- **PSRAM for the stack — ruled out.** PSRAM is ~94 % free, but per
  `meshtastic_ext_ram.h` it is only safe for CPU-only *data* that no flash-write
  path touches; a live thread's stack is read/written constantly (incl. ISR
  context-saves) and the flash-write cache-disable window makes PSRAM momentarily
  unreadable → fault. So the dynamic stack stays in internal RAM. PSRAM remains
  the home for large *data* (the NodeDB already lives there).
- **Linker union of the two regions** — reclaims the full ~35–60 KB but Zephyr's
  macros don't support shared placement; hand-placing every stack is fragile and
  fights the ESP blobs. Not pursued.
- **`esp_bt_controller_mem_release()` / WiFi equivalent** — ESP32-native; frees
  *controller* RAM to the heap for the unused radio. Helps **runtime heap**, not
  the static link fit. Worth checking whether the HAL wires it if runtime heap
  becomes tight.

With ~22.3 KB headroom after the dynamic StreamAPI stack, the internal-DRAM
pressure is resolved for now. The remaining levers (framework-stack sharing,
more CPU-only data to PSRAM) are available if a future feature needs them.

## Caveats / follow-ups

- **DRAM headroom is ~35 KB** (90.66 % of `dram0_0_seg`; the BLE stack is static — see
  the reverted dynamic-BLE note above — so ~6 KB less than the dynamic-both projection).
  Roomy. Further levers and the full analysis live in
  [`memory-savings.md`](memory-savings.md) (note: WiFi DMA buffers can't move to PSRAM,
  and thread *stacks* can't go to PSRAM at all).
- **Dynamic-stack runtime check:** the TCP serve stack is `k_thread_stack_alloc`'d from
  the ~55 KB system heap at WiFi bring-up (16 KB). The BLE work stack is static (the
  dynamic variant was reverted — BT-controller heap conflict, above).
  Confirm the per-mode heap has room on the bench (`kernel heap`, `kernel
  stacks`). On alloc failure that transport's PhoneAPI is skipped (non-fatal — the
  node stays up on the mesh), by design.
- **WiFi credentials are provisioned via `wifi cred add` (NVS)**, not from the
  phone's `NetworkConfig.wifi_ssid/psk`. So enabling WiFi from the phone toggles
  the transport but does not itself carry credentials — bridging
  `NetworkConfig.wifi_ssid/psk` → `wifi_credentials` is a natural follow-up to
  make phone-only WiFi provisioning work.
- **On-hardware bench test is pending** — neither `rzr1` nor `rzr2` was attached
  when this landed. Test plan: flash `v4-unified` (boots BLE by default) → verify
  BLE + `meshtastic version` build id → `wifi cred add` → `meshtastic transport
  wifi` → `kernel reboot cold` → verify telnet/OTA on WiFi → `meshtastic
  transport ble` → reboot → back on BLE.
- **Latent bug found (not this feature):** `overlay-v4-unified.conf` sets GNSS
  across several flattened sections (on→off→on→off) rather than once. It happens to
  resolve to **off** at the last occurrence, but the flip-flopping is confusing and
  worth reconciling to a single decision in a separate change.
