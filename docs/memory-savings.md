# Internal-RAM savings — levers for the unified image (and beyond)

Companion to [`unified-transport.md`](unified-transport.md). The unified BLE+WiFi
image is internal-DRAM-bound; this catalogues the ways to reclaim more, ranked,
with what's already done and what's left. Written 2026-07-26.

## The budget

The scarce region is **`dram0_0_seg` = 380,672 B (372 KB)** of internal SRAM
(ESP32-S3). PSRAM is a separate space and mostly free — but it is **not** a
drop-in substitute (see "PSRAM rules" below).

Unified image current fit: **338,960 / 380,672 B = 89.04 %** (~40.7 KB headroom),
after the diagnostic trim, the two dynamic transport stacks, and moving
`ob_items` + the PhoneAPI queues to PSRAM.

### Do not read `ext_dram_seg` from the build report

The memory report line for `ext_dram_seg` is misleading in **both** columns and
has no useful percentage. On the V4 it reads:

```
ext_dram_seg:  2023888 B    32 MB    6.03%
```

- **`32 MB` is not the PSRAM size.** It is the width of the ESP32-S3 external
  data-bus address window (`0x3C000000`, 32 MB), which the linker uses for
  `drom0_0_seg` (flash-mapped rodata) *and* `ext_dram_seg` (PSRAM) because they
  share that window. The part is 2 MB (V4) or 8 MB (R8), so the 6.03 % is
  meaningless.
- **`2023888 B` is not PSRAM consumption.** ~896 KB of it is `.ext_ram.dummy`, a
  `NOLOAD` hole the linker inserts to push PSRAM past the flash rodata already
  mapped into the same window. That is *address space*, not memory: PSRAM
  physical offset 0 is mapped at `_ext_ram_start`, after the dummy.

**The real number is `_ext_ram_end - _ext_ram_start`** — exactly what the
linker's own `"External SPIRAM overflowed"` ASSERT checks against
`CONFIG_ESP_SPIRAM_SIZE`. Read it from the map:

```console
$ grep -E "_ext_ram_(start|bss_end|heap_start|heap_end|end) =" build/zephyr/zephyr.map
```

Actual occupancy, unified profile:

| | **V4** (2 MB quad) | **V4-R8** (8 MB octal) |
|---|---:|---:|
| `.ext_ram.bss` (NodeDB, warm keys, config store, MQTT ctx, `ob_items`, PhoneAPI queues) | 57,808 B | 57,808 B |
| SMH heap (`ESP_SPIRAM_HEAP_SIZE`) | 1,048,576 B | 4,194,304 B |
| **Total of part** | **1.05 / 2 MB (53 %)** | **4.06 / 8 MB (51 %)** |

So the V4 has ~945 KB of its part genuinely idle, and the R8 ~3.9 MB. That is
real headroom — it just cannot absorb the things `dram0_0_seg` is short of.

### Static vs runtime — two different budgets

- **Static (link-time):** what the linker reserves in `dram0_0_seg` — the 92.39%
  figure. `.bss` / `.noinit` / `.data`. This is what makes an image *fit*.
- **Runtime (heap):** `kheap__system_heap` (`CONFIG_HEAP_MEM_POOL_SIZE`, 49,152 B
  as currently set in `overlay-v4-unified.conf`, ~49.2 KB measured with `k_heap`
  overhead) is itself a static reservation, but it is *consumed* at runtime by
  BT (BLE-mode only, `CONFIG_ESP_BT_HEAP_SYSTEM`), the WiFi/HAL blob's
  `heap_caps_malloc()` calls (routed straight to `k_malloc` on this port
  regardless of the `caps` bitfield requested — ~25 KB observed even with
  `CONFIG_ESP_WIFI_HEAP_SPIRAM=y`, see the CFB-heap note below), and our own
  dynamic TCP thread stack. **Correction, 2026-08-09:** mbedTLS does *not*
  compete for this heap in the current config — `CONFIG_MBEDTLS_ENABLE_HEAP` is
  unset, so `mbedtls_calloc`/`free` resolve to plain libc `calloc`/`free`,
  which draw from a *separate* ~90 KB Picolibc heap arena (`z_malloc_heap`,
  sized from the linker's `_end` to `_heap_sentry`), not `_system_heap`. Freeing
  memory *to the heap* (e.g. lever #1) doesn't shrink the static image; it
  relieves runtime pressure and can enable a smaller static heap.

Some levers cut static, some cut runtime — noted per lever.

## Biggest internal-DRAM consumers (measured, `ram_report`)

| Symbol | Bytes | Section | Notes |
|---|---:|---|---|
| `kheap__system_heap` | 49,244 (was 55,460 when `CONFIG_HEAP_MEM_POOL_SIZE` was larger; re-measured 2026-08-09 against the current 49,152 setting) | noinit | shared runtime heap — WiFi HAL (~25 KB) + our dynamic TCP thread stack (16 KB) account for essentially all of the observed ~83% steady-state fill; BT adds on top only in BLE mode |
| `tcp_thread_stack` | 16,384 | noinit | **now dynamic** (WiFi-mode only) |
| `mt_stack` | 8,192 | noinit | meshtastic main thread (both modes) |
| `_k_mem_slab_buf_tcp_conns_slab` | 7,440 | noinit | TCP conn contexts (`NET_MAX_CONN`) |
| `rx_thread_stack` | 6,144 | noinit | BT host RX (framework) |
| `ble_work_stack` | 6,144 | noinit | static (a dynamic variant was reverted — see below) |
| `net_buf_data_hci_rx_pool` | 5,632 | noinit | BT HCI RX buffers |
| `ble` | 5,488 | bss | BT host state |
| `smp_udp_configs` | 5,392 | bss | OTA (SMP/UDP) |
| `tcp` | 5,236 | data | TCP protocol state |
| `z_main_stack` / `shell_uart_stack` / `shell_telnet_stack` | 5,200 ea | noinit | |
| `ob_items` | 4,480 | bss | outbound scheduler array (PSRAM candidate) |
| `telemetry_stack` / `nodeinfo_stack` / `disp_stack` / `sys_work_q_stack` / `bt_stack` | 4,096 ea | noinit | |
| `g_cnxMgr` | 3,976 | bss | WiFi connection manager |
| `sntp_stack` / `net_buf_data_acl_tx_pool` | 2,560 ea | noinit | |

Takeaway: the pressure is **thread stacks + buffer pools + the shared heap**, not
IP protocol features (IPv6 is already off).

## Already done

- **Diagnostic trim** (+3.4 KB static): `NET_SHELL=n`, `NET_STATISTICS=n`.
- **Dynamic StreamAPI stack** (+16 KB static, `MESHTASTIC_TCP_DYNAMIC_STACK`): the
  16 KB serve-thread stack is heap-allocated at WiFi bring-up, so it leaves the
  static image and BLE mode never allocates it.
- **BLE work stack — kept STATIC (a dynamic variant was tried + REVERTED).** A
  `MESHTASTIC_BLE_DYNAMIC_STACK` option briefly heap-allocated this 6 KB stack too, but
  the ESP32 BT controller draws from the same system heap (`CONFIG_ESP_BT_HEAP_SYSTEM`),
  so the k_malloc starved it into an OOM abort → BLE-mode boot hung (verified on hardware
  2026-07-28: dynamic = hang, static = clean boot). The option was removed; the BLE stack
  stays static. Only the TCP stack is dynamic (WiFi mode has no BT controller competing
  for the heap). Net: this lever saves ~16 KB (TCP), not ~22 KB.
- **`ob_items` + PhoneAPI queues → PSRAM** (+12.4 KB static on unified): see
  lever #2 below.
- **NOT trimmed:** `MESHTASTIC_TCP_THREAD_STACK_SIZE` (16384) — two measured
  stack-hardening rounds (`0dd0004`, `a5b7a26`); do not cut. Same discipline for
  any hardened stack.

## PSRAM rules (why it's not a free 30 MB)

Per `meshtastic_ext_ram.h`, `MESHTASTIC_EXT_RAM_BSS_ATTR` places static BSS in
PSRAM — but **only for CPU-only data that no flash-write path reads** (the
flash-write cache-disable window makes PSRAM momentarily unreadable) and **never**
DMA / WiFi-driver / net-packet buffers (those need internal DMA-capable RAM;
routing the WiFi heap to PSRAM breaks `esp_wifi_init()`). Consequences:

- **Thread stacks can NEVER go to PSRAM** — read/written constantly incl. ISR
  context-saves, which can land in a cache-disabled window → fault.
- Large **CPU-only data** is fair game (NodeDB, config store, MQTT ctx already
  there).

## Spending the R8's 8 MB — what actually pays

Written 2026-08-03, when the 8 MB octal boards arrived. The framing that matters:
**`dram0_0_seg` is ~92 % full and PSRAM is ~50 % idle, but they are not
fungible.** PSRAM only relieves internal DRAM for CPU-only *static* data, and
that lever is close to exhausted — everything still large in `dram0_0_seg` is
thread stacks and DMA-capable buffer pools, neither of which may move (see the
rules above). So the honest split is: one lever that could *widen* what PSRAM is
allowed to hold, and a set that spends PSRAM on **new capability** rather than
on rescuing internal RAM.

### The one lever that would widen the rules: XiP from PSRAM

`CONFIG_SPIRAM_XIP_FROM_PSRAM` (= `SPIRAM_FETCH_INSTRUCTIONS` +
`SPIRAM_RODATA`) copies flash instructions and rodata into PSRAM at startup.
`CONFIG_SOC_SPIRAM_XIP_SUPPORTED=y` on the ESP32-S3, so it is available today.

The interesting part is not speed. Per the in-tree Kconfig help: *"code that
requires execution during an MSPI1 Flash operation can forgo being placed in
IRAM"* — i.e. with instructions and rodata resident in PSRAM, the flash-write
cache-disable window stops being a reason to keep things out of PSRAM. **That is
the constraint the whole `meshtastic_ext_ram.h` rulebook is built around.**
Lifting it would open up data that is currently ineligible, and would relax the
"never read by a flash-write path" review every new `MESHTASTIC_EXT_RAM_BSS_ATTR`
placement needs.

Costs and caveats, none of them verified here:
- Roughly `irom0_0_seg` + `drom0_0_seg` of PSRAM (~1.6 MB today). Affordable on
  the R8's 8 MB; **not** on the V4's 2 MB. This is R8-only.
- Boot time grows by the copy.
- **Unverified on hardware.** Treat the rule-relaxation as a hypothesis to test
  on the bench (a flash write concurrent with heavy PSRAM traffic), not as
  something to design against yet.

### Capability the 8 MB buys (PSRAM as a feature budget, not a rescue)

Ranked by value-per-effort. All are textbook PSRAM tenants: large, CPU-only,
never DMA'd, never read by a flash-write path.

1. **Packet/airtime telemetry ring.** A long rolling record of every TX/RX with
   timestamp, RSSI, SNR, port and result, dumpable over the shell. Cheapest
   thing on this list to build, and it is diagnostic leverage the bench does not
   have today — most of the bench findings in this repo were reconstructed from
   logs that had already scrolled away. At ~32 B/event, 1 MB holds ~32k events.
2. **Store-and-forward.** Upstream Meshtastic has a store-and-forward module;
   this port does not (`src/` has no equivalent). It is the canonical PSRAM
   tenant — a large ring of received packets replayed to nodes that were offline.
   8 MB holds tens of thousands of packets, which is well past the point where
   the mesh's own airtime budget becomes the binding constraint, not RAM.
3. **Local message archive for the PhoneAPI.** The phone re-requests history on
   reconnect and today there is nothing to serve it.
4. **Bigger `MESHTASTIC_DUP_CACHE_SIZE`** (32 today) — cheap flood-suppression
   improvement on a busy mesh.
5. **NodeDB / warm keys.** Already at 128 nodes + 512 warm keys, and the Kconfig
   `range` ceilings (256 / 512) bind before RAM does — 256 nodes is only ~106 KB.
   Raising the ceilings for the R8 is possible, but note the **real** limit moves
   to the NVS storage partition, not PSRAM, once `NODEDB_PERSIST_RECORDS` is on.
6. **LVGL, if the UI ever moves off CFB.** `default.ld` already reserves
   `.lvgl_buf*` / `.lvgl_heap*` output sections inside `.ext_ram.data`, so LVGL
   framebuffers would land in PSRAM by placement alone. (Same for `.mbedtls_heap*`
   — these are landing zones the linker provides; nothing emits into them today.)

### Attractive-looking, but no

- **Raising `ESP_SPIRAM_HEAP_SIZE` alone changes nothing.** It reserves address
  space for the shared-multi-heap; it does not create demand. It pays off only
  to the extent something actually allocates from SMH — today that is
  `ESP_WIFI_HEAP_SPIRAM` and any explicit
  `shared_multi_heap_alloc(SMH_REG_ATTR_EXTERNAL, ...)`. The R8's 4 MB is
  headroom for the features above, not a saving in itself.
- **`ESP32_WIFI_NET_ALLOC_SPIRAM` / static WiFi TX buffers** — broke
  `esp_wifi_init()` twice on hardware. See the long note in
  `samples/meshtastic/overlay-v4-unified.conf`; do not retry without bisecting
  the two symbols independently.
- **Thread stacks.** Never. Repeated for emphasis because it is the first idea
  everyone has when looking at a 92 %-full `dram0_0_seg` next to an idle 8 MB.

## The levers, ranked

### 1. Release the inactive radio's controller RAM — INVESTIGATED, dead end on Zephyr
`esp_bt_mem_release(ESP_BT_MODE_BLE)` / `esp_bt_controller_mem_release()` exist in
the S3 HAL (`esp32s31/esp_bt.h`) and are unwired (the Zephyr HCI driver
`hci_esp32.c` calls `esp_bt_controller_init/enable/disable`, never the release).
The header says the release frees *"about 70k bytes"* of controller BSS/data to the
heap — but on **ESP-IDF** that 70 KB is *statically reserved*, whereas on the
**Zephyr port it is heap-allocated at `esp_bt_controller_init()`** via the
controller's `osi_funcs` malloc hooks. Evidence: the `libbtdm_app` blob contributes
only **310 B** of static internal DRAM (measured from the map) — there is no 70 KB
static region to release.

**Consequence: no benefit for the toggle.** In WiFi mode we skip `bt_enable()` →
skip `esp_bt_controller_init()` → the ~70 KB is *never allocated*. `esp_bt_mem_release`
would reclaim ~0. The mode-aware runtime saving the release provides on ESP-IDF is
**already captured** by the toggle's heap-alloc-at-init model — which also means
the WiFi-mode heap has room for the dynamic StreamAPI stack (#dynamic-stack concern
largely mitigated). Symmetrically, BLE mode never allocates the WiFi runtime memory.
Not pursued.

### 2. More CPU-only data → PSRAM — DONE  *(static)*
Moved via `MESHTASTIC_EXT_RAM_BSS_ATTR` (board-gated; PSRAM boards only):
- `ob_items` (outbound staged TX frames, ~4.5 KB) — safe: the worker copies the
  chosen item to a stack local (`cur`) before `lora_send`, so the array is never a
  DMA source; mutex-guarded, never flash-read.
- The per-transport PhoneAPI FromRadio queues (`tcp_queue`, `ble_queue`, ~4.1 KB
  each) — pulled OUT of the transport structs so the kernel objects (mutex,
  work-queue) stay in internal RAM; the frames are encoded/copied to a local wire
  buffer before `zsock_send` / `bt_gatt_notify`, so never a DMA source.

**Measured reclaim (internal DRAM):** v4-unified −12.4 KB (92.39 % → **89.04 %**,
~40.7 KB headroom). Symbols confirmed at `0x3c0…` (PSRAM) on V4.

### 3. Shrink `CONFIG_HEAP_MEM_POOL_SIZE` after measuring per-mode peak  *(static)*
The 55 KB heap is the biggest single item, auto-sized to the worst-case requester.
If bench `kernel heap` high-water shows slack (esp. after #1 tops it up), trim it.
Pure measurement, high impact.

### 4. Collapse periodic emitter threads onto a shared workqueue  *(static)*
`telemetry_stack` + `nodeinfo_stack` are each a dedicated 4 KB thread running
`while(1){ emit; k_sleep(interval); }` — textbook `k_work_delayable` candidates.
Removes 8 KB of dedicated stacks, but the emit path's stack depth must fit the
shared workqueue (likely a modest `sys_work_q` bump) — net ~4–6 KB, high-water
validate. The TCP-stack lesson in miniature.

### Worth knowing, not worth doing (yet)
- **Framework stack right-sizing** (`BT_RX_STACK`, `NET_RX/TX`) — measurement-only,
  and these are the hardened kind. Don't guess.
- **Linker union of BT/net static regions** — reclaims ~35–60 KB in principle, but
  Zephyr's macros don't support shared placement and it fights the ESP blobs.
- **Shell backend rationalization** — UART + telnet backends both compiled
  (~10 KB), but each is needed in a different mode and both are framework-owned.

## Measurement notes

- Per-mode heap high-water: `kernel heap`, `kernel stacks` over the console (the
  same `STACK_USAGE` rig from `0dd0004`). Do this on the bench before trusting the
  dynamic-stack heap draw or shrinking the heap.
- None of this is required at 28 KB headroom — it's the menu for when a future
  feature runs the image tight again.
