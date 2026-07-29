# Internal-RAM savings — levers for the unified image (and beyond)

Companion to [`unified-transport.md`](unified-transport.md). The unified BLE+WiFi
image is internal-DRAM-bound; this catalogues the ways to reclaim more, ranked,
with what's already done and what's left. Written 2026-07-26.

## The budget

The scarce region is **`dram0_0_seg` = 380,672 B (372 KB)** of internal SRAM
(ESP32-S3). PSRAM (`ext_dram_seg`, 32 MB) is a separate space and ~94% free — but
it is **not** a drop-in substitute (see "PSRAM rules" below).

Unified image current fit: **338,960 / 380,672 B = 89.04 %** (~40.7 KB headroom),
after the diagnostic trim, the two dynamic transport stacks, and moving
`ob_items` + the PhoneAPI queues to PSRAM. Single-transport profiles are far
looser (v4-net 80%, v4-ble 68%).

### Static vs runtime — two different budgets

- **Static (link-time):** what the linker reserves in `dram0_0_seg` — the 92.39%
  figure. `.bss` / `.noinit` / `.data`. This is what makes an image *fit*.
- **Runtime (heap):** `kheap__system_heap` (`CONFIG_HEAP_MEM_POOL_SIZE`, ~55 KB)
  is itself a static reservation, but it is *consumed* at runtime by BT, WiFi,
  mbedTLS, and now our dynamic stacks. Freeing memory *to the heap* (e.g. lever
  #1) doesn't shrink the static image; it relieves runtime pressure and can
  enable a smaller static heap.

Some levers cut static, some cut runtime — noted per lever.

## Biggest internal-DRAM consumers (measured, `ram_report`)

| Symbol | Bytes | Section | Notes |
|---|---:|---|---|
| `kheap__system_heap` | 55,460 | noinit | shared runtime heap (both stacks draw from it) |
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
- **`ob_items` + PhoneAPI queues → PSRAM** (+12.4 KB static on unified; +8.4 KB on
  v4-net/v4-ble): see lever #2 below.
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
Moved via `MESHTASTIC_EXT_RAM_BSS_ATTR` (board-gated, no-op on V3):
- `ob_items` (outbound staged TX frames, ~4.5 KB) — safe: the worker copies the
  chosen item to a stack local (`cur`) before `lora_send`, so the array is never a
  DMA source; mutex-guarded, never flash-read.
- The per-transport PhoneAPI FromRadio queues (`tcp_queue`, `ble_queue`, ~4.1 KB
  each) — pulled OUT of the transport structs so the kernel objects (mutex,
  work-queue) stay in internal RAM; the frames are encoded/copied to a local wire
  buffer before `zsock_send` / `bt_gatt_notify`, so never a DMA source.

**Measured reclaim (internal DRAM):** v4-unified −12.4 KB (92.39 % → **89.04 %**,
~40.7 KB headroom); v4-net −8.4 KB (→ 80.09 %); v4-ble −8.4 KB (→ 68.44 %) — the
board-gated attribute benefits every V4 profile. v3-net unchanged (no-op verified —
the arrays stay internal, no `.ext_ram` link failure). Symbols confirmed at
`0x3c0…` (PSRAM) on V4, `0x3fc…` (internal) on V3.

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
