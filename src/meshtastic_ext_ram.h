/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_EXT_RAM_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_EXT_RAM_H_

#if defined(CONFIG_ESP_SPIRAM)
#include <esp_attr.h>
#endif

/* MESHTASTIC_EXT_RAM_BSS_ATTR — place a large, app-owned, CPU-only static (BSS) in the
 * ESP32-S3 external PSRAM instead of scarce internal DRAM, freeing dram0 on PSRAM boards
 * (V4). A no-op on boards without PSRAM (V3), so the same source is safe on both.
 *
 * Gate on CONFIG_ESP_SPIRAM, NOT the underlying EXT_RAM_BSS_ATTR's own guard: that macro
 * keys off CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY, which — a real Kconfig footgun found
 * 2026-07-17 — depends on the vendored ESP-IDF "SPIRAM" symbol, NOT Zephyr's board-level
 * CONFIG_ESP_SPIRAM. On V3 (no PSRAM, ESP_SPIRAM unset) that vendored symbol is still =y, so
 * EXT_RAM_BSS_ATTR is NOT the no-op it looks like — it emits an .ext_ram.bss section with
 * nowhere to go (V3's linker script has no ext_ram output region at all) and the build fails
 * to link (".ext_ram.bss.N will not fit in region IDT_LIST"). CONFIG_ESP_SPIRAM is the symbol
 * that actually reflects whether THIS board has a real PSRAM-backed linker region.
 *
 * ONLY for CPU-accessed data — never WiFi/driver allocators or DMA/net-packet buffers (those
 * need internal DMA-capable RAM; routing the WiFi heap to PSRAM breaks esp_wifi_init(), see
 * samples/meshtastic/overlay-psram.conf's notebook), and never data a flash write reads
 * directly (the flash-write cache-disable window makes PSRAM momentarily unreadable). The
 * NodeDB, the MQTT client context, and the config store all qualify — the config store
 * pb_encodes each record into a local internal-RAM buffer before any NVS write, so the flash
 * write never touches the relocated struct. */
#if defined(CONFIG_ESP_SPIRAM)
#define MESHTASTIC_EXT_RAM_BSS_ATTR EXT_RAM_BSS_ATTR
#else
#define MESHTASTIC_EXT_RAM_BSS_ATTR
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_EXT_RAM_H_ */
