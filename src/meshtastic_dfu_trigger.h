/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_DFU_TRIGGER_H_
#define MESHTASTIC_DFU_TRIGGER_H_

#include <stdbool.h>

#include <zephyr/toolchain.h>

/* Reboot into the Adafruit nRF52 bootloader. serial_only=false requests UF2
 * mode (mass-storage drive + serial DFU), true requests serial DFU only.
 * Does not return. */
FUNC_NORETURN void meshtastic_dfu_enter(bool serial_only);

/* Only stamp the DFU magic into the retained register — no logging, no
 * sleeping, no reboot — so the NEXT reset lands in the bootloader. Safe from
 * any context including CPU exceptions (the fatal handler uses it). */
void meshtastic_dfu_mark(bool serial_only);

#endif /* MESHTASTIC_DFU_TRIGGER_H_ */
