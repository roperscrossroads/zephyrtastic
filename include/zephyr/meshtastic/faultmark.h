/* SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * faultmark.h — a fault record that survives a warm reset, at a FIXED address.
 *
 * Two different firmware images agree on this location, not on a linker
 * section: a test image (an MCUboot "test" boot that never confirms) writes it
 * from its fatal-error handler, resets, MCUboot reverts, and the confirmed
 * image reads it back and prints it. That is how a board with no debug probe
 * and no console yet (a USB CDC that never enumerated because the fault came
 * first) still tells you where it died.
 *
 * The slice is the top 256 bytes of the nRF52840's RAM. Nothing links there:
 * Zephyr places .data/.bss/.noinit from the bottom up, both images end far
 * below (~0x2002d000 for the Meshtastic build, ~0x2000e300 for a spike), and
 * MCUboot's own image ends at 0x20004400. RAM keeps its contents across a
 * SYSRESETREQ, which is also what the retained boot history relies on.
 */

#ifndef ZEPHYR_INCLUDE_MESHTASTIC_FAULTMARK_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_FAULTMARK_H_

#include <stdint.h>

#if defined(CONFIG_SOC_NRF52840)
#define MESHTASTIC_FAULTMARK_ADDR 0x2003FF00UL
#endif

#define MESHTASTIC_FAULTMARK_MAGIC 0xFA17DEA1UL
#define MESHTASTIC_FAULTMARK_STAGE_LEN 24

/* 96 bytes. Every field is written by the faulting image; the reader trusts
 * nothing but the magic (and its complement, against a lucky RAM pattern). */
struct meshtastic_faultmark {
	uint32_t magic;
	uint32_t magic_inv;	/* ~magic */
	uint32_t reason;	/* K_ERR_* from the kernel, or 0 while alive */
	uint32_t pc;
	uint32_t lr;
	uint32_t xpsr;
	uint32_t cfsr;		/* SCB->CFSR: MemManage/BusFault/UsageFault status */
	uint32_t hfsr;		/* SCB->HFSR */
	uint32_t bfar;		/* SCB->BFAR, valid when CFSR.BFARVALID */
	uint32_t mmfar;		/* SCB->MMFAR, valid when CFSR.MMARVALID */
	uint32_t uptime_ms;	/* k_uptime at the fault (or at the last stage mark) */
	uint32_t thread;	/* k_current_get() at the fault */
	char stage[MESHTASTIC_FAULTMARK_STAGE_LEN]; /* last progress mark, NUL-terminated */
	uint32_t seq;		/* stage marks written this boot */
	uint32_t reserved[4];
};

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_FAULTMARK_H_ */
