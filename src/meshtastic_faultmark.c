/* SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * meshtastic_faultmark.c — read (and clear) the fixed-address fault record a
 * test image left behind before it reset and MCUboot reverted to us.
 * See include/zephyr/meshtastic/faultmark.h for the contract and the address.
 *
 * Shell:
 *   faultmark          print the record if one is present
 *   faultmark clear    zero it (a fresh test boot writes a new one)
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/faultmark.h>

static volatile struct meshtastic_faultmark *const fm =
	(volatile struct meshtastic_faultmark *)MESHTASTIC_FAULTMARK_ADDR;

static bool faultmark_present(void)
{
	return fm->magic == MESHTASTIC_FAULTMARK_MAGIC && fm->magic_inv == ~MESHTASTIC_FAULTMARK_MAGIC;
}

static const char *reason_str(uint32_t r)
{
	switch (r) {
	/* K_ERR_* is 0-based (fatal_types.h); a live mark writes reason 0 too, so
	 * "CPU exception" and "alive" share a value — the stage/IPSR tell them apart. */
	case 0:
		return "CPU exception (or alive at last mark)";
	case 1:
		return "spurious IRQ";
	case 2:
		return "stack check fail (MPU stack guard)";
	case 3:
		return "kernel oops";
	case 4:
		return "kernel panic";
	default:
		return "?";
	}
}

static int cmd_faultmark(const struct shell *sh, size_t argc, char **argv)
{
	char stage[MESHTASTIC_FAULTMARK_STAGE_LEN];

	if (argc > 1 && strcmp(argv[1], "clear") == 0) {
		for (size_t i = 0; i < sizeof(*fm) / sizeof(uint32_t); i++) {
			((volatile uint32_t *)fm)[i] = 0U;
		}
		shell_print(sh, "faultmark cleared");
		return 0;
	}

	if (!faultmark_present()) {
		shell_print(sh, "no faultmark at 0x%08lx (magic 0x%08x)", MESHTASTIC_FAULTMARK_ADDR,
			    fm->magic);
		return 0;
	}

	for (size_t i = 0; i < sizeof(stage); i++) {
		stage[i] = fm->stage[i];
	}
	stage[sizeof(stage) - 1] = '\0';

	shell_print(sh, "faultmark @0x%08lx:", MESHTASTIC_FAULTMARK_ADDR);
	shell_print(sh, "  reason  %u (%s)", fm->reason, reason_str(fm->reason));
	shell_print(sh, "  stage   \"%s\" (mark #%u, uptime %u ms)", stage, fm->seq, fm->uptime_ms);
	shell_print(sh, "  pc      0x%08x  lr 0x%08x  xpsr 0x%08x  thread 0x%08x", fm->pc, fm->lr,
		    fm->xpsr, fm->thread);
	shell_print(sh, "  cfsr    0x%08x  hfsr 0x%08x", fm->cfsr, fm->hfsr);
	shell_print(sh, "  bfar    0x%08x  mmfar 0x%08x", fm->bfar, fm->mmfar);
	shell_print(sh, "  ipsr    %u (IRQ %d)  ISER0 0x%08x ISER1 0x%08x ISPR0 0x%08x",
		    fm->reserved[0], (int)fm->reserved[0] - 16, fm->reserved[1], fm->reserved[2],
		    fm->reserved[3]);
	shell_print(sh, "  (addr2line -e <that image's zephyr.elf> 0x%08x 0x%08x)", fm->pc, fm->lr);
	return 0;
}

SHELL_CMD_ARG_REGISTER(faultmark, NULL,
		       "Fault record left by a reverted test image ([clear])", cmd_faultmark, 1, 1);
