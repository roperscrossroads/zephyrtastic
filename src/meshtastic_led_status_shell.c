/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic led`: the bench LED. Built with the shell and CONFIG_MESHTASTIC_LED_STATUS
 * (src/CMakeLists.txt); moved from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_led_status.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic led`: the bench LED (meshtastic_led_status.h) -- is this board
 * in use, and which one is it. The host sets the mode from the bench's claims. */
static int cmd_led_show(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t left = meshtastic_led_status_locate_left();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	enum meshtastic_led_color c = meshtastic_led_status_get_color();

	shell_print(sh, "mode: %s  color: %s  (%s)",
		    meshtastic_led_mode_name(meshtastic_led_status_get_mode()),
		    meshtastic_led_color_name(c),
		    !meshtastic_led_status_has_color()
			    ? "one LED: idle slow blink, in-use off, locate fast blink"
		    : c == MESHTASTIC_LED_COLOR_DEFAULT
			    ? "RGB: idle red, in-use green, locate blue"
			    : "RGB: idle blinks the color, in-use steady, locate white");
	if (left != 0U) {
		shell_print(sh, "locating: %u s left", left);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_led_show)

static int cmd_led_mode(const struct shell *sh, size_t argc, char **argv)
{
	enum meshtastic_led_mode m;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "idle") == 0) {
		m = MESHTASTIC_LED_IDLE;
	} else if (strcmp(argv[1], "in-use") == 0) {
		m = MESHTASTIC_LED_IN_USE;
	} else if (strcmp(argv[1], "off") == 0) {
		m = MESHTASTIC_LED_OFF;
	} else {
		shell_error(sh, "mode: idle, in-use or off");
		return -EINVAL;
	}
	(void)meshtastic_led_status_set_mode(m);
	shell_print(sh, "mode: %s (saved)", meshtastic_led_mode_name(m));
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_led_mode)

static int cmd_led_color(const struct shell *sh, size_t argc, char **argv)
{
	enum meshtastic_led_color c;

	ARG_UNUSED(argc);
	if (meshtastic_led_color_parse(argv[1], &c) != 0) {
		shell_error(sh, "color: default, red, green, blue, yellow, cyan or magenta");
		return -EINVAL;
	}
	(void)meshtastic_led_status_set_color(c);
	shell_print(sh, "color: %s (saved)", meshtastic_led_color_name(c));
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_led_color)

static int cmd_led_locate(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t s = (argc > 1) ? (uint32_t)strtoul(argv[1], NULL, 10)
				: CONFIG_MESHTASTIC_LED_STATUS_LOCATE_SEC;

	meshtastic_led_status_locate(s);
	if (s == 0U) {
		shell_print(sh, "locate: stopped");
	} else {
		shell_print(sh, "locate: %u s", s);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_led_locate)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_led_cmds,
	SHELL_CMD_ARG(mode, NULL, SHELL_HELP("Set and save the mode.", "<idle|in-use|off>"),
		      cmd_led_mode_on_work, 2, 0),
	SHELL_CMD_ARG(locate, NULL, SHELL_HELP("Blink \"here I am\"; 0 stops.", "[seconds]"),
		      cmd_led_locate_on_work, 1, 1),
	SHELL_CMD_ARG(color, NULL,
		      SHELL_HELP("Set and save what the board is for, as a color (RGB boards).",
				 "<default|red|green|blue|yellow|cyan|magenta>"),
		      cmd_led_color_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), led, &meshtastic_led_cmds,
		 SHELL_HELP("Bench LED: show, mode, color, locate.", NULL),
		 cmd_led_show_on_work, 0, 0);
