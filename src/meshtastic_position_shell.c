/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic position`: gps_mode and the broadcast interval. Built with the shell and
 * CONFIG_MESHTASTIC_POSITION (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_hlc.h"
#include "meshtastic_position.h"
#include "meshtastic_preset.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic position` (agents-t2hb.2) -- the two PositionConfig fields with a
 * consumer: gps_mode (the receiver's power) and position_broadcast_secs (both
 * senders' cadence). Same shape as `nodeinfo interval`: reads always available,
 * writes gated behind CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE and is_managed. */
static const char *const gps_mode_names[] = {
	[meshtastic_Config_PositionConfig_GpsMode_DISABLED] = "disabled",
	[meshtastic_Config_PositionConfig_GpsMode_ENABLED] = "enabled",
	[meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT] = "not_present",
};

static const char *gps_mode_name(meshtastic_Config_PositionConfig_GpsMode mode)
{
	return ((size_t)mode < ARRAY_SIZE(gps_mode_names) && gps_mode_names[mode] != NULL)
		       ? gps_mode_names[mode]
		       : "?";
}

static const char *loc_source_name(meshtastic_Position_LocSource src)
{
	switch (src) {
	case meshtastic_Position_LocSource_LOC_MANUAL:
		return "fixed";
	case meshtastic_Position_LocSource_LOC_INTERNAL:
		return "gnss";
	case meshtastic_Position_LocSource_LOC_EXTERNAL:
		return "phone";
	default:
		return "unset";
	}
}

static int cmd_position_show(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config cfg;
	meshtastic_Position pos;
	bool have;
	bool stored;

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	have = meshtastic_config_store_get_config(meshtastic_Config_position_tag, &cfg) == 0 &&
	       cfg.which_payload_variant == meshtastic_Config_position_tag;
	stored = have && cfg.payload_variant.position.position_broadcast_secs != 0U;

	shell_print(sh, "gps_mode: %s", gps_mode_name(meshtastic_position_gps_mode()));
#if defined(CONFIG_MESHTASTIC_GNSS)
	{
		struct meshtastic_gnss_status st;

		if (meshtastic_gnss_status_get(&st) == 0) {
			shell_print(sh, "receiver: %s", st.enabled ? "on" : "off");
		} else {
			shell_print(sh, "receiver: none in this build's devicetree");
		}
	}
#else
	shell_print(sh, "receiver: GNSS not compiled in");
#endif
	shell_print(sh, "interval: %u s%s", meshtastic_position_broadcast_secs(),
		    stored ? "" : " (compiled default; position_broadcast_secs unset)");
	/* The smart trio, the flags and whether anyone ever wrote the section: what
	 * a bench needs to explain a position that went out without altitude
	 * (2026-09-27 -- a pre-Track-C client write had stamped zeros into kit1). */
	if (have) {
		const meshtastic_Config_PositionConfig *pc = &cfg.payload_variant.position;
		struct meshtastic_hlc_stamp stamp;
		bool written = meshtastic_config_store_get_config_stamp(
				       meshtastic_Config_position_tag, &stamp) == 0 &&
			       !meshtastic_hlc_stamp_is_unset(&stamp);

		shell_print(sh, "smart: %s (%u m, %u s)",
			    pc->position_broadcast_smart_enabled ? "on" : "off",
			    pc->broadcast_smart_minimum_distance != 0U
				    ? pc->broadcast_smart_minimum_distance : 100U,
			    pc->broadcast_smart_minimum_interval_secs != 0U
				    ? pc->broadcast_smart_minimum_interval_secs : 300U);
		shell_print(sh, "flags: 0x%x%s", pc->position_flags,
			    pc->position_flags == 0U ? " (lat/lon/time only)" : "");
		shell_print(sh, "section: %s",
			    written ? "written (stamped by a client, the shell or a cluster doc)"
				    : "never written (build seed)");
	}
	if (meshtastic_position_get_current(&pos) == 0) {
		shell_print(sh, "position: lat=%d lon=%d alt=%d (%s)", pos.latitude_i,
			    pos.longitude_i, pos.altitude,
			    loc_source_name(pos.location_source));
	} else {
		shell_print(sh, "position: none");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_position_show)

/* Bare `meshtastic position` shows. Anything else reaching here is a subcommand
 * this build does not have -- the write forms, when config writes are compiled
 * out -- and must fail rather than print the status and report success. */
static int cmd_position_root(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1U) {
		shell_error(sh, "unknown subcommand: %s", argv[1]);
		return -EINVAL;
	}
	return cmd_position_show(sh, argc, argv);
}
MTSH_DEFINE_ON_WORK(cmd_position_root)

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
static int cmd_position_interval(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long secs;
	char *end;
	int ret;

	if (argc == 1U) {
		return cmd_position_show(sh, argc, argv);
	}
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	secs = strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs > UINT32_MAX) {
		shell_error(sh, "invalid seconds value: %s", argv[1]);
		return -EINVAL;
	}
	ret = meshtastic_position_set_broadcast_secs((uint32_t)secs);
	if (ret < 0) {
		shell_error(sh, "interval set failed: %d", ret);
		return ret;
	}
	shell_print(sh, "interval -> %u s%s (persisted; applies now, no reboot)",
		    meshtastic_position_broadcast_secs(),
		    (secs == 0UL) ? " (compiled default)" : "");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_position_interval)

static int cmd_position_gps_mode(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc == 1U) {
		return cmd_position_show(sh, argc, argv);
	}
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}
	for (size_t i = 0U; i < ARRAY_SIZE(gps_mode_names); i++) {
		if (gps_mode_names[i] != NULL && strcmp(argv[1], gps_mode_names[i]) == 0) {
			ret = meshtastic_position_set_gps_mode(
				(meshtastic_Config_PositionConfig_GpsMode)i);
			if (ret < 0) {
				shell_error(sh, "gps_mode set failed: %d", ret);
				return ret;
			}
			shell_print(sh, "gps_mode -> %s (persisted; applies now, no reboot)",
				    gps_mode_names[i]);
			return 0;
		}
	}
	shell_error(sh, "usage: meshtastic position gps_mode [enabled|disabled|not_present]");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_position_gps_mode)
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_position_cmds,
	SHELL_CMD(show, NULL, SHELL_HELP("gps_mode, broadcast interval, current position.", NULL),
		  cmd_position_show_on_work),
#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	SHELL_CMD_ARG(interval, NULL,
		      SHELL_HELP("Show/set position_broadcast_secs (0 = compiled default).",
				 "[secs]"),
		      cmd_position_interval_on_work, 1, 1),
	SHELL_CMD_ARG(gps_mode, NULL,
		      SHELL_HELP("Show/set PositionConfig.gps_mode.",
				 "[enabled|disabled|not_present]"),
		      cmd_position_gps_mode_on_work, 1, 1),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), position, &meshtastic_position_cmds,
		 SHELL_HELP("Position: gps_mode, broadcast interval.", NULL),
		 cmd_position_root_on_work, 0, 0);
