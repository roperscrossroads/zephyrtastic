/* SPDX-License-Identifier: GPL-3.0 */
/*
 * What more than one shell command group shares (agents-ls39.4.4.1): argument
 * parsers and the formatting of numbers and names. Each group's own helpers stay
 * with the group. The gates and the work thread are in meshtastic_shell.c.
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "meshtastic_region_presets.h"
#include "meshtastic_shell_internal.h"

int mtsh_parse_u32(const struct shell *sh, const char *arg, uint32_t *value)
{
	char *end;
	unsigned long parsed;

	if (strcmp(arg, "broadcast") == 0) {
		*value = MESHTASTIC_NODE_BROADCAST;
		return 0;
	}

	parsed = strtoul(arg, &end, 0);
	if (*end != '\0' || parsed > UINT32_MAX) {
		shell_error(sh, "invalid integer: %s", arg);
		return -EINVAL;
	}

	*value = (uint32_t)parsed;
	return 0;
}

int mtsh_parse_optional_dest(const struct shell *sh, size_t argc, char **argv, uint32_t *dest)
{
	if (argc == 1U) {
		*dest = MESHTASTIC_NODE_BROADCAST;
		return 0;
	}

	if (argc == 2U) {
		return mtsh_parse_u32(sh, argv[1], dest);
	}

	shell_error(sh, "too many arguments");
	return -EINVAL;
}

int mtsh_parse_channel_index(const struct shell *sh, const char *arg, uint8_t *index)
{
	unsigned long parsed;
	char *end;

	parsed = strtoul(arg, &end, 0);
	if (*end != '\0' || parsed >= MESHTASTIC_MAX_CHANNELS) {
		shell_error(sh, "invalid channel index: %s", arg);
		return -EINVAL;
	}

	*index = (uint8_t)parsed;
	return 0;
}

/* Match a preset display name (as reported by meshtastic_preset_display_name, so
 * the accepted spellings always equal what `meshtastic lora` shows).
 *
 * Outside the config-write guard, because parsing a preset name has nothing to
 * do with writing config: `scan presets` and `preset hop` both move the live
 * radio only, and a build with writes compiled out still wants them. */
int mtsh_parse_modem_preset(const char *s, meshtastic_Config_LoRaConfig_ModemPreset *out)
{
	static const meshtastic_Config_LoRaConfig_ModemPreset presets[] = {
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE,
		meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,
		meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
	};

	for (size_t i = 0U; i < ARRAY_SIZE(presets); i++) {
		if (strcmp(s, meshtastic_preset_display_name(presets[i], true)) == 0) {
			*out = presets[i];
			return 0;
		}
	}
	return -EINVAL;
}

int mtsh_append_argv(uint8_t *buf, size_t buf_max, size_t argc, char **argv, size_t first_arg)
{
	size_t pos = 0;

	for (size_t i = first_arg; i < argc; i++) {
		size_t len = strlen(argv[i]);

		if (i > first_arg) {
			if (pos >= buf_max) {
				return -ENOMEM;
			}
			buf[pos++] = ' ';
		}

		if (pos + len > buf_max) {
			return -ENOMEM;
		}

		memcpy(buf + pos, argv[i], len);
		pos += len;
	}

	return (int)pos;
}

int32_t mtsh_scaled_tenths(float value)
{
	return (int32_t)(value * 10.0f);
}

int32_t mtsh_scaled_whole(int32_t scaled, int32_t divisor)
{
	return scaled / divisor;
}

uint32_t mtsh_scaled_fraction(int32_t scaled, int32_t divisor)
{
	int32_t fraction = scaled % divisor;

	if (fraction < 0) {
		fraction = -fraction;
	}

	return (uint32_t)fraction;
}

const char *mtsh_clock_quality_name(enum meshtastic_clock_quality q)
{
	switch (q) {
	case MESHTASTIC_CLOCK_QUALITY_NONE:
		return "NONE (never set)";
	case MESHTASTIC_CLOCK_QUALITY_DEVICE:
		return "device RTC";
	case MESHTASTIC_CLOCK_QUALITY_NET:
		return "mesh-relayed";
	case MESHTASTIC_CLOCK_QUALITY_NTP:
		return "NTP/operator";
	case MESHTASTIC_CLOCK_QUALITY_GPS:
		return "GPS";
	default:
		return "?";
	}
}

const char *mtsh_device_role_name(meshtastic_Config_DeviceConfig_Role role)
{
	switch (role) {
	case meshtastic_Config_DeviceConfig_Role_CLIENT:
		return "client";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE:
		return "client_mute";
	case meshtastic_Config_DeviceConfig_Role_ROUTER:
		return "router";
	case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
		return "router_late";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_BASE:
		return "client_base";
	case meshtastic_Config_DeviceConfig_Role_SENSOR:
		return "sensor";
	case meshtastic_Config_DeviceConfig_Role_TRACKER:
		return "tracker";
	case meshtastic_Config_DeviceConfig_Role_TAK:
		return "tak";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN:
		return "client_hidden";
	case meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND:
		return "lost_and_found";
	case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
		return "tak_tracker";
	default:
		return "unknown";
	}
}

const char *mtsh_rebroadcast_mode_name(meshtastic_Config_DeviceConfig_RebroadcastMode mode)
{
	switch (mode) {
	case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL:
		return "all";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING:
		return "all_skip_decoding";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY:
		return "local_only";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY:
		return "known_only";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_NONE:
		return "none";
	case meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY:
		return "core_portnums_only";
	default:
		return "unknown";
	}
}
