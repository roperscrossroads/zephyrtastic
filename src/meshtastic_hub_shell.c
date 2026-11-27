/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic hub` on a hub: its workers. Built with the shell and
 * CONFIG_MESHTASTIC_HUB (src/CMakeLists.txt); moved from meshtastic_shell.c
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

#include "meshtastic_hub.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* `meshtastic hub`: the radios this hub hears through (ATTACHMENT-DESIGN
 * S6). Radio 0 is this board's own radio; the rest are workers that spoke
 * to us over the peer link. */
static void hub_print_row(const struct shell *sh, const struct meshtastic_hub_info *a)
{
	if (a->id == 0U) {
		shell_print(sh, "  0  local            preset %u  last %d dBm / %d dB", a->preset,
			    a->last_rssi, a->last_snr);
		return;
	}
	shell_print(sh, "  %u  0x%08x  preset %u  rx %u (dropped %u)  tx %u  last %d dBm / %d dB"
		    "  [%d..%d]%s%s",
		    a->id, a->node, a->preset, a->rx_frames, a->rx_dropped, a->tx_frames,
		    a->last_rssi, a->last_snr, a->rssi_min, a->rssi_max,
		    a->have_status ? "  status" : "",
		    (a->have_status &&
		     (a->status.flags & MESHTASTIC_LINK_ST_IS_WORKER) != 0U) ? " worker" : " worker-rx");
	{
		uint8_t wanted = meshtastic_hub_wanted_preset(a->node);

		if (wanted != MESHTASTIC_PRESET_UNKNOWN) {
			shell_print(sh, "     wanted preset %u%s", (unsigned int)wanted,
				    wanted == a->preset ? "" : "  (NOT there yet)");
		}
	}
	if (a->warn_fast_worker || a->warn_lag) {
		shell_print(sh, "     %s%s  (DESIGN §13: the fastest preset belongs on the hub's radio)",
			    a->warn_fast_worker ? "FAST-WORKER " : "", a->warn_lag ? "LAG " : "");
	}
	if (!a->link_up) {
		shell_print(sh, "     link DOWN %lld s (evicted after %d s)",
			    (long long)((k_uptime_get() - a->down_ms) / 1000),
			    CONFIG_MESHTASTIC_HUB_EVICT_SEC);
	}
}

static int cmd_hub_allow(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_hub_stats st;
	uint32_t node;

	if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
		meshtastic_hub_allow_clear();
		shell_print(sh, "allow-list cleared");
		return 0;
	}
	if (argc >= 2) {
		int ret;

		node = (uint32_t)strtoul(argv[1], NULL, 16);
		ret = meshtastic_hub_allow_add(node);
		if (ret < 0) {
			shell_error(sh, "allow failed (%d)", ret);
			return ret;
		}
		shell_print(sh, "0x%08x allowed", node);
		return 0;
	}
	shell_print(sh, "allowed on an untrusted link:");
	for (unsigned int i = 0U; meshtastic_hub_allow_get(i, &node); i++) {
		shell_print(sh, "  0x%08x", node);
	}
	meshtastic_hub_stats_get(&st);
	shell_print(sh, "refused %u  malformed-first-contact %u  evicted %u", st.admission_refused,
		    st.not_admitted_malformed, st.evicted);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_allow)

static int cmd_hub_list(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_hub_info a;
	unsigned int shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "links: %u (of %u + local)", meshtastic_hub_count(),
		    (unsigned int)CONFIG_MESHTASTIC_HUB_WORKERS_MAX);
	for (uint8_t id = 0U; id <= CONFIG_MESHTASTIC_HUB_WORKERS_MAX; id++) {
		if (meshtastic_hub_get(id, &a)) {
			hub_print_row(sh, &a);
			shown++;
		}
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_list)

static int cmd_hub_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_hub_info a;
	unsigned long id = strtoul(argv[1], NULL, 10);

	ARG_UNUSED(argc);
	if (id > CONFIG_MESHTASTIC_HUB_WORKERS_MAX || !meshtastic_hub_get((uint8_t)id, &a)) {
		shell_error(sh, "no radio %lu", id);
		return -ENOENT;
	}
	hub_print_row(sh, &a);
	if (a.have_status) {
		shell_print(sh, "  reports: preset %u flags 0x%02x hwid 0x%08x hub 0x%08x",
			    a.status.preset, a.status.flags, a.status.hwid, a.status.hub);
		shell_print(sh, "  reports: rx %u tx %u dropped %u uptime %u s", a.status.rx_frames,
			    a.status.tx_frames, a.status.rx_dropped, a.status.uptime_s);
		if ((a.status.flags & MESHTASTIC_LINK_ST_HAS_POS) != 0U) {
			shell_print(sh, "  position: lat=%d lon=%d alt=%d", a.status.lat,
				    a.status.lon, a.status.alt);
		}
	}
	if (a.lag_n != 0U) {
		shell_print(sh, "  link lag (relay decisions, worker's clock): n %u  p50 %u  p90 %u  max %u ms%s%s",
			    a.lag_n, a.lag_p50_ms, a.lag_p90_ms, a.lag_max_ms,
			    a.warn_lag ? "  LAG" : "", a.warn_fast_worker ? "  FAST-WORKER" : "");
	}
	shell_print(sh, "  last envelope %lld ms ago  rejected %u  tx_results %u",
		    (long long)(k_uptime_get() - a.last_ms), a.rejected, a.tx_results);
	if (a.lat_n != 0U) {
		shell_print(sh, "  link latency vs local radio: n %u  min %u  avg %u  max %u ms",
			    a.lat_n, a.lat_min_ms, a.lat_sum_ms / a.lat_n, a.lat_max_ms);
	}
	if (a.id != 0U) {
		shell_print(sh, "  tx: handed %u (seq %u)  results %u  cancelled %u  late %u  failed %u  "
				"last rc %d after %u defers%s",
			    a.tx_frames, a.tx_seq, a.tx_results, a.tx_cancelled, a.tx_late,
			    a.tx_failed, a.last_tx_rc,
			    a.last_tx_defers, meshtastic_hub_tx_ready(a.id) ? "" : "  [not ready]");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_status)

/* A preset as a name (ShortFast), its number (6), or "none". Anything else is refused: a
 * name used to parse as the number 0, which is LongFast, and put the worker there. */
static int hub_parse_preset(const char *s, uint8_t *out)
{
	meshtastic_Config_LoRaConfig_ModemPreset named;
	unsigned long n;
	char *end;

	if (strcmp(s, "none") == 0) {
		*out = MESHTASTIC_PRESET_UNKNOWN;
		return 0;
	}
	if (mtsh_parse_modem_preset(s, &named) == 0) {
		*out = (uint8_t)named;
		return 0;
	}
	n = strtoul(s, &end, 10);
	if (end == s || *end != '\0' ||
	    n > (unsigned long)_meshtastic_Config_LoRaConfig_ModemPreset_MAX) {
		return -EINVAL;
	}
	*out = (uint8_t)n;
	return 0;
}

static int cmd_hub_preset(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long id = strtoul(argv[1], &end, 10);
	uint8_t preset;
	int ret;

	ARG_UNUSED(argc);
	if (end == argv[1] || *end != '\0' || id == 0U || id > CONFIG_MESHTASTIC_HUB_WORKERS_MAX) {
		shell_error(sh, "usage: meshtastic hub preset <id> <name|number|none>");
		return -EINVAL;
	}
	if (hub_parse_preset(argv[2], &preset) < 0) {
		shell_error(sh, "unknown preset '%s' (a name as `meshtastic lora` prints it, "
				"e.g. ShortFast; its number; or none)", argv[2]);
		return -EINVAL;
	}
	ret = meshtastic_hub_set_preset((uint8_t)id, preset);
	if (preset == MESHTASTIC_PRESET_UNKNOWN) {
		if (ret == 0) {
			shell_print(sh, "worker %lu: no preset wanted of it any more", id);
		} else {
			shell_error(sh, "failed (%d)", ret);
		}
		return ret;
	}
	if (ret == -EINVAL || ret == -ENOENT || ret == -ENOSPC) {
		shell_error(sh, "set_preset failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "worker %lu: preset %u wanted and stored; %s", id,
		    (unsigned int)preset,
		    ret == 0 ? "asked it to retune"
			     : "the link is down, it is asked when it next speaks");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_preset)

static int cmd_hub_power(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long id = strtoul(argv[1], NULL, 10);
	long dbm = strtol(argv[2], NULL, 10);
	int ret;

	ARG_UNUSED(argc);
	if (id == 0U || id > CONFIG_MESHTASTIC_HUB_WORKERS_MAX || dbm < -128 || dbm > 127) {
		shell_error(sh, "usage: meshtastic hub power <id> <dBm> (0 = region max)");
		return -EINVAL;
	}
	ret = meshtastic_hub_set_tx_power((uint8_t)id, (int8_t)dbm);
	if (ret < 0) {
		shell_error(sh, "set_policy failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "asked worker %lu to transmit at %ld dBm (it persists it and reports)", id,
		    dbm);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_power)

static int cmd_hub_forget(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long id = strtoul(argv[1], NULL, 10);
	int ret;

	ARG_UNUSED(argc);
	ret = meshtastic_hub_forget((uint8_t)id);
	if (ret < 0) {
		shell_error(sh, "forget failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "worker %lu forgotten", id);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_hub_forget)

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_hub_cmds,
	SHELL_CMD(list, NULL, SHELL_HELP("Every radio this hub hears through.", NULL),
		  cmd_hub_list_on_work),
	SHELL_CMD_ARG(status, NULL, SHELL_HELP("One radio in detail.", "<id>"),
		      cmd_hub_status_on_work, 2, 0),
	SHELL_CMD_ARG(power, NULL, SHELL_HELP("Set a worker's transmit power.", "<id> <dBm>"),
		      cmd_hub_power_on_work, 3, 0),
	SHELL_CMD_ARG(preset, NULL,
		      SHELL_HELP("Put a worker on a preset and keep it there (stored; re-sent "
				 "when the worker comes back on another).",
				 "<id> <name|number|none>"),
		      cmd_hub_preset_on_work, 3, 0),
	SHELL_CMD_ARG(allow, NULL,
		      SHELL_HELP("Workers admitted on an untrusted link: show, add, clear.",
				 "[<hex node id>|clear]"),
		      cmd_hub_allow_on_work, 1, 1),
	SHELL_CMD_ARG(forget, NULL, SHELL_HELP("Drop a worker from the table.", "<id>"),
		      cmd_hub_forget_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), hub, &meshtastic_hub_cmds,
		 SHELL_HELP("This hub's radios and workers: list, status, preset, forget.", NULL),
		 cmd_hub_list_on_work, 0, 0);

/* `meshtastic attach ...`: the old name, for one release (CLASS-NAMING, renamed 2026-11-27). Each
 * alias says the new name, then runs it. No help text: the alias stays out of the descriptions. */
MTSH_RENAMED(cmd_attach_list, cmd_hub_list, "hub list")
MTSH_RENAMED(cmd_attach_status, cmd_hub_status, "hub status")
MTSH_RENAMED(cmd_attach_power, cmd_hub_power, "hub power")
MTSH_RENAMED(cmd_attach_preset, cmd_hub_preset, "hub preset")
MTSH_RENAMED(cmd_attach_allow, cmd_hub_allow, "hub allow")
MTSH_RENAMED(cmd_attach_forget, cmd_hub_forget, "hub forget")

SHELL_STATIC_SUBCMD_SET_CREATE(
	meshtastic_attach_alias_cmds,
	SHELL_CMD(list, NULL, NULL, cmd_attach_list_on_work),
	SHELL_CMD_ARG(status, NULL, NULL, cmd_attach_status_on_work, 2, 0),
	SHELL_CMD_ARG(power, NULL, NULL, cmd_attach_power_on_work, 3, 0),
	SHELL_CMD_ARG(preset, NULL, NULL, cmd_attach_preset_on_work, 3, 0),
	SHELL_CMD_ARG(allow, NULL, NULL, cmd_attach_allow_on_work, 1, 1),
	SHELL_CMD_ARG(forget, NULL, NULL, cmd_attach_forget_on_work, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), attach, &meshtastic_attach_alias_cmds, NULL,
		 cmd_attach_list_on_work, 0, 0);
