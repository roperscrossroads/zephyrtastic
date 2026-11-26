/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic netpause` (take the network down for N seconds) and `meshtastic netlog`
 * (start the remote syslog backend). Built with the shell and CONFIG_NETWORKING
 * (src/CMakeLists.txt); moved from meshtastic_shell.c (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/net/net_if.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
#include <zephyr/logging/log_backend_net.h>
#endif

/*
 * netpause <secs> — take the network fully down for a bounded window, then bring it back
 * up unattended.
 *
 * This exists because every network-based way of observing light sleep perturbs it: a
 * telnet query is WiFi traffic, and WiFi traffic makes the esp_pm "wifi" lock (and the
 * whole net/MQTT/syslog stack) wake the SoC. Residency read while connected therefore
 * measures the measurement. Here the shell thread sleeps (so the CPU may idle), the
 * network is genuinely off, and because the PM residency counters are cumulative you can
 * read them afterwards for an UNPERTURBED figure.
 *
 * Best driven over the USB/serial console: the session survives the network going down.
 * Over telnet it still works, but the session drops and you reconnect after `secs`.
 */
static int cmd_net_pause(const struct shell *sh, size_t argc, char **argv)
{
	struct net_if *iface = net_if_get_default();
	uint32_t secs;
	char *end;

	if (argc < 2U) {
		shell_error(sh, "usage: meshtastic netpause <secs>");
		return -EINVAL;
	}

	secs = (uint32_t)strtoul(argv[1], &end, 10);
	if (*end != '\0' || secs == 0U || secs > 600U) {
		shell_error(sh, "secs must be 1..600");
		return -EINVAL;
	}

	if (iface == NULL) {
		shell_error(sh, "no default network interface");
		return -ENODEV;
	}

	shell_print(sh, "network DOWN for %u s (telnet sessions will drop)...", secs);
	(void)net_if_down(iface);

	k_sleep(K_SECONDS(secs));

	(void)net_if_up(iface);
	shell_print(sh, "network back UP; read residency with `stats list`");

	return 0;
}

#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
/* Remote syslog no longer starts itself -- see the rationale on
 * CONFIG_LOG_BACKEND_NET_AUTOSTART in overlay-v4-unified.conf. It stays compiled
 * so a soak that genuinely wants continuous remote capture can turn it on for
 * that boot without a reflash. Deliberately NOT persisted: the default has to
 * stay "off" across a reset, or the reasons it was turned off quietly reassert
 * themselves the next time a node reboots unattended.
 */
static int cmd_netlog(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || strcmp(argv[1], "on") != 0) {
		shell_print(sh, "usage: meshtastic netlog on");
		shell_print(sh, "  (off by default; not persisted across a reboot)");
		return 0;
	}

	log_backend_net_start();
	shell_print(sh, "netlog: started for this boot only");

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_netlog)
#endif

SHELL_SUBCMD_ADD((meshtastic), netpause, NULL,
		 SHELL_HELP("Take the network down for N seconds, then restore it. "
		 "Lets PM residency be measured without network traffic waking "
		 "the SoC.", "<secs>"),
		 cmd_net_pause, 0, 0);
#if defined(CONFIG_LOG_BACKEND_NET) && !defined(CONFIG_LOG_BACKEND_NET_AUTOSTART)
SHELL_SUBCMD_ADD((meshtastic), netlog, NULL,
		 SHELL_HELP("Start the remote syslog backend for this boot.",
		 "netlog on"),
		 cmd_netlog_on_work, 0, 0);
#endif
