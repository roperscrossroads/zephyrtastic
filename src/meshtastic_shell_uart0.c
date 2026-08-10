/* SPDX-License-Identifier: GPL-3.0
 *
 * Second interactive shell instance, bound to whichever device a board
 * overlay names via the `zephyr,meshtastic-shell-uart0` chosen node (see
 * boards/heltec_wifi_lora32_v4_esp32s3_procpu.overlay). Mirrors the primary
 * native-USB console shell's own registration (zephyr/subsys/shell/backends/
 * shell_uart.c, enable_shell_uart()) almost line for line -- same transport
 * code (shell_uart_transport_api), same buffer-size/log-level Kconfig knobs,
 * just a different device and a distinguishing prompt so it's obvious which
 * physical port a session is on.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#if DT_HAS_CHOSEN(zephyr_meshtastic_shell_uart0)

SHELL_UART_DEFINE(meshtastic_shell_transport_uart0);
SHELL_DEFINE(meshtastic_shell_uart0, "uart0:~$ ", &meshtastic_shell_transport_uart0,
	     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_SIZE,
	     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_TIMEOUT, SHELL_FLAG_OLF_CRLF);

static int meshtastic_shell_uart0_init(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_meshtastic_shell_uart0));
	bool log_backend = CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > 0;
	uint32_t level = (CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > LOG_LEVEL_DBG)
				  ? CONFIG_LOG_MAX_LEVEL
				  : CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL;
	static const struct shell_backend_config_flags cfg_flags = SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return shell_init(&meshtastic_shell_uart0, dev, cfg_flags, log_backend, level);
}

/* Same priority as the primary UART shell (zephyr/subsys/shell/backends/
 * shell_uart.c) -- the two instances are independent peripherals with no
 * ordering dependency between them. */
SYS_INIT(meshtastic_shell_uart0_init, POST_KERNEL, CONFIG_SHELL_BACKEND_SERIAL_INIT_PRIORITY);

#endif /* DT_HAS_CHOSEN(zephyr_meshtastic_shell_uart0) */
