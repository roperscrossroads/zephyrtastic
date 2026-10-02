/* SPDX-License-Identifier: GPL-3.0 */
#ifndef MESHTASTIC_SHELL_STATE_H_
#define MESHTASTIC_SHELL_STATE_H_

#include <zephyr/shell/shell.h>

/* Print this node's state as one machine-readable line: `~S{json}*crc16`. */
int meshtastic_shell_state(const struct shell *sh);

#endif /* MESHTASTIC_SHELL_STATE_H_ */
