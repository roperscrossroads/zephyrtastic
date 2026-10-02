/* SPDX-License-Identifier: GPL-3.0 */
#ifndef MESHTASTIC_SHELL_API_H_
#define MESHTASTIC_SHELL_API_H_

#include <zephyr/shell/shell.h>

/* `meshtastic api`: the phone API carried as text over the shell. */
int meshtastic_shell_api_cmd(const struct shell *sh, size_t argc, char **argv);

/* Register the transport. Called once, before the shell work thread starts. */
void meshtastic_shell_api_init(void);

/* One step of the transport, on the shell work thread. */
void meshtastic_shell_api_work(void);

/* Provided by meshtastic_shell.c: have the work thread call
 * meshtastic_shell_api_work(). Negative if its queue is full. */
int meshtastic_shell_api_submit(void);

#endif /* MESHTASTIC_SHELL_API_H_ */
