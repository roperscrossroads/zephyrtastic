/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * The "meshtastic" log module, registered here and nowhere else.
 *
 * Every firmware file logs as "meshtastic" through LOG_MODULE_DECLARE, which
 * links to the log_dynamic_meshtastic / log_const_meshtastic objects this
 * register defines. Kept in a file of its own so that link is to a leaf: when
 * the register lived in meshtastic.c, 18 modules depended on the core for
 * nothing but this symbol, and it pulled them into the core's dependency cycle.
 * Add nothing else to this file.
 */

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);
