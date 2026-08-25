/* SPDX-License-Identifier: GPL-3.0
 *
 * See meshtastic_ble_name.h for why the advertised name is composed rather than
 * constant. This file holds only the rule, with no Bluetooth or kernel
 * dependency, so tests/ble_name can assert it directly.
 */
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "meshtastic_ble_name.h"

#if !defined(CONFIG_MESHTASTIC_BLE_NAME_PREFIX)
/* The unit test builds this file without the BLE Kconfig menu. */
#define CONFIG_MESHTASTIC_BLE_NAME_PREFIX "zeph-"
#endif

size_t meshtastic_ble_adv_name_compose(char *out, size_t out_len, const char *short_name,
				       uint32_t node_id)
{
	int n;

	if (out == NULL || out_len == 0U) {
		return 0U;
	}

	if ((short_name != NULL) && (short_name[0] != '\0')) {
		n = snprintf(out, out_len, "%s%s", CONFIG_MESHTASTIC_BLE_NAME_PREFIX, short_name);
	} else {
		/* Low three bytes: enough to separate any two boards on a bench,
		 * and it keeps the name short enough to read on a phone. */
		n = snprintf(out, out_len, "%s%06x", CONFIG_MESHTASTIC_BLE_NAME_PREFIX,
			     (unsigned int)(node_id & 0xffffffU));
	}

	if (n < 0) {
		/* Cannot happen with these formats. Emit nothing rather than
		 * leave a stale or uninitialised name in the advert. */
		out[0] = '\0';
		return 0U;
	}

	/* NOT n: snprintf returns what it WOULD have written, which overstates
	 * the truncated case and would hand the advert a length past the buffer.
	 * snprintf always terminates when out_len is non-zero, so strlen is the
	 * honest answer. */
	return strlen(out);
}
