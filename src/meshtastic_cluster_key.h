/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_CLUSTER_KEY_H_
#define MESHTASTIC_CLUSTER_KEY_H_

#include <stdint.h>

/* The cluster document's SETTINGS key format, in one place.
 *
 * Header-only so the parser is reachable from tests/cluster, which compiles the document
 * unit rather than the whole cluster module -- the version of this that lived inside
 * meshtastic_cluster.c was covered by no test at all.
 */

/* Parse a cluster entry key: "<layer><node-hex>/<section>", the shape write_key() emits
 * with "%c%08x/%u". Hand-rolled because this was the firmware's ONLY scanf caller and
 * picolibc's vfscanf costs ~2 KB of flash for this one line -- meshtastic_shell.c already
 * refuses scanf for exactly that reason. Accepts 1..8 hex digits, as "%8x" did, and is
 * stricter in one way on purpose: the key must be consumed entirely, so a malformed
 * settings key is rejected rather than silently parsed from its prefix. */
static inline int meshtastic_cluster_parse_entry_key(const char *key, char *layer, uint32_t *node_id, uint32_t *section)
{
	const char *p = key;
	uint32_t id = 0U;
	uint32_t sec = 0U;
	unsigned int digits = 0U;

	if (key == NULL || *p == '\0') {
		return -1;
	}
	*layer = *p++;

	while (digits < 8U) {
		uint32_t nib;

		if (*p >= '0' && *p <= '9') {
			nib = (uint32_t)(*p - '0');
		} else if (*p >= 'a' && *p <= 'f') {
			nib = (uint32_t)(*p - 'a') + 10U;
		} else if (*p >= 'A' && *p <= 'F') {
			nib = (uint32_t)(*p - 'A') + 10U;
		} else {
			break;
		}
		id = (id << 4) | nib;
		p++;
		digits++;
	}
	if (digits == 0U || *p != '/') {
		return -1;
	}
	p++;
	if (*p < '0' || *p > '9') {
		return -1;
	}
	while (*p >= '0' && *p <= '9') {
		uint32_t d = (uint32_t)(*p - '0');

		if (sec > (UINT32_MAX - d) / 10U) {
			return -1; /* would overflow: not a key we ever wrote */
		}
		sec = sec * 10U + d;
		p++;
	}
	if (*p != '\0') {
		return -1;
	}
	*node_id = id;
	*section = sec;
	return 0;
}


#endif /* MESHTASTIC_CLUSTER_KEY_H_ */
