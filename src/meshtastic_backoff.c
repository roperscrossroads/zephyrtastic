/* SPDX-License-Identifier: GPL-3.0 */

#include "meshtastic_backoff.h"

void meshtastic_backoff_init(struct meshtastic_backoff *b, uint32_t base_ms, uint32_t cap_ms)
{
	b->base_ms = base_ms;
	b->cap_ms = cap_ms < base_ms ? base_ms : cap_ms;
	b->attempts = 0U;
}

uint32_t meshtastic_backoff_next_ms(struct meshtastic_backoff *b, uint32_t rand)
{
	uint32_t delay = b->base_ms;

	/* Double per prior failure, stopping at cap. Looping (not shifting by
	 * attempts) keeps it free of shift-width overflow however long a
	 * failure lasts. */
	for (uint32_t i = 0U; i < b->attempts && delay < b->cap_ms; i++) {
		delay = (delay > b->cap_ms / 2U) ? b->cap_ms : delay * 2U;
	}
	if (b->attempts < UINT32_MAX) {
		b->attempts++;
	}

	return delay + rand % (delay / 4U + 1U);
}

void meshtastic_backoff_reset(struct meshtastic_backoff *b)
{
	b->attempts = 0U;
}
