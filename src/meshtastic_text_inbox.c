/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Text inbox: see meshtastic_text_inbox.h.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "meshtastic_text_inbox.h"

#define ENTRIES CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES

static struct meshtastic_text_inbox_entry ring[ENTRIES];
/* Where the next message goes, and how many are kept. */
static unsigned int head;
static unsigned int kept;
static uint32_t next_seq = 1U;

static K_MUTEX_DEFINE(inbox_lock);

void meshtastic_text_inbox_record(const struct meshtastic_packet *packet)
{
	struct meshtastic_text_inbox_entry *e;
	size_t len;

	if (packet == NULL || packet->portnum != MESHTASTIC_PORT_TEXT_MESSAGE) {
		return;
	}

	len = MIN(packet->payload_len, (size_t)CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN);

	k_mutex_lock(&inbox_lock, K_FOREVER);
	e = &ring[head];
	memset(e, 0, sizeof(*e));
	e->seq = next_seq++;
	e->uptime_ms = k_uptime_get();
	e->from = packet->from;
	e->to = packet->to;
	e->id = packet->id;
	e->rssi = packet->rssi;
	e->snr = packet->snr;
	e->channel_index = packet->channel_index;
	e->rx_attach = packet->rx_attach;
	e->rx_heard_on = packet->rx_heard_on;
	e->hops = packet->hop_start >= packet->hop_limit
			  ? (uint8_t)(packet->hop_start - packet->hop_limit)
			  : 0U;
	e->pki = packet->pki_encrypted;
	e->truncated = packet->payload_len > (size_t)CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN;
	e->len = (uint16_t)len;
	if (len > 0U && packet->payload != NULL) {
		memcpy(e->text, packet->payload, len);
	}
	e->text[len] = '\0';
	head = (head + 1U) % ENTRIES;
	if (kept < ENTRIES) {
		kept++;
	}
	k_mutex_unlock(&inbox_lock);
}

unsigned int meshtastic_text_inbox_count(void)
{
	unsigned int n;

	k_mutex_lock(&inbox_lock, K_FOREVER);
	n = kept;
	k_mutex_unlock(&inbox_lock);
	return n;
}

uint32_t meshtastic_text_inbox_total(void)
{
	uint32_t n;

	k_mutex_lock(&inbox_lock, K_FOREVER);
	n = next_seq - 1U;
	k_mutex_unlock(&inbox_lock);
	return n;
}

int meshtastic_text_inbox_get(unsigned int newest, struct meshtastic_text_inbox_entry *out)
{
	int ret = -ENOENT;

	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&inbox_lock, K_FOREVER);
	if (newest < kept) {
		*out = ring[(head + ENTRIES - 1U - newest) % ENTRIES];
		ret = 0;
	}
	k_mutex_unlock(&inbox_lock);
	return ret;
}

void meshtastic_text_inbox_clear(void)
{
	k_mutex_lock(&inbox_lock, K_FOREVER);
	kept = 0U;
	head = 0U;
	k_mutex_unlock(&inbox_lock);
}
