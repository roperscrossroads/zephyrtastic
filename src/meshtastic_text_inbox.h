/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_TEXT_INBOX_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_TEXT_INBOX_H_

/*
 * Text inbox: the last text messages delivered to this node, kept in RAM.
 *
 * The router's deliver_packet() hands every delivered packet here; a text
 * message (port 1) is copied into a ring with what is known of how it arrived.
 * The router has already dropped duplicates, so each message is kept once.
 * Readers (the shell) take a copy of one entry under the same lock the writer
 * holds, so a message arriving mid-read never tears an entry.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/meshtastic/meshtastic.h>

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_text_inbox_entry {
	/** Order of arrival since boot, from 1; never reused, also across a clear. */
	uint32_t seq;
	/** k_uptime_get() when it was kept. */
	int64_t uptime_ms;
	uint32_t from;
	uint32_t to;
	uint32_t id;
	int16_t rssi;
	int8_t snr;
	/** The channel table index it was decrypted with (struct meshtastic_packet). */
	uint8_t channel_index;
	/** 0 = this board's own radio, else the worker's id that heard it. */
	uint8_t rx_radio;
	/** 1 + the modem preset it was heard on; 0 when the bearer cannot say. */
	uint8_t rx_heard_on;
	/** hop_start - hop_limit: how many relays it took. */
	uint8_t hops;
	/** Decrypted with public-key crypto (a direct message to us). */
	bool pki;
	/** The text was longer than CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN and was cut. */
	bool truncated;
	/** Bytes in @c text, not counting its terminating NUL. */
	uint16_t len;
	char text[CONFIG_MESHTASTIC_TEXT_INBOX_TEXT_LEN + 1];
};

/** Keep @p packet if it is a text message; anything else is ignored. */
void meshtastic_text_inbox_record(const struct meshtastic_packet *packet);

/** How many messages are kept now (at most CONFIG_MESHTASTIC_TEXT_INBOX_ENTRIES). */
unsigned int meshtastic_text_inbox_count(void);

/** How many text messages were kept since boot, the cleared and the evicted included. */
uint32_t meshtastic_text_inbox_total(void);

/**
 * Copy the @p newest'th newest message (0 = the newest) into @p out.
 * @return 0, or -ENOENT when fewer are kept, or -EINVAL for a NULL @p out.
 */
int meshtastic_text_inbox_get(unsigned int newest, struct meshtastic_text_inbox_entry *out);

/** Forget every kept message. The arrival count (seq, total) goes on. */
void meshtastic_text_inbox_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_TEXT_INBOX_H_ */
