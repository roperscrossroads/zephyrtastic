/* SPDX-License-Identifier: GPL-3.0
 *
 * A seal for records kept in retained RAM (ESP32 RTC memory, .noinit elsewhere).
 *
 * Retained RAM survives a warm reset, and also an image change: the next image finds the
 * previous image's bytes. Each image links its retained variables on its own, so after an
 * update the word where the new image keeps a record's magic can hold the old image's magic
 * (the same module, linked a few bytes away), while the bytes the new image reads as the
 * record are the old image's something else. A magic alone then validates garbage: a XIAO
 * updated from 4.4 to 4.5 reported "previous ran 8241 s", a thread named "96928\nmeshtastic
 * dfu\n" and a caller of 0x65746174, which is ASCII (agents-8h29).
 *
 * The seal is the magic, the record's layout version, its length and a checksum over the
 * body, the shape Zephyr's retention subsystem gives its areas (a prefix and a checksum). A
 * body another image left, or a record of another layout, fails; a record this layout wrote,
 * whichever image wrote it, passes and is real.
 *
 * Callers seal after writing the body, with the magic written last, and void the seal to
 * consume a one-shot record. Usable from an ISR or a fatal handler: no locks, no allocation,
 * and no call at all: the checksum (FNV-1a, 32 bit) is forced inline, because the ESP32's
 * hardware watchdog stage-0 handler runs from IRAM, where a call into flash (crc32_ieee) can
 * land while the flash cache is off.
 */
#ifndef MESHTASTIC_RETAINED_H_
#define MESHTASTIC_RETAINED_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/barrier.h>
#include <zephyr/toolchain.h>

struct meshtastic_retained_seal {
	uint32_t magic;
	uint16_t version;
	uint16_t len;
	uint32_t sum;
};

static ALWAYS_INLINE uint32_t meshtastic_retained_sum(const void *body, size_t len)
{
	const uint8_t *p = (const uint8_t *)body;
	uint32_t h = 2166136261U;

	for (size_t i = 0U; i < len; i++) {
		h = (h ^ p[i]) * 16777619U;
	}
	return h;
}

static ALWAYS_INLINE void meshtastic_retained_seal(struct meshtastic_retained_seal *seal,
						   uint32_t magic, uint16_t version,
						   const void *body, size_t len)
{
	seal->magic = 0U; /* invalid while it changes */
	barrier_dmem_fence_full();
	seal->version = version;
	seal->len = (uint16_t)len;
	seal->sum = meshtastic_retained_sum(body, len);
	barrier_dmem_fence_full(); /* every store above lands before the magic */
	seal->magic = magic;
}

static ALWAYS_INLINE bool meshtastic_retained_valid(const struct meshtastic_retained_seal *seal,
						    uint32_t magic, uint16_t version,
						    const void *body, size_t len)
{
	return seal->magic == magic && seal->version == version && seal->len == (uint16_t)len &&
	       seal->sum == meshtastic_retained_sum(body, len);
}

static ALWAYS_INLINE void meshtastic_retained_void(struct meshtastic_retained_seal *seal)
{
	seal->magic = 0U;
	barrier_dmem_fence_full(); /* void before the body is touched */
}

#endif /* MESHTASTIC_RETAINED_H_ */
