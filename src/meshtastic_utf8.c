/* SPDX-License-Identifier: GPL-3.0 */

/* The reference's sanitizeUtf8 (meshUtils.cpp), ported: replace every byte that does not begin
 * a valid UTF-8 sequence -- a stray continuation byte, a sequence cut short by truncation, an
 * overlong form, a surrogate half, anything past U+10FFFF -- with '?'. A name truncated to fit
 * a buffer can end mid-character, and a phone-side protobuf decoder that validates UTF-8 would
 * then reject the whole frame carrying it. Shared by the NodeDB (names) and the cross-preset
 * relay (text it re-originates). tests/utf8 carries the reference's own test vectors. */

#include <stdint.h>
#include <string.h>

#include "meshtastic_utf8.h"

bool meshtastic_utf8_sanitize(char *buf, size_t size)
{
	size_t i = 0U;
	size_t len;
	bool replaced;

	if (buf == NULL || size == 0U) {
		return false;
	}
	replaced = (buf[size - 1U] != '\0');
	buf[size - 1U] = '\0';
	len = strlen(buf);

	while (i < len) {
		uint8_t b = (uint8_t)buf[i];
		size_t seq;
		uint32_t min_cp;
		uint32_t cp;
		bool valid = true;

		if (b <= 0x7FU) {
			i++;
			continue;
		} else if ((b & 0xE0U) == 0xC0U) {
			seq = 2U;
			min_cp = 0x80U;
			cp = b & 0x1FU;
		} else if ((b & 0xF0U) == 0xE0U) {
			seq = 3U;
			min_cp = 0x800U;
			cp = b & 0x0FU;
		} else if ((b & 0xF8U) == 0xF0U) {
			seq = 4U;
			min_cp = 0x10000U;
			cp = b & 0x07U;
		} else {
			buf[i++] = '?';
			replaced = true;
			continue;
		}

		if (i + seq > len) {
			for (size_t j = i; j < len; j++) {
				buf[j] = '?';
			}
			replaced = true;
			break;
		}
		for (size_t j = 1U; j < seq; j++) {
			uint8_t c = (uint8_t)buf[i + j];

			if ((c & 0xC0U) != 0x80U) {
				valid = false;
				break;
			}
			cp = (cp << 6) | (c & 0x3FU);
		}
		if (valid && (cp < min_cp || cp > 0x10FFFFU || (cp >= 0xD800U && cp <= 0xDFFFU))) {
			valid = false;
		}
		if (valid) {
			i += seq;
		} else {
			/* Only the lead byte; its continuations are caught on the next pass. */
			buf[i++] = '?';
			replaced = true;
		}
	}
	return replaced;
}
