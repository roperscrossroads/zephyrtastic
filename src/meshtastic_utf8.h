/* SPDX-License-Identifier: GPL-3.0 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_UTF8_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_UTF8_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The reference's sanitizeUtf8() (meshUtils.cpp), ported byte for byte.
 *
 * Forces a NUL at buf[size - 1], treats the text as ending at its first NUL,
 * and replaces every byte that does not begin a valid UTF-8 sequence with '?':
 * a stray continuation byte, a sequence cut short, an overlong form, a surrogate
 * half, anything past U+10FFFF. Only the lead byte of a bad sequence is
 * replaced; its continuations are caught on the next pass. Control characters
 * (CR included) are left alone, as the reference leaves them.
 *
 * @return true if anything was replaced or the NUL had to be forced.
 */
bool meshtastic_utf8_sanitize(char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_UTF8_H_ */
