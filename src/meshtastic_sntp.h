/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * SNTP sub-second conversion. Split out of meshtastic_sntp.c so the arithmetic
 * is unit-testable without dragging in the network stack (the .c pulls
 * net_mgmt/socket headers and registers a SYS_INIT; this header is pure).
 */
#ifndef MESHTASTIC_SNTP_H_
#define MESHTASTIC_SNTP_H_

#include <stdint.h>

/**
 * @brief Sub-second part of an SNTP response, in milliseconds, corrected for
 *        one-way network path delay.
 *
 * @warning The units of @c sntp_time.fraction DEPEND ON A KCONFIG. From
 * `zephyr/subsys/net/lib/sntp/sntp.c`:
 *
 *  - @c CONFIG_SNTP_UNCERTAINTY=n (the default, and what this project builds):
 *    `res->fraction = ntohl(pkt->tx_tm_f)` — the raw NTP 32-bit binary fraction
 *    (units of 2^-32 s) lifted straight from the server's *transmit* timestamp,
 *    with **no** delay correction applied. The one-way path delay must be added
 *    here to get the time as of reception.
 *  - @c CONFIG_SNTP_UNCERTAINTY=y: `fraction` is **microseconds** and Zephyr has
 *    already folded in the computed clock offset. Adding @p rsp_delay_us again
 *    would double-count it.
 *
 * Getting this wrong is silent — the result stays a plausible-looking timestamp
 * that is merely off by up to a second — so the branch is written out explicitly
 * rather than assumed. If the Kconfig is ever enabled, this is the one place
 * that has to change.
 *
 * @param fraction     @c sntp_time.fraction, units per the branch above.
 * @param rsp_delay_us @c sntp_time.rsp_delay_us. Already the SINGLE-sided path
 *                     delay — `sntp.c` halves the round trip when computing it —
 *                     so it is added whole here, not halved again.
 *
 * @return Milliseconds to add to @c sntp_time.seconds. Range [0, ~1999]: the
 *         sub-second part is under 1000 ms and the path-delay correction can push
 *         the total past a second on a slow link, which the caller's int64
 *         millisecond arithmetic carries correctly.
 */
static inline int32_t meshtastic_sntp_subsecond_ms(uint32_t fraction, uint32_t rsp_delay_us)
{
#if defined(CONFIG_SNTP_UNCERTAINTY)
	(void)rsp_delay_us; /* not ARG_UNUSED: this header stays free of Zephyr includes */
	/* fraction is microseconds; the clock offset is already applied upstream. */
	return (int32_t)(fraction / 1000U);
#else
	/* ms = fraction * 1000 / 2^32. The 64-bit intermediate is required:
	 * 0xFFFFFFFF * 1000 overflows 32 bits by a factor of ~1000. */
	int32_t ms = (int32_t)(((uint64_t)fraction * 1000ULL) >> 32);

	return ms + (int32_t)(rsp_delay_us / 1000U);
#endif
}

#endif /* MESHTASTIC_SNTP_H_ */
