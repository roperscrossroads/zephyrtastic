/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

int meshtastic_outbound_init(void);

/*
 * Queue a wire frame for transmission.  Returns 0 when queued, -ENOMSG if the
 * queue is full.  Does not block on the radio driver.  The NORMAL-tier default;
 * use the _prio variant to classify.
 */
int meshtastic_radio_send_wire(uint8_t *pkt, uint32_t pkt_len);

/*
 * Queue a wire frame and block until the outbound worker completes the
 * transmission or @p timeout expires (-EAGAIN).  NORMAL-tier default.
 */
int meshtastic_radio_send_wire_wait(const uint8_t *pkt, uint32_t pkt_len, k_timeout_t timeout);

/*
 * Priority-aware variants.  @p tier is a meshtastic_sched_tier value governing
 * ordering and overflow protection.  Fire-and-forget (_prio) frames may be
 * dropped per the active scheduler overflow policy; the blocking (_wait_prio)
 * variant is never silently dropped — it waits for space up to @p timeout.
 */
int meshtastic_radio_send_wire_prio(uint8_t *pkt, uint32_t pkt_len, uint8_t tier);
int meshtastic_radio_send_wire_wait_prio(const uint8_t *pkt, uint32_t pkt_len, uint8_t tier,
					 k_timeout_t timeout);

/* Driver-level TX; only called from the outbound worker thread. */
int meshtastic_radio_send_wire_now(uint8_t *pkt, uint32_t pkt_len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_OUTBOUND_H_ */
