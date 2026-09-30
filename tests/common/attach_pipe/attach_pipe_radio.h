/* SPDX-License-Identifier: GPL-3.0 */
#ifndef ATTACH_PIPE_RADIO_H_
#define ATTACH_PIPE_RADIO_H_

#include <stddef.h>
#include <zephyr/device.h>

#include "attach_pipe.h"

/* The sim radio this image plays RF into and watches. */
void attach_pipe_radio_init(const struct device *dev);
/* attach_pipe_rf_cb: play a frame at the hub's tuning. */
void attach_pipe_radio_on_rf(const struct attach_pipe_rf *rf, const uint8_t *wire, size_t len);
/* Report every frame the radio transmits (tx / txhex EVENTs). */
void attach_pipe_radio_tx_watch_start(void);

#endif /* ATTACH_PIPE_RADIO_H_ */
