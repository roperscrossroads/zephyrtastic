/* SPDX-License-Identifier: GPL-3.0 */
#ifndef LINK_PIPE_RADIO_H_
#define LINK_PIPE_RADIO_H_

#include <stddef.h>
#include <zephyr/device.h>

#include "link_pipe.h"

/* The sim radio this image plays RF into and watches. */
void link_pipe_radio_init(const struct device *dev);
/* link_pipe_rf_cb: play a frame at the switchboard's tuning. */
void link_pipe_radio_on_rf(const struct link_pipe_rf *rf, const uint8_t *wire, size_t len);
/* Report every frame the radio transmits (tx / txhex EVENTs). */
void link_pipe_radio_tx_watch_start(void);

#endif /* LINK_PIPE_RADIO_H_ */
