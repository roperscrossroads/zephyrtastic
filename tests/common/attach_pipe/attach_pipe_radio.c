/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The radio side of an image in the pipe harness: the hub's RF played into the
 * sim radio at the tuning the hub chose ("rf heard" / "rf lost" with this
 * image's clock), and everything the radio transmits reported back ("tx" with
 * start and airtime, "txhex" with the bytes). Shared by every harness app.
 */

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <meshtastic/lora_sim.h>
#include "meshtastic_packet.h"

#include "attach_pipe.h"
#include "attach_pipe_radio.h"

static const struct device *lora_dev;

void attach_pipe_radio_init(const struct device *dev)
{
	lora_dev = dev;
}

/* A frame on the air, at the tuning the hub worked out for its preset. The sim
 * radio delivers only on an exact frequency/SF/bandwidth match, so an image
 * tuned anywhere else -- a wrong slot, a wrong modem -- does not hear it, as on
 * the bench (lora_sim compares the bandwidth as a uint8_t, hence the cast). */
void attach_pipe_radio_on_rf(const struct attach_pipe_rf *rf, const uint8_t *wire, size_t len)
{
	int rc = lora_sim_inject_on(lora_dev, rf->freq_hz, rf->sf,
				    (uint8_t)(enum lora_signal_bandwidth)rf->bw_khz, wire,
				    (uint8_t)len, rf->rssi, rf->snr);

	/* A radio that is not listening -- keyed up (a head transmitting what
	 * its brain handed it, P3), or tuned elsewhere -- does not hear the
	 * frame, as on the bench. Say so, so a test can account for it. */
	if (rc != 0) {
		attach_pipe_event("rf lost rc=%d t=%lld", rc, k_uptime_get());
	} else {
		/* When this radio heard it, on this image's clock: the start of a
		 * relay window (X6). */
		attach_pipe_event("rf heard t=%lld", k_uptime_get());
	}
}

/* Everything this image's OWN radio transmits, with its start and airtime on
 * this image's clock: a test can say "not relayed here" and mean it, and can
 * check that a frame the radio missed ("rf lost") fell while it was keyed up. */
static void tx_watch_fn(void *a, void *b, void *c)
{
	struct lora_sim_frame f;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		if (lora_sim_take_tx(lora_dev, &f, K_MSEC(200)) != 0 || f.len < MESHTASTIC_HDR_LEN) {
			continue;
		}
		const struct meshtastic_wire_header *h = (const struct meshtastic_wire_header *)f.data;

		attach_pipe_event("tx src=%08x dest=%08x id=%08x hops=%u ch=%02x len=%u t=%lld air=%u",
				  sys_le32_to_cpu(h->src), sys_le32_to_cpu(h->dest),
				  sys_le32_to_cpu(h->id), h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK,
				  h->channel, f.len, f.t_ms, f.air_ms);
		/* The frame itself (up to 80 bytes, what an EVENT line holds), so a
		 * test can decrypt a reply and check what it answers. */
		char hex[2U * 80U + 1U];
		size_t n = MIN((size_t)f.len, (size_t)80U);

		for (size_t i = 0U; i < n; i++) {
			snprintf(&hex[2U * i], 3, "%02x", f.data[i]);
		}
		hex[2U * n] = '\0';
		attach_pipe_event("txhex id=%08x hex=%s", sys_le32_to_cpu(h->id), hex);
	}
}
K_THREAD_STACK_DEFINE(tx_watch_stack, 2048);
static struct k_thread tx_watch;

void attach_pipe_radio_tx_watch_start(void)
{
	k_thread_create(&tx_watch, tx_watch_stack, K_THREAD_STACK_SIZEOF(tx_watch_stack),
			tx_watch_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
}

