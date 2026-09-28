/* SPDX-License-Identifier: GPL-3.0 */
/*
 * One image of the two-image attachment harness: a real brain or a real keyless
 * head (by Kconfig), joined to the other through the attachment pipe
 * (tests/common/attach_pipe). The test driver (pytest/test_attach_pipe.py) is
 * the hub: it plays the RF each radio hears, carries envelopes between the
 * images with the latency, loss and trust it chooses, and reads what each
 * image reports.
 *
 * Environment: ATTACH_PIPE_SOCK (the hub), ATTACH_PIPE_NODE (this image's node
 * number, hex), and for a head ATTACH_PIPE_BRAIN (its brain, hex) and
 * ATTACH_PIPE_PRESET (the preset it listens on, a ModemPreset number).
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_preset.h"
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
#include "meshtastic_attachment.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
#include "meshtastic_attachment_head.h"
#endif

#include "attach_pipe.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static uint32_t env_hex(const char *name, uint32_t dflt)
{
	const char *v = attach_pipe_getenv(name, NULL);

	return (v != NULL) ? (uint32_t)strtoul(v, NULL, 16) : dflt;
}

/* A frame on the air, at the tuning the hub worked out for its preset. The sim
 * radio delivers only on an exact frequency/SF/bandwidth match, so an image
 * tuned anywhere else -- a wrong slot, a wrong modem -- does not hear it, as on
 * the bench (lora_sim compares the bandwidth as a uint8_t, hence the cast). */
static void on_rf(const struct attach_pipe_rf *rf, const uint8_t *wire, size_t len)
{
	(void)lora_sim_inject_on(lora_dev, rf->freq_hz, rf->sf,
				 (uint8_t)(enum lora_signal_bandwidth)rf->bw_khz, wire, (uint8_t)len,
				 rf->rssi, rf->snr);
}

#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)

static void on_recv(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t len, int16_t rssi, int8_t snr)
{
	char text[64];
	size_t n = MIN(len, sizeof(text) - 1U);

	memcpy(text, payload, n);
	text[n] = '\0';
	for (size_t i = 0U; i < n; i++) {
		if (text[i] < 0x20 || text[i] > 0x7e) {
			text[i] = '.';
		}
	}
	attach_pipe_event("rx from=%08x to=%08x port=%u rssi=%d snr=%d text=%s", from, to,
			  portnum, rssi, snr, (portnum == 1U) ? text : "");
}

/* Everything the brain's OWN radio transmits, so a test can say "not relayed
 * here" and mean it. */
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

		attach_pipe_event("tx src=%08x dest=%08x id=%08x hops=%u ch=%02x len=%u",
				  sys_le32_to_cpu(h->src), sys_le32_to_cpu(h->dest),
				  sys_le32_to_cpu(h->id), h->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK,
				  h->channel, f.len);
	}
}
K_THREAD_STACK_DEFINE(tx_watch_stack, 2048);
static struct k_thread tx_watch;

static void on_cmd(const char *cmd)
{
	if (strcmp(cmd, "attach") == 0) {
		struct meshtastic_attachment_info a;

		for (uint8_t id = 0U; id <= CONFIG_MESHTASTIC_ATTACHMENT_MAX; id++) {
			if (!meshtastic_attachment_get(id, &a)) {
				continue;
			}
			attach_pipe_event("attach id=%u node=%08x preset=%u rx=%u dropped=%u "
					  "rejected=%u rssi=%d snr=%d up=%d status=%d",
					  id, a.node, a.preset, a.rx_frames, a.rx_dropped, a.rejected,
					  a.last_rssi, a.last_snr, a.link_up, a.have_status);
		}
		attach_pipe_event("attach end count=%u", meshtastic_attachment_count());
	} else if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_attachment_stats s;

		meshtastic_attachment_stats_get(&s);
		attach_pipe_event("stats refused=%u malformed=%u evicted=%u self_heard=%u "
				  "last_rssi=%d last_snr=%d relayed=%u dup=%u",
				  s.admission_refused, s.not_admitted_malformed, s.evicted,
				  mt.status.self_heard, mt.status.last_rssi, mt.status.last_snr,
				  mt.status.relayed_packets, mt.status.duplicate_packets);
	} else if (strncmp(cmd, "preset ", 7) == 0) {
		unsigned long id = strtoul(&cmd[7], NULL, 10);
		const char *sp = strchr(&cmd[7], ' ');
		unsigned long p = (sp != NULL) ? strtoul(sp + 1, NULL, 10) : 0UL;

		attach_pipe_event("preset rc=%d",
				  meshtastic_attachment_set_preset((uint8_t)id, (uint8_t)p));
	} else if (strncmp(cmd, "allow ", 6) == 0) {
		attach_pipe_event("allow rc=%d", meshtastic_attachment_allow_add(
							  (uint32_t)strtoul(&cmd[6], NULL, 16)));
	} else {
		attach_pipe_event("cmd unknown %s", cmd);
	}
}

/* Slot 0 unnamed on the default PSK: its hash depends on the preset a frame
 * was heard on, which is what a head on another preset exercises (P1). */
static void set_default_primary(void)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.index = 0;
	ch.role = meshtastic_Channel_Role_PRIMARY;
	ch.has_settings = true;
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 0x01U;
	(void)meshtastic_channels_set_slot(0U, &ch);
}

static int role_start(uint32_t node)
{
	int ret;

	meshtastic_set_recv_cb(on_recv);
	set_default_primary();
	ret = meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO, NULL);
	if (ret != 0) {
		return ret;
	}
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	k_thread_create(&tx_watch, tx_watch_stack, K_THREAD_STACK_SIZEOF(tx_watch_stack),
			tx_watch_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
	ret = attach_pipe_start(ATTACH_PIPE_ROLE_BRAIN, node, on_rf, on_cmd);
	attach_pipe_event("ready role=brain node=%08x preset=%u", node, mt.modem_preset);
	return ret;
}

#elif defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)

static void on_cmd(const char *cmd)
{
	if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_attachment_head_stats s;

		meshtastic_attachment_head_stats_get(&s);
		attach_pipe_event("head heard=%u forwarded=%u no_brain=%u queue_full=%u "
				  "send_failed=%u status_sent=%u controls=%u refused=%u "
				  "untrusted=%u rejected=%u preset=%u",
				  s.heard, s.forwarded, s.no_brain, s.queue_full, s.send_failed,
				  s.status_sent, s.controls, s.refused, s.untrusted, s.rejected,
				  mt.modem_preset);
	} else {
		attach_pipe_event("cmd unknown %s", cmd);
	}
}

static int role_start(uint32_t node)
{
	const uint32_t preset = env_hex("ATTACH_PIPE_PRESET",
					meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST);
	int ret;

	ret = meshtastic_preset_switch((meshtastic_Config_LoRaConfig_ModemPreset)preset, NULL);
	if (ret != 0) {
		return ret;
	}
	ret = attach_pipe_start(ATTACH_PIPE_ROLE_HEAD, node, on_rf, on_cmd);
	meshtastic_attachment_head_set_brain(env_hex("ATTACH_PIPE_BRAIN", 0U));
	attach_pipe_event("ready role=head node=%08x preset=%u brain=%08x", node, mt.modem_preset,
			  meshtastic_attachment_head_get_brain());
	return ret;
}

#endif

int main(void)
{
	const uint32_t node = env_hex("ATTACH_PIPE_NODE", 0x0A0A0A0AU);
	static struct meshtastic_config cfg = {
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_US,
	};
	int ret;

	cfg.lora_dev = lora_dev;
	cfg.node_id = node;
	if (!device_is_ready(lora_dev)) {
		printk("attach harness: sim radio not ready\n");
		return 0;
	}
	ret = meshtastic_init(&cfg);
	if (ret != 0) {
		printk("attach harness: meshtastic_init %d\n", ret);
		return 0;
	}
	ret = role_start(node);
	if (ret != 0) {
		printk("attach harness: start %d\n", ret);
	}
	return 0;
}
