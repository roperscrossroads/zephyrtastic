/* SPDX-License-Identifier: GPL-3.0 */
/*
 * One image of the two-image link harness: a real hub or a real keyless
 * worker (by Kconfig), joined to the other through the link pipe
 * (tests/common/link_pipe). The test driver (pytest/test_link_pipe.py) is
 * the switchboard: it plays the RF each radio hears, carries envelopes between the
 * images with the latency, loss and trust it chooses, and reads what each
 * image reports.
 *
 * Environment: LINK_PIPE_SOCK (the switchboard), LINK_PIPE_NODE (this image's node
 * number, hex), and for a worker LINK_PIPE_HUB (its hub, hex) and
 * LINK_PIPE_PRESET (the preset it listens on, a ModemPreset number).
 */

#include <stdio.h>
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
#if defined(CONFIG_MESHTASTIC_HUB)
#include "meshtastic_hub.h"
#endif
#if defined(CONFIG_MESHTASTIC_WORKER)
#include "meshtastic_worker.h"
#endif

#include "link_pipe.h"
#include "link_pipe_radio.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static uint32_t env_hex(const char *name, uint32_t dflt)
{
	const char *v = link_pipe_getenv(name, NULL);

	return (v != NULL) ? (uint32_t)strtoul(v, NULL, 16) : dflt;
}

#if defined(CONFIG_MESHTASTIC_HUB)

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
	link_pipe_event("rx from=%08x to=%08x port=%u rssi=%d snr=%d text=%s", from, to,
			  portnum, rssi, snr, (portnum == 1U) ? text : "");
}


static void on_cmd(const char *cmd)
{
	if (strcmp(cmd, "link") == 0) {
		struct meshtastic_hub_info a;

		for (uint8_t id = 0U; id <= CONFIG_MESHTASTIC_HUB_WORKERS_MAX; id++) {
			if (!meshtastic_hub_get(id, &a)) {
				continue;
			}
			link_pipe_event("link id=%u node=%08x preset=%u rx=%u dropped=%u "
					  "rejected=%u rssi=%d snr=%d up=%d status=%d lag_n=%u "
					  "lag_p50=%u lag_p90=%u fast_worker=%d lag_warn=%d",
					  id, a.node, a.preset, a.rx_frames, a.rx_dropped, a.rejected,
					  a.last_rssi, a.last_snr, a.link_up, a.have_status, a.lag_n,
					  a.lag_p50_ms, a.lag_p90_ms, a.warn_fast_worker, a.warn_lag);
		}
		link_pipe_event("link end count=%u", meshtastic_hub_count());
	} else if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_hub_stats s;

		meshtastic_hub_stats_get(&s);
		link_pipe_event("stats refused=%u malformed=%u evicted=%u self_heard=%u "
				  "last_rssi=%d last_snr=%d relayed=%u dup=%u",
				  s.admission_refused, s.not_admitted_malformed, s.evicted,
				  mt.status.self_heard, mt.status.last_rssi, mt.status.last_snr,
				  mt.status.relayed_packets, mt.status.duplicate_packets);
	} else if (strncmp(cmd, "preset ", 7) == 0) {
		unsigned long id = strtoul(&cmd[7], NULL, 10);
		const char *sp = strchr(&cmd[7], ' ');
		unsigned long p = (sp != NULL) ? strtoul(sp + 1, NULL, 10) : 0UL;

		link_pipe_event("preset rc=%d",
				  meshtastic_hub_set_preset((uint8_t)id, (uint8_t)p));
	} else if (strncmp(cmd, "role ", 5) == 0) {
		meshtastic_set_device_role(
			(meshtastic_Config_DeviceConfig_Role)strtoul(&cmd[5], NULL, 10));
		link_pipe_event("role rc=0 role=%u", meshtastic_device_role());
	} else if (strncmp(cmd, "allow ", 6) == 0) {
		link_pipe_event("allow rc=%d", meshtastic_hub_allow_add(
							  (uint32_t)strtoul(&cmd[6], NULL, 16)));
	} else {
		link_pipe_event("cmd unknown %s", cmd);
	}
}

/* Slot 0 unnamed on the default PSK: its hash depends on the preset a frame
 * was heard on, which is what a worker on another preset exercises (P1). */
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
	/* ShortTurbo unless the test says otherwise (LINK_PIPE_PRESET). */
	ret = meshtastic_preset_switch(
		(meshtastic_Config_LoRaConfig_ModemPreset)env_hex(
			"LINK_PIPE_PRESET", meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO),
		NULL);
	if (ret != 0) {
		return ret;
	}
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	link_pipe_radio_tx_watch_start();
	ret = link_pipe_start(LINK_PIPE_ROLE_HUB, node, link_pipe_radio_on_rf, on_cmd);
	link_pipe_event("ready role=hub node=%08x preset=%u", node, mt.modem_preset);
	return ret;
}

#elif defined(CONFIG_MESHTASTIC_WORKER)

static void on_cmd(const char *cmd)
{
	if (strncmp(cmd, "busy ", 5) == 0) {
		/* Hold this radio's channel busy (LBT/CAD) for N ms: a peer on air. */
		lora_sim_set_busy(lora_dev, (uint32_t)strtoul(&cmd[5], NULL, 10));
		link_pipe_event("busy rc=0");
	} else if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_worker_stats s;

		meshtastic_worker_stats_get(&s);
		link_pipe_event("worker heard=%u forwarded=%u no_hub=%u queue_full=%u "
				  "send_failed=%u status_sent=%u controls=%u refused=%u "
				  "untrusted=%u rejected=%u preset=%u tx_sent=%u tx_deferred=%u "
				  "tx_cancelled=%u",
				  s.heard, s.forwarded, s.no_hub, s.queue_full, s.send_failed,
				  s.status_sent, s.controls, s.refused, s.untrusted, s.rejected,
				  mt.modem_preset, s.tx_sent, s.tx_deferred, s.tx_cancelled);
	} else {
		link_pipe_event("cmd unknown %s", cmd);
	}
}

static int role_start(uint32_t node)
{
	const uint32_t preset = env_hex("LINK_PIPE_PRESET",
					meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST);
	int ret;

	ret = meshtastic_preset_switch((meshtastic_Config_LoRaConfig_ModemPreset)preset, NULL);
	if (ret != 0) {
		return ret;
	}
	link_pipe_radio_tx_watch_start();
	ret = link_pipe_start(LINK_PIPE_ROLE_WORKER, node, link_pipe_radio_on_rf, on_cmd);
	meshtastic_worker_set_hub(env_hex("LINK_PIPE_HUB", 0U));
	link_pipe_event("ready role=worker node=%08x preset=%u hub=%08x", node, mt.modem_preset,
			  meshtastic_worker_get_hub());
	return ret;
}

#endif

int main(void)
{
	const uint32_t node = env_hex("LINK_PIPE_NODE", 0x0A0A0A0AU);
	static struct meshtastic_config cfg = {
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_US,
	};
	int ret;

	cfg.lora_dev = lora_dev;
	link_pipe_radio_init(lora_dev);
	cfg.node_id = node;
	if (!device_is_ready(lora_dev)) {
		printk("link harness: sim radio not ready\n");
		return 0;
	}
	ret = meshtastic_init(&cfg);
	if (ret != 0) {
		printk("link harness: meshtastic_init %d\n", ret);
		return 0;
	}
	ret = role_start(node);
	if (ret != 0) {
		printk("link harness: start %d\n", ret);
	}
	return 0;
}
