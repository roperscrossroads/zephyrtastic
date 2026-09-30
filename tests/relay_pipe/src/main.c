/* SPDX-License-Identifier: GPL-3.0 */
/*
 * One image of the relay's two-image harness: the receiving half
 * (CONFIG_MESHTASTIC_RELAY) or the ear (CONFIG_MESHTASTIC_RELAY_EAR), joined
 * through the attachment pipe (tests/common/attach_pipe) the way the bench pair
 * is joined by the BLE peer link: the ear sends the half a bare wire frame
 * (kind WIRE), or an attachment envelope (kind ENV) to a peer that says it
 * takes them.
 *
 * Environment: ATTACH_PIPE_SOCK (the hub), ATTACH_PIPE_NODE (this image's node
 * number, hex), and for the ear ATTACH_PIPE_PEER (its receiving half, hex).
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/meshtastic/meshtastic.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"
#include "meshtastic_relay.h"

#include "attach_pipe.h"
#include "attach_pipe_radio.h"

#define PRESET_LF meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST
#define PRESET_MF meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static uint32_t env_hex(const char *name, uint32_t dflt)
{
	const char *v = attach_pipe_getenv(name, NULL);

	return (v != NULL) ? (uint32_t)strtoul(v, NULL, 16) : dflt;
}

/* A slot on the default key (shorthand 1); an empty name is the active
 * preset's default channel, as stock stores it. */
static void set_default_slot(uint8_t index, meshtastic_Channel_Role role, const char *name)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.index = index;
	ch.role = role;
	ch.has_settings = true;
	strncpy(ch.settings.name, name, sizeof(ch.settings.name) - 1U);
	ch.settings.psk.size = 1U;
	ch.settings.psk.bytes[0] = 0x01U;
	(void)meshtastic_channels_set_slot(index, &ch);
}

#if defined(CONFIG_MESHTASTIC_RELAY)

/* What the ear sent over the peer link: a frame that did not cross OUR air,
 * so the BLE_PEER bearer, exactly as the BLE glue injects it. */
static void on_wire(uint32_t peer, const uint8_t *wire, size_t len)
{
	int ret = meshtastic_radio_rx_inject(wire, (uint16_t)len, MESHTASTIC_BEARER_BLE_PEER);

	if (ret < 0) {
		attach_pipe_event("wire rx peer=%08x rc=%d", peer, ret);
	}
}

static void on_cmd(const char *cmd)
{
	if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_relay_stats s;

		meshtastic_relay_stats_get(&s);
		attach_pipe_event("relay considered=%u relayed=%u sent=%u not_broadcast=%u "
				  "not_text=%u prefixed=%u seen=%u origin_limited=%u "
				  "rate_dropped=%u no_mapping=%u",
				  s.considered, s.relayed, s.sent, s.not_broadcast, s.not_text,
				  s.prefixed, s.seen, s.origin_limited, s.rate_dropped, s.no_mapping);
	} else {
		attach_pipe_event("cmd unknown %s", cmd);
	}
}

static int role_start(uint32_t node)
{
	int ret;

	/* The bench's receiving half: MediumFast public, the public LongFast channel in the
	 * spare slot so the ear's frames decode, relaying inbound. */
	set_default_slot(0U, meshtastic_Channel_Role_PRIMARY, "");
	set_default_slot(1U, meshtastic_Channel_Role_SECONDARY, MESHTASTIC_CHANNEL_LONGFAST);
	ret = meshtastic_preset_switch(PRESET_MF, NULL);
	if (ret != 0) {
		return ret;
	}
	ret = meshtastic_relay_set_direction(MESHTASTIC_RELAY_INBOUND);
	if (ret != 0) {
		return ret;
	}
	attach_pipe_radio_tx_watch_start();
	attach_pipe_set_wire_cb(on_wire);
	ret = attach_pipe_start(ATTACH_PIPE_ROLE_RELAY, node, attach_pipe_radio_on_rf, on_cmd);
	attach_pipe_event("ready role=relay node=%08x preset=%u", node, mt.modem_preset);
	return ret;
}

#elif defined(CONFIG_MESHTASTIC_RELAY_EAR)

/* The ear's bare-frame seam, over the pipe instead of the BLE peer link. The
 * envelope seam needs no override: it goes through the bearer registry, where
 * the pipe is registered, and asks the pipe whether the peer takes envelopes. */
int meshtastic_relay_ear_send(uint32_t peer, const uint8_t *wire, size_t wire_len)
{
	return attach_pipe_send_wire(peer, wire, wire_len);
}

static void on_cmd(const char *cmd)
{
	if (strcmp(cmd, "stats") == 0) {
		struct meshtastic_relay_ear_stats s;

		meshtastic_relay_ear_stats_get(&s);
		attach_pipe_event("ear heard=%u forwarded=%u not_text=%u not_broadcast=%u "
				  "no_peer=%u queue_full=%u send_failed=%u rate_dropped=%u",
				  s.heard, s.forwarded, s.not_text, s.not_broadcast, s.no_peer,
				  s.queue_full, s.send_failed, s.rate_dropped);
	} else {
		attach_pipe_event("cmd unknown %s", cmd);
	}
}

static int role_start(uint32_t node)
{
	int ret;

	/* The bench's ear: LongFast, its primary named "LongFast". */
	set_default_slot(0U, meshtastic_Channel_Role_PRIMARY, MESHTASTIC_CHANNEL_LONGFAST);
	ret = meshtastic_preset_switch(PRESET_LF, NULL);
	if (ret != 0) {
		return ret;
	}
	meshtastic_relay_ear_set_peer(env_hex("ATTACH_PIPE_PEER", 0U));
	attach_pipe_radio_tx_watch_start();
	ret = attach_pipe_start(ATTACH_PIPE_ROLE_EAR, node, attach_pipe_radio_on_rf, on_cmd);
	attach_pipe_event("ready role=ear node=%08x preset=%u peer=%08x", node, mt.modem_preset,
			  meshtastic_relay_ear_get_peer());
	return ret;
}

#endif

int main(void)
{
	const uint32_t node = env_hex("ATTACH_PIPE_NODE", 0x0E0E0E0EU);
	static struct meshtastic_config cfg = {
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_US,
	};
	int ret;

	cfg.lora_dev = lora_dev;
	cfg.node_id = node;
	attach_pipe_radio_init(lora_dev);
	if (!device_is_ready(lora_dev)) {
		printk("relay harness: sim radio not ready\n");
		return 0;
	}
	ret = meshtastic_init(&cfg);
	if (ret != 0) {
		printk("relay harness: meshtastic_init %d\n", ret);
		return 0;
	}
	ret = role_start(node);
	if (ret != 0) {
		printk("relay harness: start %d\n", ret);
	}
	return 0;
}
