/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * A simulated board hears made-up peers: `simradio nodeinfo|text` builds a frame from a peer
 * as the stack would send it (primary channel, its key) and injects it into the simulated LoRa
 * driver, so it takes the real receive path (radio, router, NodeDB). For native_sim builds
 * (the sim radio, drivers/lora_sim), where boards in separate processes cannot hear each
 * other: what a gateway reports, a phone sees or a test bench observes can then be exercised
 * with no hardware.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <pb_encode.h>

#include <meshtastic/lora_sim.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"

static const struct device *const sim_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static int parse_num(const struct shell *sh, const char *s, uint32_t *out)
{
	char *end;
	unsigned long v = strtoul(s[0] == '!' ? s + 1 : s, &end, 16);

	if (*end != '\0' || v == 0 || v >= 0xffffffffUL) {
		shell_error(sh, "a node number in hex, e.g. a1b2c3d4 or !a1b2c3d4");
		return -EINVAL;
	}
	*out = (uint32_t)v;
	return 0;
}

static int inject(const struct shell *sh, uint32_t from, uint32_t port, const uint8_t *payload,
		  size_t len, int snr)
{
	struct meshtastic_packet packet = {
		.from = from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = sys_rand32_get() | 1U,
		.portnum = port,
		.payload = payload,
		.payload_len = len,
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len = 0;
	int rc = meshtastic_build_wire_packet(&packet, wire, &wire_len);

	if (rc != 0) {
		shell_error(sh, "frame not built (%d)", rc);
		return rc;
	}
	for (int i = 0; i < 500 && !lora_sim_rx_armed(sim_dev); i++) {
		k_msleep(2);
	}
	rc = lora_sim_inject(sim_dev, wire, (uint8_t)wire_len, -60, (int8_t)snr);
	if (rc != 0) {
		shell_error(sh, "inject refused (%d)", rc);
		return rc;
	}
	shell_print(sh, "heard !%08x: %u bytes on the primary channel, SNR %d", from, wire_len, snr);
	return 0;
}

static int cmd_nodeinfo(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t from;
	uint8_t buf[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_User user = meshtastic_User_init_zero;

	if (parse_num(sh, argv[1], &from) != 0) {
		return -EINVAL;
	}
	snprintk(user.id, sizeof(user.id), "!%08x", from);
	strncpy(user.long_name, argv[2], sizeof(user.long_name) - 1);
	strncpy(user.short_name, argv[3], sizeof(user.short_name) - 1);
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	if (!pb_encode(&os, meshtastic_User_fields, &user)) {
		return -ENOMEM;
	}
	return inject(sh, from, MESHTASTIC_PORT_NODEINFO, buf, os.bytes_written,
		      argc > 4 ? atoi(argv[4]) : 5);
}

static int cmd_text(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t from;

	if (parse_num(sh, argv[1], &from) != 0) {
		return -EINVAL;
	}
	return inject(sh, from, MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)argv[2],
		      strlen(argv[2]), argc > 3 ? atoi(argv[3]) : 5);
}

SHELL_STATIC_SUBCMD_SET_CREATE(simradio_cmds,
	SHELL_CMD_ARG(nodeinfo, NULL, "<from> <long name> <short name> [snr]: a peer announces itself",
		      cmd_nodeinfo, 4, 1),
	SHELL_CMD_ARG(text, NULL, "<from> <text> [snr]: a peer's broadcast text", cmd_text, 3, 1),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(simradio, &simradio_cmds, "Make the simulated radio hear a peer", NULL);
