/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * Firmware updates from an Eclipse hawkBit server.
 *
 * One thread runs Zephyr's hawkBit client (DDI: poll, fetch the deployment, download into
 * slot 1, report). It wakes when the network gets an address, when asked from the shell, and
 * on the server's poll interval (floored at MESHTASTIC_HAWKBIT_MIN_POLL_SEC). It does not
 * probe:
 *   - without an address or before the server is configured;
 *   - while the running image is unconfirmed. This module confirms it after
 *     MESHTASTIC_HAWKBIT_CONFIRM_DELAY_SEC of uptime (the healthy window), and the client
 *     refuses to probe an unconfirmed image anyway; so after an update the server hears
 *     "success" only from an image that came up and stayed up.
 * An installed update reboots into the test slot.
 *
 * What the server learns of the device, besides the client's own: `build` (git describe),
 * `version` (the MCUboot image version with its build number), `class` and `board`.
 */

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/data/json.h>
#include <zephyr/mgmt/hawkbit/hawkbit.h>
#include <zephyr/mgmt/hawkbit/config.h>

#include "meshtastic_build.h"

LOG_MODULE_REGISTER(mt_hawkbit, CONFIG_MESHTASTIC_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(hb_stack, CONFIG_MESHTASTIC_HAWKBIT_STACK_SIZE);
static struct k_thread hb_thread;
static K_SEM_DEFINE(hb_kick, 0, 1);
static struct net_mgmt_event_callback ipv4_cb;
static atomic_t net_up;

static struct {
	uint32_t probes;
	uint32_t installed;
	enum hawkbit_response last;
	int64_t last_ms;          /* uptime of the last probe; -1 never */
	uint32_t next_s;
	const char *waiting;      /* why the last wake did not probe, or NULL */
} st = { .last_ms = -1 };

static const char *response_name(enum hawkbit_response r)
{
	switch (r) {
	case HAWKBIT_NO_RESPONSE: return "no response";
	case HAWKBIT_UPDATE_INSTALLED: return "update installed";
	case HAWKBIT_NO_UPDATE: return "no update";
	case HAWKBIT_NETWORKING_ERROR: return "networking error";
	case HAWKBIT_UNCONFIRMED_IMAGE: return "image unconfirmed";
	case HAWKBIT_PERMISSION_ERROR: return "permission error";
	case HAWKBIT_METADATA_ERROR: return "metadata error";
	case HAWKBIT_DOWNLOAD_ERROR: return "download error";
	case HAWKBIT_ALLOC_ERROR: return "allocation error";
	case HAWKBIT_NOT_INITIALIZED: return "not initialized";
	case HAWKBIT_PROBE_IN_PROGRESS: return "probe in progress";
	}
	return "?";
}

/* --- what the server is told about us ------------------------------------------------- */

struct hb_attr_data {
	const char *build;
	const char *version;
	const char *board;
	int32_t class;
};

struct hb_attrs {
	const char *mode;
	struct hb_attr_data data;
};

static const struct json_obj_descr hb_attr_data_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct hb_attr_data, build, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hb_attr_data, version, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hb_attr_data, board, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hb_attr_data, class, JSON_TOK_NUMBER),
};

static const struct json_obj_descr hb_attrs_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct hb_attrs, mode, JSON_TOK_STRING),
	JSON_OBJ_DESCR_OBJECT(struct hb_attrs, data, hb_attr_data_descr),
};

static int hb_attrs_cb(const char *device_id, uint8_t *buffer, const size_t buffer_size)
{
	static char version[32];
	struct meshtastic_image_version v;

	ARG_UNUSED(device_id);
	if (meshtastic_image_version(&v) == 0) {
		snprintf(version, sizeof(version), "%u.%u.%u+%u", v.major, v.minor, v.revision,
			 v.build);
	} else {
		snprintf(version, sizeof(version), "unknown");
	}
	struct hb_attrs a = {
		.mode = "merge",
		.data = {
			.build = meshtastic_build_id(),
			.version = version,
			.board = CONFIG_BOARD,
			.class = CONFIG_MESHTASTIC_FLEET_CLASS,
		},
	};

	return json_obj_encode_buf(hb_attrs_descr, ARRAY_SIZE(hb_attrs_descr), &a, buffer,
				   buffer_size);
}

/* --- the thread ------------------------------------------------------------------------ */

static void confirm_if_healthy(void)
{
#if CONFIG_MESHTASTIC_HAWKBIT_CONFIRM_DELAY_SEC > 0
	if (!boot_is_img_confirmed() &&
	    k_uptime_get() >= (int64_t)CONFIG_MESHTASTIC_HAWKBIT_CONFIRM_DELAY_SEC * 1000) {
		int rc = boot_write_img_confirmed();

		if (rc == 0) {
			LOG_INF("hawkBit: image confirmed after %d s healthy uptime",
				CONFIG_MESHTASTIC_HAWKBIT_CONFIRM_DELAY_SEC);
		} else {
			LOG_ERR("hawkBit: confirming the image failed (%d)", rc);
		}
	}
#endif
}

static bool server_configured(void)
{
	const char *addr = hawkbit_get_server_addr();

	return addr != NULL && addr[0] != '\0';
}

static uint32_t probe_once(void)
{
	enum hawkbit_response r = hawkbit_probe();

	st.probes++;
	st.last = r;
	st.last_ms = k_uptime_get();
	LOG_INF("hawkBit probe: %s", response_name(r));
	if (r == HAWKBIT_UPDATE_INSTALLED) {
		st.installed++;
		LOG_INF("hawkBit: update in slot 1, rebooting into it");
		hawkbit_reboot();   /* does not return */
	}
	uint32_t next = hawkbit_get_poll_interval();

	return MAX(next, (uint32_t)CONFIG_MESHTASTIC_HAWKBIT_MIN_POLL_SEC);
}

static void hb_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint32_t next = CONFIG_MESHTASTIC_HAWKBIT_MIN_POLL_SEC;
	int rc;

	rc = hawkbit_set_custom_data_cb(hb_attrs_cb);
	if (rc < 0) {
		LOG_WRN("hawkBit: attributes callback not set (%d)", rc);
	}
	rc = hawkbit_init();
	if (rc < 0) {
		LOG_ERR("hawkBit: init failed (%d); not polling", rc);
		return;
	}

	for (;;) {
		st.next_s = next;
		(void)k_sem_take(&hb_kick, K_SECONDS(next));
		confirm_if_healthy();
		next = 30;
		if (!atomic_get(&net_up)) {
			st.waiting = "no network address";
			continue;
		}
		if (!server_configured()) {
			st.waiting = "no server configured (hawkbit server <addr>)";
			next = CONFIG_MESHTASTIC_HAWKBIT_MIN_POLL_SEC;
			continue;
		}
		if (!boot_is_img_confirmed()) {
			st.waiting = "running image not confirmed yet";
			next = 10;
			continue;
		}
		st.waiting = NULL;
		next = probe_once();
	}
}

static void ipv4_event(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
		       struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);
	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		atomic_set(&net_up, 1);
		k_sem_give(&hb_kick);
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_DEL) {
		atomic_set(&net_up, 0);
	}
}

static int meshtastic_hawkbit_init(void)
{
	net_mgmt_init_event_callback(&ipv4_cb, ipv4_event,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&ipv4_cb);
	k_thread_create(&hb_thread, hb_stack, K_THREAD_STACK_SIZEOF(hb_stack), hb_thread_fn,
			NULL, NULL, NULL, CONFIG_MESHTASTIC_HAWKBIT_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&hb_thread, "mt_hawkbit");
	return 0;
}

SYS_INIT(meshtastic_hawkbit_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* --- shell --------------------------------------------------------------------------------- */

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const char *addr = hawkbit_get_server_addr();

	shell_print(sh, "server:     %s:%u", (addr && addr[0]) ? addr : "(none)",
		    hawkbit_get_server_port());
	const char *tok = hawkbit_get_ddi_security_token();

	shell_print(sh, "token:      %s", (tok && tok[0]) ? "set" : "(none)");
	shell_print(sh, "network:    %s", atomic_get(&net_up) ? "up" : "down");
	shell_print(sh, "image:      %s", boot_is_img_confirmed() ? "confirmed" : "UNCONFIRMED");
	shell_print(sh, "action id:  %d", hawkbit_get_action_id());
	shell_print(sh, "probes:     %u (installed %u)", st.probes, st.installed);
	if (st.last_ms >= 0) {
		shell_print(sh, "last probe: %s, %lld s ago", response_name(st.last),
			    (k_uptime_get() - st.last_ms) / 1000);
	}
	shell_print(sh, "next wake:  <= %u s%s%s", st.next_s, st.waiting ? ", waiting: " : "",
		    st.waiting ? st.waiting : "");
	return 0;
}

static int cmd_probe(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_sem_give(&hb_kick);
	shell_print(sh, "probe requested (see `hawkbit status`)");
	return 0;
}

static int cmd_server(const struct shell *sh, size_t argc, char **argv)
{
	int rc = hawkbit_set_server_addr(argv[1]);

	if (rc == 0 && argc > 2) {
		rc = hawkbit_set_server_port((uint16_t)strtoul(argv[2], NULL, 10));
	}
	if (rc) {
		shell_error(sh, "not set (%d)%s", rc, rc == -EAGAIN ? ": a probe is running" : "");
		return rc;
	}
	shell_print(sh, "server set; saved in settings");
	return 0;
}

static int cmd_token(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	int rc = hawkbit_set_ddi_security_token(argv[1]);

	if (rc) {
		shell_error(sh, "not set (%d)", rc);
		return rc;
	}
	shell_print(sh, "token set; saved in settings");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(hawkbit_cmds,
	SHELL_CMD(status, NULL, "Server, image, last probe", cmd_status),
	SHELL_CMD(probe, NULL, "Probe the server now", cmd_probe),
	SHELL_CMD_ARG(server, NULL, "<address> [port]: the server, kept in settings",
		      cmd_server, 2, 1),
	SHELL_CMD_ARG(token, NULL, "<token>: this target's DDI token, kept in settings",
		      cmd_token, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(hawkbit, &hawkbit_cmds, "Firmware updates from a hawkBit server", NULL);
