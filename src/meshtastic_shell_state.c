/* SPDX-License-Identifier: GPL-3.0
 *
 * `meshtastic state`: what this node is and how it is set, in one machine-readable line.
 *
 * Every other shell command prints prose for a person. A tool that has to decide whether a
 * node already is what it should be needs facts it can parse without guessing, and needs to
 * know when it did NOT get them: a truncated read must not look like an answer. So:
 *
 *     ~S{...json...}*hhhh
 *
 * one line, a sentinel in front so a log line on the same console cannot be mistaken for it,
 * and a CRC-16 (XMODEM: poly 0x1021, init 0) of the JSON text behind it. Numbers are the
 * protobuf enum values (the reader has the names); a key is never printed, only a fingerprint.
 *
 * The line is built and sent a piece at a time. It is longer than any buffer this file would
 * want to own: the tightest image has a few hundred bytes of RAM to spare.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_build.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
#include "meshtastic_attachment.h"
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
#include "meshtastic_attachment_head.h"
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#endif
#if defined(CONFIG_MESHTASTIC_SCANNER)
#include "meshtastic_scanner.h"
#endif

#include "meshtastic_shell_state.h"

/* Bump when a field changes meaning. Adding a field does not need it. */
#define STATE_VERSION 1

struct st {
	const struct shell *sh;
	uint16_t crc;
};

static void st_raw(struct st *s, const char *text, size_t len)
{
	s->crc = crc16_itu_t(s->crc, (const uint8_t *)text, len);
	shell_fprintf(s->sh, SHELL_NORMAL, "%s", text);
}

static void st_out(struct st *s, const char *fmt, ...)
{
	char buf[80];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n < 0) {
		return;
	}
	st_raw(s, buf, MIN((size_t)n, sizeof(buf) - 1U));
}

/* A JSON string. Names come from a phone or an operator: quotes, backslashes and control
 * characters are escaped, UTF-8 passes through. */
static void st_str(struct st *s, const char *text)
{
	char buf[8];

	st_raw(s, "\"", 1U);
	for (const unsigned char *p = (const unsigned char *)(text ? text : ""); *p != '\0'; p++) {
		if (*p == '"' || *p == '\\') {
			buf[0] = '\\';
			buf[1] = (char)*p;
			buf[2] = '\0';
			st_raw(s, buf, 2U);
		} else if (*p < 0x20U) {
			snprintk(buf, sizeof(buf), "\\u%04x", *p);
			st_raw(s, buf, 6U);
		} else {
			buf[0] = (char)*p;
			buf[1] = '\0';
			st_raw(s, buf, 1U);
		}
	}
	st_raw(s, "\"", 1U);
}

/* What a key IS without saying what it is: "none", "s<N>" for the well-known key's one-byte
 * form, else "c:" and a CRC-32 over a label and the key. Enough to tell two keys apart and to
 * check a key the reader already holds; it gives away 32 bits of a 128- or 256-bit secret. */
static void st_key(struct st *s, const meshtastic_Channel *ch)
{
	static const uint8_t label[] = "zt-kfp-1";
	uint32_t c;

	if (ch->settings.psk.size == 0U) {
		st_out(s, "\"none\"");
	} else if (ch->settings.psk.size == 1U) {
		st_out(s, "\"s%u\"", (unsigned int)ch->settings.psk.bytes[0]);
	} else {
		c = crc32_ieee(label, sizeof(label) - 1U);
		c = crc32_ieee_update(c, ch->settings.psk.bytes, ch->settings.psk.size);
		st_out(s, "\"c:%08x\"", c);
	}
}

static void st_features(struct st *s)
{
	static const char *const feat[] = {
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
		"brain",
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
		"head",
#endif
#if defined(CONFIG_MESHTASTIC_SCANNER)
		"scanner",
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
		"cluster",
#endif
#if defined(CONFIG_MESHTASTIC_BLE_PEER)
		"ble_peer",
#endif
#if defined(CONFIG_MESHTASTIC_PHONEAPI)
		"phoneapi",
#endif
		NULL,
	};

	st_out(s, "\"feat\":[");
	for (size_t i = 0U; feat[i] != NULL; i++) {
		st_out(s, "%s\"%s\"", i == 0U ? "" : ",", feat[i]);
	}
	st_out(s, "]");
}

static void st_radio(struct st *s)
{
	meshtastic_Config cfg;

	if (meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg) == 0) {
		const meshtastic_Config_LoRaConfig *l = &cfg.payload_variant.lora;

		/* preset is what is STORED and applies at boot; live is what the radio is on
		 * now (a brain may have retuned a head, a scanner may be hopping). */
		st_out(s, ",\"lora\":{\"preset\":%d,\"use_preset\":%d,\"live\":%d,", (int)l->modem_preset,
		       l->use_preset ? 1 : 0, (int)mt.modem_preset);
		st_out(s, "\"region\":%d,\"pwr\":%d,\"tx\":%d}", (int)l->region, (int)l->tx_power,
		       l->tx_enabled ? 1 : 0);
	}
	if (meshtastic_config_store_get_config(meshtastic_Config_device_tag, &cfg) == 0) {
		st_out(s, ",\"dev\":{\"role\":%d,\"rb\":%d}", (int)cfg.payload_variant.device.role,
		       (int)cfg.payload_variant.device.rebroadcast_mode);
	}
}

static void st_channels(struct st *s)
{
	bool first = true;

	st_out(s, ",\"ch\":[");
	for (uint8_t i = 0U; i < MESHTASTIC_MAX_CHANNELS; i++) {
		const meshtastic_Channel *ch = meshtastic_channels_get(i);

		if (ch == NULL || ch->role == meshtastic_Channel_Role_DISABLED) {
			continue;
		}
		/* n is the STORED name: empty on a default channel, which takes its name from
		 * the preset. h is the hash that name and key give on the air. */
		st_out(s, "%s{\"i\":%u,\"r\":%d,\"n\":", first ? "" : ",", (unsigned int)i,
		       (int)ch->role);
		st_str(s, ch->settings.name);
		st_out(s, ",\"h\":%u,\"k\":", (unsigned int)meshtastic_channels_get_hash(i));
		st_key(s, ch);
		st_out(s, ",\"up\":%d,\"dn\":%d}", ch->settings.uplink_enabled ? 1 : 0,
		       ch->settings.downlink_enabled ? 1 : 0);
		first = false;
	}
	st_out(s, "]");
}

static void st_roles(struct st *s)
{
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)
	{
		struct meshtastic_attachment_info a;
		bool first = true;

		st_out(s, ",\"brain\":{\"max\":%u,\"heads\":[",
		       (unsigned int)CONFIG_MESHTASTIC_ATTACHMENT_MAX);
		for (uint8_t id = 1U; id <= CONFIG_MESHTASTIC_ATTACHMENT_MAX; id++) {
			if (!meshtastic_attachment_get(id, &a)) {
				continue;
			}
			st_out(s, "%s{\"a\":%u,\"node\":\"0x%08x\",\"p\":%d,\"up\":%d}",
			       first ? "" : ",", (unsigned int)a.id, a.node,
			       a.preset == MESHTASTIC_PRESET_UNKNOWN ? -1 : (int)a.preset,
			       a.link_up ? 1 : 0);
			first = false;
		}
		st_out(s, "]}");
	}
#endif
#if defined(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
	st_out(s, ",\"head\":{\"brain\":\"0x%08x\"}", meshtastic_attachment_head_get_brain());
#endif
#if defined(CONFIG_MESHTASTIC_SCANNER)
	{
		meshtastic_Config_LoRaConfig_ModemPreset list[MESHTASTIC_SCANNER_MAX_PRESETS];
		int n = meshtastic_scanner_get_presets(list, ARRAY_SIZE(list));

		st_out(s, ",\"scan\":{\"shut\":%d,\"sweep\":%d,\"presets\":[",
		       meshtastic_scanner_active() ? 1 : 0, meshtastic_scanner_sweeping() ? 1 : 0);
		for (int i = 0; i < n; i++) {
			st_out(s, "%s%d", i == 0 ? "" : ",", (int)list[i]);
		}
		st_out(s, "]}");
	}
#endif
#if defined(CONFIG_MESHTASTIC_CLUSTER)
	{
		uint8_t ch_index = 0U;
		bool have = meshtastic_cluster_channel_resolved(&ch_index);

		st_out(s, ",\"cluster\":{\"ch\":%d,\"n\":%u,\"hash\":\"%08x\"}",
		       have ? (int)ch_index : -1, (unsigned int)meshtastic_cluster_entry_count(),
		       (unsigned int)meshtastic_cluster_doc_hash_now());
	}
#endif
}

int meshtastic_shell_state(const struct shell *sh)
{
	struct st s = {.sh = sh, .crc = 0U};
	struct meshtastic_image_version v;

	shell_fprintf(sh, SHELL_NORMAL, "~S");
	st_out(&s, "{\"v\":%d,\"build\":", STATE_VERSION);
	st_str(&s, meshtastic_build_id());
	if (meshtastic_image_version(&v) == 0) {
		st_out(&s, ",\"image\":\"%u.%u.%u+%u\"", v.major, v.minor, v.revision, v.build);
	} else {
		st_out(&s, ",\"image\":null");
	}
	st_out(&s, ",\"class\":%u,\"board\":\"%s\",", (unsigned int)CONFIG_MESHTASTIC_FLEET_CLASS,
	       CONFIG_BOARD);
	st_features(&s);
	st_out(&s, ",\"id\":\"0x%08x\"", meshtastic_get_node_id());
	st_radio(&s);
	st_out(&s, ",\"own\":{\"l\":");
	st_str(&s, meshtastic_config_store_long_name());
	st_out(&s, ",\"s\":");
	st_str(&s, meshtastic_config_store_short_name());
	st_out(&s, "}");
	st_channels(&s);
	st_roles(&s);
#if defined(CONFIG_MESHTASTIC_ADMIN)
	st_out(&s, ",\"managed\":%d", meshtastic_admin_is_managed() ? 1 : 0);
#endif
	st_out(&s, "}");
	shell_fprintf(sh, SHELL_NORMAL, "*%04x\n", s.crc);
	return 0;
}
