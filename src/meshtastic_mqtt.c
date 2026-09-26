/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic MQTT gateway bridge -- the core.
 *
 * Publishes and subscribes to ServiceEnvelope protobuf payloads on topics
 * compatible with official Meshtastic firmware (msh/.../2/e/...).
 *
 * Nothing here needs a network. The bridge reaches the broker through one of two
 * back ends, picked at init by ModuleConfig.mqtt.proxy_to_client_enabled:
 *   - the direct broker client, meshtastic_mqtt_broker.c (CONFIG_MESHTASTIC_MQTT_BROKER);
 *   - the client proxy (CONFIG_MESHTASTIC_MQTT_PROXY): each publish goes to the
 *     connected client as a FromRadio.mqttClientProxyMessage and each broker
 *     delivery comes back as a ToRadio one, the reference's MQTT.cpp proxy path.
 * Both drain the same publish queue, and both hand downlinks to the same decoder,
 * so a gateway behaves the same whichever carries its bytes (agents-kx8d).
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "meshtastic_ext_ram.h"

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_position.h"
#include "meshtastic_mqtt.h"
#include "meshtastic_mqtt_config.h"
#include "meshtastic_mqtt_internal.h"
#include "meshtastic_phoneapi.h"
#include "meshtastic_config_store.h"
#include "meshtastic_packet.h"
#include "meshtastic_router.h"
#include "meshtastic/config.pb.h"
#include "meshtastic/mqtt.pb.h"

LOG_MODULE_REGISTER(meshtastic_mqtt, CONFIG_MESHTASTIC_LOG_LEVEL);

#define CRYPT_TOPIC_SUFFIX             "/2/e/"
#define MAP_TOPIC_SUFFIX               "/2/map/"
#define MQTT_MAP_WARN_MS 15000

static struct {
	char crypt_prefix[64];
	char map_topic[80];
	/* ModuleConfig.mqtt resolved once at init (agents-dnr4.8). A set_module_config
	 * schedules a reboot; nothing re-reads this live. */
	struct meshtastic_mqtt_settings cfg;
	int64_t last_map_report_ms;
	int64_t last_map_no_position_ms;
	struct k_mutex lock;
	struct meshtastic_mqtt_pub_entry queue[CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE];
	uint8_t queue_head;
	uint8_t queue_tail;
	uint8_t queue_count;
	bool started;
	/* Publishing through the connected client, not a broker connection. */
	bool proxy;
	bool drop_warned; /* one WRN per queue-full episode, not per packet */
	/* EXT_RAM_BSS_ATTR must sit AFTER the declarator (below): placed after the anonymous
	 * struct's '}' it binds to the TYPE and is silently ignored (stays in internal DRAM). */
} mqtt_ctx MESHTASTIC_EXT_RAM_BSS_ATTR; /* the queue dominates -> PSRAM on V4 (no-op on V3) */

void meshtastic_mqtt_core_node_id_str(char *buf, size_t len)
{
	snprintk(buf, len, "!%08x", meshtastic_get_node_id());
}

static void mqtt_node_id_str(char *buf, size_t len)
{
	meshtastic_mqtt_core_node_id_str(buf, len);
}

static bool mqtt_is_default_broker(void)
{
	return mqtt_ctx.cfg.default_broker;
}

/* True when the configured broker is a literal RFC1918 / loopback address.
 *
 * Mirrors the reference's isMqttServerAddressPrivate, which is
 * `ip.fromString(host) && isPrivateIpAddress(ip)` — note it requires the host to
 * PARSE as an IP. A hostname is deliberately not treated as private even if it
 * happens to resolve to one: the check runs on configuration, not on DNS, so a
 * name that resolves differently later cannot silently downgrade the gate.
 */
static bool mqtt_parse_ipv4_literal(const char *s, uint32_t *host_order)
{
	/* Four decimal octets and nothing else. Parsed here rather than with
	 * net_addr_pton() because the core must build with no network stack (the
	 * client proxy); a hostname -- anything that is not exactly this -- fails. */
	uint32_t addr = 0U;

	for (int octet = 0; octet < 4; octet++) {
		uint32_t v = 0U;
		int digits = 0;

		while (*s >= '0' && *s <= '9') {
			v = v * 10U + (uint32_t)(*s - '0');
			if (++digits > 3 || v > 255U) {
				return false;
			}
			s++;
		}
		if (digits == 0 || *s != (octet < 3 ? '.' : '\0')) {
			return false;
		}
		s++;
		addr = (addr << 8) | v;
	}

	*host_order = addr;
	return true;
}

static bool mqtt_broker_is_private(void)
{
	uint32_t host_order;

	if (!mqtt_parse_ipv4_literal(mqtt_ctx.cfg.host, &host_order)) {
		return false; /* not a literal IPv4 address — treat as public */
	}

	/* 10.0.0.0/8, 172.16.0.0/12, 192.168.0.0/16, 127.0.0.0/8 */
	return ((host_order & 0xFF000000U) == 0x0A000000U) ||
	       ((host_order & 0xFFF00000U) == 0xAC100000U) ||
	       ((host_order & 0xFFFF0000U) == 0xC0A80000U) ||
	       ((host_order & 0xFF000000U) == 0x7F000000U);
}

static bool mqtt_should_skip_portnum(uint32_t portnum, bool from_us)
{
	if (from_us || !mqtt_is_default_broker()) {
		return false;
	}

	return portnum == meshtastic_PortNum_RANGE_TEST_APP ||
	       portnum == meshtastic_PortNum_DETECTION_SENSOR_APP;
}

static int mqtt_build_publish_topic(char *topic, size_t topic_len)
{
	char gateway_id[12];
	const char *channel = meshtastic_runtime_channel_name();

	if (channel == NULL) {
		return -EINVAL;
	}

	mqtt_node_id_str(gateway_id, sizeof(gateway_id));

	return snprintk(topic, topic_len, "%s%s%s/%s", mqtt_ctx.crypt_prefix, CRYPT_TOPIC_SUFFIX,
			channel, gateway_id);
}

static int mqtt_build_map_topic(char *topic, size_t topic_len)
{
	return snprintk(topic, topic_len, "%s%s", mqtt_ctx.crypt_prefix, MAP_TOPIC_SUFFIX);
}

static int mqtt_build_subscribe_topic(char *topic, size_t topic_len)
{
	const char *channel = meshtastic_runtime_channel_name();

	if (channel == NULL) {
		return -EINVAL;
	}

	return snprintk(topic, topic_len, "%s%s%s/+", mqtt_ctx.crypt_prefix, CRYPT_TOPIC_SUFFIX,
			channel);
}

static int mqtt_mesh_from_wire(const uint8_t *wire, size_t wire_len, meshtastic_MeshPacket *mesh)
{
	const struct __packed mt_wire_hdr {
		uint32_t dest;
		uint32_t src;
		uint32_t id;
		uint8_t flags;
		uint8_t channel;
		uint8_t next_hop;
		uint8_t relay_node;
	} *hdr;

	if (wire_len < sizeof(*hdr)) {
		return -EINVAL;
	}

	hdr = (const void *)wire;

	*mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
	mesh->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	mesh->from = sys_le32_to_cpu(hdr->src);
	mesh->to = sys_le32_to_cpu(hdr->dest);
	mesh->id = sys_le32_to_cpu(hdr->id);
	mesh->hop_limit = hdr->flags & 0x07U;
	mesh->hop_start = (hdr->flags >> 5) & 0x07U;
	mesh->want_ack = (hdr->flags & BIT(3)) != 0U;
	mesh->via_mqtt = (hdr->flags & BIT(4)) != 0U;
	mesh->channel = hdr->channel;
	mesh->next_hop = hdr->next_hop;
	mesh->relay_node = hdr->relay_node;
	mesh->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;

	mesh->encrypted.size = (pb_size_t)(wire_len - sizeof(*hdr));
	if (mesh->encrypted.size > sizeof(mesh->encrypted.bytes)) {
		return -ENOMEM;
	}

	memcpy(mesh->encrypted.bytes, wire + sizeof(*hdr), mesh->encrypted.size);

	return 0;
}

static int mqtt_encode_envelope(const meshtastic_MeshPacket *mesh, const char *channel_id,
				const char *gateway_id, uint8_t *out, size_t out_len,
				size_t *encoded_len)
{
	meshtastic_ServiceEnvelope env = meshtastic_ServiceEnvelope_init_zero;
	pb_ostream_t stream;

	env.packet = (meshtastic_MeshPacket *)mesh;
	env.channel_id = (char *)channel_id;
	env.gateway_id = (char *)gateway_id;

	stream = pb_ostream_from_buffer(out, out_len);
	if (!pb_encode(&stream, meshtastic_ServiceEnvelope_fields, &env)) {
		LOG_ERR("ServiceEnvelope encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	*encoded_len = stream.bytes_written;
	return 0;
}

/* Drop the oldest queued publish to make room (caller holds mqtt_ctx.lock).
 * Warn once per queue-full episode, not per packet: with the broker
 * unreachable every heard packet lands here, and a per-drop WRN floods the
 * log (and its syslog uplink). Upstream drops oldest silently (MQTT.cpp
 * MAX_MQTT_QUEUE); we keep one WRN per episode plus a DBG per drop. */
static void mqtt_kick(void);

static void mqtt_drop_oldest_locked(void)
{
	if (!mqtt_ctx.drop_warned) {
		LOG_WRN("MQTT publish queue full — dropping oldest until the broker is back");
		mqtt_ctx.drop_warned = true;
	} else {
		LOG_DBG("MQTT publish queue full, dropping oldest");
	}
	mqtt_ctx.queue_head = (mqtt_ctx.queue_head + 1U) % CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE;
	mqtt_ctx.queue_count--;
}

#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)
/* Only the map report defers a ready-made publish this way (the uplink path has
 * its own encode queue), so this rides the same compile gate. Left unfenced it is
 * defined-but-unused with MAP_REPORT=n — which is the default, and which twister's
 * -Werror refuses. The first MQTT build the variants sweep ever did found it. */
static void mqtt_queue_publish(const char *topic, const uint8_t *payload, size_t len)
{
	struct meshtastic_mqtt_pub_entry *entry;

	if (len > sizeof(entry->payload)) {
		LOG_WRN("MQTT publish too large (%zu), dropped", len);
		return;
	}

	k_mutex_lock(&mqtt_ctx.lock, K_FOREVER);

	if (mqtt_ctx.queue_count >= CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE) {
		mqtt_drop_oldest_locked();
	}

	/* Straight into the slot: an entry is a whole envelope, too big for a stack
	 * copy on the workqueue the proxy's map report runs on. */
	entry = &mqtt_ctx.queue[mqtt_ctx.queue_tail];
	strncpy(entry->topic, topic, sizeof(entry->topic) - 1U);
	entry->topic[sizeof(entry->topic) - 1U] = '\0';
	memcpy(entry->payload, payload, len);
	entry->len = (uint16_t)len;
	mqtt_ctx.queue_tail =
		(mqtt_ctx.queue_tail + 1U) % CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE;
	mqtt_ctx.queue_count++;

	LOG_DBG("MQTT publish queued: %s (%u bytes, depth %u)", entry->topic, entry->len,
		mqtt_ctx.queue_count);

	k_mutex_unlock(&mqtt_ctx.lock);
	mqtt_kick();
}
#endif /* CONFIG_MESHTASTIC_MQTT_MAP_REPORT */

#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)

static meshtastic_Config_LoRaConfig_RegionCode mqtt_lora_region(void)
{
	uint32_t hz = meshtastic_runtime_frequency();

	if (hz >= 902000000U && hz <= 928000000U) {
		return meshtastic_Config_LoRaConfig_RegionCode_US;
	}
	if (hz >= 869000000U && hz <= 870000000U) {
		return meshtastic_Config_LoRaConfig_RegionCode_EU_868;
	}
	if (hz >= 433000000U && hz <= 434000000U) {
		return meshtastic_Config_LoRaConfig_RegionCode_EU_433;
	}

	return meshtastic_Config_LoRaConfig_RegionCode_UNSET;
}

/* Reference channels.isDefaultChannel(primary). Used to compare the name to the
 * literal "LongFast"; the reference compares to the active PRESET's name, which
 * is what an unnamed channel is actually called on the air -- now shared with
 * NeighborInfo's gate in meshtastic_channels_is_default(). */
static bool mqtt_has_default_channel(void)
{
	return meshtastic_channels_is_default(meshtastic_channels_primary_index());
}

static uint32_t mqtt_map_position_precision(void)
{
	uint32_t precision = mqtt_ctx.cfg.map_position_precision;

	if (precision < 12U || precision > 15U) {
		precision = 14U;
	}

	return precision;
}

static void mqtt_apply_map_position_precision(int32_t *latitude_i, int32_t *longitude_i,
					      uint32_t precision)
{
	/* Shared with the mesh position path (G-1) so the truncation math has a
	 * single source of truth. Map-report precision is always clamped to 12..15,
	 * so the helper's 0/>=32 short-circuits never trigger here. */
	meshtastic_position_truncate_latlon(latitude_i, longitude_i, precision);
}

static void mqtt_build_map_report(meshtastic_MapReport *report, const meshtastic_Position *pos)
{
	const char *long_name = meshtastic_long_name();
	const char *short_name = meshtastic_short_name();
	uint32_t precision = mqtt_map_position_precision();

	*report = (meshtastic_MapReport)meshtastic_MapReport_init_zero;

	if (long_name != NULL) {
		strncpy(report->long_name, long_name, sizeof(report->long_name) - 1U);
	}
	if (short_name != NULL) {
		strncpy(report->short_name, short_name, sizeof(report->short_name) - 1U);
	}

	report->role = meshtastic_device_role();
	report->hw_model = meshtastic_hw_model();
	strncpy(report->firmware_version, MESHTASTIC_FIRMWARE_VERSION,
		sizeof(report->firmware_version) - 1U);
	report->region = mqtt_lora_region();
	report->modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
	report->has_default_channel = mqtt_has_default_channel();
	report->has_opted_report_location = true;
	report->position_precision = precision;
	report->num_online_local_nodes = 0U;

	if (pos != NULL && pos->has_latitude_i && pos->has_longitude_i) {
		report->latitude_i = pos->latitude_i;
		report->longitude_i = pos->longitude_i;
		mqtt_apply_map_position_precision(&report->latitude_i, &report->longitude_i,
						  precision);
		if (pos->has_altitude) {
			report->altitude = pos->altitude;
		}
	}
}

/* True while a publish would reach a broker: the direct session is up, or the
 * bytes go through the client proxy -- which, like the reference, is taken as
 * always up (the PhoneAPI queue holds a publish until a client reads it). */
static bool mqtt_link_up(void)
{
	if (mqtt_ctx.proxy) {
		return true;
	}
#if defined(CONFIG_MESHTASTIC_MQTT_BROKER)
	return meshtastic_mqtt_broker_connected();
#else
	return false;
#endif
}

static void mqtt_perhaps_report_to_map(void)
{
	/* Static, not stack: an envelope buffer plus a MeshPacket is ~1.5 KB, and in
	 * proxy mode this runs on the system workqueue. Only one back end is ever
	 * live, so there is only ever one caller. */
	static meshtastic_Position position;
	static meshtastic_MapReport map_report;
	static meshtastic_MeshPacket mesh;
	static uint8_t env_buf[CONFIG_MESHTASTIC_MQTT_TX_BUFFER_SIZE];
	meshtastic_Data *decoded;
	char gateway_id[12];
	const char *channel_id = meshtastic_runtime_channel_name();
	size_t env_len = 0U;
	int64_t now = k_uptime_get();
	int64_t interval_ms = (int64_t)mqtt_ctx.cfg.map_publish_interval_secs * MSEC_PER_SEC;

	/* Reference (MQTT.cpp perhapsReportToMap): both the module flag and the
	 * location opt-in gate the whole report, not just the coordinates. */
	if (!mqtt_ctx.cfg.map_reporting_enabled || !mqtt_ctx.cfg.map_should_report_location ||
	    !mqtt_link_up() || channel_id == NULL) {
		return;
	}

	position = (meshtastic_Position)meshtastic_Position_init_zero;
	map_report = (meshtastic_MapReport)meshtastic_MapReport_init_zero;
	mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;

	if (mqtt_ctx.last_map_report_ms != 0 && (now - mqtt_ctx.last_map_report_ms) < interval_ms) {
		return;
	}

#if defined(CONFIG_MESHTASTIC_POSITION)
	int ret;

	ret = meshtastic_position_get_current(&position);
	if (ret < 0) {
		if (mqtt_ctx.last_map_no_position_ms == 0 ||
		    (now - mqtt_ctx.last_map_no_position_ms) >= MQTT_MAP_WARN_MS) {
			LOG_WRN("MapReport enabled but no position available (%d)", ret);
			mqtt_ctx.last_map_no_position_ms = now;
		}
		return;
	}
#else
	if (mqtt_ctx.last_map_no_position_ms == 0 ||
	    (now - mqtt_ctx.last_map_no_position_ms) >= MQTT_MAP_WARN_MS) {
		LOG_WRN("MapReport enabled but position support is not compiled in");
		mqtt_ctx.last_map_no_position_ms = now;
	}
	return;
#endif

	if (!position.has_latitude_i || !position.has_longitude_i ||
	    (position.latitude_i == 0 && position.longitude_i == 0)) {
		if (mqtt_ctx.last_map_no_position_ms == 0 ||
		    (now - mqtt_ctx.last_map_no_position_ms) >= MQTT_MAP_WARN_MS) {
			LOG_WRN("MapReport enabled but position is zero");
			mqtt_ctx.last_map_no_position_ms = now;
		}
		return;
	}

	mqtt_build_map_report(&map_report, &position);

	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.from = meshtastic_get_node_id();
	mesh.to = MESHTASTIC_NODE_BROADCAST;
	mesh.id = (uint32_t)now;
	mesh.want_ack = false;
	mesh.hop_limit = meshtastic_runtime_hop_limit();
	mesh.hop_start = mesh.hop_limit;
	mesh.channel = 0U;

	decoded = &mesh.decoded;
	decoded->portnum = meshtastic_PortNum_MAP_REPORT_APP;
	{
		pb_ostream_t stream = pb_ostream_from_buffer(decoded->payload.bytes,
							     sizeof(decoded->payload.bytes));

		if (!pb_encode(&stream, meshtastic_MapReport_fields, &map_report)) {
			LOG_ERR("MapReport encode failed: %s", PB_GET_ERROR(&stream));
			return;
		}
		decoded->payload.size = (pb_size_t)stream.bytes_written;
	}

	mqtt_node_id_str(gateway_id, sizeof(gateway_id));

	ret = mqtt_encode_envelope(&mesh, channel_id, gateway_id, env_buf, sizeof(env_buf),
				   &env_len);
	if (ret < 0) {
		return;
	}

	/* Queued, and the back end drains it: the broker thread straight after this
	 * returns, the proxy synchronously inside the call. */
	mqtt_queue_publish(mqtt_ctx.map_topic, env_buf, env_len);

	mqtt_ctx.last_map_report_ms = now;
	LOG_INF("MapReport queued for %s (%u bytes)", mqtt_ctx.map_topic, (unsigned int)env_len);
}

#else

static void mqtt_perhaps_report_to_map(void)
{
}

#endif /* CONFIG_MESHTASTIC_MQTT_MAP_REPORT */

#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_UPLINK_ENABLED)
static void mqtt_queue_uplink(const struct meshtastic_packet *packet, const uint8_t *wire,
			      size_t wire_len, const meshtastic_MeshPacket *rx_mesh)
{
	uint8_t ch_index = meshtastic_channels_primary_index();

	if ((packet == NULL && rx_mesh == NULL) || wire == NULL || wire_len == 0U) {
		return;
	}
	/* Bridge disabled by module config (or never started): nothing drains the
	 * queue, so do not fill it. */
	if (!mqtt_ctx.started) {
		return;
	}

	/* C3 Phase 7: read the gate/consent fields from the decoded MeshPacket when the RX
	 * path supplied one, else the flat struct (TX path / public inject). Byte-identical —
	 * the struct is materialised from the same rx_mesh. */
	uint8_t channel_index = rx_mesh ? ((rx_mesh->channel < MESHTASTIC_MAX_CHANNELS)
						   ? (uint8_t)rx_mesh->channel
						   : MESHTASTIC_CHANNEL_INDEX_INVALID)
					: packet->channel_index;
	uint32_t from = rx_mesh ? rx_mesh->from : packet->from;
	uint32_t to = rx_mesh ? rx_mesh->to : packet->to;
	uint32_t id = rx_mesh ? rx_mesh->id : packet->id;
	bool via_mqtt = rx_mesh ? rx_mesh->via_mqtt : packet->via_mqtt;
	uint32_t portnum = rx_mesh ? (uint32_t)rx_mesh->decoded.portnum : packet->portnum;
	bool has_bitfield = rx_mesh ? rx_mesh->decoded.has_bitfield : packet->has_bitfield;
	uint8_t bitfield = rx_mesh ? (has_bitfield ? (uint8_t)rx_mesh->decoded.bitfield : 0U)
				   : packet->bitfield;

	if (channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID) {
		ch_index = channel_index;
	}

	if (!meshtastic_channels_uplink_enabled(ch_index)) {
		LOG_DBG("MQTT uplink disabled for channel %u", ch_index);
		return;
	}

	bool from_us = (from == meshtastic_get_node_id());

	if (via_mqtt) {
		LOG_DBG("Skipping MQTT uplink 0x%08x->0x%08x id=0x%08x (via_mqtt set)", from, to,
			id);
		return;
	}

	if (mqtt_should_skip_portnum(portnum, from_us)) {
		LOG_DBG("Skipping MQTT uplink port=%u on default broker", portnum);
		return;
	}

	/* Honour the sender's MQTT consent — "DontMqttMeBro" (parity: mqtt #1).
	 * Gate logic lives in meshtastic_mqtt_consent_allows_uplink() (header) so
	 * it stays unit-testable without networking. */
	if (!meshtastic_mqtt_consent_allows_uplink(from_us, mqtt_broker_is_private(), has_bitfield,
						    bitfield)) {
		LOG_DBG("Skipping MQTT uplink 0x%08x id=0x%08x (%s)", from, id,
			has_bitfield ? "OK_TO_MQTT clear" : "no bitfield, consent unknown");
		return;
	}

	char topic[128];
	int ret = mqtt_build_publish_topic(topic, sizeof(topic));
	if (ret < 0 || ret >= (int)sizeof(topic)) {
		LOG_DBG("MQTT uplink topic build failed (%d)", ret);
		return;
	}

	meshtastic_MeshPacket built = meshtastic_MeshPacket_init_zero;
	const meshtastic_MeshPacket *uplink;
	if (mqtt_ctx.cfg.encryption_enabled) {
		/* Publish the ciphertext exactly as heard/sent — built from the wire, not the
		 * decoded form. */
		ret = mqtt_mesh_from_wire(wire, wire_len, &built);
		if (ret < 0) {
			LOG_DBG("MQTT uplink wire parse failed (%d) len=%zu", ret, wire_len);
			return;
		}
		uplink = &built;
	} else if (rx_mesh != NULL) {
		/* C3 Phase 7: uplink the decoded packet we actually received, verbatim — its
		 * Data.emoji, rx_time, and any field the flat struct never modelled reach the
		 * broker, instead of being dropped by a struct->mesh rebuild (heard-packet path). */
		uplink = rx_mesh;
	} else {
		/* No decoded MeshPacket in hand (our own TX, or a public inject): rebuild from
		 * the struct. */
		ret = meshtastic_packet_to_mesh_pb(packet, &built);
		if (ret < 0) {
			LOG_DBG("MQTT uplink mesh encode failed (%d)", ret);
			return;
		}
		uplink = &built;
	}

	const char *channel_id = meshtastic_runtime_channel_name();
	char gateway_id[12];
	mqtt_node_id_str(gateway_id, sizeof(gateway_id));

	k_mutex_lock(&mqtt_ctx.lock, K_FOREVER);

	if (mqtt_ctx.queue_count >= CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE) {
		mqtt_drop_oldest_locked();
	}

	struct meshtastic_mqtt_pub_entry *entry = &mqtt_ctx.queue[mqtt_ctx.queue_tail];

	strncpy(entry->topic, topic, sizeof(entry->topic) - 1U);
	entry->topic[sizeof(entry->topic) - 1U] = '\0';

	size_t env_len = 0U;
	ret = mqtt_encode_envelope(uplink, channel_id, gateway_id, entry->payload,
				   sizeof(entry->payload), &env_len);
	if (ret < 0) {
		k_mutex_unlock(&mqtt_ctx.lock);
		return;
	}

	entry->len = (uint16_t)env_len;

	mqtt_ctx.queue_tail =
		(mqtt_ctx.queue_tail + 1U) % CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE;
	mqtt_ctx.queue_count++;

	LOG_DBG("MQTT uplink queued: %s (%u bytes, depth %u)", entry->topic, entry->len,
		mqtt_ctx.queue_count);

	k_mutex_unlock(&mqtt_ctx.lock);
	mqtt_kick();
}
#endif

void meshtastic_mqtt_core_downlink(const uint8_t *payload, size_t len, const char *topic)
{
	meshtastic_ServiceEnvelope env = meshtastic_ServiceEnvelope_init_zero;
	pb_istream_t stream;
	char local_id[12];
	meshtastic_MeshPacket inject;
	int ret;

	if (!IS_ENABLED(CONFIG_MESHTASTIC_MQTT_DOWNLINK_ENABLED)) {
		return;
	}

	stream = pb_istream_from_buffer(payload, len);
	if (!pb_decode(&stream, meshtastic_ServiceEnvelope_fields, &env)) {
		LOG_DBG("ServiceEnvelope decode failed: %s", PB_GET_ERROR(&stream));
		return;
	}

	if (env.channel_id == NULL || env.gateway_id == NULL || env.packet == NULL) {
		LOG_DBG("MQTT downlink missing envelope fields");
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	if (!meshtastic_channels_matches_mqtt_name(env.channel_id)) {
		LOG_DBG("MQTT downlink channel mismatch (got %s)", env.channel_id);
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	if (!meshtastic_channels_downlink_enabled(meshtastic_channels_primary_index())) {
		LOG_DBG("MQTT downlink disabled on primary channel");
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	mqtt_node_id_str(local_id, sizeof(local_id));
	if (strcmp(env.gateway_id, local_id) == 0) {
		LOG_DBG("Ignoring MQTT echo from our gateway id");
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	if (env.packet->from == meshtastic_get_node_id()) {
		LOG_DBG("Ignoring downlink we originally sent");
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	if (mqtt_ctx.cfg.encryption_enabled &&
	    env.packet->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		LOG_DBG("Ignoring decoded MQTT packet while encryption is enabled");
		pb_release(meshtastic_ServiceEnvelope_fields, &env);
		return;
	}

	meshtastic_mesh_packet_copy(&inject, env.packet);
	inject.via_mqtt = true;
	inject.transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT;

	if (inject.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		inject.channel = meshtastic_channels_primary_index();
	}

	LOG_DBG("MQTT downlink 0x%08x->0x%08x id=0x%08x via %s topic %s", inject.from, inject.to,
		inject.id, env.gateway_id, topic);
	LOG_INF("MQTT downlink topic %s len %zu", topic, len);
	ret = meshtastic_inject_downlink_mesh_packet(&inject);
	if (ret != 0) {
		LOG_DBG("MQTT downlink inject failed (%d)", ret);
	}

	pb_release(meshtastic_ServiceEnvelope_fields, &env);
}

void meshtastic_mqtt_on_tx(const struct meshtastic_packet *packet, const uint8_t *wire,
			   size_t wire_len, const meshtastic_MeshPacket *mesh)
{
#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_UPLINK_ENABLED)
	/*
	 * Match official firmware: uplink on LoRa TX only for packets we originate
	 * (Router::send). Heard packets are uplinked from meshtastic_mqtt_on_rx().
	 * C3 Phase 7d: when the mesh-native send engine supplies the outgoing MeshPacket,
	 * the plaintext uplink is built from it (Data.emoji and any unmodelled field survive
	 * our own uplink), else the flat struct (the pre-encrypted PKC path passes NULL).
	 */
	uint32_t from = mesh ? mesh->from : (packet != NULL ? packet->from : 0U);

	if (from != meshtastic_get_node_id()) {
		LOG_DBG("MQTT uplink skipped on TX (from 0x%08x, not us)", from);
		return;
	}

	mqtt_queue_uplink(packet, wire, wire_len, mesh);
#else
	ARG_UNUSED(packet);
	ARG_UNUSED(wire);
	ARG_UNUSED(wire_len);
	ARG_UNUSED(mesh);
#endif
}

void meshtastic_mqtt_on_rx(const struct meshtastic_packet *packet, const uint8_t *wire,
			   size_t wire_len, const meshtastic_MeshPacket *mesh)
{
#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_UPLINK_ENABLED)
	/*
	 * Match official firmware: uplink on LoRa RX only for packets from other nodes.
	 * C3 Phase 7: identity + the whole uplink come from the decoded MeshPacket when the
	 * RF path supplied one (heard-packet fields survive), else the flat struct.
	 */
	bool from_us = mesh ? (mesh->from == meshtastic_get_node_id())
			    : (packet != NULL && packet->from == meshtastic_get_node_id());

	if (from_us) {
		LOG_DBG("MQTT uplink skipped on RX (packet from us)");
		return;
	}

	mqtt_queue_uplink(packet, wire, wire_len, mesh);
#else
	ARG_UNUSED(packet);
	ARG_UNUSED(wire);
	ARG_UNUSED(wire_len);
	ARG_UNUSED(mesh);
#endif
}

bool meshtastic_mqtt_is_connected(void)
{
	/* A broker session. The proxy has no connection of its own to report: whether
	 * the client reached a broker is the client's business (the reference's
	 * isConnectedDirectly() is false in proxy mode too). */
#if defined(CONFIG_MESHTASTIC_MQTT_BROKER)
	return mqtt_ctx.started && !mqtt_ctx.proxy && meshtastic_mqtt_broker_connected();
#else
	return false;
#endif
}

/* ---- the core API the broker back end uses (meshtastic_mqtt_internal.h) ---------- */

const struct meshtastic_mqtt_settings *meshtastic_mqtt_core_settings(void)
{
	return &mqtt_ctx.cfg;
}

bool meshtastic_mqtt_core_dequeue(struct meshtastic_mqtt_pub_entry *out)
{
	bool got = false;

	k_mutex_lock(&mqtt_ctx.lock, K_FOREVER);
	if (mqtt_ctx.queue_count == 0U) {
		mqtt_ctx.drop_warned = false; /* drained: next full episode warns again */
	} else {
		*out = mqtt_ctx.queue[mqtt_ctx.queue_head];
		mqtt_ctx.queue_head =
			(mqtt_ctx.queue_head + 1U) % CONFIG_MESHTASTIC_MQTT_PUBLISH_QUEUE_SIZE;
		mqtt_ctx.queue_count--;
		got = true;
	}
	k_mutex_unlock(&mqtt_ctx.lock);

	return got;
}

bool meshtastic_mqtt_core_pending(void)
{
	bool pending;

	k_mutex_lock(&mqtt_ctx.lock, K_FOREVER);
	pending = mqtt_ctx.queue_count > 0U;
	k_mutex_unlock(&mqtt_ctx.lock);

	return pending;
}

void meshtastic_mqtt_core_link_up(void)
{
	mqtt_ctx.drop_warned = false;
}

void meshtastic_mqtt_core_perhaps_report_to_map(void)
{
	mqtt_perhaps_report_to_map();
}

int meshtastic_mqtt_core_subscribe_topic(char *topic, size_t topic_len)
{
	int ret = mqtt_build_subscribe_topic(topic, topic_len);

	return (ret < 0 || ret >= (int)topic_len) ? -EINVAL : 0;
}

/* ---- the client proxy ------------------------------------------------------------- */

#if defined(CONFIG_MESHTASTIC_MQTT_PROXY)
/* Serialises the drain: uplinks are queued from whichever thread heard or sent
 * the packet, and each of those threads then drains. Without this, two could
 * dequeue out of order into the PhoneAPI. */
static K_MUTEX_DEFINE(mqtt_proxy_lock);
static MESHTASTIC_EXT_RAM_BSS_ATTR struct meshtastic_mqtt_pub_entry mqtt_proxy_entry;

/* Hand every queued publish to the client, on the calling thread. Cheap: each is
 * one FromRadio frame onto the PhoneAPI queues, no I/O. */
static void mqtt_proxy_drain(void)
{
	k_mutex_lock(&mqtt_proxy_lock, K_FOREVER);
	while (meshtastic_mqtt_core_dequeue(&mqtt_proxy_entry)) {
		int ret = meshtastic_phoneapi_enqueue_mqtt_proxy(mqtt_proxy_entry.topic,
								  mqtt_proxy_entry.payload,
								  mqtt_proxy_entry.len);

		if (ret < 0) {
			/* Too big for MqttClientProxyMessage (topic 60, data 435). The
			 * reference truncates; a truncated topic publishes somewhere nobody
			 * subscribes and a truncated envelope does not decode, so drop. */
			LOG_WRN("MQTT proxy publish dropped (%d): %s, %u bytes", ret,
				mqtt_proxy_entry.topic, mqtt_proxy_entry.len);
		}
	}
	k_mutex_unlock(&mqtt_proxy_lock);
}

void meshtastic_mqtt_proxy_receive(const meshtastic_MqttClientProxyMessage *msg)
{
	if (msg == NULL) {
		return;
	}
	if (!mqtt_ctx.started || !mqtt_ctx.proxy) {
		/* Reference PhoneAPI: dropped unless the bridge is on AND proxying. A
		 * client must not be able to inject into a node that never agreed to
		 * be a gateway. */
		LOG_DBG("MQTT proxy message ignored: bridge not proxying (topic %s)", msg->topic);
		return;
	}

	/* payload_variant is a union: on the text variant the data size aliases the
	 * first bytes of the string, so read the member the tag names (the reference
	 * learned this the hard way -- MQTT.cpp onClientProxyReceive). */
	switch (msg->which_payload_variant) {
	case meshtastic_MqttClientProxyMessage_data_tag:
		if (msg->payload_variant.data.size == 0U) {
			break;
		}
		meshtastic_mqtt_core_downlink(msg->payload_variant.data.bytes,
					      msg->payload_variant.data.size, msg->topic);
		return;
	case meshtastic_MqttClientProxyMessage_text_tag: {
		/* nanopb sizes a max_size string one past the limit and always
		 * terminates it on decode, so strlen is bounded. */
		size_t len = strlen(msg->payload_variant.text);

		if (len == 0U) {
			break;
		}
		meshtastic_mqtt_core_downlink((const uint8_t *)msg->payload_variant.text, len,
					      msg->topic);
		return;
	}
	default:
		break;
	}

	LOG_WRN("MQTT proxy message has no payload, topic %s", msg->topic);
}

#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)
/* The broker thread asks for a map report on every pass; the proxy has no thread,
 * so a delayable work item asks instead, at the report interval. */
static void mqtt_proxy_map_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(mqtt_proxy_map_work, mqtt_proxy_map_work_fn);

static void mqtt_proxy_map_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	mqtt_perhaps_report_to_map();
	(void)k_work_reschedule(&mqtt_proxy_map_work,
				K_SECONDS(MAX(mqtt_ctx.cfg.map_publish_interval_secs, 60U)));
}
#endif
#endif /* CONFIG_MESHTASTIC_MQTT_PROXY */

static void mqtt_kick(void)
{
#if defined(CONFIG_MESHTASTIC_MQTT_PROXY)
	if (mqtt_ctx.proxy) {
		mqtt_proxy_drain();
		return;
	}
#endif
#if defined(CONFIG_MESHTASTIC_MQTT_BROKER)
	meshtastic_mqtt_broker_notify();
#endif
}

int meshtastic_mqtt_init(bool network)
{
	int ret;

	if (mqtt_ctx.started) {
		return 0;
	}

	/* ModuleConfig.mqtt is the runtime authority (agents-dnr4.8). The store is
	 * seeded from this build's Kconfig on first boot and thereafter carries
	 * whatever the admin channel wrote, so the Kconfig values are only ever the
	 * fallback for a field left empty. Resolved once, here; a later
	 * set_module_config schedules a reboot rather than re-applying live. */
	{
		meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

		if (meshtastic_config_store_get_module(meshtastic_ModuleConfig_mqtt_tag, &mod) ==
		    0) {
			meshtastic_mqtt_settings_resolve(&mod.payload_variant.mqtt, &mqtt_ctx.cfg);
		} else {
			/* Cannot happen (the store holds every module tag), but a store
			 * that answers nothing must not silently disable a bridge the
			 * build asked for: fall back to the pure Kconfig behaviour. */
			LOG_WRN("MQTT module config unreadable; using build defaults");
			meshtastic_mqtt_settings_resolve(NULL, &mqtt_ctx.cfg);
			mqtt_ctx.cfg.enabled = true;
		}
	}

	if (!mqtt_ctx.cfg.enabled) {
		LOG_INF("MQTT bridge disabled by module config");
		return 0;
	}

	/* Pick the back end. The admin path refuses a section this build cannot
	 * honour; a store written by a different image can still carry one, so each
	 * refusal is repeated here and the bridge stays off rather than guessing. */
	if (mqtt_ctx.cfg.proxy_to_client) {
		if (!IS_ENABLED(CONFIG_MESHTASTIC_MQTT_PROXY)) {
			LOG_ERR("MQTT proxy_to_client_enabled set but this image has no client "
				"proxy; bridge not started");
			return 0;
		}
		mqtt_ctx.proxy = true;
	} else {
		if (!IS_ENABLED(CONFIG_MESHTASTIC_MQTT_BROKER)) {
			/* The reference refuses the same: "proxy_to_client_enabled must
			 * be enabled on nodes that do not have a network". */
			LOG_ERR("MQTT enabled without proxy_to_client_enabled, and this image "
				"has no network stack; bridge not started");
			return 0;
		}
		if (!network) {
			LOG_INF("MQTT broker bridge idle: no network transport this boot");
			return 0;
		}
		if (mqtt_ctx.cfg.tls_enabled && !IS_ENABLED(CONFIG_MESHTASTIC_MQTT_TLS)) {
			/* Never downgrade to plaintext. */
			LOG_ERR("MQTT config requires TLS but this image has no TLS transport; "
				"bridge not started");
			return 0;
		}
	}
	if (mqtt_ctx.cfg.map_reporting_enabled && !IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)) {
		LOG_WRN("MQTT map_reporting_enabled set but map reporting is not compiled in");
	}

	k_mutex_init(&mqtt_ctx.lock);

	strncpy(mqtt_ctx.crypt_prefix, mqtt_ctx.cfg.root, sizeof(mqtt_ctx.crypt_prefix) - 1U);
	mqtt_ctx.crypt_prefix[sizeof(mqtt_ctx.crypt_prefix) - 1U] = '\0';

	if (IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT) && mqtt_ctx.cfg.map_reporting_enabled) {
		ret = mqtt_build_map_topic(mqtt_ctx.map_topic, sizeof(mqtt_ctx.map_topic));
		if (ret < 0 || ret >= (int)sizeof(mqtt_ctx.map_topic)) {
			LOG_ERR("MQTT map topic setup failed (%d)", ret);
			return -EINVAL;
		}
	}

	if (mqtt_ctx.proxy) {
#if defined(CONFIG_MESHTASTIC_MQTT_PROXY)
		mqtt_ctx.started = true;
#if IS_ENABLED(CONFIG_MESHTASTIC_MQTT_MAP_REPORT)
		if (mqtt_ctx.cfg.map_reporting_enabled) {
			(void)k_work_reschedule(&mqtt_proxy_map_work, K_SECONDS(5));
		}
#endif
		LOG_INF("Meshtastic MQTT bridge started through the client proxy (root %s)",
			mqtt_ctx.crypt_prefix);
#endif
		return 0;
	}

#if defined(CONFIG_MESHTASTIC_MQTT_BROKER)
	ret = meshtastic_mqtt_broker_start();
	if (ret < 0) {
		return ret;
	}
	mqtt_ctx.started = true;

	LOG_INF("Meshtastic MQTT bridge started (broker %s:%u%s root %s%s)", mqtt_ctx.cfg.host,
		(unsigned int)mqtt_ctx.cfg.port, mqtt_ctx.cfg.tls_enabled ? " tls" : "",
		mqtt_ctx.crypt_prefix, mqtt_ctx.cfg.address_is_custom ? "" : " [build default]");
#endif

	return 0;
}
