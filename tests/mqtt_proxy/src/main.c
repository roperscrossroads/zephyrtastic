/* SPDX-License-Identifier: GPL-3.0
 *
 * MQTT client proxy (agents-kx8d), driven the way a phone drives it: the bridge
 * on a build with NO network stack, so its only way to a broker is the connected
 * client. What goes up is a FromRadio.mqttClientProxyMessage the client would
 * publish; what comes down is a ToRadio.mqttClientProxyMessage the client would
 * have received on its subscription. Frames are real channel-encrypted wire
 * frames, built by the stack's own builder -- the same bytes a direct broker
 * connection would have carried.
 *
 * The reference: MQTT.cpp (publish -> sendMqttMessageToClientProxy,
 * onClientProxyReceive) and PhoneAPI.cpp (the ToRadio gate).
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/mqtt.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_mqtt.h"
#include "meshtastic_mqtt_config.h"
#include "meshtastic_packet.h"
#include "meshtastic_phoneapi.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

#define PEER_ID  0x55667788U
#define PEER2_ID 0x1234ABCDU

static struct meshtastic_config cfg = {
	.node_id = 0x11223344U,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
};

#define PHONE_Q 32U
static struct meshtastic_phoneapi_frame phone_q[PHONE_Q];
static struct meshtastic_phoneapi phone;
static meshtastic_ToRadio phone_to;
static meshtastic_FromRadio phone_from;

static meshtastic_FromRadio dec; /* decode scratch, off the stack */
static uint32_t me;              /* the id the stack actually runs under */
static uint32_t next_id = 0x0D0C0000U;

/* ---- helpers ---------------------------------------------------------------------- */

static bool decode_frame(const struct meshtastic_phoneapi_frame *f, meshtastic_FromRadio *out)
{
	pb_istream_t is = pb_istream_from_buffer(f->data, f->len);

	*out = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
	return pb_decode(&is, meshtastic_FromRadio_fields, out);
}

static void drain(void)
{
	struct meshtastic_phoneapi_frame f;

	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
	}
}

/* Pop frames until one of @p variant shows up (others skipped), for up to @p ms. */
static bool wait_for(pb_size_t variant, int ms)
{
	struct meshtastic_phoneapi_frame f;

	for (int t = 0; t <= ms; t += 10) {
		while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
			if (decode_frame(&f, &dec) && dec.which_payload_variant == variant) {
				return true;
			}
		}
		k_sleep(K_MSEC(10));
	}
	return false;
}

/* The packet with @p id, delivered to the phone, within @p ms. */
static bool wait_for_packet(uint32_t id, int ms)
{
	struct meshtastic_phoneapi_frame f;

	for (int t = 0; t <= ms; t += 10) {
		while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
			if (decode_frame(&f, &dec) &&
			    dec.which_payload_variant == meshtastic_FromRadio_packet_tag &&
			    dec.packet.id == id) {
				return true;
			}
		}
		k_sleep(K_MSEC(10));
	}
	return false;
}

static void send_toradio(const meshtastic_ToRadio *to)
{
	static uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	zassert_true(pb_encode(&os, meshtastic_ToRadio_fields, to), "ToRadio encode");
	meshtastic_phoneapi_handle_toradio(&phone, buf, os.bytes_written);
}

/* A channel-encrypted text frame from @p from, as it would cross the air. */
static void build_text_wire(uint32_t from, uint32_t id, const char *text, bool consent,
			    struct meshtastic_packet *pkt, uint8_t *wire, uint32_t *wire_len)
{
	*pkt = (struct meshtastic_packet){
		.from = from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)text,
		.payload_len = strlen(text),
		.hop_limit = 3U,
		.hop_start = 3U,
		.channel_index = meshtastic_channels_primary_index(),
		.has_bitfield = consent,
		.bitfield = consent ? MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK : 0U,
	};
	zassert_ok(meshtastic_build_wire_packet(pkt, wire, wire_len), "wire build");
}

static void expected_topic(char *buf, size_t len, const char *gateway)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mqtt_tag, &mod), "");
	snprintk(buf, len, "%s/2/e/%s/%s", mod.payload_variant.mqtt.root,
		 meshtastic_runtime_channel_name(), gateway);
}

/* ---- suite ---------------------------------------------------------------------- */

static void *suite_setup(void)
{
	static meshtastic_Channel ch;

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	me = meshtastic_get_node_id();

	meshtastic_phoneapi_init(&phone, "phone", phone_q, PHONE_Q, NULL, NULL, NULL, NULL,
				 &phone_to, &phone_from);
	meshtastic_phoneapi_register(&phone);

	/* A gateway channel: uplink and downlink on the primary. Seeded off, which is
	 * why a proxy that is on by default uplinks nothing until someone asks. */
	ch = *meshtastic_channels_get(meshtastic_channels_primary_index());
	ch.has_settings = true;
	ch.settings.uplink_enabled = true;
	ch.settings.downlink_enabled = true;
	zassert_ok(meshtastic_config_store_set_channel(meshtastic_channels_primary_index(), &ch),
		   "");
	zassert_true(meshtastic_channels_uplink_enabled(meshtastic_channels_primary_index()),
		     "uplink now enabled on the primary");
	return NULL;
}

static void suite_before(void *f)
{
	ARG_UNUSED(f);
	k_sleep(K_MSEC(50));
	drain();
}

ZTEST_SUITE(mqtt_proxy, NULL, suite_setup, suite_before, NULL, NULL);

/* A build with no network seeds the proxy on: otherwise its first-boot section
 * would be one it refuses (enabled, not proxying, no broker to connect to). */
ZTEST(mqtt_proxy, test_a_build_without_a_network_seeds_the_proxy_on)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_mqtt_tag, &mod), "");
	zassert_true(mod.payload_variant.mqtt.enabled, "");
	zassert_true(mod.payload_variant.mqtt.proxy_to_client_enabled,
		     "no broker back end: the proxy is the only way, so it is the seed");
	zassert_false(meshtastic_mqtt_is_connected(),
		      "the proxy has no broker session of its own to report");
}

/* The admin path refuses what this build cannot honour (reference: "proxy_to_
 * client_enabled must be enabled on nodes that do not have a network"). */
ZTEST(mqtt_proxy, test_admin_refuses_a_broker_bridge_without_a_network)
{
	meshtastic_ModuleConfig_MQTTConfig m = meshtastic_ModuleConfig_MQTTConfig_init_zero;

	m.enabled = true;
	m.proxy_to_client_enabled = false;
	zassert_equal(meshtastic_mqtt_config_validate(&m), -ENETUNREACH,
		      "enabled, no proxy, no broker back end: refused");

	m.proxy_to_client_enabled = true;
	zassert_ok(meshtastic_mqtt_config_validate(&m), "the proxy is accepted");

	m.enabled = false;
	m.proxy_to_client_enabled = false;
	zassert_ok(meshtastic_mqtt_config_validate(&m), "a disabled bridge is always fine");
}

/* A packet heard from a peer goes up to the client, not a broker: topic
 * {root}/2/e/{channel}/!{us}, payload a ServiceEnvelope carrying the ciphertext
 * exactly as heard. */
ZTEST(mqtt_proxy, test_uplink_reaches_the_client_as_a_proxy_message)
{
	static uint8_t wire[256];
	static meshtastic_ServiceEnvelope env;
	struct meshtastic_packet pkt;
	uint32_t wire_len = sizeof(wire);
	uint32_t id = next_id++;
	char topic[64];
	char gateway[12];
	pb_istream_t is;

	build_text_wire(PEER_ID, id, "hello broker", true, &pkt, wire, &wire_len);
	meshtastic_mqtt_on_rx(&pkt, wire, wire_len, NULL);

	zassert_true(wait_for(meshtastic_FromRadio_mqttClientProxyMessage_tag, 200),
		     "the uplink reached the client");
	snprintk(gateway, sizeof(gateway), "!%08x", me);
	expected_topic(topic, sizeof(topic), gateway);
	zassert_str_equal(dec.mqttClientProxyMessage.topic, topic, "");
	zassert_equal(dec.mqttClientProxyMessage.which_payload_variant,
		      meshtastic_MqttClientProxyMessage_data_tag, "an envelope is bytes");
	zassert_false(dec.mqttClientProxyMessage.retained, "");

	env = (meshtastic_ServiceEnvelope)meshtastic_ServiceEnvelope_init_zero;
	is = pb_istream_from_buffer(dec.mqttClientProxyMessage.payload_variant.data.bytes,
				    dec.mqttClientProxyMessage.payload_variant.data.size);
	zassert_true(pb_decode(&is, meshtastic_ServiceEnvelope_fields, &env), "envelope decodes");
	zassert_not_null(env.packet, "");
	zassert_equal(env.packet->from, PEER_ID, "");
	zassert_equal(env.packet->id, id, "");
	zassert_equal(env.packet->which_payload_variant, meshtastic_MeshPacket_encrypted_tag,
		      "encryption_enabled: the ciphertext as heard, not a decode");
	zassert_str_equal(env.gateway_id, gateway, "");
	zassert_str_equal(env.channel_id, meshtastic_runtime_channel_name(), "");
	pb_release(meshtastic_ServiceEnvelope_fields, &env);
}

/* The sender's consent still gates the uplink when the bytes go through a client:
 * a packet without OK_TO_MQTT is not republished to the (public, default) broker. */
ZTEST(mqtt_proxy, test_uplink_through_the_proxy_still_honours_consent)
{
	static uint8_t wire[256];
	struct meshtastic_packet pkt;
	uint32_t wire_len = sizeof(wire);

	build_text_wire(PEER_ID, next_id++, "keep me off mqtt", false, &pkt, wire, &wire_len);
	meshtastic_mqtt_on_rx(&pkt, wire, wire_len, NULL);

	zassert_false(wait_for(meshtastic_FromRadio_mqttClientProxyMessage_tag, 100),
		      "no consent, no uplink");
}

static void send_proxy_downlink(uint32_t from, uint32_t id, const char *text)
{
	static uint8_t wire[256];
	static meshtastic_MeshPacket mesh;
	static meshtastic_ToRadio to;
	struct meshtastic_packet pkt;
	uint32_t wire_len = sizeof(wire);
	meshtastic_ServiceEnvelope env = meshtastic_ServiceEnvelope_init_zero;
	char channel[16];
	char gateway[] = "!0badc0de"; /* some other gateway on the broker */
	pb_ostream_t os;

	build_text_wire(from, id, text, true, &pkt, wire, &wire_len);

	/* The envelope another gateway would have published for that frame: the
	 * 16-byte header unpacked into MeshPacket fields, the ciphertext as-is. */
	mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
	mesh.to = sys_get_le32(&wire[0]);
	mesh.from = sys_get_le32(&wire[4]);
	mesh.id = sys_get_le32(&wire[8]);
	mesh.hop_limit = wire[12] & 0x07U;
	mesh.hop_start = (wire[12] >> 5) & 0x07U;
	mesh.channel = wire[13];
	mesh.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	mesh.encrypted.size = (pb_size_t)(wire_len - 16U);
	memcpy(mesh.encrypted.bytes, &wire[16], mesh.encrypted.size);

	snprintk(channel, sizeof(channel), "%s", meshtastic_runtime_channel_name());
	env.packet = &mesh;
	env.channel_id = channel;
	env.gateway_id = gateway;

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_mqttClientProxyMessage_tag;
	expected_topic(to.mqttClientProxyMessage.topic, sizeof(to.mqttClientProxyMessage.topic),
		       gateway);
	to.mqttClientProxyMessage.which_payload_variant = meshtastic_MqttClientProxyMessage_data_tag;
	os = pb_ostream_from_buffer(to.mqttClientProxyMessage.payload_variant.data.bytes,
				    sizeof(to.mqttClientProxyMessage.payload_variant.data.bytes));
	zassert_true(pb_encode(&os, meshtastic_ServiceEnvelope_fields, &env), "envelope encode");
	to.mqttClientProxyMessage.payload_variant.data.size = (pb_size_t)os.bytes_written;

	send_toradio(&to);
}

/* What the client received on its subscription comes back down and reaches the
 * mesh: decrypted with the channel key and delivered, flagged as via MQTT. */
ZTEST(mqtt_proxy, test_downlink_from_the_client_reaches_the_mesh)
{
	uint32_t id = next_id++;

	send_proxy_downlink(PEER2_ID, id, "hello mesh");

	zassert_true(wait_for_packet(id, 500), "the downlink was injected and delivered");
	zassert_equal(dec.packet.from, PEER2_ID, "");
	zassert_true(dec.packet.via_mqtt, "marked as having come from MQTT");
	zassert_equal(dec.packet.which_payload_variant, meshtastic_MeshPacket_decoded_tag,
		      "decrypted with the channel key");
	zassert_equal(dec.packet.decoded.portnum, meshtastic_PortNum_TEXT_MESSAGE_APP, "");
	zassert_mem_equal(dec.packet.decoded.payload.bytes, "hello mesh", 10, "");
}

/* Reference PhoneAPI: a proxy message is ignored until the client is past the
 * config handshake. */
ZTEST(mqtt_proxy, test_downlink_during_the_config_handshake_is_ignored)
{
	struct meshtastic_phoneapi_frame f;
	uint32_t id = next_id++;

	meshtastic_phoneapi_enqueue_phone_config(&phone, 0x5151U);
	send_proxy_downlink(PEER2_ID, id, "too early");
	while (meshtastic_phoneapi_next_config_frame(&phone, &f) == 0) {
	}

	zassert_false(wait_for_packet(id, 300), "mid-handshake: not injected");
}

/* The same frame twice is one delivery: a downlink is subject to the mesh's own
 * duplicate suppression, so a client relaying a broker echo cannot flood. */
ZTEST(mqtt_proxy, test_a_repeated_downlink_is_delivered_once)
{
	uint32_t id = next_id++;

	send_proxy_downlink(PEER2_ID, id, "once");
	zassert_true(wait_for_packet(id, 500), "first copy delivered");
	send_proxy_downlink(PEER2_ID, id, "once");
	zassert_false(wait_for_packet(id, 300), "second copy suppressed");
}

/* The message caps (mesh.options: topic 60, data 435) are enforced before a frame
 * is built, and refused rather than truncated -- a truncated topic publishes where
 * nobody subscribes, a truncated envelope does not decode. */
ZTEST(mqtt_proxy, test_the_proxy_refuses_what_it_cannot_carry)
{
	static uint8_t big[436];
	char topic[61];

	memset(topic, 'a', sizeof(topic) - 1U);
	topic[sizeof(topic) - 1U] = '\0';
	zassert_equal(meshtastic_phoneapi_enqueue_mqtt_proxy(topic, big, 10), -ENAMETOOLONG, "");
	zassert_equal(meshtastic_phoneapi_enqueue_mqtt_proxy("msh/x", big, sizeof(big)),
		      -EMSGSIZE, "");
	zassert_equal(meshtastic_phoneapi_enqueue_mqtt_proxy("msh/x", big, sizeof(big) - 1U), 1,
		      "435 bytes fits, and reached the one registered transport");
	zassert_true(wait_for(meshtastic_FromRadio_mqttClientProxyMessage_tag, 100), "");
	zassert_equal(dec.mqttClientProxyMessage.payload_variant.data.size, 435U,
		      "the largest payload still encodes into one FromRadio frame");
}
