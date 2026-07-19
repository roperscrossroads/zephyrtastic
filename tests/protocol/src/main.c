#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic_admin.h"
#include "meshtastic_admin_session.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_region_presets.h"
#include "meshtastic_reliable.h"
#include "meshtastic_router.h"
#include "meshtastic_sched.h"

#define TEST_NODE_ID  0x12345678U
#define PEER_NODE_ID  0x87654321U
#define OTHER_NODE_ID 0x13572468U

struct mock_lora_state {
	struct k_mutex lock;
	struct lora_modem_config config;
	lora_recv_cb rx_cb;
	void *rx_user_data;
	int send_result;
	uint32_t send_count;
	uint32_t config_count;
	uint8_t last_tx[MESHTASTIC_PKT_MAX];
	uint32_t last_tx_len;

	/* Gate for scheduler-egress ordering tests: when enabled, the worker
	 * thread parks inside send() until the test releases it one frame at a
	 * time, and every transmitted frame's wire id is logged in order. */
	bool gate_enabled;
	struct k_sem gate;    /* worker waits here in send() while gated */
	struct k_sem entered; /* given as the worker enters a gated send() */
	uint32_t tx_ids[16];  /* wire ids in transmit order */
	uint8_t tx_ids_count;
};

static struct mock_lora_state mock_lora;

static int mock_lora_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	k_mutex_init(&mock_lora.lock);
	k_sem_init(&mock_lora.gate, 0, ARRAY_SIZE(mock_lora.tx_ids));
	k_sem_init(&mock_lora.entered, 0, ARRAY_SIZE(mock_lora.tx_ids));
	mock_lora.send_result = 0;

	return 0;
}

static int mock_lora_config(const struct device *dev, const struct lora_modem_config *config)
{
	ARG_UNUSED(dev);

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.config = *config;
	mock_lora.config_count++;
	k_mutex_unlock(&mock_lora.lock);

	return 0;
}

static uint32_t mock_lora_airtime(const struct device *dev, uint32_t data_len)
{
	ARG_UNUSED(dev);

	return data_len;
}

static int mock_lora_send(const struct device *dev, uint8_t *data, uint32_t data_len)
{
	int ret;

	ARG_UNUSED(dev);

	bool gated;

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_true(data_len <= sizeof(mock_lora.last_tx), "unexpected tx len %u", data_len);
	memcpy(mock_lora.last_tx, data, data_len);
	mock_lora.last_tx_len = data_len;
	mock_lora.send_count++;
	if (mock_lora.tx_ids_count < ARRAY_SIZE(mock_lora.tx_ids) &&
	    data_len >= MESHTASTIC_HDR_LEN) {
		const struct meshtastic_wire_header *hdr =
			(const struct meshtastic_wire_header *)data;

		mock_lora.tx_ids[mock_lora.tx_ids_count++] = sys_le32_to_cpu(hdr->id);
	}
	ret = mock_lora.send_result;
	gated = mock_lora.gate_enabled;
	k_mutex_unlock(&mock_lora.lock);

	/* Park the worker so the test can stack up frames and release them one at
	 * a time — never hold mock_lora.lock while blocked. */
	if (gated) {
		k_sem_give(&mock_lora.entered);
		(void)k_sem_take(&mock_lora.gate, K_FOREVER);
	}

	return ret;
}

static int mock_lora_send_async(const struct device *dev, uint8_t *data, uint32_t data_len,
				struct k_poll_signal *async)
{
	int ret = mock_lora_send(dev, data, data_len);

	if (async != NULL) {
		k_poll_signal_raise(async, ret);
	}

	return ret;
}

static int mock_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
			  k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);

	return -ENOTSUP;
}

static int mock_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	ARG_UNUSED(dev);

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.rx_cb = cb;
	mock_lora.rx_user_data = user_data;
	k_mutex_unlock(&mock_lora.lock);

	return 0;
}

static DEVICE_API(lora, mock_lora_api) = {
	.config = mock_lora_config,
	.airtime = mock_lora_airtime,
	.send = mock_lora_send,
	.send_async = mock_lora_send_async,
	.recv = mock_lora_recv,
	.recv_async = mock_lora_recv_async,
};

DEVICE_DEFINE(mock_lora, "mock_lora", mock_lora_init, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mock_lora_api);

static const struct device *const lora_dev = DEVICE_GET(mock_lora);

struct test_state {
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	struct meshtastic_packet last_rx;
	struct meshtastic_packet last_event_packet;
	struct meshtastic_packet last_received_packet;
	uint8_t last_rx_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t last_event_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t last_received_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	size_t last_rx_payload_len;
	size_t last_event_payload_len;
	size_t last_received_payload_len;
	struct meshtastic_event last_event;
	struct meshtastic_event last_received;
	uint32_t recv_count;
	uint32_t event_count;
};

static struct test_state state;

static void reset_callbacks_state(void)
{
	memset(&state.last_rx, 0, sizeof(state.last_rx));
	memset(&state.last_event_packet, 0, sizeof(state.last_event_packet));
	memset(&state.last_received_packet, 0, sizeof(state.last_received_packet));
	memset(state.last_rx_payload, 0, sizeof(state.last_rx_payload));
	memset(state.last_event_payload, 0, sizeof(state.last_event_payload));
	memset(state.last_received_payload, 0, sizeof(state.last_received_payload));
	memset(&state.last_event, 0, sizeof(state.last_event));
	memset(&state.last_received, 0, sizeof(state.last_received));
	state.last_rx_payload_len = 0U;
	state.last_event_payload_len = 0U;
	state.last_received_payload_len = 0U;
	state.recv_count = 0U;
	state.event_count = 0U;
	k_sem_reset(&state.rx_sem);
	k_sem_reset(&state.tx_sem);
}

static void reset_mock_lora(void)
{
	lora_recv_cb rx_cb;
	void *rx_user_data;

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	rx_cb = mock_lora.rx_cb;
	rx_user_data = mock_lora.rx_user_data;
	memset(&mock_lora.config, 0, sizeof(mock_lora.config));
	mock_lora.rx_cb = rx_cb;
	mock_lora.rx_user_data = rx_user_data;
	mock_lora.send_result = 0;
	mock_lora.send_count = 0U;
	mock_lora.config_count = 0U;
	mock_lora.last_tx_len = 0U;
	memset(mock_lora.last_tx, 0, sizeof(mock_lora.last_tx));
	mock_lora.gate_enabled = false;
	mock_lora.tx_ids_count = 0U;
	k_mutex_unlock(&mock_lora.lock);
	k_sem_reset(&mock_lora.gate);
	k_sem_reset(&mock_lora.entered);
}

static void on_recv(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t payload_len, int16_t rssi, int8_t snr)
{
	state.last_rx = (struct meshtastic_packet){
		.from = from,
		.to = to,
		.portnum = portnum,
		.payload = state.last_rx_payload,
		.payload_len = payload_len,
		.rssi = rssi,
		.snr = snr,
	};
	state.last_rx_payload_len = payload_len;
	if (payload_len > 0U) {
		memcpy(state.last_rx_payload, payload, payload_len);
	}
	state.recv_count++;
	k_sem_give(&state.rx_sem);
}

static void on_event(const struct meshtastic_event *event, void *user_data)
{
	ARG_UNUSED(user_data);

	state.last_event = *event;
	if (event->packet != NULL) {
		state.last_event_packet = *event->packet;
		state.last_event_packet.payload = state.last_event_payload;
		state.last_event_payload_len = event->packet->payload_len;
		if (event->packet->payload_len > 0U) {
			memcpy(state.last_event_payload, event->packet->payload,
			       event->packet->payload_len);
		}
	} else {
		memset(&state.last_event_packet, 0, sizeof(state.last_event_packet));
		memset(state.last_event_payload, 0, sizeof(state.last_event_payload));
		state.last_event_payload_len = 0U;
	}
	/*
	 * Capture PACKET_RECEIVED into a dedicated slot so assertions on a
	 * delivered packet survive a later async event (e.g. the relay TX_DONE
	 * a broadcast downlink also produces) clobbering the shared last_event.
	 */
	if (event->type == MESHTASTIC_EVENT_PACKET_RECEIVED) {
		state.last_received = *event;
		if (event->packet != NULL) {
			state.last_received_packet = *event->packet;
			state.last_received_packet.payload = state.last_received_payload;
			state.last_received_payload_len = event->packet->payload_len;
			if (event->packet->payload_len > 0U) {
				memcpy(state.last_received_payload, event->packet->payload,
				       event->packet->payload_len);
			}
		}
	}
	state.event_count++;
	if (event->type == MESHTASTIC_EVENT_TX_DONE || event->type == MESHTASTIC_EVENT_TX_FAILED) {
		k_sem_give(&state.tx_sem);
	}
}

static void *protocol_suite_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};
	int ret;

	k_sem_init(&state.rx_sem, 0, 1);
	k_sem_init(&state.tx_sem, 0, 1);

	zassert_true(device_is_ready(lora_dev), "mock lora device not ready");

	ret = meshtastic_init(&cfg);
	zassert_ok(ret, "meshtastic_init failed: %d", ret);

	meshtastic_set_recv_cb(on_recv);
	meshtastic_set_event_cb(on_event, NULL);

	reset_mock_lora();
	reset_callbacks_state();

	return NULL;
}

static void protocol_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	memset(mt.dup_cache, 0, sizeof(mt.dup_cache));
	mt.dup_head = 0U;
	meshtastic_sched_defaults();
	/* This suite asserts routing *semantics* — hop-limit decrement, dedup,
	 * egress ordering — by injecting a frame and checking what went out. The
	 * default contention window defers a relay by hundreds of milliseconds to
	 * seconds, which would turn every one of those into a sleep-and-hope race.
	 * Pin the window off so the timing is deterministic and the assertions
	 * measure what they claim to. The window itself is covered by the
	 * contention tests below and by the vector suite. */
	zassert_ok(meshtastic_sched_set("cw.max", "0"));
	meshtastic_reliable_reset();
	reset_mock_lora();
	reset_callbacks_state();
}

static void assert_payload(const uint8_t *actual, size_t actual_len, const void *expected,
			   size_t expected_len)
{
	zassert_equal(actual_len, expected_len, "unexpected payload len");
	if (expected_len > 0U) {
		zassert_not_null(actual, "expected payload pointer");
		zassert_mem_equal(actual, expected, expected_len, "unexpected payload");
	} else {
		zassert_is_null(actual, "empty payload should use NULL pointer");
	}
}

static void assert_rx_payload(const void *expected, size_t expected_len)
{
	zassert_equal(state.last_rx_payload_len, expected_len, "unexpected RX payload len");
	if (expected_len > 0U) {
		zassert_mem_equal(state.last_rx_payload, expected, expected_len,
				  "unexpected RX payload");
	}
}

static void assert_received_payload(const void *expected, size_t expected_len)
{
	zassert_equal(state.last_received_payload_len, expected_len,
		      "unexpected received payload len");
	if (expected_len > 0U) {
		zassert_mem_equal(state.last_received_payload, expected, expected_len,
				  "unexpected received payload");
	}
}

static void assert_mock_send_count(uint32_t expected)
{
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_equal(mock_lora.send_count, expected, "unexpected lora_send count");
	k_mutex_unlock(&mock_lora.lock);
}

static void set_mock_send_result(int send_result)
{
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.send_result = send_result;
	k_mutex_unlock(&mock_lora.lock);
}

static void build_wire_packet(uint32_t from, uint32_t to, uint32_t id, uint8_t hop_limit,
			      uint32_t portnum, const uint8_t *payload, size_t payload_len,
			      uint8_t *wire, uint32_t *wire_len)
{
	struct meshtastic_packet packet = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
		.hop_limit = hop_limit,
		.hop_start = hop_limit,
		.channel_index = meshtastic_channels_primary_index(),
	};
	int ret;

	ret = meshtastic_build_wire_packet(&packet, wire, wire_len);
	zassert_ok(ret, "meshtastic_build_wire_packet failed: %d", ret);
}

static void build_peer_wire_packet(uint32_t to, uint32_t id, uint8_t hop_limit, const char *text,
				   uint8_t *wire, uint32_t *wire_len)
{
	build_wire_packet(PEER_NODE_ID, to, id, hop_limit, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)text, strlen(text), wire, wire_len);
}

static void decode_last_tx(struct meshtastic_packet *decoded, uint8_t *payload, size_t payload_len)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	int ret;

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_equal(mock_lora.send_count, 1U, "expected one lora_send call");
	zassert_true(mock_lora.last_tx_len > MESHTASTIC_HDR_LEN, "expected wire payload");
	wire_len = mock_lora.last_tx_len;
	memcpy(wire, mock_lora.last_tx, wire_len);
	k_mutex_unlock(&mock_lora.lock);

	ret = meshtastic_decode_wire_packet(wire, wire_len, 0, 0, decoded, payload, payload_len);
	zassert_ok(ret, "meshtastic_decode_wire_packet failed: %d", ret);
}

static void copy_last_tx_header(struct meshtastic_wire_header *hdr)
{
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_true(mock_lora.last_tx_len >= MESHTASTIC_HDR_LEN, "expected tx header");
	memcpy(hdr, mock_lora.last_tx, sizeof(*hdr));
	k_mutex_unlock(&mock_lora.lock);
}

static void assert_wire_header(const struct meshtastic_wire_header *hdr, uint32_t from, uint32_t to,
			       uint32_t id, uint8_t hop_limit, uint8_t hop_start, bool want_ack,
			       bool via_mqtt, uint8_t next_hop, uint8_t relay_node)
{
	uint8_t flags = hdr->flags;

	zassert_equal(sys_le32_to_cpu(hdr->src), from, "unexpected wire source");
	zassert_equal(sys_le32_to_cpu(hdr->dest), to, "unexpected wire destination");
	zassert_equal(sys_le32_to_cpu(hdr->id), id, "unexpected wire id");
	zassert_equal(flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, hop_limit, "unexpected hop limit");
	zassert_equal((flags & MESHTASTIC_FLAGS_HOP_START_MASK) >> MESHTASTIC_FLAGS_HOP_START_SHIFT,
		      hop_start, "unexpected hop start");
	zassert_equal((flags & MESHTASTIC_FLAGS_WANT_ACK) != 0U, want_ack,
		      "unexpected want_ack flag");
	zassert_equal((flags & MESHTASTIC_FLAGS_VIA_MQTT) != 0U, via_mqtt,
		      "unexpected via_mqtt flag");
	zassert_equal(hdr->channel,
		      meshtastic_channels_get_hash(meshtastic_channels_primary_index()),
		      "unexpected channel hash");
	zassert_equal(hdr->next_hop, next_hop, "unexpected next hop");
	zassert_equal(hdr->relay_node, relay_node, "unexpected relay node");
}

static void inject_rx_frame(const uint8_t *wire, uint32_t wire_len, int16_t rssi, int8_t snr)
{
	lora_recv_cb cb;
	void *user_data;
	uint8_t frame[MESHTASTIC_PKT_MAX];

	zassert_true(wire_len <= sizeof(frame), "unexpected rx len %u", wire_len);

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	cb = mock_lora.rx_cb;
	user_data = mock_lora.rx_user_data;
	k_mutex_unlock(&mock_lora.lock);

	zassert_not_null(cb, "rx callback not armed");

	memcpy(frame, wire, wire_len);
	cb(lora_dev, frame, wire_len, rssi, snr, user_data);
}


/* Every packet we originate must carry Data.bitfield, with bit 0 reflecting
 * config.lora.config_ok_to_mqtt (parity: mqtt #1, our half of the consent).
 *
 * The field is emitted even when consent is false. That is the point: a
 * receiver cannot tell "declined" from "too old to have an opinion" if the
 * field is absent, and it must treat absence as declined — so staying silent
 * would mean our own traffic is dropped by any gateway honouring the flag.
 */
ZTEST(protocol_stack, test_outgoing_packets_carry_mqtt_consent)
{
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool saved = mt.config_ok_to_mqtt;

	/* Consent granted: the bit is set. */
	mt.config_ok_to_mqtt = true;
	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "consent-yes"),
		   "send failed");
	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_true(decoded.has_bitfield, "an originated packet must carry a bitfield");
	zassert_true(decoded.bitfield & MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK,
		     "OK_TO_MQTT must be set when config_ok_to_mqtt is true");

	/* Consent withheld: the field is still present, the bit is clear. */
	protocol_before(NULL);
	mt.config_ok_to_mqtt = false;
	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "consent-no"),
		   "send failed");
	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_true(decoded.has_bitfield,
		     "the bitfield must be emitted even when consent is withheld, so "
		     "\"declined\" is distinguishable from \"never asked\"");
	zassert_false(decoded.bitfield & MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK,
		      "OK_TO_MQTT must be clear when config_ok_to_mqtt is false");

	mt.config_ok_to_mqtt = saved;
}

/* Bit 1 of the same field mirrors want_response, matching the reference's
 * BITFIELD_WANT_RESPONSE_SHIFT. Asserted so a future edit to the consent bit
 * cannot quietly clobber its neighbour.
 */
ZTEST(protocol_stack, test_bitfield_carries_want_response)
{
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	bool saved = mt.config_ok_to_mqtt;

	mt.config_ok_to_mqtt = true;
	/* A plain text send does not request a response. */
	zassert_ok(meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "no-resp"), "send failed");
	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_false(decoded.bitfield & MESHTASTIC_BITFIELD_WANT_RESPONSE_MASK,
		      "want_response bit should be clear for a plain text message");
	zassert_true(decoded.bitfield & MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK,
		     "the consent bit is independent of want_response");

	mt.config_ok_to_mqtt = saved;
}

ZTEST_SUITE(protocol_stack, NULL, protocol_suite_setup, protocol_before, NULL, NULL);

/* Verifies local text sends produce one LoRa frame, a TX_DONE event, and a decodable payload. */
ZTEST(protocol_stack, test_send_text_uses_mock_lora_and_round_trips)
{
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	ret = meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "ping");
	zassert_ok(ret, "meshtastic_send_text failed: %d", ret);
	zassert_ok(k_sem_take(&state.tx_sem, K_SECONDS(1)), "timed out waiting for tx event");

	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_equal(decoded.from, TEST_NODE_ID, "unexpected source");
	zassert_equal(decoded.to, MESHTASTIC_NODE_BROADCAST, "unexpected destination");
	zassert_equal(decoded.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "unexpected port");
	assert_payload(decoded.payload, decoded.payload_len, "ping", 4U);
	zassert_equal(state.last_event.type, MESHTASTIC_EVENT_TX_DONE, "unexpected event");

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.tx_packets, before.tx_packets + 1U, "tx counter not incremented");
}

/* Verifies invalid send API arguments fail before reaching the LoRa driver. */
ZTEST(protocol_stack, test_invalid_send_inputs_do_not_transmit)
{
	static uint8_t too_large_payload[MESHTASTIC_MAX_PAYLOAD_LEN + 1U];
	char too_long_text[MESHTASTIC_MAX_TEXT_LEN + 2U];
	int ret;

	memset(too_long_text, 'x', sizeof(too_long_text) - 1U);
	too_long_text[sizeof(too_long_text) - 1U] = '\0';

	ret = meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, NULL);
	zassert_equal(ret, -EINVAL, "NULL text should be rejected");
	ret = meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "");
	zassert_equal(ret, -EINVAL, "empty text should be rejected");
	ret = meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, too_long_text);
	zassert_equal(ret, -EINVAL, "oversized text should be rejected");
	ret = meshtastic_send_data(0U, MESHTASTIC_PORT_PRIVATE, NULL, 0U, K_FOREVER);
	zassert_equal(ret, -EINVAL, "zero destination should be rejected");
	ret = meshtastic_send_data(MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_PRIVATE, NULL, 1U,
				   K_FOREVER);
	zassert_equal(ret, -EINVAL, "missing payload should be rejected");
	ret = meshtastic_send_data(MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_PRIVATE,
				   too_large_payload, sizeof(too_large_payload), K_FOREVER);
	zassert_equal(ret, -EINVAL, "oversized payload should be rejected");

	assert_mock_send_count(0U);
	zassert_equal(state.event_count, 0U, "invalid sends should not emit events");
}

/* Verifies non-text data may carry an empty payload and decodes as a zero-length Data payload. */
ZTEST(protocol_stack, test_zero_length_data_payload_round_trips)
{
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	int ret;

	ret = meshtastic_send_data(MESHTASTIC_NODE_BROADCAST, MESHTASTIC_PORT_PRIVATE, NULL, 0U,
				   K_FOREVER);
	zassert_ok(ret, "empty data send failed: %d", ret);
	zassert_ok(k_sem_take(&state.tx_sem, K_SECONDS(1)), "timed out waiting for tx event");

	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_equal(decoded.portnum, MESHTASTIC_PORT_PRIVATE, "unexpected port");
	assert_payload(decoded.payload, decoded.payload_len, NULL, 0U);
}

/* Verifies explicit packet metadata is preserved in both the wire header and decoded payload. */
ZTEST(protocol_stack, test_send_packet_preserves_explicit_metadata)
{
	const uint8_t body[] = {0x10, 0x20, 0x30};
	struct meshtastic_packet packet = {
		.from = TEST_NODE_ID,
		.to = PEER_NODE_ID,
		.id = 0x44445555U,
		.portnum = MESHTASTIC_PORT_PRIVATE,
		.payload = body,
		.payload_len = sizeof(body),
		.data_dest = OTHER_NODE_ID,
		.data_source = TEST_NODE_ID,
		.request_id = 0x1010U,
		.reply_id = 0x2020U,
		.hop_limit = 5U,
		.hop_start = 6U,
		.channel_index = meshtastic_channels_primary_index(),
		.next_hop = 0x21U,
		.relay_node = 0x43U,
		.want_ack = true,
		.via_mqtt = true,
		.want_response = true,
	};
	struct meshtastic_wire_header hdr;
	struct meshtastic_packet decoded;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	int ret;

	ret = meshtastic_send_packet(&packet, K_FOREVER);
	zassert_ok(ret, "metadata packet send failed: %d", ret);
	zassert_ok(k_sem_take(&state.tx_sem, K_SECONDS(1)), "timed out waiting for tx event");

	copy_last_tx_header(&hdr);
	assert_wire_header(&hdr, TEST_NODE_ID, PEER_NODE_ID, packet.id, packet.hop_limit,
			   packet.hop_start, true, true, packet.next_hop, packet.relay_node);

	decode_last_tx(&decoded, payload, sizeof(payload));
	zassert_equal(decoded.data_dest, packet.data_dest, "unexpected Data.dest");
	zassert_equal(decoded.data_source, packet.data_source, "unexpected Data.source");
	zassert_equal(decoded.request_id, packet.request_id, "unexpected request id");
	zassert_equal(decoded.reply_id, packet.reply_id, "unexpected reply id");
	zassert_true(decoded.want_response, "want_response not preserved");
	assert_payload(decoded.payload, decoded.payload_len, body, sizeof(body));
}

/* Verifies radio send errors emit TX_FAILED and update failure status instead of TX_DONE. */
ZTEST(protocol_stack, test_radio_send_failure_emits_failed_event)
{
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	set_mock_send_result(-EIO);
	ret = meshtastic_send_text(MESHTASTIC_NODE_BROADCAST, "fail");
	zassert_equal(ret, -EIO, "send should return mock radio failure");
	zassert_ok(k_sem_take(&state.tx_sem, K_SECONDS(1)), "timed out waiting for failure event");

	assert_mock_send_count(1U);
	zassert_equal(state.last_event.type, MESHTASTIC_EVENT_TX_FAILED,
		      "unexpected failure event");
	zassert_equal(state.last_event.err, -EIO, "unexpected failure errno");

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.tx_failures, before.tx_failures + 1U,
		      "tx failure counter not incremented");
	zassert_equal(after.tx_packets, before.tx_packets, "failed TX should not count as sent");
}

/* Verifies decoded MeshPacket conversion preserves application payload and packet metadata. */
ZTEST(protocol_stack, test_mesh_packet_conversion_preserves_decoded_metadata)
{
	const uint8_t body[] = "meshpb";
	struct meshtastic_packet packet = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x5101U,
		.portnum = MESHTASTIC_PORT_PRIVATE,
		.payload = body,
		.payload_len = sizeof(body) - 1U,
		.data_dest = TEST_NODE_ID,
		.data_source = PEER_NODE_ID,
		.request_id = 0x55U,
		.reply_id = 0x66U,
		.hop_limit = 4U,
		.hop_start = 5U,
		.channel_index = meshtastic_channels_primary_index(),
		.next_hop = 0x78U,
		.relay_node = 0x87U,
		.want_ack = true,
		.via_mqtt = true,
		.want_response = true,
	};
	meshtastic_MeshPacket mesh;
	struct meshtastic_packet out;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	int ret;

	ret = meshtastic_packet_to_mesh_pb(&packet, &mesh);
	zassert_ok(ret, "packet_to_mesh_pb failed: %d", ret);
	ret = meshtastic_mesh_pb_to_packet(&mesh, &out, payload, sizeof(payload));
	zassert_ok(ret, "mesh_pb_to_packet failed: %d", ret);

	zassert_equal(out.from, packet.from, "unexpected source");
	zassert_equal(out.to, packet.to, "unexpected destination");
	zassert_equal(out.id, packet.id, "unexpected packet id");
	zassert_equal(out.portnum, packet.portnum, "unexpected port");
	zassert_equal(out.channel_index, packet.channel_index, "unexpected channel index");
	zassert_equal(out.channel, meshtastic_channels_primary_hash(), "unexpected channel hash");
	zassert_equal(out.next_hop, packet.next_hop, "unexpected next hop");
	zassert_equal(out.relay_node, packet.relay_node, "unexpected relay node");
	zassert_true(out.want_ack, "want_ack not preserved");
	zassert_true(out.via_mqtt, "via_mqtt not preserved");
	zassert_true(out.want_response, "want_response not preserved");
	assert_payload(out.payload, out.payload_len, body, sizeof(body) - 1U);
}

/* Verifies MeshPacket copies preserve encrypted union data and the active payload variant. */
ZTEST(protocol_stack, test_mesh_packet_copy_preserves_encrypted_payload)
{
	const uint8_t encrypted[] = {0xaa, 0xbb, 0xcc, 0xdd};
	meshtastic_MeshPacket src = meshtastic_MeshPacket_init_zero;
	meshtastic_MeshPacket dst = meshtastic_MeshPacket_init_zero;

	src.from = PEER_NODE_ID;
	src.to = TEST_NODE_ID;
	src.id = 0x5202U;
	src.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	src.encrypted.size = sizeof(encrypted);
	memcpy(src.encrypted.bytes, encrypted, sizeof(encrypted));

	meshtastic_mesh_packet_copy(&dst, &src);

	zassert_equal(dst.which_payload_variant, meshtastic_MeshPacket_encrypted_tag,
		      "encrypted variant not preserved");
	zassert_equal(dst.encrypted.size, sizeof(encrypted), "encrypted length not preserved");
	zassert_mem_equal(dst.encrypted.bytes, encrypted, sizeof(encrypted),
			  "encrypted bytes not preserved");
}

/* Verifies encrypted MeshPacket payloads decode with the active channel and bad payloads fail. */
ZTEST(protocol_stack, test_mesh_pb_try_decode_accepts_valid_encrypted_payload_only)
{
	const uint8_t body[] = "secret";
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	meshtastic_MeshPacket empty = meshtastic_MeshPacket_init_zero;
	meshtastic_MeshPacket bad = meshtastic_MeshPacket_init_zero;
	const struct meshtastic_wire_header *hdr;
	int ret;

	build_wire_packet(PEER_NODE_ID, TEST_NODE_ID, 0x5303U, 3U, MESHTASTIC_PORT_PRIVATE, body,
			  sizeof(body) - 1U, wire, &wire_len);

	hdr = (const struct meshtastic_wire_header *)wire;
	mesh.from = PEER_NODE_ID;
	mesh.to = TEST_NODE_ID;
	mesh.id = 0x5303U;
	mesh.channel = hdr->channel;
	mesh.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	mesh.encrypted.size = wire_len - MESHTASTIC_HDR_LEN;
	memcpy(mesh.encrypted.bytes, wire + MESHTASTIC_HDR_LEN, mesh.encrypted.size);

	ret = meshtastic_mesh_pb_try_decode(&mesh);
	zassert_ok(ret, "valid encrypted MeshPacket did not decode: %d", ret);
	zassert_equal(mesh.which_payload_variant, meshtastic_MeshPacket_decoded_tag,
		      "payload variant not switched to decoded");
	zassert_equal((uint32_t)mesh.decoded.portnum, MESHTASTIC_PORT_PRIVATE,
		      "unexpected decoded port");
	zassert_equal(mesh.decoded.payload.size, sizeof(body) - 1U, "unexpected decoded length");
	zassert_mem_equal(mesh.decoded.payload.bytes, body, sizeof(body) - 1U,
			  "unexpected decoded payload");

	empty.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	ret = meshtastic_mesh_pb_try_decode(&empty);
	zassert_true(ret < 0, "empty encrypted payload should be rejected");

	bad.from = PEER_NODE_ID;
	bad.id = 0x5304U;
	bad.channel = hdr->channel;
	bad.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
	bad.encrypted.size = 3U;
	memset(bad.encrypted.bytes, 0xa5, bad.encrypted.size);
	ret = meshtastic_mesh_pb_try_decode(&bad);
	zassert_true(ret < 0, "invalid encrypted payload should be rejected");
}

/* Verifies a valid LoRa frame addressed to this node is delivered with RSSI/SNR metadata. */
ZTEST(protocol_stack, test_mock_radio_receive_delivers_packet)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	build_peer_wire_packet(TEST_NODE_ID, 0x1001U, 3U, "pong", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -42, 7);

	zassert_ok(k_sem_take(&state.rx_sem, K_SECONDS(1)), "timed out waiting for rx callback");
	zassert_equal(state.recv_count, 1U, "expected one delivery");
	zassert_equal(state.last_rx.from, PEER_NODE_ID, "unexpected source");
	zassert_equal(state.last_rx.to, TEST_NODE_ID, "unexpected destination");
	zassert_equal(state.last_rx.portnum, MESHTASTIC_PORT_TEXT_MESSAGE, "unexpected port");
	assert_rx_payload("pong", 4U);
	zassert_equal(state.last_rx.rssi, -42, "unexpected rssi");
	zassert_equal(state.last_rx.snr, 7, "unexpected snr");
	zassert_equal(state.last_event.type, MESHTASTIC_EVENT_PACKET_RECEIVED, "unexpected event");

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.rx_packets, before.rx_packets + 1U, "rx counter not incremented");
}

/* Verifies too-short LoRa frames are ignored before duplicate, decode, or delivery handling. */
ZTEST(protocol_stack, test_too_short_rx_frame_is_ignored)
{
	uint8_t wire[MESHTASTIC_HDR_LEN - 1U] = {0};
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	inject_rx_frame(wire, sizeof(wire), -10, 1);
	k_sleep(K_MSEC(100));

	zassert_equal(state.recv_count, 0U, "short frame should not be delivered");
	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.rx_packets, before.rx_packets, "short frame should not count as RX");
	zassert_equal(after.decode_failures, before.decode_failures,
		      "short frame should not count as decode failure");
}

/* Verifies undecodable channel hashes count as decode failures without local delivery. */
ZTEST(protocol_stack, test_unknown_channel_hash_counts_decode_failure_without_delivery)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_wire_header *hdr;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	build_peer_wire_packet(TEST_NODE_ID, 0x5404U, 3U, "badch", wire, &wire_len);
	hdr = (struct meshtastic_wire_header *)wire;
	hdr->channel ^= 0xffU;
	inject_rx_frame(wire, wire_len, -20, 4);
	k_sleep(K_MSEC(100));

	zassert_equal(state.recv_count, 0U, "undecodable packet should not be delivered");
	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.rx_packets, before.rx_packets + 1U, "rx counter not incremented");
	zassert_equal(after.decode_failures, before.decode_failures + 1U,
		      "decode failure counter not incremented");
}

/* Verifies broadcast LoRa packets are delivered locally just like direct unicasts. */
ZTEST(protocol_stack, test_broadcast_rx_packet_is_delivered_locally)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	build_peer_wire_packet(MESHTASTIC_NODE_BROADCAST, 0x5505U, 0U, "all", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -33, 6);

	zassert_ok(k_sem_take(&state.rx_sem, K_SECONDS(1)), "timed out waiting for rx callback");
	zassert_equal(state.recv_count, 1U, "expected one delivery");
	zassert_equal(state.last_rx.to, MESHTASTIC_NODE_BROADCAST, "unexpected destination");
	assert_rx_payload("all", 3U);
}

/* Verifies duplicate local packets are delivered once and counted as duplicates thereafter. */
ZTEST(protocol_stack, test_duplicate_packets_are_suppressed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	build_peer_wire_packet(TEST_NODE_ID, 0x2002U, 3U, "dupe", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -30, 5);
	zassert_ok(k_sem_take(&state.rx_sem, K_SECONDS(1)), "timed out waiting for first rx");

	inject_rx_frame(wire, wire_len, -30, 5);
	zassert_equal(-EAGAIN, k_sem_take(&state.rx_sem, K_MSEC(100)),
		      "duplicate unexpectedly delivered");

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.duplicate_packets, before.duplicate_packets + 1U,
		      "duplicate counter not incremented");
	zassert_equal(state.recv_count, 1U, "expected single delivery");
}

/* Verifies a packet re-sent after the dedup TTL has elapsed is treated as fresh
 * (delivered again) instead of being suppressed forever, and that the expiry is
 * counted. Uses a 1-second TTL so the test doesn't stall. */
ZTEST(protocol_stack, test_dedup_ttl_expiry_allows_resend)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_sched_stats sst;

	zassert_ok(meshtastic_sched_set("dedup.ttl", "1"));

	build_peer_wire_packet(TEST_NODE_ID, 0x2071U, 3U, "ttl", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -30, 5);
	zassert_ok(k_sem_take(&state.rx_sem, K_SECONDS(1)), "first delivery timed out");

	/* Immediate resend is still within the TTL window -> suppressed. */
	inject_rx_frame(wire, wire_len, -30, 5);
	zassert_equal(-EAGAIN, k_sem_take(&state.rx_sem, K_MSEC(100)),
		      "resend within TTL should be suppressed");

	/* Age the entry out, then resend -> treated as fresh -> delivered again. */
	k_sleep(K_MSEC(1200));
	inject_rx_frame(wire, wire_len, -30, 5);
	zassert_ok(k_sem_take(&state.rx_sem, K_SECONDS(1)),
		   "resend after TTL expiry should be delivered again");

	zassert_equal(state.recv_count, 2U, "expected two deliveries across the TTL boundary");
	meshtastic_sched_stats_get(&sst);
	zassert_true(sst.dedup_expired >= 1U, "dedup TTL expiry should be counted");
}

/* Verifies the TraceRoute responder replies to a want_response RouteDiscovery
 * addressed to us: on TRACEROUTE_APP, back to the requester, correlated by
 * request_id, with our RX SNR appended to snr_towards (dB scaled by 4) and no
 * self-insertion into route[]. */
ZTEST(protocol_stack, test_traceroute_responder_replies_with_snr)
{
	meshtastic_RouteDiscovery rd = meshtastic_RouteDiscovery_init_zero;
	uint8_t rd_buf[64];
	pb_ostream_t os = pb_ostream_from_buffer(rd_buf, sizeof(rd_buf));
	struct meshtastic_packet req = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x7A01U,
		.portnum = MESHTASTIC_PORT_TRACEROUTE,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_response = true,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet reply;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_RouteDiscovery got = meshtastic_RouteDiscovery_init_zero;
	pb_istream_t is;
	const int8_t rx_snr = 5;

	/* Empty RouteDiscovery = a direct trace, no intermediate hops recorded yet. */
	zassert_true(pb_encode(&os, meshtastic_RouteDiscovery_fields, &rd),
		     "RouteDiscovery encode failed");
	req.payload = rd_buf;
	req.payload_len = os.bytes_written;

	zassert_ok(meshtastic_build_wire_packet(&req, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, rx_snr);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(1U);
	decode_last_tx(&reply, payload, sizeof(payload));

	zassert_equal(reply.portnum, MESHTASTIC_PORT_TRACEROUTE, "reply not on TRACEROUTE port");
	zassert_equal(reply.to, PEER_NODE_ID, "reply must return to the requester");
	zassert_equal(reply.from, TEST_NODE_ID, "reply must originate from us");
	zassert_equal(reply.request_id, req.id, "reply must correlate via request_id");
	zassert_false(reply.want_response, "reply must not itself request a response");

	is = pb_istream_from_buffer(payload, reply.payload_len);
	zassert_true(pb_decode(&is, meshtastic_RouteDiscovery_fields, &got),
		     "reply payload must be a RouteDiscovery");
	zassert_equal(got.route_count, 0, "destination must not add itself to route[]");
	zassert_equal(got.snr_towards_count, 1, "destination appends exactly one snr_towards");
	zassert_equal(got.snr_towards[0], (int)rx_snr * 4, "snr_towards must be RX snr scaled x4");
}

/* Verifies a TRACEROUTE request addressed to us but WITHOUT want_response draws
 * no reply (the request is consumed locally, nothing is transmitted). */
ZTEST(protocol_stack, test_traceroute_no_reply_without_want_response)
{
	meshtastic_RouteDiscovery rd = meshtastic_RouteDiscovery_init_zero;
	uint8_t rd_buf[64];
	pb_ostream_t os = pb_ostream_from_buffer(rd_buf, sizeof(rd_buf));
	struct meshtastic_packet req = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x7A02U,
		.portnum = MESHTASTIC_PORT_TRACEROUTE,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_response = false,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_true(pb_encode(&os, meshtastic_RouteDiscovery_fields, &rd), "encode failed");
	req.payload = rd_buf;
	req.payload_len = os.bytes_written;

	zassert_ok(meshtastic_build_wire_packet(&req, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(0U);
}

/* Verifies the responder APPENDS to an already-accumulated request (as it would
 * arrive relayed through other nodes) rather than overwriting: a pre-existing
 * route[] / snr_towards[] entry is preserved and our RX SNR is added after it. */
ZTEST(protocol_stack, test_traceroute_appends_to_accumulated_route)
{
	meshtastic_RouteDiscovery rd = meshtastic_RouteDiscovery_init_zero;
	uint8_t rd_buf[64];
	pb_ostream_t os = pb_ostream_from_buffer(rd_buf, sizeof(rd_buf));
	struct meshtastic_packet req = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x7A03U,
		.portnum = MESHTASTIC_PORT_TRACEROUTE,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_response = true,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet reply;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_RouteDiscovery got = meshtastic_RouteDiscovery_init_zero;
	pb_istream_t is;
	const int8_t rx_snr = 5;

	/* One prior hop already recorded (node 0xAAAA1111 relayed at 8 dB = 32 q4). */
	rd.route[0] = 0xAAAA1111U;
	rd.route_count = 1;
	rd.snr_towards[0] = 8 * 4;
	rd.snr_towards_count = 1;
	zassert_true(pb_encode(&os, meshtastic_RouteDiscovery_fields, &rd), "encode failed");
	req.payload = rd_buf;
	req.payload_len = os.bytes_written;

	zassert_ok(meshtastic_build_wire_packet(&req, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, rx_snr);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(1U);
	decode_last_tx(&reply, payload, sizeof(payload));

	is = pb_istream_from_buffer(payload, reply.payload_len);
	zassert_true(pb_decode(&is, meshtastic_RouteDiscovery_fields, &got), "decode failed");
	zassert_equal(got.route_count, 1, "existing route must be preserved (we don't add ourselves)");
	zassert_equal(got.route[0], 0xAAAA1111U, "prior hop id must survive");
	zassert_equal(got.snr_towards_count, 2, "our snr appended after the prior hop's");
	zassert_equal(got.snr_towards[0], 8 * 4, "prior snr preserved");
	zassert_equal(got.snr_towards[1], (int)rx_snr * 4, "our snr appended last");
}

/* Send a want_ack unicast text DM from us to PEER with an explicit id. */
static void send_reliable_dm(uint32_t id)
{
	struct meshtastic_packet dm = {
		.from = 0U, /* filled with our node id */
		.to = PEER_NODE_ID,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_ack = true,
		.channel_index = meshtastic_channels_primary_index(),
	};

	zassert_ok(meshtastic_send_packet(&dm, K_NO_WAIT));
}

/* Inject a ROUTING ACK (or NAK) from PEER to us correlating request_id, with an
 * explicit relay_node byte (0 = arrived direct). */
static void inject_routing_ack_via(uint32_t request_id, meshtastic_Routing_Error err,
				   uint8_t relay_node)
{
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	uint8_t rbuf[16];
	pb_ostream_t os = pb_ostream_from_buffer(rbuf, sizeof(rbuf));
	struct meshtastic_packet ack = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = request_id ^ 0xACC0U,
		.portnum = MESHTASTIC_PORT_ROUTING,
		.request_id = request_id,
		.hop_limit = 3U,
		.hop_start = 3U,
		.relay_node = relay_node,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	routing.which_variant = meshtastic_Routing_error_reason_tag;
	routing.error_reason = err;
	zassert_true(pb_encode(&os, meshtastic_Routing_fields, &routing), "ack encode failed");
	ack.payload = rbuf;
	ack.payload_len = os.bytes_written;
	zassert_ok(meshtastic_build_wire_packet(&ack, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, 5);
}

static void inject_routing_ack(uint32_t request_id, meshtastic_Routing_Error err)
{
	inject_routing_ack_via(request_id, err, 0U);
}

/* Verifies an explicit ROUTING ACK for a want_ack DM we sent cancels
 * retransmission and counts as delivered. */
ZTEST(protocol_stack, test_reliable_explicit_ack_cancels_retransmit)
{
	struct meshtastic_sched_stats st;

	zassert_ok(meshtastic_sched_set("reliable.retries", "3"));
	zassert_ok(meshtastic_sched_set("reliable.timeout", "120"));

	send_reliable_dm(0x9101U);
	k_sleep(K_MSEC(30));
	assert_mock_send_count(1U); /* original transmit only */

	inject_routing_ack(0x9101U, meshtastic_Routing_Error_NONE);
	k_sleep(K_MSEC(30)); /* processed well before the 120 ms retry */

	k_sleep(K_MSEC(250)); /* past several retry intervals */
	assert_mock_send_count(1U); /* no retransmissions */

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.reliable_acked, 1U, "ack should mark delivered");
	zassert_equal(st.reliable_failed, 0U);
}

/* Verifies a want_ack DM with no ACK is retransmitted reliable.retries times and
 * then reported as failed. */
ZTEST(protocol_stack, test_reliable_retransmits_then_fails)
{
	struct meshtastic_sched_stats st;

	zassert_ok(meshtastic_sched_set("reliable.retries", "2"));
	zassert_ok(meshtastic_sched_set("reliable.timeout", "100"));

	send_reliable_dm(0x9201U);
	/* original at t~0, retransmits at ~100 and ~200 ms, exhaust at ~300 ms. */
	k_sleep(K_MSEC(450));

	assert_mock_send_count(3U); /* 1 original + 2 retransmits */
	meshtastic_sched_stats_get(&st);
	zassert_equal(st.reliable_failed, 1U, "exhaustion should count as failed");
	zassert_equal(st.reliable_acked, 0U);
}

/* Verifies hearing our own packet rebroadcast (implicit ACK) cancels
 * retransmission. */
ZTEST(protocol_stack, test_reliable_implicit_ack_cancels_retransmit)
{
	struct meshtastic_sched_stats st;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_sched_set("reliable.retries", "3"));
	zassert_ok(meshtastic_sched_set("reliable.timeout", "120"));

	send_reliable_dm(0x9301U);
	k_sleep(K_MSEC(30));
	assert_mock_send_count(1U);

	/* A neighbour rebroadcasts our packet: same id, wire src is our node id. */
	build_wire_packet(TEST_NODE_ID, PEER_NODE_ID, 0x9301U, 2U, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)"hi", 2U, wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(30));

	k_sleep(K_MSEC(250));
	assert_mock_send_count(1U); /* implicit ack stopped retransmission */

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.reliable_acked, 1U, "implicit ack should stop retransmission");
	zassert_equal(st.reliable_failed, 0U);
}

/* Verifies a repeated reliable DM to us (the originator retransmitted because
 * our first ACK was lost — hop_start == hop_limit) is re-ACKed straight from
 * the dedup path without a second app delivery, while a relayed duplicate copy
 * (hop_limit already decremented) stays a plain drop. */
ZTEST(protocol_stack, test_repeated_reliable_dm_is_reacked_not_redelivered)
{
	struct meshtastic_packet dm = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x8801U,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_ack = true,
		.channel_index = meshtastic_channels_primary_index(),
	};
	struct meshtastic_wire_header *whdr;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet ack;
	uint8_t ack_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	pb_istream_t is;

	zassert_ok(meshtastic_build_wire_packet(&dm, wire, &wire_len));

	/* First copy: delivered (and ACKed on the decoded path). */
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(100));
	zassert_equal(state.recv_count, 1U, "first copy must deliver");

	/* Identical retransmission: re-ACK from the dup path, no re-delivery. */
	reset_mock_lora();
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(100));

	zassert_equal(state.recv_count, 1U, "duplicate must not re-deliver");
	decode_last_tx(&ack, ack_payload, sizeof(ack_payload));
	zassert_equal(ack.portnum, MESHTASTIC_PORT_ROUTING, "re-ACK should be a ROUTING packet");
	zassert_equal(ack.to, PEER_NODE_ID, "re-ACK should target the sender");
	zassert_equal(ack.from, TEST_NODE_ID, "re-ACK should originate from us");
	zassert_equal(ack.request_id, dm.id, "re-ACK should reference the DM id");
	is = pb_istream_from_buffer(ack_payload, ack.payload_len);
	zassert_true(pb_decode(&is, meshtastic_Routing_fields, &routing), "routing decode");
	zassert_equal(routing.error_reason, meshtastic_Routing_Error_NONE,
		      "re-ACK should carry error NONE");

	/* A relayed duplicate (hop_limit != hop_start) is a plain dupe: no TX. */
	reset_mock_lora();
	whdr = (struct meshtastic_wire_header *)wire;
	whdr->flags = (whdr->flags & ~MESHTASTIC_FLAGS_HOP_LIMIT_MASK) | 2U;
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(100));
	assert_mock_send_count(0U);
	zassert_equal(state.recv_count, 1U, "relayed dupe must not re-deliver");
}

/* Verifies a duplicate arriving with strictly more hops left is relayed once as
 * an upgraded copy (wider reach), while later copies at or below the stored
 * hop budget drop as plain dupes (H3, upstream PacketHistory hop upgrade). */
ZTEST(protocol_stack, test_hop_limit_upgraded_duplicate_relays_once)
{
	uint8_t wire2[MESHTASTIC_PKT_MAX];
	uint8_t wire3[MESHTASTIC_PKT_MAX];
	uint32_t wire2_len;
	uint32_t wire3_len;
	struct meshtastic_wire_header hdr;
	struct meshtastic_status base;
	struct meshtastic_status after;

	/* Prime PEER as known (has_user) so no NodeInfo-request TX interleaves
	 * with the relay assertions below. */
	{
		meshtastic_User user = meshtastic_User_init_zero;
		uint8_t ni_buf[64];
		pb_ostream_t os = pb_ostream_from_buffer(ni_buf, sizeof(ni_buf));
		uint8_t ni_wire[MESHTASTIC_PKT_MAX];
		uint32_t ni_wire_len;

		strcpy(user.id, "!87654321");
		strcpy(user.long_name, "Peer");
		strcpy(user.short_name, "P1");
		zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
		build_wire_packet(PEER_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0xA301U, 1U,
				  MESHTASTIC_PORT_NODEINFO, ni_buf, os.bytes_written, ni_wire,
				  &ni_wire_len);
		inject_rx_frame(ni_wire, ni_wire_len, -40, 5);
		k_sleep(K_MSEC(100));
	}

	zassert_ok(meshtastic_get_status(&base));

	/* First sighting at hop_limit=2: normal relay, hop 1 on the wire. */
	build_wire_packet(PEER_NODE_ID, OTHER_NODE_ID, 0xA302U, 2U, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)"up", 2U, wire2, &wire2_len);
	inject_rx_frame(wire2, wire2_len, -40, 5);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, base.relayed_packets + 1U, "first copy must relay");
	copy_last_tx_header(&hdr);
	zassert_equal(hdr.flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 1U,
		      "first relay should carry hop_limit-1");

	/* Same (src,id) with hop_limit=3: one upgraded relay, hop 2 on the wire. */
	build_wire_packet(PEER_NODE_ID, OTHER_NODE_ID, 0xA302U, 3U, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)"up", 2U, wire3, &wire3_len);
	inject_rx_frame(wire3, wire3_len, -40, 5);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, base.relayed_packets + 2U,
		      "hop-upgraded duplicate must relay once");
	zassert_equal(after.duplicate_packets, base.duplicate_packets,
		      "an upgrade is not a plain duplicate");
	copy_last_tx_header(&hdr);
	zassert_equal(hdr.flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 2U,
		      "upgraded relay should carry the higher hop_limit-1");

	/* Equal hop budget again: plain dupe, no further relay. */
	inject_rx_frame(wire3, wire3_len, -40, 5);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, base.relayed_packets + 2U,
		      "equal-hop duplicate must not relay");
	zassert_equal(after.duplicate_packets, base.duplicate_packets + 1U,
		      "equal-hop duplicate counts as a dupe");

	/* Lower hop budget: plain dupe as well. */
	inject_rx_frame(wire2, wire2_len, -40, 5);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, base.relayed_packets + 2U,
		      "lower-hop duplicate must not relay");
	zassert_equal(after.duplicate_packets, base.duplicate_packets + 2U,
		      "lower-hop duplicate counts as a dupe");

	/* Only the NodeInfo primer was addressed to us (broadcast): the upgrade
	 * path never re-delivers to the app. */
	zassert_equal(state.recv_count, 1U, "upgrade must not re-deliver");
}

/* Inject an on-air NodeInfo broadcast from @p from carrying @p user. */
static void inject_nodeinfo(uint32_t from, uint32_t id, const meshtastic_User *user)
{
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_true(pb_encode(&os, meshtastic_User_fields, user), "User encode failed");
	build_wire_packet(from, MESHTASTIC_NODE_BROADCAST, id, 1U, MESHTASTIC_PORT_NODEINFO, buf,
			  os.bytes_written, wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(50));
}

/* B-1: once a peer's 32-byte public key is pinned, a NodeInfo carrying a
 * different key must not replace it (or the identity it authenticates), and a
 * keyless NodeInfo must not wipe it. A matching key keeps updating normally. */
ZTEST(protocol_stack, test_nodeinfo_pubkey_pinning_refuses_key_change)
{
	const uint32_t node = 0x22334455U;
	struct meshtastic_nodedb_node snap;
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t key_a[32];
	uint8_t key_b[32];

	memset(key_a, 0xA5, sizeof(key_a));
	memset(key_b, 0x5B, sizeof(key_b));

	/* First NodeInfo pins key A. */
	strcpy(user.id, "!22334455");
	strcpy(user.long_name, "Genuine");
	strcpy(user.short_name, "GN");
	user.public_key.size = 32U;
	memcpy(user.public_key.bytes, key_a, 32U);
	inject_nodeinfo(node, 0xB201U, &user);

	zassert_ok(meshtastic_nodedb_get(node, &snap));
	zassert_equal(snap.public_key_len, 32U, "key A should be stored");
	zassert_mem_equal(snap.public_key, key_a, 32U, "key A should be stored");

	/* Spoofed NodeInfo with key B: key AND identity update refused. */
	strcpy(user.long_name, "Impostor");
	strcpy(user.short_name, "IM");
	memcpy(user.public_key.bytes, key_b, 32U);
	inject_nodeinfo(node, 0xB202U, &user);

	zassert_ok(meshtastic_nodedb_get(node, &snap));
	zassert_equal(snap.public_key_len, 32U, "pinned key must survive");
	zassert_mem_equal(snap.public_key, key_a, 32U, "pinned key must not be replaced");
	zassert_equal(strcmp(snap.long_name, "Genuine"), 0,
		      "a mismatched key must not rename the node");

	/* Keyless NodeInfo: benign metadata applies, pinned key kept. */
	strcpy(user.long_name, "Renamed");
	strcpy(user.short_name, "RN");
	user.public_key.size = 0U;
	inject_nodeinfo(node, 0xB203U, &user);

	zassert_ok(meshtastic_nodedb_get(node, &snap));
	zassert_equal(snap.public_key_len, 32U, "keyless NodeInfo must not wipe the pinned key");
	zassert_mem_equal(snap.public_key, key_a, 32U, "pinned key kept");
	zassert_equal(strcmp(snap.long_name, "Renamed"), 0,
		      "keyless NodeInfo still updates benign metadata");

	/* The same key A again: normal updates keep flowing. */
	strcpy(user.long_name, "Genuine2");
	user.public_key.size = 32U;
	memcpy(user.public_key.bytes, key_a, 32U);
	inject_nodeinfo(node, 0xB204U, &user);

	zassert_ok(meshtastic_nodedb_get(node, &snap));
	zassert_equal(strcmp(snap.long_name, "Genuine2"), 0, "matching key updates apply");
}

/* --- Next-hop route learning (increment 3) -------------------------------- */

/* Inject a decoded TEXT frame from @p from to @p to carrying an explicit
 * relay_node byte (the neighbour that last relayed it toward us). */
static void inject_with_relay(uint32_t from, uint32_t to, uint32_t id, uint8_t relay_node)
{
	struct meshtastic_packet p = {
		.from = from,
		.to = to,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.relay_node = relay_node,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_build_wire_packet(&p, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, 5);
}

/* Drive the full (M2-gated) learn round-trip for a route to PEER via
 * @p relayer_num: prime the relayer as a known node, send a want_ack DM
 * (id @p dm_id), overhear our own DM rebroadcast by that relayer (two-way
 * correlation evidence), then take PEER's ROUTING ACK carried by the same
 * relayer. Leaves next_hop(PEER) == low byte of @p relayer_num. */
static void learn_route_via(uint32_t dm_id, uint32_t relayer_num)
{
	uint8_t rbyte = (uint8_t)(relayer_num & 0xFFU);
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_wire_header *whdr;

	/* The relayer must be a known node for unique last-byte resolution. */
	inject_with_relay(relayer_num, MESHTASTIC_NODE_BROADCAST, dm_id ^ 0x10000U, 0U);
	k_sleep(K_MSEC(30));

	send_reliable_dm(dm_id);
	k_sleep(K_MSEC(30));

	/* Our own DM echoed back, rebroadcast by the relayer. */
	build_wire_packet(TEST_NODE_ID, PEER_NODE_ID, dm_id, 2U, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)"hi", 2U, wire, &wire_len);
	whdr = (struct meshtastic_wire_header *)wire;
	whdr->relay_node = rbyte;
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(30));

	/* PEER's ACK for the DM, carried by the same relayer. */
	inject_routing_ack_via(dm_id, meshtastic_Routing_Error_NONE, rbyte);
	k_sleep(K_MSEC(30));
}

/* M2 positive path: an ACK correlated to our own send (request_id), carried by
 * a relayer that provably also relayed our original (we overheard the echo) and
 * whose last byte resolves to exactly one known node, teaches the next hop. */
ZTEST(protocol_stack, test_learn_next_hop_from_correlated_ack)
{
	/* OTHER_NODE_ID low byte 0x68 — unique among known nodes. */
	learn_route_via(0xC101U, OTHER_NODE_ID);

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU),
		      "correlated + unique relayer should be learned as the next hop");
}

/* M2: a plain unicast to us (not an ACK/reply to anything we sent) must no
 * longer teach a route — no correlation, nothing to trust. */
ZTEST(protocol_stack, test_no_learn_next_hop_from_plain_unicast)
{
	/* Clear any route to PEER left behind by other tests. */
	(void)meshtastic_nodedb_set_next_hop(PEER_NODE_ID, 0U);

	inject_with_relay(PEER_NODE_ID, TEST_NODE_ID, 0xC111U, 0x99U);
	k_sleep(K_MSEC(30));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0U,
		      "an uncorrelated unicast must not set a next hop");
}

/* M2: an ACK whose relayer never relayed our original (no overheard echo) must
 * not teach a route, even if the byte is unique. */
ZTEST(protocol_stack, test_no_learn_next_hop_without_correlation)
{
	/* Clear any route to PEER left behind by other tests. */
	(void)meshtastic_nodedb_set_next_hop(PEER_NODE_ID, 0U);

	/* Make OTHER known so the byte would resolve uniquely. */
	inject_with_relay(OTHER_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0xC120U, 0U);
	k_sleep(K_MSEC(30));

	send_reliable_dm(0xC121U);
	k_sleep(K_MSEC(30));

	/* ACK arrives via OTHER's byte, but we never heard OTHER relay the DM. */
	inject_routing_ack_via(0xC121U, meshtastic_Routing_Error_NONE,
			       (uint8_t)(OTHER_NODE_ID & 0xFFU));
	k_sleep(K_MSEC(30));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0U,
		      "an uncorrelated ACK relayer must not be learned");
}

/* M2: a relayer byte shared by two known nodes is ambiguous and must not be
 * learned even when fully correlated. */
ZTEST(protocol_stack, test_no_learn_next_hop_ambiguous_relayer_byte)
{
	/* Clear any route to PEER left behind by other tests. */
	(void)meshtastic_nodedb_set_next_hop(PEER_NODE_ID, 0U);

	/* A second known node sharing OTHER's low byte 0x68. */
	inject_with_relay(0x44556668U, MESHTASTIC_NODE_BROADCAST, 0xC130U, 0U);
	k_sleep(K_MSEC(30));

	learn_route_via(0xC131U, OTHER_NODE_ID);

	uint8_t learned = meshtastic_nodedb_get_next_hop(PEER_NODE_ID);

	/* Un-pollute the shared NodeDB before asserting: later tests rely on
	 * OTHER's low byte resolving uniquely again. */
	zassert_ok(meshtastic_nodedb_remove(0x44556668U));

	zassert_equal(learned, 0U, "an ambiguous relayer byte must not be learned");
}

/* Broadcasts have no confirmed return path, so they must not set a next hop. */
ZTEST(protocol_stack, test_no_learn_next_hop_from_broadcast)
{
	/* Establish a known route first. */
	learn_route_via(0xC141U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU), "baseline learn failed");

	/* A broadcast from the same node with a different relay_node must not change it. */
	inject_with_relay(PEER_NODE_ID, MESHTASTIC_NODE_BROADCAST, 0xC142U, 0x22U);
	k_sleep(K_MSEC(30));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU),
		      "a broadcast must not overwrite the learned next hop");
}

/* A relay_node equal to our own low byte would mean routing through ourselves;
 * it must be rejected rather than learned. */
ZTEST(protocol_stack, test_no_learn_next_hop_when_relay_is_self)
{
	uint8_t self_byte = (uint8_t)(TEST_NODE_ID & 0xFFU);
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_wire_header *whdr;

	/* Establish a known route first. */
	learn_route_via(0xC151U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU), "baseline learn failed");

	/* A full round trip whose relayer byte is our own must not change it. */
	send_reliable_dm(0xC152U);
	k_sleep(K_MSEC(30));
	build_wire_packet(TEST_NODE_ID, PEER_NODE_ID, 0xC152U, 2U, MESHTASTIC_PORT_TEXT_MESSAGE,
			  (const uint8_t *)"hi", 2U, wire, &wire_len);
	whdr = (struct meshtastic_wire_header *)wire;
	whdr->relay_node = self_byte;
	inject_rx_frame(wire, wire_len, -40, 5);
	k_sleep(K_MSEC(30));
	inject_routing_ack_via(0xC152U, meshtastic_Routing_Error_NONE, self_byte);
	k_sleep(K_MSEC(30));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU),
		      "our own low byte must not be learned as a next hop");
}

/* A packet delivered via the MQTT gateway never rode the LoRa air, so its
 * relay_node says nothing about topology and must not be learned. */
ZTEST(protocol_stack, test_no_learn_next_hop_from_mqtt_downlink)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	int ret;

	/* Establish a known route first. */
	learn_route_via(0xC161U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU), "baseline learn failed");

	/* An MQTT downlink to us from the same node, carrying a different relay_node. */
	mesh.from = PEER_NODE_ID;
	mesh.to = TEST_NODE_ID;
	mesh.id = 0xC162U;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.hop_limit = 3U;
	mesh.hop_start = 3U;
	mesh.relay_node = 0x44U;
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = MESHTASTIC_PORT_TEXT_MESSAGE;
	memcpy(mesh.decoded.payload.bytes, "hi", 2U);
	mesh.decoded.payload.size = 2U;

	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	zassert_ok(ret, "downlink inject failed: %d", ret);
	k_sleep(K_MSEC(30));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID),
		      (uint8_t)(OTHER_NODE_ID & 0xFFU),
		      "an MQTT-injected packet must not overwrite the learned next hop");
}

/* M4: repeated delivery failures decay a learned route back to flood — but a
 * single exhausted send must not (3-strike tolerance, upstream RouteHealth). */
ZTEST(protocol_stack, test_reliable_failures_decay_learned_next_hop)
{
	uint8_t rbyte = (uint8_t)(OTHER_NODE_ID & 0xFFU);

	learn_route_via(0xC171U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte, "baseline learn failed");

	zassert_ok(meshtastic_sched_set("reliable.retries", "1"));
	zassert_ok(meshtastic_sched_set("reliable.timeout", "100"));

	/* First exhausted send: one strike, route still trusted. */
	send_reliable_dm(0xC172U);
	k_sleep(K_MSEC(350));
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte,
		      "one failure must not drop the route");

	/* Second strike: still trusted. */
	send_reliable_dm(0xC173U);
	k_sleep(K_MSEC(350));
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte,
		      "two failures must not drop the route");

	/* Third strike: decay to flood. */
	send_reliable_dm(0xC174U);
	k_sleep(K_MSEC(350));
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0U,
		      "three failures should decay the route to flood");
}

/* M4: a stale route (past route.ttl) is decayed to flood at read time. */
ZTEST(protocol_stack, test_route_ttl_decay_returns_to_flood)
{
	uint8_t rbyte = (uint8_t)(OTHER_NODE_ID & 0xFFU);

	learn_route_via(0xC181U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte, "baseline learn failed");

	zassert_ok(meshtastic_sched_set("route.ttl", "1"));
	/* Decay compares truncated seconds with a strict >, so clearing a ttl of 1
	 * needs the second-counter to advance by 2. A 1500 ms sleep only achieves
	 * that when the learn happened late in its second — it passed or failed on
	 * sub-second phase alone, and adding unrelated tests ahead of this one was
	 * enough to flip it. Sleep past the boundary regardless of phase. */
	k_sleep(K_MSEC(2500));

	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0U,
		      "a stale route must decay to flood");
}

/* M4: a delivered send resets the failure strike count. */
ZTEST(protocol_stack, test_route_success_resets_failure_count)
{
	uint8_t rbyte = (uint8_t)(OTHER_NODE_ID & 0xFFU);

	learn_route_via(0xC191U, OTHER_NODE_ID);
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte, "baseline learn failed");

	zassert_ok(meshtastic_sched_set("reliable.retries", "1"));
	zassert_ok(meshtastic_sched_set("reliable.timeout", "100"));

	/* Two strikes... */
	send_reliable_dm(0xC192U);
	k_sleep(K_MSEC(350));
	send_reliable_dm(0xC193U);
	k_sleep(K_MSEC(350));
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte,
		      "two failures: route still trusted");

	/* ...then a delivered send resets the count... */
	send_reliable_dm(0xC194U);
	k_sleep(K_MSEC(30));
	inject_routing_ack(0xC194U, meshtastic_Routing_Error_NONE);
	k_sleep(K_MSEC(50));

	/* ...so two MORE failures still leave the route trusted. */
	send_reliable_dm(0xC195U);
	k_sleep(K_MSEC(350));
	send_reliable_dm(0xC196U);
	k_sleep(K_MSEC(350));
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), rbyte,
		      "a delivered send should have reset the strike count");
}

/* A want_ack unicast addressed to us that we cannot decode is NAKed back to the
 * sender with a ROUTING error (NO_CHANNEL here — PKI is off in this build), so the
 * sender learns the reason instead of timing out. */
ZTEST(protocol_stack, test_undecodable_want_ack_dm_nak_to_sender)
{
	struct meshtastic_packet dm = {
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.id = 0x9A01U,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_ack = true,
		.channel_index = meshtastic_channels_primary_index(),
	};
	struct meshtastic_wire_header *hdr;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_packet nak;
	uint8_t nak_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_Routing routing = meshtastic_Routing_init_zero;
	pb_istream_t is;

	zassert_ok(meshtastic_build_wire_packet(&dm, wire, &wire_len));
	/* Corrupt the channel hash so no PSK we hold matches -> undecodable. */
	hdr = (struct meshtastic_wire_header *)wire;
	hdr->channel ^= 0xffU;

	inject_rx_frame(wire, wire_len, -20, 4);
	k_sleep(K_MSEC(50));

	/* Exactly one TX: the ROUTING NAK back to the sender. */
	assert_mock_send_count(1U);
	decode_last_tx(&nak, nak_payload, sizeof(nak_payload));
	zassert_equal(nak.from, TEST_NODE_ID, "NAK should originate from us");
	zassert_equal(nak.to, PEER_NODE_ID, "NAK should target the sender");
	zassert_equal(nak.portnum, MESHTASTIC_PORT_ROUTING, "NAK should be a ROUTING packet");
	zassert_equal(nak.request_id, dm.id, "NAK should reference the failed packet id");

	is = pb_istream_from_buffer(nak_payload, nak.payload_len);
	zassert_true(pb_decode(&is, meshtastic_Routing_fields, &routing), "routing payload decode");
	zassert_equal(routing.error_reason, meshtastic_Routing_Error_NO_CHANNEL,
		      "NAK should carry NO_CHANNEL");
}

/* A want_ack BROADCAST we cannot decode must NOT be NAKed (no unicast sender to
 * answer, and NAKing broadcasts would storm the mesh). It is still flood-relayed,
 * so the single TX must be that relay (to BROADCAST), not a NAK to the sender. */
ZTEST(protocol_stack, test_undecodable_broadcast_is_not_naked)
{
	struct meshtastic_packet bc = {
		.from = PEER_NODE_ID,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = 0x9A02U,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = 3U,
		.hop_start = 3U,
		.want_ack = true,
		.channel_index = meshtastic_channels_primary_index(),
	};
	struct meshtastic_wire_header *hdr;
	struct meshtastic_wire_header tx_hdr;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_build_wire_packet(&bc, wire, &wire_len));
	hdr = (struct meshtastic_wire_header *)wire;
	hdr->channel ^= 0xffU;

	inject_rx_frame(wire, wire_len, -20, 4);
	k_sleep(K_MSEC(50));

	/* Exactly one TX and it is the relayed broadcast, never a unicast NAK. */
	assert_mock_send_count(1U);
	copy_last_tx_header(&tx_hdr);
	zassert_equal(sys_le32_to_cpu(tx_hdr.dest), MESHTASTIC_NODE_BROADCAST,
		      "broadcast should be relayed, not NAKed");
	zassert_equal(sys_le32_to_cpu(tx_hdr.src), PEER_NODE_ID,
		      "relayed frame keeps the original sender");
}

/* A device/config factory reset clears every peer: only the local node survives,
 * and the learned routes toward the wiped peers are gone. */
ZTEST(protocol_stack, test_nodedb_reset_removes_all_peers)
{
	/* Populate the DB with two peers and plant a route to each (learning is
	 * correlation-gated — M2 — so set the routes directly). */
	inject_with_relay(PEER_NODE_ID, TEST_NODE_ID, 0xD100U, 0x99U);
	inject_with_relay(OTHER_NODE_ID, TEST_NODE_ID, 0xD101U, 0x33U);
	k_sleep(K_MSEC(30));
	zassert_ok(meshtastic_nodedb_set_next_hop(PEER_NODE_ID, 0x99U), "peer route not set");
	zassert_ok(meshtastic_nodedb_set_next_hop(OTHER_NODE_ID, 0x33U), "other route not set");

	meshtastic_nodedb_reset(false);

	/* Both peers gone -> no route survives; only the local node remains. */
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0U,
		      "peer should be gone after reset");
	zassert_equal(meshtastic_nodedb_get_next_hop(OTHER_NODE_ID), 0U,
		      "other should be gone after reset");
	zassert_equal(meshtastic_nodedb_count(), 1U, "only the local node should remain");
}

/* A node-db reset with keep_favorites spares favorited peers (mirrors
 * NodeDB::resetNodes) while still evicting the rest. */
ZTEST(protocol_stack, test_nodedb_reset_keeps_favorites)
{
	inject_with_relay(PEER_NODE_ID, TEST_NODE_ID, 0xD110U, 0x99U);
	inject_with_relay(OTHER_NODE_ID, TEST_NODE_ID, 0xD111U, 0x33U);
	k_sleep(K_MSEC(30));
	zassert_ok(meshtastic_nodedb_set_next_hop(PEER_NODE_ID, 0x99U), "peer route not set");
	zassert_ok(meshtastic_nodedb_set_next_hop(OTHER_NODE_ID, 0x33U), "other route not set");
	zassert_ok(meshtastic_nodedb_set_favorite(PEER_NODE_ID, true), "favorite set failed");

	meshtastic_nodedb_reset(true);

	/* The favorite (and its planted route) survives; the non-favorite is gone. */
	zassert_equal(meshtastic_nodedb_get_next_hop(PEER_NODE_ID), 0x99U,
		      "favorite peer should be retained across reset");
	zassert_equal(meshtastic_nodedb_get_next_hop(OTHER_NODE_ID), 0U,
		      "non-favorite peer should be evicted");
	zassert_equal(meshtastic_nodedb_count(), 2U, "self + one favorite should remain");

	/* Leave the DB clean for later tests. */
	(void)meshtastic_nodedb_set_favorite(PEER_NODE_ID, false);
	meshtastic_nodedb_reset(false);
}

/* --- Flood-redundancy measurement ------------------------------------------ */

/* A broadcast with independently-set hop budget and relayer byte. hop_start
 * greater than hop_limit is what distinguishes a relayed copy from one straight
 * off the originator. */
static void inject_broadcast_hops(uint32_t from, uint32_t id, uint8_t hop_limit, uint8_t hop_start,
				  uint8_t relay_node)
{
	struct meshtastic_packet p = {
		.from = from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = id,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
		.hop_limit = hop_limit,
		.hop_start = hop_start,
		.relay_node = relay_node,
		.channel_index = meshtastic_channels_primary_index(),
	};
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	zassert_ok(meshtastic_build_wire_packet(&p, wire, &wire_len));
	inject_rx_frame(wire, wire_len, -40, 5);
}

/* We relay a broadcast, then hear a peer relay the same frame: that relay of
 * ours was redundant, and is the thing a contention delay plus overhear-cancel
 * would have saved. */
ZTEST(protocol_stack, test_peer_relay_of_our_relay_is_counted_redundant)
{
	struct meshtastic_sched_stats st;

	meshtastic_sched_stats_reset();

	/* Straight from the originator (hop_start == hop_limit): we relay it. */
	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70001U, 3U, 3U, 0U);
	k_sleep(K_MSEC(30));

	meshtastic_sched_stats_get(&st);
	zassert_true(st.relay_sent >= 1U, "expected to have relayed the broadcast");
	zassert_equal(st.relay_redundant, 0U, "nothing redundant yet");

	/* The same frame, now relayed by another node (hop_limit decremented, a
	 * relayer byte that is not ours). */
	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70001U, 2U, 3U, 0x99U);
	k_sleep(K_MSEC(30));

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.relay_redundant, 1U, "peer's relay of our relay should be counted");
	{
		uint32_t bucketed = 0U;

		for (int i = 0; i < MT_RELAY_GAP_BUCKETS; i++) {
			bucketed += st.relay_gap[i];
		}
		zassert_equal(bucketed, 1U, "the redundant relay should land in exactly one bucket");
	}
}

/* An originator retransmitting (reliable delivery) is not a relay. Counting it
 * would inflate the redundancy figure and argue for work that would not help. */
ZTEST(protocol_stack, test_originator_retransmit_is_not_counted_redundant)
{
	struct meshtastic_sched_stats st;

	meshtastic_sched_stats_reset();

	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70002U, 3U, 3U, 0U);
	k_sleep(K_MSEC(30));

	/* Same frame again, still hop_start == hop_limit and no relayer byte. */
	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70002U, 3U, 3U, 0U);
	k_sleep(K_MSEC(30));

	meshtastic_sched_stats_get(&st);
	zassert_true(st.relay_sent >= 1U, "baseline relay expected");
	zassert_equal(st.relay_redundant, 0U,
		      "an originator retransmission must not count as a peer relay");
}

/* Our own relayer byte coming back to us is our echo, not a peer relaying. */
ZTEST(protocol_stack, test_own_relayer_byte_is_not_counted_redundant)
{
	struct meshtastic_sched_stats st;

	meshtastic_sched_stats_reset();

	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70003U, 3U, 3U, 0U);
	k_sleep(K_MSEC(30));

	inject_broadcast_hops(PEER_NODE_ID, 0xE1A70003U, 2U, 3U, (uint8_t)(TEST_NODE_ID & 0xFFU));
	k_sleep(K_MSEC(30));

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.relay_redundant, 0U, "our own relayer byte must not count as a peer");
}

/* --- Contention window on the relay path ----------------------------------- */

/* With the window enabled a relay must NOT go out immediately — it sits in the
 * outbound queue until its deadline. That gap is the whole point: it is the
 * only interval during which an overhear-cancel could suppress the relay. */
ZTEST(protocol_stack, test_relay_is_deferred_by_the_contention_window)
{
	uint32_t before;

	/* A narrow but non-zero window keeps the test quick while still deferring:
	 * cw.max 3 with the default offset gives 2*3 = 6 slots of ~28 ms minimum. */
	zassert_ok(meshtastic_sched_set("cw.min", "3"));
	zassert_ok(meshtastic_sched_set("cw.max", "3"));

	before = mock_lora.send_count;
	inject_broadcast_hops(PEER_NODE_ID, 0xC0FFEE01U, 3U, 3U, 0U);

	/* Immediately after injection the relay is queued, not sent. */
	k_sleep(K_MSEC(30));
	zassert_equal(mock_lora.send_count, before,
		      "a relay must not transmit before its contention delay elapses");

	/* It does go out once the window closes. */
	k_sleep(K_MSEC(1200));
	zassert_true(mock_lora.send_count > before,
		     "the deferred relay should have transmitted after the window");
}

/* The window is policy: zeroed, a relay goes out on the spot. This is both the
 * port's original behaviour and the control arm of the on-air A/B. */
ZTEST(protocol_stack, test_zero_window_relays_immediately)
{
	uint32_t before = mock_lora.send_count;

	zassert_ok(meshtastic_sched_set("cw.max", "0"));

	inject_broadcast_hops(PEER_NODE_ID, 0xC0FFEE02U, 3U, 3U, 0U);
	k_sleep(K_MSEC(50));

	zassert_true(mock_lora.send_count > before,
		     "with the window disabled the relay should already be out");
}

/* A deferred relay must not block traffic queued behind it: the worker skips
 * over anything not yet due. Without that, one waiting relay would stall every
 * ACK and reply behind it for the length of its window. */
ZTEST(protocol_stack, test_deferred_relay_does_not_block_later_traffic)
{
	uint32_t before;

	zassert_ok(meshtastic_sched_set("cw.min", "4"));
	zassert_ok(meshtastic_sched_set("cw.max", "4"));

	before = mock_lora.send_count;
	inject_broadcast_hops(PEER_NODE_ID, 0xC0FFEE03U, 3U, 3U, 0U);
	k_sleep(K_MSEC(20));
	zassert_equal(mock_lora.send_count, before, "relay should still be waiting");

	/* Our own traffic is not deferred, so it must overtake the parked relay. */
	zassert_ok(meshtastic_send_text(OTHER_NODE_ID, "urgent"));
	k_sleep(K_MSEC(60));
	zassert_true(mock_lora.send_count > before,
		     "undeferred traffic must overtake a relay still inside its window");

	/* Do not leave the deferred relay queued: it would fire partway through
	 * whichever test runs next and be counted against that test's traffic.
	 * A deferred frame outliving its test is a leak, so drain it here. */
	for (int i = 0; i < 100 && mock_lora.send_count < before + 2U; i++) {
		k_sleep(K_MSEC(20));
	}
	zassert_true(mock_lora.send_count >= before + 2U,
		     "the deferred relay should have drained before the test ended");
}

/* --- Packet-id unpredictability -------------------------------------------- */

/* The AES-CTR channel nonce is built from (packet id, source node id) and
 * nothing else, and the source id never changes — so the packet id is the only
 * thing keeping the (key, nonce) pair unique. A counter that restarts at a
 * fixed value each boot therefore re-emits packets under nonces already used,
 * and two ciphertexts sharing a keystream leak the XOR of their plaintexts.
 *
 * A sim test cannot reboot, so it asserts the property that makes the reuse
 * impossible: ids are not a plain ascending counter. Before the fix every
 * allocation was exactly the previous one plus one, so this fails outright. */
ZTEST(protocol_stack, test_packet_ids_are_not_a_plain_counter)
{
	uint32_t ids[64];
	unsigned int non_sequential = 0U;

	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		ids[i] = meshtastic_allocate_packet_id();
		zassert_not_equal(ids[i], 0U, "packet id 0 is reserved and must never be issued");
	}

	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		for (size_t j = i + 1U; j < ARRAY_SIZE(ids); j++) {
			zassert_not_equal(ids[i], ids[j], "packet ids %zu and %zu collided (0x%08x)",
					  i, j, ids[i]);
		}
	}

	for (size_t i = 1; i < ARRAY_SIZE(ids); i++) {
		if (ids[i] != ids[i - 1U] + 1U) {
			non_sequential++;
		}
	}

	/* Randomised high bits make essentially every step non-sequential. The
	 * threshold is deliberately loose (half) so this can never flake: failing
	 * it would need ~32 independent 2^-22 coincidences. A pure ++ counter
	 * scores 0. */
	zassert_true(non_sequential >= (ARRAY_SIZE(ids) - 1U) / 2U,
		     "packet ids look like a plain counter (%u/%zu steps non-sequential)",
		     non_sequential, ARRAY_SIZE(ids) - 1U);
}

/* --- MQTT downlink must not reach remote admin (security H4) --------------- */

static meshtastic_Config_DeviceConfig_Role current_device_role(void)
{
	meshtastic_Config dev;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &dev),
		   "device config read failed");
	return dev.payload_variant.device.role;
}

/* Admin writes the role through the config store; meshtastic_set_device_role()
 * only updates the runtime copy, so it cannot be used to set up or clean up
 * these tests. Write the store directly, the way admin does. */
static void force_device_role(meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_Config dev = meshtastic_Config_init_zero;

	dev.which_payload_variant = meshtastic_Config_device_tag;
	dev.payload_variant.device.role = role;
	zassert_ok(meshtastic_config_store_set_config(&dev), "device config write failed");
}

/* Enable the legacy identity-less admin gate and name the primary channel
 * "admin" — the exact configuration that made the downlink path exploitable. */
static void enable_legacy_admin_channel(bool enable)
{
	meshtastic_Config sec = meshtastic_Config_init_zero;

	sec.which_payload_variant = meshtastic_Config_security_tag;
	sec.payload_variant.security.admin_channel_enabled = enable;
	zassert_ok(meshtastic_config_store_set_config(&sec), "security config write failed");
}

/* Build an MQTT-borne, decoded ADMIN_APP packet carrying a valid session
 * passkey. Everything about it is legitimate except its provenance. */
static void build_admin_downlink(meshtastic_MeshPacket *mesh, uint32_t id)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	pb_ostream_t os;

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_device_tag;
	am.payload_variant.set_config.payload_variant.device.role =
		meshtastic_Config_DeviceConfig_Role_ROUTER;
	am.session_passkey.size = (pb_size_t)sizeof(key);
	memcpy(am.session_passkey.bytes, key, sizeof(key));

	*mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
	mesh->from = PEER_NODE_ID;
	mesh->to = TEST_NODE_ID;
	mesh->id = id;
	mesh->channel = meshtastic_channels_primary_index();
	mesh->hop_limit = 3U;
	mesh->hop_start = 3U;
	mesh->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh->decoded.portnum = (meshtastic_PortNum)MESHTASTIC_PORT_ADMIN;

	os = pb_ostream_from_buffer(mesh->decoded.payload.bytes,
				    sizeof(mesh->decoded.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");
	mesh->decoded.payload.size = (pb_size_t)os.bytes_written;
}

/* H4: an ADMIN_APP packet arriving over the MQTT downlink must be refused at
 * ingest. The broker is not a mesh peer, and an injected packet's channel is
 * forced to primary, so the legacy gate would authorize it on channel *name*
 * alone — no identity at all. On a plaintext or bridged broker that is remote
 * admin for any internet peer, gated only by a cleartext-visible passkey. */
ZTEST(protocol_stack, test_mqtt_downlink_admin_is_rejected)
{
	meshtastic_MeshPacket mesh;
	int ret;

	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	enable_legacy_admin_channel(true);

	build_admin_downlink(&mesh, 0xADD10001U);
	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	k_sleep(K_MSEC(50));

	zassert_equal(ret, -EACCES, "ADMIN_APP downlink must be rejected at ingest, got %d", ret);
	zassert_equal(current_device_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "an MQTT-borne admin packet must not mutate config");

	enable_legacy_admin_channel(false);
	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
}

/* Rename the primary channel, returning the previous slot so it can be put
 * back — the channel hash is derived from the name, so leaving it changed
 * would perturb every later test. */
static void swap_primary_channel_name(const char *name, meshtastic_Channel *saved)
{
	uint8_t index = meshtastic_channels_primary_index();
	meshtastic_Channel ch;

	*saved = *meshtastic_channels_get(index);
	ch = *saved;
	strncpy(ch.settings.name, name, sizeof(ch.settings.name) - 1U);
	ch.settings.name[sizeof(ch.settings.name) - 1U] = '\0';
	zassert_ok(meshtastic_channels_set_slot(index, &ch), "channel rename failed");
}

/* H4, second layer: even with the primary channel literally named "admin" and
 * the legacy gate enabled — the one configuration where identity-less admin is
 * accepted — a packet marked via_mqtt must still be refused.
 *
 * This is the case the ingest filter cannot see: an admin payload that arrives
 * encrypted has no visible portnum until after decrypt, and a frame can also
 * reach us over RF with the via_mqtt wire flag already set. Authorization by
 * channel name carries no identity, so MQTT provenance has to disqualify it. */
ZTEST(protocol_stack, test_mqtt_borne_admin_refused_on_legacy_admin_channel)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t payload[128];
	pb_ostream_t os;
	meshtastic_Channel saved;
	struct meshtastic_packet pkt = {0};

	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	enable_legacy_admin_channel(true);
	swap_primary_channel_name("admin", &saved);

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_device_tag;
	am.payload_variant.set_config.payload_variant.device.role =
		meshtastic_Config_DeviceConfig_Role_ROUTER;
	am.session_passkey.size = (pb_size_t)sizeof(key);
	memcpy(am.session_passkey.bytes, key, sizeof(key));

	os = pb_ostream_from_buffer(payload, sizeof(payload));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am), "admin encode failed");

	pkt.from = PEER_NODE_ID;
	pkt.to = TEST_NODE_ID;
	pkt.id = 0xADD10003U;
	pkt.portnum = MESHTASTIC_PORT_ADMIN;
	pkt.channel_index = meshtastic_channels_primary_index();
	pkt.payload = payload;
	pkt.payload_len = os.bytes_written;

	/* The MQTT-marked packet runs first so the refusal is asserted against a
	 * known-CLIENT starting state, with no store write needed between the two
	 * phases. */
	pkt.via_mqtt = true;
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(K_MSEC(30));
	zassert_equal(current_device_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "an MQTT-borne packet must not get identity-less channel authorization");

	/* Now the identical packet without the MQTT mark. It must apply — proving
	 * the assertion above was the via_mqtt check firing and not simply a
	 * fixture that never authorizes anything. */
	pkt.id = 0xADD10004U;
	pkt.via_mqtt = false;
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(K_MSEC(30));
	zassert_equal(current_device_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "control: a mesh-borne admin on the legacy channel should apply");

	zassert_ok(meshtastic_channels_set_slot(meshtastic_channels_primary_index(), &saved),
		   "channel restore failed");
	enable_legacy_admin_channel(false);
	force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
}

/* The rejection must be specific to ADMIN_APP — ordinary downlink traffic is
 * the whole point of the feature and has to keep working. */
ZTEST(protocol_stack, test_mqtt_downlink_non_admin_still_delivered)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	int ret;

	mesh.from = PEER_NODE_ID;
	mesh.to = TEST_NODE_ID;
	mesh.id = 0xADD10002U;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.hop_limit = 3U;
	mesh.hop_start = 3U;
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = MESHTASTIC_PORT_TEXT_MESSAGE;
	memcpy(mesh.decoded.payload.bytes, "hi", 2U);
	mesh.decoded.payload.size = 2U;

	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	k_sleep(K_MSEC(30));

	zassert_ok(ret, "a non-admin downlink must still be accepted: %d", ret);
}

/* --- Region -> preset map (want_config handshake) ------------------------- */

/* Find the preset group a region maps to, or -1 if the region is absent. */
static int region_group_index(const meshtastic_LoRaRegionPresetMap *map,
			      meshtastic_Config_LoRaConfig_RegionCode region)
{
	for (pb_size_t i = 0U; i < map->region_groups_count; i++) {
		if (map->region_groups[i].region == region) {
			return (int)map->region_groups[i].group_index;
		}
	}
	return -1;
}

static bool group_has_preset(const meshtastic_LoRaPresetGroup *grp,
			     meshtastic_Config_LoRaConfig_ModemPreset preset)
{
	for (pb_size_t i = 0U; i < grp->presets_count; i++) {
		if (grp->presets[i] == preset) {
			return true;
		}
	}
	return false;
}

/* The region->preset map is well-formed and matches the reference table: US is a
 * standard, unlicensed LONG_FAST group; HAM regions are licensed; regions sharing
 * a preset list but differing in licensing land in distinct groups. */
ZTEST(protocol_stack, test_region_preset_map_matches_reference)
{
	meshtastic_LoRaRegionPresetMap map;
	int gi;

	meshtastic_build_region_preset_map(&map);

	/* Capacities respected; the reference table has 36 regions coalescing into
	 * 7 distinct (profile, default) groups. */
	zassert_true(map.region_groups_count <= ARRAY_SIZE(map.region_groups), "region overflow");
	zassert_true(map.groups_count <= ARRAY_SIZE(map.groups), "group overflow");
	zassert_equal(map.region_groups_count, 36U, "unexpected region count");
	zassert_equal(map.groups_count, 7U, "unexpected group count");

	/* US -> standard group: LONG_FAST default, unlicensed, LONG_FAST legal. */
	gi = region_group_index(&map, meshtastic_Config_LoRaConfig_RegionCode_US);
	zassert_true(gi >= 0, "US missing from map");
	zassert_equal(map.groups[gi].default_preset,
		      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, "US default not LONG_FAST");
	zassert_false(map.groups[gi].licensed_only, "US should not be licensed-only");
	zassert_true(group_has_preset(&map.groups[gi],
				      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST),
		     "US group should list LONG_FAST");

	/* A HAM 2 m region -> licensed-only group defaulting to TINY_FAST. */
	gi = region_group_index(&map, meshtastic_Config_LoRaConfig_RegionCode_ITU1_2M);
	zassert_true(gi >= 0, "ITU1_2M missing from map");
	zassert_true(map.groups[gi].licensed_only, "HAM region must be licensed-only");
	zassert_equal(map.groups[gi].default_preset,
		      meshtastic_Config_LoRaConfig_ModemPreset_TINY_FAST, "HAM default not TINY_FAST");

	/* EU_866 -> LITE profile defaulting to LITE_FAST. */
	gi = region_group_index(&map, meshtastic_Config_LoRaConfig_RegionCode_EU_866);
	zassert_true(gi >= 0, "EU_866 missing from map");
	zassert_equal(map.groups[gi].default_preset,
		      meshtastic_Config_LoRaConfig_ModemPreset_LITE_FAST, "EU_866 default not LITE_FAST");

	/* EU_N_868 (unlicensed) and ITU1_70CM (licensed) both use the NARROW preset
	 * list but must NOT be coalesced — licensing differs. */
	int narrow = region_group_index(&map, meshtastic_Config_LoRaConfig_RegionCode_EU_N_868);
	int ham = region_group_index(&map, meshtastic_Config_LoRaConfig_RegionCode_ITU1_70CM);
	zassert_true(narrow >= 0 && ham >= 0, "narrow/ham region missing");
	zassert_not_equal(narrow, ham, "licensed and unlicensed NARROW must be separate groups");
	zassert_false(map.groups[narrow].licensed_only, "EU_N_868 must be unlicensed");
	zassert_true(map.groups[ham].licensed_only, "ITU1_70CM must be licensed");

	/* The whole map must encode inside one FromRadio frame (the PhoneAPI buffer
	 * is 512 B) or the want_config stage silently drops it on real hardware. */
	{
		meshtastic_FromRadio from = meshtastic_FromRadio_init_zero;
		uint8_t buf[512];
		pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

		from.which_payload_variant = meshtastic_FromRadio_region_presets_tag;
		from.region_presets = map;
		zassert_true(pb_encode(&os, meshtastic_FromRadio_fields, &from),
			     "region preset FromRadio must fit the 512 B frame buffer");
	}
}

/* Verifies duplicate foreign packets do not trigger additional relay transmissions. */
ZTEST(protocol_stack, test_duplicate_foreign_packets_do_not_relay_again)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	build_peer_wire_packet(OTHER_NODE_ID, 0x5606U, 3U, "relay-once", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -35, 4);
	k_sleep(K_MSEC(100));
	assert_mock_send_count(1U);

	reset_mock_lora();
	inject_rx_frame(wire, wire_len, -35, 4);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(0U);
	zassert_equal(state.recv_count, 0U, "foreign packets should not be locally delivered");
	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.duplicate_packets, before.duplicate_packets + 1U,
		      "duplicate counter not incremented");
}

/* Verifies foreign unicasts with hop limit remaining are relayed with the hop limit decremented. */
ZTEST(protocol_stack, test_foreign_unicast_is_relayed_with_decremented_hop_limit)
{
	struct meshtastic_wire_header hdr;
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	build_peer_wire_packet(OTHER_NODE_ID, 0x3003U, 3U, "relay", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -55, 2);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(1U);
	copy_last_tx_header(&hdr);
	zassert_equal(hdr.flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK, 2U,
		      "relay hop limit not decremented");
	zassert_equal(state.recv_count, 0U, "foreign unicast should not be delivered locally");

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.relayed_packets, before.relayed_packets + 1U,
		      "relay counter not incremented");
}

/* Verifies foreign unicasts with no hop limit remaining are not relayed. */
ZTEST(protocol_stack, test_foreign_unicast_with_zero_hop_limit_is_not_relayed)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	build_peer_wire_packet(OTHER_NODE_ID, 0x5707U, 0U, "terminal", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -48, 2);
	k_sleep(K_MSEC(100));

	assert_mock_send_count(0U);
	zassert_equal(state.recv_count, 0U, "foreign unicast should not be delivered locally");
}

/* Verifies rebroadcast policy NONE and CLIENT_MUTE role both suppress foreign relays. */
ZTEST(protocol_stack, test_rebroadcast_policy_can_suppress_foreign_relay)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;
	int ret;

	ret = meshtastic_get_status(&before);
	zassert_ok(ret, "status read failed: %d", ret);

	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_NONE);
	build_peer_wire_packet(OTHER_NODE_ID, 0x5808U, 3U, "none", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -45, 2);
	k_sleep(K_MSEC(100));
	assert_mock_send_count(0U);

	reset_mock_lora();
	meshtastic_set_rebroadcast_mode(meshtastic_Config_DeviceConfig_RebroadcastMode_ALL);
	meshtastic_set_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE);
	build_peer_wire_packet(OTHER_NODE_ID, 0x5809U, 3U, "mute", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -45, 2);
	k_sleep(K_MSEC(100));
	assert_mock_send_count(0U);

	ret = meshtastic_get_status(&after);
	zassert_ok(ret, "status read failed: %d", ret);
	zassert_equal(after.relayed_packets, before.relayed_packets,
		      "relay counter should not change when policy suppresses relay");
}

/* Verifies a frame whose source claims the broadcast address is dropped at
 * ingress, before it can poison the dedup cache or the NodeDB. */
ZTEST(protocol_stack, test_broadcast_source_frame_dropped_at_ingress)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;

	zassert_ok(meshtastic_get_status(&before));

	build_wire_packet(MESHTASTIC_NODE_BROADCAST, TEST_NODE_ID, 0x7001U, 3U,
			  MESHTASTIC_PORT_TEXT_MESSAGE, (const uint8_t *)"bad", 3U, wire,
			  &wire_len);
	inject_rx_frame(wire, wire_len, -40, 4);
	k_sleep(K_MSEC(50));

	zassert_equal(state.recv_count, 0U, "broadcast-source frame must not be delivered");
	zassert_equal(mt.dup_head, 0U, "broadcast-source frame must not enter the dup cache");
	zassert_equal(mt.dup_cache[0].src, 0U, "dup cache should be untouched");
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.rx_packets, before.rx_packets,
		      "broadcast-source frame must not count as RX");

	/* A legit frame from a real source still processes normally afterward. */
	build_peer_wire_packet(TEST_NODE_ID, 0x7002U, 3U, "ok", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 4);
	k_sleep(K_MSEC(50));
	zassert_equal(state.recv_count, 1U, "legit frame after the drop must deliver");
}

/* Verifies frames from a node on the ignore-list are dropped at ingress, and
 * flow again once un-ignored. */
ZTEST(protocol_stack, test_ignored_node_frames_dropped)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;

	/* Make PEER known to the NodeDB, then ignore it. */
	build_peer_wire_packet(MESHTASTIC_NODE_BROADCAST, 0x7101U, 3U, "hello", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 4);
	k_sleep(K_MSEC(50));
	zassert_equal(state.recv_count, 1U, "baseline delivery failed");
	zassert_ok(meshtastic_nodedb_set_ignored(PEER_NODE_ID, true));

	build_peer_wire_packet(MESHTASTIC_NODE_BROADCAST, 0x7102U, 3U, "nope", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 4);
	k_sleep(K_MSEC(50));
	zassert_equal(state.recv_count, 1U, "ignored node's frame must not be delivered");

	zassert_ok(meshtastic_nodedb_set_ignored(PEER_NODE_ID, false));
	build_peer_wire_packet(MESHTASTIC_NODE_BROADCAST, 0x7103U, 3U, "back", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -40, 4);
	k_sleep(K_MSEC(50));
	zassert_equal(state.recv_count, 2U, "un-ignored node should deliver again");
}

/* Verifies CORE_PORTNUMS_ONLY also suppresses the *relay* of a decoded non-core
 * portnum (not just its local delivery), while core portnums still relay. */
ZTEST(protocol_stack, test_core_portnums_only_suppresses_non_core_relay)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len;
	struct meshtastic_status before;
	struct meshtastic_status after;

	/* Track the relay counter, not raw sends: hearing an unknown node can also
	 * TX a NodeInfo request, which is not a relay. */
	zassert_ok(meshtastic_get_status(&before));

	meshtastic_set_rebroadcast_mode(
		meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY);

	/* Foreign non-core packet (PRIVATE_APP): must not be relayed. */
	build_wire_packet(PEER_NODE_ID, OTHER_NODE_ID, 0x6001U, 3U, MESHTASTIC_PORT_PRIVATE,
			  (const uint8_t *)"nc", 2U, wire, &wire_len);
	inject_rx_frame(wire, wire_len, -45, 3);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, before.relayed_packets,
		      "non-core portnum must not be relayed in CORE_PORTNUMS_ONLY");

	/* Foreign core packet (TEXT_MESSAGE): still relayed in the same mode. */
	build_peer_wire_packet(OTHER_NODE_ID, 0x6002U, 3U, "core", wire, &wire_len);
	inject_rx_frame(wire, wire_len, -45, 3);
	k_sleep(K_MSEC(100));
	zassert_ok(meshtastic_get_status(&after));
	zassert_equal(after.relayed_packets, before.relayed_packets + 1U,
		      "core portnum should still relay in CORE_PORTNUMS_ONLY");
}

/* Verifies decoded MQTT downlink broadcasts are delivered locally and marked via MQTT. */
ZTEST(protocol_stack, test_downlink_decoded_broadcast_delivers_locally_via_mqtt)
{
	const uint8_t body[] = "mqtt";
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	int ret;

	mesh.from = PEER_NODE_ID;
	mesh.to = MESHTASTIC_NODE_BROADCAST;
	mesh.id = 0x5901U;
	mesh.hop_limit = 2U;
	mesh.hop_start = 2U;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = MESHTASTIC_PORT_TEXT_MESSAGE;
	mesh.decoded.payload.size = sizeof(body) - 1U;
	memcpy(mesh.decoded.payload.bytes, body, sizeof(body) - 1U);

	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	zassert_ok(ret, "downlink inject failed: %d", ret);

	zassert_equal(state.recv_count, 1U, "broadcast downlink should be delivered");
	zassert_equal(state.last_received.type, MESHTASTIC_EVENT_PACKET_RECEIVED,
		      "unexpected downlink event");
	zassert_true(state.last_received_packet.via_mqtt, "downlink should be marked via MQTT");
	assert_received_payload(body, sizeof(body) - 1U);
	assert_mock_send_count(1U);
}

/* Verifies foreign decoded MQTT downlinks relay onto LoRa without local packet delivery. */
ZTEST(protocol_stack, test_downlink_foreign_packet_relays_without_local_delivery)
{
	const uint8_t body[] = "fwd";
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	int ret;

	mesh.from = PEER_NODE_ID;
	mesh.to = OTHER_NODE_ID;
	mesh.id = 0x5902U;
	mesh.hop_limit = 2U;
	mesh.hop_start = 2U;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = MESHTASTIC_PORT_TEXT_MESSAGE;
	mesh.decoded.payload.size = sizeof(body) - 1U;
	memcpy(mesh.decoded.payload.bytes, body, sizeof(body) - 1U);

	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	zassert_ok(ret, "foreign downlink inject failed: %d", ret);

	assert_mock_send_count(1U);
	zassert_equal(state.recv_count, 0U, "foreign downlink should not be delivered locally");
	zassert_equal(state.last_event.type, MESHTASTIC_EVENT_TX_DONE,
		      "relay should emit TX_DONE only");
}

/* Verifies duplicate MQTT downlinks are rejected before relay or local delivery. */
ZTEST(protocol_stack, test_duplicate_downlink_returns_ealready)
{
	const uint8_t body[] = "dupe";
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;
	int ret;

	mesh.from = PEER_NODE_ID;
	mesh.to = MESHTASTIC_NODE_BROADCAST;
	mesh.id = 0x5903U;
	mesh.hop_limit = 2U;
	mesh.hop_start = 2U;
	mesh.channel = meshtastic_channels_primary_index();
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = MESHTASTIC_PORT_TEXT_MESSAGE;
	mesh.decoded.payload.size = sizeof(body) - 1U;
	memcpy(mesh.decoded.payload.bytes, body, sizeof(body) - 1U);

	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	zassert_ok(ret, "first downlink inject failed: %d", ret);

	reset_mock_lora();
	reset_callbacks_state();
	ret = meshtastic_inject_downlink_mesh_packet(&mesh);
	zassert_equal(ret, -EALREADY, "duplicate downlink should return -EALREADY");
	assert_mock_send_count(0U);
	zassert_equal(state.recv_count, 0U, "duplicate downlink should not be delivered");
	zassert_equal(state.event_count, 0U, "duplicate downlink should not emit events");
}

/* ------------------------------------------------------------------------- */
/* Scheduler egress: priority ordering + overflow drop, verified end-to-end   */
/* through the outbound worker by gating the mock radio's send().             */
/* ------------------------------------------------------------------------- */

#define EGRESS_HOP 3U

static void egress_gate_enable(bool on)
{
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.gate_enabled = on;
	k_mutex_unlock(&mock_lora.lock);
}

/* Start a gated egress test from a known-clean state: free any worker a prior
 * test left parked on the gate (k_sem_reset does NOT wake a blocked taker),
 * drain the queue to idle, zero the tx log, then re-enable the gate. */
static void egress_begin(void)
{
	egress_gate_enable(false);
	for (int i = 0; i < (int)ARRAY_SIZE(mock_lora.tx_ids); i++) {
		k_sem_give(&mock_lora.gate); /* release a parked worker, if any */
	}
	k_msleep(50); /* let the worker finish and drain to the ob_avail wait */

	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	mock_lora.tx_ids_count = 0U;
	mock_lora.send_count = 0U;
	k_mutex_unlock(&mock_lora.lock);
	k_sem_reset(&mock_lora.gate);
	k_sem_reset(&mock_lora.entered);

	egress_gate_enable(true);
}

/* Settle the worker after a gated test: disable the gate, release it, and let
 * the in-flight send_wire_now() finish — which re-arms the mock RX callback the
 * next test relies on. */
static void egress_end(void)
{
	egress_gate_enable(false);
	for (int i = 0; i < (int)ARRAY_SIZE(mock_lora.tx_ids); i++) {
		k_sem_give(&mock_lora.gate);
	}
	k_msleep(50);
	zassert_not_null(mock_lora.rx_cb, "egress test left the RX callback un-armed");
}

/* Enqueue a raw wire frame at an explicit tier (bypasses portnum->tier mapping
 * so the queue mechanism is tested directly). Returns the enqueue result. */
static int egress_send(uint32_t id, uint8_t tier)
{
	uint8_t wire[MESHTASTIC_PKT_MAX];
	uint32_t wire_len = 0U;
	static const uint8_t body[] = {0xDEU, 0xADU};

	build_wire_packet(TEST_NODE_ID, MESHTASTIC_NODE_BROADCAST, id, EGRESS_HOP,
			  MESHTASTIC_PORT_TEXT_MESSAGE, body, sizeof(body), wire, &wire_len);
	return meshtastic_radio_send_wire_prio(wire, wire_len, tier);
}

/* Wait until the gated worker has parked inside send() (i.e. it dequeued and is
 * about to transmit the next frame). */
static void egress_wait_parked(void)
{
	zassert_ok(k_sem_take(&mock_lora.entered, K_SECONDS(2)),
		   "timed out waiting for the outbound worker to park in send()");
}

static void egress_release_one(void)
{
	k_sem_give(&mock_lora.gate);
}

static void egress_assert_order(const uint32_t *expected, size_t count)
{
	k_mutex_lock(&mock_lora.lock, K_FOREVER);
	zassert_equal(mock_lora.tx_ids_count, count, "expected %zu transmits, got %u", count,
		      mock_lora.tx_ids_count);
	for (size_t i = 0; i < count; i++) {
		zassert_equal(mock_lora.tx_ids[i], expected[i],
			      "tx #%zu: expected id 0x%08x, got 0x%08x", i, expected[i],
			      mock_lora.tx_ids[i]);
	}
	k_mutex_unlock(&mock_lora.lock);
}

/* Frames queued together drain highest-tier-first, FIFO within a tier. */
ZTEST(protocol_stack, test_egress_priority_ordering)
{
	/* Expected drain order after the primer: ACK, HIGH, the two NORMAL frames
	 * in arrival order, then the two BG frames in arrival order. */
	static const uint32_t expected[] = {
		0x1000U, 0x1003U, 0x1004U, 0x1002U, 0x1006U, 0x1001U, 0x1005U,
	};

	meshtastic_sched_defaults(); /* priority order, drop-lowest */
	meshtastic_sched_stats_reset();
	egress_begin();

	/* Primer occupies the worker so the rest queue up together. */
	zassert_ok(egress_send(0x1000U, MT_SCHED_TIER_NORMAL));
	egress_wait_parked();

	zassert_ok(egress_send(0x1001U, MT_SCHED_TIER_BG));
	zassert_ok(egress_send(0x1002U, MT_SCHED_TIER_NORMAL));
	zassert_ok(egress_send(0x1003U, MT_SCHED_TIER_ACK));
	zassert_ok(egress_send(0x1004U, MT_SCHED_TIER_HIGH));
	zassert_ok(egress_send(0x1005U, MT_SCHED_TIER_BG));
	zassert_ok(egress_send(0x1006U, MT_SCHED_TIER_NORMAL));

	for (int i = 0; i < 6; i++) {
		egress_release_one();
		egress_wait_parked();
	}
	egress_release_one(); /* let the last frame's send() return */

	egress_gate_enable(false);
	egress_assert_order(expected, ARRAY_SIZE(expected));
	egress_end();
}

/* When the queue is full, a fire-and-forget frame evicts a strictly-lower-tier
 * frame if it can, else is itself dropped — never a higher-tier frame. */
ZTEST(protocol_stack, test_egress_overflow_drop_lowest)
{
	static const uint32_t expected[] = {0x2000U, 0x2005U, 0x2001U, 0x2002U};
	struct meshtastic_sched_stats st;

	meshtastic_sched_defaults(); /* priority order, drop-lowest */
	zassert_ok(meshtastic_sched_set("tx.depth", "3"));
	meshtastic_sched_stats_reset();
	egress_begin();

	zassert_ok(egress_send(0x2000U, MT_SCHED_TIER_NORMAL)); /* primer */
	egress_wait_parked();

	/* Fill the depth-3 queue with background frames. */
	zassert_ok(egress_send(0x2001U, MT_SCHED_TIER_BG));
	zassert_ok(egress_send(0x2002U, MT_SCHED_TIER_BG));
	zassert_ok(egress_send(0x2003U, MT_SCHED_TIER_BG));

	/* A 4th BG frame has nothing lower-ranked to evict -> dropped. */
	zassert_equal(egress_send(0x2004U, MT_SCHED_TIER_BG), -ENOMSG,
		      "same-tier overflow frame should be rejected");

	/* An ACK frame evicts the newest BG frame (0x2003) and takes its slot. */
	zassert_ok(egress_send(0x2005U, MT_SCHED_TIER_ACK));

	for (int i = 0; i < 3; i++) {
		egress_release_one();
		egress_wait_parked();
	}
	egress_release_one();

	egress_gate_enable(false);

	/* 0x2003 evicted and 0x2004 rejected never transmit; ACK drains first. */
	egress_assert_order(expected, ARRAY_SIZE(expected));

	meshtastic_sched_stats_get(&st);
	zassert_equal(st.tx_drop[MT_SCHED_TIER_BG], 2,
		      "expected two dropped BG frames (one rejected, one evicted)");
	egress_end();
}

/* ------------------------------------------------------------------------- */
/* Admin auth foundation — session-passkey engine + is_managed local gate.    */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_MESHTASTIC_ADMIN)

/* The session-key engine is standalone (no meshtastic_init needed): its own
 * static state, seeded from sys_rand_get + k_uptime. Real-time rotation (150 s)
 * and expiry (300 s) aren't exercised here — that would need a fake clock; these
 * cover issuance, stability, match/mismatch, and reset. */
ZTEST(admin_session, test_issued_key_validates)
{
	uint8_t k[MESHTASTIC_ADMIN_SESSION_KEY_LEN];

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(k);
	zassert_true(meshtastic_admin_session_valid(k, sizeof(k)),
		     "a freshly issued session key must validate");
}

ZTEST(admin_session, test_bad_key_rejected)
{
	uint8_t k[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t bad[MESHTASTIC_ADMIN_SESSION_KEY_LEN];

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(k);

	memcpy(bad, k, sizeof(bad));
	bad[0] ^= 0xFFU;
	zassert_false(meshtastic_admin_session_valid(bad, sizeof(bad)),
		      "a mismatched key must be rejected");
	zassert_false(meshtastic_admin_session_valid(k, sizeof(k) - 1U),
		      "a wrong-length key must be rejected");
	zassert_false(meshtastic_admin_session_valid(k, 0U), "an empty key must be rejected");
	zassert_false(meshtastic_admin_session_valid(NULL, sizeof(k)),
		      "a NULL key must be rejected");
}

ZTEST(admin_session, test_key_stable_then_reset_invalidates)
{
	uint8_t first[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t again[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	uint8_t second[MESHTASTIC_ADMIN_SESSION_KEY_LEN];

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(first);
	meshtastic_admin_session_current(again);
	/* Within the rotation window, the same key is returned. */
	zassert_mem_equal(first, again, sizeof(first),
			  "session key must be stable within the rotation window");

	meshtastic_admin_session_reset();
	zassert_false(meshtastic_admin_session_valid(first, sizeof(first)),
		      "after reset the previous key must not validate");

	meshtastic_admin_session_current(second);
	/* A reissued key should differ (8 random bytes; collision ~2^-64). Guards
	 * against a stuck generator. */
	zassert_true(memcmp(first, second, sizeof(first)) != 0,
		     "reissued key should differ from the reset one");
}

ZTEST_SUITE(admin_session, NULL, NULL, NULL, NULL, NULL);

/* Encode an AdminMessage set_config(device.role), optionally carrying a session
 * passkey (pass NULL/0 to omit it), into buf; returns length. */
static size_t encode_admin_set_role_key(meshtastic_Config_DeviceConfig_Role role,
					const uint8_t *passkey, size_t passkey_len, uint8_t *buf,
					size_t cap)
{
	meshtastic_AdminMessage am = meshtastic_AdminMessage_init_zero;
	pb_ostream_t os = pb_ostream_from_buffer(buf, cap);

	am.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
	am.payload_variant.set_config.which_payload_variant = meshtastic_Config_device_tag;
	am.payload_variant.set_config.payload_variant.device.role = role;
	if (passkey != NULL && passkey_len > 0U) {
		am.session_passkey.size = (pb_size_t)passkey_len;
		memcpy(am.session_passkey.bytes, passkey, passkey_len);
	}
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &am),
		     "admin set_config encode failed");
	return os.bytes_written;
}

static size_t encode_admin_set_role(meshtastic_Config_DeviceConfig_Role role, uint8_t *buf,
				    size_t cap)
{
	return encode_admin_set_role_key(role, NULL, 0U, buf, cap);
}

static void admin_set_is_managed(bool managed)
{
	meshtastic_Config cfg = meshtastic_Config_init_zero;

	cfg.which_payload_variant = meshtastic_Config_security_tag;
	cfg.payload_variant.security.is_managed = managed;
	zassert_ok(meshtastic_config_store_set_config(&cfg),
		   "failed to write SecurityConfig.is_managed");
}

static meshtastic_Config_DeviceConfig_Role admin_current_role(void)
{
	meshtastic_Config dev;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &dev),
		   "device config read failed");
	return dev.payload_variant.device.role;
}

/* Force the persisted device role. protocol_before() resets only the runtime
 * role (meshtastic_set_device_role), not the config store these admin tests
 * read/write — so each self-baselines to CLIENT and restores it, staying
 * order-independent and not leaking a role into other tests. */
static void admin_force_device_role(meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_Config dev = meshtastic_Config_init_zero;

	dev.which_payload_variant = meshtastic_Config_device_tag;
	dev.payload_variant.device.role = role;
	zassert_ok(meshtastic_config_store_set_config(&dev), "force device role failed");
}

/* A managed node consumes but does not apply local admin; an unmanaged node
 * applies it. Runs in protocol_stack so meshtastic_init + the role reset in
 * protocol_before() apply. */
ZTEST(protocol_stack, test_managed_node_refuses_local_admin)
{
	uint8_t buf[256];
	meshtastic_MeshPacket pkt = meshtastic_MeshPacket_init_zero;
	size_t len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, buf,
					   sizeof(buf));

	pkt.from = TEST_NODE_ID;
	pkt.to = TEST_NODE_ID;
	pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	pkt.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	pkt.decoded.payload.size = (pb_size_t)len;
	memcpy(pkt.decoded.payload.bytes, buf, len);

	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);

	/* Managed: the set is consumed but NOT applied. */
	admin_set_is_managed(true);
	zassert_true(meshtastic_admin_handle_local(&pkt), "admin packet should be consumed");
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "managed node must not apply local admin");

	/* Unmanaged: the same set now takes effect. */
	admin_set_is_managed(false);
	zassert_true(meshtastic_admin_handle_local(&pkt), "admin packet should be consumed");
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "unmanaged node must apply local admin");

	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	admin_set_is_managed(false);
}

/* Build a decoded internal packet carrying an AdminMessage received "over the
 * mesh" from PEER_NODE_ID to us, on the given channel. want_ack is a parameter
 * because the full RX path (meshtastic_handle_inbound_packet) also sends a
 * transport-level ROUTING ACK for want_ack unicasts — desirable in production
 * but a blocking (K_FOREVER) send that would deadlock the mock gate in tests. */
static void make_remote_admin_packet(struct meshtastic_packet *pkt, const uint8_t *payload,
				     size_t len, uint8_t channel_index, bool want_ack)
{
	*pkt = (struct meshtastic_packet){
		.from = PEER_NODE_ID,
		.to = TEST_NODE_ID,
		.portnum = MESHTASTIC_PORT_ADMIN,
		.channel_index = channel_index,
		.payload = payload,
		.payload_len = len,
		.want_ack = want_ack,
	};
}

/* Let the outbound worker transmit a fire-and-forget admin reply (NAK/ACK sent
 * K_NO_WAIT, so there is no TX_DONE event to wait on) — same 50 ms settle the
 * suite's other async-TX tests use, so the reply is counted in this test rather
 * than leaking into the next test's send_count. */
#define ADMIN_REPLY_SETTLE K_MSEC(50)

/* Test-only admin channel: a secondary slot named "admin" plus the legacy
 * admin_channel_enabled flag, so a plaintext (non-PKC) remote admin packet on
 * that channel authorizes — the only remote-auth path testable without PKI. */
#define ADMIN_TEST_CH_INDEX 1

static void admin_channel_set(bool enabled)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	meshtastic_Config sec = meshtastic_Config_init_zero;

	ch.role = enabled ? meshtastic_Channel_Role_SECONDARY : meshtastic_Channel_Role_DISABLED;
	ch.has_settings = true;
	if (enabled) {
		strncpy(ch.settings.name, "admin", sizeof(ch.settings.name) - 1U);
	}
	zassert_ok(meshtastic_config_store_set_channel(ADMIN_TEST_CH_INDEX, &ch),
		   "admin test channel set failed");

	sec.which_payload_variant = meshtastic_Config_security_tag;
	sec.payload_variant.security.admin_channel_enabled = enabled;
	zassert_ok(meshtastic_config_store_set_config(&sec), "admin_channel_enabled set failed");
}

/* Remote admin from an unauthorized sender (not PKC, not on the admin channel)
 * is refused: config is not changed. The core security property. */
ZTEST(protocol_stack, test_remote_admin_unauthorized_refused)
{
	uint8_t buf[256];
	struct meshtastic_packet pkt;
	size_t len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, buf,
					   sizeof(buf));

	admin_set_is_managed(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	make_remote_admin_packet(&pkt, buf, len, meshtastic_channels_primary_index(), true);

	reset_mock_lora();
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(ADMIN_REPLY_SETTLE);
	assert_mock_send_count(1U); /* the NAK back to the sender, drained here */
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "unauthorized remote admin must not apply");
}

/* The router routes an ADMIN_APP unicast-to-us into the remote-admin handler
 * (consumed), not to the phone as an ordinary RX packet. Unauthorized here, so
 * it is also not applied. */
ZTEST(protocol_stack, test_remote_admin_routed_not_delivered)
{
	uint8_t buf[256];
	struct meshtastic_packet pkt;
	size_t len = encode_admin_set_role(meshtastic_Config_DeviceConfig_Role_ROUTER, buf,
					   sizeof(buf));

	admin_set_is_managed(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	/* want_ack=false: the full RX path would also send a blocking transport ACK
	 * that deadlocks the mock gate; the admin NAK alone proves routing. */
	make_remote_admin_packet(&pkt, buf, len, meshtastic_channels_primary_index(), false);

	reset_mock_lora();
	meshtastic_handle_inbound_packet(&pkt, NULL, 0U, true);
	k_sleep(ADMIN_REPLY_SETTLE);

	zassert_equal(state.recv_count, 0U,
		      "remote admin must be consumed by handle_remote, not delivered to the phone");
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "unauthorized remote admin must not apply");
}

/* On the (authorized) legacy admin channel, a remote mutating op still needs a
 * valid session passkey: rejected without one, applied with a live one. */
ZTEST(protocol_stack, test_remote_admin_channel_requires_passkey)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	struct meshtastic_packet pkt;
	size_t len;

	admin_set_is_managed(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	admin_channel_set(true);

	/* Authorized by channel, but no passkey -> rejected, not applied. */
	len = encode_admin_set_role_key(meshtastic_Config_DeviceConfig_Role_ROUTER, NULL, 0U, buf,
					sizeof(buf));
	make_remote_admin_packet(&pkt, buf, len, ADMIN_TEST_CH_INDEX, true);
	reset_mock_lora();
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(ADMIN_REPLY_SETTLE);
	assert_mock_send_count(1U); /* BAD_SESSION_KEY NAK */
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "remote setter without a session passkey must be rejected");

	/* Same op with a live passkey -> applied. */
	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role_key(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
					buf, sizeof(buf));
	make_remote_admin_packet(&pkt, buf, len, ADMIN_TEST_CH_INDEX, true);
	reset_mock_lora();
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(ADMIN_REPLY_SETTLE);
	assert_mock_send_count(1U); /* success ROUTING ACK */
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "remote setter with a valid session passkey must apply");

	/* Teardown: disable the admin channel and restore the baseline role. */
	admin_channel_set(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	admin_set_is_managed(false);
}

/* A fixed 32-byte "peer public key". The admin_key authorization path only
 * memcmps NodeDB key vs SecurityConfig.admin_key — no crypto validation — so any
 * stable 32 bytes exercise the PKC match/mismatch without a PKI build. (The real
 * X25519 decrypt that sets pki_encrypted is covered by tests/admin_pki.) */
static const uint8_t admin_peer_pubkey[32] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};

/* Seed PEER_NODE_ID's public key into the NodeDB by delivering a NodeInfo — the
 * same path the stack learns a peer's key on the air (apply_user). */
static void seed_peer_pubkey(const uint8_t key[32])
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = PEER_NODE_ID,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	user.public_key.size = 32U;
	memcpy(user.public_key.bytes, key, 32U);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode failed");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;

	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

/* Set (key != NULL) or clear (key == NULL) SecurityConfig.admin_key[0]. */
static void admin_set_admin_key(const uint8_t *key, size_t len)
{
	meshtastic_Config sec = meshtastic_Config_init_zero;

	sec.which_payload_variant = meshtastic_Config_security_tag;
	if (key != NULL && len > 0U) {
		sec.payload_variant.security.admin_key_count = 1U;
		sec.payload_variant.security.admin_key[0].size = (pb_size_t)len;
		memcpy(sec.payload_variant.security.admin_key[0].bytes, key, len);
	}
	zassert_ok(meshtastic_config_store_set_config(&sec), "set admin_key failed");
}

/* A PKC (pki_encrypted) remote admin whose sender key is in admin_key[] is
 * authorized; with a valid passkey the mutating op applies. Exercises the
 * admin_key match without real crypto (pki_encrypted set directly). */
ZTEST(protocol_stack, test_remote_admin_pkc_admin_key_authorized)
{
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	struct meshtastic_packet pkt;
	size_t len;

	admin_set_is_managed(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	seed_peer_pubkey(admin_peer_pubkey);
	admin_set_admin_key(admin_peer_pubkey, sizeof(admin_peer_pubkey));

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role_key(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
					buf, sizeof(buf));

	make_remote_admin_packet(&pkt, buf, len, 0U, true); /* channel 0 = PKC pseudo-channel */
	pkt.pki_encrypted = true;

	reset_mock_lora();
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(ADMIN_REPLY_SETTLE);
	assert_mock_send_count(1U); /* success ROUTING ACK */
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "PKC admin whose key is in admin_key[] must be authorized and applied");

	admin_set_admin_key(NULL, 0U);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	admin_set_is_managed(false);
}

/* A PKC remote admin whose sender key is NOT in admin_key[] is refused
 * (ADMIN_PUBLIC_KEY_UNAUTHORIZED) and not applied. */
ZTEST(protocol_stack, test_remote_admin_pkc_wrong_key_unauthorized)
{
	static const uint8_t other_key[32] = {
		0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44,
		0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
	};
	uint8_t buf[256];
	uint8_t key[MESHTASTIC_ADMIN_SESSION_KEY_LEN];
	struct meshtastic_packet pkt;
	size_t len;

	admin_set_is_managed(false);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	seed_peer_pubkey(admin_peer_pubkey);         /* PEER's key in the NodeDB */
	admin_set_admin_key(other_key, sizeof(other_key)); /* a different admin_key */

	meshtastic_admin_session_reset();
	meshtastic_admin_session_current(key);
	len = encode_admin_set_role_key(meshtastic_Config_DeviceConfig_Role_ROUTER, key, sizeof(key),
					buf, sizeof(buf));

	make_remote_admin_packet(&pkt, buf, len, 0U, true);
	pkt.pki_encrypted = true;

	reset_mock_lora();
	meshtastic_admin_handle_remote(&pkt);
	k_sleep(ADMIN_REPLY_SETTLE);
	assert_mock_send_count(1U); /* ADMIN_PUBLIC_KEY_UNAUTHORIZED NAK */
	zassert_equal(admin_current_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "PKC admin whose key is not in admin_key[] must be refused");

	admin_set_admin_key(NULL, 0U);
	admin_force_device_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
	admin_set_is_managed(false);
}

#endif /* CONFIG_MESHTASTIC_ADMIN */
