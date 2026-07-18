#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
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
	uint8_t last_rx_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	uint8_t last_event_payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	size_t last_rx_payload_len;
	size_t last_event_payload_len;
	struct meshtastic_event last_event;
	uint32_t recv_count;
	uint32_t event_count;
};

static struct test_state state;

static void reset_callbacks_state(void)
{
	memset(&state.last_rx, 0, sizeof(state.last_rx));
	memset(&state.last_event_packet, 0, sizeof(state.last_event_packet));
	memset(state.last_rx_payload, 0, sizeof(state.last_rx_payload));
	memset(state.last_event_payload, 0, sizeof(state.last_event_payload));
	memset(&state.last_event, 0, sizeof(state.last_event));
	state.last_rx_payload_len = 0U;
	state.last_event_payload_len = 0U;
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

static void assert_event_payload(const void *expected, size_t expected_len)
{
	zassert_equal(state.last_event_payload_len, expected_len, "unexpected event payload len");
	if (expected_len > 0U) {
		zassert_mem_equal(state.last_event_payload, expected, expected_len,
				  "unexpected event payload");
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

/* Inject a ROUTING ACK (or NAK) from PEER to us correlating request_id. */
static void inject_routing_ack(uint32_t request_id, meshtastic_Routing_Error err)
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
	zassert_equal(state.last_event.type, MESHTASTIC_EVENT_PACKET_RECEIVED,
		      "unexpected downlink event");
	zassert_true(state.last_event_packet.via_mqtt, "downlink should be marked via MQTT");
	assert_event_payload(body, sizeof(body) - 1U);
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
