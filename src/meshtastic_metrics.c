/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <pb_encode.h>

#include <zephyr/drivers/fuel_gauge.h>

#include "meshtastic_modules.h"
#include "meshtastic_telemetry_internal.h"
#include "meshtastic_airtime.h"
#include "meshtastic_sched.h"
#if defined(CONFIG_MESHTASTIC_BATTERY)
#include "meshtastic_battery.h"
#endif

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
#include <zephyr/sys/sys_heap.h>

#include "meshtastic_clock.h"
#include "meshtastic_phoneapi.h"
#if defined(CONFIG_MESHTASTIC_NODEDB)
#include <zephyr/meshtastic/nodedb.h>
#endif
/* The kernel only DEFINES _system_heap when a pool was actually asked for
 * (K_HEAP_MEM_POOL_SIZE > 0 in kernel/mempool.c), so both halves of this guard
 * are load-bearing: a build with runtime stats on and no pool compiles fine and
 * then fails at link. See meshtastic_watchdog.c for why this
 * undocumented-public symbol is the right one to read -- it is the pool
 * k_malloc(), and on the ESP32 port the vendored HAL's heap_caps_malloc(),
 * actually allocates from. */
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS) && (CONFIG_HEAP_MEM_POOL_SIZE > 0)
#define MESHTASTIC_LOCAL_STATS_HAVE_HEAP 1
extern struct k_heap _system_heap;
#endif
#endif /* CONFIG_MESHTASTIC_LOCAL_STATS */

#include <zephyr/meshtastic/telemetry.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if defined(CONFIG_MESHTASTIC_FUEL_GAUGE) && DT_NODE_EXISTS(DT_ALIAS(fuel_gauge0))
#define MESHTASTIC_HAS_FUEL_GAUGE0 1
static const struct device *const fuel_gauge_dev = DEVICE_DT_GET(DT_ALIAS(fuel_gauge0));
#else
#define MESHTASTIC_HAS_FUEL_GAUGE0 0
#endif

/* battery_level > 100 means "powered / no battery" on the wire (telemetry.proto:
 * "0-100 (>100 means powered)"). Upstream sends this MAGIC_USB_BATTERY_LEVEL when a
 * node has no battery or is on external power (DeviceTelemetry.cpp). We advertise it
 * whenever we have no valid state-of-charge reading, so a healthy USB-powered node
 * reports "powered" rather than a bare 0 % that clients render as critically flat. */
#define MESHTASTIC_BATTERY_LEVEL_POWERED 101U

static void collect_fuel_gauge(meshtastic_DeviceMetrics *metrics)
{
#if MESHTASTIC_HAS_FUEL_GAUGE0
	union fuel_gauge_prop_val val;
	int ret;

	if (!device_is_ready(fuel_gauge_dev)) {
		return;
	}

	ret = fuel_gauge_get_prop(fuel_gauge_dev, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &val);
	if (ret == 0) {
		metrics->has_battery_level = true;
		metrics->battery_level = MIN((uint32_t)val.relative_state_of_charge, 100U);
	} else {
		ret = fuel_gauge_get_prop(fuel_gauge_dev, FUEL_GAUGE_ABSOLUTE_STATE_OF_CHARGE,
					  &val);
		if (ret == 0) {
			metrics->has_battery_level = true;
			metrics->battery_level = MIN((uint32_t)val.absolute_state_of_charge, 100U);
		}
	}

	ret = fuel_gauge_get_prop(fuel_gauge_dev, FUEL_GAUGE_VOLTAGE, &val);
	if (ret == 0) {
		metrics->has_voltage = true;
		metrics->voltage = (float)val.voltage / 1000000.0f;
	}
#else
	ARG_UNUSED(metrics);
#endif
}

/*
 * Battery telemetry from the board's VBAT voltage-divider, for the boards that
 * have one instead of a fuel-gauge IC (the Heltec V4 family). Without this the
 * divider reading only ever reached the local display, and every node reported
 * the "powered" sentinel over the mesh no matter what the pack was doing.
 */
static void collect_battery_divider(meshtastic_DeviceMetrics *metrics)
{
#if defined(CONFIG_MESHTASTIC_BATTERY)
	int mv = meshtastic_battery_millivolts();
	int pct;

	if (mv < 0 || !meshtastic_battery_present()) {
		/* No reading, or no cell fitted: leave both fields unset so the
		 * caller's fallback advertises "powered" rather than a phantom
		 * half-volt pack. */
		return;
	}

	metrics->has_voltage = true;
	metrics->voltage = (float)mv / 1000.0f;

	/* Above the charge-termination voltage the pack is being charged, so a
	 * state-of-charge derived from a resting-voltage curve would be wrong.
	 * Upstream reports the same sentinel in that case rather than a bad
	 * percentage. */
	if (meshtastic_battery_external_power()) {
		metrics->has_battery_level = true;
		metrics->battery_level = MESHTASTIC_BATTERY_LEVEL_POWERED;
		return;
	}

	pct = meshtastic_battery_percent();
	if (pct >= 0) {
		metrics->has_battery_level = true;
		metrics->battery_level = (uint32_t)pct;
	}
#else
	ARG_UNUSED(metrics);
#endif
}

int meshtastic_collect_device_metrics(meshtastic_DeviceMetrics *metrics)
{
	if (metrics == NULL) {
		return -EINVAL;
	}

	*metrics = (meshtastic_DeviceMetrics)meshtastic_DeviceMetrics_init_zero;
	metrics->has_uptime_seconds = true;
	metrics->uptime_seconds = k_uptime_seconds();

	collect_fuel_gauge(metrics);
	if (!metrics->has_battery_level) {
		/* A real fuel gauge wins; the divider is the fallback for boards
		 * that only have one of those. */
		collect_battery_divider(metrics);
	}

	if (!metrics->has_battery_level) {
		/* No fuel gauge, or the read failed: report the "powered" sentinel
		 * instead of leaving battery_level unset (which decodes to 0 % and
		 * reads as a dead node). Voltage stays unset — we have no reading. */
		metrics->has_battery_level = true;
		metrics->battery_level = MESHTASTIC_BATTERY_LEVEL_POWERED;
	}

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	metrics->has_channel_utilization = true;
	metrics->channel_utilization = meshtastic_airtime_channel_util_percent();
	metrics->has_air_util_tx = true;
	metrics->air_util_tx = meshtastic_airtime_tx_util_percent();
#endif

	return 0;
}

int meshtastic_send_device_metrics(uint32_t dest, k_timeout_t wait)
{
	meshtastic_DeviceMetrics metrics;
	meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_zero;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	pb_ostream_t stream;
	int ret;

	ret = meshtastic_collect_device_metrics(&metrics);
	if (ret < 0) {
		meshtastic_emit_event(MESHTASTIC_EVENT_METRICS_ERROR, ret, NULL);
		return ret;
	}

	telemetry.which_variant = meshtastic_Telemetry_device_metrics_tag;
	telemetry.variant.device_metrics = metrics;

	stream = pb_ostream_from_buffer(payload, sizeof(payload));
	if (!pb_encode(&stream, meshtastic_Telemetry_fields, &telemetry)) {
		const char *err = PB_GET_ERROR(&stream);

		LOG_ERR("Telemetry encode failed: %s", err);
		meshtastic_emit_event(MESHTASTIC_EVENT_METRICS_ERROR, -ENOMEM, NULL);
		return -ENOMEM;
	}

	return meshtastic_send_data(dest, MESHTASTIC_PORT_TELEMETRY, payload, stream.bytes_written,
				    wait);
}

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)

/* Upstream's NUM_ONLINE_SECS (NodeDB.cpp): a node heard inside the last two
 * hours counts as online. */
#define LOCAL_STATS_ONLINE_SEC (2U * 60U * 60U)

#if defined(CONFIG_MESHTASTIC_NODEDB)
/* Age of a NodeDB entry in seconds, or UINT32_MAX when it cannot be known.
 *
 * Two clocks, deliberately in this order. last_heard_uptime_sec is the uptime AT
 * WHICH we last heard the node, and the NVS load path zeroes it — so non-zero
 * means "heard this boot" and can be differenced against uptime directly. Zero
 * does NOT mean "just now"; reading it that way is exactly the bug 3e57727 fixed
 * in the shell. It means "not heard this boot", and then only the persisted
 * wall-clock epoch can date the entry — and only if a clock has ever been
 * seeded. With neither, the age is unknown, and an unknown age must not be
 * counted as online: on a fleet with no time source that would report every
 * restored entry as live. */
uint32_t meshtastic_local_stats_node_age_sec(const struct meshtastic_nodedb_node *node)
{
	if (node->last_heard_uptime_sec != 0U) {
		uint32_t now = (uint32_t)k_uptime_seconds();

		return (now > node->last_heard_uptime_sec) ? (now - node->last_heard_uptime_sec)
							   : 0U;
	}

	if (node->last_heard_epoch != 0U && meshtastic_clock_valid()) {
		uint32_t now = meshtastic_clock_now_epoch();

		return (now > node->last_heard_epoch) ? (now - node->last_heard_epoch) : 0U;
	}

	return UINT32_MAX;
}
#endif /* CONFIG_MESHTASTIC_NODEDB */

static void count_nodes(uint32_t *total_out, uint32_t *online_out)
{
#if defined(CONFIG_MESHTASTIC_NODEDB)
	size_t total = meshtastic_nodedb_count();
	uint32_t online = 0U;

	for (size_t i = 0; i < total; i++) {
		struct meshtastic_nodedb_node node;

		if (meshtastic_nodedb_get_by_index(i, &node) != 0) {
			continue;
		}
		if (meshtastic_local_stats_node_age_sec(&node) < LOCAL_STATS_ONLINE_SEC) {
			online++;
		}
	}

	*total_out = (uint32_t)total;
	*online_out = online;
#else
	*total_out = 0U;
	*online_out = 0U;
#endif
}

/* The telemetry cadence scales by this (reference numOnlineNodes); the same
 * count LocalStats reports, so the two never disagree about "online". */
uint32_t meshtastic_telemetry_online_nodes(void)
{
	uint32_t total, online;

	count_nodes(&total, &online);
	return online;
}

static void collect_node_counts(meshtastic_LocalStats *stats)
{
	uint32_t total, online;

	count_nodes(&total, &online);
	stats->num_total_nodes = total;
	stats->num_online_nodes = online;
}

static void collect_heap(meshtastic_LocalStats *stats)
{
#if defined(MESHTASTIC_LOCAL_STATS_HAVE_HEAP)
	struct sys_memory_stats heap;

	/* _system_heap backs k_malloc(); the same symbol the watchdog heartbeat
	 * and Zephyr's own `kernel heap` shell command read. free+allocated is
	 * the pool size as the allocator sees it, which is what the wire field
	 * means (upstream fills it from memGet.getHeapSize()). */
	if (sys_heap_runtime_stats_get(&_system_heap.heap, &heap) == 0) {
		stats->heap_free_bytes = (uint32_t)heap.free_bytes;
		stats->heap_total_bytes = (uint32_t)(heap.free_bytes + heap.allocated_bytes);
	}
#else
	ARG_UNUSED(stats);
#endif
}

int meshtastic_collect_local_stats(meshtastic_LocalStats *stats)
{
	struct meshtastic_status status;
	struct meshtastic_sched_stats sched;
	int ret;

	if (stats == NULL) {
		return -EINVAL;
	}

	*stats = (meshtastic_LocalStats)meshtastic_LocalStats_init_zero;

	ret = meshtastic_get_status(&status);
	if (ret < 0) {
		return ret;
	}
	meshtastic_sched_stats_get(&sched);

	stats->uptime_seconds = (uint32_t)k_uptime_seconds();

#if defined(CONFIG_MESHTASTIC_AIRTIME)
	/* Same two numbers device_metrics already carries, and since the per-preset
	 * split both are now meaningful under time-slicing rather than a blend:
	 *
	 *   channel_utilization is scoped to the preset the radio is ON, which is
	 *   the only channel the figure could describe — two presets cannot hear
	 *   each other.
	 *   air_util_tx is deliberately BAND-WIDE. It is the duty-cycle figure, and
	 *   the regulator counts every second the PA is keyed regardless of preset.
	 *
	 * The earlier caution here ("do not build a gauge on these until per-preset
	 * accounting lands") is discharged. See MULTI-PRESET-OPERATION.md §4.3. */
	stats->channel_utilization = meshtastic_airtime_channel_util_percent();
	stats->air_util_tx = meshtastic_airtime_tx_util_percent();
#endif

	stats->num_packets_tx = status.tx_packets;
	/* The wire says num_packets_rx is "both good and bad" and num_packets_rx_bad
	 * is the malformed subset of it. That is exactly how the router counts:
	 * decode_failures is incremented and then rx_packets is incremented on the
	 * same pass (meshtastic_router.c:700/713, no early return between), so a bad
	 * frame lands in both. Do not "fix" this into a disjoint pair. */
	stats->num_packets_rx = status.rx_packets;
	stats->num_packets_rx_bad = status.decode_failures;
	stats->num_rx_dupe = status.duplicate_packets;
	stats->num_tx_relay = status.relayed_packets;
	stats->num_tx_relay_canceled = sched.relay_cancelled;

	for (size_t i = 0; i < ARRAY_SIZE(sched.tx_drop); i++) {
		stats->num_tx_dropped += sched.tx_drop[i];
	}

	collect_node_counts(stats);
	collect_heap(stats);

	/* noise_floor is left at 0. Nothing in this port measures one:
	 * meshtastic_rf_measure.c records per-RECEPTION RSSI/SNR, which is a
	 * different quantity (a property of a frame that arrived, not of the
	 * channel when nothing is on it). Upstream does measure one -- it does
	 * not ask the driver, it samples instantaneous RSSI itself
	 * (RadioLibInterface::updateNoiseFloor: GetRssiInst in idle receive,
	 * gated on !isActivelyReceiving() && !isIRQPending(), every 5 s, a
	 * 20-sample mean, default -120 dBm) -- and its only consumer is this
	 * same telemetry field. Reporting-only; tracked in the tooling repo's
	 * docs/RF-PARITY-PLAN.md §4, buildable now that the activity gate
	 * (meshtastic_radio_actively_receiving) exists. */

	return 0;
}

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE)
/* Only the phone path encodes a bare Telemetry: the on-request mesh reply goes
 * through meshtastic_telemetry_encode_packet() instead, which builds a whole
 * reply packet. Scoped inside this guard so a LOCAL_STATS=y / TO_PHONE=n build
 * does not carry -- or -Werror on -- a function nothing calls.
 */
static int local_stats_encode(meshtastic_LocalStats *stats, uint8_t *payload, size_t payload_size,
			      size_t *written)
{
	meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_zero;
	pb_ostream_t stream;
	int ret;

	ret = meshtastic_collect_local_stats(stats);
	if (ret < 0) {
		return ret;
	}

	telemetry.which_variant = meshtastic_Telemetry_local_stats_tag;
	telemetry.variant.local_stats = *stats;

	stream = pb_ostream_from_buffer(payload, payload_size);
	if (!pb_encode(&stream, meshtastic_Telemetry_fields, &telemetry)) {
		LOG_ERR("LocalStats encode failed: %s", PB_GET_ERROR(&stream));
		return -ENOMEM;
	}

	*written = stream.bytes_written;
	return 0;
}

/* Hand LocalStats straight to whatever phone transports are attached, without
 * touching the radio.
 *
 * This is the whole point of the layer and it mirrors upstream exactly:
 * DeviceTelemetryModule::sendLocalStatsToPhone() calls service->sendToPhone(),
 * never sendToMesh(). LocalStats is a node describing ITSELF to its own client,
 * so broadcasting it would spend airtime telling the mesh things the mesh has no
 * use for. The only way it ever reaches the air is as a reply to an explicit
 * Telemetry(local_stats) request — see the alloc_reply handler below.
 *
 * meshtastic_phoneapi_on_packet() is the established local-emit path (the admin
 * module uses it the same way for its PhoneAPI-bound replies): it wraps the
 * packet as a FromRadio and fans it out to every registered transport. A
 * TELEMETRY_APP frame is already classified droppable by the queue, so a phone
 * that is not reading cannot starve real traffic out of the queue. */
int meshtastic_send_local_stats_to_phone(void)
{
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_LocalStats stats;
	struct meshtastic_packet pkt = {0};
	size_t len;
	int ret;

	ret = local_stats_encode(&stats, payload, sizeof(payload), &len);
	if (ret < 0) {
		return ret;
	}

	pkt.portnum = MESHTASTIC_PORT_TELEMETRY;
	pkt.from = meshtastic_get_node_id();
	pkt.to = MESHTASTIC_NODE_BROADCAST;
	pkt.id = meshtastic_allocate_packet_id();
	pkt.payload = payload;
	pkt.payload_len = (uint16_t)len;

	meshtastic_phoneapi_on_packet(&pkt, NULL);

	LOG_DBG("LocalStats to phone: up=%us tx=%u rx=%u bad=%u dupe=%u relay=%u/%u nodes=%u/%u",
		stats.uptime_seconds, stats.num_packets_tx, stats.num_packets_rx,
		stats.num_packets_rx_bad, stats.num_rx_dupe, stats.num_tx_relay,
		stats.num_tx_relay_canceled, stats.num_online_nodes, stats.num_total_nodes);

	return 0;
}
#endif /* CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE */

#endif /* CONFIG_MESHTASTIC_LOCAL_STATS */

#if defined(CONFIG_MESHTASTIC_TELEMETRY_WANT_RESPONSE)

#define DEVICE_TELEMETRY_PEER_SLOTS 4

struct device_telemetry_peer {
	uint32_t from;
	int64_t last_reply_ms;
	bool reply_time_valid;
};

static struct {
	struct k_mutex lock;
	struct device_telemetry_peer peers[DEVICE_TELEMETRY_PEER_SLOTS];
} device_telemetry_state;

static bool interval_elapsed(bool valid, int64_t last_ms, int64_t now_ms, int64_t interval_ms)
{
	return !valid || (now_ms - last_ms) >= interval_ms;
}

static struct device_telemetry_peer *peer_get_locked(uint32_t from, int64_t now_ms)
{
	struct device_telemetry_peer *oldest = &device_telemetry_state.peers[0];
	int64_t oldest_ms = INT64_MAX;

	for (size_t i = 0; i < ARRAY_SIZE(device_telemetry_state.peers); i++) {
		struct device_telemetry_peer *p = &device_telemetry_state.peers[i];

		if (p->from == from) {
			return p;
		}

		if (!p->reply_time_valid) {
			p->from = from;
			p->reply_time_valid = false;
			return p;
		}

		if (p->last_reply_ms < oldest_ms) {
			oldest_ms = p->last_reply_ms;
			oldest = p;
		}
	}

	oldest->from = from;
	oldest->reply_time_valid = false;
	return oldest;
}

static int meshtastic_module_device_telemetry_alloc_reply(const struct meshtastic_packet *req,
							  const meshtastic_MeshPacket *mesh,
							  struct meshtastic_packet *reply)
{
	static uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
	meshtastic_Telemetry request = meshtastic_Telemetry_init_zero;
	meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_zero;
	meshtastic_DeviceMetrics metrics;
	struct device_telemetry_peer *peer;
	bool want_local_stats = false;
	int64_t now_ms;
	int ret;

	if (req == NULL || reply == NULL || req->from == 0U ||
	    req->from == meshtastic_get_node_id()) {
		return -EINVAL;
	}

	/* Phase 5c: decode the request payload from the MeshPacket (currency) on the
	 * RF path, struct fallback on the NULL-mesh boundary (byte-identical). The
	 * reply envelope (dest/channel/response_to_id) stays sourced from req until
	 * the send-path currency migration (Phase 6) reworks the reply builders. */
	if (!meshtastic_telemetry_decode_request(mesh ? mesh->decoded.payload.bytes : req->payload,
						 mesh ? mesh->decoded.payload.size : req->payload_len,
						 &request)) {
		return -ENOENT;
	}

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
	/* Upstream answers a local_stats request on this same port and module
	 * (DeviceTelemetry.cpp allocReply()). This is the ONLY way LocalStats ever
	 * costs airtime: somebody asked for it by name. */
	want_local_stats = (request.which_variant == meshtastic_Telemetry_local_stats_tag);
#endif

	if (request.which_variant != meshtastic_Telemetry_device_metrics_tag &&
	    request.which_variant != 0U && !want_local_stats) {
		return -ENOENT;
	}

	now_ms = k_uptime_get();
	k_mutex_lock(&device_telemetry_state.lock, K_FOREVER);
	peer = peer_get_locked(req->from, now_ms);
	if (!interval_elapsed(peer->reply_time_valid, peer->last_reply_ms, now_ms,
			      (int64_t)CONFIG_MESHTASTIC_TELEMETRY_REPLY_SUPPRESS_SEC *
				      MSEC_PER_SEC)) {
		k_mutex_unlock(&device_telemetry_state.lock);
		return -ENOENT;
	}
	peer->reply_time_valid = true;
	peer->last_reply_ms = now_ms;
	k_mutex_unlock(&device_telemetry_state.lock);

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS)
	if (want_local_stats) {
		meshtastic_LocalStats stats;

		ret = meshtastic_collect_local_stats(&stats);
		if (ret < 0) {
			return ret;
		}

		telemetry.which_variant = meshtastic_Telemetry_local_stats_tag;
		telemetry.variant.local_stats = stats;
	} else
#endif
	{
		ret = meshtastic_collect_device_metrics(&metrics);
		if (ret < 0) {
			return ret;
		}

		telemetry.which_variant = meshtastic_Telemetry_device_metrics_tag;
		telemetry.variant.device_metrics = metrics;
	}

	ret = meshtastic_telemetry_encode_packet(req->from, req->id, &telemetry, payload, reply);
	if (ret == 0) {
		LOG_INF("Device telemetry request from 0x%08x, sending %s response", req->from,
			want_local_stats ? "LocalStats" : "DeviceMetrics");
	}

	return ret;
}

MESHTASTIC_MODULE_DEFINE(device_telemetry, MESHTASTIC_PORT_TELEMETRY, 0, NULL,
			 meshtastic_module_device_telemetry_alloc_reply);

#endif /* CONFIG_MESHTASTIC_TELEMETRY_WANT_RESPONSE */

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS_AUTO_SEND) ||                                         \
	defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE) ||                                         \
	(defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS) &&                                         \
	 defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS_AUTO_SEND))
#define MESHTASTIC_TELEMETRY_THREAD 1
/* 2026-08-05: the "~300 B measured peak" below was stale — it measured the
 * bounded build+encode helper in isolation, not the actual call this thread
 * makes. meshtastic_send_device_metrics() calls into meshtastic_send_data(),
 * whose C3-migrated mesh-native wire-builder path measured ~3012/3072 (98%)
 * on a *different*, larger-stack thread (meshtastic_shell, manually
 * triggering the same send via `meshtastic metrics send`) — this dedicated
 * 2048 B stack overflowed on every real firing of the 1-hour auto-send timer
 * across all 3 bench nodes (STACK_SENTINEL didn't catch it: a fast/severe
 * overflow corrupts before the next-context-switch check fires). 6 KB keeps
 * the same ~50% margin now targeted for other threads on this send path. */
static K_THREAD_STACK_DEFINE(telemetry_stack, 6144);
static struct k_thread telemetry_thread;

/*
 * One thread, several senders, each on its OWN period.
 *
 * The previous loop slept for the MINIMUM of the configured intervals and then
 * fired every enabled sender on each tick, so two senders with different
 * intervals both effectively ran at the shorter one. That was invisible while
 * both intervals defaulted to 3600 s — and it stops being invisible the moment
 * anything on this thread wants a shorter period. The LocalStats push does
 * (900 s, matching upstream), and under the old loop that would have quietly
 * made the device-metrics BROADCAST 4x chattier on the air: exactly the
 * regression the 3600 s default was chosen to avoid ("900 s was ~4x chattier",
 * Kconfig.device_metrics). So each sender now carries its own deadline.
 */
struct telemetry_job {
	/* Resolve "should this fire, and how often" from the store (or Kconfig for
	 * the phone push). Read at every pass: a ModuleConfig.telemetry write
	 * takes effect at the next deadline without a reboot (agents-dnr4.10). */
	bool (*resolve)(uint32_t *interval_sec);
	void (*fire)(void);
	uint32_t last_sec; /* uptime second this sender last fired (or was enabled) */
	bool enabled;      /* as of the last resolve: an off->on edge restarts the clock */
};

/* Resolve a job and keep its clock honest: while disabled the clock is held at
 * now, and the moment it turns on it is restarted, so enabling fires one full
 * interval later rather than at once (the thread may have slept for an hour
 * with the job off, and "last fired an hour ago" must not count). */
static bool job_resolve(struct telemetry_job *job, uint32_t now, uint32_t *interval_sec)
{
	bool on = job->resolve(interval_sec);

	if (!on || !job->enabled) {
		job->last_sec = now;
	}
	job->enabled = on;
	return on;
}

#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS_AUTO_SEND)
static bool resolve_device_metrics(uint32_t *interval_sec)
{
	struct meshtastic_telemetry_settings s;

	meshtastic_telemetry_settings(&s);
	*interval_sec = s.device_interval_sec;
	return s.device_enabled;
}

static void fire_device_metrics(void)
{
	(void)meshtastic_send_device_metrics(MESHTASTIC_NODE_BROADCAST, K_NO_WAIT);
}
#endif

#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS) &&                                              \
	defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS_AUTO_SEND)
static bool resolve_environment(uint32_t *interval_sec)
{
	struct meshtastic_telemetry_settings s;

	meshtastic_telemetry_settings(&s);
	*interval_sec = s.environment_interval_sec;
	return s.environment_enabled;
}

static void fire_environment(void)
{
	(void)meshtastic_send_environment(MESHTASTIC_NODE_BROADCAST, K_NO_WAIT);
}
#endif

#if defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE)
/* Reference sendStatsToPhoneIntervalMs is a constant, not a config field. */
static bool resolve_local_stats(uint32_t *interval_sec)
{
	*interval_sec = CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE_SEC;
	return true;
}

static void fire_local_stats(void)
{
	(void)meshtastic_send_local_stats_to_phone();
}
#endif

/* Never sleep longer than this even with nothing due: the role or the
 * online-node count can move the resolved interval without any config write. */
#define TELEMETRY_MAX_SLEEP_SEC 3600U

static void telemetry_thread_fn(void *p1, void *p2, void *p3)
{
	static struct telemetry_job jobs[] = {
#if defined(CONFIG_MESHTASTIC_DEVICE_METRICS_AUTO_SEND)
		{ .resolve = resolve_device_metrics, .fire = fire_device_metrics },
#endif
#if defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS) &&                                              \
	defined(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS_AUTO_SEND)
		{ .resolve = resolve_environment, .fire = fire_environment },
#endif
#if defined(CONFIG_MESHTASTIC_LOCAL_STATS_TO_PHONE)
		{ .resolve = resolve_local_stats, .fire = fire_local_stats },
#endif
	};
	uint32_t now = (uint32_t)k_uptime_seconds();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (size_t i = 0; i < ARRAY_SIZE(jobs); i++) {
		jobs[i].last_sec = now;
		jobs[i].enabled = true; /* a job that starts disabled is caught by job_resolve */
	}

	while (true) {
		uint32_t sleep_sec = TELEMETRY_MAX_SLEEP_SEC;

		now = (uint32_t)k_uptime_seconds();
		for (size_t i = 0; i < ARRAY_SIZE(jobs); i++) {
			uint32_t interval, due, remaining;

			if (!job_resolve(&jobs[i], now, &interval)) {
				continue;
			}
			due = jobs[i].last_sec + interval;
			remaining = (due > now) ? (due - now) : 0U;
			sleep_sec = MIN(sleep_sec, remaining);
		}

		/* Never a zero sleep: a job that is already overdue is fired below,
		 * and a rounding-down of sub-second remainder must not spin. A config
		 * write wakes this early (meshtastic_telemetry_config_changed); the
		 * deadlines are simply re-resolved on the next pass. */
		k_sleep(K_SECONDS(MAX(sleep_sec, 1U)));

		now = (uint32_t)k_uptime_seconds();
		for (size_t i = 0; i < ARRAY_SIZE(jobs); i++) {
			uint32_t interval;

			if (job_resolve(&jobs[i], now, &interval) &&
			    now >= jobs[i].last_sec + interval) {
				jobs[i].fire();
				/* Next deadline measured from NOW, not from the missed
				 * one -- a slow send must not queue up a catch-up burst. */
				jobs[i].last_sec = now;
			}
		}
	}
}
#endif /* MESHTASTIC_TELEMETRY_THREAD */

int meshtastic_metrics_init(void)
{
#if defined(CONFIG_MESHTASTIC_AIRTIME)
	int ret;

	ret = meshtastic_airtime_init();
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_MESHTASTIC_TELEMETRY_WANT_RESPONSE)
	k_mutex_init(&device_telemetry_state.lock);
#endif

#if defined(MESHTASTIC_TELEMETRY_THREAD)
	k_thread_create(&telemetry_thread, telemetry_stack, K_THREAD_STACK_SIZEOF(telemetry_stack),
			telemetry_thread_fn, NULL, NULL, NULL, CONFIG_MESHTASTIC_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&telemetry_thread, "meshtastic_telemetry");
	meshtastic_telemetry_cadence_watch(&telemetry_thread);
#endif

	return 0;
}
