/* SPDX-License-Identifier: GPL-3.0
 *
 * The device-metrics broadcast thread, driven by ModuleConfig.telemetry
 * (agents-dnr4.10). tests/telemetry proves the RESOLUTION of the section (the
 * defaults, the coercion, the scaling) without the stack; this suite proves the
 * thread OBEYS it: the frame lands on the sim radio when the interval says,
 * a write re-arms the deadline without a reboot, a disable stops the sender,
 * and a stored record nobody ever configured follows the build's seed after a
 * load -- the case an existing bench node is in at its next flash.
 *
 * Simulated time makes a 30 s interval free to wait out.
 */
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/meshtastic/meshtastic.h>
#include <meshtastic/lora_sim.h>
#include "meshtastic/mesh.pb.h"
#include "meshtastic/telemetry.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_packet.h"
#include "meshtastic_telemetry_internal.h"

#define TEST_NODE_ID 0x0A0A0A0AU

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static void *cadence_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};
	meshtastic_Channel ch;

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");

	/* Off the default channel: a configured interval below 30 min would
	 * otherwise be coerced up (tests/telemetry covers that rule), and these
	 * tests want intervals simulated time can wait out. Through the STORE,
	 * because apply_core() (which one test runs) re-copies the stored channel
	 * table over the live one. */
	ch = *meshtastic_channels_get(meshtastic_channels_primary_index());
	strcpy(ch.settings.name, "bench");
	zassert_ok(meshtastic_config_store_set_channel(meshtastic_channels_primary_index(), &ch),
		   "");
	zassert_ok(meshtastic_channels_set_slot(meshtastic_channels_primary_index(), &ch), "");
	return NULL;
}

ZTEST_SUITE(telemetry_cadence, NULL, cadence_setup, NULL, NULL, NULL);

static void store_telemetry(uint32_t device_iv, bool device_en)
{
	meshtastic_ModuleConfig mod = meshtastic_ModuleConfig_init_zero;

	mod.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
	mod.payload_variant.telemetry.device_update_interval = device_iv;
	mod.payload_variant.telemetry.device_telemetry_enabled = device_en;
	zassert_ok(meshtastic_config_store_set_module(&mod), "store write");
	/* What the admin path does after the write. */
	meshtastic_telemetry_config_changed();
}

/* Next DeviceMetrics broadcast on the sim radio within timeout_ms, skipping any
 * other frame. 0 and the frame, or -EAGAIN. */
static int take_device_metrics(uint32_t timeout_ms, struct lora_sim_frame *out)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (true) {
		int64_t left = deadline - k_uptime_get();
		struct meshtastic_packet pkt;
		uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
		meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
		pb_istream_t is;
		int ret;

		if (left <= 0) {
			return -EAGAIN;
		}
		ret = lora_sim_take_tx(lora_dev, out, K_MSEC(left));
		if (ret < 0) {
			return -EAGAIN;
		}
		if (meshtastic_decode_wire_packet(out->data, out->len, 0, 0, &pkt, payload,
						  sizeof(payload)) != 0 ||
		    pkt.portnum != MESHTASTIC_PORT_TELEMETRY) {
			continue;
		}
		is = pb_istream_from_buffer(pkt.payload, pkt.payload_len);
		if (pb_decode(&is, meshtastic_Telemetry_fields, &t) &&
		    t.which_variant == meshtastic_Telemetry_device_metrics_tag) {
			return 0;
		}
	}
}

/* Runs first (ztest order is by name; the later tests stamp the section by
 * writing it, and this one needs it unstamped). The upgrade case: a telemetry
 * record loaded from settings with every flag zero and NO write-stamp is what a
 * node flashed from a build that never read the flags carries. After the
 * post-load reconcile the build's seed rules; after an admin write (stamped),
 * the written value rules. */
ZTEST(telemetry_cadence, test_0_unstamped_record_follows_the_seed_after_a_load)
{
	uint8_t rec[128];
	int rec_len;
	struct meshtastic_telemetry_settings s;

	/* An all-zero telemetry record in the store's own on-flash format: write
	 * zeros, read the record back through the settings getter. */
	store_telemetry(0U, false);
	rec_len = meshtastic_config_store_setting_get("module/telemetry", rec, sizeof(rec));
	zassert_true(rec_len > 0, "record encode (%d)", rec_len);

	/* What the settings load does on an upgraded node: the record lands, and
	 * its stamp record is absent/unset. */
	zassert_ok(meshtastic_config_store_setting_set("module/telemetry", rec, (size_t)rec_len),
		   "");
	zassert_ok(meshtastic_config_store_setting_set("hlc/module/telemetry", rec, 0U), "");
	meshtastic_telemetry_settings(&s);
	zassert_false(s.device_enabled, "raw loaded record: flag is zero");

	zassert_ok(meshtastic_config_store_apply_core(), "post-load apply");
	meshtastic_telemetry_settings(&s);
	zassert_equal(s.device_enabled, IS_ENABLED(CONFIG_MESHTASTIC_DEVICE_METRICS_AUTO_SEND),
		      "unstamped record: the build's seed rules");

	/* An admin write stamps it; a disable then survives the same apply. */
	store_telemetry(0U, false);
	zassert_ok(meshtastic_config_store_apply_core(), "");
	meshtastic_telemetry_settings(&s);
	zassert_false(s.device_enabled, "stamped record: the written value rules");
	store_telemetry(0U, true);
}

ZTEST(telemetry_cadence, test_broadcast_follows_the_stored_interval)
{
	struct lora_sim_frame first, second;
	int64_t t0 = k_uptime_get();

	/* Zero interval: the Kconfig default (30 s here). */
	store_telemetry(0U, true);
	lora_sim_reset(lora_dev);
	zassert_ok(take_device_metrics(45000U, &first), "a DeviceMetrics broadcast within 45 s");
	zassert_true(first.t_ms - t0 >= 29000, "not before the interval elapsed (%lld ms)",
		     (long long)(first.t_ms - t0));

	/* A longer interval written mid-flight re-arms from the last send: the
	 * next frame is one NEW interval after the previous one, no reboot. */
	store_telemetry(50U, true);
	zassert_ok(take_device_metrics(70000U, &second), "next broadcast within 70 s");
	zassert_true(second.t_ms - first.t_ms >= 49000 && second.t_ms - first.t_ms <= 53000,
		     "spaced by the new interval (%lld ms)",
		     (long long)(second.t_ms - first.t_ms));
}

ZTEST(telemetry_cadence, test_disable_stops_and_enable_rearms)
{
	struct lora_sim_frame frame;
	int64_t enabled_at;

	store_telemetry(0U, false);
	lora_sim_reset(lora_dev);
	zassert_equal(take_device_metrics(100000U, &frame), -EAGAIN,
		      "disabled: nothing on the air across three intervals");

	/* Enabling fires one full interval later, not at once (the reference's
	 * lastTelemetry stays where it was; here the clock kept running). */
	enabled_at = k_uptime_get();
	store_telemetry(0U, true);
	zassert_ok(take_device_metrics(45000U, &frame), "enabled: broadcast resumes");
	zassert_true(frame.t_ms - enabled_at >= 29000, "one interval after the enable (%lld ms)",
		     (long long)(frame.t_ms - enabled_at));
}
