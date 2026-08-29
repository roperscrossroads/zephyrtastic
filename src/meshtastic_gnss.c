/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 *
 * GNSS position *source*. Subscribes to the Zephyr GNSS driver, converts fixes
 * into meshtastic_Position, and feeds them to the Position module
 * (meshtastic_position.c), which owns the portnum handling, TX, replies, and
 * fixed-position support. Keeping the source separate lets a GNSS-less node
 * still advertise a manually-set fixed position.
 */

#include <errno.h>
#include <string.h>
#include <time.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>

#include <zephyr/meshtastic/gnss_pps.h>

#include "meshtastic_clock.h"
#include "meshtastic_gnss.h"

#include "meshtastic_position.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
#define MESHTASTIC_HAS_GNSS_ALIAS 1
#define MESHTASTIC_GNSS_NODE      DT_ALIAS(gnss)
static const struct device *const gnss_dev = DEVICE_DT_GET(MESHTASTIC_GNSS_NODE);
#else
#define MESHTASTIC_HAS_GNSS_ALIAS 0
#endif

static struct {
	struct k_mutex lock;
	bool has_fix;
	int64_t last_sent_ms;
	int64_t last_attempt_ms;
} gnss_state;

#if MESHTASTIC_HAS_GNSS_ALIAS
static uint32_t mdeg_to_centideg(uint32_t bearing_mdeg)
{
	return bearing_mdeg / 10U;
}

static uint32_t hdop_to_centidop(uint32_t hdop_milli)
{
	return hdop_milli / 10U;
}

static void fill_position(const struct gnss_data *data, meshtastic_Position *position)
{
	*position = (meshtastic_Position)meshtastic_Position_init_zero;

	position->has_latitude_i = true;
	position->latitude_i = (int32_t)(data->nav_data.latitude / 100);
	position->has_longitude_i = true;
	position->longitude_i = (int32_t)(data->nav_data.longitude / 100);
	position->has_altitude = true;
	position->altitude = data->nav_data.altitude / 1000;
	position->location_source = meshtastic_Position_LocSource_LOC_INTERNAL;
	position->altitude_source = meshtastic_Position_AltSource_ALT_INTERNAL;
	position->HDOP = hdop_to_centidop(data->info.hdop);
	position->fix_quality = data->info.fix_quality;
	position->fix_type = (data->info.fix_status == GNSS_FIX_STATUS_DGNSS_FIX) ? 3U
			     : (data->info.fix_status == GNSS_FIX_STATUS_NO_FIX)  ? 0U
										  : 2U;
	position->sats_in_view = data->info.satellites_cnt;
	position->next_update = CONFIG_MESHTASTIC_GNSS_SEND_INTERVAL_SEC;
	position->precision_bits = 32U;
	position->time = meshtastic_clock_now_epoch(); /* epoch secs, 0 if unseeded */

	if (data->nav_data.speed != 0U) {
		position->has_ground_speed = true;
		position->ground_speed = data->nav_data.speed / 1000U;
	}

	if (data->nav_data.bearing != 0U) {
		position->has_ground_track = true;
		position->ground_track = mdeg_to_centideg(data->nav_data.bearing);
	}

	if (data->info.geoid_separation != 0) {
		position->has_altitude_geoidal_separation = true;
		position->altitude_geoidal_separation = data->info.geoid_separation / 1000;
	}
}

/* The whole automatic-send apparatus — queue, stack, work item and handler —
 * lives under one guard. Leaving any of it outside is not merely dead code: the
 * work item is only ever referenced from the submit site below, so with
 * AUTO_SEND=n it becomes an unused static and -Werror=unused-variable fails the
 * build. Found by the AUTO_SEND=n test configuration (agents-isfr); the earlier
 * fix in gnss_data_cb() closed the reference but left this behind, and the
 * application build does not use -Werror, so only twister sees it. */
#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
K_THREAD_STACK_DEFINE(gnss_send_wq_stack, CONFIG_MESHTASTIC_GNSS_SEND_WORK_STACK_SIZE);
static struct k_work_q gnss_send_wq;

static void position_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	/* G-5: periodic auto-broadcast goes out fire-and-forget so the airtime gate
	 * can throttle it under congestion (a manual send stays blocking/ungated). */
	ret = meshtastic_send_position_periodic();
	if (ret == -ENODATA) {
		return;
	}
	if (ret < 0) {
		LOG_ERR("Position TX failed (%d)", ret);
		return;
	}

	k_mutex_lock(&gnss_state.lock, K_FOREVER);
	gnss_state.last_sent_ms = k_uptime_get();
	k_mutex_unlock(&gnss_state.lock);
}

static K_WORK_DEFINE(position_send_work, position_work_handler);
#endif /* CONFIG_MESHTASTIC_GNSS_AUTO_SEND */

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	meshtastic_Position position;
	int64_t now;
	int64_t send_interval_ms;
	int64_t retry_interval_ms;
	bool due;
	bool can_retry;

#if defined(CONFIG_MESHTASTIC_GNSS_TIME_DEBUG)
	/* Stamp FIRST, and before the no-fix gate. Everything below this point —
	 * including the gate itself — is delay we would otherwise attribute to the
	 * receiver. And a module that is tracking satellites but has not yet fixed
	 * is exactly the state we most need to see on an indoor bench, which the
	 * gate would otherwise make invisible. */
	{
		uint32_t cyc = k_cycle_get_32();
		int64_t up = k_uptime_get();

		if (data != NULL) {
			uint32_t ms = data->utc.millisecond;

			/* gnss_time carries no seconds field: seconds AND milliseconds
			 * both live in .millisecond, range [0, 60999]. */
			LOG_INF("gnsst cyc=%u up=%lld utc=%02u:%02u:%02u.%03u "
				"date=%02u/%02u/%02u fix=%u sats=%u",
				cyc, up, (unsigned int)data->utc.hour,
				(unsigned int)data->utc.minute,
				(unsigned int)(ms / MSEC_PER_SEC),
				(unsigned int)(ms % MSEC_PER_SEC),
				(unsigned int)data->utc.month_day, (unsigned int)data->utc.month,
				(unsigned int)data->utc.century_year,
				(unsigned int)data->info.fix_status,
				(unsigned int)data->info.satellites_cnt);
		} else {
			LOG_INF("gnsst cyc=%u up=%lld (null data)", cyc, up);
		}
	}
#endif

	if (dev != gnss_dev || data == NULL || data->info.fix_status == GNSS_FIX_STATUS_NO_FIX) {
		return;
	}

	/* A position fix implies valid UTC — seed the wall clock so uptime-relative
	 * timestamps (e.g. NodeInfo.last_heard, own Position.time) can be reported as
	 * epoch. gnss_time has no seconds field; seconds live in millisecond. The
	 * clock helper rejects pre-2020 epochs, guarding against a bogus date. */
	{
		struct tm gnss_tm = {
			.tm_year = 100 + data->utc.century_year, /* years since 1900 */
			.tm_mon = (int)data->utc.month - 1,      /* 0-11 */
			.tm_mday = data->utc.month_day,
			.tm_hour = data->utc.hour,
			.tm_min = data->utc.minute,
			.tm_sec = (int)(data->utc.millisecond / MSEC_PER_SEC),
		};
		int64_t epoch = timeutil_timegm64(&gnss_tm);
		int64_t edge_uptime_ms;
		int64_t edge_age_ms;

		if (epoch > 0) {
			/*
			 * WHEN was this true, not just WHAT is it.
			 *
			 * The sentence names second N, but it is handed to us well after
			 * second N began: the receiver's fix latency plus the whole
			 * sentence's UART time, because the modem layer only delivers on a
			 * complete match. Measured here at 853 ms, and one-sided, so
			 * anchoring at arrival makes the clock late by that whole amount
			 * however precisely the receiver knew the time.
			 *
			 * The 1PPS edge IS the moment second N began. Pairing the two —
			 * digits from the sentence, instant from the edge — is what turns a
			 * sub-second-accurate source into a sub-second-accurate clock.
			 *
			 * The pairing rule is "the most recent edge", and it is sound only
			 * while delivery takes less than a second: the edge for N arrives
			 * first, the sentence naming N follows. If an edge were missed the
			 * most recent one would belong to N+1 and the clock would be a
			 * whole second fast — silent, plausible, and the worst possible
			 * size of error. meshtastic_gnss_pps_last_edge() refuses any edge
			 * older than a second, and refuses entirely until the pulse train
			 * has been seen to be 1 Hz, so both of those become a fallback
			 * rather than a wrong answer.
			 */
			if (meshtastic_gnss_pps_last_edge(&edge_uptime_ms, &edge_age_ms)) {
				meshtastic_clock_set_epoch_ms_at(epoch * MSEC_PER_SEC,
								 edge_uptime_ms,
								 MESHTASTIC_CLOCK_QUALITY_GPS);
				LOG_DBG("gnss: clock anchored on PPS edge %lld ms ago",
					edge_age_ms);
			} else {
				/*
				 * No usable edge: anchor at arrival and add back the delivery
				 * latency, if it has been measured for this receiver.
				 * MESHTASTIC_GNSS_FIX_LATENCY_MS defaults to 0 — the historical
				 * behaviour — because the figure is a property of the module and
				 * its baud rate, not of this code, and a guessed constant is
				 * worse than an honest offset of zero.
				 */
				meshtastic_clock_set_epoch_ms(
					epoch * MSEC_PER_SEC +
						CONFIG_MESHTASTIC_GNSS_FIX_LATENCY_MS,
					MESHTASTIC_CLOCK_QUALITY_GPS);
			}
		}
	}

	/* Hand the fresh fix to the Position module before deciding to broadcast. */
	fill_position(data, &position);
	meshtastic_position_set_current(&position);

	send_interval_ms = (int64_t)CONFIG_MESHTASTIC_GNSS_SEND_INTERVAL_SEC * MSEC_PER_SEC;
	retry_interval_ms = (int64_t)CONFIG_MESHTASTIC_GNSS_RETRY_INTERVAL_SEC * MSEC_PER_SEC;

	k_mutex_lock(&gnss_state.lock, K_FOREVER);
	gnss_state.has_fix = true;
	now = k_uptime_get();

	due = (now - gnss_state.last_sent_ms) >= send_interval_ms;
	can_retry = (now - gnss_state.last_attempt_ms) >= retry_interval_ms;

	/* #if, not IS_ENABLED(): IS_ENABLED keeps both arms COMPILED so the
	 * compiler can check them, which is normally the point — but gnss_send_wq
	 * only EXISTS under this same guard, so referencing it from a live arm
	 * fails to build with AUTO_SEND=n. (Found 2026-08-29 building the first
	 * listener image, which is the first config to turn it off.) */
#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
	if (due && can_retry && !k_work_busy_get(&position_send_work)) {
		gnss_state.last_attempt_ms = now;
		k_work_submit_to_queue(&gnss_send_wq, &position_send_work);
	}
#else
	ARG_UNUSED(due);
	ARG_UNUSED(can_retry);
#endif
	k_mutex_unlock(&gnss_state.lock);

	meshtastic_emit_event(MESHTASTIC_EVENT_GNSS_FIX, 0, NULL);
}

GNSS_DT_DATA_CALLBACK_DEFINE(MESHTASTIC_GNSS_NODE, gnss_data_cb);
#endif

/* Both stamps start one full interval in the past so the very first fix after
 * boot is immediately due rather than waiting out an interval the node spent
 * unpowered. */
static void gnss_gate_reset(void)
{
	gnss_state.last_sent_ms =
		-((int64_t)CONFIG_MESHTASTIC_GNSS_SEND_INTERVAL_SEC * MSEC_PER_SEC);
	gnss_state.last_attempt_ms =
		-((int64_t)CONFIG_MESHTASTIC_GNSS_RETRY_INTERVAL_SEC * MSEC_PER_SEC);
}

#if defined(CONFIG_ZTEST)
void meshtastic_gnss_test_reset(void)
{
	k_mutex_lock(&gnss_state.lock, K_FOREVER);
	gnss_gate_reset();
	gnss_state.has_fix = false;
	k_mutex_unlock(&gnss_state.lock);
}
#endif

int meshtastic_gnss_init(void)
{
	k_mutex_init(&gnss_state.lock);
	gnss_gate_reset();

#if MESHTASTIC_HAS_GNSS_ALIAS
#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
	k_work_queue_start(&gnss_send_wq, gnss_send_wq_stack,
			   K_THREAD_STACK_SIZEOF(gnss_send_wq_stack),
			   CONFIG_MESHTASTIC_GNSS_SEND_WORK_PRIORITY, NULL);
#endif

	if (!device_is_ready(gnss_dev)) {
		LOG_WRN("GNSS alias exists but device is not ready");
		return 0;
	}

	LOG_INF("Meshtastic position module using %s", gnss_dev->name);

	/* The gnss-nmea-generic driver boots pm_device_init_suspended() and only opens
	 * its UART pipe (i.e. starts reading NMEA) on RESUME. With CONFIG_PM_DEVICE=y and
	 * neither runtime nor system-managed PM, nothing resumes it — so without this
	 * explicit resume the module is never read at all (zero NMEA, no fix). */
	if (IS_ENABLED(CONFIG_PM_DEVICE)) {
		int rc = pm_device_action_run(gnss_dev, PM_DEVICE_ACTION_RESUME);

		if (rc < 0 && rc != -EALREADY) {
			LOG_ERR("GNSS resume failed (%d) — no NMEA will be read", rc);
		}
	}
#else
	LOG_WRN("CONFIG_MESHTASTIC_GNSS enabled but no ready gnss alias exists");
#endif

	return 0;
}
