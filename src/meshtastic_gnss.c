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
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>

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

#if defined(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)
K_THREAD_STACK_DEFINE(gnss_send_wq_stack, CONFIG_MESHTASTIC_GNSS_SEND_WORK_STACK_SIZE);
static struct k_work_q gnss_send_wq;
#endif

static void position_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	ret = meshtastic_send_position(MESHTASTIC_NODE_BROADCAST);
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

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	meshtastic_Position position;
	int64_t now;
	int64_t send_interval_ms;
	int64_t retry_interval_ms;
	bool due;
	bool can_retry;

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

		if (epoch > 0) {
			meshtastic_clock_set_epoch((uint32_t)epoch);
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

	if (IS_ENABLED(CONFIG_MESHTASTIC_GNSS_AUTO_SEND) && due && can_retry &&
	    !k_work_busy_get(&position_send_work)) {
		gnss_state.last_attempt_ms = now;
		k_work_submit_to_queue(&gnss_send_wq, &position_send_work);
	}
	k_mutex_unlock(&gnss_state.lock);

	meshtastic_emit_event(MESHTASTIC_EVENT_GNSS_FIX, 0, NULL);
}

GNSS_DT_DATA_CALLBACK_DEFINE(MESHTASTIC_GNSS_NODE, gnss_data_cb);
#endif

int meshtastic_gnss_init(void)
{
	k_mutex_init(&gnss_state.lock);
	gnss_state.last_sent_ms =
		-((int64_t)CONFIG_MESHTASTIC_GNSS_SEND_INTERVAL_SEC * MSEC_PER_SEC);
	gnss_state.last_attempt_ms =
		-((int64_t)CONFIG_MESHTASTIC_GNSS_RETRY_INTERVAL_SEC * MSEC_PER_SEC);

#if MESHTASTIC_HAS_GNSS_ALIAS
	if (IS_ENABLED(CONFIG_MESHTASTIC_GNSS_AUTO_SEND)) {
		k_work_queue_start(&gnss_send_wq, gnss_send_wq_stack,
				   K_THREAD_STACK_SIZEOF(gnss_send_wq_stack),
				   CONFIG_MESHTASTIC_GNSS_SEND_WORK_PRIORITY, NULL);
	}

	if (!device_is_ready(gnss_dev)) {
		LOG_WRN("GNSS alias exists but device is not ready");
		return 0;
	}

	LOG_INF("Meshtastic position module using %s", gnss_dev->name);
#else
	LOG_WRN("CONFIG_MESHTASTIC_GNSS enabled but no ready gnss alias exists");
#endif

	return 0;
}
