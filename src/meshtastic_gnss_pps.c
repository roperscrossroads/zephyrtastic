/* SPDX-License-Identifier: GPL-3.0
 *
 * GNSS 1PPS capture — see meshtastic_gnss_pps.h.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <zephyr/meshtastic/gnss_pps.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

#define PPS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(meshtastic_gnss_pps)

static const struct gpio_dt_spec pps_pin = GPIO_DT_SPEC_GET(PPS_NODE, pps_gpios);
static struct gpio_callback pps_cb;
static bool pps_armed;
static bool pps_log;

/*
 * Written from the edge ISR, read under irq_lock() by threads.
 *
 * Stamped with BOTH clocks on purpose. k_cycle_get_32() is the fine ruler and
 * is what the interval statistics are built from — measuring a microsecond-grade
 * pulse with a millisecond tick would discard the property being measured.
 * k_uptime_get() is what the wall clock's anchor is expressed in, so it is what
 * the discipline path actually consumes. Taking both in the same ISR is the only
 * way they can be known to refer to the same edge.
 */
static struct {
	uint32_t count;
	uint32_t intervals;
	uint32_t min_cyc;
	uint32_t max_cyc;
	uint64_t sum_cyc;
	uint32_t last_cyc;
	int64_t last_uptime_ms;
	bool locked;
} pps;

/*
 * A pulse train is "locked" once an interval has been measured near one second.
 *
 * Five percent, and the looseness is the point: this is not a precision test. It
 * separates a 1 Hz signal from a floating pin picking up noise, and the actual
 * precision question is answered by the jitter statistics, which on hardware
 * report tens of microseconds — four orders inside this window.
 *
 * It started at one percent and that was wrong in a way worth recording. The
 * measured interval is not the pulse, it is the pulse plus however long the ISR
 * took to run, so anything that delays the handler widens it: the sim test's
 * k_msleep(1000) lands at 1.010 s and was rejected by a hair. On hardware the
 * same thing happens under load, and the failure mode is bad — a good receiver
 * silently declared untrustworthy, the clock quietly falling back to an 853 ms
 * late anchor, and nothing saying why. A window this side of the noise floor
 * costs nothing and removes that whole class.
 */
static bool interval_is_plausible(uint32_t d_cyc)
{
	uint32_t hz = sys_clock_hw_cycles_per_sec();
	uint32_t tol = hz / 20U; /* 5% */

	return d_cyc > (hz - tol) && d_cyc < (hz + tol);
}

static void pps_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	uint32_t cyc = k_cycle_get_32();
	int64_t up = k_uptime_get();
	uint32_t d = 0U;
	bool have_d;

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	have_d = (pps.count > 0U);
	if (have_d) {
		d = cyc - pps.last_cyc; /* unsigned: correct across one 32-bit wrap */
		if (pps.intervals == 0U || d < pps.min_cyc) {
			pps.min_cyc = d;
		}
		if (pps.intervals == 0U || d > pps.max_cyc) {
			pps.max_cyc = d;
		}
		pps.sum_cyc += d;
		pps.intervals++;
		/* Lock on a plausible interval; drop the lock on an implausible one,
		 * so a receiver that loses its fix and stops pulsing stops being
		 * believed rather than going stale at the last good value. */
		pps.locked = interval_is_plausible(d);
	}
	pps.last_cyc = cyc;
	pps.last_uptime_ms = up;
	pps.count++;

	if (pps_log) {
		if (have_d) {
			LOG_INF("PPS #%u cyc=%u d=%u cyc (%u us)", pps.count, cyc, d,
				(unsigned int)k_cyc_to_us_near32(d));
		} else {
			LOG_INF("PPS #%u cyc=%u (first edge)", pps.count, cyc);
		}
	}
}

int meshtastic_gnss_pps_start(bool log_edges)
{
	int ret;

	if (!gpio_is_ready_dt(&pps_pin)) {
		return -ENODEV;
	}

	pps_log = log_edges;

	ret = gpio_pin_configure_dt(&pps_pin, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	if (!pps_armed) {
		gpio_init_callback(&pps_cb, pps_isr, BIT(pps_pin.pin));
		ret = gpio_add_callback(pps_pin.port, &pps_cb);
		if (ret < 0) {
			return ret;
		}
	}

	memset(&pps, 0, sizeof(pps));

	ret = gpio_pin_interrupt_configure_dt(&pps_pin, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		gpio_remove_callback(pps_pin.port, &pps_cb);
		return ret;
	}

	pps_armed = true;
	return 0;
}

int meshtastic_gnss_pps_stop(void)
{
	if (!pps_armed) {
		return -EALREADY;
	}
	gpio_pin_interrupt_configure_dt(&pps_pin, GPIO_INT_DISABLE);
	gpio_remove_callback(pps_pin.port, &pps_cb);
	pps_armed = false;
	return 0;
}

void meshtastic_gnss_pps_get_stats(struct meshtastic_gnss_pps_stats *out)
{
	unsigned int key;

	if (out == NULL) {
		return;
	}

	key = irq_lock();
	out->count = pps.count;
	out->intervals = pps.intervals;
	out->min_cyc = pps.min_cyc;
	out->max_cyc = pps.max_cyc;
	out->sum_cyc = pps.sum_cyc;
	out->last_cyc = pps.last_cyc;
	out->last_uptime_ms = pps.last_uptime_ms;
	out->locked = pps.locked;
	irq_unlock(key);
	out->armed = pps_armed;
}

bool meshtastic_gnss_pps_last_edge(int64_t *uptime_ms_out, int64_t *age_ms_out)
{
	unsigned int key;
	int64_t edge;
	int64_t age;
	bool locked;

	if (!pps_armed) {
		return false;
	}

	key = irq_lock();
	edge = pps.last_uptime_ms;
	locked = pps.locked;
	irq_unlock(key);

	if (!locked) {
		return false;
	}

	age = k_uptime_get() - edge;
	/* Strictly inside one second. A negative age is impossible unless the edge
	 * stamp is garbage; an age of a second or more means an edge was missed, and
	 * pairing with the wrong edge is a whole second of silent error. */
	if (age < 0 || age >= MSEC_PER_SEC) {
		return false;
	}

	if (uptime_ms_out != NULL) {
		*uptime_ms_out = edge;
	}
	if (age_ms_out != NULL) {
		*age_ms_out = age;
	}
	return true;
}

/*
 * Armed at init, not on demand. This is a time source, not a diagnostic: a
 * counter nobody armed disciplines nothing, and the first window trip collected
 * no PPS data for exactly that reason — a node at a window is on battery with
 * nobody typing at it.
 */
static int pps_init(void)
{
	int ret = meshtastic_gnss_pps_start(IS_ENABLED(CONFIG_MESHTASTIC_GNSS_PPS_LOG_EDGES));

	if (ret < 0) {
		LOG_WRN("gnss pps: capture unavailable (%d) — the clock will fall back to "
			"NMEA arrival", ret);
	}
	return 0;
}

SYS_INIT(pps_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
