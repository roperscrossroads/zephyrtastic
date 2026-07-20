/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * On-device screen UI on a small monochrome panel using Zephyr's Character
 * Framebuffer (CFB). It renders a few read-only status pages from the existing
 * Meshtastic C getters. No phone app or shell is required.
 *
 * Navigation (when the board exposes a button via the sw0 alias):
 *   - short press  -> next: the highlight moves to the next item, cycling in a
 *                     loop (menu entry, or the Back/Next footer of a page).
 *   - long  press  -> confirm the highlighted choice (open a page; Back to the
 *                     launcher; Next to the following page).
 *   - idle         -> the panel blanks after a timeout; the next press wakes it
 *                     without also navigating.
 * With no button (no sw0 alias, or CONFIG_MESHTASTIC_DISPLAY_BUTTON=n) the UI
 * falls back to auto-cycling the pages on a timer.
 *
 * The panel is whatever the board selects as `chosen { zephyr,display }` — the
 * 128x64 SSD1306 OLED on the Heltec WiFi LoRa 32 V4. The renderer reads the
 * display geometry at runtime and lays pages out in character rows, so the same
 * code adapts to other small mono panels (e.g. the 160x80 Heltec Tracker V2).
 */

#include <stdarg.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

#include <zephyr/meshtastic/nodedb.h>
#include "meshtastic_core.h" /* public getters + internal name/freq/chan/hop getters */
#include "meshtastic_airtime.h"  /* channel/tx utilisation (CONFIG_MESHTASTIC_AIRTIME) */
#include "meshtastic_clock.h"    /* epoch time (compiled unconditionally) */
#include "meshtastic_position.h" /* fix lat/lon/alt (CONFIG_MESHTASTIC_POSITION) */
#if defined(CONFIG_MESHTASTIC_MQTT)
#include "meshtastic_mqtt.h"
#endif

LOG_MODULE_REGISTER(meshtastic_display, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Button navigation is compiled in only when it is both enabled and the board
 * actually wires a user button as the sw0 alias. */
#define HAS_BUTTON \
	(IS_ENABLED(CONFIG_MESHTASTIC_DISPLAY_BUTTON) && DT_NODE_EXISTS(DT_ALIAS(sw0)))

#if HAS_BUTTON
#include <zephyr/input/input.h>
#include <zephyr/sys/atomic.h>
#endif

/* Battery UI is compiled in only when enabled and the board wires a vbatt
 * voltage-divider node. */
#define HAS_BATTERY \
	(IS_ENABLED(CONFIG_MESHTASTIC_DISPLAY_BATTERY) && DT_NODE_EXISTS(DT_NODELABEL(vbatt)))

#if HAS_BATTERY
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/voltage_divider.h>
#include <zephyr/drivers/gpio.h>
#endif

int meshtastic_display_init(void);

#if !DT_HAS_CHOSEN(zephyr_display)

/* No panel wired on this board: compile a stub so the init hook still links. */
int meshtastic_display_init(void)
{
	LOG_WRN("MESHTASTIC_DISPLAY enabled but no chosen zephyr,display; UI off");
	return 0;
}

#else

static const struct device *const disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static uint8_t font_idx;
static uint8_t font_w, font_h;
static uint16_t disp_w, disp_h;
static uint8_t rows, cols;
static uint8_t content_rows; /* rows available to page content (footer excluded) */

static K_THREAD_STACK_DEFINE(disp_stack, CONFIG_MESHTASTIC_DISPLAY_STACK_SIZE);
static struct k_thread disp_thread;

/* --- rendering helpers --------------------------------------------------- */

/* Draw one line of text at character row @p row (0-based), truncated to fit. */
static void draw_row(uint8_t row, const char *fmt, ...)
{
	char buf[32];
	va_list ap;

	if (row >= rows) {
		return;
	}

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (cols > 0 && cols < sizeof(buf) && buf[cols] != '\0') {
		buf[cols] = '\0';
	}

	cfb_print(disp, buf, 0, (int16_t)(row * font_h));
}

#if HAS_BATTERY

/* VBAT voltage-divider (ADC1 ch0). The esp32 ADC driver returns a calibrated
 * pseudo-raw, so adc_raw_to_millivolts_dt yields true pin millivolts. */
static const struct voltage_divider_dt_spec vbatt =
	VOLTAGE_DIVIDER_DT_SPEC_GET(DT_NODELABEL(vbatt));

#if DT_NODE_EXISTS(DT_NODELABEL(adc_ctrl))
/* The V4 gates the divider behind ADC_CTRL (GPIO37); drive it high only while
 * sampling. Absent on the V4-R8, where the divider is always connected. The
 * adc_ctrl power-domain is inert without PM_DEVICE, so the UI owns this line. */
static const struct gpio_dt_spec adc_ctrl_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(adc_ctrl), enable_gpios);
#define HAS_ADC_CTRL 1
#else
#define HAS_ADC_CTRL 0
#endif

static bool batt_ready;
static int batt_mv_cached = -1;
static int64_t batt_last_ms;

static void battery_setup(void)
{
	if (!adc_is_ready_dt(&vbatt.port) || adc_channel_setup_dt(&vbatt.port) != 0) {
		LOG_WRN("vbatt ADC not ready; battery UI off");
		return;
	}
#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_configure_dt(&adc_ctrl_en, GPIO_OUTPUT_INACTIVE);
	}
#endif
	batt_ready = true;
}

/* Battery millivolts (single cell), or -1 on error. Cached for 5 s so button
 * traffic does not re-sample (and re-toggle ADC_CTRL) on every redraw. */
static int battery_millivolts(void)
{
	int64_t now = k_uptime_get();
	uint16_t raw = 0;
	int32_t mv;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	if (!batt_ready) {
		return -1;
	}
	if (batt_mv_cached >= 0 && (now - batt_last_ms) < 5000) {
		return batt_mv_cached;
	}
	batt_last_ms = now;

#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_set_dt(&adc_ctrl_en, 1);
		k_sleep(K_MSEC(2)); /* let the divider settle */
	}
#endif

	batt_mv_cached = -1;
	if (adc_sequence_init_dt(&vbatt.port, &seq) == 0 &&
	    adc_read_dt(&vbatt.port, &seq) == 0) {
		mv = raw;
		if (adc_raw_to_millivolts_dt(&vbatt.port, &mv) == 0) {
			/* x full/output (4.9) then the empirical factor (x1.045 on V4). */
			(void)voltage_divider_scale_dt(&vbatt, &mv);
			mv = (int32_t)((int64_t)mv *
				       CONFIG_MESHTASTIC_DISPLAY_BATTERY_CAL_PERMILLE / 1000);
			batt_mv_cached = mv;
		}
	}

#if HAS_ADC_CTRL
	if (gpio_is_ready_dt(&adc_ctrl_en)) {
		(void)gpio_pin_set_dt(&adc_ctrl_en, 0);
	}
#endif

	return batt_mv_cached;
}

/* Open-circuit-voltage lookup -> state of charge (0-100), or -1 for "no
 * battery". Matches the upstream firmware curve (firmware/src/Power.{h,cpp}):
 * single-cell LiPo, integer math. */
static int battery_percent(int mv)
{
	static const uint16_t ocv[] = {
		4190, 4050, 3990, 3890, 3800, 3720, 3630, 3530, 3420, 3300, 3100,
	};
	const int n = (int)ARRAY_SIZE(ocv);

	if (mv < (int)ocv[n - 1] - 500) {
		return -1; /* below the "no battery installed" threshold (2600 mV) */
	}
	for (int i = 0; i < n; i++) {
		if (mv >= (int)ocv[i]) {
			if (i == 0) {
				return 100;
			}
			int seg = (int)ocv[i - 1] - (int)ocv[i];
			int soc = 10 * (n - 1 - i) + (10 * (mv - (int)ocv[i])) / seg;

			return CLAMP(soc, 0, 100);
		}
	}
	return 0;
}

#endif /* HAS_BATTERY */

/* Index of each page within pages[]/page_names[] (keep in sync with them). The
 * Nodes page is navigated specially, so its index is referenced by name. */
enum {
	PAGE_DEVICE,
	PAGE_NODES,
	PAGE_STATUS,
	PAGE_RADIO,
	PAGE_GPS,
	PAGE_TIME,
};

/* When the Nodes page is being browsed, report which node index is highlighted
 * (or -1) and window the visible list via @p first. Defined per build variant:
 * the button build reads the nav cursor, the auto-cycle build has none. */
static int nodes_cursor(size_t n, uint8_t list_rows, uint8_t *first);

static void page_device(void)
{
	const char *sn = meshtastic_short_name();
	const char *ch = meshtastic_runtime_channel_name();
	uint32_t freq = meshtastic_runtime_frequency();

	draw_row(0, "%s H%u", (sn && sn[0]) ? sn : "Meshtastic",
		 meshtastic_runtime_hop_limit());
	draw_row(1, "ID %08X", meshtastic_get_node_id());
	draw_row(2, "F%u.%u %s", freq / 1000000U, (freq % 1000000U) / 100000U,
		 (ch && ch[0]) ? ch : "-");
}

static void page_nodes(void)
{
	size_t n = meshtastic_nodedb_count();
	uint8_t list_rows = (content_rows > 1U) ? (uint8_t)(content_rows - 1U) : 0U;
	uint8_t first = 0;
	int sel = -1; /* index of the highlighted node row, or -1 for none */

	draw_row(0, "Nodes: %u", (unsigned int)n);

	if (n == 0) {
		draw_row(1, "(none yet)");
		return;
	}

	sel = nodes_cursor(n, list_rows, &first);

	for (uint8_t i = 0; i < list_rows && (size_t)(first + i) < n; i++) {
		struct meshtastic_nodedb_node node;
		char fav, cur;

		if (meshtastic_nodedb_get_by_index((size_t)(first + i), &node) != 0) {
			break;
		}

		cur = ((int)(first + i) == sel) ? '>' : ' ';
		/* '*' marks a favourite node (upstream device-ui sorts these first). */
		fav = node.is_favorite ? '*' : ' ';

		/* cursor + favourite + name + SNR (whole dB — hops move to the detail
		 * view). SNR is a float, cast to int to avoid pulling in FP printf. */
		draw_row((uint8_t)(1 + i), "%c%c%-4s%+4d", cur, fav, node.short_name,
			 (int)node.snr);
	}
}

static void page_radio(void)
{
#if defined(CONFIG_MESHTASTIC_AIRTIME)
	/* Utilisation getters return float percentages; show whole percent to
	 * avoid pulling in floating-point printf. */
	draw_row(0, "ChUtil %d%%", (int)meshtastic_airtime_channel_util_percent());
	draw_row(1, "TxUtil %d%%", (int)meshtastic_airtime_tx_util_percent());
#else
	draw_row(0, "airtime off");
#endif

#if defined(CONFIG_MESHTASTIC_MQTT)
	draw_row(2, "MQTT %s", meshtastic_mqtt_is_connected() ? "up" : "--");
#else
	{
		uint32_t freq = meshtastic_runtime_frequency();

		draw_row(2, "F%u.%u MHz", freq / 1000000U, (freq % 1000000U) / 100000U);
	}
#endif
}

static void page_gps(void)
{
#if defined(CONFIG_MESHTASTIC_POSITION)
	meshtastic_Position pos;

	if (meshtastic_position_get_current(&pos) != 0 ||
	    !pos.has_latitude_i || !pos.has_longitude_i) {
		draw_row(0, "No GPS fix");
		draw_row(1, "Sats %u", pos.sats_in_view);
		return;
	}

	/* latitude_i/longitude_i are degrees * 1e7. Print sign, whole degrees and
	 * four decimals (~11 m) as integers — no FP. Sign is tracked separately so
	 * coordinates in (-1, 0) degrees keep their minus. */
	{
		int32_t la = pos.latitude_i, lo = pos.longitude_i;
		uint32_t lam = (uint32_t)(la < 0 ? -(int64_t)la : la);
		uint32_t lom = (uint32_t)(lo < 0 ? -(int64_t)lo : lo);

		draw_row(0, "Lat %s%u.%04u", la < 0 ? "-" : "", lam / 10000000U,
			 (lam % 10000000U) / 1000U);
		draw_row(1, "Lon %s%u.%04u", lo < 0 ? "-" : "", lom / 10000000U,
			 (lom % 10000000U) / 1000U);
		draw_row(2, "Alt %dm S%u", pos.has_altitude ? pos.altitude : 0,
			 pos.sats_in_view);
	}
#else
	draw_row(0, "GPS off");
#endif
}

/* Days since 1970-01-01 -> civil Y/M/D (proleptic Gregorian), no libc/time.h.
 * Howard Hinnant's algorithm. Valid for the full uint32 epoch range. */
static void civil_from_days(uint32_t days, uint32_t *y, uint32_t *m, uint32_t *d)
{
	uint32_t z = days + 719468U;
	uint32_t era = z / 146097U;
	uint32_t doe = z - era * 146097U;                              /* [0, 146096] */
	uint32_t yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
	uint32_t doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);     /* [0, 365] */
	uint32_t mp = (5U * doy + 2U) / 153U;                          /* [0, 11] */

	*d = doy - (153U * mp + 2U) / 5U + 1U;                         /* [1, 31] */
	*m = mp < 10U ? mp + 3U : mp - 9U;                             /* [1, 12] */
	*y = yoe + era * 400U + (*m <= 2U);
}

static void page_time(void)
{
	uint32_t epoch, days, secs, y, mo, d;

	if (!meshtastic_clock_valid()) {
		draw_row(0, "Clock unset");
		draw_row(1, "Up %us", (unsigned int)(k_uptime_get() / 1000));
		return;
	}

	epoch = meshtastic_clock_now_epoch();
	days = epoch / 86400U;
	secs = epoch % 86400U;
	civil_from_days(days, &y, &mo, &d);

	draw_row(0, "%04u-%02u-%02u", y, mo, d);
	draw_row(1, "%02u:%02u:%02u UTC", secs / 3600U, (secs % 3600U) / 60U, secs % 60U);
	draw_row(2, "Up %us", (unsigned int)(k_uptime_get() / 1000));
}

static void page_status(void)
{
	struct meshtastic_status st;

	if (meshtastic_get_status(&st) != 0) {
		draw_row(0, "status err");
		return;
	}

#if HAS_BATTERY
	{
		int mv = battery_millivolts();
		int pct = (mv >= 0) ? battery_percent(mv) : -1;

		if (mv < 0) {
			draw_row(0, "Bat: n/a");
		} else if (pct < 0) {
			draw_row(0, "Bat: no batt");
		} else {
			draw_row(0, "Bat %u.%uV %d%%", (unsigned int)(mv / 1000),
				 (unsigned int)((mv % 1000) / 100), pct);
		}
	}
#else
	draw_row(0, "RSSI %d", st.last_rssi);
#endif
	draw_row(1, "TX%u RX%u", st.tx_packets, st.rx_packets);
	draw_row(2, "Up%us %s", (unsigned int)(k_uptime_get() / 1000),
		 st.ble_connected ? "BLE" : "");
}

typedef void (*page_fn)(void);

static const page_fn pages[] = {
	page_device,
	page_nodes,
	page_status,
	page_radio,
	page_gps,
	page_time,
};

static const char *const page_names[] = {
	"Device",
	"Nodes",
	"Status",
	"Radio",
	"GPS",
	"Time",
};

BUILD_ASSERT(ARRAY_SIZE(pages) == ARRAY_SIZE(page_names),
	     "pages[] and page_names[] must stay in step");

#define NUM_PAGES ARRAY_SIZE(pages)

/* --- navigation ---------------------------------------------------------- */

#if HAS_BUTTON

/* Footer choices on a page, cycled by short press. On most screens the focus
 * ring is just these two; the Nodes page prepends one entry per node. */
enum page_focus { FOCUS_BACK = 0, FOCUS_NEXT, FOCUS_COUNT };

/* Top-level UI state: launcher menu, an open page, or a node-detail view. */
static enum { UI_MENU, UI_PAGE, UI_NODE_DETAIL } ui_state = UI_MENU;
static uint8_t menu_cursor;              /* highlighted launcher entry */
static uint8_t cur_page;                 /* page shown in UI_PAGE */
static uint8_t page_focus = FOCUS_BACK;  /* highlighted choice on a page/detail */
static uint8_t detail_idx;               /* node index shown in UI_NODE_DETAIL */

/* Short-press focus positions on the current screen. The Nodes page ring is
 * every node (each openable) followed by the Back and Next footer choices;
 * every other page and the detail view has only the two footer choices. */
static uint8_t focus_count(void)
{
	if (ui_state == UI_PAGE && cur_page == PAGE_NODES) {
		return (uint8_t)(meshtastic_nodedb_count() + FOCUS_COUNT);
	}

	return FOCUS_COUNT;
}

static int nodes_cursor(size_t n, uint8_t list_rows, uint8_t *first)
{
	*first = 0;

	/* Only the Nodes page, and only when the focus is on a node (below the
	 * Back/Next footer choices), gets a cursor. */
	if (ui_state != UI_PAGE || cur_page != PAGE_NODES || page_focus >= n) {
		return -1;
	}

	if (list_rows > 0 && page_focus >= list_rows) {
		*first = (uint8_t)(page_focus - (list_rows - 1U));
	}

	return (int)page_focus;
}

/* Draw the launcher: a windowed list of pages with a '>' cursor. */
static void render_menu(void)
{
	uint8_t first = 0;

	if (rows > 0 && menu_cursor >= rows) {
		first = (uint8_t)(menu_cursor - (rows - 1));
	}

	for (uint8_t r = 0; r < rows && (size_t)(first + r) < NUM_PAGES; r++) {
		uint8_t item = (uint8_t)(first + r);

		draw_row(r, "%c%s", item == menu_cursor ? '>' : ' ', page_names[item]);
	}
}

/* Map a device role to a 3-char code for the node-detail view. */
static const char *role_abbrev(uint8_t role)
{
	switch (role) {
	case meshtastic_Config_DeviceConfig_Role_CLIENT:         return "CLI";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE:    return "MUT";
	case meshtastic_Config_DeviceConfig_Role_ROUTER:         return "RTR";
	case meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT:  return "RTC";
	case meshtastic_Config_DeviceConfig_Role_REPEATER:       return "REP";
	case meshtastic_Config_DeviceConfig_Role_TRACKER:        return "TRK";
	case meshtastic_Config_DeviceConfig_Role_SENSOR:         return "SEN";
	case meshtastic_Config_DeviceConfig_Role_TAK:            return "TAK";
	case meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN:  return "HID";
	case meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND: return "LAF";
	case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:    return "TKT";
	case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:    return "RTL";
	default:                                                 return "?";
	}
}

/* Full-screen details of the node at detail_idx (drawn under the footer). */
static void render_node_detail(void)
{
	struct meshtastic_nodedb_node node;
	const char *ln;

	if (meshtastic_nodedb_get_by_index((size_t)detail_idx, &node) != 0) {
		draw_row(0, "(node gone)");
		return;
	}

	ln = (node.long_name[0] != '\0') ? node.long_name : node.short_name;

	draw_row(0, "%s", ln);
	draw_row(1, "%08X %s", node.num, role_abbrev(node.role));

	/* SNR (whole dB, no FP), hops, and flags: M = heard via MQTT, K = public
	 * key known (PKC-capable). */
	if (node.has_hops_away) {
		draw_row(2, "%+ddB h%u %c%c", (int)node.snr, node.hops_away,
			 node.via_mqtt ? 'M' : ' ', node.public_key_len ? 'K' : ' ');
	} else {
		draw_row(2, "%+ddB %c%c", (int)node.snr, node.via_mqtt ? 'M' : ' ',
			 node.public_key_len ? 'K' : ' ');
	}
}

/* Draw the Back/Next footer, highlighting whichever choice has focus (on the
 * Nodes page the focus may instead be on a node, so neither is marked). */
static void render_footer(void)
{
	uint8_t back = (uint8_t)(focus_count() - FOCUS_COUNT + FOCUS_BACK);

	draw_row((uint8_t)(rows - 1), "%cBack  %cNext",
		 page_focus == back ? '>' : ' ',
		 page_focus == (uint8_t)(back + 1U) ? '>' : ' ');
}

/* Keep the focus/index valid against the live NodeDB, which the mesh threads
 * mutate underneath us. */
static void clamp_focus(void)
{
	if (ui_state == UI_NODE_DETAIL) {
		size_t n = meshtastic_nodedb_count();

		if (n == 0) {
			ui_state = UI_PAGE;
			cur_page = PAGE_NODES;
			page_focus = FOCUS_BACK;
		} else if (detail_idx >= n) {
			detail_idx = (uint8_t)(n - 1);
		}
	} else if (ui_state == UI_PAGE) {
		uint8_t fc = focus_count();

		if (page_focus >= fc) {
			page_focus = (uint8_t)(fc - 1); /* fc >= FOCUS_COUNT >= 2 */
		}
	}
}

static void nav_short(void)
{
	if (ui_state == UI_MENU) {
		menu_cursor = (uint8_t)((menu_cursor + 1U) % NUM_PAGES);
	} else {
		page_focus = (uint8_t)((page_focus + 1U) % focus_count());
	}
}

static void nav_long(void)
{
	uint8_t back;

	if (ui_state == UI_MENU) {
		cur_page = menu_cursor;
		page_focus = FOCUS_BACK;
		ui_state = UI_PAGE;
		return;
	}

	if (ui_state == UI_NODE_DETAIL) {
		size_t n = meshtastic_nodedb_count();

		if (page_focus == FOCUS_BACK || n == 0) {
			ui_state = UI_PAGE; /* return to the node list, cursor on this node */
			cur_page = PAGE_NODES;
			page_focus = detail_idx;
		} else { /* FOCUS_NEXT: flip to the next node */
			detail_idx = (uint8_t)((detail_idx + 1U) % n);
		}
		return;
	}

	/* ui_state == UI_PAGE */
	back = (uint8_t)(focus_count() - FOCUS_COUNT + FOCUS_BACK);

	if (cur_page == PAGE_NODES && page_focus < back) {
		/* Focus is on a node (below the footer): open its detail view. */
		detail_idx = page_focus;
		page_focus = FOCUS_NEXT; /* default focus = flip to the next node */
		ui_state = UI_NODE_DETAIL;
	} else if (page_focus == back) {
		ui_state = UI_MENU;
	} else { /* Next */
		cur_page = (uint8_t)((cur_page + 1U) % NUM_PAGES);
		page_focus = FOCUS_BACK;
	}
}

static void render_current(void)
{
	cfb_framebuffer_clear(disp, false);

	switch (ui_state) {
	case UI_MENU:
		render_menu();
		break;
	case UI_NODE_DETAIL:
		render_node_detail();
		render_footer();
		break;
	case UI_PAGE:
	default:
		pages[cur_page]();
		render_footer();
		break;
	}

	cfb_framebuffer_finalize(disp);
}

/* --- button input -------------------------------------------------------- */

enum ui_ev { UI_EV_SHORT = 1, UI_EV_LONG };

K_MSGQ_DEFINE(ui_evq, sizeof(uint8_t), 8, 1);

static atomic_t btn_held;  /* button currently pressed */
static atomic_t long_sent; /* long press already emitted for this hold */

static void longpress_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);

	if (atomic_get(&btn_held)) {
		uint8_t ev = UI_EV_LONG;

		atomic_set(&long_sent, 1);
		(void)k_msgq_put(&ui_evq, &ev, K_NO_WAIT);
	}
}

K_TIMER_DEFINE(longpress_timer, longpress_expiry, NULL);

static void ui_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_0) {
		return;
	}

	if (evt->value) { /* press */
		atomic_set(&btn_held, 1);
		atomic_set(&long_sent, 0);
		k_timer_start(&longpress_timer,
			      K_MSEC(CONFIG_MESHTASTIC_DISPLAY_LONGPRESS_MS), K_NO_WAIT);
	} else { /* release */
		atomic_set(&btn_held, 0);
		k_timer_stop(&longpress_timer);
		if (!atomic_get(&long_sent)) {
			uint8_t ev = UI_EV_SHORT;

			(void)k_msgq_put(&ui_evq, &ev, K_NO_WAIT);
		}
	}
}

INPUT_CALLBACK_DEFINE(NULL, ui_input_cb, NULL);

static void display_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	const int64_t timeout_ms =
		(int64_t)CONFIG_MESHTASTIC_DISPLAY_TIMEOUT_SECONDS * 1000;
	int64_t last_activity = k_uptime_get();
	bool blanked = false;

	render_current();

	while (1) {
		uint8_t ev;

		if (k_msgq_get(&ui_evq, &ev, K_MSEC(CONFIG_MESHTASTIC_DISPLAY_REFRESH_MS)) ==
		    0) {
			/* A button event: wake a blanked screen (swallowing the
			 * navigation), otherwise apply it. */
			if (blanked) {
				(void)display_blanking_off(disp);
				blanked = false;
			} else {
				/* Keep the cursor valid against the live NodeDB, then act. */
				clamp_focus();
				if (ev == UI_EV_SHORT) {
					nav_short();
				} else if (ev == UI_EV_LONG) {
					nav_long();
				}
			}
			last_activity = k_uptime_get();
		}

		if (!blanked && timeout_ms > 0 &&
		    k_uptime_get() - last_activity >= timeout_ms) {
			(void)display_blanking_on(disp);
			blanked = true;
		}

		if (!blanked) {
			clamp_focus();
			render_current();
		}
	}
}

#else /* !HAS_BUTTON: auto-cycle the pages on a timer */

/* No cursor without a button: the node list is purely informational. */
static int nodes_cursor(size_t n, uint8_t list_rows, uint8_t *first)
{
	ARG_UNUSED(n);
	ARG_UNUSED(list_rows);
	*first = 0;
	return -1;
}

static void render(uint8_t page)
{
	cfb_framebuffer_clear(disp, false);
	pages[page]();
	cfb_framebuffer_finalize(disp);
}

static void display_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	uint8_t page = 0;
	int64_t page_since = k_uptime_get();
	const int64_t page_ms = (int64_t)CONFIG_MESHTASTIC_DISPLAY_PAGE_SECONDS * 1000;

	while (1) {
		render(page);
		k_sleep(K_MSEC(CONFIG_MESHTASTIC_DISPLAY_REFRESH_MS));

		if (k_uptime_get() - page_since >= page_ms) {
			page = (uint8_t)((page + 1U) % NUM_PAGES);
			page_since = k_uptime_get();
		}
	}
}

#endif /* HAS_BUTTON */

/* --- setup --------------------------------------------------------------- */

/* Pick the smallest registered CFB font that fits the panel height. */
static int pick_smallest_font(void)
{
	int nfonts = cfb_get_numof_fonts(disp);
	int best = -1;
	uint8_t best_h = UINT8_MAX;

	for (int i = 0; i < nfonts; i++) {
		uint8_t w, h;

		if (cfb_get_font_size(disp, i, &w, &h) != 0) {
			continue;
		}
		if (h <= disp_h && h < best_h) {
			best = i;
			best_h = h;
			font_w = w;
			font_h = h;
		}
	}

	return best;
}

int meshtastic_display_init(void)
{
	int ret, fi;

	if (!device_is_ready(disp)) {
		LOG_ERR("display %s not ready; UI off", disp->name);
		return 0; /* non-fatal: the mesh stack runs headless */
	}

	/* SSD1306-class panels accept one of the two mono formats. */
	if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO10) != 0) {
		(void)display_set_pixel_format(disp, PIXEL_FORMAT_MONO01);
	}

	ret = cfb_framebuffer_init(disp);
	if (ret != 0) {
		LOG_ERR("cfb init failed (%d); UI off", ret);
		return 0;
	}

	disp_w = (uint16_t)cfb_get_display_parameter(disp, CFB_DISPLAY_WIDTH);
	disp_h = (uint16_t)cfb_get_display_parameter(disp, CFB_DISPLAY_HEIGHT);

	fi = pick_smallest_font();
	if (fi < 0) {
		LOG_ERR("no usable CFB font; UI off");
		return 0;
	}
	font_idx = (uint8_t)fi;
	cfb_framebuffer_set_font(disp, font_idx);

	cols = (font_w > 0) ? (uint8_t)(disp_w / font_w) : 0;
	rows = (font_h > 0) ? (uint8_t)(disp_h / font_h) : 0;

	/* A page reserves the bottom row for the Back/Next footer when the button
	 * navigation is compiled in; otherwise all rows carry content. */
	content_rows = HAS_BUTTON ? (uint8_t)((rows > 0) ? rows - 1 : 0) : rows;

	cfb_framebuffer_clear(disp, true);
	(void)display_blanking_off(disp);

#if HAS_BATTERY
	battery_setup();
#endif

	LOG_INF("display UI up: %s %ux%u, font %ux%u -> %ux%u chars, %s", disp->name,
		disp_w, disp_h, font_w, font_h, cols, rows,
		HAS_BUTTON ? "button nav" : "auto-cycle");

	k_thread_create(&disp_thread, disp_stack, K_THREAD_STACK_SIZEOF(disp_stack),
			display_loop, NULL, NULL, NULL, CONFIG_MESHTASTIC_DISPLAY_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&disp_thread, "mt_display");

	return 0;
}

#endif /* DT_HAS_CHOSEN(zephyr_display) */
