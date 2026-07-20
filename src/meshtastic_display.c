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

	draw_row(0, "Nodes: %u", (unsigned int)n);

	if (n == 0) {
		draw_row(1, "(none yet)");
		return;
	}

	for (uint8_t r = 1; r < content_rows && (size_t)(r - 1) < n; r++) {
		struct meshtastic_nodedb_node node;
		char fav;

		if (meshtastic_nodedb_get_by_index((size_t)(r - 1), &node) != 0) {
			break;
		}

		/* '*' marks a favourite node (upstream device-ui sorts these first). */
		fav = node.is_favorite ? '*' : ' ';

		/* SNR is a float; format as whole dB to avoid pulling in FP printf. */
		if (node.has_hops_away) {
			draw_row(r, "%c%-4s%+3d h%u", fav, node.short_name, (int)node.snr,
				 node.hops_away);
		} else {
			draw_row(r, "%c%-4s%+3d", fav, node.short_name, (int)node.snr);
		}
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

	draw_row(0, "TX%u RX%u", st.tx_packets, st.rx_packets);
	draw_row(1, "RSSI %d", st.last_rssi);
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

/* Footer choices on a page, cycled by short press. */
enum page_focus { FOCUS_BACK = 0, FOCUS_NEXT, FOCUS_COUNT };

/* Top-level UI state: the launcher menu, or an open page. */
static enum { UI_MENU, UI_PAGE } ui_state = UI_MENU;
static uint8_t menu_cursor;              /* highlighted launcher entry */
static uint8_t cur_page;                 /* page shown in UI_PAGE */
static uint8_t page_focus = FOCUS_BACK;  /* highlighted footer choice */

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

/* Draw the Back/Next footer, highlighting the focused choice. */
static void render_footer(void)
{
	draw_row((uint8_t)(rows - 1), "%cBack  %cNext",
		 page_focus == FOCUS_BACK ? '>' : ' ',
		 page_focus == FOCUS_NEXT ? '>' : ' ');
}

static void nav_short(void)
{
	if (ui_state == UI_MENU) {
		menu_cursor = (uint8_t)((menu_cursor + 1U) % NUM_PAGES);
	} else {
		page_focus = (uint8_t)((page_focus + 1U) % FOCUS_COUNT);
	}
}

static void nav_long(void)
{
	if (ui_state == UI_MENU) {
		cur_page = menu_cursor;
		page_focus = FOCUS_BACK;
		ui_state = UI_PAGE;
	} else if (page_focus == FOCUS_BACK) {
		ui_state = UI_MENU;
	} else { /* FOCUS_NEXT */
		cur_page = (uint8_t)((cur_page + 1U) % NUM_PAGES);
	}
}

static void render_current(void)
{
	cfb_framebuffer_clear(disp, false);

	if (ui_state == UI_MENU) {
		render_menu();
	} else {
		pages[cur_page]();
		render_footer();
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
			} else if (ev == UI_EV_SHORT) {
				nav_short();
			} else if (ev == UI_EV_LONG) {
				nav_long();
			}
			last_activity = k_uptime_get();
		}

		if (!blanked && timeout_ms > 0 &&
		    k_uptime_get() - last_activity >= timeout_ms) {
			(void)display_blanking_on(disp);
			blanked = true;
		}

		if (!blanked) {
			render_current();
		}
	}
}

#else /* !HAS_BUTTON: auto-cycle the pages on a timer */

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
