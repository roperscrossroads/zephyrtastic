/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * On-device screen UI on a small monochrome panel using Zephyr's Character
 * Framebuffer (CFB). This is a first prototype: it renders a few status pages
 * from the existing Meshtastic C getters and auto-cycles between them (there is
 * no button input yet). No phone app or shell is required.
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

LOG_MODULE_REGISTER(meshtastic_display, CONFIG_MESHTASTIC_LOG_LEVEL);

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

	draw_row(0, "%s", (sn && sn[0]) ? sn : "Meshtastic");
	draw_row(1, "ID %08X", meshtastic_get_node_id());
	draw_row(2, "F %u.%u MHz", freq / 1000000U, (freq % 1000000U) / 100000U);
	draw_row(3, "CH %s H%u", (ch && ch[0]) ? ch : "-", meshtastic_runtime_hop_limit());
}

static void page_nodes(void)
{
	size_t n = meshtastic_nodedb_count();

	draw_row(0, "Nodes: %u", (unsigned int)n);

	if (n == 0) {
		draw_row(1, "(none yet)");
		return;
	}

	for (uint8_t r = 1; r < rows && (size_t)(r - 1) < n; r++) {
		struct meshtastic_nodedb_node node;

		if (meshtastic_nodedb_get_by_index((size_t)(r - 1), &node) != 0) {
			break;
		}

		/* SNR is a float; format as whole dB to avoid pulling in FP printf. */
		if (node.has_hops_away) {
			draw_row(r, "%-4s %+ddB h%u", node.short_name, (int)node.snr,
				 node.hops_away);
		} else {
			draw_row(r, "%-4s %+ddB", node.short_name, (int)node.snr);
		}
	}
}

static void page_status(void)
{
	struct meshtastic_status st;

	if (meshtastic_get_status(&st) != 0) {
		draw_row(0, "status err");
		return;
	}

	draw_row(0, "TX %u", st.tx_packets);
	draw_row(1, "RX %u", st.rx_packets);
	draw_row(2, "RSSI %d", st.last_rssi);
	draw_row(3, "Up %us %s", (unsigned int)(k_uptime_get() / 1000),
		 st.ble_connected ? "BLE" : "");
}

typedef void (*page_fn)(void);

static const page_fn pages[] = {
	page_device,
	page_nodes,
	page_status,
};

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
			page = (uint8_t)((page + 1U) % ARRAY_SIZE(pages));
			page_since = k_uptime_get();
		}
	}
}

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

	cfb_framebuffer_clear(disp, true);
	(void)display_blanking_off(disp);

	LOG_INF("display UI up: %s %ux%u, font %ux%u -> %ux%u chars", disp->name, disp_w,
		disp_h, font_w, font_h, cols, rows);

	k_thread_create(&disp_thread, disp_stack, K_THREAD_STACK_SIZEOF(disp_stack),
			display_loop, NULL, NULL, NULL, CONFIG_MESHTASTIC_DISPLAY_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&disp_thread, "mt_display");

	return 0;
}

#endif /* DT_HAS_CHOSEN(zephyr_display) */
