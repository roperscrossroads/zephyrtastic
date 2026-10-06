/*
 * The display shim (src/meshtastic_display_shim.c): a frame reaches the panel only when it
 * differs from the one on the glass, and never sooner than the floor after the last one.
 *
 * The panel is Zephyr's dummy display: it accepts every write and shows nothing. The shim's
 * counters say what it forwarded, what it found unchanged and what it held back; the test
 * writes frames the way CFB does (the whole buffer, at 0,0) and reads the counters.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "meshtastic_display_shim.h"

static const struct device *shim;

#define W 128
#define H 64
static uint8_t frame[W * H / 8];

static const struct display_buffer_descriptor desc = {
	.buf_size = sizeof(frame), .width = W, .height = H, .pitch = W,
};

static void *suite_setup(void)
{
	shim = meshtastic_display_shim_device();
	return NULL;
}

/* Each test starts with the glass forgotten: a blanking toggle is how the shim is told the
 * glass may show anything (and what the previous test left there is exactly "anything"). */
static void forget_glass(void *fixture)
{
	ARG_UNUSED(fixture);
	(void)display_blanking_on(shim);
	(void)display_blanking_off(shim);
}

static struct meshtastic_display_shim_stats st(void)
{
	struct meshtastic_display_shim_stats s;

	meshtastic_display_shim_stats(&s);
	return s;
}

static void write_frame(uint8_t fill)
{
	memset(frame, fill, sizeof(frame));
	zassert_equal(display_write(shim, 0, 0, &desc, frame), 0, "the panel refused a write");
}

ZTEST(display_shim, test_the_shim_is_a_display_over_the_panel)
{
	struct display_capabilities caps, panel_caps;

	zassert_true(device_is_ready(shim), "shim not ready");
	zassert_true(device_is_ready(meshtastic_display_shim_panel()), "panel not ready");
	display_get_capabilities(shim, &caps);
	display_get_capabilities(meshtastic_display_shim_panel(), &panel_caps);
	zassert_equal(caps.x_resolution, panel_caps.x_resolution, "");
	zassert_equal(caps.y_resolution, panel_caps.y_resolution, "");
	zassert_equal(caps.x_resolution, W, "the overlay's panel");
}

#if CONFIG_MESHTASTIC_DISPLAY_MIN_REFRESH_MS == 0

ZTEST(display_shim, test_an_unchanged_frame_never_reaches_the_panel)
{
	struct meshtastic_display_shim_stats a = st();

	write_frame(0x00);
	write_frame(0x00);
	write_frame(0x00);
	struct meshtastic_display_shim_stats b = st();

	zassert_equal(b.written - a.written, 1U, "the first frame is written");
	zassert_equal(b.unchanged - a.unchanged, 2U, "the two repeats are not");
	zassert_equal(b.held - a.held, 0U, "no floor: nothing held");
}

ZTEST(display_shim, test_a_changed_frame_is_written_and_a_repeat_of_it_is_not)
{
	struct meshtastic_display_shim_stats a = st();

	write_frame(0x00);
	write_frame(0xff);
	write_frame(0xff);
	write_frame(0x00);
	struct meshtastic_display_shim_stats b = st();

	zassert_equal(b.written - a.written, 3U, "three distinct frames in a row");
	zassert_equal(b.unchanged - a.unchanged, 1U, "one repeat");
}

ZTEST(display_shim, test_blanking_forgets_what_the_glass_held)
{
	struct meshtastic_display_shim_stats a = st();

	write_frame(0x3c);
	/* The driver may redraw or clear on blanking; the shim cannot know what the
	 * glass shows afterwards, so the next frame goes through even if identical. */
	zassert_equal(display_blanking_on(shim), 0, "");
	zassert_equal(display_blanking_off(shim), 0, "");
	write_frame(0x3c);
	struct meshtastic_display_shim_stats b = st();

	zassert_equal(b.written - a.written, 2U, "written again after blanking");
	zassert_equal(b.unchanged - a.unchanged, 0U, "");
}

ZTEST(display_shim, test_the_position_and_shape_are_part_of_the_frame)
{
	struct meshtastic_display_shim_stats a = st();
	struct display_buffer_descriptor half = {
		.buf_size = sizeof(frame) / 2, .width = W, .height = H / 2, .pitch = W,
	};

	memset(frame, 0x55, sizeof(frame));
	zassert_equal(display_write(shim, 0, 0, &half, frame), 0, "");
	zassert_equal(display_write(shim, 0, H / 2, &half, frame), 0, "");
	zassert_equal(display_write(shim, 0, H / 2, &half, frame), 0, "");
	struct meshtastic_display_shim_stats b = st();

	zassert_equal(b.written - a.written, 2U, "same bytes at another place is another frame");
	zassert_equal(b.unchanged - a.unchanged, 1U, "");
}

#else /* a floor between refreshes */

ZTEST(display_shim, test_a_change_inside_the_floor_is_held_and_carried_later)
{
	struct meshtastic_display_shim_stats a = st();

	write_frame(0x00);			/* on the glass */
	write_frame(0xff);			/* changed, but too soon: held */
	struct meshtastic_display_shim_stats b = st();

	zassert_equal(b.written - a.written, 1U, "");
	zassert_equal(b.held - a.held, 1U, "held inside the floor");

	write_frame(0xff);			/* the UI renders it again, still too soon */
	zassert_equal(st().held - a.held, 2U, "");

	k_sleep(K_MSEC(CONFIG_MESHTASTIC_DISPLAY_MIN_REFRESH_MS + 100));
	write_frame(0xff);			/* the floor has passed: carried now */
	struct meshtastic_display_shim_stats c = st();

	zassert_equal(c.written - a.written, 2U, "written once the floor passed");
	write_frame(0xff);
	zassert_equal(st().unchanged - c.unchanged, 1U, "and a repeat is still unchanged");
}

#endif

ZTEST_SUITE(display_shim, NULL, suite_setup, forget_glass, NULL, NULL);
