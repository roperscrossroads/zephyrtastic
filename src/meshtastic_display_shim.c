/*
 * A frame reaches the panel only when it differs from the one on the glass.
 *
 * The UI renders its page into the Character Framebuffer every REFRESH_MS and CFB hands the
 * whole frame to the display driver each time, changed or not. On an OLED that is a few
 * hundred bytes over SPI. On a bistable e-ink panel it is a full refresh: a second of
 * flashing, visible wear, and one more of the roughly one million refreshes the controller
 * is rated for; at every 10 s that rating is spent in four months. The panel keeps its
 * image with no refresh at all, so an unchanged frame should never reach it.
 *
 * This is a display device that sits between CFB and the panel. It forwards everything
 * (capabilities, pixel format, orientation, blanking) and, on write, hashes the frame and
 * forwards it only when the hash differs from the last frame written, or when the glass
 * may no longer hold that frame (blanking was toggled). A minimum interval between
 * refreshes can be set as well (MIN_REFRESH_MS): a differing frame that arrives sooner is
 * held back, and the UI's next render carries it once the interval has passed.
 *
 * It is generic: nothing in it knows e-ink from OLED. The panel's own driver decides what
 * a write costs; this only makes sure it is not asked to spend it for nothing.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

#include "meshtastic_display_shim.h"

LOG_MODULE_REGISTER(mt_display_shim, CONFIG_MESHTASTIC_LOG_LEVEL);

#if DT_HAS_CHOSEN(zephyr_display)

struct shim_data {
	const struct device *panel;
	uint32_t last_hash;
	bool have_last;	 /* the glass holds the frame of last_hash */
	int64_t last_write_ms;
	struct meshtastic_display_shim_stats stats;
};

static struct shim_data shim = {
	.panel = DEVICE_DT_GET(DT_CHOSEN(zephyr_display)),
};

static uint32_t frame_hash(uint16_t x, uint16_t y, const struct display_buffer_descriptor *desc,
			   const void *buf)
{
	uint32_t h = crc32_ieee_update(0U, (const uint8_t *)&x, sizeof(x));

	h = crc32_ieee_update(h, (const uint8_t *)&y, sizeof(y));
	h = crc32_ieee_update(h, (const uint8_t *)&desc->width, sizeof(desc->width));
	h = crc32_ieee_update(h, (const uint8_t *)&desc->height, sizeof(desc->height));
	h = crc32_ieee_update(h, (const uint8_t *)&desc->pitch, sizeof(desc->pitch));
	return crc32_ieee_update(h, buf, desc->buf_size);
}

static int shim_write(const struct device *dev, const uint16_t x, const uint16_t y,
		      const struct display_buffer_descriptor *desc, const void *buf)
{
	struct shim_data *d = dev->data;
	uint32_t h = frame_hash(x, y, desc, buf);
	int64_t now = k_uptime_get();
	int err;

	if (d->have_last && h == d->last_hash) {
		d->stats.unchanged++;
		return 0;
	}
	if (CONFIG_MESHTASTIC_DISPLAY_MIN_REFRESH_MS > 0 && d->have_last &&
	    now - d->last_write_ms < CONFIG_MESHTASTIC_DISPLAY_MIN_REFRESH_MS) {
		/* Too soon after the last refresh: held back. The next render brings it. */
		d->stats.held++;
		return 0;
	}
	err = display_write(d->panel, x, y, desc, buf);
	if (err != 0) {
		d->stats.errors++;
		return err;
	}
	d->last_hash = h;
	d->have_last = true;
	d->last_write_ms = now;
	d->stats.written++;
	return 0;
}

static int shim_blanking_on(const struct device *dev)
{
	struct shim_data *d = dev->data;

	d->have_last = false; /* what the glass shows is the driver's business now */
	return display_blanking_on(d->panel);
}

static int shim_blanking_off(const struct device *dev)
{
	struct shim_data *d = dev->data;

	d->have_last = false; /* the driver may redraw on the next write: let it */
	return display_blanking_off(d->panel);
}

static int shim_read(const struct device *dev, const uint16_t x, const uint16_t y,
		     const struct display_buffer_descriptor *desc, void *buf)
{
	struct shim_data *d = dev->data;

	return display_read(d->panel, x, y, desc, buf);
}

static void *shim_get_framebuffer(const struct device *dev)
{
	struct shim_data *d = dev->data;

	return display_get_framebuffer(d->panel);
}

static int shim_set_brightness(const struct device *dev, const uint8_t brightness)
{
	struct shim_data *d = dev->data;

	return display_set_brightness(d->panel, brightness);
}

static int shim_set_contrast(const struct device *dev, const uint8_t contrast)
{
	struct shim_data *d = dev->data;

	return display_set_contrast(d->panel, contrast);
}

static void shim_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	struct shim_data *d = dev->data;

	display_get_capabilities(d->panel, caps);
}

static int shim_set_pixel_format(const struct device *dev, const enum display_pixel_format fmt)
{
	struct shim_data *d = dev->data;

	d->have_last = false;
	return display_set_pixel_format(d->panel, fmt);
}

static int shim_set_orientation(const struct device *dev,
				const enum display_orientation orientation)
{
	struct shim_data *d = dev->data;

	d->have_last = false;
	return display_set_orientation(d->panel, orientation);
}

static DEVICE_API(display, shim_api) = {
	.blanking_on = shim_blanking_on,
	.blanking_off = shim_blanking_off,
	.write = shim_write,
	.read = shim_read,
	.get_framebuffer = shim_get_framebuffer,
	.set_brightness = shim_set_brightness,
	.set_contrast = shim_set_contrast,
	.get_capabilities = shim_get_capabilities,
	.set_pixel_format = shim_set_pixel_format,
	.set_orientation = shim_set_orientation,
};

static int shim_init(const struct device *dev)
{
	struct shim_data *d = dev->data;

	if (!device_is_ready(d->panel)) {
		LOG_ERR("panel %s not ready", d->panel->name);
		return -ENODEV;
	}
	return 0;
}

/* After the panel (a display driver's own priority), before the application. */
DEVICE_DEFINE(mt_display_shim, MESHTASTIC_DISPLAY_SHIM_NAME, shim_init, NULL, &shim, NULL,
	      POST_KERNEL, CONFIG_MESHTASTIC_DISPLAY_SHIM_INIT_PRIORITY, &shim_api);

const struct device *meshtastic_display_shim_device(void)
{
	return DEVICE_GET(mt_display_shim);
}

const struct device *meshtastic_display_shim_panel(void)
{
	return shim.panel;
}

void meshtastic_display_shim_stats(struct meshtastic_display_shim_stats *out)
{
	*out = shim.stats;
}

#endif /* DT_HAS_CHOSEN(zephyr_display) */
