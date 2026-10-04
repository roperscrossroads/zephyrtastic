/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_LED_STATUS_H_
#define MESHTASTIC_LED_STATUS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Bench diagnostic: the board LED says whether this node is in use, so a hub
 * of identical boards can be read at a glance (Kconfig.led_status). The host
 * sets the mode -- from the bench's claims -- and the node keeps it across a
 * reboot; locate is a timed override that falls back to the saved mode.
 *
 *            XIAO (RGB)            one-LED board (Heltec white)
 *   idle     red, slow blink       slow blink
 *   in-use   green, solid          off
 *   locate   blue, fast blink      fast blink
 *   off      dark                  dark
 */
enum meshtastic_led_mode {
	MESHTASTIC_LED_IDLE = 0,
	MESHTASTIC_LED_IN_USE = 1,
	MESHTASTIC_LED_OFF = 2,
};

/* Starts the pattern in the saved mode (idle if none). Returns 0, or a
 * negative errno if led0 is declared but not ready. */
/* What a board is FOR, as a colour on an RGB LED (the XIAO's): set by the bench from the
 * board's place in its layout. DEFAULT keeps the original colours (idle red, in-use green,
 * locate blue). With a colour set, idle blinks it slowly, in-use shows it steady, and locate
 * blinks WHITE fast: white is no purpose's colour, so a located board cannot be mistaken
 * for one. A one-LED board ignores the colour; its patterns say the same. */
enum meshtastic_led_color {
	MESHTASTIC_LED_COLOR_DEFAULT = 0,
	MESHTASTIC_LED_COLOR_RED = 1,
	MESHTASTIC_LED_COLOR_GREEN = 2,
	MESHTASTIC_LED_COLOR_BLUE = 3,
	MESHTASTIC_LED_COLOR_YELLOW = 4,
	MESHTASTIC_LED_COLOR_CYAN = 5,
	MESHTASTIC_LED_COLOR_MAGENTA = 6,
};

int meshtastic_led_status_init(void);

/* Set and persist the mode (ends a locate). -EINVAL for an unknown mode. */
int meshtastic_led_status_set_mode(enum meshtastic_led_mode mode);
enum meshtastic_led_mode meshtastic_led_status_get_mode(void);

/* Blink "here I am" for @p seconds (0: stop), then return to the mode. */
void meshtastic_led_status_locate(uint32_t seconds);
/* Seconds of locate left, 0 when not locating. */
uint32_t meshtastic_led_status_locate_left(void);

/* True when the board has the green and blue LEDs the colours need. */
bool meshtastic_led_status_has_color(void);

const char *meshtastic_led_mode_name(enum meshtastic_led_mode mode);

/* The colour (saved as mtled/color); -EINVAL for one this enum does not have. */
int meshtastic_led_status_set_color(enum meshtastic_led_color color);
enum meshtastic_led_color meshtastic_led_status_get_color(void);
const char *meshtastic_led_color_name(enum meshtastic_led_color color);
/* A colour by its name ("default", "red", ... "magenta"); -EINVAL for any other. */
int meshtastic_led_color_parse(const char *name, enum meshtastic_led_color *out);

#endif /* MESHTASTIC_LED_STATUS_H_ */
