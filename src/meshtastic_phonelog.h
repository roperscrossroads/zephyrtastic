/* SPDX-License-Identifier: GPL-3.0
 *
 * Log forwarding to the phone over the PhoneAPI. See meshtastic_phonelog.c for
 * why this exists and why it is not a substitute for the RTC log ring.
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_PHONELOG_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_PHONELOG_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_phonelog_stats {
	uint32_t forwarded;
	uint32_t dropped_rate;  /* over the per-second cap, or dropped by the log core */
	uint32_t dropped_queue; /* a transport's FromRadio queue was full */
	uint32_t dropped_level; /* below the runtime severity ceiling */
	uint32_t dropped_core;  /* the log core dropped them before any backend ran */
	bool panicked;          /* LOG_PANIC() ran; forwarding has stood down */
};

#if defined(CONFIG_MESHTASTIC_PHONELOG)

/** Current runtime severity ceiling (0 = off, 1 = ERR ... 4 = DBG). */
uint8_t meshtastic_phonelog_get_level(void);

/**
 * @brief Set the runtime severity ceiling.
 *
 * Independent of what each module is compiled at and of what the console shows
 * -- Zephyr's filtering is per-(backend, module), so raising this cannot make
 * the console quieter or louder.
 *
 * @retval 0       Applied.
 * @retval -EINVAL Level above LOG_LEVEL_DBG.
 */
int meshtastic_phonelog_set_level(uint8_t level);

void meshtastic_phonelog_get_stats(struct meshtastic_phonelog_stats *out);
void meshtastic_phonelog_reset_stats(void);

#else

static inline uint8_t meshtastic_phonelog_get_level(void)
{
	return 0U;
}

static inline int meshtastic_phonelog_set_level(uint8_t level)
{
	ARG_UNUSED(level);
	return -ENOTSUP;
}

static inline void meshtastic_phonelog_get_stats(struct meshtastic_phonelog_stats *out)
{
	if (out != NULL) {
		*out = (struct meshtastic_phonelog_stats){0};
	}
}

static inline void meshtastic_phonelog_reset_stats(void)
{
}

#endif /* CONFIG_MESHTASTIC_PHONELOG */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_PHONELOG_H_ */
