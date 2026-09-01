/* SPDX-License-Identifier: GPL-3.0
 *
 * Boot history in retained RAM — see meshtastic_bootlog.h.
 *
 * WHY THIS EXISTS
 * ---------------
 * On 2026-08-26 the two XIAO bench nodes had rebooted three times in nine hours
 * with no explanation available from either board, and the reason turned out to
 * be that nothing on an nRF target ever asked:
 *
 *   - The one `hwinfo_get_reset_cause()` call in the tree sits inside
 *     `#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)`, along with the RTC ring
 *     that stores it and the helper that decodes it. Every non-ESP32 board is
 *     outside all three.
 *   - `logring` kept the last words across a warm reset — but only on ESP32; its
 *     `#else` branch defined the persistence attribute EMPTY, so on nRF the ring
 *     was plain `.bss` and reported "cold start" after every reset because it
 *     could not report anything else. (Fixed alongside this.)
 *
 * So a reboot left three kinds of silence at once, and the one message that
 * looked like evidence was a constant.
 *
 * WHAT IT CAN AND CANNOT KNOW
 * ---------------------------
 * Retained RAM answers the question that actually distinguishes the failure
 * classes here: did the RAM rail hold? A watchdog reset, a fault, a software
 * reboot all leave it intact, so the history survives and the boot counter goes
 * up. Losing power does not, so the magic fails to validate and the counter
 * restarts at 1 — and THAT is the signal for a power event, arrived at by
 * absence rather than by a register nobody can read.
 *
 * The reset-reason register is recorded when it says anything, but is not relied
 * on. `meshtastic_dfu_trigger.c` records why: on this board the Adafruit
 * bootloader runs before the app on every reset and consumes RESETREAS for its
 * own double-tap detection — a previous guard keyed on the watchdog bit "never
 * fired and dead images just boot-looped in the dark". A cause of zero here may
 * mean "power-on" or may mean "already eaten", so the two are recorded
 * differently: MESHTASTIC_BOOT_F_CAUSE_OK marks a reading that meant something.
 * A gauge that cannot tell "no" from "don't know" is worse than no gauge.
 */

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif

#include <zephyr/meshtastic/bootlog.h>

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* Same three-way choice logring makes, and for the same reason: the ESP32 has a
 * dedicated RTC region that outlives more than ordinary RAM, everything else has
 * .noinit, and native_sim has no reset to survive. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#define BOOTLOG_ATTR RTC_NOINIT_ATTR
#elif defined(CONFIG_ARCH_POSIX)
#define BOOTLOG_ATTR
#else
#define BOOTLOG_ATTR __noinit
#endif

#define BOOTLOG_MAGIC   0x424F4F54U /* "BOOT" */
#define BOOTLOG_ENTRIES CONFIG_MESHTASTIC_BOOTLOG_ENTRIES

static BOOTLOG_ATTR uint32_t bl_magic;
static BOOTLOG_ATTR uint32_t bl_boot_num;
static BOOTLOG_ATTR uint32_t bl_next; /* next write slot */
static BOOTLOG_ATTR uint32_t bl_count; /* valid entries, saturating at ENTRIES */
static BOOTLOG_ATTR struct meshtastic_boot_record bl_ring[BOOTLOG_ENTRIES];
/* Updated while running; read by the NEXT boot, which is the whole point. */
static BOOTLOG_ATTR uint16_t bl_uptime_s;

/* This boot's record, copied out of retained RAM at init so later reads cannot
 * be confused by the ring having wrapped since. */
static struct meshtastic_boot_record bl_this;

static int bootlog_init(void)
{
	uint32_t cause = 0U;
	bool cause_ok = false;
	bool warm;
	uint16_t prev_uptime;

#if defined(CONFIG_HWINFO)
	/* Read AND clear: the register latches across resets, so leaving it set
	 * would make the next boot inherit this one's reason. Clearing is what
	 * makes a later non-zero reading mean "this reset", not "some reset". */
	if (hwinfo_get_reset_cause(&cause) == 0) {
		cause_ok = (cause != 0U);
		(void)hwinfo_clear_reset_cause();
	}
#endif

	warm = (bl_magic == BOOTLOG_MAGIC);
	prev_uptime = warm ? bl_uptime_s : 0U;

	if (!warm || bl_next >= BOOTLOG_ENTRIES || bl_count > BOOTLOG_ENTRIES) {
		/* Retained RAM lost, or its indices are garbage. Either way the
		 * history cannot be trusted, so start a fresh one rather than print
		 * whatever was in the memory. A counter that restarts at 1 IS the
		 * report: something took the RAM rail down. */
		bl_magic = BOOTLOG_MAGIC;
		bl_boot_num = 0U;
		bl_next = 0U;
		bl_count = 0U;
		memset(bl_ring, 0, sizeof(bl_ring));
		warm = false;
		prev_uptime = 0U;
	}

	bl_boot_num++;
	bl_uptime_s = 0U;

	bl_this.boot_num = bl_boot_num;
	bl_this.cause = cause;
	bl_this.flags = (uint16_t)((warm ? MESHTASTIC_BOOT_F_WARM : 0U) |
				   (cause_ok ? MESHTASTIC_BOOT_F_CAUSE_OK : 0U));
	bl_this.prev_uptime_s = prev_uptime;

	bl_ring[bl_next] = bl_this;
	bl_next = (bl_next + 1U) % BOOTLOG_ENTRIES;
	if (bl_count < BOOTLOG_ENTRIES) {
		bl_count++;
	}

	return 0;
}

/* PRE_KERNEL_1: before anything can log, and well before any driver that might
 * itself reset the part. Nothing here needs the kernel — it is register reads
 * and stores into retained RAM. */
SYS_INIT(bootlog_init, PRE_KERNEL_1, 1);

void meshtastic_bootlog_heartbeat(uint32_t uptime_s)
{
	bl_uptime_s = (uint16_t)MIN(uptime_s, (uint32_t)UINT16_MAX);
}

void meshtastic_bootlog_this_boot(struct meshtastic_boot_record *out)
{
	if (out != NULL) {
		*out = bl_this;
	}
}

size_t meshtastic_bootlog_history(struct meshtastic_boot_record *out, size_t max)
{
	size_t n;

	if (out == NULL || max == 0U || bl_magic != BOOTLOG_MAGIC) {
		return 0U;
	}

	n = MIN((size_t)bl_count, max);
	for (size_t i = 0; i < n; i++) {
		/* Oldest first. bl_next points one past the newest, so walking back
		 * n slots from it lands on the oldest entry we are returning. */
		size_t idx = (bl_next + BOOTLOG_ENTRIES - n + i) % BOOTLOG_ENTRIES;

		out[i] = bl_ring[idx];
	}
	return n;
}

const char *meshtastic_bootlog_cause_str(uint32_t cause, char *buf, size_t buflen)
{
	static const struct {
		uint32_t bit;
		const char *name;
	} names[] = {
#if defined(CONFIG_HWINFO)
		{ RESET_PIN, "PIN" },
		{ RESET_SOFTWARE, "SOFTWARE" },
		{ RESET_BROWNOUT, "BROWNOUT" },
		{ RESET_POR, "POR" },
		{ RESET_WATCHDOG, "WATCHDOG" },
		{ RESET_DEBUG, "DEBUG" },
		{ RESET_SECURITY, "SECURITY" },
		{ RESET_LOW_POWER_WAKE, "LOW_POWER_WAKE" },
		{ RESET_CPU_LOCKUP, "LOCKUP" },
		{ RESET_PARITY, "PARITY" },
		{ RESET_PLL, "PLL" },
		{ RESET_CLOCK, "CLOCK" },
		{ RESET_HARDWARE, "HARDWARE" },
		{ RESET_USER, "USER" },
		{ RESET_TEMPERATURE, "TEMPERATURE" },
#endif
	};
	size_t used = 0;

	if (buf == NULL || buflen == 0U) {
		return "";
	}
	buf[0] = '\0';

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		if ((cause & names[i].bit) == 0U) {
			continue;
		}
		used += (size_t)snprintk(buf + used, buflen - used, "%s%s",
					 used ? " " : "", names[i].name);
		if (used >= buflen) {
			return buf;
		}
	}

	if (used == 0U) {
		(void)snprintk(buf, buflen, "none");
	}
	return buf;
}

#if defined(CONFIG_MESHTASTIC_BOOTLOG_DURABLE)

/* ---- the durable ring -------------------------------------------------- *
 *
 * Everything above lives in retained RAM and dies with the RAM rail. This half
 * is the same history in flash, for the reason set out in Kconfig.bootlog: on a
 * fleet reachable only by LoRa and BLE, recovering a node that answers neither
 * means removing its power, so the rescue is exactly what erases the record of
 * why it needed rescuing.
 *
 * One settings key holds the whole ring rather than a key per slot. That makes
 * an append a single write of a couple of hundred bytes instead of a
 * read-modify-write across N keys, and it means the ring can never be found
 * half-updated with slots from two different generations.
 */
#include <zephyr/settings/settings.h>

#include "meshtastic_clock.h"

#define BL_DUR_SUBTREE "mtboot"
#define BL_DUR_KEY     BL_DUR_SUBTREE "/ring"
#define BL_DUR_MAGIC   0x424C4452U /* "BLDR" */
#define BL_DUR_VERSION 1U
#define BL_DUR_ENTRIES CONFIG_MESHTASTIC_BOOTLOG_DURABLE_ENTRIES

struct bl_durable_blob {
	uint32_t magic;
	uint16_t version;
	uint16_t count; /* valid entries, saturating at BL_DUR_ENTRIES */
	uint16_t next;  /* next write slot */
	uint16_t _pad;  /* keep the ring 4-byte aligned and the size stable */
	struct meshtastic_boot_durable ring[BL_DUR_ENTRIES];
} __packed;

static struct bl_durable_blob bl_dur;

static void bl_durable_fresh(void)
{
	memset(&bl_dur, 0, sizeof(bl_dur));
	bl_dur.magic = BL_DUR_MAGIC;
	bl_dur.version = BL_DUR_VERSION;
}

/* Pure: append into the cached ring. Split out because the wrap arithmetic is
 * the only part with anywhere to hide a bug, and it is testable without flash. */
static void bl_durable_append(const struct meshtastic_boot_durable *rec)
{
	if (bl_dur.magic != BL_DUR_MAGIC || bl_dur.next >= BL_DUR_ENTRIES ||
	    bl_dur.count > BL_DUR_ENTRIES) {
		/* Never written, wrong version, or indices we cannot trust. Starting
		 * clean loses history; printing garbage would lose the reader. */
		bl_durable_fresh();
	}

	bl_dur.ring[bl_dur.next] = *rec;
	bl_dur.next = (uint16_t)((bl_dur.next + 1U) % BL_DUR_ENTRIES);
	if (bl_dur.count < BL_DUR_ENTRIES) {
		bl_dur.count++;
	}
}

static int bl_durable_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				   void *cb_arg)
{
	struct bl_durable_blob in;

	if (strcmp(key, "ring") != 0) {
		return -ENOENT;
	}
	/* A size or version change means the on-flash layout is not this one.
	 * Drop it rather than reinterpret the bytes: a mis-decoded boot history
	 * is worse than an empty one, because it reads as fact. */
	if (len != sizeof(in) || read_cb(cb_arg, &in, len) != (ssize_t)len) {
		return 0;
	}
	if (in.magic != BL_DUR_MAGIC || in.version != BL_DUR_VERSION ||
	    in.next >= BL_DUR_ENTRIES || in.count > BL_DUR_ENTRIES) {
		return 0;
	}

	bl_dur = in;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mt_bootlog, BL_DUR_SUBTREE, NULL, bl_durable_settings_set, NULL,
			       NULL);

size_t meshtastic_bootlog_durable_history(struct meshtastic_boot_durable *out, size_t max)
{
	size_t n;

	if (out == NULL || max == 0U || bl_dur.magic != BL_DUR_MAGIC) {
		return 0U;
	}

	n = MIN((size_t)bl_dur.count, max);
	for (size_t i = 0; i < n; i++) {
		size_t idx = ((size_t)bl_dur.next + BL_DUR_ENTRIES - n + i) % BL_DUR_ENTRIES;

		out[i] = bl_dur.ring[idx];
	}
	return n;
}

/* Runs after settings have loaded (so the ring is the one from flash) and after
 * the clock module has had its chance to restore a persisted epoch.
 *
 * The record is written IMMEDIATELY rather than waiting for the clock to become
 * valid from GNSS or the mesh. A timestamp would be nicer; a record that a dying
 * node never got round to writing is worthless. wall_s == 0 says "no clock yet"
 * and the rest of the record still carries the reset cause, the warm/cold bit
 * and how long the previous run lasted.
 */
static int bl_durable_record_boot(void)
{
	struct meshtastic_boot_durable rec = {
		.boot_num = bl_this.boot_num,
		.cause = bl_this.cause,
		.wall_s = meshtastic_clock_valid() ? meshtastic_clock_now_epoch() : 0U,
		.flags = bl_this.flags,
		.prev_uptime_s = bl_this.prev_uptime_s,
	};

	bl_durable_append(&rec);

	if (settings_save_one(BL_DUR_KEY, &bl_dur, sizeof(bl_dur)) != 0) {
		/* Not fatal: the RAM history still works for warm resets. But say so,
		 * because the whole point of this ring is the case where nobody is
		 * watching, and a silent failure here would be discovered only when
		 * the history was needed and absent. */
		LOG_WRN("bootlog: durable history save FAILED — this boot will not "
			"survive a power cycle");
		return 0;
	}

	LOG_DBG("bootlog: durable record #%u written (%u retained)", rec.boot_num,
		bl_dur.count);
	return 0;
}

/* APPLICATION, after MESHTASTIC_SETTINGS_INIT_PRIORITY has loaded the subtree. */
SYS_INIT(bl_durable_record_boot, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void meshtastic_bootlog_durable_report(void)
{
	struct meshtastic_boot_durable hist[BL_DUR_ENTRIES];
	char cbuf[64];
	size_t n = meshtastic_bootlog_durable_history(hist, ARRAY_SIZE(hist));

	if (n == 0U) {
		LOG_INF("durable boot history: empty (nothing written yet)");
		return;
	}

	LOG_INF("durable boot history, oldest first (%u kept, survives power loss):",
		(unsigned int)n);
	for (size_t i = 0; i < n; i++) {
		LOG_INF("  #%u %s ran %us at %u cause 0x%08x %s", hist[i].boot_num,
			(hist[i].flags & MESHTASTIC_BOOT_F_WARM) ? "warm" : "COLD",
			hist[i].prev_uptime_s, hist[i].wall_s, hist[i].cause,
			(hist[i].flags & MESHTASTIC_BOOT_F_CAUSE_OK)
				? meshtastic_bootlog_cause_str(hist[i].cause, cbuf, sizeof(cbuf))
				: "(cause unavailable)");
	}
}

void meshtastic_bootlog_test_durable_reset(void)
{
	bl_durable_fresh();
}

void meshtastic_bootlog_test_durable_append(const struct meshtastic_boot_durable *rec)
{
	if (rec != NULL) {
		bl_durable_append(rec);
	}
}

int meshtastic_bootlog_test_durable_save(void)
{
	return settings_save_one(BL_DUR_KEY, &bl_dur, sizeof(bl_dur));
}

#else /* !CONFIG_MESHTASTIC_BOOTLOG_DURABLE */

size_t meshtastic_bootlog_durable_history(struct meshtastic_boot_durable *out, size_t max)
{
	ARG_UNUSED(out);
	ARG_UNUSED(max);
	return 0U;
}

void meshtastic_bootlog_durable_report(void)
{
	LOG_INF("durable boot history: not built in "
		"(CONFIG_MESHTASTIC_BOOTLOG_DURABLE=n)");
}

void meshtastic_bootlog_test_durable_reset(void)
{
}

void meshtastic_bootlog_test_durable_append(const struct meshtastic_boot_durable *rec)
{
	ARG_UNUSED(rec);
}

int meshtastic_bootlog_test_durable_save(void)
{
	return -ENOTSUP;
}

#endif /* CONFIG_MESHTASTIC_BOOTLOG_DURABLE */

void meshtastic_bootlog_report(void)
{
	struct meshtastic_boot_record hist[BOOTLOG_ENTRIES];
	char cbuf[64];
	size_t n;

	/* The headline is warm-vs-cold, because that is the half this can state
	 * with confidence on every target. */
	if ((bl_this.flags & MESHTASTIC_BOOT_F_WARM) != 0U) {
		LOG_INF("Boot #%u: WARM reset (retained RAM survived); previous run lasted "
			"%u s", bl_this.boot_num, bl_this.prev_uptime_s);
	} else {
		LOG_INF("Boot #%u: retained RAM was LOST — power cycle, brownout, or a "
			"reset that clears RAM. No history from before this boot.",
			bl_this.boot_num);
	}

	if ((bl_this.flags & MESHTASTIC_BOOT_F_CAUSE_OK) != 0U) {
		LOG_INF("  reset cause 0x%08x: %s", bl_this.cause,
			meshtastic_bootlog_cause_str(bl_this.cause, cbuf, sizeof(cbuf)));
	} else {
		/* Not "cause: none" — that would claim a reading we did not get. */
		LOG_INF("  reset cause unavailable (register empty or consumed by the "
			"bootloader before the app ran)");
	}

	n = meshtastic_bootlog_history(hist, ARRAY_SIZE(hist));
	if (n > 1U) {
		LOG_INF("  boot history, oldest first (%u retained):", (unsigned int)n);
		for (size_t i = 0; i < n; i++) {
			LOG_INF("    #%u %s ran %us cause 0x%08x", hist[i].boot_num,
				(hist[i].flags & MESHTASTIC_BOOT_F_WARM) ? "warm" : "COLD",
				hist[i].prev_uptime_s, hist[i].cause);
		}
	}
}
