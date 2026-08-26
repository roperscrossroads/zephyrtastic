/* SPDX-License-Identifier: GPL-3.0
 *
 * Log ring in RTC-persistent memory: the last few KB of the log stream, kept
 * across a warm reset and dumped on the next boot.
 *
 * WHY THIS EXISTS
 * ---------------
 * Every diagnostic this project has for a hang currently depends on the network
 * being up:
 *
 *   - log_backend_net is the only log backend on the shipping profile, so a node
 *     that loses WiFi is silent by definition, and a node that hangs stops
 *     sending mid-sentence with nothing kept locally.
 *   - Worse, LOG_PANIC() puts every backend into panic mode, where
 *     log_backend_net unconditionally no-ops (BSD sockets are not safe to drive
 *     synchronously from that context). So the moment things go wrong is exactly
 *     the moment netlog stops -- documented repeatedly in docs/KNOWN-ISSUES.md,
 *     and the reason the crash breadcrumbs had to be RTC-persistent structs in
 *     the first place.
 *   - And on 2026-08-11T14:45Z all three bench nodes reset inside a 20 s window
 *     with nothing in any log to explain it. The netlog's last line is simply
 *     wherever the node got to; there is no local tail to inspect afterwards.
 *
 * A breadcrumb struct answers "what state was it in". It cannot answer "what was
 * it doing". This fills that gap: the actual last log lines, verbatim, readable
 * after the reboot, with no dependency on WiFi, no filesystem, and -- unlike a
 * flash-backed log -- no flash write at all.
 *
 * That last point is not incidental. On the ESP32 a flash write disables the
 * instruction cache, during which non-IRAM ISRs cannot run: the same
 * interrupt-masking mechanism currently suspected of causing the 14:45 stall.
 * Writing plain stores into RTC memory has none of that.
 *
 * WHAT IT SURVIVES
 * ----------------
 * Warm resets: watchdog (both stages), software reboot, and any fatal-error
 * path. That is every failure class the bench has actually seen. A true POR
 * (power cycle) clears it -- the magic check below then just starts fresh
 * rather than printing garbage.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <string.h>

#include <zephyr/meshtastic/logring.h>

LOG_MODULE_REGISTER(mt_logring, CONFIG_MESHTASTIC_LOGRING_LOG_LEVEL);

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_attr.h>
#define LOGRING_RTC_ATTR RTC_NOINIT_ATTR
#elif defined(CONFIG_ARCH_POSIX)
/* native_sim / unit tests: an ordinary static. Nothing survives a reset there,
 * but there is no reset to survive either, and it keeps the file building. */
#define LOGRING_RTC_ATTR
#else
/* Everywhere else -- nRF above all, which is now most of the fleet -- .noinit.
 *
 * This branch used to be the empty one, sharing the native_sim case by omission
 * rather than by decision, and the consequence was silent: on every XIAO the
 * ring was plain .bss, zeroed by the C runtime before this file ever ran. So
 * `logring prev` answered "no pre-reset tail (cold start)" after EVERY reset,
 * warm or cold, because it could not answer anything else. A message that is
 * constant looks exactly like a finding, and on 2026-08-26 it was read as one.
 *
 * Verified in the built image rather than assumed: the noinit section begins at
 * 0x20014740, boot_guard_magic (meshtastic_dfu_trigger.c, __noinit) sits inside
 * it at 0x200274d4 and does persist, while ring_buf sat at 0x200132c8 -- below
 * it, in .bss. So __noinit demonstrably works on this target; the ring just was
 * not using it.
 *
 * Everything downstream was already written for memory that survives: the latch
 * validates a magic, bounds-checks the head index, and starts clean on garbage.
 * It was waiting for this attribute. */
#define LOGRING_RTC_ATTR __noinit
#endif

#define LOGRING_MAGIC 0x4C4F4752U /* "LOGR" */
#define RING_SIZE     CONFIG_MESHTASTIC_LOGRING_SIZE

/* Layout kept deliberately dumb -- three scalars and a byte array. Anything
 * cleverer (per-entry headers, checksums) would have to be written correctly by
 * a context that might be mid-panic, and would still not survive a torn write
 * any better than accepting one truncated line does. */
static LOGRING_RTC_ATTR uint32_t ring_magic;
static LOGRING_RTC_ATTR uint32_t ring_head;    /* next write offset */
static LOGRING_RTC_ATTR uint32_t ring_wrapped; /* has it filled at least once */
static LOGRING_RTC_ATTR char ring_buf[RING_SIZE];

/* Snapshot of what was in RTC memory when THIS boot started, taken before the
 * current boot has had a chance to overwrite it. Without this, the previous
 * boot's tail -- the interesting part -- would be steadily eaten by our own
 * boot-time logging in the ~30 s before the network is up enough to report it.
 * Costs a second RING_SIZE buffer in DRAM; that is the price of being able to
 * read the thing at a convenient moment rather than the earliest possible one. */
static char prev_buf[RING_SIZE];
static uint32_t prev_len;
static bool prev_valid;

/* Set while dumping so the dump's own output is not fed straight back into the
 * ring, which would otherwise overwrite the tail as we print it. */
static bool dumping;

/* The automatic boot dump is one-shot; the SNAPSHOT ITSELF is retained for the
 * life of the boot so `logring prev` can re-read it later. That distinction
 * matters now that netlog does not autostart: the boot dump goes to whatever
 * backends happen to be live at IPv4-up, which with remote syslog off may be
 * nobody at all. Discarding the data at that point would mean a reboot's
 * evidence existed only in a log line nobody received. */
static bool dumped_once;

static uint8_t out_buf[CONFIG_MESHTASTIC_LOGRING_LINE_MAX];

static void ring_write(const char *data, size_t len)
{
	unsigned int key;

	if (len == 0U) {
		return;
	}
	if (len > RING_SIZE) {
		/* Keep the tail: the end of an over-long line is the part with the
		 * interesting values in it. */
		data += (len - RING_SIZE);
		len = RING_SIZE;
	}

	/* irq_lock rather than a spinlock: this can be entered from a thread, an
	 * ISR, or a panicking context that interrupted any of those, and a
	 * spinlock taken here could deadlock against a holder we interrupted.
	 * irq_lock cannot, is safe from every context, and the critical section
	 * is a memcpy of at most CONFIG_MESHTASTIC_LOGRING_LINE_MAX bytes --
	 * sub-microsecond, and worth stating plainly given that this project is
	 * currently chasing a fault whose signature IS masked interrupts. */
	key = irq_lock();

	size_t first = MIN(len, (size_t)(RING_SIZE - ring_head));

	memcpy(&ring_buf[ring_head], data, first);
	if (len > first) {
		memcpy(&ring_buf[0], data + first, len - first);
		ring_wrapped = 1U;
	}
	ring_head += len;
	if (ring_head >= RING_SIZE) {
		ring_head -= RING_SIZE;
		ring_wrapped = 1U;
	}
	ring_magic = LOGRING_MAGIC;

	irq_unlock(key);
}

static int out_func(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	if (!dumping) {
		ring_write((const char *)data, length);
	}

	return (int)length;
}

LOG_OUTPUT_DEFINE(logring_output, out_func, out_buf, sizeof(out_buf));

static uint32_t fmt_flags(void)
{
	return LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP |
	       (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP) ? LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP
							        : 0);
}

static void backend_process(const struct log_backend *const backend, union log_msg_generic *msg)
{
	ARG_UNUSED(backend);

	log_output_msg_process(&logring_output, &msg->log, fmt_flags());
}

static void backend_dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	log_output_dropped_process(&logring_output, cnt);
}

/* Deliberately does NOT stop writing. This is the whole point of the module:
 * log_backend_net no-ops in panic mode, which is precisely when the output
 * matters, so the one backend that CAN keep going here must keep going. Writing
 * is plain stores into RTC memory behind irq_lock -- no driver, no socket, no
 * allocation, nothing that can block or fail. */
static void backend_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	log_output_flush(&logring_output);
}

static void backend_init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
}

static const struct log_backend_api logring_api = {
	.process = backend_process,
	.dropped = backend_dropped,
	.panic = backend_panic,
	.init = backend_init,
};

LOG_BACKEND_DEFINE(log_backend_logring, logring_api, true);

bool meshtastic_logring_have_previous(void)
{
	return prev_valid && prev_len > 0U;
}

void meshtastic_logring_dump(void)
{
	if (dumped_once) {
		return;
	}

	if (!meshtastic_logring_have_previous()) {
		LOG_INF("Log ring: nothing from a previous boot "
			"(cold start, or first boot with this feature)");
		dumped_once = true;
		return;
	}

	dumping = true;

	LOG_INF("===== Log ring: %u bytes recovered from before the last reset =====",
		(unsigned int)prev_len);

	/* Emitted as raw printk lines rather than LOG_INF per line: the content is
	 * already formatted log output (it has its own timestamps and levels), so
	 * re-wrapping it in another LOG_INF would double every prefix and make the
	 * recovered timestamps much harder to read against the live ones. */
	for (uint32_t i = 0; i < prev_len; i++) {
		char c = prev_buf[i];

		if (c == '\r') {
			continue;
		}
		printk("%c", c);
	}
	printk("\n");

	LOG_INF("===== end of recovered log ring =====");

	dumping = false;

	/* One-shot for the AUTOMATIC dump only -- a later, unrelated boot must not
	 * re-report a stale tail as if it were fresh. prev_buf itself is kept so
	 * `logring prev` can still retrieve it; it is cleared at the next boot's
	 * latch, which is the correct lifetime. */
	dumped_once = true;
}

/* Snapshot RTC memory into DRAM before this boot's own logging overwrites it.
 * PRE_KERNEL_1 so it runs ahead of essentially everything, including the log
 * subsystem's own startup output. */
static int logring_latch(void)
{
	if (ring_magic != LOGRING_MAGIC) {
		/* Cold boot, or RTC memory lost (a true POR clears it). Start clean
		 * rather than printing whatever noise is in there. */
		ring_magic = LOGRING_MAGIC;
		ring_head = 0U;
		ring_wrapped = 0U;
		prev_valid = false;
		prev_len = 0U;
		return 0;
	}

	if (ring_head > RING_SIZE) {
		/* Corrupt index -- treat as unusable rather than reading OOB. */
		ring_head = 0U;
		ring_wrapped = 0U;
		prev_valid = false;
		return 0;
	}

	if (ring_wrapped) {
		/* Oldest byte is at head; copy head..end then 0..head so the DRAM
		 * copy reads in chronological order. */
		uint32_t tail = RING_SIZE - ring_head;

		memcpy(prev_buf, &ring_buf[ring_head], tail);
		memcpy(prev_buf + tail, &ring_buf[0], ring_head);
		prev_len = RING_SIZE;
	} else {
		memcpy(prev_buf, ring_buf, ring_head);
		prev_len = ring_head;
	}
	prev_valid = (prev_len > 0U);

	/* Start this boot's own capture from the beginning. The previous content
	 * is safe in prev_buf now. */
	ring_head = 0U;
	ring_wrapped = 0U;

	return 0;
}

SYS_INIT(logring_latch, PRE_KERNEL_1, 0);

#if defined(CONFIG_SHELL)
/* On-demand readback. The boot dump is one-shot and goes out over whatever
 * backend is live at the time; this is for the case where you are already on the
 * node's shell and want the tail without waiting for -- or causing -- a reset.
 * Reads the LIVE ring, not the boot snapshot, so it shows what has happened
 * since this boot started. */
static int cmd_logring(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t len, start;

	/* `logring prev` -- the tail recovered from BEFORE the last reset. Kept
	 * readable for the whole boot precisely because, with netlog no longer
	 * autostarting, the automatic boot dump may reach no backend at all. This
	 * is how a monitor retrieves a reboot's evidence after the fact. */
	if (argc >= 2 && strcmp(argv[1], "prev") == 0) {
		if (!prev_valid || prev_len == 0U) {
			shell_print(sh, "log ring: no pre-reset tail (cold start)");
			return 0;
		}
		shell_print(sh, "--- log ring: %u bytes recovered from before the last reset ---",
			    (unsigned int)prev_len);
		for (uint32_t i = 0; i < prev_len; i++) {
			char c = prev_buf[i];

			if (c != '\r') {
				shell_fprintf(sh, SHELL_NORMAL, "%c", c);
			}
		}
		shell_print(sh, "\n--- end ---");
		return 0;
	}

	if (ring_magic != LOGRING_MAGIC) {
		shell_print(sh, "log ring: empty");
		return 0;
	}

	len = ring_wrapped ? RING_SIZE : ring_head;
	start = ring_wrapped ? ring_head : 0U;

	shell_print(sh, "--- log ring: %u bytes from the current boot ---", (unsigned int)len);
	for (uint32_t i = 0; i < len; i++) {
		char c = ring_buf[(start + i) % RING_SIZE];

		if (c != '\r') {
			shell_fprintf(sh, SHELL_NORMAL, "%c", c);
		}
	}
	shell_print(sh, "\n--- end ---");

	return 0;
}

SHELL_CMD_REGISTER(logring, NULL,
		   "Dump the RTC log ring. `logring` = this boot, `logring prev` = "
		   "the tail recovered from before the last reset.",
		   cmd_logring);
#endif /* CONFIG_SHELL */
