/* SPDX-License-Identifier: GPL-3.0
 *
 * Log forwarding to the phone: each log message wrapped as a
 * FromRadio.log_record and handed to whatever PhoneAPI transports are attached.
 *
 * WHY THIS EXISTS
 * ---------------
 * On 2026-08-25 the fleet went all-LoRa+BLE. That removed the netlog: the
 * observability backbone here has been log_backend_net -- RFC5424 syslog over
 * WiFi -- and no node has WiFi any more. What was left was a USB console per
 * node, which resets an ESP32 board simply by being opened and which is also
 * the shell's own I/O path.
 *
 * Meanwhile FromRadio.log_record has been in our schema the whole time and we
 * have never sent one, while the official Android client already consumes it
 * (DebugViewModel, persisted into the app's MeshLog table). So the cheapest
 * observability available was a field we already had: no new protocol, no new
 * channel, no PSK, and -- because the PhoneAPI is a BLE/serial link, not the
 * radio -- no airtime whatsoever.
 *
 * WHAT IT IS NOT
 * --------------
 * Not a replacement for meshtastic_logring.c. That one survives a reset and
 * keeps writing during a panic, because all it does is store bytes in RTC
 * memory behind irq_lock. This one takes mutexes and hands work to a BLE stack,
 * so it stands down the moment LOG_PANIC() runs -- exactly the failure log
 * backends are notorious for pretending to handle. The two are complements: the
 * ring answers "what was it doing before it died", this answers "what is it
 * doing now, while I stand next to it with a phone".
 *
 * THE FEEDBACK LOOP, AND WHY IT CANNOT START
 * ------------------------------------------
 * A log backend that logs is a loop. Three independent guards, because one is
 * not enough for something that fails by running away:
 *
 *   1. Our own module is never forwarded. A record about the log path cannot
 *      produce another record about the log path.
 *   2. The queue path used here does not log at ALL. The ordinary
 *      meshtastic_phoneapi_enqueue_fromradio() logs on every path it takes,
 *      including the successful one, so log records get their own silent
 *      enqueue that drops on a full queue instead of evicting.
 *      meshtastic_phoneapi_enqueue_log_record() owns that rule; see its comment
 *      for why it is not a shortcut.
 *   3. A token bucket caps the rate outright. Even if some future caller closed
 *      a loop the other two missed, it would saturate at the cap and stop
 *      rather than consume the node.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_output.h>

#include "meshtastic_clock.h"
#include "meshtastic_core.h"
#include "meshtastic_phonelog.h"
#include "meshtastic_phoneapi.h"

LOG_MODULE_REGISTER(mt_phonelog, LOG_LEVEL_INF);

#define LINE_MAX CONFIG_MESHTASTIC_PHONELOG_LINE_MAX

BUILD_ASSERT(LINE_MAX <= sizeof(((meshtastic_LogRecord *)0)->message),
	     "PHONELOG_LINE_MAX exceeds LogRecord.message (see mesh.options)");

static struct {
	/* Runtime severity ceiling, 0 = forwarding off. Seeded from Kconfig. */
	uint8_t level;
	bool panicked;
	/* Set for the duration of one backend_process() call. Belt-and-braces
	 * against a same-thread re-entry; the module filter below is the guard
	 * that actually matters. */
	bool busy;

	/* Token bucket: `tokens` refilled to the configured rate each second. */
	uint8_t tokens;
	int64_t window_start_ms;

	uint32_t forwarded;
	uint32_t dropped_rate;  /* bucket empty */
	uint32_t dropped_queue; /* every attached transport was full */
	uint32_t dropped_level; /* below the runtime ceiling */
	uint32_t dropped_core;  /* the log core dropped them before we saw them */
} phonelog = {
	.level = CONFIG_MESHTASTIC_PHONELOG_LEVEL,
};

/* One log line, formatted. Only ever touched by the logging thread (this
 * backend requires LOG_MODE_DEFERRED, so there is exactly one). */
static char line[LINE_MAX];
static size_t line_len;
static uint8_t out_buf[LINE_MAX];

/* Zephyr severities are 1..4 (ERR..DBG); the wire uses python-style numbers.
 * Indexed by log level, so index 0 is the "none" slot. */
static const uint8_t level_to_wire[] = {
	[LOG_LEVEL_NONE] = meshtastic_LogRecord_Level_UNSET,
	[LOG_LEVEL_ERR] = meshtastic_LogRecord_Level_ERROR,
	[LOG_LEVEL_WRN] = meshtastic_LogRecord_Level_WARNING,
	[LOG_LEVEL_INF] = meshtastic_LogRecord_Level_INFO,
	[LOG_LEVEL_DBG] = meshtastic_LogRecord_Level_DEBUG,
};

/* Collect the formatted text rather than writing it anywhere: log_output calls
 * this in chunks, and a record is one line. Truncates at LINE_MAX; a truncated
 * diagnostic still says more than a dropped one. */
static int out_func(uint8_t *data, size_t length, void *ctx)
{
	size_t room;

	ARG_UNUSED(ctx);

	room = (line_len < sizeof(line)) ? (sizeof(line) - line_len) : 0U;
	if (room > 0U) {
		size_t take = MIN(room, length);

		memcpy(&line[line_len], data, take);
		line_len += take;
	}

	/* Always claim the whole chunk: reporting a short write makes log_output
	 * retry, and we have deliberately thrown the excess away. */
	return (int)length;
}

LOG_OUTPUT_DEFINE(phonelog_output, out_func, out_buf, sizeof(out_buf));

/* Refill once per wall second and spend one token. False when the bucket is
 * empty, which is the caller's cue to drop. */
static bool take_token(void)
{
	int64_t now = k_uptime_get();

	if ((now - phonelog.window_start_ms) >= (int64_t)MSEC_PER_SEC) {
		phonelog.window_start_ms = now;
		phonelog.tokens = CONFIG_MESHTASTIC_PHONELOG_RATE;
	}

	if (phonelog.tokens == 0U) {
		return false;
	}

	phonelog.tokens--;
	return true;
}

static void backend_process(const struct log_backend *const backend, union log_msg_generic *msg)
{
	static meshtastic_LogRecord record; /* ~420 B: too big for the log thread's stack */
	struct log_msg *m = &msg->log;
	uint8_t level = log_msg_get_level(m);
	int16_t source_id = log_msg_get_source_id(m);
	const char *source = NULL;
	int sent;

	ARG_UNUSED(backend);

	if (phonelog.panicked || phonelog.busy || phonelog.level == 0U) {
		return;
	}
	if (level == LOG_LEVEL_NONE || level > phonelog.level) {
		phonelog.dropped_level++;
		return;
	}

	if (source_id >= 0) {
		source = log_source_name_get(log_msg_get_domain(m), (uint32_t)source_id);
	}

	/* Guard 1: never forward our own module. Anything this file logs -- a
	 * warning about dropping records, most obviously -- would otherwise be a
	 * record, whose handling could log again. */
	if (source != NULL && strcmp(source, "mt_phonelog") == 0) {
		return;
	}

	/* Guard 3: the rate cap. Checked before formatting so a flood is cheap to
	 * refuse as well as bounded. */
	if (!take_token()) {
		phonelog.dropped_rate++;
		return;
	}

	phonelog.busy = true;

	/* Message text only. The app renders level, timestamp and source from the
	 * record's OWN fields, so letting log_output prefix them into the text
	 * would print every column twice in its Debug panel: SKIP_SOURCE drops the
	 * "module: " prefix, and passing neither TIMESTAMP nor LEVEL keeps those
	 * out too. CRLF_NONE keeps a line ending out of what is a plain string. */
	line_len = 0U;
	log_output_msg_process(&phonelog_output, m,
			       LOG_OUTPUT_FLAG_SKIP_SOURCE | LOG_OUTPUT_FLAG_CRLF_NONE);

	/* Belt and braces: strip any line ending that got through anyway. */
	while (line_len > 0U && (line[line_len - 1U] == '\n' || line[line_len - 1U] == '\r')) {
		line_len--;
	}

	record = (meshtastic_LogRecord)meshtastic_LogRecord_init_zero;
	memcpy(record.message, line, MIN(line_len, sizeof(record.message) - 1U));
	record.message[MIN(line_len, sizeof(record.message) - 1U)] = '\0';
	record.level = level_to_wire[level];
	if (source != NULL) {
		strncpy(record.source, source, sizeof(record.source) - 1U);
	}
	/* 0 means "unknown" on the wire, which is the honest answer on a node that
	 * has never been told the time -- every bench node right now. The app falls
	 * back to its own receive time. */
	record.time = meshtastic_clock_valid() ? meshtastic_clock_now_epoch() : 0U;

	sent = meshtastic_phoneapi_enqueue_log_record(&record, &phonelog.dropped_queue);
	if (sent > 0) {
		phonelog.forwarded++;
	}

	phonelog.busy = false;
}

static void backend_dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	/* Messages the log CORE dropped before any backend saw them -- its buffer
	 * filled faster than the logging thread drained it. Counted separately from
	 * our own rate cap because the two have different fixes
	 * (CONFIG_LOG_BUFFER_SIZE versus MESHTASTIC_PHONELOG_RATE), and conflating
	 * them would send anyone reading the counters to the wrong knob. Counted,
	 * never logged. */
	phonelog.dropped_core += cnt;
}

/* Stand down. Everything below this point wants a mutex and a working BLE
 * stack, and panic context offers neither. meshtastic_logring.c is the backend
 * that deliberately keeps writing here. */
static void backend_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	phonelog.panicked = true;
}

static void backend_init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	phonelog.window_start_ms = k_uptime_get();
	phonelog.tokens = CONFIG_MESHTASTIC_PHONELOG_RATE;
}

static const struct log_backend_api phonelog_api = {
	.process = backend_process,
	.dropped = backend_dropped,
	.panic = backend_panic,
	.init = backend_init,
};

LOG_BACKEND_DEFINE(log_backend_phonelog, phonelog_api, true);

uint8_t meshtastic_phonelog_get_level(void)
{
	return phonelog.level;
}

int meshtastic_phonelog_set_level(uint8_t level)
{
	if (level > LOG_LEVEL_DBG) {
		return -EINVAL;
	}

	phonelog.level = level;
	return 0;
}

void meshtastic_phonelog_get_stats(struct meshtastic_phonelog_stats *out)
{
	if (out == NULL) {
		return;
	}

	out->forwarded = phonelog.forwarded;
	out->dropped_rate = phonelog.dropped_rate;
	out->dropped_queue = phonelog.dropped_queue;
	out->dropped_level = phonelog.dropped_level;
	out->dropped_core = phonelog.dropped_core;
	out->panicked = phonelog.panicked;
}

void meshtastic_phonelog_reset_stats(void)
{
	phonelog.forwarded = 0U;
	phonelog.dropped_rate = 0U;
	phonelog.dropped_queue = 0U;
	phonelog.dropped_level = 0U;
	phonelog.dropped_core = 0U;
}
