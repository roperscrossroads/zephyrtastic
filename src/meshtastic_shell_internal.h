/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The shell's own API between its command files (agents-ls39.4.4.1): the work
 * thread, the config gates, and the parsers and formatters more than one command
 * group uses. Not a public header: only src/meshtastic_shell*.c and the
 * *_shell.c command files include it.
 */
#ifndef MESHTASTIC_SHELL_INTERNAL_H_
#define MESHTASTIC_SHELL_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/toolchain.h>

#include "meshtastic/config.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_clock.h"

/* ---- The work thread ---------------------------------------------------- */

/* The shell work thread (agents-ls39.4.4.1). A command that must not run on the shell
 * thread's own stack (the send path, PBKDF2, a cluster publish: each overflowed the
 * XIAO's 2048-byte shell stack once, see the run functions' comments) parses its
 * arguments there, and queues a run function with them. The thread knows no command:
 * it calls the function, then prints the result line ("done" or "failed: ..."). Each
 * command owns its argument layout; the arguments live in a pool item, not on the
 * shell stack. */
typedef int (*mtsh_work_fn)(const struct shell *sh, const void *arg, size_t len);

/* Room for the largest argument: a text message and its address. */
#define MTSH_WORK_ARG_MAX (MESHTASTIC_MAX_TEXT_LEN + 1U + 8U)

struct mtsh_work {
	const struct shell *sh;
	mtsh_work_fn run;
	bool quiet; /* no result line: the caller waits and prints itself */
	size_t len;
	uint8_t arg[MTSH_WORK_ARG_MAX] __aligned(4);
};

/** @brief An empty item for @p run from the pool; NULL (said on @p sh) when the queue is full. */
struct mtsh_work *mtsh_work_alloc(const struct shell *sh, mtsh_work_fn run);

/** @brief Give an item back unqueued (its command refused the arguments after all). */
void mtsh_work_cancel(struct mtsh_work *w);

/** @brief Queue a filled item with @p len bytes of argument, and print "queued". */
int mtsh_work_submit(struct mtsh_work *w, size_t len);

/** @brief Queue @p run with a copy of @p len bytes at @p arg. */
int mtsh_enqueue(const struct shell *sh, mtsh_work_fn run, const void *arg, size_t len);

/** @brief The common argument, one node id, back out of an item. */
uint32_t mtsh_work_dest(const void *arg, size_t len);

/** @brief `<cmd> [dest]`: queue @p run with the node id (broadcast when absent). */
int mtsh_deferred_send(const struct shell *sh, size_t argc, char **argv, mtsh_work_fn run);

/**
 * @brief Run a command's handler on the work thread while the shell thread waits.
 *
 * Every command that changes state runs this way (agents-ls39.4.4.1): the shell
 * thread's stack is 2 KB on a XIAO, and each config write, NVS save or send that ran
 * on it was one more way to overflow it, found one crash at a time. Its output and
 * return code are the handler's own, unchanged: the shell lends its context to the
 * work thread while a command runs (exec_cmd releases the shell's lock for exactly
 * this). No "queued" or "done" line.
 *
 * Every `meshtastic` command is registered this way except the few in
 * mtsh_shell_thread_cmds[]: one that reads from the console (shell_readline) or waits
 * on the work thread has to run on the shell thread.
 */
int mtsh_run_on_work(const struct shell *sh, shell_cmd_handler handler, size_t argc, char **argv);

/**
 * @brief Define `<handler>_on_work`, the handler run through mtsh_run_on_work().
 *
 * Register the result instead of the handler:
 *   MTSH_DEFINE_ON_WORK(cmd_owner_set)
 *   SHELL_CMD(set, NULL, help, cmd_owner_set_on_work)
 */
#define MTSH_DEFINE_ON_WORK(handler)                                                         \
	static int handler##_on_work(const struct shell *sh, size_t argc, char **argv)       \
	{                                                                                    \
		return mtsh_run_on_work(sh, handler, argc, argv);                            \
	}                                                                                    \
	Z_MTSH_WRAPPED(handler)

/** @brief A command that stays on the shell thread, and why (one line). */
struct mtsh_shell_thread_cmd {
	const char *path; /* the words after `meshtastic` */
	const char *why;
};

extern const struct mtsh_shell_thread_cmd mtsh_shell_thread_cmds[];
extern const size_t mtsh_shell_thread_cmds_len;

#if defined(CONFIG_ZTEST)
/* Test builds list every wrapper in a section, so tests/shell can tell a wrapped
 * handler from one that runs on the shell thread. Production builds carry nothing. */
struct mtsh_wrapped {
	shell_cmd_handler fn;
};

#define Z_MTSH_WRAPPED(handler)                                                              \
	static const STRUCT_SECTION_ITERABLE(mtsh_wrapped, handler##_wrapped) = {            \
		.fn = handler##_on_work,                                                     \
	};

/** @brief Test-only: the shell work thread. */
k_tid_t mtsh_test_work_thread(void);
#else
#define Z_MTSH_WRAPPED(handler)
#endif

/* ---- Gates (each prints why it refused) ---------------------------------- */

/** @brief True while lockdown holds the store: config is neither readable nor writable. */
bool mtsh_store_locked(const struct shell *sh);

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/** @brief True on an administratively managed node. */
bool mtsh_managed_refused(const struct shell *sh);

/** @brief Every config write: refused while the store is locked, then on a managed node. */
bool mtsh_config_write_refused(const struct shell *sh);
#endif

/* ---- Parsers (meshtastic_shell_util.c; each prints what it refused) ------ */

/** @brief An integer (any base strtoul takes), or "broadcast". */
int mtsh_parse_u32(const struct shell *sh, const char *arg, uint32_t *value);

/** @brief `<cmd> [dest]`: broadcast when absent. */
int mtsh_parse_optional_dest(const struct shell *sh, size_t argc, char **argv, uint32_t *dest);

/** @brief A channel slot, 0..MESHTASTIC_MAX_CHANNELS-1. */
int mtsh_parse_channel_index(const struct shell *sh, const char *arg, uint8_t *index);

/** @brief A preset by its display name (as `meshtastic lora` prints it). Prints nothing. */
int mtsh_parse_modem_preset(const char *s, meshtastic_Config_LoRaConfig_ModemPreset *out);

/** @brief argv[first_arg..] joined by single spaces into @p buf (not terminated).
 * @return the length, or -ENOMEM past @p buf_max. */
int mtsh_append_argv(uint8_t *buf, size_t buf_max, size_t argc, char **argv, size_t first_arg);

/* ---- Formatting ----------------------------------------------------------- */

/* A float as tenths, then its whole and fractional parts, for "%d.%u" without %f. */
int32_t mtsh_scaled_tenths(float value);
int32_t mtsh_scaled_whole(int32_t scaled, int32_t divisor);
uint32_t mtsh_scaled_fraction(int32_t scaled, int32_t divisor);

const char *mtsh_clock_quality_name(enum meshtastic_clock_quality q);
const char *mtsh_device_role_name(meshtastic_Config_DeviceConfig_Role role);
const char *mtsh_rebroadcast_mode_name(meshtastic_Config_DeviceConfig_RebroadcastMode mode);

#endif /* MESHTASTIC_SHELL_INTERNAL_H_ */
