/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"
#include "meshtastic_storage.h" /* declarations only; safe unconditionally */
#if defined(CONFIG_MESHTASTIC_ADMIN)
#include "meshtastic_admin.h"
#endif
#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
#include "meshtastic_lockdown.h"
#endif
#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
#include "meshtastic_shell_api.h"
#endif

LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

/* One writer per console. The serial shell prints the logs itself (its log backend) and
 * holds its output lock while a command prints; log_backend_uart writes the same UART with
 * uart_poll_out() from the log thread, outside that lock. Both on one UART print every log
 * line twice and put a log line's bytes inside a command's output: a `meshtastic api`
 * frame split that way reached the phone-API tunnel as a damaged line (agents-62n9). The
 * plain backend picks zephyr,log-uart when chosen, else zephyr,console. */
#if defined(CONFIG_LOG_BACKEND_UART) && defined(CONFIG_SHELL_BACKEND_SERIAL) &&                    \
	DT_HAS_CHOSEN(zephyr_shell_uart)
#if DT_HAS_CHOSEN(zephyr_log_uart)
#define MT_LOG_UART_NODE DT_CHOSEN(zephyr_log_uart)
#else
#define MT_LOG_UART_NODE DT_CHOSEN(zephyr_console)
#endif
BUILD_ASSERT(!DT_SAME_NODE(MT_LOG_UART_NODE, DT_CHOSEN(zephyr_shell_uart)),
	     "CONFIG_LOG_BACKEND_UART on the serial shell's UART: the shell's own log backend "
	     "already prints the logs there; drop CONFIG_LOG_BACKEND_UART");
#endif

static const char *shell_err_msg(int ret)
{
	switch (ret) {
	case -ENODATA:
		return "no GNSS fix available yet";
	case -ENODEV:
		return "no sensor readings available";
	case -ENOTSUP:
		return "not enabled in Kconfig";
	default:
		return strerror(-ret);
	}
}

static void shell_report_result(const struct shell *sh, int ret)
{
	if (ret < 0) {
		LOG_ERR("Meshtastic shell command failed: %s (%d)", shell_err_msg(ret), -ret);
	}

	if (sh == NULL) {
		return;
	}

	/*
	 * shell_print() can block on the shell TX mutex while the shell thread
	 * is busy (e.g. with log output). Log the outcome and attempt a shell
	 * message without stalling the work queue indefinitely.
	 */
	if (ret < 0) {
		shell_error(sh, "failed: %s (%d)", shell_err_msg(ret), -ret);
	} else {
		LOG_INF("Meshtastic shell command completed");
		shell_print(sh, "done");
	}
}

/* The pool holds the queued items' arguments; the queue carries pointers into it. An
 * item goes back to the pool as the thread takes it, so the depth is what it was when
 * the queue carried whole items: CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE waiting, one running. */
K_MEM_SLAB_DEFINE_STATIC(shell_work_slab, sizeof(struct mtsh_work), CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE,
			 4);
K_MSGQ_DEFINE(shell_work_msgq, sizeof(struct mtsh_work *), CONFIG_MESHTASTIC_SHELL_QUEUE_SIZE, 4);
K_THREAD_STACK_DEFINE(shell_work_stack, CONFIG_MESHTASTIC_SHELL_WORK_STACK_SIZE);
static struct k_thread shell_work_thread;

static void shell_work_thread_fn(void *p1, void *p2, void *p3)
{
	struct mtsh_work *queued;
	struct mtsh_work item;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msgq_get(&shell_work_msgq, &queued, K_FOREVER);
		memcpy(&item, queued, sizeof(item));
		memset(queued, 0, sizeof(*queued)); /* a passphrase may have been here */
		k_mem_slab_free(&shell_work_slab, queued);

		ret = item.run(item.sh, item.arg, item.len);
		if (!item.quiet) {
			shell_report_result(item.sh, ret);
		}
		memset(item.arg, 0, sizeof(item.arg));
	}
}

/* An empty item for @p run, from the pool. NULL when the queue is full (said on @p sh). */
struct mtsh_work *mtsh_work_alloc(const struct shell *sh, mtsh_work_fn run)
{
	struct mtsh_work *w;

	if (k_mem_slab_alloc(&shell_work_slab, (void **)&w, K_NO_WAIT) != 0) {
		if (sh != NULL) {
			shell_error(sh, "Meshtastic shell queue is full");
		}
		return NULL;
	}
	memset(w, 0, sizeof(*w));
	w->sh = sh;
	w->run = run;
	return w;
}

/* Give an item back unqueued: its command refused its arguments after all. */
void mtsh_work_cancel(struct mtsh_work *w)
{
	memset(w, 0, sizeof(*w));
	k_mem_slab_free(&shell_work_slab, w);
}

/* Queue a filled item (@p len bytes of argument) and say "queued". */
int mtsh_work_submit(struct mtsh_work *w, size_t len)
{
	const struct shell *sh = w->sh;
	int ret;

	w->len = len;
	ret = k_msgq_put(&shell_work_msgq, &w, K_NO_WAIT);
	if (ret < 0) {
		/* Not reached: the queue has a slot for every pool item. */
		mtsh_work_cancel(w);
		if (sh != NULL) {
			shell_error(sh, "Meshtastic shell queue is full");
		}
		return ret;
	}
	if (sh != NULL && !w->quiet) {
		shell_print(sh, "queued");
	}
	return 0;
}

/* Queue @p run with a copy of @p len bytes at @p arg. */
int mtsh_enqueue(const struct shell *sh, mtsh_work_fn run, const void *arg, size_t len)
{
	struct mtsh_work *w;

	__ASSERT_NO_MSG(len <= sizeof(w->arg));
	w = mtsh_work_alloc(sh, run);
	if (w == NULL) {
		return -ENOMSG;
	}
	if (len > 0U) {
		memcpy(w->arg, arg, len);
	}
	return mtsh_work_submit(w, len);
}

/* A command run on the work thread for a waiting shell thread: mtsh_run_on_work(). */
struct mtsh_call {
	shell_cmd_handler handler;
	size_t argc;
	char **argv;
	int rc;
	struct k_sem done;
};

static int mtsh_call_run(const struct shell *sh, const void *arg, size_t len)
{
	struct mtsh_call *call;

	ARG_UNUSED(len);
	memcpy(&call, arg, sizeof(call));
	call->rc = call->handler(sh, call->argc, call->argv);
	k_sem_give(&call->done);
	return 0;
}

int mtsh_run_on_work(const struct shell *sh, shell_cmd_handler handler, size_t argc, char **argv)
{
	struct mtsh_call call = {.handler = handler, .argc = argc, .argv = argv};
	struct mtsh_call *ptr = &call;
	struct mtsh_work *w;

	/* Already on the work thread (a queued command that runs another): no one to
	 * wait for, so run it here. */
	if (k_current_get() == &shell_work_thread) {
		return handler(sh, argc, argv);
	}

	/* The caller waits anyway, so it also waits for a free item rather than being
	 * told the queue is full: a command that used to run inline must not start
	 * failing because sends are queued. */
	if (k_mem_slab_alloc(&shell_work_slab, (void **)&w, K_FOREVER) != 0) {
		return -ENOMSG;
	}
	memset(w, 0, sizeof(*w));
	w->sh = sh;
	w->run = mtsh_call_run;
	w->quiet = true;
	w->len = sizeof(ptr);
	memcpy(w->arg, &ptr, sizeof(ptr));
	k_sem_init(&call.done, 0, 1);
	(void)k_msgq_put(&shell_work_msgq, &w, K_FOREVER);
	(void)k_sem_take(&call.done, K_FOREVER);
	return call.rc;
}

/* The commands that stay on the shell thread. Every other `meshtastic` command runs on
 * the work thread through mtsh_run_on_work() (agents-ls39.4.4.1), and tests/shell
 * walks the registered tree to check that each command is one or the other. A path is
 * the words after `meshtastic`. */
const struct mtsh_shell_thread_cmd mtsh_shell_thread_cmds[] = {
	{"api", "waits on the work thread for each step: there it would wait on itself"},
	{"crashtest", "`stack` overflows the shell thread's own stack on purpose"},
	{"dfu", "reboots into the bootloader at once: nothing to protect"},
	{"netpause", "sleeps for N s by design: on the work thread it would hold queued sends"},
	{"lockdown provision", "prompts for the passphrase (shell_readline); its work is queued"},
	{"lockdown unlock", "prompts for the passphrase (shell_readline); its work is queued"},
	{"lockdown disable", "prompts for the passphrase (shell_readline); its work is queued"},
};

const size_t mtsh_shell_thread_cmds_len = ARRAY_SIZE(mtsh_shell_thread_cmds);

#if defined(CONFIG_ZTEST)
k_tid_t mtsh_test_work_thread(void)
{
	return &shell_work_thread;
}
#endif

/* The common argument: one node id. */
uint32_t mtsh_work_dest(const void *arg, size_t len)
{
	uint32_t dest;

	__ASSERT_NO_MSG(len == sizeof(dest));
	ARG_UNUSED(len);
	memcpy(&dest, arg, sizeof(dest));
	return dest;
}

/* Shared by the position / metrics / environment / nodeinfo "send" commands. */
int mtsh_deferred_send(const struct shell *sh, size_t argc, char **argv, mtsh_work_fn run)
{
	uint32_t dest;
	int ret;

	ret = mtsh_parse_optional_dest(sh, argc, argv, &dest);
	if (ret < 0) {
		return ret;
	}

	return mtsh_enqueue(sh, run, &dest, sizeof(dest));
}

#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
/* Prints nothing on the work thread: the shell thread is waiting for this step and
 * does the printing. */
static int shell_api_run(const struct shell *sh, const void *arg, size_t len)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	meshtastic_shell_api_work();
	return 0;
}

int meshtastic_shell_api_submit(void)
{
	struct mtsh_work *w = mtsh_work_alloc(NULL, shell_api_run);

	if (w == NULL) {
		return -ENOMSG;
	}
	w->quiet = true;
	return mtsh_work_submit(w, 0U);
}
#endif

/* Refuse a config read or write while lockdown holds the store: locked, or
 * unlocked but not yet reloaded (agents-ls39.1.8). RAM then holds the seed's
 * placeholders, not the node's config. A write would change them live and its
 * save would be refused by the wrap, with nothing said; a read would show them
 * as if they were the node's. The admin dispatcher drops every payload in the
 * same state; this is the console's half. `lockdown`, `version`, `status`,
 * `resets` and the clock stay usable: they are how the operator gets out.
 *
 * Here and not in the config store's setters for now: the store's one
 * config_write_allowed(origin) is agents-ls39.4.2. */
bool mtsh_store_locked(const struct shell *sh)
{
#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
	if (meshtastic_lockdown_active() && !meshtastic_lockdown_store_ready()) {
		shell_error(sh, "refused: the store is locked (`meshtastic lockdown unlock` "
				"first); config is not readable or writable until then");
		return true;
	}
#else
	ARG_UNUSED(sh);
#endif
	return false;
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/* Refuse a config write when the node is administratively managed. The admin
 * model says a managed node takes configuration only from an authorized remote
 * admin; the shell writes the same config store, so it has to honour the same
 * answer or the gate means nothing. Returns true when the caller should stop.
 *
 * With admin compiled out there is no admin model to defer to, so nothing is
 * refused — the compile-time CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE switch is the
 * control that still applies in that build. */
bool mtsh_managed_refused(const struct shell *sh)
{
#if defined(CONFIG_MESHTASTIC_ADMIN)
	if (meshtastic_admin_is_managed()) {
		shell_error(sh, "refused: node is managed (SecurityConfig.is_managed) — "
				"configuration is set by an authorized remote admin");
		return true;
	}
#else
	ARG_UNUSED(sh);
#endif
	return false;
}

/* Every config write: refused while the store is locked, then on a managed node. */
bool mtsh_config_write_refused(const struct shell *sh)
{
	return mtsh_store_locked(sh) || mtsh_managed_refused(sh);
}
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

/* The root, `meshtastic`, is a section-backed set (agents-ls39.4.4.1): each command
 * adds itself with SHELL_SUBCMD_ADD((meshtastic), ...), so a command file needs no
 * line in a central table. `meshtastic help` lists them in name order. */
SHELL_SUBCMD_SET_CREATE(meshtastic_cmds, (meshtastic));

#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
SHELL_SUBCMD_ADD((meshtastic), api, NULL,
		 SHELL_HELP("The phone API as text, for tools: ToRadio in, ~P<FromRadio> lines out.",
		 "[[+] <base64> [<crc16>]]"),
		 meshtastic_shell_api_cmd, 0, 0);
#endif
#if defined(CONFIG_MESHTASTIC_STORAGE_STATS)
MTSH_DEFINE_ON_WORK(meshtastic_storage_shell_cmd)
SHELL_SUBCMD_ADD((meshtastic), storage, NULL,
		 SHELL_HELP("Settings store: free space, per-subtree use, boot restore time.",
		 NULL),
		 meshtastic_storage_shell_cmd_on_work, 0, 0);
#endif

SHELL_CMD_REGISTER(meshtastic, &meshtastic_cmds,
		   SHELL_HELP("Meshtastic mesh radio commands.", NULL), NULL);

static int meshtastic_shell_init(void)
{
#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)
	meshtastic_shell_api_init();
#endif
	k_thread_create(&shell_work_thread, shell_work_stack,
			K_THREAD_STACK_SIZEOF(shell_work_stack), shell_work_thread_fn, NULL, NULL,
			NULL, CONFIG_MESHTASTIC_SHELL_WORK_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&shell_work_thread, "meshtastic_shell");

	return 0;
}

SYS_INIT(meshtastic_shell_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
