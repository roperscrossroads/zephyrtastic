/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic lockdown` (agents-dnr4.15): the console side of lockdown. Built with
 * the shell and CONFIG_MESHTASTIC_LOCKDOWN (src/CMakeLists.txt). The first command
 * file out of meshtastic_shell.c (agents-ls39.4.4.1), and the template for the rest:
 * its own includes, its subcommand set, its work-thread run functions, and one
 * SHELL_SUBCMD_ADD into the root.
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "meshtastic_core.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_shell_internal.h"

/* Phase 4 gives the operator an obscured prompt: run `provision` / `unlock` /
 * `disable` with NO passphrase argument and the shell asks for it with echoing
 * turned off (shell_readline + shell_obscure_set), so it never reaches the
 * console log or the shell history. The inline `<passphrase>` form is kept for
 * the bench and for scripting, but warns that what was typed was visible --
 * because on a real console it was. The app's PhoneAPI path (phase 3) is the
 * other, and primary, way in. */
static int cmd_lockdown_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_lockdown_active()) {
		shell_print(sh, "lockdown: inactive (no passphrase provisioned; stock behaviour)");
		return 0;
	}
	shell_print(sh, "lockdown: active, %s (%s)",
		    meshtastic_lockdown_unlocked() ? "UNLOCKED" : "LOCKED",
		    meshtastic_lockdown_lock_reason());
	shell_print(sh, "  store: %s%s", meshtastic_lockdown_store_ready() ? "ready" : "not writable",
		    meshtastic_lockdown_busy() ? " (workqueue busy: reload or rewrite running)" : "");
	shell_print(sh, "  radio: %s", mt.radio_held ? "HELD (locked boot)" : "running");
	shell_print(sh, "  token: %u boots left, valid until epoch %u",
		    meshtastic_lockdown_boots_remaining(), meshtastic_lockdown_valid_until());
	if (meshtastic_lockdown_backoff_remaining() != 0U) {
		shell_print(sh, "  backoff: ~%u s", meshtastic_lockdown_backoff_remaining());
	}
	return 0;
}

/* What a lockdown verb's run function gets: the passphrase, and the token's boot count. */
struct shell_lockdown_work {
	uint8_t boots; /* 0: the default */
	uint8_t pass_len;
	uint8_t pass[MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX];
};

BUILD_ASSERT(sizeof(struct shell_lockdown_work) <= MTSH_WORK_ARG_MAX,
	     "a passphrase rides in a shell work item");

/* PBKDF2 (~1 s on an M4) plus NVS writes: never on the shell thread. */
static int shell_lockdown_provision_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_lockdown_work *lw = arg;
	int ret;

	ARG_UNUSED(len);
	ret = meshtastic_lockdown_provision(lw->pass, lw->pass_len, lw->boots, 0U, 0U);
	if (ret == 0) {
		shell_print(sh, "provisioned: sealing the store now; "
			    "%u boots on the token",
			    meshtastic_lockdown_boots_remaining());
	} else if (ret == -EALREADY) {
		shell_error(sh, "already provisioned (lockdown disable first)");
	}
	return ret;
}

static int shell_lockdown_unlock_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_lockdown_work *lw = arg;
	int ret;

	ARG_UNUSED(len);
	ret = meshtastic_lockdown_unlock(lw->pass, lw->pass_len, lw->boots, 0U, 0U);
	if (ret == 0) {
		shell_print(sh, "unlocked; the store reloads on the workqueue "
			    "and the radio follows");
	} else if (ret == -EACCES) {
		shell_error(sh, "wrong passphrase (next attempt after a "
			    "reboot and ~%u s)",
			    meshtastic_lockdown_backoff_remaining());
	} else if (ret == -EAGAIN) {
		shell_error(sh, "backoff: ~%u s (and a reboot) still to wait",
			    meshtastic_lockdown_backoff_remaining());
	}
	return ret;
}

static int shell_lockdown_disable_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_lockdown_work *lw = arg;
	int ret;

	ARG_UNUSED(len);
	ret = meshtastic_lockdown_disable(lw->pass, lw->pass_len);
	if (ret == 0) {
		shell_print(sh, "disabling: the store is rewritten in the clear "
			    "on the workqueue; reboot when `lockdown` says inactive");
	} else if (ret == -EACCES) {
		shell_error(sh, "wrong passphrase");
	}
	return ret;
}

static int lockdown_enqueue(const struct shell *sh, mtsh_work_fn run, const uint8_t *pass,
			    size_t pass_len, const char *boots_arg)
{
	struct shell_lockdown_work lw = {0};
	int ret;

	if (pass_len == 0U || pass_len > MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX) {
		shell_error(sh, "passphrase must be 1-%u bytes", MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX);
		return -EINVAL;
	}
	if (boots_arg != NULL) {
		uint32_t boots;

		ret = mtsh_parse_u32(sh, boots_arg, &boots);
		if (ret < 0 || boots > 255U) {
			shell_error(sh, "boots must be 0-255 (0 = default)");
			return -EINVAL;
		}
		lw.boots = (uint8_t)boots;
	}
	memcpy(lw.pass, pass, pass_len);
	lw.pass_len = (uint8_t)pass_len;
	ret = mtsh_enqueue(sh, run, &lw, sizeof(lw));
	memset(&lw, 0, sizeof(lw));
	return ret;
}

/* Ask for the passphrase with echoing off (the reference's obscured console
 * input). Returns the length, or <0; the buffer is the caller's to zero. On a
 * non-interactive backend (the dummy backend a test drives, no cmd context)
 * shell_readline refuses with -EACCES -- there is no way to prompt, so say so. */
static int read_passphrase(const struct shell *sh, uint8_t *buf, size_t cap)
{
	int ret;

	shell_obscure_set(sh, true);
	shell_readline_prompt_set(sh, "passphrase: ");
	ret = shell_readline(sh, buf, cap, K_SECONDS(60));
	shell_readline_prompt_set(sh, NULL);
	shell_obscure_set(sh, false);
	shell_print(sh, ""); /* the obscured Enter left the cursor mid-line */

	if (ret == -EACCES) {
		shell_error(sh, "no passphrase argument, and this console cannot prompt "
				"(pass it inline, or use the app)");
	} else if (ret == -ETIMEDOUT) {
		shell_error(sh, "timed out waiting for the passphrase");
	} else if (ret == -ENOBUFS) {
		shell_error(sh, "passphrase too long (max %u bytes)",
			    MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX);
	} else if (ret < 0) {
		shell_error(sh, "passphrase entry failed (%d)", ret);
	}
	return ret;
}

/* Gather the passphrase from argv[1] (inline, visible) or an obscured prompt,
 * then enqueue @p op. `pass_argc` is the index the passphrase would sit at. */
static int lockdown_pass_cmd(const struct shell *sh, size_t argc, char **argv,
			     mtsh_work_fn run, const char *boots_arg)
{
	uint8_t pass[MESHTASTIC_LOCKDOWN_PASSPHRASE_MAX + 1];
	int ret;

	if (argc >= 2) {
		shell_warn(sh, "the inline passphrase is echoed to the console and its log; "
			       "run with no argument for an obscured prompt");
		return lockdown_enqueue(sh, run, (const uint8_t *)argv[1], strlen(argv[1]), boots_arg);
	}
	ret = read_passphrase(sh, pass, sizeof(pass));
	if (ret < 0) {
		return ret;
	}
	ret = lockdown_enqueue(sh, run, pass, (size_t)ret, NULL);
	memset(pass, 0, sizeof(pass));
	return ret;
}

static int cmd_lockdown_provision(const struct shell *sh, size_t argc, char **argv)
{
	return lockdown_pass_cmd(sh, argc, argv, shell_lockdown_provision_run,
				 argc > 2 ? argv[2] : NULL);
}

static int cmd_lockdown_unlock(const struct shell *sh, size_t argc, char **argv)
{
	return lockdown_pass_cmd(sh, argc, argv, shell_lockdown_unlock_run,
				 argc > 2 ? argv[2] : NULL);
}

static int cmd_lockdown_disable(const struct shell *sh, size_t argc, char **argv)
{
	return lockdown_pass_cmd(sh, argc, argv, shell_lockdown_disable_run, NULL);
}

static int cmd_lockdown_lock(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!meshtastic_lockdown_active()) {
		shell_error(sh, "not provisioned");
		return -ENOENT;
	}
	meshtastic_lockdown_lock_now();
	shell_print(sh, "locked: token deleted, keys zeroed. The store in RAM stays until "
			"reboot; reboot to boot locked (radio silent)");
	return 0;
}

/* Deletes the token and writes NVS: on the work thread. */
MTSH_DEFINE_ON_WORK(cmd_lockdown_lock)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_lockdown_cmds,
			       SHELL_CMD_ARG(provision, NULL,
					     SHELL_HELP("Provision a passphrase and seal the store. "
							"Omit the passphrase for an obscured prompt.",
							"[passphrase [boots]]"),
					     cmd_lockdown_provision, 1, 2),
			       SHELL_CMD_ARG(unlock, NULL,
					     SHELL_HELP("Present the passphrase; issues a fresh token. "
							"Omit it for an obscured prompt.",
							"[passphrase [boots]]"),
					     cmd_lockdown_unlock, 1, 2),
			       SHELL_CMD(lock, NULL,
					 SHELL_HELP("Lock now: delete the token, zero the keys.", NULL),
					 cmd_lockdown_lock_on_work),
			       SHELL_CMD_ARG(disable, NULL,
					     SHELL_HELP("Verify the passphrase, rewrite the store in "
							"the clear, remove the artifacts. Omit the "
							"passphrase for an obscured prompt.",
							"[passphrase]"),
					     cmd_lockdown_disable, 1, 1),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), lockdown, &meshtastic_lockdown_cmds,
		 SHELL_HELP("Lockdown mode: status, provision, unlock, lock, disable.", NULL),
		 cmd_lockdown_status, 0, 0);
