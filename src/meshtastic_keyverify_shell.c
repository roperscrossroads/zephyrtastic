/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic keyverify` (the manual key-verification handshake) and `meshtastic xeddsa`
 * (packet signature verification). Built with the shell and
 * CONFIG_MESHTASTIC_KEYVERIFY (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_core.h"
#include "meshtastic_keyverify.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_shell_internal.h"
#include "meshtastic_xeddsa.h"

/* `meshtastic keyverify` (agents-dnr4.13): the bench's stand-in for the app's
 * key-verification prompts. */
static const char *kv_state_name(enum meshtastic_keyverify_state s)
{
	switch (s) {
	case MESHTASTIC_KEYVERIFY_IDLE: return "idle";
	case MESHTASTIC_KEYVERIFY_SENDER_HAS_INITIATED: return "sent request, awaiting reply";
	case MESHTASTIC_KEYVERIFY_SENDER_AWAITING_NUMBER: return "awaiting the peer's security number";
	case MESHTASTIC_KEYVERIFY_SENDER_AWAITING_USER: return "awaiting your accept (compare codes)";
	case MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_USER: return "awaiting your accept (compare codes)";
	case MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1: return "showing security number, awaiting the peer";
	default: return "?";
	}
}

#if defined(CONFIG_MESHTASTIC_XEDDSA)
/* `meshtastic xeddsa` -- the only way to see signature verification working on a bench node:
 * the verify path's success line is LOG_DBG and the bench images compile logging at inf. */
static int cmd_xeddsa_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_xeddsa_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_xeddsa_get_stats(&st);
	shell_print(sh, "xeddsa: verify %s, sign %s",
		    IS_ENABLED(CONFIG_MESHTASTIC_XEDDSA) ? "on" : "off",
		    IS_ENABLED(CONFIG_MESHTASTIC_XEDDSA_SIGN) ? "on" : "off");
	shell_print(sh, "  rx verified   : %u  (of which first-contact NodeInfo: %u)",
		    st.verified, st.bootstrapped);
	shell_print(sh, "  rx DROPPED    : %u bad signature, %u malformed", st.failed,
		    st.malformed);
	shell_print(sh, "  rx unverifiable: %u (signed, no key for the sender)", st.no_key);
	shell_print(sh, "  rx unsigned   : %u accepted, %u dropped (strict), %u dropped (balanced)",
		    st.unsigned_ok, st.unsigned_drop, st.balanced_drop);
	shell_print(sh, "  tx            : %u signed, %u unsigned (would not fit)", st.signed_tx,
		    st.sign_skipped);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_xeddsa_show)

static int cmd_xeddsa_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_xeddsa_reset_stats();
	shell_print(sh, "xeddsa counters cleared");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_xeddsa_reset)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_xeddsa_cmds,
			       SHELL_CMD(reset, NULL, SHELL_HELP("Zero the counters.", NULL),
					 cmd_xeddsa_reset_on_work),
			       SHELL_SUBCMD_SET_END);
#endif /* CONFIG_MESHTASTIC_XEDDSA */

static int cmd_keyverify_show(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_keyverify_status st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_keyverify_status(&st);
	shell_print(sh, "keyverify: %s", kv_state_name(st.state));
	if (st.state != MESHTASTIC_KEYVERIFY_IDLE) {
		shell_print(sh, "  peer 0x%08x nonce %llu", st.remote_node, (unsigned long long)st.nonce);
	}
	if (st.state == MESHTASTIC_KEYVERIFY_RECEIVER_AWAITING_HASH1) {
		shell_print(sh, "  SECURITY NUMBER to read out to the peer: %03u %03u",
			    st.security_number / 1000U, st.security_number % 1000U);
	}
	if (st.code[0] != '\0') {
		shell_print(sh, "  verification code: %s (must match the peer's)", st.code);
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_show)

/* PSA RNG, protobuf encode, wire build with the PKC encrypt: the shell thread's own
 * stack cannot carry it. On the XIAO (CONFIG_SHELL_STACK_SIZE=2048) `keyverify start`
 * locked the CPU up -- reset cause 0x100, no fatal record, because the overflow lands
 * in the MPU guard and the exception entry cannot stack. Bench 2026-09-09. */
static int shell_keyverify_start_run(const struct shell *sh, const void *arg, size_t len)
{
	int ret = meshtastic_keyverify_start(mtsh_work_dest(arg, len));

	if (ret == -EBUSY) {
		shell_error(sh, "a session is already open (keyverify reject "
			    "to drop it)");
	} else if (ret == 0) {
		shell_print(sh, "request sent; ask the peer for the security "
			    "number it shows, then `meshtastic keyverify number <n>`");
	}
	return ret;
}

static int shell_keyverify_number_run(const struct shell *sh, const void *arg, size_t len)
{
	struct meshtastic_keyverify_status st;
	int ret;

	meshtastic_keyverify_status(&st);
	ret = meshtastic_keyverify_provide_number(st.nonce, mtsh_work_dest(arg, len));
	if (ret == -EACCES) {
		shell_error(sh, "that number does not match the peer's reply: "
			    "retype it, or suspect a man in the middle");
	} else if (ret < 0) {
		shell_error(sh, "not expecting a number now (%d)", ret);
	} else {
		meshtastic_keyverify_status(&st);
		shell_print(sh, "verification code: %s -- if the peer shows "
			    "the same, `meshtastic keyverify accept`", st.code);
	}
	return ret;
}

/* Ends with a NodeInfo send: the same path as `start`. */
static int shell_keyverify_accept_run(const struct shell *sh, const void *arg, size_t len)
{
	struct meshtastic_keyverify_status st;
	int ret;

	ARG_UNUSED(arg);
	ARG_UNUSED(len);
	meshtastic_keyverify_status(&st);
	ret = meshtastic_keyverify_accept(st.nonce);
	if (ret < 0) {
		shell_error(sh, "nothing to accept (%d)", ret);
	} else {
		shell_print(sh, "peer 0x%08x marked key-verified",
			    st.remote_node);
	}
	return ret;
}

static int cmd_keyverify_start(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dest;
	unsigned long node;
	char *end;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic keyverify start <node-hex>");
		return -EINVAL;
	}
	node = strtoul(argv[1], &end, 16);
	if (*end != '\0' || node == 0UL || node > UINT32_MAX) {
		shell_error(sh, "invalid node id: %s", argv[1]);
		return -EINVAL;
	}
	/* Deferred to the shell work thread: the send path does not fit the shell
	 * thread's stack (see shell_keyverify_start_run). */
	dest = (uint32_t)node;
	ret = mtsh_enqueue(sh, shell_keyverify_start_run, &dest, sizeof(dest));
	return ret;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_start)

static int cmd_keyverify_number(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t number;
	unsigned long n;
	char *end;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic keyverify number <6 digits>");
		return -EINVAL;
	}
	n = strtoul(argv[1], &end, 10);
	if (*end != '\0' || n == 0UL || n > 999999UL) {
		shell_error(sh, "invalid security number: %s", argv[1]);
		return -EINVAL;
	}
	/* Deferred like `start`: this sends M3 and hashes on the way. */
	number = (uint32_t)n;
	return mtsh_enqueue(sh, shell_keyverify_number_run, &number, sizeof(number));
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_number)

static int cmd_keyverify_accept(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Deferred: accept ends by sending our NodeInfo to the peer (the send
	 * path again). Bench 2026-09-09: inline, it locked kit2 up like `start`. */
	return mtsh_enqueue(sh, shell_keyverify_accept_run, NULL, 0U);
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_accept)

static int cmd_keyverify_reject(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_keyverify_reject();
	shell_print(sh, "session dropped; nothing learned was kept");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_keyverify_reject)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_keyverify_cmds,
			       SHELL_CMD_ARG(start, NULL,
					     SHELL_HELP("Start verifying a peer's key.",
							"<node-hex>"),
					     cmd_keyverify_start_on_work, 2, 0),
			       SHELL_CMD_ARG(number, NULL,
					     SHELL_HELP("Enter the security number the peer shows.",
							"<6 digits>"),
					     cmd_keyverify_number_on_work, 2, 0),
			       SHELL_CMD(accept, NULL,
					 SHELL_HELP("The codes match: commit the key as verified.",
						    NULL),
					 cmd_keyverify_accept_on_work),
			       SHELL_CMD(reject, NULL, SHELL_HELP("Drop the session.", NULL),
					 cmd_keyverify_reject_on_work),
			       SHELL_SUBCMD_SET_END);

#if defined(CONFIG_MESHTASTIC_XEDDSA)
SHELL_SUBCMD_ADD((meshtastic), xeddsa, &meshtastic_xeddsa_cmds,
		 SHELL_HELP("Packet signature verification: counters, reset.", NULL),
		 cmd_xeddsa_show_on_work, 0, 0);
#endif
SHELL_SUBCMD_ADD((meshtastic), keyverify, &meshtastic_keyverify_cmds,
		 SHELL_HELP("Manual key verification (security-number handshake): status, "
		 "start, number, accept, reject.",
		 NULL),
		 cmd_keyverify_show_on_work, 0, 0);


