/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic admin`: remote-admin trust and the admin client. Built with the shell and
 * CONFIG_MESHTASTIC_ADMIN (src/CMakeLists.txt); moved from meshtastic_shell.c
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
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic_admin.h"
#include "meshtastic_admin_client.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

enum admin_remote_op {
	ADMIN_REMOTE_GET_OWNER,
	ADMIN_REMOTE_GET_CONFIG,
	ADMIN_REMOTE_SET_TIME,
	ADMIN_REMOTE_SET_OWNER,
};

/* meshtastic admin trust [<node> [off]] — manage SecurityConfig.admin_key by
 * NodeDB lookup: "trust that node as my remote admin". The headless-bench
 * equivalent of the app's key-entry screen — the key comes from the target's
 * own NodeInfo broadcast, so trust-on-first-use applies: verify the node id
 * belongs to hardware you hold before trusting it.
 *
 * `admin trust key:<hex-prefix> off` removes an entry by its leading bytes
 * (>= 4, unique) instead: the form for a key whose node no longer exists in
 * the NodeDB — a re-keyed peer's old key (bench 2026-08-27: kit2 kept the
 * courier's stale `510dd592…` beside its real one, and nothing could name it). */
static int cmd_admin_trust(const struct shell *sh, size_t argc, char **argv)
{
	meshtastic_Config cfg;
	meshtastic_Config_SecurityConfig *sec;
	uint8_t key[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];
	uint32_t node_num = 0U;
	bool remove;
	bool by_prefix;
	size_t prefix_len = 0U;
	pb_size_t match;
	int ret;

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	ret = meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg);
	if (ret < 0 || cfg.which_payload_variant != meshtastic_Config_security_tag) {
		shell_error(sh, "security config unavailable (%d)", ret);
		return ret < 0 ? ret : -EIO;
	}
	sec = &cfg.payload_variant.security;

	if (argc == 1U) {
		shell_print(sh, "admin keys: %u/%u", (unsigned int)sec->admin_key_count,
			    (unsigned int)ARRAY_SIZE(sec->admin_key));
		for (pb_size_t i = 0; i < sec->admin_key_count; i++) {
			shell_print(sh, "  [%u] %02x%02x%02x%02x… (%u bytes)", (unsigned int)i,
				    sec->admin_key[i].bytes[0], sec->admin_key[i].bytes[1],
				    sec->admin_key[i].bytes[2], sec->admin_key[i].bytes[3],
				    (unsigned int)sec->admin_key[i].size);
		}
		return 0;
	}

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
	shell_error(sh, "config writes are compiled out of this build");
	ARG_UNUSED(remove);
	ARG_UNUSED(by_prefix);
	ARG_UNUSED(prefix_len);
	ARG_UNUSED(match);
	ARG_UNUSED(node_num);
	ARG_UNUSED(key);
	return -ENOTSUP;
#else
	if (mtsh_config_write_refused(sh)) {
		return -EACCES;
	}

	remove = (argc >= 3U && strcmp(argv[2], "off") == 0);
	by_prefix = (strncmp(argv[1], "key:", 4U) == 0);

	if (by_prefix) {
		const char *hex = argv[1] + 4;
		size_t n = strlen(hex);

		if (!remove) {
			shell_error(sh, "key:<prefix> only removes (`off`); trust a node by its id");
			return -EINVAL;
		}
		if (n < 8U || (n & 1U) != 0U || n / 2U > sizeof(key) ||
		    hex2bin(hex, n, key, sizeof(key)) != n / 2U) {
			shell_error(sh, "key:<prefix> needs 4..%u bytes of hex",
				    (unsigned int)sizeof(key));
			return -EINVAL;
		}
		prefix_len = n / 2U;
	} else {
		ret = mtsh_parse_u32(sh, argv[1], &node_num);
		if (ret < 0) {
			return ret;
		}
		if (meshtastic_nodedb_copy_pubkey(node_num, key) != 0) {
			shell_error(sh, "no public key for 0x%08x in the NodeDB (heard its NodeInfo?)",
				    node_num);
			return -ENOENT;
		}
		prefix_len = sizeof(key);
	}

	match = sec->admin_key_count;
	for (pb_size_t i = 0; i < sec->admin_key_count; i++) {
		if (sec->admin_key[i].size == sizeof(key) &&
		    memcmp(sec->admin_key[i].bytes, key, prefix_len) == 0) {
			if (match != sec->admin_key_count) {
				shell_error(sh, "key:%s matches more than one entry — give more bytes",
					    argv[1] + 4);
				return -EINVAL;
			}
			match = i;
			if (!by_prefix) {
				break;
			}
		}
	}

	if (remove) {
		if (match == sec->admin_key_count) {
			if (by_prefix) {
				shell_print(sh, "no admin key starts with %s", argv[1] + 4);
			} else {
				shell_print(sh, "0x%08x was not trusted", node_num);
			}
			return 0;
		}
		for (pb_size_t i = match; i + 1U < sec->admin_key_count; i++) {
			sec->admin_key[i] = sec->admin_key[i + 1U];
		}
		sec->admin_key_count--;
	} else {
		if (match != sec->admin_key_count) {
			shell_print(sh, "0x%08x already trusted", node_num);
			return 0;
		}
		if (sec->admin_key_count >= ARRAY_SIZE(sec->admin_key)) {
			shell_error(sh, "admin key list full (%u)",
				    (unsigned int)ARRAY_SIZE(sec->admin_key));
			return -ENOSPC;
		}
		sec->admin_key[sec->admin_key_count].size = sizeof(key);
		memcpy(sec->admin_key[sec->admin_key_count].bytes, key, sizeof(key));
		sec->admin_key_count++;
	}

	ret = meshtastic_config_store_set_config(&cfg);
	if (ret < 0) {
		shell_error(sh, "config write failed (%d)", ret);
		return ret;
	}
	if (by_prefix) {
		shell_print(sh, "key %s… removed (%u key%s)", argv[1] + 4,
			    (unsigned int)sec->admin_key_count,
			    sec->admin_key_count == 1U ? "" : "s");
	} else {
		shell_print(sh, "0x%08x %s as remote admin (%u key%s)", node_num,
			    remove ? "untrusted" : "trusted", (unsigned int)sec->admin_key_count,
			    sec->admin_key_count == 1U ? "" : "s");
	}
	return 0;
#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */
}
MTSH_DEFINE_ON_WORK(cmd_admin_trust)

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
/* meshtastic admin remote <node> get owner
 *                        <node> get config <type 0-8>
 *                        <node> set-owner <long> [short]
 *                        <node> set-time [<unix-epoch-seconds>]  (default: ours)
 * Responses arrive asynchronously on the RX path and print as `admin client:`
 * log lines; a getter also caches the target's session passkey for the
 * mutating op that follows. */
/* `admin remote`: payload[0] is an enum admin_remote_op, the rest its argument. */
struct shell_admin_remote_work {
	uint32_t dest;
	uint8_t payload[MESHTASTIC_MAX_PAYLOAD_LEN];
};

BUILD_ASSERT(sizeof(struct shell_admin_remote_work) <= MTSH_WORK_ARG_MAX,
	     "an admin request rides in a shell work item");

/* Run inline it reset kit1 from the shell thread's stack (bench 2026-10-10). */
static int shell_admin_remote_run(const struct shell *sh, const void *arg, size_t len)
{
	const struct shell_admin_remote_work *aw = arg;
	uint32_t epoch;
	int ret;

	ARG_UNUSED(len);
	switch (aw->payload[0]) {
	case ADMIN_REMOTE_GET_OWNER:
		ret = meshtastic_admin_client_get_owner(aw->dest);
		break;
	case ADMIN_REMOTE_GET_CONFIG:
		ret = meshtastic_admin_client_get_config(aw->dest, aw->payload[1]);
		break;
	case ADMIN_REMOTE_SET_TIME:
		memcpy(&epoch, &aw->payload[1], sizeof(epoch));
		ret = meshtastic_admin_client_set_time(aw->dest, epoch);
		break;
	case ADMIN_REMOTE_SET_OWNER: {
		const char *long_name = (const char *)&aw->payload[2];
		const char *short_name =
			aw->payload[1] != 0U ? long_name + strlen(long_name) + 1 : NULL;

		ret = meshtastic_admin_client_set_owner(aw->dest, long_name, short_name);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}
	if (ret == -EACCES) {
		shell_error(sh, "request failed (%d) -- no public key / passkey", ret);
	} else if (ret == 0) {
		shell_print(sh, "request sent -- watch for `admin client:` log lines");
	}
	return ret;
}

static int cmd_admin_remote(const struct shell *sh, size_t argc, char **argv)
{
	struct shell_admin_remote_work *aw;
	struct mtsh_work *w;
	uint32_t dest;
	int ret;

	if (argc < 3U) {
		goto usage;
	}
	ret = mtsh_parse_u32(sh, argv[1], &dest);
	if (ret < 0) {
		return ret;
	}
	/* The request is built in the queued item itself, not on this stack. */
	w = mtsh_work_alloc(sh, shell_admin_remote_run);
	if (w == NULL) {
		return -ENOMSG;
	}
	aw = (struct shell_admin_remote_work *)w->arg;
	aw->dest = dest;

	/* Everything below only parses and checks. The send itself runs on the work queue:
	 * it is the whole send path, which the XIAO shell stack cannot carry. */
	if (strcmp(argv[2], "get") == 0 && argc >= 4U) {
		if (strcmp(argv[3], "owner") == 0) {
			aw->payload[0] = ADMIN_REMOTE_GET_OWNER;
		} else if (strcmp(argv[3], "config") == 0 && argc >= 5U) {
			uint32_t type;

			ret = mtsh_parse_u32(sh, argv[4], &type);
			if (ret < 0) {
				mtsh_work_cancel(w);
				return ret;
			}
			aw->payload[0] = ADMIN_REMOTE_GET_CONFIG;
			aw->payload[1] = (uint8_t)type;
			if (type > UINT8_MAX) {
				mtsh_work_cancel(w);
				goto usage;
			}
		} else {
			mtsh_work_cancel(w);
			goto usage;
		}
	} else if (strcmp(argv[2], "set-time") == 0) {
		/*
		 * Relay a wall clock to a peer (agents-xhli.14). With no explicit
		 * epoch, relay OUR OWN -- which is the whole point: a node that has a
		 * time source hands one to a node that does not.
		 *
		 * Refusing when our own clock is unset is not a nicety. An unset
		 * clock reads epoch 0, which the target's sanity window rejects, so
		 * the request would look sent and do nothing; and a node that cannot
		 * say when anything happened has no business being a time source.
		 */
		uint32_t epoch;

		if (argc >= 4U) {
			ret = mtsh_parse_u32(sh, argv[3], &epoch);
			if (ret < 0) {
				mtsh_work_cancel(w);
				return ret;
			}
		} else {
			epoch = meshtastic_clock_now_epoch();
			if (epoch == 0U) {
				shell_error(sh, "refused: this node's own clock is unset, so "
						"it cannot be a time source (set it first, or "
						"pass an explicit epoch)");
				mtsh_work_cancel(w);
				return -EAGAIN;
			}
		}

		if (!meshtastic_admin_client_have_passkey(aw->dest)) {
			shell_error(sh, "no fresh session passkey for 0x%08x -- run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    aw->dest, aw->dest);
			mtsh_work_cancel(w);
			return -EACCES;
		}
		aw->payload[0] = ADMIN_REMOTE_SET_TIME;
		memcpy(&aw->payload[1], &epoch, sizeof(epoch));
	} else if (strcmp(argv[2], "set-owner") == 0 && argc >= 4U) {
		size_t long_len = strlen(argv[3]);
		size_t short_len = argc >= 5U ? strlen(argv[4]) : 0U;

		if (!meshtastic_admin_client_have_passkey(aw->dest)) {
			shell_error(sh, "no fresh session passkey for 0x%08x -- run "
				    "`meshtastic admin remote 0x%08x get owner` first",
				    aw->dest, aw->dest);
			mtsh_work_cancel(w);
			return -EACCES;
		}
		if (2U + long_len + 1U + short_len + 1U > sizeof(aw->payload)) {
			shell_error(sh, "name too long");
			mtsh_work_cancel(w);
			return -EINVAL;
		}
		aw->payload[0] = ADMIN_REMOTE_SET_OWNER;
		aw->payload[1] = argc >= 5U ? 1U : 0U;
		memcpy(&aw->payload[2], argv[3], long_len + 1U);
		if (argc >= 5U) {
			memcpy(&aw->payload[2U + long_len + 1U], argv[4], short_len + 1U);
		}
	} else {
		mtsh_work_cancel(w);
		goto usage;
	}

	return mtsh_work_submit(w, sizeof(*aw));

usage:
	shell_error(sh, "usage: meshtastic admin remote <node> get owner\n"
		    "       meshtastic admin remote <node> get config <type 0-8>\n"
		    "       meshtastic admin remote <node> set-owner <long> [short]\n"
		    "       meshtastic admin remote <node> set-time [<unix-epoch-seconds>]");
	return -EINVAL;
}
MTSH_DEFINE_ON_WORK(cmd_admin_remote)
#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_admin_cmds,
			       SHELL_CMD(trust, NULL,
					 SHELL_HELP("List/manage trusted remote-admin keys.",
						    "[<node> [off] | key:<hex-prefix> off]"),
					 cmd_admin_trust_on_work),
#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
			       SHELL_CMD(remote, NULL,
					 SHELL_HELP("Administer a peer node (PKC, any bearer).",
						    "<node> get owner | get config <t> | "
						    "set-owner <long> [short] | "
						    "set-time [<epoch>]"),
					 cmd_admin_remote_on_work),
#endif
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), admin, &meshtastic_admin_cmds,
		 SHELL_HELP("Remote-admin trust and client commands.", NULL),
		 NULL, 0, 0);
