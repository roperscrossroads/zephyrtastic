/* SPDX-License-Identifier: GPL-3.0
 *
 * Storage report — see meshtastic_storage.h.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SETTINGS_NVS)
#include <zephyr/drivers/flash.h>
#include <zephyr/kvss/nvs.h>
#endif
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif

#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
#include "meshtastic_lockdown.h"
#endif
#include "meshtastic_storage.h"
#if defined(CONFIG_MESHTASTIC_BULK_STORE)
#include "meshtastic_bulk.h"
#endif

/* NVS stores every settings record as TWO entries (the name under one id, the
 * value under id + 0x4000, settings_nvs.c), and every entry costs an 8-byte
 * allocation-table entry plus its data rounded up to the write block. */
#define NVS_ATE_BYTES 8U

static uint32_t load_us[MESHTASTIC_STORAGE_LOAD_COUNT];

void meshtastic_storage_note_load(enum meshtastic_storage_load what, uint32_t cycles)
{
	if ((unsigned int)what < ARRAY_SIZE(load_us)) {
		load_us[what] = (uint32_t)k_cyc_to_us_floor64(cycles);
	}
}

struct walk_ctx {
	struct meshtastic_storage_stats *out;
	uint32_t align;
};

static uint32_t aligned(uint32_t n, uint32_t align)
{
	return (align > 1U) ? ROUND_UP(n, align) : n;
}

static struct meshtastic_storage_subtree *subtree_for(struct meshtastic_storage_stats *out,
						       const char *key)
{
	const char *slash = strchr(key, '/');
	size_t len = (slash != NULL) ? (size_t)(slash - key) : strlen(key);

	len = MIN(len, (size_t)MESHTASTIC_STORAGE_NAME_LEN - 1U);
	for (uint8_t i = 0U; i < out->subtree_count; i++) {
		if (strncmp(out->subtree[i].name, key, len) == 0 &&
		    out->subtree[i].name[len] == '\0') {
			return &out->subtree[i];
		}
	}
	if (out->subtree_count >= ARRAY_SIZE(out->subtree)) {
		out->subtrees_truncated = true;
		return NULL;
	}
	memcpy(out->subtree[out->subtree_count].name, key, len);
	out->subtree[out->subtree_count].name[len] = '\0';
	return &out->subtree[out->subtree_count++];
}

static int walk_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		   void *param)
{
	struct walk_ctx *ctx = param;
	struct meshtastic_storage_subtree *st;
	uint32_t name_len = (uint32_t)strlen(key);
	uint32_t cost = 2U * NVS_ATE_BYTES + aligned(name_len, ctx->align) +
			aligned((uint32_t)len, ctx->align);

	ctx->out->total_records++;
	ctx->out->total_flash_bytes += cost;

	st = subtree_for(ctx->out, key);
	if (st == NULL) {
		return 0;
	}
	st->records++;
	st->name_bytes += name_len;
	st->value_bytes += (uint32_t)len;
	st->flash_bytes += cost;

#if defined(CONFIG_MESHTASTIC_LOCKDOWN)
	{
		uint8_t head[4];

		/* The seal magic is the value's first word; only the length check
		 * needs the full size, which the walk already knows. */
		if (len >= sizeof(head) && read_cb(cb_arg, head, sizeof(head)) == sizeof(head) &&
		    meshtastic_lockdown_is_sealed(head, len)) {
			st->sealed++;
		}
	}
#else
	ARG_UNUSED(read_cb);
	ARG_UNUSED(cb_arg);
#endif
	return 0;
}

int meshtastic_storage_stats(struct meshtastic_storage_stats *out)
{
	struct walk_ctx ctx = { .out = out, .align = 4U };
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	memcpy(out->load_us, load_us, sizeof(load_us));

	ret = settings_subsys_init(); /* idempotent */
	if (ret != 0) {
		return ret;
	}

#if defined(CONFIG_SETTINGS_NVS)
	{
		void *storage = NULL;
		struct nvs_fs *fs;

		ret = settings_storage_get(&storage);
		if (ret == 0 && storage != NULL) {
			fs = storage;
			out->nvs = true;
			out->sector_count = fs->sector_count;
			out->sector_size = fs->sector_size;
			out->write_sector = (uint16_t)(fs->ate_wra >> 16);
			out->free_bytes = (int32_t)nvs_calc_free_space(fs);
			if (fs->flash_parameters != NULL &&
			    fs->flash_parameters->write_block_size > 0U) {
				ctx.align = (uint32_t)fs->flash_parameters->write_block_size;
			}
		}
	}
#endif

	/* A NULL subtree walks every key in the store. */
	return settings_load_subtree_direct(NULL, walk_cb, &ctx);
}

#if defined(CONFIG_SHELL)
static const char *const load_names[MESHTASTIC_STORAGE_LOAD_COUNT] = {
	[MESHTASTIC_STORAGE_LOAD_CONFIG] = "config",
	[MESHTASTIC_STORAGE_LOAD_NODE_KEYS] = "node keys",
	[MESHTASTIC_STORAGE_LOAD_NODE_RECS] = "node records",
};

/* Static: the stats struct is ~500 B, too much for the shell thread's stack. */
static struct meshtastic_storage_stats shell_stats;

int meshtastic_storage_shell_cmd(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_storage_stats *st = &shell_stats;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = meshtastic_storage_stats(st);
	if (ret != 0) {
		shell_error(sh, "storage stats failed (%d)", ret);
		return ret;
	}

	if (st->nvs) {
		uint32_t total = (uint32_t)st->sector_count * st->sector_size;

		shell_print(sh, "settings NVS: %u sectors x %u B = %u B, writing sector %u",
			    st->sector_count, st->sector_size, total, st->write_sector);
		if (st->free_bytes >= 0) {
			/* One sector is always held empty for garbage collection. */
			uint32_t usable = (st->sector_count > 0U)
						  ? (uint32_t)(st->sector_count - 1U) * st->sector_size
						  : 0U;

			shell_print(sh, "free: %d B of ~%u B usable (%u%% used)", st->free_bytes,
				    usable,
				    usable ? (unsigned int)(100U - ((uint32_t)st->free_bytes * 100U /
								    usable))
					   : 0U);
		} else {
			shell_print(sh, "free: unknown (%d)", st->free_bytes);
		}
	} else {
		shell_print(sh, "settings backend is not NVS: no sector or free-space figures");
	}

	shell_print(sh, "records: %u, ~%u B on flash (estimate: name + value + 2 ATEs each)",
		    st->total_records, st->total_flash_bytes);
	shell_print(sh, "%-12s %7s %6s %9s %9s", "subtree", "records", "sealed", "value B",
		    "flash B");
	for (uint8_t i = 0U; i < st->subtree_count; i++) {
		const struct meshtastic_storage_subtree *s = &st->subtree[i];

		shell_print(sh, "%-12s %7u %6u %9u %9u", s->name, s->records, s->sealed,
			    s->value_bytes, s->flash_bytes);
	}
	if (st->subtrees_truncated) {
		shell_warn(sh, "(more subtrees than the table holds; totals above are complete)");
	}

#if defined(CONFIG_MESHTASTIC_BULK_STORE)
	{
		struct meshtastic_bulk_info bi;

		meshtastic_bulk_info_get(&bi);
		if (bi.ready) {
			shell_print(sh, "bulk store: %u sectors x %u B, %d B free, %u format(s) this boot",
				    bi.sector_count, bi.sector_size, bi.free_bytes, bi.formats);
		} else {
			shell_print(sh, "bulk store: not mounted (node records stay in settings)");
		}
	}
#endif
	shell_print(sh, "boot restore:");
	for (size_t i = 0U; i < ARRAY_SIZE(load_names); i++) {
		if (st->load_us[i] == 0U) {
			shell_print(sh, "  %-13s not measured", load_names[i]);
		} else {
			shell_print(sh, "  %-13s %u us", load_names[i], st->load_us[i]);
		}
	}
	return 0;
}
#endif /* CONFIG_SHELL */
