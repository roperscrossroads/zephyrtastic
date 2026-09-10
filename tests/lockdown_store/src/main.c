/* SPDX-License-Identifier: GPL-3.0
 *
 * Lockdown phase 2 (agents-dnr4.15): the record wrap over the settings store,
 * with the whole stack up on the sim radio and the real NVS behind the settings
 * subsystem.
 *
 * A "locked boot" cannot be a process restart here, so it is staged the way the
 * firmware experiences it: the RAM store is put back to its seed, the lockdown
 * module is re-initialised from the artifacts on flash (which is what a cold
 * boot does), and the subtree is loaded. What the load reads back -- the real
 * records or the defaults -- is the assertion.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <psa/crypto.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_pki.h"

/* meshtastic_router.h drags the wire header in; the injector is all this needs. */
void meshtastic_handle_inbound_packet(const struct meshtastic_packet *packet, const uint8_t *wire,
				      size_t wire_len, bool decoded);

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const uint8_t PP[] = "correct horse";
static const uint8_t WRONG[] = "battery staple";
#define PPLEN (sizeof(PP) - 1U)

#define TEST_NODE_ID 0x11223344U
#define PEER_ID      0x55667788U

static struct meshtastic_config cfg = {
	.node_id = TEST_NODE_ID,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "stock name",
	.short_name = "stck",
};

/* Raw record access, to see what actually sits in NVS. */
struct raw {
	uint8_t buf[320];
	size_t len;
	bool found;
};

static int raw_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param)
{
	struct raw *r = param;

	ARG_UNUSED(key);
	if (len <= sizeof(r->buf) && read_cb(cb_arg, r->buf, len) == (ssize_t)len) {
		r->len = len;
		r->found = true;
	}
	return 1;
}

static bool raw_read(const char *name, struct raw *r)
{
	r->len = 0U;
	r->found = false;
	(void)settings_load_subtree_direct(name, raw_cb, r);
	return r->found;
}

static void wait_idle(void)
{
	for (int i = 0; i < 200 && meshtastic_lockdown_busy(); i++) {
		k_sleep(K_MSEC(10));
	}
	zassert_false(meshtastic_lockdown_busy(), "the workqueue item did not run");
}

static void set_long_name(const char *name)
{
	meshtastic_User user = meshtastic_User_init_zero;

	(void)snprintk(user.long_name, sizeof(user.long_name), "%s", name);
	(void)snprintk(user.short_name, sizeof(user.short_name), "stck");
	zassert_ok(meshtastic_config_store_set_owner(&user), "");
}

static const char *long_name(void)
{
	return meshtastic_config_store_long_name();
}

/* A peer announces itself with its key: the way the NodeDB learns one (the
 * admin_pki suite's seeding, verbatim in spirit). */
static void seed_peer(uint32_t node, const uint8_t key[32])
{
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = node,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	(void)snprintk(user.long_name, sizeof(user.long_name), "peer");
	(void)snprintk(user.short_name, sizeof(user.short_name), "peer");
	user.public_key.size = 32U;
	memcpy(user.public_key.bytes, key, 32U);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

static void flush_all(void)
{
	zassert_ok(settings_save_subtree("meshtastic"), "");
	(void)settings_save_subtree("mtnode");
	(void)settings_save_subtree("mtrec");
}

/* A cold boot's view: the RAM store back to its seed, lockdown re-read from the
 * artifacts, the subtrees loaded. */
static void stage_boot(void)
{
	zassert_ok(meshtastic_config_store_seed(&cfg), "");
	meshtastic_lockdown_init();
	mt.radio_held = meshtastic_lockdown_locked();
	if (mt.radio_held) {
		/* A real locked boot never arms RX; this staged one must put the
		 * sim radio where that boot would leave it. */
		(void)lora_recv_async(lora_dev, NULL, NULL);
		mt.radio_rx_armed = false;
	}
	zassert_ok(settings_load_subtree("meshtastic"), "");
	(void)settings_load_subtree("mtnode");
	(void)settings_load_subtree("mtrec");
	(void)meshtastic_config_store_apply_core();
}

static void *suite_setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa");
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	return NULL;
}

static void suite_before(void *f)
{
	ARG_UNUSED(f);
	/* Every test starts stock, on a store that says "stock name". */
	wait_idle();
	if (meshtastic_lockdown_active()) {
		if (!meshtastic_lockdown_unlocked()) {
			(void)meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U);
			wait_idle();
		}
		zassert_ok(meshtastic_lockdown_disable(PP, PPLEN), "");
		wait_idle();
	}
	zassert_false(meshtastic_lockdown_active(), "");
	mt.radio_held = false;
	set_long_name("stock name");
	flush_all();
}

ZTEST_SUITE(lockdown_store, NULL, suite_setup, suite_before, NULL, NULL);

ZTEST(lockdown_store, test_inactive_records_are_plaintext_and_the_radio_runs)
{
	struct raw r;

	zassert_true(raw_read("meshtastic/owner", &r), "owner record present");
	zassert_false(meshtastic_lockdown_is_sealed(r.buf, r.len), "plaintext when inactive");
	zassert_true(meshtastic_lockdown_store_ready(), "");
	zassert_false(mt.radio_held, "");
	stage_boot();
	zassert_str_equal(long_name(), "stock name", "a stock boot reads its records");
}

ZTEST(lockdown_store, test_provision_seals_every_record_already_on_flash)
{
	struct raw r;

	set_long_name("sealed name");
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle(); /* the eager seal-all */

	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "owner sealed on flash");
	zassert_true(raw_read("meshtastic/config/lora", &r), "");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "lora sealed on flash");

	/* Unlocked: a load opens them. */
	stage_boot();
	zassert_true(meshtastic_lockdown_unlocked(), "token unlocks the boot");
	zassert_str_equal(long_name(), "sealed name", "sealed records open on an unlocked boot");
	zassert_false(mt.radio_held, "");
}

ZTEST(lockdown_store, test_locked_boot_keeps_defaults_holds_the_radio_and_refuses_writes)
{
	struct raw before, after;

	set_long_name("sealed name");
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	zassert_true(raw_read("meshtastic/owner", &before), "");

	/* Lock, then boot: the token is gone, so the loads keep the seed. */
	meshtastic_lockdown_lock_now();
	stage_boot();
	zassert_true(meshtastic_lockdown_locked(), "");
	zassert_true(mt.radio_held, "a locked boot holds the radio");
	zassert_str_equal(long_name(), "stock name", "placeholders, not the sealed records");
	zassert_false(meshtastic_lockdown_store_ready(), "");

	/* Nothing may be written over the sealed store while locked: a save
	 * refuses, and the record on flash is untouched. */
	set_long_name("placeholder edit");
	zassert_equal(settings_save_subtree("meshtastic"), -EACCES, "save refused while locked");
	zassert_true(raw_read("meshtastic/owner", &after), "");
	zassert_equal(after.len, before.len, "");
	zassert_mem_equal(after.buf, before.buf, before.len, "the sealed record survived");

	/* TX refused while held: the frame may queue, but nothing reaches the air. */
	{
		struct meshtastic_packet pkt = {
			.to = MESHTASTIC_NODE_BROADCAST,
			.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
			.payload = (const uint8_t *)"hi",
			.payload_len = 2U,
		};
		uint32_t tx_before = mt.status.tx_packets;

		(void)meshtastic_send_packet(&pkt, K_NO_WAIT);
		k_sleep(K_MSEC(500));
		zassert_equal(mt.status.tx_packets, tx_before, "no transmit while the radio is held");
		zassert_false(mt.radio_rx_armed, "no RX while the radio is held");
	}
}

ZTEST(lockdown_store, test_unlock_reloads_the_real_records_and_releases_the_radio)
{
	set_long_name("sealed name");
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_lockdown_lock_now();
	stage_boot();
	zassert_true(mt.radio_held, "");
	zassert_str_equal(long_name(), "stock name", "");

	zassert_equal(meshtastic_lockdown_unlock(WRONG, sizeof(WRONG) - 1U, 0U, 0U, 0U), -EACCES,
		      "wrong passphrase");
	/* The reboot floor: a wrong attempt costs a boot before the next one.
	 * Stage that boot (the token is still gone, so it is still a locked boot). */
	stage_boot();
	zassert_true(mt.radio_held, "");

	zassert_ok(meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U), "right passphrase");
	zassert_true(meshtastic_lockdown_unlocked(), "");
	zassert_false(meshtastic_lockdown_store_ready(),
		      "unlocked but not reloaded: still not writable");
	wait_idle(); /* the deferred reload on the workqueue */
	zassert_true(meshtastic_lockdown_store_ready(), "");
	zassert_str_equal(long_name(), "sealed name", "the reload brought the real config back");
	zassert_false(mt.radio_held, "the reload released the radio");
	zassert_true(mt.radio_rx_armed, "and armed RX");
}

ZTEST(lockdown_store, test_plaintext_leftover_is_accepted_and_sealed_at_the_next_save)
{
	struct raw r;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	/* Plant a plaintext record behind the wrap's back: a pre-provisioning
	 * leftover, or a downgrade's write. */
	set_long_name("leftover");
	flush_all();
	/* Rewrite the owner record ourselves, plaintext, straight to NVS. */
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "");
	{
		uint8_t plain[256];
		int n = meshtastic_lockdown_open("meshtastic/owner", r.buf, r.len, plain, sizeof(plain));

		zassert_true(n > 0, "");
		zassert_ok(settings_save_one("meshtastic/owner", plain, (size_t)n), "");
	}
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_false(meshtastic_lockdown_is_sealed(r.buf, r.len), "planted plaintext");

	stage_boot();
	zassert_true(meshtastic_lockdown_unlocked(), "");
	zassert_str_equal(long_name(), "leftover", "the plaintext leftover is accepted");
	flush_all();
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "and sealed at the next save");
}

ZTEST(lockdown_store, test_nodedb_records_are_sealed_and_come_back_after_unlock)
{
	static uint8_t pub[32] = {1, 2, 3};
	struct raw r;
	char name[40];

	seed_peer(PEER_ID, pub);
	{
		uint8_t got[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];

		zassert_ok(meshtastic_nodedb_copy_pubkey(PEER_ID, got), "peer learned");
	}
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	(void)snprintk(name, sizeof(name), "mtnode/%08x", PEER_ID);
	zassert_true(raw_read(name, &r), "warm key persisted");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "and sealed");

	meshtastic_lockdown_lock_now();
	stage_boot();
	zassert_ok(meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	{
		uint8_t got[MESHTASTIC_NODEDB_PUBLIC_KEY_MAX_LEN];

		zassert_ok(meshtastic_nodedb_copy_pubkey(PEER_ID, got),
			   "the peer key is back after the reload");
		zassert_mem_equal(got, pub, 32, "");
	}
}

ZTEST(lockdown_store, test_disable_writes_everything_back_in_the_clear)
{
	struct raw r;

	set_long_name("sealed name");
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	zassert_equal(meshtastic_lockdown_disable(WRONG, sizeof(WRONG) - 1U), -EACCES,
		      "a disable needs the passphrase");
	zassert_true(meshtastic_lockdown_active(), "");

	zassert_ok(meshtastic_lockdown_disable(PP, PPLEN), "");
	wait_idle();
	zassert_true(meshtastic_lockdown_disable_done(), "");
	zassert_false(meshtastic_lockdown_active(), "stock again");
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_false(meshtastic_lockdown_is_sealed(r.buf, r.len), "owner in the clear");
	zassert_true(raw_read("meshtastic/config/lora", &r), "");
	zassert_false(meshtastic_lockdown_is_sealed(r.buf, r.len), "lora in the clear");
	zassert_false(raw_read("mtlock/dek", &r), "artifacts gone");

	stage_boot();
	zassert_false(meshtastic_lockdown_active(), "");
	zassert_str_equal(long_name(), "sealed name", "a stock boot reads the rewritten records");
	zassert_false(mt.radio_held, "");
}

ZTEST(lockdown_store, test_iteration_count_travels_in_the_dek_record)
{
	struct raw r;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	zassert_true(raw_read("mtlock/dek", &r), "");
	zassert_equal(r.len, 52U, "MDEK | iters | nonce | ct | tag");
	zassert_equal(sys_get_le32(r.buf + 4), CONFIG_MESHTASTIC_LOCKDOWN_PBKDF2_ITERATIONS, "");
	/* A changed count does not unlock: the KEK derived with it no longer
	 * matches the one the DEK was wrapped under (the AAD over the count
	 * only makes the tamper explicit; the mismatch alone already refuses). */
	meshtastic_lockdown_lock_now();
	sys_put_le32(1U, r.buf + 4);
	zassert_ok(settings_save_one("mtlock/dek", r.buf, r.len), "");
	stage_boot();
	zassert_equal(meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U), -EACCES,
		      "a changed iteration count does not unlock");
	/* Put it back so the suite's teardown can disable. */
	sys_put_le32(CONFIG_MESHTASTIC_LOCKDOWN_PBKDF2_ITERATIONS, r.buf + 4);
	zassert_ok(settings_save_one("mtlock/dek", r.buf, r.len), "");
	stage_boot();
	stage_boot(); /* the reboot floor after the wrong attempt */
}
