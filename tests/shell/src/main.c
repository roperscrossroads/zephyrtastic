/* SPDX-License-Identifier: GPL-3.0
 *
 * Shell trust-boundary tests.
 *
 * The "meshtastic" shell writes the same config store the admin model guards,
 * but the console is not an authenticated transport: anyone with UART, USB or
 * RTT access reaches it with no pairing, passkey or key check. So a managed
 * node that correctly refuses local admin over the PhoneAPI must refuse the
 * equivalent shell writes too, or is_managed is only advertising a gate it does
 * not have.
 *
 * These drive the REAL command handlers through shell_execute_cmd() against the
 * dummy backend and read back what they printed, rather than calling the
 * config-store helpers directly — the gate lives in the command layer, so
 * anything below it would not exercise the thing under test.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_clock.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"

#define TEST_NODE_ID 0x12345678U

/* ---- Minimal mock LoRa driver (meshtastic_init needs a device) ------------ */

static int mock_lora_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int mock_lora_config(const struct device *dev, const struct lora_modem_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);
	return 0;
}

static uint32_t mock_lora_airtime(const struct device *dev, uint32_t data_len)
{
	ARG_UNUSED(dev);
	return data_len;
}

static int mock_lora_send(const struct device *dev, uint8_t *data, uint32_t data_len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(data_len);
	return 0;
}

static int mock_lora_send_async(const struct device *dev, uint8_t *data, uint32_t data_len,
				struct k_poll_signal *async)
{
	int ret = mock_lora_send(dev, data, data_len);

	if (async != NULL) {
		k_poll_signal_raise(async, ret);
	}
	return ret;
}

static int mock_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
			  k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	return -ENOTSUP;
}

static int mock_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return 0;
}

static DEVICE_API(lora, mock_lora_api) = {
	.config = mock_lora_config,
	.airtime = mock_lora_airtime,
	.send = mock_lora_send,
	.send_async = mock_lora_send_async,
	.recv = mock_lora_recv,
	.recv_async = mock_lora_recv_async,
};

DEVICE_DEFINE(mock_lora, "mock_lora", mock_lora_init, NULL, NULL, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mock_lora_api);

static const struct device *const lora_dev = DEVICE_GET(mock_lora);

/* ---- Fixture -------------------------------------------------------------- */

/* Run a shell command and return its exit code; *out points at everything the
 * command printed (owned by the dummy backend, valid until the next command). */
static int run_cmd(const char *cmd, const char **out)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	size_t len = 0;
	int ret;

	shell_backend_dummy_clear_output(sh);
	ret = shell_execute_cmd(sh, cmd);
	if (out != NULL) {
		*out = shell_backend_dummy_get_output(sh, &len);
	}
	return ret;
}

static void set_managed(bool managed)
{
	meshtastic_Config sec = meshtastic_Config_init_zero;

	sec.which_payload_variant = meshtastic_Config_security_tag;
	sec.payload_variant.security.is_managed = managed;
	zassert_ok(meshtastic_config_store_set_config(&sec), "security config write failed");
}

static meshtastic_Config_DeviceConfig_Role stored_role(void)
{
	meshtastic_Config dev;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &dev),
		   "device config read failed");
	return dev.payload_variant.device.role;
}

static void set_stored_role(meshtastic_Config_DeviceConfig_Role role)
{
	meshtastic_Config dev = meshtastic_Config_init_zero;

	dev.which_payload_variant = meshtastic_Config_device_tag;
	dev.payload_variant.device.role = role;
	zassert_ok(meshtastic_config_store_set_config(&dev), "device config write failed");
}

static void *shell_suite_setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "mock lora not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");

	/* Let the shell backend finish coming up before driving commands. */
	k_sleep(K_MSEC(50));
	return NULL;
}

static void shell_before(void *fixture)
{
	ARG_UNUSED(fixture);
	set_managed(false);
	set_stored_role(meshtastic_Config_DeviceConfig_Role_CLIENT);
}

ZTEST_SUITE(meshtastic_shell, NULL, shell_suite_setup, shell_before, NULL, NULL);

/* ---- Reads are always available ------------------------------------------ */

/* Reading state must survive every gate — an operator has to be able to see
 * what a node is doing even on a locked-down build. */
ZTEST(meshtastic_shell, test_reads_always_available)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic device role", &out), "role read failed");
	zassert_not_null(strstr(out, "role:"), "expected a role line, got: %s", out);

	zassert_ok(run_cmd("meshtastic channel list", &out), "channel list failed");
	zassert_not_null(strstr(out, "[0]"), "expected slot 0 in the listing, got: %s", out);

	zassert_ok(run_cmd("meshtastic channel show 0", &out), "channel show failed");
	zassert_not_null(strstr(out, "psk:"), "expected a psk summary, got: %s", out);
}

/* ---- PSK disclosure ------------------------------------------------------- */

/* Raw key material is opt-in at build time. The summary (kind/length) is always
 * printed and is what an operator normally needs. */
ZTEST(meshtastic_shell, test_psk_hex_follows_kconfig)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic channel show 0", &out), "channel show failed");

	if (IS_ENABLED(CONFIG_MESHTASTIC_SHELL_PSK_HEX)) {
		zassert_not_null(strstr(out, "psk hex:"),
				 "PSK_HEX=y should disclose raw key bytes, got: %s", out);
	} else {
		zassert_is_null(strstr(out, "psk hex:"),
				"raw PSK bytes must not be printed unless PSK_HEX=y, got: %s",
				out);
	}
}

/* ---- The managed gate ----------------------------------------------------- */

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)

/* Control: on an unmanaged node the write works. Without this, the refusal
 * tests below could pass simply because the command was broken. */
ZTEST(meshtastic_shell, test_unmanaged_node_allows_config_write)
{
	zassert_ok(run_cmd("meshtastic device role router", NULL), "role write failed");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "an unmanaged node should accept a shell role write");
}

/* The favourite command is a config write: a managed node refuses it before it
 * reaches the NodeDB, exactly like the other mutating commands. */
ZTEST(meshtastic_shell, test_managed_node_refuses_favorite)
{
	const char *out;

	set_managed(true);

	zassert_not_equal(run_cmd("meshtastic nodedb favorite 0x12345678 on", &out), 0,
			  "a managed node must refuse a favourite write");
	zassert_not_null(strstr(out, "managed"), "expected a managed-node refusal, got: %s", out);
}

/* Unmanaged, the command reaches the NodeDB and reports the node is absent —
 * proving it passed the gate rather than being refused by it. That "reached the
 * store" signal is what distinguishes the gate from the operation. */
ZTEST(meshtastic_shell, test_unmanaged_favorite_reaches_nodedb)
{
	const char *out;

	/* An id certainly not in the DB (0x12345678 is our own node, which exists).
	 * The error proves the command passed the gate and reached the store. */
	zassert_not_equal(run_cmd("meshtastic nodedb favorite 0xDEADBEEF on", &out), 0,
			  "favouriting an absent node should fail");
	zassert_not_null(strstr(out, "DB"),
			 "expected the NodeDB-layer error, not a gate refusal, got: %s", out);
}

/* The finding: a managed node refuses local admin over the PhoneAPI but the
 * shell wrote config regardless, so is_managed could be bypassed entirely by
 * anyone at the console. */
ZTEST(meshtastic_shell, test_managed_node_refuses_role_write)
{
	const char *out;

	set_managed(true);

	zassert_not_equal(run_cmd("meshtastic device role router", &out), 0,
			  "a managed node must refuse a shell role write");
	zassert_not_null(strstr(out, "managed"), "expected a managed-node refusal, got: %s", out);
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "refused write must not reach the config store");
}

ZTEST(meshtastic_shell, test_managed_node_refuses_rebroadcast_write)
{
	set_managed(true);

	zassert_not_equal(run_cmd("meshtastic device rebroadcast none", NULL), 0,
			  "a managed node must refuse a shell rebroadcast write");
}

/* The PSK-rewrite path is the sharpest edge of the finding: it does not just
 * reconfigure the node, it re-keys the channel. */
ZTEST(meshtastic_shell, test_managed_node_refuses_channel_psk_rewrite)
{
	const char *out;
	uint8_t hash_before = meshtastic_channels_get_hash(0U);

	set_managed(true);

	zassert_not_equal(
		run_cmd("meshtastic channel set 0 psk hex "
			"000102030405060708090a0b0c0d0e0f",
			&out),
		0, "a managed node must refuse a shell PSK rewrite");
	zassert_equal(meshtastic_channels_get_hash(0U), hash_before,
		      "refused PSK rewrite must leave the channel key untouched");
}

ZTEST(meshtastic_shell, test_managed_node_refuses_channel_disable)
{
	set_managed(true);

	zassert_not_equal(run_cmd("meshtastic channel disable 1", NULL), 0,
			  "a managed node must refuse a shell channel disable");
}

/* A managed node that is later unmanaged must accept writes again — the gate is
 * policy, not a latch. */
ZTEST(meshtastic_shell, test_unmanaging_restores_config_write)
{
	set_managed(true);
	zassert_not_equal(run_cmd("meshtastic device role router", NULL), 0, "expected refusal");

	set_managed(false);
	zassert_ok(run_cmd("meshtastic device role router", NULL), "write should work again");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_ROUTER,
		      "unmanaged node should accept the write");
}

/* `meshtastic time set` seeds the wall clock at NTP quality; a value outside
 * the sane window is refused. No assertion on the pristine (unset) state —
 * the clock is global and another test may have run first. */
ZTEST(meshtastic_shell, test_time_set_and_readback)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic time set 1756000000", &out), "time set failed");
	zassert_not_null(strstr(out, "epoch 1756"), "expected the epoch readback, got: %s", out);

	zassert_ok(run_cmd("meshtastic time", &out), "time show failed");
	zassert_not_null(strstr(out, "epoch 1756"), "clock should hold the epoch, got: %s", out);

	zassert_not_equal(run_cmd("meshtastic time set 1000", &out), 0,
			  "an epoch outside the sane window must be refused");
}

/* The fresh-node seed honours MESHTASTIC_TX_ENABLED_DEFAULT — the compile-time
 * half of the a4it.8 safety story: with =n the first boot after a flash is
 * already receive-only, no config-write window. Asserting against IS_ENABLED
 * exercises whichever value this scenario builds with; the dedicated
 * tx_default_off scenario runs the =n branch. */
ZTEST(meshtastic_shell, test_lora_tx_seed_follows_kconfig)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "lora config read failed");
	zassert_equal(cfg.payload_variant.lora.tx_enabled,
		      IS_ENABLED(CONFIG_MESHTASTIC_TX_ENABLED_DEFAULT),
		      "seeded tx_enabled should match MESHTASTIC_TX_ENABLED_DEFAULT");
}

/* `lora tx off` is the safety switch for a node with damaged RF hardware
 * (agents-a4it.8): it must persist to the stored LoRaConfig, apply live (no
 * reboot — set_config runs apply_core), and read back as receive-only. */
ZTEST(meshtastic_shell, test_lora_tx_off_persists_and_reads_back)
{
	meshtastic_Config cfg;
	const char *out;

	zassert_ok(run_cmd("meshtastic lora tx off", &out), "tx off failed");
	zassert_not_null(strstr(out, "receive only"), "expected the muted confirmation, got: %s",
			 out);

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "lora config read failed");
	zassert_false(cfg.payload_variant.lora.tx_enabled, "tx_enabled should be stored false");

	zassert_ok(run_cmd("meshtastic lora", &out), "lora show failed");
	zassert_not_null(strstr(out, "NO — receive only"),
			 "the show command should prove the mute, got: %s", out);

	zassert_ok(run_cmd("meshtastic lora tx on", NULL), "tx on failed");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg),
		   "lora config re-read failed");
	zassert_true(cfg.payload_variant.lora.tx_enabled, "tx_enabled should be restored");

	/* Leave the store at the build's seed value so the seed-follows-Kconfig
	 * test holds regardless of execution order. */
	zassert_ok(run_cmd(IS_ENABLED(CONFIG_MESHTASTIC_TX_ENABLED_DEFAULT)
				   ? "meshtastic lora tx on"
				   : "meshtastic lora tx off",
			   NULL),
		   "restore failed");
}

ZTEST(meshtastic_shell, test_managed_node_refuses_lora_tx_write)
{
	const char *out;

	set_managed(true);
	zassert_not_equal(run_cmd("meshtastic lora tx off", &out), 0,
			  "a managed node must refuse the tx write");
	zassert_not_null(strstr(out, "managed"), "expected a managed-node refusal, got: %s", out);
}

#else /* !CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

/* Compiled-out build: the mutating subcommands are gone and the dual
 * read/write commands refuse their write form, while the reads above still
 * work. */
ZTEST(meshtastic_shell, test_config_write_compiled_out)
{
	zassert_not_equal(run_cmd("meshtastic channel set 0 name nope", NULL), 0,
			  "channel set must not exist when config writes are compiled out");
	zassert_not_equal(run_cmd("meshtastic channel disable 1", NULL), 0,
			  "channel disable must not exist when config writes are compiled out");
	zassert_not_equal(run_cmd("meshtastic device role router", NULL), 0,
			  "device role write must be refused when compiled out");
	zassert_equal(stored_role(), meshtastic_Config_DeviceConfig_Role_CLIENT,
		      "nothing should have reached the config store");
}

#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

#if defined(CONFIG_MESHTASTIC_RF_PATH_REPORT)

/* ---- `meshtastic rf` ------------------------------------------------------
 *
 * The report's whole value is that it distinguishes "in effect" from
 * "configured", so the tests assert on the MARKERS rather than on the numbers —
 * the numbers are hardware, the markers are the logic.
 *
 * native_sim is the only place two of these branches are reachable at all: it
 * has no board FEM override, so the weak defaults apply and the
 * "not on this hardware" row is exercised; and its mock radio is not an SX126x,
 * so the driver readback is genuinely unavailable and must report unknown
 * rather than off.
 */

ZTEST(meshtastic_shell, test_rf_reports_every_chain_stage)
{
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic rf", &out), "rf should always be readable");
	zassert_not_null(out, "rf printed nothing");

	/* Every stage present, in signal order. A stage that silently stopped
	 * being printed would look identical to a stage that is fine. */
	zassert_not_null(strstr(out, "FRONT END"), "missing FRONT END section");
	zassert_not_null(strstr(out, "RECEIVE"), "missing RECEIVE section");
	zassert_not_null(strstr(out, "TRANSMIT"), "missing TRANSMIT section");
	zassert_not_null(strstr(out, "HEALTH"), "missing HEALTH section");

	zassert_not_null(strstr(out, "rx boosted gain"), "missing the rx boost row");
	zassert_not_null(strstr(out, "fem lna"), "missing the fem lna row");
	zassert_not_null(strstr(out, "tx power"), "missing the tx power row");
}

/*
 * The row that only this platform can prove. With no board override,
 * meshtastic_radio_fem_lna_can_control() is the weak false, so the LNA row must
 * report "not present" — NOT "disabled". Reporting absence as a disabled
 * setting is the exact confusion the marker scheme exists to prevent, and on
 * every bench board this branch is unreachable.
 */
ZTEST(meshtastic_shell, test_rf_marks_absent_hardware_as_absent)
{
	const char *out = NULL;
	const char *row;

	zassert_ok(run_cmd("meshtastic rf", &out), "rf failed");
	row = strstr(out, "fem lna");
	zassert_not_null(row, "missing the fem lna row");

	zassert_not_null(strstr(row, "no controllable receive path"),
			 "a board with no LNA control must say so, not report a mode");
}

/*
 * The mock radio is not an SX126x, so meshtastic_radio_rx_boosted_applied()
 * cannot know what the chip is doing. It must say "unknown". If this ever reads
 * OFF, the tristate has been collapsed to a bool somewhere and the report is
 * now claiming knowledge it does not have.
 */
ZTEST(meshtastic_shell, test_rf_reports_unknown_readback_as_unknown)
{
	const char *out = NULL;
	const char *row;

	zassert_ok(run_cmd("meshtastic rf", &out), "rf failed");
	row = strstr(out, "rx boosted gain");
	zassert_not_null(row, "missing the rx boost row");

	zassert_not_null(strstr(row, "applied unknown"),
			 "an unreportable applied gain must read unknown, never OFF");
	zassert_not_null(strstr(out, "not assuming it is off"),
			 "the unknown case should explain itself");
}

#endif /* CONFIG_MESHTASTIC_RF_PATH_REPORT */

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/*
 * `meshtastic owner set` — the local writer that did not exist.
 *
 * Worth having tests rather than just a command, because the shell is the ONLY
 * way to name two states of node: one whose admin_key list is empty (nothing may
 * administer it) and one that cannot transmit on the mesh (it cannot answer the
 * getter that must precede a remote set). Both have a console and nothing else.
 */
ZTEST(meshtastic_shell, test_owner_set_and_readback)
{
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic owner set \"RX Unit\" rxru", &out), "owner set failed");
	zassert_ok(run_cmd("meshtastic owner", &out), "owner show failed");
	zassert_not_null(strstr(out, "long=\"RX Unit\""), "the long name must read back");
	zassert_not_null(strstr(out, "short=\"rxru\""), "the short name must read back");
}

/* The long name alone is a legal call: the short name is optional, and an empty
 * one means "leave it alone" (upstream handleSetOwner parity), NOT "clear it". */
ZTEST(meshtastic_shell, test_owner_set_without_short_name_keeps_the_short_name)
{
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic owner set First aaaa", &out), "owner set failed");
	zassert_ok(run_cmd("meshtastic owner set Second", &out), "owner set failed");
	zassert_ok(run_cmd("meshtastic owner", &out), "owner show failed");
	zassert_not_null(strstr(out, "long=\"Second\""), "the long name must have changed");
	zassert_not_null(strstr(out, "short=\"aaaa\""),
			 "omitting the short name must leave it alone, not clear it");
}

/*
 * THE TRAP THIS TEST EXISTS FOR. set_owner takes a whole User and reads
 * is_licensed as a plain proto3 bool, so an unset field is indistinguishable
 * from an explicit false — building a User from zero for a rename would clear an
 * operator's licence as a silent side effect, and with it the transmit power
 * that was resolved under it.
 */
ZTEST(meshtastic_shell, test_owner_set_does_not_clear_the_licence)
{
	meshtastic_User licensed_user = meshtastic_User_init_zero;
	const char *out = NULL;
	bool licensed = false;

	strcpy(licensed_user.long_name, "Licensed");
	strcpy(licensed_user.short_name, "lic1");
	licensed_user.is_licensed = true;
	zassert_ok(meshtastic_config_store_set_owner(&licensed_user), "owner write failed");

	meshtastic_config_store_get_owner_flags(&licensed, NULL);
	zassert_true(licensed, "setup: the licence must be set before the rename");

	zassert_ok(run_cmd("meshtastic owner set Renamed rnm1", &out), "owner set failed");

	meshtastic_config_store_get_owner_flags(&licensed, NULL);
	zassert_true(licensed, "a rename must not clear the operator licence");

	/* Leave the licence off — it changes the resolved TX power, and later tests
	 * must not inherit an elevated one. */
	licensed_user.is_licensed = false;
	zassert_ok(meshtastic_config_store_set_owner(&licensed_user), "owner restore failed");
}

#else /* !CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

/* The other half of the build coverage: with writes compiled out the command
 * must still EXIST and still report, and only the setter is refused. A command
 * that vanished entirely would look like a missing feature rather than a
 * deliberate build choice. */
ZTEST(meshtastic_shell, test_owner_set_is_refused_when_writes_are_compiled_out)
{
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic owner", &out), "showing the owner must still work");
	zassert_not_null(strstr(out, "long="), "the report must still be produced");

	zassert_not_equal(run_cmd("meshtastic owner set Nope nope", &out), 0,
			  "a compiled-out write must be refused, not silently accepted");
	zassert_not_null(strstr(out, "compiled out"), "the refusal should name the reason");
}

#endif /* CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE */

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)
/*
 * `admin remote <node> set-time` with no epoch relays OUR clock, which means a
 * node with no clock has nothing to relay (agents-xhli.14).
 *
 * Refusing is not politeness. An unset clock reads epoch 0, the target's sanity
 * window rejects it, and the operator would see a request "sent" that silently
 * did nothing — the least debuggable outcome available.
 */
ZTEST(meshtastic_shell, test_admin_set_time_refuses_when_this_node_has_no_clock)
{
	const char *out = NULL;

	meshtastic_clock_test_reset();

	zassert_not_equal(run_cmd("meshtastic admin remote 0x12345678 set-time", &out), 0,
			  "a node with no clock must not offer itself as a time source");
	zassert_not_null(strstr(out, "clock is unset"), "the refusal should name the reason");

	/* With a clock, it gets as far as the passkey gate instead — which proves
	 * the refusal above was about the clock and not about the peer. */
	meshtastic_clock_set_epoch(1750000000U, MESHTASTIC_CLOCK_QUALITY_GPS);
	zassert_not_equal(run_cmd("meshtastic admin remote 0x12345678 set-time", &out), 0,
			  "no passkey is cached for that node, so this must still fail");
	zassert_not_null(strstr(out, "session passkey"),
			 "with a clock in hand the next gate is the passkey, not the clock");

	/* As above: leave it unset so a GPS-quality clock does not outrank a later
	 * test's `meshtastic time set`. */
	meshtastic_clock_test_reset();
}
#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */

#if defined(CONFIG_MESHTASTIC_CLUSTER)

/* ---- Cluster sync commands ------------------------------------------------ */

/*
 * These exist first of all to COMPILE the `meshtastic cluster …` block: no
 * twister scenario built the shell and the cluster module together until this
 * one, so those commands reached hardware unbuilt by CI. What they assert
 * beyond that is the shell-visible half of the M4b contract — the promote gates
 * and the origin marker's refusal — which is where an operator actually meets
 * it.
 */

/* Provision the channel the module binds to, so status reports it bound. */
static void provision_cluster_channel(void)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;

	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	strncpy(ch.settings.name, CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME,
		sizeof(ch.settings.name) - 1U);
	ch.settings.psk.size = 16U;
	ch.settings.psk.bytes[0] = 0x42U;
	zassert_ok(meshtastic_channels_set_slot(2U, &ch), "cluster channel set failed");
}

/*
 * A NODE WITH NO CLOCK HAS NO DRIFT HORIZON, and until agents-xhli.14 nothing
 * said so. The horizon (D12) is measured against the wall clock, so an unset
 * clock means every stamp is accepted — including one dated 2100, which would
 * then win every LWW comparison for good.
 *
 * Keeping that behaviour is deliberate: a node that cannot say when anything
 * happened has no basis to refuse, and refusing everything would strand a fresh
 * node permanently. What was not deliberate was the silence. This asserts the
 * status report names it, for the same reason `sections_held` exists — a
 * deliberate non-enforcement nobody can see is indistinguishable from a broken
 * one.
 */
ZTEST(meshtastic_shell, test_cluster_status_names_a_missing_horizon)
{
	const char *out = NULL;

	provision_cluster_channel();

	meshtastic_clock_test_reset();
	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "horizon"), "status must report the horizon at all");
	zassert_not_null(strstr(out, "NONE"),
			 "with no clock the report must say the horizon is absent, not "
			 "leave the reader to infer it from a missing line");

	meshtastic_clock_set_epoch(1750000000U, MESHTASTIC_CLOCK_QUALITY_GPS);
	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_is_null(strstr(out, "NONE"), "with a clock the horizon must not read as absent");
	zassert_not_null(strstr(out, "horizon"), "the horizon line must still be present");

	/* Leave the clock UNSET, not GPS. ztest orders by name, and a GPS-quality
	 * clock legitimately refuses the NTP-class write that `meshtastic time set`
	 * performs — so parking GPS here breaks a later test for a reason that looks
	 * nothing like the cause. */
	meshtastic_clock_test_reset();
}

ZTEST(meshtastic_shell, test_cluster_status_reports_channel_binding)
{
	const char *out = NULL;
	meshtastic_Channel off = meshtastic_Channel_init_zero;

	off.role = meshtastic_Channel_Role_DISABLED;
	off.has_settings = true;
	zassert_ok(meshtastic_channels_set_slot(2U, &off), "channel teardown failed");

	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "NOT PROVISIONED"),
			 "an unbound module must say so, not report a silent idle");

	provision_cluster_channel();
	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "= index 2"), "status must name the bound slot");
	zassert_not_null(strstr(out, "sync    : idle"), "status must report the walk state");
}

/* The secret boundary and the lora hazard, as an operator meets them. */
ZTEST(meshtastic_shell, test_cluster_promote_refuses_secrets_and_lora)
{
	const char *out = NULL;

	provision_cluster_channel();

	/* Not in the parser's table at all — security and network are not even
	 * nameable, which is the outermost layer of the D9 ban. */
	zassert_not_equal(run_cmd("meshtastic cluster promote security", &out), 0,
			  "security must never be promotable");
	zassert_not_equal(run_cmd("meshtastic cluster promote network", &out), 0,
			  "network must never be promotable");

	/* lora IS nameable, deliberately: the refusal that follows teaches the
	 * §7.9 straggler problem, which "unknown section" would not. */
	zassert_not_equal(run_cmd("meshtastic cluster promote lora", &out), 0,
			  "lora promote must be refused until the straggler sweep exists");
	zassert_not_null(strstr(out, "orphans nodes"), "the refusal should explain itself");
}

/*
 * The origin marker, from the operator's side (CLUSTER-SYNC-M4.md D10). A
 * promote applies its own entry back through the store, which leaves the
 * store's stamp equal to the document's — the marker that says "this value came
 * from the document". Promoting again would mint a second stamp for identical
 * bytes and make the whole fleet churn through an apply that changes nothing,
 * so it is refused until a local edit moves the stamp again.
 */
ZTEST(meshtastic_shell, test_cluster_promote_is_idempotent_via_origin_marker)
{
	const char *out = NULL;

	provision_cluster_channel();

	zassert_ok(run_cmd("meshtastic cluster promote display", &out), "promote failed");
	zassert_not_null(strstr(out, "promoted to fleet base"), "promote should confirm");

	/* The reconciler runs on the system workqueue. */
	k_sleep(K_MSEC(50));

	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "sections applied=1"),
			 "the promoting node must apply its own base entry");

	zassert_not_equal(run_cmd("meshtastic cluster promote display", &out), 0,
			  "re-promoting an unchanged doc-derived section must be refused");
	zassert_not_null(strstr(out, "already IS the fleet base"),
			 "the refusal should name the reason");
}

ZTEST(meshtastic_shell, test_cluster_pull_needs_a_channel_and_a_peer)
{
	const char *out = NULL;
	meshtastic_Channel off = meshtastic_Channel_init_zero;

	off.role = meshtastic_Channel_Role_DISABLED;
	off.has_settings = true;
	zassert_ok(meshtastic_channels_set_slot(2U, &off), "channel teardown failed");

	zassert_not_equal(run_cmd("meshtastic cluster pull 0xDEADBEEF", &out), 0,
			  "a pull with no cluster channel must fail, not idle silently");
	zassert_not_null(strstr(out, "nothing to pull over"), "the refusal should explain itself");

	provision_cluster_channel();
	zassert_not_equal(run_cmd("meshtastic cluster pull 0x12345678", &out), 0,
			  "pulling from ourselves is not a walk");
}
/*
 * The claim has to be legible, because nothing else on the node would ever say
 * that this one has stopped being a place the fleet can recover a pin from.
 * CORE costs nothing in what the node RUNS, which is exactly why it would
 * otherwise be invisible.
 */
ZTEST(meshtastic_shell, test_cluster_status_names_the_scope_and_what_core_costs)
{
	const char *out = NULL;

	provision_cluster_channel();

	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "scope   : FULL"),
			 "status must name what this node claims to track");
	zassert_not_null(strstr(out, "tracking the whole fleet"),
			 "and say plainly what that means");

	zassert_ok(run_cmd("meshtastic cluster scope core", &out), "narrowing failed");
	zassert_ok(run_cmd("meshtastic cluster status", &out), "status failed");
	zassert_not_null(strstr(out, "scope   : CORE"), "a narrowed claim must show as CORE");
	zassert_not_null(strstr(out, "NOT a restore source"),
			 "and must say what the fleet has lost, since the node itself is "
			 "running exactly what it was");

	zassert_ok(run_cmd("meshtastic cluster scope auto", &out), "restoring failed");
}

ZTEST(meshtastic_shell, test_cluster_scope_command_sets_and_reads_back)
{
	const char *out = NULL;

	provision_cluster_channel();
	zassert_ok(run_cmd("meshtastic cluster scope auto", &out), "baseline");

	zassert_not_equal(run_cmd("meshtastic cluster scope", &out), 0,
			  "a scope change is not something to do by accident");
	zassert_not_equal(run_cmd("meshtastic cluster scope enormous", &out), 0,
			  "an unknown claim must be refused, not guessed at");
	zassert_not_null(strstr(out, "full, core or auto"), "the refusal should list the choices");

	zassert_ok(run_cmd("meshtastic cluster scope core", &out), "narrowing failed");
	zassert_not_null(strstr(out, "no longer a restore source"),
			 "the operator must be told what they gave up");
	zassert_not_null(strstr(out, "what it RUNS is unchanged"),
			 "and what they did not");

	zassert_ok(run_cmd("meshtastic cluster scope core", &out), "a repeat must not error");
	zassert_not_null(strstr(out, "already claiming that"), "it should say so");

	/* FULL pinned is a real choice with a real cost, and the shell says so:
	 * it reinstates the table that fills and never heals. */
	zassert_ok(run_cmd("meshtastic cluster scope full", &out), "widening failed");
	zassert_not_null(strstr(out, "pinned"), "a pinned claim must announce itself");

	zassert_ok(run_cmd("meshtastic cluster scope auto", &out), "restoring failed");
	zassert_not_null(strstr(out, "auto"), "and so must an automatic one");
}

/*
 * The one destructive verb here that the fleet cannot argue with. Every other
 * write mints a versioned entry a peer can outrank; this throws away the whole
 * local copy — so it asks, and it says what the operator has NOT achieved.
 */
ZTEST(meshtastic_shell, test_cluster_reset_needs_confirmation_and_says_what_it_did_not_do)
{
	const char *out = NULL;

	provision_cluster_channel();

	zassert_not_equal(run_cmd("meshtastic cluster reset", &out), 0,
			  "a bare `reset` next to `status` must not empty the document");
	zassert_not_null(strstr(out, "--confirm"), "and must say how to mean it");

	zassert_ok(run_cmd("meshtastic cluster reset --confirm", &out), "confirmed reset failed");
	zassert_not_null(strstr(out, "cleared"), "it should report what it dropped");
	zassert_not_null(strstr(out, "told NOBODY"),
			 "and must be explicit that the fleet has not forgotten anything — "
			 "otherwise an operator reads a clean node as a clean fleet");
	zassert_not_null(strstr(out, "lora tx off"),
			 "including the way to actually clear a fleet");
}

#endif /* CONFIG_MESHTASTIC_CLUSTER */
