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

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/intents.h>
#include <zephyr/meshtastic/meshtastic.h>

#include <zephyr/meshtastic/nodeinfo.h>

#include "meshtastic_clock.h"
#include "meshtastic_channels.h"
#include "meshtastic_scanner.h"
#include "meshtastic_config_store.h"
#include "meshtastic_position.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"

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
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	const char *out;

	/* A slot with a real key: the primary holds the well-known key in its one-byte
	 * form, which has no bytes worth hiding and is never printed as hex. */
	ch.index = 1;
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	strcpy(ch.settings.name, "keyed");
	ch.settings.psk.size = 16U;
	memset(ch.settings.psk.bytes, 0xA5, 16U);
	zassert_ok(meshtastic_channels_set_slot(1U, &ch));
	zassert_ok(run_cmd("meshtastic channel show 1", &out), "channel show failed");
	ch.role = meshtastic_Channel_Role_DISABLED;
	zassert_ok(meshtastic_channels_set_slot(1U, &ch));

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

/* `lora power` sets this node's own transmit power from the console: stored,
 * shown, and 0 restores the region maximum. Before it, only the phone API
 * could, and the bench's brains sat at 14 dBm beside 2 dBm heads. */
ZTEST(meshtastic_shell, test_lora_power_sets_stores_and_reads_back)
{
	meshtastic_Config cfg;
	const char *out;
	int32_t before;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg), "");
	before = cfg.payload_variant.lora.tx_power;

	zassert_ok(run_cmd("meshtastic lora power 2", &out), "power set failed");
	zassert_not_null(strstr(out, "2 dBm at the antenna"), "show should say 2 dBm, got: %s", out);
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg), "");
	zassert_equal(cfg.payload_variant.lora.tx_power, 2, "stored %d",
		      (int)cfg.payload_variant.lora.tx_power);

	zassert_not_equal(run_cmd("meshtastic lora power 31", NULL), 0, "above 30 is refused");
	zassert_not_equal(run_cmd("meshtastic lora power x", NULL), 0, "a non-number is refused");
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &cfg), "");
	zassert_equal(cfg.payload_variant.lora.tx_power, 2, "a refused value changes nothing");

	zassert_ok(run_cmd("meshtastic lora power 0", &out), "");
	zassert_not_null(strstr(out, "[region default]"), "0 is the region maximum, got: %s", out);

	cfg.payload_variant.lora.tx_power = before;
	cfg.which_payload_variant = meshtastic_Config_lora_tag;
	zassert_ok(meshtastic_config_store_set_config(&cfg), "restore");
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

/*
 * `lora preset` must reach the radio live, no reboot (agents-k8oe) — matching
 * the reference (AdminModule.cpp:1108-1110, requiresReboot = false
 * unconditionally for the lora section). Proven the same way an operator
 * would prove it on the bench: the command's own confirmation text, and the
 * preset-generation counter, which is bumped only by a successful
 * meshtastic_preset_switch()/_apply_stored() retune — not by persisting a
 * config value nobody pushed to the chip.
 */
ZTEST(meshtastic_shell, test_lora_preset_applies_live)
{
	const char *out = NULL;
	uint32_t generation_before = meshtastic_preset_generation();

	zassert_ok(run_cmd("meshtastic lora preset ShortTurbo", &out), "lora preset failed");
	zassert_not_null(strstr(out, "applied live"),
			 "preset change must say it applied live, not the old 'reboot to "
			 "apply' (agents-k8oe), got: %s",
			 out);
	zassert_true(meshtastic_preset_generation() != generation_before,
		     "`lora preset` must actually retune the radio, not just persist "
		     "the config");

	zassert_ok(run_cmd("meshtastic lora", &out), "lora show failed");
	zassert_not_null(strstr(out, "modem preset: ShortTurbo"),
			 "the show command should confirm the new preset, got: %s", out);

	/* Leave the store on the fixture's default preset so other tests (and a
	 * re-run of this one) start from the same place. */
	zassert_ok(run_cmd("meshtastic lora preset LongFast", NULL), "restore failed");
}

/*
 * `nodeinfo interval` (agents-t2hb.1): device.node_info_broadcast_secs must
 * persist, read back through both the dedicated command and
 * meshtastic_nodeinfo_interval_secs() (what the auto-send thread actually
 * calls), and 0 must fall back to the compiled default rather than sticking
 * at 0 forever — same idiom as LoRa tx_power.
 */
ZTEST(meshtastic_shell, test_nodeinfo_interval_persists_and_reads_back)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic nodeinfo interval 60", &out), "interval set failed");
	zassert_not_null(strstr(out, "60 s"), "expected the new interval echoed back, got: %s",
			 out);
	zassert_not_null(strstr(out, "no reboot"),
			 "must confirm this applies without a reboot, got: %s", out);
	zassert_equal(meshtastic_nodeinfo_interval_secs(), 60U,
		      "the function the auto-send thread actually calls must see 60");

	zassert_ok(run_cmd("meshtastic nodeinfo interval", &out), "interval show failed");
	zassert_not_null(strstr(out, "interval: 60 s"), "show should confirm 60 s, got: %s", out);

	zassert_ok(run_cmd("meshtastic nodeinfo interval 0", NULL), "interval reset failed");
	zassert_equal(meshtastic_nodeinfo_interval_secs(), CONFIG_MESHTASTIC_NODEINFO_INTERVAL_SEC,
		      "0 must fall back to the compiled default, not stick at 0");
}

/*
 * `meshtastic position` (agents-t2hb.2): the two PositionConfig fields with a
 * consumer. interval must reach meshtastic_position_broadcast_secs() (what both
 * senders call) with 0 falling back to the compiled default; gps_mode must reach
 * the stored section by name; an unknown mode is refused, not stored.
 */
ZTEST(meshtastic_shell, test_position_interval_and_gps_mode)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic position interval 77", &out), "interval set failed");
	zassert_not_null(strstr(out, "77 s"), "expected the interval echoed, got: %s", out);
	zassert_not_null(strstr(out, "no reboot"), "must say it applies live, got: %s", out);
	zassert_equal(meshtastic_position_broadcast_secs(), 77U,
		      "the function both senders call must see 77");

	zassert_ok(run_cmd("meshtastic position gps_mode disabled", &out), "gps_mode set failed");
	zassert_equal(meshtastic_position_gps_mode(),
		      meshtastic_Config_PositionConfig_GpsMode_DISABLED, "");
	zassert_not_equal(run_cmd("meshtastic position gps_mode sideways", &out), 0,
			  "an unknown mode must be refused");
	zassert_equal(meshtastic_position_gps_mode(),
		      meshtastic_Config_PositionConfig_GpsMode_DISABLED, "and not stored");

	zassert_ok(run_cmd("meshtastic position", &out), "show failed");
	zassert_not_null(strstr(out, "gps_mode: disabled"), "show gps_mode, got: %s", out);
	zassert_not_null(strstr(out, "interval: 77 s"), "show interval, got: %s", out);
	zassert_not_null(strstr(out, "position: none"), "no position in this image, got: %s", out);

	zassert_ok(run_cmd("meshtastic position interval 0", &out), "interval reset failed");
	zassert_equal(meshtastic_position_broadcast_secs(),
		      CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC,
		      "0 must fall back to the compiled default");
	zassert_not_null(strstr(out, "compiled default"), "got: %s", out);
	zassert_ok(run_cmd("meshtastic position gps_mode not_present", NULL), "");
}

ZTEST(meshtastic_shell, test_managed_node_refuses_position_write)
{
	const char *out;

	zassert_ok(run_cmd("meshtastic position interval 0", NULL), "");
	set_managed(true);
	zassert_not_equal(run_cmd("meshtastic position interval 90", &out), 0,
			  "a managed node must refuse the write");
	zassert_equal(meshtastic_position_broadcast_secs(),
		      CONFIG_MESHTASTIC_POSITION_BROADCAST_INTERVAL_SEC, "nothing stored");
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

ZTEST(meshtastic_shell, test_position_write_compiled_out)
{
	const char *out;

	zassert_not_equal(run_cmd("meshtastic position interval 90", NULL), 0,
			  "position interval must not exist when config writes are compiled out");
	zassert_ok(run_cmd("meshtastic position", &out), "the read still works");
	zassert_not_null(strstr(out, "interval:"), "got: %s", out);
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

/* `admin trust key:<prefix> off` names an entry by its bytes, for a key whose
 * node is gone from the NodeDB (a re-keyed peer's old key: nothing else can
 * name it). Too short, ambiguous and unknown prefixes are refused or reported,
 * and only the unique match is removed. */
ZTEST(meshtastic_shell, test_admin_trust_removes_a_key_by_prefix)
{
	meshtastic_Config sec = meshtastic_Config_init_zero;
	meshtastic_Config chk;
	const char *out = NULL;

	sec.which_payload_variant = meshtastic_Config_security_tag;
	sec.payload_variant.security.admin_key_count = 2U;
	for (int i = 0; i < 2; i++) {
		sec.payload_variant.security.admin_key[i].size = 32U;
		memset(sec.payload_variant.security.admin_key[i].bytes, 0x51, 32U);
		sec.payload_variant.security.admin_key[i].bytes[0] = 0x51;
		sec.payload_variant.security.admin_key[i].bytes[1] = 0x0d;
		sec.payload_variant.security.admin_key[i].bytes[2] = 0xd5;
		sec.payload_variant.security.admin_key[i].bytes[3] = 0x92;
		sec.payload_variant.security.admin_key[i].bytes[4] = (uint8_t)(0xa0 + i);
	}
	zassert_ok(meshtastic_config_store_set_config(&sec), "security config write failed");

	zassert_ok(run_cmd("meshtastic admin trust", &out), "list failed");
	zassert_not_null(strstr(out, "admin keys: 2/"), "two keys seeded");

	zassert_not_equal(run_cmd("meshtastic admin trust key:510d off", &out), 0,
			  "fewer than 4 bytes is refused");
	zassert_not_equal(run_cmd("meshtastic admin trust key:510dd592 off", &out), 0,
			  "a prefix both keys share is ambiguous");
	zassert_not_equal(run_cmd("meshtastic admin trust key:510dd592a0", &out), 0,
			  "key: only removes");
	zassert_ok(run_cmd("meshtastic admin trust key:ffffffff off", &out), "unknown prefix");
	zassert_not_null(strstr(out, "no admin key starts with ffffffff"), "…is reported, not an error");

	zassert_ok(run_cmd("meshtastic admin trust key:510dd592a1 off", &out), "removal failed");
	zassert_not_null(strstr(out, "key 510dd592a1"), "the removal names the key: %s", out);
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_security_tag, &chk));
	zassert_equal(chk.payload_variant.security.admin_key_count, 1U, "one key left");
	zassert_equal(chk.payload_variant.security.admin_key[0].bytes[4], 0xa0,
		      "the OTHER key survived");
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

#if defined(CONFIG_MESHTASTIC_FLEET)
/*
 * The fleet verbs (DECLARATIVE-FLEET.md §7): `desire` publishes a base/FW row
 * for a class, `status` reads it back per class, `pin`/`unpin` write and
 * withdraw a node's own row. Driven through the real handlers; what a master's
 * shell session looks like.
 */
ZTEST(meshtastic_shell, test_fleet_desire_status_pin_and_unpin)
{
	const char *out = NULL;

	provision_cluster_channel();

	zassert_ok(run_cmd("meshtastic fleet status", &out), "status failed");
	zassert_not_null(strstr(out, "0 base rows"), "an empty document has no intent");

	zassert_not_equal(run_cmd("meshtastic fleet desire 0 0.3.0", &out), 0,
			  "class 0 is not a class");
	zassert_not_equal(run_cmd("meshtastic fleet desire 1 three", &out), 0,
			  "a version must be maj.min.rev");
	zassert_ok(run_cmd("meshtastic fleet desire 1 0.3.0", &out), "desire failed");
	zassert_not_null(strstr(out, "published: class 1 -> 0.3.0"), "desire should confirm");
	zassert_ok(run_cmd("meshtastic fleet desire 2 0.2.7 deadbeef pause", &out),
		   "desire with hash + flag failed");

	/* The reconciler (and the fleet hook) run on the system workqueue. */
	k_sleep(K_MSEC(50));

	zassert_ok(run_cmd("meshtastic fleet status", &out), "status failed");
	zassert_not_null(strstr(out, "class 1 -> 0.3.0"), "class 1's row must read back");
	zassert_not_null(strstr(out, "class 2 -> 0.2.7"), "class 2's row must read back");
	zassert_not_null(strstr(out, "PAUSED"), "the flag must read back");
	zassert_not_null(strstr(out, "2 base rows"), "two classes, two rows");

	zassert_ok(run_cmd("meshtastic fleet pin 0b0b0b0b 0.2.4", &out), "pin failed");
	zassert_ok(run_cmd("meshtastic fleet status", &out), "status failed");
	zassert_not_null(strstr(out, "pin     0x0b0b0b0b"), "the pin must be listed");
	zassert_ok(run_cmd("meshtastic fleet unpin 0b0b0b0b", &out), "unpin failed");
	zassert_ok(run_cmd("meshtastic fleet status", &out), "status failed");
	zassert_not_null(strstr(out, "withdrawn (tombstone)"),
			 "a withdrawn pin is a tombstone, and says so");
}
#endif /* CONFIG_MESHTASTIC_FLEET */

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
	zassert_not_null(strstr(out, "blepeer"),
			 "and BOTH bearers — `lora tx off` alone leaves a pull free to divert "
			 "to a BLE peer link, which is how a bench clear undid itself");
}

#endif /* CONFIG_MESHTASTIC_CLUSTER */

#if defined(CONFIG_MESHTASTIC_LOGRING)
/* ---- logring line bounding ------------------------------------------------
 *
 * `logring <n>` exists because the unbounded form is unusable on the transport
 * that matters most. Over SMP/BLE the whole ring cannot fit one response buffer
 * and the node answers MGMT_ERR.EMSGSIZE — and a node reachable only by BLE is
 * precisely a node that has left the bench, which is when its log is most worth
 * reading and its USB port least available.
 *
 * The counting is a backwards walk over a ring that may have wrapped, with a
 * trailing newline that terminates the last line rather than starting one. Every
 * part of that is an off-by-one waiting to happen, and an off-by-one here is
 * invisible: you get almost the right answer, from a node you cannot check.
 */
LOG_MODULE_REGISTER(shell_test_logring, LOG_LEVEL_INF);

static void logring_fill(int n)
{
	for (int i = 0; i < n; i++) {
		LOG_INF("LRMARK%03d", i);
	}
}

ZTEST(meshtastic_shell, test_logring_bounds_output_to_the_last_n_lines)
{
	const char *out;

	logring_fill(12);

	/* Ask for 3. The last marker must be there and an early one must not —
	 * asserting both directions, because "shows the newest" and "drops the
	 * oldest" are separate claims and a wrong offset can satisfy one alone. */
	zassert_ok(run_cmd("logring 3", &out), "logring 3 failed");
	zassert_not_null(strstr(out, "LRMARK011"), "the newest line must survive bounding");
	zassert_is_null(strstr(out, "LRMARK000"),
			"a 3-line request must not carry 12 lines of history");

	/* The header reports both numbers, so "bounded" is visible rather than
	 * inferred from the absence of output. */
	zassert_not_null(strstr(out, " of "), "header should read '<shown> of <total> bytes'");
}

ZTEST(meshtastic_shell, test_logring_line_count_is_exact)
{
	const char *out;
	int found = 0;

	logring_fill(12);

	/* Exactly 5 markers for a 5-line request. The trailing-newline rule is
	 * what this pins: counting the terminator of the final line would quietly
	 * return four. */
	zassert_ok(run_cmd("logring 5", &out), "logring 5 failed");
	for (int i = 0; i < 12; i++) {
		char needle[16];

		snprintk(needle, sizeof(needle), "LRMARK%03d", i);
		if (strstr(out, needle) != NULL) {
			found++;
		}
	}
	zassert_equal(found, 5, "expected exactly 5 markers in `logring 5`, got %d", found);
}

ZTEST(meshtastic_shell, test_logring_more_lines_than_held_returns_everything)
{
	const char *out, *h;
	unsigned long shown, total;

	logring_fill(4);

	/* Asking for more than exists is not an error and must not truncate: the
	 * whole ring IS the complete answer to "the last 9999 lines".
	 *
	 * Asserted from the HEADER, not by looking for a marker in the body. The
	 * unbounded dump is larger than the dummy backend's capture buffer, so the
	 * body gets clipped and the newest line — the one a body check would look
	 * for — is the first casualty. Which is the whole reason `logring <n>`
	 * exists: it is the same size limit as SMP's, met in the test harness. */
	zassert_ok(run_cmd("logring 9999", &out), "logring with a large count failed");

	h = strstr(out, "log ring: ");
	zassert_not_null(h, "expected the '<shown> of <total> bytes' header");
	h += strlen("log ring: ");
	shown = strtoul(h, (char **)&h, 10);
	h = strstr(h, "of ");
	zassert_not_null(h, "malformed header");
	total = strtoul(h + 3, NULL, 10);

	zassert_true(total > 0UL, "the ring should hold something after 4 log lines");
	zassert_equal(shown, total,
		      "a count above the lines held must return the whole ring (%lu of %lu)",
		      shown, total);
}

ZTEST(meshtastic_shell, test_logring_rejects_a_bad_count)
{
	const char *out;

	/* Non-numeric and zero are usage errors, not "show me everything" — a
	 * silently-ignored argument on a size-capped transport reads as the node
	 * failing rather than the request being malformed. */
	zassert_not_equal(run_cmd("logring banana", &out), 0, "non-numeric count must fail");
	zassert_not_null(strstr(out, "usage:"), "and should say how to call it");
	zassert_not_equal(run_cmd("logring 0", &out), 0, "zero lines is not a request");
}
#endif /* CONFIG_MESHTASTIC_LOGRING */

#if defined(CONFIG_MESHTASTIC_SCANNER_AUTOSTART)
/* ---- an autostart image comes up mute ------------------------------------
 *
 * The defect this pins: the TX gate used to close when the SWEEP started, and
 * the sweep starts at the tail of init because starting it earlier fights the
 * radio bring-up. On hardware that left an ~11 s window at every boot — the
 * sweep began at uptime 11 s and the node transmitted at ~8 s, heard by two
 * neighbours, on every cold boot.
 *
 * What made it durable is that the refusal counter read ZERO throughout. The
 * counter only counts what the gate refuses, so a frame sent before the gate
 * exists is invisible to it — and the listener's central claim, that it did not
 * perturb what it measured, rests on that zero.
 */
/*
 * Sampled from a SYS_INIT, which is the only vantage point that can see this.
 *
 * The fixture calls meshtastic_init(), and init's tail is what autostarts the
 * sweep — so by the time any ZTEST body runs the gate is shut either way, and a
 * test that merely asserts "the gate is shut" passes with the fix reverted. It
 * was written that way first and proved exactly nothing. A SYS_INIT runs before
 * main(), hence before the fixture, so what it records is the state the node
 * boots in rather than the state init leaves behind.
 */
static bool gate_shut_before_init;

static int sample_boot_gate(void)
{
	gate_shut_before_init = meshtastic_scanner_active();
	return 0;
}
SYS_INIT(sample_boot_gate, APPLICATION, 99);

ZTEST(meshtastic_shell, test_scanner_autostart_a_comes_up_with_tx_refused)
{
	const char *out;

	zassert_true(gate_shut_before_init,
		     "an autostart image must refuse TX from BOOT, not from the moment "
		     "init's tail starts the sweep — that gap is ~11 s on hardware");

	zassert_ok(run_cmd("meshtastic scan status", &out), "scan status failed");
	zassert_not_null(strstr(out, "REFUSED"), "status must report the gate as shut");
}

/*
 * The a_/b_ in these two names is load-bearing: ztest runs in NAME order, and
 * the second test opens the gate the first one needs shut. Without the ordering
 * they sort the other way round ('_' < 'u') and the first fails for a reason
 * that has nothing to do with what it checks.
 */
ZTEST(meshtastic_shell, test_scanner_autostart_b_gate_can_be_reopened)
{
	const char *out;

	/* Recoverability, and the reason stop() no longer keys on `sweeping`.
	 * "Shut but not sweeping" is reachable — an autostart image boots in it and
	 * stays there if the sweep fails to start — and an early return on !sweeping
	 * made it PERMANENT: a node mute for the rest of its life, no way back short
	 * of a reflash. */
	zassert_true(meshtastic_scanner_active(), "precondition: the gate is shut");

	zassert_ok(run_cmd("meshtastic scan stop", &out), "scan stop failed");
	zassert_false(meshtastic_scanner_active(), "stop must reopen a boot-shut gate");

	zassert_ok(run_cmd("meshtastic scan status", &out), "scan status failed");
	zassert_not_null(strstr(out, "allowed"), "and say so");
}

/*
 * A survey's preset list comes from the build, because the runtime list is
 * RAM-only and a power-cycled listener must resume the SAME survey. The
 * scenario (testcase.yaml, scanner_autostart_list) spells the names in mixed
 * case with a bogus one in the middle: matching is case-insensitive, and an
 * unknown name is skipped rather than failing the whole list.
 */
ZTEST(meshtastic_shell, test_scanner_autostart_c_list_from_build)
{
	meshtastic_Config_LoRaConfig_ModemPreset got[MESHTASTIC_SCANNER_MAX_PRESETS];
	int n;

	if (CONFIG_MESHTASTIC_SCANNER_AUTOSTART_PRESETS[0] == '\0') {
		ztest_test_skip();
	}

	n = meshtastic_scanner_get_presets(got, ARRAY_SIZE(got));
	zassert_equal(n, 3, "three names resolve, the bogus one is skipped (got %d)", n);
	zassert_equal(got[0], meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO);
	zassert_equal(got[1], meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_TURBO);
	zassert_equal(got[2], meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO);
}

#if defined(CONFIG_MESHTASTIC_SETTINGS)
#include <zephyr/settings/settings.h>

struct pin_read {
	uint8_t raw[MESHTASTIC_SCANNER_MAX_PRESETS];
	ssize_t len;
};

static int pin_read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		       void *param)
{
	struct pin_read *r = param;

	ARG_UNUSED(key);
	r->len = read_cb(cb_arg, r->raw, MIN(len, sizeof(r->raw)));
	return 0;
}

/* What a boot does to the scanner, without the reboot: the sweep is stopped, the list is
 * back at the full set as after the scanner's own init, and the autostart runs. */
static int scanner_boots_again(meshtastic_Config_LoRaConfig_ModemPreset *got, size_t cap)
{
	(void)meshtastic_scanner_stop();
	zassert_ok(meshtastic_scanner_set_presets(NULL, 0U));
	meshtastic_scanner_autostart();
	return meshtastic_scanner_get_presets(got, cap);
}

/* Named to run after the three above: it leaves the build's list in place, as it found it. */
ZTEST(meshtastic_shell, test_scanner_autostart_d_a_pinned_list_survives_a_boot)
{
	meshtastic_Config_LoRaConfig_ModemPreset got[MESHTASTIC_SCANNER_MAX_PRESETS];
	struct pin_read r = {0};
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic scan presets MediumFast", &out), "%s", out);
	zassert_not_null(strstr(out, "stored"), "%s", out);
	zassert_ok(settings_load_subtree_direct("mtscan/presets", pin_read_cb, &r));
	zassert_equal(r.len, 1, "one byte per preset");
	zassert_equal(r.raw[0], (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST);

	zassert_equal(scanner_boots_again(got, ARRAY_SIZE(got)), 1,
		      "the stored list, not the build's three");
	zassert_equal(got[0], meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST);
	zassert_true(meshtastic_scanner_sweeping(), "and the survey is running on it");

	/* `all` forgets the pin: the record goes, and a boot is the build's again. */
	zassert_ok(run_cmd("meshtastic scan presets all", &out), "%s", out);
	memset(&r, 0, sizeof(r));
	zassert_ok(settings_load_subtree_direct("mtscan/presets", pin_read_cb, &r));
	zassert_equal(r.len, 0, "no record left");
	zassert_equal(scanner_boots_again(got, ARRAY_SIZE(got)), 3, "the build's list");
	zassert_equal(got[0], meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO);
}

/* The role intents (include/zephyr/meshtastic/intents.h): a pinned list is one, `meshtastic
 * intents` lists it, and clearing forgets the record and the pin. The shell's `clear` reboots
 * (native_sim cannot come back from that), so the module's own clear is what is driven here;
 * the shell's listing is checked before and after. */
ZTEST(meshtastic_shell, test_scanner_autostart_e_a_pinned_list_is_an_intent_and_clearing_forgets_it)
{
	meshtastic_Config_LoRaConfig_ModemPreset got[MESHTASTIC_SCANNER_MAX_PRESETS];
	struct pin_read r = {0};
	const char *out = NULL;

	zassert_ok(run_cmd("meshtastic intents", &out), "%s", out);
	zassert_not_null(strstr(out, "scan     none"), "nothing pinned yet: %s", out);
	zassert_not_null(strstr(out, "intents: 0 set"), "%s", out);

	zassert_ok(run_cmd("meshtastic scan presets MediumFast", &out), "%s", out);
	zassert_ok(run_cmd("meshtastic intents", &out), "%s", out);
	zassert_not_null(strstr(out, "scan     set   1 preset(s) pinned"), "%s", out);
	zassert_not_null(strstr(out, "intents: 1 set"), "%s", out);
	zassert_equal(meshtastic_intents_set(), 1U, "");

	zassert_equal(meshtastic_intents_clear(), 1, "one intent cleared");
	zassert_ok(settings_load_subtree_direct("mtscan/presets", pin_read_cb, &r));
	zassert_equal(r.len, 0, "the record is gone");
	zassert_equal(meshtastic_intents_set(), 0U, "");
	zassert_equal(scanner_boots_again(got, ARRAY_SIZE(got)), 3, "a boot is the build's list");
	zassert_ok(run_cmd("meshtastic intents clear", &out), "%s", out);
	zassert_not_null(strstr(out, "nothing to clear"), "%s", out);
}
#endif /* CONFIG_MESHTASTIC_SETTINGS */
#endif /* CONFIG_MESHTASTIC_SCANNER_AUTOSTART */

#if defined(CONFIG_MESHTASTIC_RELAY) || defined(CONFIG_MESHTASTIC_RELAY_EAR)
#include "meshtastic_relay.h"
#endif
#if defined(CONFIG_MESHTASTIC_RELAY)

/* `meshtastic relay`: off at boot, `dir in` turns it on, the v1-refused
 * directions say why and do not stick, and reset returns to boot state. */
ZTEST(meshtastic_shell, test_relay_commands)
{
	const char *out;

	meshtastic_relay_reset();
	zassert_ok(run_cmd("meshtastic relay", &out));
	zassert_not_null(strstr(out, "direction: off"), "boot direction: %s", out);

	zassert_ok(run_cmd("meshtastic relay dir in", &out));
	zassert_equal(meshtastic_relay_get_direction(), MESHTASTIC_RELAY_INBOUND);

	zassert_not_equal(run_cmd("meshtastic relay dir both", &out), 0);
	zassert_not_null(strstr(out, "ear that transmits"), "refusal reason: %s", out);
	zassert_equal(meshtastic_relay_get_direction(), MESHTASTIC_RELAY_INBOUND,
		      "a refused direction must not stick");

	zassert_not_equal(run_cmd("meshtastic relay dir sideways", &out), 0);
	zassert_ok(run_cmd("meshtastic relay ignore 0d0d0d0d", &out));
	zassert_not_null(strstr(out, "0x0d0d0d0d"), "%s", out);
	zassert_not_equal(run_cmd("meshtastic relay ignore zz", &out), 0);

	zassert_ok(run_cmd("meshtastic relay show", &out));
	zassert_not_null(strstr(out, "considered 0"), "%s", out);

	zassert_ok(run_cmd("meshtastic relay reset", &out));
	zassert_equal(meshtastic_relay_get_direction(), MESHTASTIC_RELAY_OFF);
}
#endif /* CONFIG_MESHTASTIC_RELAY */

#if defined(CONFIG_MESHTASTIC_RELAY_EAR)
/* `meshtastic ear`: set the peer, see it, stop forwarding with 0. */
ZTEST(meshtastic_shell, test_ear_commands)
{
	const char *out;

	meshtastic_relay_ear_reset();
	zassert_ok(run_cmd("meshtastic ear", &out));
	zassert_not_null(strstr(out, "not forwarding"), "%s", out);
	zassert_ok(run_cmd("meshtastic ear peer 0e0e0e0e", &out));
	zassert_equal(meshtastic_relay_ear_get_peer(), 0x0e0e0e0eU);
	zassert_ok(run_cmd("meshtastic ear show", &out));
	zassert_not_null(strstr(out, "0x0e0e0e0e"), "%s", out);
	zassert_not_null(strstr(out, "transmit: possible"), "%s", out);
	zassert_ok(run_cmd("meshtastic ear peer 0", &out));
	zassert_equal(meshtastic_relay_ear_get_peer(), 0U);
}
#endif /* CONFIG_MESHTASTIC_RELAY_EAR */

/* ---- `meshtastic state`: the machine-readable line ------------------------- */

/* Run `meshtastic state` and return its JSON text in @p json (NUL-terminated), having
 * checked the frame: one `~S{...}*hhhh` line whose CRC-16 matches the text. */
static void state_json(char *json, size_t cap)
{
	const char *out = NULL;
	const char *start;
	const char *star;
	unsigned long want;
	size_t len;

	zassert_ok(run_cmd("meshtastic state", &out), "state failed");
	start = strstr(out, "~S{");
	zassert_not_null(start, "no sentinel: %s", out);
	start += 2;
	star = strrchr(start, '*');
	zassert_not_null(star, "no checksum: %s", out);
	len = (size_t)(star - start);
	zassert_true(len < cap, "state line is %u bytes", (unsigned int)len);
	zassert_equal(start[len - 1U], '}', "the JSON must end the framed text");
	want = strtoul(star + 1, NULL, 16);
	zassert_equal(crc16_itu_t(0U, (const uint8_t *)start, len), (uint16_t)want,
		      "CRC over the JSON text must match the one printed");
	memcpy(json, start, len);
	json[len] = '\0';
}

ZTEST(meshtastic_shell, test_state_is_one_framed_line_with_the_node_s_facts)
{
	char json[1024];
	char want[48];

	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "{\"v\":1,\"build\":\""), "%s", json);
	snprintk(want, sizeof(want), "\"class\":%u,", (unsigned int)CONFIG_MESHTASTIC_FLEET_CLASS);
	zassert_not_null(strstr(json, want), "%s", json);
	snprintk(want, sizeof(want), "\"id\":\"0x%08x\"", meshtastic_get_node_id());
	zassert_not_null(strstr(json, want), "%s", json);
	zassert_not_null(strstr(json, "\"board\":\"" CONFIG_BOARD "\""), "%s", json);
	zassert_not_null(strstr(json, "\"lora\":{\"preset\":"), "%s", json);
	zassert_not_null(strstr(json, "\"dev\":{\"role\":"), "%s", json);
	zassert_not_null(strstr(json, "\"ch\":[{\"i\":0,\"r\":1,"), "the primary: %s", json);
	/* This node was started as the sample app starts one: meshtastic_init() given the
	 * well-known key as its 16 bytes. Its primary must still hold the one-byte form, the
	 * one the client apps show without a lock. */
	zassert_not_null(strstr(json, "\"h\":8,\"k\":\"s1\""), "the default key's short form: %s",
			 json);
	zassert_is_null(strchr(json, '\n'), "one line");
}

#if defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
ZTEST(meshtastic_shell, test_state_follows_what_is_set_and_never_prints_a_key)
{
	char json[1024];
	meshtastic_Config before;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &before));
	set_managed(false);

	/* The default key in its one-byte form is named as such. */
	zassert_ok(run_cmd("meshtastic channel set 1 name side role secondary psk default", NULL));
	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "{\"i\":1,\"r\":2,\"n\":\"side\","), "%s", json);
	zassert_not_null(strstr(json, "\"k\":\"s1\""), "%s", json);

	/* A real key is a fingerprint: the same for the same key, different for another,
	 * and the key's bytes are nowhere in the line. */
	zassert_ok(run_cmd("meshtastic channel set 1 psk hex "
			   "00112233445566778899aabbccddeeff", NULL));
	state_json(json, sizeof(json));
	{
		const char *k = strstr(json, "\"n\":\"side\"");
		char fp1[16] = {0};
		char fp2[16] = {0};

		zassert_not_null(k);
		k = strstr(k, "\"k\":\"c:");
		zassert_not_null(k, "a fingerprint: %s", json);
		memcpy(fp1, k + 5, 10);
		zassert_is_null(strstr(json, "00112233"), "the key itself must not appear");
		zassert_is_null(strstr(json, "ccddeeff"), "the key itself must not appear");

		zassert_ok(run_cmd("meshtastic channel set 1 psk hex "
				   "00112233445566778899aabbccddee00", NULL));
		state_json(json, sizeof(json));
		k = strstr(strstr(json, "\"n\":\"side\""), "\"k\":\"c:");
		memcpy(fp2, k + 5, 10);
		zassert_true(strcmp(fp1, fp2) != 0, "another key, another fingerprint");
	}

	/* Stored radio settings appear as stored. */
	zassert_ok(run_cmd("meshtastic lora power 2", NULL));
	zassert_ok(run_cmd("meshtastic lora tx off", NULL));
	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "\"pwr\":2,\"tx\":0}"), "%s", json);

	/* A name with a quote and a backslash stays valid JSON. */
	zassert_ok(run_cmd("meshtastic owner set \"a\\\"b\" ab", NULL));
	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "\"own\":{\"l\":\"a\\\"b\",\"s\":\"ab\"}"), "%s", json);

	zassert_ok(run_cmd("meshtastic channel disable 1", NULL));
	zassert_ok(run_cmd("meshtastic lora tx on", NULL));
	state_json(json, sizeof(json));
	zassert_is_null(strstr(json, "\"i\":1,"), "a disabled slot is not listed: %s", json);
	zassert_ok(meshtastic_config_store_set_config(&before));
}
#endif

#if defined(CONFIG_MESHTASTIC_SCANNER)
ZTEST(meshtastic_shell, test_state_reports_the_scanner)
{
	char json[1024];

	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "\"scanner\""), "the feature: %s", json);
	zassert_not_null(strstr(json, "\"scan\":{\"shut\":"), "%s", json);
	zassert_not_null(strstr(json, "\"presets\":["), "%s", json);
}
#endif

/* ---- `meshtastic api`: the phone API as text ------------------------------- */

#if defined(CONFIG_MESHTASTIC_SHELL_PHONEAPI)

#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/sys/base64.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic_phoneapi.h"

/* One answer, parsed: the status line and the frames in front of it. */
struct api_reply {
	char status;      /* '.', '+', ':' or '!' */
	unsigned int n;   /* the number behind it ('!': 0) */
	char why[8];      /* '!': the word behind it */
	unsigned int frames;
};

#define API_FRAMES_MAX 8U
static meshtastic_FromRadio api_frames[API_FRAMES_MAX];
static uint32_t api_next_id = 0x0A710000U;

/* Run one command and parse what it printed. Every frame line must be whole: sentinel,
 * base64, a CRC-16 over the base64 text, and a FromRadio inside. */
static void api_cmd(const char *cmd, struct api_reply *r)
{
	static uint8_t raw[MESHTASTIC_API_FRAME_MAX];
	const char *out = NULL;
	const char *p;

	*r = (struct api_reply){0};
	(void)run_cmd(cmd, &out);
	p = strstr(out, "~P");
	zassert_not_null(p, "no answer to `%.40s`: %s", cmd, out);

	while (p != NULL) {
		const char *eol = strchr(p, '\n');
		char kind = p[2];

		zassert_not_null(eol, "an unterminated line: %s", p);
		if (kind == '.' || kind == '+' || kind == ':') {
			r->status = kind;
			r->n = (unsigned int)strtoul(p + 3, NULL, 10);
		} else if (kind == '!') {
			size_t len = MIN((size_t)(eol - (p + 3)), sizeof(r->why) - 1U);

			r->status = kind;
			memcpy(r->why, p + 3, len);
			while (len > 0U && (r->why[len - 1U] == '\r' || r->why[len - 1U] == '\n')) {
				r->why[--len] = '\0';
			}
		} else {
			const char *star = memchr(p, '*', (size_t)(eol - p));
			pb_istream_t is;
			size_t n = 0U;

			zassert_equal(r->status, '\0', "a frame after the status line: %s", out);
			zassert_not_null(star, "a frame without a checksum: %s", p);
			zassert_equal(crc16_itu_t(0U, (const uint8_t *)p + 2, (size_t)(star - p) - 2U),
				      (uint16_t)strtoul(star + 1, NULL, 16),
				      "CRC over the base64 text must match");
			zassert_ok(base64_decode(raw, sizeof(raw), &n, (const uint8_t *)p + 2,
						 (size_t)(star - p) - 2U),
				   "base64");
			zassert_true(r->frames < API_FRAMES_MAX, "more frames than the burst");
			is = pb_istream_from_buffer(raw, n);
			api_frames[r->frames] = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
			zassert_true(pb_decode(&is, meshtastic_FromRadio_fields,
					       &api_frames[r->frames]),
				     "a frame must be a FromRadio");
			r->frames++;
		}
		p = strstr(eol, "~P");
	}
	zassert_not_equal(r->status, '\0', "no status line: %s", out);
	if (r->status == '.' || r->status == '+') {
		zassert_equal(r->n, r->frames, "the status line counts the frames");
	}
}

/* Send a ToRadio in parts of at most @p part bytes (a multiple of 3). */
static void api_send(const meshtastic_ToRadio *to, size_t part, struct api_reply *r)
{
	static uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	static char cmd[64 + MESHTASTIC_API_FRAME_MAX * 4U / 3U];
	char b64[MESHTASTIC_API_FRAME_MAX * 4U / 3U + 8U];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	size_t off = 0U;
	size_t n;

	zassert_true(pb_encode(&os, meshtastic_ToRadio_fields, to), "ToRadio encode");
	while (os.bytes_written - off > part) {
		zassert_ok(base64_encode((uint8_t *)b64, sizeof(b64), &n, &buf[off], part));
		snprintk(cmd, sizeof(cmd), "meshtastic api + %s", b64);
		api_cmd(cmd, r);
		off += part;
		zassert_equal(r->status, ':', "a part is acknowledged");
		zassert_equal(r->n, off, "with the bytes held so far");
	}
	zassert_ok(base64_encode((uint8_t *)b64, sizeof(b64), &n, &buf[off],
				 os.bytes_written - off));
	snprintk(cmd, sizeof(cmd), "meshtastic api %s %04x", b64,
		 crc16_itu_t(0U, buf, os.bytes_written));
	api_cmd(cmd, r);
}

static void api_send_admin(const meshtastic_AdminMessage *a, size_t part, struct api_reply *r)
{
	static meshtastic_ToRadio to;
	pb_ostream_t os;

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_packet_tag;
	to.packet.to = meshtastic_get_node_id();
	to.packet.id = api_next_id++;
	to.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	to.packet.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	os = pb_ostream_from_buffer(to.packet.decoded.payload.bytes,
				    sizeof(to.packet.decoded.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, a), "AdminMessage encode");
	to.packet.decoded.payload.size = (pb_size_t)os.bytes_written;
	api_send(&to, part, r);
}

/* Serve until nothing is waiting, so a test starts from an empty transport. */
static void api_drain(void)
{
	struct api_reply r;

	do {
		api_cmd("meshtastic api", &r);
	} while (r.status == '+' || r.frames > 0U);
}

static void api_end_session(void)
{
	static meshtastic_ToRadio to;
	struct api_reply r;

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_disconnect_tag;
	to.disconnect = true;
	api_send(&to, 150U, &r);
	api_drain();
}

ZTEST(meshtastic_shell, test_api_serves_the_config_stream_in_bursts)
{
	static meshtastic_ToRadio to;
	struct api_reply r;
	unsigned int total = 0U;
	unsigned int bursts = 0U;
	bool my_info = false;
	bool complete = false;

	api_drain();
	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_want_config_id_tag;
	to.want_config_id = 42U;
	api_send(&to, 150U, &r);

	/* The request's own answer already carries the first frames. */
	zassert_true(r.frames > 0U, "a send serves frames in the same answer");
	zassert_equal(api_frames[0].which_payload_variant, meshtastic_FromRadio_my_info_tag,
		      "the stream opens with MyNodeInfo");
	zassert_equal(api_frames[0].my_info.my_node_num, meshtastic_get_node_id());
	my_info = true;

	while (true) {
		zassert_true(r.frames <= CONFIG_MESHTASTIC_SHELL_PHONEAPI_BURST,
			     "never more than the burst in one answer");
		total += r.frames;
		for (unsigned int i = 0U; i < r.frames; i++) {
			zassert_false(complete, "nothing of the stream after its end");
			if (api_frames[i].which_payload_variant ==
			    meshtastic_FromRadio_config_complete_id_tag) {
				zassert_equal(api_frames[i].config_complete_id, 42U);
				complete = true;
			}
		}
		if (complete) {
			break;
		}
		zassert_equal(r.status, '+', "more is waiting while the stream runs");
		zassert_true(++bursts < 200U, "the stream must end");
		api_cmd("meshtastic api", &r);
	}
	zassert_true(my_info && complete);
	zassert_true(total > CONFIG_MESHTASTIC_SHELL_PHONEAPI_BURST,
		     "the stream is longer than one burst (%u frames)", total);

	api_drain();
	api_cmd("meshtastic api", &r);
	zassert_equal(r.status, '.', "drained");
	zassert_equal(r.frames, 0U);
	api_end_session();
}

ZTEST(meshtastic_shell, test_api_request_and_reply_are_one_command)
{
	static meshtastic_ToRadio to;
	struct api_reply r;
	bool seen = false;

	api_drain();
	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_heartbeat_tag;
	api_send(&to, 150U, &r);
	for (unsigned int i = 0U; i < r.frames; i++) {
		seen |= api_frames[i].which_payload_variant == meshtastic_FromRadio_queueStatus_tag;
	}
	zassert_true(seen, "a heartbeat's QueueStatus comes back in the same answer");
}

ZTEST(meshtastic_shell, test_api_admin_write_in_parts_reaches_the_store)
{
	static meshtastic_AdminMessage a;
	struct api_reply r;
	char before[40];

	api_drain();
	set_managed(false);
	strncpy(before, meshtastic_config_store_long_name(), sizeof(before) - 1U);
	before[sizeof(before) - 1U] = '\0';

	a = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	a.which_payload_variant = meshtastic_AdminMessage_set_owner_tag;
	strcpy(a.payload_variant.set_owner.long_name, "over the wire, in three parts");
	strcpy(a.payload_variant.set_owner.short_name, "wire");
	/* 15 bytes a part: the packet is some 50 bytes, so this is several commands. */
	api_send_admin(&a, 15U, &r);
	zassert_true(r.status == '.' || r.status == '+', "accepted: !%s", r.why);
	zassert_str_equal(meshtastic_config_store_long_name(), "over the wire, in three parts");
	zassert_str_equal(meshtastic_config_store_short_name(), "wire");

	strcpy(a.payload_variant.set_owner.long_name, before);
	strcpy(a.payload_variant.set_owner.short_name, "back");
	api_send_admin(&a, 150U, &r);
	zassert_str_equal(meshtastic_config_store_long_name(), before);
	api_drain();
}

/* The transport is the phone API, so the phone API's gate holds: a managed node refuses
 * local admin here as it does over Bluetooth. */
ZTEST(meshtastic_shell, test_api_managed_node_refuses_admin_writes)
{
	static meshtastic_AdminMessage a;
	struct api_reply r;
	char before[40];

	api_drain();
	strncpy(before, meshtastic_config_store_long_name(), sizeof(before) - 1U);
	before[sizeof(before) - 1U] = '\0';
	set_managed(true);

	a = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	a.which_payload_variant = meshtastic_AdminMessage_set_owner_tag;
	strcpy(a.payload_variant.set_owner.long_name, "must not land");
	strcpy(a.payload_variant.set_owner.short_name, "no");
	api_send_admin(&a, 150U, &r);
	zassert_str_equal(meshtastic_config_store_long_name(), before,
			  "a managed node must not take a local admin write");
	set_managed(false);
	api_drain();
}

ZTEST(meshtastic_shell, test_api_refuses_what_it_cannot_trust)
{
	static meshtastic_ToRadio to;
	static char cmd[900];
	struct api_reply r;

	api_drain();

	api_cmd("meshtastic api no-base64 0000", &r);
	zassert_equal(r.status, '!');
	zassert_str_equal(r.why, "b64");

	/* A good frame with the wrong checksum is not handled: no QueueStatus follows. */
	api_cmd("meshtastic api OgA= ffff", &r); /* heartbeat {} */
	zassert_equal(r.status, '!');
	zassert_str_equal(r.why, "crc");
	api_cmd("meshtastic api", &r);
	zassert_equal(r.frames, 0U, "a refused frame did nothing");

	api_cmd("meshtastic api OgA= zz", &r);
	zassert_str_equal(r.why, "crc", "a checksum that is not hex");

	api_cmd("meshtastic api a b c", &r);
	zassert_str_equal(r.why, "args");
	api_cmd("meshtastic api OgA=", &r);
	zassert_str_equal(r.why, "args", "a frame without its checksum");

	/* More than a ToRadio can be: 150-byte parts until the buffer refuses one. */
	strcpy(cmd, "meshtastic api + ");
	memset(cmd + strlen("meshtastic api + "), 'A', 200U);
	cmd[strlen("meshtastic api + ") + 200U] = '\0';
	for (unsigned int i = 0U; i < 3U; i++) {
		api_cmd(cmd, &r);
		zassert_equal(r.status, ':', "part %u fits", i);
	}
	api_cmd(cmd, &r);
	zassert_str_equal(r.why, "len", "the fourth 150 bytes do not fit in 512");

	/* A refusal drops what was held: the next frame stands alone. */
	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_heartbeat_tag;
	api_send(&to, 150U, &r);
	zassert_equal(r.status, '.', "!%s", r.why);
	zassert_equal(r.frames, 1U, "one QueueStatus for one heartbeat");

	/* So does a poll in between. */
	api_cmd("meshtastic api + AAAA", &r);
	zassert_equal(r.status, ':');
	zassert_equal(r.n, 3U);
	api_cmd("meshtastic api", &r);
	api_cmd("meshtastic api + AAAA", &r);
	zassert_equal(r.n, 3U, "serving frames dropped the part that was held");
	api_cmd("meshtastic api", &r);
}

ZTEST(meshtastic_shell, test_api_disconnect_ends_the_config_stream)
{
	static meshtastic_ToRadio to;
	struct api_reply r;

	api_drain();
	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_want_config_id_tag;
	to.want_config_id = 7U;
	api_send(&to, 150U, &r);
	zassert_equal(r.status, '+', "mid-stream");

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_disconnect_tag;
	to.disconnect = true;
	api_send(&to, 150U, &r);
	do {
		for (unsigned int i = 0U; i < r.frames; i++) {
			zassert_not_equal(api_frames[i].which_payload_variant,
					  meshtastic_FromRadio_config_complete_id_tag,
					  "the stream a client left must not run on");
			zassert_not_equal(api_frames[i].which_payload_variant,
					  meshtastic_FromRadio_config_tag);
		}
		api_cmd("meshtastic api", &r);
	} while (r.status == '+' || r.frames > 0U);
}

#else /* !CONFIG_MESHTASTIC_SHELL_PHONEAPI */

#if !defined(CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE)
/* A build with the shell's writes compiled out must not grow them back through the phone
 * API: the command is not there at all. */
ZTEST(meshtastic_shell, test_api_is_absent_without_config_write)
{
	const char *out = NULL;

	zassert_not_equal(run_cmd("meshtastic api", &out), 0, "the command must not exist");
	zassert_is_null(strstr(out, "~P"), "%s", out);
}
#endif

#endif /* CONFIG_MESHTASTIC_SHELL_PHONEAPI */

/* ---- `meshtastic attach preset`: names, numbers, and nothing else ------------ */

#if defined(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN)

#include "meshtastic_attachment.h"
#include "meshtastic_attachment_codec.h"

#define SHELL_HEAD_NODE 0x00E10001U

/* agents-pcs2.12: `attach preset 1 ShortFast` used to parse the name as the number 0 and
 * put the head on LongFast. */
ZTEST(meshtastic_shell, test_attach_preset_takes_a_name_or_a_number_and_refuses_junk)
{
	uint8_t env[MESHTASTIC_ATTACHMENT_ENV_MAX];
	const struct meshtastic_attachment_status st = {
		.preset = (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,
		.flags = MESHTASTIC_ATTACHMENT_ST_IS_HEAD,
		.hwid = SHELL_HEAD_NODE,
	};
	const char *out = NULL;
	char json[1024];
	int len = meshtastic_attachment_encode_status(&st, env, sizeof(env));

	/* A head, admitted by the allow-list: there is no link in this build to trust. */
	zassert_ok(meshtastic_attachment_allow_add(SHELL_HEAD_NODE));
	zassert_ok(meshtastic_attachment_ingest(SHELL_HEAD_NODE, env, (size_t)len));
	zassert_equal(meshtastic_attachment_id_for_node(SHELL_HEAD_NODE), 1U);

	zassert_not_equal(run_cmd("meshtastic attach preset 1 Bogus", &out), 0);
	zassert_not_null(strstr(out, "unknown preset"), "%s", out);
	zassert_not_equal(run_cmd("meshtastic attach preset 1 6x", &out), 0, "a number and more");
	zassert_not_equal(run_cmd("meshtastic attach preset 1 99", &out), 0, "no such preset");
	zassert_not_equal(run_cmd("meshtastic attach preset one 6", &out), 0, "an id is a number");
	zassert_equal(meshtastic_attachment_wanted_preset(SHELL_HEAD_NODE),
		      MESHTASTIC_PRESET_UNKNOWN, "a refused command stores nothing");

	zassert_ok(run_cmd("meshtastic attach preset 1 ShortFast", &out), "%s", out);
	zassert_equal(meshtastic_attachment_wanted_preset(SHELL_HEAD_NODE),
		      (uint8_t)meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,
		      "the name is the preset, not the number 0");
	zassert_ok(run_cmd("meshtastic attach list", &out));
	zassert_not_null(strstr(out, "wanted preset 6  (NOT there yet)"), "%s", out);

	/* The state line says what the head reports and what is wanted of it. */
	state_json(json, sizeof(json));
	zassert_not_null(strstr(json, "\"node\":\"0x00e10001\",\"p\":4,\"w\":6,"), "%s", json);

	zassert_ok(run_cmd("meshtastic attach preset 1 4", &out), "a number still works");
	zassert_equal(meshtastic_attachment_wanted_preset(SHELL_HEAD_NODE), 4U);
	zassert_ok(run_cmd("meshtastic attach preset 1 none", &out));
	zassert_equal(meshtastic_attachment_wanted_preset(SHELL_HEAD_NODE),
		      MESHTASTIC_PRESET_UNKNOWN);

	zassert_ok(meshtastic_attachment_forget(1U));
	meshtastic_attachment_allow_clear();
}

#endif /* CONFIG_MESHTASTIC_ATTACHMENT_BRAIN */
