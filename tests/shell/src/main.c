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
