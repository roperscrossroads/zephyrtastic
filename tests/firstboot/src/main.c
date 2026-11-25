/* SPDX-License-Identifier: GPL-3.0
 *
 * What a freshly flashed node does. The whole stack on the sim radio with the
 * real NVS behind the settings subsystem; a "boot" is staged the way the
 * lockdown_store suite stages one (RAM store back to its seed, subtree loaded,
 * apply_core), because a process restart is not available here.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>
#include <psa/crypto.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_settings.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static struct meshtastic_config cfg = {
	.node_id = 0x11223344U,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "fresh",
	.short_name = "frsh",
};

struct raw {
	uint8_t buf[64];
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

static void flush_all(void)
{
	zassert_ok(settings_save_subtree("meshtastic"), "");
}

/* A cold boot's view: the RAM store back to its seed, the subtree loaded, applied. */
static void stage_boot(void)
{
	zassert_ok(meshtastic_config_store_seed(&cfg), "");
	zassert_ok(settings_load_subtree("meshtastic"), "");
	zassert_ok(meshtastic_config_store_apply_core(), "");
}

static bool a_text_goes_out(void)
{
	struct meshtastic_packet pkt = {
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_TEXT_MESSAGE,
		.payload = (const uint8_t *)"hi",
		.payload_len = 2U,
	};
	uint32_t before = mt.status.tx_packets;

	(void)meshtastic_send_packet(&pkt, K_NO_WAIT);
	k_sleep(K_MSEC(1500));
	return mt.status.tx_packets != before;
}

static void write_lora_tx(bool on)
{
	meshtastic_Config c;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &c), "");
	c.payload_variant.lora.tx_enabled = on;
	zassert_ok(meshtastic_config_store_set_config(&c), "");
}

static void *suite_setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa");
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	return NULL;
}

ZTEST_SUITE(firstboot, NULL, suite_setup, NULL, NULL, NULL);

/* Runs first (by name): nothing has been written yet. */
ZTEST(firstboot, test_0_a_fresh_node_is_quiet_and_shares_nothing)
{
	meshtastic_Config c;
	meshtastic_ModuleConfig m;
	meshtastic_Channel ch;

	/* The seed says transmit (TX_ENABLED_DEFAULT=y); the hold is what mutes it. */
	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &c), "");
	zassert_true(c.payload_variant.lora.tx_enabled, "the seed itself is the build's guess");
	zassert_false(mt.tx_enabled, "held: the LoRa config was never written");
	zassert_false(a_text_goes_out(), "a fresh node transmits nothing");

	zassert_ok(meshtastic_config_store_get_channel(0U, &ch), "");
	zassert_equal(ch.settings.module_settings.position_precision, 0U, "no position sharing");
	zassert_false(ch.settings.uplink_enabled, "no MQTT uplink");
	zassert_false(ch.settings.downlink_enabled, "no MQTT downlink");

	zassert_ok(meshtastic_config_store_get_module(meshtastic_ModuleConfig_telemetry_tag, &m), "");
	zassert_false(m.payload_variant.telemetry.device_telemetry_enabled, "no device telemetry");
	zassert_false(m.payload_variant.telemetry.environment_measurement_enabled, "no env telemetry");
}

ZTEST(firstboot, test_1_other_writes_do_not_release_the_hold)
{
	meshtastic_User user = meshtastic_User_init_zero;
	meshtastic_Config c;

	(void)snprintk(user.long_name, sizeof(user.long_name), "named");
	(void)snprintk(user.short_name, sizeof(user.short_name), "nmd");
	zassert_ok(meshtastic_config_store_set_owner(&user), "");

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &c), "");
	zassert_ok(meshtastic_config_store_set_config(&c), "a device write");
	flush_all();
	stage_boot();
	zassert_false(mt.tx_enabled, "owner, device and saves do not configure the radio");
}

ZTEST(firstboot, test_2_a_lora_write_releases_it_live_and_across_a_boot)
{
	write_lora_tx(true);
	zassert_true(mt.tx_enabled, "released at once, no reboot");
	zassert_true(a_text_goes_out(), "and the node transmits");

	flush_all();
	stage_boot();
	zassert_true(mt.tx_enabled, "a stored config wins: the stamp comes back from flash");

	/* An explicit off stays off: the hold only ever removes a transmit. */
	write_lora_tx(false);
	zassert_false(mt.tx_enabled, "");
	flush_all();
	stage_boot();
	zassert_false(mt.tx_enabled, "");
	write_lora_tx(true);
}

/* Runs after the above (by name): the node now holds a stored config. */
ZTEST(firstboot, test_z_an_old_node_moves_to_opt_in_once)
{
	meshtastic_Channel pub, priv;
	struct raw r;

	/* The node an older build seeded: public default key at precision 13, and a
	 * private channel at full precision. No marker on flash. */
	zassert_ok(meshtastic_config_store_get_channel(0U, &pub), "");
	pub.settings.module_settings.position_precision = 13U;
	zassert_ok(meshtastic_config_store_set_channel(0U, &pub), "");

	priv = (meshtastic_Channel)meshtastic_Channel_init_zero;
	priv.role = meshtastic_Channel_Role_SECONDARY;
	priv.has_settings = true;
	(void)snprintk(priv.settings.name, sizeof(priv.settings.name), "family");
	priv.settings.psk.size = 16U;
	memset(priv.settings.psk.bytes, 0x5a, 16U);
	priv.settings.has_module_settings = true;
	priv.settings.module_settings.position_precision = 32U;
	zassert_ok(meshtastic_config_store_set_channel(1U, &priv), "");
	flush_all();
	(void)settings_delete("meshtastic/seed/optin");
	zassert_false(raw_read("meshtastic/seed/optin", &r), "no marker on flash");
	/* ...and a cold boot's RAM, where the marker starts at 0 (the save set it). */
	zassert_ok(meshtastic_config_store_setting_set("seed/optin", r.buf, 0U), "");

	stage_boot();
	zassert_ok(meshtastic_config_store_get_channel(0U, &pub), "");
	zassert_ok(meshtastic_config_store_get_channel(1U, &priv), "");
	zassert_equal(pub.settings.module_settings.position_precision, 0U,
		      "the public channel stops sharing");
	zassert_equal(priv.settings.module_settings.position_precision, 32U,
		      "a private key is a choice and stays");
	zassert_equal(meshtastic_channels_get(0U)->settings.module_settings.position_precision, 0U,
		      "and the live slot agrees");

	/* The save writes the marker; after it a chosen 13 is a 13. */
	flush_all();
	zassert_true(raw_read("meshtastic/seed/optin", &r) && r.len == 1U && r.buf[0] == 1U,
		     "marker persisted");
	pub.settings.module_settings.position_precision = 13U;
	zassert_ok(meshtastic_config_store_set_channel(0U, &pub), "");
	flush_all();
	stage_boot(); /* RAM keeps the marker the save set; flash holds it too */
	zassert_ok(meshtastic_config_store_get_channel(0U, &pub), "");
	zassert_equal(pub.settings.module_settings.position_precision, 13U,
		      "never migrated twice");
}
