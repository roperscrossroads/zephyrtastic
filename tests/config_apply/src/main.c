/* SPDX-License-Identifier: GPL-3.0
 *
 * agents-ls39.1.7, claim (b): after a live-only preset change (a hop, a head's
 * SET_PRESET, the scanner) a write to ANOTHER section must not make the node's
 * state describe a preset the chip is not on. apply_core() re-derives the
 * preset from the stored LoRa section on every call.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <psa/crypto.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_preset.h"

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static struct meshtastic_config cfg = {
	.node_id = 0x11223344U,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "cfg",
	.short_name = "cfg",
};

static void *suite_setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa");
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	return NULL;
}

ZTEST_SUITE(config_apply, NULL, suite_setup, NULL, NULL, NULL);

static void write_device_section(void)
{
	meshtastic_Config c;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_device_tag, &c), "");
	zassert_ok(meshtastic_config_store_set_config(&c), "");
}

ZTEST(config_apply, test_a_write_to_another_section_keeps_a_live_only_preset)
{
	meshtastic_Config c;
	uint8_t sf_before;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_lora_tag, &c), "");
	zassert_equal(c.payload_variant.lora.modem_preset,
		      meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST, "stored: LongFast");

	/* A live-only move: the chip and mt go to ShortTurbo, the store is not told. */
	zassert_ok(meshtastic_preset_switch(meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
					    NULL), "");
	zassert_equal(mt.modem_preset, meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO, "");
	sf_before = mt.modem.spread_factor;

	write_device_section();

	zassert_equal(mt.modem_preset, meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
		      "a device write must not move mt off the preset the chip is on");
	zassert_equal(mt.modem.spread_factor, sf_before, "nor its modem params");
}
