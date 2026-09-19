/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * A second USB CDC-ACM port for the serial PhoneAPI, next to the console.
 *
 * Zephyr's CDC_ACM_SERIAL_INITIALIZE_AT_BOOT brings up the device stack with
 * exactly one CDC-ACM function (it registers "cdc_acm_0" by name and calls
 * usbd_init() in the same function), so a second devicetree instance never
 * reaches the bus. This is that init with the one difference that matters:
 * every CDC-ACM instance is registered, making the device a composite of two
 * serial ports — the console/shell on the first, the Meshtastic StreamAPI
 * (zephyr,meshtastic-uart) on the second. A browser's Web Serial or the
 * Python CLI then opens the second port without fighting the shell for it.
 *
 * The identity is deliberately the stock one (VID/PID, manufacturer and
 * product strings, hardware serial number): host tooling that finds the
 * board by /dev/serial/by-id keeps finding the console at interface 00; the
 * PhoneAPI port appears beside it at interface 02.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_phone_cdc, CONFIG_USBD_LOG_LEVEL);

USBD_DEVICE_DEFINE(phone_cdc_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0004);

USBD_DESC_LANG_DEFINE(phone_cdc_lang);
USBD_DESC_MANUFACTURER_DEFINE(phone_cdc_mfr, "Zephyr Project");
USBD_DESC_PRODUCT_DEFINE(phone_cdc_product, "CDC ACM serial backend");
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(phone_cdc_sn)));

USBD_DESC_CONFIG_DEFINE(phone_cdc_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(phone_cdc_fs_config, 0, 125, &phone_cdc_fs_cfg_desc);

static int usb_phone_cdc_init(void)
{
	int err;

	err = usbd_add_descriptor(&phone_cdc_usbd, &phone_cdc_lang);
	if (err == 0) {
		err = usbd_add_descriptor(&phone_cdc_usbd, &phone_cdc_mfr);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&phone_cdc_usbd, &phone_cdc_product);
	}
	IF_ENABLED(CONFIG_HWINFO, (
		if (err == 0) {
			err = usbd_add_descriptor(&phone_cdc_usbd, &phone_cdc_sn);
		}
	))
	if (err) {
		LOG_ERR("descriptors (%d)", err);
		return err;
	}

	/* The nRF52840 is full-speed only, so one configuration is enough. */
	err = usbd_add_configuration(&phone_cdc_usbd, USBD_SPEED_FS, &phone_cdc_fs_config);
	if (err) {
		LOG_ERR("configuration (%d)", err);
		return err;
	}

	err = usbd_register_all_classes(&phone_cdc_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("classes (%d)", err);
		return err;
	}

	/* Two functions, each an interface pair: the host needs the IAD triple
	 * to bind them as two ports rather than one confused device. */
	err = usbd_device_set_code_triple(&phone_cdc_usbd, USBD_SPEED_FS,
					  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	if (err == 0) {
		err = usbd_init(&phone_cdc_usbd);
	}
	if (err == 0) {
		err = usbd_enable(&phone_cdc_usbd);
	}
	if (err) {
		LOG_ERR("device (%d)", err);
	}
	return err;
}

SYS_INIT(usb_phone_cdc_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
