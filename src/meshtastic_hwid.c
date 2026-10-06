/*
 * The node's factory hardware identifier, for MyNodeInfo.device_id.
 *
 * The reference fills device_id with 16 bytes burned into the silicon (NodeDB::init ->
 * getDeviceId(), per platform), and the clients key their local databases to it: it is
 * the one identity that survives a firmware change, an erase and a key change, where
 * my_node_num (crc32 of the public key since 2.8, and here) does not. The Android app
 * accepts only 16 to 64 hex digits as a hardware id, so a shorter value counts as absent.
 *
 * The bytes are the reference's for the same chip, so a board that moves between the
 * stock firmware and this one is one device to the app:
 *
 *   ESP32-S2/S3/C3/C6   the 128-bit OPTIONAL_UNIQUE_ID eFuse, as the blob API lays it out
 *                       (main-esp32.cpp: esp_efuse_read_field_blob into uint32_t[4], memcpy'd)
 *   nRF52               FICR DEVICEID[0..1] then DEVICEADDR[0..1], each as a little-endian
 *                       uint64 (main-nrf52.cpp)
 *   anything else       the reference falls back to the 6-byte MAC, zero-padded: here the
 *                       platform's hwinfo id, zero-padded (native_sim: 4 bytes from its
 *                       command line, so a test can predict it)
 *
 * The phone API sends it only on an authorized connection (lockdown redacts it, as the
 * reference does for an unauthenticated client): a stable hardware fingerprint is
 * correlation material, and a locked connection has no need of it.
 */
#include <string.h>

#include <zephyr/kernel.h>
#if defined(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/meshtastic/hwid.h>

#if defined(CONFIG_SOC_SERIES_ESP32S2) || defined(CONFIG_SOC_SERIES_ESP32S3) || \
	defined(CONFIG_SOC_SERIES_ESP32C3) || defined(CONFIG_SOC_SERIES_ESP32C6)
#define HWID_FROM_EFUSE 1
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#include <esp_err.h>
#elif defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <nrfx.h>
#endif

LOG_MODULE_REGISTER(meshtastic_hwid, CONFIG_MESHTASTIC_LOG_LEVEL);

static uint8_t cached[MESHTASTIC_HWID_LEN];
static int cached_len; /* 0: not read yet; <0: no id on this hardware */

static int read_hwid(uint8_t out[MESHTASTIC_HWID_LEN])
{
#if defined(HWID_FROM_EFUSE)
	/* 128 bits, filled least-significant byte first: the same bytes the reference's
	 * uint32_t[4] holds on this little-endian core. */
	if (esp_efuse_read_field_blob(ESP_EFUSE_OPTIONAL_UNIQUE_ID, out,
				      MESHTASTIC_HWID_LEN * 8U) != ESP_OK) {
		LOG_WRN("eFuse unique id unreadable");
		return -EIO;
	}
	return MESHTASTIC_HWID_LEN;
#elif defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
	uint64_t id = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
	uint64_t addr = ((uint64_t)NRF_FICR->DEVICEADDR[1] << 32) | NRF_FICR->DEVICEADDR[0];

	sys_put_le64(id, out);
	sys_put_le64(addr, out + 8);
	return MESHTASTIC_HWID_LEN;
#elif defined(CONFIG_HWINFO)
	ssize_t n = hwinfo_get_device_id(out, MESHTASTIC_HWID_LEN);

	if (n <= 0) {
		return -ENODEV;
	}
	memset(out + n, 0, MESHTASTIC_HWID_LEN - (size_t)n);
	return MESHTASTIC_HWID_LEN;
#else
	/* No silicon id and no hwinfo in this build: the field stays empty (absent to the app). */
	ARG_UNUSED(out);
	return -ENODEV;
#endif
}

int meshtastic_hwid_get(uint8_t out[MESHTASTIC_HWID_LEN])
{
	if (cached_len == 0) {
		int n = read_hwid(cached);

		cached_len = (n > 0) ? n : -1;
	}
	if (cached_len < 0) {
		return -ENODEV;
	}
	memcpy(out, cached, MESHTASTIC_HWID_LEN);
	return MESHTASTIC_HWID_LEN;
}
