/*
 * The node's factory hardware identifier: 16 bytes burned into the silicon, the bytes the
 * reference firmware reports for the same chip (src/meshtastic_hwid.c says which per SoC).
 * It is what MyNodeInfo.device_id carries to an authorized phone, and it is not an identity
 * on the air: the mesh knows a node by its key-derived number.
 */
#ifndef ZEPHYR_INCLUDE_MESHTASTIC_HWID_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_HWID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MESHTASTIC_HWID_LEN 16

/**
 * @brief The hardware identifier, read once from the silicon and cached.
 *
 * @param out  MESHTASTIC_HWID_LEN bytes.
 * @return MESHTASTIC_HWID_LEN, or -ENODEV when this hardware has none to give.
 */
int meshtastic_hwid_get(uint8_t out[MESHTASTIC_HWID_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_HWID_H_ */
