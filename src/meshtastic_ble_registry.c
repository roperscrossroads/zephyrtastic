/* SPDX-License-Identifier: GPL-3.0 */

#include <errno.h>

#include "meshtastic_ble_registry.h"

struct ble_reg_slot {
	bool in_use;
	bool phone_noted;
};

static struct ble_reg_slot slots[MESHTASTIC_BLE_REG_SLOTS];

void meshtastic_ble_reg_reset(void)
{
	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		slots[i].in_use = false;
		slots[i].phone_noted = false;
	}
}

int meshtastic_ble_reg_connect(unsigned int index, bool phone_noted)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return -EINVAL;
	}
	if (slots[index].in_use) {
		return -EALREADY;
	}

	slots[index].in_use = true;
	slots[index].phone_noted = phone_noted;
	return 0;
}

bool meshtastic_ble_reg_disconnect(unsigned int index)
{
	bool noted;

	if (index >= MESHTASTIC_BLE_REG_SLOTS || !slots[index].in_use) {
		return false;
	}

	noted = slots[index].phone_noted;
	slots[index].in_use = false;
	slots[index].phone_noted = false;
	return noted;
}

bool meshtastic_ble_reg_phone_noted(unsigned int index)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return false;
	}
	return slots[index].in_use && slots[index].phone_noted;
}

unsigned int meshtastic_ble_reg_active(void)
{
	unsigned int n = 0U;

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		if (slots[i].in_use) {
			n++;
		}
	}
	return n;
}
