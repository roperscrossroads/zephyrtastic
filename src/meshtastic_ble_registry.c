/* SPDX-License-Identifier: GPL-3.0 */

#include <errno.h>
#include <string.h>

#include "meshtastic_ble_registry.h"

struct ble_reg_slot {
	enum meshtastic_ble_conn_kind kind; /* NONE = free */
	bool phone_noted;
};

static struct ble_reg_slot slots[MESHTASTIC_BLE_REG_SLOTS];
static struct meshtastic_ble_reg_stats stats;

void meshtastic_ble_reg_reset(void)
{
	memset(slots, 0, sizeof(slots));
	memset(&stats, 0, sizeof(stats));
}

int meshtastic_ble_reg_connect(unsigned int index, enum meshtastic_ble_conn_kind initial)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		stats.slot_index_out_of_range++;
		return -EINVAL;
	}
	if (initial != MESHTASTIC_BLE_CONN_PEER && initial != MESHTASTIC_BLE_CONN_UNCLASSIFIED) {
		return -EINVAL;
	}
	if (slots[index].kind != MESHTASTIC_BLE_CONN_NONE) {
		return -EALREADY;
	}

	slots[index].kind = initial;
	slots[index].phone_noted = false;
	if (initial == MESHTASTIC_BLE_CONN_PEER) {
		stats.classified_peer_by_role++;
	}
	return 0;
}

int meshtastic_ble_reg_classify(unsigned int index, enum meshtastic_ble_conn_kind kind,
				enum meshtastic_ble_classify_reason reason)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		stats.slot_index_out_of_range++;
		return -EINVAL;
	}
	if (kind != MESHTASTIC_BLE_CONN_PHONE && kind != MESHTASTIC_BLE_CONN_PEER) {
		return -EINVAL;
	}
	if (slots[index].kind == MESHTASTIC_BLE_CONN_NONE) {
		return -EINVAL;
	}
	if (slots[index].kind != MESHTASTIC_BLE_CONN_UNCLASSIFIED) {
		return -EALREADY;
	}

	slots[index].kind = kind;
	if (kind == MESHTASTIC_BLE_CONN_PHONE) {
		/* The one place the phone note is granted; the caller charges
		 * the PM inhibitor on our 0 return. */
		slots[index].phone_noted = true;
		stats.phone_notes++;
		if (reason == MESHTASTIC_BLE_CLASSIFY_TIMER) {
			stats.classified_phone_default++;
		} else {
			stats.classified_phone_by_traffic++;
		}
	} else {
		stats.classified_peer_by_hello++;
	}
	return 0;
}

bool meshtastic_ble_reg_disconnect(unsigned int index)
{
	bool noted;

	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		stats.slot_index_out_of_range++;
		return false;
	}
	if (slots[index].kind == MESHTASTIC_BLE_CONN_NONE) {
		return false;
	}

	noted = slots[index].phone_noted;
	slots[index].kind = MESHTASTIC_BLE_CONN_NONE;
	slots[index].phone_noted = false;
	if (noted) {
		stats.phone_unnotes++;
	}
	return noted;
}

enum meshtastic_ble_conn_kind meshtastic_ble_reg_kind(unsigned int index)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return MESHTASTIC_BLE_CONN_NONE;
	}
	return slots[index].kind;
}

bool meshtastic_ble_reg_phone_noted(unsigned int index)
{
	if (index >= MESHTASTIC_BLE_REG_SLOTS) {
		return false;
	}
	return slots[index].phone_noted;
}

unsigned int meshtastic_ble_reg_active(void)
{
	unsigned int n = 0U;

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		if (slots[i].kind != MESHTASTIC_BLE_CONN_NONE) {
			n++;
		}
	}
	return n;
}

unsigned int meshtastic_ble_reg_unclassified(void)
{
	unsigned int n = 0U;

	for (unsigned int i = 0U; i < MESHTASTIC_BLE_REG_SLOTS; i++) {
		if (slots[i].kind == MESHTASTIC_BLE_CONN_UNCLASSIFIED) {
			n++;
		}
	}
	return n;
}

void meshtastic_ble_reg_stats(struct meshtastic_ble_reg_stats *out)
{
	*out = stats;
}
