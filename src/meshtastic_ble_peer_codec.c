/* SPDX-License-Identifier: GPL-3.0 */

#include <errno.h>
#include <string.h>

#include "meshtastic_ble_peer_codec.h"

/* Local LE helpers rather than zephyr/sys/byteorder.h: this file stays free of
 * Zephyr headers so the ztest can compile it with no BT host at all. */
static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)((v >> 16) & 0xFFU);
	p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

void meshtastic_ble_peer_beat_encode(const struct meshtastic_ble_peer_beat *beat,
				     uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN])
{
	buf[0] = MESHTASTIC_BLE_PEER_BEAT_MAGIC;
	buf[1] = MESHTASTIC_BLE_PEER_BEAT_VERSION;
	buf[2] = beat->flags;
	buf[3] = 0U;
	put_le32(&buf[4], beat->node_num);
	put_le32(&buf[8], beat->seq);
	put_le32(&buf[12], beat->uptime_s);
}

int meshtastic_ble_peer_beat_decode(const uint8_t *buf, size_t len,
				    struct meshtastic_ble_peer_beat *beat)
{
	if (len != MESHTASTIC_BLE_PEER_BEAT_LEN) {
		return -EINVAL;
	}
	if (buf[0] != MESHTASTIC_BLE_PEER_BEAT_MAGIC) {
		return -EBADMSG;
	}
	if (buf[1] != MESHTASTIC_BLE_PEER_BEAT_VERSION) {
		return -ENOTSUP;
	}

	beat->flags = buf[2];
	beat->node_num = get_le32(&buf[4]);
	beat->seq = get_le32(&buf[8]);
	beat->uptime_s = get_le32(&buf[12]);
	return 0;
}

void meshtastic_ble_peer_adv_encode(uint8_t buf[MESHTASTIC_BLE_PEER_ADV_LEN],
				    uint32_t node_num)
{
	buf[0] = (uint8_t)(MESHTASTIC_BLE_PEER_ADV_COMPANY & 0xFFU);
	buf[1] = (uint8_t)(MESHTASTIC_BLE_PEER_ADV_COMPANY >> 8);
	buf[2] = MESHTASTIC_BLE_PEER_ADV_FORMAT;
	put_le32(&buf[3], node_num);
}

int meshtastic_ble_peer_adv_decode(const uint8_t *buf, size_t len, uint32_t *node_num)
{
	if (len != MESHTASTIC_BLE_PEER_ADV_LEN) {
		return -EINVAL;
	}
	if (buf[0] != (uint8_t)(MESHTASTIC_BLE_PEER_ADV_COMPANY & 0xFFU) ||
	    buf[1] != (uint8_t)(MESHTASTIC_BLE_PEER_ADV_COMPANY >> 8) ||
	    buf[2] != MESHTASTIC_BLE_PEER_ADV_FORMAT) {
		return -EBADMSG;
	}

	*node_num = get_le32(&buf[3]);
	return 0;
}

void meshtastic_ble_peer_rx_reset(struct meshtastic_ble_peer_rx *st)
{
	memset(st, 0, sizeof(*st));
}

void meshtastic_ble_peer_rx_account(struct meshtastic_ble_peer_rx *st,
				    const struct meshtastic_ble_peer_beat *beat)
{
	bool hello = (beat->flags & MESHTASTIC_BLE_PEER_FLAG_HELLO) != 0U;

	if (!st->synced) {
		st->synced = true;
	} else if (hello || beat->seq <= st->last.seq) {
		/* Link restart (announced or inferred from a backwards seq):
		 * resync rather than inventing a bogus loss figure. */
		st->resyncs++;
	} else {
		st->lost += beat->seq - (st->last.seq + 1U);
	}

	st->beats++;
	st->last = *beat;
}
