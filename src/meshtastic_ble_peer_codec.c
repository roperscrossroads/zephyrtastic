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
	/* Shorter than version 1 is malformed; longer is a newer sender whose
	 * appended fields this decoder does not know — read what it does. */
	if (len < MESHTASTIC_BLE_PEER_BEAT_LEN) {
		return -EINVAL;
	}
	if (buf[0] != MESHTASTIC_BLE_PEER_BEAT_MAGIC) {
		return -EBADMSG;
	}
	if (buf[1] < MESHTASTIC_BLE_PEER_BEAT_VERSION) {
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

int meshtastic_ble_peer_chunker_start(struct meshtastic_ble_peer_chunker *ck,
				      const uint8_t *frame, size_t len)
{
	if (len == 0U) {
		return -EINVAL;
	}
	if (len > MESHTASTIC_BLE_PEER_FRAME_MAX) {
		return -EMSGSIZE;
	}

	ck->frame = frame;
	ck->len = (uint16_t)len;
	ck->off = 0U;
	ck->seq = 0U;
	ck->first_sent = false;
	return 0;
}

int meshtastic_ble_peer_chunker_next(struct meshtastic_ble_peer_chunker *ck,
				     uint8_t *out, size_t out_size)
{
	bool first = !ck->first_sent;
	size_t body_cap;
	size_t take;
	size_t pos;

	if (out_size < MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF) {
		return -EINVAL;
	}
	if (ck->first_sent && ck->off == ck->len) {
		return 0;
	}

	pos = MESHTASTIC_BLE_PEER_CHUNK_HDR_LEN;
	if (first) {
		out[pos] = (uint8_t)(ck->len & 0xFFU);
		out[pos + 1U] = (uint8_t)(ck->len >> 8);
		pos += 2U;
	}
	body_cap = out_size - pos;
	take = ck->len - ck->off;
	if (take > body_cap) {
		take = body_cap;
	}
	memcpy(&out[pos], &ck->frame[ck->off], take);
	ck->off += (uint16_t)take;

	out[0] = ck->seq & MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK;
	if (first) {
		out[0] |= MESHTASTIC_BLE_PEER_CHUNK_FIRST;
	}
	if (ck->off == ck->len) {
		out[0] |= MESHTASTIC_BLE_PEER_CHUNK_LAST;
	}
	ck->seq = (ck->seq + 1U) & MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK;
	ck->first_sent = true;
	return (int)(pos + take);
}

void meshtastic_ble_peer_reasm_reset(struct meshtastic_ble_peer_reasm *rs)
{
	memset(rs, 0, sizeof(*rs));
}

/* Kill a partial without touching the lifetime counters the caller owns. */
static void reasm_abort(struct meshtastic_ble_peer_reasm *rs)
{
	if (rs->active) {
		rs->active = false;
		rs->aborted++;
	}
}

int meshtastic_ble_peer_reasm_ingest(struct meshtastic_ble_peer_reasm *rs,
				     const uint8_t *chunk, size_t len, size_t *frame_len)
{
	bool first;
	bool last;
	uint8_t seq;
	const uint8_t *body;
	size_t blen;

	if (len < MESHTASTIC_BLE_PEER_CHUNK_HDR_LEN + 1U) {
		rs->rejected++;
		return -EBADMSG;
	}

	first = (chunk[0] & MESHTASTIC_BLE_PEER_CHUNK_FIRST) != 0U;
	last = (chunk[0] & MESHTASTIC_BLE_PEER_CHUNK_LAST) != 0U;
	seq = chunk[0] & MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK;
	body = &chunk[MESHTASTIC_BLE_PEER_CHUNK_HDR_LEN];
	blen = len - MESHTASTIC_BLE_PEER_CHUNK_HDR_LEN;

	if (first) {
		uint16_t declared;

		/* A FIRST always supersedes: the sender restarted, and the
		 * stale partial can never complete. */
		reasm_abort(rs);

		if (blen < 2U) {
			rs->rejected++;
			return -EBADMSG;
		}
		declared = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
		body += 2U;
		blen -= 2U;

		if (declared == 0U) {
			rs->rejected++;
			return -EBADMSG;
		}
		if (declared > MESHTASTIC_BLE_PEER_FRAME_MAX) {
			rs->rejected++;
			return -EMSGSIZE;
		}
		if (blen > declared) {
			rs->rejected++;
			return -EBADMSG;
		}

		memcpy(rs->frame, body, blen);
		rs->expect = declared;
		rs->got = (uint16_t)blen;
		rs->next_seq = (seq + 1U) & MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK;
		rs->active = true;
	} else {
		if (!rs->active) {
			/* The FIRST was missed (or lost): nothing this chunk
			 * could be reassembled into. */
			rs->rejected++;
			return -EBADMSG;
		}
		if (seq != rs->next_seq || blen > (size_t)(rs->expect - rs->got)) {
			reasm_abort(rs);
			rs->rejected++;
			return -EBADMSG;
		}

		memcpy(&rs->frame[rs->got], body, blen);
		rs->got += (uint16_t)blen;
		rs->next_seq = (seq + 1U) & MESHTASTIC_BLE_PEER_CHUNK_SEQ_MASK;
	}

	if (last) {
		if (rs->got != rs->expect) {
			reasm_abort(rs);
			rs->rejected++;
			return -EBADMSG;
		}
		rs->active = false;
		rs->frames++;
		*frame_len = rs->got;
		return 1;
	}
	if (rs->got == rs->expect) {
		/* Full without LAST: the sender promises more bytes than the
		 * declared length holds. Corrupt by construction. */
		reasm_abort(rs);
		rs->rejected++;
		return -EBADMSG;
	}
	return 0;
}
