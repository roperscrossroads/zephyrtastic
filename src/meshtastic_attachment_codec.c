/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The attachment envelope, encode and decode (meshtastic_attachment_codec.h).
 * Pure C: byte packing only.
 */

#include <errno.h>
#include <string.h>

#include "meshtastic_attachment_codec.h"

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)((v >> 16) & 0xFFU);
	p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

int meshtastic_attachment_encode_rx_frame(const struct meshtastic_attachment_rx_frame *m,
					  uint8_t *out, size_t out_size)
{
	size_t len;

	if (m == NULL || out == NULL || m->wire == NULL || m->wire_len == 0U) {
		return -EINVAL;
	}
	if (m->wire_len > MESHTASTIC_ATTACHMENT_WIRE_MAX) {
		return -EMSGSIZE;
	}
	len = MESHTASTIC_ATTACHMENT_RX_HDR_LEN + m->wire_len;
	if (out_size < len) {
		return -EMSGSIZE;
	}

	out[0] = MESHTASTIC_ATTACHMENT_RX_FRAME;
	out[1] = m->preset;
	put_u16(&out[2], (uint16_t)m->rssi);
	out[4] = (uint8_t)m->snr;
	put_u32(&out[5], m->rx_ms);
	out[9] = m->flags;
	memcpy(&out[MESHTASTIC_ATTACHMENT_RX_HDR_LEN], m->wire, m->wire_len);
	return (int)len;
}

int meshtastic_attachment_encode_tx_frame(const struct meshtastic_attachment_tx_frame *m,
					  uint8_t *out, size_t out_size)
{
	size_t len;

	if (m == NULL || out == NULL || m->wire == NULL || m->wire_len == 0U) {
		return -EINVAL;
	}
	if (m->wire_len > MESHTASTIC_ATTACHMENT_WIRE_MAX) {
		return -EMSGSIZE;
	}
	const size_t hdr = ((m->flags & MESHTASTIC_ATTACHMENT_TXF_RELAY) != 0U)
				   ? MESHTASTIC_ATTACHMENT_TX_RELAY_HDR_LEN
				   : MESHTASTIC_ATTACHMENT_TX_HDR_LEN;

	len = hdr + m->wire_len;
	if (out_size < len) {
		return -EMSGSIZE;
	}

	out[0] = MESHTASTIC_ATTACHMENT_TX_FRAME;
	out[1] = m->preset;
	out[2] = m->flags;
	put_u16(&out[3], m->tx_seq);
	if (hdr == MESHTASTIC_ATTACHMENT_TX_RELAY_HDR_LEN) {
		put_u32(&out[5], m->relay_src);
		put_u32(&out[9], m->relay_id);
		put_u32(&out[13], m->rx_ms);
		put_u32(&out[17], m->not_before_ms);
		out[21] = m->dupe;
	}
	memcpy(&out[hdr], m->wire, m->wire_len);
	return (int)len;
}

int meshtastic_attachment_encode_tx_result(const struct meshtastic_attachment_tx_result *m,
					   uint8_t *out, size_t out_size)
{
	if (m == NULL || out == NULL) {
		return -EINVAL;
	}
	if (out_size < MESHTASTIC_ATTACHMENT_TX_RESULT_LEN) {
		return -EMSGSIZE;
	}

	out[0] = MESHTASTIC_ATTACHMENT_TX_RESULT;
	put_u16(&out[1], m->tx_seq);
	out[3] = (uint8_t)m->rc;
	out[4] = m->defers;
	put_u32(&out[5], m->tx_ms);
	return (int)MESHTASTIC_ATTACHMENT_TX_RESULT_LEN;
}

int meshtastic_attachment_encode_status(const struct meshtastic_attachment_status *m,
					uint8_t *out, size_t out_size)
{
	const bool pos = (m != NULL) && ((m->flags & MESHTASTIC_ATTACHMENT_ST_HAS_POS) != 0U);
	const size_t len = pos ? MESHTASTIC_ATTACHMENT_STATUS_POS_LEN
			       : MESHTASTIC_ATTACHMENT_STATUS_LEN;

	if (m == NULL || out == NULL) {
		return -EINVAL;
	}
	if (out_size < len) {
		return -EMSGSIZE;
	}

	out[0] = MESHTASTIC_ATTACHMENT_STATUS;
	out[1] = m->preset;
	out[2] = m->flags;
	put_u32(&out[3], m->hwid);
	put_u32(&out[7], m->brain);
	put_u32(&out[11], m->rx_frames);
	put_u32(&out[15], m->tx_frames);
	put_u32(&out[19], m->uptime_s);
	put_u32(&out[23], m->rx_dropped);
	if (pos) {
		put_u32(&out[27], (uint32_t)m->lat);
		put_u32(&out[31], (uint32_t)m->lon);
		put_u32(&out[35], (uint32_t)m->alt);
	}
	return (int)len;
}

int meshtastic_attachment_encode_tx_cancel(uint32_t src, uint32_t id, uint8_t *out,
					   size_t out_size)
{
	if (out == NULL || out_size < MESHTASTIC_ATTACHMENT_TX_CANCEL_LEN) {
		return -EMSGSIZE;
	}
	out[0] = MESHTASTIC_ATTACHMENT_TX_CANCEL;
	put_u32(&out[1], src);
	put_u32(&out[5], id);
	return (int)MESHTASTIC_ATTACHMENT_TX_CANCEL_LEN;
}

int meshtastic_attachment_encode_set_policy(const struct meshtastic_attachment_policy *m,
					    uint8_t *out, size_t out_size)
{
	if (m == NULL || out == NULL || out_size < MESHTASTIC_ATTACHMENT_SET_POLICY_LEN) {
		return -EMSGSIZE;
	}
	out[0] = MESHTASTIC_ATTACHMENT_SET_POLICY;
	out[1] = m->flags;
	out[2] = (uint8_t)m->tx_power;
	return (int)MESHTASTIC_ATTACHMENT_SET_POLICY_LEN;
}

int meshtastic_attachment_encode_set_preset(uint8_t preset, uint8_t *out, size_t out_size)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (out_size < MESHTASTIC_ATTACHMENT_SET_PRESET_LEN) {
		return -EMSGSIZE;
	}
	out[0] = MESHTASTIC_ATTACHMENT_SET_PRESET;
	out[1] = preset;
	return (int)MESHTASTIC_ATTACHMENT_SET_PRESET_LEN;
}

int meshtastic_attachment_decode(const uint8_t *env, size_t len,
				 struct meshtastic_attachment_msg *out)
{
	if (env == NULL || out == NULL || len < 1U) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->type = env[0];

	switch (env[0]) {
	case MESHTASTIC_ATTACHMENT_RX_FRAME:
		if (len <= MESHTASTIC_ATTACHMENT_RX_HDR_LEN ||
		    len > MESHTASTIC_ATTACHMENT_RX_HDR_LEN + MESHTASTIC_ATTACHMENT_WIRE_MAX) {
			return -EBADMSG;
		}
		out->u.rx.preset = env[1];
		out->u.rx.rssi = (int16_t)get_u16(&env[2]);
		out->u.rx.snr = (int8_t)env[4];
		out->u.rx.rx_ms = get_u32(&env[5]);
		out->u.rx.flags = env[9];
		out->u.rx.wire = &env[MESHTASTIC_ATTACHMENT_RX_HDR_LEN];
		out->u.rx.wire_len = (uint16_t)(len - MESHTASTIC_ATTACHMENT_RX_HDR_LEN);
		return 0;
	case MESHTASTIC_ATTACHMENT_TX_FRAME: {
		size_t hdr = MESHTASTIC_ATTACHMENT_TX_HDR_LEN;

		if (len > 2U && (env[2] & MESHTASTIC_ATTACHMENT_TXF_RELAY) != 0U) {
			hdr = MESHTASTIC_ATTACHMENT_TX_RELAY_HDR_LEN;
		}
		if (len <= hdr || len > hdr + MESHTASTIC_ATTACHMENT_WIRE_MAX) {
			return -EBADMSG;
		}
		out->u.tx.preset = env[1];
		out->u.tx.flags = env[2];
		out->u.tx.tx_seq = get_u16(&env[3]);
		if (hdr == MESHTASTIC_ATTACHMENT_TX_RELAY_HDR_LEN) {
			out->u.tx.relay_src = get_u32(&env[5]);
			out->u.tx.relay_id = get_u32(&env[9]);
			out->u.tx.rx_ms = get_u32(&env[13]);
			out->u.tx.not_before_ms = get_u32(&env[17]);
			out->u.tx.dupe = env[21];
		}
		out->u.tx.wire = &env[hdr];
		out->u.tx.wire_len = (uint16_t)(len - hdr);
		return 0;
	}
	case MESHTASTIC_ATTACHMENT_TX_CANCEL:
		if (len < MESHTASTIC_ATTACHMENT_TX_CANCEL_LEN) {
			return -EBADMSG;
		}
		out->u.cancel.src = get_u32(&env[1]);
		out->u.cancel.id = get_u32(&env[5]);
		return 0;
	case MESHTASTIC_ATTACHMENT_SET_POLICY:
		if (len < MESHTASTIC_ATTACHMENT_SET_POLICY_LEN) {
			return -EBADMSG;
		}
		out->u.policy.flags = env[1];
		out->u.policy.tx_power = (int8_t)env[2];
		return 0;
	case MESHTASTIC_ATTACHMENT_TX_RESULT:
		if (len < MESHTASTIC_ATTACHMENT_TX_RESULT_LEN) {
			return -EBADMSG;
		}
		out->u.result.tx_seq = get_u16(&env[1]);
		out->u.result.rc = (int8_t)env[3];
		out->u.result.defers = env[4];
		out->u.result.tx_ms = get_u32(&env[5]);
		return 0;
	case MESHTASTIC_ATTACHMENT_STATUS: {
		bool pos;

		/* The flags byte says whether the position tail is present; it must
		 * itself be present before it is read (review F8). */
		if (len < MESHTASTIC_ATTACHMENT_STATUS_LEN) {
			return -EBADMSG;
		}
		pos = (env[2] & MESHTASTIC_ATTACHMENT_ST_HAS_POS) != 0U;
		if (pos && len < MESHTASTIC_ATTACHMENT_STATUS_POS_LEN) {
			return -EBADMSG;
		}
		out->u.status.preset = env[1];
		out->u.status.flags = env[2];
		out->u.status.hwid = get_u32(&env[3]);
		out->u.status.brain = get_u32(&env[7]);
		out->u.status.rx_frames = get_u32(&env[11]);
		out->u.status.tx_frames = get_u32(&env[15]);
		out->u.status.uptime_s = get_u32(&env[19]);
		out->u.status.rx_dropped = get_u32(&env[23]);
		if (pos) {
			out->u.status.lat = (int32_t)get_u32(&env[27]);
			out->u.status.lon = (int32_t)get_u32(&env[31]);
			out->u.status.alt = (int32_t)get_u32(&env[35]);
		}
		return 0;
	}
	case MESHTASTIC_ATTACHMENT_SET_PRESET:
		if (len < MESHTASTIC_ATTACHMENT_SET_PRESET_LEN) {
			return -EBADMSG;
		}
		out->u.preset = env[1];
		return 0;
	default:
		return -EBADMSG;
	}
}
