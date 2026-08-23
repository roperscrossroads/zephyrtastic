/* SPDX-License-Identifier: GPL-3.0
 *
 * Unit tests for the BLE peer-link wire formats and receive accounting
 * (src/meshtastic_ble_peer_codec.c). native_sim has no BLE controller, so the
 * radio-borne halves (GATT service, advertiser, scanner) are hardware-only —
 * what IS testable, exhaustively, is every byte of the framing and the
 * loss/resync arithmetic, which is exactly what these tests pin down.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "meshtastic_ble_peer_codec.h"

ZTEST_SUITE(ble_peer_codec, NULL, NULL, NULL, NULL, NULL);

ZTEST(ble_peer_codec, test_beat_roundtrip)
{
	struct meshtastic_ble_peer_beat in = {
		.flags = MESHTASTIC_BLE_PEER_FLAG_HELLO,
		.node_num = 0x699C0E88U,
		.seq = 0x01020304U,
		.uptime_s = 86400U,
	};
	struct meshtastic_ble_peer_beat out;
	uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN];

	meshtastic_ble_peer_beat_encode(&in, buf);
	zassert_ok(meshtastic_ble_peer_beat_decode(buf, sizeof(buf), &out));
	zassert_equal(out.flags, in.flags);
	zassert_equal(out.node_num, in.node_num);
	zassert_equal(out.seq, in.seq);
	zassert_equal(out.uptime_s, in.uptime_s);
}

/* Pin the exact bytes, not just the roundtrip: the other end of this link may
 * one day be a different implementation reading this header comment. */
ZTEST(ble_peer_codec, test_beat_wire_bytes)
{
	struct meshtastic_ble_peer_beat in = {
		.flags = 0U,
		.node_num = 0x04E14BB4U,
		.seq = 7U,
		.uptime_s = 0x00010203U,
	};
	static const uint8_t expect[MESHTASTIC_BLE_PEER_BEAT_LEN] = {
		0x4D, 0x01, 0x00, 0x00,             /* magic, version, flags, reserved */
		0xB4, 0x4B, 0xE1, 0x04,             /* node_num LE */
		0x07, 0x00, 0x00, 0x00,             /* seq LE */
		0x03, 0x02, 0x01, 0x00,             /* uptime_s LE */
	};
	uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN];

	meshtastic_ble_peer_beat_encode(&in, buf);
	zassert_mem_equal(buf, expect, sizeof(expect));
}

ZTEST(ble_peer_codec, test_beat_decode_rejects_bad_frames)
{
	struct meshtastic_ble_peer_beat in = {.node_num = 1U};
	struct meshtastic_ble_peer_beat out;
	uint8_t buf[MESHTASTIC_BLE_PEER_BEAT_LEN];

	meshtastic_ble_peer_beat_encode(&in, buf);

	zassert_equal(meshtastic_ble_peer_beat_decode(buf, sizeof(buf) - 1U, &out), -EINVAL);
	zassert_equal(meshtastic_ble_peer_beat_decode(buf, sizeof(buf) + 1U, &out), -EINVAL);
	zassert_equal(meshtastic_ble_peer_beat_decode(buf, 0U, &out), -EINVAL);

	buf[0] = 0x00U;
	zassert_equal(meshtastic_ble_peer_beat_decode(buf, sizeof(buf), &out), -EBADMSG);
	buf[0] = MESHTASTIC_BLE_PEER_BEAT_MAGIC;

	buf[1] = MESHTASTIC_BLE_PEER_BEAT_VERSION + 1U;
	zassert_equal(meshtastic_ble_peer_beat_decode(buf, sizeof(buf), &out), -ENOTSUP);
	buf[1] = MESHTASTIC_BLE_PEER_BEAT_VERSION;

	/* The reserved byte is ignored, not validated (forward compat). */
	buf[3] = 0xAAU;
	zassert_ok(meshtastic_ble_peer_beat_decode(buf, sizeof(buf), &out));
}

ZTEST(ble_peer_codec, test_adv_roundtrip_and_wire_bytes)
{
	static const uint8_t expect[MESHTASTIC_BLE_PEER_ADV_LEN] = {
		0xFF, 0xFF,             /* company 0xFFFF LE (SIG "internal use") */
		0x01,                   /* format */
		0x88, 0x0E, 0x9C, 0x69, /* node_num LE */
	};
	uint8_t buf[MESHTASTIC_BLE_PEER_ADV_LEN];
	uint32_t node = 0U;

	meshtastic_ble_peer_adv_encode(buf, 0x699C0E88U);
	zassert_mem_equal(buf, expect, sizeof(expect));
	zassert_ok(meshtastic_ble_peer_adv_decode(buf, sizeof(buf), &node));
	zassert_equal(node, 0x699C0E88U);
}

ZTEST(ble_peer_codec, test_adv_decode_rejects_foreign_blobs)
{
	uint8_t buf[MESHTASTIC_BLE_PEER_ADV_LEN];
	uint32_t node;

	meshtastic_ble_peer_adv_encode(buf, 42U);

	zassert_equal(meshtastic_ble_peer_adv_decode(buf, sizeof(buf) - 1U, &node), -EINVAL);

	/* Another 0xFFFF user with a different format byte is NOT a peer. */
	buf[2] = 0x02U;
	zassert_equal(meshtastic_ble_peer_adv_decode(buf, sizeof(buf), &node), -EBADMSG);
	buf[2] = MESHTASTIC_BLE_PEER_ADV_FORMAT;

	/* A real company id is not us either. */
	buf[0] = 0x4CU;
	buf[1] = 0x00U;
	zassert_equal(meshtastic_ble_peer_adv_decode(buf, sizeof(buf), &node), -EBADMSG);
}

static struct meshtastic_ble_peer_beat mk_beat(uint32_t seq, uint8_t flags)
{
	struct meshtastic_ble_peer_beat b = {
		.flags = flags,
		.node_num = 0x11111111U,
		.seq = seq,
		.uptime_s = seq, /* arbitrary */
	};
	return b;
}

ZTEST(ble_peer_codec, test_rx_clean_sequence_counts_no_loss)
{
	struct meshtastic_ble_peer_rx st;
	struct meshtastic_ble_peer_beat b;

	meshtastic_ble_peer_rx_reset(&st);
	for (uint32_t s = 0U; s < 100U; s++) {
		b = mk_beat(s, s == 0U ? MESHTASTIC_BLE_PEER_FLAG_HELLO : 0U);
		meshtastic_ble_peer_rx_account(&st, &b);
	}
	zassert_equal(st.beats, 100U);
	zassert_equal(st.lost, 0U);
	zassert_equal(st.resyncs, 0U);
	zassert_equal(st.last.seq, 99U);
}

ZTEST(ble_peer_codec, test_rx_gaps_are_proven_loss)
{
	struct meshtastic_ble_peer_rx st;
	struct meshtastic_ble_peer_beat b;

	meshtastic_ble_peer_rx_reset(&st);
	b = mk_beat(0U, MESHTASTIC_BLE_PEER_FLAG_HELLO);
	meshtastic_ble_peer_rx_account(&st, &b);
	b = mk_beat(1U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);
	b = mk_beat(5U, 0U); /* 2,3,4 lost */
	meshtastic_ble_peer_rx_account(&st, &b);
	b = mk_beat(6U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);
	b = mk_beat(8U, 0U); /* 7 lost */
	meshtastic_ble_peer_rx_account(&st, &b);

	zassert_equal(st.beats, 5U);
	zassert_equal(st.lost, 4U);
	zassert_equal(st.resyncs, 0U);
}

ZTEST(ble_peer_codec, test_rx_first_beat_midstream_syncs_without_loss)
{
	struct meshtastic_ble_peer_rx st;
	struct meshtastic_ble_peer_beat b;

	/* Joining at seq 500 proves nothing about 0..499 — no invented loss. */
	meshtastic_ble_peer_rx_reset(&st);
	b = mk_beat(500U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);
	zassert_equal(st.beats, 1U);
	zassert_equal(st.lost, 0U);
	zassert_equal(st.resyncs, 0U);
	zassert_true(st.synced);
}

ZTEST(ble_peer_codec, test_rx_hello_and_backwards_seq_resync)
{
	struct meshtastic_ble_peer_rx st;
	struct meshtastic_ble_peer_beat b;

	meshtastic_ble_peer_rx_reset(&st);
	b = mk_beat(40U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);

	/* Announced restart: HELLO at seq 0 is a resync, not 2^32 of loss. */
	b = mk_beat(0U, MESHTASTIC_BLE_PEER_FLAG_HELLO);
	meshtastic_ble_peer_rx_account(&st, &b);
	zassert_equal(st.resyncs, 1U);
	zassert_equal(st.lost, 0U);

	b = mk_beat(1U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);
	zassert_equal(st.lost, 0U);

	/* Unannounced restart (peer rebooted, HELLO dropped): a seq at or below
	 * the last one is inferred as a restart, again with no invented loss. */
	b = mk_beat(1U, 0U);
	meshtastic_ble_peer_rx_account(&st, &b);
	zassert_equal(st.resyncs, 2U);
	zassert_equal(st.lost, 0U);
	zassert_equal(st.beats, 4U);
	zassert_equal(st.last.seq, 1U);
}
