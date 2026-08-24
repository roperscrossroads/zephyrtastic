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

/* ---- the frame channel (agents-xhli.1): chunker + reassembler ---- */

ZTEST_SUITE(ble_peer_frame, NULL, NULL, NULL, NULL, NULL);

static void fill_pattern(uint8_t *buf, size_t len)
{
	for (size_t i = 0U; i < len; i++) {
		buf[i] = (uint8_t)(i * 7U + 3U);
	}
}

/* Chunk `frame` at `chunk_buf` bytes per chunk, feed every chunk straight into
 * `rs` (the two sim endpoints of the M1 milestone), and require the exact
 * frame back. Returns the chunk count so tests can pin the arithmetic. */
static unsigned int frame_loopback(struct meshtastic_ble_peer_reasm *rs, const uint8_t *frame,
				   size_t len, size_t chunk_buf)
{
	struct meshtastic_ble_peer_chunker ck;
	uint8_t buf[64];
	size_t out_len = 0U;
	unsigned int chunks = 0U;
	int ret = -1;
	int n;

	zassert_true(chunk_buf <= sizeof(buf));
	zassert_ok(meshtastic_ble_peer_chunker_start(&ck, frame, len));
	while ((n = meshtastic_ble_peer_chunker_next(&ck, buf, chunk_buf)) > 0) {
		ret = meshtastic_ble_peer_reasm_ingest(rs, buf, (size_t)n, &out_len);
		chunks++;
		if ((buf[0] & MESHTASTIC_BLE_PEER_CHUNK_LAST) == 0U) {
			/* Not the final chunk: must be accepted, not complete. */
			zassert_equal(ret, 0, "mid-frame chunk %u -> %d", chunks, ret);
		}
	}
	zassert_equal(n, 0, "chunker did not finish cleanly (%d)", n);
	zassert_equal(ret, 1, "frame did not complete (%d)", ret);
	zassert_equal(out_len, len);
	zassert_mem_equal(rs->frame, frame, len);
	return chunks;
}

/* Pin the exact wire bytes of a single-chunk frame (FIRST|LAST, seq 0, LE
 * length prefix) — the other end may one day be another implementation. */
ZTEST(ble_peer_frame, test_frame_single_chunk_wire_bytes)
{
	static const uint8_t frame[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
	static const uint8_t expect[8] = {
		0xC0,             /* FIRST|LAST, seq 0 */
		0x05, 0x00,       /* frame length LE */
		0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
	};
	struct meshtastic_ble_peer_chunker ck;
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	int n;

	zassert_ok(meshtastic_ble_peer_chunker_start(&ck, frame, sizeof(frame)));
	n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf));
	zassert_equal(n, (int)sizeof(expect));
	zassert_mem_equal(buf, expect, sizeof(expect));
	zassert_equal(meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf)), 0);
}

ZTEST(ble_peer_frame, test_frame_loopback_max_frame_at_mtu23)
{
	uint8_t frame[MESHTASTIC_BLE_PEER_FRAME_MAX];
	struct meshtastic_ble_peer_reasm rs;
	unsigned int chunks;

	fill_pattern(frame, sizeof(frame));
	meshtastic_ble_peer_reasm_reset(&rs);
	chunks = frame_loopback(&rs, frame, sizeof(frame), MESHTASTIC_BLE_PEER_CHUNK_MTU23);

	/* 255 bytes at MTU 23: FIRST carries 20-1-2=17, the rest 19 each ->
	 * 1 + ceil(238/19) = 14. Pinned so a header-size change is loud. */
	zassert_equal(chunks, 14U);
	zassert_equal(rs.frames, 1U);
	zassert_equal(rs.aborted, 0U);
	zassert_equal(rs.rejected, 0U);
}

/* The seq field is 6 bits; a max frame over a minimal buffer needs 86 chunks,
 * so this proves reassembly across the mod-64 wrap. */
ZTEST(ble_peer_frame, test_frame_seq_wraps_mod64)
{
	uint8_t frame[MESHTASTIC_BLE_PEER_FRAME_MAX];
	struct meshtastic_ble_peer_reasm rs;
	unsigned int chunks;

	fill_pattern(frame, sizeof(frame));
	meshtastic_ble_peer_reasm_reset(&rs);
	chunks = frame_loopback(&rs, frame, sizeof(frame), MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF);

	/* FIRST carries 1 byte, the rest 3 each: 1 + ceil(254/3) = 86 > 64. */
	zassert_equal(chunks, 86U);
	zassert_equal(rs.frames, 1U);
}

/* Interleave refusal, the shape it actually takes here: a sender restart
 * (new FIRST) kills the stale partial instead of splicing two frames. */
ZTEST(ble_peer_frame, test_frame_first_aborts_stale_partial)
{
	uint8_t frame_a[100];
	uint8_t frame_b[40];
	struct meshtastic_ble_peer_chunker ck;
	struct meshtastic_ble_peer_reasm rs;
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	size_t out_len = 0U;
	int n;

	fill_pattern(frame_a, sizeof(frame_a));
	memset(frame_b, 0x5A, sizeof(frame_b));
	meshtastic_ble_peer_reasm_reset(&rs);

	/* Frame A: deliver only the FIRST chunk, then "restart". */
	zassert_ok(meshtastic_ble_peer_chunker_start(&ck, frame_a, sizeof(frame_a)));
	n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf));
	zassert_true(n > 0);
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, buf, (size_t)n, &out_len), 0);
	zassert_true(rs.active);

	(void)frame_loopback(&rs, frame_b, sizeof(frame_b), sizeof(buf));
	zassert_equal(rs.aborted, 1U);
	zassert_equal(rs.frames, 1U);
}

ZTEST(ble_peer_frame, test_frame_oversize_refused)
{
	struct meshtastic_ble_peer_chunker ck;
	struct meshtastic_ble_peer_reasm rs;
	uint8_t frame[MESHTASTIC_BLE_PEER_FRAME_MAX + 1U] = {0};
	uint8_t chunk[6];
	size_t out_len = 0U;

	/* Sender side refuses to start. */
	zassert_equal(meshtastic_ble_peer_chunker_start(&ck, frame, sizeof(frame)), -EMSGSIZE);
	zassert_equal(meshtastic_ble_peer_chunker_start(&ck, frame, 0U), -EINVAL);

	/* Receiver side refuses a declared length past FRAME_MAX outright —
	 * nothing left in flight to overflow later. */
	meshtastic_ble_peer_reasm_reset(&rs);
	chunk[0] = MESHTASTIC_BLE_PEER_CHUNK_FIRST;
	chunk[1] = 0x00; /* 256 LE */
	chunk[2] = 0x01;
	chunk[3] = 0xAB;
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, chunk, 4U, &out_len), -EMSGSIZE);
	zassert_false(rs.active);
	zassert_equal(rs.rejected, 1U);

	/* A declared length of zero is meaningless. */
	chunk[0] = MESHTASTIC_BLE_PEER_CHUNK_FIRST | MESHTASTIC_BLE_PEER_CHUNK_LAST;
	chunk[1] = 0x00;
	chunk[2] = 0x00;
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, chunk, 3U, &out_len), -EBADMSG);
	zassert_false(rs.active);
}

/* A non-FIRST chunk with nothing in flight means the FIRST was missed: there
 * is nothing it could be reassembled into, so it is refused, not buffered. */
ZTEST(ble_peer_frame, test_frame_missed_first_refused)
{
	struct meshtastic_ble_peer_reasm rs;
	uint8_t chunk[4] = {0x01U, 0xAA, 0xBB, 0xCC}; /* seq 1, no FIRST */
	size_t out_len = 0U;

	meshtastic_ble_peer_reasm_reset(&rs);
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, chunk, sizeof(chunk), &out_len),
		      -EBADMSG);
	zassert_equal(rs.rejected, 1U);
	zassert_false(rs.active);
}

ZTEST(ble_peer_frame, test_frame_seq_gap_aborts_then_recovers)
{
	uint8_t frame[60];
	struct meshtastic_ble_peer_chunker ck;
	struct meshtastic_ble_peer_reasm rs;
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	size_t out_len = 0U;
	int n;

	fill_pattern(frame, sizeof(frame));
	meshtastic_ble_peer_reasm_reset(&rs);

	/* FIRST (seq 0) accepted, seq 1 dropped on the floor, seq 2 arrives:
	 * a gap is proven, the partial can never be completed honestly. */
	zassert_ok(meshtastic_ble_peer_chunker_start(&ck, frame, sizeof(frame)));
	n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf));
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, buf, (size_t)n, &out_len), 0);
	n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf)); /* dropped */
	zassert_true(n > 0);
	n = meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf));
	zassert_true(n > 0);
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, buf, (size_t)n, &out_len), -EBADMSG);
	zassert_false(rs.active);
	zassert_equal(rs.aborted, 1U);

	/* The next clean frame goes through untouched by the wreckage. */
	(void)frame_loopback(&rs, frame, sizeof(frame), sizeof(buf));
	zassert_equal(rs.frames, 1U);
}

ZTEST(ble_peer_frame, test_frame_early_last_and_overrun_refused)
{
	struct meshtastic_ble_peer_reasm rs;
	uint8_t chunk[8];
	size_t out_len = 0U;

	/* LAST with fewer bytes than declared: the frame lied about itself. */
	meshtastic_ble_peer_reasm_reset(&rs);
	chunk[0] = MESHTASTIC_BLE_PEER_CHUNK_FIRST | MESHTASTIC_BLE_PEER_CHUNK_LAST;
	chunk[1] = 10U; /* declares 10 */
	chunk[2] = 0U;
	chunk[3] = 0xAA; /* delivers 1 */
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, chunk, 4U, &out_len), -EBADMSG);
	zassert_false(rs.active);
	zassert_equal(rs.aborted, 1U);

	/* More bytes than declared, within the FIRST chunk itself. */
	meshtastic_ble_peer_reasm_reset(&rs);
	chunk[0] = MESHTASTIC_BLE_PEER_CHUNK_FIRST;
	chunk[1] = 2U; /* declares 2 */
	chunk[2] = 0U;
	chunk[3] = 0xAA;
	chunk[4] = 0xBB;
	chunk[5] = 0xCC; /* delivers 3 */
	zassert_equal(meshtastic_ble_peer_reasm_ingest(&rs, chunk, 6U, &out_len), -EBADMSG);
	zassert_false(rs.active);
}

ZTEST(ble_peer_frame, test_frame_chunker_rejects_tiny_buffer)
{
	static const uint8_t frame[4] = {1U, 2U, 3U, 4U};
	struct meshtastic_ble_peer_chunker ck;
	uint8_t buf[MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF];

	zassert_ok(meshtastic_ble_peer_chunker_start(&ck, frame, sizeof(frame)));
	zassert_equal(meshtastic_ble_peer_chunker_next(&ck, buf,
						       MESHTASTIC_BLE_PEER_CHUNK_MIN_BUF - 1U),
		      -EINVAL);
	/* The refusal consumed nothing: the frame still chunks cleanly. */
	zassert_true(meshtastic_ble_peer_chunker_next(&ck, buf, sizeof(buf)) > 0);
}
