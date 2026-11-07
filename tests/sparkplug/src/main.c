/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * The gateway's Sparkplug B session core (meshtastic_sparkplug_session.c), driven with a fake
 * publish that decodes every message with the same schema.
 */

#include <string.h>

#include <zephyr/ztest.h>
#include <pb_decode.h>

#include "meshtastic_sparkplug_session.h"
#include "zephyrtastic/sparkplug.pb.h"

#define MAXMSG 64

struct msg {
	char topic[MT_SPB_TOPIC_MAX];
	int qos;
	zt_sparkplug_Payload p;
};

static struct msg msgs[MAXMSG];
static size_t nmsgs;
static int64_t now = 1794000000000LL;
static struct mt_spb_session sess;

static int fake_publish(void *ctx, const char *topic, const uint8_t *payload, size_t len,
			int qos, bool retain)
{
	ARG_UNUSED(ctx);
	zassert_false(retain, "Sparkplug node and device messages are never retained");
	zassert_true(nmsgs < MAXMSG);
	struct msg *m = &msgs[nmsgs++];
	pb_istream_t is = pb_istream_from_buffer(payload, len);

	strncpy(m->topic, topic, sizeof(m->topic) - 1);
	m->qos = qos;
	zassert_true(pb_decode(&is, zt_sparkplug_Payload_fields, &m->p), "%s does not decode", topic);
	return 0;
}

static int64_t fake_now(void)
{
	return now;
}

static const zt_sparkplug_Metric *metric(const struct msg *m, const char *name)
{
	for (size_t i = 0; i < m->p.metrics_count; i++) {
		if (strcmp(m->p.metrics[i].name, name) == 0) {
			return &m->p.metrics[i];
		}
	}
	return NULL;
}

static struct mt_spb_node node(uint32_t num, uint32_t heard_s, float snr)
{
	struct mt_spb_node n = {
		.num = num, .last_heard_uptime_s = heard_s, .last_heard_epoch = 1794000000U,
		.snr = snr, .has_hops = true, .hops_away = 1, .channel = 0, .hw_model = 43,
		.role = 0,
	};

	snprintf(n.long_name, sizeof(n.long_name), "node %x", num);
	snprintf(n.short_name, sizeof(n.short_name), "%04x", num & 0xffff);
	return n;
}

static void reset(void *f)
{
	ARG_UNUSED(f);
	nmsgs = 0;
	memset(msgs, 0, sizeof(msgs));
	zassert_ok(mt_spb_init(&sess, "zt", "gw1", fake_publish, NULL, fake_now, 300, "abc1234",
			       0x11111111));
}

static void connect(uint8_t bdseq)
{
	mt_spb_session_start(&sess, bdseq);
	mt_spb_connected(&sess);
}

ZTEST(sparkplug, test_ids_refused)
{
	struct mt_spb_session s;

	zassert_equal(mt_spb_init(&s, "a/b", "e", fake_publish, NULL, fake_now, 1, "", 1), -EINVAL);
	zassert_equal(mt_spb_init(&s, "g", "e+", fake_publish, NULL, fake_now, 1, "", 1), -EINVAL);
	zassert_equal(mt_spb_init(&s, "", "e", fake_publish, NULL, fake_now, 1, "", 1), -EINVAL);
}

ZTEST(sparkplug, test_will_is_ndeath_with_bdseq)
{
	char topic[MT_SPB_TOPIC_MAX];
	uint8_t buf[64];
	size_t len = sizeof(buf);
	zt_sparkplug_Payload p = {0};

	mt_spb_session_start(&sess, 7);
	zassert_ok(mt_spb_will(&sess, topic, sizeof(topic), buf, &len));
	zassert_str_equal(topic, "spBv1.0/zt/NDEATH/gw1");
	pb_istream_t is = pb_istream_from_buffer(buf, len);

	zassert_true(pb_decode(&is, zt_sparkplug_Payload_fields, &p));
	zassert_false(p.has_seq, "NDEATH carries no seq");
	zassert_equal(p.metrics_count, 1);
	zassert_str_equal(p.metrics[0].name, "bdSeq");
	zassert_equal(p.metrics[0].datatype, MT_SPB_INT64);
	zassert_equal(p.metrics[0].value.long_value, 7);
	zassert_ok(mt_spb_ncmd_topic(&sess, topic, sizeof(topic)));
	zassert_str_equal(topic, "spBv1.0/zt/NCMD/gw1");
}

ZTEST(sparkplug, test_births_then_data_by_exception)
{
	struct mt_spb_node nodes[3] = {node(0xa1, 100, 5.5f), node(0x11111111, 100, 0), node(0xb2, 0, 1)};

	mt_spb_update(&sess, nodes, 3, 110);       /* before CONNACK: remembered, nothing sent */
	zassert_equal(nmsgs, 0);
	connect(3);
	/* NBIRTH, then DBIRTH for a1 only: b2 never heard, 11111111 is the gateway itself */
	zassert_equal(nmsgs, 2);
	zassert_str_equal(msgs[0].topic, "spBv1.0/zt/NBIRTH/gw1");
	zassert_equal(msgs[0].p.seq, 0);
	zassert_equal(metric(&msgs[0], "bdSeq")->value.long_value, 3);
	zassert_equal(metric(&msgs[0], "Node Control/Rebirth")->datatype, MT_SPB_BOOLEAN);
	zassert_false(metric(&msgs[0], "Node Control/Rebirth")->value.boolean_value);
	zassert_str_equal(metric(&msgs[0], "gateway/node_id")->value.string_value, "11111111");
	zassert_equal(metric(&msgs[0], "gateway/devices")->value.long_value, 1);
	zassert_str_equal(msgs[1].topic, "spBv1.0/zt/DBIRTH/gw1/000000a1");
	zassert_equal(msgs[1].p.seq, 1);
	const zt_sparkplug_Metric *snr = metric(&msgs[1], "radio/snr");

	zassert_not_null(snr);
	zassert_equal(snr->datatype, MT_SPB_DOUBLE, "births carry datatypes");
	zassert_within(snr->value.double_value, 5.5, 1e-6);
	zassert_true(metric(&msgs[1], "position/latitude")->is_null, "no position: a null");
	zassert_equal(msgs[1].p.metrics_count, 11);
	for (size_t i = 0; i < msgs[1].p.metrics_count; i++) {
		zassert_true(msgs[1].p.metrics[i].has_timestamp, "every birth metric has a timestamp");
	}

	mt_spb_update(&sess, nodes, 3, 120);       /* nothing changed */
	zassert_equal(nmsgs, 2);
	nodes[0].snr = -3.25f;
	nodes[0].last_heard_uptime_s = 118;        /* heard again; uptime alone is not a metric */
	mt_spb_update(&sess, nodes, 3, 120);
	zassert_equal(nmsgs, 3);
	zassert_str_equal(msgs[2].topic, "spBv1.0/zt/DDATA/gw1/000000a1");
	zassert_equal(msgs[2].p.seq, 2);
	zassert_equal(msgs[2].p.metrics_count, 1, "only what changed");
	zassert_false(msgs[2].p.metrics[0].has_datatype, "no datatype outside births (6.4.6)");
	zassert_within(msgs[2].p.metrics[0].value.double_value, -3.25, 1e-6);
}

ZTEST(sparkplug, test_new_node_birth_and_count)
{
	struct mt_spb_node nodes[2] = {node(0xa1, 100, 1), node(0xc3, 100, 2)};

	connect(0);
	mt_spb_update(&sess, nodes, 1, 100);
	zassert_str_equal(msgs[1].topic, "spBv1.0/zt/DBIRTH/gw1/000000a1");
	zassert_str_equal(msgs[2].topic, "spBv1.0/zt/NDATA/gw1");
	zassert_equal(metric(&msgs[2], "gateway/devices")->value.long_value, 1);
	mt_spb_update(&sess, nodes, 2, 100);
	zassert_str_equal(msgs[3].topic, "spBv1.0/zt/DBIRTH/gw1/000000c3");
	zassert_equal(metric(&msgs[4], "gateway/devices")->value.long_value, 2);
	for (size_t i = 0; i < nmsgs; i++) {
		zassert_equal(msgs[i].p.seq, i, "one seq for the node and its devices");
	}
}

ZTEST(sparkplug, test_silence_and_departure_are_deaths)
{
	struct mt_spb_node nodes[2] = {node(0xa1, 100, 1), node(0xc3, 100, 2)};

	connect(0);
	mt_spb_update(&sess, nodes, 2, 100);
	size_t before = nmsgs;

	mt_spb_update(&sess, nodes, 2, 401);       /* 301 s silent > 300 */
	zassert_str_equal(msgs[before].topic, "spBv1.0/zt/DDEATH/gw1/000000a1");
	zassert_equal(msgs[before].p.metrics_count, 0);
	zassert_true(msgs[before].p.has_seq);
	zassert_str_equal(msgs[before + 1].topic, "spBv1.0/zt/DDEATH/gw1/000000c3");
	before = nmsgs;
	nodes[1].last_heard_uptime_s = 400;        /* c3 heard again: born again */
	mt_spb_update(&sess, nodes, 2, 401);
	zassert_str_equal(msgs[before].topic, "spBv1.0/zt/DBIRTH/gw1/000000c3");
	before = nmsgs;
	mt_spb_update(&sess, nodes, 0, 402);       /* forgotten by the NodeDB */
	zassert_str_equal(msgs[before].topic, "spBv1.0/zt/DDEATH/gw1/000000c3");
	zassert_equal(sess.stats.ddeaths, 3);
}

/* Eclipse Tahu's host's rebirth request, captured 2026-11-07: no datatype, as the spec asks
 * of commands. benchcast's first decoder read it as null; this one must not. */
static const uint8_t tahu_rebirth[] = {
	0x08, 0xc8, 0xc4, 0xc1, 0xba, 0x9b, 0x34, 0x12, 0x21, 0x0a, 0x14, 0x4e, 0x6f, 0x64,
	0x65, 0x20, 0x43, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x2f, 0x52, 0x65, 0x62, 0x69,
	0x72, 0x74, 0x68, 0x18, 0xc8, 0xc4, 0xc1, 0xba, 0x9b, 0x34, 0x38, 0x00, 0x70, 0x01,
};

ZTEST(sparkplug, test_rebirth_request)
{
	struct mt_spb_node nodes[1] = {node(0xa1, 100, 1)};

	connect(9);
	mt_spb_update(&sess, nodes, 1, 100);
	nodes[0].snr = 2;
	mt_spb_update(&sess, nodes, 1, 100);
	size_t before = nmsgs;

	zassert_equal(mt_spb_ncmd(&sess, tahu_rebirth, sizeof(tahu_rebirth)), 1);
	zassert_str_equal(msgs[before].topic, "spBv1.0/zt/NBIRTH/gw1");
	zassert_equal(msgs[before].p.seq, 0);
	zassert_equal(metric(&msgs[before], "bdSeq")->value.long_value, 9, "same bdSeq");
	zassert_str_equal(msgs[before + 1].topic, "spBv1.0/zt/DBIRTH/gw1/000000a1");
	zassert_within(metric(&msgs[before + 1], "radio/snr")->value.double_value, 2.0, 1e-6,
		       "the birth carries the current values");
	static const uint8_t garbage[] = {0xff, 0xff, 0xff};

	zassert_equal(mt_spb_ncmd(&sess, garbage, sizeof(garbage)), -EBADMSG);
	zassert_equal(sess.stats.rebirth_requests, 1);
}

ZTEST(sparkplug, test_seq_wraps)
{
	struct mt_spb_node nodes[1] = {node(0xa1, 100, 0)};

	connect(0);
	mt_spb_update(&sess, nodes, 1, 100);
	for (int i = 1; i < 300; i++) {
		nodes[0].snr = (float)i;
		mt_spb_update(&sess, nodes, 1, 100);
		if (nmsgs >= MAXMSG - 2) {
			uint8_t last = msgs[nmsgs - 1].p.seq;

			nmsgs = 0;
			memset(msgs, 0, sizeof(msgs));
			msgs[0].p.seq = last;           /* carry the last seq across the reset */
			nmsgs = 1;
		}
	}
	for (size_t i = 1; i < nmsgs; i++) {
		zassert_equal((msgs[i - 1].p.seq + 1) % 256, msgs[i].p.seq);
	}
	zassert_true(sess.stats.ddata >= 299);
}

ZTEST(sparkplug, test_table_full_is_counted)
{
	struct mt_spb_node nodes[6];

	for (int i = 0; i < 6; i++) {
		nodes[i] = node(0x100 + i, 100, 0);
	}
	connect(0);
	mt_spb_update(&sess, nodes, 6, 100);
	zassert_equal(sess.stats.dbirths, CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES);
	zassert_equal(sess.stats.table_full, 6 - CONFIG_MESHTASTIC_SPARKPLUG_MAX_DEVICES);
}

ZTEST_SUITE(sparkplug, NULL, NULL, reset, NULL, NULL);
