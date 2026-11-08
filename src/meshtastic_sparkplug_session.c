/*
 * SPDX-FileCopyrightText: Adam Roper
 * SPDX-License-Identifier: GPL-3.0
 *
 * The Sparkplug B session core of the mesh gateway; see meshtastic_sparkplug_session.h.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/sys/util.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_sparkplug_session.h"
#include "zephyrtastic/sparkplug.pb.h"

#define REBIRTH "Node Control/Rebirth"
#define BDSEQ "bdSeq"

/* One payload at a time: the session is driven from a single thread. */
static zt_sparkplug_Payload pb_msg;

/* --- building a payload ---------------------------------------------------------------- */

static void pl_begin(struct mt_spb_session *s, bool with_seq)
{
	memset(&pb_msg, 0, sizeof(pb_msg));
	pb_msg.has_timestamp = true;
	pb_msg.timestamp = (uint64_t)s->now_ms();
	if (with_seq) {
		pb_msg.has_seq = true;
		pb_msg.seq = s->seq;
		s->seq = (uint8_t)(s->seq + 1U);
	}
}

static zt_sparkplug_Metric *pl_metric(const char *name, uint32_t dt, bool typed)
{
	if (pb_msg.metrics_count >= ARRAY_SIZE(pb_msg.metrics)) {
		return NULL;
	}
	zt_sparkplug_Metric *m = &pb_msg.metrics[pb_msg.metrics_count++];

	m->has_name = true;
	strncpy(m->name, name, sizeof(m->name) - 1);
	m->has_timestamp = true;
	m->timestamp = pb_msg.timestamp;
	if (typed) {
		m->has_datatype = true;
		m->datatype = dt;
	}
	return m;
}

static void put_i64(const char *name, bool typed, bool null, int64_t v)
{
	zt_sparkplug_Metric *m = pl_metric(name, MT_SPB_INT64, typed);

	if (m == NULL) {
		return;
	}
	if (null) {
		m->has_is_null = true;
		m->is_null = true;
		return;
	}
	m->which_value = zt_sparkplug_Metric_long_value_tag;
	m->value.long_value = (uint64_t)v;
}

static void put_f64(const char *name, bool typed, bool null, double v)
{
	zt_sparkplug_Metric *m = pl_metric(name, MT_SPB_DOUBLE, typed);

	if (m == NULL) {
		return;
	}
	if (null) {
		m->has_is_null = true;
		m->is_null = true;
		return;
	}
	m->which_value = zt_sparkplug_Metric_double_value_tag;
	m->value.double_value = v;
}

static void put_bool(const char *name, bool typed, bool v)
{
	zt_sparkplug_Metric *m = pl_metric(name, MT_SPB_BOOLEAN, typed);

	if (m != NULL) {
		m->which_value = zt_sparkplug_Metric_boolean_value_tag;
		m->value.boolean_value = v;
	}
}

static void put_str(const char *name, bool typed, const char *v)
{
	zt_sparkplug_Metric *m = pl_metric(name, MT_SPB_STRING, typed);

	if (m != NULL) {
		m->which_value = zt_sparkplug_Metric_string_value_tag;
		strncpy(m->value.string_value, v, sizeof(m->value.string_value) - 1);
	}
}

static int pl_send(struct mt_spb_session *s, int qos)
{
	pb_ostream_t os = pb_ostream_from_buffer(s->buf, sizeof(s->buf));

	if (!pb_encode(&os, zt_sparkplug_Payload_fields, &pb_msg)) {
		s->stats.encode_errors++;
		return -ENOMEM;
	}
	if (s->publish(s->ctx, s->topic, s->buf, os.bytes_written, qos, false) != 0) {
		s->stats.publish_errors++;
		return -EIO;
	}
	return 0;
}

static int set_topic(struct mt_spb_session *s, const char *type, const char *device)
{
	int n = device ? snprintf(s->topic, sizeof(s->topic), "spBv1.0/%s/%s/%s/%s", s->group,
				  type, s->edge, device)
		       : snprintf(s->topic, sizeof(s->topic), "spBv1.0/%s/%s/%s", s->group, type,
				  s->edge);
	return (n > 0 && (size_t)n < sizeof(s->topic)) ? 0 : -ENAMETOOLONG;
}

/* --- the device's metrics ----------------------------------------------------------------- */

/* Every device carries the same metrics, so a birth never changes shape (no rebirth for a
 * metric appearing): absent values go as nulls of their type. `typed` for births. */
static void put_device(const struct mt_spb_node *n, bool typed, const struct mt_spb_node *old)
{
	/* old == NULL: all of them (a birth); else only what differs from old (DDATA) */
#define CHANGED(f) (old == NULL || old->f != n->f)
	if (old == NULL || strcmp(old->long_name, n->long_name) != 0) {
		put_str("node/long_name", typed, n->long_name);
	}
	if (old == NULL || strcmp(old->short_name, n->short_name) != 0) {
		put_str("node/short_name", typed, n->short_name);
	}
	if (CHANGED(hw_model)) {
		put_i64("node/hw_model", typed, false, n->hw_model);
	}
	if (CHANGED(role)) {
		put_i64("node/role", typed, false, n->role);
	}
	if (CHANGED(snr)) {
		put_f64("radio/snr", typed, false, (double)n->snr);
	}
	if (CHANGED(has_hops) || CHANGED(hops_away)) {
		put_i64("radio/hops_away", typed, !n->has_hops, n->hops_away);
	}
	if (CHANGED(channel)) {
		put_i64("radio/channel", typed, false, n->channel);
	}
	if (CHANGED(via_mqtt)) {
		put_bool("radio/via_mqtt", typed, n->via_mqtt);
	}
	if (CHANGED(last_heard_epoch)) {
		put_i64("heard/epoch_s", typed, n->last_heard_epoch == 0U, n->last_heard_epoch);
	}
	if (CHANGED(has_position) || CHANGED(latitude_i) || CHANGED(longitude_i)) {
		put_f64("position/latitude", typed, !n->has_position, n->latitude_i * 1e-7);
		put_f64("position/longitude", typed, !n->has_position, n->longitude_i * 1e-7);
	}
#undef CHANGED
}

void mt_spb_device_id(uint32_t num, char out[9])
{
	snprintf(out, 9, "%08x", num);
}

/* --- the session ----------------------------------------------------------------------------- */

bool mt_spb_valid_id(const char *id)
{
	if (id == NULL || id[0] == '\0' || strlen(id) >= MT_SPB_ID_MAX) {
		return false;
	}
	return strpbrk(id, "+/#") == NULL;
}

int mt_spb_init(struct mt_spb_session *s, const char *group, const char *edge,
		mt_spb_publish_fn publish, void *ctx, mt_spb_now_ms_fn now_ms,
		uint32_t death_after_s, const char *build, uint32_t own_num)
{
	if (!mt_spb_valid_id(group) || !mt_spb_valid_id(edge) || publish == NULL || now_ms == NULL) {
		return -EINVAL;
	}
	memset(s, 0, sizeof(*s));
	strncpy(s->group, group, sizeof(s->group) - 1);
	strncpy(s->edge, edge, sizeof(s->edge) - 1);
	s->publish = publish;
	s->ctx = ctx;
	s->now_ms = now_ms;
	s->death_after_s = death_after_s;
	strncpy(s->build, build ? build : "", sizeof(s->build) - 1);
	s->own_num = own_num;
	s->sent_devices = -1;
	return 0;
}

void mt_spb_session_start(struct mt_spb_session *s, uint8_t bdseq)
{
	s->bdseq = bdseq;
	s->born = false;
	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		s->devs[i].born = false;
	}
}

int mt_spb_will(struct mt_spb_session *s, char *topic, size_t topic_len, uint8_t *payload,
		size_t *payload_len)
{
	if (set_topic(s, "NDEATH", NULL) != 0 || strlen(s->topic) >= topic_len) {
		return -ENAMETOOLONG;
	}
	strcpy(topic, s->topic);
	pl_begin(s, false);
	put_i64(BDSEQ, true, false, s->bdseq);

	pb_ostream_t os = pb_ostream_from_buffer(payload, *payload_len);

	if (!pb_encode(&os, zt_sparkplug_Payload_fields, &pb_msg)) {
		return -ENOMEM;
	}
	*payload_len = os.bytes_written;
	return 0;
}

int mt_spb_ncmd_topic(const struct mt_spb_session *s, char *topic, size_t len)
{
	int n = snprintf(topic, len, "spBv1.0/%s/NCMD/%s", s->group, s->edge);

	return (n > 0 && (size_t)n < len) ? 0 : -ENAMETOOLONG;
}

static size_t live_count(const struct mt_spb_session *s)
{
	size_t n = 0;

	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		n += s->devs[i].used ? 1U : 0U;
	}
	return n;
}

static void send_dbirth(struct mt_spb_session *s, struct mt_spb_dev *d)
{
	char id[9];

	mt_spb_device_id(d->last.num, id);
	if (set_topic(s, "DBIRTH", id) != 0) {
		return;
	}
	pl_begin(s, true);
	put_device(&d->last, true, NULL);
	if (pl_send(s, 0) == 0) {
		d->born = true;
		s->stats.dbirths++;
	}
}

static void birth_all(struct mt_spb_session *s)
{
	char own[9];

	s->seq = 0;
	if (set_topic(s, "NBIRTH", NULL) != 0) {
		return;
	}
	mt_spb_device_id(s->own_num, own);
	pl_begin(s, true);
	put_i64(BDSEQ, true, false, s->bdseq);
	put_bool(REBIRTH, true, false);
	put_str("gateway/node_id", true, own);
	put_str("gateway/build", true, s->build);
	s->sent_devices = (int64_t)live_count(s);
	put_i64("gateway/devices", true, false, s->sent_devices);
	if (pl_send(s, 0) != 0) {
		return;
	}
	s->born = true;
	s->stats.nbirths++;
	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		s->devs[i].born = false;
		if (s->devs[i].used) {
			send_dbirth(s, &s->devs[i]);
		}
	}
}

void mt_spb_connected(struct mt_spb_session *s)
{
	birth_all(s);
}

void mt_spb_disconnected(struct mt_spb_session *s)
{
	s->born = false;
}

static struct mt_spb_dev *find(struct mt_spb_session *s, uint32_t num)
{
	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		if (s->devs[i].used && s->devs[i].last.num == num) {
			return &s->devs[i];
		}
	}
	return NULL;
}

static struct mt_spb_dev *alloc(struct mt_spb_session *s)
{
	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		if (!s->devs[i].used) {
			return &s->devs[i];
		}
	}
	return NULL;
}

static void ddeath(struct mt_spb_session *s, struct mt_spb_dev *d)
{
	char id[9];

	if (s->born && d->born) {
		mt_spb_device_id(d->last.num, id);
		if (set_topic(s, "DDEATH", id) == 0) {
			pl_begin(s, true);
			if (pl_send(s, 0) == 0) {
				s->stats.ddeaths++;
			}
		}
	}
	memset(d, 0, sizeof(*d));
}

static bool same(const struct mt_spb_node *a, const struct mt_spb_node *b)
{
	return a->snr == b->snr && a->has_hops == b->has_hops && a->hops_away == b->hops_away &&
	       a->channel == b->channel && a->via_mqtt == b->via_mqtt &&
	       a->last_heard_epoch == b->last_heard_epoch && a->hw_model == b->hw_model &&
	       a->role == b->role && a->has_position == b->has_position &&
	       a->latitude_i == b->latitude_i && a->longitude_i == b->longitude_i &&
	       strcmp(a->long_name, b->long_name) == 0 &&
	       strcmp(a->short_name, b->short_name) == 0;
}

void mt_spb_update(struct mt_spb_session *s, const struct mt_spb_node *nodes, size_t n,
		   uint32_t now_uptime_s)
{
	/* 1. deaths: silent too long, or gone from the snapshot */
	for (size_t i = 0; i < ARRAY_SIZE(s->devs); i++) {
		struct mt_spb_dev *d = &s->devs[i];
		bool present = false;
		bool alive = false;

		if (!d->used) {
			continue;
		}
		for (size_t k = 0; k < n; k++) {
			if (nodes[k].num == d->last.num) {
				present = true;
				alive = nodes[k].last_heard_uptime_s != 0U &&
					now_uptime_s - nodes[k].last_heard_uptime_s <= s->death_after_s;
				break;
			}
		}
		if (!present || !alive) {
			ddeath(s, d);
		}
	}
	/* 2. births and changes */
	for (size_t k = 0; k < n; k++) {
		const struct mt_spb_node *nd = &nodes[k];

		if (nd->num == s->own_num || nd->last_heard_uptime_s == 0U ||
		    now_uptime_s - nd->last_heard_uptime_s > s->death_after_s) {
			continue;
		}
		struct mt_spb_dev *d = find(s, nd->num);

		if (d == NULL) {
			d = alloc(s);
			if (d == NULL) {
				s->stats.table_full++;
				continue;
			}
			d->used = true;
			d->last = *nd;
			if (s->born) {
				send_dbirth(s, d);
			}
			continue;
		}
		if (s->born && !d->born) {
			d->last = *nd;
			send_dbirth(s, d);
			continue;
		}
		if (!same(&d->last, nd)) {
			if (s->born) {
				char id[9];

				mt_spb_device_id(nd->num, id);
				if (set_topic(s, "DDATA", id) == 0) {
					pl_begin(s, true);
					put_device(nd, false, &d->last);
					if (pl_send(s, 0) == 0) {
						s->stats.ddata++;
					}
				}
			}
			d->last = *nd;
		}
	}
	/* 3. the node's own count, by exception */
	int64_t count = (int64_t)live_count(s);

	if (s->born && count != s->sent_devices && set_topic(s, "NDATA", NULL) == 0) {
		pl_begin(s, true);
		put_i64("gateway/devices", false, false, count);
		if (pl_send(s, 0) == 0) {
			s->sent_devices = count;
			s->stats.ndata++;
		}
	}
}

int mt_spb_ncmd(struct mt_spb_session *s, const uint8_t *payload, size_t len)
{
	pb_istream_t is = pb_istream_from_buffer(payload, len);

	memset(&pb_msg, 0, sizeof(pb_msg));
	if (!pb_decode(&is, zt_sparkplug_Payload_fields, &pb_msg)) {
		return -EBADMSG;
	}
	for (size_t i = 0; i < pb_msg.metrics_count; i++) {
		const zt_sparkplug_Metric *m = &pb_msg.metrics[i];

		/* the value by its field, not its datatype: a command carries none (spec 6.4.6) */
		if (m->has_name && strcmp(m->name, REBIRTH) == 0 &&
		    m->which_value == zt_sparkplug_Metric_boolean_value_tag &&
		    m->value.boolean_value) {
			s->stats.rebirth_requests++;
			if (s->born) {
				birth_all(s);
			}
			return 1;
		}
	}
	return 0;
}
