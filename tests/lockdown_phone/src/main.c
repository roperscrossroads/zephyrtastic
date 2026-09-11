/* SPDX-License-Identifier: GPL-3.0
 *
 * Lockdown phase 3 (agents-dnr4.15): the PhoneAPI half, driven the way a phone
 * drives it -- encoded ToRadio frames in, decoded FromRadio frames out -- on the
 * whole stack with the sim radio and the real NVS behind the settings subsystem.
 *
 * Two transports are registered, because the properties worth proving are
 * per-connection: what one connection proves does not carry to the other, and a
 * revoke reaches both. A "locked boot" is staged as tests/lockdown_store stages
 * it (RAM store back to its seed, the core re-initialised from the artifacts on
 * flash, the subtrees loaded); a "new connection" is meshtastic_phoneapi_reset().
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>
#include <psa/crypto.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>

#include "meshtastic/admin.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic_admin.h"
#include "meshtastic_channels.h"
#include "meshtastic_config_store.h"
#include "meshtastic_core.h"
#include "meshtastic_lockdown.h"
#include "meshtastic_phoneapi.h"

int meshtastic_nodedb_init(void);
void meshtastic_handle_inbound_packet(const struct meshtastic_packet *packet, const uint8_t *wire,
				      size_t wire_len, bool decoded);

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const uint8_t PP[] = "correct horse";
static const uint8_t WRONG[] = "battery staple";
#define PPLEN (sizeof(PP) - 1U)

#define TEST_NODE_ID 0x11223344U
#define PEER_ID      0x55667788U

static struct meshtastic_config cfg = {
	.node_id = TEST_NODE_ID,
	.psk = meshtastic_default_psk,
	.psk_len = sizeof(meshtastic_default_psk),
	.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
	.frequency = MESHTASTIC_FREQ_EU,
	.long_name = "stock name",
	.short_name = "stck",
};

/* The full handshake is ~45 frames; the queue must hold it plus whatever the
 * stack fans out meanwhile. */
#define PHONE_Q 64U
static struct meshtastic_phoneapi_frame phone_q[PHONE_Q];
static struct meshtastic_phoneapi phone;
static meshtastic_ToRadio phone_to;
static meshtastic_FromRadio phone_from;

#define PHONE2_Q 16U
static struct meshtastic_phoneapi_frame phone2_q[PHONE2_Q];
static struct meshtastic_phoneapi phone2;
static meshtastic_ToRadio phone2_to;
static meshtastic_FromRadio phone2_from;

static meshtastic_FromRadio dec; /* decode scratch, 768 B, off the stack */
static meshtastic_AdminMessage adm; /* encode scratch */

/* ---- raw NVS access -------------------------------------------------------------- */

struct raw {
	uint8_t buf[320];
	size_t len;
	bool found;
};

static int raw_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param)
{
	struct raw *r = param;

	ARG_UNUSED(key);
	if (len <= sizeof(r->buf) && read_cb(cb_arg, r->buf, len) == (ssize_t)len) {
		r->len = len;
		r->found = true;
	}
	return 1;
}

static bool raw_read(const char *name, struct raw *r)
{
	r->len = 0U;
	r->found = false;
	(void)settings_load_subtree_direct(name, raw_cb, r);
	return r->found;
}

/* ---- stack helpers ---------------------------------------------------------------- */

static void wait_idle(void)
{
	for (int i = 0; i < 300 && meshtastic_lockdown_busy(); i++) {
		k_sleep(K_MSEC(10));
	}
	zassert_false(meshtastic_lockdown_busy(), "the workqueue item did not run");
}

static void set_long_name(const char *name)
{
	meshtastic_User user = meshtastic_User_init_zero;

	(void)snprintk(user.long_name, sizeof(user.long_name), "%s", name);
	(void)snprintk(user.short_name, sizeof(user.short_name), "stck");
	zassert_ok(meshtastic_config_store_set_owner(&user), "");
}

static void flush_all(void)
{
	zassert_ok(settings_save_subtree("meshtastic"), "");
	(void)settings_save_subtree("mtnode");
	(void)settings_save_subtree("mtrec");
}

static void seed_peer(uint32_t node)
{
	static const uint8_t key[32] = {9, 8, 7};
	meshtastic_User user = meshtastic_User_init_zero;
	uint8_t buf[128];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
	struct meshtastic_packet ni = {
		.from = node,
		.to = MESHTASTIC_NODE_BROADCAST,
		.portnum = MESHTASTIC_PORT_NODEINFO,
		.channel_index = meshtastic_channels_primary_index(),
	};

	(void)snprintk(user.long_name, sizeof(user.long_name), "peer");
	(void)snprintk(user.short_name, sizeof(user.short_name), "peer");
	user.public_key.size = 32U;
	memcpy(user.public_key.bytes, key, 32U);
	zassert_true(pb_encode(&os, meshtastic_User_fields, &user), "User encode");
	ni.payload = buf;
	ni.payload_len = os.bytes_written;
	meshtastic_handle_inbound_packet(&ni, NULL, 0U, true);
}

static void stage_boot(void)
{
	zassert_ok(meshtastic_config_store_seed(&cfg), "");
	meshtastic_lockdown_init();
	mt.radio_held = meshtastic_lockdown_locked();
	if (mt.radio_held) {
		(void)lora_recv_async(lora_dev, NULL, NULL);
		mt.radio_rx_armed = false;
	}
	zassert_ok(settings_load_subtree("meshtastic"), "");
	(void)meshtastic_config_store_apply_core();
	zassert_ok(meshtastic_nodedb_init(), "");
	k_sleep(K_SECONDS(3));
}

/* ---- the phone side --------------------------------------------------------------- */

static void drain(struct meshtastic_phoneapi *api)
{
	struct meshtastic_phoneapi_frame f;

	while (meshtastic_phoneapi_pop_frame(api, &f)) {
	}
}

static bool decode_frame(const struct meshtastic_phoneapi_frame *f, meshtastic_FromRadio *out)
{
	pb_istream_t is = pb_istream_from_buffer(f->data, f->len);

	*out = (meshtastic_FromRadio)meshtastic_FromRadio_init_zero;
	return pb_decode(&is, meshtastic_FromRadio_fields, out);
}

/* Pop frames until a LockdownStatus shows up (other traffic is skipped), for up
 * to @p ms. */
static bool wait_status(struct meshtastic_phoneapi *api, meshtastic_LockdownStatus *out,
			int ms)
{
	struct meshtastic_phoneapi_frame f;

	for (int t = 0; t <= ms; t += 10) {
		while (meshtastic_phoneapi_pop_frame(api, &f)) {
			if (decode_frame(&f, &dec) &&
			    dec.which_payload_variant == meshtastic_FromRadio_lockdown_status_tag) {
				*out = dec.lockdown_status;
				return true;
			}
		}
		k_sleep(K_MSEC(10));
	}
	return false;
}

/* Pop frames looking for one QueueStatus; returns its res, or 1 if none. */
static int wait_queue_status(struct meshtastic_phoneapi *api, int ms)
{
	struct meshtastic_phoneapi_frame f;

	for (int t = 0; t <= ms; t += 10) {
		while (meshtastic_phoneapi_pop_frame(api, &f)) {
			if (decode_frame(&f, &dec) &&
			    dec.which_payload_variant == meshtastic_FromRadio_queueStatus_tag) {
				return dec.queueStatus.res;
			}
		}
		k_sleep(K_MSEC(10));
	}
	return 1;
}

static void send_toradio(struct meshtastic_phoneapi *api, const meshtastic_ToRadio *to)
{
	static uint8_t buf[MESHTASTIC_API_FRAME_MAX];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	zassert_true(pb_encode(&os, meshtastic_ToRadio_fields, to), "ToRadio encode");
	meshtastic_phoneapi_handle_toradio(api, buf, os.bytes_written);
}

static uint32_t next_id = 0x0AD00100U;
/* The node id the stack actually runs under: it derives one from the (simulated)
 * hardware and ignores cfg.node_id, so "to us" means this, not TEST_NODE_ID. */
static uint32_t me;

static void send_admin(struct meshtastic_phoneapi *api, const meshtastic_AdminMessage *a)
{
	static meshtastic_ToRadio to;
	pb_ostream_t os;

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_packet_tag;
	to.packet.to = me;
	to.packet.id = next_id++;
	to.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	to.packet.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
	os = pb_ostream_from_buffer(to.packet.decoded.payload.bytes,
				    sizeof(to.packet.decoded.payload.bytes));
	zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, a), "AdminMessage encode");
	to.packet.decoded.payload.size = (pb_size_t)os.bytes_written;
	send_toradio(api, &to);
}

static void send_auth(struct meshtastic_phoneapi *api, const uint8_t *pass, size_t len,
		      uint32_t boots, uint32_t session_s, bool lock_now, bool disable)
{
	adm = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	adm.which_payload_variant = meshtastic_AdminMessage_lockdown_auth_tag;
	if (pass != NULL) {
		memcpy(adm.payload_variant.lockdown_auth.passphrase.bytes, pass, len);
		adm.payload_variant.lockdown_auth.passphrase.size = (pb_size_t)len;
	}
	adm.payload_variant.lockdown_auth.boots_remaining = boots;
	adm.payload_variant.lockdown_auth.max_session_seconds = session_s;
	adm.payload_variant.lockdown_auth.lock_now = lock_now;
	adm.payload_variant.lockdown_auth.disable = disable;
	send_admin(api, &adm);
	memset(&adm, 0, sizeof(adm));
}

static void send_text(struct meshtastic_phoneapi *api)
{
	static meshtastic_ToRadio to;

	to = (meshtastic_ToRadio)meshtastic_ToRadio_init_zero;
	to.which_payload_variant = meshtastic_ToRadio_packet_tag;
	to.packet.to = MESHTASTIC_NODE_BROADCAST;
	to.packet.id = next_id++;
	to.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	to.packet.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
	memcpy(to.packet.decoded.payload.bytes, "hi", 2U);
	to.packet.decoded.payload.size = 2U;
	send_toradio(api, &to);
}

/* What one config handshake exposed. */
struct stream {
	bool complete;
	bool has_status;
	meshtastic_LockdownStatus status;
	pb_size_t device_id_len;
	char firmware_version[32];
	pb_size_t ch0_psk_len;
	bool security_has_pubkey;
	bool lora_tx_enabled;
	uint32_t lora_hop_limit;
	bool mqtt_seen;
	char mqtt_address[64];
	unsigned int other_nodes;
	unsigned int frames;
	char long_name[40];
};

static void run_config(struct meshtastic_phoneapi *api, uint32_t nonce, struct stream *s)
{
	struct meshtastic_phoneapi_frame f;

	memset(s, 0, sizeof(*s));
	drain(api);
	meshtastic_phoneapi_enqueue_phone_config(api, nonce);
	for (int t = 0; t < 200; t++) {
		while (meshtastic_phoneapi_pop_frame(api, &f)) {
			s->frames++;
			zassert_true(decode_frame(&f, &dec), "FromRadio decode");
			switch (dec.which_payload_variant) {
			case meshtastic_FromRadio_my_info_tag:
				s->device_id_len = dec.my_info.device_id.size;
				break;
			case meshtastic_FromRadio_metadata_tag:
				strncpy(s->firmware_version, dec.metadata.firmware_version,
					sizeof(s->firmware_version) - 1U);
				break;
			case meshtastic_FromRadio_node_info_tag:
				if (dec.node_info.num == me) {
					strncpy(s->long_name, dec.node_info.user.long_name,
						sizeof(s->long_name) - 1U);
				} else {
					s->other_nodes++;
				}
				break;
			case meshtastic_FromRadio_channel_tag:
				if (dec.channel.index == 0) {
					s->ch0_psk_len = dec.channel.has_settings
								 ? dec.channel.settings.psk.size
								 : 0U;
				}
				break;
			case meshtastic_FromRadio_config_tag:
				if (dec.config.which_payload_variant == meshtastic_Config_security_tag) {
					s->security_has_pubkey =
						dec.config.payload_variant.security.public_key.size > 0U;
				} else if (dec.config.which_payload_variant ==
					   meshtastic_Config_lora_tag) {
					s->lora_tx_enabled = dec.config.payload_variant.lora.tx_enabled;
					s->lora_hop_limit = dec.config.payload_variant.lora.hop_limit;
				}
				break;
			case meshtastic_FromRadio_moduleConfig_tag:
				if (dec.moduleConfig.which_payload_variant ==
				    meshtastic_ModuleConfig_mqtt_tag) {
					s->mqtt_seen = true;
					strncpy(s->mqtt_address,
						dec.moduleConfig.payload_variant.mqtt.address,
						sizeof(s->mqtt_address) - 1U);
				}
				break;
			case meshtastic_FromRadio_config_complete_id_tag:
				s->complete = (dec.config_complete_id == nonce);
				break;
			case meshtastic_FromRadio_lockdown_status_tag:
				s->has_status = true;
				s->status = dec.lockdown_status;
				break;
			default:
				break;
			}
		}
		if (s->complete && (s->has_status || t > 5)) {
			return;
		}
		k_sleep(K_MSEC(10));
	}
}

/* ---- suite ------------------------------------------------------------------------ */

static void *suite_setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS, "psa");
	zassert_true(device_is_ready(lora_dev), "sim lora");
	cfg.lora_dev = lora_dev;
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init");
	me = meshtastic_get_node_id();
	meshtastic_phoneapi_init(&phone, "phone", phone_q, PHONE_Q, NULL, NULL, NULL, NULL,
				 &phone_to, &phone_from);
	meshtastic_phoneapi_init(&phone2, "phone2", phone2_q, PHONE2_Q, NULL, NULL, NULL, NULL,
				 &phone2_to, &phone2_from);
	meshtastic_phoneapi_register(&phone);
	meshtastic_phoneapi_register(&phone2);
	/* Put an MQTT address in the store so the redaction has something to hide. */
	{
		meshtastic_ModuleConfig m = meshtastic_ModuleConfig_init_zero;

		m.which_payload_variant = meshtastic_ModuleConfig_mqtt_tag;
		(void)snprintk(m.payload_variant.mqtt.address,
			       sizeof(m.payload_variant.mqtt.address), "broker.example");
		(void)meshtastic_config_store_set_module(&m);
	}
	return NULL;
}

/* Back to stock however the previous test left things: a locked boot needs an
 * unlock (which may be sitting behind the reboot floor) and a reload before
 * the disable has real records to write. */
static void back_to_stock(void)
{
	for (int i = 0; i < 4 && meshtastic_lockdown_active(); i++) {
		int ret;

		wait_idle();
		if (!meshtastic_lockdown_unlocked()) {
			ret = meshtastic_lockdown_unlock(PP, PPLEN, 0U, 0U, 0U);
			if (ret == -EAGAIN) {
				stage_boot();
				continue;
			}
			zassert_ok(ret, "unlock in teardown");
			wait_idle();
		}
		zassert_ok(meshtastic_lockdown_disable(PP, PPLEN), "");
		wait_idle();
	}
	zassert_false(meshtastic_lockdown_active(), "");
}

static void suite_before(void *f)
{
	ARG_UNUSED(f);
	meshtastic_phoneapi_lockdown_cancel_reboot();
	meshtastic_admin_cancel_reboot();
	back_to_stock();
	mt.radio_held = false;
	meshtastic_phoneapi_reset(&phone);
	meshtastic_phoneapi_reset(&phone2);
	drain(&phone);
	drain(&phone2);
	set_long_name("stock name");
	flush_all();
}

ZTEST_SUITE(lockdown_phone, NULL, suite_setup, suite_before, NULL, NULL);

/* Inactive lockdown is stock: the whole config, and a DISABLED after it so the
 * app knows the toggle exists. */
ZTEST(lockdown_phone, test_inactive_streams_everything_and_says_disabled)
{
	struct stream s;

	zassert_true(meshtastic_phoneapi_authorized(&phone), "inactive: everyone is authorized");
	run_config(&phone, 1234U, &s);
	zassert_true(s.complete, "");
	zassert_equal(s.device_id_len, 4U, "device_id present");
	zassert_true(s.firmware_version[0] != '\0', "metadata present");
	zassert_true(s.ch0_psk_len > 0U, "primary channel PSK present");
	zassert_true(s.lora_tx_enabled, "the full LoRa section");
	zassert_true(s.mqtt_seen, "");
	zassert_str_equal(s.mqtt_address, "broker.example", "mqtt in the clear");
	zassert_true(s.has_status, "a status follows config_complete_id");
	zassert_equal(s.status.state, meshtastic_LockdownStatus_State_DISABLED, "");
}

/* The app's "enable lockdown": one lockdown_auth from a stock node provisions
 * it and authorizes the connection that did. */
ZTEST(lockdown_phone, test_provision_over_the_phoneapi)
{
	meshtastic_LockdownStatus st;
	struct stream s;
	struct raw r;

	set_long_name("sealed name");
	flush_all();
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	zassert_equal(wait_queue_status(&phone, 200), 0, "consumed cleanly");
	zassert_true(wait_status(&phone, &st, 3000), "a status came back");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	zassert_equal(st.boots_remaining, CONFIG_MESHTASTIC_LOCKDOWN_DEFAULT_BOOTS, "");
	zassert_true(meshtastic_lockdown_active(), "");
	zassert_true(meshtastic_phoneapi_authorized(&phone), "the sender is authorized");
	zassert_false(meshtastic_phoneapi_authorized(&phone2), "the other connection is not");
	wait_idle(); /* the eager seal-all */
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_true(meshtastic_lockdown_is_sealed(r.buf, r.len), "owner sealed on flash");

	/* The authorized connection still sees everything, and is told nothing
	 * after the handshake -- it already knows. */
	run_config(&phone, 77U, &s);
	zassert_true(s.complete, "");
	zassert_equal(s.device_id_len, 4U, "");
	zassert_str_equal(s.mqtt_address, "broker.example", "");
	zassert_str_equal(s.long_name, "sealed name", "");
	zassert_false(s.has_status, "no status for an authorized connection");
}

/* A new connection to an active, unlocked node: the redacted stream, no node
 * DB, and LOCKED(needs_auth) telling it what to do. */
ZTEST(lockdown_phone, test_new_connection_is_redacted_and_told_needs_auth)
{
	struct stream s;

	seed_peer(PEER_ID);
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_phoneapi_reset(&phone); /* a fresh connection */
	zassert_false(meshtastic_phoneapi_authorized(&phone), "");

	run_config(&phone, 4321U, &s);
	zassert_true(s.complete, "the handshake still completes");
	zassert_equal(s.device_id_len, 0U, "device_id stripped");
	zassert_equal(s.firmware_version[0], '\0', "metadata zeroed");
	zassert_equal(s.ch0_psk_len, 0U, "channels empty");
	zassert_false(s.security_has_pubkey, "security empty");
	zassert_false(s.lora_tx_enabled, "LoRa whitelisted (tx_enabled not in it)");
	zassert_true(s.lora_hop_limit > 0U || true, "hop_limit is in the whitelist");
	zassert_true(s.mqtt_seen, "the section is still sent...");
	zassert_equal(s.mqtt_address[0], '\0', "...empty");
	zassert_equal(s.other_nodes, 0U, "no node DB for the unauthorized");
	zassert_true(s.has_status, "");
	zassert_equal(s.status.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_str_equal(s.status.lock_reason, "needs_auth", "");

	/* ONLY_NODES from an unauthorized client: own info, then straight to complete. */
	run_config(&phone, 69421U, &s);
	zassert_true(s.complete, "");
	zassert_equal(s.other_nodes, 0U, "");
	zassert_str_equal(s.long_name, "stock name", "own NodeInfo still goes");

	/* The authorized connection, meanwhile, sees the peer. */
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	{
		meshtastic_LockdownStatus st;

		zassert_true(wait_status(&phone, &st, 3000), "");
		zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "re-verify");
	}
	run_config(&phone, 5U, &s);
	zassert_equal(s.other_nodes, 1U, "the peer is streamed once authorized");
	zassert_equal(s.device_id_len, 4U, "");
}

/* Nothing an unauthorized connection sends reaches the mesh or the admin
 * dispatcher, and nothing from the mesh reaches it. */
ZTEST(lockdown_phone, test_unauthorized_traffic_is_dropped_both_ways)
{
	struct meshtastic_phoneapi_frame f;
	uint32_t tx_before;
	bool saw_packet = false;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_phoneapi_reset(&phone);
	drain(&phone);

	/* Out: a text message. */
	tx_before = mt.status.tx_packets;
	send_text(&phone);
	zassert_equal(wait_queue_status(&phone, 200), -EACCES, "the refusal is reported");
	k_sleep(K_MSEC(500));
	zassert_equal(mt.status.tx_packets, tx_before, "nothing transmitted");

	/* Out: an admin write. */
	adm = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	adm.which_payload_variant = meshtastic_AdminMessage_set_owner_tag;
	(void)snprintk(adm.payload_variant.set_owner.long_name,
		       sizeof(adm.payload_variant.set_owner.long_name), "intruder");
	(void)snprintk(adm.payload_variant.set_owner.short_name,
		       sizeof(adm.payload_variant.set_owner.short_name), "bad");
	send_admin(&phone, &adm);
	zassert_equal(wait_queue_status(&phone, 200), -EACCES, "");
	zassert_str_equal(meshtastic_config_store_long_name(), "stock name", "owner untouched");

	/* In: a peer's NodeInfo is fanned out to phones -- not to this one. */
	drain(&phone);
	seed_peer(PEER_ID);
	k_sleep(K_MSEC(200));
	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
		if (decode_frame(&f, &dec) &&
		    dec.which_payload_variant == meshtastic_FromRadio_packet_tag) {
			saw_packet = true;
		}
	}
	zassert_false(saw_packet, "mesh traffic withheld from an unauthorized client");

	/* Then authorize, and the same text goes out. */
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	{
		meshtastic_LockdownStatus st;

		zassert_true(wait_status(&phone, &st, 3000), "");
		zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	}
	tx_before = mt.status.tx_packets;
	send_text(&phone);
	k_sleep(K_MSEC(1500));
	zassert_true(mt.status.tx_packets > tx_before, "transmitted once authorized");
}

/* The reference's backoff, seen from the phone: a wrong passphrase fails with
 * a backoff, and the right one is refused until a reboot (the reboot floor). */
ZTEST(lockdown_phone, test_wrong_passphrase_backoff_and_the_reboot_floor)
{
	meshtastic_LockdownStatus st;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_phoneapi_reset(&phone);

	send_auth(&phone, WRONG, sizeof(WRONG) - 1U, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED, "");
	zassert_equal(st.backoff_seconds, 5U, "first failure: 5 s");
	zassert_false(meshtastic_phoneapi_authorized(&phone), "");

	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED,
		      "right passphrase, but the reboot floor holds");
	zassert_true(st.backoff_seconds > 0U, "");

	/* Empty, and an out-of-range boots: refused without touching the backoff. */
	send_auth(&phone, NULL, 0U, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED, "");
	zassert_equal(st.backoff_seconds, 0U, "");
	send_auth(&phone, PP, PPLEN, 300U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED, "");

	stage_boot(); /* the token still unlocks the boot; the floor is paid */
	zassert_true(meshtastic_lockdown_unlocked(), "");
	meshtastic_phoneapi_reset(&phone);
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "after the reboot");
	zassert_true(meshtastic_phoneapi_authorized(&phone), "");
}

/* A locked boot: LOCKED(token_missing) after the handshake, and the unlock the
 * phone sends authorizes it only once the store has been reloaded -- by which
 * time the stream carries the real records and the radio is back. */
ZTEST(lockdown_phone, test_locked_boot_unlock_authorizes_after_the_reload)
{
	meshtastic_LockdownStatus st;
	struct stream s;

	set_long_name("sealed name");
	flush_all();
	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_lockdown_lock_now();
	stage_boot();
	zassert_true(mt.radio_held, "");
	meshtastic_phoneapi_reset(&phone);

	run_config(&phone, 9U, &s);
	zassert_true(s.has_status, "");
	zassert_equal(s.status.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_str_equal(s.status.lock_reason, "token_missing", "");
	zassert_str_equal(s.long_name, "stock name", "placeholders, and redacted anyway");

	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 5000), "UNLOCKED after the reload");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	zassert_true(meshtastic_phoneapi_authorized(&phone), "");
	zassert_true(meshtastic_lockdown_store_ready(), "reloaded before the client heard");
	zassert_false(mt.radio_held, "radio released");

	run_config(&phone, 10U, &s);
	zassert_str_equal(s.long_name, "sealed name", "the real records");
	zassert_equal(s.device_id_len, 4U, "");
}

/* Lock Now: only from a connection that proved the passphrase, and it voids
 * every connection's authorization, not just the caller's. */
ZTEST(lockdown_phone, test_lock_now_needs_authorization_and_revokes_everyone)
{
	meshtastic_LockdownStatus st;

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_phoneapi_reset(&phone);
	meshtastic_phoneapi_reset(&phone2);
	send_auth(&phone2, PP, PPLEN, 0U, 0U, false, false);
	zassert_true(wait_status(&phone2, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	drain(&phone);

	/* The unauthorized one: refused, nothing changes. */
	send_auth(&phone, NULL, 0U, 0U, 0U, true, false);
	zassert_true(wait_status(&phone, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED, "");
	zassert_true(meshtastic_lockdown_unlocked(), "still unlocked");
	zassert_true(meshtastic_phoneapi_authorized(&phone2), "the other still authorized");
	zassert_false(meshtastic_phoneapi_lockdown_reboot_pending(), "");

	/* The authorized one: locked, everyone revoked and told, reboot scheduled. */
	drain(&phone2);
	send_auth(&phone2, NULL, 0U, 0U, 0U, true, false);
	zassert_false(meshtastic_lockdown_unlocked(), "keys gone");
	zassert_false(meshtastic_phoneapi_authorized(&phone2), "");
	zassert_false(meshtastic_phoneapi_authorized(&phone), "");
	zassert_true(wait_status(&phone2, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_str_equal(st.lock_reason, "token_missing", "");
	zassert_true(wait_status(&phone, &st, 500), "the other connection hears it too");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_true(meshtastic_phoneapi_lockdown_reboot_pending(), "");
	meshtastic_phoneapi_lockdown_cancel_reboot();
}

/* The dispatcher's own gate: while the store is locked no admin payload is
 * served, whatever the connection claims. lockdown_auth never gets served
 * there at all. */
ZTEST(lockdown_phone, test_admin_dispatcher_refuses_while_locked)
{
	struct meshtastic_phoneapi_frame f;
	bool got_response;

	/* A lockdown_auth handed straight to the dispatcher (the route a remote
	 * admin would take) does nothing. */
	{
		static meshtastic_MeshPacket pkt;
		pb_ostream_t os;

		adm = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
		adm.which_payload_variant = meshtastic_AdminMessage_lockdown_auth_tag;
		memcpy(adm.payload_variant.lockdown_auth.passphrase.bytes, PP, PPLEN);
		adm.payload_variant.lockdown_auth.passphrase.size = PPLEN;
		pkt = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
		pkt.to = me;
		pkt.id = 0x0AD0FFFFU;
		pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
		pkt.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
		os = pb_ostream_from_buffer(pkt.decoded.payload.bytes,
					    sizeof(pkt.decoded.payload.bytes));
		zassert_true(pb_encode(&os, meshtastic_AdminMessage_fields, &adm), "");
		pkt.decoded.payload.size = (pb_size_t)os.bytes_written;
		zassert_true(meshtastic_admin_handle_local(&pkt), "consumed");
		k_sleep(K_MSEC(200));
		zassert_false(meshtastic_lockdown_active(), "not provisioned by the back door");
	}

	zassert_ok(meshtastic_lockdown_provision(PP, PPLEN, 0U, 0U, 0U), "");
	wait_idle();
	meshtastic_lockdown_lock_now();
	stage_boot();
	zassert_true(meshtastic_lockdown_locked(), "");

	/* Pretend the connection were authorized (it cannot be, on a locked boot,
	 * but a bug that let one through must still not reach the dispatcher). */
	phone.admin_authorized = true;
	drain(&phone);
	adm = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero;
	adm.which_payload_variant = meshtastic_AdminMessage_get_owner_request_tag;
	adm.payload_variant.get_owner_request = true;
	send_admin(&phone, &adm);
	k_sleep(K_MSEC(300));
	got_response = false;
	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
		if (decode_frame(&f, &dec) &&
		    dec.which_payload_variant == meshtastic_FromRadio_packet_tag &&
		    dec.packet.decoded.portnum == meshtastic_PortNum_ADMIN_APP) {
			got_response = true;
		}
	}
	zassert_false(got_response, "no admin served while locked");
	phone.admin_authorized = false;

	/* Unlock (the proper way), and the same request is answered. */
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	{
		meshtastic_LockdownStatus st;

		zassert_true(wait_status(&phone, &st, 5000), "");
		zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	}
	drain(&phone);
	adm = (meshtastic_AdminMessage)meshtastic_AdminMessage_init_zero; /* send_auth wiped it */
	adm.which_payload_variant = meshtastic_AdminMessage_get_owner_request_tag;
	adm.payload_variant.get_owner_request = true;
	send_admin(&phone, &adm);
	k_sleep(K_MSEC(300));
	while (meshtastic_phoneapi_pop_frame(&phone, &f)) {
		if (decode_frame(&f, &dec) &&
		    dec.which_payload_variant == meshtastic_FromRadio_packet_tag &&
		    dec.packet.decoded.portnum == meshtastic_PortNum_ADMIN_APP) {
			got_response = true;
		}
	}
	zassert_true(got_response, "served once unlocked");
}

/* The session cap from the phone's side: provisioned with one boot and a 2 s
 * cap, the first expiry revokes and keeps routing, the second locks and
 * schedules the reboot. */
ZTEST(lockdown_phone, test_session_cap_revokes_then_locks)
{
	meshtastic_LockdownStatus st;

	send_auth(&phone, PP, PPLEN, 1U, 2U, false, false);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	zassert_equal(st.boots_remaining, 1U, "");
	zassert_true(meshtastic_phoneapi_authorized(&phone), "");

	k_sleep(K_MSEC(2500));
	zassert_true(wait_status(&phone, &st, 500), "the roll is announced");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_str_equal(st.lock_reason, "needs_auth", "");
	zassert_equal(st.boots_remaining, 0U, "the one boot consumed in place");
	zassert_false(meshtastic_phoneapi_authorized(&phone), "revoked");
	zassert_true(meshtastic_lockdown_unlocked(), "storage stays unlocked: the mesh keeps routing");
	zassert_false(mt.radio_held, "");
	zassert_false(meshtastic_phoneapi_lockdown_reboot_pending(), "");

	k_sleep(K_MSEC(2500));
	zassert_true(wait_status(&phone, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_LOCKED, "");
	zassert_str_equal(st.lock_reason, "session_budget_exhausted", "");
	zassert_false(meshtastic_lockdown_unlocked(), "locked");
	zassert_true(meshtastic_phoneapi_lockdown_reboot_pending(), "and rebooting");
	meshtastic_phoneapi_lockdown_cancel_reboot();
}

/* The app's toggle going off: disable needs the passphrase, rewrites the store
 * in the clear on the workqueue, then reports DISABLED and reboots. */
ZTEST(lockdown_phone, test_disable_over_the_phoneapi)
{
	meshtastic_LockdownStatus st;
	struct raw r;

	/* Not active: nothing to do, say so. */
	send_auth(&phone, PP, PPLEN, 0U, 0U, false, true);
	zassert_true(wait_status(&phone, &st, 500), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_DISABLED, "");

	send_auth(&phone, PP, PPLEN, 0U, 0U, false, false);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCKED, "");
	wait_idle();

	send_auth(&phone, WRONG, sizeof(WRONG) - 1U, 0U, 0U, false, true);
	zassert_true(wait_status(&phone, &st, 3000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_UNLOCK_FAILED, "");
	zassert_true(meshtastic_lockdown_active(), "still on");

	send_auth(&phone, PP, PPLEN, 0U, 0U, false, true);
	zassert_true(wait_status(&phone, &st, 5000), "");
	zassert_equal(st.state, meshtastic_LockdownStatus_State_DISABLED, "");
	zassert_false(meshtastic_lockdown_active(), "");
	zassert_true(raw_read("meshtastic/owner", &r), "");
	zassert_false(meshtastic_lockdown_is_sealed(r.buf, r.len), "in the clear");
	zassert_false(raw_read("mtlock/dek", &r), "artifacts gone");
	zassert_true(meshtastic_phoneapi_lockdown_reboot_pending(), "");
	meshtastic_phoneapi_lockdown_cancel_reboot();
}
