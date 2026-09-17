/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The receive-side XEdDSA policy gate (reference: Router.cpp checkXeddsaReceivePolicy).
 *
 * tests/xeddsa proves the primitive agrees with upstream's signer. This suite is about what
 * the ROUTER does with the answer:
 *   - a genuine signature from a node whose key we hold is accepted and marked signed;
 *   - a tampered one is DROPPED, not merely "not marked";
 *   - a signature length that is neither 0 nor 64 is dropped;
 *   - an unsigned packet is accepted under COMPATIBLE/BALANCED and dropped under STRICT,
 *     unless PKC already authenticated it;
 *   - a signed packet from a stranger cannot be verified: accepted unless STRICT;
 *   - a first-contact NodeInfo bootstraps the sender's key, but ONLY when the id commits to
 *     that key -- otherwise it is dropped;
 *   - an inbound xeddsa_signed flag is never believed.
 *
 * The gate is driven directly, with the stack initialised (it reads SecurityConfig and the
 * NodeDB), because the thing under test is a decision, not a radio path.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>
#include <zephyr/meshtastic/nodedb.h>
#include <meshtastic/lora_sim.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic_config_store.h"
#include "meshtastic_packet.h"
#include "meshtastic_router.h"
#include "meshtastic_core.h"
#include "meshtastic_xeddsa.h"
#include "vectors/meshtastic_xeddsa_vectors.h"

#define TEST_NODE_ID 0x5A5A0001U

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* The vectors, by name: each is a genuine upstream signature over its own bytes. */
static const struct mt_xeddsa_vector *vec(const char *label)
{
	for (size_t i = 0; i < ARRAY_SIZE(mt_xeddsa_vectors); i++) {
		if (strcmp(mt_xeddsa_vectors[i].label, label) == 0) {
			return &mt_xeddsa_vectors[i];
		}
	}
	zassert_unreachable("no vector %s", label);
	return NULL;
}

static void set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy policy)
{
	meshtastic_Config cfg;

	zassert_ok(meshtastic_config_store_get_config(meshtastic_Config_security_tag, &cfg));
	cfg.which_payload_variant = meshtastic_Config_security_tag;
	cfg.payload_variant.security.packet_signature_policy = policy;
	zassert_ok(meshtastic_config_store_set_config(&cfg));
}

/* Build the packet a vector describes, plus the MeshPacket carrying its signature. Callers
 * mutate one field to make a case. */
static void make(const struct mt_xeddsa_vector *v, struct meshtastic_packet *pkt,
		 meshtastic_MeshPacket *mesh, bool with_signature)
{
	*pkt = (struct meshtastic_packet){
		.from = v->from,
		.to = MESHTASTIC_NODE_BROADCAST,
		.id = v->id,
		.portnum = v->portnum,
		.payload = v->payload,
		.payload_len = v->payload_len,
	};
	*mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
	mesh->from = v->from;
	mesh->id = v->id;
	mesh->decoded.portnum = (meshtastic_PortNum)v->portnum;
	if (with_signature) {
		mesh->decoded.xeddsa_signature.size = 64;
		memcpy(mesh->decoded.xeddsa_signature.bytes, v->sig, 64);
	}
}

/* Give the NodeDB a key for this sender. commit_pubkey answers -ENOENT for a node it has
 * never seen, so the node must exist first -- the NodeDB learns nodes from delivered
 * packets, which is exactly what the gate runs before. */
static void seed_key(const struct mt_xeddsa_vector *v)
{
	meshtastic_User user = meshtastic_User_init_zero;

	user.public_key.size = 32;
	memcpy(user.public_key.bytes, v->x_pub, 32);
	zassert_ok(meshtastic_nodedb_add_contact(v->from, &user, false, false),
		   "could not seed a NodeDB entry for 0x%08x", v->from);
	zassert_ok(meshtastic_nodedb_commit_pubkey(v->from, v->x_pub));
}

static void *setup(void)
{
	static struct meshtastic_config cfg = {
		.lora_dev = lora_dev,
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	zassert_true(device_is_ready(lora_dev), "sim lora device not ready");
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed");
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_nodedb_reset(false);
	set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_COMPATIBLE);
}

ZTEST_SUITE(xeddsa_rx, NULL, setup, before, NULL, NULL);

/* --- a known signer ------------------------------------------------------------------ */

ZTEST(xeddsa_rx, test_good_signature_from_known_node_is_accepted_and_marked)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	seed_key(v);
	make(v, &pkt, &mesh, true);

	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "genuine signature dropped");
	zassert_true(mesh.xeddsa_signed, "verified packet must be marked signed");
}

/* The point of the gate: a bad signature is a DROP, not a downgrade to "unsigned". */
ZTEST(xeddsa_rx, test_bad_signature_from_known_node_is_dropped)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	seed_key(v);
	make(v, &pkt, &mesh, true);
	mesh.decoded.xeddsa_signature.bytes[10] ^= 0x01;

	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "forged signature accepted");
	zassert_false(mesh.xeddsa_signed, "a dropped packet must not be marked signed");
}

/* Same signature, different packet: replaying a signature onto altered content must fail. */
ZTEST(xeddsa_rx, test_signature_replayed_onto_other_content_is_dropped)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	seed_key(v);
	make(v, &pkt, &mesh, true);
	pkt.id = v->id + 1U; /* the id is inside the signed bytes */

	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		      "a signature must not verify over a different packet id");
}

/* --- malformed ----------------------------------------------------------------------- */

ZTEST(xeddsa_rx, test_partial_signature_is_dropped)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	seed_key(v);
	make(v, &pkt, &mesh, true);
	mesh.decoded.xeddsa_signature.size = 32; /* neither absent nor whole */

	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		      "a partial signature must be dropped, not treated as unsigned");
}

/* --- unsigned ------------------------------------------------------------------------ */

ZTEST(xeddsa_rx, test_unsigned_is_accepted_under_compatible_and_dropped_under_strict)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	make(v, &pkt, &mesh, false);
	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "COMPATIBLE must accept");

	set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_BALANCED);
	make(v, &pkt, &mesh, false);
	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		     "BALANCED accepts an unsigned packet from a node not known to sign");

	set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_STRICT);
	make(v, &pkt, &mesh, false);
	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "STRICT must drop");
}

/* PKC decryption already proves who sent it, so STRICT does not also demand a signature --
 * otherwise turning STRICT on would break every direct message. */
ZTEST(xeddsa_rx, test_strict_accepts_unsigned_pkc_packet)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_STRICT);
	make(v, &pkt, &mesh, false);
	pkt.pki_encrypted = true;

	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		     "a PKC-authenticated packet needs no signature");
}

/* --- a stranger ---------------------------------------------------------------------- */

ZTEST(xeddsa_rx, test_signed_by_unknown_node_is_accepted_unless_strict)
{
	const struct mt_xeddsa_vector *v = vec("max_payload"); /* not a NodeInfo: no bootstrap */
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	make(v, &pkt, &mesh, true); /* no key in the NodeDB for this sender */
	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		     "COMPATIBLE keeps a packet we simply cannot check");
	zassert_false(mesh.xeddsa_signed, "unverifiable must never be marked signed");

	set_policy(meshtastic_Config_SecurityConfig_PacketSignaturePolicy_PACKET_SIGNATURE_POLICY_STRICT);
	make(v, &pkt, &mesh, true);
	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		      "STRICT drops what it cannot verify");
}

/* --- first contact ------------------------------------------------------------------- */

/* The bootstrap is safe only because the id commits to the key: a NodeInfo whose payload
 * carries the key, whose id is crc32 of it, and whose signature that key verifies, cannot
 * have come from anyone else. */
ZTEST(xeddsa_rx, test_first_contact_nodeinfo_bootstraps_the_key)
{
	const struct mt_xeddsa_vector *v = vec("nodeinfo_bootstrap");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;
	uint8_t stored[32];

	zassert_equal(meshtastic_nodedb_copy_pubkey(v->from, stored), -ENOENT,
		      "precondition: no key for this node yet");

	make(v, &pkt, &mesh, true);
	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		     "a self-consistent signed NodeInfo must be accepted");
	zassert_true(mesh.xeddsa_signed, "bootstrapped packet is signed");
	/* The gate decides; it does not store. The key reaches the NodeDB from this same
	 * packet once it is delivered, on the ordinary NodeInfo path -- so after the gate
	 * alone there is still nothing stored, and that is the intended split. */
	zassert_equal(meshtastic_nodedb_copy_pubkey(v->from, stored), -ENOENT,
		      "the gate must not write keys itself");
}

/* Same packet under an id that does NOT derive from the key: this is the forgery the
 * commitment exists to stop, so it is dropped rather than merely unverified. */
ZTEST(xeddsa_rx, test_first_contact_nodeinfo_with_mismatched_id_is_dropped)
{
	const struct mt_xeddsa_vector *v = vec("nodeinfo_bootstrap");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;
	uint8_t stored[32];

	make(v, &pkt, &mesh, true);
	pkt.from = v->from ^ 0x00000001U;
	mesh.from = pkt.from;

	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh),
		      "a NodeInfo whose id does not commit to its key must be dropped");
	zassert_equal(meshtastic_nodedb_copy_pubkey(pkt.from, stored), -ENOENT,
		      "nothing may be learned from a dropped packet");
}

/* A NodeInfo with the right id but a tampered signature: the key must not be stored. */
ZTEST(xeddsa_rx, test_first_contact_nodeinfo_with_bad_signature_learns_nothing)
{
	const struct mt_xeddsa_vector *v = vec("nodeinfo_bootstrap");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;
	uint8_t stored[32];

	make(v, &pkt, &mesh, true);
	mesh.decoded.xeddsa_signature.bytes[0] ^= 0x01;

	zassert_false(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "bad bootstrap accepted");
	zassert_equal(meshtastic_nodedb_copy_pubkey(v->from, stored), -ENOENT,
		      "a key must never be learned from an unverified NodeInfo");
}

/* --- the inbound flag ---------------------------------------------------------------- */

/* The sender does not get to declare its own packet verified. The phone receives this
 * struct verbatim, so a believed flag would show as "signed" in the app. */
ZTEST(xeddsa_rx, test_inbound_signed_flag_is_never_believed)
{
	const struct mt_xeddsa_vector *v = vec("position");
	struct meshtastic_packet pkt;
	meshtastic_MeshPacket mesh;

	make(v, &pkt, &mesh, false);
	mesh.xeddsa_signed = true; /* the attacker's claim */

	zassert_true(meshtastic_xeddsa_check_rx_policy(&pkt, &mesh), "unsigned is fine here");
	zassert_false(mesh.xeddsa_signed, "an unverified packet must not stay marked signed");
}

/* ==========================================================================
 * The gate is actually WIRED IN.
 *
 * Everything above calls the gate directly, which cannot tell whether the router consults
 * it -- a gate invoked after delivery would pass every test above. These two inject a
 * decoded packet into the real inbound pipeline (the path an MQTT downlink takes, which
 * joins the RF path inside handle_inbound_impl, where the gate sits) and watch the public
 * receive callback.
 * ========================================================================== */

static uint32_t delivered_from;
static uint32_t delivered_count;

static void recv_cb(uint32_t from, uint32_t to, uint32_t portnum, const uint8_t *payload,
		    size_t len, int16_t rssi, int8_t snr)
{
	ARG_UNUSED(to);
	ARG_UNUSED(portnum);
	ARG_UNUSED(payload);
	ARG_UNUSED(len);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	delivered_from = from;
	delivered_count++;
}

static void inject(const struct mt_xeddsa_vector *v, uint32_t id, bool corrupt)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;

	mesh.from = v->from;
	mesh.to = MESHTASTIC_NODE_BROADCAST;
	mesh.id = id;
	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = (meshtastic_PortNum)v->portnum;
	mesh.decoded.payload.size = (pb_size_t)v->payload_len;
	memcpy(mesh.decoded.payload.bytes, v->payload, v->payload_len);
	mesh.decoded.xeddsa_signature.size = 64;
	memcpy(mesh.decoded.xeddsa_signature.bytes, v->sig, 64);
	if (corrupt) {
		mesh.decoded.xeddsa_signature.bytes[3] ^= 0x01;
	}
	zassert_ok(meshtastic_inject_downlink_mesh_packet(&mesh), "inject failed");
	k_msleep(50);
}

ZTEST(xeddsa_rx, test_router_drops_a_forged_packet_before_delivery)
{
	const struct mt_xeddsa_vector *v = vec("position");

	seed_key(v);
	meshtastic_set_recv_cb(recv_cb);
	delivered_count = 0U;
	delivered_from = 0U;

	/* A DIFFERENT id from the genuine test above: ztest runs cases alphabetically, and the
	 * duplicate filter would otherwise reject this inject before the gate ever saw it (it
	 * did, on the first run). The id is inside the signed bytes, so a fresh one makes the
	 * signature invalid all by itself -- which is the case under test. */
	inject(v, v->id ^ 0x00005A5AU, true);
	zassert_equal(delivered_count, 0U,
		      "a packet with a bad signature reached the application");

	meshtastic_set_recv_cb(NULL);
}

ZTEST(xeddsa_rx, test_router_delivers_a_genuine_signed_packet)
{
	const struct mt_xeddsa_vector *v = vec("position");

	seed_key(v);
	meshtastic_set_recv_cb(recv_cb);
	delivered_count = 0U;
	delivered_from = 0U;

	inject(v, v->id, false);
	zassert_equal(delivered_count, 1U, "a genuine signed packet was not delivered");
	zassert_equal(delivered_from, v->from, "delivered from the wrong node");

	meshtastic_set_recv_cb(NULL);
}
