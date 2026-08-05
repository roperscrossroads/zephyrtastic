/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

/*
 * Meshtastic protobuf, crypto, and wire-packet conversion helpers.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <psa/crypto.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include "meshtastic_channels.h"
#include "meshtastic_clock.h"
#include "meshtastic_core.h"
#include "meshtastic_outbound.h"
#include "meshtastic_packet.h"
#include "meshtastic_router.h"
#include "meshtastic_mqtt.h"
#if defined(CONFIG_MESHTASTIC_PKI)
#include "meshtastic_config_store.h"
#include "meshtastic_pki.h"
#include <zephyr/meshtastic/nodedb.h>
#if defined(CONFIG_MESHTASTIC_NODEINFO)
/* Module-internal (meshtastic_nodeinfo.c): pull a peer's NodeInfo — and thus
 * its public key — throttled per peer and non-blocking. Not in the public
 * header, so declared here. */
int meshtastic_nodeinfo_request(uint32_t peer);
#endif
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(meshtastic, CONFIG_MESHTASTIC_LOG_LEVEL);

BUILD_ASSERT(sizeof(struct meshtastic_wire_header) == MESHTASTIC_HDR_LEN,
	     "Meshtastic wire header must be exactly 16 bytes");

static int ctr_crypt(const uint8_t *key, size_t key_len, const uint8_t nonce[16], const uint8_t *in,
		     uint8_t *out, size_t len)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_cipher_operation_t op = PSA_CIPHER_OPERATION_INIT;
	psa_key_id_t kid = PSA_KEY_ID_NULL;
	psa_status_t status;
	size_t out_len;
	size_t finish_len;
	int ret = 0;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_CTR);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, (uint32_t)(key_len * 8U));
	psa_set_key_lifetime(&attr, PSA_KEY_LIFETIME_VOLATILE);

	status = psa_import_key(&attr, key, key_len, &kid);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_import_key failed (%d)", (int)status);
		return -EIO;
	}

	status = psa_cipher_encrypt_setup(&op, kid, PSA_ALG_CTR);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_cipher_encrypt_setup failed (%d)", (int)status);
		ret = -EIO;
		goto destroy;
	}

	status = psa_cipher_set_iv(&op, nonce, 16U);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_cipher_set_iv failed (%d)", (int)status);
		(void)psa_cipher_abort(&op);
		ret = -EIO;
		goto destroy;
	}

	status = psa_cipher_update(&op, in, len, out, len + 16U, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_cipher_update failed (%d)", (int)status);
		(void)psa_cipher_abort(&op);
		ret = -EIO;
		goto destroy;
	}

	status = psa_cipher_finish(&op, out + out_len, 16U, &finish_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_cipher_finish failed (%d)", (int)status);
		ret = -EIO;
	}

destroy:
	(void)psa_destroy_key(kid);
	return ret;
}

static int encode_packet_data(const meshtastic_MeshPacket *mesh, uint8_t *buf, size_t buf_len,
			      size_t *encoded_len)
{
	meshtastic_Data data;
	pb_ostream_t stream;

	if (mesh == NULL || buf == NULL || encoded_len == NULL) {
		return -EINVAL;
	}

	if (mesh->decoded.payload.size > MESHTASTIC_MAX_PAYLOAD_LEN) {
		return -EINVAL;
	}

	/* C3 Phase 6: mesh->decoded IS the authoritative outgoing Data -- payload, portnum,
	 * want_response, dest/source, request_id/reply_id, and any field the flat struct never
	 * models (Data.emoji, TXT-1) all by construction. Struct originators reach here via
	 * to_mesh_pb (which fills decoded from the struct, after the TX-sanitiser has rewritten
	 * the payload); the phone path carries its own decoded verbatim. So the encoder no
	 * longer merges a separate `base`. */
	data = mesh->decoded;

	/* Every packet we originate carries the bitfield, mirroring the reference
	 * (Router.cpp: set under isFromUs). Bit 0 is our own MQTT consent, taken
	 * from config.lora.config_ok_to_mqtt; bit 1 mirrors want_response. This stays
	 * node-authoritative and is re-stamped here (never trusted from the phone): a
	 * receiver cannot distinguish "declined" from "too old to say" if the field is
	 * absent and must treat absence as declined, so we always emit it.
	 */
	data.has_bitfield = true;
	data.bitfield = 0U;
	if (mt.config_ok_to_mqtt) {
		data.bitfield |= MESHTASTIC_BITFIELD_OK_TO_MQTT_MASK;
	}
	if (data.want_response) {
		data.bitfield |= MESHTASTIC_BITFIELD_WANT_RESPONSE_MASK;
	}

	stream = pb_ostream_from_buffer(buf, buf_len);
	if (!pb_encode(&stream, meshtastic_Data_fields, &data)) {
		const char *err = PB_GET_ERROR(&stream);

		LOG_ERR("Data encode failed: %s", err);
		return -ENOMEM;
	}

	*encoded_len = stream.bytes_written;
	return 0;
}

int meshtastic_encode_data(uint32_t portnum, const uint8_t *payload, size_t payload_len,
			   uint8_t *buf, size_t buf_len, size_t *encoded_len)
{
	meshtastic_MeshPacket mesh = meshtastic_MeshPacket_init_zero;

	if ((payload == NULL && payload_len != 0U) || payload_len > MESHTASTIC_MAX_PAYLOAD_LEN) {
		return -EINVAL;
	}

	mesh.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
	mesh.decoded.portnum = (meshtastic_PortNum)portnum;
	mesh.decoded.payload.size = (pb_size_t)payload_len;
	if (payload_len > 0U) {
		memcpy(mesh.decoded.payload.bytes, payload, payload_len);
	}

	return encode_packet_data(&mesh, buf, buf_len, encoded_len);
}

int meshtastic_packet_to_mesh_pb(const struct meshtastic_packet *packet,
				 meshtastic_MeshPacket *mesh)
{
	meshtastic_Data *decoded;

	if (packet == NULL || mesh == NULL ||
	    (packet->payload == NULL && packet->payload_len != 0U) ||
	    packet->payload_len > MESHTASTIC_MAX_PAYLOAD_LEN) {
		return -EINVAL;
	}

	*mesh = (meshtastic_MeshPacket)meshtastic_MeshPacket_init_zero;
	mesh->from = packet->from;
	mesh->to = packet->to;
	mesh->channel = (packet->channel_index != MESHTASTIC_CHANNEL_INDEX_INVALID)
				? packet->channel_index
				: 0U;
	mesh->id = packet->id;
	mesh->rx_snr = (float)packet->snr;
	mesh->rx_rssi = packet->rssi;
	mesh->hop_limit = packet->hop_limit;
	mesh->hop_start = packet->hop_start;
	mesh->want_ack = packet->want_ack;
	mesh->via_mqtt = packet->via_mqtt;
	mesh->next_hop = packet->next_hop;
	mesh->relay_node = packet->relay_node;
	/* TXT-2: a DM decrypted via PKC must be flagged as such on the phone's FromRadio
	 * view, so the app threads it into the private conversation. The struct already
	 * carries this (set on the PKC decode path); it was simply never copied here. */
	mesh->pki_encrypted = packet->pki_encrypted;
	mesh->transport_mechanism =
		packet->via_mqtt ? meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT
				 : meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
	mesh->which_payload_variant = meshtastic_MeshPacket_decoded_tag;

	decoded = &mesh->decoded;
	*decoded = (meshtastic_Data)meshtastic_Data_init_zero;
	decoded->portnum = (meshtastic_PortNum)packet->portnum;
	decoded->payload.size = (pb_size_t)packet->payload_len;
	if (packet->payload_len > 0U) {
		memcpy(decoded->payload.bytes, packet->payload, packet->payload_len);
	}
	decoded->want_response = packet->want_response;
	decoded->dest = packet->data_dest;
	decoded->source = packet->data_source;
	decoded->request_id = packet->request_id;
	decoded->reply_id = packet->reply_id;
	/* Carry Data.bitfield (OK_TO_MQTT consent + the want_response mirror) AND its
	 * optional presence, so the phone / MQTT uplink view can tell "sender declined"
	 * from "sender never sent the field" (absent == consent unknown). Preserving the
	 * nanopb presence flag is what keeps that distinction — see the MQTT consent gate. */
	if (packet->has_bitfield) {
		decoded->has_bitfield = true;
		decoded->bitfield = packet->bitfield;
	}

	return 0;
}

void meshtastic_mesh_packet_copy(meshtastic_MeshPacket *dst, const meshtastic_MeshPacket *src)
{
	if (dst == NULL || src == NULL) {
		return;
	}

	*dst = *src;
	/*
	 * Match official firmware: copy the full decoded/encrypted union backing
	 * store so payload bytes survive even if which_payload_variant is unset.
	 */
	memcpy(&dst->decoded, &src->decoded, MAX(sizeof(dst->decoded), sizeof(dst->encrypted)));
	dst->which_payload_variant = src->which_payload_variant;
}

static int decrypt_mesh_encrypted_key(uint32_t from, uint32_t id, const uint8_t *enc,
				      size_t enc_len, const struct meshtastic_channel_key *key,
				      meshtastic_Data *data)
{
	uint8_t nonce[16];
	pb_istream_t stream;
	int ret;

	if (enc == NULL || data == NULL || enc_len == 0U || key == NULL) {
		return -EINVAL;
	}

	if (enc_len > MESHTASTIC_PAYLOAD_MAX) {
		return -EINVAL;
	}

	if (key->len == 0U) {
		return -ENOTSUP;
	}

	if (key->len != 16U && key->len != 32U) {
		return -EINVAL;
	}

	k_mutex_lock(&mt_ws.lock, K_FOREVER);

	memset(nonce, 0, sizeof(nonce));
	sys_put_le32(id, nonce);
	sys_put_le32(from, nonce + 8U);

	ret = ctr_crypt(key->bytes, key->len, nonce, enc, mt_ws.rx_dec, enc_len);
	if (ret < 0) {
		k_mutex_unlock(&mt_ws.lock);
		return ret;
	}

	stream = pb_istream_from_buffer(mt_ws.rx_dec, enc_len);
	if (!pb_decode(&stream, meshtastic_Data_fields, data)) {
		k_mutex_unlock(&mt_ws.lock);
		return -EIO;
	}

	k_mutex_unlock(&mt_ws.lock);

	if (data->portnum == meshtastic_PortNum_UNKNOWN_APP) {
		return -EIO;
	}

	return 0;
}

uint8_t meshtastic_packet_wire_hash_for_index(uint8_t channel_index)
{
	return meshtastic_channels_get_hash(channel_index);
}

static int try_decrypt_wire_hash(uint8_t wire_hash, uint32_t from, uint32_t id, const uint8_t *enc,
				 size_t enc_len, meshtastic_Data *data, uint8_t *channel_index_out)
{
	struct meshtastic_channel_key key;

	for (uint8_t ch = 0; ch < MESHTASTIC_MAX_CHANNELS; ch++) {
		const meshtastic_Channel *slot = meshtastic_channels_get(ch);
		int ret;

		if (slot == NULL || slot->role == meshtastic_Channel_Role_DISABLED) {
			continue;
		}

		if (!meshtastic_channels_decrypt_for_hash(ch, wire_hash)) {
			continue;
		}

		ret = meshtastic_channels_get_key(ch, &key);
		if (ret < 0) {
			continue;
		}

		ret = decrypt_mesh_encrypted_key(from, id, enc, enc_len, &key, data);
		if (ret == 0) {
			*channel_index_out = ch;
			return 0;
		}
	}

	return -EBADMSG;
}

#if defined(CONFIG_MESHTASTIC_PKI)
/* PKC (X25519 + AES-CCM) decrypt of a DM addressed to us, then decode the Data
 * protobuf. Mirrors decrypt_mesh_encrypted_key() but for the PKI path; uses the
 * shared rx workspace under its lock. */
static int try_decrypt_pki(uint32_t from, uint32_t id, const uint8_t *enc, size_t enc_len,
			   meshtastic_Data *data)
{
	size_t plain_len;
	pb_istream_t stream;
	int ret;

	k_mutex_lock(&mt_ws.lock, K_FOREVER);
	ret = meshtastic_pki_decrypt(from, id, enc, enc_len, mt_ws.rx_dec, sizeof(mt_ws.rx_dec),
				     &plain_len);
	if (ret < 0) {
		k_mutex_unlock(&mt_ws.lock);
		return ret;
	}

	stream = pb_istream_from_buffer(mt_ws.rx_dec, plain_len);
	if (!pb_decode(&stream, meshtastic_Data_fields, data)) {
		k_mutex_unlock(&mt_ws.lock);
		return -EIO;
	}
	k_mutex_unlock(&mt_ws.lock);

	if (data->portnum == meshtastic_PortNum_UNKNOWN_APP) {
		return -EIO;
	}
	return 0;
}
#endif /* CONFIG_MESHTASTIC_PKI */

int meshtastic_mesh_pb_try_decode(meshtastic_MeshPacket *mesh)
{
	meshtastic_Data data = meshtastic_Data_init_zero;
	size_t enc_len;
	int ret;

	if (mesh == NULL) {
		return -EINVAL;
	}

	if (mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		return 0;
	}

	if (mesh->which_payload_variant != meshtastic_MeshPacket_encrypted_tag) {
		if (mesh->encrypted.size > 0U) {
			mesh->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
		} else {
			return -ENOTSUP;
		}
	}

	enc_len = mesh->encrypted.size;
	if (enc_len == 0U) {
		return -EINVAL;
	}

	{
		uint8_t ch_index = MESHTASTIC_CHANNEL_INDEX_INVALID;
		uint8_t wire_hash = (uint8_t)mesh->channel;

#if defined(CONFIG_MESHTASTIC_PKI)
		/* PKC FIRST (parity with upstream Router::perhapsDecode): a frame carrying
		 * the PKC marker (wire channel-hash 0) addressed to us is X25519+AES-CCM.
		 * CCM authenticates, so a wrong key is cleanly rejected — try it BEFORE the
		 * unauthenticated channel (AES-CTR) path, so a private DM is never first
		 * mis-decrypted against a channel key (the RX mirror of the 0x00 TX marker).
		 * If this is not a PKC-marked frame to us, or PKC decrypt fails, fall through
		 * to channel decryption below. */
		if (wire_hash == 0U && mesh->to == mt.node_id &&
		    mesh->to != MESHTASTIC_NODE_BROADCAST && meshtastic_pki_have_key() &&
		    try_decrypt_pki(mesh->from, mesh->id, mesh->encrypted.bytes, enc_len, &data) == 0) {
			struct meshtastic_nodedb_node node;

			mesh->decoded = data;
			mesh->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
			mesh->channel = 0U;
			mesh->pki_encrypted = true;

			if (meshtastic_nodedb_get(mesh->from, &node) == 0 &&
			    node.public_key_len == MESHTASTIC_PKI_KEY_LEN) {
				memcpy(mesh->public_key.bytes, node.public_key, MESHTASTIC_PKI_KEY_LEN);
				mesh->public_key.size = MESHTASTIC_PKI_KEY_LEN;
			}
			return 0;
		}
#endif

		/* Channel decryption: match the wire hash against configured channels. A
		 * wire byte of 0 that was not PKC (a PKC frame not addressed to us, or a
		 * channel whose hash is 0) is matched literally against the channel hashes,
		 * mirroring upstream's decryptForHash loop. */
		ret = try_decrypt_wire_hash(wire_hash, mesh->from, mesh->id, mesh->encrypted.bytes,
					    enc_len, &data, &ch_index);
		if (ret < 0) {
			return ret;
		}

		mesh->decoded = data;
		mesh->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
		mesh->channel = ch_index;
	}

	return 0;
}

static int copy_data_payload(const meshtastic_Data *data, uint8_t *payload, size_t payload_len,
			     const uint8_t **packet_payload)
{
	if (data->payload.size == 0U) {
		*packet_payload = NULL;
		return 0;
	}

	if (payload == NULL || data->payload.size > payload_len) {
		return -ENOMEM;
	}

	memcpy(payload, data->payload.bytes, data->payload.size);
	*packet_payload = payload;
	return 0;
}

int meshtastic_mesh_pb_to_packet(const meshtastic_MeshPacket *mesh,
				 struct meshtastic_packet *packet, uint8_t *payload,
				 size_t payload_len)
{
	const uint8_t *packet_payload = NULL;
	int ret;

	if (mesh == NULL || packet == NULL) {
		return -EINVAL;
	}

	if (mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		ret = copy_data_payload(&mesh->decoded, payload, payload_len, &packet_payload);
		if (ret < 0) {
			return ret;
		}

		*packet = (struct meshtastic_packet){
			.from = mesh->from,
			.to = mesh->to,
			.id = mesh->id,
			.portnum = (uint32_t)mesh->decoded.portnum,
			.payload = packet_payload,
			.payload_len = mesh->decoded.payload.size,
			.data_dest = mesh->decoded.dest,
			.data_source = mesh->decoded.source,
			.request_id = mesh->decoded.request_id,
			.reply_id = mesh->decoded.reply_id,
			.hop_limit = mesh->hop_limit,
			.hop_start = mesh->hop_start,
			.channel = (mesh->channel < MESHTASTIC_MAX_CHANNELS)
					   ? meshtastic_channels_get_hash((uint8_t)mesh->channel)
					   : ((mesh->channel != 0U) ? (uint8_t)mesh->channel
								    : mt.ch_hash),
			.channel_index = (mesh->channel < MESHTASTIC_MAX_CHANNELS)
						 ? (uint8_t)mesh->channel
						 : MESHTASTIC_CHANNEL_INDEX_INVALID,
			.next_hop = mesh->next_hop,
			.relay_node = mesh->relay_node,
			.want_ack = mesh->want_ack,
			.via_mqtt = mesh->via_mqtt,
			.pki_encrypted = mesh->pki_encrypted,
			.want_response = mesh->decoded.want_response,
			/* C3 Phase 4a: this converter is the receive-side boundary adapter, so it
			 * must reproduce the RX metadata the flat struct models -- link quality and
			 * the OK_TO_MQTT consent bitfield -- rather than zeroing them. All four ride
			 * in the MeshPacket; carry them so the RX pipeline can materialise the struct
			 * on demand (Phase 4c) without dropping SNR/RSSI or the consent signal. */
			.rssi = mesh->rx_rssi,
			.snr = (int8_t)mesh->rx_snr,
			.has_bitfield = mesh->decoded.has_bitfield,
			.bitfield = mesh->decoded.has_bitfield ? (uint8_t)mesh->decoded.bitfield : 0U,
		};
		return 0;
	}

	{
		meshtastic_MeshPacket work;

		meshtastic_mesh_packet_copy(&work, mesh);
		ret = meshtastic_mesh_pb_try_decode(&work);
		if (ret < 0) {
			return ret;
		}

		return meshtastic_mesh_pb_to_packet(&work, packet, payload, payload_len);
	}
}

static void fill_packet_from_header(const struct meshtastic_wire_header *hdr, int16_t rssi,
				    int8_t snr, struct meshtastic_packet *packet)
{
	packet->from = sys_le32_to_cpu(hdr->src);
	packet->to = sys_le32_to_cpu(hdr->dest);
	packet->id = sys_le32_to_cpu(hdr->id);
	packet->hop_limit = hdr->flags & MESHTASTIC_FLAGS_HOP_LIMIT_MASK;
	packet->hop_start =
		(hdr->flags & MESHTASTIC_FLAGS_HOP_START_MASK) >> MESHTASTIC_FLAGS_HOP_START_SHIFT;
	packet->channel = hdr->channel;
	packet->channel_index = MESHTASTIC_CHANNEL_INDEX_INVALID;
	packet->next_hop = hdr->next_hop;
	packet->relay_node = hdr->relay_node;
	packet->want_ack = (hdr->flags & MESHTASTIC_FLAGS_WANT_ACK) != 0U;
	packet->via_mqtt = (hdr->flags & MESHTASTIC_FLAGS_VIA_MQTT) != 0U;
	packet->pki_encrypted = false; /* set true below only if PKC decrypt succeeds */
	packet->rssi = rssi;
	packet->snr = snr;
}

int meshtastic_try_decode_wire_packet(const uint8_t *buf, int len, int16_t rssi, int8_t snr,
				      struct meshtastic_packet *packet, uint8_t *payload,
				      size_t payload_len, bool *decoded,
				      enum meshtastic_decode_fail *fail_reason,
				      meshtastic_MeshPacket *out_mesh)
{
	const struct meshtastic_wire_header *hdr;
	meshtastic_Data data = meshtastic_Data_init_zero;
	const uint8_t *packet_payload = NULL;
	uint8_t channel_index = MESHTASTIC_CHANNEL_INDEX_INVALID;
	int ret;

	if (decoded != NULL) {
		*decoded = false;
	}
	if (fail_reason != NULL) {
		*fail_reason = MESHTASTIC_DECODE_FAIL_NONE;
	}

	if (buf == NULL || packet == NULL || len < (int)MESHTASTIC_HDR_LEN) {
		return -EINVAL;
	}

	hdr = (const struct meshtastic_wire_header *)buf;
	fill_packet_from_header(hdr, rssi, snr, packet);
	packet->payload = NULL;
	packet->payload_len = 0U;

	if (!meshtastic_decode_known_only(packet->from)) {
		return 0;
	}

	{
		uint8_t wire_hash = hdr->channel;
		bool pkc_done = false;
#if defined(CONFIG_MESHTASTIC_PKI)
		int pret = -EBADMSG;

		/* PKC FIRST (parity with upstream Router::perhapsDecode and with the
		 * pb-decode entry meshtastic_mesh_pb_try_decode): a frame carrying the
		 * PKC marker (wire channel-hash 0) addressed to us is X25519+AES-CCM.
		 * CCM authenticates, so a channel frame or a wrong key is cleanly
		 * rejected — try PKC BEFORE the unauthenticated channel (AES-CTR) loop,
		 * so a private DM is never first mis-decrypted against a channel whose
		 * hash is also 0 (the RX mirror of the 0x00 TX marker). If this is not a
		 * PKC-marked frame to us, or PKC decrypt fails, fall through to the
		 * channel loop below. */
		if (wire_hash == 0U && packet->to == mt.node_id &&
		    packet->to != MESHTASTIC_NODE_BROADCAST && meshtastic_pki_have_key()) {
			pret = try_decrypt_pki(packet->from, packet->id, buf + MESHTASTIC_HDR_LEN,
					       (size_t)(len - (int)MESHTASTIC_HDR_LEN), &data);
			if (pret == 0) {
				channel_index = 0U;           /* PKC pseudo-channel */
				packet->pki_encrypted = true; /* authenticated to sender's key */
				pkc_done = true;
			}
		}
#endif

		if (!pkc_done) {
			ret = try_decrypt_wire_hash(wire_hash, packet->from, packet->id,
						    buf + MESHTASTIC_HDR_LEN,
						    (size_t)(len - (int)MESHTASTIC_HDR_LEN), &data,
						    &channel_index);
			if (ret < 0) {
				/* Neither PKC (tried first, above) nor any channel/PSK we hold
				 * decoded this. Report NO_CHANNEL so the router NAKs a want_ack
				 * unicast to us with a reason instead of dropping it silently. */
				if (fail_reason != NULL) {
					*fail_reason = MESHTASTIC_DECODE_FAIL_NO_CHANNEL;
				}
#if defined(CONFIG_MESHTASTIC_PKI)
				/* A PKC-marker DM to us that failed only because we hold no
				 * public key for the sender (-ENOENT): request their NodeInfo
				 * so the next DM decodes, and report the more specific reason.
				 * Matches the reference ReliableRouter (channel==0 && no key).
				 * Throttled + K_NO_WAIT — this runs on the RX thread and the
				 * sender id is attacker-chosen, so it must not amplify or block. */
				if (pret == -ENOENT) {
					if (fail_reason != NULL && wire_hash == 0U) {
						*fail_reason =
							MESHTASTIC_DECODE_FAIL_PKI_UNKNOWN_PUBKEY;
					}
#if defined(CONFIG_MESHTASTIC_NODEINFO)
					(void)meshtastic_nodeinfo_request(packet->from);
#endif
				}
#endif
				return 0;
			}
		}
	}

	/* channel_index is needed by both output paths: the MeshPacket envelope (below) and the
	 * flat struct (else). pki_encrypted was already set on the decrypt path above. */
	packet->channel_index = channel_index;

	if (out_mesh != NULL) {
		/* C3 Phase 4d: the RX path's currency is the MeshPacket. The flat struct is
		 * materialised on demand at the module boundary (Phase 4c), so its decoded fields
		 * are read nowhere before then -- and they were only ever consumed here to build
		 * out_mesh->decoded, which is immediately replaced by the faithful Data below. So
		 * skip filling them: packet_to_mesh_pb takes the envelope from the header-filled
		 * struct + channel_index (+ pki_encrypted), then out_mesh->decoded gets the decrypted
		 * Data verbatim (emoji/TXT-1 and any field the struct never modelled). rx_time lives
		 * on no wire and no struct -- stamp it from the node clock (0 if the clock is unset,
		 * matching upstream). The resulting MeshPacket is byte-identical to the pre-4d
		 * struct->mesh build (test_c3_detector_*; the RX suite runs through this path). */
		(void)meshtastic_packet_to_mesh_pb(packet, out_mesh);
		out_mesh->decoded = data;
		out_mesh->rx_time = meshtastic_clock_now_epoch();
	} else {
		/* Struct-only decode path (meshtastic_decode_wire_packet + tests that want the flat
		 * struct): fill its decoded fields from the decrypted Data. */
		ret = copy_data_payload(&data, payload, payload_len, &packet_payload);
		if (ret < 0) {
			return ret;
		}

		packet->portnum = (uint32_t)data.portnum;
		packet->payload = packet_payload;
		packet->payload_len = data.payload.size;
		packet->data_dest = data.dest;
		packet->data_source = data.source;
		packet->request_id = data.request_id;
		packet->reply_id = data.reply_id;
		packet->want_response = data.want_response;
		/* has_bitfield kept separate from the value: an absent field means the sender never
		 * expressed a preference, NOT the same as "no" -- though both mean do-not-republish
		 * to the MQTT bridge. */
		packet->has_bitfield = data.has_bitfield;
		packet->bitfield = data.has_bitfield ? (uint8_t)data.bitfield : 0U;
	}

	if (decoded != NULL) {
		*decoded = true;
	}

	return 0;
}

int meshtastic_decode_wire_packet(const uint8_t *buf, int len, int16_t rssi, int8_t snr,
				  struct meshtastic_packet *packet, uint8_t *payload,
				  size_t payload_len)
{
	bool decoded = false;
	int ret;

	ret = meshtastic_try_decode_wire_packet(buf, len, rssi, snr, packet, payload, payload_len,
						&decoded, NULL, NULL);
	if (ret < 0) {
		return ret;
	}

	if (!decoded) {
		return -EBADMSG;
	}

	return 0;
}

int meshtastic_build_wire_from_mesh(const meshtastic_MeshPacket *mesh, uint8_t *out,
				    uint32_t *out_len)
{
	uint8_t nonce[16];
	struct meshtastic_wire_header *hdr;
	struct meshtastic_channel_key key;
	uint8_t ch_index;
	uint8_t wire_hash;
	size_t encoded_len;
	size_t payload_len;
	bool pki_done = false;
	int ret;

	if (mesh == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}

	ret = encode_packet_data(mesh, mt_ws.pb_buf, sizeof(mt_ws.pb_buf), &encoded_len);
	if (ret < 0) {
		return ret;
	}

	/* mesh->channel is the fully-resolved send index (the caller ran
	 * meshtastic_channels_resolve_send_index); use it directly for the key + wire hash. */
	ch_index = (uint8_t)mesh->channel;
	ret = meshtastic_channels_get_key(ch_index, &key);
	if (ret < 0) {
		return ret;
	}

	wire_hash = meshtastic_channels_get_hash(ch_index);
	payload_len = encoded_len;

#if defined(CONFIG_MESHTASTIC_PKI)
	/* PKC-vs-channel decision, parity with upstream Router::perhapsEncode. A
	 * directed DM is PKC-encrypted (AES key = SHA256(X25519(our_priv, peer_pub)),
	 * wire channel-hash byte 0x00) when: we hold a private key, the peer is not the
	 * broadcast address, we are not in licensed/Ham mode, and the portnum is not one
	 * upstream keeps on the channel (traceroute/nodeinfo/routing/position are
	 * broadcast-oriented). If PKC is chosen but we lack the peer's public key we
	 * REFUSE — a DM must never be silently downgraded to channel encryption, which
	 * would leak it to every node on the channel (upstream returns
	 * PKI_SEND_FAIL_PUBLIC_KEY). */
	{
		bool is_licensed = false;
		bool is_unmessagable = false;

		meshtastic_config_store_get_owner_flags(&is_licensed, &is_unmessagable);

		if (mesh->to != MESHTASTIC_NODE_BROADCAST && mesh->to != 0U &&
		    meshtastic_pki_have_key() && !is_licensed &&
		    mesh->decoded.portnum != meshtastic_PortNum_TRACEROUTE_APP &&
		    mesh->decoded.portnum != meshtastic_PortNum_NODEINFO_APP &&
		    mesh->decoded.portnum != meshtastic_PortNum_ROUTING_APP &&
		    mesh->decoded.portnum != meshtastic_PortNum_POSITION_APP) {
			size_t pki_len;
			int pret;

			if (MESHTASTIC_HDR_LEN + encoded_len + MESHTASTIC_PKI_OVERHEAD >
			    MESHTASTIC_PKT_MAX) {
				return -EMSGSIZE; /* too large for PKC — refuse (parity: TOO_LARGE) */
			}

			pret = meshtastic_pki_encrypt(mesh->to, mesh->from, mesh->id,
						      mt_ws.pb_buf, encoded_len, mt_ws.enc_buf,
						      sizeof(mt_ws.enc_buf), &pki_len);
			if (pret == 0) {
				wire_hash = 0x00U;
				payload_len = pki_len;
				pki_done = true;
			} else {
				LOG_WRN("No public key for 0x%08x; refusing to send DM as channel "
					"traffic", (unsigned int)mesh->to);
				/* EACCES, not ENOKEY: the latter is a Linux errno extension
				 * absent from the xtensa/picolibc target libc. */
				return -EACCES;
			}
		}
	}
#endif

	if (!pki_done) {
		memset(nonce, 0, sizeof(nonce));
		sys_put_le32(mesh->id, nonce);
		sys_put_le32(mesh->from, nonce + 8U);

		ret = ctr_crypt(key.bytes, key.len, nonce, mt_ws.pb_buf, mt_ws.enc_buf,
				encoded_len);
		if (ret < 0) {
			return ret;
		}
	}

	hdr = (struct meshtastic_wire_header *)out;
	hdr->dest = sys_cpu_to_le32(mesh->to);
	hdr->src = sys_cpu_to_le32(mesh->from);
	hdr->id = sys_cpu_to_le32(mesh->id);
	hdr->flags = ((uint8_t)mesh->hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
		     (((uint8_t)mesh->hop_start & 0x07U) << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	if (mesh->want_ack) {
		hdr->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	}
	if (mesh->via_mqtt) {
		hdr->flags |= MESHTASTIC_FLAGS_VIA_MQTT;
	}
	hdr->channel = wire_hash;
	hdr->next_hop = (uint8_t)mesh->next_hop;
	hdr->relay_node = (uint8_t)mesh->relay_node;

	memcpy(out + MESHTASTIC_HDR_LEN, mt_ws.enc_buf, payload_len);
	*out_len = (uint32_t)(MESHTASTIC_HDR_LEN + payload_len);

	return 0;
}

int meshtastic_build_wire_packet(const struct meshtastic_packet *packet, uint8_t *out,
				 uint32_t *out_len)
{
	meshtastic_MeshPacket mesh;
	int ret;

	if (packet == NULL) {
		return -EINVAL;
	}

	/* Public/struct adapter into the mesh-native builder. to_mesh_pb fills the outgoing
	 * MeshPacket from the struct (its decoded Data carries no emoji -- struct originators
	 * never have one). to_mesh_pb stores the raw channel_index in mesh->channel, so resolve
	 * the actual send index here (honouring a bare wire-hash originator/test) first. */
	ret = meshtastic_packet_to_mesh_pb(packet, &mesh);
	if (ret < 0) {
		return ret;
	}
	mesh.channel = meshtastic_channels_resolve_send_index(packet->to, packet->channel_index,
							      packet->channel);
	return meshtastic_build_wire_from_mesh(&mesh, out, out_len);
}

int meshtastic_send_mesh_pb(const meshtastic_MeshPacket *mesh)
{
	uint32_t encrypted_len;
	struct meshtastic_wire_header *hdr;
	uint8_t hop_limit;
	uint8_t hop_start;
	int ret;

	if (mesh == NULL) {
		return -EINVAL;
	}

	if (mesh->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
		/* C3 Phase 6c: feed the phone's decoded MeshPacket straight into the
		 * mesh-native send engine. Its decoded Data is the authoritative outgoing
		 * payload, so fields the flat struct never models (Data.emoji, a reaction's
		 * flag, anything upstream adds next) reach the wire by construction — no struct
		 * round-trip, no hand-patched `base`. */
		return meshtastic_send_mesh_decoded(mesh, K_FOREVER);
	}

	if (mesh->which_payload_variant != meshtastic_MeshPacket_encrypted_tag) {
		return -ENOTSUP;
	}

	encrypted_len = mesh->encrypted.size;
	if (encrypted_len > MESHTASTIC_PAYLOAD_MAX) {
		return -EINVAL;
	}

	hop_limit = (mesh->hop_limit == 0U) ? mt.hop_limit : mesh->hop_limit;
	hop_start = (mesh->hop_start == 0U) ? hop_limit : mesh->hop_start;

	k_mutex_lock(&mt_ws.lock, K_FOREVER);

	hdr = (struct meshtastic_wire_header *)mt_ws.wire;
	hdr->dest = sys_cpu_to_le32((mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST);
	hdr->src = sys_cpu_to_le32((mesh->from != 0U) ? mesh->from : mt.node_id);
	hdr->id = sys_cpu_to_le32((mesh->id != 0U) ? mesh->id : meshtastic_allocate_packet_id());
	hdr->flags = (hop_limit & MESHTASTIC_FLAGS_HOP_LIMIT_MASK) |
		     ((hop_start & 0x07U) << MESHTASTIC_FLAGS_HOP_START_SHIFT);
	if (mesh->want_ack) {
		hdr->flags |= MESHTASTIC_FLAGS_WANT_ACK;
	}
	if (mesh->via_mqtt) {
		hdr->flags |= MESHTASTIC_FLAGS_VIA_MQTT;
	}
	if (mesh->pki_encrypted) {
		/* PKC-encrypted DM (the phone did the X25519+AES): the wire channel-hash
		 * byte is the 0x00 PKC marker so peers route it to PKI decryption, not
		 * channel decryption. Without this a private DM goes out stamped with the
		 * primary channel hash (e.g. LongFast) — peers try the channel key, it is
		 * neither private nor reliably delivered, and interop breaks. Matches the
		 * decoded-path PKI branch in meshtastic_build_wire_packet. */
		hdr->channel = 0x00U;
	} else if (mesh->channel < MESHTASTIC_MAX_CHANNELS) {
		hdr->channel = meshtastic_channels_get_hash((uint8_t)mesh->channel);
	} else {
		hdr->channel = (mesh->channel != 0U) ? (uint8_t)mesh->channel : mt.ch_hash;
	}
	uint8_t nh = mesh->next_hop;
	uint8_t rn = mesh->relay_node;

	/* Next-hop routing (increment 2): the app/PKC path arrives already
	 * encrypted and bypasses the mesh-native send engine, so stamp our relayer byte +
	 * learned next hop here. hdr->dest/src are already normalized above. */
	meshtastic_router_stamp_originated(sys_le32_to_cpu(hdr->dest), sys_le32_to_cpu(hdr->src),
					   &nh, &rn);
	hdr->next_hop = nh;
	hdr->relay_node = rn;
	memcpy(mt_ws.wire + MESHTASTIC_HDR_LEN, mesh->encrypted.bytes, encrypted_len);

	{
		uint8_t wire[MESHTASTIC_PKT_MAX];
		const uint32_t wire_len = MESHTASTIC_HDR_LEN + encrypted_len;

		memcpy(wire, mt_ws.wire, wire_len);
		k_mutex_unlock(&mt_ws.lock);

		ret = meshtastic_radio_send_wire_wait(wire, wire_len, K_FOREVER);

		if (ret == 0) {
#if defined(CONFIG_MESHTASTIC_MQTT)
			struct meshtastic_packet tx_packet = {
				.from = (mesh->from != 0U) ? mesh->from : mt.node_id,
				.to = (mesh->to != 0U) ? mesh->to : MESHTASTIC_NODE_BROADCAST,
				.id = sys_le32_to_cpu(hdr->id),
				.hop_limit = hop_limit,
				.hop_start = hop_start,
				.channel = hdr->channel,
				.want_ack = mesh->want_ack,
				.via_mqtt = mesh->via_mqtt,
				.next_hop = nh,
				.relay_node = rn,
			};

			/* Pre-encrypted PKC send: no decoded MeshPacket in hand — the mqtt
			 * uplink falls back to the wire (encrypted) / struct path. */
			meshtastic_mqtt_on_tx(&tx_packet, wire, wire_len, NULL);
#endif
		}
	}

	return ret;
}
