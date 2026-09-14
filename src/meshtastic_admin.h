/* SPDX-License-Identifier: GPL-3.0
 *
 * AdminMessage handling (portnum ADMIN_APP) — lets a client WRITE config, not
 * just read it. Two entry points: the directly-connected app (local, trusted)
 * and an authorized remote node over the mesh (PKC admin_key / session passkey).
 */
#ifndef MESHTASTIC_ADMIN_H_
#define MESHTASTIC_ADMIN_H_

#include <stdbool.h>

#include "meshtastic/mesh.pb.h"
#include "meshtastic/module_config.pb.h"

struct meshtastic_packet;

/**
 * Handle a locally-submitted ADMIN_APP packet (to == this node), from the
 * directly-connected app via ToRadio — trusted transport, no passkey required
 * (but refused outright when the node is SecurityConfig.is_managed).
 *
 * @param pkt Decoded MeshPacket from ToRadio (which_payload_variant decoded).
 * @return true if consumed as admin (caller must NOT transmit it on the mesh).
 */
bool meshtastic_admin_handle_local(const meshtastic_MeshPacket *pkt);

/**
 * Handle an ADMIN_APP packet received over the mesh (to == this node,
 * from != self). Authorizes the sender (PKC key in SecurityConfig.admin_key, or
 * the legacy admin channel) and enforces the session passkey on mutating ops,
 * replying — including any ACK/error — back over the mesh. Always consumes the
 * packet (the caller must not deliver it to the phone as a normal RX packet).
 *
 * @param pkt Decoded internal packet carrying the AdminMessage bytes.
 * @return true when the admin path answered the requester itself (a NAK, a
 *         response, or its own ACK). The router must then NOT send its generic
 *         want_ack ACK for the same packet: one request, one answer, as the
 *         reference's ReliableRouter (agents-dnr4.32). false when nothing was
 *         sent (a dropped or undecodable payload, a response to our own client),
 *         so the router's ACK stands exactly as for any other packet.
 */
bool meshtastic_admin_handle_remote(const struct meshtastic_packet *pkt,
				    const meshtastic_MeshPacket *mesh);

/**
 * Note an admin getter a client is sending through this node to a remote node,
 * so the remote's response is recognized and delivered to the client instead of
 * being refused as an unauthorized request (agents-dnr4.33; reference
 * AdminModule::noteOutgoingAdminRequest). Call on the client-to-mesh path before
 * the send. Assigns @p pkt->id when it is 0, because the response must echo it.
 */
void meshtastic_admin_note_outgoing_request(meshtastic_MeshPacket *pkt);

/**
 * True when @p pkt (an ADMIN_APP unicast to us) is a response to a request
 * noted by meshtastic_admin_note_outgoing_request(): same sender, the request's
 * packet id echoed, within the window, over PKC from the pinned key when the
 * request went PKC. Consumes the match. The caller then delivers the packet to
 * the client like any other received packet.
 */
bool meshtastic_admin_take_solicited_response(const struct meshtastic_packet *pkt,
					      const meshtastic_MeshPacket *mesh);

/** Reset admin edit-transaction state (call on phone disconnect). */
void meshtastic_admin_reset(void);

/**
 * True if @p node_id's known public key is in SecurityConfig.admin_key[] — this
 * node's "is that one of my masters?" test, over the hot→warm NodeDB key lookup
 * so an evicted admin is not locked out (A-1).
 *
 * Two callers ask it about different subjects. Remote admin asks it about a
 * packet's SENDER. Cluster sync asks it about the AUTHOR recorded in a document
 * entry's stamp — the D4 base-authorship gate (docs/CLUSTER-SYNC-M4.md §4) —
 * which is a claim no receiver can cryptographically refute today (there is no
 * signing primitive in Meshtastic PKC; §4 states that plainly). The gate is
 * still worth having: it stops an ordinary fleet member's honest mistake from
 * rewriting fleet base config, and the channel PSK remains the real boundary.
 */
bool meshtastic_admin_node_is_trusted(uint32_t node_id);

/**
 * True when this node is administratively managed (SecurityConfig.is_managed):
 * configuration may be changed only by an authorized remote admin, so the local
 * path is refused.
 *
 * Exported so other local write surfaces can honour the same policy. The gate is
 * only meaningful if every path that writes config consults it — a surface that
 * skips it silently reopens what a managed node is asserting is closed.
 */
bool meshtastic_admin_is_managed(void);

/**
 * @brief Whether a deferred config-change reboot is currently scheduled.
 *
 * A LoRa config write (F-1), a module-config write, and other sections that only
 * take effect on restart schedule a reboot via the set_config path so the change
 * is actually applied. Exposed for introspection and tests.
 */
bool meshtastic_admin_reboot_scheduled(void);

#if defined(CONFIG_MESHTASTIC_DFU_TRIGGER)
/** @brief A deferred enter-DFU (admin enter_dfu_mode_request) is pending. */
bool meshtastic_admin_dfu_scheduled(void);
#endif

/** Cancel a scheduled config-change reboot (used by tests to avoid rebooting). */
void meshtastic_admin_cancel_reboot(void);

/**
 * @brief Strip secrets from a module section before it leaves over the mesh.
 *
 * Reference (AdminModule.cpp, secretReserved): a get_module_config answered to a
 * REMOTE requester carries "sekrit" in place of ModuleConfig.mqtt.password. The
 * locally connected app (PhoneAPI) still sees the real value — it is the one that
 * displays and edits it. Applied by the remote get path; public so the sim suite
 * can prove the substitution without decrypting a PKC reply.
 */
void meshtastic_admin_redact_module_config_for_mesh(meshtastic_ModuleConfig *module);

/**
 * @brief Validate and normalise a set_module_config section before it is stored.
 *
 * mqtt: a password equal to "sekrit" means "keep the stored one" (the reference's
 * writeSecret), so an app that round-trips a redacted get does not wipe the
 * credential; and a section asking for TLS on a build with no TLS transport is
 * refused rather than silently downgraded (reference MQTT::isValidConfig).
 * Other sections pass through unchanged.
 *
 * @return 0 to store; -ENOTSUP / -EINVAL to refuse (the caller NAKs BAD_REQUEST).
 */
int meshtastic_admin_prepare_module_config_write(meshtastic_ModuleConfig *module);

/**
 * @brief Strip secrets from a Config section before it leaves over the mesh.
 *
 * Reference (AdminModule::handleGetConfig): the device identity private key is
 * backup material for the LOCAL owner only. A get_config(SECURITY) answered to a
 * remote requester, even an authorized admin, carries no private_key; public_key
 * and admin_key are public and stay. Applied by the remote get path; public so the
 * sim suite can prove it without decrypting a PKC reply.
 */
void meshtastic_admin_redact_config_for_mesh(meshtastic_Config *config);

/**
 * @brief Reconcile an incoming SecurityConfig write with the stored one.
 *
 * The two identity rules of the reference's handleSetConfig(security):
 *  - a write that OMITS the private key keeps the current keypair. It is a partial
 *    or remote client (a remote admin is never given the private key, so every
 *    remote security write omits it), not an identity reset -- that goes through
 *    factory_reset_device. Without this the empty key is stored, and the next boot
 *    generates a new identity, orphaning the node from every peer that pinned it.
 *  - a BARE keypair rotation (a new private key, every other field at its default)
 *    keeps the other security fields, so rotating the key does not also drop the
 *    admin keys and lock the owner out of remote admin.
 * @p incoming is rewritten in place; @p current is the stored section.
 */
void meshtastic_admin_prepare_security_write(meshtastic_Config_SecurityConfig *incoming,
					     const meshtastic_Config_SecurityConfig *current);

#endif /* MESHTASTIC_ADMIN_H_ */
