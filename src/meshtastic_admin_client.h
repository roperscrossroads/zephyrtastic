/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_ADMIN_CLIENT_H_
#define MESHTASTIC_ADMIN_CLIENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Remote-admin CLIENT (agents-xhli.3): originate AdminMessage requests to a
 * peer node, the role the phone app/CLI plays against stock firmware. Zero new
 * protocol — requests are ordinary PKC-encrypted ADMIN_APP unicasts through
 * meshtastic_send_packet(), so they ride whatever bearer the TX path picks
 * (LoRa, or the BLE peer link when one reaches the target).
 *
 * Flow mirrors the reference CLI: any getter's response carries the target's
 * rotating session passkey (150 s reissue / 300 s validity); the client caches
 * it and must echo it in a mutating op. So: get first, then set.
 *
 * One request pending at a time (bench-shaped). Responses are matched by
 * (PKC-authenticated sender == target) AND (Data.request_id == our packet id)
 * — tighter than the server's admin_key gate, and correct for a client: we
 * accept admin traffic only from the node we just asked, about the thing we
 * asked.
 */

#if defined(CONFIG_MESHTASTIC_ADMIN_CLIENT)

/* Send get_owner_request / get_config_request (config_type is a
 * meshtastic_AdminMessage_ConfigType value) to @p node, want_response set.
 * Returns 0 once queued, else the send error (-EACCES: no public key for the
 * target — it must be in the NodeDB first). */
int meshtastic_admin_client_get_owner(uint32_t node);
int meshtastic_admin_client_get_config(uint32_t node, uint32_t config_type);

/* Mutating op: set the target's owner names. Requires a passkey cached from a
 * recent getter response for THIS node; -EACCES with none/stale (re-run a
 * get). Empty long/short skips that field (target keeps its current value). */
int meshtastic_admin_client_set_owner(uint32_t node, const char *long_name,
				      const char *short_name);

/* True while a usable session passkey for @p node is cached. */
bool meshtastic_admin_client_have_passkey(uint32_t node);

/*
 * RX hook, called by meshtastic_admin_handle_remote() BEFORE the admin_key
 * authorization gate: a response to our pending request must be accepted from
 * the target we queried (authenticated by the PKC wire crypto), which is not
 * required to hold OUR key in ITS admin list. Returns true when the packet was
 * consumed as such a response; false hands it to the normal server path.
 */
bool meshtastic_admin_client_on_admin(uint32_t from, uint32_t request_id, bool pki_encrypted,
				      const uint8_t *payload, size_t payload_len);

#else /* !CONFIG_MESHTASTIC_ADMIN_CLIENT */

static inline bool meshtastic_admin_client_on_admin(uint32_t from, uint32_t request_id,
						    bool pki_encrypted, const uint8_t *payload,
						    size_t payload_len)
{
	(void)from;
	(void)request_id;
	(void)pki_encrypted;
	(void)payload;
	(void)payload_len;
	return false;
}

#endif /* CONFIG_MESHTASTIC_ADMIN_CLIENT */

#endif /* MESHTASTIC_ADMIN_CLIENT_H_ */
