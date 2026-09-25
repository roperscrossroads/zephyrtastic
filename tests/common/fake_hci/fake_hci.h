/* SPDX-License-Identifier: GPL-3.0
 *
 * A fake Bluetooth LE controller for native_sim tests.
 *
 * native_sim has no controller, so until this existed no test in the tree
 * enabled CONFIG_BT and every BLE bug was found on the bench. This driver sits
 * where a real controller would (chosen zephyr,bt-hci) and answers the real
 * Zephyr host's HCI commands from a small state model — enough for the host
 * to come up as peripheral + central + observer, advertise, scan, initiate,
 * connect and disconnect.
 *
 * What it models, deliberately, is the controller state the host's own
 * address logic depends on, with the Core spec's rules enforced:
 *
 *   - LE Set Scan/Adv Enable with own-address-type RANDOM and no random
 *     address ever written -> 0x12 Invalid HCI Command Parameters
 *     (Vol 4 Part E 7.8.9 / 7.8.11). This is the ESP32-S3 peer-scan failure
 *     (agents-f5f2, agents-t2hb.12).
 *   - LE Set Random Address while legacy advertising, scanning or
 *     initiating -> 0x0C Command Disallowed (7.8.4).
 *   - Changing adv/scan parameters while enabled -> 0x0C.
 *
 * The controller's public address is configurable: a non-zero one gives the
 * host a PUBLIC identity (the ESP32 shape); zero leaves the host to make a
 * static random identity (the nRF52 / XIAO shape).
 *
 * ATT: every ATT request on a link is answered with Error Response
 * "Attribute Not Found" (MTU requests get 23) unless the link is set silent,
 * so a GATT client's discovery fails fast or hangs, as a test chooses. There
 * is no GATT server here — a happy-path peer link is out of scope.
 */

#ifndef FAKE_HCI_H_
#define FAKE_HCI_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/addr.h>

#define FAKE_HCI_MAX_CONN 4

struct fake_hci_state {
	/* Controller address state. */
	bool random_addr_set;
	bt_addr_t random_addr;
	uint32_t random_addr_writes;

	/* Legacy advertising. */
	bool adv_enabled;
	uint8_t adv_type;
	uint8_t adv_own_addr_type;
	uint32_t adv_enable_ok;
	uint32_t adv_enable_rejected;
	uint8_t adv_enable_last_status;

	/* Legacy scanning. */
	bool scan_enabled;
	uint8_t scan_type;
	uint8_t scan_own_addr_type;
	uint32_t scan_enable_ok;
	uint32_t scan_enable_rejected;
	uint8_t scan_enable_last_status;

	/* Initiating. */
	bool initiating;
	bt_addr_le_t init_peer;
	uint32_t create_conn_count;
	uint32_t create_conn_cancel_count;

	/* Links. */
	uint32_t conns_up;     /* connections ever established */
	uint32_t conns_active; /* connections up now */
	uint32_t disconnect_cmds;
	uint32_t att_requests;
};

/* Before bt_enable(): the public address the controller reports (NULL or
 * all-zero = none, the host then makes a static random identity). */
void fake_hci_set_public_addr(const bt_addr_t *addr);

/* A copy of the controller's state, taken under its lock. */
void fake_hci_get_state(struct fake_hci_state *out);

/* Fail the next @p count commands with @p opcode, answering @p status.
 * @p count 0 clears any pending failure for @p opcode. */
void fake_hci_fail(uint16_t opcode, uint8_t status, uint32_t count);

/* LE Create Connection: complete it automatically after @p delay_ms with
 * success (true), or leave it pending for the test (false, the default). */
void fake_hci_set_auto_connect(bool on, uint32_t delay_ms);

/* Complete the pending LE Create Connection. @p status 0 = success; returns
 * the new handle, or -1 if nothing was initiating. 0x3E (failed to be
 * established) is delivered as a real legacy controller does it: a
 * connection that completes and at once drops with reason 0x3E. */
int fake_hci_complete_create(uint8_t status);

/* An incoming connection from @p peer (we are peripheral); stops legacy
 * advertising, as a real controller does. Returns the handle or -1. */
int fake_hci_incoming_conn(const bt_addr_le_t *peer);

/* The remote end drops link @p handle. */
void fake_hci_remote_disconnect(uint16_t handle, uint8_t reason);

/* The remote end drops every link (test teardown). */
void fake_hci_remote_disconnect_all(uint8_t reason);

/* Deliver one legacy advertising report, if scanning. Returns false if the
 * controller is not scanning (a real one would not have heard it). */
bool fake_hci_adv_report(const bt_addr_le_t *addr, uint8_t evt_type, const uint8_t *ad,
			 uint8_t ad_len, int8_t rssi);

/* Stop answering ATT requests on every link (a peer that never responds). */
void fake_hci_set_att_silent(bool silent);

#endif /* FAKE_HCI_H_ */
