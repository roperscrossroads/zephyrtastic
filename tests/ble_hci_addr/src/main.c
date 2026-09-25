/* SPDX-License-Identifier: GPL-3.0
 *
 * The ESP32-S3 peer-scan failure (agents-f5f2, agents-t2hb.12), reproduced on
 * native_sim with the real Zephyr host and a fake controller.
 *
 * Mechanism, in the pinned host (subsys/bluetooth/host/id.c,
 * bt_id_set_scan_own_addr, privacy off, SCAN_WITH_IDENTITY off): if a legacy
 * advertiser is running WITHOUT BT_LE_ADV_OPT_USE_IDENTITY, the scanner
 * assumes it shares the advertiser's random address and asks for own-address
 * type RANDOM — without writing one. A connectable advert does not use a
 * random address when the identity is PUBLIC (ESP32: the eFuse MAC), so no
 * random address was ever set, and the controller must refuse LE Set Scan
 * Enable with 0x12 Invalid Parameters. A static random identity (nRF52) was
 * already written as the random address, which is why the XIAOs never saw it.
 */

#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fake_hci.h"

static const bt_addr_t esp32_mac = {{0x74, 0x4b, 0xe1, 0x04, 0xa7, 0xac}};

/* The firmware's peer scan (meshtastic_ble_peer.c peer_scan_param). */
static const struct bt_le_scan_param peer_scan = {
	.type = BT_LE_SCAN_TYPE_PASSIVE,
	.options = BT_LE_SCAN_OPT_NONE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static bool public_identity(void)
{
	return !IS_ENABLED(CONFIG_TEST_RANDOM_IDENTITY);
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		    struct net_buf_simple *buf)
{
}

static void *setup(void)
{
	fake_hci_set_public_addr(public_identity() ? &esp32_mac : NULL);
	zassert_ok(bt_enable(NULL));
	return NULL;
}

static void after(void *f)
{
	(void)bt_le_scan_stop();
	(void)bt_le_adv_stop();
}

ZTEST_SUITE(ble_hci_addr, NULL, setup, NULL, after, NULL);

/*
 * ORDER MATTERS, and the tests are numbered because of it (ztest runs them
 * in name order). The failure needs a controller that has never been given a
 * random address. Once any scan has written an NRPA, that address stays in
 * the controller, and a later scan-while-advertising "works" by reusing it.
 * That is the bench's intermittency exactly: a node whose saved peer scan ran
 * at boot, before the phone advertiser, could link out; the same node asked
 * to scan later, from the shell, could not.
 */

ZTEST(ble_hci_addr, test_1_identity_type_matches_controller)
{
	bt_addr_le_t id[CONFIG_BT_ID_MAX];
	size_t n = ARRAY_SIZE(id);

	bt_id_get(id, &n);
	zassert_equal(n, 1U);
	zassert_equal(id[0].type, public_identity() ? BT_ADDR_LE_PUBLIC : BT_ADDR_LE_RANDOM);
}

/* The firmware's order: phone advertiser up (BT_LE_ADV_CONN_FAST_1), then
 * the peer scan, on a controller that has never scanned. Fails exactly where
 * the bench failed. */
ZTEST(ble_hci_addr, test_2_scan_while_connectable_advertising)
{
	struct fake_hci_state s;
	bool bug = public_identity() && !IS_ENABLED(CONFIG_BT_SCAN_WITH_IDENTITY);
	int err;

	fake_hci_get_state(&s);
	if (public_identity()) {
		zassert_equal(s.random_addr_writes, 0U, "precondition: fresh controller");
	}

	zassert_ok(bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0));
	err = bt_le_scan_start(&peer_scan, scan_cb);
	fake_hci_get_state(&s);

	if (bug) {
		zassert_equal(err, -EINVAL, "err %d", err);
		zassert_equal(s.scan_enable_last_status, BT_HCI_ERR_INVALID_PARAM);
		zassert_equal(s.scan_own_addr_type, BT_HCI_OWN_ADDR_RANDOM);
		zassert_false(s.random_addr_set, "no random address was ever written");
	} else {
		zassert_ok(err, "err %d", err);
		zassert_true(s.scan_enabled && s.adv_enabled, "both at once");
	}
	zassert_true(s.adv_enabled, "the advertiser is unaffected either way");
}

/* The in-code fix: advertise with USE_IDENTITY. On air nothing changes —
 * privacy is off, so a connectable advert already uses the identity — but
 * the host now knows it, and the scanner shares the identity legally. */
ZTEST(ble_hci_addr, test_3_scan_while_identity_advertising)
{
	const struct bt_le_adv_param *param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_FAST_INT_MIN_1,
		BT_GAP_ADV_FAST_INT_MAX_1, NULL);
	struct fake_hci_state s;

	zassert_ok(bt_le_adv_start(param, ad, ARRAY_SIZE(ad), NULL, 0));
	zassert_ok(bt_le_scan_start(&peer_scan, scan_cb));
	fake_hci_get_state(&s);
	zassert_true(s.scan_enabled && s.adv_enabled);
	zassert_equal(s.adv_own_addr_type,
		      public_identity() ? BT_HCI_OWN_ADDR_PUBLIC : BT_HCI_OWN_ADDR_RANDOM);
}

/* The reverse order: scan first, then the phone advertiser — what a restored
 * peer scan does at boot. With a PUBLIC identity it works: the scanner writes
 * an NRPA, the advertiser uses the public address and writes nothing. With a
 * static RANDOM identity (XIAO) the advertiser must put the identity back
 * into the one shared random-address register while the scan is running,
 * which the spec forbids (7.8.4, zephyr#65744's shape) — so on a spec-strict
 * controller the phone advert fails to start. Scanning with the identity
 * removes the conflict for both. */
ZTEST(ble_hci_addr, test_4_advertise_while_scanning)
{
	struct fake_hci_state s;
	bool clash = !public_identity() && !IS_ENABLED(CONFIG_BT_SCAN_WITH_IDENTITY);
	int err;

	zassert_ok(bt_le_scan_start(&peer_scan, scan_cb));
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	fake_hci_get_state(&s);
	if (clash) {
		zassert_equal(err, -EACCES, "err %d", err);
		zassert_false(s.adv_enabled);
	} else {
		zassert_ok(err, "err %d", err);
		zassert_true(s.scan_enabled && s.adv_enabled);
	}
}

/* ...and once that NRPA exists, the failing order of test 2 "works" on a
 * public identity too — the intermittency, pinned. */
ZTEST(ble_hci_addr, test_5_scan_while_advertising_after_an_earlier_scan)
{
	struct fake_hci_state s;

	fake_hci_get_state(&s);
	zassert_true(s.random_addr_set || IS_ENABLED(CONFIG_BT_SCAN_WITH_IDENTITY));
	zassert_ok(bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0));
	zassert_ok(bt_le_scan_start(&peer_scan, scan_cb));
}
