/* SPDX-License-Identifier: GPL-3.0
 *
 * The firmware's own BLE code — meshtastic_ble.c (the phone advertiser and
 * slot registry) and meshtastic_ble_peer.c (the peer scanner and outbound
 * link) — booted by meshtastic_init() on the real Zephyr host, against the
 * fake controller in tests/common/fake_hci.
 *
 * Until this suite, none of it had run anywhere but the bench. Each test is a
 * behaviour the bench showed broken (or could not show at all):
 *
 *   - the peer scan refused while the phone advert runs (agents-f5f2,
 *     agents-t2hb.12) — on a public identity, the ESP32 shape;
 *   - a peer scan restored at boot racing the phone advertiser, on a random
 *     identity, the XIAO shape (a second route to agents-selv's adv_starts=0);
 *   - failed scan/advertise starts never retried;
 *   - a failed connect retried with no delay (the 174-in-50-s hot loop);
 *   - a link that connected but never finished discovery, held forever;
 *   - a peer already linked inbound, re-dialled on every advert.
 *
 * Variants (testcase.yaml): public/random identity x booted armed or not.
 * Tests are numbered because ztest runs them in name order and test 1 is the
 * boot itself.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "fake_hci.h"
#include "meshtastic_ble_peer.h"
#include "meshtastic_ble_peer_codec.h"

#define TEST_NODE_ID 0x7E57B1E0U
#define PEER_NODE    0x0BADF00DU

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static const bt_addr_t esp32_mac = {{0x74, 0x4b, 0xe1, 0x04, 0xa7, 0xac}};
static const bt_addr_le_t peer_addr = {
	.type = BT_ADDR_LE_RANDOM, .a = {{0x11, 0x22, 0x33, 0x44, 0x55, 0xC6}}};
static const bt_addr_le_t phone_addr = {
	.type = BT_ADDR_LE_RANDOM, .a = {{0x99, 0x88, 0x77, 0x66, 0x55, 0xC4}}};

#define CONNECT_OK()                                                                               \
	do {                                                                                       \
		int _e = meshtastic_ble_peer_connect(PEER_NODE);                                   \
		zassert_ok(_e, "blepeer connect: %d", _e);                                         \
	} while (0)

#define UNTIL(cond, ms)                                                                         \
	({                                                                                         \
		int64_t _end = k_uptime_get() + (ms);                                              \
		bool _ok;                                                                          \
		while (!(_ok = (cond)) && k_uptime_get() < _end) {                                  \
			k_msleep(10);                                                              \
		}                                                                                  \
		_ok;                                                                               \
	})

static struct fake_hci_state hci(void)
{
	struct fake_hci_state s;

	fake_hci_get_state(&s);
	return s;
}

static struct meshtastic_ble_peer_stats peer_stats(void)
{
	struct meshtastic_ble_peer_stats s;

	meshtastic_ble_peer_stats_get(&s);
	return s;
}

/* The peer's advert, byte for byte what meshtastic_ble.c start_advertising()
 * sends in a peer build: flags, the Meshtastic service UUID, the node blob. */
static bool peer_advert(const bt_addr_le_t *from, uint32_t node)
{
	uint8_t ad[3 + 18 + 9];
	uint8_t *p = ad;

	*p++ = 2U;
	*p++ = BT_DATA_FLAGS;
	*p++ = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
	*p++ = 17U;
	*p++ = BT_DATA_UUID128_ALL;
	memcpy(p, meshtastic_ble_service_uuid128(), 16U);
	p += 16;
	*p++ = 1U + MESHTASTIC_BLE_PEER_ADV_LEN;
	*p++ = BT_DATA_MANUFACTURER_DATA;
	meshtastic_ble_peer_adv_encode(p, node);
	return fake_hci_adv_report(from, BT_HCI_ADV_IND, ad, sizeof(ad), -40);
}

static bool outbound_link(void)
{
	struct meshtastic_ble_peer_link l;

	meshtastic_ble_peer_link_get(&l);
	return l.connected;
}

static bool public_identity(void)
{
	return !IS_ENABLED(CONFIG_TEST_RANDOM_IDENTITY);
}

static void *setup(void)
{
	static struct meshtastic_config cfg = {
		.node_id = TEST_NODE_ID,
		.psk = meshtastic_default_psk,
		.psk_len = sizeof(meshtastic_default_psk),
		.channel_name = MESHTASTIC_CHANNEL_LONGFAST,
		.frequency = MESHTASTIC_FREQ_EU,
	};

	cfg.lora_dev = lora_dev;
	fake_hci_set_public_addr(public_identity() ? &esp32_mac : NULL);

	if (IS_ENABLED(CONFIG_TEST_BOOT_ARMED)) {
		/* What `blepeer connect <PEER>` persisted before the reboot:
		 * blepeer/central = {scan_on, target, last}. */
		const uint8_t rec[9] = {1U,
					PEER_NODE & 0xFFU,
					(PEER_NODE >> 8) & 0xFFU,
					(PEER_NODE >> 16) & 0xFFU,
					PEER_NODE >> 24,
					0U,
					0U,
					0U,
					0U};

		zassert_ok(settings_subsys_init());
		zassert_ok(settings_save_one("blepeer/central", rec, sizeof(rec)));
	}

	zassert_true(device_is_ready(lora_dev));
	zassert_ok(meshtastic_init(&cfg));
	return NULL;
}

/* Back to idle: scan disarmed, no links, the phone advert up again. */
static void after(void *f)
{
	ARG_UNUSED(f);

	fake_hci_set_att_silent(false);
	fake_hci_fail(BT_HCI_OP_LE_SET_SCAN_ENABLE, 0U, 0U);
	fake_hci_fail(BT_HCI_OP_LE_SET_ADV_ENABLE, 0U, 0U);
	(void)meshtastic_ble_peer_scan_set(false);
	(void)meshtastic_ble_peer_disconnect(); /* also cancels a pending create */
	fake_hci_remote_disconnect_all(BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	zassert_true(UNTIL(hci().conns_active == 0U && !hci().initiating && !hci().scan_enabled &&
				   hci().adv_enabled && !outbound_link(),
			   5000),
		     "teardown did not return to idle");
}

ZTEST_SUITE(ble_sim, NULL, setup, NULL, after, NULL);

/* Boot: the phone advertiser comes up — and with a restored peer-scan arm,
 * the scanner too. On a random identity, a scan restored before the advert
 * made the advert's start collide on the shared random address. */
ZTEST(ble_sim, test_01_boot_advertises_and_restores_scan)
{
	zassert_true(UNTIL(hci().adv_enabled, 2000), "phone advert never started");
	zassert_true(meshtastic_ble_adv_starts() >= 1U, "adv_starts %u",
		     meshtastic_ble_adv_starts());
	if (IS_ENABLED(CONFIG_TEST_BOOT_ARMED)) {
		zassert_true(meshtastic_ble_peer_scan_armed());
		zassert_equal(meshtastic_ble_peer_scan_target(), PEER_NODE);
		zassert_true(UNTIL(hci().scan_enabled, 3000), "restored scan never ran");
		zassert_true(hci().adv_enabled, "advert lost to the scan");
	}
}

/* `blepeer connect` while the phone advert is up: the Heltec failure. */
ZTEST(ble_sim, test_02_scan_starts_while_advertising)
{
	zassert_true(hci().adv_enabled);
	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000), "scan refused (last status 0x%02x)",
		     hci().scan_enable_last_status);
	zassert_true(hci().adv_enabled, "the phone advert must stay up");
}

/* A matching advert makes the central stop scanning and initiate. */
ZTEST(ble_sim, test_03_advert_starts_a_connect)
{
	uint32_t before = hci().create_conn_count;

	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	zassert_true(peer_advert(&peer_addr, PEER_NODE));
	zassert_true(UNTIL(hci().initiating, 2000), "no LE Create Connection");
	zassert_equal(hci().create_conn_count, before + 1U);
	struct fake_hci_state s = hci();

	zassert_true(bt_addr_le_eq(&s.init_peer, &peer_addr));
}

/* The link comes up, but the remote has no peer service: discovery fails.
 * The link must be released and the hunt must resume — not held, not ready,
 * blocking every rescan. */
ZTEST(ble_sim, test_04_failed_discovery_releases_the_link)
{
	uint32_t disc_before = hci().disconnect_cmds;

	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	zassert_true(peer_advert(&peer_addr, PEER_NODE));
	zassert_true(UNTIL(hci().initiating, 2000));
	zassert_true(fake_hci_complete_create(0U) > 0);

	zassert_true(UNTIL(hci().att_requests > 0U, 2000), "no discovery attempted");
	zassert_true(UNTIL(hci().disconnect_cmds > disc_before, 3000),
		     "link held after discovery failed");
	zassert_true(peer_stats().discovery_failures > 0U);
	zassert_true(UNTIL(hci().scan_enabled, 5000), "hunt did not resume");
}

/* The remote connects and then never answers: bounded by the bring-up
 * watchdog, not by the remote's goodwill. */
ZTEST(ble_sim, test_05_silent_peer_is_released)
{
	uint32_t disc_before = hci().disconnect_cmds;

	fake_hci_set_att_silent(true);
	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	zassert_true(peer_advert(&peer_addr, PEER_NODE));
	zassert_true(UNTIL(hci().initiating, 2000));
	zassert_true(fake_hci_complete_create(0U) > 0);
	zassert_true(UNTIL(hci().disconnect_cmds > disc_before, 15000),
		     "silent peer held forever");
}

/* Every connect attempt fails while the peer keeps advertising. The retry
 * must back off, not redial on every advert. */
ZTEST(ble_sim, test_06_failed_connects_back_off)
{
	uint32_t before;
	int64_t end;

	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	before = hci().create_conn_count;
	end = k_uptime_get() + 3000;
	while (k_uptime_get() < end) {
		(void)peer_advert(&peer_addr, PEER_NODE);
		if (hci().initiating) {
			(void)fake_hci_complete_create(BT_HCI_ERR_CONN_FAIL_TO_ESTAB);
		}
		k_msleep(20);
	}
	/* 3 s at 20 ms adverts: an undelayed loop makes dozens. A backoff
	 * starting at 500 ms and doubling makes at most ~4. */
	zassert_true(hci().create_conn_count - before <= 4U, "%u connects in 3 s",
		     hci().create_conn_count - before);
	zassert_true(hci().create_conn_count - before >= 2U, "it must keep trying");
}

/* The controller refuses one scan start: it must be retried. */
ZTEST(ble_sim, test_07_refused_scan_start_is_retried)
{
	fake_hci_fail(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_ERR_CMD_DISALLOWED, 1U);
	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enable_rejected > 0U || !hci().scan_enabled, 500));
	zassert_true(UNTIL(hci().scan_enabled, 5000), "scanner never retried");
}

/* A phone connects; the controller refuses the advert restart once. The
 * phone advert must come back without waiting for another connection event
 * (with one phone link held, there may never be another). */
ZTEST(ble_sim, test_08_refused_advert_restart_is_retried)
{
	int handle;

	fake_hci_fail(BT_HCI_OP_LE_SET_ADV_ENABLE, BT_HCI_ERR_CMD_DISALLOWED, 1U);
	handle = fake_hci_incoming_conn(&phone_addr);
	zassert_true(handle > 0);
	zassert_true(UNTIL(hci().adv_enable_rejected > 0U, 2000), "restart not attempted");
	zassert_true(UNTIL(hci().adv_enabled, 5000), "advert never came back");
}

/* The peer is already linked to us inbound (it dialled first). Its adverts
 * must not make us dial it again — the host refuses a second link to one
 * address, and an immediate rescan turns that into a spin. */
ZTEST(ble_sim, test_09_inbound_peer_is_not_redialled)
{
	uint32_t attempts, scans;
	int handle = fake_hci_incoming_conn(&peer_addr);

	zassert_true(handle > 0);
	zassert_true(UNTIL(hci().conns_active == 1U && hci().adv_enabled, 2000));
	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	attempts = peer_stats().connects_attempted;
	scans = hci().scan_enable_ok;
	for (int i = 0; i < 50; i++) {
		(void)peer_advert(&peer_addr, PEER_NODE);
		k_msleep(20);
	}
	zassert_equal(peer_stats().connects_attempted, attempts, "redialled an existing link");
	zassert_true(hci().scan_enable_ok - scans <= 1U, "scanner churned %u times",
		     hci().scan_enable_ok - scans);
}

/* One peer has already connected IN; now this node must dial OUT, while still
 * advertising (one slot is free, so it re-armed the advert). Zephyr holds a
 * connection object in reserve for a connectable advert, so on a two-slot
 * build (every XIAO, Heltec classes 4/6) there was none left to dial with:
 * bt_conn_le_create() answered -ENOMEM on every attempt. Found on the bench
 * 2026-09-26: rzr5, kit1 linked in, 1516 failed dials to the courier in two
 * minutes. The dial must get through, and the advert must come back if it
 * does not end in a link. */
ZTEST(ble_sim, test_10_dial_out_with_one_link_in)
{
	int handle = fake_hci_incoming_conn(&phone_addr);
	uint32_t creates;

	zassert_true(handle > 0);
	zassert_true(UNTIL(hci().conns_active == 1U && hci().adv_enabled, 3000),
		     "advert not re-armed after the inbound link");
	creates = hci().create_conn_count;

	CONNECT_OK();
	zassert_true(UNTIL(hci().scan_enabled, 3000));
	zassert_true(peer_advert(&peer_addr, PEER_NODE));
	zassert_true(UNTIL(hci().initiating, 3000), "dial never reached the controller");
	zassert_equal(hci().create_conn_count, creates + 1U);

	/* The attempt fails: the advert must be restored for the free slot. */
	(void)fake_hci_complete_create(BT_HCI_ERR_UNKNOWN_CONN_ID);
	zassert_true(UNTIL(hci().adv_enabled, 3000), "advert not restored after the dial");
}

/* agents-z7rg: a frame completed on the peer frame channel is handed to its
 * callback (the attachment head's or brain's ingest) with the peer lock
 * RELEASED. The head's ingest takes the head's lock, and the head sends to its
 * brain through this module, which takes the peer lock: delivering under the
 * peer lock was one half of the AB-BA that crashed rzr5 (head_tx waiting for
 * peer_lock, the BT RX thread holding it and waiting for head_lock). Here a
 * central writes one frame chunk to our frame characteristic, and the callback
 * asks another thread to take the peer lock -- as head_tx would -- and waits for
 * it. Under the old contract it never could. */
#define FRAME_CHAR_UUID                                                                            \
	BT_UUID_128_ENCODE(0xf3f1a2c2, 0x8e5e, 0x4d8b, 0x9f2a, 0x6c1d3b7e5a10)

static K_SEM_DEFINE(lockprobe_go, 0, 1);
static K_SEM_DEFINE(lockprobe_done, 0, 1);
static volatile bool frame_cb_called;
static volatile bool frame_cb_lock_free;
static uint8_t frame_cb_bytes[8];

static void lockprobe_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&lockprobe_go, K_FOREVER);
		(void)peer_stats(); /* takes peer_lock */
		k_sem_give(&lockprobe_done);
	}
}
K_THREAD_DEFINE(ble_lockprobe, 2048, lockprobe_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, 0);

static void probing_frame_cb(unsigned int index, const uint8_t *frame, size_t len, uint8_t kind)
{
	ARG_UNUSED(index);
	ARG_UNUSED(kind);
	memcpy(frame_cb_bytes, frame, MIN(len, sizeof(frame_cb_bytes)));
	k_sem_reset(&lockprobe_done);
	k_sem_give(&lockprobe_go);
	frame_cb_lock_free = (k_sem_take(&lockprobe_done, K_MSEC(300)) == 0);
	frame_cb_called = true;
}

ZTEST(ble_sim, test_11_frame_callback_runs_without_the_peer_lock)
{
	static const struct bt_uuid_128 frame_uuid = BT_UUID_INIT_128(FRAME_CHAR_UUID);
	const uint8_t frame[] = {0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6};
	struct meshtastic_ble_peer_chunker ck;
	const struct bt_gatt_attr *chrc;
	uint8_t pdu[3 + MESHTASTIC_BLE_PEER_CHUNK_MTU23];
	uint16_t value_handle;
	int handle;
	int n;

	/* The value attribute carries the characteristic's own UUID. */
	chrc = bt_gatt_find_by_uuid(NULL, 0, &frame_uuid.uuid);
	zassert_not_null(chrc, "frame characteristic not in the local GATT db");
	value_handle = bt_gatt_attr_get_handle(chrc);
	zassert_true(value_handle != 0U);

	handle = fake_hci_incoming_conn(&peer_addr);
	zassert_true(handle > 0);
	zassert_true(UNTIL(hci().conns_active == 1U, 2000));

	meshtastic_ble_peer_frame_rx_register(probing_frame_cb);
	frame_cb_called = false;
	frame_cb_lock_free = false;

	zassert_ok(meshtastic_ble_peer_chunker_start_kind(&ck, frame, sizeof(frame),
							  MESHTASTIC_BLE_PEER_KIND_WIRE));
	n = meshtastic_ble_peer_chunker_next(&ck, &pdu[3], MESHTASTIC_BLE_PEER_CHUNK_MTU23);
	zassert_true(n > 0 && n <= MESHTASTIC_BLE_PEER_CHUNK_MTU23, "one chunk (%d)", n);
	pdu[0] = 0x52U; /* ATT Write Command */
	sys_put_le16(value_handle, &pdu[1]);
	fake_hci_remote_att((uint16_t)handle, pdu, (uint16_t)(3 + n));

	zassert_true(UNTIL(frame_cb_called, 3000), "the frame never reached the callback");
	meshtastic_ble_peer_frame_rx_register(NULL);
	zassert_mem_equal(frame_cb_bytes, frame, sizeof(frame), "the frame as written");
	zassert_true(frame_cb_lock_free,
		     "another thread could not take the peer lock while the callback ran");
}
