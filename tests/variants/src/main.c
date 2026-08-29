/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Variant sweep (agents-zo3o.4): does every supported combination of the
 * module-level Kconfig bools still BUILD, LINK and BOOT?
 *
 * WHY THIS EXISTS. On 2026-08-29, CONFIG_MESHTASTIC_GNSS_AUTO_SEND=n -- an
 * ordinary, documented, default-y bool -- did not compile. Not because the code
 * was wrong in an interesting way, but because no build had ever selected it:
 * fleet class 5 (the listener) was the first configuration in the project's
 * history to want a node that receives GNSS fixes and never transmits them. A
 * config that looks supported and is not is worse than one that does not exist.
 *
 * The specific trap that produced it, and the reason a sweep rather than a
 * reading catches it: a symbol declared under `#if defined(CONFIG_X)` but
 * referenced from inside `IS_ENABLED(CONFIG_X)`. IS_ENABLED deliberately keeps
 * BOTH arms compiled so the compiler can check the dead one -- which is a good
 * property everywhere except over a symbol that only exists in one arm. The
 * same shape also produces -Werror=unused-variable when a static's only
 * reference is in the arm that was compiled out (found by this sweep's sibling,
 * the AUTO_SEND=n GNSS scenario).
 *
 * WHAT THIS IS NOT. It is not a behaviour test -- every scenario runs the same
 * near-empty body, and the assertions below are deliberately shallow. The value
 * is entirely in the MATRIX in testcase.yaml. Behaviour for each module belongs
 * in that module's own suite; if you find yourself wanting to assert something
 * specific here, that is a signal the module needs a suite, not that this one
 * needs a branch.
 *
 * WHAT IT DOES NOT COVER. native_sim only, so it says nothing about a
 * board-specific link (the CONFIG_BT=y rot that motivated agents-zo3o.4
 * originally would still slip past). Hardware-target coverage of the five fleet
 * IMAGE CLASSES is the other half of that bead and lives with the sample.
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_channels.h"
#include "meshtastic_core.h"

#define TEST_NODE_ID 0x0A0A0A0AU

static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/*
 * Booting is the assertion. meshtastic_init() runs every enabled module's init
 * in one ordered pass, so a configuration that links but wires itself up wrong
 * -- a module whose init depends on one that was compiled out, an init ordering
 * that only holds when some optional peer is present -- fails here rather than
 * on a bench node an hour later.
 */
ZTEST(variants, test_stack_initialises_in_this_configuration)
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
	zassert_ok(meshtastic_init(&cfg), "meshtastic_init failed in this configuration");

	/* Give the enabled modules' start-up work a moment to run, so an init that
	 * defers its real work to a workqueue still gets to fault here. */
	k_msleep(200);
}

ZTEST_SUITE(variants, NULL, NULL, NULL, NULL, NULL);
