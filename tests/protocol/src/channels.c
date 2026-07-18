/* Channel key-resolution regression tests.
 *
 * SPDX-License-Identifier: GPL-3.0
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "meshtastic_channels.h"

static void channels_before(void *fixture)
{
	ARG_UNUSED(fixture);
	meshtastic_channels_init_defaults();
}

/* An empty-PSK SECONDARY borrows the primary's key. If the primary slot is
 * itself marked SECONDARY, that borrow points at its own index and recurses
 * forever -- nothing enforces that channel_slots[primary_index] has role
 * PRIMARY. Reachable from meshtastic_channels_set_slot(), i.e. from the admin
 * set_channel path, so a peer that can set a channel can wedge the node.
 *
 * Note on failure mode: a regression here HANGS rather than failing, because
 * the recursive call is in tail position and the compiler turns it into an
 * infinite loop instead of blowing the stack. Detecting it cleanly in-process
 * would mean adding a depth counter to production code purely for testability,
 * which is not worth it -- twister's per-suite timeout catches the hang.
 * Reaching the assertions below at all is the real signal.
 */
ZTEST(channels, test_secondary_at_primary_index_does_not_recurse)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	struct meshtastic_channel_key key;

	/* Slot 0 is primary_index after init_defaults(). Installing a SECONDARY
	 * there leaves primary_index pointing at a non-PRIMARY slot -- set_slot
	 * only reassigns primary_index when the incoming role IS primary.
	 */
	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 0;
	strncpy(ch.settings.name, "selfref", sizeof(ch.settings.name) - 1U);

	zassert_ok(meshtastic_channels_set_slot(0, &ch),
		   "set_slot must return rather than recurse");

	zassert_equal(meshtastic_channels_primary_index(), 0,
		      "precondition: slot 0 is still primary_index");

	/* The self-referential borrow resolves to cleartext, matching a PRIMARY
	 * with an empty PSK, rather than looping.
	 */
	zassert_ok(meshtastic_channels_primary_key(&key),
		   "primary_key must return rather than recurse");
	zassert_equal(key.len, 0, "self-referential secondary resolves to cleartext");
}

/* The legitimate borrow must keep working: a SECONDARY at a different index
 * with no PSK still inherits the primary's key.
 */
ZTEST(channels, test_secondary_borrows_primary_key)
{
	meshtastic_Channel ch = meshtastic_Channel_init_zero;
	struct meshtastic_channel_key primary, borrowed;

	zassert_ok(meshtastic_channels_primary_key(&primary), "primary key unavailable");
	zassert_true(primary.len > 0, "default primary should carry a key");

	ch.role = meshtastic_Channel_Role_SECONDARY;
	ch.has_settings = true;
	ch.settings.psk.size = 0;
	strncpy(ch.settings.name, "borrower", sizeof(ch.settings.name) - 1U);
	zassert_ok(meshtastic_channels_set_slot(1, &ch), "set_slot failed");

	zassert_ok(meshtastic_channels_get_key(1, &borrowed), "get_key failed");
	zassert_equal(borrowed.len, primary.len, "borrowed key length differs");
	zassert_mem_equal(borrowed.bytes, primary.bytes, primary.len,
			  "secondary did not inherit the primary key");
}

ZTEST_SUITE(channels, NULL, NULL, channels_before, NULL, NULL);
