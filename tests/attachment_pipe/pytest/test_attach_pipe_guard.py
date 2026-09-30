# SPDX-License-Identifier: GPL-3.0
"""X9: the placement guard (ATTACHMENT-DESIGN §13) with a real head measuring a
real link.

A head reports, in every relay's TX_RESULT, how long after it heard the frame
the brain's decision reached it; the brain keeps a per-head p90 and warns when
it exceeds half the head preset's ROUTER window while the identity relays early
(LAG), and when a head's preset is faster than the brain's own (FAST-HEAD).
The unit suite checks the arithmetic on injected samples; here the samples are
made by the hub's latency knob, through both images.
"""

import time

import pytest

from conftest import BRAIN, HEAD1, start_brain, start_head
from meshwire import LONG_FAST, MEDIUM_FAST, SHORT_TURBO, airframe, slot_ms

FAR = 0x0D0D0D0D
PRIVATE_KEY = bytes(range(16))
ROLE_CLIENT, ROLE_ROUTER, ROLE_ROUTER_LATE = 0, 2, 11


def blind(pid, preset):
    return airframe(FAR, pid, "x9", preset=preset, name="x9-private", key=PRIVATE_KEY)


def attach_row(hub, node):
    for line in hub.query(BRAIN, "attach", r"^attach end"):
        if line.startswith("attach id=") and f"node={node:08x}" in line:
            return dict(f.split("=", 1) for f in line.split()[1:])
    raise AssertionError(f"{node:08x} not in the brain's table")


def relays(hub, preset, n, base):
    """Play @p n blind frames at the head and wait for each relay to key up."""
    for i in range(n):
        pid = base + i
        mh = hub.mark(HEAD1)
        hub.rf(HEAD1, preset, -110, -20, blind(pid, preset))
        hub.wait_event(HEAD1, rf"^tx src=0d0d0d0d .*id={pid:08x}", 3, since=mh)
        time.sleep(0.2)  # the TX_RESULT's way back


def rig(hub, images, role, lat_ms, head_preset=MEDIUM_FAST, brain_preset=None):
    start_brain(hub, images, preset=brain_preset)
    start_head(hub, images, HEAD1, head_preset)
    hub.link(BRAIN, HEAD1, lat=(lat_ms, lat_ms))
    hub.ask(BRAIN, f"role {role}", r"^role rc=0")
    time.sleep(0.5)


@pytest.mark.parametrize("lat_ms", [0, 25])
def test_x9a_lag_is_measured_and_warns_a_router(hub, images, lat_ms):
    """A ROUTER brain, a MediumFast head (ROUTER window 60 ms, half is 30 ms).
    With 25 ms each way the decision reaches the head ~50 ms after it heard the
    frame: the head measures it, the brain's p90 is over 30 ms, LAG warns. The
    twin, no link latency: the lag is the brain's own processing, well under
    30 ms, and nothing warns."""
    rig(hub, images, ROLE_ROUTER, lat_ms)
    relays(hub, MEDIUM_FAST, 4, 0x9A00 + lat_ms)
    row = attach_row(hub, HEAD1)
    assert int(row["lag_n"]) >= 4, row
    p50, p90 = int(row["lag_p50"]), int(row["lag_p90"])
    half = (2 * 3 - 1) * slot_ms(MEDIUM_FAST) // 2
    if lat_ms:
        # The head's figure is the link, both ways, plus a little processing.
        assert 2 * lat_ms <= p50 <= 2 * lat_ms + 20, row
        assert p90 > half and row["lag_warn"] == "1", row
    else:
        assert p90 < half and row["lag_warn"] == "0", row


@pytest.mark.parametrize("role", [
    ROLE_CLIENT,
    pytest.param(ROLE_ROUTER_LATE, marks=pytest.mark.xfail(
        strict=True, reason="the LAG guard counts ROUTER_LATE as relaying early; it relays in "
                            "the late (client) window, which the link cannot threaten")),
])
def test_x9b_lag_does_not_warn_a_late_role(hub, images, role):
    """The same 25 ms link under a role that relays in the late window (16 slots
    in, 192 ms here): the lag is measured but cannot make a relay late, so LAG
    stays quiet. CLIENT and ROUTER_LATE both wait out that offset."""
    rig(hub, images, role, 25)
    relays(hub, MEDIUM_FAST, 4, 0x9B00 + role * 16)
    row = attach_row(hub, HEAD1)
    assert int(row["lag_n"]) >= 4, row
    assert row["lag_warn"] == "0", row


@pytest.mark.parametrize("head_preset,warns", [(SHORT_TURBO, True), (LONG_FAST, False)])
def test_x9c_a_head_faster_than_the_brain_warns(hub, images, head_preset, warns):
    """A MediumFast brain (12 ms slot). A ShortTurbo head (8 ms) carries the
    faster preset: FAST-HEAD. A LongFast head (28 ms) is where a head belongs:
    quiet."""
    rig(hub, images, ROLE_CLIENT, 0, head_preset=head_preset, brain_preset=MEDIUM_FAST)
    relays(hub, head_preset, 1, 0x9C00 + head_preset)
    row = attach_row(hub, HEAD1)
    assert row["preset"] == str(head_preset), row
    assert row["fast_head"] == ("1" if warns else "0"), row
