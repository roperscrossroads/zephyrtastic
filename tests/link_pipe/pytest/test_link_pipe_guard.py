# SPDX-License-Identifier: GPL-3.0
"""X9: the placement guard (ATTACHMENT-DESIGN §13) with a real worker measuring a
real link.

A worker reports, in every relay's TX_RESULT, how long after it heard the frame
the hub's decision reached it; the hub keeps a per-worker p90 and warns when
it exceeds half the worker preset's ROUTER window while the identity relays early
(LAG), and when a worker's preset is faster than the hub's own (FAST-WORKER).
The unit suite checks the arithmetic on injected samples; here the samples are
made by the switchboard's latency knob, through both images.
"""

import time

import pytest

from conftest import HUB, WORKER1, start_hub, start_worker
from meshwire import LONG_FAST, MEDIUM_FAST, SHORT_TURBO, airframe, slot_ms

FAR = 0x0D0D0D0D
PRIVATE_KEY = bytes(range(16))
ROLE_CLIENT, ROLE_ROUTER, ROLE_ROUTER_LATE = 0, 2, 11


def blind(pid, preset):
    return airframe(FAR, pid, "x9", preset=preset, name="x9-private", key=PRIVATE_KEY)


def link_row(switchboard, node):
    for line in switchboard.query(HUB, "link", r"^link end"):
        if line.startswith("link id=") and f"node={node:08x}" in line:
            return dict(f.split("=", 1) for f in line.split()[1:])
    raise AssertionError(f"{node:08x} not in the hub's table")


def relays(switchboard, preset, n, base):
    """Play @p n blind frames at the worker and wait for each relay to key up."""
    for i in range(n):
        pid = base + i
        mh = switchboard.mark(WORKER1)
        switchboard.rf(WORKER1, preset, -110, -20, blind(pid, preset))
        switchboard.wait_event(WORKER1, rf"^tx src=0d0d0d0d .*id={pid:08x}", 3, since=mh)
        time.sleep(0.2)  # the TX_RESULT's way back


def hub_setup(switchboard, images, role, lat_ms, worker_preset=MEDIUM_FAST, hub_preset=None):
    start_hub(switchboard, images, preset=hub_preset)
    start_worker(switchboard, images, WORKER1, worker_preset)
    switchboard.link(HUB, WORKER1, lat=(lat_ms, lat_ms))
    switchboard.ask(HUB, f"role {role}", r"^role rc=0")
    time.sleep(0.5)


@pytest.mark.parametrize("lat_ms", [0, 25])
def test_x9a_lag_is_measured_and_warns_a_router(switchboard, images, lat_ms):
    """A ROUTER hub, a MediumFast worker (ROUTER window 60 ms, half is 30 ms).
    With 25 ms each way the decision reaches the worker ~50 ms after it heard the
    frame: the worker measures it, the hub's p90 is over 30 ms, LAG warns. The
    twin, no link latency: the lag is the hub's own processing, well under
    30 ms, and nothing warns."""
    hub_setup(switchboard, images, ROLE_ROUTER, lat_ms)
    relays(switchboard, MEDIUM_FAST, 4, 0x9A00 + lat_ms)
    row = link_row(switchboard, WORKER1)
    assert int(row["lag_n"]) >= 4, row
    p50, p90 = int(row["lag_p50"]), int(row["lag_p90"])
    half = (2 * 3 - 1) * slot_ms(MEDIUM_FAST) // 2
    if lat_ms:
        # The worker's figure is the link, both ways, plus a little processing.
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
def test_x9b_lag_does_not_warn_a_late_role(switchboard, images, role):
    """The same 25 ms link under a role that relays in the late window (16 slots
    in, 256 ms here): the lag is measured but cannot make a relay late, so LAG
    stays quiet. CLIENT and ROUTER_LATE both wait out that offset."""
    hub_setup(switchboard, images, role, 25)
    relays(switchboard, MEDIUM_FAST, 4, 0x9B00 + role * 16)
    row = link_row(switchboard, WORKER1)
    assert int(row["lag_n"]) >= 4, row
    assert row["lag_warn"] == "0", row


@pytest.mark.parametrize("worker_preset,warns", [(SHORT_TURBO, True), (LONG_FAST, False)])
def test_x9c_a_worker_faster_than_the_hub_warns(switchboard, images, worker_preset, warns):
    """A MediumFast hub (16 ms slot). A ShortTurbo worker (8 ms) carries the
    faster preset: FAST-WORKER. A LongFast worker (44 ms) is where a worker belongs:
    quiet."""
    hub_setup(switchboard, images, ROLE_CLIENT, 0, worker_preset=worker_preset, hub_preset=MEDIUM_FAST)
    relays(switchboard, worker_preset, 1, 0x9C00 + worker_preset)
    row = link_row(switchboard, WORKER1)
    assert row["preset"] == str(worker_preset), row
    assert row["fast_worker"] == ("1" if warns else "0"), row
