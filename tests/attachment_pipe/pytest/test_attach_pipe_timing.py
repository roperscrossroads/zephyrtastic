# SPDX-License-Identifier: GPL-3.0
"""X6 of the attachment test plan: a relay through a head keeps the reference's
rebroadcast timing (ATTACHMENT-DESIGN §12, the operator's constraint), whatever
the latency of the link between brain and head.

The brain decides whether to relay and by which rule; the head keys up at
(its own reception of the frame) + not_before, on its own clock. So the delay
between the head hearing a frame and the head transmitting the relay must fall
inside the window the reference computes for the head's modem and the SNR the
head heard -- the oracle is meshwire.relay_window(), written from
RadioInterface.cpp, not from the port.

The frames are on a channel the brain has no key for: it relays them
undecoded, as a stock node in rebroadcast ALL does, and there is no NodeInfo
request through the head to share its transmitter with the relay.
"""

import time

import pytest

from conftest import BRAIN, HEAD1, start_brain, start_head
from meshwire import MEDIUM_FAST, airframe, relay_window, relay_worst

FAR = 0x0D0D0D0D
SNR = -20                        # the tightest window: CW 3
PRIVATE_KEY = bytes(range(16))   # a channel only FAR has
ROLE_CLIENT, ROLE_ROUTER, ROLE_ROUTER_LATE = 0, 2, 11
# Both clocks tick at 1 ms; key-up adds the head's CAD/LBT and the sim radio's
# settle. Early is never excused beyond the timestamps' own rounding.
EARLY_MS, LATE_MS = 2, 25


def blind(pid, hop_limit=3, relay_node=None):
    return airframe(FAR, pid, "x6", preset=MEDIUM_FAST, name="x6-private", key=PRIVATE_KEY,
                    hop_limit=hop_limit, hop_start=3, relay_node=relay_node)


def setup(hub, images, lat_ms, role):
    start_brain(hub, images)
    start_head(hub, images, HEAD1, MEDIUM_FAST)
    hub.link(BRAIN, HEAD1, lat=(lat_ms, lat_ms))
    hub.ask(BRAIN, f"role {role}", r"^role rc=0")
    time.sleep(0.5)  # the head's STATUS: the brain learns its preset (and modem)


def head_heard_then_tx(hub, pid, wire, timeout=3):
    """Play @p wire at the head; return (delay heard->key-up in ms, tx line),
    or (None, None) when the head never relayed it."""
    mh = hub.mark(HEAD1)
    hub.rf(HEAD1, MEDIUM_FAST, -110, SNR, wire)
    heard = int(hub.wait_event(HEAD1, r"^rf heard ", 2, since=mh).split("t=")[1])
    try:
        tx = hub.wait_event(HEAD1, rf"^tx src=0d0d0d0d .*id={pid:08x}", timeout, since=mh)
    except TimeoutError:
        return None, None
    return int(tx.split(" t=")[1].split()[0]) - heard, tx


@pytest.mark.parametrize("lat_ms", [0, 40, 80])
def test_x6a_client_relay_keys_up_inside_the_reference_window(hub, images, lat_ms):
    """A CLIENT brain's relay through a head, with the link adding lat_ms each
    way (up to 160 ms round trip, still inside the 192 ms client offset): every
    key-up falls in the reference window measured from the HEAD's reception,
    i.e. the link latency does not show."""
    setup(hub, images, lat_ms, ROLE_CLIENT)
    lo, hi = relay_window(MEDIUM_FAST, SNR)
    for i in range(4):
        pid = 0x6A00 + lat_ms * 8 + i
        d, tx = head_heard_then_tx(hub, pid, blind(pid))
        assert tx is not None, f"no relay through the head for {pid:08x}"
        assert "hops=2" in tx, tx  # one hop spent, as any relay
        assert lo - EARLY_MS <= d <= hi + LATE_MS, f"key-up {d} ms outside [{lo}, {hi}]"
        time.sleep(0.4)


@pytest.mark.parametrize("lat_ms", [0, 150])
def test_x6b_router_relays_early_and_a_late_decision_still_relays(hub, images, lat_ms):
    """A ROUTER's window is 0-60 ms here. On time (no latency) the head keys up
    inside it. With 150 ms each way the decision reaches the head after the
    window closed: D3 says a ROUTER relays anyway -- as soon as it can."""
    setup(hub, images, lat_ms, ROLE_ROUTER)
    lo, hi = relay_window(MEDIUM_FAST, SNR, router=True)
    for i in range(3):
        pid = 0x6B00 + lat_ms * 8 + i
        d, tx = head_heard_then_tx(hub, pid, blind(pid))
        assert tx is not None, f"a ROUTER's relay was dropped ({pid:08x})"
        if lat_ms == 0:
            assert lo - EARLY_MS <= d <= hi + LATE_MS, f"key-up {d} ms outside [{lo}, {hi}]"
        else:
            # Not before the decision could arrive; promptly once it has.
            assert 2 * lat_ms - EARLY_MS <= d <= 2 * lat_ms + hi + LATE_MS, d
        time.sleep(0.4)


def test_x6c_a_late_client_decision_is_dropped(hub, images):
    """A CLIENT's window here closes 276 ms after the head heard the frame; with
    200 ms each way the decision arrives at ~400 ms. D3: a late client relay is
    the duplicate cancel-on-dupe exists to prevent, so it is dropped. Twin:
    X6a, the same client relaying on time."""
    setup(hub, images, 200, ROLE_CLIENT)
    pid = 0x6C01
    d, tx = head_heard_then_tx(hub, pid, blind(pid), timeout=2)
    assert tx is None, f"a late CLIENT relay went out {d} ms after the head heard the frame"


def test_x6d_a_client_head_cancels_on_a_heard_duplicate(hub, images):
    """The head hears a neighbour relay the frame (hop 2, a foreign relay byte)
    60 ms after the original -- before the client window opens at 192 ms. The
    reference's cancel-on-duplicate: our relay is withdrawn. Twin: X6a, the
    same relay with no duplicate, goes out."""
    setup(hub, images, 0, ROLE_CLIENT)
    pid = 0x6D01
    mh = hub.mark(HEAD1)
    hub.rf(HEAD1, MEDIUM_FAST, -110, SNR, blind(pid))
    time.sleep(0.06)
    hub.rf(HEAD1, MEDIUM_FAST, -100, SNR, blind(pid, hop_limit=2, relay_node=0x55))
    time.sleep(1.0)
    assert hub.lines(HEAD1, mh, rf"^tx src=0d0d0d0d .*id={pid:08x}") == [], \
        "the head relayed a frame a neighbour had already relayed"


def test_x6e_router_late_clamps_to_the_end_of_the_window(hub, images):
    """ROUTER_LATE hears the duplicate and does not cancel: the reference
    (RadioLibInterface::clampToLateRebroadcastWindow) moves its relay to
    tx_after = now + getTxDelayMsecWeightedWorst(snr) -- measured from the
    moment the COPY is heard, 288 ms here."""
    setup(hub, images, 0, ROLE_ROUTER_LATE)
    worst = relay_worst(MEDIUM_FAST, SNR)
    pid = 0x6E01
    mh = hub.mark(HEAD1)
    hub.rf(HEAD1, MEDIUM_FAST, -110, SNR, blind(pid))
    time.sleep(0.06)
    mdup = hub.mark(HEAD1)
    hub.rf(HEAD1, MEDIUM_FAST, -100, SNR, blind(pid, hop_limit=2, relay_node=0x55))
    dup_heard = int(hub.wait_event(HEAD1, r"^rf heard ", 2, since=mdup).split("t=")[1])
    tx = hub.wait_event(HEAD1, rf"^tx src=0d0d0d0d .*id={pid:08x}", 3, since=mh)
    d = int(tx.split(" t=")[1].split()[0]) - dup_heard
    assert worst - EARLY_MS <= d <= worst + LATE_MS, \
        f"ROUTER_LATE keyed up {d} ms after hearing the copy; the reference says {worst}"
