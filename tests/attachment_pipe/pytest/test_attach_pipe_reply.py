# SPDX-License-Identifier: GPL-3.0
"""X7: reply-on-arrival (P3 slice 2) through both images. A request heard
through a head is answered through that head, on its preset, with the channel
hash a node on that preset expects -- not on the brain's own radio, where the
sender cannot hear it."""

import time

from conftest import BRAIN, HEAD1, start_brain, start_head
from meshwire import (MEDIUM_FAST, PORT_ROUTING, SHORT_TURBO, airframe, channel_hash,
                      read_airframe)

FAR = 0x0D0D0D0D


def replies(hub, node, since):
    """Every frame @p node's own radio transmitted since @p since, read back."""
    return [read_airframe(bytes.fromhex(l.split("hex=")[1]))
            for l in hub.lines(node, since, r"^txhex ")]


def wait_ack(hub, node, since, request_id, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        for f in replies(hub, node, since):
            if f.get("port") == PORT_ROUTING and f.get("request_id") == request_id:
                return f
        time.sleep(0.1)
    return None


def test_x7_a_dm_through_a_head_is_acked_through_that_head(hub, images):
    """FAR, on MediumFast, sends the brain a channel DM with want_ack; only the
    head hears it. The ACK leaves through the head, from the brain's id, to
    FAR, with MediumFast's hash (0x1f) on the brain's unnamed slot -- and the
    brain's ShortTurbo radio sends FAR nothing. Twin: the same DM heard by
    the brain's own radio is ACKed on that radio, with ShortTurbo's hash."""
    start_brain(hub, images)
    start_head(hub, images, HEAD1, MEDIUM_FAST)
    hub.link(BRAIN, HEAD1)
    time.sleep(0.5)

    mb, mh = hub.mark(BRAIN), hub.mark(HEAD1)
    hub.rf(HEAD1, MEDIUM_FAST, -80, 5,
           airframe(FAR, 0x7001, "x7-dm", dest=BRAIN, want_ack=True, preset=MEDIUM_FAST))
    hub.wait_event(BRAIN, r"^rx from=0d0d0d0d to=0a0a0a0a .*text=x7-dm$", 5, since=mb)
    ack = wait_ack(hub, HEAD1, mh, 0x7001)
    assert ack is not None, f"no ACK through the head: {replies(hub, HEAD1, mh)}"
    assert (ack["src"], ack["dest"]) == (BRAIN, FAR), ack
    assert ack["ch"] == channel_hash("MediumFast"), f"ACK hash {ack['ch']:#04x}, not MediumFast's"
    assert [f for f in replies(hub, BRAIN, mb) if f["dest"] == FAR] == [], \
        "the brain's own radio answered a sender on another preset"

    mb, mh = hub.mark(BRAIN), hub.mark(HEAD1)
    hub.rf(BRAIN, SHORT_TURBO, -80, 5,
           airframe(FAR, 0x7002, "x7-local", dest=BRAIN, want_ack=True, preset=SHORT_TURBO))
    hub.wait_event(BRAIN, r"^rx from=0d0d0d0d to=0a0a0a0a .*text=x7-local$", 5, since=mb)
    ack = wait_ack(hub, BRAIN, mb, 0x7002)
    assert ack is not None, f"no ACK on the brain's own radio: {replies(hub, BRAIN, mb)}"
    assert ack["ch"] == channel_hash("ShortTurbo"), f"ACK hash {ack['ch']:#04x}"
    assert wait_ack(hub, HEAD1, mh, 0x7002, timeout=1) is None, \
        "a request heard by the brain's own radio was answered through the head"
