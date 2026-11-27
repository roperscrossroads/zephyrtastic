# SPDX-License-Identifier: GPL-3.0
"""X7: reply-on-arrival (P3 slice 2) through both images. A request heard
through a worker is answered through that worker, on its preset, with the channel
hash a node on that preset expects -- not on the hub's own radio, where the
sender cannot hear it."""

import time

from conftest import HUB, WORKER1, start_hub, start_worker
from meshwire import (MEDIUM_FAST, PORT_ROUTING, SHORT_TURBO, airframe, channel_hash,
                      read_airframe)

FAR = 0x0D0D0D0D


def replies(switchboard, node, since):
    """Every frame @p node's own radio transmitted since @p since, read back."""
    return [read_airframe(bytes.fromhex(l.split("hex=")[1]))
            for l in switchboard.lines(node, since, r"^txhex ")]


def wait_ack(switchboard, node, since, request_id, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        for f in replies(switchboard, node, since):
            if f.get("port") == PORT_ROUTING and f.get("request_id") == request_id:
                return f
        time.sleep(0.1)
    return None


def test_x7_a_dm_through_a_worker_is_acked_through_that_worker(switchboard, images):
    """FAR, on MediumFast, sends the hub a channel DM with want_ack; only the
    worker hears it. The ACK leaves through the worker, from the hub's id, to
    FAR, with MediumFast's hash (0x1f) on the hub's unnamed slot -- and the
    hub's ShortTurbo radio sends FAR nothing. Twin: the same DM heard by
    the hub's own radio is ACKed on that radio, with ShortTurbo's hash."""
    start_hub(switchboard, images)
    start_worker(switchboard, images, WORKER1, MEDIUM_FAST)
    switchboard.link(HUB, WORKER1)
    time.sleep(0.5)

    mb, mh = switchboard.mark(HUB), switchboard.mark(WORKER1)
    switchboard.rf(WORKER1, MEDIUM_FAST, -80, 5,
           airframe(FAR, 0x7001, "x7-dm", dest=HUB, want_ack=True, preset=MEDIUM_FAST))
    switchboard.wait_event(HUB, r"^rx from=0d0d0d0d to=0a0a0a0a .*text=x7-dm$", 5, since=mb)
    ack = wait_ack(switchboard, WORKER1, mh, 0x7001)
    assert ack is not None, f"no ACK through the worker: {replies(switchboard, WORKER1, mh)}"
    assert (ack["src"], ack["dest"]) == (HUB, FAR), ack
    assert ack["ch"] == channel_hash("MediumFast"), f"ACK hash {ack['ch']:#04x}, not MediumFast's"
    assert [f for f in replies(switchboard, HUB, mb) if f["dest"] == FAR] == [], \
        "the hub's own radio answered a sender on another preset"

    mb, mh = switchboard.mark(HUB), switchboard.mark(WORKER1)
    switchboard.rf(HUB, SHORT_TURBO, -80, 5,
           airframe(FAR, 0x7002, "x7-local", dest=HUB, want_ack=True, preset=SHORT_TURBO))
    switchboard.wait_event(HUB, r"^rx from=0d0d0d0d to=0a0a0a0a .*text=x7-local$", 5, since=mb)
    ack = wait_ack(switchboard, HUB, mb, 0x7002)
    assert ack is not None, f"no ACK on the hub's own radio: {replies(switchboard, HUB, mb)}"
    assert ack["ch"] == channel_hash("ShortTurbo"), f"ACK hash {ack['ch']:#04x}"
    assert wait_ack(switchboard, WORKER1, mh, 0x7002, timeout=1) is None, \
        "a request heard by the hub's own radio was answered through the worker"
