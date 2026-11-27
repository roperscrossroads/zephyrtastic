# SPDX-License-Identifier: GPL-3.0
"""The cross-preset relay (agents-jbrq.12) as two images: a real worker_rx hears a
LongFast frame, forwards it over the peer link (here the pipe), and a real
receiving half re-originates the text on MediumFast. What the half puts on the
air is read back from its radio and decrypted, so every "relayed" below is a
frame a MediumFast node would receive.

The single-image suite (tests/relay) scripts the worker_rx's side; this puts both
halves' real code on each end of a link with latency, and checks the worker_rx's
framing choice (bare frame vs link envelope) against the peer's flag.
"""

import time

from conftest import WORKER_RX, HALF, pair
from meshwire import (BROADCAST, LONG_FAST, PORT_TEXT, airframe, channel_hash,
                      read_airframe)

FAR = 0x0D0D0D0D
PREFIX = "[0d0d] "
PORT_POSITION = 3


def heard_by_worker_rx(switchboard, pid, text, **kw):
    switchboard.rf(WORKER_RX, LONG_FAST, -90, 4, airframe(FAR, pid, text, preset=LONG_FAST, **kw))


def half_texts(switchboard, since):
    """The texts the half transmitted since @p since, read back from its air."""
    out = []
    for l in switchboard.lines(HALF, since, r"^txhex "):
        f = read_airframe(bytes.fromhex(l.split("hex=")[1]))
        if f.get("port") == PORT_TEXT:
            f["text"] = f["payload"].decode(errors="replace")
            out.append(f)
    return out


def wait_text(switchboard, since, text, timeout=4):
    deadline = time.time() + timeout
    while time.time() < deadline:
        for f in half_texts(switchboard, since):
            if f["text"] == text:
                return f
        time.sleep(0.1)
    return None


def stats(switchboard, node, prefix):
    line = switchboard.ask(node, "stats", rf"^{prefix} ")
    return {k: int(v) for k, v in (f.split("=") for f in line.split()[1:])}


def test_rp1_a_longfast_text_crosses_to_mediumfast(switchboard, images):
    """A public LongFast text heard by the worker_rx leaves the half as a NEW packet:
    from the half's id, broadcast, "[0d0d] " prefixed, a fresh hop count, on
    MediumFast's public hash (0x1f). The worker_rx sent a bare frame -- the half's
    link does not take envelopes."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    heard_by_worker_rx(switchboard, 0x1001, "hello from LongFast")
    f = wait_text(switchboard, mh, PREFIX + "hello from LongFast")
    assert f is not None, f"nothing crossed: {half_texts(switchboard, mh)}"
    assert (f["src"], f["dest"]) == (HALF, BROADCAST), f
    assert f["id"] != 0x1001, "a re-originated packet has its own id"
    assert f["ch"] == channel_hash("MediumFast"), f"{f['ch']:#04x}"
    assert len(switchboard.wires_forwarded) == 1 and switchboard.forwarded == [], "the worker_rx must send a bare frame"
    assert stats(switchboard, HALF, "relay")["relayed"] == 1


def test_rp2_only_broadcast_text_crosses(switchboard, images):
    """A position broadcast and a text DM are heard by the worker_rx and go no
    further; the worker_rx counts why. Twin: the text broadcast after them crosses."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    heard_by_worker_rx(switchboard, 0x2001, "not text", port=PORT_POSITION)
    heard_by_worker_rx(switchboard, 0x2002, "a DM", dest=0x12345678)
    time.sleep(1.0)
    worker_rx = stats(switchboard, WORKER_RX, "worker_rx")
    assert (worker_rx["not_text"], worker_rx["not_broadcast"], worker_rx["forwarded"]) == (1, 1, 0), worker_rx
    assert switchboard.wires_forwarded == []
    heard_by_worker_rx(switchboard, 0x2003, "text")
    assert wait_text(switchboard, mh, PREFIX + "text") is not None
    assert [f["text"] for f in half_texts(switchboard, mh)] == [PREFIX + "text"]


def test_rp3_a_relayed_text_is_not_relayed_again(switchboard, images):
    """Loop guard: a text that already carries a relay prefix (another relay's
    work) is forwarded by the worker_rx but refused by the half. Twin: RP1."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    heard_by_worker_rx(switchboard, 0x3001, "[abcd] relayed elsewhere")
    time.sleep(1.5)
    assert half_texts(switchboard, mh) == []
    assert stats(switchboard, HALF, "relay")["prefixed"] == 1


def test_rp4_one_text_crosses_once(switchboard, images):
    """The worker_rx hears the same packet twice (the original, then a neighbour's
    rebroadcast): its router dedups, one frame crosses, one text goes out. The
    same text again under a new id inside the seen TTL is refused by the half.
    Twin: a different text from the same origin still crosses."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    heard_by_worker_rx(switchboard, 0x4001, "once")
    time.sleep(0.1)
    heard_by_worker_rx(switchboard, 0x4001, "once", hop_limit=2, relay_node=0x55)
    assert wait_text(switchboard, mh, PREFIX + "once") is not None
    heard_by_worker_rx(switchboard, 0x4002, "once")
    time.sleep(1.5)
    assert [f["text"] for f in half_texts(switchboard, mh)] == [PREFIX + "once"]
    assert len(switchboard.wires_forwarded) == 2, "the worker_rx forwards each distinct packet once"
    assert stats(switchboard, HALF, "relay")["seen"] == 1
    heard_by_worker_rx(switchboard, 0x4003, "twice")
    assert wait_text(switchboard, mh, PREFIX + "twice") is not None


def test_rp5_the_per_origin_cap(switchboard, images):
    """Three texts per origin per minute (the bench's cap): the fourth is held
    back and counted."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    for i in range(4):
        heard_by_worker_rx(switchboard, 0x5001 + i, f"burst {i}")
        time.sleep(0.3)
    time.sleep(2.0)
    sent = sorted(f["text"] for f in half_texts(switchboard, mh))
    assert sent == [PREFIX + f"burst {i}" for i in range(3)], sent
    assert stats(switchboard, HALF, "relay")["origin_limited"] == 1


def test_rp6_the_worker_rx_frames_by_the_peers_flag(switchboard, images):
    """If the peer says it takes envelopes (a hub's beats carry LINK), the
    worker_rx sends an link envelope instead of a bare frame. A relay half
    cannot use one -- it has no link code -- so nothing crosses: a half
    must never advertise LINK. Twin: RP1, the same text as a bare frame."""
    pair(switchboard, images, takes_env=True)
    mh = switchboard.mark(HALF)
    heard_by_worker_rx(switchboard, 0x6001, "wrapped")
    switchboard.wait_event(HALF, r"^pipe env_rx ", 3, since=mh)
    time.sleep(1.0)
    assert switchboard.wires_forwarded == [] and len(switchboard.forwarded) == 1, "the worker_rx sent a bare frame"
    assert switchboard.forwarded[0][2][0] == 1, "not an RX_FRAME envelope"
    assert half_texts(switchboard, mh) == []


def test_rp7_a_dropped_link_is_counted_and_recovers(switchboard, images):
    """With the peer link down the worker_rx counts a failed send and nothing
    crosses; with it back, the next text does."""
    pair(switchboard, images)
    mh = switchboard.mark(HALF)
    switchboard.tell_link(WORKER_RX, HALF, up=False, takes_env=False)
    heard_by_worker_rx(switchboard, 0x7001, "into the void")
    time.sleep(1.0)
    assert stats(switchboard, WORKER_RX, "worker_rx")["send_failed"] == 1
    assert half_texts(switchboard, mh) == []
    switchboard.tell_link(WORKER_RX, HALF, up=True, takes_env=False)
    heard_by_worker_rx(switchboard, 0x7002, "back")
    assert wait_text(switchboard, mh, PREFIX + "back") is not None
