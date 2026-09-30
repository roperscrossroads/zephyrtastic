# SPDX-License-Identifier: GPL-3.0
"""The cross-preset relay (agents-jbrq.12) as two images: a real ear hears a
LongFast frame, forwards it over the peer link (here the pipe), and a real
receiving half re-originates the text on MediumFast. What the half puts on the
air is read back from its radio and decrypted, so every "relayed" below is a
frame a MediumFast node would receive.

The single-image suite (tests/relay) scripts the ear's side; this puts both
halves' real code on each end of a link with latency, and checks the ear's
framing choice (bare frame vs attachment envelope) against the peer's flag.
"""

import time

from conftest import EAR, HALF, pair
from meshwire import (BROADCAST, LONG_FAST, PORT_TEXT, airframe, channel_hash,
                      read_airframe)

FAR = 0x0D0D0D0D
PREFIX = "[0d0d] "
PORT_POSITION = 3


def heard_by_ear(hub, pid, text, **kw):
    hub.rf(EAR, LONG_FAST, -90, 4, airframe(FAR, pid, text, preset=LONG_FAST, **kw))


def half_texts(hub, since):
    """The texts the half transmitted since @p since, read back from its air."""
    out = []
    for l in hub.lines(HALF, since, r"^txhex "):
        f = read_airframe(bytes.fromhex(l.split("hex=")[1]))
        if f.get("port") == PORT_TEXT:
            f["text"] = f["payload"].decode(errors="replace")
            out.append(f)
    return out


def wait_text(hub, since, text, timeout=4):
    deadline = time.time() + timeout
    while time.time() < deadline:
        for f in half_texts(hub, since):
            if f["text"] == text:
                return f
        time.sleep(0.1)
    return None


def stats(hub, node, prefix):
    line = hub.ask(node, "stats", rf"^{prefix} ")
    return {k: int(v) for k, v in (f.split("=") for f in line.split()[1:])}


def test_rp1_a_longfast_text_crosses_to_mediumfast(hub, images):
    """A public LongFast text heard by the ear leaves the half as a NEW packet:
    from the half's id, broadcast, "[0d0d] " prefixed, a fresh hop count, on
    MediumFast's public hash (0x1f). The ear sent a bare frame -- the half's
    link does not take envelopes."""
    pair(hub, images)
    mh = hub.mark(HALF)
    heard_by_ear(hub, 0x1001, "hello from LongFast")
    f = wait_text(hub, mh, PREFIX + "hello from LongFast")
    assert f is not None, f"nothing crossed: {half_texts(hub, mh)}"
    assert (f["src"], f["dest"]) == (HALF, BROADCAST), f
    assert f["id"] != 0x1001, "a re-originated packet has its own id"
    assert f["ch"] == channel_hash("MediumFast"), f"{f['ch']:#04x}"
    assert len(hub.wires_forwarded) == 1 and hub.forwarded == [], "the ear must send a bare frame"
    assert stats(hub, HALF, "relay")["relayed"] == 1


def test_rp2_only_broadcast_text_crosses(hub, images):
    """A position broadcast and a text DM are heard by the ear and go no
    further; the ear counts why. Twin: the text broadcast after them crosses."""
    pair(hub, images)
    mh = hub.mark(HALF)
    heard_by_ear(hub, 0x2001, "not text", port=PORT_POSITION)
    heard_by_ear(hub, 0x2002, "a DM", dest=0x12345678)
    time.sleep(1.0)
    ear = stats(hub, EAR, "ear")
    assert (ear["not_text"], ear["not_broadcast"], ear["forwarded"]) == (1, 1, 0), ear
    assert hub.wires_forwarded == []
    heard_by_ear(hub, 0x2003, "text")
    assert wait_text(hub, mh, PREFIX + "text") is not None
    assert [f["text"] for f in half_texts(hub, mh)] == [PREFIX + "text"]


def test_rp3_a_relayed_text_is_not_relayed_again(hub, images):
    """Loop guard: a text that already carries a relay prefix (another relay's
    work) is forwarded by the ear but refused by the half. Twin: RP1."""
    pair(hub, images)
    mh = hub.mark(HALF)
    heard_by_ear(hub, 0x3001, "[abcd] relayed elsewhere")
    time.sleep(1.5)
    assert half_texts(hub, mh) == []
    assert stats(hub, HALF, "relay")["prefixed"] == 1


def test_rp4_one_text_crosses_once(hub, images):
    """The ear hears the same packet twice (the original, then a neighbour's
    rebroadcast): its router dedups, one frame crosses, one text goes out. The
    same text again under a new id inside the seen TTL is refused by the half.
    Twin: a different text from the same origin still crosses."""
    pair(hub, images)
    mh = hub.mark(HALF)
    heard_by_ear(hub, 0x4001, "once")
    time.sleep(0.1)
    heard_by_ear(hub, 0x4001, "once", hop_limit=2, relay_node=0x55)
    assert wait_text(hub, mh, PREFIX + "once") is not None
    heard_by_ear(hub, 0x4002, "once")
    time.sleep(1.5)
    assert [f["text"] for f in half_texts(hub, mh)] == [PREFIX + "once"]
    assert len(hub.wires_forwarded) == 2, "the ear forwards each distinct packet once"
    assert stats(hub, HALF, "relay")["seen"] == 1
    heard_by_ear(hub, 0x4003, "twice")
    assert wait_text(hub, mh, PREFIX + "twice") is not None


def test_rp5_the_per_origin_cap(hub, images):
    """Three texts per origin per minute (the bench's cap): the fourth is held
    back and counted."""
    pair(hub, images)
    mh = hub.mark(HALF)
    for i in range(4):
        heard_by_ear(hub, 0x5001 + i, f"burst {i}")
        time.sleep(0.3)
    time.sleep(2.0)
    sent = sorted(f["text"] for f in half_texts(hub, mh))
    assert sent == [PREFIX + f"burst {i}" for i in range(3)], sent
    assert stats(hub, HALF, "relay")["origin_limited"] == 1


def test_rp6_the_ear_frames_by_the_peers_flag(hub, images):
    """If the peer says it takes envelopes (a brain's beats carry ATTACH), the
    ear sends an attachment envelope instead of a bare frame. A relay half
    cannot use one -- it has no attachment code -- so nothing crosses: a half
    must never advertise ATTACH. Twin: RP1, the same text as a bare frame."""
    pair(hub, images, takes_env=True)
    mh = hub.mark(HALF)
    heard_by_ear(hub, 0x6001, "wrapped")
    hub.wait_event(HALF, r"^pipe env_rx ", 3, since=mh)
    time.sleep(1.0)
    assert hub.wires_forwarded == [] and len(hub.forwarded) == 1, "the ear sent a bare frame"
    assert hub.forwarded[0][2][0] == 1, "not an RX_FRAME envelope"
    assert half_texts(hub, mh) == []


def test_rp7_a_dropped_link_is_counted_and_recovers(hub, images):
    """With the peer link down the ear counts a failed send and nothing
    crosses; with it back, the next text does."""
    pair(hub, images)
    mh = hub.mark(HALF)
    hub.tell_link(EAR, HALF, up=False, takes_env=False)
    heard_by_ear(hub, 0x7001, "into the void")
    time.sleep(1.0)
    assert stats(hub, EAR, "ear")["send_failed"] == 1
    assert half_texts(hub, mh) == []
    hub.tell_link(EAR, HALF, up=True, takes_env=False)
    heard_by_ear(hub, 0x7002, "back")
    assert wait_text(hub, mh, PREFIX + "back") is not None
