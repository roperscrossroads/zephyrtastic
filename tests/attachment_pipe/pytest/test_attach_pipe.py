# SPDX-License-Identifier: GPL-3.0
"""Layer 3 of the attachment test plan (ATTACHMENT-REVIEW part 2): a real brain
image and real keyless head images, joined through the attachment pipe.

X1-X5 are the pre-P3 cases. Every "does not" assertion has its positive twin in
the same case (ATTACHMENT-SCOPE §5).
"""

import time

from conftest import BRAIN, HEAD1, HEAD2, start_brain, start_head
from hub import AUTH_ENCRYPTED, AUTH_NONE
from meshwire import MEDIUM_FAST, SHORT_TURBO, airframe, env_rx_frame, us_tuning

FAR = 0x0D0D0D0D
RX = r"^rx from=0d0d0d0d "


def rx_lines(hub, since, text):
    return hub.lines(BRAIN, since, rf"^rx from=0d0d0d0d .*text={text}$")


def tx_lines(hub, since, packet_id):
    return hub.lines(BRAIN, since, rf"^tx src=0d0d0d0d .*id={packet_id:08x}")


def attach_rows(hub):
    rows = {}
    for line in hub.query(BRAIN, "attach", r"^attach end"):
        if line.startswith("attach id="):
            kv = dict(f.split("=", 1) for f in line.split()[1:])
            rows[int(kv["node"], 16)] = kv
    return rows


def pair(hub, images, node=HEAD1, preset=MEDIUM_FAST, **knobs):
    start_head(hub, images, node, preset)
    hub.link(BRAIN, node, **knobs)
    # The head introduces itself with STATUS once it has a link.
    hub.wait_event(node, r"^ready ", 5)


def test_x1_head_on_another_preset_decodes_on_the_brain(hub, images):
    """The 05:55Z bench proof, on native_sim: a MediumFast frame heard by a head
    is delivered on the ShortTurbo brain from the origin's id, with the head's
    signal, and is not relayed on the brain's radio. Twin: the same kind of
    frame heard by the brain's OWN radio on ShortTurbo is relayed."""
    start_brain(hub, images)
    pair(hub, images)
    m = hub.mark(BRAIN)

    hub.rf(HEAD1, MEDIUM_FAST, -70, 10, airframe(FAR, 0x1001, "x1-hello", preset=MEDIUM_FAST))
    line = hub.wait_event(BRAIN, RX + r".*text=x1-hello$", 5, since=m)
    assert "rssi=-70 snr=10" in line, line
    time.sleep(2.0)
    assert tx_lines(hub, m, 0x1001) == [], "a head's frame left on the brain's own radio"
    row = attach_rows(hub)[HEAD1]
    assert row["preset"] == str(MEDIUM_FAST) and int(row["rx"]) >= 1, row

    # Twin: the brain's own radio, its own preset -- relayed.
    m = hub.mark(BRAIN)
    hub.rf(BRAIN, SHORT_TURBO, -80, 5, airframe(FAR, 0x1002, "x1-local", preset=SHORT_TURBO))
    hub.wait_event(BRAIN, RX + r".*text=x1-local$", 5, since=m)
    hub.wait_event(BRAIN, r"^tx src=0d0d0d0d .*id=00001002", 5, since=m)


def test_x2_latency_and_loss_deliver_at_most_once(hub, images):
    """300 +/- 250 ms each way and 10 % loss on the link: every frame the hub
    delivered reaches the brain exactly once, every frame it lost never does."""
    start_brain(hub, images)
    pair(hub, images, lat=(50, 550), loss=0.10)
    m = hub.mark(BRAIN)
    mh = hub.mark(HEAD1)
    n = 30
    for i in range(n):
        hub.rf(HEAD1, MEDIUM_FAST, -80, 6, airframe(FAR, 0x2000 + i, f"x2-{i}"))
        time.sleep(0.15)
    time.sleep(2.0)

    lost = [e for s, d, e in hub.dropped if s == HEAD1 and e[0] == 1]  # RX_FRAMEs only
    # A frame played while the head's radio was keyed up (P3: the brain answers an
    # unknown sender through the head) is never heard -- deaf, as on the bench.
    deaf = hub.lines(HEAD1, mh, r"^rf lost")
    delivered = [l.split("text=")[1] for l in hub.lines(BRAIN, m, r"^rx from=0d0d0d0d ")]
    assert len(delivered) == len(set(delivered)), f"delivered twice: {delivered}"
    sent_through = [e for s, d, e in hub.forwarded if s == HEAD1 and e[0] == 1]
    assert len(delivered) == len(sent_through), (len(delivered), len(sent_through), len(lost))
    assert len(sent_through) + len(lost) + len(deaf) == n, (len(sent_through), len(lost), len(deaf))
    # Deafness has one cause: the head keyed up for its brain. Every frame it
    # missed must fall inside one of its own transmissions (start .. start +
    # airtime, on the head's clock, with a slot's margin for the settle and
    # the RX re-arm) -- a head deaf for any other reason, e.g. while merely
    # waiting out a contention window, would otherwise hide in this count.
    keyed = [(int(f["t"]), int(f["air"])) for f in
             (dict(kv.split("=", 1) for kv in l.split()[1:]) for l in hub.lines(HEAD1, mh, r"^tx "))]
    margin = 30
    for l in deaf:
        t = int(l.split("t=")[1])
        assert any(s - margin <= t <= s + air + margin for s, air in keyed), \
            f"head deaf at t={t} with no transmission around it: {keyed}"
    assert lost, "the loss knob never fired: the test proves nothing about loss"


def test_x3_two_heads_one_frame_one_delivery(hub, images):
    """Two head processes hear the same frame; the second copy arrives 400 ms
    later. One delivery, the first arrival's signal, both heads counted."""
    start_brain(hub, images)
    pair(hub, images, HEAD1, lat=(20, 20))
    pair(hub, images, HEAD2, lat=(400, 400))
    m = hub.mark(BRAIN)
    wire = airframe(FAR, 0x3001, "x3-both")
    hub.rf(HEAD1, MEDIUM_FAST, -95, 2, wire)
    hub.rf(HEAD2, MEDIUM_FAST, -60, 11, wire)
    line = hub.wait_event(BRAIN, RX + r".*text=x3-both$", 5, since=m)
    time.sleep(1.5)
    assert len(rx_lines(hub, m, "x3-both")) == 1
    assert "rssi=-95 snr=2" in line, line
    rows = attach_rows(hub)
    assert int(rows[HEAD1]["rx"]) >= 1 and int(rows[HEAD2]["rx"]) >= 1, rows

    # Twin: a different frame through head 2 alone is delivered.
    m = hub.mark(BRAIN)
    hub.rf(HEAD2, MEDIUM_FAST, -60, 11, airframe(FAR, 0x3002, "x3-two"))
    hub.wait_event(BRAIN, RX + r".*text=x3-two$", 5, since=m)


def test_x4_head_restart(hub, images):
    """The head process dies and comes back. The brain sees the link go down,
    the head re-introduces itself with STATUS, new frames flow, and a frame the
    brain already delivered before the restart is not delivered again."""
    start_brain(hub, images)
    pair(hub, images)
    m = hub.mark(BRAIN)
    wire = airframe(FAR, 0x4001, "x4-before")
    hub.rf(HEAD1, MEDIUM_FAST, -70, 8, wire)
    hub.wait_event(BRAIN, RX + r".*text=x4-before$", 5, since=m)

    head = hub.procs[-1]
    hub.stop(head)
    hub.tell_link(BRAIN, HEAD1, up=False)
    time.sleep(0.5)
    assert attach_rows(hub)[HEAD1]["up"] == "0"

    pair(hub, images)
    # The restarted head's first STATUS goes out at boot, before it has a
    # link, and is lost; the brain learns it is back from the next one (the
    # STATUS period, 5 s in this build, 60 s on hardware).
    deadline = time.time() + 12
    while attach_rows(hub).get(HEAD1, {}).get("up") != "1" and time.time() < deadline:
        time.sleep(0.5)
    assert attach_rows(hub)[HEAD1]["up"] == "1", "the restarted head never reached the brain"

    m = hub.mark(BRAIN)
    hub.rf(HEAD1, MEDIUM_FAST, -70, 8, wire)  # the old frame again
    hub.rf(HEAD1, MEDIUM_FAST, -70, 8, airframe(FAR, 0x4002, "x4-after"))
    hub.wait_event(BRAIN, RX + r".*text=x4-after$", 5, since=m)
    time.sleep(1.0)
    assert rx_lines(hub, m, "x4-before") == [], "a frame delivered before the restart came back"


def test_x5_a_stranger_on_the_link_is_refused(hub, images):
    """A peer the brain has no trusted link to and no allow-list entry for sends
    an RX_FRAME: refused and counted. Twin: the operator allow-lists it and the
    same envelope is delivered."""
    stranger = 0x00BADBAD
    start_brain(hub, images)
    hub.tell_link(BRAIN, stranger, up=True, auth=AUTH_NONE)
    m = hub.mark(BRAIN)
    env = env_rx_frame(MEDIUM_FAST, -40, 12, airframe(FAR, 0x5001, "x5-evil"))
    hub.env_from(stranger, BRAIN, env)
    time.sleep(1.0)
    assert rx_lines(hub, m, "x5-evil") == []
    stats = hub.ask(BRAIN, "stats", r"^stats ")
    assert "refused=0 " not in stats, stats
    assert stranger not in attach_rows(hub)

    hub.query(BRAIN, f"allow {stranger:08x}", r"^allow rc=0")
    m = hub.mark(BRAIN)
    hub.env_from(stranger, BRAIN, env_rx_frame(MEDIUM_FAST, -40, 12,
                                               airframe(FAR, 0x5002, "x5-allowed")))
    hub.wait_event(BRAIN, RX + r".*text=x5-allowed$", 5, since=m)


def head_heard(hub, node):
    line = hub.ask(node, "stats", r"^head ")
    return int(line.split("heard=")[1].split()[0])


def test_x8_a_head_listens_where_its_preset_transmits(hub, images):
    """A head on MediumFast hears a frame on the frequency a stock MediumFast
    node uses (913.125 MHz, the hub's own arithmetic), and the brain decodes
    it. Twin: the same modem on the slot a channel NAMED "LongFast" would pick
    (906.875 MHz) is not heard -- the bench bug where a head kept its old
    channel name's slot and sat deaf (SCOPE B6)."""
    start_brain(hub, images)
    pair(hub, images)
    wrong = (us_tuning(MEDIUM_FAST, "LongFast")[0], *us_tuning(MEDIUM_FAST)[1:])
    assert wrong[0] != us_tuning(MEDIUM_FAST)[0]

    before = head_heard(hub, HEAD1)
    m = hub.mark(BRAIN)
    hub.rf(HEAD1, MEDIUM_FAST, -70, 9, airframe(FAR, 0x8001, "x8-slot"))
    hub.wait_event(BRAIN, RX + r".*text=x8-slot$", 5, since=m)
    assert head_heard(hub, HEAD1) == before + 1

    before = head_heard(hub, HEAD1)
    m = hub.mark(BRAIN)
    hub.rf(HEAD1, MEDIUM_FAST, -70, 9, airframe(FAR, 0x8002, "x8-wrong"), tuning=wrong)
    time.sleep(1.5)
    assert head_heard(hub, HEAD1) == before, "heard a frame off its preset's slot"
    assert rx_lines(hub, m, "x8-wrong") == []
