# SPDX-License-Identifier: GPL-3.0
"""Layer 3 of the link test plan (ATTACHMENT-REVIEW part 2): a real hub
image and real keyless worker images, joined through the link pipe.

X1-X5 are the pre-P3 cases. Every "does not" assertion has its positive twin in
the same case (ATTACHMENT-SCOPE §5).
"""

import time

from conftest import HUB, WORKER1, WORKER2, start_hub, start_worker
from switchboard import AUTH_ENCRYPTED, AUTH_NONE
from meshwire import MEDIUM_FAST, SHORT_TURBO, airframe, env_rx_frame, us_tuning

FAR = 0x0D0D0D0D
RX = r"^rx from=0d0d0d0d "


def rx_lines(switchboard, since, text):
    return switchboard.lines(HUB, since, rf"^rx from=0d0d0d0d .*text={text}$")


def tx_lines(switchboard, since, packet_id):
    return switchboard.lines(HUB, since, rf"^tx src=0d0d0d0d .*id={packet_id:08x}")


def link_rows(switchboard):
    rows = {}
    for line in switchboard.query(HUB, "link", r"^link end"):
        if line.startswith("link id="):
            kv = dict(f.split("=", 1) for f in line.split()[1:])
            rows[int(kv["node"], 16)] = kv
    return rows


def pair(switchboard, images, node=WORKER1, preset=MEDIUM_FAST, **knobs):
    start_worker(switchboard, images, node, preset)
    switchboard.link(HUB, node, **knobs)
    # The worker introduces itself with STATUS once it has a link.
    switchboard.wait_event(node, r"^ready ", 5)


def test_x1_worker_on_another_preset_decodes_on_the_hub(switchboard, images):
    """The 05:55Z bench proof, on native_sim: a MediumFast frame heard by a worker
    is delivered on the ShortTurbo hub from the origin's id, with the worker's
    signal, and is not relayed on the hub's radio. Twin: the same kind of
    frame heard by the hub's OWN radio on ShortTurbo is relayed."""
    start_hub(switchboard, images)
    pair(switchboard, images)
    m = switchboard.mark(HUB)

    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 10, airframe(FAR, 0x1001, "x1-hello", preset=MEDIUM_FAST))
    line = switchboard.wait_event(HUB, RX + r".*text=x1-hello$", 5, since=m)
    assert "rssi=-70 snr=10" in line, line
    time.sleep(2.0)
    assert tx_lines(switchboard, m, 0x1001) == [], "a worker's frame left on the hub's own radio"
    row = link_rows(switchboard)[WORKER1]
    assert row["preset"] == str(MEDIUM_FAST) and int(row["rx"]) >= 1, row

    # Twin: the hub's own radio, its own preset -- relayed.
    m = switchboard.mark(HUB)
    switchboard.rf(HUB, SHORT_TURBO, -80, 5, airframe(FAR, 0x1002, "x1-local", preset=SHORT_TURBO))
    switchboard.wait_event(HUB, RX + r".*text=x1-local$", 5, since=m)
    switchboard.wait_event(HUB, r"^tx src=0d0d0d0d .*id=00001002", 5, since=m)


def test_x2_latency_and_loss_deliver_at_most_once(switchboard, images):
    """300 +/- 250 ms each way and 10 % loss on the link: every frame the switchboard
    delivered reaches the hub exactly once, every frame it lost never does."""
    start_hub(switchboard, images)
    pair(switchboard, images, lat=(50, 550), loss=0.10)
    m = switchboard.mark(HUB)
    mh = switchboard.mark(WORKER1)
    n = 30
    for i in range(n):
        switchboard.rf(WORKER1, MEDIUM_FAST, -80, 6, airframe(FAR, 0x2000 + i, f"x2-{i}"))
        time.sleep(0.15)
    time.sleep(2.0)

    lost = [e for s, d, e in switchboard.dropped if s == WORKER1 and e[0] == 1]  # RX_FRAMEs only
    # A frame played while the worker's radio was keyed up (P3: the hub answers an
    # unknown sender through the worker) is never heard -- deaf, as on the bench.
    deaf = switchboard.lines(WORKER1, mh, r"^rf lost")
    delivered = [l.split("text=")[1] for l in switchboard.lines(HUB, m, r"^rx from=0d0d0d0d ")]
    assert len(delivered) == len(set(delivered)), f"delivered twice: {delivered}"
    sent_through = [e for s, d, e in switchboard.forwarded if s == WORKER1 and e[0] == 1]
    assert len(delivered) == len(sent_through), (len(delivered), len(sent_through), len(lost))
    assert len(sent_through) + len(lost) + len(deaf) == n, (len(sent_through), len(lost), len(deaf))
    # Deafness has one cause: the worker keyed up for its hub. Every frame it
    # missed must fall inside one of its own transmissions (start .. start +
    # airtime, on the worker's clock, with a slot's margin for the settle and
    # the RX re-arm) -- a worker deaf for any other reason, e.g. while merely
    # waiting out a contention window, would otherwise hide in this count.
    keyed = [(int(f["t"]), int(f["air"])) for f in
             (dict(kv.split("=", 1) for kv in l.split()[1:]) for l in switchboard.lines(WORKER1, mh, r"^tx "))]
    margin = 30
    for l in deaf:
        t = int(l.split("t=")[1])
        assert any(s - margin <= t <= s + air + margin for s, air in keyed), \
            f"worker deaf at t={t} with no transmission around it: {keyed}"
    assert lost, "the loss knob never fired: the test proves nothing about loss"


def test_x3_two_workers_one_frame_one_delivery(switchboard, images):
    """Two worker processes hear the same frame; the second copy arrives 400 ms
    later. One delivery, the first arrival's signal, both workers counted."""
    start_hub(switchboard, images)
    pair(switchboard, images, WORKER1, lat=(20, 20))
    pair(switchboard, images, WORKER2, lat=(400, 400))
    m = switchboard.mark(HUB)
    wire = airframe(FAR, 0x3001, "x3-both")
    switchboard.rf(WORKER1, MEDIUM_FAST, -95, 2, wire)
    switchboard.rf(WORKER2, MEDIUM_FAST, -60, 11, wire)
    line = switchboard.wait_event(HUB, RX + r".*text=x3-both$", 5, since=m)
    time.sleep(1.5)
    assert len(rx_lines(switchboard, m, "x3-both")) == 1
    assert "rssi=-95 snr=2" in line, line
    rows = link_rows(switchboard)
    assert int(rows[WORKER1]["rx"]) >= 1 and int(rows[WORKER2]["rx"]) >= 1, rows

    # Twin: a different frame through worker 2 alone is delivered.
    m = switchboard.mark(HUB)
    switchboard.rf(WORKER2, MEDIUM_FAST, -60, 11, airframe(FAR, 0x3002, "x3-two"))
    switchboard.wait_event(HUB, RX + r".*text=x3-two$", 5, since=m)


def test_x4_worker_restart(switchboard, images):
    """The worker process dies and comes back. The hub sees the link go down,
    the worker re-introduces itself with STATUS, new frames flow, and a frame the
    hub already delivered before the restart is not delivered again."""
    start_hub(switchboard, images)
    pair(switchboard, images)
    m = switchboard.mark(HUB)
    wire = airframe(FAR, 0x4001, "x4-before")
    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 8, wire)
    switchboard.wait_event(HUB, RX + r".*text=x4-before$", 5, since=m)

    worker = switchboard.procs[-1]
    switchboard.stop(worker)
    switchboard.tell_link(HUB, WORKER1, up=False)
    time.sleep(0.5)
    assert link_rows(switchboard)[WORKER1]["up"] == "0"

    pair(switchboard, images)
    # The restarted worker introduces itself with STATUS when its link comes up,
    # not at the next STATUS period (5 s here, 60 s on hardware).
    deadline = time.time() + 2
    while link_rows(switchboard).get(WORKER1, {}).get("up") != "1" and time.time() < deadline:
        time.sleep(0.2)
    assert link_rows(switchboard)[WORKER1]["up"] == "1", "the restarted worker never reached the hub"

    m = switchboard.mark(HUB)
    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 8, wire)  # the old frame again
    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 8, airframe(FAR, 0x4002, "x4-after"))
    switchboard.wait_event(HUB, RX + r".*text=x4-after$", 5, since=m)
    time.sleep(1.0)
    assert rx_lines(switchboard, m, "x4-before") == [], "a frame delivered before the restart came back"


def test_x5_a_stranger_on_the_link_is_refused(switchboard, images):
    """A peer the hub has no trusted link to and no allow-list entry for sends
    an RX_FRAME: refused and counted. Twin: the operator allow-lists it and the
    same envelope is delivered."""
    stranger = 0x00BADBAD
    start_hub(switchboard, images)
    switchboard.tell_link(HUB, stranger, up=True, auth=AUTH_NONE)
    m = switchboard.mark(HUB)
    env = env_rx_frame(MEDIUM_FAST, -40, 12, airframe(FAR, 0x5001, "x5-evil"))
    switchboard.env_from(stranger, HUB, env)
    time.sleep(1.0)
    assert rx_lines(switchboard, m, "x5-evil") == []
    stats = switchboard.ask(HUB, "stats", r"^stats ")
    assert "refused=0 " not in stats, stats
    assert stranger not in link_rows(switchboard)

    switchboard.query(HUB, f"allow {stranger:08x}", r"^allow rc=0")
    m = switchboard.mark(HUB)
    switchboard.env_from(stranger, HUB, env_rx_frame(MEDIUM_FAST, -40, 12,
                                               airframe(FAR, 0x5002, "x5-allowed")))
    switchboard.wait_event(HUB, RX + r".*text=x5-allowed$", 5, since=m)


def worker_heard(switchboard, node):
    line = switchboard.ask(node, "stats", r"^worker ")
    return int(line.split("heard=")[1].split()[0])


def test_x8_a_worker_listens_where_its_preset_transmits(switchboard, images):
    """A worker on MediumFast hears a frame on the frequency a stock MediumFast
    node uses (913.125 MHz, the switchboard's own arithmetic), and the hub decodes
    it. Twin: the same modem on the slot a channel NAMED "LongFast" would pick
    (906.875 MHz) is not heard -- the bench bug where a worker kept its old
    channel name's slot and sat deaf (SCOPE B6)."""
    start_hub(switchboard, images)
    pair(switchboard, images)
    wrong = (us_tuning(MEDIUM_FAST, "LongFast")[0], *us_tuning(MEDIUM_FAST)[1:])
    assert wrong[0] != us_tuning(MEDIUM_FAST)[0]

    before = worker_heard(switchboard, WORKER1)
    m = switchboard.mark(HUB)
    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 9, airframe(FAR, 0x8001, "x8-slot"))
    switchboard.wait_event(HUB, RX + r".*text=x8-slot$", 5, since=m)
    assert worker_heard(switchboard, WORKER1) == before + 1

    before = worker_heard(switchboard, WORKER1)
    m = switchboard.mark(HUB)
    switchboard.rf(WORKER1, MEDIUM_FAST, -70, 9, airframe(FAR, 0x8002, "x8-wrong"), tuning=wrong)
    time.sleep(1.5)
    assert worker_heard(switchboard, WORKER1) == before, "heard a frame off its preset's slot"
    assert rx_lines(switchboard, m, "x8-wrong") == []
