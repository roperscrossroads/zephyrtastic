# SPDX-License-Identifier: GPL-3.0
"""The hub of the two-image attachment harness (tests/common/attach_pipe).

Each image connects to the hub's Unix socket and says HELLO with its node
number. The hub is the wire between them: it forwards envelopes with the
latency, jitter and loss a test sets, says what trust a link has, plays the RF
each radio hears, and collects the text EVENTs the images report.

Frame: [len u16 LE][crc16 u16 LE][kind u8][peer u32 LE][payload], len counting
kind..payload, CRC-16/CCITT-FALSE over the same bytes.
"""

import heapq
import os
import random
import re
import socket
import struct
import subprocess
import tempfile
import threading
import time

from meshwire import us_tuning

ENV, HELLO, LINK, RF, CMD, EVENT = range(6)
AUTH_NONE, AUTH_PHYSICAL, AUTH_ENCRYPTED = range(3)


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(kind, peer, payload=b""):
    body = struct.pack("<BI", kind, peer) + payload
    return struct.pack("<HH", len(body), crc16_ccitt_false(body)) + body


class Conn:
    def __init__(self, hub, sock):
        self.hub, self.sock = hub, sock
        self.node = None
        self.role = None
        self.lock = threading.Lock()

    def send(self, data):
        with self.lock:
            try:
                self.sock.sendall(data)
            except OSError:
                pass


class Hub:
    """One hub per test. Knobs are per direction: link(a, b) sets both."""

    def __init__(self, tmpdir, log=None):
        # sun_path holds 108 bytes: the socket lives in a short directory of
        # its own, the logs in @p tmpdir.
        self.logdir = tmpdir
        self.sockdir = tempfile.mkdtemp(prefix="apipe")
        self.path = os.path.join(self.sockdir, "s")
        self.log = log or (lambda s: None)
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(self.path)
        self.srv.listen(8)
        self.nodes = {}             # node -> Conn
        self.events = {}            # node -> [(t, line)]
        self.cond = threading.Condition()
        self.knobs = {}             # (src, dst) -> dict(lat=(lo, hi) ms, loss=p)
        self.dropped = []           # (src, dst, env) the hub lost on purpose
        self.forwarded = []         # (src, dst, env)
        self.q = []                 # (due, seq, conn, data)
        self.seq = 0
        self.rng = random.Random(1234)
        self.closed = False
        self.procs = []
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._scheduler, daemon=True).start()

    # ---- plumbing ----------------------------------------------------------
    def _accept(self):
        while not self.closed:
            try:
                s, _ = self.srv.accept()
            except OSError:
                return
            c = Conn(self, s)
            threading.Thread(target=self._reader, args=(c,), daemon=True).start()

    def _reader(self, c):
        buf = b""
        while True:
            try:
                chunk = c.sock.recv(4096)
            except OSError:
                chunk = b""
            if not chunk:
                self._gone(c)
                return
            buf += chunk
            while len(buf) >= 4:
                blen, crc = struct.unpack_from("<HH", buf)
                if len(buf) < 4 + blen:
                    break
                body, buf = buf[4:4 + blen], buf[4 + blen:]
                if crc16_ccitt_false(body) != crc:
                    self.log(f"hub: CRC error from {c.node}")
                    continue
                kind, peer = struct.unpack_from("<BI", body)
                self._on_frame(c, kind, peer, body[5:])

    def _gone(self, c):
        with self.cond:
            if c.node is not None and self.nodes.get(c.node) is c:
                del self.nodes[c.node]
            self.cond.notify_all()

    def _on_frame(self, c, kind, peer, payload):
        if kind == HELLO:
            c.node, c.role = peer, payload[0]
            with self.cond:
                self.nodes[peer] = c
                self.cond.notify_all()
        elif kind == EVENT:
            line = payload.decode(errors="replace")
            self.log(f"[{c.node:08x}] {line}")
            with self.cond:
                self.events.setdefault(c.node, []).append((time.monotonic(), line))
                self.cond.notify_all()
        elif kind == ENV:
            self._route(c.node, peer, payload)

    def _route(self, src, dst, env):
        k = self.knobs.get((src, dst), {})
        if self.rng.random() < k.get("loss", 0.0):
            self.dropped.append((src, dst, env))
            return
        lo, hi = k.get("lat", (0, 0))
        due = time.monotonic() + self.rng.uniform(lo, hi) / 1000.0
        self.forwarded.append((src, dst, env))
        self._schedule(due, dst, frame(ENV, src, env))

    def _schedule(self, due, dst, data):
        with self.cond:
            self.seq += 1
            heapq.heappush(self.q, (due, self.seq, dst, data))
            self.cond.notify_all()

    def _scheduler(self):
        while not self.closed:
            with self.cond:
                while not self.q or self.q[0][0] > time.monotonic():
                    timeout = (self.q[0][0] - time.monotonic()) if self.q else 0.1
                    self.cond.wait(max(0.0, min(timeout, 0.1)))
                    if self.closed:
                        return
                _, _, dst, data = heapq.heappop(self.q)
                c = self.nodes.get(dst)
            if c is not None:
                c.send(data)

    # ---- the test's controls -----------------------------------------------
    def start(self, exe, node, extra_env=None, name=None):
        env = dict(os.environ, ATTACH_PIPE_SOCK=self.path, ATTACH_PIPE_NODE=f"{node:08x}")
        env.update(extra_env or {})
        # Only a "ready" from THIS launch counts: a restarted image's earlier
        # one is still in the event list.
        since = self.mark(node)
        logf = open(os.path.join(self.logdir, f"{name or node}.log"), "a")
        p = subprocess.Popen([exe], env=env, stdout=logf, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
        self.procs.append(p)
        self.wait_event(node, r"^ready ", 30, since=since)
        return p

    def stop(self, p):
        p.kill()
        p.wait()

    def close(self):
        self.closed = True
        for p in self.procs:
            if p.poll() is None:
                p.kill()
                p.wait()
        self.srv.close()
        try:
            os.unlink(self.path)
            os.rmdir(self.sockdir)
        except OSError:
            pass
        with self.cond:
            self.cond.notify_all()

    def link(self, a, b, up=True, auth=AUTH_ENCRYPTED, lat=(0, 0), loss=0.0):
        """A link between a and b: tell both ends, set both directions' knobs."""
        for x, y in ((a, b), (b, a)):
            self.knobs[(x, y)] = {"lat": lat, "loss": loss}
            self.tell_link(x, y, up, auth)

    def tell_link(self, node, peer, up=True, auth=AUTH_ENCRYPTED, rtt_ms=0):
        c = self.nodes.get(node)
        if c is None:
            raise KeyError(f"{node:08x} is not connected to the hub")
        c.send(frame(LINK, peer, struct.pack("<BBBH", int(up), auth, 1, rtt_ms)))

    def rf(self, node, preset, rssi, snr, wire, tuning=None):
        """@p node's radio hears @p wire, sent on @p tuning (freq_hz, sf,
        bw_khz): by default where a stock node on @p preset transmits, worked
        out here and not by the firmware. A radio tuned anywhere else does not
        hear it."""
        freq, sf, bw = tuning or us_tuning(preset)
        hdr = struct.pack("<BhbIBH", preset, rssi, snr, freq, sf, bw)
        self.nodes[node].send(frame(RF, 0, hdr + wire))

    def env_from(self, fake_peer, node, env):
        """An envelope to @p node claiming to come from @p fake_peer (no hub
        knobs): what a stranger on the link could send."""
        self.nodes[node].send(frame(ENV, fake_peer, env))

    def cmd(self, node, text):
        self.nodes[node].send(frame(CMD, 0, text.encode()))

    def mark(self, node):
        with self.cond:
            return len(self.events.get(node, []))

    def lines(self, node, since=0, pattern=None):
        with self.cond:
            ls = [l for _, l in self.events.get(node, [])[since:]]
        return [l for l in ls if pattern is None or re.search(pattern, l)]

    def wait_event(self, node, pattern, timeout, since=0):
        deadline = time.monotonic() + timeout
        rx = re.compile(pattern)
        with self.cond:
            while True:
                for _, l in self.events.get(node, [])[since:]:
                    if rx.search(l):
                        return l
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"{node:08x}: no event /{pattern}/ in {timeout}s")
                self.cond.wait(left)

    def query(self, node, cmd, end_pattern, timeout=5):
        m = self.mark(node)
        self.cmd(node, cmd)
        self.wait_event(node, end_pattern, timeout, since=m)
        return self.lines(node, since=m)

    def ask(self, node, cmd, pattern, timeout=5):
        """Send @p cmd and return the one line that answers it (matching
        @p pattern): other events -- rx, tx, rf lost -- may land in between."""
        m = self.mark(node)
        self.cmd(node, cmd)
        return self.wait_event(node, pattern, timeout, since=m)

    def wait_node(self, node, timeout=10):
        deadline = time.monotonic() + timeout
        with self.cond:
            while node not in self.nodes:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"{node:08x} never connected")
                self.cond.wait(left)
