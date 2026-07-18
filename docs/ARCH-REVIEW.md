# Architecture review — concurrency, queueing, and drop behavior

Focus: how packets flow through threads/queues, and where they can be silently
dropped, delayed, or mis-ordered. Motivated by "sort of working" on-air behavior
(peers visible, some messages missing). Companion to the
[`meshtastic sched`](../src/meshtastic_sched.c) scheduler-policy module, which is
where these findings get fixed *and* made tunable at runtime.

## The model

```
        ┌─────────────── RX ───────────────┐         ┌──────────── TX ────────────┐
LoRa │ mt_rx_cb (driver sysq): memcpy → mt_rx_msgq  │  outbound priority queue ─► "meshtastic_tx"
 air │        depth 4, K_NO_WAIT, drop-NEWEST        │  (was FIFO, depth 8)  thread
     │                    │                          │        ▲          send_wire_now:
     │        "meshtastic" thread (prio 7)           │        │          sem, stop RX,
     │        router_process_lora_rx:                │        │          LBT+retries, TX,
     │        dedup(16) → decrypt → dispatch ────────┼── send (tier-classified)
     │        → deliver → phoneapi fan-out → mqtt     │
     └──────────────────┬───────────────────────────┘
                        ▼
        per-transport FromRadio ring (BLE/serial/tcp)
           depth 8, selective evict  ──► phone
           (protect text/routing/admin; drop bg first)
```

Every queue is a fixed-policy FIFO with no notion of packet importance — the
system drops by *position* (newest/oldest), never by *priority*.

## Findings (ranked by likely impact on "messages not all arriving")

1. **Inverted TX drop policy.** `meshtastic_send_packet(…, K_NO_WAIT)` drops when
   the outbound queue is full. The droppable (`K_NO_WAIT`) callers were flood
   **relays** and **`want_response` replies**; the never-drop (`K_FOREVER`)
   callers were the node's own periodic **telemetry/position/nodeinfo**. So under
   load the node preferentially dropped forwarding-others' traffic while
   protecting its own chatter. **Backwards.** → fixed by the priority egress.
2. **FIFO egress, no priority.** An ACK sat behind up to 7 telemetry frames
   (hundreds of ms of SF11 airtime each); `want_ack` senders time out. → fixed.
3. **Phone queue depth-8, drop-oldest** (`phoneapi.c`). A telemetry/position
   burst could evict a received **text message** before the phone reads it. →
   fixed by selective eviction (`sched` `phone.evict`): the `protect` policy
   evicts the oldest *droppable* frame (position/telemetry/nodeinfo/queueStatus)
   first, so text/routing/admin/handshake frames survive a burst; `legacy` preset
   restores strict drop-oldest.
4. **Airtime accounted, never enforced.** Nothing throttled self-TX; every TX
   blinds the receiver during the LBT+send+re-arm window. → fixed by the airtime
   gate (`sched` `airtime.max`): when channel utilization is at/above the
   threshold, the node suppresses its *own* background broadcasts
   (position/telemetry/nodeinfo, broadcast + fire-and-forget). Requested replies
   and every higher tier are never gated. Default 40% (off in `legacy`).
5. **Dedup cache 16, linear, no TTL** — small for a multi-node flood mesh;
   forgotten packets got needlessly rebroadcast. → fixed: cache enlarged to 32
   entries and a runtime TTL added (`sched` `dedup.ttl`, default 300 s). A
   matching (src,id) older than the TTL is treated as fresh, so a legitimately
   re-sent packet is no longer swallowed; `legacy` preset restores the
   never-expiring ring.
6. **RX hand-off depth 4** — fine for LoRa's arrival rate unless the processing
   thread (decrypt + module dispatch + phone fan-out + MQTT, all inline) stalls.
   → open.

## The fix vehicle: `meshtastic sched`

Rather than hardcode fixes, they live behind a runtime policy surface (Linux
I/O-scheduler analogy): named presets, live knobs, and counters, all RAM-only so
a reboot restores compiled defaults. Shell: `meshtastic sched [show|policy
<name>|set <key> <val>|stats|defaults]`. Changing a knob, applying a preset, or
restoring defaults **resets the live stats** — so every drop counter
(`tx_drop[tier]`, `phone_drop`, `tx_airtime_drop`, `dedup_expired`) measures the
policy currently in force, from a clean slate.

- **v1 (done, pending on-air test):** priority egress (findings #1, #2) —
  tiers ACK > admin/replies > text/relay > background; runtime `tx.order` /
  `tx.overflow` / `tx.depth`. `legacy` preset reproduces the old FIFO/drop-newest
  behavior for A/B comparison. Verified by the `protocol_stack` + `sched` ztest
  suites, including gated-mock ordering + drop-under-contention tests
  (`test_egress_priority_ordering`, `test_egress_overflow_drop_lowest`).
- **v2 (done, pending on-air test):** selective phone-queue eviction (finding #3)
  — `phone.evict protect|drop-oldest`, with `phone_drop` / `phone_drop_protected`
  counters in `sched stats`. Verified by the `phonequeue` ztest suite
  (`tests/protocol/src/phonequeue.c`).
- **v3 (done, pending on-air test):** airtime-gated background self-TX (#4,
  `airtime.max`) + enlarged dedup cache with runtime TTL (#5, `dedup.ttl`), plus
  `tx_airtime_drop` / `dedup_expired` counters and stats-reset-on-mode-change.
  Verified by the extended `sched` suite and a `dedup.ttl` expiry behavior test
  (`test_dedup_ttl_expiry_allows_resend`).

### Concurrency: snapshot discipline

The live policy is mutated by the shell thread while the TX worker, enqueue
callers, the RX/router thread, the phone-enqueue path and `send_packet` read it.
Every field is an aligned scalar, so a single field read can never tear into a
garbage value — the only hazards are a multi-field preset write seen
half-applied, and one decision reading several fields that then disagree. Both
are closed by a **snapshot rule** (documented on the API in `meshtastic_sched.h`):

- Writers (`set` / `apply_preset` / `defaults`) mutate under a leaf config mutex.
- A consumer reading **two or more** fields for one decision calls
  `meshtastic_sched_snapshot()` once and decides from the local copy — the
  outbound enqueue snapshots per loop iteration; the TX worker snapshots once per
  send before taking the queue lock.
- A consumer reading a **single** scalar (airtime gate, dedup TTL, phone evict)
  captures it once into a local; the atomic load needs no lock.

So a `sched policy …` applied mid-flight can never make an in-progress send
observe a blend of two policies. Snapshot coherence is covered by
`test_snapshot_coherent`.
