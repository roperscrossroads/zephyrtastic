# Known-answer vectors from upstream firmware

Reference data harvested from a stock Meshtastic firmware tree, used to assert
that this port is byte-compatible on the wire.

## Why

Every other crypto/wire test here is a **self-loopback** — `tests/admin_pki`
forges the peer→us frame with the same nonce/CCM code that decodes it. That
structurally cannot catch a *symmetric* error: swap a nonce field on both sides
and the suite stays green while the node is mute to every stock radio on the
mesh. This repo has already been bitten by exactly that class once (the
`pki_encrypted` episode).

These vectors are the only check in the tree that can fail when we are
*consistently* wrong.

They are also the reference data needed to implement preset support, region
frequency slots, preset-aware channel hashing, and duty-cycle enforcement — so
harvesting them is a prerequisite for that work, not a separate chore.

## Trust model

A vector is worthless if it came from a reimplementation — a wrong algorithm
would just get locked in as "expected". So there are exactly two permitted
sources:

- **(a) verbatim extraction** of an upstream source region, compiled and executed
- **(b) mechanical parse** of an upstream data table

Nothing is hand-transcribed. Four algorithms are extracted verbatim:

| Symbol | Upstream location | Feeds |
|---|---|---|
| `xorHash` | `src/mesh/Channels.cpp` | channel hash (the wire header channel byte) |
| `hash` (djb2) | `src/mesh/RadioInterface.cpp` | frequency-slot selection |
| `modemPresetToParams` | `src/mesh/MeshRadio.h` | preset → SF/BW/CR |
| `initNonce` | `src/mesh/CryptoEngine.cpp` | AES-CTR / PKC nonce layout |

`xorHash` and `hash` are **different functions**. Conflating them is silent and
yields a node on the wrong frequency that still believes it is configured
correctly.

Each extracted region is recorded in `upstream.lock` with a sha256. If upstream
edits that region, re-harvesting fails loudly rather than quietly changing what
"correct" means.

## Usage

```console
# harvest + regenerate the header
python3 tools/vectors/harvest.py --upstream /path/to/meshtastic/firmware

# verify the lock still matches upstream; writes nothing (use in CI)
python3 tools/vectors/harvest.py --upstream /path/to/meshtastic/firmware --check
```

Requires a C++ compiler (`g++` or `clang++`) — the probe is compiled and run on
the host, not cross-compiled.

Output: `tests/vectors/meshtastic_vectors.h`, consumed by the `wire_vectors`
suite in `tests/protocol/src/vectors.c`.

## When upstream drift is reported

`--check` failing means the algorithm changed. Do **not** delete the lock. Read
the upstream diff first and decide whether the wire format actually moved — if it
did, this port has an interop break to fix, and that is precisely the signal this
harness exists to raise.

## Gotcha: short PSK indices

A single-byte PSK is an *index*, not a key. Index `0` disables encryption; index
`1` is the default key verbatim; index `N>1` is the default key with its last
byte `+= (N-1)`. Both trees expand before hashing, so the raw single-byte hash in
the vector table is a **component**, not a channel hash. Tests that compare
channel hashes directly must use full-length (16/32-byte) PSKs.

## Known gap this data covers

The port currently resolves an empty channel name to the hardcoded string
`"LongFast"` rather than the active preset's display name, and its modem config
is frozen at LongFast. Both are correct only for a default US LongFast mesh.
`test_empty_name_currently_defaults_to_longfast` characterises that behaviour
deliberately, so that when preset support lands the change is visible rather than
silent.
