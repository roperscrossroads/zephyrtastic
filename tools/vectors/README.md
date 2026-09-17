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

## XEdDSA vectors (`harvest_xeddsa.py`)

A second harvester, same trust model, for the signatures this port must VERIFY. It needs two
sources, because upstream's signer is not in the firmware tree -- it is a PlatformIO
dependency, pinned per variant:

```console
# the pinned revision comes from the firmware tree itself:
#   firmware/variants/nrf52840/nrf52.ini -> github.com/meshtastic/Crypto archive <sha>.zip
curl -sSL -o crypto.zip https://github.com/meshtastic/Crypto/archive/<sha>.zip
unzip -q crypto.zip            # sources sit at the archive root

python3 tools/vectors/harvest_xeddsa.py \
    --upstream   /path/to/meshtastic/firmware \
    --crypto-lib /path/to/Crypto-<sha>

# CI / drift check (writes nothing)
python3 tools/vectors/harvest_xeddsa.py --upstream ... --crypto-lib ... --check
```

Output: `tests/vectors/meshtastic_xeddsa_vectors.h`, consumed by the `xeddsa` suite. Lock:
`tools/vectors/xeddsa.lock` (region sha256s **and** a sha256 per library file used).

Two things worth knowing before regenerating:

- **The library is compiled, never vendored.** `XEdDSA.cpp` carries no licence (the repo has
  no LICENSE file; its other files are MIT), so only the resulting DATA lands in this repo.
  The verify path we ship is orlp/ed25519 instead -- `src/crypto/ed25519/PROVENANCE.md`.
- **Signing is made deterministic.** XEdDSA mixes `signature[0..31]` into the nonce as the
  spec's random Z; upstream seeds it from the hardware RNG, the probe from a fixed pattern.
  A fixed Z changes nothing about verification and makes the vectors reproducible.

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
