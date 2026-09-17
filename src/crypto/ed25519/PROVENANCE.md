# Vendored: orlp/ed25519 (Ed25519 verify only)

| | |
|---|---|
| Upstream | `https://github.com/orlp/ed25519` |
| Commit | `b1f19fab4aebe607805620d25a5e42566ce46a0e` (2022-10-03) |
| Licence | **zlib** — `license.txt`, retained verbatim |
| Modified? | **No.** Every file listed below is byte-identical to that commit. |

This port is GPL-3. zlib is permissive and GPL-3 absorbs it; the notice stays, and nothing here
claims to be original work of this project.

## What was taken, and what was not

Vendored verbatim: `fe.c/.h`, `ge.c/.h`, `sc.c/.h`, `verify.c`, `precomp_data.h`, `fixedint.h`,
`ed25519.h`, `license.txt`.

**Not vendored:** `sign.c`, `keypair.c`, `seed.c`, `add_scalar.c`, `key_exchange.c` — signing and key
generation, which this port does not do (verify-only; `agents-ooma.5`). `ed25519.h` still *declares*
them; the declarations are harmless and keeping the header unmodified is worth more than trimming it.

**Substituted:** `sha512.c/.h` are **not** vendored. `verify.c` includes `"sha512.h"` and uses the
streaming init/update/final API, which our own `sha512.h` + `sha512_psa.c` provide on top of Zephyr's
PSA crypto.

⚠️ This is **not free**, and an earlier version of this note wrongly implied it was: PKC uses SHA-**256**,
so `PSA_WANT_ALG_SHA_512` pulls in mbedTLS's SHA-512 that nothing else needed — **3.3 KB** measured on
the XIAO. The reason to do it this way is that vendoring orlp's `sha512.c` would cost about the same
while adding a second hash implementation to audit and update; the platform's is one already in the
tree. Measured 2026-09-17.

## Size

`ge.c` references `base[32][8]` from `precomp_data.h` (~30 KB of tables) **only** from
`ge_scalarmult_base()`, which is a signing primitive. Verify reaches only `Bi[8]` (960 B). Zephyr builds
with `-ffunction-sections -fdata-sections --gc-sections`, so the unused function and its table are
discarded at link. That is why these files are vendored unmodified rather than edited down — check
with `nm zephyr.elf | grep -w base` if the image ever looks too big.

**Measured cost** (XIAO nRF52840, class 1, 2026-09-17): **+19,320 B** of flash total —
ed25519 arithmetic 12.9 KB, SHA-512 3.3 KB, our wrapper and RX gate 0.9 KB. GC verified: neither
`base` nor `ge_scalarmult_base` is present in the linked image.

## Why this implementation

Decision and evidence: `SIGNING-AND-IDENTITY-DESIGN.md` §6.1–§6.2 (tooling repo). In short: upstream
Meshtastic's own XEdDSA library has no LICENSE file and its *signing* half carries no licence at all,
while the verify half is MIT; this replaces it with one permissively-licensed, plain-C dependency.
Agreement with the reference is not assumed — upstream's signer was compiled on the host, and the
signatures it produced verify here (see §6.2, and the vectors suite).
