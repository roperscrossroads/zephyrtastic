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
PSA crypto — already linked by every build for PKC. One SHA-512 in the image, and it is the
platform's.

## Size

`ge.c` references `base[32][8]` from `precomp_data.h` (~30 KB of tables) **only** from
`ge_scalarmult_base()`, which is a signing primitive. Verify reaches only `Bi[8]`. Zephyr builds with
`-ffunction-sections -fdata-sections --gc-sections`, so the unused function and its table are
discarded at link. That is why these files are vendored unmodified rather than edited down — check
with `nm zephyr.elf | grep ge_scalarmult_base` if the image ever looks too big.

## Why this implementation

Decision and evidence: `SIGNING-AND-IDENTITY-DESIGN.md` §6.1–§6.2 (tooling repo). In short: upstream
Meshtastic's own XEdDSA library has no LICENSE file and its *signing* half carries no licence at all,
while the verify half is MIT; this replaces it with one permissively-licensed, plain-C dependency.
Agreement with the reference is not assumed — upstream's signer was compiled on the host, and the
signatures it produced verify here (see §6.2, and the vectors suite).
