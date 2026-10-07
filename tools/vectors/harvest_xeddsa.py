#!/usr/bin/env python3
"""Known-answer vectors for XEdDSA VERIFY, harvested from upstream Meshtastic.

Same trust model as harvest.py (read its README): a vector is worthless if it came from a
reimplementation, so every byte here is produced by COMPILING AND RUNNING upstream's own
code. Nothing is hand-transcribed.

What makes this one different from harvest.py is that upstream's signer is NOT in the
firmware tree. It is a PlatformIO dependency, `github.com/meshtastic/Crypto`, pinned per
variant (see firmware/variants/nrf52840/nrf52.ini). So this needs two sources:

    python3 tools/vectors/harvest_xeddsa.py \\
        --upstream   /path/to/meshtastic/firmware \\
        --crypto-lib /path/to/meshtastic/Crypto        # the pinned checkout/extract

From the firmware tree, two regions are extracted verbatim -- both GPL-3, same licence as
this port:
    buildSigningBuffer  the exact bytes that get signed -- since 2.8.1 (#11422) a 34-byte
                        header (version | from | id | to | portnum | request_id | reply_id |
                        emoji | bitfield | flags) then the payload; before it, 12 bytes
                        (from | id | portnum). No negotiation between the two: a 2.8.1
                        verifier drops a 2.8.0 signature under every policy.
    putLE32             the integer encoding that header pins explicitly
    signing constants   the version byte, header length and flag bits (CryptoEngine.h)
    curve_to_ed_pub     the X25519 -> Ed25519 map, for a second opinion on our own

From the library, the signer itself is compiled and run. ⚠️ Its XEdDSA.cpp carries NO
licence (the repo has no LICENSE file; its other files are MIT) -- which is precisely why
it is COMPILED HERE and never vendored: what lands in this repo is the resulting DATA, in
tests/vectors/meshtastic_xeddsa_vectors.h, plus a sha256 of every file used.

Signing is deterministic here: XEdDSA mixes signature[0..31] into the nonce as the spec's
random Z (upstream seeds it from the hardware RNG), so the probe seeds it with a fixed
pattern. The vectors are therefore reproducible, and a signature still verifies exactly as
a hedged one does.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from harvest import REPO, Region, grab, upstream_revision

HERE = Path(__file__).resolve().parent
LOCK = HERE / "xeddsa.lock"
HEADER_OUT = REPO / "tests" / "vectors" / "meshtastic_xeddsa_vectors.h"

# Verbatim regions from the firmware tree.
TARGETS = [
    ("put_le32", "src/mesh/CryptoEngine.cpp", "static void putLE32("),
    ("build_signing_buffer", "src/mesh/CryptoEngine.cpp", "static size_t buildSigningBuffer("),
    ("curve_to_ed_pub", "src/mesh/CryptoEngine.cpp", "void CryptoEngine::curve_to_ed_pub("),
]

# The constants buildSigningBuffer refers to, taken line by line from the header (they are
# not braced, so grab() cannot take them). Order is the order they appear in the probe.
CONSTANT_NAMES = ["XEDDSA_SIGNING_VERSION", "XEDDSA_SIGNED_HEADER_LEN",
                  "XEDDSA_SIGNED_FLAG_WANT_RESPONSE", "XEDDSA_SIGNED_FLAG_HAS_BITFIELD"]


def grab_constants(upstream: Path) -> Region:
    """The signing constants from src/mesh/CryptoEngine.h, verbatim, one line each."""
    rel = "src/mesh/CryptoEngine.h"
    lines = (upstream / rel).read_text(errors="replace").splitlines()
    picked, first = [], None
    for name in CONSTANT_NAMES:
        for n, ln in enumerate(lines, 1):
            if re.match(rf"\s*(#define|static constexpr \w+)\s+{name}\b", ln):
                picked.append(ln)
                first = first or n
                break
        else:
            sys.exit(f"error: {rel} has no definition of {name}")
    return Region(name="signing_constants", relpath=rel, anchor=CONSTANT_NAMES[0],
                  text="\n".join(picked) + "\n", line=first)

# Library translation units the probe compiles. XEdDSA.cpp is the unlicensed one.
LIB_SOURCES = ["XEdDSA.cpp", "Ed25519.cpp", "Curve25519.cpp", "BigNumberUtil.cpp",
               "SHA512.cpp", "Crypto.cpp", "Hash.cpp"]

BROADCAST = 0xFFFFFFFF
DATA_PAYLOAD_LEN = 233  # meshtastic_Constants_DATA_PAYLOAD_LEN


class Case:
    """One signed packet: the MeshPacket header part (from, id, to) and the Data envelope."""

    def __init__(self, label, seed, frm, pid, to, port, payload, *, request_id=0, reply_id=0,
                 emoji=0, bitfield=None, want_response=False):
        self.label, self.seed, self.frm, self.pid, self.to, self.port = label, seed, frm, pid, to, port
        self.payload = payload
        self.request_id, self.reply_id, self.emoji = request_id, reply_id, emoji
        self.has_bitfield = bitfield is not None
        self.bitfield = bitfield or 0
        self.want_response = want_response


# Chosen to cover: an ordinary broadcast, an empty payload (headers only), a payload at the
# Data schema's ceiling, a key whose clamped form differs in both clamped bytes, and -- the
# fields #11422 added to the envelope -- a unicast reply with reply_id and emoji (a tapback),
# want_response, and a bitfield that is present (OK_TO_MQTT set) against one that is absent,
# which the flags byte must tell apart.
CASES = [
    Case("position", 0x11, 0x075c78e8, 0x2a2b2c2d, BROADCAST, 3, bytes(range(0, 24))),
    Case("nodeinfo", 0x42, 0x121f8bac, 0x00000001, BROADCAST, 4, bytes(range(200, 256)),
         want_response=True, bitfield=0x03),
    Case("empty_payload", 0x7f, 0x03c1adc6, 0xfffffffe, BROADCAST, 67, b""),
    Case("max_payload", 0xa5, 0x051c2856, 0x12345678, BROADCAST, 1,
         bytes((i * 7) & 0xFF for i in range(DATA_PAYLOAD_LEN))),
    Case("tapback", 0x3c, 0x7cc1c864, 0x6a5b4c3d, 0x1f6b56cb, 1, "\U0001F44D".encode(),
         reply_id=0x2a2b2c2d, emoji=1, bitfield=0x01),
    # A signed unicast, as a licensed station sends. Addressed to tests/xeddsa_rx's own node
    # id, so that suite can receive it as a DM to us without re-addressing it (which is now
    # exactly what the signature refuses).
    Case("licensed_request", 0x58, 0x96b698d4, 0x00000100, 0x5A5A0001, 6, b"\x01",
         request_id=0x00000099, want_response=True),
]

# The first-contact NodeInfo bootstrap needs a case that cannot be written down in advance:
# the payload is a User carrying the signer's own public key, and `from` must be
# crc32(that key) -- neither is known until the probe has derived the key. So the probe runs
# TWICE: pass 1 produces the key for this seed, python then builds the User payload and the
# id from it, and pass 2 signs that packet. Same trust model -- upstream signs, we only
# arrange the inputs.
BOOTSTRAP_SEED = 0xC3
BOOTSTRAP_PORT = 4  # NODEINFO_APP


def user_payload(pub: bytes) -> bytes:
    """A minimal meshtastic.User carrying only public_key (field 8, bytes)."""
    return bytes([(8 << 3) | 2, len(pub)]) + pub


def crc32_ieee(data: bytes) -> int:
    import zlib
    return zlib.crc32(data) & 0xFFFFFFFF

PROBE = r"""
#include <cstdio>
#include <cstring>
#include <cstdint>
#include "XEdDSA.h"
#include "Curve25519.h"
#include "RNG.h"

/* The probe never generates a key, so the library's RNG is never called; these exist only
 * to satisfy the linker without dragging in an Arduino entropy source. */
RNGClass CryptRNG;
RNGClass::RNGClass() {}
RNGClass::~RNGClass() {}
void RNGClass::rand(uint8_t *data, size_t len) { memset(data, 0, len); }

/* The slice of nanopb's meshtastic_Data that buildSigningBuffer reads, with the generated
 * field types (mesh.pb.h: bitfield is uint8_t, the rest uint32_t / bool). */
typedef struct { uint16_t size; uint8_t bytes[233]; } meshtastic_Data_payload_t;
typedef struct {
    int portnum;
    meshtastic_Data_payload_t payload;
    bool want_response;
    uint32_t dest, source, request_id, reply_id, emoji;
    bool has_bitfield;
    uint8_t bitfield;
} meshtastic_Data;

/* --- verbatim from upstream: the signing constants (CryptoEngine.h) --- */
%(signing_constants)s

/* --- verbatim from upstream: the integer encoding --- */
%(put_le32)s

/* --- verbatim from upstream: the bytes that get signed --- */
%(build_signing_buffer)s

/* --- verbatim from upstream (class qualifier dropped so it is a free function) --- */
%(curve_to_ed_pub)s

static void emit_hex(const char *key, const uint8_t *b, size_t n)
{
    printf("\"%%s\": \"", key);
    for (size_t i = 0; i < n; i++) printf("%%02x", b[i]);
    printf("\", ");
}

int main()
{
    printf("{\n  \"cases\": [\n");
%(cases)s
    printf("    {\"_\": 0}\n  ]\n}\n");
    return 0;
}
"""

CASE_TMPL = r"""    {
        uint8_t priv[32], xpub[32], edpriv[32], edpub[32], edpub_from_x[32], sig[64];
        static const uint8_t payload[%(payload_n)s + 1] = {%(payload)s};
        uint8_t buf[XEDDSA_SIGNED_HEADER_LEN + 233];
        meshtastic_Data d;
        memset(&d, 0, sizeof(d));
        d.portnum = %(port)s;
        d.payload.size = %(payload_n)s;
        memcpy(d.payload.bytes, payload, %(payload_n)s);
        d.request_id = %(request_id)sU; d.reply_id = %(reply_id)sU; d.emoji = %(emoji)sU;
        d.has_bitfield = %(has_bitfield)s; d.bitfield = %(bitfield)s;
        d.want_response = %(want_response)s;
        for (int i = 0; i < 32; i++) priv[i] = (uint8_t)(%(seed)s + i);
        /* X25519 clamping. Curve25519::eval masks the POINT, not the scalar, while
         * priv_curve_to_ed_keys clamps -- an unclamped seed yields a public key that does
         * not match the derived Ed25519 key, which looks exactly like a broken map. */
        priv[0] &= 0xF8; priv[31] &= 0x7F; priv[31] |= 0x40;
        Curve25519::eval(xpub, priv, 0);
        uint8_t privcopy[32]; memcpy(privcopy, priv, 32);
        XEdDSA::priv_curve_to_ed_keys(privcopy, edpriv, edpub);
        curve_to_ed_pub(xpub, edpub_from_x);
        size_t len = buildSigningBuffer(buf, sizeof(buf), %(from)sU, %(id)sU, %(to)sU, &d);
        for (int i = 0; i < 32; i++) sig[i] = (uint8_t)(0xA0 + i);   /* deterministic Z */
        XEdDSA::sign(sig, edpriv, edpub, buf, len);
        printf("    {");
        printf("\"label\": \"%(label)s\", ");
        printf("\"from\": %%u, \"id\": %%u, \"to\": %%u, \"port\": %%u, ",
               (unsigned)%(from)sU, (unsigned)%(id)sU, (unsigned)%(to)sU, (unsigned)%(port)sU);
        printf("\"request_id\": %%u, \"reply_id\": %%u, \"emoji\": %%u, ",
               (unsigned)d.request_id, (unsigned)d.reply_id, (unsigned)d.emoji);
        printf("\"has_bitfield\": %%d, \"bitfield\": %%u, \"want_response\": %%d, ",
               (int)d.has_bitfield, (unsigned)d.bitfield, (int)d.want_response);
        emit_hex("x_priv", priv, 32);
        emit_hex("x_pub", xpub, 32);
        emit_hex("ed_pub", edpub, 32);
        emit_hex("ed_pub_from_x", edpub_from_x, 32);
        emit_hex("payload", payload, %(payload_n)s);
        emit_hex("signed_bytes", buf, len);
        emit_hex("sig", sig, 64);
        printf("\"self_verify\": %%d", (int)Ed25519::verify(sig, edpub, buf, len));
        printf("},\n");
    }
"""


def build_probe(regions: dict[str, Region], cases_in=None) -> str:
    bsb = regions["build_signing_buffer"].text
    # Drop only the class qualifier so the member becomes a free function; the body, which
    # is the algorithm, stays verbatim.
    ctep = regions["curve_to_ed_pub"].text.replace("CryptoEngine::", "", 1)
    cases = "".join(
        CASE_TMPL % {
            "label": c.label,
            "seed": f"0x{c.seed:02x}",
            "from": f"0x{c.frm:08x}",
            "id": f"0x{c.pid:08x}",
            "to": f"0x{c.to:08x}",
            "port": str(c.port),
            "payload": ", ".join(f"0x{b:02x}" for b in c.payload) or "0",
            "payload_n": str(len(c.payload)),
            "request_id": f"0x{c.request_id:08x}",
            "reply_id": f"0x{c.reply_id:08x}",
            "emoji": str(c.emoji),
            "has_bitfield": "true" if c.has_bitfield else "false",
            "bitfield": str(c.bitfield),
            "want_response": "true" if c.want_response else "false",
        }
        for c in (cases_in if cases_in is not None else CASES)
    )
    return PROBE % {
        "signing_constants": regions["signing_constants"].text,
        "put_le32": regions["put_le32"].text,
        "build_signing_buffer": bsb,
        "curve_to_ed_pub": ctep,
        "cases": cases,
    }


def run_probe(probe_src: str, lib: Path) -> dict:
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        sys.exit("error: no C++ compiler found (need g++ or clang++)")
    missing = [s for s in LIB_SOURCES if not (lib / s).is_file()]
    if missing:
        sys.exit(f"error: --crypto-lib is missing {', '.join(missing)} "
                 f"(expected the meshtastic/Crypto sources at the top level of {lib})")

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        src, exe = td / "probe.cpp", td / "probe"
        src.write_text(probe_src)
        cmd = [cxx, "-std=c++17", "-O1", "-w", f"-I{lib}", f"-I{lib / 'utility'}",
               str(src), *[str(lib / s) for s in LIB_SOURCES], "-o", str(exe)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            (HERE / "probe_xeddsa.failed.cpp").write_text(probe_src)
            sys.exit("error: probe failed to compile\n" + r.stderr +
                     "\nsource saved to tools/vectors/probe_xeddsa.failed.cpp")
        r = subprocess.run([str(exe)], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"error: probe crashed\n{r.stderr}")

    data = json.loads(r.stdout)
    data["cases"] = [c for c in data["cases"] if "_" not in c]
    return data


def lib_hashes(lib: Path) -> dict[str, str]:
    out = {}
    for s in LIB_SOURCES + ["XEdDSA.h", "Ed25519.h"]:
        p = lib / s
        if p.is_file():
            out[s] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


def c_bytes(hexstr: str) -> str:
    b = bytes.fromhex(hexstr)
    return ", ".join(f"0x{x:02x}" for x in b) or "0"


def emit_header(data: dict, regions: dict[str, Region], rev: str, lib_rev: str) -> str:
    L = []
    a = L.append
    a("/* XEdDSA known-answer vectors harvested from upstream Meshtastic.")
    a(" *")
    a(" * GENERATED by tools/vectors/harvest_xeddsa.py -- DO NOT EDIT BY HAND.")
    a(" * Regenerate:  python3 tools/vectors/harvest_xeddsa.py \\")
    a(" *                  --upstream <firmware-tree> --crypto-lib <meshtastic/Crypto>")
    a(" *")
    a(" * Every signature below was produced by COMPILING AND RUNNING upstream's own")
    a(" * signer against upstream's own signing-buffer layout, so they are the only data in")
    a(" * the tree that can prove our VERIFY (and, with the same Z, our SIGN) agrees with")
    a(" * the thing it has to interoperate with -- a self-test cannot.")
    a(" *")
    a(" * The deterministic Z mixed into every nonce below is 0xA0, 0xA1, ... 0xBF -- the")
    a(" * signing tests must pass exactly that to reproduce these signatures.")
    a(" *")
    a(f" * Upstream firmware: {rev}")
    a(f" * Signer library:    meshtastic/Crypto {lib_rev}")
    a(" * Verbatim regions (sha256 of extracted text):")
    for r in regions.values():
        a(f" *   {r.name:22s} {r.relpath}:{r.line}  {r.sha256[:16]}")
    a(" */")
    a("")
    a("#ifndef MESHTASTIC_XEDDSA_VECTORS_H_")
    a("#define MESHTASTIC_XEDDSA_VECTORS_H_")
    a("")
    a("#include <stdbool.h>")
    a("#include <stdint.h>")
    a("#include <stddef.h>")
    a("")
    a("struct mt_xeddsa_vector {")
    a("\tconst char *label;")
    a("\t/* MeshPacket header part */")
    a("\tuint32_t from;")
    a("\tuint32_t id;")
    a("\tuint32_t to;")
    a("\t/* Data envelope part (#11422: everything but dest/source is covered) */")
    a("\tuint32_t portnum;")
    a("\tuint32_t request_id;")
    a("\tuint32_t reply_id;")
    a("\tuint32_t emoji;")
    a("\tuint8_t bitfield;")
    a("\tbool has_bitfield;")
    a("\tbool want_response;")
    a("\t/* TEST KEY ONLY: derived from a fixed seed by the harvester, never a real node's")
    a("\t * key. Present so the signing tests can reproduce upstream's signature. */")
    a("\tuint8_t x_priv[32];")
    a("\tuint8_t x_pub[32];        /* the sender's X25519 identity key */")
    a("\tuint8_t ed_pub[32];       /* what upstream derives from the PRIVATE key */")
    a("\tconst uint8_t *payload;")
    a("\tsize_t payload_len;")
    a("\tconst uint8_t *signed_bytes;  /* buildSigningBuffer output */")
    a("\tsize_t signed_len;")
    a("\tuint8_t sig[64];")
    a("};")
    a("")
    for c in data["cases"]:
        a(f"static const uint8_t mt_xeddsa_payload_{c['label']}[] = {{{c_bytes(c['payload'])}}};")
        a(f"static const uint8_t mt_xeddsa_signed_{c['label']}[] = "
          f"{{{c_bytes(c['signed_bytes'])}}};")
    a("")
    a("static const struct mt_xeddsa_vector mt_xeddsa_vectors[] = {")
    for c in data["cases"]:
        assert c["ed_pub"] == c["ed_pub_from_x"], (
            f"{c['label']}: upstream's own map disagrees with its key derivation")
        assert c["self_verify"] == 1, f"{c['label']}: upstream cannot verify its own signature"
        a("\t{")
        a(f"\t\t.label = \"{c['label']}\",")
        a(f"\t\t.from = 0x{c['from']:08x}U, .id = 0x{c['id']:08x}U, .to = 0x{c['to']:08x}U,")
        a(f"\t\t.portnum = {c['port']}U, .request_id = 0x{c['request_id']:08x}U, "
          f".reply_id = 0x{c['reply_id']:08x}U, .emoji = {c['emoji']}U,")
        a(f"\t\t.bitfield = {c['bitfield']}U, "
          f".has_bitfield = {'true' if c['has_bitfield'] else 'false'}, "
          f".want_response = {'true' if c['want_response'] else 'false'},")
        a(f"\t\t.x_priv = {{{c_bytes(c['x_priv'])}}},")
        a(f"\t\t.x_pub = {{{c_bytes(c['x_pub'])}}},")
        a(f"\t\t.ed_pub = {{{c_bytes(c['ed_pub'])}}},")
        a(f"\t\t.payload = mt_xeddsa_payload_{c['label']},")
        # Not sizeof(): an empty payload is emitted as a one-element placeholder array.
        a(f"\t\t.payload_len = {len(bytes.fromhex(c['payload']))}U,")
        a(f"\t\t.signed_bytes = mt_xeddsa_signed_{c['label']},")
        a(f"\t\t.signed_len = sizeof(mt_xeddsa_signed_{c['label']}),")
        a(f"\t\t.sig = {{{c_bytes(c['sig'])}}},")
        a("\t},")
    a("};")
    a("")
    a("#endif /* MESHTASTIC_XEDDSA_VECTORS_H_ */")
    return "\n".join(L) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--upstream", required=True, type=Path, help="meshtastic/firmware tree")
    ap.add_argument("--crypto-lib", required=True, type=Path,
                    help="meshtastic/Crypto sources (the pinned revision)")
    ap.add_argument("--check", action="store_true",
                    help="verify the lock still matches; writes nothing")
    args = ap.parse_args()

    regions = {n: grab(args.upstream, n, rel, anchor) for n, rel, anchor in TARGETS}
    regions["signing_constants"] = grab_constants(args.upstream)
    for n, r in regions.items():
        print(f"  extracted {n:22s} {r.relpath}:{r.line}  {r.sha256[:16]}")

    libs = lib_hashes(args.crypto_lib)

    # Pass 1: the fixed cases, plus a throwaway case that only exists to reveal the
    # bootstrap seed's public key.
    probe_cases = list(CASES) + [Case("bootstrap_probe", BOOTSTRAP_SEED, 1, 1, BROADCAST,
                                      BOOTSTRAP_PORT, b"")]
    first = run_probe(build_probe(regions, probe_cases), args.crypto_lib)
    pub = bytes.fromhex(next(c for c in first["cases"] if c["label"] == "bootstrap_probe")["x_pub"])

    # Pass 2: now the User payload and the key-derived id are computable.
    payload = user_payload(pub)
    node_id = crc32_ieee(pub)
    probe_cases = list(CASES) + [
        Case("nodeinfo_bootstrap", BOOTSTRAP_SEED, node_id, 0x5150C0DE, BROADCAST,
             BOOTSTRAP_PORT, payload)
    ]
    data = run_probe(build_probe(regions, probe_cases), args.crypto_lib)
    boot = next(c for c in data["cases"] if c["label"] == "nodeinfo_bootstrap")
    if bytes.fromhex(boot["x_pub"]) != pub or boot["from"] != node_id:
        sys.exit("error: bootstrap case is not self-consistent between passes")
    rev = upstream_revision(args.upstream)
    lib_rev = (args.crypto_lib / ".git").exists() and upstream_revision(args.crypto_lib) or "pinned archive"

    if args.check:
        if not LOCK.is_file():
            sys.exit("error: no xeddsa.lock to check against")
        lock = json.loads(LOCK.read_text())
        drift = [n for n, r in regions.items()
                 if lock.get("regions", {}).get(n, {}).get("sha256") not in (None, r.sha256)]
        drift += [f"lib:{s}" for s, h in libs.items()
                  if lock.get("crypto_lib", {}).get("files", {}).get(s) not in (None, h)]
        if drift:
            sys.exit("error: upstream drift in " + ", ".join(drift) +
                     "\nRead the upstream diff before regenerating: if the signed-byte layout "
                     "or the signer moved, this port has an interop break to fix.")
        print("lock matches upstream")
        return 0

    HEADER_OUT.parent.mkdir(parents=True, exist_ok=True)
    HEADER_OUT.write_text(emit_header(data, regions, rev, lib_rev))
    LOCK.write_text(json.dumps({
        "upstream": rev,
        "crypto_lib": {"revision": lib_rev, "files": libs},
        "regions": {n: {"file": r.relpath, "line": r.line, "anchor": r.anchor,
                        "sha256": r.sha256} for n, r in regions.items()},
    }, indent=2) + "\n")
    print(f"wrote {HEADER_OUT.relative_to(REPO)}")
    print(f"wrote {LOCK.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
