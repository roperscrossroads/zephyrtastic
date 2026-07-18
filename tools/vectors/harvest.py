#!/usr/bin/env python3
"""Harvest known-answer vectors from an upstream Meshtastic firmware tree.

Why this exists
---------------
`tests/admin_pki` is a self-loopback: it forges the peer->us frame with the same
code that decodes it, so a *symmetric* wire error cancels out and passes green.
Nothing in the suite validates our output against bytes produced by stock
firmware. These vectors close that gap, and they are simultaneously the
reference data needed to implement preset/region/channel-hash support.

Trust model (the whole point)
-----------------------------
A vector is only worth something if it did NOT come from a reimplementation --
otherwise a wrong algorithm gets locked in as "expected". So there are exactly
two permitted sources here:

  (a) VERBATIM extraction of an upstream source region, compiled and executed.
  (b) MECHANICAL parse of an upstream data table.

Nothing is hand-transcribed. Every extracted region is recorded in
upstream.lock with a sha256; if upstream edits that region, re-harvesting fails
loudly instead of silently producing different numbers.

Usage
-----
    ./harvest.py --upstream /path/to/firmware            # harvest + write header
    ./harvest.py --upstream /path/to/firmware --check    # verify lock, no writes
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
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
LOCK = HERE / "upstream.lock"
HEADER_OUT = REPO / "tests" / "vectors" / "meshtastic_vectors.h"


# --------------------------------------------------------------------------
# Verbatim extraction
# --------------------------------------------------------------------------

@dataclass
class Region:
    """A verbatim slice of upstream source, with provenance."""
    name: str
    relpath: str
    anchor: str
    text: str
    line: int

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.text.encode()).hexdigest()


def extract_braced(src: str, anchor: str) -> tuple[str, int]:
    """Return the verbatim text from `anchor` through its matching close brace.

    Brace counting is string/char/comment aware -- a `'}'` inside a literal must
    not end the region.
    """
    start = src.find(anchor)
    if start < 0:
        raise LookupError(f"anchor not found: {anchor!r}")

    i = src.find("{", start)
    if i < 0:
        raise LookupError(f"no opening brace after anchor: {anchor!r}")

    depth = 0
    in_str = in_chr = in_line_c = in_block_c = False
    while i < len(src):
        c, nxt = src[i], src[i + 1 : i + 2]
        if in_line_c:
            if c == "\n":
                in_line_c = False
        elif in_block_c:
            if c == "*" and nxt == "/":
                in_block_c = False
                i += 1
        elif in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif in_chr:
            if c == "\\":
                i += 1
            elif c == "'":
                in_chr = False
        elif c == "/" and nxt == "/":
            in_line_c = True
            i += 1
        elif c == "/" and nxt == "*":
            in_block_c = True
            i += 1
        elif c == '"':
            in_str = True
        elif c == "'":
            in_chr = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return src[start : i + 1], src[:start].count("\n") + 1
        i += 1
    raise LookupError(f"unbalanced braces from anchor: {anchor!r}")


def grab(upstream: Path, name: str, relpath: str, anchor: str) -> Region:
    path = upstream / relpath
    if not path.is_file():
        sys.exit(f"error: upstream file missing: {relpath}")
    text, line = extract_braced(path.read_text(errors="replace"), anchor)
    return Region(name=name, relpath=relpath, anchor=anchor, text=text, line=line)


# The four algorithms we execute rather than reimplement.
TARGETS = [
    # channel hash -> the packet header channel byte (parity: crypto #1)
    ("xor_hash", "src/mesh/Channels.cpp", "uint8_t xorHash("),
    # djb2 -> frequency slot selection (parity: radio D3). NOTE: a *different*
    # function from xorHash; conflating the two is an easy and silent bug.
    ("djb2_hash", "src/mesh/RadioInterface.cpp", "uint32_t hash(const char *str)"),
    # preset -> SF/BW/CR (parity: radio D2)
    ("modem_preset_to_params", "src/mesh/MeshRadio.h", "static inline void modemPresetToParams("),
    # AES-CTR / PKC nonce layout (parity: crypto, security H1)
    ("init_nonce", "src/mesh/CryptoEngine.cpp", "void CryptoEngine::initNonce("),
    # preset -> display name. NOT cosmetic: the display name is hashed to pick
    # the frequency slot, and substitutes for an empty channel name when
    # hashing the channel. Upstream calls it "a stable literal for
    # channel-name hashing and default-channel detection". A wrong string here
    # puts the node on the wrong frequency AND the wrong channel hash.
    ("preset_display_name", "src/DisplayFormatters.cpp",
     "const char *DisplayFormatters::getModemPresetDisplayName("),
]


# --------------------------------------------------------------------------
# Mechanical table parsing
# --------------------------------------------------------------------------

def parse_preset_enum(upstream: Path) -> dict[str, int]:
    """Parse ModemPreset enum values from the generated protobuf header."""
    src = (upstream / "src/mesh/generated/meshtastic/config.pb.h").read_text(errors="replace")
    out = {}
    for m in re.finditer(
        r"meshtastic_Config_LoRaConfig_ModemPreset_([A-Z_0-9]+)\s*=\s*(\d+)", src
    ):
        out[m.group(1)] = int(m.group(2))
    if not out:
        sys.exit("error: parsed zero modem presets from config.pb.h")
    return out


def parse_profiles(upstream: Path) -> dict[str, dict]:
    """Parse `const RegionProfile PROFILE_X = {presets, spacing, padding, ...}`.

    spacing/padding feed the slot-width calculation, so they must come from
    upstream rather than being assumed.
    """
    src = (upstream / "src/mesh/RadioInterface.cpp").read_text(errors="replace")
    out = {}
    for m in re.finditer(
        r"const\s+RegionProfile\s+(PROFILE_\w+)\s*=\s*\{([^}]*)\}", src
    ):
        name, body = m.group(1), m.group(2)
        parts = [p.strip() for p in body.split(",")]
        if len(parts) < 3:
            continue
        try:
            out[name] = {
                "presets": parts[0],
                "spacing": float(parts[1]),
                "padding": float(parts[2]),
            }
        except ValueError:
            continue
    if not out:
        sys.exit("error: parsed zero region profiles")
    return out


def parse_regions(upstream: Path) -> list[dict]:
    """Parse the RDEF(...) region table -- freq range, duty cycle, power limit.

    Duty-cycle percentages here are exactly what `airtime CP-1` needs; the port
    currently has no regulatory duty-cycle enforcement at all.
    """
    src = (upstream / "src/mesh/RadioInterface.cpp").read_text(errors="replace")
    out = []
    for m in re.finditer(
        r"RDEF\(\s*(\w+)\s*,\s*([\d.]+)f?\s*,\s*([\d.]+)f?\s*,\s*([\d.]+)\s*,"
        r"\s*(-?[\d.]+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,"
        r"\s*PRESET\((\w+)\)\s*,\s*(-?\d+)\s*\)",
        src,
    ):
        out.append({
            "region": m.group(1),
            "freq_start": float(m.group(2)),
            "freq_end": float(m.group(3)),
            "duty_cycle": float(m.group(4)),
            "power_limit": float(m.group(5)),
            "wide_lora": m.group(7) == "true",
            "profile": m.group(8),
            "default_preset": m.group(9),
            "override_slot": int(m.group(10)),
        })
    if not out:
        sys.exit("error: parsed zero regions from the RDEF table")
    return out


# --------------------------------------------------------------------------
# Probe: compile the verbatim regions and execute them
# --------------------------------------------------------------------------

# Channel names to hash. Includes the preset display names (used for the preset
# hash slot) and awkward cases: empty, spaces, non-ASCII, max-length.
CHANNEL_NAMES = [
    "", "LongFast", "LongSlow", "MediumFast", "MediumSlow", "ShortFast",
    "ShortSlow", "LongMod", "ShortTurbo", "admin", "gpio", "mqtt", "serial",
    "Test Channel", "channel-with-dash", "UPPER", "lower", "MiXeD123",
    "ünïcødé", "0123456789AB",
]

# PSKs as (label, bytes). Covers the "no PSK" case, the default single-byte
# index form, and full 16/32-byte keys.
PSKS = [
    ("empty", []),
    ("default_index_1", [0x01]),
    ("index_2", [0x02]),
    ("aes128_zero", [0x00] * 16),
    ("aes128_ff", [0xFF] * 16),
    ("aes128_counting", list(range(16))),
    ("aes256_counting", list(range(32))),
]

PROBE_TEMPLATE = r"""
// GENERATED by tools/vectors/harvest.py -- do not edit.
// Upstream regions below are VERBATIM; only the surrounding shims are ours.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

// -- enum shim: values parsed mechanically from config.pb.h ----------------
enum meshtastic_Config_LoRaConfig_ModemPreset {
%(preset_enum)s
};
#define PRESET(name) meshtastic_Config_LoRaConfig_ModemPreset_##name

// ===== VERBATIM UPSTREAM: xorHash (%(xor_hash_src)s) =====
%(xor_hash)s
// ===== END VERBATIM =====

// ===== VERBATIM UPSTREAM: hash (%(djb2_hash_src)s) =====
%(djb2_hash)s
// ===== END VERBATIM =====

// ===== VERBATIM UPSTREAM: modemPresetToParams (%(modem_preset_to_params_src)s) =====
%(modem_preset_to_params)s
// ===== END VERBATIM =====

// getModemPresetDisplayName is a DisplayFormatters static member. Only the
// class qualifier is dropped to make it a free function; the body -- every
// returned literal -- is verbatim. (%(preset_display_name_src)s)
%(preset_display_name_shim)s

// initNonce is a CryptoEngine member writing to a member buffer. The BODY below
// is verbatim; only the signature is shimmed to a free function over a caller
// buffer. The subtlety worth capturing: extraNonce lands at offset 4, which
// OVERLAPS the 8-byte packetId written at offset 0.
static uint8_t nonce[16];
%(init_nonce_shim)s

static void emit_bytes(const char *k, const uint8_t *b, int n) {
    printf("    \"%%s\": [", k);
    for (int i = 0; i < n; i++) printf("%%s%%u", i ? "," : "", b[i]);
    printf("],\n");
}

int main(void) {
    printf("{\n");

    // ---- channel-name xor hashes ----
    printf("  \"xor_hash_names\": {\n");
%(name_hash_calls)s
    printf("    \"_\": 0\n  },\n");

    // ---- djb2 slot hashes ----
    printf("  \"djb2_names\": {\n");
%(djb2_calls)s
    printf("    \"_\": 0\n  },\n");

    // ---- psk xor hashes (channel hash = xorHash(name) ^ xorHash(psk)) ----
    printf("  \"xor_hash_psks\": {\n");
%(psk_hash_calls)s
    printf("    \"_\": 0\n  },\n");

    // ---- preset -> SF/BW/CR, both narrow and wide LoRa ----
    printf("  \"presets\": {\n");
%(preset_calls)s
    printf("    \"_\": {\"sf\":0,\"bw\":0,\"cr\":0}\n  },\n");

    // ---- preset display names + the slot hash computed over them ----
    printf("  \"preset_names\": {\n");
%(preset_name_calls)s
    printf("    \"_\": {}\n  },\n");

    // ---- nonce layouts ----
    printf("  \"nonces\": {\n");
%(nonce_calls)s
    printf("    \"_\": []\n  }\n");

    printf("}\n");
    return 0;
}
"""


def c_str(s: str) -> str:
    """Encode a Python str as a C string literal, escaping non-ASCII as \\x.."""
    out = []
    for b in s.encode("utf-8"):
        if b == 0x22:
            out.append('\\"')
        elif b == 0x5C:
            out.append("\\\\")
        elif 0x20 <= b < 0x7F:
            out.append(chr(b))
        else:
            # trailing "" stops C from swallowing following hex digits
            out.append(f'\\x{b:02x}""')
    return '"' + "".join(out) + '"'


def build_probe(regions: dict[str, Region], presets: dict[str, int]) -> str:
    enum_lines = ",\n".join(
        f"    meshtastic_Config_LoRaConfig_ModemPreset_{k} = {v}"
        for k, v in sorted(presets.items(), key=lambda kv: kv[1])
    )

    # Shim initNonce's signature while keeping its body verbatim.
    nonce_region = regions["init_nonce"].text
    body = nonce_region[nonce_region.find("{") :]
    init_nonce_shim = (
        "static void initNonce(uint32_t fromNode, uint64_t packetId, uint32_t extraNonce = 0)\n"
        + body
    )

    # Drop only the class qualifier so the member becomes a free function.
    # Every returned literal stays verbatim -- those literals ARE the data.
    dn_region = regions["preset_display_name"].text
    display_name_shim = "static " + dn_region.replace("DisplayFormatters::", "", 1)

    name_calls = "\n".join(
        f'    printf("    \\"%s\\": %u,\\n", {c_str(n)}, '
        f"(unsigned)xorHash((const uint8_t *){c_str(n)}, strlen({c_str(n)})));"
        for n in CHANNEL_NAMES
    )
    djb2_calls = "\n".join(
        f'    printf("    \\"%s\\": %u,\\n", {c_str(n)}, (unsigned)hash({c_str(n)}));'
        for n in CHANNEL_NAMES
    )

    psk_calls = []
    for label, data in PSKS:
        arr = ", ".join(f"0x{b:02x}" for b in data) or "0"
        psk_calls.append(
            f"    {{ static const uint8_t p[] = {{{arr}}};\n"
            f'      printf("    \\"{label}\\": %u,\\n", '
            f"(unsigned)xorHash(p, {len(data)})); }}"
        )

    preset_calls = []
    for pname in sorted(presets):
        for wide in (False, True):
            key = f"{pname}{'_wide' if wide else ''}"
            preset_calls.append(
                f"    {{ float bw = 0; uint8_t sf = 0, cr = 0;\n"
                f"      modemPresetToParams(PRESET({pname}), {str(wide).lower()}, bw, sf, cr);\n"
                f'      printf("    \\"{key}\\": {{\\"sf\\":%u,\\"bw\\":%.3f,\\"cr\\":%u}},\\n",'
                f" sf, bw, cr); }}"
            )

    # Display name per preset, plus the djb2 hash OF that name -- which is the
    # value that actually selects a frequency slot. Also emits the usePreset=false
    # case, whose literal ("Custom") is what a custom-modem node hashes.
    preset_name_calls = []
    for pname in sorted(presets):
        preset_name_calls.append(
            f"    {{ const char *d = getModemPresetDisplayName(PRESET({pname}), false, true);\n"
            f"      const char *s = getModemPresetDisplayName(PRESET({pname}), true, true);\n"
            f'      printf("    \\"{pname}\\": {{\\"display\\":\\"%s\\",\\"short\\":\\"%s\\","'
            f'             "\\"djb2\\":%u,\\"xor\\":%u}},\\n",'
            f" d, s, (unsigned)hash(d),"
            f" (unsigned)xorHash((const uint8_t *)d, strlen(d))); }}"
        )
    preset_name_calls.append(
        '    { const char *d = getModemPresetDisplayName(PRESET(LONG_FAST), false, false);\n'
        '      printf("    \\"_CUSTOM\\": {\\"display\\":\\"%s\\",\\"short\\":\\"%s\\","'
        '             "\\"djb2\\":%u,\\"xor\\":%u},\\n",'
        ' d, d, (unsigned)hash(d),'
        ' (unsigned)xorHash((const uint8_t *)d, strlen(d))); }'
    )

    # Cases chosen to expose the offset-4 overlap between packetId and extraNonce.
    nonce_cases = [
        ("psk_id1_from0", 1, 0, 0),
        ("psk_id_max32", 0xFFFFFFFF, 0, 0),
        ("psk_id1_from_deadbeef", 1, 0xDEADBEEF, 0),
        ("psk_id_0102030405060708", 0x0102030405060708, 0x11223344, 0),
        ("pkc_extra_1", 1, 0xDEADBEEF, 1),
        ("pkc_extra_aabbccdd", 1, 0xDEADBEEF, 0xAABBCCDD),
        ("pkc_bigid_extra", 0x0102030405060708, 0x11223344, 0xAABBCCDD),
    ]
    # upstream signature order is (fromNode, packetId, extraNonce)
    nonce_calls = "\n".join(
        f"    {{ initNonce({frm}U, {pid}ULL, {extra}U); "
        f'emit_bytes("{label}", nonce, 16); }}'
        for label, pid, frm, extra in nonce_cases
    )

    return PROBE_TEMPLATE % {
        "preset_enum": enum_lines,
        "xor_hash": regions["xor_hash"].text,
        "xor_hash_src": f"{regions['xor_hash'].relpath}:{regions['xor_hash'].line}",
        "djb2_hash": regions["djb2_hash"].text,
        "djb2_hash_src": f"{regions['djb2_hash'].relpath}:{regions['djb2_hash'].line}",
        "modem_preset_to_params": regions["modem_preset_to_params"].text,
        "modem_preset_to_params_src": (
            f"{regions['modem_preset_to_params'].relpath}:"
            f"{regions['modem_preset_to_params'].line}"
        ),
        "init_nonce_shim": init_nonce_shim,
        "preset_display_name_shim": display_name_shim,
        "preset_display_name_src": (
            f"{regions['preset_display_name'].relpath}:"
            f"{regions['preset_display_name'].line}"
        ),
        "preset_name_calls": "\n".join(preset_name_calls),
        "name_hash_calls": name_calls,
        "djb2_calls": djb2_calls,
        "psk_hash_calls": "\n".join(psk_calls),
        "preset_calls": "\n".join(preset_calls),
        "nonce_calls": nonce_calls,
    }


def run_probe(probe_src: str) -> dict:
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        sys.exit("error: no C++ compiler found (need g++ or clang++)")

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        src, exe = td / "probe.cpp", td / "probe"
        src.write_text(probe_src)
        r = subprocess.run(
            [cxx, "-std=c++17", "-O0", "-w", str(src), "-o", str(exe)],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            (HERE / "probe.failed.cpp").write_text(probe_src)
            sys.exit(
                "error: probe failed to compile\n"
                f"{r.stderr}\nsource saved to tools/vectors/probe.failed.cpp"
            )
        r = subprocess.run([str(exe)], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"error: probe crashed\n{r.stderr}")

    # strip the "_" sentinels used to keep trailing commas legal
    data = json.loads(r.stdout)
    for v in data.values():
        if isinstance(v, dict):
            v.pop("_", None)
    return data


# --------------------------------------------------------------------------
# Header emission
# --------------------------------------------------------------------------

def slot_math(regions_tbl: list[dict], profiles: dict[str, dict],
              presets_out: dict, djb2: dict[str, int]) -> list[dict]:
    """Reproduce upstream's slot-count arithmetic from parsed constants only.

        freqSlotWidth = spacing + padding*2 + bw_khz/1000
        numFreqSlots  = round((freqEnd - freqStart + spacing) / freqSlotWidth)
        slot          = hash(name) % numFreqSlots

    Every input is parsed or probe-computed -- nothing assumed.
    """
    out = []
    for r in regions_tbl:
        prof = profiles.get(r["profile"])
        if not prof:
            continue
        key = r["default_preset"] + ("_wide" if r["wide_lora"] else "")
        p = presets_out.get(key) or presets_out.get(r["default_preset"])
        if not p:
            continue
        width = prof["spacing"] + prof["padding"] * 2 + p["bw"] / 1000.0
        if width <= 0:
            continue
        # C++ round() is half-away-from-zero; Python's round() is banker's.
        raw = (r["freq_end"] - r["freq_start"] + prof["spacing"]) / width
        num = int(math_round_half_away(raw))
        entry = dict(r)
        entry["slot_width_mhz"] = round(width, 6)
        entry["num_freq_slots"] = num
        entry["default_preset_bw_khz"] = p["bw"]
        if num > 0:
            entry["slot_longfast"] = djb2.get("LongFast", 0) % num
        out.append(entry)
    return out


def math_round_half_away(x: float) -> float:
    import math
    return math.floor(x + 0.5) if x >= 0 else math.ceil(x - 0.5)


def emit_header(data: dict, regions: dict[str, Region], upstream: Path,
                upstream_rev: str) -> str:
    L = []
    a = L.append
    a("/* Known-answer vectors harvested from upstream Meshtastic firmware.")
    a(" *")
    a(" * GENERATED by tools/vectors/harvest.py -- DO NOT EDIT BY HAND.")
    a(" * Regenerate:  python3 tools/vectors/harvest.py --upstream <firmware-tree>")
    a(" *")
    a(" * Every value below was produced by COMPILING AND RUNNING upstream's own")
    a(" * code, or by mechanically parsing an upstream table -- never by")
    a(" * reimplementing an algorithm. That is what makes these vectors able to")
    a(" * catch a symmetric error that the self-loopback tests cannot.")
    a(" *")
    a(f" * Upstream: {upstream_rev}")
    a(" * Verbatim regions (sha256 of extracted text):")
    for r in regions.values():
        a(f" *   {r.name:24s} {r.relpath}:{r.line}  {r.sha256[:16]}")
    a(" */")
    a("")
    a("#ifndef MESHTASTIC_VECTORS_H_")
    a("#define MESHTASTIC_VECTORS_H_")
    a("")
    a("#include <stdint.h>")
    a("")

    a("/* --- Channel-name xor hash (parity: crypto #1) ------------------------")
    a(" * Channel hash = xorHash(name) ^ xorHash(psk). The port currently")
    a(' * hardcodes the name "LongFast", so every non-default channel hashes')
    a(" * wrong and stock nodes silently ignore the traffic.")
    a(" */")
    a("struct mt_vec_name_hash { const char *name; uint8_t hash; };")
    a("static const struct mt_vec_name_hash mt_vec_name_hashes[] = {")
    for n, h in data["xor_hash_names"].items():
        a(f"\t{{ {json.dumps(n)}, {h} }},")
    a("};")
    a("")

    a("/* --- PSK xor hash (second half of the channel hash) ------------------ */")
    a("struct mt_vec_psk_hash { const char *label; uint8_t hash; };")
    a("static const struct mt_vec_psk_hash mt_vec_psk_hashes[] = {")
    for k, h in data["xor_hash_psks"].items():
        a(f"\t{{ {json.dumps(k)}, {h} }},")
    a("};")
    a("")

    a("/* --- djb2 hash, frequency-slot selection (parity: radio D3) ----------")
    a(" * NOTE: this is a DIFFERENT function from the channel xorHash above.")
    a(" * Conflating the two is silent and produces a node on the wrong")
    a(" * frequency that still believes it is configured correctly.")
    a(" */")
    a("struct mt_vec_djb2 { const char *name; uint32_t hash; };")
    a("static const struct mt_vec_djb2 mt_vec_djb2_hashes[] = {")
    for n, h in data["djb2_names"].items():
        a(f"\t{{ {json.dumps(n)}, {h}u }},")
    a("};")
    a("")

    a("/* --- Modem preset -> SF/BW/CR (parity: radio D2) ---------------------")
    a(" * The port is frozen at LongFast; these are the parameters stock")
    a(" * firmware uses for every preset, narrow and wide.")
    a(" */")
    a("struct mt_vec_preset {")
    a("\tconst char *name;")
    a("\tint preset_enum;")
    a("\tuint8_t wide;")
    a("\tuint8_t sf;")
    a("\tuint32_t bw_hz;")
    a("\tuint8_t cr;")
    a("};")
    a("static const struct mt_vec_preset mt_vec_presets[] = {")
    for k, v in data["presets"].items():
        wide = 1 if k.endswith("_wide") else 0
        base = k[:-5] if wide else k
        a(f"\t{{ {json.dumps(base)}, {data['preset_enum'][base]}, {wide}, {v['sf']}, "
          f"{int(round(v['bw'] * 1000))}u, {v['cr']} }},")
    a("};")
    a("")

    a("/* --- Preset display names (parity: radio D3 + crypto #1) -------------")
    a(" * Not cosmetic. `djb2` is the hash OF the display name, and that is what")
    a(" * selects the frequency slot. The display name also substitutes for an")
    a(" * empty channel name when computing the channel hash -- which is why the")
    a(' * port hardcoding "LongFast" is only correct while the modem is frozen')
    a(" * at LongFast. Get a string wrong and the node lands on the wrong")
    a(" * frequency AND the wrong channel hash, with no error anywhere.")
    a(" *")
    a(' * `_CUSTOM` is the use_preset=false literal, hashed the same way.')
    a(" */")
    a("struct mt_vec_preset_name {")
    a("\tconst char *preset;")
    a("\tconst char *display;")
    a("\tconst char *short_name;")
    a("\tuint32_t djb2;")
    a("\tuint8_t xor_hash;")
    a("};")
    a("static const struct mt_vec_preset_name mt_vec_preset_names[] = {")
    for k, v in data["preset_names"].items():
        a(f"\t{{ {json.dumps(k)}, {json.dumps(v['display'])}, "
          f"{json.dumps(v['short'])}, {v['djb2']}u, {v['xor']} }},")
    a("};")
    a("")

    a("/* --- Region table (parity: radio D3 + airtime CP-1) ------------------")
    a(" * duty_cycle_pct is the regulatory ceiling. The port measures TX")
    a(" * airtime but never gates on it, so EU/UA builds can transmit past")
    a(" * the legal limit -- these are the numbers that enforcement needs.")
    a(" */")
    a("struct mt_vec_region {")
    a("\tconst char *name;")
    a("\tfloat freq_start_mhz;")
    a("\tfloat freq_end_mhz;")
    a("\tfloat duty_cycle_pct;")
    a("\tfloat power_limit_dbm;")
    a("\tuint8_t wide_lora;")
    a("\tconst char *default_preset;")
    a("\tuint32_t num_freq_slots;")
    a("\tfloat slot_width_mhz;")
    a("};")
    a("static const struct mt_vec_region mt_vec_regions[] = {")
    for r in data["regions"]:
        a(f"\t{{ {json.dumps(r['region'])}, {r['freq_start']}f, {r['freq_end']}f, "
          f"{r['duty_cycle']}f, {r['power_limit']}f, "
          f"{1 if r['wide_lora'] else 0}, {json.dumps(r['default_preset'])}, "
          f"{r.get('num_freq_slots', 0)}u, {r.get('slot_width_mhz', 0)}f }},")
    a("};")
    a("")

    a("/* --- Nonce layout (parity: crypto, security H1) ----------------------")
    a(" * 16-byte buffer; first 13 are used. Note the deliberate OVERLAP:")
    a(" * packetId is written as 8 bytes at offset 0, then a non-zero")
    a(" * extraNonce is written at offset 4 -- on top of packetId's high half.")
    a(" * A reimplementation that appends extraNonce instead produces a")
    a(" * plausible, wrong, and self-consistent nonce.")
    a(" */")
    a("struct mt_vec_nonce { const char *label; uint8_t nonce[16]; };")
    a("static const struct mt_vec_nonce mt_vec_nonces[] = {")
    for k, v in data["nonces"].items():
        bs = ", ".join(f"0x{b:02x}" for b in v)
        a(f"\t{{ {json.dumps(k)}, {{ {bs} }} }},")
    a("};")
    a("")

    a("#define MT_VEC_COUNT(a) ((unsigned)(sizeof(a) / sizeof((a)[0])))")
    a("")
    a("#endif /* MESHTASTIC_VECTORS_H_ */")
    return "\n".join(L) + "\n"


# --------------------------------------------------------------------------

def upstream_revision(upstream: Path) -> str:
    r = subprocess.run(
        ["git", "-C", str(upstream), "rev-parse", "--short", "HEAD"],
        capture_output=True, text=True,
    )
    if r.returncode == 0 and r.stdout.strip():
        return f"{upstream.name} @ {r.stdout.strip()}"
    return f"{upstream.name} (not a git checkout)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--upstream", required=True, type=Path,
                    help="path to an upstream meshtastic/firmware tree")
    ap.add_argument("--check", action="store_true",
                    help="verify extracted regions still match upstream.lock; write nothing")
    args = ap.parse_args()

    upstream = args.upstream.expanduser().resolve()
    if not (upstream / "src/mesh/Channels.cpp").is_file():
        sys.exit(f"error: {upstream} does not look like a meshtastic firmware tree")

    print(f"upstream: {upstream}")

    regions = {}
    for name, relpath, anchor in TARGETS:
        try:
            r = grab(upstream, name, relpath, anchor)
        except LookupError as e:
            sys.exit(
                f"error: {e}\n"
                "  Upstream moved or renamed this code. Re-anchor TARGETS in\n"
                "  harvest.py against the new source before trusting any vector."
            )
        regions[name] = r
        print(f"  extracted {name:24s} {relpath}:{r.line}  {r.sha256[:16]}")

    # Drift check against the lock.
    lock = json.loads(LOCK.read_text()) if LOCK.is_file() else None
    if lock:
        drift = [
            n for n, r in regions.items()
            if lock.get("regions", {}).get(n, {}).get("sha256") not in (None, r.sha256)
        ]
        if drift:
            msg = (
                "error: upstream drift in: " + ", ".join(drift) + "\n"
                "  The algorithm changed since these vectors were harvested.\n"
                "  Re-read the upstream diff, then re-run without --check to\n"
                "  regenerate. Do NOT just delete the lock."
            )
            if args.check:
                sys.exit(msg)
            print("warning: " + msg.split("\n", 1)[1], file=sys.stderr)
    if args.check:
        print("check: extracted regions match upstream.lock" if lock
              else "check: no lock yet (run without --check to create)")
        return 0

    presets = parse_preset_enum(upstream)
    profiles = parse_profiles(upstream)
    regions_tbl = parse_regions(upstream)
    print(f"  parsed {len(presets)} presets, {len(profiles)} profiles, "
          f"{len(regions_tbl)} regions")

    data = run_probe(build_probe(regions, presets))
    data["preset_enum"] = presets
    data["regions"] = slot_math(regions_tbl, profiles, data["presets"],
                                data["djb2_names"])
    print(f"  probe produced {len(data['xor_hash_names'])} name hashes, "
          f"{len(data['presets'])} preset params, {len(data['nonces'])} nonces")

    rev = upstream_revision(upstream)
    HEADER_OUT.parent.mkdir(parents=True, exist_ok=True)
    HEADER_OUT.write_text(emit_header(data, regions, upstream, rev))
    LOCK.write_text(json.dumps({
        "upstream": rev,
        "regions": {
            n: {"file": r.relpath, "line": r.line, "anchor": r.anchor,
                "sha256": r.sha256}
            for n, r in regions.items()
        },
    }, indent=2) + "\n")

    print(f"wrote {HEADER_OUT.relative_to(REPO)}")
    print(f"wrote {LOCK.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
