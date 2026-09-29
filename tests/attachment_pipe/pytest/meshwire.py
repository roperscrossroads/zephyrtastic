# SPDX-License-Identifier: GPL-3.0
"""Meshtastic airframes and attachment envelopes, built in Python.

An independent implementation on purpose: the harness plays RF with frames the
firmware did not build, so a shared bug between builder and decoder cannot hide.
Sources: upstream firmware CryptoEngine::initNonce / Channels::generateHash, and
the port's meshtastic_attachment_codec.h.
"""

import struct

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# The default channel key ("AQ==", PSK index 1).
DEFAULT_KEY = bytes.fromhex("d4f1bb3a20290759f0bcffabcf4e6901")

# meshtastic_Config_LoRaConfig_ModemPreset
LONG_FAST, MEDIUM_FAST, SHORT_TURBO = 0, 4, 8
PRESET_NAMES = {LONG_FAST: "LongFast", MEDIUM_FAST: "MediumFast", SHORT_TURBO: "ShortTurbo"}

BROADCAST = 0xFFFFFFFF
PORT_TEXT = 1


def xor_bytes(data):
    h = 0
    for b in data:
        h ^= b
    return h


def channel_hash(name, key=DEFAULT_KEY):
    """The wire channel byte: xor(name) ^ xor(key). An unnamed slot uses the
    display name of the preset it is used on."""
    return xor_bytes(name.encode()) ^ xor_bytes(key)


def encrypt(key, packet_id, sender, plaintext):
    """AES-CTR with the nonce (id u64 LE, from u32 LE, 0 u32)."""
    nonce = struct.pack("<QII", packet_id, sender, 0)
    algo = algorithms.AES(key)
    enc = Cipher(algo, modes.CTR(nonce)).encryptor()
    return enc.update(plaintext) + enc.finalize()


def data_pb(portnum, payload):
    """meshtastic.Data {portnum = 1 (varint), payload = 2 (bytes)}."""
    assert portnum < 0x80 and len(payload) < 0x80
    return bytes([0x08, portnum, 0x12, len(payload)]) + payload


def airframe(sender, packet_id, text, *, dest=BROADCAST, preset=MEDIUM_FAST, hop_limit=3,
             hop_start=3, relay_node=None, name=None, key=DEFAULT_KEY):
    """A text broadcast as a stock node on @p preset would send it on its
    default (unnamed) channel."""
    ch = channel_hash(name if name is not None else PRESET_NAMES[preset], key)
    flags = (hop_limit & 7) | ((hop_start & 7) << 5)
    relay = (sender & 0xFF) if relay_node is None else relay_node
    hdr = struct.pack("<IIIBBBB", dest, sender, packet_id, flags, ch, 0, relay)
    return hdr + encrypt(key, packet_id, sender, data_pb(PORT_TEXT, text.encode()))


# ---- attachment envelopes (meshtastic_attachment_codec.h) -------------------

ENV_RX_FRAME, ENV_TX_FRAME, ENV_TX_RESULT, ENV_STATUS, ENV_SET_PRESET = 1, 2, 3, 4, 5


def env_rx_frame(preset, rssi, snr, wire, rx_ms=0, flags=0):
    return struct.pack("<BBhbIB", ENV_RX_FRAME, preset, rssi, snr, rx_ms, flags) + wire


# ---- where a stock node on a preset transmits (US, standard profile) ---------
# meshtastic_region_presets.c / upstream RadioInterface::applyModemConfig:
# slot = djb2(channel name) % num_slots, an unnamed channel named after its
# preset; frequency = start + bw/2 + slot * bw (no spacing or padding in US).

PRESET_MODEM = {LONG_FAST: (11, 250), MEDIUM_FAST: (9, 250), SHORT_TURBO: (7, 500)}
US_START_HZ, US_END_HZ = 902_000_000, 928_000_000


def djb2(s):
    h = 5381
    for c in s.encode():
        h = (h * 33 + c) & 0xFFFFFFFF
    return h


def us_tuning(preset, channel_name=None):
    """(freq_hz, sf, bw_khz) of a node on @p preset whose primary channel is
    @p channel_name (None: unnamed, i.e. named after the preset)."""
    sf, bw_khz = PRESET_MODEM[preset]
    width = bw_khz * 1000
    num_slots = (US_END_HZ - US_START_HZ + width // 2) // width
    slot = djb2(channel_name or PRESET_NAMES[preset]) % num_slots
    return US_START_HZ + width // 2 + slot * width, sf, bw_khz


# ---- the reference's relay window (RadioInterface.cpp), for X6 --------------
# computeSlotTimeMsec: max(2.25, NUM_SYM_CAD + 0.5) * 2^sf / bw + 7.6 ms, as an
# integer; getCWsize: Arduino map(snr, -20, 10, CWmin 3, CWmax 8);
# getTxDelayMsecWeighted: a ROUTER waits random(0, 2*CW) slots, everyone else
# 2*CWmax slots and then random(0, 2^CW) more (random's upper bound is
# exclusive).

NUM_SYM_CAD, CW_MIN, CW_MAX = 2, 3, 8


def slot_ms(preset):
    sf, bw_khz = PRESET_MODEM[preset]
    return int(max(2.25, NUM_SYM_CAD + 0.5) * (2 ** sf) / bw_khz + 7.6)


def cw_size(snr):
    v = int((snr + 20) * (CW_MAX - CW_MIN) / 30) + CW_MIN  # Arduino map: truncates
    return max(CW_MIN, min(CW_MAX, v))


def relay_window(preset, snr, router=False):
    """(earliest, latest) key-up after the frame was heard, in ms."""
    s, cw = slot_ms(preset), cw_size(snr)
    if router:
        return 0, (2 * cw - 1) * s
    return 2 * CW_MAX * s, 2 * CW_MAX * s + (2 ** cw - 1) * s


def relay_worst(preset, snr):
    """getTxDelayMsecWeightedWorst: 2*CWmax slots + 2^CW slots. What
    clampToLateRebroadcastWindow adds to the moment the duplicate is heard."""
    s = slot_ms(preset)
    return 2 * CW_MAX * s + (2 ** cw_size(snr)) * s
