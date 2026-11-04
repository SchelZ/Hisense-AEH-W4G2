"""
Hisense AEH-W4G2 bus-log decoder.

Reads an esphome debug log containing captured hex frames and prints each
frame in human-readable form, with per-field confidence markers.

Confidence tiers:
  [OK] CONFIRMED  - verified against reality (app readings, known constants)
  [~]  LIKELY     - strong evidence, probably right
  [?]  GUESS      - initial hypothesis, needs testing
  [??] UNKNOWN    - byte varies but meaning unknown

Flags (positional, order-insensitive):
    --all        : show every frame, no filtering
    --raw        : print raw hex alongside decoded fields
    --dedup N    : size of recent-frames dedup memory (default 20)

Usage:
    python3 hisense_decode.py ac-capture.log
    python3 hisense_decode.py ac-capture.log --raw
    python3 hisense_decode.py ac-capture.log --all

    # Live, filtered, saved:
    esphome logs ac-sniffer-esp32.yaml 2>&1 \
      | python3 -u hisense_decode.py - \
      | tee decoded-live.txt
"""

import re
import sys
from collections import deque
from pathlib import Path


# =============================================================================
#  Frame kinds
# =============================================================================
FRAME_KINDS = {
    0x0B: "short-poll",
    0x0C: "status-poll",
    0x0D: "unknown-0x0D",
    0x0F: "unknown-0x0F",
    0x13: "poll-response",
    0x23: "unknown-0x23",
    0x29: "command",
    0x7B: "full-state-dump",
}

BORING_KINDS = {0x0B, 0x0C, 0x13}

# -----------------------------------------------------------------------------
#  0x29 COMMAND frames (module -> AC, 50 bytes)
#  Each command touches ONE field at a time; others stay at 0.
# -----------------------------------------------------------------------------
COMMAND_BYTE_FIELDS = {
    13: ("header_marker",   None,          "C"),
    16: ("fan",             "fan_raw",     "C"),  # CONFIRMED cmd values: AUTO=0x01 LOW=0x0B MED-LOW=0x0D MED=0x0F MED-HIGH=0x11 HIGH=0x13
    17: ("sleep",           "sleep",       "C"),  # CONFIRMED: 0x01=OFF 0x03=GEN 0x05=OLD 0x07=YOUNG 0x09=KIDS
    18: ("mode_cmd",        "mode_cmd",    "C"),  # CONFIRMED: ((mode<<1)|1)<<4
    19: ("target_temp",     "temp_shifted","C"),
    23: ("buzzer",          "buzzer",      "C"),  # CONFIRMED: 0x04=BEEP, 0x00=MUTE (per-command flag)
    32: ("swing_main",      "swing1",      "C"),  # CONFIRMED: 0x40=V_OFF, 0xC0=V_ON. H and BOTH still unverified.
    33: ("eco_boost",       "byte33",      "C"),  # CONFIRMED: upper=ECO (0x10/0x30), lower=BOOST (0x04/0x0C)
    35: ("quiet",           "byte35_cmd",  "C"),  # CONFIRMED: 0x10=QUIET OFF, 0x30=QUIET ON
    36: ("display",         "display",     "L"),
    46: ("crc_hi",          None,          "C"),
    47: ("crc_lo",          None,          "C"),
}

# -----------------------------------------------------------------------------
#  0x7B FULL-STATE DUMP (AC -> module, 132 bytes)
# -----------------------------------------------------------------------------
FULLSTATE_BYTE_FIELDS = {
    15: ("fs_const_1",       None,       "C"),
    16: ("fs_fan_current",   "fan_raw",  "C"),  # CONFIRMED state values: AUTO=0x01 LOW=0x0A MED-LOW=0x0C MED=0x0E MED-HIGH=0x10 HIGH=0x12
    17: ("fs_sleep_current", "sleep",   "C"),  # CONFIRMED: 0x00=OFF 0x02=GEN 0x04=OLD 0x06=YOUNG 0x08=KIDS
    18: ("fs_mode_state",   "mode_state", "C"),  # CONFIRMED: upper=mode, lower bit 3=running. FAN=0x08 HEAT=0x18 COOL=0x28 DRY=0x38. AUTO=0x78 (sub-flag)
    19: ("fs_target_temp",  "temp_raw", "C"),  # CONFIRMED: raw degC. 23/24/25/26 match app changes exactly.
    20: ("fs_room_temp",     "temp_raw", "C"),  # CONFIRMED: indoor room temp in raw degC. Matches app reading.
    21: ("fs_counter",       None,       "G"),
    22: ("fs_const_0x80_a",  None,       "C"),
    23: ("fs_const_0x80_b",  None,       "C"),
    25: ("fs_proto_ver",     None,       "G"),
    26: ("fs_state_flags",   None,       "?"),
    35: ("fs_features_a",    "fs_status35", "C"),  # CONFIRMED bits: 0x80=V-swing, 0x04=ECO, 0x02=BOOST
    36: ("fs_features_b",    "fs_status36", "C"),  # CONFIRMED: 0x04=QUIET
    37: ("fs_const_0x80_c",  None,       "C"),
    38: ("fs_const_5",       None,       "C"),
    41: ("fs_byte41",        None,       "?"),  # varies — unknown
    42: ("fs_byte42",        None,       "?"),  # varies — unknown
    43: ("fs_byte43",        None,       "?"),  # varies — unknown
    44: ("fs_byte44?",       "temp_raw", "?"),  # 33-36 range; possibly 2nd temp sensor (pipe/outlet?)
    45: ("fs_byte45?",       "temp_raw", "?"),  # 29-33 range; candidate for actual room sensor
    46: ("fs_outdoor_cond",  "temp_raw", "L"),  # LIKELY outdoor condenser temp. 50-70degC during COOL confirms this is NOT indoor coil.
    50: ("fs_outdoor_raw",   None,       "?"),  # 218-220 stable. NOT byte/2. Try byte-192 or signed(byte-256).
    55: ("fs_compressor_state?", None,   "?"),  # 5-8 range; likely compressor state code
    56: ("fs_comp_freq_hz?", None,       "?"),  # 30-230; likely compressor frequency in Hz
    60: ("fs_power_deciamp?",None,       "?"),  # 12-22; likely current draw in 0.1A units
}


# =============================================================================
#  Decoders
# =============================================================================

FAN_NAMES_STATE = {
    1:  "AUTO",
    10: "LOW",
    12: "MED-LOW",
    14: "MEDIUM",
    16: "MED-HIGH",
    18: "HIGH",
}
FAN_NAMES_CMD = {
    1:  "AUTO",
    11: "LOW",
    13: "MED-LOW",
    15: "MEDIUM",
    17: "MED-HIGH",
    19: "HIGH",
}


def decode_fan_raw(b):
    """Fan byte. CONFIRMED from labeled capture:
      Command (0x29 byte[16]) uses ODD values: 0x01, 0x0B, 0x0D, 0x0F, 0x11, 0x13
      State   (0x7B byte[16]) uses EVEN values: 0x01, 0x0A, 0x0C, 0x0E, 0x10, 0x12
    Pattern: command_byte = state_byte | 1  (confirmation bit, like temp)
    """
    if b == 0:
        return "(unset)"
    if b in FAN_NAMES_STATE:
        return f"{FAN_NAMES_STATE[b]} state (0x{b:02X})"
    if b in FAN_NAMES_CMD:
        return f"{FAN_NAMES_CMD[b]} cmd (0x{b:02X})"
    return f"UNKNOWN raw={b} (0x{b:02X})"


def decode_temp_shifted(b):
    """Target temp in a 0x29 command byte: (temp<<1)|1. CONFIRMED."""
    if b == 0:
        return "(unset)"
    if not (b & 1):
        return f"invalid LSB=0 raw=0x{b:02X}"
    return f"{(b - 1) >> 1} degC (0x{b:02X})"


def decode_temp_x2(b):
    """Temperature in a 0x7B frame: value * 2.
    CONFIRMED: app showed 25 degC, byte read 50. degC = byte / 2.
    This gives 0.5 degC resolution."""
    if b == 0:
        return "(unset)"
    c = b / 2.0
    # Show as X.Y if there's a half-degree, else as integer
    if b % 2 == 0:
        return f"{int(c)} degC (0x{b:02X})"
    return f"{c:.1f} degC (0x{b:02X})"


def decode_sleep(b):
    """Sleep profile byte. CONFIRMED from labeled capture.
    Command (0x29 byte[17], ODD values with LSB=1 confirmation bit):
      0x01 = OFF
      0x03 = GENERAL
      0x05 = OLD
      0x07 = YOUNG
      0x09 = KIDS
    State (0x7B byte[17], EVEN values):
      0x00 = OFF
      0x02 = GENERAL
      0x04 = OLD
      0x06 = YOUNG
      0x08 = KIDS
    Pattern: command = state | 1 (same scheme as target_temp and fan).
    """
    if b == 0:
        return "OFF state (0x00)"
    cmd_map = {
        0x01: "OFF",
        0x03: "GENERAL",
        0x05: "OLD",
        0x07: "YOUNG",
        0x09: "KIDS",
    }
    state_map = {
        0x02: "GENERAL",
        0x04: "OLD",
        0x06: "YOUNG",
        0x08: "KIDS",
    }
    if b in cmd_map:
        return f"{cmd_map[b]} cmd (0x{b:02X})"
    if b in state_map:
        return f"{state_map[b]} state (0x{b:02X})"
    return f"UNKNOWN sleep (0x{b:02X})"


def decode_mode_cmd(b):
    """0x29 byte[18] mode byte.
    CONFIRMED formula: ((mode << 1) | 1) << 4
      mode=0 FAN_ONLY = 0x10
      mode=1 HEAT     = 0x30
      mode=2 COOL     = 0x50
      mode=3 DRY      = 0x70
      mode=4 AUTO     = 0x90
    Special values:
      0x04 = POWER_OFF marker
      0x0C = NO_CHANGE keepalive
    """
    if b == 0:
        return "(unset)"
    specials = {
        0x04: "POWER_OFF",
        0x0C: "NO_CHANGE (keepalive)",
    }
    if b in specials:
        return f"{specials[b]} (0x{b:02X})"
    mode_bytes = {
        0x10: "FAN_ONLY",
        0x30: "HEAT",
        0x50: "COOL",
        0x70: "DRY",
        0x90: "AUTO",
    }
    if b in mode_bytes:
        return f"{mode_bytes[b]} (0x{b:02X})"
    # Fallback — try the formula
    upper = (b >> 4) & 0xF
    if upper & 1:
        mode = (upper - 1) // 2
        names = {0: "FAN_ONLY?", 1: "HEAT?", 2: "COOL?", 3: "DRY?", 4: "AUTO?"}
        if mode in names:
            return f"{names[mode]} formula-match (0x{b:02X})"
    return f"UNKNOWN (0x{b:02X})"


def decode_swing1(b):
    """0x29 byte[32] swing command.
    CONFIRMED from labeled captures:
      0x40 = ALL SWING OFF (base)
      0xC0 = VERTICAL ON (base + 0x80)
    Hypothesis for H and BOTH based on bit-position pattern + reference repos:
      0x70 = HORIZONTAL ON? (base + 0x30)  [reference repo value]
      0xF0 = BOTH?           [reference repo value]
    Horizontal only reachable via IR remote on this AC, so command byte not
    captured directly. State byte[35] bit 6 (0x40) IS confirmed as H-active.
    """
    if b == 0:
        return "(unset)"
    known = {
        0x40: "SWING_OFF (confirmed)",
        0xC0: "VERTICAL_ON (confirmed)",
        0x70: "HORIZONTAL_ON? (ref, unverified)",
        0xF0: "BOTH? (ref, unverified)",
        0x50: "swing?-50 (ref)",
        0xD0: "swing?-D0 (ref)",
    }
    return f"{known.get(b, 'UNKNOWN')} (0x{b:02X})"


def decode_buzzer(b):
    """0x29 byte[23] buzzer control. CONFIRMED:
      0x04 = BEEP (AC audibly confirms the command)
      0x00 = MUTE (silent command, no beep)
    Per-command flag, not a persistent setting. Previously mis-labeled as
    "frame type constant" — our ESPHome component sends 0x04 by default so
    every command beeps. Set to 0x00 for silent operation.
    """
    if b == 0x04:
        return "BEEP (0x04)"
    if b == 0x00:
        return "MUTE (0x00)"
    return f"UNKNOWN (0x{b:02X})"


def decode_byte33(b):
    """0x29 byte[33]. CONFIRMED multi-feature byte on this AC:
      Upper bits control ECO: 0x10=OFF base, 0x30=ON (toggle bit 0x20)
      Lower bits control BOOST: 0x04=OFF base, 0x0C=ON (toggle bit 0x08)
    Can be combined: ECO+BOOST ON = 0x30 | 0x0C = 0x3C.
    """
    if b == 0:
        return "(unset)"
    parts = []
    # ECO portion (upper nibble)
    eco_bits = b & 0x30
    if eco_bits == 0x30:
        parts.append("ECO ON")
    elif eco_bits == 0x10:
        parts.append("ECO OFF")
    elif eco_bits != 0:
        parts.append(f"ECO?=0x{eco_bits:02X}")
    # BOOST portion (lower nibble)
    boost_bits = b & 0x0F
    if boost_bits == 0x0C:
        parts.append("BOOST ON")
    elif boost_bits == 0x04:
        parts.append("BOOST OFF")
    elif boost_bits != 0:
        parts.append(f"BOOST?=0x{boost_bits:02X}")
    rest = b & 0xC0
    if rest:
        parts.append(f"extra=0x{rest:02X}")
    return f"{' + '.join(parts)} (0x{b:02X})"


def decode_byte35_cmd(b):
    """0x29 byte[35]. CONFIRMED controls QUIET on this AC:
      0x10 = QUIET OFF (base)
      0x30 = QUIET ON  (base + toggle bit 0x20)
    """
    if b == 0:
        return "(unset)"
    if b == 0x30:
        return f"QUIET ON (0x{b:02X})"
    if b == 0x10:
        return f"QUIET OFF (0x{b:02X})"
    return f"UNKNOWN (0x{b:02X})"


def decode_fs_status35(b):
    """0x7B byte[35] feature-status bitfield. CONFIRMED bits:
      0x80 = Vertical swing active
      0x40 = Horizontal swing active (confirmed via IR-remote test)
      0x04 = ECO active
      0x02 = BOOST active
    """
    if b == 0:
        return "all OFF"
    parts = []
    if b & 0x80: parts.append("V-SWING")
    if b & 0x40: parts.append("H-SWING")
    if b & 0x04: parts.append("ECO")
    if b & 0x02: parts.append("BOOST")
    rest = b & 0x39
    if rest:
        parts.append(f"other=0x{rest:02X}")
    return f"{'+'.join(parts) if parts else 'unknown'} (0x{b:02X})"


def decode_fs_status36(b):
    """0x7B byte[36] feature-status bitfield.
    CONFIRMED: 0x04 = QUIET active.
    """
    if b == 0:
        return "(unset)"
    parts = []
    if b & 0x04: parts.append("QUIET")
    rest = b & 0xFB
    if rest:
        parts.append(f"other=0x{rest:02X}")
    return f"{'+'.join(parts) if parts else 'unknown'} (0x{b:02X})"


def decode_display(b):
    if b == 0:
        return "(unset)"
    if b == 0xC0:
        return "ON"
    if b == 0x40:
        return "OFF"
    return f"UNKNOWN 0x{b:02X}"


def decode_temp_raw(b):
    """Raw temperature byte in degC (no shift, no scaling)."""
    if b == 0:
        return "(unset)"
    # Handle negative (signed) values that may appear for outdoor temp
    signed = b if b < 128 else b - 256
    if signed == b:
        return f"{b} degC (0x{b:02X})"
    return f"{signed} degC signed (0x{b:02X})"


def decode_temp_raw_or_x2(b):
    """Show both raw-degC and byte/2 interpretations until we figure out which."""
    if b == 0:
        return "(unset)"
    raw = b
    half = b / 2.0
    if b % 2 == 0:
        half_s = f"{int(half)}"
    else:
        half_s = f"{half:.1f}"
    return f"raw={raw}degC OR /2={half_s}degC (0x{b:02X})"


def decode_mode_state(b):
    """0x7B byte[18] mode state. CONFIRMED mappings from labeled captures:
      0x08 = FAN_ONLY running
      0x10 = HEAT off/transitional
      0x18 = HEAT running
      0x28 = COOL running
      0x38 = DRY running
      0x78 = AUTO running (upper nibble 7, has auto sub-mode flag bit 6)
    Pattern: upper nibble = mode_enum (0-4) with bit 6 (0x40) added for AUTO.
    Lower nibble bit 3 (0x08) = running flag.
    """
    if b == 0:
        return "(unset / off)"
    # Direct lookups first
    known = {
        0x00: "OFF",
        0x08: "FAN_ONLY (running)",
        0x10: "HEAT (off/transitional)",
        0x18: "HEAT (running)",
        0x1A: "HEAT (running, swinging)",
        0x20: "COOL (off/transitional)",
        0x28: "COOL (running)",
        0x2A: "COOL (running, swinging)",
        0x30: "DRY (off/transitional)",
        0x38: "DRY (running)",
        0x3A: "DRY (running, swinging)",
        0x40: "AUTO (off/transitional)",
        0x48: "AUTO (running, no sub)",
        0x78: "AUTO (running, cool sub-mode)",
        0x7A: "AUTO (running, cool sub-mode, swinging)",
    }
    if b in known:
        return f"{known[b]} (0x{b:02X})"
    upper = (b >> 4) & 0xF
    lower = b & 0xF
    mode_names = {0: "FAN_ONLY", 1: "HEAT", 2: "COOL", 3: "DRY", 4: "AUTO"}
    # Strip the AUTO submode bit (0x40) when guessing mode
    base_upper = upper & 0x3
    name = mode_names.get(base_upper, f"UNK({base_upper})")
    flags = "running" if (lower & 0x8) else "idle"
    extras = []
    if upper & 0x4:
        extras.append("auto-sub")
    if lower & 0x1:
        extras.append("f0x1")
    if lower & 0x2:
        extras.append("f0x2")
    if lower & 0x4:
        extras.append("f0x4")
    extra_s = (" " + " ".join(extras)) if extras else ""
    return f"{name} ({flags}{extra_s}) (0x{b:02X})"


DECODERS = {
    "fan_raw":      decode_fan_raw,
    "temp_shifted": decode_temp_shifted,
    "temp_x2":      decode_temp_x2,
    "temp_raw":     decode_temp_raw,
    "temp_raw_or_x2": decode_temp_raw_or_x2,
    "mode_state":   decode_mode_state,
    "sleep":        decode_sleep,
    "mode_cmd":     decode_mode_cmd,
    "swing1":       decode_swing1,
    "fs_status35":  decode_fs_status35,
    "fs_status36":  decode_fs_status36,
    "byte33":       decode_byte33,
    "byte35_cmd":   decode_byte35_cmd,
    "buzzer":       decode_buzzer,
    "display":      decode_display,
}


# =============================================================================
#  Frame handling
# =============================================================================

FRAME_RE = re.compile(r"F4\s*F5(?:\s*[0-9A-Fa-f]{2})+?\s*F4\s*FB")
MARKER_RE = re.compile(r"^=+.*?=+$")

CONF_SYMBOL = {"C": "[OK]", "L": "[~]", "G": "[?]", "?": "[??]"}


def parse_hex_frame(hex_str):
    return [int(x, 16) for x in hex_str.split()]


def verify_crc(frame):
    if len(frame) < 6:
        return False, 0, 0
    body = frame[2:-4]
    calc = sum(body) & 0xFFFF
    recv = (frame[-4] << 8) | frame[-3]
    return calc == recv, calc, recv


def describe_frame(frame):
    out = []
    if len(frame) < 6:
        return "  (frame too short)"
    kind = frame[4]
    kind_name = FRAME_KINDS.get(kind, f"unknown-0x{kind:02X}")
    ok, calc, recv = verify_crc(frame)
    status = "CRC OK" if ok else f"CRC BAD (calc=0x{calc:04X} recv=0x{recv:04X})"
    out.append(f"  kind=0x{kind:02X} ({kind_name}) len={len(frame)} {status}")

    fields = None
    if kind == 0x29 and len(frame) >= 50:
        fields = COMMAND_BYTE_FIELDS
    elif kind == 0x7B and len(frame) >= 70:
        fields = FULLSTATE_BYTE_FIELDS

    if fields:
        for pos, (name, dec_key, conf) in fields.items():
            if pos >= len(frame):
                continue
            b = frame[pos]
            sym = CONF_SYMBOL.get(conf, "")
            if dec_key is None:
                if b != 0:
                    out.append(f"    byte[{pos}] {sym:4s} {name:20s} = 0x{b:02X}")
            else:
                decoded = DECODERS[dec_key](b)
                if "unset" in decoded and b == 0:
                    continue
                out.append(f"    byte[{pos}] {sym:4s} {name:20s} = {decoded}")
    return "\n".join(out)


# =============================================================================
#  Main
# =============================================================================

def iter_log_lines(path):
    if path == "-":
        for ln in sys.stdin:
            yield ln.rstrip("\n")
    else:
        with Path(path).open(errors="replace") as f:
            for ln in f:
                yield ln.rstrip("\n")


def main():
    args = sys.argv[1:]
    if not args:
        print("usage: hisense_decode.py <logfile|-> [--all] [--raw] "
              "[--dedup N]", file=sys.stderr)
        sys.exit(2)

    show_all = "--all" in args
    show_raw = "--raw" in args
    dedup_size = 20
    if "--dedup" in args:
        i = args.index("--dedup")
        dedup_size = int(args[i + 1])
        args.pop(i + 1); args.pop(i)
    for flag in ("--all", "--raw"):
        while flag in args:
            args.remove(flag)
    if not args:
        print("error: missing <logfile|->", file=sys.stderr)
        sys.exit(2)
    path = args[0]

    recent = deque(maxlen=dedup_size)
    seen = ok_count = shown = boring = dedup = 0

    for line in iter_log_lines(path):
        if MARKER_RE.match(line.strip()):
            print()
            print(line.strip(), flush=True)
            print()
            continue

        m = FRAME_RE.search(line.upper())
        if not m:
            continue
        hex_str = m.group(0)
        frame = parse_hex_frame(hex_str)
        seen += 1
        ok, _, _ = verify_crc(frame)
        if ok:
            ok_count += 1

        kind = frame[4] if len(frame) > 4 else 0

        if not show_all and kind in BORING_KINDS:
            boring += 1
            continue

        frame_tuple = tuple(frame)
        if not show_all and frame_tuple in recent:
            dedup += 1
            continue
        recent.append(frame_tuple)

        ts_m = re.search(r"\[(\d\d:\d\d:\d\d(?:\.\d+)?)\]", line)
        ts = ts_m.group(1) if ts_m else "?"

        if show_raw:
            print(f"[{ts}] {hex_str}", flush=True)
        else:
            print(f"[{ts}]", flush=True)
        print(describe_frame(frame), flush=True)
        print(flush=True)
        shown += 1

    print("---", file=sys.stderr)
    print(f"seen={seen}  crc_ok={ok_count}  shown={shown}  "
          f"skipped_boring={boring}  skipped_dedup={dedup}", file=sys.stderr)
    print("Legend: [OK]=confirmed  [~]=likely  [?]=guess  [??]=unknown",
          file=sys.stderr)


if __name__ == "__main__":
    main()
