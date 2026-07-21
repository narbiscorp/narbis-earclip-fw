#!/usr/bin/env python3
"""gen_proto_consts.py — cross-language protocol constant generator.

Single source of truth is the firmware header set:

    firmware/components/narbis_core/include/narbis/proto.h
    firmware/components/narbis_core/include/narbis/nc_types.h
    firmware/components/narbis_core/include/narbis/knob_list.h

This script regex-parses those headers (integer #defines, every typedef
enum, the rate/ODR Hz tables, the NC_UUID_BASE_BYTES array, the KNOB_LIST
X-macro) and emits tools/testapp/proto_consts.js: a plain-script-tag
`const PROTO = {...}` global (with a CommonJS export guard for node).
proto_consts.js is GENERATED AND COMMITTED — regenerate after any header
change; tools/tests/test_proto_consts.py fails if it goes stale.

It then verifies every shared constant against the Python mirror
(tools/narbis_client/proto.py + uuids.py) and exits non-zero on any
unwaived mismatch, so the three languages cannot drift silently.

Usage:
    python tools/goldens/gen_proto_consts.py           # write + verify
    python tools/goldens/gen_proto_consts.py --check   # verify only, fail if
                                                       # the committed JS is stale
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CORE_INC = REPO / "firmware" / "components" / "narbis_core" / "include" / "narbis"
PROTO_H = CORE_INC / "proto.h"
NC_TYPES_H = CORE_INC / "nc_types.h"
KNOB_LIST_H = CORE_INC / "knob_list.h"
OUT_JS = REPO / "tools" / "testapp" / "proto_consts.js"

# ------------------------------------------------------------------ #
# Known cross-mirror discrepancies we tolerate (loudly).              #
# name -> (header_value, python_value, note)                          #
# ------------------------------------------------------------------ #
WAIVED_PY_MISMATCH = {
    # proto.h: NC_KNOB_REC_FIXED = 21 (id2+type+flags+min/max/def/current16
    # +name_len1). proto.py ships 19; the constant is unused by its codec
    # logic (which walks _KNOB_REC_HEAD.size = 20 directly), so the wire
    # format agrees — only the informational constant is wrong. Owner of
    # narbis_client should fix proto.py to 21; then delete this waiver.
    "KNOB_REC_FIXED": (21, 19, "proto.py informational constant is stale"),
}

# Constants that exist only as comments in proto.h (payload bit maps /
# sentinels) but are part of the wire contract and present in proto.py.
# Value documented at: NC_OP_AGC_MANUAL comment (apply_mask bits) and the
# clock_drift_ppm_x10 field comment (0x7FFF = unknown).
DERIVED = {
    "AGC_APPLY_IR": 1 << 0,
    "AGC_APPLY_RED": 1 << 1,
    "AGC_APPLY_GAIN": 1 << 2,
    "CLOCK_DRIFT_UNKNOWN": 0x7FFF,
}

# SIG 16-bit ids mirrored by tools/narbis_client/uuids.py (not in proto.h;
# they are Bluetooth SIG assigned numbers). Cross-checked against uuids.py.
SIG_IDS = {
    "BATTERY_SVC": 0x180F,
    "DEVICE_INFO_SVC": 0x180A,
    "HEART_RATE_SVC": 0x180D,
    "BATTERY_LEVEL": 0x2A19,
    "HR_MEASUREMENT": 0x2A37,
    "MODEL_NUMBER": 0x2A24,
    "FIRMWARE_REVISION": 0x2A26,
    "HARDWARE_REVISION": 0x2A27,
    "SOFTWARE_REVISION": 0x2A28,
    "MANUFACTURER_NAME": 0x2A29,
}
SIG_UUID_FMT = "0000{:04x}-0000-1000-8000-00805f9b34fb"


# ------------------------------------------------------------------ #
# C header micro-parsers                                              #
# ------------------------------------------------------------------ #

def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"//[^\n]*", " ", text)
    return text


def _eval_int(expr: str, env: dict) -> int:
    """Evaluate a C integer constant expression (numbers, <<, |, +, ~, parens,
    previously-resolved macro names). Raises on anything else."""
    e = re.sub(r"(?<=[0-9a-fA-F])[uUlL]+\b", "", expr)  # 0x4E415242u -> ...42

    def sub_name(m: re.Match) -> str:
        name = m.group(0)
        if name in env:
            return str(env[name])
        raise KeyError(name)

    e = re.sub(r"\b[A-Za-z_]\w*\b", sub_name, e)
    if not re.fullmatch(r"[0-9xXa-fA-F()<>|&+\-~*\s]+", e):
        raise ValueError(expr)
    v = eval(e, {"__builtins__": {}})  # noqa: S307 — sanitized above
    if not isinstance(v, int):
        raise ValueError(expr)
    return v


def parse_defines(text: str) -> dict[str, int]:
    """All #define NAME <int-expr> in order; silently skips non-integer
    macros (NC_PACKED, NC_UUID_BASE_BYTES, function-like macros)."""
    out: dict[str, int] = {}
    # join backslash continuations so multi-line macros become one line
    joined = re.sub(r"\\\s*\n", " ", text)
    joined = strip_comments(joined)
    for m in re.finditer(r"^[ \t]*#define[ \t]+(\w+)[ \t]+([^\n]+)$",
                         joined, flags=re.M):
        name, rhs = m.group(1), m.group(2).strip()
        if "(" == name[-1:] or re.match(r"^\w+\(", m.group(0).split(None, 1)[1]):
            pass  # function-like: name captured w/o '(' can't happen with \w+
        try:
            out[name] = _eval_int(rhs, out)
        except Exception:
            continue  # not an integer constant — not ours to mirror
    return out


def parse_enums(text: str) -> dict[str, list[tuple[str, int]]]:
    """typedef enum { ... } tag_t; -> {tag: [(member, value), ...]} in order."""
    out: dict[str, list[tuple[str, int]]] = {}
    clean = strip_comments(text)
    for m in re.finditer(r"typedef\s+enum\s*\{(.*?)\}\s*(\w+)\s*;", clean, re.S):
        body, tag = m.group(1), m.group(2)
        members: list[tuple[str, int]] = []
        next_val = 0
        env: dict[str, int] = {}
        for entry in body.split(","):
            entry = entry.strip()
            if not entry:
                continue
            if "=" in entry:
                name, expr = (s.strip() for s in entry.split("=", 1))
                val = _eval_int(expr, env)
            else:
                name, val = entry, next_val
            members.append((name, val))
            env[name] = val
            next_val = val + 1
        out[tag] = members
    return out


def parse_uuid_base(text: str) -> str:
    """NC_UUID_BASE_BYTES (NimBLE little-endian byte array) -> the canonical
    UUID format string with {:04x} where the 16-bit alias goes."""
    joined = strip_comments(re.sub(r"\\\s*\n", " ", text))
    m = re.search(r"#define\s+NC_UUID_BASE_BYTES\s*\{([^}]*)\}", joined)
    if not m:
        raise SystemExit("NC_UUID_BASE_BYTES not found in proto.h")
    b = [int(x, 0) for x in m.group(1).split(",")]
    if len(b) != 16:
        raise SystemExit(f"NC_UUID_BASE_BYTES has {len(b)} bytes, want 16")
    big = bytes(reversed(b))  # LE array -> canonical big-endian text order
    h = big.hex()
    s = f"{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"
    # alias occupies LE bytes [12..13] == text chars [4..8) of group 1
    if s[4:8] != "0000":
        raise SystemExit(f"alias slot of UUID base is not zero: {s}")
    return s[:4] + "{:04x}" + s[8:]


def parse_hz_table(text: str, array_name: str, count_sym: str) -> list[int]:
    clean = strip_comments(text)
    m = re.search(
        rf"uint16_t\s+{array_name}\s*\[\s*{count_sym}\s*\]\s*=\s*\{{([^}}]*)\}}",
        clean)
    if not m:
        raise SystemExit(f"{array_name}[] table not found in nc_types.h")
    return [int(x.strip()) for x in m.group(1).split(",") if x.strip()]


def parse_status_size(text: str) -> int:
    m = re.search(r"_Static_assert\s*\(\s*sizeof\s*\(\s*nc_status_t\s*\)\s*==\s*(\d+)",
                  text)
    if not m:
        raise SystemExit("nc_status_t size static_assert not found")
    return int(m.group(1))


_KNOB_RE = re.compile(
    r'X\((\w+),\s*(0x[0-9A-Fa-f]+),\s*"([^"]*)",\s*NC_KNOB_(\w+),'
    r'\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*"([^"]*)",\s*(\w+)\)')


def parse_knobs(text: str, kf: dict[str, int]) -> list[dict]:
    """KNOB_LIST X-macro rows + the local KP/KPL/... flag shorthands."""
    flag_env = dict(kf)
    for m in re.finditer(r"#define\s+(KP\w*)\s+\(([^)]*)\)", text):
        flag_env[m.group(1)] = _eval_int(m.group(2), flag_env)
    type_codes = {"BOOL": 0, "U8": 1, "U16": 2, "I32": 3}
    knobs = []
    for m in _KNOB_RE.finditer(text):
        sym, kid, name, ktype, kmin, kmax, kdef, unit, flags = m.groups()
        knobs.append({
            "sym": sym, "id": int(kid, 16), "name": name,
            "type": type_codes[ktype], "min": int(kmin), "max": int(kmax),
            "def": int(kdef), "unit": unit, "flags": flag_env[flags],
        })
    if not knobs:
        raise SystemExit("no knobs parsed from knob_list.h")
    ids = [k["id"] for k in knobs]
    if len(set(ids)) != len(ids):
        raise SystemExit("duplicate knob ids in knob_list.h")
    for k in knobs:
        if not k["min"] <= k["def"] <= k["max"]:
            raise SystemExit(f"knob {k['name']} default out of range")
    return knobs


# ------------------------------------------------------------------ #
# Model assembly                                                      #
# ------------------------------------------------------------------ #

def strip_nc(name: str) -> str:
    return name[3:] if name.startswith("NC_") else name


def build_model() -> dict:
    proto_txt = PROTO_H.read_text(encoding="utf-8")
    types_txt = NC_TYPES_H.read_text(encoding="utf-8")
    knob_txt = KNOB_LIST_H.read_text(encoding="utf-8")

    defines: dict[str, int] = {}
    defines.update(parse_defines(types_txt))
    defines.update(parse_defines(proto_txt))
    enums: dict[str, list[tuple[str, int]]] = {}
    enums.update(parse_enums(types_txt))
    enums.update(parse_enums(proto_txt))

    # flatten: NC_ stripped, defines first then enum members, collision-checked
    flat: dict[str, int] = {}

    def put(name: str, val: int):
        key = strip_nc(name)
        if key in flat and flat[key] != val:
            raise SystemExit(f"flatten collision: {key} = {flat[key]} vs {val}")
        flat[key] = val

    for n, v in defines.items():
        if n.startswith("NC_"):
            put(n, v)
    for members in enums.values():
        for n, v in members:
            put(n, v)
    put("STATUS_SIZE", parse_status_size(proto_txt))
    for n, v in DERIVED.items():
        put(n, v)

    # rate / ODR tables indexed by their enum codes
    sps = parse_hz_table(types_txt, "sps", "NC_RATE_COUNT")
    hz = parse_hz_table(types_txt, "hz", "NC_ODR_COUNT")
    rate_members = [(n, v) for n, v in enums["nc_rate_t"] if not n.endswith("_COUNT")]
    odr_members = [(n, v) for n, v in enums["nc_acc_odr_t"] if not n.endswith("_COUNT")]
    if len(rate_members) != len(sps) or len(odr_members) != len(hz):
        raise SystemExit("rate/ODR table length != enum member count")
    rate_sps = {v: sps[v] for _, v in rate_members}
    odr_hz = {v: hz[v] for _, v in odr_members}

    # UUIDs
    base_fmt = parse_uuid_base(proto_txt)
    uuid_map = {strip_nc(n)[len("ALIAS_"):]: base_fmt.format(v)
                for n, v in defines.items() if n.startswith("NC_ALIAS_")}
    uuid_map.update({n: SIG_UUID_FMT.format(v) for n, v in SIG_IDS.items()})

    # value -> short name maps for UI logs (enum tag, prefix to strip, drops)
    def name_map(tag: str, prefix: str, drop: tuple[str, ...] = ()) -> dict[int, str]:
        out = {}
        for n, v in enums[tag]:
            short = strip_nc(n)
            if short in drop:
                continue
            assert short.startswith(prefix), (tag, n)
            out.setdefault(v, short[len(prefix):])
        return out

    return {
        "flat": flat,
        "rate_sps": rate_sps,
        "odr_hz": odr_hz,
        "uuid_base_fmt": base_fmt,
        "uuids": uuid_map,
        "knobs": parse_knobs(knob_txt,
                             {n: v for n, v in defines.items()
                              if n.startswith("NC_KF_")}),
        "op_name": name_map("nc_opcode_t", "OP_"),
        "ctrl_status_name": name_map("nc_ctrl_status_t", "ST_"),
        "event_type_name": name_map("nc_event_type_t", "EV_"),
        "selftest_id_name": name_map("nc_selftest_id_t", "TEST_", drop=("TEST_COUNT_",)),
        "test_status_name": name_map("nc_test_status_t", "TR_"),
        "error_code_name": name_map("nc_error_code_t", "ERR_"),
    }


# ------------------------------------------------------------------ #
# JS emission                                                         #
# ------------------------------------------------------------------ #

def _jsv(v: int) -> str:
    if v < 0:
        return str(v)
    return f"0x{v:02X}" if v > 9 else str(v)


def emit_js(model: dict) -> str:
    L: list[str] = []
    L.append("/* proto_consts.js — GENERATED by tools/goldens/gen_proto_consts.py")
    L.append(" * from proto.h / nc_types.h / knob_list.h. DO NOT EDIT BY HAND.")
    L.append(" * Regenerate: python tools/goldens/gen_proto_consts.py")
    L.append(" * Plain-script global (no modules) + CommonJS guard for node. */")
    L.append("/* eslint-disable */")
    L.append("const PROTO = {")
    L.append("  /* ---- constants (NC_ prefix stripped; names match proto.py) ---- */")
    for n, v in model["flat"].items():
        L.append(f"  {n}: {_jsv(v)},")
    L.append("")
    L.append("  /* ---- rate/ODR code -> Hz ---- */")
    L.append("  RATE_SPS: { " + ", ".join(f"{k}: {v}" for k, v in
                                          sorted(model["rate_sps"].items())) + " },")
    L.append("  ODR_HZ: { " + ", ".join(f"{k}: {v}" for k, v in
                                        sorted(model["odr_hz"].items())) + " },")
    L.append("")
    L.append("  /* ---- 128-bit UUID strings (lowercase, Web Bluetooth form) ---- */")
    L.append(f'  UUID_BASE_FMT: "{model["uuid_base_fmt"].replace("{:04x}", "%04x")}",')
    L.append("  UUID: {")
    for n, u in model["uuids"].items():
        L.append(f'    {n}: "{u}",')
    L.append("  },")
    L.append("")
    L.append("  /* ---- knob table (knob_list.h X-macro) ---- */")
    L.append("  KNOBS: [")
    for k in model["knobs"]:
        L.append(
            f'    {{ sym: "{k["sym"]}", id: 0x{k["id"]:04X}, name: "{k["name"]}", '
            f'type: {k["type"]}, min: {k["min"]}, max: {k["max"]}, '
            f'def: {k["def"]}, unit: "{k["unit"]}", flags: {k["flags"]} }},')
    L.append("  ],")
    L.append("")
    L.append("  /* ---- value -> name maps for logs/UI ---- */")
    for field, key in (("OP_NAME", "op_name"),
                       ("CTRL_STATUS_NAME", "ctrl_status_name"),
                       ("EVENT_TYPE_NAME", "event_type_name"),
                       ("SELFTEST_ID_NAME", "selftest_id_name"),
                       ("TEST_STATUS_NAME", "test_status_name"),
                       ("ERROR_CODE_NAME", "error_code_name")):
        pairs = ", ".join(f'{v}: "{s}"' for v, s in sorted(model[key].items()))
        L.append(f"  {field}: {{ {pairs} }},")
    L.append("};")
    L.append('if (typeof module !== "undefined" && module.exports) '
             "{ module.exports = { PROTO }; }")
    L.append("")
    return "\n".join(L)


# ------------------------------------------------------------------ #
# Verification against the Python mirror                              #
# ------------------------------------------------------------------ #

class VerifyResult:
    def __init__(self):
        self.matched: list[str] = []
        self.js_only: list[str] = []
        self.mismatched: list[tuple[str, int, object]] = []
        self.waived: list[tuple[str, int, object]] = []

    @property
    def ok(self) -> bool:
        return not self.mismatched


def verify_against_python(model: dict) -> VerifyResult:
    sys.path.insert(0, str(REPO / "tools"))
    from narbis_client import proto as P  # noqa: PLC0415
    from narbis_client import uuids as U  # noqa: PLC0415

    r = VerifyResult()

    def check(name: str, ours: int, theirs: object):
        if theirs == ours:
            r.matched.append(name)
        elif name in WAIVED_PY_MISMATCH and \
                WAIVED_PY_MISMATCH[name][:2] == (ours, theirs):
            r.waived.append((name, ours, theirs))
        else:
            r.mismatched.append((name, ours, theirs))

    for name, val in model["flat"].items():
        pyv = getattr(P, name, None)
        if isinstance(pyv, bool) or not isinstance(pyv, int):
            r.js_only.append(name)
        else:
            check(name, val, pyv)

    check("RATE_SPS{}", model["rate_sps"], P.RATE_SPS)
    check("ODR_HZ{}", model["odr_hz"], P.ODR_HZ)

    # UUIDs: alias values + full strings vs uuids.py
    for name, uuid in model["uuids"].items():
        if name in SIG_IDS:
            check(f"UUID.{name}", uuid, getattr(U, f"UUID_{name}", None))
        else:
            alias = model["flat"][f"ALIAS_{name}"]
            check(f"uuids.ALIAS_{name}", alias, getattr(U, f"ALIAS_{name}", None))
            check(f"UUID.{name}", uuid, U.narbis_uuid(alias))
    return r


def main(argv: list[str]) -> int:
    check_only = "--check" in argv
    model = build_model()
    js = emit_js(model)
    res = verify_against_python(model)

    n_knobs = len(model["knobs"])
    print(f"gen_proto_consts: parsed {len(model['flat'])} constants, "
          f"{n_knobs} knobs, {len(model['uuids'])} UUIDs, "
          f"{len(model['op_name'])} opcodes")
    print(f"verify vs proto.py/uuids.py: {len(res.matched)} matched, "
          f"{len(res.js_only)} JS-only, {len(res.waived)} waived, "
          f"{len(res.mismatched)} MISMATCHED")
    for name, ours, theirs in res.waived:
        note = WAIVED_PY_MISMATCH[name][2]
        print(f"  WAIVED  {name}: header={ours} python={theirs}  ({note})")
    for name, ours, theirs in res.mismatched:
        print(f"  MISMATCH {name}: header={ours} python={theirs}")

    if check_only:
        on_disk = OUT_JS.read_text(encoding="utf-8") if OUT_JS.exists() else ""
        if on_disk != js:
            print(f"STALE: {OUT_JS} does not match headers — regenerate")
            return 2
        print(f"{OUT_JS.name}: up to date")
    else:
        OUT_JS.parent.mkdir(parents=True, exist_ok=True)
        OUT_JS.write_text(js, encoding="utf-8", newline="\n")
        print(f"wrote {OUT_JS} ({len(js)} bytes)")

    return 0 if res.ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
