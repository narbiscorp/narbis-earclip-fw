"""Pytest wrapper around tools/goldens/gen_proto_consts.py.

Guards two invariants:
  1. the committed tools/testapp/proto_consts.js is byte-identical to what
     the current firmware headers generate (no stale generated file);
  2. every constant shared between the headers and the Python mirror
     (proto.py / uuids.py) agrees — the same check the generator script
     runs standalone, so CI and the command line cannot disagree.

Run from the repo root:  python -m pytest tools/tests -q
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "goldens"))
sys.path.insert(0, str(REPO / "tools"))

import gen_proto_consts as G  # noqa: E402


def _model():
    return G.build_model()


def test_generated_js_is_current():
    js = G.emit_js(_model())
    on_disk = G.OUT_JS.read_text(encoding="utf-8")
    assert on_disk == js, (
        "proto_consts.js is stale — run: python tools/goldens/gen_proto_consts.py")


def test_python_mirror_agrees():
    res = G.verify_against_python(_model())
    assert res.mismatched == [], f"cross-language drift: {res.mismatched}"
    # the waiver list must not rot: every waiver must still be exercised,
    # else it is stale and must be deleted.
    assert {w[0] for w in res.waived} == set(G.WAIVED_PY_MISMATCH), (
        "stale entries in WAIVED_PY_MISMATCH — the underlying file was fixed; "
        "delete the waiver")
    # sanity floor so a regex regression can't silently verify nothing
    assert len(res.matched) > 150


def test_spot_checks():
    m = _model()
    flat = m["flat"]
    assert flat["PROTO_VER"] == 0x0101
    assert flat["OP_TEST_SELFTEST_ONE"] == 0xE0
    assert flat["OP_TEST_REPORT"] == 0xEA
    assert flat["FACTORY_MAGIC"] == 0x4E415242
    assert flat["STATUS_SIZE"] == 32
    assert flat["ATT_PAYLOAD_MAX"] == 244
    assert m["rate_sps"] == {0: 50, 1: 100, 2: 200, 3: 250, 4: 500}
    assert m["odr_hz"][5] == 400
    assert m["uuids"]["SENSOR_SVC"] == "a5e90100-c6a0-43c0-b0d0-6e6172626973"
    assert m["uuids"]["CONTROL"] == "a5e90106-c6a0-43c0-b0d0-6e6172626973"
    assert m["uuids"]["BATTERY_SVC"] == "0000180f-0000-1000-8000-00805f9b34fb"
    # knob table shape
    ids = [k["id"] for k in m["knobs"]]
    assert len(ids) == len(set(ids))
    assert any(k["name"] == "st_xtalk_max" for k in m["knobs"])
    # every TEST opcode 0xE0..0xEA is present and named
    for op in range(0xE0, 0xEB):
        assert op in m["op_name"], f"missing TEST opcode 0x{op:02X}"
