"""Round-trip + cross-language fixtures for tools/narbis_client/proto.py.

Mirrors test_host/tests/t_proto_roundtrip.c. The golden_* builders below
reproduce the FIXTURE SEEDS documented at the top of that file; the C
suite writes test_host/out/fixtures/c_<name>.bin from those seeds and
byte-compares our py_<name>.bin twins. A c_/py_ pair with the same
<name> must be byte-identical.

Run from the repo root:  python -m pytest tools/tests -q
"""

import re
import sys
import zlib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client import proto as P  # noqa: E402
from narbis_client import uuids as U  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
FIXTURE_DIR = REPO / "test_host" / "out" / "fixtures"
KNOB_LIST_H = (REPO / "firmware" / "components" / "narbis_core" /
               "include" / "narbis" / "knob_list.h")


# ------------------------------------------------------------------ #
# Shared deterministic fixture seeds (t_proto_roundtrip.c header)      #
# ------------------------------------------------------------------ #

def golden_ppg_noamb():
    n = P.PPG_MAX_N_NOAMB  # 28
    return P.PpgBatch(
        seq=1, t0_us=1_000_000, rate_code=1, flags=0x41,
        ir=[100000 + 1111 * i for i in range(n)],
        red=[-100000 - 2222 * i for i in range(n)])


def golden_ppg_amb():
    n = P.PPG_MAX_N_AMB  # 19
    return P.PpgBatch(
        seq=2, t0_us=2_000_000, rate_code=4, flags=P.PPGF_AMB,
        ir=[200000 + 333 * i for i in range(n)],
        red=[-200000 + 444 * i for i in range(n)],
        amb=[40000 + 555 * i for i in range(n)])


def golden_accel():
    n = P.ACCEL_MAX_N  # 38
    return P.AccelBatch(
        seq=3, t0_us=3_000_000, odr_code=2, flags=0x01,  # FS +/-4g
        samples=[(-16000 + 900 * i, 16000 - 900 * i, -100 * i)
                 for i in range(n)])


def golden_ibi():
    n = P.IBI_MAX_N  # 19
    return P.IbiBatch(seq=4, records=[
        P.IbiRecord(t_beat_us=4_000_000 + 800_000 * i,
                    ibi_ms=800 + 7 * i,
                    confidence=(5 * i) % 101,
                    flags=i & 0x0F)
        for i in range(n)])


def golden_event_all():
    # One record per type in enum order, t_us = 5000000 + 1000*i,
    # data bytes per fix_events[] in the C suite.
    t = lambda i: 5_000_000 + 1000 * i  # noqa: E731
    return P.EventBatch(seq=5, events=[
        P.EvAgcStep(t(0), led=0, old_ma=10, new_ma=12, old_rf=5, new_rf=5),
        P.EvGate(t(1), state=1, reason_mask=0x05),
        P.EvWear(t(2), worn=1),
        P.EvMarker(t(3), source=1, marker_id=0x0102),
        P.EvError(t(4), code=0x0006, arg=0xDEADBEEF),
        P.EvRateChange(t(5), old_code=1, new_code=4),
        P.EvAgcOffdac(t(6), phase=2, old_code=3, new_code=7),
        P.EvSelftestDone(t(7), pass_count=7, fail_count=1),
    ])


def golden_status():
    return P.Status(
        sys_state=P.STATE_STREAMING,
        flags=P.STF_CHARGING | P.STF_USB | P.STF_WORN | P.STF_HRS_ACTIVE,  # 0x4D
        batt_mv=3987, batt_pct=76, ppg_rate_code=1,
        led_ir_ma=12, led_red_ma=8, tia_gain_code=4, tia_cf_code=2,
        gate_duty_x100=123, notif_drop_count=42, i2c_err_count=7,
        clock_drift_ppm_x10=-153, uptime_s=3600, ibi_last_ms=812, hr_bpm=74)


# --- knob discovery chunk 0: derived from knob_list.h defaults ------- #

_KNOB_TYPE = {"BOOL": P.KNOB_BOOL, "U8": P.KNOB_U8,
              "U16": P.KNOB_U16, "I32": P.KNOB_I32}
_KNOB_FLAGS = {"KP": P.KF_PERSIST,
               "KPL": P.KF_PERSIST | P.KF_LIVE,
               "KPS": P.KF_PERSIST | P.KF_RESTREAM,
               "KPR": P.KF_PERSIST | P.KF_REBOOT}
_KNOB_RE = re.compile(
    r'X\(\w+,\s*(0x[0-9A-Fa-f]+),\s*"([^"]*)",\s*NC_KNOB_(\w+),'
    r'\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*"([^"]*)",\s*(\w+)\)')


def load_knob_descriptors():
    recs = []
    for m in _KNOB_RE.finditer(KNOB_LIST_H.read_text()):
        kid, name, ktype, kmin, kmax, kdef, unit, flags = m.groups()
        recs.append(P.KnobRecord(
            id=int(kid, 16), type=_KNOB_TYPE[ktype], flags=_KNOB_FLAGS[flags],
            min=int(kmin), max=int(kmax), default=int(kdef),
            current=int(kdef),  # fixture is over defaults
            name=name, unit=unit))
    return recs


def golden_knob_disc_chunk0():
    """First discovery chunk: whole records greedily packed into the 244-byte
    ATT budget, current == default, start index 0."""
    knobs = load_knob_descriptors()
    fit = []
    used = P.KNOB_DISC_HDR_SIZE
    for r in knobs:
        rec_len = 22 + len(r.name) + len(r.unit)  # 20 fixed + 2 len bytes
        if used + rec_len > P.ATT_PAYLOAD_MAX:
            break
        used += rec_len
        fit.append(r)
    return P.KnobDiscoverChunk(total=len(knobs), first_idx=0, records=fit)


GOLDEN_BYTES = {
    "ppg_noamb": lambda: P.build_ppg(golden_ppg_noamb()),
    "ppg_amb": lambda: P.build_ppg(golden_ppg_amb()),
    "accel": lambda: P.build_accel(golden_accel()),
    "ibi": lambda: P.build_ibi(golden_ibi()),
    "event_all": lambda: P.build_event_batch(golden_event_all()),
    "status": lambda: P.build_status(golden_status()),
    "knob_disc_chunk0": lambda: P.build_knob_discover_chunk(
        golden_knob_disc_chunk0()),
}

GOLDEN_PARSED = {
    "ppg_noamb": (P.parse_ppg, golden_ppg_noamb),
    "ppg_amb": (P.parse_ppg, golden_ppg_amb),
    "accel": (P.parse_accel, golden_accel),
    "ibi": (P.parse_ibi, golden_ibi),
    "event_all": (P.parse_event_batch, golden_event_all),
    "status": (P.parse_status, golden_status),
    "knob_disc_chunk0": (P.parse_knob_discover_chunk, golden_knob_disc_chunk0),
}

# Cross-fixture re-encode identity for any c_*.bin we did not anticipate:
# parse then rebuild must reproduce the exact bytes, whatever the values.
REENCODERS = {
    "ppg": lambda b: P.build_ppg(P.parse_ppg(b)),
    "accel": lambda b: P.build_accel(P.parse_accel(b)),
    "ibi": lambda b: P.build_ibi(P.parse_ibi(b)),
    "event": lambda b: P.build_event_batch(P.parse_event_batch(b)),
    "status": lambda b: P.build_status(P.parse_status(b)),
    # "knob_disc" must outrank "chunk" ("knob_disc_chunk0" contains both)
    "knob_disc": lambda b: P.build_knob_discover_chunk(
        P.parse_knob_discover_chunk(b)),
    "selftest": lambda b: P.build_selftest_blob(P.parse_selftest_blob(b)),
    "chunk": lambda b: P.build_chunk(P.parse_chunk(b)),
    "ctrl_resp": lambda b: (lambda r: P.build_control_response(
        r.op, r.tid, r.status, r.payload))(P.parse_control_response(b)),
    "ctrl_req": lambda b: P.build_control_request(b[0], b[1], b[2:]),
    "ota_data": lambda b: P.build_ota_data(*P.parse_ota_data(b)),
}


def reencoder_for(stem: str):
    # Longest-key-first so "ctrl_resp" wins over "ctrl", "knob" over "ppg"...
    for key in sorted(REENCODERS, key=len, reverse=True):
        if key in stem:
            return REENCODERS[key]
    return None


# ------------------------------------------------------------------ #
# (2) CRC sanity — firmware nc_crc32 is standard CRC-32 (zlib)         #
# ------------------------------------------------------------------ #

def test_crc32_check_vector():
    assert zlib.crc32(b"123456789") == 0xCBF43926


# ------------------------------------------------------------------ #
# (1) Self round-trips                                                #
# ------------------------------------------------------------------ #

def test_ppg_roundtrip_noamb_max():
    b = golden_ppg_noamb()
    wire = P.build_ppg(b)
    assert len(wire) == 239 <= P.ATT_PAYLOAD_MAX  # 15 + 28*8, proto.h comment
    assert P.parse_ppg(wire) == b


def test_ppg_roundtrip_amb_max():
    b = golden_ppg_amb()
    wire = P.build_ppg(b)
    assert len(wire) == 243 <= P.ATT_PAYLOAD_MAX  # 15 + 19*12
    assert P.parse_ppg(wire) == b


def test_ppg_golden_bytes():
    # Hand-computed wire image locks the layout, not just symmetry.
    wire = P.build_ppg(P.PpgBatch(
        seq=1, t0_us=1_000_000, rate_code=P.RATE_500, flags=P.PPGF_AMB,
        ir=[1, 2], red=[3, 4], amb=[5, 6]))
    assert wire == bytes.fromhex(
        "01000000" "40420f0000000000" "04" "02" "10"
        "010000000300000005000000" "020000000400000006000000")


def test_ppg_roundtrip_empty():
    b = P.PpgBatch(seq=0xFFFFFFFF, t0_us=0xFFFFFFFFFFFFFFFF,
                   rate_code=P.RATE_50, flags=0, ir=[], red=[])
    wire = P.build_ppg(b)
    assert len(wire) == P.PPG_HDR_SIZE
    assert P.parse_ppg(wire) == b


def test_ppg_build_rejects_bad_shapes():
    with pytest.raises(ValueError):
        P.build_ppg(P.PpgBatch(0, 0, 0, 0, ir=[1] * 29, red=[1] * 29))
    with pytest.raises(ValueError):
        P.build_ppg(P.PpgBatch(0, 0, 0, P.PPGF_AMB, ir=[1] * 20, red=[1] * 20,
                               amb=[1] * 20))
    with pytest.raises(ValueError):  # amb list without the flag
        P.build_ppg(P.PpgBatch(0, 0, 0, 0, ir=[1], red=[1], amb=[1]))
    with pytest.raises(ValueError):  # flag without the amb list
        P.build_ppg(P.PpgBatch(0, 0, 0, P.PPGF_AMB, ir=[1], red=[1]))


def test_ppg_parse_rejects_bad_lengths():
    wire = P.build_ppg(golden_ppg_noamb())
    for bad in (wire[:-1], wire + b"\x00", wire[:P.PPG_HDR_SIZE - 1]):
        with pytest.raises(ValueError):
            P.parse_ppg(bad)


def test_accel_roundtrip():
    b = golden_accel()
    wire = P.build_accel(b)
    assert len(wire) == 243 <= P.ATT_PAYLOAD_MAX  # 15 + 38*6
    assert P.parse_accel(wire) == b

    empty = P.AccelBatch(0, 0, P.ODR_10, 0, [])
    assert P.parse_accel(P.build_accel(empty)) == empty

    extremes = P.AccelBatch(1, 2, P.ODR_400, P.ACCF_FIFO_OVERRUN | 0x03,
                            [(-32768, 32767, 0)])
    assert P.parse_accel(P.build_accel(extremes)) == extremes

    with pytest.raises(ValueError):
        P.build_accel(P.AccelBatch(0, 0, 0, 0, [(0, 0, 0)] * (P.ACCEL_MAX_N + 1)))
    with pytest.raises(ValueError):
        P.parse_accel(P.build_accel(b) + b"\x00")


def test_ibi_roundtrip():
    b = golden_ibi()
    wire = P.build_ibi(b)
    assert len(wire) == 233 <= P.ATT_PAYLOAD_MAX  # 5 + 19*12
    assert P.parse_ibi(wire) == b

    empty = P.IbiBatch(0, [])
    assert P.parse_ibi(P.build_ibi(empty)) == empty

    with pytest.raises(ValueError):
        P.build_ibi(P.IbiBatch(0, [P.IbiRecord(0, 0, 0, 0)] * (P.IBI_MAX_N + 1)))
    with pytest.raises(ValueError):
        P.parse_ibi(wire[:-1])


def test_event_roundtrip_all_types():
    batch = golden_event_all()
    wire = P.build_event_batch(batch)
    expect = P.EVENT_HDR_SIZE + sum(
        2 + ln for ln in (P.EVLEN_AGC_STEP, P.EVLEN_GATE, P.EVLEN_WEAR,
                          P.EVLEN_MARKER, P.EVLEN_ERROR, P.EVLEN_RATE_CHANGE,
                          P.EVLEN_AGC_OFFDAC, P.EVLEN_SELFTEST_DONE))
    assert len(wire) == expect
    assert P.parse_event_batch(wire) == batch

    empty = P.EventBatch(1, [])
    assert P.parse_event_batch(P.build_event_batch(empty)) == empty


def test_event_unknown_type_is_skippable():
    unk = P.EvUnknown(type=0x7F, payload=bytes.fromhex("8813000000000000aa"),
                      t_us=5000)
    known = golden_event_all().events[:3]
    batch = P.EventBatch(9, [known[0], unk, known[1], known[2]])
    parsed = P.parse_event_batch(P.build_event_batch(batch))
    assert parsed == batch
    assert isinstance(parsed.events[1], P.EvUnknown)
    assert parsed.events[1].t_us == 5000
    # records after the unknown record parse normally
    assert parsed.events[2:] == known[1:3]


def test_event_known_type_with_wrong_len_degrades_to_unknown():
    # A hypothetical v2 firmware appending a field to GATE must not break us.
    payload = bytes.fromhex("d007000000000000" "01" "03" "ff")  # len 11 != 10
    wire = P.build_event_batch(P.EventBatch(0, [P.EvUnknown(P.EV_GATE, payload)]))
    ev = P.parse_event_batch(wire).events[0]
    assert isinstance(ev, P.EvUnknown)
    assert ev.t_us == 2000 and ev.payload == payload


def test_event_truncation_rejected():
    wire = P.build_event_batch(golden_event_all())
    for bad in (wire[:-1], wire + b"\x00", wire[:3]):
        with pytest.raises(ValueError):
            P.parse_event_batch(bad)


def test_status_golden_bytes_and_roundtrip():
    s = golden_status()
    wire = P.build_status(s)
    assert len(wire) == P.STATUS_SIZE
    assert wire == bytes.fromhex(
        "024d930f4c010c0804027b002a00000007006"
        "7ff100e00002c034a0000000000")
    assert P.parse_status(wire) == s


def test_status_append_only():
    wire = P.build_status(golden_status()) + b"\xaa"
    s = P.parse_status(wire)
    assert s.extra == b"\xaa" and s.hr_bpm == 74
    assert P.build_status(s) == wire  # re-encode keeps the tail
    with pytest.raises(ValueError):
        P.parse_status(wire[:31])


def test_control_roundtrip_and_builders():
    reqs = [
        (P.build_stream_start(1, P.STREAM_MASK_PPG | P.STREAM_MASK_EVENT),
         P.OP_STREAM_START, b"\x09"),
        (P.build_stream_stop(2, 0x0F), P.OP_STREAM_STOP, b"\x0f"),
        (P.build_set_rate(3, P.RATE_250), P.OP_SET_RATE, b"\x03"),
        (P.build_knob_get(4, 0x0301), P.OP_KNOB_GET, b"\x01\x03"),
        (P.build_knob_set(5, 0x0405, -2), P.OP_KNOB_SET,
         b"\x05\x04\xfe\xff\xff\xff"),
        (P.build_knob_save(6), P.OP_KNOB_SAVE, b""),
        (P.build_knob_reset(7, 1), P.OP_KNOB_RESET, b"\x01"),
        (P.build_knob_discover(8, 40), P.OP_KNOB_DISCOVER, b"\x28\x00"),
        (P.build_time_sync(9, 2**63), P.OP_TIME_SYNC,
         b"\x00\x00\x00\x00\x00\x00\x00\x80"),
        (P.build_get_time(10), P.OP_GET_TIME, b""),
        (P.build_marker(11, 0xBEEF), P.OP_MARKER, b"\xef\xbe"),
        (P.build_agc_freeze(12, True), P.OP_AGC_FREEZE, b"\x01"),
        (P.build_agc_manual(13, 20, 15, 4, P.AGC_APPLY_IR | P.AGC_APPLY_GAIN),
         P.OP_AGC_MANUAL, b"\x14\x0f\x04\x05"),
        (P.build_selftest_run(14, 0), P.OP_SELFTEST_RUN, b"\x00" * 4),
        (P.build_selftest_result(15, 100), P.OP_SELFTEST_RESULT, b"\x64\x00"),
        (P.build_enter_ota(16), P.OP_ENTER_OTA, b""),
        (P.build_power_off(17), P.OP_POWER_OFF, b""),
        (P.build_reboot(18), P.OP_REBOOT, b""),
        (P.build_factory_reset(19), P.OP_FACTORY_RESET, b"\x42\x52\x41\x4e"),
    ]
    for i, (wire, op, payload) in enumerate(reqs):
        assert wire[0] == op and wire[1] == i + 1 and wire[2:] == payload

    resp = P.parse_control_response(
        P.build_control_response(P.OP_KNOB_GET, 7, P.ST_OK,
                                 b"\x01\x03\x01\x00\x00\x00"))
    assert (resp.op, resp.tid, resp.status) == (P.OP_KNOB_GET, 7, P.ST_OK)
    assert P.parse_knob_get_resp(resp.payload) == (0x0301, 1)

    # TEST-block opcodes already carry bit7; response op is unchanged.
    tresp = P.parse_control_response(
        P.build_control_response(P.OP_TEST_BATT_RAW, 1, P.ST_OK,
                                 b"\x40\x0f\x00\x08"))
    assert tresp.op == P.OP_TEST_BATT_RAW

    with pytest.raises(ValueError):  # request byte is not a response
        P.parse_control_response(P.build_knob_get(1, 1))
    with pytest.raises(ValueError):
        P.parse_control_response(b"\x90\x01")  # too short

    echo, dev = P.parse_time_sync_resp(bytes(range(16)))
    assert echo == 0x0706050403020100 and dev == 0x0F0E0D0C0B0A0908
    assert P.parse_get_time_resp(b"\x01" + b"\x00" * 7) == 1


def test_knob_discover_chunk_roundtrip():
    chunk = golden_knob_disc_chunk0()
    # knob_list.h: 62 knobs; records 0..5 (225 bytes) fit the 244 budget.
    assert chunk.total == 62
    assert len(chunk.records) == 6
    wire = P.build_knob_discover_chunk(chunk)
    assert len(wire) == 225 <= P.ATT_PAYLOAD_MAX
    parsed = P.parse_knob_discover_chunk(wire)
    assert parsed == chunk
    assert parsed.records[0].name == "idle_timeout_s"
    assert parsed.records[0].current == 300
    assert parsed.records[2].unit == ""  # motion_wake_en is dimensionless

    empty = P.KnobDiscoverChunk(total=62, first_idx=62, records=[])
    assert P.parse_knob_discover_chunk(P.build_knob_discover_chunk(empty)) == empty

    with pytest.raises(ValueError):
        P.parse_knob_discover_chunk(wire[:-1])


def test_chunk_envelope_roundtrip():
    c = P.Chunk(total=1234, offset=200, data=bytes(range(100)))
    assert P.parse_chunk(P.build_chunk(c)) == c
    empty = P.Chunk(total=0, offset=0, data=b"")
    assert P.parse_chunk(P.build_chunk(empty)) == empty
    with pytest.raises(ValueError):
        P.parse_chunk(P.build_chunk(c)[:-1])  # n disagrees with data length


def test_selftest_blob_roundtrip():
    blob = P.SelftestBlob(P.ST_BLOB_VER, 9_999_999, [
        P.SelftestRecord(P.TEST_I2C_SCAN, P.TR_PASS, 2, 2),
        P.SelftestRecord(P.TEST_AFE_DARK, P.TR_FAIL, 250, 200),
        P.SelftestRecord(P.TEST_CHARGER, P.TR_SKIP, 0, 0),
    ])
    wire = P.build_selftest_blob(blob)
    assert len(wire) == 10 + 3 * P.ST_REC_SIZE
    assert P.parse_selftest_blob(wire) == blob

    empty = P.SelftestBlob(P.ST_BLOB_VER, 0, [])
    assert P.parse_selftest_blob(P.build_selftest_blob(empty)) == empty

    full = P.SelftestBlob(P.ST_BLOB_VER, 1, [
        P.SelftestRecord(i, P.TR_PASS, -i * 1000, i * 1000)
        for i in range(1, P.TEST_COUNT_ + 1)])
    assert P.parse_selftest_blob(P.build_selftest_blob(full)) == full

    # blob arrives via SELFTEST_RESULT chunk envelopes: reassemble and parse
    half = len(wire) // 2
    chunks = [P.Chunk(len(wire), 0, wire[:half]),
              P.Chunk(len(wire), half, wire[half:])]
    blob2 = b"".join(c.data for c in
                     (P.parse_chunk(P.build_chunk(c)) for c in chunks))
    assert P.parse_selftest_blob(blob2) == blob


def test_ota_roundtrip():
    req = P.build_ota_begin(9, 0x00030000, 0xCBF43926, "1.2.3+g1234abc")
    assert req[0] == P.OTA_BEGIN and req[1] == 9
    assert P.parse_ota_begin_payload(req[2:]) == \
        (0x00030000, 0xCBF43926, "1.2.3+g1234abc")

    assert P.parse_ota_begin_resp(b"\x00\x10\x00\x00") == 0x1000

    st = P.parse_ota_status_resp(bytes.fromhex("01" "00e10000" "0100"))
    assert st == P.OtaStatus(P.OTA_RECEIVING, 0xE100, P.OTAERR_EXPECTED_OFFSET)

    data = bytes(range(240))
    off, payload = P.parse_ota_data(P.build_ota_data(0x20000, data))
    assert off == 0x20000 and payload == data
    with pytest.raises(ValueError):
        P.build_ota_data(0, bytes(P.OTA_CHUNK_MAX + 1))


def test_uuid_construction():
    assert U.UUID_SENSOR_SVC == "a5e90100-c6a0-43c0-b0d0-6e6172626973"
    assert U.UUID_CONTROL == "a5e90106-c6a0-43c0-b0d0-6e6172626973"
    assert U.UUID_OTA_DATA == "a5e90202-c6a0-43c0-b0d0-6e6172626973"
    assert U.UUID_BATTERY_SVC == "0000180f-0000-1000-8000-00805f9b34fb"
    assert U.UUID_HR_MEASUREMENT == "00002a37-0000-1000-8000-00805f9b34fb"
    # tail spells "narbis"
    assert bytes.fromhex(U.UUID_PPG.split("-")[-1]) == b"narbis"


def test_constants_spot_checks():
    assert P.PROTO_VER == 0x0101
    assert P.OP_KNOB_DISCOVER == 0x14 and P.OP_TEST_REPORT == 0xEA
    assert P.FACTORY_MAGIC == 0x4E415242
    assert P.RATE_SPS[P.RATE_250] == 250 and P.rate_sps(99) == 0
    assert P.ODR_HZ[P.ODR_400] == 400 and P.acc_odr_hz(-1) == 0
    assert P.EVLEN_ERROR == 14 and P.EVENT_REC_MAX == 24
    assert P.ST_NEEDS_RESTART == 11 and P.OTAERR_STATE == 6


# ------------------------------------------------------------------ #
# (4) Write py_<name>.bin twins for the C suite's compare_py()        #
# ------------------------------------------------------------------ #

def test_write_py_fixtures():
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    for stale in FIXTURE_DIR.glob("py_*.bin"):
        if stale.name[len("py_"):-len(".bin")] not in GOLDEN_BYTES:
            stale.unlink()  # a renamed fixture must not leave a stale twin
    for name, make in GOLDEN_BYTES.items():
        (FIXTURE_DIR / f"py_{name}.bin").write_bytes(make())
    assert sorted(p.name for p in FIXTURE_DIR.glob("py_*.bin")) == sorted(
        f"py_{n}.bin" for n in GOLDEN_BYTES)


# ------------------------------------------------------------------ #
# (3) Cross-fixtures written by the C suite                           #
# ------------------------------------------------------------------ #

def test_c_fixtures_cross_parse():
    c_files = sorted(FIXTURE_DIR.glob("c_*.bin")) if FIXTURE_DIR.is_dir() else []
    if not c_files:
        pytest.skip("no C fixtures yet (test_host/out/fixtures/c_*.bin absent)")
    seen = set()
    for f in c_files:
        stem = f.name[len("c_"):-len(".bin")]
        blob = f.read_bytes()
        matched = stem in GOLDEN_BYTES
        if matched:
            seen.add(stem)
            # byte-identical to our twin built from the documented seeds...
            assert blob == GOLDEN_BYTES[stem](), f"{f.name} != documented golden"
            # ...and field-exact through the parser.
            parse, make = GOLDEN_PARSED[stem]
            assert parse(blob) == make(), f"{f.name} parsed fields mismatch"
        # Independent of values: parse -> rebuild must be the identity.
        reenc = reencoder_for(stem)
        if reenc is not None:
            assert reenc(blob) == blob, f"{f.name} re-encode mismatch"
        else:
            assert matched, f"no parser known for fixture {f.name}"
    # every shared fixture the C suite documents must actually be present
    assert seen == set(GOLDEN_BYTES), f"missing C fixtures: {set(GOLDEN_BYTES) - seen}"
