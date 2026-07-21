"""Unit tests for narbis_client.client + recorder against an in-memory
FakeBleak transport — no BLE hardware, no bleak import (the client only
imports bleak lazily when it has to scan).

Run from the repo root:  python -m pytest tools/tests -q
"""

import asyncio
import json
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client import proto as P  # noqa: E402
from narbis_client import uuids as U  # noqa: E402
from narbis_client.client import (NarbisClient, NarbisCtrlError,  # noqa: E402
                                  NarbisDisconnectedError, NarbisError)
from narbis_client.recorder import GapTracker, Recorder  # noqa: E402


# ------------------------------------------------------------------ #
# FakeBleak: the minimal backend surface NarbisClient touches          #
# ------------------------------------------------------------------ #

class FakeBleak:
    """In-memory GATT server. `handlers[uuid](request) -> [(uuid, bytes)]`
    indications are delivered synchronously inside write_gatt_char, which
    is a stricter ordering than real BLE (response can land before the
    write call returns) — the client must survive both."""

    def __init__(self):
        self.address = "AA:BB:CC:DD:EE:FF"
        self.connected = False
        self.mtu_size = 247
        self.notify_cbs = {}
        self.handlers = {}
        self.char_values = {}
        self.written = []
        self._disconnected_cb = None

    # backend surface ------------------------------------------------ #

    def set_disconnected_callback(self, cb):
        self._disconnected_cb = cb

    @property
    def is_connected(self):
        return self.connected

    async def connect(self):
        self.connected = True

    async def disconnect(self):
        self.drop_link()

    async def start_notify(self, uuid, cb):
        self.notify_cbs[uuid.lower()] = cb

    async def stop_notify(self, uuid):
        self.notify_cbs.pop(uuid.lower(), None)

    async def read_gatt_char(self, uuid):
        key = uuid.lower()
        if key not in self.char_values:
            raise RuntimeError(f"no such char {uuid}")
        return bytearray(self.char_values[key])

    async def write_gatt_char(self, uuid, data, response=True):
        key = uuid.lower()
        self.written.append((key, bytes(data), response))
        h = self.handlers.get(key)
        if h is not None:
            for out_uuid, payload in (h(bytes(data)) or []):
                self.indicate(out_uuid, payload)

    # test-side controls ---------------------------------------------- #

    def indicate(self, uuid, payload):
        cb = self.notify_cbs.get(uuid.lower())
        if cb is not None:
            cb(uuid, bytearray(payload))

    def drop_link(self):
        self.connected = False
        if self._disconnected_cb is not None:
            self._disconnected_cb(self)


def req_fields(data: bytes):
    """[u8 op][u8 tid][payload] of a CONTROL request."""
    return data[0], data[1], data[2:]


def make_client():
    fake = FakeBleak()
    client = NarbisClient(backend=fake)
    return fake, client


def run(coro):
    return asyncio.run(coro)


# ------------------------------------------------------------------ #
# control(): tid correlation, unsolicited interleave, errors           #
# ------------------------------------------------------------------ #

def test_control_roundtrip_and_tid_increment():
    fake, client = make_client()
    seen_tids = []

    def handler(data):
        op, tid, payload = req_fields(data)
        seen_tids.append(tid)
        return [(U.UUID_CONTROL,
                 P.build_control_response(op, tid, P.ST_OK, b"\x99"))]

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        r1 = await client.control(P.OP_MARKER, struct.pack("<H", 7))
        r2 = await client.control(P.OP_MARKER, struct.pack("<H", 8))
        return r1, r2

    r1, r2 = run(scenario())
    assert r1.status == P.ST_OK and r1.payload == b"\x99"
    assert seen_tids == [1, 2]  # auto-increment from 1
    # request bytes on the wire came from proto builders
    op, tid, payload = req_fields(fake.written[-1][1])
    assert (op, tid, payload) == (P.OP_MARKER, 2, struct.pack("<H", 8))


def test_control_ignores_interleaved_unsolicited_indications():
    fake, client = make_client()

    def handler(data):
        op, tid, payload = req_fields(data)
        wrong_tid = (tid + 100) % 256 or 1
        return [
            # unsolicited: unknown tid
            (U.UUID_CONTROL,
             P.build_control_response(P.OP_GET_TIME, wrong_tid, P.ST_OK,
                                      struct.pack("<Q", 42))),
            # stale: right tid, WRONG op — must not resolve the future
            (U.UUID_CONTROL,
             P.build_control_response(P.OP_STREAM_STOP, tid, P.ST_BUSY)),
            # the real response
            (U.UUID_CONTROL,
             P.build_control_response(op, tid, P.ST_OK, b"OK")),
        ]

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        return await client.control(P.OP_STREAM_START, b"\x0f")

    resp = run(scenario())
    assert resp.op == P.OP_STREAM_START
    assert resp.payload == b"OK"
    assert client.unsolicited_count == 2
    kinds = [(r.op, r.status) for r in client.unsolicited]
    assert (P.OP_STREAM_STOP, P.ST_BUSY) in kinds


def test_control_concurrent_out_of_order_responses():
    fake, client = make_client()
    parked = []

    def handler(data):
        op, tid, payload = req_fields(data)
        parked.append((op, tid))
        if len(parked) == 2:
            # answer in REVERSE order
            out = []
            for p_op, p_tid in reversed(parked):
                out.append((U.UUID_CONTROL, P.build_control_response(
                    p_op, p_tid, P.ST_OK, bytes([p_tid]))))
            return out
        return []

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        return await asyncio.gather(
            client.control(P.OP_STREAM_START, b"\x01"),
            client.control(P.OP_STREAM_STOP, b"\x01"))

    r_start, r_stop = run(scenario())
    assert r_start.op == P.OP_STREAM_START
    assert r_stop.op == P.OP_STREAM_STOP
    # each future got the payload stamped with ITS tid
    assert r_start.payload == bytes([r_start.tid])
    assert r_stop.payload == bytes([r_stop.tid])


def test_control_error_maps_to_ctrl_error():
    fake, client = make_client()
    fake.handlers[U.UUID_CONTROL] = lambda d: [(
        U.UUID_CONTROL,
        P.build_control_response(d[0], d[1], P.ST_OUT_OF_RANGE))]

    async def scenario():
        await client.connect()
        await client.set_rate(200)

    with pytest.raises(NarbisCtrlError) as ei:
        run(scenario())
    assert ei.value.status == P.ST_OUT_OF_RANGE
    assert ei.value.op == P.OP_SET_RATE
    assert "OUT_OF_RANGE" in str(ei.value)


def test_control_timeout_and_pending_cleanup():
    fake, client = make_client()
    fake.handlers[U.UUID_CONTROL] = lambda d: []  # silence

    async def scenario():
        await client.connect()
        with pytest.raises(NarbisError, match="no response"):
            await client.control(P.OP_KNOB_SAVE, timeout=0.05)
        assert client._pending == {}  # timed-out entry reaped

    run(scenario())


def test_test_block_opcode_response_matches_unmodified_op():
    """TEST ops (>= 0xE0) respond with op unchanged, not op|0x80."""
    fake, client = make_client()
    fake.handlers[U.UUID_CONTROL] = lambda d: [(
        U.UUID_CONTROL,
        P.build_control_response(d[0], d[1], P.ST_OK, b"\x01"))]

    async def scenario():
        await client.connect()
        return await client.control(P.OP_TEST_BUTTON_ECHO)

    resp = run(scenario())
    assert resp.op == P.OP_TEST_BUTTON_ECHO
    # and the wire byte really was 0xE5, no flag games
    assert fake.written[-1][1][0] == 0xE5


def test_disconnect_fails_pending_requests():
    fake, client = make_client()
    fake.handlers[U.UUID_CONTROL] = lambda d: []

    async def scenario():
        await client.connect()
        task = asyncio.ensure_future(client.control(P.OP_KNOB_SAVE,
                                                    timeout=5))
        await asyncio.sleep(0.01)
        fake.drop_link()
        with pytest.raises(NarbisDisconnectedError):
            await task
        assert client.disconnected.is_set()

    run(scenario())


# ------------------------------------------------------------------ #
# knob discovery walk + get/set                                        #
# ------------------------------------------------------------------ #

KNOB_TABLE = [
    P.KnobRecord(0x0001, P.KNOB_U8, P.KF_PERSIST | P.KF_RESTREAM,
                 0, 4, 1, 1, "ppg_rate", ""),
    P.KnobRecord(0x0010, P.KNOB_U16, P.KF_PERSIST | P.KF_LIVE,
                 100, 5000, 2000, 2500, "agc_hold_ms", "ms"),
    P.KnobRecord(0x0011, P.KNOB_BOOL, P.KF_PERSIST | P.KF_LIVE,
                 0, 1, 1, 0, "agc_offdac_en", ""),
    P.KnobRecord(0x0020, P.KNOB_I32, P.KF_PERSIST,
                 -100000, 100000, 0, -42, "gate_acc_thr", "counts"),
    P.KnobRecord(0x0030, P.KNOB_U16, P.KF_PERSIST | P.KF_REBOOT,
                 0, 3600, 300, 300, "idle_timeout_s", "s"),
]


def knob_device(fake, chunk_len=2):
    """Serve KNOB_DISCOVER (chunk_len records per chunk, built with the
    proto.py builders) + KNOB_GET/SET on the fake."""
    current = {k.id: k.current for k in KNOB_TABLE}

    def handler(data):
        op, tid, payload = req_fields(data)
        if op == P.OP_KNOB_DISCOVER:
            (start,) = struct.unpack("<H", payload)
            recs = KNOB_TABLE[start:start + chunk_len]
            chunk = P.KnobDiscoverChunk(len(KNOB_TABLE), start, list(recs))
            return [(U.UUID_CONTROL, P.build_control_response(
                op, tid, P.ST_OK, P.build_knob_discover_chunk(chunk)))]
        if op == P.OP_KNOB_GET:
            (kid,) = struct.unpack("<H", payload)
            if kid not in current:
                return [(U.UUID_CONTROL, P.build_control_response(
                    op, tid, P.ST_BAD_PARAM))]
            return [(U.UUID_CONTROL, P.build_control_response(
                op, tid, P.ST_OK, struct.pack("<Hi", kid, current[kid])))]
        if op == P.OP_KNOB_SET:
            kid, value = struct.unpack("<Hi", payload)
            rec = next((k for k in KNOB_TABLE if k.id == kid), None)
            if rec is None:
                return [(U.UUID_CONTROL, P.build_control_response(
                    op, tid, P.ST_BAD_PARAM))]
            if not rec.min <= value <= rec.max:
                return [(U.UUID_CONTROL, P.build_control_response(
                    op, tid, P.ST_OUT_OF_RANGE))]
            current[kid] = value
            st = P.ST_NEEDS_RESTART if rec.flags & P.KF_REBOOT else P.ST_OK
            return [(U.UUID_CONTROL, P.build_control_response(op, tid, st))]
        return [(U.UUID_CONTROL, P.build_control_response(
            op, tid, P.ST_UNKNOWN_OP))]

    fake.handlers[U.UUID_CONTROL] = handler
    return current


def test_knob_discovery_walk_multichunk():
    fake, client = make_client()
    knob_device(fake, chunk_len=2)  # 5 knobs -> chunks of 2+2+1

    async def scenario():
        await client.connect()
        return await client.discover_knobs()

    knobs = run(scenario())
    assert sorted(knobs) == ["agc_hold_ms", "agc_offdac_en", "gate_acc_thr",
                             "idle_timeout_s", "ppg_rate"]
    k = knobs["agc_hold_ms"]
    assert (k.id, k.type, k.min, k.max, k.default, k.current, k.unit) == \
        (0x0010, P.KNOB_U16, 100, 5000, 2000, 2500, "ms")
    assert knobs["gate_acc_thr"].current == -42  # i32 sign survives


def test_knob_get_set_by_name_and_needs_restart():
    fake, client = make_client()
    current = knob_device(fake, chunk_len=3)

    async def scenario():
        await client.connect()
        v = await client.knob_get("agc_hold_ms")
        st_live = await client.knob_set("agc_hold_ms", 3000)
        st_reboot = await client.knob_set("idle_timeout_s", 600)
        with pytest.raises(NarbisCtrlError) as ei:
            await client.knob_set("agc_hold_ms", 999999)
        with pytest.raises(NarbisError, match="unknown knob"):
            await client.knob_get("no_such_knob")
        return v, st_live, st_reboot, ei.value.status

    v, st_live, st_reboot, st_err = run(scenario())
    assert v == 2500
    assert st_live == P.ST_OK
    assert st_reboot == P.ST_NEEDS_RESTART  # KF_REBOOT knob, still success
    assert st_err == P.ST_OUT_OF_RANGE
    assert current[0x0010] == 3000
    assert client.knobs["agc_hold_ms"].current == 3000  # cache refreshed


# ------------------------------------------------------------------ #
# time_sync min-RTT selection                                          #
# ------------------------------------------------------------------ #

def test_time_sync_min_rtt_math():
    fake, client = make_client()

    # host clock script: (send, recv) pairs -> RTTs 3000, 1000, 5000 us
    host_times = [1_000_000, 1_003_000,
                  2_000_000, 2_001_000,
                  3_000_000, 3_005_000]
    dev_times = [500_000, 1_500_000, 2_600_000]
    it = iter(host_times)
    client._now_us = lambda: next(it)
    dev_it = iter(dev_times)

    def handler(data):
        op, tid, payload = req_fields(data)
        assert op == P.OP_TIME_SYNC
        (host_us,) = struct.unpack("<Q", payload)
        return [(U.UUID_CONTROL, P.build_control_response(
            op, tid, P.ST_OK, struct.pack("<QQ", host_us, next(dev_it))))]

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        return await client.time_sync(n=3)

    offset_us, rtt_us = run(scenario())
    # exchange 1 has min RTT 1000; offset = 1_500_000 - (2_000_000 + 500)
    assert rtt_us == 1000
    assert offset_us == 1_500_000 - 2_000_500


def test_time_sync_discards_bad_echo():
    fake, client = make_client()
    host_times = iter([10_000, 12_000, 20_000, 21_000])
    client._now_us = lambda: next(host_times)
    calls = [0]

    def handler(data):
        op, tid, payload = req_fields(data)
        calls[0] += 1
        (host_us,) = struct.unpack("<Q", payload)
        echo = host_us + (999 if calls[0] == 1 else 0)  # corrupt 1st echo
        return [(U.UUID_CONTROL, P.build_control_response(
            op, tid, P.ST_OK, struct.pack("<QQ", echo, 7_000)))]

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        return await client.time_sync(n=2)

    offset_us, rtt_us = run(scenario())
    assert rtt_us == 1000  # only the second (valid) exchange counted
    assert offset_us == 7_000 - (20_000 + 500)


# ------------------------------------------------------------------ #
# stream subscription -> parsed objects                                #
# ------------------------------------------------------------------ #

def test_subscribe_delivers_parsed_batches_and_survives_garbage():
    fake, client = make_client()
    got = []

    async def scenario():
        await client.connect()
        await client.subscribe("ppg", got.append)
        batch = P.PpgBatch(seq=9, t0_us=123456, rate_code=P.RATE_100,
                           flags=P.PPGF_USB_PRESENT,
                           ir=[1, 2, 3], red=[-1, -2, -3])
        fake.indicate(U.UUID_PPG, P.build_ppg(batch))
        fake.indicate(U.UUID_PPG, b"\x01\x02")  # garbage: logged, no crash
        fake.indicate(U.UUID_PPG, P.build_ppg(batch))

    run(scenario())
    assert len(got) == 2
    assert got[0].seq == 9 and got[0].ir == [1, 2, 3]
    assert got[0].flags & P.PPGF_USB_PRESENT


def test_status_subscription_parses_append_only_tail():
    fake, client = make_client()
    got = []

    async def scenario():
        await client.connect()
        await client.subscribe("status", got.append)
        s = P.Status(P.STATE_STREAMING, P.STF_WORN, 3900, 80, 1, 12, 10,
                     4, 3, 150, 0, 0, P.CLOCK_DRIFT_UNKNOWN, 3600, 800, 75)
        fake.indicate(U.UUID_STATUS, P.build_status(s) + b"\xAA\xBB")

    run(scenario())
    assert got[0].hr_bpm == 75
    assert got[0].extra == b"\xAA\xBB"  # newer-firmware tail preserved


# ------------------------------------------------------------------ #
# selftest chunk reassembly                                            #
# ------------------------------------------------------------------ #

def test_selftest_blob_chunk_walk():
    fake, client = make_client()
    blob = P.SelftestBlob(P.ST_BLOB_VER, 999_000, [
        P.SelftestRecord(P.TEST_I2C_SCAN, P.TR_PASS, 2, 2),
        P.SelftestRecord(P.TEST_AFE_DARK, P.TR_FAIL, 5000, 1200),
        P.SelftestRecord(P.TEST_CHARGER, P.TR_SKIP, 0, 0),
    ])
    wire = P.build_selftest_blob(blob)

    def handler(data):
        op, tid, payload = req_fields(data)
        assert op == P.OP_SELFTEST_RESULT
        (off,) = struct.unpack("<H", payload)
        piece = wire[off:off + 7]  # deliberately tiny chunks
        return [(U.UUID_CONTROL, P.build_control_response(
            op, tid, P.ST_OK,
            P.build_chunk(P.Chunk(len(wire), off, piece))))]

    fake.handlers[U.UUID_CONTROL] = handler

    async def scenario():
        await client.connect()
        return await client.selftest_read_blob()

    out = run(scenario())
    assert out == blob


# ------------------------------------------------------------------ #
# recorder: gap detection + sample expansion                           #
# ------------------------------------------------------------------ #

def test_gap_tracker_wraparound_is_not_a_gap():
    t = GapTracker("x")
    assert t.feed(0xFFFFFFFE) is None
    assert t.feed(0xFFFFFFFF) is None
    assert t.feed(0x00000000) is None  # u32 wrap
    gap = t.feed(2)  # skipped 1
    assert gap is not None and gap["lost_batches"] == 1


def test_recorder_seq_gap_detection_and_expansion(tmp_path):
    rec = Recorder(tmp_path, warn_gaps=False)

    def ppg(seq, t0):
        return P.PpgBatch(seq=seq, t0_us=t0, rate_code=P.RATE_100, flags=0,
                          ir=[10, 11, 12], red=[20, 21, 22])

    rec.feed_ppg(ppg(1, 1_000_000))
    rec.feed_ppg(ppg(2, 1_030_000))
    rec.feed_ppg(ppg(5, 1_120_000))    # gap: lost 3, 4
    rec.feed_ibi(P.IbiBatch(seq=10, records=[
        P.IbiRecord(2_000_000, 800, 90, 0)]))
    rec.feed_ibi(P.IbiBatch(seq=12, records=[   # gap: lost 11
        P.IbiRecord(2_800_000, 810, 88, P.IBIF_FIRST_AFTER_GAP)]))
    rec.feed_event(P.EventBatch(seq=1, events=[
        P.EvGate(3_000_000, state=1, reason_mask=P.GATE_REASON_ACCEL),
        P.EvMarker(3_100_000, source=P.MARKER_SRC_HOST, marker_id=5)]))
    rec.feed_status(P.Status(P.STATE_STREAMING, 0, 3800, 70, 1, 10, 8,
                             4, 3, 0, 0, 0, 0, 60, 820, 73))
    report = rec.close(session={"note": "synthetic"})

    assert report["total_gaps"] == 2
    ppg_gaps = report["per_stream"]["ppg"]["gaps"]
    assert ppg_gaps[0]["expected"] == 3
    assert ppg_gaps[0]["got"] == 5
    assert ppg_gaps[0]["lost_batches"] == 2
    assert report["per_stream"]["ibi"]["gaps"][0]["lost_batches"] == 1
    assert report["per_stream"]["event"]["gaps"] == []

    # gaps.json + session.json on disk
    on_disk = json.loads((tmp_path / "gaps.json").read_text())
    assert on_disk["total_gaps"] == 2
    assert json.loads((tmp_path / "session.json").read_text())["note"] == \
        "synthetic"

    # ppg.csv: 9 sample rows expanded on the 100 sps nominal grid
    lines = (tmp_path / "ppg.csv").read_text().strip().splitlines()
    assert len(lines) == 1 + 9
    row1 = lines[2].split(",")  # seq 1, i=1
    assert int(row1[1]) == 1
    assert int(row1[2]) == 1_000_000 + 10_000  # t0 + 1 * (1e6/100)
    assert int(row1[3]) == 11

    # events.csv carries decoded names + fields
    ev_lines = (tmp_path / "events.csv").read_text().strip().splitlines()
    assert "gate" in ev_lines[1] and "reason_mask=1" in ev_lines[1]
    assert "marker" in ev_lines[2] and "marker_id=5" in ev_lines[2]


def test_recorder_expands_ambient_column(tmp_path):
    rec = Recorder(tmp_path, warn_gaps=False)
    rec.feed_ppg(P.PpgBatch(seq=1, t0_us=0, rate_code=P.RATE_500,
                            flags=P.PPGF_AMB, ir=[1, 2], red=[3, 4],
                            amb=[5, 6]))
    rec.close()
    lines = (tmp_path / "ppg.csv").read_text().strip().splitlines()
    assert lines[1].split(",")[5] == "5"
    assert lines[2].split(",")[2] == "2000"  # 1 sample @ 500 sps = 2000 us


# ------------------------------------------------------------------ #
# end-to-end record() against the fake device                          #
# ------------------------------------------------------------------ #

def test_record_end_to_end_with_fake_device(tmp_path):
    from narbis_client.recorder import record

    fake, client = make_client()
    knob_current = knob_device(fake, chunk_len=3)
    base_handler = fake.handlers[U.UUID_CONTROL]
    started = []

    def handler(data):
        op, tid, payload = req_fields(data)
        if op in (P.OP_STREAM_START, P.OP_STREAM_STOP):
            started.append(op)
            return [(U.UUID_CONTROL,
                     P.build_control_response(op, tid, P.ST_OK))]
        if op == P.OP_TIME_SYNC:
            (host_us,) = struct.unpack("<Q", payload)
            return [(U.UUID_CONTROL, P.build_control_response(
                op, tid, P.ST_OK, struct.pack("<QQ", host_us, 111_222)))]
        return base_handler(data)

    fake.handlers[U.UUID_CONTROL] = handler
    fake.char_values[U.UUID_FIRMWARE_REVISION] = b"v1.2.3-4-gabc"
    fake.char_values[U.UUID_PROTO_VER] = struct.pack("<H", P.PROTO_VER)

    async def scenario():
        await client.connect()
        task = asyncio.ensure_future(record(client, tmp_path, duration=0.6,
                                            timesync_period_s=1000))
        await asyncio.sleep(0.25)
        fake.indicate(U.UUID_PPG, P.build_ppg(P.PpgBatch(
            seq=1, t0_us=0, rate_code=1, flags=0, ir=[7], red=[8])))
        fake.indicate(U.UUID_IBI, P.build_ibi(P.IbiBatch(
            seq=1, records=[P.IbiRecord(1000, 850, 95, 0)])))
        return await task

    report = run(scenario())
    assert report["total_gaps"] == 0
    assert P.OP_STREAM_START in started and P.OP_STREAM_STOP in started
    session = json.loads((tmp_path / "session.json").read_text())
    assert session["device_info"]["fw_revision"] == "v1.2.3-4-gabc"
    assert session["knobs"]["ppg_rate"]["id"] == 1
    assert len(session["time_sync"]) >= 2  # start + end
    assert session["sample_counts"]["ppg"] == 1
    assert (tmp_path / "ppg.csv").exists()
    assert (tmp_path / "gaps.json").exists()


# ------------------------------------------------------------------ #
# reconnect re-arms subscriptions                                      #
# ------------------------------------------------------------------ #

def test_reconnect_rearms_stream_subscriptions():
    fake, client = make_client()
    got = []

    async def scenario():
        await client.connect()
        await client.subscribe("ibi", got.append)
        fake.drop_link()
        assert client.disconnected.is_set()
        fake.notify_cbs.clear()  # a real link drop forgets CCCDs
        await client.reconnect(attempts=1)
        assert not client.disconnected.is_set()
        fake.indicate(U.UUID_IBI, P.build_ibi(P.IbiBatch(seq=3, records=[])))

    run(scenario())
    assert len(got) == 1 and got[0].seq == 3
