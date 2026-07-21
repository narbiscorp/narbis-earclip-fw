"""proto.py — Python mirror of firmware proto.h (the BLE wire contract).

Constants carry the proto.h names minus the NC_ prefix. Layouts are
little-endian packed structs; every builder produces exactly the bytes
the firmware emits and every parser accepts exactly the bytes the
firmware accepts, with two deliberate liberalities required by the
contract:

  * STATUS is append-only: parse_status() accepts any length >= 32 and
    keeps the surplus in .extra;
  * EVENT records are {type, len, payload}: unknown types (or known
    types whose len does not match this proto version) are preserved as
    EvUnknown rather than rejected, so old clients skip new record
    types without desync.

Pure stdlib (struct/dataclasses only) so tools/testapp's generator can
consume this file mechanically. No numpy, no bleak.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union

# ------------------------------------------------------------------ #
# Protocol version                                                    #
# ------------------------------------------------------------------ #

PROTO_VER_MAJOR = 1
PROTO_VER_MINOR = 0
PROTO_VER = (PROTO_VER_MAJOR << 8) | PROTO_VER_MINOR

# ATT payload budget at MTU 247
ATT_PAYLOAD_MAX = 244

# ------------------------------------------------------------------ #
# nc_types.h: rate / ODR codes and their Hz maps                      #
# ------------------------------------------------------------------ #

RATE_50 = 0
RATE_100 = 1  # product default
RATE_200 = 2
RATE_250 = 3
RATE_500 = 4
RATE_COUNT = 5

RATE_SPS = {RATE_50: 50, RATE_100: 100, RATE_200: 200, RATE_250: 250, RATE_500: 500}


def rate_sps(code: int) -> int:
    """Mirror of nc_rate_sps(): 0 for out-of-range codes."""
    return RATE_SPS.get(code, 0)


ODR_10 = 0
ODR_25 = 1
ODR_50 = 2
ODR_100 = 3
ODR_200 = 4
ODR_400 = 5
ODR_COUNT = 6

ODR_HZ = {ODR_10: 10, ODR_25: 25, ODR_50: 50, ODR_100: 100, ODR_200: 200, ODR_400: 400}


def acc_odr_hz(code: int) -> int:
    """Mirror of nc_acc_odr_hz(): 0 for out-of-range codes."""
    return ODR_HZ.get(code, 0)


# nc_sys_state_t (STATUS wire values)
STATE_IDLE = 0
STATE_CONNECTED = 1
STATE_STREAMING = 2
STATE_OTA = 3
STATE_SELFTEST = 4
STATE_LOWBATT = 5

# nc_charger_state_t
CHG_ON_BATTERY = 0
CHG_CHARGING = 1
CHG_COMPLETE = 2

# ------------------------------------------------------------------ #
# PPG_STREAM  [hdr 15B][n x {i32 ir, i32 red (, i32 amb)}]            #
# ------------------------------------------------------------------ #

PPG_HDR_SIZE = 15
PPG_MAX_N_NOAMB = 28  # 15 + 28*8  = 239 <= 244
PPG_MAX_N_AMB = 19    # 15 + 19*12 = 243 <= 244

# PPG batch flag bits (nc_types.h NC_PPGF_*)
PPGF_GATE = 1 << 0
PPGF_AGC_SETTLING = 1 << 1
PPGF_USB_PRESENT = 1 << 2
PPGF_RATE_CHANGED = 1 << 3
PPGF_AMB = 1 << 4          # amb field present in batch
PPGF_WEAR_OFF = 1 << 5
PPGF_CLIPPED = 1 << 6      # >=1 sample near rail

_PPG_HDR = struct.Struct("<IQBBB")
assert _PPG_HDR.size == PPG_HDR_SIZE


@dataclass
class PpgBatch:
    seq: int
    t0_us: int
    rate_code: int
    flags: int
    ir: List[int]
    red: List[int]
    amb: Optional[List[int]] = None  # present iff flags & PPGF_AMB

    @property
    def n(self) -> int:
        return len(self.ir)


def build_ppg(batch: PpgBatch) -> bytes:
    n = len(batch.ir)
    if len(batch.red) != n:
        raise ValueError("ir/red length mismatch")
    flags = batch.flags
    if batch.amb is not None:
        if len(batch.amb) != n:
            raise ValueError("amb length mismatch")
        if not flags & PPGF_AMB:
            raise ValueError("amb samples present but PPGF_AMB clear")
        if n > PPG_MAX_N_AMB:
            raise ValueError(f"n={n} > PPG_MAX_N_AMB")
        body = struct.pack(
            f"<{3 * n}i",
            *(v for triple in zip(batch.ir, batch.red, batch.amb) for v in triple),
        )
    else:
        if flags & PPGF_AMB:
            raise ValueError("PPGF_AMB set but no amb samples")
        if n > PPG_MAX_N_NOAMB:
            raise ValueError(f"n={n} > PPG_MAX_N_NOAMB")
        body = struct.pack(
            f"<{2 * n}i", *(v for pair in zip(batch.ir, batch.red) for v in pair)
        )
    return _PPG_HDR.pack(batch.seq, batch.t0_us, batch.rate_code, n, flags) + body


def parse_ppg(buf: bytes) -> PpgBatch:
    if len(buf) < PPG_HDR_SIZE:
        raise ValueError(f"PPG batch too short: {len(buf)}")
    seq, t0_us, rate_code, n, flags = _PPG_HDR.unpack_from(buf)
    has_amb = bool(flags & PPGF_AMB)
    stride = 12 if has_amb else 8
    if len(buf) != PPG_HDR_SIZE + n * stride:
        raise ValueError(f"PPG batch length {len(buf)} != {PPG_HDR_SIZE + n * stride}")
    vals = struct.unpack_from(f"<{(3 if has_amb else 2) * n}i", buf, PPG_HDR_SIZE)
    if has_amb:
        return PpgBatch(seq, t0_us, rate_code, flags,
                        list(vals[0::3]), list(vals[1::3]), list(vals[2::3]))
    return PpgBatch(seq, t0_us, rate_code, flags, list(vals[0::2]), list(vals[1::2]))


# ------------------------------------------------------------------ #
# ACCEL_STREAM [hdr 15B][n x {i16 x, i16 y, i16 z}]                   #
# ------------------------------------------------------------------ #

ACCEL_HDR_SIZE = 15
ACCEL_MAX_N = 38  # 15 + 38*6 = 243 <= 244

ACCF_FS_MASK = 0x03  # 0=+/-2g 1=+/-4g 2=+/-8g 3=+/-16g
ACCF_FIFO_OVERRUN = 1 << 2

_ACCEL_HDR = struct.Struct("<IQBBB")
assert _ACCEL_HDR.size == ACCEL_HDR_SIZE


@dataclass
class AccelBatch:
    seq: int
    t0_us: int
    odr_code: int
    flags: int
    samples: List[Tuple[int, int, int]]  # (x, y, z) raw counts

    @property
    def n(self) -> int:
        return len(self.samples)


def build_accel(batch: AccelBatch) -> bytes:
    n = len(batch.samples)
    if n > ACCEL_MAX_N:
        raise ValueError(f"n={n} > ACCEL_MAX_N")
    body = struct.pack(f"<{3 * n}h", *(v for s in batch.samples for v in s))
    return _ACCEL_HDR.pack(batch.seq, batch.t0_us, batch.odr_code, n, batch.flags) + body


def parse_accel(buf: bytes) -> AccelBatch:
    if len(buf) < ACCEL_HDR_SIZE:
        raise ValueError(f"ACCEL batch too short: {len(buf)}")
    seq, t0_us, odr_code, n, flags = _ACCEL_HDR.unpack_from(buf)
    if len(buf) != ACCEL_HDR_SIZE + n * 6:
        raise ValueError(f"ACCEL batch length {len(buf)} != {ACCEL_HDR_SIZE + n * 6}")
    vals = struct.unpack_from(f"<{3 * n}h", buf, ACCEL_HDR_SIZE)
    samples = [(vals[i], vals[i + 1], vals[i + 2]) for i in range(0, 3 * n, 3)]
    return AccelBatch(seq, t0_us, odr_code, flags, samples)


# ------------------------------------------------------------------ #
# IBI_STREAM [u32 seq][u8 n][n x 12B]                                 #
# ------------------------------------------------------------------ #

IBI_HDR_SIZE = 5
IBI_REC_SIZE = 12
IBI_MAX_N = 19  # 5 + 19*12 = 233 <= 244

# IBI record flag bits (nc_types.h NC_IBIF_*)
IBIF_GATED_CTX = 1 << 0
IBIF_AGC_SETTLING = 1 << 1
IBIF_INTERPOLATED = 1 << 2
IBIF_FIRST_AFTER_GAP = 1 << 3

_IBI_HDR = struct.Struct("<IB")
_IBI_REC = struct.Struct("<QHBB")
assert _IBI_HDR.size == IBI_HDR_SIZE and _IBI_REC.size == IBI_REC_SIZE


@dataclass
class IbiRecord:
    t_beat_us: int
    ibi_ms: int
    confidence: int  # 0..100
    flags: int


@dataclass
class IbiBatch:
    seq: int
    records: List[IbiRecord]

    @property
    def n(self) -> int:
        return len(self.records)


def build_ibi(batch: IbiBatch) -> bytes:
    n = len(batch.records)
    if n > IBI_MAX_N:
        raise ValueError(f"n={n} > IBI_MAX_N")
    out = bytearray(_IBI_HDR.pack(batch.seq, n))
    for r in batch.records:
        out += _IBI_REC.pack(r.t_beat_us, r.ibi_ms, r.confidence, r.flags)
    return bytes(out)


def parse_ibi(buf: bytes) -> IbiBatch:
    if len(buf) < IBI_HDR_SIZE:
        raise ValueError(f"IBI batch too short: {len(buf)}")
    seq, n = _IBI_HDR.unpack_from(buf)
    if len(buf) != IBI_HDR_SIZE + n * IBI_REC_SIZE:
        raise ValueError(f"IBI batch length {len(buf)} != {IBI_HDR_SIZE + n * IBI_REC_SIZE}")
    recs = [IbiRecord(*_IBI_REC.unpack_from(buf, IBI_HDR_SIZE + i * IBI_REC_SIZE))
            for i in range(n)]
    return IbiBatch(seq, recs)


# ------------------------------------------------------------------ #
# EVENT_STREAM [u32 seq][u8 n][n x {u8 type, u8 len, payload}]        #
# Payload always starts with u64 t_us; len = 8 + internal len.        #
# ------------------------------------------------------------------ #

EVENT_HDR_SIZE = 5
EVENT_REC_MAX = 2 + 8 + 14  # type+len+t_us+data

# nc_event_type_t
EV_AGC_STEP = 0x01
EV_GATE = 0x02
EV_WEAR = 0x03
EV_MARKER = 0x04
EV_ERROR = 0x05
EV_RATE_CHANGE = 0x06
EV_AGC_OFFDAC = 0x07
EV_SELFTEST_DONE = 0x08

# wire payload lengths (t_us included)
EVLEN_AGC_STEP = 13
EVLEN_GATE = 10
EVLEN_WEAR = 9
EVLEN_MARKER = 11
EVLEN_ERROR = 14
EVLEN_RATE_CHANGE = 10
EVLEN_AGC_OFFDAC = 11
EVLEN_SELFTEST_DONE = 10

MARKER_SRC_BUTTON = 0
MARKER_SRC_HOST = 1

# Gate reason mask bits (NC_EV_GATE payload)
GATE_REASON_ACCEL = 1 << 0
GATE_REASON_SAT = 1 << 1
GATE_REASON_DC_STEP = 1 << 2
GATE_REASON_COLLAPSE = 1 << 3
GATE_REASON_AGC = 1 << 4
GATE_REASON_WEAR = 1 << 5

# NC_EV_ERROR codes (nc_error_code_t)
ERR_NONE = 0
ERR_I2C = 1            # arg = consecutive failure count
ERR_AFE_INIT = 2
ERR_ACCEL_INIT = 3
ERR_PPG_OVERRUN = 4    # afe_rdy_q or ppg_queue overflow
ERR_FIFO_OVERRUN = 5   # LIS2DH12 FIFO
ERR_NOTIFY_DROP = 6    # arg = cumulative drops
ERR_NVS = 7
ERR_OTA = 8
ERR_BROWNOUT = 9


@dataclass
class EvAgcStep:
    t_us: int
    led: int
    old_ma: int
    new_ma: int
    old_rf: int
    new_rf: int
    type: int = EV_AGC_STEP


@dataclass
class EvGate:
    t_us: int
    state: int
    reason_mask: int
    type: int = EV_GATE


@dataclass
class EvWear:
    t_us: int
    worn: int
    type: int = EV_WEAR


@dataclass
class EvMarker:
    t_us: int
    source: int
    marker_id: int
    type: int = EV_MARKER


@dataclass
class EvError:
    t_us: int
    code: int
    arg: int
    type: int = EV_ERROR


@dataclass
class EvRateChange:
    t_us: int
    old_code: int
    new_code: int
    type: int = EV_RATE_CHANGE


@dataclass
class EvAgcOffdac:
    t_us: int
    phase: int
    old_code: int
    new_code: int
    type: int = EV_AGC_OFFDAC


@dataclass
class EvSelftestDone:
    t_us: int
    pass_count: int
    fail_count: int
    type: int = EV_SELFTEST_DONE


@dataclass
class EvUnknown:
    """Record whose type (or length, for a known type) this proto version
    does not understand. payload is the full wire payload including the
    leading t_us if len >= 8 (t_us is then also decoded for convenience)."""
    type: int
    payload: bytes
    t_us: Optional[int] = None


Event = Union[EvAgcStep, EvGate, EvWear, EvMarker, EvError,
              EvRateChange, EvAgcOffdac, EvSelftestDone, EvUnknown]

_EVENT_HDR = struct.Struct("<IB")

# type -> (wire payload len, struct fmt of fields after t_us, dataclass)
_EVENT_CODECS = {
    EV_AGC_STEP: (EVLEN_AGC_STEP, "<BBBBB", EvAgcStep),
    EV_GATE: (EVLEN_GATE, "<BB", EvGate),
    EV_WEAR: (EVLEN_WEAR, "<B", EvWear),
    EV_MARKER: (EVLEN_MARKER, "<BH", EvMarker),
    EV_ERROR: (EVLEN_ERROR, "<HI", EvError),
    EV_RATE_CHANGE: (EVLEN_RATE_CHANGE, "<BB", EvRateChange),
    EV_AGC_OFFDAC: (EVLEN_AGC_OFFDAC, "<BBB", EvAgcOffdac),
    EV_SELFTEST_DONE: (EVLEN_SELFTEST_DONE, "<BB", EvSelftestDone),
}
for _t, (_ln, _fmt, _cls) in _EVENT_CODECS.items():
    assert _ln == 8 + struct.calcsize(_fmt), _t


@dataclass
class EventBatch:
    seq: int
    events: List[Event]

    @property
    def n(self) -> int:
        return len(self.events)


def build_event_record(ev: Event) -> bytes:
    if isinstance(ev, EvUnknown):
        if len(ev.payload) > 8 + 14:
            raise ValueError("event payload exceeds EVENT_REC_MAX budget")
        return bytes((ev.type, len(ev.payload))) + ev.payload
    ln, fmt, cls = _EVENT_CODECS[ev.type]
    fields = [getattr(ev, f) for f in cls.__dataclass_fields__
              if f not in ("t_us", "type")]
    payload = struct.pack("<Q", ev.t_us) + struct.pack(fmt, *fields)
    assert len(payload) == ln
    return bytes((ev.type, ln)) + payload


def parse_event_record(type_: int, payload: bytes) -> Event:
    codec = _EVENT_CODECS.get(type_)
    if codec is not None and len(payload) == codec[0]:
        _, fmt, cls = codec
        (t_us,) = struct.unpack_from("<Q", payload)
        return cls(t_us, *struct.unpack_from(fmt, payload, 8))
    t_us = struct.unpack_from("<Q", payload)[0] if len(payload) >= 8 else None
    return EvUnknown(type_, payload, t_us)


def build_event_batch(batch: EventBatch) -> bytes:
    if len(batch.events) > 255:
        raise ValueError("too many event records")
    out = bytearray(_EVENT_HDR.pack(batch.seq, len(batch.events)))
    for ev in batch.events:
        out += build_event_record(ev)
    if len(out) > ATT_PAYLOAD_MAX:
        raise ValueError("event batch exceeds ATT_PAYLOAD_MAX")
    return bytes(out)


def parse_event_batch(buf: bytes) -> EventBatch:
    if len(buf) < EVENT_HDR_SIZE:
        raise ValueError(f"EVENT batch too short: {len(buf)}")
    seq, n = _EVENT_HDR.unpack_from(buf)
    events: List[Event] = []
    off = EVENT_HDR_SIZE
    for _ in range(n):
        if off + 2 > len(buf):
            raise ValueError("truncated event record header")
        type_, ln = buf[off], buf[off + 1]
        off += 2
        if off + ln > len(buf):
            raise ValueError("truncated event record payload")
        events.append(parse_event_record(type_, buf[off:off + ln]))
        off += ln
    if off != len(buf):
        raise ValueError(f"trailing bytes after {n} event records")
    return EventBatch(seq, events)


# ------------------------------------------------------------------ #
# STATUS — 32 bytes, append-only                                      #
# ------------------------------------------------------------------ #

STATUS_SIZE = 32

STF_CHARGING = 1 << 0
STF_CHARGE_DONE = 1 << 1
STF_USB = 1 << 2
STF_WORN = 1 << 3
STF_GATE = 1 << 4
STF_AGC_FROZEN = 1 << 5
STF_HRS_ACTIVE = 1 << 6
STF_LOWBATT_WARN = 1 << 7

CLOCK_DRIFT_UNKNOWN = 0x7FFF  # clock_drift_ppm_x10 sentinel

_STATUS = struct.Struct("<BBHBBBBBBHIHhIHB5s")
assert _STATUS.size == STATUS_SIZE


@dataclass
class Status:
    sys_state: int
    flags: int
    batt_mv: int
    batt_pct: int
    ppg_rate_code: int
    led_ir_ma: int
    led_red_ma: int
    tia_gain_code: int      # RF code 0..7
    tia_cf_code: int        # CF code 0..7
    gate_duty_x100: int     # 0..10000 over 60 s window
    notif_drop_count: int
    i2c_err_count: int
    clock_drift_ppm_x10: int  # CLOCK_DRIFT_UNKNOWN = unknown
    uptime_s: int
    ibi_last_ms: int
    hr_bpm: int             # smoothed; 0 = none
    reserved: bytes = b"\x00" * 5
    extra: bytes = b""      # append-only tail from a newer firmware


def build_status(s: Status) -> bytes:
    if len(s.reserved) != 5:
        raise ValueError("reserved must be 5 bytes")
    return _STATUS.pack(
        s.sys_state, s.flags, s.batt_mv, s.batt_pct, s.ppg_rate_code,
        s.led_ir_ma, s.led_red_ma, s.tia_gain_code, s.tia_cf_code,
        s.gate_duty_x100, s.notif_drop_count, s.i2c_err_count,
        s.clock_drift_ppm_x10, s.uptime_s, s.ibi_last_ms, s.hr_bpm,
        s.reserved) + s.extra


def parse_status(buf: bytes) -> Status:
    # Append-only rule: any length >= the 32 bytes this version knows.
    if len(buf) < STATUS_SIZE:
        raise ValueError(f"STATUS too short: {len(buf)} < {STATUS_SIZE}")
    fields = _STATUS.unpack_from(buf)
    return Status(*fields, extra=bytes(buf[STATUS_SIZE:]))


# ------------------------------------------------------------------ #
# CONTROL: request [u8 op][u8 tid][payload]                           #
#          response          [u8 op|0x80][u8 tid][u8 status][payload] #
# ------------------------------------------------------------------ #

OP_STREAM_START = 0x01     # u8 mask
OP_STREAM_STOP = 0x02      # u8 mask
OP_SET_RATE = 0x03         # u8 rate_code
OP_KNOB_GET = 0x10         # u16 id -> {u16 id, i32 value}
OP_KNOB_SET = 0x11         # {u16 id, i32 value}
OP_KNOB_SAVE = 0x12
OP_KNOB_RESET = 0x13       # u8 scope: 0 RAM, 1 RAM+NVS
OP_KNOB_DISCOVER = 0x14    # u16 start_index -> chunk
OP_TIME_SYNC = 0x20        # u64 host_epoch_us -> {u64 echo, u64 dev_t_us}
OP_GET_TIME = 0x21         # -> u64 dev_t_us
OP_MARKER = 0x30           # u16 marker_id (0 = unnumbered)
OP_AGC_FREEZE = 0x40       # u8 freeze
OP_AGC_MANUAL = 0x41       # {u8 ir_ma, u8 red_ma, u8 rf_code, u8 apply_mask}
OP_SELFTEST_RUN = 0x50     # u32 test_mask (0 = all)
OP_SELFTEST_RESULT = 0x51  # u16 offset -> {u16 total, u16 off, u8 n, bytes}
OP_ENTER_OTA = 0x60
OP_POWER_OFF = 0x70
OP_REBOOT = 0x71
OP_FACTORY_RESET = 0x72    # u32 FACTORY_MAGIC

# TEST block — only when built with NARBIS_TEST_MODE=1
OP_TEST_SELFTEST_ONE = 0xE0
OP_TEST_LED_DRIVE = 0xE1
OP_TEST_LED_SWEEP = 0xE2
OP_TEST_RX_SWEEP = 0xE3
OP_TEST_RATE_COUNT = 0xE4
OP_TEST_BUTTON_ECHO = 0xE5
OP_TEST_CHARGER_LIVE = 0xE6
OP_TEST_BATT_RAW = 0xE7
OP_TEST_ACCEL_LIVE = 0xE8
OP_TEST_SLEEP_NOW = 0xE9
OP_TEST_REPORT = 0xEA

OP_RESP_FLAG = 0x80
FACTORY_MAGIC = 0x4E415242  # "NARB"

STREAM_MASK_PPG = 1 << 0
STREAM_MASK_ACCEL = 1 << 1
STREAM_MASK_IBI = 1 << 2
STREAM_MASK_EVENT = 1 << 3

# nc_ctrl_status_t
ST_OK = 0
ST_UNKNOWN_OP = 1
ST_BAD_LEN = 2
ST_BAD_PARAM = 3
ST_OUT_OF_RANGE = 4
ST_READ_ONLY = 5
ST_BUSY = 6
ST_WRONG_STATE = 7
ST_NVS_ERR = 8
ST_CRC_ERR = 9
ST_VERSION_MISMATCH = 10
ST_NEEDS_RESTART = 11  # success, effect deferred to restart
ST_UNAUTHORIZED = 12
ST_LOWBATT = 13


def build_control_request(op: int, tid: int, payload: bytes = b"") -> bytes:
    if not 0 <= op <= 0xFF or op & OP_RESP_FLAG and op < 0xE0:
        # 0xE0..0xEF TEST opcodes legitimately have bit7 set; anything else
        # with bit7 would alias a response.
        raise ValueError(f"invalid request opcode {op:#x}")
    if not 0 <= tid <= 0xFF:
        raise ValueError("tid out of range")
    return bytes((op, tid)) + payload


@dataclass
class ControlResponse:
    op: int        # original opcode (response flag stripped)
    tid: int
    status: int    # nc_ctrl_status_t
    payload: bytes


def build_control_response(op: int, tid: int, status: int,
                           payload: bytes = b"") -> bytes:
    """Device-side encoding (host uses it for round-trip tests/simulators).
    TEST-block opcodes (>= 0xE0) already carry bit7 and go out unchanged."""
    op_raw = op if op >= 0xE0 else op | OP_RESP_FLAG
    return bytes((op_raw, tid, status)) + payload


def parse_control_response(buf: bytes) -> ControlResponse:
    if len(buf) < 3:
        raise ValueError(f"CONTROL response too short: {len(buf)}")
    op_raw, tid, status = buf[0], buf[1], buf[2]
    # TEST opcodes 0xE0.. already carry bit7; their responses are >= 0xE0|0x80
    # which wraps out of u8 — the firmware responds with op unchanged for the
    # TEST block. Accept both: plain TEST opcode, or anything with bit7.
    if op_raw >= 0xE0:
        op = op_raw
    elif op_raw & OP_RESP_FLAG:
        op = op_raw & ~OP_RESP_FLAG
    else:
        raise ValueError(f"not a CONTROL response: op byte {op_raw:#x}")
    return ControlResponse(op, tid, status, bytes(buf[3:]))


# --- request builders (payload layouts from the opcode table) -------- #

def build_stream_start(tid: int, mask: int) -> bytes:
    return build_control_request(OP_STREAM_START, tid, struct.pack("<B", mask))


def build_stream_stop(tid: int, mask: int) -> bytes:
    return build_control_request(OP_STREAM_STOP, tid, struct.pack("<B", mask))


def build_set_rate(tid: int, rate_code: int) -> bytes:
    return build_control_request(OP_SET_RATE, tid, struct.pack("<B", rate_code))


def build_knob_get(tid: int, knob_id: int) -> bytes:
    return build_control_request(OP_KNOB_GET, tid, struct.pack("<H", knob_id))


def build_knob_set(tid: int, knob_id: int, value: int) -> bytes:
    return build_control_request(OP_KNOB_SET, tid, struct.pack("<Hi", knob_id, value))


def build_knob_save(tid: int) -> bytes:
    return build_control_request(OP_KNOB_SAVE, tid)


def build_knob_reset(tid: int, scope: int) -> bytes:
    return build_control_request(OP_KNOB_RESET, tid, struct.pack("<B", scope))


def build_knob_discover(tid: int, start_index: int) -> bytes:
    return build_control_request(OP_KNOB_DISCOVER, tid, struct.pack("<H", start_index))


def build_time_sync(tid: int, host_epoch_us: int) -> bytes:
    return build_control_request(OP_TIME_SYNC, tid, struct.pack("<Q", host_epoch_us))


def build_get_time(tid: int) -> bytes:
    return build_control_request(OP_GET_TIME, tid)


def build_marker(tid: int, marker_id: int = 0) -> bytes:
    return build_control_request(OP_MARKER, tid, struct.pack("<H", marker_id))


def build_agc_freeze(tid: int, freeze: bool) -> bytes:
    return build_control_request(OP_AGC_FREEZE, tid, struct.pack("<B", int(freeze)))


AGC_APPLY_IR = 1 << 0
AGC_APPLY_RED = 1 << 1
AGC_APPLY_GAIN = 1 << 2


def build_agc_manual(tid: int, ir_ma: int, red_ma: int, rf_code: int,
                     apply_mask: int) -> bytes:
    return build_control_request(
        OP_AGC_MANUAL, tid, struct.pack("<BBBB", ir_ma, red_ma, rf_code, apply_mask))


def build_selftest_run(tid: int, test_mask: int = 0) -> bytes:
    return build_control_request(OP_SELFTEST_RUN, tid, struct.pack("<I", test_mask))


def build_selftest_result(tid: int, offset: int) -> bytes:
    return build_control_request(OP_SELFTEST_RESULT, tid, struct.pack("<H", offset))


def build_enter_ota(tid: int) -> bytes:
    return build_control_request(OP_ENTER_OTA, tid)


def build_power_off(tid: int) -> bytes:
    return build_control_request(OP_POWER_OFF, tid)


def build_reboot(tid: int) -> bytes:
    return build_control_request(OP_REBOOT, tid)


def build_factory_reset(tid: int) -> bytes:
    return build_control_request(OP_FACTORY_RESET, tid, struct.pack("<I", FACTORY_MAGIC))


# --- response payload parsers ---------------------------------------- #

def parse_knob_get_resp(payload: bytes) -> Tuple[int, int]:
    """-> (knob_id, value)"""
    return struct.unpack("<Hi", payload)


def parse_time_sync_resp(payload: bytes) -> Tuple[int, int]:
    """-> (host_epoch_us echo, dev_t_us)"""
    return struct.unpack("<QQ", payload)


def parse_get_time_resp(payload: bytes) -> int:
    return struct.unpack("<Q", payload)[0]


# ------------------------------------------------------------------ #
# Knob discovery chunk                                                #
# [u16 total][u16 first_idx][u8 n][n x record]                        #
# record = {u16 id, u8 type, u8 flags, i32 min, i32 max, i32 def,     #
#           i32 current, u8 name_len, name..., u8 unit_len, unit...}  #
# ------------------------------------------------------------------ #

KNOB_DISC_HDR_SIZE = 5
KNOB_REC_FIXED = 19  # as defined in proto.h (see note in test suite)

# nc_knob_type_t
KNOB_BOOL = 0
KNOB_U8 = 1
KNOB_U16 = 2
KNOB_I32 = 3

# NC_KF_* knob flags
KF_PERSIST = 1 << 0
KF_LIVE = 1 << 1      # applies immediately
KF_RESTREAM = 1 << 2  # applies at next stream start
KF_REBOOT = 1 << 3    # needs restart

_KNOB_DISC_HDR = struct.Struct("<HHB")
_KNOB_REC_HEAD = struct.Struct("<HBBiiii")  # id..current (20 bytes)


@dataclass
class KnobRecord:
    id: int
    type: int      # nc_knob_type_t
    flags: int     # NC_KF_*
    min: int
    max: int
    default: int
    current: int
    name: str
    unit: str


@dataclass
class KnobDiscoverChunk:
    total: int      # total knob count on the device
    first_idx: int  # dense index of records[0]
    records: List[KnobRecord] = field(default_factory=list)


def build_knob_discover_chunk(chunk: KnobDiscoverChunk) -> bytes:
    out = bytearray(_KNOB_DISC_HDR.pack(chunk.total, chunk.first_idx,
                                        len(chunk.records)))
    for r in chunk.records:
        name = r.name.encode("ascii")
        unit = r.unit.encode("ascii")
        if len(name) > 255 or len(unit) > 255:
            raise ValueError("name/unit too long")
        out += _KNOB_REC_HEAD.pack(r.id, r.type, r.flags,
                                   r.min, r.max, r.default, r.current)
        out += bytes((len(name),)) + name
        out += bytes((len(unit),)) + unit
    return bytes(out)


def parse_knob_discover_chunk(payload: bytes) -> KnobDiscoverChunk:
    if len(payload) < KNOB_DISC_HDR_SIZE:
        raise ValueError("knob discover chunk too short")
    total, first_idx, n = _KNOB_DISC_HDR.unpack_from(payload)
    off = KNOB_DISC_HDR_SIZE
    records: List[KnobRecord] = []
    for _ in range(n):
        if off + _KNOB_REC_HEAD.size + 1 > len(payload):
            raise ValueError("truncated knob record")
        kid, ktype, kflags, kmin, kmax, kdef, kcur = \
            _KNOB_REC_HEAD.unpack_from(payload, off)
        off += _KNOB_REC_HEAD.size
        name_len = payload[off]
        off += 1
        if off + name_len + 1 > len(payload):
            raise ValueError("truncated knob name")
        name = payload[off:off + name_len].decode("ascii")
        off += name_len
        unit_len = payload[off]
        off += 1
        if off + unit_len > len(payload):
            raise ValueError("truncated knob unit")
        unit = payload[off:off + unit_len].decode("ascii")
        off += unit_len
        records.append(KnobRecord(kid, ktype, kflags, kmin, kmax, kdef, kcur,
                                  name, unit))
    if off != len(payload):
        raise ValueError("trailing bytes after knob records")
    return KnobDiscoverChunk(total, first_idx, records)


# ------------------------------------------------------------------ #
# Chunk envelope for SELFTEST_RESULT / TEST_REPORT style transfers:   #
# {u16 total, u16 off, u8 n, bytes}                                   #
# ------------------------------------------------------------------ #

_CHUNK_HDR = struct.Struct("<HHB")


@dataclass
class Chunk:
    total: int   # total blob size in bytes
    offset: int  # offset of data[0] within the blob
    data: bytes


def build_chunk(chunk: Chunk) -> bytes:
    if len(chunk.data) > 255:
        raise ValueError("chunk data too long")
    return _CHUNK_HDR.pack(chunk.total, chunk.offset, len(chunk.data)) + chunk.data


def parse_chunk(payload: bytes) -> Chunk:
    if len(payload) < _CHUNK_HDR.size:
        raise ValueError("chunk too short")
    total, off, n = _CHUNK_HDR.unpack_from(payload)
    data = payload[_CHUNK_HDR.size:]
    if len(data) != n:
        raise ValueError(f"chunk data length {len(data)} != n={n}")
    return Chunk(total, off, bytes(data))


# ------------------------------------------------------------------ #
# Self-test result blob (reassembled from CONTROL 0x51 chunks):       #
# [u8 blob_ver][u64 t_run_us][u8 n][n x {u8 id,u8 status,i32 val,i32 thr}]
# ------------------------------------------------------------------ #

ST_BLOB_VER = 1
ST_REC_SIZE = 10

# nc_selftest_id_t
TEST_I2C_SCAN = 1
TEST_ACCEL_WHOAMI = 2
TEST_AFE_REG_RW = 3
TEST_AFE_DARK = 4
TEST_XTALK = 5
TEST_ACCEL_ST = 6
TEST_BATT = 7
TEST_CHARGER = 8
TEST_COUNT_ = 8

# nc_test_status_t
TR_PASS = 0
TR_FAIL = 1
TR_SKIP = 2

_ST_BLOB_HDR = struct.Struct("<BQB")
_ST_REC = struct.Struct("<BBii")
assert _ST_REC.size == ST_REC_SIZE


@dataclass
class SelftestRecord:
    id: int       # nc_selftest_id_t
    status: int   # nc_test_status_t
    value: int
    threshold: int


@dataclass
class SelftestBlob:
    blob_ver: int
    t_run_us: int
    records: List[SelftestRecord]


def build_selftest_blob(blob: SelftestBlob) -> bytes:
    out = bytearray(_ST_BLOB_HDR.pack(blob.blob_ver, blob.t_run_us,
                                      len(blob.records)))
    for r in blob.records:
        out += _ST_REC.pack(r.id, r.status, r.value, r.threshold)
    return bytes(out)


def parse_selftest_blob(buf: bytes) -> SelftestBlob:
    if len(buf) < _ST_BLOB_HDR.size:
        raise ValueError("selftest blob too short")
    ver, t_run_us, n = _ST_BLOB_HDR.unpack_from(buf)
    if len(buf) != _ST_BLOB_HDR.size + n * ST_REC_SIZE:
        raise ValueError("selftest blob length mismatch")
    recs = [SelftestRecord(*_ST_REC.unpack_from(buf, _ST_BLOB_HDR.size + i * ST_REC_SIZE))
            for i in range(n)]
    return SelftestBlob(ver, t_run_us, recs)


# ------------------------------------------------------------------ #
# OTA service                                                         #
# OTA_CTRL uses the CONTROL envelope; OTA_DATA is write-no-response   #
# [u32 offset][data <= 240B]                                          #
# ------------------------------------------------------------------ #

# nc_ota_op_t
OTA_BEGIN = 0x01   # {u32 size, u32 crc32, u8 vlen, char v[]} -> {u32 resume_offset}
OTA_STATUS = 0x02  # -> {u8 state, u32 bytes_rx, u16 last_err}
OTA_FINISH = 0x03  # validate + set boot + restart
OTA_ABORT = 0x04

OTA_CHUNK_MAX = 240

# nc_ota_state_t
OTA_IDLE = 0
OTA_RECEIVING = 1
OTA_VALIDATING = 2
OTA_READY = 3
OTA_FAILED = 4

# nc_ota_err_t
OTAERR_NONE = 0
OTAERR_EXPECTED_OFFSET = 1  # host must GET_STATUS + reseek
OTAERR_SIZE = 2
OTAERR_CRC = 3
OTAERR_IMAGE = 4
OTAERR_FLASH = 5
OTAERR_STATE = 6


def build_ota_begin(tid: int, size: int, crc32: int, version: str) -> bytes:
    v = version.encode("utf-8")
    if len(v) > 255:
        raise ValueError("version string too long")
    payload = struct.pack("<IIB", size, crc32, len(v)) + v
    return build_control_request(OTA_BEGIN, tid, payload)


def parse_ota_begin_payload(payload: bytes) -> Tuple[int, int, str]:
    """Firmware-side view of an OTA_BEGIN request: -> (size, crc32, version)."""
    size, crc, vlen = struct.unpack_from("<IIB", payload)
    v = payload[9:9 + vlen]
    if len(v) != vlen or len(payload) != 9 + vlen:
        raise ValueError("OTA_BEGIN payload length mismatch")
    return size, crc, v.decode("utf-8")


def parse_ota_begin_resp(payload: bytes) -> int:
    """-> resume_offset"""
    return struct.unpack("<I", payload)[0]


def build_ota_status(tid: int) -> bytes:
    return build_control_request(OTA_STATUS, tid)


@dataclass
class OtaStatus:
    state: int     # nc_ota_state_t
    bytes_rx: int
    last_err: int  # nc_ota_err_t


def parse_ota_status_resp(payload: bytes) -> OtaStatus:
    return OtaStatus(*struct.unpack("<BIH", payload))


def build_ota_finish(tid: int) -> bytes:
    return build_control_request(OTA_FINISH, tid)


def build_ota_abort(tid: int) -> bytes:
    return build_control_request(OTA_ABORT, tid)


def build_ota_data(offset: int, data: bytes) -> bytes:
    if len(data) > OTA_CHUNK_MAX:
        raise ValueError(f"OTA data chunk {len(data)} > {OTA_CHUNK_MAX}")
    return struct.pack("<I", offset) + data


def parse_ota_data(buf: bytes) -> Tuple[int, bytes]:
    """-> (offset, data)"""
    if len(buf) < 4:
        raise ValueError("OTA data too short")
    return struct.unpack_from("<I", buf)[0], bytes(buf[4:])
