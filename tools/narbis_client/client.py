"""client.py — asyncio BLE client for the Narbis Edge Earclip.

Wire work goes exclusively through proto.py / uuids.py (the contract
mirrors of firmware proto.h). This module adds the session layer:

  * discovery/connection (bleak), MTU logging, robust disconnect
    handling with a reconnect helper that re-arms all subscriptions;
  * stream subscriptions delivering *parsed* objects (PpgBatch,
    AccelBatch, IbiBatch, EventBatch, Status);
  * CONTROL correlation: tid auto-increment 1..255, response matching
    by (tid, op) tolerant of interleaved unsolicited indications;
  * knob discovery/get/set/save/reset by name or id;
  * time sync with min-RTT pair selection;
  * self-test run + chunked result reassembly.

bleak is imported lazily so that the FakeBleak-based unit tests (and
`--help` on machines without BLE support) never touch it. Any object
implementing the small backend surface used here (connect, disconnect,
is_connected, mtu_size, start_notify, stop_notify, write_gatt_char,
read_gatt_char, address) can be injected via `backend=`.
"""

from __future__ import annotations

import asyncio
import logging
import struct
import time
from collections import deque
from typing import Awaitable, Callable, Dict, Optional, Tuple, Union

from . import proto as P
from . import uuids as U

log = logging.getLogger("narbis.client")

DEVICE_NAME = "Narbis Edge Earclip"

# Alias so callers can type-annotate discovery results without importing proto.
KnobInfo = P.KnobRecord

# stream name -> (characteristic UUID, batch parser)
STREAMS: Dict[str, Tuple[str, Callable[[bytes], object]]] = {
    "ppg": (U.UUID_PPG, P.parse_ppg),
    "accel": (U.UUID_ACCEL, P.parse_accel),
    "ibi": (U.UUID_IBI, P.parse_ibi),
    "event": (U.UUID_EVENT, P.parse_event_batch),
    "status": (U.UUID_STATUS, P.parse_status),
}

STREAM_MASK_ALL = (P.STREAM_MASK_PPG | P.STREAM_MASK_ACCEL |
                   P.STREAM_MASK_IBI | P.STREAM_MASK_EVENT)

CTRL_STATUS_NAMES = {
    P.ST_OK: "OK", P.ST_UNKNOWN_OP: "UNKNOWN_OP", P.ST_BAD_LEN: "BAD_LEN",
    P.ST_BAD_PARAM: "BAD_PARAM", P.ST_OUT_OF_RANGE: "OUT_OF_RANGE",
    P.ST_READ_ONLY: "READ_ONLY", P.ST_BUSY: "BUSY",
    P.ST_WRONG_STATE: "WRONG_STATE", P.ST_NVS_ERR: "NVS_ERR",
    P.ST_CRC_ERR: "CRC_ERR", P.ST_VERSION_MISMATCH: "VERSION_MISMATCH",
    P.ST_NEEDS_RESTART: "NEEDS_RESTART", P.ST_UNAUTHORIZED: "UNAUTHORIZED",
    P.ST_LOWBATT: "LOWBATT",
}

SELFTEST_NAMES = {
    P.TEST_I2C_SCAN: "i2c_scan", P.TEST_ACCEL_WHOAMI: "accel_whoami",
    P.TEST_AFE_REG_RW: "afe_reg_rw", P.TEST_AFE_DARK: "afe_dark",
    P.TEST_XTALK: "xtalk", P.TEST_ACCEL_ST: "accel_selftest",
    P.TEST_BATT: "battery", P.TEST_CHARGER: "charger",
}

TEST_STATUS_NAMES = {P.TR_PASS: "PASS", P.TR_FAIL: "FAIL", P.TR_SKIP: "SKIP"}


class NarbisError(Exception):
    """Base for all client-side errors."""


class DeviceNotFoundError(NarbisError):
    pass


class NarbisDisconnectedError(NarbisError):
    pass


class NarbisCtrlError(NarbisError):
    """Nonzero nc_ctrl_status_t in a CONTROL response."""

    def __init__(self, status: int, op: int, payload: bytes = b""):
        self.status = status
        self.op = op
        self.payload = payload
        name = CTRL_STATUS_NAMES.get(status, str(status))
        super().__init__(f"CONTROL op {op:#04x} failed: {name} ({status})")


async def find_device(address: Optional[str] = None,
                      name_prefix: str = DEVICE_NAME,
                      timeout: float = 10.0):
    """Scan for the earclip. Returns a bleak BLEDevice or None. The
    prefix match accepts factory/test-mode suffixes ('... TEST')."""
    from bleak import BleakScanner  # lazy: keep import-clean without BLE

    if address:
        return await BleakScanner.find_device_by_address(address, timeout=timeout)

    def _filt(device, adv) -> bool:
        n = adv.local_name or device.name
        return bool(n) and n.startswith(name_prefix)

    return await BleakScanner.find_device_by_filter(_filt, timeout=timeout)


class NarbisClient:
    """One BLE session to one earclip.

    Typical use::

        c = NarbisClient()               # discover by name
        await c.connect()
        await c.subscribe("ppg", on_ppg)
        await c.start_streams()
        ...
        await c.disconnect()
    """

    def __init__(self, address: Optional[str] = None,
                 name_prefix: str = DEVICE_NAME,
                 backend=None,
                 scan_timeout: float = 10.0):
        self.address = address
        self._name_prefix = name_prefix
        self._scan_timeout = scan_timeout
        self._backend = backend

        self._tid = 0
        # tid -> (expected op, Future[ControlResponse]); tid is the primary
        # correlation key across BOTH control chars (sensor CONTROL and
        # OTA_CTRL) because they share the allocator.
        self._pending: Dict[int, Tuple[int, asyncio.Future]] = {}
        # unsolicited/late CONTROL indications, kept for diagnostics
        self.unsolicited: deque = deque(maxlen=64)
        self.unsolicited_count = 0

        # stream name -> user callback, kept for resubscribe-on-reconnect
        self._subs: Dict[str, Callable] = {}
        self._ctrl_chars_armed: set = set()

        self.disconnected = asyncio.Event()
        self.on_disconnect: Optional[Callable[["NarbisClient"], None]] = None

        self.knobs: Dict[str, KnobInfo] = {}
        self._knob_by_id: Dict[int, str] = {}
        self.device_info: Dict[str, Optional[str]] = {}

        # monotonic-ish epoch clock in us; overridable for deterministic tests
        self._now_us: Callable[[], int] = lambda: time.time_ns() // 1000

    # ---------------------------------------------------------------- #
    # connection                                                        #
    # ---------------------------------------------------------------- #

    @property
    def is_connected(self) -> bool:
        try:
            return bool(self._backend and self._backend.is_connected)
        except Exception:
            return False

    async def connect(self, timeout: float = 20.0) -> None:
        if self._backend is None:
            dev = await find_device(self.address, self._name_prefix,
                                    self._scan_timeout)
            if dev is None:
                raise DeviceNotFoundError(
                    f"no device found (address={self.address!r}, "
                    f"name prefix {self._name_prefix!r})")
            from bleak import BleakClient
            self._backend = BleakClient(
                dev, disconnected_callback=self._handle_disconnect,
                timeout=timeout)
            self.address = getattr(dev, "address", self.address)
        else:
            # Injected backends (FakeBleak) expose an attribute instead of
            # the constructor argument.
            if hasattr(self._backend, "set_disconnected_callback"):
                self._backend.set_disconnected_callback(self._handle_disconnect)
            self.address = getattr(self._backend, "address", self.address)

        self.disconnected.clear()
        await self._backend.connect()
        try:
            log.info("connected to %s, ATT MTU %d", self.address,
                     self._backend.mtu_size)
        except Exception:
            log.info("connected to %s (MTU unavailable on this backend)",
                     self.address)

        await self._arm_control_char(U.UUID_CONTROL, required=True)
        # OTA service is optional on stripped builds — arm if present.
        await self._arm_control_char(U.UUID_OTA_CTRL, required=False)

        # Re-arm stream subscriptions (no-op on first connect).
        for name, cb in list(self._subs.items()):
            await self._start_stream_notify(name, cb)

    async def _arm_control_char(self, char_uuid: str, required: bool) -> None:
        try:
            await self._backend.start_notify(char_uuid, self._on_control)
            self._ctrl_chars_armed.add(char_uuid)
        except Exception as e:
            if required:
                raise
            log.debug("optional control char %s not armed: %s", char_uuid, e)

    async def disconnect(self) -> None:
        if self._backend is not None:
            try:
                await self._backend.disconnect()
            except Exception as e:
                log.debug("disconnect: %s", e)

    def _handle_disconnect(self, _client=None) -> None:
        """bleak disconnected_callback — runs on the event loop thread."""
        log.warning("BLE link dropped (%s)", self.address)
        self.disconnected.set()
        self._ctrl_chars_armed.clear()
        for tid, (op, fut) in list(self._pending.items()):
            if not fut.done():
                fut.set_exception(NarbisDisconnectedError(
                    f"disconnected while waiting for op {op:#04x} tid {tid}"))
        self._pending.clear()
        if self.on_disconnect is not None:
            try:
                self.on_disconnect(self)
            except Exception:
                log.exception("on_disconnect hook raised")

    async def wait_for_disconnect(self, timeout: Optional[float] = None) -> bool:
        try:
            await asyncio.wait_for(self.disconnected.wait(), timeout)
            return True
        except asyncio.TimeoutError:
            return False

    async def reconnect(self, attempts: int = 8, delay: float = 1.0,
                        backoff: float = 1.5) -> None:
        """Awaitable reconnect helper: fresh backend to the same address,
        re-arms CONTROL and every previously subscribed stream."""
        last_err: Optional[Exception] = None
        for i in range(attempts):
            try:
                # Force a fresh backend unless one was injected (tests): a
                # half-dead BleakClient cannot be reused reliably on WinRT.
                if self._backend is not None and \
                        not hasattr(self._backend, "set_disconnected_callback"):
                    self._backend = None
                await self.connect()
                return
            except Exception as e:  # noqa: BLE001 - retry loop by design
                last_err = e
                log.info("reconnect attempt %d/%d failed: %s", i + 1, attempts, e)
                await asyncio.sleep(delay)
                delay *= backoff
        raise NarbisDisconnectedError(f"reconnect failed after {attempts} "
                                      f"attempts: {last_err}")

    # ---------------------------------------------------------------- #
    # streams                                                           #
    # ---------------------------------------------------------------- #

    async def subscribe(self, stream_name: str,
                        callback: Callable[[object], Union[None, Awaitable[None]]]
                        ) -> None:
        """Subscribe to one of ppg/accel/ibi/event/status. `callback`
        receives the *parsed* object (PpgBatch/AccelBatch/IbiBatch/
        EventBatch/Status)."""
        if stream_name not in STREAMS:
            raise ValueError(f"unknown stream {stream_name!r} "
                             f"(want one of {sorted(STREAMS)})")
        self._subs[stream_name] = callback
        await self._start_stream_notify(stream_name, callback)

    async def _start_stream_notify(self, stream_name: str,
                                   callback: Callable) -> None:
        char_uuid, parser = STREAMS[stream_name]

        def _on_notify(_sender, data) -> None:
            try:
                obj = parser(bytes(data))
            except ValueError as e:
                log.error("%s: unparseable notification (%d B): %s",
                          stream_name, len(data), e)
                return
            r = callback(obj)
            if asyncio.iscoroutine(r):
                asyncio.ensure_future(r)

        await self._backend.start_notify(char_uuid, _on_notify)

    async def unsubscribe(self, stream_name: str) -> None:
        char_uuid, _ = STREAMS[stream_name]
        self._subs.pop(stream_name, None)
        try:
            await self._backend.stop_notify(char_uuid)
        except Exception as e:
            log.debug("stop_notify %s: %s", stream_name, e)

    # ---------------------------------------------------------------- #
    # CONTROL correlation                                               #
    # ---------------------------------------------------------------- #

    def _next_tid(self) -> int:
        """1..255 wrap; 0 never used so an all-zero packet can't correlate.
        Skips tids still in flight (255 outstanding would deadlock long
        before this matters)."""
        for _ in range(255):
            self._tid = (self._tid % 255) + 1
            if self._tid not in self._pending:
                return self._tid
        raise NarbisError("no free control tid (255 requests in flight?)")

    def _on_control(self, _sender, data) -> None:
        try:
            resp = P.parse_control_response(bytes(data))
        except ValueError as e:
            log.error("bad CONTROL indication (%d B): %s", len(data), e)
            return
        ent = self._pending.get(resp.tid)
        # Match tid first, then op: a stale response to a timed-out request
        # whose tid got reused must not resolve the wrong future.
        if ent is not None and ent[0] == resp.op:
            del self._pending[resp.tid]
            if not ent[1].done():
                ent[1].set_result(resp)
            return
        self.unsolicited.append(resp)
        self.unsolicited_count += 1
        log.debug("unsolicited CONTROL indication op=%#04x tid=%d status=%d",
                  resp.op, resp.tid, resp.status)

    async def control_call(self, builder: Callable[[int], bytes], op: int,
                           timeout: float = 5.0,
                           char_uuid: str = U.UUID_CONTROL,
                           ok_statuses: Tuple[int, ...] = (P.ST_OK,)
                           ) -> P.ControlResponse:
        """Send one CONTROL request built by `builder(tid)` (a proto.py
        request builder partial) and await the matching indication.

        Response matching: TEST-block ops (>= 0xE0) respond with op ==
        request op; all others with op|0x80 — parse_control_response
        normalizes both to the request opcode, so the expected op here is
        always `op` itself.
        """
        if self._backend is None:
            raise NarbisError("not connected")
        tid = self._next_tid()
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[tid] = (op, fut)
        try:
            await self._backend.write_gatt_char(char_uuid, builder(tid),
                                                response=True)
            resp: P.ControlResponse = await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            self._pending.pop(tid, None)
            raise NarbisError(
                f"CONTROL op {op:#04x} tid {tid}: no response in {timeout}s"
            ) from None
        except Exception:
            self._pending.pop(tid, None)
            raise
        if resp.status not in ok_statuses:
            raise NarbisCtrlError(resp.status, op, resp.payload)
        return resp

    async def control(self, opcode: int, payload: bytes = b"",
                      timeout: float = 5.0,
                      char_uuid: str = U.UUID_CONTROL,
                      ok_statuses: Tuple[int, ...] = (P.ST_OK,)
                      ) -> P.ControlResponse:
        """Generic CONTROL op: raw opcode + payload, raises
        NarbisCtrlError(status) on any status not in `ok_statuses`."""
        return await self.control_call(
            lambda tid: P.build_control_request(opcode, tid, payload),
            opcode, timeout=timeout, char_uuid=char_uuid,
            ok_statuses=ok_statuses)

    # ---------------------------------------------------------------- #
    # knob API                                                          #
    # ---------------------------------------------------------------- #

    async def discover_knobs(self, timeout: float = 5.0) -> Dict[str, KnobInfo]:
        """Walk OP_KNOB_DISCOVER chunks until all `total` records arrived.
        Returns {name: KnobInfo}; also cached on self.knobs."""
        knobs: Dict[str, KnobInfo] = {}
        by_id: Dict[int, str] = {}
        idx = 0
        total = None
        while total is None or idx < total:
            resp = await self.control_call(
                lambda tid, i=idx: P.build_knob_discover(tid, i),
                P.OP_KNOB_DISCOVER, timeout=timeout)
            chunk = P.parse_knob_discover_chunk(resp.payload)
            if total is None:
                total = chunk.total
            elif chunk.total != total:
                raise NarbisError(f"knob table changed mid-walk "
                                  f"({total} -> {chunk.total})")
            if chunk.first_idx != idx:
                raise NarbisError(f"knob discover chunk starts at "
                                  f"{chunk.first_idx}, expected {idx}")
            if not chunk.records and idx < total:
                raise NarbisError("empty knob discover chunk before end")
            for rec in chunk.records:
                knobs[rec.name] = rec
                by_id[rec.id] = rec.name
            idx += len(chunk.records)
        self.knobs = knobs
        self._knob_by_id = by_id
        return knobs

    async def _knob_id(self, knob: Union[str, int]) -> int:
        if isinstance(knob, int):
            return knob
        if not self.knobs:
            await self.discover_knobs()
        try:
            return self.knobs[knob].id
        except KeyError:
            raise NarbisError(f"unknown knob {knob!r}") from None

    async def knob_get(self, knob: Union[str, int]) -> int:
        kid = await self._knob_id(knob)
        resp = await self.control_call(
            lambda tid: P.build_knob_get(tid, kid), P.OP_KNOB_GET)
        rid, value = P.parse_knob_get_resp(resp.payload)
        if rid != kid:
            raise NarbisError(f"KNOB_GET id mismatch: asked {kid}, got {rid}")
        name = self._knob_by_id.get(kid)
        if name:
            self.knobs[name].current = value
        return value

    async def knob_set(self, knob: Union[str, int], value: int) -> int:
        """Returns the response status (ST_OK, or ST_NEEDS_RESTART for
        KF_REBOOT knobs — both are success)."""
        kid = await self._knob_id(knob)
        resp = await self.control_call(
            lambda tid: P.build_knob_set(tid, kid, value), P.OP_KNOB_SET,
            ok_statuses=(P.ST_OK, P.ST_NEEDS_RESTART))
        name = self._knob_by_id.get(kid)
        if name:
            self.knobs[name].current = value
        return resp.status

    async def knob_save(self) -> None:
        await self.control_call(P.build_knob_save, P.OP_KNOB_SAVE)

    async def knob_reset(self, scope_nvs: bool = False) -> None:
        """scope_nvs False: RAM only; True: RAM + NVS."""
        await self.control_call(
            lambda tid: P.build_knob_reset(tid, 1 if scope_nvs else 0),
            P.OP_KNOB_RESET)

    # ---------------------------------------------------------------- #
    # time sync                                                         #
    # ---------------------------------------------------------------- #

    async def time_sync(self, n: int = 5,
                        timeout: float = 5.0) -> Tuple[int, int]:
        """n TIME_SYNC exchanges, min-RTT pair selection.

        Returns (offset_us, rtt_us) where offset_us = dev_t_us -
        host_epoch_us at the same instant, i.e. dev = host + offset.
        The midpoint assumption (device sampled its clock at host_send +
        rtt/2) is the standard NTP-style estimate; min-RTT picks the
        exchange where that assumption is tightest.
        """
        if n < 1:
            raise ValueError("n must be >= 1")
        best: Optional[Tuple[int, int]] = None  # (rtt, offset)
        for _ in range(n):
            t_send = self._now_us()
            resp = await self.control_call(
                lambda tid, t=t_send: P.build_time_sync(tid, t),
                P.OP_TIME_SYNC, timeout=timeout)
            t_recv = self._now_us()
            echo, dev_t_us = P.parse_time_sync_resp(resp.payload)
            if echo != t_send:
                log.warning("TIME_SYNC echo mismatch (sent %d, echo %d)",
                            t_send, echo)
                continue
            rtt = t_recv - t_send
            offset = dev_t_us - (t_send + rtt // 2)
            if best is None or rtt < best[0]:
                best = (rtt, offset)
        if best is None:
            raise NarbisError("time_sync: no valid exchange")
        return best[1], best[0]

    async def get_device_time(self) -> int:
        resp = await self.control_call(P.build_get_time, P.OP_GET_TIME)
        return P.parse_get_time_resp(resp.payload)

    # ---------------------------------------------------------------- #
    # self-test                                                         #
    # ---------------------------------------------------------------- #

    async def selftest_run(self, test_mask: int = 0,
                           timeout: float = 30.0) -> None:
        """Kick a self-test run (0 = all tests). The run itself is
        synchronous on-device; give it a generous timeout."""
        await self.control_call(
            lambda tid: P.build_selftest_run(tid, test_mask),
            P.OP_SELFTEST_RUN, timeout=timeout)

    async def selftest_read_blob(self, timeout: float = 5.0) -> P.SelftestBlob:
        """Reassemble the result blob from SELFTEST_RESULT chunks."""
        data = bytearray()
        off = 0
        total = None
        while total is None or off < total:
            resp = await self.control_call(
                lambda tid, o=off: P.build_selftest_result(tid, o),
                P.OP_SELFTEST_RESULT, timeout=timeout)
            chunk = P.parse_chunk(resp.payload)
            if total is None:
                total = chunk.total
            if chunk.offset != off:
                raise NarbisError(f"selftest chunk at {chunk.offset}, "
                                  f"expected {off}")
            if not chunk.data and off < total:
                raise NarbisError("empty selftest chunk before end")
            data += chunk.data
            off += len(chunk.data)
        return P.parse_selftest_blob(bytes(data))

    # ---------------------------------------------------------------- #
    # convenience ops                                                   #
    # ---------------------------------------------------------------- #

    async def start_streams(self, mask: int = STREAM_MASK_ALL) -> None:
        await self.control_call(lambda tid: P.build_stream_start(tid, mask),
                                P.OP_STREAM_START)

    async def stop_streams(self, mask: int = STREAM_MASK_ALL) -> None:
        await self.control_call(lambda tid: P.build_stream_stop(tid, mask),
                                P.OP_STREAM_STOP)

    async def set_rate(self, rate: int) -> None:
        """Accepts a rate code (0..4) or a literal sps value (50..500)."""
        code = rate
        if rate not in P.RATE_SPS:
            for c, sps in P.RATE_SPS.items():
                if sps == rate:
                    code = c
                    break
            else:
                raise ValueError(f"bad rate {rate}: want code 0..4 or sps in "
                                 f"{sorted(P.RATE_SPS.values())}")
        await self.control_call(lambda tid: P.build_set_rate(tid, code),
                                P.OP_SET_RATE)

    async def marker(self, marker_id: int = 0) -> None:
        await self.control_call(lambda tid: P.build_marker(tid, marker_id),
                                P.OP_MARKER)

    async def agc_freeze(self, freeze: bool) -> None:
        await self.control_call(lambda tid: P.build_agc_freeze(tid, freeze),
                                P.OP_AGC_FREEZE)

    async def agc_manual(self, ir_ma: int, red_ma: int, rf_code: int,
                         apply_mask: int) -> None:
        await self.control_call(
            lambda tid: P.build_agc_manual(tid, ir_ma, red_ma, rf_code,
                                           apply_mask), P.OP_AGC_MANUAL)

    async def enter_ota(self) -> None:
        await self.control_call(P.build_enter_ota, P.OP_ENTER_OTA)

    async def power_off(self) -> None:
        """Device may drop the link before (or instead of) indicating —
        treat timeout/disconnect as success."""
        try:
            await self.control_call(P.build_power_off, P.OP_POWER_OFF,
                                    timeout=3.0)
        except (NarbisError, NarbisDisconnectedError) as e:
            log.debug("power_off: link dropped as expected (%s)", e)

    async def reboot(self) -> None:
        try:
            await self.control_call(P.build_reboot, P.OP_REBOOT, timeout=3.0)
        except (NarbisError, NarbisDisconnectedError) as e:
            log.debug("reboot: link dropped as expected (%s)", e)

    async def factory_reset(self) -> None:
        await self.control_call(P.build_factory_reset, P.OP_FACTORY_RESET)

    # ---------------------------------------------------------------- #
    # device info                                                       #
    # ---------------------------------------------------------------- #

    async def read_device_info(self) -> Dict[str, Optional[str]]:
        """DIS strings + battery level + protocol version char. Missing
        chars come back as None rather than raising."""
        async def rd_str(uuid: str) -> Optional[str]:
            try:
                return bytes(await self._backend.read_gatt_char(uuid)).decode(
                    "utf-8", "replace")
            except Exception:
                return None

        info: Dict[str, Optional[str]] = {
            "address": self.address,
            "manufacturer": await rd_str(U.UUID_MANUFACTURER_NAME),
            "model": await rd_str(U.UUID_MODEL_NUMBER),
            "fw_revision": await rd_str(U.UUID_FIRMWARE_REVISION),
            "hw_revision": await rd_str(U.UUID_HARDWARE_REVISION),
            "sw_revision": await rd_str(U.UUID_SOFTWARE_REVISION),
        }
        try:
            raw = bytes(await self._backend.read_gatt_char(U.UUID_PROTO_VER))
            if len(raw) >= 2:
                ver = struct.unpack_from("<H", raw)[0]
                info["proto_ver"] = f"{ver >> 8}.{ver & 0xFF}"
        except Exception:
            info["proto_ver"] = None
        try:
            raw = bytes(await self._backend.read_gatt_char(U.UUID_BATTERY_LEVEL))
            info["battery_pct"] = str(raw[0]) if raw else None
        except Exception:
            info["battery_pct"] = None
        self.device_info = info
        return info

    # raw char access for modules that need it (ota.py OTA_DATA writes)
    async def write_char(self, char_uuid: str, data: bytes,
                         response: bool = False) -> None:
        await self._backend.write_gatt_char(char_uuid, data, response=response)


# ---------------------------------------------------------------------- #
# shared CLI plumbing for tools/scripts and tools/acceptance              #
# ---------------------------------------------------------------------- #

def add_device_args(parser) -> None:
    parser.add_argument("--address", "-a", default=None,
                        help="BLE address (skips name scan)")
    parser.add_argument("--name", default=DEVICE_NAME,
                        help=f"advertised name prefix (default {DEVICE_NAME!r})")
    parser.add_argument("--scan-timeout", type=float, default=10.0,
                        help="discovery timeout, s (default 10)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="debug logging")


def client_from_args(args) -> NarbisClient:
    logging.basicConfig(
        level=logging.DEBUG if getattr(args, "verbose", False) else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    return NarbisClient(address=args.address, name_prefix=args.name,
                        scan_timeout=args.scan_timeout)


async def connect_or_exit(client: NarbisClient) -> None:
    """Connect; on discovery failure (or an unusable BLE stack — adapter
    off, no adapter) print a clean one-line message and exit(1) — every
    CLI must be runnable to this point without hardware."""
    import sys
    try:
        from bleak.exc import BleakError
    except ImportError:  # bleak missing: connect() will raise ImportError
        BleakError = ()  # type: ignore[assignment]
    try:
        await client.connect()
    except DeviceNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
    except (BleakError, OSError, asyncio.TimeoutError) as e:
        print(f"error: BLE unavailable/connect failed: {e}", file=sys.stderr)
        sys.exit(1)
