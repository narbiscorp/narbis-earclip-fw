"""recorder.py — record every stream to CSV (+ optional parquet mirrors).

Layout of an output directory:

    ppg.csv     seq,i,t_us,ir,red,amb,rate_code,flags
    accel.csv   seq,i,t_us,x,y,z,odr_code,flags
    ibi.csv     seq,t_beat_us,ibi_ms,confidence,flags
    events.csv  seq,type,name,t_us,detail        (detail = k=v;k=v)
    status.csv  host_t_us,<all Status fields>,extra_hex
    session.json  device info, host/device time pairs, knob snapshot, versions
    gaps.json     every sequence discontinuity, per stream

Per-batch sample rows expand with t = t0 + round(i * 1e6 / nominal_sps)
using the rate/odr code carried in that batch header — the nominal grid,
not a resampled one (offline analysis re-grids as it pleases).

CSV is always written incrementally (a crash loses at most the current
OS buffer). Parquet mirrors are produced at close() iff pyarrow imports.

The Recorder core is transport-free: feed_ppg()/feed_accel()/... take
parsed proto objects, so tests drive it with synthetic batches and the
live path is just subscriptions wired to the feeders.
"""

from __future__ import annotations

import asyncio
import csv
import json
import logging
import time
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional

from . import __version__ as _tools_version
from . import proto as P

log = logging.getLogger("narbis.recorder")

_U32 = 0xFFFFFFFF

_EVENT_NAMES = {
    P.EV_AGC_STEP: "agc_step", P.EV_GATE: "gate", P.EV_WEAR: "wear",
    P.EV_MARKER: "marker", P.EV_ERROR: "error",
    P.EV_RATE_CHANGE: "rate_change", P.EV_AGC_OFFDAC: "agc_offdac",
    P.EV_SELFTEST_DONE: "selftest_done",
}

_STATUS_FIELDS = [
    "sys_state", "flags", "batt_mv", "batt_pct", "ppg_rate_code",
    "led_ir_ma", "led_red_ma", "tia_gain_code", "tia_cf_code",
    "gate_duty_x100", "notif_drop_count", "i2c_err_count",
    "clock_drift_ppm_x10", "uptime_s", "ibi_last_ms", "hr_bpm",
]


class GapTracker:
    """Per-stream u32 sequence continuity. seq increments by 1 per batch
    and wraps at 2^32; anything else is a gap (lost notifications)."""

    def __init__(self, stream: str):
        self.stream = stream
        self.last_seq: Optional[int] = None
        self.batches = 0
        self.gaps: List[dict] = []

    def feed(self, seq: int) -> Optional[dict]:
        self.batches += 1
        gap = None
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & _U32
            if seq != expected:
                lost = (seq - expected) & _U32
                gap = {"stream": self.stream, "expected": expected,
                       "got": seq, "lost_batches": lost,
                       "host_t_us": time.time_ns() // 1000}
                self.gaps.append(gap)
        self.last_seq = seq
        return gap


class Recorder:
    def __init__(self, outdir, warn_gaps: bool = True):
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.warn_gaps = warn_gaps
        self.trackers: Dict[str, GapTracker] = {
            s: GapTracker(s) for s in ("ppg", "accel", "ibi", "event", "status")}
        self.sample_counts: Dict[str, int] = {"ppg": 0, "accel": 0, "ibi": 0,
                                              "event": 0, "status": 0}
        self._files: Dict[str, object] = {}
        self._writers: Dict[str, csv.writer] = {}
        self._closed = False

        self._open("ppg", ["seq", "i", "t_us", "ir", "red", "amb",
                           "rate_code", "flags"])
        self._open("accel", ["seq", "i", "t_us", "x", "y", "z",
                             "odr_code", "flags"])
        self._open("ibi", ["seq", "t_beat_us", "ibi_ms", "confidence", "flags"])
        self._open("events", ["seq", "type", "name", "t_us", "detail"])
        self._open("status", ["host_t_us"] + _STATUS_FIELDS + ["extra_hex"])

    def _open(self, name: str, header: List[str]) -> None:
        f = open(self.outdir / f"{name}.csv", "w", newline="",
                 encoding="utf-8")
        w = csv.writer(f)
        w.writerow(header)
        self._files[name] = f
        self._writers[name] = w

    def _gap(self, tracker: GapTracker, seq: int) -> None:
        gap = tracker.feed(seq)
        if gap and self.warn_gaps:
            log.warning("SEQ GAP on %s: expected %d got %d (%d batch(es) lost)",
                        gap["stream"], gap["expected"], gap["got"],
                        gap["lost_batches"])

    # ------------------------------------------------------------ feeders

    def feed_ppg(self, b: P.PpgBatch) -> None:
        self._gap(self.trackers["ppg"], b.seq)
        sps = P.rate_sps(b.rate_code)
        w = self._writers["ppg"]
        for i in range(b.n):
            t = b.t0_us + (round(i * 1_000_000 / sps) if sps else 0)
            amb = b.amb[i] if b.amb is not None else ""
            w.writerow([b.seq, i, t, b.ir[i], b.red[i], amb,
                        b.rate_code, b.flags])
        self.sample_counts["ppg"] += b.n

    def feed_accel(self, b: P.AccelBatch) -> None:
        self._gap(self.trackers["accel"], b.seq)
        hz = P.acc_odr_hz(b.odr_code)
        w = self._writers["accel"]
        for i, (x, y, z) in enumerate(b.samples):
            t = b.t0_us + (round(i * 1_000_000 / hz) if hz else 0)
            w.writerow([b.seq, i, t, x, y, z, b.odr_code, b.flags])
        self.sample_counts["accel"] += b.n

    def feed_ibi(self, b: P.IbiBatch) -> None:
        self._gap(self.trackers["ibi"], b.seq)
        w = self._writers["ibi"]
        for r in b.records:
            w.writerow([b.seq, r.t_beat_us, r.ibi_ms, r.confidence, r.flags])
        self.sample_counts["ibi"] += b.n

    def feed_event(self, b: P.EventBatch) -> None:
        self._gap(self.trackers["event"], b.seq)
        w = self._writers["events"]
        for ev in b.events:
            if isinstance(ev, P.EvUnknown):
                name, t_us = "unknown", ev.t_us if ev.t_us is not None else ""
                detail = "payload=" + ev.payload.hex()
            else:
                name = _EVENT_NAMES.get(ev.type, f"type{ev.type}")
                t_us = ev.t_us
                d = asdict(ev)
                d.pop("t_us", None)
                d.pop("type", None)
                detail = ";".join(f"{k}={v}" for k, v in d.items())
            w.writerow([b.seq, ev.type, name, t_us, detail])
        self.sample_counts["event"] += b.n

    def feed_status(self, s: P.Status) -> None:
        # STATUS has no seq — 1 Hz notify, no continuity to check.
        w = self._writers["status"]
        row = [time.time_ns() // 1000]
        row += [getattr(s, f) for f in _STATUS_FIELDS]
        row.append(s.extra.hex())
        w.writerow(row)
        self.sample_counts["status"] += 1

    # ------------------------------------------------------------ close

    @property
    def total_gaps(self) -> int:
        return sum(len(t.gaps) for t in self.trackers.values())

    def gap_report(self) -> dict:
        return {
            "total_gaps": self.total_gaps,
            "per_stream": {
                s: {"batches": t.batches, "gaps": t.gaps}
                for s, t in self.trackers.items()},
        }

    def close(self, session: Optional[dict] = None) -> dict:
        """Flush + close all CSVs, write gaps.json (+ session.json when
        session metadata given), emit parquet mirrors if pyarrow imports.
        Returns the gap report. Idempotent."""
        if self._closed:
            return self.gap_report()
        self._closed = True
        for f in self._files.values():
            f.close()

        report = self.gap_report()
        (self.outdir / "gaps.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8")
        if report["total_gaps"]:
            log.warning("recording finished with %d sequence gap(s) — "
                        "see gaps.json", report["total_gaps"])

        if session is not None:
            (self.outdir / "session.json").write_text(
                json.dumps(session, indent=2, default=str), encoding="utf-8")

        self._write_parquet_mirrors()
        return report

    def _write_parquet_mirrors(self) -> None:
        try:
            import pyarrow.csv as pacsv
            import pyarrow.parquet as pq
        except ImportError:
            log.info("pyarrow not installed — CSV only (pip install "
                     "narbis-tools[data])")
            return
        for name in ("ppg", "accel", "ibi", "events", "status"):
            src = self.outdir / f"{name}.csv"
            try:
                table = pacsv.read_csv(src)
                pq.write_table(table, self.outdir / f"{name}.parquet")
            except Exception as e:  # a mirror must never kill the recording
                log.warning("parquet mirror for %s failed: %s", name, e)


async def record(client, outdir, duration: Optional[float] = None,
                 stream_mask: Optional[int] = None,
                 timesync_period_s: float = 60.0) -> dict:
    """Record all streams for `duration` seconds (None = until cancelled /
    Ctrl+C). Time-syncs at start, every `timesync_period_s`, and at end;
    the (host_us, dev offset/rtt) pairs land in session.json so offline
    tools can map device t_us to host epoch with drift correction.

    Returns the gap report dict.
    """
    from .client import STREAM_MASK_ALL  # local import: avoid cycle at import

    rec = Recorder(outdir)
    session: dict = {
        "tool": "narbis_client.recorder",
        "sw_version": _tools_version,
        "proto_ver": f"{P.PROTO_VER_MAJOR}.{P.PROTO_VER_MINOR}",
        "start_host_us": time.time_ns() // 1000,
        "time_sync": [],
    }

    try:
        session["device_info"] = await client.read_device_info()
        knobs = await client.discover_knobs()
        session["knobs"] = {name: asdict(k) for name, k in knobs.items()}
    except Exception as e:
        log.warning("session metadata incomplete: %s", e)

    async def _sync(tag: str) -> None:
        try:
            offset_us, rtt_us = await client.time_sync()
            session["time_sync"].append({
                "tag": tag, "host_us": time.time_ns() // 1000,
                "offset_us": offset_us, "rtt_us": rtt_us})
        except Exception as e:
            log.warning("time_sync (%s) failed: %s", tag, e)

    await _sync("start")

    await client.subscribe("ppg", rec.feed_ppg)
    await client.subscribe("accel", rec.feed_accel)
    await client.subscribe("ibi", rec.feed_ibi)
    await client.subscribe("event", rec.feed_event)
    await client.subscribe("status", rec.feed_status)
    await client.start_streams(STREAM_MASK_ALL if stream_mask is None
                               else stream_mask)

    t_end = None if duration is None else time.monotonic() + duration
    try:
        next_sync = time.monotonic() + timesync_period_s
        while t_end is None or time.monotonic() < t_end:
            await asyncio.sleep(0.2)
            if client.disconnected.is_set():
                log.error("device disconnected mid-recording")
                break
            if time.monotonic() >= next_sync:
                await _sync("periodic")
                next_sync += timesync_period_s
    except (KeyboardInterrupt, asyncio.CancelledError):
        log.info("recording interrupted — closing files")
    finally:
        if client.is_connected:
            try:
                await client.stop_streams()
            except Exception as e:
                log.debug("stop_streams: %s", e)
            await _sync("end")
        session["end_host_us"] = time.time_ns() // 1000
        session["sample_counts"] = dict(rec.sample_counts)
        report = rec.close(session)
    n = rec.sample_counts
    log.info("recorded: %d ppg, %d accel, %d ibi samples, %d events, "
             "%d status; %d gap(s) -> %s",
             n["ppg"], n["accel"], n["ibi"], n["event"], n["status"],
             report["total_gaps"], rec.outdir)
    return report


def main(argv=None) -> int:
    """CLI entry (console_script narbis-record, tools/scripts/narbis_record.py)."""
    import argparse
    from .client import add_device_args, client_from_args, connect_or_exit

    ap = argparse.ArgumentParser(
        prog="narbis-record",
        description="Record all Narbis earclip streams to CSV (+ parquet).")
    add_device_args(ap)
    ap.add_argument("--out", "-o", default=None,
                    help="output directory (default rec_YYYYmmdd_HHMMSS)")
    ap.add_argument("--duration", "-d", type=float, default=None,
                    help="seconds to record (default: until Ctrl+C)")
    ap.add_argument("--rate", type=int, default=None,
                    help="set PPG rate first (code 0..4 or sps)")
    args = ap.parse_args(argv)

    outdir = args.out or time.strftime("rec_%Y%m%d_%H%M%S")

    async def run() -> int:
        client = client_from_args(args)
        await connect_or_exit(client)
        try:
            if args.rate is not None:
                await client.set_rate(args.rate)
            report = await record(client, outdir, duration=args.duration)
            return 0 if report["total_gaps"] == 0 else 2
        finally:
            await client.disconnect()

    try:
        return asyncio.run(run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
