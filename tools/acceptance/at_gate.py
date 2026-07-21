#!/usr/bin/env python3
"""AT: artifact gating — §5.9 DoD (guided, needs an operator wearing
the clip).

Three phases, operator-prompted:
  1. STILL  (--still, default 30 s): baseline, gate should stay open;
  2. SHAKE  (--shake, default 15 s): operator shakes head / taps the
     clip continuously;
  3. STILL  again (recovery).

Checks (device-time phase windows via time_sync):
  * >=1 gate-on span overlapping the shake window (EV_GATE events);
  * ZERO IBI records with t_beat inside any gate-on span — gating must
    suppress beats, not just annotate (proto flag IBIF_GATED_CTX marks
    post-gap context beats, which are legal OUTSIDE spans);
  * gate duty inside the still windows < --max-still-duty (default 2 %).

PASS/FAIL per check + overall. Raw samples keep flowing while gated
(annotation only) — verified by PPG batches arriving during shake.
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client import proto as P  # noqa: E402
from narbis_client.client import (add_device_args, client_from_args,  # noqa: E402
                                  connect_or_exit)


class GateLog:
    def __init__(self):
        self.spans = []          # [t_on_us, t_off_us|None] device time
        self.ibis = []           # (t_beat_us, flags)
        self.ppg_batches = []    # (host_mono, t0_us)

    def on_event(self, b: P.EventBatch):
        for ev in b.events:
            if isinstance(ev, P.EvGate):
                if ev.state:
                    if not self.spans or self.spans[-1][1] is not None:
                        self.spans.append([ev.t_us, None])
                elif self.spans and self.spans[-1][1] is None:
                    self.spans[-1][1] = ev.t_us

    def on_ibi(self, b: P.IbiBatch):
        for r in b.records:
            self.ibis.append((r.t_beat_us, r.flags))

    def on_ppg(self, b: P.PpgBatch):
        self.ppg_batches.append((time.monotonic(), b.t0_us))

    def closed_spans(self, t_end_us):
        return [(a, b if b is not None else t_end_us) for a, b in self.spans]


def overlap_us(a0, a1, b0, b1):
    return max(0, min(a1, b1) - max(a0, b0))


async def prompt(text: str):
    print(f"\n>>> {text}")
    await asyncio.to_thread(input, "    press Enter when ready... ")


async def run(args) -> int:
    client = client_from_args(args)
    await connect_or_exit(client)
    gl = GateLog()
    phases = {}  # name -> (dev_t0_us, dev_t1_us)

    try:
        offset_us, rtt_us = await client.time_sync()
        print(f"time sync: offset {offset_us} us, rtt {rtt_us} us")

        def dev_now_us():
            return time.time_ns() // 1000 + offset_us

        await client.subscribe("event", gl.on_event)
        await client.subscribe("ibi", gl.on_ibi)
        await client.subscribe("ppg", gl.on_ppg)
        await client.start_streams(P.STREAM_MASK_PPG | P.STREAM_MASK_IBI |
                                   P.STREAM_MASK_EVENT |
                                   P.STREAM_MASK_ACCEL)

        async def phase(name: str, seconds: float, instruction: str):
            await prompt(instruction)
            t0 = dev_now_us()
            print(f"    {name}: {seconds:.0f} s ...")
            await asyncio.sleep(seconds)
            phases[name] = (t0, dev_now_us())

        await phase("still1", args.still,
                    f"Wear the clip and hold STILL for {args.still:.0f} s")
        await phase("shake", args.shake,
                    f"Now SHAKE continuously (nod/shake head, tap the "
                    f"cable) for {args.shake:.0f} s")
        await phase("still2", args.still,
                    f"Hold STILL again for {args.still:.0f} s")

        await client.stop_streams()
    finally:
        await client.disconnect()

    t_end_us = max(t1 for _, t1 in phases.values())
    spans = gl.closed_spans(t_end_us)
    checks = []

    # 1. gate-on during shake
    sh0, sh1 = phases["shake"]
    shake_overlap = sum(overlap_us(a, b, sh0, sh1) for a, b in spans)
    checks.append(("gate-on span(s) during shake",
                   shake_overlap > 0,
                   f"{shake_overlap / 1e6:.1f} s gated of "
                   f"{(sh1 - sh0) / 1e6:.1f} s"))

    # 2. zero IBIs inside gate-on spans
    bad_ibis = [t for t, fl in gl.ibis
                if any(a <= t <= b for a, b in spans)]
    checks.append(("zero IBI beats inside gate-on spans",
                   len(bad_ibis) == 0,
                   f"{len(bad_ibis)} beat(s) inside spans, "
                   f"{len(gl.ibis)} total"))

    # 3. still-phase gate duty
    for name in ("still1", "still2"):
        p0, p1 = phases[name]
        duty = (sum(overlap_us(a, b, p0, p1) for a, b in spans)
                / max(1, p1 - p0))
        checks.append((f"{name} gate duty < {args.max_still_duty * 100:.0f}%",
                       duty < args.max_still_duty, f"{duty * 100:.2f}%"))

    # 4. raw stream never stops (annotation-only gating)
    shake_wall = (sh1 - sh0) / 1e6
    ppg_in_shake = sum(1 for _, t0 in gl.ppg_batches if sh0 <= t0 <= sh1)
    checks.append(("PPG batches kept flowing while gated",
                   ppg_in_shake > 0, f"{ppg_in_shake} batches in "
                   f"{shake_wall:.0f} s shake"))

    print(f"\ngate acceptance ({len(spans)} span(s), {len(gl.ibis)} IBIs):")
    ok = True
    for name, passed, detail in checks:
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} ({detail})")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Acceptance: artifact gating (guided still/shake/"
                    "still).")
    add_device_args(ap)
    ap.add_argument("--still", type=float, default=30.0,
                    help="still phase length, s (default 30)")
    ap.add_argument("--shake", type=float, default=15.0,
                    help="shake phase length, s (default 15)")
    ap.add_argument("--max-still-duty", type=float, default=0.02,
                    help="max gate duty while still (default 0.02)")
    args = ap.parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
