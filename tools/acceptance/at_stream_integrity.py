#!/usr/bin/env python3
"""AT: stream integrity — §5.4 DoD.

Gap-free streaming proven by sequence numbers: subscribe PPG + ACCEL +
IBI + EVENT, stream for --duration (default 1800 s = 30 min), count
every u32 seq discontinuity. PASS iff zero gaps on every stream AND
data actually flowed (a silent link must not pass).

Exit code 0 = PASS, 2 = FAIL, 1 = no device / setup error.
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
from narbis_client.recorder import GapTracker  # noqa: E402


async def run(args) -> int:
    client = client_from_args(args)
    await connect_or_exit(client)

    trackers = {s: GapTracker(s) for s in ("ppg", "accel", "ibi", "event")}
    counts = {"ppg": 0, "accel": 0, "ibi": 0, "event": 0}

    def feeder(name):
        def cb(batch):
            gap = trackers[name].feed(batch.seq)
            if gap:
                print(f"  !! gap on {name}: expected {gap['expected']} got "
                      f"{gap['got']} ({gap['lost_batches']} lost)")
            counts[name] += getattr(batch, "n", 1)
        return cb

    try:
        if args.rate is not None:
            await client.set_rate(args.rate)
        for name in trackers:
            await client.subscribe(name, feeder(name))
        await client.start_streams()

        t_end = time.monotonic() + args.duration
        next_tick = time.monotonic() + 60
        while time.monotonic() < t_end:
            await asyncio.sleep(0.5)
            if client.disconnected.is_set():
                print("FAIL: device disconnected mid-test")
                return 2
            if time.monotonic() >= next_tick:
                left = t_end - time.monotonic()
                print(f"  ... {left / 60:5.1f} min left | samples: "
                      + " ".join(f"{k}={v}" for k, v in counts.items()))
                next_tick += 60

        await client.stop_streams()
    finally:
        await client.disconnect()

    total_gaps = sum(len(t.gaps) for t in trackers.values())
    print("\nstream integrity over "
          f"{args.duration:.0f} s (rate arg: {args.rate}):")
    for name, t in trackers.items():
        print(f"  {name:<6} {t.batches:>8} batches, {counts[name]:>9} "
              f"samples, {len(t.gaps)} gap(s)")
    ok = (total_gaps == 0 and trackers["ppg"].batches > 0
          and trackers["accel"].batches > 0)
    if trackers["ppg"].batches == 0:
        print("  no PPG batches at all — is the clip worn / acquisition "
              "gated off?")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Acceptance: seq-gap-free streaming (default 30 min).")
    add_device_args(ap)
    ap.add_argument("--duration", type=float, default=1800.0,
                    help="test length in seconds (default 1800)")
    ap.add_argument("--rate", type=int, default=None,
                    help="PPG rate to test (code 0..4 or sps; default: "
                         "leave device setting)")
    args = ap.parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
