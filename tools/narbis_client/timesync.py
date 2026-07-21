"""timesync.py — thin wrapper around NarbisClient.time_sync().

Runs n TIME_SYNC exchanges (min-RTT pair selection lives in client.py),
prints offset/RTT, and optionally re-measures after a dwell to estimate
clock drift in ppm — a host-side cross-check of the drift figure the
firmware reports in STATUS.
"""

from __future__ import annotations

import asyncio
import time
from typing import Optional, Tuple

from .client import NarbisClient


async def measure(client: NarbisClient, n: int = 5) -> Tuple[int, int]:
    """(offset_us, rtt_us) — see NarbisClient.time_sync for semantics."""
    return await client.time_sync(n=n)


async def measure_drift(client: NarbisClient, n: int = 5,
                        dwell_s: float = 30.0) -> Tuple[int, int, float]:
    """Two measurements dwell_s apart -> (offset_us, rtt_us, drift_ppm).
    drift_ppm > 0 means the device clock runs fast vs the host."""
    off1, rtt1 = await client.time_sync(n=n)
    t1 = time.monotonic()
    await asyncio.sleep(dwell_s)
    off2, rtt2 = await client.time_sync(n=n)
    elapsed = time.monotonic() - t1
    drift_ppm = (off2 - off1) / elapsed if elapsed > 0 else 0.0
    return off2, min(rtt1, rtt2), drift_ppm


def main(argv=None) -> int:
    import argparse
    from .client import add_device_args, client_from_args, connect_or_exit

    ap = argparse.ArgumentParser(
        prog="narbis-timesync",
        description="Measure host<->device clock offset (min-RTT of n "
                    "TIME_SYNC exchanges).")
    add_device_args(ap)
    ap.add_argument("-n", type=int, default=5, help="exchanges per "
                    "measurement (default 5)")
    ap.add_argument("--drift-dwell", type=float, default=None, metavar="S",
                    help="also estimate drift over S seconds")
    args = ap.parse_args(argv)

    async def run() -> int:
        client = client_from_args(args)
        await connect_or_exit(client)
        try:
            offset_us, rtt_us = await measure(client, n=args.n)
            print(f"offset: {offset_us} us (dev = host + offset), "
                  f"best RTT: {rtt_us} us")
            if args.drift_dwell:
                print(f"dwelling {args.drift_dwell:.0f} s for drift...")
                _, _, ppm = await measure_drift(client, n=args.n,
                                                dwell_s=args.drift_dwell)
                print(f"drift: {ppm:+.1f} ppm (device vs host)")
            return 0
        finally:
            await client.disconnect()

    try:
        return asyncio.run(run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
