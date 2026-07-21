#!/usr/bin/env python3
"""AT: PPG rate accuracy — §5.5 DoD.

For each rate in {50,100,200,250,500} sps: set the rate, stream PPG for
--window seconds (default 60) after a settle period, and measure the
actual sample rate two ways:

  * primary: samples per device-timestamp span (batch t0_us delta over
    the counted batches) — this compares the AFE's internal 4 MHz
    oscillator against the C6 crystal, which is exactly what the DoD's
    "count ADC_RDY over 60 s" measures on-device;
  * secondary (informational): samples per host wall clock, which adds
    host<->device crystal difference (tens of ppm — noise here) plus
    BLE batching jitter at the window edges.

PASS iff |measured/nominal - 1| <= --tol (default 0.5 %) at every rate
and no seq gaps corrupted the count.
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


async def measure_rate(client, rate_code: int, window_s: float,
                       settle_s: float):
    """-> (ok, measured_sps_dev, measured_sps_host, n_samples, gaps)"""
    batches = []
    tracker = GapTracker(f"ppg@r{rate_code}")
    t_host_first = [None]

    def on_ppg(b: P.PpgBatch):
        tracker.feed(b.seq)
        if t_host_first[0] is None:
            t_host_first[0] = time.monotonic()
        batches.append(b)

    await client.set_rate(rate_code)
    await client.subscribe("ppg", on_ppg)
    await client.start_streams(P.STREAM_MASK_PPG)
    try:
        await asyncio.sleep(settle_s)
        batches.clear()
        tracker.gaps.clear()
        tracker.last_seq = None  # gap check applies to the window only
        t0_host = time.monotonic()
        await asyncio.sleep(window_s)
        t1_host = time.monotonic()
    finally:
        try:
            await client.stop_streams(P.STREAM_MASK_PPG)
        except Exception:
            pass
        await client.unsubscribe("ppg")

    if len(batches) < 3:
        return False, 0.0, 0.0, 0, len(tracker.gaps)

    # device-clock measurement: samples in [first batch, last batch) over
    # the t0_us span — exact when the window is gap-free
    span_us = batches[-1].t0_us - batches[0].t0_us
    n_between = sum(b.n for b in batches[:-1])
    sps_dev = n_between * 1e6 / span_us if span_us > 0 else 0.0

    # host-clock measurement (informational)
    n_total = sum(b.n for b in batches)
    sps_host = n_total / (t1_host - t0_host)

    return True, sps_dev, sps_host, n_total, len(tracker.gaps)


async def run(args) -> int:
    client = client_from_args(args)
    await connect_or_exit(client)

    codes = args.rates if args.rates else sorted(P.RATE_SPS)
    results = []
    try:
        for code in codes:
            nominal = P.rate_sps(code)
            if nominal == 0:
                print(f"skipping invalid rate code {code}")
                continue
            print(f"rate {nominal} sps: settling {args.settle:.0f} s + "
                  f"measuring {args.window:.0f} s ...")
            ok, sps_dev, sps_host, n, gaps = await measure_rate(
                client, code, args.window, args.settle)
            results.append((code, nominal, ok, sps_dev, sps_host, n, gaps))
    finally:
        await client.disconnect()

    print(f"\nrate accuracy (window {args.window:.0f} s, "
          f"tolerance +-{args.tol * 100:.2f} %):")
    print(f"  {'nominal':>8} {'dev-clock':>10} {'err%':>8} "
          f"{'host-clock':>10} {'samples':>8} {'gaps':>5}  verdict")
    all_ok = True
    for code, nominal, got_data, sps_dev, sps_host, n, gaps in results:
        if not got_data:
            print(f"  {nominal:>8} {'—':>10} {'—':>8} {'—':>10} {n:>8} "
                  f"{gaps:>5}  FAIL (no data)")
            all_ok = False
            continue
        err = sps_dev / nominal - 1.0
        ok = abs(err) <= args.tol and gaps == 0
        all_ok &= ok
        print(f"  {nominal:>8} {sps_dev:>10.3f} {err * 100:>+8.3f} "
              f"{sps_host:>10.3f} {n:>8} {gaps:>5}  "
              f"{'ok' if ok else 'FAIL'}")
    print("PASS" if all_ok and results else "FAIL")
    return 0 if all_ok and results else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Acceptance: measured PPG rate within +-0.5% at all "
                    "rates.")
    add_device_args(ap)
    ap.add_argument("--window", type=float, default=60.0,
                    help="measurement window per rate, s (default 60)")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="settle time after each rate switch, s")
    ap.add_argument("--tol", type=float, default=0.005,
                    help="relative tolerance (default 0.005 = 0.5%%)")
    ap.add_argument("--rates", type=int, nargs="*", default=None,
                    help="rate codes to test (default: all five)")
    args = ap.parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
