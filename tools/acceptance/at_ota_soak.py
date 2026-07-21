#!/usr/bin/env python3
"""AT: OTA soak — §5.12.1 DoD.

Pushes the same image --loops times (default 20) via narbis_client.ota
and requires 100 % success. Same-build pushes leave the DIS firmware
string unchanged, so expect_same_fw is set for every round; the per-
round verdict is transfer complete + FINISH accepted + device back on
air with a readable DIS.

Rollback proof (flash a deliberately-broken image and watch the device
boot the old one) is a manual step — this script covers the 20x happy
path the DoD asks for.
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client.client import (NarbisError, add_device_args,  # noqa: E402
                                  client_from_args, connect_or_exit)
from narbis_client.ota import OtaError, push  # noqa: E402


async def run(args) -> int:
    client = client_from_args(args)
    await connect_or_exit(client)

    results = []
    try:
        for i in range(args.loops):
            t0 = time.monotonic()
            try:
                r = await push(client, args.image, version=args.version,
                               status_every=args.status_every,
                               chunk_delay_ms=args.chunk_delay_ms,
                               reboot_wait_s=args.reboot_wait,
                               expect_same_fw=True)
                results.append((True, r.seconds, r.reseeks, None))
                print(f"[{i + 1:2d}/{args.loops}] ok    {r.seconds:6.1f} s  "
                      f"{r.reseeks} reseek(s)  fw {r.fw_after!r}")
            except (OtaError, NarbisError) as e:
                results.append((False, time.monotonic() - t0, 0, str(e)))
                print(f"[{i + 1:2d}/{args.loops}] FAIL  {e}")
                if args.stop_on_fail:
                    break
                if not client.is_connected:
                    try:
                        await client.reconnect()
                    except NarbisError as e2:
                        print(f"reconnect failed ({e2}) — aborting soak")
                        break
    finally:
        await client.disconnect()

    n_ok = sum(1 for ok, *_ in results if ok)
    times = [t for ok, t, *_ in results if ok]
    print(f"\nOTA soak: {n_ok}/{args.loops} succeeded"
          + (f", avg {sum(times) / len(times):.1f} s" if times else ""))
    ok = n_ok == args.loops
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Acceptance: 20x BLE OTA of the same image, 100%% "
                    "success required.")
    add_device_args(ap)
    ap.add_argument("image", help="app image (.bin)")
    ap.add_argument("--loops", type=int, default=20)
    ap.add_argument("--version", default=None)
    ap.add_argument("--status-every", type=int, default=128)
    ap.add_argument("--chunk-delay-ms", type=float, default=0.0)
    ap.add_argument("--reboot-wait", type=float, default=8.0)
    ap.add_argument("--stop-on-fail", action="store_true")
    args = ap.parse_args(argv)

    if not Path(args.image).is_file():
        print(f"error: {args.image}: no such file", file=sys.stderr)
        return 1
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
