"""selftest.py — run the on-device self-test and print the PASS/FAIL table.

Flow: OP_SELFTEST_RUN (blocking on-device; generous timeout), then
reassemble the result blob from OP_SELFTEST_RESULT chunks and render it.
The blob format is proto.py SelftestBlob (blob_ver 1).
"""

from __future__ import annotations

import asyncio
from typing import Optional

from . import proto as P
from .client import (NarbisClient, SELFTEST_NAMES, TEST_STATUS_NAMES)


def format_blob(blob: P.SelftestBlob) -> str:
    lines = [f"self-test results (blob v{blob.blob_ver}, "
             f"t_run={blob.t_run_us} us):",
             f"  {'test':<16} {'result':<6} {'value':>12} {'threshold':>12}",
             f"  {'-' * 16} {'-' * 6} {'-' * 12} {'-' * 12}"]
    n_pass = n_fail = n_skip = 0
    for r in blob.records:
        name = SELFTEST_NAMES.get(r.id, f"test{r.id}")
        st = TEST_STATUS_NAMES.get(r.status, str(r.status))
        n_pass += r.status == P.TR_PASS
        n_fail += r.status == P.TR_FAIL
        n_skip += r.status == P.TR_SKIP
        lines.append(f"  {name:<16} {st:<6} {r.value:>12} {r.threshold:>12}")
    verdict = "PASS" if n_fail == 0 else "FAIL"
    lines.append(f"  => {verdict}  ({n_pass} pass, {n_fail} fail, "
                 f"{n_skip} skip)")
    return "\n".join(lines)


async def run_selftest(client: NarbisClient, test_mask: int = 0,
                       run_timeout: float = 60.0) -> P.SelftestBlob:
    """Run (0 = all tests) and fetch the result blob."""
    await client.selftest_run(test_mask, timeout=run_timeout)
    return await client.selftest_read_blob()


def main(argv=None) -> int:
    """CLI entry (console_script narbis-selftest,
    tools/scripts/narbis_selftest.py). Exit 0 = all PASS, 2 = any FAIL."""
    import argparse
    from .client import add_device_args, client_from_args, connect_or_exit

    ap = argparse.ArgumentParser(
        prog="narbis-selftest",
        description="Run the on-device self-test and print the PASS/FAIL "
                    "table.")
    add_device_args(ap)
    ap.add_argument("--mask", type=lambda s: int(s, 0), default=0,
                    help="test_mask for SELFTEST_RUN (0 = all tests)")
    ap.add_argument("--fetch-only", action="store_true",
                    help="skip the run, just read the last result blob")
    ap.add_argument("--run-timeout", type=float, default=60.0,
                    help="seconds to allow the on-device run (dark/xtalk "
                         "tests take a while)")
    args = ap.parse_args(argv)

    async def run() -> int:
        client = client_from_args(args)
        await connect_or_exit(client)
        try:
            if args.fetch_only:
                blob = await client.selftest_read_blob()
            else:
                blob = await run_selftest(client, args.mask,
                                          run_timeout=args.run_timeout)
            print(format_blob(blob))
            return 0 if all(r.status != P.TR_FAIL for r in blob.records) else 2
        finally:
            await client.disconnect()

    try:
        return asyncio.run(run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
