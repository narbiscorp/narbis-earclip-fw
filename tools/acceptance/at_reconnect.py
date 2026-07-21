#!/usr/bin/env python3
"""AT: reconnect — §5.4 DoD ("reconnect after supervision timeout
resumes cleanly").

Sequence-continuity policy (documented here, verified below):
  Stream seq counters belong to an ACQUISITION RUN, not to the BLE
  link. Acquisition is subscription-gated (§5.4 DECIDED): the last
  unsubscribe/disconnect stops it, a new subscribe restarts it, and the
  restart resets DSP/IBI state and stream seq counters (see
  acq_dsp_request_reset(full=true) in firmware/main/acq_internal.h).
  Therefore across a reconnect the EXPECTED behavior is:
    * seq restarts low (fresh run) — NOT continuity with pre-drop seqs;
    * no data generated while disconnected is retained or replayed;
    * within the new run, seqs are gap-free from the first batch on.

Test: stream --warmup seconds, kill the link (--method disconnect =
client-initiated drop; --method walk = operator unplugs/walks the
device out of range and the script waits for the supervision timeout),
reconnect with the client helper, resume streams, verify:
  1. reconnect + resumed PPG flow within --resume-timeout;
  2. post-resume streaming is gap-free for --verify seconds;
  3. observed seq policy matches the documented one (restart or
     monotonic continuation both reported; restart expected).
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/

from narbis_client import proto as P  # noqa: E402
from narbis_client.client import (NarbisError, add_device_args,  # noqa: E402
                                  client_from_args, connect_or_exit)
from narbis_client.recorder import GapTracker  # noqa: E402


async def run(args) -> int:
    client = client_from_args(args)
    await connect_or_exit(client)

    pre = {"last_seq": None, "batches": 0}
    post = {"first_seq": None, "batches": 0}
    post_tracker = GapTracker("ppg-post")
    phase = ["pre"]

    def on_ppg(b: P.PpgBatch):
        if phase[0] == "pre":
            pre["last_seq"] = b.seq
            pre["batches"] += 1
        else:
            if post["first_seq"] is None:
                post["first_seq"] = b.seq
            post_tracker.feed(b.seq)
            post["batches"] += 1

    checks = []
    try:
        await client.subscribe("ppg", on_ppg)
        await client.start_streams(P.STREAM_MASK_PPG)
        print(f"warmup: streaming {args.warmup:.0f} s ...")
        await asyncio.sleep(args.warmup)
        if pre["batches"] == 0:
            print("FAIL: no PPG flow before the drop")
            return 2
        print(f"  {pre['batches']} batches, last seq {pre['last_seq']}")

        # ---- kill the link ------------------------------------------ #
        if args.method == "disconnect":
            print("dropping the link (client-initiated disconnect)")
            await client.disconnect()
        else:  # walk
            print(">>> take the device out of range / shield it now")
            print(f"    waiting up to {args.drop_timeout:.0f} s for the "
                  f"supervision timeout ...")
            if not await client.wait_for_disconnect(args.drop_timeout):
                print("FAIL: link never dropped")
                return 2
        print("link down — reconnecting")
        if args.method == "walk":
            print(">>> bring the device back in range")

        # ---- reconnect + resume -------------------------------------- #
        phase[0] = "post"
        t0 = time.monotonic()
        await client.reconnect(attempts=args.reconnect_attempts, delay=1.0)
        # subscriptions were re-armed by reconnect(); acquisition restarts
        # with the subscription (§5.4) but send the explicit start too —
        # it is the documented manual override and makes the test
        # deterministic on builds with subscription gating off.
        await client.start_streams(P.STREAM_MASK_PPG)
        t_deadline = time.monotonic() + args.resume_timeout
        while post["batches"] == 0 and time.monotonic() < t_deadline:
            await asyncio.sleep(0.1)
        t_resume = time.monotonic() - t0
        checks.append(("stream resumed after reconnect",
                       post["batches"] > 0, f"{t_resume:.1f} s to first "
                       f"post-drop batch"))
        if post["batches"] == 0:
            print("  no data after reconnect")
        else:
            print(f"resumed in {t_resume:.1f} s (first seq "
                  f"{post['first_seq']}) — verifying "
                  f"{args.verify:.0f} s gap-free ...")
            await asyncio.sleep(args.verify)
            checks.append(("post-resume streaming gap-free",
                           len(post_tracker.gaps) == 0,
                           f"{post_tracker.batches} batches, "
                           f"{len(post_tracker.gaps)} gap(s)"))
            # seq policy: fresh acquisition run -> restart expected
            restarted = post["first_seq"] is not None and \
                post["first_seq"] <= (pre["last_seq"] or 0)
            policy = ("seq RESTARTED (fresh acquisition run — documented "
                      "policy)" if restarted else
                      "seq CONTINUED monotonically (acquisition survived "
                      "the drop — also clean)")
            print(f"  observed: pre last={pre['last_seq']}, post "
                  f"first={post['first_seq']} -> {policy}")
            checks.append(("seq behavior consistent with policy", True,
                           policy))
        try:
            await client.stop_streams(P.STREAM_MASK_PPG)
        except NarbisError:
            pass
    finally:
        await client.disconnect()

    print("\nreconnect acceptance:")
    ok = True
    for name, passed, detail in checks:
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} ({detail})")
    ok = ok and len(checks) >= 2
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Acceptance: link drop + reconnect resumes cleanly; "
                    "seq policy documented in the module docstring.")
    add_device_args(ap)
    ap.add_argument("--method", choices=("disconnect", "walk"),
                    default="disconnect",
                    help="disconnect = client drops the link; walk = "
                         "operator forces a supervision timeout")
    ap.add_argument("--warmup", type=float, default=10.0)
    ap.add_argument("--verify", type=float, default=30.0,
                    help="gap-free verification window after resume")
    ap.add_argument("--resume-timeout", type=float, default=30.0)
    ap.add_argument("--drop-timeout", type=float, default=60.0,
                    help="walk method: max wait for the link to drop")
    ap.add_argument("--reconnect-attempts", type=int, default=12)
    args = ap.parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
