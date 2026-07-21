#!/usr/bin/env python3
"""AT: IBI bench — §5.8 DoD.

Records --minutes of all streams (narbis_client.recorder does the
writing), then runs offline stats on ibi.csv:

  * beat count, mean/median/std IBI, implied HR, confidence stats;
  * duplicate check: two beats closer than --dup-ms (default 250, i.e.
    inside any plausible refractory) count as duplicates -> FAIL;
  * range check: fraction of IBIs inside [300, 2000] ms;
  * with --reference REF.csv (a prior recording's ibi.csv, or any CSV
    with a t_beat_us column, or one beat-time-per-line in seconds or
    us): timebase alignment by best-match offset search, then
    detection rate = matched device beats / reference beats. DoD:
    >= 98 % detection (--min-detect), zero duplicates, and |IBI error|
    <= 1 sample at 100 sps (10 ms) median on matched pairs.

Can also run offline on an existing recording with --analyze DIR
(no device needed).
"""

import argparse
import asyncio
import csv
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # tools/


def load_beats_us(path: Path):
    """Reference loader: ibi.csv-style (t_beat_us column) or one beat
    time per line. Unit auto-detection is per FILE, not per line (a
    recording can straddle any per-line threshold): if the largest value
    is < 1e7 the file is in seconds, else microseconds."""
    txt = path.read_text(encoding="utf-8").strip().splitlines()
    if not txt:
        return []
    if "," in txt[0] and not txt[0].replace(",", "").replace(".", "").isdigit():
        with open(path, newline="", encoding="utf-8") as f:
            rd = csv.DictReader(f)
            col = "t_beat_us" if "t_beat_us" in (rd.fieldnames or []) else None
            if col is None:
                raise SystemExit(f"error: {path}: no t_beat_us column")
            beats = [float(row[col]) for row in rd]
    else:
        beats = [float(line.split(",")[0]) for line in txt]
    if beats and max(beats) < 1e7:  # whole file in seconds
        beats = [b * 1e6 for b in beats]
    return sorted(int(b) for b in beats)


def _greedy_match(dev_us, ref_us, off, tol_us):
    matched = 0
    errs = []
    j = 0
    for d in dev_us:
        target = d - off
        while j < len(ref_us) and ref_us[j] < target - tol_us:
            j += 1
        if j < len(ref_us) and abs(ref_us[j] - target) <= tol_us:
            matched += 1
            errs.append(ref_us[j] - target)
            j += 1
    return matched, errs


def match_beats(dev_us, ref_us, tol_us: int):
    """Align timebases by the candidate offset that maximizes matches
    (grid = pairwise diffs of the first few beats), refine by the median
    residual, then greedy 1:1 match in time order.
    Returns (n_matched, errors_us list, offset_us)."""
    if not dev_us or not ref_us:
        return 0, [], 0
    candidates = {d - r for d in dev_us[:8] for r in ref_us[:8]}
    best = (0, [], 0)
    for off in candidates:
        matched, errs = _greedy_match(dev_us, ref_us, off, tol_us)
        if matched > best[0]:
            best = (matched, errs, off)
    if best[1]:  # second pass: recenter on the median residual
        off = best[2] + int(statistics.median(best[1]))
        matched, errs = _greedy_match(dev_us, ref_us, off, tol_us)
        if matched >= best[0]:
            best = (matched, errs, off)
    return best


def analyze(outdir: Path, args) -> int:
    ibi_csv = outdir / "ibi.csv"
    if not ibi_csv.is_file():
        print(f"error: {ibi_csv} not found", file=sys.stderr)
        return 1
    beats = []
    with open(ibi_csv, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            beats.append((int(row["t_beat_us"]), int(row["ibi_ms"]),
                          int(row["confidence"]), int(row["flags"])))
    beats.sort()
    print(f"\nIBI bench on {outdir} ({len(beats)} beats):")
    if not beats:
        print("  no beats at all")
        print("FAIL")
        return 2

    ibis = [b[1] for b in beats]
    confs = [b[2] for b in beats]
    mean_ibi = statistics.fmean(ibis)
    print(f"  IBI: mean {mean_ibi:.1f} ms, median "
          f"{statistics.median(ibis):.1f} ms, "
          f"std {statistics.pstdev(ibis):.1f} ms "
          f"-> HR {60000 / mean_ibi:.1f} bpm")
    print(f"  confidence: mean {statistics.fmean(confs):.1f}, "
          f"min {min(confs)}")

    checks = []

    dups = sum(1 for a, b in zip(beats, beats[1:])
               if b[0] - a[0] < args.dup_ms * 1000)
    checks.append(("no duplicate beats", dups == 0, f"{dups} duplicates"))

    in_range = sum(1 for v in ibis if 300 <= v <= 2000)
    frac = in_range / len(ibis)
    checks.append((">=90% IBIs in [300,2000] ms", frac >= 0.90,
                   f"{frac * 100:.1f}% in range"))

    if args.reference:
        ref = load_beats_us(Path(args.reference))
        tol_us = args.match_tol_ms * 1000
        n_match, errs, off = match_beats([b[0] for b in beats], ref, tol_us)
        detect = n_match / len(ref) if ref else 0.0
        extra = len(beats) - n_match
        med_err = statistics.median([abs(e) for e in errs]) / 1000 if errs else 0
        print(f"  reference: {len(ref)} beats, matched {n_match} "
              f"(offset {off} us), median |err| {med_err:.1f} ms, "
              f"{extra} unmatched device beats")
        checks.append((f">={args.min_detect * 100:.0f}% detection",
                       detect >= args.min_detect,
                       f"{detect * 100:.2f}%"))
        checks.append(("median |IBI err| <= 10 ms (1 sample @100 sps)",
                       med_err <= 10.0, f"{med_err:.1f} ms"))
        checks.append(("unmatched device beats <= 2% of ref",
                       extra <= 0.02 * len(ref), f"{extra} extra"))

    ok = True
    print()
    for name, passed, detail in checks:
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} ({detail})")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


async def run(args) -> int:
    from narbis_client.client import client_from_args, connect_or_exit
    from narbis_client.recorder import record

    client = client_from_args(args)
    await connect_or_exit(client)
    outdir = Path(args.out or "ibi_bench_rec")
    try:
        print(f"recording {args.minutes:.1f} min to {outdir} — keep the "
              f"clip worn and still")
        await record(client, outdir, duration=args.minutes * 60)
    finally:
        await client.disconnect()
    return analyze(outdir, args)


def main(argv=None) -> int:
    from narbis_client.client import add_device_args

    ap = argparse.ArgumentParser(
        description="Acceptance: IBI quality vs offline stats / reference "
                    "beat file.")
    add_device_args(ap)
    ap.add_argument("--minutes", type=float, default=5.0,
                    help="recording length (default 5 min)")
    ap.add_argument("--out", default=None, help="recording directory")
    ap.add_argument("--analyze", default=None, metavar="DIR",
                    help="skip recording, analyze an existing directory")
    ap.add_argument("--reference", default=None,
                    help="reference beat file (ibi.csv / t_beat_us CSV / "
                         "one beat time per line)")
    ap.add_argument("--match-tol-ms", type=int, default=150,
                    help="beat match tolerance (default 150 ms)")
    ap.add_argument("--min-detect", type=float, default=0.98,
                    help="required detection rate vs reference")
    ap.add_argument("--dup-ms", type=int, default=250,
                    help="beats closer than this are duplicates")
    args = ap.parse_args(argv)

    if args.analyze:
        return analyze(Path(args.analyze), args)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
