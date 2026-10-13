#!/usr/bin/env python
"""
replay_session.py — replay a dashboard guided recording through the firmware's own DSP,
artifact gate and beat detector (replay_ibi.exe) under one or more settings, and score
every run against the Polar H10 recorded beside it.

  python tools/replay/replay_session.py <session.zip | session dir> [--config NAME=ARGS]...

  NAME=ARGS is a label and the replay_ibi arguments, e.g.
    --config live                                   the recording's own beats, no replay
    --config old="--gate replay --knob 0x0708=1"    1.3's single motion threshold
    --config gentle="--gate replay"                 the shipped defaults (two-level motion)
  With no --config: live, then the shipped defaults.

Per state block (rest, foot_tap, chew, talk, ...):
  H10      beats the strap saw
  found    H10 beats an earclip beat matched (within 60 ms, after the time shift)
  false    earclip beats that matched no H10 beat (double counts, noise)
  n/med/p90/<=10/<=20   interval error (ms) where two consecutive earclip beats match two
           consecutive H10 beats, unflagged beats only
  flag     ARTIFACT/GATED-flagged earclip intervals that were right (within 30 ms) / wrong

The H10 sends its RR intervals in ~1 s batches, so its arrival times are not beat times: its
beat times are rebuilt from the RRs (exact relative to each other, clock drift fitted against
the arrivals) and lined up with the earclip's per block by a time-shift search.
"""
import argparse
import bisect
import csv
import pathlib
import shlex
import statistics as st
import subprocess
import sys
import tempfile
import zipfile

REPO = pathlib.Path(__file__).resolve().parents[2]
EXE = REPO / "test_host" / "out" / "replay_ibi.exe"
FLAG_NAMES = [(0x01, "GATED"), (0x02, "AGC_SETTLING"), (0x04, "INTERPOLATED"),
              (0x08, "FIRST_AFTER_GAP"), (0x10, "ARTIFACT"), (0x20, "MOTION")]


def session_dir(path: pathlib.Path, tmp: pathlib.Path) -> pathlib.Path:
    if path.is_file():
        with zipfile.ZipFile(path) as z:
            z.extractall(tmp)
        path = tmp
    subs = [p for p in path.iterdir() if p.is_dir() and (p / "v2").is_dir()]
    return subs[0] if subs else path


def h10_beats(d: pathlib.Path):
    """(host_ms, rr_ms) per H10 beat, from cumulative RRs with the drift fitted to arrivals."""
    cum, beats, arr = 0.0, [], []
    for r in csv.DictReader(open(d / "reference_h10" / "h10_beats.csv")):
        for x in [int(v) for v in r["rr_intervals_ms"].split("|") if v]:
            cum += x
            beats.append((cum, x))
        arr.append((cum, float(r["timestamp_ms"]) - cum))
    n = len(arr)
    mx = sum(c for c, _ in arr) / n
    my = sum(v for _, v in arr) / n
    b = sum((c - mx) * (v - my) for c, v in arr) / sum((c - mx) ** 2 for c, _ in arr)
    a = my - b * mx
    return [(c + a + b * c, x) for c, x in beats]


def live_beats(d: pathlib.Path):
    return [(float(r["host_ms"]), int(r["ibi_ms"]), r["flag_names"])
            for r in csv.DictReader(open(d / "v2" / "ibi.csv"))]


def replay_beats(d: pathlib.Path, args: str, out: pathlib.Path):
    cmd = [str(EXE), str(d / "v2" / "ppg_raw.csv"), str(out), "--accel", str(d / "v2" / "accel_raw.csv")]
    cmd += shlex.split(args)
    subprocess.run(cmd, check=True, stderr=subprocess.DEVNULL)
    live = [(float(r["dev_t_beat_us"]), float(r["host_ms"])) for r in csv.DictReader(open(d / "v2" / "ibi.csv"))]
    n = len(live)
    mx = sum(x for x, _ in live) / n
    my = sum(y for _, y in live) / n
    k = sum((x - mx) * (y - my) for x, y in live) / sum((x - mx) ** 2 for x, _ in live)
    c0 = my - k * mx
    return [(c0 + k * float(r["dev_t_beat_us"]), int(r["ibi_ms"]),
             "|".join(nm for bit, nm in FLAG_NAMES if int(r["flags"]) & bit))
            for r in csv.DictReader(open(out))]


def score(d: pathlib.Path, H, beats):
    ht = [t for t, _ in H]
    et = [t for t, _, _ in beats]

    def count(o, ts, tol=40):
        k = 0
        for t in ts:
            i = bisect.bisect_left(ht, t - o - tol)
            if i < len(ht) and ht[i] <= t - o + tol:
                k += 1
        return k

    g = max(range(-3000, 3001, 4), key=lambda o: count(o, et))
    rows = []
    for blk in [b for b in csv.DictReader(open(d / "protocol" / "blocks.csv")) if b["kind"] == "state"]:
        A, Z = float(blk["start_host_ms"]), float(blk["end_host_ms"])
        idx = [i for i, t in enumerate(et) if A <= t < Z]
        off = max(range(g - 1500, g + 1501, 2), key=lambda o: count(o, [et[i] for i in idx]))

        def match(t):
            i = bisect.bisect_left(ht, t - off)
            c = [j for j in (i - 1, i) if 0 <= j < len(ht) and abs(ht[j] - (t - off)) <= 60]
            return min(c, key=lambda j: abs(ht[j] - (t - off))) if c else None

        mj = {i: match(et[i]) for i in range(len(et)) if A - 2000 <= et[i] < Z}
        nh = sum(1 for t in ht if A - off <= t < Z - off)
        found = len({mj[i] for i in idx if mj[i] is not None})
        false = sum(1 for i in idx if mj[i] is None)
        errs, right, wrong = [], 0, 0
        for i in idx:
            _, ibi, names = beats[i]
            flagged = "ARTIFACT" in names or "GATED" in names
            pair = (ibi > 0 and (i - 1) in mj and mj[i] is not None and mj[i - 1] is not None
                    and mj[i] - mj[i - 1] == 1)
            if flagged and ibi > 0:
                if pair and abs(ibi - H[mj[i]][1]) <= 30:
                    right += 1
                else:
                    wrong += 1
            if pair and not flagged:
                errs.append(abs(ibi - H[mj[i]][1]))
        q = lambda p: sorted(errs)[int(p * (len(errs) - 1))] if errs else 0
        pct = lambda lim: 100 * sum(1 for x in errs if x <= lim) / len(errs) if errs else 0
        rows.append((blk["label"], nh, found, false, len(errs), st.median(errs) if errs else 0,
                     q(0.9), pct(10), pct(20), right, wrong))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=pathlib.Path)
    ap.add_argument("--config", action="append", default=[], help='NAME=ARGS, or "live"')
    a = ap.parse_args()
    configs = a.config or ["live", "defaults=--gate replay"]
    if any(c != "live" for c in configs) and not EXE.exists():
        print(f"{EXE} not built — see tools/replay/README.md", file=sys.stderr)
        return 1
    with tempfile.TemporaryDirectory() as tmp:
        d = session_dir(a.session, pathlib.Path(tmp))
        fw = open(d / "v2" / "link_stats.json").read().split('"firmware"')[1].split('"')[1]
        H = h10_beats(d)
        print(f"{d.name}  (recorded on {fw}; {len(H)} H10 beats)")
        for c in configs:
            name, _, args = c.partition("=")
            beats = live_beats(d) if name == "live" else replay_beats(d, args, pathlib.Path(tmp) / f"{name}.csv")
            print(f"\n[{name}] {args}".rstrip())
            print(f"{'block':9s} {'H10':>4s} {'found':>9s} {'false':>5s} | {'n':>3s} {'med':>4s} {'p90':>4s} "
                  f"{'<=10':>5s} {'<=20':>5s} | flagged right/wrong")
            for lbl, nh, fd, fa, n, med, p90, w10, w20, r, w in score(d, H, beats):
                print(f"{lbl:9s} {nh:4d} {fd:4d} {100 * fd / max(nh, 1):3.0f}% {fa:5d} | {n:3d} {med:4.0f} {p90:4.0f} "
                      f"{w10:4.0f}% {w20:4.0f}% | {r:3d} / {w}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
