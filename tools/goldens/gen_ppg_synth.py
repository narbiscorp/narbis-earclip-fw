#!/usr/bin/env python3
"""
gen_ppg_synth.py — synthetic band-passed PPG vectors + ground-truth beat
times for the host IBI detector suite (test_host/tests/t_ibi.c).

Signal model: each beat contributes two Gaussians — a systolic pulse and
a dicrotic bump at 35% amplitude. Width and dicrotic delay scale mildly
with the local RR so the 180 bpm end of the ramp stays morphologically
sane. The result is "band-passed-like": a zero-baseline pulse train; the
detector's rectified-derivative front end ignores residual DC anyway.

Cases (spec §5.8 test plan), each at 100 and 500 sps:
  a  steady 60 bpm, 60 s.  RR = 998.3 ms — deliberately NOT an integer
     multiple of either sample period, so the beat phase sweeps the
     sample grid and the sub-sample jitter check actually exercises the
     parabolic interpolation instead of hitting the same phase forever.
  b  steady 100 bpm, 60 s (RR = 598.9 ms, same reasoning).
  c  ramp 60 -> 180 bpm over 120 s (instantaneous-HR integration).
  d  HRV: RR = 998.3 ms + N(0, 28 ms) i.i.d. -> RMSSD = sqrt(2)*28 ~ 40 ms,
     120 s, fixed seed.
  e  case b + Gaussian noise at 12 dB SNR, band-limited to the detector
     pass band (0.5-8 Hz brick wall). Rationale: nc_ibi consumes the
     nc_dsp band-pass OUTPUT, so noise present at that node is in-band by
     construction; raw broadband white noise would be dominated (at
     500 sps especially) by out-of-band HF power the real chain removes,
     making the two rates incomparable. "White" = flat within the band.
  f  case a with a 10 s dropout zeroed starting at an inter-beat midpoint
     (clean template truncation at both edges). Truth excludes beats in
     or bordering the dropout.

Outputs (committed, compact):
  test_host/vectors/ibi_<case>_<rate>.bin   int32 LE bp samples
  test_host/vectors/ibi_<case>_<rate>.json  {"case","fs","ts_us","gap_us","beats_us"}

Ground-truth beat time = systolic Gaussian center, in microseconds.
Truth excludes beats closer than 0.3 s to the vector end (template cut
off -> the detector cannot legitimately commit them).
"""
import json
import pathlib

import numpy as np

OUT = pathlib.Path(__file__).resolve().parents[2] / "test_host" / "vectors"
AMP = 100_000.0        # counts; typical band-passed 24-bit-AFE pulse amplitude
RATES = (100, 500)
T_FIRST = 0.5037       # first beat: off-grid phase (see module docstring)
END_GUARD_S = 0.3


def beats_const(rr, dur):
    return np.arange(T_FIRST, dur, rr)


def beats_ramp(dur):
    ts, t = [], T_FIRST
    while t < dur:
        hr = 60.0 + (180.0 - 60.0) * t / dur
        ts.append(t)
        t += 60.0 / hr
    return np.array(ts)


def beats_hrv(dur, seed=7):
    rng = np.random.default_rng(seed)
    ts, t = [], T_FIRST
    while t < dur:
        ts.append(t)
        t += float(np.clip(0.9983 + rng.normal(0.0, 0.028), 0.7, 1.4))
    return np.array(ts)


def synth(beats, dur, fs):
    n = int(round(dur * fs))
    x = np.zeros(n)
    for k, tb in enumerate(beats):
        if k + 1 < len(beats):
            rr = beats[k + 1] - tb          # shape the CURRENT cycle
        elif k > 0:
            rr = tb - beats[k - 1]
        else:
            rr = 1.0
        ss = 0.045 * np.sqrt(rr)            # systolic width
        td = 0.05 + 0.20 * rr               # dicrotic delay (shrinks w/ HR)
        sd = 0.060 * np.sqrt(rr)
        lo = max(0, int((tb - 5 * ss) * fs))
        hi = min(n, int((tb + td + 5 * sd) * fs) + 1)
        t = np.arange(lo, hi) / fs
        x[lo:hi] += AMP * (np.exp(-0.5 * ((t - tb) / ss) ** 2)
                           + 0.35 * np.exp(-0.5 * ((t - tb - td) / sd) ** 2))
    return x


def add_band_noise(x, fs, snr_db, seed=11):
    """Add Gaussian noise, brick-walled to 0.5-8 Hz, at snr_db vs x RMS."""
    rng = np.random.default_rng(seed)
    n = rng.standard_normal(len(x))
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    spec = np.fft.rfft(n)
    spec[(f < 0.5) | (f > 8.0)] = 0.0
    nb = np.fft.irfft(spec, len(x))
    target_p = np.mean(x ** 2) / 10.0 ** (snr_db / 10.0)
    nb *= np.sqrt(target_p / np.mean(nb ** 2))
    return x + nb


def emit(case, fs, x, beats, dur, gap=None):
    ts_us = round(1e6 / fs)
    keep = beats < dur - END_GUARD_S
    if gap is not None:
        gs, ge = gap
        keep &= ~((beats > gs - 0.15) & (beats < ge + 0.15))
    truth = beats[keep]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"ibi_{case}_{fs}.bin").write_bytes(
        np.round(x).astype("<i4").tobytes())
    meta = {
        "case": case,
        "fs": fs,
        "ts_us": ts_us,
        "gap_us": [round(gap[0] * 1e6), round(gap[1] * 1e6)] if gap else None,
        "beats_us": [round(tb * 1e6) for tb in truth],
    }
    (OUT / f"ibi_{case}_{fs}.json").write_text(
        json.dumps(meta, separators=(",", ":")))
    print(f"ibi_{case}_{fs}: {len(x)} samples, {len(truth)} truth beats"
          + (f", gap {gap[0]:.3f}-{gap[1]:.3f} s" if gap else ""))


def main():
    for fs in RATES:
        ba = beats_const(0.9983, 60.0)
        emit("a", fs, synth(ba, 60.0, fs), ba, 60.0)

        bb = beats_const(0.5989, 60.0)
        xb = synth(bb, 60.0, fs)
        emit("b", fs, xb, bb, 60.0)

        bc = beats_ramp(120.0)
        emit("c", fs, synth(bc, 120.0, fs), bc, 120.0)

        bd = beats_hrv(120.0)
        emit("d", fs, synth(bd, 120.0, fs), bd, 120.0)

        emit("e", fs, add_band_noise(xb, fs, 12.0), bb, 60.0)

        # f: dropout starts at the midpoint of an inter-beat interval so
        # the edge templates are cleanly truncated, lasts exactly 10 s.
        gs = 0.5 * (ba[25] + ba[26])
        ge = gs + 10.0
        xf = synth(ba, 60.0, fs)
        i0, i1 = int(gs * fs), int(ge * fs)
        xf[i0:i1] = 0.0
        emit("f", fs, xf, ba, 60.0, gap=(gs, ge))


if __name__ == "__main__":
    main()
