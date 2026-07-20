#!/usr/bin/env python
"""
gen_dsp_vectors.py -- golden vectors for test_host/tests/t_dsp.c.

BIT-EXACT integer replica of nc_dsp.c (nc_bq_run DF2T + DC tracker +
full chain). Every operation below mirrors a C statement:
  - python ">>" on a negative int floors, identical to the arithmetic
    right shift gcc/clang emit for signed int64 -- truncation toward
    -inf on both sides, NO rounding constants anywhere;
  - the biquad output is sat32(y64 >> 30) computed once and fed back;
  - the DC tracker is update-then-subtract: acc += ((x-dc)*alpha)>>15;
    dc = i32(acc>>16); hp = x - dc  (alpha is Q31: 15+16 = 31);
  - int64 state headroom is ASSERTED (C signed overflow would be UB,
    so the replica proves the vectors never get near it).

Coefficients are PARSED out of the committed dsp_coeffs_gen.h -- not
re-derived with scipy -- so vectors can never drift from the header.

Vector set (names/rates/notch config MUST match the cfgs[] table in
t_dsp.c):
  impulse  all rates, notch off        1<<20 at n=0, 5 s
  step     all rates, notch off        1<<20 DC, 5 s
  twotone  all rates, notch 50 Hz      2^21*(sin 1.2 Hz + sin 6 Hz), 20 s
  chirp    all rates, notch 60 Hz      2^21 * chirp 0.2->10 Hz, 20 s
  noise    all rates, notch 50 Hz      RandomState(42).randn * 2^19, 10 s
  mains50  rate 100 only, notch 50 Hz  1.5 Hz pulse + 50 Hz mains, 10 s
           (50 Hz == Nyquist at 100 sps: the notch must AUTO-DISABLE,
            pi/4 phase so the alias lands at +/-0.707 not identical 0)
All durations cap at 10 s for 500 sps. Files (little-endian int32):
  test_host/vectors/dsp_<name>_r<rate>_in.bin    n samples
  test_host/vectors/dsp_<name>_r<rate>_exp.bin   n x {dc, hp, bp}
"""

import re
from pathlib import Path

import numpy as np
from scipy import signal as sps

REPO = Path(__file__).resolve().parents[2]
HDR = REPO / "firmware/components/narbis_core/src/dsp_coeffs_gen.h"
OUT_DIR = REPO / "test_host/vectors"

RATES = [50, 100, 200, 250, 500]  # index == nc_rate_t
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1
I64_MIN, I64_MAX = -(1 << 63), (1 << 63) - 1

# ---------------------------------------------------------------- #
# Parse the committed coefficient header                            #
# ---------------------------------------------------------------- #

ROW_RE = re.compile(r"\{ \.b0 = (-?\d+), \.b1 = (-?\d+), \.b2 = (-?\d+), "
                    r"\.a1 = (-?\d+), \.a2 = (-?\d+) \}")


def parse_header():
    text = HDR.read_text()

    def rows_of(decl, count):
        m = re.search(re.escape(decl) + r" = \{(.*?)\n\};", text, re.S)
        assert m, f"missing table {decl}"
        rows = [tuple(int(v) for v in r) for r in ROW_RE.findall(m.group(1))]
        assert len(rows) == count, f"{decl}: {len(rows)} rows"
        return rows

    def ints_of(decl, count):
        m = re.search(re.escape(decl) + r" = \{ ([-\d, ]+) \};", text)
        assert m, f"missing table {decl}"
        vals = [int(v) for v in m.group(1).split(",")]
        assert len(vals) == count
        return vals

    flat = rows_of("static const nc_bq_coeff_t nc_dsp_bp_sos[NC_RATE_COUNT][2]",
                   len(RATES) * 2)
    bp = [flat[2 * i: 2 * i + 2] for i in range(len(RATES))]
    n50 = rows_of("static const nc_bq_coeff_t nc_dsp_notch50[NC_RATE_COUNT]",
                  len(RATES))
    n60 = rows_of("static const nc_bq_coeff_t nc_dsp_notch60[NC_RATE_COUNT]",
                  len(RATES))
    v50 = ints_of("static const uint8_t nc_dsp_notch50_valid[NC_RATE_COUNT]",
                  len(RATES))
    v60 = ints_of("static const uint8_t nc_dsp_notch60_valid[NC_RATE_COUNT]",
                  len(RATES))
    alpha = ints_of("static const int32_t nc_dsp_dc_alpha_q31[NC_RATE_COUNT]",
                    len(RATES))
    return bp, n50, n60, v50, v60, alpha


# ---------------------------------------------------------------- #
# Bit-exact replica of nc_dsp.c                                     #
# ---------------------------------------------------------------- #

def sat32(v):
    return I32_MIN if v < I32_MIN else (I32_MAX if v > I32_MAX else v)


def i32(v):
    """The (int32_t) narrowing cast. The DC tracker's dc always fits,
    but mirror the cast anyway so the replica IS the C code."""
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v >= (1 << 31) else v


class BQ:
    """nc_bq_run: DF2T, Q30 coefficients, int64 s1/s2."""

    def __init__(self, coeff):
        self.b0, self.b1, self.b2, self.a1, self.a2 = coeff
        self.s1 = 0
        self.s2 = 0

    def run(self, x):
        y64 = self.b0 * x + self.s1
        y = sat32(y64 >> 30)
        self.s1 = self.b1 * x - self.a1 * y + self.s2
        self.s2 = self.b2 * x - self.a2 * y
        # C keeps these in int64; overflow there is UB, so prove the
        # vectors keep clear of it instead of emulating wraparound.
        assert I64_MIN <= self.s1 <= I64_MAX and I64_MIN <= self.s2 <= I64_MAX
        return y


class Chain:
    """nc_chan_dsp_t + nc_dsp_run."""

    def __init__(self, alpha_q31, bp_secs, notch_coeff):
        self.alpha = alpha_q31
        self.acc = 0
        self.dc = 0
        self.bq = [BQ(bp_secs[0]), BQ(bp_secs[1])]
        self.notch = BQ(notch_coeff) if notch_coeff is not None else None

    def run(self, x):
        self.acc += ((x - self.dc) * self.alpha) >> 15
        assert I64_MIN <= self.acc <= I64_MAX
        self.dc = i32(self.acc >> 16)
        hp = x - self.dc
        bp = self.bq[0].run(hp)
        bp = self.bq[1].run(bp)
        if self.notch is not None:
            bp = self.notch.run(bp)
        return self.dc, hp, bp


# ---------------------------------------------------------------- #
# Input vectors                                                     #
# ---------------------------------------------------------------- #

def dur_s(nominal, fs):
    return min(nominal, 10) if fs >= 500 else nominal


def make_input(name, fs):
    if name == "impulse":
        n = fs * dur_s(5, fs)
        x = np.zeros(n)
        x[0] = 1 << 20
    elif name == "step":
        n = fs * dur_s(5, fs)
        x = np.full(n, float(1 << 20))
    elif name == "twotone":
        n = fs * dur_s(20, fs)
        t = np.arange(n) / fs
        x = (1 << 21) * (np.sin(2 * np.pi * 1.2 * t)
                         + np.sin(2 * np.pi * 6.0 * t))
    elif name == "chirp":
        n = fs * dur_s(20, fs)
        t = np.arange(n) / fs
        x = (1 << 21) * sps.chirp(t, f0=0.2, t1=n / fs, f1=10.0)
    elif name == "noise":
        n = fs * dur_s(10, fs)
        x = np.random.RandomState(42).randn(n) * (1 << 19)
    elif name == "mains50":
        assert fs == 100
        n = fs * 10
        t = np.arange(n) / fs
        x = ((1 << 20) * np.sin(2 * np.pi * 1.5 * t)
             + (1 << 19) * np.sin(2 * np.pi * 50.0 * t + np.pi / 4))
    else:
        raise ValueError(name)
    xi = np.round(x).astype(np.int64)
    lim = (1 << 23) - 1  # 24-bit sign-extended input contract
    assert np.all(np.abs(xi) <= lim), f"{name} fs={fs} exceeds 24-bit"
    return [int(v) for v in xi]


# (name, rate_mask, notch_en, notch_hz) -- keep in sync with t_dsp.c.
VECTORS = [
    ("impulse", 0x1F, 0, 60),
    ("step",    0x1F, 0, 50),
    ("twotone", 0x1F, 1, 50),
    ("chirp",   0x1F, 1, 60),
    ("noise",   0x1F, 1, 50),
    ("mains50", 0x02, 1, 50),
]


def main():
    bp, n50, n60, v50, v60, alpha = parse_header()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    total = 0

    for name, mask, notch_en, notch_hz in VECTORS:
        for r, fs in enumerate(RATES):
            if not (mask >> r) & 1:
                continue
            # Mirror nc_dsp_init's selection: >= 55 -> 60 Hz design;
            # the _valid flag encodes f_notch <= 0.45*fs auto-disable.
            notch = None
            if notch_en:
                want60 = notch_hz >= 55
                valid = v60[r] if want60 else v50[r]
                if valid:
                    notch = n60[r] if want60 else n50[r]

            x = make_input(name, fs)
            ch = Chain(alpha[r], bp[r], notch)
            out = np.empty(3 * len(x), dtype="<i4")
            for k, xi in enumerate(x):
                out[3 * k], out[3 * k + 1], out[3 * k + 2] = ch.run(xi)

            (OUT_DIR / f"dsp_{name}_r{r}_in.bin").write_bytes(
                np.asarray(x, dtype="<i4").tobytes())
            (OUT_DIR / f"dsp_{name}_r{r}_exp.bin").write_bytes(out.tobytes())
            total += len(x) * 16
            print(f"dsp_{name}_r{r}: n={len(x)} notch={'on' if notch else 'off'}")

    print(f"total vector bytes: {total} ({total / 1e6:.2f} MB)")
    assert total < 2_000_000, "vector budget exceeded"


if __name__ == "__main__":
    main()
