#!/usr/bin/env python
"""
gen_dsp_coeffs.py -- design + quantize the per-rate fixed-point DSP
coefficient tables for narbis_core, emitting
firmware/components/narbis_core/src/dsp_coeffs_gen.h (committed).

Design (locked, see firmware_handoff.md 5.7 + nc_dsp.h):
  - Band-pass: 4th-order Butterworth 0.5-8 Hz as 2 cascaded biquads
    (scipy sos), per rate {50,100,200,250,500} sps.
  - Notch: iirnotch at 50 and 60 Hz, Q=8, per rate; a notch is only
    valid where f_notch <= 0.45*fs (else the entry is zeroed and its
    valid flag cleared -- nc_dsp_init auto-disables from the flag).
  - DC tracker alpha for the DEFAULT hp corner fc = 0.5 Hz:
      alpha_q31 = round(2^31 * (1 - exp(-2*pi*fc/fs)))
    (exact discretization of the 1-pole; consumed as Q31 by the C
    tracker via >>15 then >>16).
  - All biquad coefficients quantized to Q30 int32 (a0 normalized
    to 1 and dropped). |coef| < 2 is asserted; the low-corner a1 of
    the 0.5 Hz sections approaches -2 at 500 sps, so the int32 fit
    check is real, not decorative.
  - Post-quantization pole radii are recomputed from the *quantized*
    a1/a2 and asserted < 0.9995 (stability margin after rounding).

The emitted header is the single fixed-point truth: gen_dsp_vectors.py
PARSES it (rather than re-running scipy) so committed vectors can never
drift from committed coefficients.
"""

import math
import re
from pathlib import Path

import numpy as np
from scipy import signal

RATES = [50, 100, 200, 250, 500]          # index == nc_rate_t
Q30 = 1 << 30
Q31 = 1 << 31
BP_LO_HZ = 0.5
BP_HI_HZ = 8.0
HP_FC_HZ = 0.5                             # default hp_fc_x100 = 50
NOTCH_Q = 8.0
NOTCH_FREQS = [50, 60]
POLE_RADIUS_MAX = 0.9995

REPO = Path(__file__).resolve().parents[2]
OUT_H = REPO / "firmware/components/narbis_core/src/dsp_coeffs_gen.h"


def q30(x, what):
    v = int(round(x * Q30))
    # int32 fit: |coef| < 2.0 in Q30. a1 ~ -1.99x at 500 sps is legal;
    # exactly -2.0 (v == -2^31) would still fit but flags a design at
    # the stability edge, so reject it too.
    assert -(1 << 31) < v < (1 << 31), f"Q30 overflow in {what}: {x}"
    return v


def quant_biquad(b, a, what):
    """(b[3], a[3]) float, a[0]==1 -> (b0,b1,b2,a1,a2) Q30 ints."""
    assert abs(a[0] - 1.0) < 1e-12, f"{what}: a0 not normalized"
    return tuple(q30(c, what) for c in (b[0], b[1], b[2], a[1], a[2]))


def pole_radius_quantized(a1q, a2q):
    """Pole radius of z^2 + (a1q/2^30) z + (a2q/2^30) -- i.e. of the
    coefficients the target actually runs, not the float design."""
    r = np.roots([1.0, a1q / Q30, a2q / Q30])
    return float(max(abs(r)))


def design():
    bp = {}      # fs -> [sec0, sec1] of Q30 tuples
    notch = {}   # (fs, f0) -> Q30 tuple or None
    alpha = {}   # fs -> alpha_q31

    for fs in RATES:
        # Design in zpk and distribute the overall gain k evenly across
        # the sections: scipy's sos output puts all of k into section 0,
        # leaving the other section with b = [1, +/-2, 1] -- |b1| = 2.0
        # exactly, which does not fit Q30. sqrt(k) per section keeps
        # every b coefficient well inside (-2, 2) and improves the
        # intermediate-node headroom of the cascade.
        z, p, k = signal.butter(2, [BP_LO_HZ, BP_HI_HZ], btype="bandpass",
                                fs=fs, output="zpk")
        sos = signal.zpk2sos(z, p, 1.0, pairing="nearest")
        assert sos.shape == (2, 6), f"expected 2 sos sections, got {sos.shape}"
        sos[:, :3] *= k ** (1.0 / sos.shape[0])
        secs = []
        for i, row in enumerate(sos):
            sec = quant_biquad(row[:3], row[3:], f"bp fs={fs} sec{i}")
            rr = pole_radius_quantized(sec[3], sec[4])
            print(f"bp    fs={fs:3d} sec{i}: pole radius (quantized) = {rr:.6f}")
            assert rr < POLE_RADIUS_MAX, f"bp fs={fs} sec{i} radius {rr}"
            secs.append(sec)
        bp[fs] = secs

        for f0 in NOTCH_FREQS:
            # Validity rule shared with nc_dsp_init and the vector
            # generator: notch usable only if f0 <= 0.45*fs. (At 100
            # sps a 50 Hz notch sits exactly on Nyquist -- degenerate.)
            if f0 * 100 > 45 * fs:
                notch[(fs, f0)] = None
                continue
            b, a = signal.iirnotch(f0, NOTCH_Q, fs=fs)
            sec = quant_biquad(b, a, f"notch{f0} fs={fs}")
            rr = pole_radius_quantized(sec[3], sec[4])
            print(f"notch fs={fs:3d} f0={f0}: pole radius (quantized) = {rr:.6f}")
            assert rr < POLE_RADIUS_MAX, f"notch{f0} fs={fs} radius {rr}"
            notch[(fs, f0)] = sec

        a_val = round(Q31 * (1.0 - math.exp(-2.0 * math.pi * HP_FC_HZ / fs)))
        assert 0 < a_val < (1 << 31), f"alpha_q31 out of range fs={fs}"
        alpha[fs] = int(a_val)
        print(f"dc    fs={fs:3d}: alpha_q31 = {alpha[fs]}")

    return bp, notch, alpha


def coeff_row(sec, indent="        "):
    if sec is None:
        sec = (0, 0, 0, 0, 0)
    b0, b1, b2, a1, a2 = sec
    return (f"{indent}{{ .b0 = {b0}, .b1 = {b1}, .b2 = {b2}, "
            f".a1 = {a1}, .a2 = {a2} }},")


def emit(bp, notch, alpha):
    L = []
    L.append("/*")
    L.append(" * dsp_coeffs_gen.h -- GENERATED by tools/goldens/gen_dsp_coeffs.py.")
    L.append(" * DO NOT EDIT; rerun the generator (then gen_dsp_vectors.py) instead.")
    L.append(" *")
    L.append(" * Include only from nc_dsp.c, after narbis/nc_dsp.h (needs")
    L.append(" * nc_bq_coeff_t and NC_RATE_COUNT).")
    L.append(" *")
    L.append(" * Derivation:")
    L.append(f" *  - band-pass: scipy.signal.butter(2, [{BP_LO_HZ}, {BP_HI_HZ}], 'bandpass',")
    L.append(" *    output='zpk') per rate -> zpk2sos with the overall gain k split")
    L.append(" *    sqrt(k) per section (all-in-one-section would put |b1| = 2.0")
    L.append(" *    outside Q30) -> 2 cascaded biquads, Q30, a0 dropped;")
    L.append(f" *  - notch: scipy.signal.iirnotch(f0, Q={NOTCH_Q:g}) at 50/60 Hz where")
    L.append(" *    f0 <= 0.45*fs (zeroed + valid=0 otherwise);")
    L.append(f" *  - DC alpha (default hp fc = {HP_FC_HZ} Hz):")
    L.append(" *    alpha_q31 = round(2^31 * (1 - exp(-2*pi*fc/fs)));")
    L.append(f" *  - post-quantization pole radii asserted < {POLE_RADIUS_MAX} by the")
    L.append(" *    generator. Table index == nc_rate_t.")
    L.append(" */")
    L.append("#pragma once")
    L.append("")
    L.append("/* 4th-order Butterworth band-pass "
             f"{BP_LO_HZ}-{BP_HI_HZ} Hz: [rate][section] */")
    L.append("static const nc_bq_coeff_t nc_dsp_bp_sos[NC_RATE_COUNT][2] = {")
    for fs in RATES:
        L.append(f"    {{ /* {fs} sps */")
        for sec in bp[fs]:
            L.append(coeff_row(sec))
        L.append("    },")
    L.append("};")
    L.append("")
    for f0 in NOTCH_FREQS:
        L.append(f"/* {f0} Hz notch, Q={NOTCH_Q:g}; all-zero rows are invalid rates */")
        L.append(f"static const nc_bq_coeff_t nc_dsp_notch{f0}[NC_RATE_COUNT] = {{")
        for fs in RATES:
            L.append(f"    {coeff_row(notch[(fs, f0)], indent='')} /* {fs} sps */")
        L.append("};")
        L.append("")
        L.append(f"/* 1 where {f0} <= 0.45*fs (nc_dsp_init auto-disable rule) */")
        vals = ", ".join("1" if notch[(fs, f0)] is not None else "0"
                         for fs in RATES)
        L.append(f"static const uint8_t nc_dsp_notch{f0}_valid[NC_RATE_COUNT] = "
                 f"{{ {vals} }};")
        L.append("")
    L.append(f"/* DC tracker alpha, Q31, default fc = {HP_FC_HZ} Hz */")
    vals = ", ".join(str(alpha[fs]) for fs in RATES)
    L.append(f"static const int32_t nc_dsp_dc_alpha_q31[NC_RATE_COUNT] = "
             f"{{ {vals} }};")
    L.append("")
    return "\n".join(L)


def main():
    bp, notch, alpha = design()
    text = emit(bp, notch, alpha)
    OUT_H.write_text(text, newline="\n")
    # Self-check: the parse format contract with gen_dsp_vectors.py.
    n = len(re.findall(r"\{ \.b0 = ", text))
    assert n == len(RATES) * 2 + len(RATES) * len(NOTCH_FREQS), n
    print(f"wrote {OUT_H} ({len(text)} bytes, {n} biquad rows)")


if __name__ == "__main__":
    main()
