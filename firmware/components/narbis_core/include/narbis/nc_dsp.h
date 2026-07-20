/*
 * nc_dsp.h — per-channel fixed-point PPG DSP chain (handoff §5.7).
 *
 * Chain (ambient subtract, if enabled, happens in the caller):
 *   x —> DC tracker (1-pole HP) —> band-pass (2 cascaded Q30 biquads,
 *   4th-order Butterworth, default 0.5–8 Hz) —> optional 50/60 Hz
 *   notch (Q=8) —> outputs {dc, hp, bp}.
 *
 * Everything here is pure C11 integer arithmetic (narbis_core rule:
 * no float in per-sample paths; coefficients are designed offline by
 * tools/goldens/gen_dsp_coeffs.py). Bit-exactness against the python
 * replica in tools/goldens/gen_dsp_vectors.py is a hard contract —
 * any change to the arithmetic below requires regenerating vectors.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/nc_types.h"

/* Q30 biquad coefficients, a0 normalized to 1 and dropped. */
typedef struct { int32_t b0, b1, b2, a1, a2; } nc_bq_coeff_t;

/* Direct Form 2 Transposed state; int64 so the Q30 partial sums keep
 * full precision between samples (inputs are 24-bit: |b·x| < 2^55,
 * |a·y| < 2^62 only if y saturates — normal operation stays ≪ 2^62). */
typedef struct { int64_t s1, s2; } nc_bq_state_t;

/* One DF2T step. Rounding contract: y = sat32(y64 >> 30) — arithmetic
 * shift, NO rounding constant (truncation toward −∞). This matches the
 * python golden-vector replica exactly; do not "improve" it. */
int32_t nc_bq_run(const nc_bq_coeff_t *c, nc_bq_state_t *s, int32_t x);

typedef struct {
    /* DC tracker: dc_acc holds dc with 16 fractional bits (dc<<16). */
    int64_t dc_acc;
    int32_t dc;
    int32_t alpha_q31;
    /* [0..1] band-pass sections, [2] notch. Coefficients are copied at
     * init (not pointed to) so a later slow-path regeneration can swap
     * a channel's set without racing other channels. */
    nc_bq_coeff_t bq_c[3];
    nc_bq_state_t bq_s[3];
    bool notch_en;
} nc_chan_dsp_t;

typedef struct { int32_t dc, hp, bp; } nc_dsp_out_t;

/* Load per-rate defaults from the generated coefficient tables plus
 * the notch knobs (notch_en / notch_hz via nc_knob_get; the notch is
 * force-disabled where f_notch > 0.45*fs). The DC alpha comes from the
 * precomputed per-rate table for the DEFAULT hp corner (0.5 Hz);
 * runtime hp_fc_x100 / bp corner knob changes take effect through
 * coefficients regenerated on the target slow path (future work, not
 * here) — this init does no float math at all. States are zeroed. */
void nc_dsp_init(nc_chan_dsp_t *d, nc_rate_t rate);

/* One sample through the whole chain. x: 24-bit sign-extended
 * (|x| < 2^23; ambient already subtracted by the caller if enabled). */
void nc_dsp_run(nc_chan_dsp_t *d, int32_t x, nc_dsp_out_t *out);
