/*
 * nc_dsp.c — fixed-point PPG DSP chain. Bit-exact contract with
 * tools/goldens/gen_dsp_vectors.py (proven by test_host/tests/t_dsp.c).
 */
#include <string.h>
#include "narbis/nc_dsp.h"
#include "narbis/nc_knobs.h"

/* Generated tables: nc_dsp_bp_sos, nc_dsp_notch50/60(+_valid),
 * nc_dsp_dc_alpha_q31. Regenerate with tools/goldens/gen_dsp_coeffs.py. */
#include "dsp_coeffs_gen.h"

int32_t nc_bq_run(const nc_bq_coeff_t *c, nc_bq_state_t *s, int32_t x)
{
    /* DF2T, Q30. y is computed ONCE from the saturated 32-bit value and
     * that same y feeds both state updates — using the unsaturated y64
     * in the feedback would diverge from the golden replica the first
     * time saturation hits. >> on a negative int64 is arithmetic on
     * every compiler we target (gcc/clang, RV32 + x86-64); python's >>
     * floors identically, so truncation is toward −∞ on both sides. */
    int64_t y64 = (int64_t)c->b0 * x + s->s1;
    int32_t y = nc_sat32(y64 >> 30);
    s->s1 = (int64_t)c->b1 * x - (int64_t)c->a1 * y + s->s2;
    s->s2 = (int64_t)c->b2 * x - (int64_t)c->a2 * y;
    return y;
}

void nc_dsp_init(nc_chan_dsp_t *d, nc_rate_t rate)
{
    if ((unsigned)rate >= NC_RATE_COUNT) {
        rate = NC_RATE_100; /* defensive: product default */
    }
    memset(d, 0, sizeof(*d));

    d->alpha_q31 = nc_dsp_dc_alpha_q31[rate];
    d->bq_c[0] = nc_dsp_bp_sos[rate][0];
    d->bq_c[1] = nc_dsp_bp_sos[rate][1];

    /* notch_hz knob range is 50..60; anything >= 55 selects the 60 Hz
     * design, else 50 Hz. The _valid tables already encode the
     * f_notch <= 0.45*fs rule, so an enabled-but-invalid combination
     * (e.g. 50 Hz notch at 100 sps, where 50 Hz IS Nyquist) silently
     * degrades to notch-off rather than running a degenerate biquad. */
    if (nc_knob_get(KNOB_NOTCH_EN) != 0) {
        bool want60 = nc_knob_get(KNOB_NOTCH_HZ) >= 55;
        const uint8_t *valid = want60 ? nc_dsp_notch60_valid
                                      : nc_dsp_notch50_valid;
        if (valid[rate]) {
            d->bq_c[2] = want60 ? nc_dsp_notch60[rate]
                                : nc_dsp_notch50[rate];
            d->notch_en = true;
        }
    }
}

void nc_dsp_run(nc_chan_dsp_t *d, int32_t x, nc_dsp_out_t *out)
{
    /* DC tracker (1-pole HP), all-integer. Shift bookkeeping:
     *   dc_acc holds dc·2^16;  alpha_q31 is α·2^31.
     *   (x−dc)·alpha_q31            has scale 2^31
     *   >> 15                       -> scale 2^16, add into dc_acc
     *   dc = dc_acc >> 16           -> integer dc
     * Net: dc += α·(x−dc) with 16 fractional bits retained across
     * samples (total right-shift 15+16 = 31 = the α scale). Both
     * shifts are arithmetic/floor — same as the python replica.
     * hp uses the UPDATED dc (update-then-subtract), per the locked
     * design. |dc| tracks the 24-bit input, so dc_acc stays < 2^40
     * and the (int32_t) narrowing of dc_acc>>16 cannot overflow. */
    d->dc_acc += ((int64_t)(x - d->dc) * d->alpha_q31) >> 15;
    d->dc = (int32_t)(d->dc_acc >> 16);
    int32_t hp = x - d->dc;

    int32_t bp = nc_bq_run(&d->bq_c[0], &d->bq_s[0], hp);
    bp = nc_bq_run(&d->bq_c[1], &d->bq_s[1], bp);
    if (d->notch_en) {
        bp = nc_bq_run(&d->bq_c[2], &d->bq_s[2], bp);
    }

    out->dc = d->dc;
    out->hp = hp;
    out->bp = bp;
}
