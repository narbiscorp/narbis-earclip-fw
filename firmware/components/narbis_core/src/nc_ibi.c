/*
 * nc_ibi.c — slope-sum + adaptive-threshold beat detector (fixed point).
 * See nc_ibi.h for the emission contract; handoff §5.8 for the design.
 */
#include "narbis/nc_ibi.h"
#include "narbis/nc_knobs.h"
#include <string.h>

#define Q31_ONE (1LL << 31)

/* ------------------------------------------------------------------ */
/* Derived parameters                                                  */
/* ------------------------------------------------------------------ */

/* window samples = round(ms * fs / 1000) = round(ms * 1000 / ts_us) */
static uint16_t win_from_ms(uint32_t ms, uint32_t ts_us)
{
    uint32_t w = (ms * 1000u + ts_us / 2u) / ts_us;
    if (w < 1) w = 1;
    if (w > NC_IBI_SSF_MAX) w = NC_IBI_SSF_MAX;
    return (uint16_t)w;
}

static uint32_t smp_from_ms(uint32_t ms, uint32_t ts_us)
{
    uint32_t n = (ms * 1000u + ts_us / 2u) / ts_us;
    return n ? n : 1;
}

/* decay_q31 = round(2^31 * exp(-Ts/tau)). Init/reconfig slow path: double
 * is allowed here (spec) and never touched per-sample. narbis_core's
 * header whitelist excludes <math.h>, so exp() is a 6-term Maclaurin
 * series: for the worst in-spec x = 20 ms / 500 ms = 0.04 the truncation
 * error is < x^6/720 < 6e-12 — far below the Q31 lsb. */
static int32_t decay_q31_slow(uint32_t ts_us, uint32_t tau_ms)
{
    double x = (double)ts_us / ((double)tau_ms * 1000.0);
    double e = (((((-x / 120.0 + 1.0 / 24.0) * x - 1.0 / 6.0) * x + 0.5)
                 * x - 1.0) * x + 1.0);
    double d = e * 2147483648.0 + 0.5;
    if (d >= 2147483647.0) return INT32_MAX;
    if (d < 1.0) return 1;
    return (int32_t)d;
}

/* Integer-only refresh for nc_ibi_set_ts() (timing-EMA path — keep it
 * float-free and cheap). exp(-x) ~= 1 - x + x^2/2: the dropped x^3/6
 * term is < 1.1e-5 at x = 0.04, i.e. an effective tau error < 0.04% —
 * noise against the thr_tau_ms knob granularity. */
static int32_t decay_q31_cheap(uint32_t ts_us, uint32_t tau_ms)
{
    int64_t tau_us = (int64_t)tau_ms * 1000;
    int64_t x = ((int64_t)ts_us * Q31_ONE) / tau_us;
    if (x >= Q31_ONE) return 1;              /* degenerate ts >> tau */
    int64_t d = Q31_ONE - x + ((x * x) >> 32);
    if (d > INT32_MAX) d = INT32_MAX;
    if (d < 1) d = 1;
    return (int32_t)d;
}

static void derive(nc_ibi_t *s, uint32_t ts_us, bool init_path)
{
    if (ts_us == 0) ts_us = 1;
    s->ts_us = ts_us;

    uint16_t w = win_from_ms(s->k_ssf_win_ms, ts_us);
    if (w != s->ssf_win) {
        /* Window length crossed an integer sample boundary (rare drift):
         * restart the ring — costs <= one window (200 ms) of warm-up. */
        memset(s->ring, 0, sizeof s->ring);
        s->ssf_sum = 0;
        s->ring_idx = 0;
        s->ssf_win = w;
    }
    s->refract_smp = smp_from_ms(s->k_refract_ms, ts_us);
    s->decay_q31 = init_path ? decay_q31_slow(ts_us, s->k_thr_tau_ms)
                             : decay_q31_cheap(ts_us, s->k_thr_tau_ms);
}

void nc_ibi_init(nc_ibi_t *s, uint32_t ts_us_measured)
{
    memset(s, 0, sizeof *s);
    s->k_ssf_win_ms    = (uint16_t)nc_knob_get(KNOB_SSF_WIN_MS);
    s->k_thr_frac_x100 = nc_knob_get(KNOB_THR_FRAC_X100);
    s->k_thr_tau_ms    = (uint16_t)nc_knob_get(KNOB_THR_TAU_MS);
    s->k_refract_ms    = (uint16_t)nc_knob_get(KNOB_REFRACT_MS);
    s->k_interp_en     = nc_knob_get(KNOB_INTERP_EN) != 0;
    s->k_ibi_min_ms    = (uint16_t)nc_knob_get(KNOB_IBI_MIN_MS);
    s->k_ibi_max_ms    = (uint16_t)nc_knob_get(KNOB_IBI_MAX_MS);
    s->k_thr_min       = nc_knob_get(KNOB_IBI_THR_MIN);
    derive(s, ts_us_measured, true);
}

void nc_ibi_set_ts(nc_ibi_t *s, uint32_t ts_us_measured)
{
    if (ts_us_measured == s->ts_us) return;
    derive(s, ts_us_measured, false);
}

/* ------------------------------------------------------------------ */
/* Per-sample path (integer only)                                      */
/* ------------------------------------------------------------------ */

/* peak_track * decay >> 31. peak_track (Q8 of a <= 2^38 SSF) can reach
 * 2^46, so a single 64-bit product with a Q31 factor would overflow:
 * split at bit 31 — hi <= 2^15, both partials stay well inside int64. */
static inline int64_t pt_decay(int64_t pt, int32_t d_q31)
{
    return (pt >> 31) * d_q31 + (((pt & (Q31_ONE - 1)) * d_q31) >> 31);
}

/* Commit the captured peak: sub-sample timing, IBI/gap classification,
 * confidence, tracker updates. Returns true if a record was emitted. */
static bool commit(nc_ibi_t *s, int64_t thr, nc_ibi_rec_t *out)
{
    s->in_peak = false;
    s->refract_cnt = s->refract_smp;

    const int64_t y0 = s->y0;
    uint8_t fl = 0;

    /* Parabola through (ym1, y0, yp1): vertex offset in Q15 samples,
     * clamped to +-0.5 sample. den = ym1 - 2*y0 + yp1 is the discrete
     * curvature; a strict local max needs den < 0 — den >= 0 (flat top,
     * or no +1 neighbor was observed: want_next still set) skips the
     * interpolation instead of dividing by garbage. */
    int32_t delta_q15 = 0;
    if (s->k_interp_en && !s->want_next) {
        int64_t den = s->ym1 - 2 * y0 + s->yp1;
        if (den < 0) {
            int64_t dq = ((s->ym1 - s->yp1) * (1LL << 14)) / den;
            if (dq < -16384) dq = -16384;
            if (dq > 16384) dq = 16384;
            delta_q15 = (int32_t)dq;
            fl |= NC_IBIF_INTERPOLATED;
        }
    }
    uint64_t t_beat = (uint64_t)((int64_t)s->t_peak_us +
                                 (((int64_t)delta_q15 * s->ts_us) >> 15));

    /* Confidence terms, computed against the PRE-update context so the
     * current beat is judged by history, not by itself. */
    int64_t thr_c = (thr > 0) ? thr : 1;
    int64_t margin = 40 * (y0 - thr) / thr_c;
    if (margin < 0) margin = 0;
    if (margin > 40) margin = 40;

    int64_t ema_c = (s->amp_ema > 0) ? s->amp_ema : 1;
    int64_t dev = y0 - s->amp_ema;
    if (dev < 0) dev = -dev;
    int64_t incons = 40 * dev / ema_c;
    if (incons > 40) incons = 40;
    int64_t consist = 40 - incons;

    /* Interval vs previous accepted beat (see nc_ibi.h contract). */
    uint32_t dt_ms = 0;
    if (s->t_last_valid) {
        uint64_t r = ((t_beat - s->t_last_us) + 500u) / 1000u;
        dt_ms = (r > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)r;
    }

    bool emit;
    int plaus;
    uint16_t ibi_out = 0;
    if (!s->t_last_valid || dt_ms > (uint32_t)s->k_ibi_max_ms) {
        /* First beat ever / first beat after a too-long gap: no interval
         * exists. Emit the timestamp with ibi_ms = 0 + FIRST_AFTER_GAP;
         * plausibility context resets. */
        fl |= NC_IBIF_FIRST_AFTER_GAP;
        plaus = (s->prev_ibi_ms > 0) ? 8 : 0;
        s->prev_ibi_ms = 0;
        emit = true;
    } else if (dt_ms < (uint32_t)s->k_ibi_min_ms) {
        /* Too-short interval (double-fire class): emit nothing, but the
         * beat clock still advances (spec). */
        plaus = 0;
        emit = false;
    } else {
        if (s->prev_ibi_ms > 0) {
            uint32_t p = s->prev_ibi_ms;
            plaus = (dt_ms >= p - p / 4 && dt_ms <= p + p / 4) ? 20 : 8;
        } else {
            plaus = 0;
        }
        ibi_out = (uint16_t)dt_ms;
        s->prev_ibi_ms = ibi_out;
        emit = true;
    }
    s->t_last_valid = true;
    s->t_last_us = t_beat;

    /* Trackers update on every accepted beat, emitted or not. The >> 2
     * on a possibly-negative delta relies on arithmetic right shift
     * (gcc/clang-defined; matches the RV32 sra the target emits). */
    int64_t tgt = y0 * 256; /* Q8 */
    s->peak_track += (tgt - s->peak_track) >> 2;
    if (s->peak_track < (tgt >> 1)) s->peak_track = tgt >> 1;
    if (s->amp_ema == 0) s->amp_ema = y0;
    else s->amp_ema += (y0 - s->amp_ema) >> 2;

    if (emit) {
        if (s->gated_since_beat) fl |= NC_IBIF_GATED_CTX;
        int conf = (int)(margin + consist) + plaus;
        if (conf < 0) conf = 0;
        if (conf > 100) conf = 100;
        out->t_beat_us = t_beat;
        out->ibi_ms = ibi_out;
        out->confidence = (uint8_t)conf;
        out->flags = fl;
    }
    s->gated_since_beat = false;
    return emit;
}

bool nc_ibi_update(nc_ibi_t *s, int32_t bp, uint64_t t_us, bool gated,
                   nc_ibi_rec_t *out)
{
    /* Rectified derivative. int64 diff: INT32 endpoints cannot wrap;
     * saturate into the ring's int32 slots (real band-passed PPG is
     * 25-bit at most — saturation is a defensive bound, not a path). */
    int32_t d = 0;
    if (s->have_bp) {
        int64_t dd = (int64_t)bp - s->bp_prev;
        if (dd > 0) d = (dd > INT32_MAX) ? INT32_MAX : (int32_t)dd;
    } else {
        s->have_bp = true;
    }
    s->bp_prev = bp;

    /* Gate exit: the ring holds artifact-era derivatives — restart it so
     * the first post-gate window cannot fire on stale energy. */
    if (s->was_gated && !gated) {
        memset(s->ring, 0, sizeof s->ring);
        s->ssf_sum = 0;
        s->ring_idx = 0;
    }
    s->was_gated = gated;

    /* Slope-sum over the ring window (O(1) update, exact int64 sum). */
    s->ssf_sum += (int64_t)d - s->ring[s->ring_idx];
    s->ring[s->ring_idx] = d;
    if (++s->ring_idx >= s->ssf_win) s->ring_idx = 0;
    const int64_t ssf = s->ssf_sum;

    /* Threshold tracker decays every sample; floor at ibi_thr_min so a
     * silent input (dropout) cannot drag the threshold to zero. */
    s->peak_track = pt_decay(s->peak_track, s->decay_q31);
    int64_t thr = (s->peak_track * s->k_thr_frac_x100 / 100) >> 8;
    if (thr < s->k_thr_min) thr = s->k_thr_min;

    if (s->refract_cnt) s->refract_cnt--;

    /* Gated sample: annotation-only policy — SSF/threshold state above
     * keeps flowing, but no capture, no emission; consistency context
     * resets (handoff §5.9). */
    if (gated) {
        s->in_peak = false;
        s->want_next = false;
        s->gated_since_beat = true;
        s->amp_ema = 0;
        s->prev_ibi_ms = 0;
        s->ssf_prev = ssf;
        return false;
    }

    bool emitted = false;
    if (s->in_peak) {
        /* Order matters: capture the +1 neighbor of the current running
         * max BEFORE possibly promoting this sample to the new max, and
         * before the commit test (so yp1 is valid at commit). */
        if (s->want_next) {
            s->yp1 = ssf;
            s->want_next = false;
        }
        if (ssf > s->y0) {
            s->ym1 = s->ssf_prev;
            s->y0 = ssf;
            s->t_peak_us = t_us;
            s->want_next = true;
        }
        if (2 * ssf < s->y0 || ssf < thr)
            emitted = commit(s, thr, out);
    } else if (!s->refract_cnt && ssf > thr) {
        s->in_peak = true;
        s->y0 = ssf;
        s->ym1 = s->ssf_prev;
        s->yp1 = ssf;              /* placeholder until want_next fires */
        s->want_next = true;
        s->t_peak_us = t_us;
    }
    s->ssf_prev = ssf;
    return emitted;
}
