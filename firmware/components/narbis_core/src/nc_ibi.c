/*
 * nc_ibi.c — slope-sum + adaptive-threshold beat detector (fixed point).
 * See nc_ibi.h for the emission contract; handoff §5.8 for the design.
 */
#include "narbis/nc_ibi.h"
#include "narbis/nc_knobs.h"
#include "narbis/nc_types.h"
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
    s->k_art_en        = nc_knob_get(KNOB_IBI_ART_EN) != 0;
    s->k_art_tol_pct   = (uint8_t)nc_knob_get(KNOB_IBI_ART_TOL_PCT);
    s->k_art_tol_motion_pct = (uint8_t)nc_knob_get(KNOB_IBI_ART_TOL_MOTION_PCT);
    s->k_refract_frac_pct = (uint8_t)nc_knob_get(KNOB_REFRACT_FRAC_PCT);
    s->k_fiducial = (uint8_t)nc_knob_get(KNOB_IBI_FIDUCIAL);
    nc_ibi_plaus_reset(&s->plaus);
    derive(s, ts_us_measured, true);
}

/* ------------------------------------------------------------------ */
/* Plausibility (ARTIFACT, nc_ibi.h)                                   */
/* ------------------------------------------------------------------ */

void nc_ibi_plaus_reset(nc_ibi_plaus_t *p)
{
    memset(p, 0, sizeof *p);
}

/* Insertion sort into s (n <= NC_IBI_ART_ENV), ~1 call per beat. */
static void sort_u16(uint16_t *s, const uint16_t *v, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) s[i] = v[i];
    for (uint8_t i = 1; i < n; i++) {
        uint16_t x = s[i];
        uint8_t j = i;
        while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; j--; }
        s[j] = x;
    }
}

static uint16_t median_u16(const uint16_t *v, uint8_t n)
{
    uint16_t s[NC_IBI_ART_ENV];
    sort_u16(s, v, n);
    return (n & 1) ? s[n / 2] : (uint16_t)(((uint32_t)s[n / 2 - 1] + s[n / 2]) / 2u);
}

/* core: accepted within tolerance of the rhythm, not by the envelope
 * alone — the adaptive refractory's reference (nc_ibi_plaus_core_median). */
static void plaus_accept(nc_ibi_plaus_t *p, uint16_t ibi_ms, bool core)
{
    p->ref[p->idx] = ibi_ms;
    p->idx = (uint8_t)((p->idx + 1) % NC_IBI_ART_ENV);
    if (p->n < NC_IBI_ART_ENV) p->n++;
    p->last = ibi_ms;
    if (core) {
        p->core[p->core_idx] = ibi_ms;
        p->core_idx = (uint8_t)((p->core_idx + 1) % NC_IBI_ART_REF);
        if (p->core_n < NC_IBI_ART_REF) p->core_n++;
    }
}

uint16_t nc_ibi_plaus_median(const nc_ibi_plaus_t *p)
{
    if (p->n < 3) return 0;
    const uint8_t m = (p->n < NC_IBI_ART_REF) ? p->n : NC_IBI_ART_REF;
    uint16_t v[NC_IBI_ART_REF];
    for (uint8_t k = 0; k < m; k++) {          /* the newest m, from idx back */
        v[k] = p->ref[(p->idx + NC_IBI_ART_ENV - 1u - k) % NC_IBI_ART_ENV];
    }
    return median_u16(v, m);
}

uint16_t nc_ibi_plaus_core_median(const nc_ibi_plaus_t *p)
{
    return (p->core_n < 3) ? 0 : median_u16(p->core, p->core_n);
}

/* Inside the breathing envelope: the range of the accepted intervals
 * without the single longest and shortest, widened by 1/NC_IBI_ART_ENV_WIDEN
 * of it unless the beat came through motion (nc_ibi.h contract). */
static bool in_envelope(const nc_ibi_plaus_t *p, uint16_t ibi_ms, bool motion)
{
    if (p->n < NC_IBI_ART_ENV_MIN) return false;
    uint16_t s[NC_IBI_ART_ENV];
    sort_u16(s, p->ref, p->n);
    const int32_t lo = s[1], hi = s[p->n - 2];
    const int32_t w = motion ? 0 : (hi - lo) / NC_IBI_ART_ENV_WIDEN;
    return (int32_t)ibi_ms >= lo - w && (int32_t)ibi_ms <= hi + w;
}

/* |a - b| * 100 > tol * b, in integers. */
static bool off_by(uint16_t a, uint16_t b, uint8_t tol_pct)
{
    const uint32_t d = (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
    return d * 100u > (uint32_t)tol_pct * b;
}

bool nc_ibi_plaus_check(nc_ibi_plaus_t *p, uint16_t ibi_ms, uint8_t tol_pct, bool motion)
{
    if (p->n < 3) {                            /* warm-up: nothing to judge by */
        plaus_accept(p, ibi_ms, true);
        return false;
    }
    const uint16_t med = nc_ibi_plaus_median(p);
    const uint16_t cmed = nc_ibi_plaus_core_median(p);
    const uint32_t x100 = (uint32_t)ibi_ms * 100u;
    bool flag, core = false;
    if (x100 > NC_IBI_ART_MISSED_PCT * (uint32_t)p->last ||
        x100 < NC_IBI_ART_SPLIT_PCT * (uint32_t)p->last) {
        flag = true;                           /* a missed beat / a split one */
    } else {
        core = !(off_by(ibi_ms, med, tol_pct) && off_by(ibi_ms, p->last, tol_pct));
        flag = !core && !in_envelope(p, ibi_ms, motion);
    }
    /* The other part of a split: a short interval right after a flagged
     * short one, the two together one plausible interval (in the envelope).
     * A dicrotic hump splits a pulse unevenly (542 + 589 for a ~1130 ms
     * pulse, 2026-10-16), and a wide breathing envelope can reach the
     * longer part. Two real fast beats that agree are confirmed below. */
    if (!flag && p->run_n && x100 < (100u - tol_pct) * (uint32_t)med) {
        const uint16_t prev = p->run[p->run_n - 1];
        const uint32_t sum = (uint32_t)prev + ibi_ms;
        if (prev < med && sum <= UINT16_MAX &&
            (p->n >= NC_IBI_ART_ENV_MIN ? in_envelope(p, (uint16_t)sum, motion)
                                        : !off_by((uint16_t)sum, med, tol_pct))) {
            flag = true;
            core = false;
        }
    }
    if (!flag) {
        p->run_n = 0;
        p->run_len = 0;
        plaus_accept(p, ibi_ms, core);
        return false;
    }
    /* Flagged. Right after one flagged interval, one that agrees with it,
     * both within a breathing swing of the median (0.7-1.5x), confirms a
     * real swing: accept this one and let both into the reference. */
    if (p->run_n == 1) {
        const uint16_t prev = p->run[0];
        const bool prev_swing = (uint32_t)prev * 10u >= 7u * med && (uint32_t)prev * 10u <= 15u * med;
        const bool this_swing = (uint32_t)ibi_ms * 10u >= 7u * med && (uint32_t)ibi_ms * 10u <= 15u * med;
        if (!off_by(ibi_ms, prev, tol_pct) && prev_swing && this_swing) {
            p->run_n = 0;
            p->run_len = 0;
            plaus_accept(p, prev, true);
            plaus_accept(p, ibi_ms, true);
            return false;
        }
    }
    /* NC_IBI_ART_ANCHOR in a row that agree with each other are a real rate
     * change: adopt them as the reference, when within a breathing swing
     * (0.7-1.5x) of the refractory's reference. A run beyond it is a halved
     * rhythm (double counts) or a doubled one (missed beats): a heart does
     * not change that much in four beats, and adopting it would set the
     * refractory to match and keep the error going. It takes NC_IBI_ART_ENV
     * in a row — then the reference is what was wrong. */
    if (p->run_len < UINT8_MAX) p->run_len++;
    if (p->run_n < NC_IBI_ART_ANCHOR) {
        p->run[p->run_n++] = ibi_ms;
    } else {
        for (uint8_t i = 1; i < NC_IBI_ART_ANCHOR; i++) p->run[i - 1] = p->run[i];
        p->run[NC_IBI_ART_ANCHOR - 1] = ibi_ms;
    }
    if (p->run_n == NC_IBI_ART_ANCHOR) {
        uint16_t lo = p->run[0], hi = p->run[0];
        for (uint8_t i = 1; i < NC_IBI_ART_ANCHOR; i++) {
            if (p->run[i] < lo) lo = p->run[i];
            if (p->run[i] > hi) hi = p->run[i];
        }
        const uint16_t rmed = median_u16(p->run, NC_IBI_ART_ANCHOR);
        const uint32_t ref = cmed ? cmed : med;
        const bool swing = (uint32_t)rmed * 10u >= 7u * ref && (uint32_t)rmed * 10u <= 15u * ref;
        if ((uint32_t)(hi - lo) * 100u <= (uint32_t)tol_pct * rmed &&
            (swing || p->run_len >= NC_IBI_ART_ENV)) {
            uint16_t run[NC_IBI_ART_ANCHOR];   /* the reset clears p->run */
            memcpy(run, p->run, sizeof run);
            nc_ibi_plaus_reset(p);
            for (uint8_t i = 0; i < NC_IBI_ART_ANCHOR; i++) plaus_accept(p, run[i], true);
        }
    }
    return true;
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

/* Onset of the captured beat: where its slope sum first rose through half
 * its peak, linearly interpolated between the two samples around it. Found
 * from the capture's first sample: back while still above half (the rise
 * began before the threshold), or forward while not yet above it. False
 * when the crossing has left the history (then the peak time stands). */
static bool onset_time(const nc_ibi_t *s, uint64_t *t_out)
{
    const int64_t half = s->y0 / 2;
    const uint32_t oldest = s->n_smp - (NC_IBI_HIST - 1u);
    uint32_t k = s->start_n;                       /* first sample > half */
    if (s->ssf_hist[k % NC_IBI_HIST] > half) {
        while (k != oldest && s->ssf_hist[(k - 1u) % NC_IBI_HIST] > half) k--;
        if (k == oldest) return false;
    } else {
        while (k != s->peak_n && s->ssf_hist[k % NC_IBI_HIST] <= half) k++;
        if (k == s->start_n) return false;
    }
    const int64_t a = s->ssf_hist[(k - 1u) % NC_IBI_HIST];   /* <= half */
    const int64_t b = s->ssf_hist[k % NC_IBI_HIST];          /* >  half */
    if (b <= a || a > half) return false;
    const int64_t frac_q15 = ((half - a) << 15) / (b - a);   /* 0..1 of a sample */
    /* k sits (peak_n - k) samples before the max; the crossing a fraction
     * of a sample after k - 1. */
    const int64_t back_q15 = ((int64_t)(s->peak_n - k + 1u) << 15) - frac_q15;
    *t_out = (uint64_t)((int64_t)s->t_peak_us - ((back_q15 * (int64_t)s->ts_us) >> 15));
    return true;
}

/* First hump of the captured beat: the earliest local max of its slope sum
 * that reaches 70 % of the capture's max, parabola-refined. A two-stage
 * upstroke has two near-equal humps; this keeps the mark on the first. */
static bool first_hump_time(const nc_ibi_t *s, uint64_t *t_out, bool *refined)
{
    for (uint32_t j = s->start_n; j != s->peak_n; j++) {
        const int64_t y = s->ssf_hist[j % NC_IBI_HIST];
        if (y * 10 < s->y0 * 7) continue;
        const int64_t ym = s->ssf_hist[(j - 1u) % NC_IBI_HIST];
        const int64_t yp = s->ssf_hist[(j + 1u) % NC_IBI_HIST];
        if (!(y >= ym && y > yp)) continue;
        int64_t delta_q15 = 0;
        const int64_t den = ym - 2 * y + yp;
        *refined = false;
        if (den < 0) {
            delta_q15 = ((ym - yp) * (1LL << 14)) / den;
            if (delta_q15 < -16384) delta_q15 = -16384;
            if (delta_q15 > 16384) delta_q15 = 16384;
            *refined = true;
        }
        const int64_t back_q15 = ((int64_t)(s->peak_n - j) << 15) - delta_q15;
        *t_out = (uint64_t)((int64_t)s->t_peak_us - ((back_q15 * (int64_t)s->ts_us) >> 15));
        return true;
    }
    return false;   /* the max itself is the first hump: the peak time stands */
}

/* Threshold tracker step toward a slope-sum peak y. The >> 2 on a
 * possibly-negative delta relies on arithmetic right shift
 * (gcc/clang-defined; matches the RV32 sra the target emits). */
static void track_peak(nc_ibi_t *s, int64_t y)
{
    const int64_t tgt = y * 256; /* Q8 */
    s->peak_track += (tgt - s->peak_track) >> 2;
    if (s->peak_track < (tgt >> 1)) s->peak_track = tgt >> 1;
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
    if (s->k_fiducial == 1) {
        uint64_t t_on;
        if (onset_time(s, &t_on)) {
            t_beat = t_on;
            fl |= NC_IBIF_INTERPOLATED;
        }
    } else if (s->k_fiducial == 2) {
        uint64_t t_h;
        bool refined = false;
        if (first_hump_time(s, &t_h, &refined)) {
            t_beat = t_h;
            fl = (uint8_t)(refined ? (fl | NC_IBIF_INTERPOLATED) : (fl & ~NC_IBIF_INTERPOLATED));
        }
    }

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
        nc_ibi_plaus_reset(&s->plaus);       /* a gap: the old rate is moot */
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
        const uint8_t tol = s->motion_since_beat ? s->k_art_tol_motion_pct : s->k_art_tol_pct;
        if (s->k_art_en && nc_ibi_plaus_check(&s->plaus, ibi_out, tol, s->motion_since_beat)) {
            fl |= NC_IBIF_ARTIFACT;
        }
        emit = true;
    }
    s->t_last_valid = true;
    s->t_last_us = t_beat;

    /* Adaptive refractory: a double count (a second slope peak inside one
     * pulse, from the dicrotic wave or from chewing/talking noise) lands at
     * ~40-55 % of the interval, which the fixed refract_ms only covers at
     * heart rates above ~107 bpm. So the next beat also waits out
     * refract_frac_pct of the median CLEAN interval (the plausibility
     * reference: flagged beats never enter it, so a double count cannot
     * shorten its own guard). No reference yet (warm-up, after a gap):
     * refract_ms alone. Measured on 2026-10-09 against an H10: chewing and
     * talking read a median 570 ms against the strap's 750 ms.
     * Only the core intervals (within tolerance of the rhythm) count, not
     * ones the breathing envelope alone let through: on the 2026-10-14
     * talk replay two junk shorts the envelope took shortened the guard by
     * 14 ms and five more junk beats came through after them. */
    const uint32_t med_ms = nc_ibi_plaus_core_median(&s->plaus);
    if (s->k_refract_frac_pct && med_ms) {
        const uint32_t adapt_ms = (med_ms * s->k_refract_frac_pct + 50u) / 100u;
        const uint32_t adapt = smp_from_ms(adapt_ms, s->ts_us);
        if (adapt > s->refract_cnt) s->refract_cnt = adapt;
    }

    /* Trackers update on every accepted beat, emitted or not (the >> 2
     * below: arithmetic right shift, as in track_peak). */
    track_peak(s, y0);
    if (s->amp_ema == 0) s->amp_ema = y0;
    else s->amp_ema += (y0 - s->amp_ema) >> 2;

    if (emit) {
        if (s->gated_since_beat) fl |= NC_IBIF_GATED_CTX;
        if (s->motion_since_beat) fl |= NC_IBIF_MOTION;
        int conf = (int)(margin + consist) + plaus;
        if (conf < 0) conf = 0;
        if (conf > 100) conf = 100;
        if ((fl & NC_IBIF_ARTIFACT) && conf > NC_IBI_ART_CONF_MAX) {
            conf = NC_IBI_ART_CONF_MAX;      /* for consumers that ignore flags */
        }
        out->t_beat_us = t_beat;
        out->ibi_ms = ibi_out;
        out->confidence = (uint8_t)conf;
        out->flags = fl;
    }
    s->gated_since_beat = false;
    s->motion_since_beat = false;
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
    s->ssf_hist[s->n_smp % NC_IBI_HIST] = ssf;

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
        s->n_smp++;
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
            s->peak_n = s->n_smp;
            s->want_next = true;
        }
        if (2 * ssf < s->y0 || ssf < thr)
            emitted = commit(s, thr, out);
    } else if (!s->refract_cnt && s->armed && ssf > thr) {
        s->in_peak = true;
        s->armed = false;
        s->y0 = ssf;
        s->ym1 = s->ssf_prev;
        s->yp1 = ssf;              /* placeholder until want_next fires */
        s->want_next = true;
        s->t_peak_us = t_us;
        s->start_n = s->peak_n = s->n_smp;
    }

    /* Re-arm: a capture may start only after the slope sum has come back
     * down to half the threshold since the last one. Between two pulses
     * the band-passed signal runs the other way for longer than the SSF
     * window, so the slope sum falls to zero: before 1278 of 1291
     * captures on four recordings (the H10 chew and talk blocks
     * included), and the 8 that started without dipping below half the
     * threshold first were all extra beats. Such a capture is the tail
     * of a hump the refractory hid, still above the threshold when it
     * ended: on the 2026-10-16 recording a precursor bump was committed,
     * the refractory hid the pulse's main hump, and its tail was counted
     * ~550 ms later whenever the interval jumped (820 -> 1200 ms; 12
     * beats where 8 pulses were, 36-44 s). The dip between precursor and
     * main hump stayed at 0.77-1.03 x thr there; one down to half the
     * threshold would re-arm in between, and the tail would count again.
     * The hidden hump also tells the threshold where the pulses are: one
     * over NC_IBI_HID_X times the committed peak means a precursor was
     * caught, and without this step the tracker follows the precursors
     * down until ever smaller bumps fire (42-47 s there; the tails used
     * to break that by accident). */
    if (!s->in_peak && !s->armed) {
        if (ssf > s->y_hid) s->y_hid = ssf;
        if (2 * ssf <= thr) {
            s->armed = true;
            if (s->y_hid > NC_IBI_HID_X * s->y0) track_peak(s, s->y_hid);
            s->y_hid = 0;
        }
    }
    s->ssf_prev = ssf;
    s->n_smp++;
    return emitted;
}

/* ------------------------------------------------------------------ */
/* AGC step compensation (nc_agcstep.h)                                */
/* ------------------------------------------------------------------ */
static int32_t scale_i32(int32_t v, int32_t r_q16)
{
    return nc_sat32(nc_mul_q16(v, r_q16));
}

void nc_ibi_scale(nc_ibi_t *s, int32_t r_q16)
{
    s->bp_prev = scale_i32(s->bp_prev, r_q16);
    for (int i = 0; i < NC_IBI_SSF_MAX; i++) {
        s->ring[i] = scale_i32(s->ring[i], r_q16);
    }
    /* Re-sum rather than scale the sum: the ring entries were rounded
     * one by one, and the O(1) update relies on sum == sum(ring). */
    int64_t sum = 0;
    for (int i = 0; i < s->ssf_win; i++) {
        sum += s->ring[i];
    }
    s->ssf_sum = sum;
    s->ssf_prev = nc_mul_q16(s->ssf_prev, r_q16);
    s->peak_track = nc_mul_q16(s->peak_track, r_q16);
    s->y0 = nc_mul_q16(s->y0, r_q16);
    s->ym1 = nc_mul_q16(s->ym1, r_q16);
    s->yp1 = nc_mul_q16(s->yp1, r_q16);
    s->y_hid = nc_mul_q16(s->y_hid, r_q16);
    s->amp_ema = nc_mul_q16(s->amp_ema, r_q16);
}

void nc_ibi_restart_amp(nc_ibi_t *s)
{
    memset(s->ring, 0, sizeof s->ring);
    s->ring_idx = 0;
    s->ssf_sum = 0;
    s->ssf_prev = 0;
    s->have_bp = false;
    s->bp_prev = 0;
    s->peak_track = 0;
    s->in_peak = false;
    s->want_next = false;
    s->y_hid = 0;              /* re-arms on the restarted (zero) slope sum */
    s->amp_ema = 0;
}
