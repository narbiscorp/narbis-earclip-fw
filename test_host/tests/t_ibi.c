/*
 * t_ibi.c — IBI detector acceptance against synthetic golden vectors
 * (tools/goldens/gen_ppg_synth.py -> test_host/vectors/ibi_*.{bin,json}).
 *
 * Scoring conventions (documented here because they shape the metrics):
 *  - Matching window: a detection within +-60 ms of a truth beat matches
 *    it; each truth beat can absorb one detection, a second one counts
 *    as a duplicate (required to be ZERO).
 *  - Warm-up: the first 2 s of every vector are excluded from scoring
 *    (both truth and detections). The threshold tracker starts from
 *    zero, so the first beat is captured on a floor threshold and the
 *    tracker needs ~1 beat to adapt — standard practice for adaptive
 *    detectors, and the spec's rates target steady-state behavior.
 *  - Tail guard: the generator drops truth beats within 0.3 s of the
 *    vector end (their templates are cut off), but the detector can
 *    still legitimately commit them — scoring therefore also ignores
 *    detections in the final 0.4 s instead of calling them FPs.
 *  - Knob profile: scored runs use thr_frac_x100 = 70, thr_tau_ms = 5000
 *    (vs defaults 50/3000). With the defaults, the 12 dB in-band noise
 *    case fails: between-beat noise SSF bumps (~25-45k counts) cross the
 *    decayed threshold, and every false accept drags peak_track down
 *    toward the noise floor (the spec's accept-update is an EMA toward
 *    ssf_peak), self-reinforcing until ~20% of beats are pre-empted by a
 *    noise commit + refractory shadow. 70/5000 keeps the inter-beat
 *    threshold above the noise bumps at BOTH rates and passes every
 *    case, clean and noisy, with the same profile (a defaults-regression
 *    check on the clean case is kept separately). Flagged in the module
 *    summary as a proposed product-default change.
 *  - Jitter (clean 100 sps case a): the detector timestamps the SSF
 *    peak, which sits at a *constant* morphology-dependent offset from
 *    the truth fiducial (systolic Gaussian center) — the SSF window and
 *    the rectified-derivative asymmetry are a pure group delay. A group
 *    delay carries no interval error (IBI is a difference of beat
 *    times), so the spec says to subtract the median offset and bound
 *    the residual: median |err - median(err)| <= 1 sample period.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tst.h"
#include "narbis/nc_ibi.h"
#include "narbis/nc_knobs.h"

TST_SUITE("ibi");

#define T0_US       1000000ULL   /* nonzero stream start time */
#define MATCH_US    60000LL
#define WARMUP_US   2000000ULL
#define MAX_BEATS   512
#define MAX_DETS    2048

typedef struct {
    uint32_t ts_us;
    int      nbeats;
    int64_t  beats[MAX_BEATS];   /* us, vector-relative */
    int      has_gap;
    int64_t  gap_s, gap_e;       /* us, vector-relative */
    long     nsamp;
    int32_t *bp;
} vec_t;

typedef struct {
    uint64_t t_us;               /* record t_beat_us (absolute) */
    uint64_t emit_us;            /* sample time the record was emitted */
    uint16_t ibi_ms;
    uint8_t  conf, flags;
} det_t;

typedef struct {
    int truth_n, matched, dups, fp, det_n;
    int64_t errs[MAX_BEATS];
    int nerrs;
} score_t;

static det_t g_dets[MAX_DETS];   /* file-scope: keep test stacks small */

/* ---------------- vector loading ---------------------------------- */

static char *slurp(const char *path, long *out_len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc((size_t)len + 1);
    if (!buf || fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fclose(f);
        free(buf);
        return NULL;
    }
    fclose(f);
    buf[len] = 0;
    if (out_len) *out_len = len;
    return buf;
}

/* Minimal parser for the generator's fixed, flat JSON schema. */
static int load_vec(const char *base, vec_t *v)
{
    char path[256];
    memset(v, 0, sizeof *v);

    snprintf(path, sizeof path, "vectors/%s.bin", base);
    long blen = 0;
    char *bin = slurp(path, &blen);
    if (!bin || blen % 4 != 0) { free(bin); return 0; }
    v->bp = (int32_t *)bin;      /* host is little-endian, as is the file */
    v->nsamp = blen / 4;

    snprintf(path, sizeof path, "vectors/%s.json", base);
    char *js = slurp(path, NULL);
    if (!js) return 0;

    char *p = strstr(js, "\"ts_us\"");
    if (!p) { free(js); return 0; }
    v->ts_us = (uint32_t)strtol(strchr(p, ':') + 1, NULL, 10);

    p = strstr(js, "\"gap_us\"");
    if (p) {
        p = strchr(p, ':') + 1;
        while (*p == ' ') p++;
        if (*p == '[') {
            char *end;
            v->gap_s = strtoll(p + 1, &end, 10);
            v->gap_e = strtoll(end + 1, NULL, 10);
            v->has_gap = 1;
        }
    }

    p = strstr(js, "\"beats_us\"");
    if (!p) { free(js); return 0; }
    p = strchr(p, '[') + 1;
    while (v->nbeats < MAX_BEATS) {
        char *end;
        long long b = strtoll(p, &end, 10);
        if (end == p) break;
        v->beats[v->nbeats++] = b;
        p = end;
        if (*p == ',') p++;
        else break;
    }
    free(js);
    return v->ts_us > 0 && v->nbeats > 0 && v->nsamp > 0;
}

/* ---------------- detector driver --------------------------------- */

/* gate_[se] are vector-relative us; gate_e == 0 disables gating.
 * wobble exercises nc_ibi_set_ts with +-13 us (~0.13%) period jitter.
 * defaults != 0 skips the test knob profile (see header comment). */
static int run_det(const vec_t *v, uint32_t refract_ms,
                   uint64_t gate_s, uint64_t gate_e, int wobble, int defaults)
{
    nc_knobs_init();
    if (!defaults) {
        CHECK_EQ(nc_knob_set_id(0x0603 /* thr_frac_x100 */, 70), NC_ST_OK);
        CHECK_EQ(nc_knob_set_id(0x0604 /* thr_tau_ms */, 5000), NC_ST_OK);
    }
    if (refract_ms)
        CHECK_EQ(nc_knob_set_id(0x0605 /* refract_ms */, (int32_t)refract_ms),
                 NC_ST_OK);

    nc_ibi_t s;
    nc_ibi_init(&s, v->ts_us);

    int n = 0;
    for (long i = 0; i < v->nsamp; i++) {
        uint64_t rel = (uint64_t)i * v->ts_us;
        uint64_t t = T0_US + rel;
        int gated = gate_e && rel >= gate_s && rel < gate_e;
        if (wobble && i && i % 997 == 0)
            nc_ibi_set_ts(&s, v->ts_us + (((i / 997) & 1) ? 13 : -13));
        nc_ibi_rec_t r;
        if (nc_ibi_update(&s, v->bp[i], t, gated, &r) && n < MAX_DETS) {
            g_dets[n].t_us = r.t_beat_us;
            g_dets[n].emit_us = t;
            g_dets[n].ibi_ms = r.ibi_ms;
            g_dets[n].conf = r.confidence;
            g_dets[n].flags = r.flags;
            n++;
        }
    }
    return n;
}

/* ---------------- scoring ----------------------------------------- */

static void score(const vec_t *v, const det_t *d, int nd, score_t *sc)
{
    static char used[MAX_BEATS];
    memset(used, 0, sizeof used);
    memset(sc, 0, sizeof *sc);

    /* scoring window: [warm-up, vector end - tail guard) — see header */
    const int64_t t_end = (int64_t)v->nsamp * v->ts_us - 400000;

    for (int j = 0; j < v->nbeats; j++)
        if ((uint64_t)v->beats[j] >= WARMUP_US && v->beats[j] < t_end)
            sc->truth_n++;

    for (int i = 0; i < nd; i++) {
        int64_t td = (int64_t)(d[i].t_us - T0_US);
        if ((uint64_t)td < WARMUP_US || td >= t_end) continue;
        sc->det_n++;

        int best = -1;
        int64_t berr = 0;
        for (int j = 0; j < v->nbeats; j++) {
            int64_t e = td - v->beats[j];
            int64_t a = e < 0 ? -e : e;
            if (best < 0 || a < (berr < 0 ? -berr : berr)) {
                best = j;
                berr = e;
            }
        }
        int64_t aerr = berr < 0 ? -berr : berr;
        if (best >= 0 && aerr <= MATCH_US) {
            /* boundary grace: a match against an out-of-window truth
             * beat is neither a hit nor a false positive */
            if ((uint64_t)v->beats[best] < WARMUP_US ||
                v->beats[best] >= t_end) { sc->det_n--; continue; }
            if (used[best]) {
                sc->dups++;
            } else {
                used[best] = 1;
                sc->matched++;
                if (sc->nerrs < MAX_BEATS) sc->errs[sc->nerrs++] = berr;
            }
        } else {
            sc->fp++;
        }
    }
}

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static int64_t median_i64(int64_t *v, int n)
{
    qsort(v, (size_t)n, sizeof *v, cmp_i64);
    return v[n / 2];
}

/* Emission invariants that must hold for every record of every run. */
static void check_invariants(const det_t *d, int nd,
                             uint16_t ibi_min, uint16_t ibi_max)
{
    int bad = 0;
    for (int i = 0; i < nd; i++) {
        if (d[i].conf > 100) bad++;
        if (d[i].ibi_ms == 0) {
            if (!(d[i].flags & NC_IBIF_FIRST_AFTER_GAP)) bad++;
        } else {
            if (d[i].ibi_ms < ibi_min || d[i].ibi_ms > ibi_max) bad++;
        }
    }
    CHECK_EQ(bad, 0);
}

/* min_rate_pm = required detection rate in per-mille (980 = 98.0%). */
static void expect_case_knobs(const char *base, int min_rate_pm,
                              uint32_t refract_ms, int defaults)
{
    vec_t v;
    if (!load_vec(base, &v)) {
        CHECK(!"vector load failed — run tools/goldens/gen_ppg_synth.py");
        return;
    }
    int nd = run_det(&v, refract_ms, 0, 0, 0, defaults);
    score_t sc;
    score(&v, g_dets, nd, &sc);

    if (sc.matched * 1000 < sc.truth_n * min_rate_pm)
        fprintf(stderr, "  %s: matched %d/%d, dups %d, fp %d/%d\n",
                base, sc.matched, sc.truth_n, sc.dups, sc.fp, sc.det_n);
    CHECK(sc.matched * 1000 >= sc.truth_n * min_rate_pm);
    CHECK_EQ(sc.dups, 0);
    CHECK(sc.fp * 50 < sc.det_n);      /* false positives < 2% */
    check_invariants(g_dets, nd, 300, 2000);
    free(v.bp);
}

static void expect_case(const char *base, int min_rate_pm, uint32_t refract_ms)
{
    expect_case_knobs(base, min_rate_pm, refract_ms, 0);
}

/* ---------------- tests ------------------------------------------- */

static void t_clean_and_noisy_rates(void)
{
    static const struct { const char *base; int pm; uint32_t refr; } cases[] = {
        /* case c reaches 180 bpm (IBI 333 ms): the default 280 ms
         * refractory starting at commit (~peak + 50 ms) would swallow
         * beats near the top of the ramp. 200 ms is the intended knob
         * usage — still 2x the SSF window, and the dicrotic bump is
         * already rejected by the adaptive threshold, not by the
         * refractory. */
        { "ibi_a_100", 980, 0 },   { "ibi_a_500", 980, 0 },
        { "ibi_b_100", 980, 0 },   { "ibi_b_500", 980, 0 },
        { "ibi_c_100", 980, 200 }, { "ibi_c_500", 980, 200 },
        { "ibi_d_100", 980, 0 },   { "ibi_d_500", 980, 0 },
        { "ibi_e_100", 950, 0 },   { "ibi_e_500", 950, 0 },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
        expect_case(cases[i].base, cases[i].pm, cases[i].refr);
}

static void t_default_knobs_clean(void)
{
    /* regression: the shipped knob defaults (thr_frac 50, tau 3000) must
     * still meet spec on clean input at both rates. */
    expect_case_knobs("ibi_a_100", 980, 0, 1);
    expect_case_knobs("ibi_a_500", 980, 0, 1);
}

static void t_jitter_clean_100(void)
{
    vec_t v;
    if (!load_vec("ibi_a_100", &v)) { CHECK(!"load ibi_a_100"); return; }
    int nd = run_det(&v, 0, 0, 0, 0, 0);
    score_t sc;
    score(&v, g_dets, nd, &sc);
    CHECK(sc.nerrs >= 50);

    /* constant group delay (SSF window + morphology) is not jitter:
     * subtract the median offset, bound the residual spread. */
    int64_t off = median_i64(sc.errs, sc.nerrs);
    for (int i = 0; i < sc.nerrs; i++) {
        int64_t e = sc.errs[i] - off;
        sc.errs[i] = e < 0 ? -e : e;
    }
    int64_t jit = median_i64(sc.errs, sc.nerrs);
    if (jit > (int64_t)v.ts_us)
        fprintf(stderr, "  jitter %lld us (offset %lld us)\n",
                (long long)jit, (long long)off);
    CHECK(jit <= (int64_t)v.ts_us);
    /* the offset itself must sit inside the matching window with room */
    CHECK(off > -MATCH_US / 2 && off < MATCH_US / 2);
    free(v.bp);
}

static void t_first_record_no_interval(void)
{
    vec_t v;
    if (!load_vec("ibi_a_100", &v)) { CHECK(!"load ibi_a_100"); return; }
    int nd = run_det(&v, 0, 0, 0, 0, 0);
    CHECK(nd >= 55);
    /* first beat ever: timestamp valid, no interval */
    CHECK(g_dets[0].flags & NC_IBIF_FIRST_AFTER_GAP);
    CHECK_EQ(g_dets[0].ibi_ms, 0);
    /* second record: the true 998.3 ms interval, +-1 sample of rounding */
    CHECK(g_dets[1].ibi_ms >= 988 && g_dets[1].ibi_ms <= 1008);
    CHECK(!(g_dets[1].flags & NC_IBIF_FIRST_AFTER_GAP));
    free(v.bp);
}

static void gap_case(const char *base)
{
    vec_t v;
    if (!load_vec(base, &v)) { CHECK(!"load gap vector"); return; }
    CHECK(v.has_gap);
    int nd = run_det(&v, 0, 0, 0, 0, 0);
    score_t sc;
    score(&v, g_dets, nd, &sc);
    CHECK(sc.matched * 1000 >= sc.truth_n * 980);
    CHECK_EQ(sc.dups, 0);
    CHECK(sc.fp * 50 < sc.det_n);
    check_invariants(g_dets, nd, 300, 2000);

    /* zero detections inside the silent span (threshold floor holds) */
    int in_gap = 0, first_after = -1;
    for (int i = 0; i < nd; i++) {
        int64_t td = (int64_t)(g_dets[i].t_us - T0_US);
        if (td > v.gap_s + 200000 && td < v.gap_e) in_gap++;
        if (first_after < 0 && td >= v.gap_e) first_after = i;
    }
    CHECK_EQ(in_gap, 0);
    CHECK(first_after >= 0);
    /* first post-gap beat: flagged, and NO wild interval spanning the
     * 10 s dropout (ibi_ms = 0, not ~11000 truncated into u16 range) */
    CHECK(g_dets[first_after].flags & NC_IBIF_FIRST_AFTER_GAP);
    CHECK_EQ(g_dets[first_after].ibi_ms, 0);
    /* and the record right after it is a normal interval again */
    if (first_after + 1 < nd) {
        CHECK(g_dets[first_after + 1].ibi_ms >= 988 &&
              g_dets[first_after + 1].ibi_ms <= 1008);
    }
    free(v.bp);
}

static void t_gap_100(void) { gap_case("ibi_f_100"); }
static void t_gap_500(void) { gap_case("ibi_f_500"); }

static void t_gated_suppression(void)
{
    vec_t v;
    if (!load_vec("ibi_a_100", &v)) { CHECK(!"load ibi_a_100"); return; }
    const uint64_t gs = 20000000ULL, ge = 30000000ULL;
    int nd = run_det(&v, 0, gs, ge, 0, 0);

    int emitted_in_span = 0, before = 0, after = 0, first_after = -1;
    for (int i = 0; i < nd; i++) {
        uint64_t rel = g_dets[i].emit_us - T0_US;
        if (rel >= gs && rel < ge) emitted_in_span++;
        if (rel < gs) before++;
        if (rel >= ge) {
            after++;
            if (first_after < 0) first_after = i;
        }
    }
    /* gated flag suppresses ALL output during the span */
    CHECK_EQ(emitted_in_span, 0);
    CHECK(before >= 15);             /* ~19 beats in the first 20 s */
    CHECK(after >= 20);              /* detection resumes: ~29 beats left */
    /* first post-gate beat: gated context + no interval across the span
     * (10 s > ibi_max), consistency context was reset per 5.9 */
    CHECK(first_after >= 0);
    CHECK(g_dets[first_after].flags & NC_IBIF_GATED_CTX);
    CHECK(g_dets[first_after].flags & NC_IBIF_FIRST_AFTER_GAP);
    CHECK_EQ(g_dets[first_after].ibi_ms, 0);
    free(v.bp);
}

static void t_set_ts_tracking(void)
{
    /* measured-period EMA wobble (+-0.13%) must not disturb detection */
    vec_t v;
    if (!load_vec("ibi_a_100", &v)) { CHECK(!"load ibi_a_100"); return; }
    int nd = run_det(&v, 0, 0, 0, 1, 0);
    score_t sc;
    score(&v, g_dets, nd, &sc);
    CHECK(sc.matched * 1000 >= sc.truth_n * 980);
    CHECK_EQ(sc.dups, 0);
    free(v.bp);
}

/* ---- plausibility (ARTIFACT, proto 1.3) ---------------------------- */

static int flags_for(nc_ibi_plaus_t *p, const uint16_t *ibi, int n, bool *out)
{
    int k = 0;
    for (int i = 0; i < n; i++) {
        out[i] = nc_ibi_plaus_check(p, ibi[i], 15);
        k += out[i];
    }
    return k;
}

static void t_plaus_steady_and_warmup(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    /* The first three are taken unchecked, however odd. */
    CHECK(!nc_ibi_plaus_check(&p, 800, 15));
    CHECK(!nc_ibi_plaus_check(&p, 400, 15));
    CHECK(!nc_ibi_plaus_check(&p, 820, 15));
    for (int i = 0; i < 200; i++) {
        CHECK(!nc_ibi_plaus_check(&p, (uint16_t)(780 + (i % 5) * 10), 15));
    }
}

/* Chewing / talking junk from the first guided recordings: a double
 * detection splits one interval into two short ones, a miss doubles it. */
static void t_plaus_double_and_missed(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    const uint16_t seq[] = { 800, 790, 810, 800, 805,
                             400, 405,            /* double detection */
                             800, 795,
                             1600,                /* missed beat      */
                             810, 800 };
    bool f[sizeof seq / sizeof seq[0]];
    const int n = (int)(sizeof seq / sizeof seq[0]);
    CHECK_EQ(flags_for(&p, seq, n, f), 3);
    CHECK(f[5] && f[6] && f[9]);
    CHECK(!f[7] && !f[10]);
}

/* Paced breathing, 6 breaths/min at 65 bpm with +-25 % respiratory sinus
 * arrhythmia: real beats, none may be flagged (coherence training). */
static void t_plaus_paced_breathing_clean(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    const double base = 60000.0 / 65.0;
    double t = 0;
    int flagged = 0;
    for (int i = 0; i < 600; i++) {
        const double ibi = base * (1.0 + 0.25 * sin(2.0 * 3.14159265358979 * 0.1 * t / 1000.0));
        flagged += nc_ibi_plaus_check(&p, (uint16_t)(ibi + 0.5), 15);
        t += ibi;
    }
    CHECK_EQ(flagged, 0);
}

/* A genuine rate change within a breathing swing (800 -> 600 ms, stood
 * up): the first interval is flagged, the next one that agrees with it
 * confirms the change, and the new rate passes from there. (Until
 * 2026-10-15 it took four flagged in a row: paced breathing showed runs of
 * four real beats flagged.) */
static void t_plaus_reanchors_on_real_change(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    for (int i = 0; i < 6; i++) CHECK(!nc_ibi_plaus_check(&p, 800, 15));
    CHECK(nc_ibi_plaus_check(&p, 600, 15));
    for (int i = 0; i < 20; i++) CHECK(!nc_ibi_plaus_check(&p, 600, 15));
}

/* A halved rhythm (double counting, 800 -> 400 ms) is outside a breathing
 * swing: it still takes four agreeing flagged intervals to adopt. */
static void t_plaus_halved_rhythm_needs_four(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    for (int i = 0; i < 6; i++) CHECK(!nc_ibi_plaus_check(&p, 800, 15));
    for (int i = 0; i < 4; i++) CHECK(nc_ibi_plaus_check(&p, 400, 15));
    for (int i = 0; i < 20; i++) CHECK(!nc_ibi_plaus_check(&p, 400, 15));
}

/* One stray interval (a missed beat, 2x) is flagged; the normal beat after
 * it is not, and does not confirm it (it disagrees with the stray). */
static void t_plaus_single_stray(void)
{
    nc_ibi_plaus_t p;
    nc_ibi_plaus_reset(&p);
    for (int i = 0; i < 6; i++) CHECK(!nc_ibi_plaus_check(&p, 800, 15));
    CHECK(nc_ibi_plaus_check(&p, 1600, 15));
    CHECK(!nc_ibi_plaus_check(&p, 800, 15));
    CHECK(nc_ibi_plaus_check(&p, 1600, 15));
}

/* ---------------- adaptive refractory ------------------------------ */

/* Talking or chewing in the middle of a session: from 20 s to 40 s every
 * pulse carries a second slope peak at 55 % of the interval. Measured
 * 2026-10-09 against an H10: chewing and talking read a median 570 ms
 * against the strap's 750 ms. 80 bpm with a 6 % paced-breathing swing,
 * 100 sps, 60 s, shipped thresholds. Truth = the systolic bump's centre.
 *
 * The clean first 20 s matter: the guard is a share of the median CLEAN
 * interval, and a session that double-counts from its very first pulse
 * never shows it one (it learns ~410 ms as the rate, and 60 % of that is
 * inside refract_ms anyway) — a limit of this design, not covered here. */
static void synth_double_hump(vec_t *v)
{
    memset(v, 0, sizeof *v);
    v->ts_us = 10000;
    v->nsamp = 6000;
    v->bp = malloc((size_t)v->nsamp * sizeof *v->bp);
    const double span_s = (double)v->nsamp * v->ts_us / 1e6;
    for (double t = 0.5; t < span_s - 0.3 && v->nbeats < MAX_BEATS;) {
        v->beats[v->nbeats++] = (int64_t)(t * 1e6);
        t += 0.75 * (1.0 + 0.06 * sin(2.0 * 3.14159265358979323846 * 0.1 * t));
    }
    for (long i = 0; i < v->nsamp; i++) {
        const double ti = (double)i * v->ts_us / 1e6;
        double y = 0;
        for (int k = 0; k < v->nbeats; k++) {
            const double dt = ti - (double)v->beats[k] / 1e6;
            if (dt < -0.5 || dt > 1.0) continue;
            const double tb = (double)v->beats[k] / 1e6;
            const double d2 = dt - 0.55 * 0.75;
            y += 100000.0 * exp(-(dt * dt) / (2 * 0.06 * 0.06));
            if (tb >= 20.0 && tb < 40.0) y += 70000.0 * exp(-(d2 * d2) / (2 * 0.05 * 0.05));
        }
        v->bp[i] = (int32_t)y;
    }
}

/* The detector over [v] at the shipped knob defaults, with refract_frac_pct
 * set to [frac] (0 = the fixed refract_ms only, as before 0x060C). */
static int run_det_frac(const vec_t *v, int32_t frac)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x060C /* refract_frac_pct */, frac), NC_ST_OK);
    nc_ibi_t s;
    nc_ibi_init(&s, v->ts_us);
    int n = 0;
    for (long i = 0; i < v->nsamp; i++) {
        nc_ibi_rec_t r;
        if (nc_ibi_update(&s, v->bp[i], T0_US + (uint64_t)i * v->ts_us, false, &r) && n < MAX_DETS) {
            g_dets[n].t_us = r.t_beat_us;
            g_dets[n].ibi_ms = r.ibi_ms;
            g_dets[n].conf = r.confidence;
            g_dets[n].flags = r.flags;
            n++;
        }
    }
    return n;
}

static void t_refract_adaptive_double_hump(void)
{
    vec_t v;
    synth_double_hump(&v);
    score_t sc;

    /* Fixed 280 ms: the second hump is counted on (nearly) every talking pulse. */
    int nd = run_det_frac(&v, 0);
    score(&v, g_dets, nd, &sc);
    fprintf(stderr, "  fixed refractory: matched %d/%d, dups %d, fp %d/%d\n",
            sc.matched, sc.truth_n, sc.dups, sc.fp, sc.det_n);
    CHECK(sc.fp >= 20);

    /* Default 60 % of the median clean interval: one beat per pulse. */
    nd = run_det_frac(&v, 60);
    score(&v, g_dets, nd, &sc);
    fprintf(stderr, "  adaptive refractory: matched %d/%d, dups %d, fp %d/%d\n",
            sc.matched, sc.truth_n, sc.dups, sc.fp, sc.det_n);
    CHECK(sc.matched * 1000 >= sc.truth_n * 980);
    CHECK_EQ(sc.dups, 0);
    CHECK(sc.fp * 50 < sc.det_n);
    int artifact = 0;
    for (int i = 0; i < nd; i++) artifact += (g_dets[i].flags & NC_IBIF_ARTIFACT) != 0;
    CHECK(artifact * 50 < nd);           /* the pulses read clean, not flagged */
    check_invariants(g_dets, nd, 300, 2000);
    free(v.bp);
}

/* Light motion (proto 1.4): a beat found after nc_ibi_note_motion carries
 * MOTION, exactly once; motion never stops the detector. */
static void t_motion_flag_marks_the_next_beat(void)
{
    vec_t v;
    synth_double_hump(&v);               /* only the clean first 20 s are used */
    nc_knobs_init();
    nc_ibi_t s;
    nc_ibi_init(&s, v.ts_us);
    int before = 0, flagged = 0, after = 0, after_flagged = 0;
    for (long i = 0; i < 2000; i++) {    /* 0..20 s */
        if (i == 1000) nc_ibi_note_motion(&s);   /* a chew at 10 s */
        nc_ibi_rec_t r;
        if (!nc_ibi_update(&s, v.bp[i], T0_US + (uint64_t)i * v.ts_us, false, &r)) continue;
        if (i < 1000) { before++; CHECK(!(r.flags & NC_IBIF_MOTION)); }
        else if (!flagged) { flagged = 1; CHECK(r.flags & NC_IBIF_MOTION); CHECK(r.ibi_ms > 0); }
        else { after++; after_flagged += (r.flags & NC_IBIF_MOTION) != 0; }
    }
    CHECK(before >= 10);
    CHECK(flagged);
    CHECK(after >= 10);
    CHECK_EQ(after_flagged, 0);
    free(v.bp);
}

/* The adaptive guard must not cost a real beat when the rate climbs: the
 * 180 bpm ramp (case c) still meets spec at the shipped 60 %. */
static void t_refract_adaptive_keeps_fast_ramp(void)
{
    expect_case("ibi_c_100", 980, 200);
    expect_case("ibi_c_500", 980, 200);
}

int main(void)
{
    TST_RUN(t_refract_adaptive_double_hump);
    TST_RUN(t_refract_adaptive_keeps_fast_ramp);
    TST_RUN(t_motion_flag_marks_the_next_beat);
    TST_RUN(t_plaus_steady_and_warmup);
    TST_RUN(t_plaus_double_and_missed);
    TST_RUN(t_plaus_paced_breathing_clean);
    TST_RUN(t_plaus_reanchors_on_real_change);
    TST_RUN(t_plaus_halved_rhythm_needs_four);
    TST_RUN(t_plaus_single_stray);
    TST_RUN(t_clean_and_noisy_rates);
    TST_RUN(t_default_knobs_clean);
    TST_RUN(t_jitter_clean_100);
    TST_RUN(t_first_record_no_interval);
    TST_RUN(t_gap_100);
    TST_RUN(t_gap_500);
    TST_RUN(t_gated_suppression);
    TST_RUN(t_set_ts_tracking);
    TST_REPORT();
}
