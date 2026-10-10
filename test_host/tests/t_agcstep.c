/*
 * t_agcstep.c — AGC step compensation (nc_agcstep.h): sequencing, ratio
 * helpers, and end-to-end beat continuity through the real chain
 * (nc_dsp x2 -> DC-step gate -> nc_ibi) on a synthetic raw PPG with an
 * AGC step in the middle.
 *
 * Synthetic raw (ambient subtracted), 100 sps: DC 1.0M counts with a
 * +-1.5 % breathing modulation (0.25 Hz), a 1 % pulse (systolic +
 * dicrotic absorption bumps), heart rate 72 bpm +-4 % RSA, +-300 counts
 * of noise. The AGC step multiplies everything from t_step on by r, the
 * way an LED or TIA gain change does.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "tst.h"
#include "narbis/nc_agcstep.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_agcstep");

#define FS        100
#define TS_US     10000u
#define DUR_S     60
#define T_STEP_US (20ULL * 1000000ULL)
#define MAX_BEATS 200

typedef struct {
    double   t_true[MAX_BEATS];
    int      n_true;
} truth_t;

static uint32_t g_lcg = 12345u;
static double noise(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return ((double)(g_lcg >> 8) / (double)(1u << 24) - 0.5) * 600.0;   /* +-300 */
}

static void make_truth(truth_t *tr)
{
    double t = 0.4;
    tr->n_true = 0;
    while (t < DUR_S && tr->n_true < MAX_BEATS) {
        tr->t_true[tr->n_true++] = t;
        t += 0.833 * (1.0 + 0.04 * sin(2.0 * 3.14159265358979 * 0.25 * t));
    }
}

/* Absorption pulse at time t (s): sum of the beats' waveforms. */
static double pulse(const truth_t *tr, double t)
{
    double p = 0.0;
    for (int k = 0; k < tr->n_true; k++) {
        const double tau = t - tr->t_true[k];
        if (tau < -0.2 || tau > 1.2) continue;
        const double a = (tau - 0.12) / 0.05;
        const double b = (tau - 0.35) / 0.08;
        p += exp(-a * a) + 0.4 * exp(-b * b);
    }
    return p;
}

static int32_t raw_at(const truth_t *tr, double t, double scale_red, double r)
{
    const double dc = 1.0e6 * (1.0 + 0.015 * sin(2.0 * 3.14159265358979 * 0.25 * t));
    const double v = dc * (1.0 - 0.01 * pulse(tr, t)) * scale_red + noise();
    return (int32_t)(v * r);
}

typedef struct {
    int beats_after;       /* valid intervals emitted in the window      */
    int true_after;        /* true beats in the window                   */
    int bad_after;         /* emitted intervals > 40 ms off the truth    */
    int gated_after;       /* samples gated in the window                */
} run_res_t;

/* One run: comp = use nc_agcstep (else the pre-v0.3.0-35 path: plain
 * nc_dsp_run + 200 ms agc_settling gate). Window = [t_step, t_step + 12 s]. */
static run_res_t run(double r, bool comp)
{
    nc_knobs_init();
    truth_t tr;
    make_truth(&tr);
    g_lcg = 12345u;

    nc_chan_dsp_t ch[2];
    nc_dsp_init(&ch[0], NC_RATE_100);
    nc_dsp_init(&ch[1], NC_RATE_100);
    nc_ibi_t ibi;
    nc_ibi_init(&ibi, TS_US);
    nc_gate_t gate;
    nc_gate_init(&gate);
    nc_gate_dcstep_t ds;
    nc_gate_dcstep_init(&ds);
    nc_agcstep_t st;
    nc_agcstep_init(&st);

    run_res_t res = { 0, 0, 0, 0 };
    const uint64_t win_end = T_STEP_US + 12ULL * 1000000ULL;
    for (int k = 0; k < tr.n_true; k++) {
        const uint64_t tk = (uint64_t)(tr.t_true[k] * 1e6);
        if (tk >= T_STEP_US && tk < win_end) res.true_after++;
    }

    bool armed = false;
    for (uint64_t t_us = TS_US; t_us < (uint64_t)DUR_S * 1000000ULL; t_us += TS_US) {
        const double t = (double)t_us / 1e6;
        const double rr = (t_us >= T_STEP_US) ? r : 1.0;
        const int32_t x[2] = { raw_at(&tr, t, 1.0, rr), raw_at(&tr, t, 0.8, rr) };

        nc_dsp_out_t o[2];
        bool settling = false;
        if (comp) {
            if (!armed && t_us + TS_US >= T_STEP_US) {
                /* sys_task: the writes land just before the step sample */
                const int32_t rq = (int32_t)lround(r * 65536.0);
                const int32_t rv[2] = { rq, rq };
                nc_agcstep_guard(&st, T_STEP_US - 2000);
                nc_agcstep_arm(&st, T_STEP_US - 2000, T_STEP_US - 1500, rv);
                armed = true;
            }
            nc_agcstep_run(&st, ch, &ibi, 0, &ds, t_us, x, o);
        } else {
            nc_dsp_run(&ch[0], x[0], &o[0]);
            nc_dsp_run(&ch[1], x[1], &o[1]);
            settling = t_us >= T_STEP_US && t_us < T_STEP_US + 200000ULL;
        }

        nc_gate_in_t gin;
        nc_gate_build_input(&gin, &ds, 0, o[0].dc, false, false, settling, false);
        const bool gated = nc_gate_update(&gate, &gin, (uint32_t)(t_us / 1000u), NULL);
        if (gated && t_us >= T_STEP_US && t_us < win_end) res.gated_after++;

        nc_ibi_rec_t rec;
        if (nc_ibi_update(&ibi, o[0].bp, t_us, gated, &rec) &&
            rec.t_beat_us >= T_STEP_US && rec.t_beat_us < win_end && rec.ibi_ms > 0) {
            res.beats_after++;
            /* the true interval ending nearest this beat (detector latency
             * is constant, so intervals compare directly) */
            double best = 1e9;
            for (int k = 1; k < tr.n_true; k++) {
                const double ti = (tr.t_true[k] - tr.t_true[k - 1]) * 1000.0;
                const double dt = fabs((double)rec.t_beat_us / 1e6 - tr.t_true[k]);
                if (dt < 0.6 && fabs(ti - rec.ibi_ms) < best) best = fabs(ti - rec.ibi_ms);
            }
            if (best > 40.0) res.bad_after++;
        }
    }
    return res;
}

/* ------------------------------------------------------------------ */

static void test_ratio_helpers(void)
{
    CHECK_EQ(nc_agcstep_ratio_q16(20, 25), 52428);        /* 0.8 */
    CHECK_EQ(nc_agcstep_ratio_q16(50, 25), 131072);       /* 2.0 */
    CHECK_EQ(nc_agcstep_ratio_q16(0, 25), 0);             /* LEDs off */
    CHECK_EQ(nc_agcstep_ratio_q16(25, 0), 0);             /* from off */
    CHECK_EQ(nc_agcstep_ratio_q16(1000, 10), 0);          /* x100: out */
    CHECK_EQ(nc_agcstep_chain_q16(131072, 32768), 65536); /* 2 x 0.5 */
    CHECK_EQ(nc_agcstep_chain_q16(0, 65536), 0);
    CHECK_EQ(nc_agcstep_chain_q16(524288, 131072), 0);    /* 16: out */
}

static void test_sequencing(void)
{
    nc_knobs_init();
    nc_chan_dsp_t ch[2];
    nc_dsp_init(&ch[0], NC_RATE_100);
    nc_dsp_init(&ch[1], NC_RATE_100);
    nc_agcstep_t st;
    nc_agcstep_init(&st);
    nc_dsp_out_t o[2];
    const int32_t x[2] = { 1000000, 800000 };
    uint64_t t = 0;
    for (int i = 0; i < 5; i++) {
        t += TS_US;
        CHECK_EQ(nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o), NC_AGCSTEP_PASS);
    }
    /* writes between this sample and the next */
    const int32_t r[2] = { 52428, 65536 };
    nc_agcstep_guard(&st, t + 100);
    t += TS_US;                                       /* guard: held */
    CHECK_EQ(nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o), NC_AGCSTEP_HOLD);
    nc_agcstep_arm(&st, t - TS_US + 100, t - TS_US + 600, r);
    t += TS_US;                                       /* first after t_hi */
    CHECK_EQ(nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o), NC_AGCSTEP_HOLD);
    t += TS_US;
    CHECK_EQ(nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o), NC_AGCSTEP_APPLY);
    t += TS_US;
    CHECK_EQ(nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o), NC_AGCSTEP_PASS);

    /* a guard whose arm never comes is dropped after the cap */
    nc_agcstep_guard(&st, t);
    int held = 0;
    for (int i = 0; i < 20; i++) {
        t += TS_US;
        if (nc_agcstep_run(&st, ch, NULL, 0, NULL, t, x, o) == NC_AGCSTEP_HOLD) held++;
    }
    CHECK_EQ(held, NC_AGCSTEP_HOLD_MAX);
    CHECK(!st.guard && !st.armed);

    /* two arms before the switch merge: ratios multiply */
    const int32_t r2[2] = { 131072, 131072 };
    nc_agcstep_arm(&st, t + 10, t + 20, r);
    nc_agcstep_arm(&st, t + 30, t + 40, r2);
    CHECK_EQ(st.r_q16[0], (int32_t)(((int64_t)52428 * 131072) >> 16));
    CHECK_EQ(st.r_q16[1], 131072);
    CHECK_EQ(st.t_lo_us, t + 10);
    CHECK_EQ(st.t_hi_us, t + 40);
}

/* A compensated step leaves no transient: hp and bp continue smoothly. */
static void test_no_transient(void)
{
    nc_knobs_init();
    nc_chan_dsp_t a[2], b[2];
    nc_dsp_init(&a[0], NC_RATE_100); nc_dsp_init(&a[1], NC_RATE_100);
    nc_dsp_init(&b[0], NC_RATE_100); nc_dsp_init(&b[1], NC_RATE_100);
    nc_agcstep_t st;
    nc_agcstep_init(&st);
    truth_t tr;
    make_truth(&tr);
    g_lcg = 1u;
    int32_t peak_comp = 0, peak_raw = 0, peak_ref = 0;
    bool armed = false;
    for (uint64_t t_us = TS_US; t_us < 30ULL * 1000000ULL; t_us += TS_US) {
        const double t = (double)t_us / 1e6;
        const double rr = (t_us >= T_STEP_US) ? 0.6 : 1.0;
        const int32_t x[2] = { raw_at(&tr, t, 1.0, rr), raw_at(&tr, t, 0.8, rr) };
        nc_dsp_out_t o[2], q[2];
        if (!armed && t_us + TS_US >= T_STEP_US) {
            const int32_t rv[2] = { 39322, 39322 };      /* 0.6 */
            nc_agcstep_arm(&st, T_STEP_US - 2000, T_STEP_US - 1500, rv);
            armed = true;
        }
        nc_agcstep_run(&st, a, NULL, 0, NULL, t_us, x, o);
        nc_dsp_run(&b[0], x[0], &q[0]);
        nc_dsp_run(&b[1], x[1], &q[1]);
        const int32_t pc = o[0].bp < 0 ? -o[0].bp : o[0].bp;
        const int32_t pr = q[0].bp < 0 ? -q[0].bp : q[0].bp;
        if (t_us >= 10ULL * 1000000ULL && t_us < T_STEP_US) {
            if (pc > peak_ref) peak_ref = pc;            /* before: the pulse */
        } else if (t_us >= T_STEP_US && t_us < T_STEP_US + 3000000ULL) {
            if (pc > peak_comp) peak_comp = pc;
            if (pr > peak_raw) peak_raw = pr;
        }
    }
    printf("  bp peak: pulse %d, after step comp %d, uncompensated %d\n",
           (int)peak_ref, (int)peak_comp, (int)peak_raw);
    CHECK(peak_comp <= peak_ref);                     /* no bigger than a pulse */
    CHECK(peak_raw > 10 * peak_ref);                  /* the old transient */
}

/* A compensated step must look exactly like the same signal with no step
 * at all. (The synthetic breathing also trips the DC-step detector now
 * and then — the baseline counts that, it is not the step's doing.) */
static void check_continuity(double r)
{
    const run_res_t b = run(1.0, true);
    const run_res_t c = run(r, true);
    const run_res_t u = run(r, false);
    printf("  r=%.2f: %d true beats in 12 s; no step %d (bad %d, gated %d smp); "
           "compensated %d (bad %d, gated %d); uncompensated %d (bad %d, gated %d)\n",
           r, c.true_after, b.beats_after, b.bad_after, b.gated_after,
           c.beats_after, c.bad_after, c.gated_after,
           u.beats_after, u.bad_after, u.gated_after);
    CHECK_EQ(c.beats_after, b.beats_after);           /* as if no step happened */
    CHECK(c.bad_after <= b.bad_after);
    CHECK(c.gated_after <= b.gated_after + 3);        /* at most the HOLD samples */
    CHECK(u.beats_after < c.beats_after);             /* the problem it fixes */
}

static void test_beats_continue_led_down(void) { check_continuity(0.7); }
static void test_beats_continue_gain_down(void) { check_continuity(0.5); }
static void test_beats_continue_gain_up(void) { check_continuity(2.0); }
static void test_beats_continue_led_up(void) { check_continuity(1.3); }

int main(void)
{
    TST_RUN(test_ratio_helpers);
    TST_RUN(test_sequencing);
    TST_RUN(test_no_transient);
    TST_RUN(test_beats_continue_led_down);
    TST_RUN(test_beats_continue_gain_down);
    TST_RUN(test_beats_continue_gain_up);
    TST_RUN(test_beats_continue_led_up);
    TST_REPORT();
}
