/*
 * t_dsp.c — bit-exact verification of the fixed-point DSP chain
 * (nc_dsp.c) against golden vectors from tools/goldens/gen_dsp_vectors.py.
 *
 * The cfgs[] table below (names / rate masks / notch knob settings)
 * MUST match the VECTORS table in gen_dsp_vectors.py — the generator
 * and this test are two halves of one contract.
 *
 * Run from test_host/ (run_tests.sh does): vector paths are relative.
 */
#include <stdio.h>
#include "tst.h"
#include "narbis/nc_dsp.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_dsp");

/* Largest committed vector: 5000 samples (250 sps x 20 s / 500 x 10 s). */
#define MAX_N 5000

static int32_t vin[MAX_N];
static int32_t vexp[3 * MAX_N];

/* Returns element count, -1 if unopenable, -2 if the file was larger
 * than maxn (truncated read would silently weaken the test). */
static long load_i32(const char *path, int32_t *dst, long maxn)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    long n = (long)fread(dst, sizeof(int32_t), (size_t)maxn, f);
    int over = fgetc(f);
    fclose(f);
    return (over == EOF) ? n : -2;
}

/* ------------------------------------------------------------------ */
/* nc_bq_run unit checks                                               */
/* ------------------------------------------------------------------ */

static void test_bq_hand_case(void)
{
    /* Hand-computed DF2T trace. Coefficients (Q30): b0=1.0, b1=0.5,
     * b2=0, a1=-0.5, a2=0. Input x = {2^20, 0, 0}:
     *   n=0: y64 = 2^30*2^20         = 2^50 -> y = 2^20
     *        s1  = 2^29*2^20 - (-2^29)*2^20 = 2^49 + 2^49 = 2^50
     *        s2  = 0
     *   n=1: y64 = 0 + 2^50                 -> y = 2^20
     *        s1  = 0 - (-2^29)*2^20 + 0     = 2^49
     *   n=2: y64 = 2^49                     -> y = 2^19            */
    const nc_bq_coeff_t c = { 1 << 30, 1 << 29, 0, -(1 << 29), 0 };
    nc_bq_state_t s = { 0, 0 };
    CHECK_EQ(nc_bq_run(&c, &s, 1 << 20), 1 << 20);
    CHECK_EQ(s.s1, (int64_t)1 << 50);
    CHECK_EQ(s.s2, 0);
    CHECK_EQ(nc_bq_run(&c, &s, 0), 1 << 20);
    CHECK_EQ(s.s1, (int64_t)1 << 49);
    CHECK_EQ(nc_bq_run(&c, &s, 0), 1 << 19);
}

static void test_bq_truncation_direction(void)
{
    /* The >>30 is an ARITHMETIC shift with no rounding constant:
     * y64 = -1 must give -1 (floor), not 0 (truncate-toward-zero).
     * This is the exact property the python replica relies on. */
    const nc_bq_coeff_t c = { 1, 0, 0, 0, 0 }; /* b0 = 2^-30 */
    nc_bq_state_t s = { 0, 0 };
    CHECK_EQ(nc_bq_run(&c, &s, -1), -1);
    s.s1 = 0; s.s2 = 0;
    CHECK_EQ(nc_bq_run(&c, &s, 1), 0);
}

static void test_bq_saturation(void)
{
    /* Preloaded state drives y64>>30 past int32; y must saturate and
     * the SATURATED y must feed the state update (a1 = 1.0 Q30). */
    const nc_bq_coeff_t c = { 0, 0, 0, 1 << 30, 0 };
    nc_bq_state_t s = { (int64_t)1 << 62, 0 };
    CHECK_EQ(nc_bq_run(&c, &s, 0), INT32_MAX);
    CHECK_EQ(s.s1, -((int64_t)1 << 30) * INT32_MAX);
    s.s1 = -((int64_t)1 << 62); s.s2 = 0;
    CHECK_EQ(nc_bq_run(&c, &s, 0), INT32_MIN);
}

/* ------------------------------------------------------------------ */
/* nc_dsp_init notch policy                                            */
/* ------------------------------------------------------------------ */

static void set_notch_knobs(int en, int hz)
{
    CHECK_EQ(nc_knob_set_id(0x0504, en), NC_ST_OK); /* notch_en */
    CHECK_EQ(nc_knob_set_id(0x0505, hz), NC_ST_OK); /* notch_hz */
}

static void test_init_notch_policy(void)
{
    nc_chan_dsp_t d;

    set_notch_knobs(1, 50);
    nc_dsp_init(&d, NC_RATE_100);   /* 50 Hz == Nyquist at 100 sps */
    CHECK(!d.notch_en);
    nc_dsp_init(&d, NC_RATE_50);
    CHECK(!d.notch_en);
    nc_dsp_init(&d, NC_RATE_200);
    CHECK(d.notch_en);
    nc_dsp_init(&d, NC_RATE_500);
    CHECK(d.notch_en);

    set_notch_knobs(0, 60);
    nc_dsp_init(&d, NC_RATE_500);
    CHECK(!d.notch_en);

    /* hz >= 55 selects the 60 Hz design */
    nc_chan_dsp_t d55, d60;
    set_notch_knobs(1, 55);
    nc_dsp_init(&d55, NC_RATE_200);
    set_notch_knobs(1, 60);
    nc_dsp_init(&d60, NC_RATE_200);
    CHECK(d55.notch_en && d60.notch_en);
    CHECK_MEMEQ(&d55.bq_c[2], &d60.bq_c[2], sizeof(nc_bq_coeff_t));

    /* Out-of-range rate falls back to the 100 sps product default */
    nc_chan_dsp_t dbad, d100;
    set_notch_knobs(0, 50);
    nc_dsp_init(&dbad, (nc_rate_t)NC_RATE_COUNT);
    nc_dsp_init(&d100, NC_RATE_100);
    CHECK_EQ(dbad.alpha_q31, d100.alpha_q31);
    CHECK_MEMEQ(dbad.bq_c, d100.bq_c, sizeof(d100.bq_c));

    /* Fresh init zeroes all state */
    CHECK_EQ(d100.dc_acc, 0);
    CHECK_EQ(d100.dc, 0);
    CHECK_EQ(d100.bq_s[0].s1 | d100.bq_s[1].s1 | d100.bq_s[2].s1, 0);
}

/* ------------------------------------------------------------------ */
/* Golden vectors — bit-exact, sample by sample                        */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    uint8_t rate_mask;   /* bit r == nc_rate_t r */
    uint8_t notch_en;
    uint8_t notch_hz;
} vec_cfg_t;

/* MUST match VECTORS in gen_dsp_vectors.py */
static const vec_cfg_t cfgs[] = {
    { "impulse", 0x1F, 0, 60 },
    { "step",    0x1F, 0, 50 },
    { "twotone", 0x1F, 1, 50 },
    { "chirp",   0x1F, 1, 60 },
    { "noise",   0x1F, 1, 50 },
    { "mains50", 0x02, 1, 50 },
};

static void run_vector(const vec_cfg_t *cfg, int rate)
{
    char pin[64], pexp[64];
    snprintf(pin, sizeof(pin), "vectors/dsp_%s_r%d_in.bin", cfg->name, rate);
    snprintf(pexp, sizeof(pexp), "vectors/dsp_%s_r%d_exp.bin", cfg->name, rate);

    long n = load_i32(pin, vin, MAX_N);
    long ne = load_i32(pexp, vexp, 3 * MAX_N);
    if (n <= 0 || ne != 3 * n) {
        fprintf(stderr, "  %s: load n=%ld ne=%ld\n", pin, n, ne);
        CHECK(n > 0 && ne == 3 * n);
        return;
    }

    set_notch_knobs(cfg->notch_en, cfg->notch_hz);
    nc_chan_dsp_t d;
    nc_dsp_init(&d, (nc_rate_t)rate);

    long mism = -1;
    nc_dsp_out_t o = { 0, 0, 0 };
    for (long i = 0; i < n; i++) {
        nc_dsp_run(&d, vin[i], &o);
        if (o.dc != vexp[3 * i] || o.hp != vexp[3 * i + 1] ||
            o.bp != vexp[3 * i + 2]) {
            mism = i;
            break;
        }
    }
    if (mism >= 0) {
        fprintf(stderr,
                "  %s r%d: first mismatch at %ld: "
                "dc %ld/%ld hp %ld/%ld bp %ld/%ld (got/want)\n",
                cfg->name, rate, mism,
                (long)o.dc, (long)vexp[3 * mism],
                (long)o.hp, (long)vexp[3 * mism + 1],
                (long)o.bp, (long)vexp[3 * mism + 2]);
    }
    CHECK(mism == -1);
}

static void test_golden_vectors(void)
{
    for (size_t v = 0; v < sizeof(cfgs) / sizeof(cfgs[0]); v++) {
        for (int r = 0; r < NC_RATE_COUNT; r++) {
            if ((cfgs[v].rate_mask >> r) & 1) {
                run_vector(&cfgs[v], r);
            }
        }
    }
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(test_bq_hand_case);
    TST_RUN(test_bq_truncation_direction);
    TST_RUN(test_bq_saturation);
    TST_RUN(test_init_notch_policy);
    TST_RUN(test_golden_vectors);
    TST_REPORT();
}
