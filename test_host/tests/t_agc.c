/*
 * t_agc.c — AGC policy engine against a simulated optical plant.
 *
 * Plant model (per task spec):
 *   dc[c] = k[c] * led_ma[c] * rf_kohm(rf)/100 + offdac[c] * dacg[c]
 * with the offset-DAC gain dacg carrying a ±20% error vs the 10000
 * counts/code nominal (8000 / 12000 in the tests below) — the policy
 * steps blindly, so convergence must not depend on the exact value.
 * sat asserts when the ideal (unclipped) dc reaches 95% of 2^21;
 * dc_raw clips at full scale like the real ADC.
 */
#include <stdio.h>
#include "tst.h"
#include "narbis/nc_agc.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_agc");

#define FS       NC_AGC_FS_COUNTS          /* 2097152 */
#define SAT_LEV  1992294                   /* 95% of FS */
/* Default band: target 50% +- 15% of FS (integer math as the module) */
#define BAND_LO  734004
#define BAND_HI  1363148
#define HOLD_MS  2000

/* code -> Rf in kOhm (gain factor = kohm/100, i.e. 100k = x1.0) */
static const int32_t rf_kohm[8] = { 500, 250, 100, 50, 25, 10, 1000, 2000 };

typedef struct {
    int32_t  k[2];       /* counts per (mA at 100k gain) */
    int32_t  dacg[2];    /* actual counts per offdac code */
    uint8_t  led[2];
    int8_t   od[2];
    uint8_t  rf;
    uint64_t now;
    bool     frozen;
    int32_t  noise[2];   /* additive dc noise for this read */
} plant_t;

static int64_t plant_ideal(const plant_t *p, int c)
{
    return ((int64_t)p->k[c] * p->led[c] * rf_kohm[p->rf]) / 100 +
           (int64_t)p->od[c] * p->dacg[c];
}

static void plant_read(const plant_t *p, nc_agc_in_t *in)
{
    for (int c = 0; c < 2; c++) {
        int64_t v = plant_ideal(p, c) + p->noise[c];
        in->sat[c] = v >= SAT_LEV;
        if (v > FS - 1) v = FS - 1;
        if (v < -FS) v = -FS;
        in->dc_raw[c] = (int32_t)v;
        in->led_ma[c] = p->led[c];
        in->offdac[c] = p->od[c];
    }
    in->rf_code = p->rf;
    in->now_ms = p->now;
    in->frozen = p->frozen;
}

static bool in_band(const plant_t *p, int c)
{
    int64_t v = plant_ideal(p, c);
    return v >= BAND_LO && v <= BAND_HI;
}

/* Applies actions; every emitted value is contract-checked here so all
 * scenario tests share the clamp assertions. Returns #gain actions. */
static int apply(plant_t *p, const nc_agc_act_t *a, int n)
{
    int gains = 0;
    int32_t maxma[2] = { nc_knob_get(KNOB_AGC_MAX_MA_IR),
                         nc_knob_get(KNOB_AGC_MAX_MA_RED) };
    if (maxma[0] > NC_AGC_HARD_MAX_IR_MA) maxma[0] = NC_AGC_HARD_MAX_IR_MA;
    if (maxma[1] > NC_AGC_HARD_MAX_RED_MA) maxma[1] = NC_AGC_HARD_MAX_RED_MA;
    int32_t minma[2] = { nc_knob_get(KNOB_AGC_MIN_MA_IR),
                         nc_knob_get(KNOB_AGC_MIN_MA_RED) };

    for (int i = 0; i < n; i++) {
        switch (a[i].kind) {
        case NC_AGC_LED:
            CHECK(a[i].chan <= 1);
            CHECK(a[i].led_ma >= minma[a[i].chan]);
            CHECK(a[i].led_ma <= maxma[a[i].chan]);
            CHECK(a[i].led_ma <= (a[i].chan ? NC_AGC_HARD_MAX_RED_MA
                                            : NC_AGC_HARD_MAX_IR_MA));
            p->led[a[i].chan] = a[i].led_ma;
            break;
        case NC_AGC_OFFDAC:
            CHECK(a[i].chan <= 1);
            CHECK(a[i].offdac >= NC_AGC_OFFDAC_MIN);
            CHECK(a[i].offdac <= NC_AGC_OFFDAC_MAX);
            p->od[a[i].chan] = a[i].offdac;
            break;
        case NC_AGC_GAIN:
            CHECK(a[i].rf_code <= 7);
            p->rf = a[i].rf_code;
            gains++;
            break;
        default:
            CHECK(0 && "unexpected NC_AGC_NONE");
            break;
        }
    }
    CHECK(gains <= 1);   /* at most one GAIN per evaluate */
    return gains;
}

/* One evaluate at the current plant time, then advance one hold. */
static int drive(nc_agc_t *s, plant_t *p, nc_agc_act_t acts[4])
{
    nc_agc_in_t in;
    plant_read(p, &in);
    int n = nc_agc_evaluate(s, &in, acts);
    CHECK(n >= 0 && n <= 4);
    apply(p, acts, n);
    p->now += HOLD_MS;
    return n;
}

/* ------------------------------------------------------------------ */

static void test_rf_order(void)
{
    /* Value order codes: [5,4,3,2,1,0,6,7] */
    static const uint8_t order[8] = { 5, 4, 3, 2, 1, 0, 6, 7 };
    for (int r = 0; r < 8; r++) {
        CHECK_EQ(nc_agc_rf_code_at(r), order[r]);
        CHECK_EQ(nc_agc_rf_rank(order[r]), r);
    }
    CHECK_EQ(nc_agc_rf_rank(8), -1);
    CHECK_EQ(nc_agc_rf_code_at(-3), 5);
    CHECK_EQ(nc_agc_rf_code_at(99), 7);
}

static void test_converge_from_dark(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    plant_t p = { .k = { 150000, 200000 }, .dacg = { 10000, 10000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    int conv = -1;
    for (int i = 0; i < 10; i++) {
        drive(&s, &p, acts);
        if (conv < 0 && in_band(&p, 0) && in_band(&p, 1)) conv = i;
    }
    CHECK(conv >= 0 && conv < 10);
    CHECK(in_band(&p, 0) && in_band(&p, 1));
    CHECK_EQ(p.led[0], 5);   /* 150000*5 = 750000 */
    CHECK_EQ(p.led[1], 4);   /* 200000*4 = 800000 */
    /* Converged and static: no further actions. */
    CHECK_EQ(drive(&s, &p, acts), 0);
}

static void test_converge_from_saturated(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    plant_t p = { .k = { 150000, 150000 }, .dacg = { 10000, 10000 },
                  .led = { 14, 12 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    nc_agc_in_t in;
    plant_read(&p, &in);
    CHECK(in.sat[0]);            /* 14*150000 = 2.1M >= 95% FS */

    int conv = -1;
    for (int i = 0; i < 10; i++) {
        drive(&s, &p, acts);
        if (conv < 0 && in_band(&p, 0) && in_band(&p, 1)) conv = i;
    }
    CHECK(conv >= 0 && conv < 10);
    CHECK_EQ(p.led[0], 9);       /* 1.35M, in band */
    CHECK_EQ(p.led[1], 9);
}

static void test_no_oscillation_1000(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    /* Converged start (from test_converge_from_dark end state). */
    plant_t p = { .k = { 150000, 200000 }, .dacg = { 10000, 10000 },
                  .led = { 5, 4 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];
    uint32_t rng = 0x1234567u;

    int last_dir[2] = { 0, 0 }, rev[2] = { 0, 0 };
    bool banded = false;

    for (int i = 0; i < 1000; i++) {
        /* Slow triangular drift on IR coupling: up to +130000 at the
         * apex (dc crosses the band top going up, the bottom coming
         * down) + white dc noise well inside the deadband. */
        p.k[0] = 150000 + ((i < 500) ? i : (999 - i)) * 260;
        for (int c = 0; c < 2; c++) {
            rng = rng * 1664525u + 1013904223u;
            p.noise[c] = (int32_t)((rng >> 16) % 20001u) - 10000;
        }

        nc_agc_in_t in;
        plant_read(&p, &in);
        int n = nc_agc_evaluate(&s, &in, acts);
        apply(&p, acts, n);
        p.now += HOLD_MS;

        if (!banded && in_band(&p, 0) && in_band(&p, 1)) banded = true;
        if (banded) {
            for (int j = 0; j < n; j++) {
                if (acts[j].kind != NC_AGC_LED) continue;
                int c = acts[j].chan;
                int dir = (acts[j].led_ma > in.led_ma[c]) ? 1 : -1;
                if (last_dir[c] != 0 && dir != last_dir[c]) rev[c]++;
                last_dir[c] = dir;
            }
        }
    }
    CHECK(banded);
    CHECK(rev[0] <= 2);
    CHECK(rev[1] <= 2);
}

static void test_hold_honored(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    plant_t p = { .k = { 150000, 150000 }, .dacg = { 10000, 10000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_in_t in;
    nc_agc_act_t acts[4];

    plant_read(&p, &in);
    in.now_ms = 0;
    int n = nc_agc_evaluate(&s, &in, acts);
    CHECK(n > 0);                                 /* first call acts */
    apply(&p, acts, n);

    plant_read(&p, &in);                          /* still out of band */
    in.now_ms = 500;
    CHECK_EQ(nc_agc_evaluate(&s, &in, acts), 0);
    in.now_ms = 1999;
    CHECK_EQ(nc_agc_evaluate(&s, &in, acts), 0);
    in.now_ms = 2000;                             /* hold expired */
    CHECK(nc_agc_evaluate(&s, &in, acts) > 0);
}

static void test_frozen_and_disabled(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    plant_t p = { .k = { 150000, 150000 }, .dacg = { 10000, 10000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = true, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    for (int i = 0; i < 5; i++) CHECK_EQ(drive(&s, &p, acts), 0);

    p.frozen = false;
    CHECK_EQ(nc_knob_set_id(0x0401, 0), NC_ST_OK);   /* agc_en off */
    for (int i = 0; i < 5; i++) CHECK_EQ(drive(&s, &p, acts), 0);

    CHECK_EQ(nc_knob_set_id(0x0401, 1), NC_ST_OK);
    CHECK(drive(&s, &p, acts) > 0);
}

static void test_offdac_down_path(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    /* IR: at min LED already, dc = 1.5M > band top but NOT saturated;
     * dacg = 12000 (+20% error) -> in band at code -12. RED in band. */
    plant_t p = { .k = { 750000, 200000 }, .dacg = { 12000, 12000 },
                  .led = { 2, 5 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    int8_t expect = 0;
    for (int i = 0; i < 12; i++) {
        nc_agc_in_t in;
        plant_read(&p, &in);
        int n = nc_agc_evaluate(&s, &in, acts);
        CHECK_EQ(n, 1);
        CHECK_EQ(acts[0].kind, NC_AGC_OFFDAC);
        CHECK_EQ(acts[0].chan, NC_AGC_CH_IR);
        expect--;
        CHECK_EQ(acts[0].offdac, expect);          /* one code per step */
        apply(&p, acts, n);
        p.now += HOLD_MS;
    }
    CHECK_EQ(p.od[0], -12);
    CHECK(in_band(&p, 0));
    CHECK_EQ(drive(&s, &p, acts), 0);              /* settled */
}

static void test_gain_down_ladder(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    /* Both channels blast past saturation even at min LED; offdac too
     * weak to fix it -> expect offdac exhaustion then gain-down steps
     * following VALUE order 100k -> 50k -> 25k -> 10k = codes 3,4,5,
     * each paired with recenter suggestions for both channels. */
    plant_t p = { .k = { 5000000, 5000000 }, .dacg = { 12000, 12000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    uint8_t gain_seq[8];
    int ngain = 0;
    for (int i = 0; i < 60; i++) {
        nc_agc_in_t in;
        plant_read(&p, &in);
        int n = nc_agc_evaluate(&s, &in, acts);
        if (n > 0 && acts[0].kind == NC_AGC_GAIN) {
            CHECK(ngain < 8);
            gain_seq[ngain++] = acts[0].rf_code;
            /* offdac was at -15 on both channels -> two recenters */
            CHECK_EQ(n, 3);
            CHECK_EQ(acts[1].kind, NC_AGC_OFFDAC);
            CHECK_EQ(acts[1].offdac, 0);
            CHECK_EQ(acts[2].kind, NC_AGC_OFFDAC);
            CHECK_EQ(acts[2].offdac, 0);
            CHECK(acts[1].chan != acts[2].chan);
        }
        apply(&p, acts, n);
        p.now += HOLD_MS;
    }
    CHECK_EQ(ngain, 3);
    CHECK_EQ(gain_seq[0], 3);
    CHECK_EQ(gain_seq[1], 4);
    CHECK_EQ(gain_seq[2], 5);
    CHECK_EQ(p.rf, 5);                     /* 10k: dc = 1M, in band */
    CHECK_EQ(p.od[0], 0);
    CHECK_EQ(p.od[1], 0);
    CHECK(in_band(&p, 0) && in_band(&p, 1));
    CHECK_EQ(drive(&s, &p, acts), 0);
}

static void test_gain_up_ladder_and_clamps(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    /* Starved plant: even at max LED + full gain + offdac the dc never
     * reaches the band. Expect: LED ramp to the hard-capped maxima
     * (50 IR / 40 RED), offdac to +15, then gain-up in VALUE order
     * 100k -> 250k -> 500k -> 1M -> 2M = codes 1, 0, 6, 7, then park
     * with zero actions at the rail. dacg = 8000 (-20% error). */
    plant_t p = { .k = { 100, 100 }, .dacg = { 8000, 8000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    uint8_t gain_seq[8];
    int ngain = 0;
    for (int i = 0; i < 140; i++) {
        nc_agc_in_t in;
        plant_read(&p, &in);
        int n = nc_agc_evaluate(&s, &in, acts);
        for (int j = 0; j < n; j++) {
            if (acts[j].kind == NC_AGC_GAIN) {
                CHECK(ngain < 8);
                gain_seq[ngain++] = acts[j].rf_code;
            }
        }
        apply(&p, acts, n);       /* asserts every led_ma / offdac / rf */
        p.now += HOLD_MS;
    }
    CHECK_EQ(ngain, 4);
    CHECK_EQ(gain_seq[0], 1);
    CHECK_EQ(gain_seq[1], 0);
    CHECK_EQ(gain_seq[2], 6);
    CHECK_EQ(gain_seq[3], 7);
    CHECK_EQ(p.led[0], NC_AGC_HARD_MAX_IR_MA);
    CHECK_EQ(p.led[1], NC_AGC_HARD_MAX_RED_MA);
    CHECK_EQ(p.od[0], NC_AGC_OFFDAC_MAX);
    CHECK_EQ(p.od[1], NC_AGC_OFFDAC_MAX);
    CHECK_EQ(p.rf, 7);
    /* Everything railed: the loop must go quiet, not spin. */
    for (int i = 0; i < 5; i++) CHECK_EQ(drive(&s, &p, acts), 0);
}

static void test_gain_up_requires_both(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_init(&s);
    /* IR starved, RED healthy: gain-up needs BOTH channels to agree,
     * so no GAIN action may ever fire. */
    plant_t p = { .k = { 100, 200000 }, .dacg = { 10000, 10000 },
                  .led = { 2, 4 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];

    for (int i = 0; i < 80; i++) {
        nc_agc_in_t in;
        plant_read(&p, &in);
        int n = nc_agc_evaluate(&s, &in, acts);
        for (int j = 0; j < n; j++) CHECK(acts[j].kind != NC_AGC_GAIN);
        apply(&p, acts, n);
        p.now += HOLD_MS;
    }
    CHECK_EQ(p.rf, 2);
    CHECK_EQ(p.led[0], NC_AGC_HARD_MAX_IR_MA);
    CHECK_EQ(p.od[0], NC_AGC_OFFDAC_MAX);
    CHECK(in_band(&p, 1));
    CHECK_EQ(drive(&s, &p, acts), 0);      /* parked, not spinning */
}

static void test_knob_max_respected(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0407, 45), NC_ST_OK);  /* agc_max_ma_ir */
    nc_agc_t s;
    nc_agc_init(&s);
    plant_t p = { .k = { 100, 100 }, .dacg = { 10000, 10000 },
                  .led = { 2, 2 }, .od = { 0, 0 }, .rf = 2,
                  .now = 0, .frozen = false, .noise = { 0, 0 } };
    nc_agc_act_t acts[4];
    for (int i = 0; i < 60; i++) drive(&s, &p, acts);  /* apply asserts */
    CHECK_EQ(p.led[0], 45);
    nc_knobs_init();
}

static void test_rails_and_bad_rf(void)
{
    nc_knobs_init();
    nc_agc_t s;
    nc_agc_act_t acts[4];

    /* Gain already at the bottom (10k, rank 0), both channels
     * saturated with LED at min and offdac exhausted: nothing left. */
    nc_agc_in_t in = {
        .dc_raw = { FS - 1, FS - 1 }, .sat = { true, true },
        .led_ma = { 2, 2 }, .rf_code = 5, .offdac = { -15, -15 },
        .now_ms = 0, .frozen = false
    };
    nc_agc_init(&s);
    CHECK_EQ(nc_agc_evaluate(&s, &in, acts), 0);

    /* Corrupt rf code: no gain action and no crash. */
    in.rf_code = 9;
    nc_agc_init(&s);
    CHECK_EQ(nc_agc_evaluate(&s, &in, acts), 0);

    /* offdac disabled counts as exhausted: gain-down fires directly,
     * without recenter suggestions (offdac not in use). */
    CHECK_EQ(nc_knob_set_id(0x040C, 0), NC_ST_OK);   /* agc_offdac_en */
    in.rf_code = 2;
    in.offdac[0] = 0;
    in.offdac[1] = 0;
    nc_agc_init(&s);
    int n = nc_agc_evaluate(&s, &in, acts);
    CHECK_EQ(n, 1);
    CHECK_EQ(acts[0].kind, NC_AGC_GAIN);
    CHECK_EQ(acts[0].rf_code, 3);                    /* 100k -> 50k */
    nc_knobs_init();
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(test_rf_order);
    TST_RUN(test_converge_from_dark);
    TST_RUN(test_converge_from_saturated);
    TST_RUN(test_no_oscillation_1000);
    TST_RUN(test_hold_honored);
    TST_RUN(test_frozen_and_disabled);
    TST_RUN(test_offdac_down_path);
    TST_RUN(test_gain_down_ladder);
    TST_RUN(test_gain_up_ladder_and_clamps);
    TST_RUN(test_gain_up_requires_both);
    TST_RUN(test_knob_max_respected);
    TST_RUN(test_rails_and_bad_rf);
    TST_REPORT();
}
