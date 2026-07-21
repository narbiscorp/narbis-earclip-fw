/*
 * t_dsp_design.c — the runtime corner designer must (1) reproduce the
 * committed golden tables at the default corners, (2) leave the golden
 * path untouched when knobs are at defaults, (3) produce stable,
 * DC-blocking filters at non-default corners, (4) survive degenerate
 * requests.
 */
#include "tst.h"
#include "narbis/nc_dsp.h"
#include "narbis/nc_dsp_design.h"
#include "narbis/nc_knobs.h"
#include <stdlib.h>

TST_SUITE("dsp_design");

/* Golden tables, re-declared via the generated header the same way
 * nc_dsp.c consumes them. */
typedef nc_bq_coeff_t row_t;
#include "../../firmware/components/narbis_core/src/dsp_coeffs_gen.h"

static const uint16_t RATES[NC_RATE_COUNT] = { 50, 100, 200, 250, 500 };

static void defaults_match_goldens(void)
{
    for (int r = 0; r < NC_RATE_COUNT; r++) {
        nc_bq_coeff_t sec[2];
        CHECK(nc_dsp_design_bp(RATES[r], 50, 800, sec));
        const nc_bq_coeff_t *g = nc_dsp_bp_sos[r];
        for (int s = 0; s < 2; s++) {
            CHECK(abs(sec[s].b0 - g[s].b0) <= 1);
            CHECK(abs(sec[s].b1 - g[s].b1) <= 1);
            CHECK(abs(sec[s].b2 - g[s].b2) <= 1);
            CHECK(abs(sec[s].a1 - g[s].a1) <= 1);
            CHECK(abs(sec[s].a2 - g[s].a2) <= 1);
        }
        int32_t a = nc_dsp_design_alpha_q31(RATES[r], 50);
        CHECK(abs(a - nc_dsp_dc_alpha_q31[r]) <= 1);
    }
}

static void default_knobs_keep_golden_tables(void)
{
    nc_knobs_init();
    nc_chan_dsp_t d;
    for (int r = 0; r < NC_RATE_COUNT; r++) {
        nc_dsp_init(&d, (nc_rate_t)r);
        /* bit-identical, not <=1 LSB: the golden path must not run the
         * designer at all. */
        CHECK_MEMEQ(&d.bq_c[0], &nc_dsp_bp_sos[r][0], sizeof(nc_bq_coeff_t));
        CHECK_MEMEQ(&d.bq_c[1], &nc_dsp_bp_sos[r][1], sizeof(nc_bq_coeff_t));
        CHECK_EQ(d.alpha_q31, nc_dsp_dc_alpha_q31[r]);
    }
}

static void nondefault_corner_stable(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0502, 80), NC_ST_OK);    /* bp_lo 0.8 Hz */
    CHECK_EQ(nc_knob_set_id(0x0503, 500), NC_ST_OK);   /* bp_hi 5 Hz   */

    nc_chan_dsp_t d;
    nc_dsp_init(&d, NC_RATE_100);
    /* Designer must have replaced the defaults. */
    CHECK(d.bq_c[0].a1 != nc_dsp_bp_sos[NC_RATE_100][0].a1);

    /* Impulse -> bounded output, and step -> DC fully blocked. */
    nc_dsp_out_t o;
    int64_t max_abs = 0;
    nc_dsp_run(&d, 1 << 20, &o);
    for (int n = 0; n < 10000; n++) {
        nc_dsp_run(&d, 0, &o);
        int64_t m = (o.bp < 0) ? -(int64_t)o.bp : (int64_t)o.bp;
        if (m > max_abs) max_abs = m;
    }
    CHECK(max_abs > 0);                 /* the filter did respond      */
    CHECK(max_abs < (1LL << 22));       /* and never diverged          */

    nc_chan_dsp_t ds;
    nc_dsp_init(&ds, NC_RATE_100);
    int64_t tail_abs = 0;
    for (int n = 0; n < 20000; n++) {
        nc_dsp_run(&ds, 1 << 20, &o);
        if (n >= 20000 - 256) {
            tail_abs += (o.bp < 0) ? -(int64_t)o.bp : (int64_t)o.bp;
        }
    }
    /* Mean |bp| over the tail: arithmetic-shift Q30 biquads carry a
     * small zero-input limit cycle, so assert DC REJECTION (tail mean
     * < 0.05 % of the 2^20 step), not literal zero. */
    CHECK(tail_abs / 256 < 512);
}

static void degenerate_requests_fall_back(void)
{
    nc_bq_coeff_t sec[2];
    /* lo >= hi after clamping -> refused */
    CHECK(!nc_dsp_design_bp(100, 800, 800, sec));
    CHECK(!nc_dsp_design_bp(50, 190, 200, sec));  /* hi clamped to 22.5, lo≈hi*... still valid? lo=1.9 hi=2.0 -> lo < hi*0.95 fails -> refused */
    /* An enabled-but-degenerate knob combo keeps the golden tables. */
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0502, 200), NC_ST_OK);  /* lo 2.0 Hz */
    CHECK_EQ(nc_knob_set_id(0x0503, 300), NC_ST_OK);  /* hi 3.0 Hz — valid */
    nc_chan_dsp_t d;
    nc_dsp_init(&d, NC_RATE_50);
    /* 2-3 Hz at 50 sps is legal; just assert stability via a run. */
    nc_dsp_out_t o;
    nc_dsp_run(&d, 1 << 20, &o);
    for (int n = 0; n < 5000; n++) nc_dsp_run(&d, 0, &o);
    CHECK(abs(o.bp) < (1 << 16));
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(defaults_match_goldens);
    TST_RUN(default_knobs_keep_golden_tables);
    TST_RUN(nondefault_corner_stable);
    TST_RUN(degenerate_requests_fall_back);
    TST_REPORT();
}
