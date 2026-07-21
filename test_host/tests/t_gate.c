/*
 * t_gate.c — artifact gate core, accel window energy, DC-step detector.
 */
#include <stdio.h>
#include "tst.h"
#include "narbis/nc_gate.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_gate");

/* Wrapper: one update call from loose parameters. */
static bool upd(nc_gate_t *g, uint32_t e, bool sat, bool step, bool col,
                bool agc, bool wear, uint32_t now, uint8_t *mask)
{
    nc_gate_in_t in = { .acc_energy = e, .sat = sat, .dc_step = step,
                        .amp_collapse = col, .agc_settling = agc,
                        .wear_off = wear };
    return nc_gate_update(g, &in, now, mask);
}

static void test_truth_table(void)
{
    nc_knobs_init();
    uint8_t m;
    nc_gate_t g;

    /* Default gate_acc_thr = 8000: >= gates, below does not. */
    nc_gate_init(&g);
    CHECK(upd(&g, 8000, 0, 0, 0, 0, 0, 0, &m));
    CHECK_EQ(m, NC_GATE_REASON_ACCEL);
    nc_gate_init(&g);
    CHECK(!upd(&g, 7999, 0, 0, 0, 0, 0, 0, &m));
    CHECK_EQ(m, 0);

    struct { int which; uint8_t bit; } cases[] = {
        { 0, NC_GATE_REASON_SAT },
        { 1, NC_GATE_REASON_DC_STEP },
        { 2, NC_GATE_REASON_COLLAPSE },
        { 3, NC_GATE_REASON_AGC },
        { 4, NC_GATE_REASON_WEAR },
    };
    for (int i = 0; i < 5; i++) {
        nc_gate_init(&g);
        CHECK(upd(&g, 0, cases[i].which == 0, cases[i].which == 1,
                  cases[i].which == 2, cases[i].which == 3,
                  cases[i].which == 4, 0, &m));
        CHECK_EQ(m, cases[i].bit);
    }

    /* No reason, fresh gate: ungated. NULL reason_out must be safe. */
    nc_gate_init(&g);
    CHECK(!upd(&g, 0, 0, 0, 0, 0, 0, 0, NULL));

    /* Simultaneous reasons OR into the mask. */
    nc_gate_init(&g);
    CHECK(upd(&g, 10000, 1, 0, 0, 0, 1, 0, &m));
    CHECK_EQ(m, NC_GATE_REASON_ACCEL | NC_GATE_REASON_SAT |
                NC_GATE_REASON_WEAR);
}

static void test_release_timing(void)
{
    nc_knobs_init();                 /* gate_release_ms = 500 */
    uint8_t m;
    nc_gate_t g;
    nc_gate_init(&g);

    CHECK(upd(&g, 0, 1, 0, 0, 0, 0, 1000, &m));       /* attack at 1000 */
    /* Reason gone: release tail holds, latched mask reported. */
    CHECK(upd(&g, 0, 0, 0, 0, 0, 0, 1001, &m));
    CHECK_EQ(m, NC_GATE_REASON_SAT);
    CHECK(upd(&g, 0, 0, 0, 0, 0, 0, 1499, &m));
    CHECK_EQ(m, NC_GATE_REASON_SAT);
    /* Exact boundary: now >= last_active + release -> ungated. */
    CHECK(!upd(&g, 0, 0, 0, 0, 0, 0, 1500, &m));
    CHECK_EQ(m, 0);

    /* Re-trigger inside the tail restarts the countdown. */
    nc_gate_init(&g);
    CHECK(upd(&g, 0, 1, 0, 0, 0, 0, 2000, &m));
    CHECK(upd(&g, 0, 0, 1, 0, 0, 0, 2300, &m));       /* new reason    */
    CHECK_EQ(m, NC_GATE_REASON_DC_STEP);              /* mask replaced */
    CHECK(upd(&g, 0, 0, 0, 0, 0, 0, 2799, &m));
    CHECK(!upd(&g, 0, 0, 0, 0, 0, 0, 2800, &m));

    /* release = 0: drops on the first reason-free call. */
    CHECK_EQ(nc_knob_set_id(0x0707, 0), NC_ST_OK);
    nc_gate_init(&g);
    CHECK(upd(&g, 0, 1, 0, 0, 0, 0, 5000, &m));
    CHECK(!upd(&g, 0, 0, 0, 0, 0, 0, 5001, &m));
    nc_knobs_init();
}

static void test_master_enable(void)
{
    nc_knobs_init();
    uint8_t m;
    nc_gate_t g;
    nc_gate_init(&g);

    CHECK_EQ(nc_knob_set_id(0x0701, 0), NC_ST_OK);    /* gate_en off */
    CHECK(!upd(&g, 100000, 1, 1, 1, 1, 1, 0, &m));
    CHECK_EQ(m, 0);

    /* Re-enable: no stale release tail may survive the off period. */
    CHECK_EQ(nc_knob_set_id(0x0701, 1), NC_ST_OK);
    CHECK(!upd(&g, 0, 0, 0, 0, 0, 0, 1, &m));
    CHECK(upd(&g, 0, 1, 0, 0, 0, 0, 2, &m));
    nc_knobs_init();
}

static void test_acc_thr_knob(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0703, 500000), NC_ST_OK);  /* gate_acc_thr */
    uint8_t m;
    nc_gate_t g;
    nc_gate_init(&g);
    CHECK(!upd(&g, 400000, 0, 0, 0, 0, 0, 0, &m));
    CHECK(upd(&g, 500000, 0, 0, 0, 0, 0, 600, &m));
    CHECK_EQ(m, NC_GATE_REASON_ACCEL);
    nc_knobs_init();
}

/* ------------------------------------------------------------------ */
/* Accel energy                                                        */
/* ------------------------------------------------------------------ */

static void test_accel_quiet_vs_shake(void)
{
    nc_knobs_init();                /* win 200 ms */
    nc_gate_acc_t a;

    nc_gate_acc_init(&a, 50);       /* 50 Hz -> 10-sample window */
    CHECK_EQ(a.wlen, 10);

    /* Still device, 1 g on Z: the seeded HP kills the DC entirely. */
    uint32_t e = 0;
    for (int i = 0; i < 100; i++) e = nc_gate_accel_feed(&a, 0, 0, 16384);
    CHECK_EQ(e, 0);

    /* Add LSB-level noise: energy stays far below the 8000 default. */
    for (int i = 0; i < 100; i++)
        e = nc_gate_accel_feed(&a, 0, 0, (int16_t)(16384 + ((i & 1) ? 2 : -2)));
    CHECK(e < 8000);

    /* Shake: +-8000-count square wave on X clears the threshold. */
    for (int i = 0; i < 40; i++)
        e = nc_gate_accel_feed(&a, (int16_t)((i & 1) ? 8000 : -8000),
                               0, 16384);
    CHECK(e >= 8000);

    /* Violent shake saturates the u32 window energy, no wrap. */
    for (int i = 0; i < 40; i++)
        e = nc_gate_accel_feed(&a, (int16_t)((i & 1) ? 30000 : -30000),
                               (int16_t)((i & 1) ? -30000 : 30000), 0);
    CHECK_EQ(e, UINT32_MAX);

    /* Back to quiet: the window flushes and energy decays again (200
     * samples: the Z DC tracker rewinds from the shake block through
     * ~15000 counts with tau 16, so the tail needs to die out too). */
    for (int i = 0; i < 200; i++) e = nc_gate_accel_feed(&a, 0, 0, 16384);
    CHECK(e < 8000);
}

static void test_accel_window_clamp(void)
{
    nc_knobs_init();
    nc_gate_acc_t a;
    /* 400 Hz x 1000 ms = 400 samples, within the ring. */
    CHECK_EQ(nc_knob_set_id(0x0702, 1000), NC_ST_OK);
    nc_gate_acc_init(&a, 400);
    CHECK_EQ(a.wlen, 400);
    /* Degenerate: 10 Hz x 50 ms -> clamps to 1 sample minimum. */
    CHECK_EQ(nc_knob_set_id(0x0702, 50), NC_ST_OK);
    nc_gate_acc_init(&a, 10);
    CHECK_EQ(a.wlen, 1);
    nc_knobs_init();
}

/* ------------------------------------------------------------------ */
/* DC-step detector + input builder                                    */
/* ------------------------------------------------------------------ */

/* Feed n samples of baseline+noise; returns number of flags raised.
 * Noise is +-4 counts so worst-case |ddc| = 8 == the detector's
 * threshold floor (strict >): quiet input can never flag, however low
 * the EMA sits. */
static int feed_quiet(nc_gate_dcstep_t *d, int n, int32_t base,
                      uint32_t *rng)
{
    int flags = 0;
    for (int i = 0; i < n; i++) {
        *rng = *rng * 1664525u + 1013904223u;
        int32_t noise = (int32_t)((*rng >> 16) % 9u) - 4;
        if (nc_gate_dcstep_feed(d, base + noise)) flags++;
    }
    return flags;
}

static void test_dcstep_warmup_guard(void)
{
    nc_knobs_init();
    nc_gate_dcstep_t d;
    nc_gate_dcstep_init(&d);
    uint32_t rng = 42;

    /* Huge step at sample 30: inside the 50-sample warmup, no flag. */
    CHECK_EQ(feed_quiet(&d, 29, 100000, &rng), 0);
    CHECK(!nc_gate_dcstep_feed(&d, 180000));
    /* Boundary: a jump on sample 50 is still guarded... */
    nc_gate_dcstep_init(&d);
    rng = 42;
    CHECK_EQ(feed_quiet(&d, 49, 100000, &rng), 0);
    CHECK(!nc_gate_dcstep_feed(&d, 180000));       /* sample 50 */
    /* ...a jump on sample 51 is not. */
    nc_gate_dcstep_init(&d);
    rng = 42;
    CHECK_EQ(feed_quiet(&d, 50, 100000, &rng), 0);
    CHECK(nc_gate_dcstep_feed(&d, 180000));        /* sample 51 */
}

static void test_dcstep_detect_and_recover(void)
{
    nc_knobs_init();
    nc_gate_dcstep_t d;
    nc_gate_dcstep_init(&d);
    uint32_t rng = 7;

    CHECK_EQ(feed_quiet(&d, 200, 100000, &rng), 0);
    /* AGC-sized DC jump: flags exactly on the jump sample... */
    CHECK(nc_gate_dcstep_feed(&d, 150000));
    /* ...and the baseline moves with it: the next normal-noise samples
     * at the new level do not keep flagging. */
    CHECK_EQ(feed_quiet(&d, 100, 150000, &rng), 0);
    /* Slow drift (1 count/sample) never flags. */
    for (int i = 0; i < 200; i++)
        CHECK(!nc_gate_dcstep_feed(&d, 150000 + i));
}

static void test_dcstep_sigma_knob(void)
{
    nc_knobs_init();
    nc_gate_dcstep_t d;
    uint32_t rng = 99;

    /* Alternating +-40 deltas -> EMA(|ddc|) ~ 40-80. With sigma 10.0
     * a 300-count step stays under threshold; with the default 4.0 it
     * flags. */
    CHECK_EQ(nc_knob_set_id(0x0705, 100), NC_ST_OK); /* step_sigma 10.0 */
    nc_gate_dcstep_init(&d);
    for (int i = 0; i < 100; i++)
        nc_gate_dcstep_feed(&d, 100000 + ((i & 1) ? 40 : -40));
    CHECK(!nc_gate_dcstep_feed(&d, 100400));

    CHECK_EQ(nc_knob_set_id(0x0705, 40), NC_ST_OK);  /* sigma 4.0 */
    nc_gate_dcstep_init(&d);
    for (int i = 0; i < 100; i++)
        nc_gate_dcstep_feed(&d, 100000 + ((i & 1) ? 40 : -40));
    CHECK(nc_gate_dcstep_feed(&d, 100400));
    (void)rng;
    nc_knobs_init();
}

static void test_build_input(void)
{
    nc_knobs_init();
    nc_gate_dcstep_t ds;
    nc_gate_dcstep_init(&ds);
    nc_gate_in_t in;
    uint32_t rng = 3;

    for (int i = 0; i < 100; i++) {
        rng = rng * 1664525u + 1013904223u;
        nc_gate_build_input(&in, &ds, 123, 100000 + (int32_t)(rng % 7u),
                            false, false, false, false);
        CHECK(!in.dc_step);
    }
    CHECK_EQ(in.acc_energy, 123);

    /* Step flows through the builder into the flag + full passthrough. */
    nc_gate_build_input(&in, &ds, 9999, 160000, true, true, true, true);
    CHECK(in.dc_step);
    CHECK_EQ(in.acc_energy, 9999);
    CHECK(in.sat && in.amp_collapse && in.agc_settling && in.wear_off);

    /* Builder output drives the gate with the full OR-mask. */
    nc_gate_t g;
    nc_gate_init(&g);
    uint8_t m;
    CHECK(nc_gate_update(&g, &in, 0, &m));
    CHECK_EQ(m, NC_GATE_REASON_ACCEL | NC_GATE_REASON_SAT |
                NC_GATE_REASON_DC_STEP | NC_GATE_REASON_COLLAPSE |
                NC_GATE_REASON_AGC | NC_GATE_REASON_WEAR);
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(test_truth_table);
    TST_RUN(test_release_timing);
    TST_RUN(test_master_enable);
    TST_RUN(test_acc_thr_knob);
    TST_RUN(test_accel_quiet_vs_shake);
    TST_RUN(test_accel_window_clamp);
    TST_RUN(test_dcstep_warmup_guard);
    TST_RUN(test_dcstep_detect_and_recover);
    TST_RUN(test_build_input);
    TST_RUN(test_dcstep_sigma_knob);
    TST_REPORT();
}
