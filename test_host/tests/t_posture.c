/*
 * t_posture.c — ear posture from gravity (nc_posture.h).
 *
 * Vectors are the measured V2.1 ones (2026-10-09, board 58:E6:C5:15:A9:74,
 * 50 Hz, 1 g = 498 counts): worn on the lobe (-0.99, -0.09, -0.07) g,
 * lying on the table (0.13, -0.10, -0.99) g. Defaults: ear_axis 1 (-x),
 * ear_cos_pct 75, ear_hold_ms 2000 -> 100 samples at 50 Hz.
 */
#include "tst.h"
#include "narbis/nc_posture.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_posture");

#define ODR 50
#define G   498

static void feed_g(nc_posture_t *p, int n, int gx_pct, int gy_pct, int gz_pct, int g)
{
    for (int i = 0; i < n; i++) {
        (void)nc_posture_feed(p, (int16_t)(gx_pct * g / 100), (int16_t)(gy_pct * g / 100),
                              (int16_t)(gz_pct * g / 100), ODR);
    }
}

static void test_worn_vector_after_hold(void)
{
    nc_knobs_init();
    nc_posture_t p;
    nc_posture_init(&p);
    feed_g(&p, 99, -99, -9, -7, G);
    CHECK(!nc_posture_is_ear(&p));          /* 99 samples < 2 s */
    feed_g(&p, 1, -99, -9, -7, G);
    CHECK(nc_posture_is_ear(&p));           /* exactly 100 */
}

static void test_table_vector_never(void)
{
    nc_knobs_init();
    nc_posture_t p;
    nc_posture_init(&p);
    feed_g(&p, 1000, 13, -10, -99, G);
    CHECK(!nc_posture_is_ear(&p));
}

static void test_release_needs_the_hold_too(void)
{
    nc_knobs_init();
    nc_posture_t p;
    nc_posture_init(&p);
    feed_g(&p, 200, -99, -9, -7, G);
    CHECK(nc_posture_is_ear(&p));
    /* Head turn / lean: 1 s off-axis keeps the verdict ... */
    feed_g(&p, 50, 0, -70, -70, G);
    feed_g(&p, 60, -99, -9, -7, G);
    CHECK(nc_posture_is_ear(&p));
    /* ... laid on the table for good drops it after the hold (the
     * low-pass needs a few samples to swing, so allow its lag). */
    feed_g(&p, 99, 13, -10, -99, G);
    CHECK(nc_posture_is_ear(&p));
    feed_g(&p, 40, 13, -10, -99, G);
    CHECK(!nc_posture_is_ear(&p));
}

static void test_cone_edge(void)
{
    nc_knobs_init();
    nc_posture_t p;
    nc_posture_init(&p);
    feed_g(&p, 300, -71, 0, -71, G);        /* 45 deg: cos 0.71 < 0.75 */
    CHECK(!nc_posture_is_ear(&p));
    nc_posture_init(&p);
    feed_g(&p, 300, -87, 0, -50, G);        /* 30 deg: cos 0.87 */
    CHECK(nc_posture_is_ear(&p));
    nc_posture_init(&p);
    feed_g(&p, 300, 99, 9, 7, G);           /* hanging upside down: +x */
    CHECK(!nc_posture_is_ear(&p));
}

static void test_scale_free_and_axis_knob(void)
{
    nc_knobs_init();
    nc_posture_t p;
    nc_posture_init(&p);
    feed_g(&p, 200, -99, -9, -7, 16384);    /* +-2 g high-res scale */
    CHECK(nc_posture_is_ear(&p));

    CHECK_EQ(nc_knob_set_id(0x0804, 5), NC_ST_OK);   /* ear_axis = -z */
    nc_posture_init(&p);
    feed_g(&p, 200, 13, -10, -99, G);       /* the table now qualifies */
    CHECK(nc_posture_is_ear(&p));
    nc_knobs_init();
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(test_worn_vector_after_hold);
    TST_RUN(test_table_vector_never);
    TST_RUN(test_release_needs_the_hold_too);
    TST_RUN(test_cone_edge);
    TST_RUN(test_scale_free_and_axis_knob);
    TST_REPORT();
}
