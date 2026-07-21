/*
 * t_batt_curve.c — knot exactness, clamps, midpoint rounding, and
 * monotonicity over the whole plausible ADC range.
 */
#include "narbis/nc_batt_curve.h"
#include "tst.h"

TST_SUITE("batt_curve");

static void test_knots_exact(void)
{
    CHECK_EQ(nc_batt_pct(3300), 0);
    CHECK_EQ(nc_batt_pct(3500), 5);
    CHECK_EQ(nc_batt_pct(3680), 10);
    CHECK_EQ(nc_batt_pct(3730), 20);
    CHECK_EQ(nc_batt_pct(3760), 30);
    CHECK_EQ(nc_batt_pct(3790), 40);
    CHECK_EQ(nc_batt_pct(3820), 50);
    CHECK_EQ(nc_batt_pct(3870), 60);
    CHECK_EQ(nc_batt_pct(3920), 70);
    CHECK_EQ(nc_batt_pct(3980), 80);
    CHECK_EQ(nc_batt_pct(4060), 90);
    CHECK_EQ(nc_batt_pct(4200), 100);
}

static void test_clamps(void)
{
    CHECK_EQ(nc_batt_pct(0), 0);
    CHECK_EQ(nc_batt_pct(3000), 0);
    CHECK_EQ(nc_batt_pct(3299), 0);
    CHECK_EQ(nc_batt_pct(4201), 100);
    CHECK_EQ(nc_batt_pct(4400), 100);
    CHECK_EQ(nc_batt_pct(65535), 100);
}

static void test_midpoints_round_to_nearest(void)
{
    /* 4060..4200 (90..100): 4130 is exactly half -> 95 */
    CHECK_EQ(nc_batt_pct(4130), 95);
    /* 3820..3870 (50..60): 3845 -> 55 */
    CHECK_EQ(nc_batt_pct(3845), 55);
    /* 3300..3500 (0..5): 3400 -> 2.5 rounds to 3 */
    CHECK_EQ(nc_batt_pct(3400), 3);
    /* 3500..3680 (5..10): 3590 -> 5 + 450/180 = 7.5 -> 8 */
    CHECK_EQ(nc_batt_pct(3590), 8);
}

static void test_monotonic_full_sweep(void)
{
    int violations = 0;
    uint8_t prev = nc_batt_pct(3000);
    for (uint16_t mv = 3001; mv <= 4400; mv++) {
        uint8_t cur = nc_batt_pct(mv);
        if (cur < prev) violations++;
        prev = cur;
    }
    CHECK_EQ(violations, 0);
    /* endpoints of the sweep */
    CHECK_EQ(nc_batt_pct(3000), 0);
    CHECK_EQ(nc_batt_pct(4400), 100);
}

int main(void)
{
    TST_RUN(test_knots_exact);
    TST_RUN(test_clamps);
    TST_RUN(test_midpoints_round_to_nearest);
    TST_RUN(test_monotonic_full_sweep);
    TST_REPORT();
}
