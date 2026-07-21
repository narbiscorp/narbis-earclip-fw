/*
 * t_wear.c — wear detector: entry/stay bands, bp-power floor, 2-tick
 * attack, wear_off_s release, hysteresis between the two upper
 * thresholds (see nc_wear.h for the band semantics).
 *
 * Knob defaults: dark 2000, off_thr 10000 (entry upper), on_thr 20000
 * (stay upper). bp floor = 64 * ibi_thr_min^2 = 64 * 100^2 = 640000.
 */
#include <stdio.h>
#include "tst.h"
#include "narbis/nc_wear.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_wear");

#define BP_GOOD  1000000    /* comfortably above the 640000 floor */
#define DC_ONEAR 6000       /* inside the entry band (2000, 10000) */

static void ticks(nc_wear_t *w, int n, int32_t dc, int64_t bp)
{
    for (int i = 0; i < n; i++) nc_wear_tick_1hz(w, dc, bp);
}

static void test_on_after_two_ticks(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    CHECK(!nc_wear_is_worn(&w));
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));            /* one tick is not enough */
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));             /* exactly two */
}

static void test_on_streak_must_be_consecutive(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    nc_wear_tick_1hz(&w, 500, BP_GOOD);     /* dark blip resets streak */
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
}

static void test_near_rail_never_worn(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    /* Open clip: direct LED->PD coupling near the 2^21 rail. Even with
     * a (spoofed) plausible bp power the DC alone must veto. */
    ticks(&w, 100, 2000000, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));
}

static void test_dark_never_worn(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    ticks(&w, 100, 500, BP_GOOD);           /* clip in a case */
    CHECK(!nc_wear_is_worn(&w));
    ticks(&w, 100, 2000, BP_GOOD);          /* boundary is exclusive */
    CHECK(!nc_wear_is_worn(&w));
}

static void test_bp_floor(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    /* Table top / open clip at on-ear-like DC: no pulse, no wear. */
    ticks(&w, 100, DC_ONEAR, 639999);       /* one below the floor */
    CHECK(!nc_wear_is_worn(&w));
    ticks(&w, 2, DC_ONEAR, 640000);         /* floor is inclusive */
    CHECK(nc_wear_is_worn(&w));
}

static void test_entry_stay_hysteresis(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);

    /* 15000 sits between off_thr (10000) and on_thr (20000): outside
     * the ENTRY band -> a cold detector must never turn on there... */
    ticks(&w, 100, 15000, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));

    /* ...but once worn, the STAY band (up to on_thr) tolerates it
     * indefinitely: no wear_off_s expiry, no chatter. */
    ticks(&w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
    ticks(&w, 200, 15000, BP_GOOD);         /* > default wear_off_s=30 */
    CHECK(nc_wear_is_worn(&w));
}

static void test_off_timing_exact(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0903, 5), NC_ST_OK);   /* wear_off_s = 5 */
    nc_wear_t w;
    nc_wear_init(&w);
    ticks(&w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));

    /* Clip removed: DC runs to the rail. Worn holds for 4 bad ticks,
     * drops exactly on the 5th. */
    for (int i = 0; i < 4; i++) {
        nc_wear_tick_1hz(&w, 2000000, 0);
        CHECK(nc_wear_is_worn(&w));
    }
    nc_wear_tick_1hz(&w, 2000000, 0);
    CHECK(!nc_wear_is_worn(&w));

    /* Re-entry still takes the full 2-tick attack. */
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));
    nc_wear_tick_1hz(&w, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
    nc_knobs_init();
}

static void test_off_counter_resets_on_good_tick(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0903, 5), NC_ST_OK);
    nc_wear_t w;
    nc_wear_init(&w);
    ticks(&w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));

    /* 4 bad, 1 good, 4 bad: never 5 consecutive -> stays worn. */
    ticks(&w, 4, 2000000, 0);
    ticks(&w, 1, DC_ONEAR, BP_GOOD);
    ticks(&w, 4, 2000000, 0);
    CHECK(nc_wear_is_worn(&w));
    ticks(&w, 1, 2000000, 0);               /* 5th consecutive bad */
    CHECK(!nc_wear_is_worn(&w));
    nc_knobs_init();
}

static void test_flicker_obeys_hysteresis(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0903, 5), NC_ST_OK);
    nc_wear_t w;
    nc_wear_init(&w);
    ticks(&w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));

    /* Loose clip: DC alternates between in-band and the 10k..20k
     * hysteresis zone. Every tick is good for the STAY band, so the
     * wearer never gets dropped. */
    for (int i = 0; i < 100; i++) {
        nc_wear_tick_1hz(&w, (i & 1) ? 15000 : DC_ONEAR, BP_GOOD);
        CHECK(nc_wear_is_worn(&w));
    }

    /* Alternating good/bad ticks: bad streak never reaches 5. */
    for (int i = 0; i < 100; i++)
        nc_wear_tick_1hz(&w, (i & 1) ? 2000000 : DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
    nc_knobs_init();
}

int main(void)
{
    nc_knobs_init();
    TST_RUN(test_on_after_two_ticks);
    TST_RUN(test_on_streak_must_be_consecutive);
    TST_RUN(test_near_rail_never_worn);
    TST_RUN(test_dark_never_worn);
    TST_RUN(test_bp_floor);
    TST_RUN(test_entry_stay_hysteresis);
    TST_RUN(test_off_timing_exact);
    TST_RUN(test_off_counter_resets_on_good_tick);
    TST_RUN(test_flicker_obeys_hysteresis);
    TST_REPORT();
}
