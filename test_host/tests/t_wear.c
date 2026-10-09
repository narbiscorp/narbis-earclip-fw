/*
 * t_wear.c — wear detector: entry/stay bands, bp-power floor, 2-tick
 * attack, wear_off_s release, hysteresis between the two upper
 * thresholds, and the rail / jolt fast-off paths (see nc_wear.h).
 *
 * Knob defaults: dark 100000, off_thr 1900000 (entry upper), on_thr
 * 2050000 (stay upper), wear_off_s 30, wear_rail_s 2, wear_jolt_off_s 3,
 * wear_jolt_win_s 10. bp floor = 64 * ibi_thr_min^2 = 64 * 100^2 = 640000.
 * DC levels are the measured V2.1 ones (amb-subtracted IR DC): earlobe
 * with the AGC locked ~565k, empty clip in air ~35k.
 */
#include <stdio.h>
#include "tst.h"
#include "narbis/nc_wear.h"
#include "narbis/nc_knobs.h"

TST_SUITE("t_wear");

#define BP_GOOD   1000000   /* comfortably above the 640000 floor        */
#define DC_ONEAR  565000    /* earlobe, AGC locked                       */
#define DC_AIR    35000     /* empty clip in air: below dark             */
#define DC_GAP    2000000   /* between off_thr and on_thr (hysteresis)   */
#define DC_HIGH   2090000   /* above the stay band                       */
#define DC_TABLE  1760000   /* empty clip on a table, amb-subtracted: in
                             * band, flat (bp 0), raw IR at the rail     */

static void ticks(nc_wear_t *w, int n, int32_t dc, int64_t bp)
{
    for (int i = 0; i < n; i++) nc_wear_tick_1hz(w, dc, bp);
}

static void tick_ex(nc_wear_t *w, int32_t dc, int64_t bp, bool rail, bool jolt)
{
    const nc_wear_in_t in = { .dc_ir = dc, .bp_power_1s = bp,
                              .rail = rail, .jolt = jolt };
    nc_wear_tick(w, &in);
}

static void worn_now(nc_wear_t *w)
{
    nc_wear_init(w);
    ticks(w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(w));
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
    nc_wear_tick_1hz(&w, DC_AIR, BP_GOOD);  /* dark blip resets streak */
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
    ticks(&w, 100, DC_GAP, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));
}

static void test_dark_never_worn(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    ticks(&w, 100, DC_AIR, BP_GOOD);        /* empty clip in air */
    CHECK(!nc_wear_is_worn(&w));
    ticks(&w, 100, 100000, BP_GOOD);        /* boundary is exclusive */
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

    /* DC_GAP sits between off_thr and on_thr: outside the ENTRY band ->
     * a cold detector must never turn on there... */
    ticks(&w, 100, DC_GAP, BP_GOOD);
    CHECK(!nc_wear_is_worn(&w));

    /* ...but once worn, the STAY band (up to on_thr) tolerates it
     * indefinitely: no wear_off_s expiry, no chatter. */
    ticks(&w, 2, DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
    ticks(&w, 200, DC_GAP, BP_GOOD);        /* > default wear_off_s=30 */
    CHECK(nc_wear_is_worn(&w));
}

static void test_off_timing_exact(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0903, 5), NC_ST_OK);   /* wear_off_s = 5 */
    nc_wear_t w;
    worn_now(&w);

    /* Clip removed, DC out of the stay band (no rail flag: the slow
     * path). Worn holds for 4 bad ticks, drops exactly on the 5th. */
    for (int i = 0; i < 4; i++) {
        nc_wear_tick_1hz(&w, DC_HIGH, 0);
        CHECK(nc_wear_is_worn(&w));
    }
    nc_wear_tick_1hz(&w, DC_HIGH, 0);
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
    worn_now(&w);

    /* 4 bad, 1 good, 4 bad: never 5 consecutive -> stays worn. */
    ticks(&w, 4, DC_HIGH, 0);
    ticks(&w, 1, DC_ONEAR, BP_GOOD);
    ticks(&w, 4, DC_HIGH, 0);
    CHECK(nc_wear_is_worn(&w));
    ticks(&w, 1, DC_HIGH, 0);               /* 5th consecutive bad */
    CHECK(!nc_wear_is_worn(&w));
    nc_knobs_init();
}

static void test_flicker_obeys_hysteresis(void)
{
    nc_knobs_init();
    CHECK_EQ(nc_knob_set_id(0x0903, 5), NC_ST_OK);
    nc_wear_t w;
    worn_now(&w);

    /* Loose clip: DC alternates between in-band and the hysteresis zone.
     * Every tick is good for the STAY band, so the wearer never gets
     * dropped. */
    for (int i = 0; i < 100; i++) {
        nc_wear_tick_1hz(&w, (i & 1) ? DC_GAP : DC_ONEAR, BP_GOOD);
        CHECK(nc_wear_is_worn(&w));
    }

    /* Alternating good/bad ticks: bad streak never reaches 5. */
    for (int i = 0; i < 100; i++)
        nc_wear_tick_1hz(&w, (i & 1) ? DC_HIGH : DC_ONEAR, BP_GOOD);
    CHECK(nc_wear_is_worn(&w));
    nc_knobs_init();
}

/* ---- rail fast-off ------------------------------------------------- */

static void test_rail_drops_after_two_seconds(void)
{
    nc_knobs_init();
    nc_wear_t w;
    worn_now(&w);
    /* Clip on the table: amb-subtracted DC still in band, but flat and
     * the raw IR at the rail. Off after wear_rail_s = 2, not 30. */
    tick_ex(&w, DC_TABLE, 0, true, false);
    CHECK(nc_wear_is_worn(&w));
    tick_ex(&w, DC_TABLE, 0, true, false);
    CHECK(!nc_wear_is_worn(&w));

    /* Even a (spoofed) pulse does not save a railed clip. */
    worn_now(&w);
    tick_ex(&w, DC_ONEAR, BP_GOOD, true, false);
    tick_ex(&w, DC_ONEAR, BP_GOOD, true, false);
    CHECK(!nc_wear_is_worn(&w));
}

static void test_rail_must_be_consecutive(void)
{
    nc_knobs_init();
    nc_wear_t w;
    worn_now(&w);
    for (int i = 0; i < 50; i++) {
        tick_ex(&w, DC_ONEAR, BP_GOOD, (i & 1) != 0, false);
        CHECK(nc_wear_is_worn(&w));
    }
}

static void test_rail_never_enters(void)
{
    nc_knobs_init();
    nc_wear_t w;
    nc_wear_init(&w);
    for (int i = 0; i < 100; i++) tick_ex(&w, DC_ONEAR, BP_GOOD, true, false);
    CHECK(!nc_wear_is_worn(&w));
    tick_ex(&w, DC_ONEAR, BP_GOOD, false, false);
    tick_ex(&w, DC_ONEAR, BP_GOOD, false, false);
    CHECK(nc_wear_is_worn(&w));
}

/* ---- jolt fast-off ------------------------------------------------- */

static void test_jolt_shortens_the_countdown(void)
{
    nc_knobs_init();
    nc_wear_t w;
    worn_now(&w);
    /* Taken off and held in the air: the handling jolt, then dark ticks.
     * Off after wear_jolt_off_s = 3 instead of wear_off_s = 30. */
    tick_ex(&w, DC_ONEAR, BP_GOOD, false, true);
    CHECK(nc_wear_is_worn(&w));
    tick_ex(&w, DC_AIR, 0, false, false);
    tick_ex(&w, DC_AIR, 0, false, false);
    CHECK(nc_wear_is_worn(&w));
    tick_ex(&w, DC_AIR, 0, false, false);
    CHECK(!nc_wear_is_worn(&w));

    /* Without the jolt the same removal takes the full 30 s. */
    worn_now(&w);
    for (int i = 0; i < 29; i++) {
        tick_ex(&w, DC_AIR, 0, false, false);
        CHECK(nc_wear_is_worn(&w));
    }
    tick_ex(&w, DC_AIR, 0, false, false);
    CHECK(!nc_wear_is_worn(&w));
}

static void test_jolt_alone_never_drops_a_wearer(void)
{
    nc_knobs_init();
    nc_wear_t w;
    worn_now(&w);
    /* Head movement, nodding, walking: jolts every second with a good
     * pulse change nothing. */
    for (int i = 0; i < 200; i++) {
        tick_ex(&w, DC_ONEAR, BP_GOOD, false, true);
        CHECK(nc_wear_is_worn(&w));
    }
}

static void test_jolt_window_expires(void)
{
    nc_knobs_init();
    nc_wear_t w;
    worn_now(&w);
    tick_ex(&w, DC_ONEAR, BP_GOOD, false, true);   /* jolt, age 0 */
    for (int i = 0; i < 10; i++) tick_ex(&w, DC_ONEAR, BP_GOOD, false, false);
    /* First bad tick is 11 s after the jolt: outside wear_jolt_win_s. */
    for (int i = 0; i < 29; i++) {
        tick_ex(&w, DC_AIR, 0, false, false);
        CHECK(nc_wear_is_worn(&w));
    }
    tick_ex(&w, DC_AIR, 0, false, false);
    CHECK(!nc_wear_is_worn(&w));
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
    TST_RUN(test_rail_drops_after_two_seconds);
    TST_RUN(test_rail_must_be_consecutive);
    TST_RUN(test_rail_never_enters);
    TST_RUN(test_jolt_shortens_the_countdown);
    TST_RUN(test_jolt_alone_never_drops_a_wearer);
    TST_RUN(test_jolt_window_expires);
    TST_REPORT();
}
