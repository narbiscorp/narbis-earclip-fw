/*
 * t_button.c — button gesture FSM timing matrix.
 * Defaults: press_double_ms=400, press_long_ms=1500, press_reboot_ms=8000.
 * Every step also asserts the exact arm value (the timeout contract).
 */
#include "narbis/nc_button.h"
#include "narbis/nc_knobs.h"
#include "tst.h"

TST_SUITE("button");

static nc_btn_fsm_t f;
static uint32_t arm;

static nc_btn_act_t step(nc_btn_ev_t ev, uint32_t t)
{
    return nc_btn_step(&f, ev, t, &arm);
}

static void reset(void)
{
    nc_knobs_init();
    nc_btn_init(&f);
    arm = 0xDEADu;
}

static void test_single_short_marker_at_expiry(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 1000), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);                       /* long-hold watchdog */
    CHECK_EQ(step(NC_BTN_EV_UP, 1100), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);                        /* double window */
    /* MARKER exactly at window expiry: latency = 400 <= 450 budget */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1500), NC_BTN_ACT_MARKER);
    CHECK_EQ(arm, 0);
    CHECK(1500 - 1100 <= 450);
}

static void test_double_press_pairing_no_marker(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_UP, 80), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 300), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);                       /* 2nd press long watchdog */
    CHECK_EQ(step(NC_BTN_EV_UP, 400), NC_BTN_ACT_PAIRING);
    CHECK_EQ(arm, 0);
    /* stale window timeout the caller failed to cancel: swallowed */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 480), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 0);
}

static void test_long_hold_off_on_release(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1500), NC_BTN_ACT_NONE); /* nothing yet */
    CHECK_EQ(arm, 6500);                       /* reboot at t_down+8000 */
    CHECK_EQ(step(NC_BTN_EV_UP, 1600), NC_BTN_ACT_OFF);       /* fires on UP */
    CHECK_EQ(arm, 0);
}

static void test_reboot_at_timeout_without_release(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1500), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 6500);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 8000), NC_BTN_ACT_REBOOT);
    CHECK_EQ(arm, 0);
}

static void test_ghost_pulse_no_action(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_UP, 10), NC_BTN_ACT_NONE);  /* 10 ms: ghost */
    CHECK_EQ(arm, 0);                                   /* cancelled */
    /* even if the caller leaks the timeout, it must be inert */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1500), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 0);

    /* 29 ms: still ghost */
    CHECK_EQ(step(NC_BTN_EV_DOWN, 2000), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 2029), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 0);

    /* exactly 30 ms: real short press */
    CHECK_EQ(step(NC_BTN_EV_DOWN, 3000), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 3030), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 3430), NC_BTN_ACT_MARKER);
}

static void test_ghost_pulse_inside_double_window(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 100), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);                        /* window ends at 500 */
    CHECK_EQ(step(NC_BTN_EV_DOWN, 200), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_UP, 210), NC_BTN_ACT_NONE);  /* ghost */
    CHECK_EQ(arm, 290);                        /* window end unchanged */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 500), NC_BTN_ACT_MARKER);
    CHECK_EQ(arm, 0);
}

/* DECIDED resolution: 2nd DOWN in the window held >= press_long_ms
 * cancels pairing; the hold path (OFF on release) wins. */
static void test_second_press_held_long_wins_over_pairing(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 100), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 300), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1800), NC_BTN_ACT_NONE); /* no PAIRING */
    CHECK_EQ(arm, 6500);                       /* reboot at 300+8000 */
    CHECK_EQ(step(NC_BTN_EV_UP, 2000), NC_BTN_ACT_OFF);
    CHECK_EQ(arm, 0);
}

static void test_second_press_held_to_reboot(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 100), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 300), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1800), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 8300), NC_BTN_ACT_REBOOT);
    CHECK_EQ(arm, 0);
}

/* Timer/ISR race: UP crosses the long threshold before the TIMEOUT is
 * delivered — the hold semantics must still apply. */
static void test_release_after_long_without_timeout(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 1600), NC_BTN_ACT_OFF);
    CHECK_EQ(arm, 0);

    /* same race on the second press: OFF, never PAIRING */
    CHECK_EQ(step(NC_BTN_EV_DOWN, 5000), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 5100), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 5300), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 6900), NC_BTN_ACT_OFF);   /* held 1600 */
    CHECK_EQ(arm, 0);
}

/* Second DOWN loses the race against the window expiry: the overdue
 * MARKER is emitted and the DOWN starts a fresh gesture. */
static void test_late_down_after_window_expiry(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_UP, 100), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 600), NC_BTN_ACT_MARKER); /* window ended @500 */
    CHECK_EQ(arm, 1500);                       /* fresh first press */
    CHECK_EQ(step(NC_BTN_EV_UP, 700), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1100), NC_BTN_ACT_MARKER);
}

/* Duplicate edges re-emit the remaining time to the SAME deadline. */
static void test_duplicate_edges_keep_deadline(void)
{
    reset();
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 500), NC_BTN_ACT_NONE); /* dup DOWN */
    CHECK_EQ(arm, 1000);                       /* still fires at 1500 */
    CHECK_EQ(step(NC_BTN_EV_UP, 600), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_UP, 700), NC_BTN_ACT_NONE);  /* stray UP */
    CHECK_EQ(arm, 300);                        /* window still ends at 1000 */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1000), NC_BTN_ACT_MARKER);
}

static void test_knob_timings_are_live(void)
{
    reset();
    CHECK_EQ(nc_knob_set_id(0x0102 /* press_double_ms */, 250), NC_ST_OK);
    CHECK_EQ(nc_knob_set_id(0x0101 /* press_long_ms   */, 800), NC_ST_OK);
    CHECK_EQ(nc_knob_set_id(0x0103 /* press_reboot_ms */, 4000), NC_ST_OK);
    CHECK_EQ(step(NC_BTN_EV_DOWN, 0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 800);
    CHECK_EQ(step(NC_BTN_EV_UP, 50), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 250);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 300), NC_BTN_ACT_MARKER);

    CHECK_EQ(step(NC_BTN_EV_DOWN, 1000), NC_BTN_ACT_NONE);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 1800), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 3200);                       /* 4000 - 800 */
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, 5000), NC_BTN_ACT_REBOOT);
}

/* uint32 ms rollover (49.7 days): elapsed math must survive the wrap */
static void test_time_wraparound(void)
{
    reset();
    uint32_t t0 = 0xFFFFFF00u;
    CHECK_EQ(step(NC_BTN_EV_DOWN, t0), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 1500);
    CHECK_EQ(step(NC_BTN_EV_UP, t0 + 128), NC_BTN_ACT_NONE);
    CHECK_EQ(arm, 400);
    CHECK_EQ(step(NC_BTN_EV_TIMEOUT, t0 + 528), NC_BTN_ACT_MARKER); /* wrapped */
    CHECK_EQ(arm, 0);
}

int main(void)
{
    TST_RUN(test_single_short_marker_at_expiry);
    TST_RUN(test_double_press_pairing_no_marker);
    TST_RUN(test_long_hold_off_on_release);
    TST_RUN(test_reboot_at_timeout_without_release);
    TST_RUN(test_ghost_pulse_no_action);
    TST_RUN(test_ghost_pulse_inside_double_window);
    TST_RUN(test_second_press_held_long_wins_over_pairing);
    TST_RUN(test_second_press_held_to_reboot);
    TST_RUN(test_release_after_long_without_timeout);
    TST_RUN(test_late_down_after_window_expiry);
    TST_RUN(test_duplicate_edges_keep_deadline);
    TST_RUN(test_knob_timings_are_live);
    TST_RUN(test_time_wraparound);
    TST_REPORT();
}
