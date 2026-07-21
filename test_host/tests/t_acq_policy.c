/*
 * t_acq_policy.c — subscription-gated acquisition rule matrix (§5.4).
 */
#include "narbis/nc_acq_policy.h"
#include "narbis/proto.h"
#include "tst.h"

TST_SUITE("acq_policy");

static nc_acq_in_t base(void)
{
    nc_acq_in_t in;
    memset(&in, 0, sizeof in);
    in.usb_stream_ok = true;   /* streaming allowed unless a test blocks it */
    return in;
}

static nc_acq_out_t ev(nc_acq_in_t in)
{
    nc_acq_out_t out;
    nc_acq_eval(&in, &out);
    /* invariant everywhere: DSP runs iff the AFE runs */
    CHECK_EQ(out.dsp_on, out.ppg_on);
    return out;
}

static void test_idle_nothing_runs(void)
{
    nc_acq_out_t o = ev(base());
    CHECK(!o.ppg_on && !o.accel_on && !o.dsp_on);
}

static void test_event_and_status_start_nothing(void)
{
    nc_acq_in_t in = base();
    in.sub_event = true;
    nc_acq_out_t o = ev(in);
    CHECK(!o.ppg_on && !o.accel_on);

    in = base();
    in.ctrl_start_mask = NC_STREAM_MASK_EVENT;  /* EVENT start bit inert too */
    o = ev(in);
    CHECK(!o.ppg_on && !o.accel_on);
}

static void test_each_ppg_source_starts_ppg(void)
{
    /* five independent ways to demand PPG */
    for (int src = 0; src < 5; src++) {
        nc_acq_in_t in = base();
        switch (src) {
        case 0: in.sub_ppg = true; break;
        case 1: in.sub_ibi = true; break;
        case 2: in.sub_hrs = true; break;   /* HRS alone starts the AFE */
        case 3: in.ctrl_start_mask = NC_STREAM_MASK_PPG; break;
        case 4: in.ctrl_start_mask = NC_STREAM_MASK_IBI; break;
        }
        nc_acq_out_t o = ev(in);
        CHECK(o.ppg_on && o.dsp_on);
        CHECK(!o.accel_on);                 /* gate_en off: no coupling */
    }
}

static void test_gate_couples_accel_to_ppg(void)
{
    nc_acq_in_t in = base();
    in.gate_en = true;
    nc_acq_out_t o = ev(in);
    CHECK(!o.accel_on);                     /* gate alone starts nothing */

    in.sub_hrs = true;
    o = ev(in);
    CHECK(o.ppg_on && o.accel_on);          /* gate needs motion data */

    in.gate_en = false;
    o = ev(in);
    CHECK(o.ppg_on && !o.accel_on);
}

static void test_explicit_accel(void)
{
    nc_acq_in_t in = base();
    in.sub_accel = true;
    nc_acq_out_t o = ev(in);
    CHECK(!o.ppg_on && o.accel_on);

    in = base();
    in.ctrl_start_mask = NC_STREAM_MASK_ACCEL;
    o = ev(in);
    CHECK(!o.ppg_on && o.accel_on);
}

static void test_stop_mask_per_source(void)
{
    /* stop PPG kills PPG-side demand... */
    nc_acq_in_t in = base();
    in.sub_ppg = true;
    in.ctrl_stop_mask = NC_STREAM_MASK_PPG;
    nc_acq_out_t o = ev(in);
    CHECK(!o.ppg_on);

    /* ...but not IBI-side demand (IBI still needs the AFE) */
    in.sub_ibi = true;
    o = ev(in);
    CHECK(o.ppg_on);

    /* stop IBI suppresses sub_ibi and sub_hrs, not sub_ppg */
    in = base();
    in.sub_ibi = true;
    in.sub_hrs = true;
    in.ctrl_stop_mask = NC_STREAM_MASK_IBI;
    o = ev(in);
    CHECK(!o.ppg_on);
    in.sub_ppg = true;
    o = ev(in);
    CHECK(o.ppg_on);

    /* stop beats a latched start of the same source */
    in = base();
    in.ctrl_start_mask = NC_STREAM_MASK_PPG;
    in.ctrl_stop_mask = NC_STREAM_MASK_PPG;
    o = ev(in);
    CHECK(!o.ppg_on);

    /* stop is stateless here: caller clearing the bit re-enables */
    in.ctrl_stop_mask = 0;
    o = ev(in);
    CHECK(o.ppg_on);
}

static void test_stop_accel_vs_gate_coupling(void)
{
    nc_acq_in_t in = base();
    in.sub_accel = true;
    in.ctrl_stop_mask = NC_STREAM_MASK_ACCEL;
    nc_acq_out_t o = ev(in);
    CHECK(!o.accel_on);                     /* explicit source stopped */

    /* gate coupling is internal use — a BLE accel stop cannot break
     * the artifact gate while PPG runs */
    in.sub_ppg = true;
    in.gate_en = true;
    o = ev(in);
    CHECK(o.ppg_on && o.accel_on);
}

static void test_wear_pause(void)
{
    nc_acq_in_t in = base();
    in.sub_ppg = true;
    in.wear_paused = true;
    nc_acq_out_t o = ev(in);
    CHECK(!o.ppg_on && !o.dsp_on);          /* AFE paused off-ear */
    CHECK(!o.accel_on);                     /* gate off: nothing keeps it */

    /* with the gate enabled, accel keeps running off-ear (motion wake) */
    in.gate_en = true;
    o = ev(in);
    CHECK(!o.ppg_on && o.accel_on);

    /* explicit accel subscription is not wear-paused either */
    in = base();
    in.sub_accel = true;
    in.wear_paused = true;
    o = ev(in);
    CHECK(o.accel_on);
}

static void test_usb_block_forces_all_off(void)
{
    nc_acq_in_t in = base();
    in.sub_ppg = in.sub_accel = in.sub_ibi = in.sub_hrs = true;
    in.ctrl_start_mask = NC_STREAM_MASK_PPG | NC_STREAM_MASK_ACCEL |
                         NC_STREAM_MASK_IBI;
    in.gate_en = true;
    in.usb_stream_ok = false;               /* stream_on_usb=0 + charging */
    nc_acq_out_t o = ev(in);
    CHECK(!o.ppg_on && !o.accel_on && !o.dsp_on);
}

static void test_full_combo(void)
{
    /* everything on, nothing blocked */
    nc_acq_in_t in = base();
    in.sub_ppg = in.sub_accel = in.sub_ibi = in.sub_event = in.sub_hrs = true;
    in.gate_en = true;
    nc_acq_out_t o = ev(in);
    CHECK(o.ppg_on && o.accel_on && o.dsp_on);
}

int main(void)
{
    TST_RUN(test_idle_nothing_runs);
    TST_RUN(test_event_and_status_start_nothing);
    TST_RUN(test_each_ppg_source_starts_ppg);
    TST_RUN(test_gate_couples_accel_to_ppg);
    TST_RUN(test_explicit_accel);
    TST_RUN(test_stop_mask_per_source);
    TST_RUN(test_stop_accel_vs_gate_coupling);
    TST_RUN(test_wear_pause);
    TST_RUN(test_usb_block_forces_all_off);
    TST_RUN(test_full_combo);
    TST_REPORT();
}
