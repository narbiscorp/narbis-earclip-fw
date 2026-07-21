/*
 * t_timesync.c — drift EMA: known-drift accuracy, spacing gate,
 * EMA smoothing, 1 h pruning, ring cap, clamps, dev-clock reset.
 */
#include "narbis/nc_timesync.h"
#include "tst.h"

TST_SUITE("timesync");

/* realistic epoch-us host base exercises the int64 paths */
#define HB 1700000000000000ULL
#define DB 5000000ULL

static void test_unknown_until_spacing(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0x7FFF);

    nc_ts_add_pair(&t, HB, DB);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0x7FFF);      /* one pair */

    nc_ts_add_pair(&t, HB + 30000000, DB + 30000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0x7FFF);      /* 30 s < 60 s */

    nc_ts_add_pair(&t, HB + 70000000, DB + 70000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0);           /* zero drift, known */
}

static void test_plus_50_ppm(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    /* host = dev * (1 + 50e-6): +d/20000, exact for d multiple of 20000 */
    for (int k = 0; k < 4; k++) {
        uint64_t d = (uint64_t)k * 100000000ULL;    /* 100 s apart */
        nc_ts_add_pair(&t, HB + d + d / 20000, DB + d);
    }
    int16_t ppm = nc_ts_drift_ppm_x10(&t);
    CHECK(ppm >= 500 - 5 && ppm <= 500 + 5);        /* +50.0 ppm */
}

static void test_minus_120_ppm(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    /* host = dev * (1 - 120e-6): -3d/25000, exact for these d */
    for (int k = 0; k < 4; k++) {
        uint64_t d = (uint64_t)k * 100000000ULL;
        nc_ts_add_pair(&t, HB + d - (d * 3) / 25000, DB + d);
    }
    int16_t ppm = nc_ts_drift_ppm_x10(&t);
    CHECK(ppm >= -1200 - 5 && ppm <= -1200 + 5);    /* -120.0 ppm */
}

static void test_ema_smooths_jitter(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    nc_ts_add_pair(&t, HB, DB);

    /* jittered pair: instantaneous estimate 800 seeds the EMA */
    nc_ts_add_pair(&t, HB + 100000000 + 8000, DB + 100000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 800);

    /* true-zero pairs pull it down by alpha=0.25 steps, not jumps */
    nc_ts_add_pair(&t, HB + 200000000, DB + 200000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 600);         /* 800 - 200 */
    nc_ts_add_pair(&t, HB + 300000000, DB + 300000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 450);         /* 600 - 150 */
}

static void test_one_hour_pruning(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    nc_ts_add_pair(&t, HB, DB);
    nc_ts_add_pair(&t, HB + 100000000, DB + 100000000);
    CHECK_EQ(t.count, 2);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0);

    /* ~66.7 min later: both old pairs are > 1 h older -> pruned */
    nc_ts_add_pair(&t, HB + 4000000000ULL, DB + 4000000000ULL);
    CHECK_EQ(t.count, 1);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0);           /* last EMA retained */

    /* estimation resumes on the fresh baseline: inst 500 -> EMA 125 */
    uint64_t d = 4100000000ULL;
    nc_ts_add_pair(&t, HB + d + 5000, DB + d);      /* +50 ppm over 100 s */
    CHECK_EQ(t.count, 2);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 125);         /* 0 + 500/4 */
}

static void test_ring_capped_at_8(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    for (int k = 0; k < 10; k++) {
        uint64_t d = (uint64_t)k * 100000000ULL;
        nc_ts_add_pair(&t, HB + d, DB + d);
    }
    CHECK_EQ(t.count, 8);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0);
    /* oldest retained pair is #2 (dropped 0 and 1) */
    CHECK_EQ((long long)(t.pair[0].dev_us - DB), 200000000LL);
}

static void test_clamp_on_host_steps(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    nc_ts_add_pair(&t, HB, DB);
    /* absurd forward host step (NTP jump): clamps, no overflow */
    nc_ts_add_pair(&t, HB + 100000000 + (1ULL << 40), DB + 100000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 32000);

    nc_ts_init(&t);
    nc_ts_add_pair(&t, HB, DB);
    /* backward host step */
    nc_ts_add_pair(&t, HB + 100000000 - (1ULL << 40), DB + 100000000);
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), -32000);
}

static void test_dev_clock_reset_flushes(void)
{
    nc_ts_t t;
    nc_ts_init(&t);
    nc_ts_add_pair(&t, HB, DB + 5000000000ULL);
    /* device rebooted: dev time restarts near zero */
    nc_ts_add_pair(&t, HB + 100000000, 1000000ULL);
    CHECK_EQ(t.count, 1);                           /* stale pair flushed */
    CHECK_EQ(nc_ts_drift_ppm_x10(&t), 0x7FFF);      /* never had an estimate */
}

int main(void)
{
    TST_RUN(test_unknown_until_spacing);
    TST_RUN(test_plus_50_ppm);
    TST_RUN(test_minus_120_ppm);
    TST_RUN(test_ema_smooths_jitter);
    TST_RUN(test_one_hour_pruning);
    TST_RUN(test_ring_capped_at_8);
    TST_RUN(test_clamp_on_host_steps);
    TST_RUN(test_dev_clock_reset_flushes);
    TST_REPORT();
}
