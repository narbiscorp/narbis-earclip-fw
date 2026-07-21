/*
 * nc_timesync.h — device clock drift estimation from TIME_SYNC pairs
 * (handoff §5.12 item 3; feeds STATUS.clock_drift_ppm_x10).
 *
 * Each CONTROL TIME_SYNC exchange yields one (host_epoch_us, dev_us)
 * pair. Pairs are kept in an 8-deep window, oldest..newest; pairs whose
 * dev time is more than 1 hour older than the newest are dropped, which
 * bounds the baseline dDev <= 3.6e9 us. On every add that leaves the
 * oldest and newest pair >= 60 s apart in dev time, an instantaneous
 * estimate is computed against the OLDEST retained pair:
 *
 *     inst_ppm_x10 = ((dHost - dDev) * 1e7) / dDev
 *
 * Overflow bounds (all int64): dHost-dDev is clamped to +-2^31 before
 * the multiply, so |diff * 1e7| <= 2.15e16 << 2^63. The result is
 * clamped to +-32000 (0x7FFF is reserved for "unknown"), then folded
 * into an EMA with alpha = 0.25 (integer: ema += (inst - ema) / 4,
 * truncation toward zero).
 *
 * A dev_us that goes BACKWARD (device reboot resets esp_timer) makes
 * the age computation of older pairs underflow huge, so they are all
 * pruned and estimation restarts cleanly from the new epoch; the last
 * EMA value is retained for STATUS.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NC_TS_RING         8
#define NC_TS_UNKNOWN      0x7FFF            /* no estimate yet          */
#define NC_TS_MIN_BASE_US  60000000ULL       /* 60 s min baseline        */
#define NC_TS_MAX_AGE_US   3600000000ULL     /* prune pairs > 1 h old    */
#define NC_TS_CLAMP_X10    32000             /* +-3200.0 ppm             */

typedef struct { uint64_t host_us, dev_us; } nc_ts_pair_t;

typedef struct {
    nc_ts_pair_t pair[NC_TS_RING]; /* compacted, index 0 = oldest */
    uint8_t count;
    bool    have_est;
    int32_t ema_ppm_x10;
} nc_ts_t;

void nc_ts_init(nc_ts_t *t);
void nc_ts_add_pair(nc_ts_t *t, uint64_t host_us, uint64_t dev_us);

/* NC_TS_UNKNOWN until two pairs >= 60 s apart (dev time) have been
 * seen; afterwards the latest EMA, clamped to +-32000. */
int16_t nc_ts_drift_ppm_x10(const nc_ts_t *t);
