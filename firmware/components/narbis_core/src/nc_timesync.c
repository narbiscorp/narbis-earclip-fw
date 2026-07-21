/*
 * nc_timesync.c — clock drift EMA from TIME_SYNC pairs.
 * Contract + overflow analysis in nc_timesync.h. Slow path only
 * (one call per host sync exchange): memmove compaction is fine.
 */
#include "narbis/nc_timesync.h"
#include <string.h>

void nc_ts_init(nc_ts_t *t)
{
    memset(t, 0, sizeof *t);
}

void nc_ts_add_pair(nc_ts_t *t, uint64_t host_us, uint64_t dev_us)
{
    if (t->count == NC_TS_RING) {
        memmove(&t->pair[0], &t->pair[1],
                (NC_TS_RING - 1) * sizeof t->pair[0]);
        t->count--;
    }
    t->pair[t->count].host_us = host_us;
    t->pair[t->count].dev_us = dev_us;
    t->count++;

    /* Prune pairs > 1 h older than the newest (bounds dDev, and a
     * backward dev clock — reboot — underflows huge and flushes all
     * stale pairs, restarting estimation from the new epoch). */
    uint8_t drop = 0;
    while (drop < (uint8_t)(t->count - 1) &&
           dev_us - t->pair[drop].dev_us > NC_TS_MAX_AGE_US) {
        drop++;
    }
    if (drop) {
        memmove(&t->pair[0], &t->pair[drop],
                (size_t)(t->count - drop) * sizeof t->pair[0]);
        t->count = (uint8_t)(t->count - drop);
    }

    if (t->count < 2) return;

    const nc_ts_pair_t *o = &t->pair[0];
    uint64_t ddev = dev_us - o->dev_us;   /* o->dev_us <= dev_us after prune */
    if (ddev < NC_TS_MIN_BASE_US) return; /* baseline too short to trust */

    /* diff = dHost - dDev; uint64 subtraction then cast handles a host
     * clock stepped backward (NTP). Clamp to +-2^31 pre-multiply so
     * diff * 1e7 <= 2.15e16 << INT64_MAX. */
    int64_t diff = (int64_t)((host_us - o->host_us) - ddev);
    if (diff > (int64_t)1 << 31) diff = (int64_t)1 << 31;
    if (diff < -((int64_t)1 << 31)) diff = -((int64_t)1 << 31);

    int64_t inst = (diff * 10000000LL) / (int64_t)ddev;
    if (inst > NC_TS_CLAMP_X10) inst = NC_TS_CLAMP_X10;
    if (inst < -NC_TS_CLAMP_X10) inst = -NC_TS_CLAMP_X10;

    if (!t->have_est) {
        t->ema_ppm_x10 = (int32_t)inst;   /* seed EMA at first estimate */
        t->have_est = true;
    } else {
        /* alpha = 0.25; truncation toward zero keeps |ema| <= 32000 */
        t->ema_ppm_x10 += ((int32_t)inst - t->ema_ppm_x10) / 4;
    }
}

int16_t nc_ts_drift_ppm_x10(const nc_ts_t *t)
{
    return t->have_est ? (int16_t)t->ema_ppm_x10 : (int16_t)NC_TS_UNKNOWN;
}
