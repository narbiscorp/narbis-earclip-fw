/*
 * nc_wear.c — wear detector core. Pure C11; knobs via nc_knob_get.
 * Band semantics, the bp-power floor and the rail / jolt fast-off
 * paths are documented in nc_wear.h.
 */
#include <string.h>
#include "narbis/nc_wear.h"
#include "narbis/nc_knobs.h"

void nc_wear_init(nc_wear_t *w)
{
    memset(w, 0, sizeof(*w));
    w->jolt_age_s = UINT16_MAX;   /* no recent jolt */
}

bool nc_wear_is_worn(const nc_wear_t *w)
{
    return w->worn;
}

static void drop(nc_wear_t *w)
{
    w->worn = false;
    w->on_streak = 0;
    w->off_streak = 0;
    w->rail_streak = 0;
}

void nc_wear_tick(nc_wear_t *w, const nc_wear_in_t *in)
{
    const int32_t dc_ir = in->dc_ir;
    const int32_t dark = nc_knob_get(KNOB_WEAR_DARK_THR);
    const int32_t on_thr = nc_knob_get(KNOB_WEAR_ON_THR);
    const int32_t off_thr = nc_knob_get(KNOB_WEAR_OFF_THR);

    /* Defensive against inverted knob settings: entry band must never
     * be wider than the stay band or hysteresis flips into a chatter
     * generator. */
    const int32_t enter_hi = (off_thr < on_thr) ? off_thr : on_thr;
    const int32_t stay_hi = (on_thr > off_thr) ? on_thr : off_thr;

    /* ibi_thr_min knob max is 2^22, so 64 * thr^2 < 2^50: no overflow. */
    const int64_t thr_min = nc_knob_get(KNOB_IBI_THR_MIN);
    const int64_t bp_floor = 64 * thr_min * thr_min;
    const bool bp_ok = in->bp_power_1s >= bp_floor;

    if (in->jolt) {
        w->jolt_age_s = 0;
    } else if (w->jolt_age_s < UINT16_MAX) {
        w->jolt_age_s++;
    }

    if (!w->worn) {
        bool good = !in->rail && (dc_ir > dark) && (dc_ir < enter_hi) && bp_ok;
        if (good) {
            if (++w->on_streak >= NC_WEAR_ON_TICKS) {
                w->worn = true;
                w->on_streak = 0;
                w->off_streak = 0;
                w->rail_streak = 0;
            }
        } else {
            w->on_streak = 0;
        }
        return;
    }

    if (in->rail) {
        if (++w->rail_streak >= (uint8_t)nc_knob_get(KNOB_WEAR_RAIL_S)) {
            drop(w);
            return;
        }
    } else {
        w->rail_streak = 0;
    }

    bool bad = in->rail || !((dc_ir > dark) && (dc_ir < stay_hi) && bp_ok);
    if (bad) {
        uint32_t limit = (uint32_t)nc_knob_get(KNOB_WEAR_OFF_S);
        if (w->jolt_age_s <= (uint32_t)nc_knob_get(KNOB_WEAR_JOLT_WIN_S)) {
            const uint32_t j = (uint32_t)nc_knob_get(KNOB_WEAR_JOLT_OFF_S);
            if (j < limit) limit = j;
        }
        if (++w->off_streak >= limit) {
            drop(w);
        }
    } else {
        w->off_streak = 0;
    }
}

void nc_wear_tick_1hz(nc_wear_t *w, int32_t dc_ir, int64_t bp_power_1s)
{
    const nc_wear_in_t in = { .dc_ir = dc_ir, .bp_power_1s = bp_power_1s,
                              .rail = false, .jolt = false };
    nc_wear_tick(w, &in);
}
