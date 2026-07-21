/*
 * nc_wear.c — wear detector core. Pure C11; knobs via nc_knob_get.
 * Band semantics and the bp-power floor are documented in nc_wear.h.
 */
#include <string.h>
#include "narbis/nc_wear.h"
#include "narbis/nc_knobs.h"

void nc_wear_init(nc_wear_t *w)
{
    memset(w, 0, sizeof(*w));
}

bool nc_wear_is_worn(const nc_wear_t *w)
{
    return w->worn;
}

void nc_wear_tick_1hz(nc_wear_t *w, int32_t dc_ir, int64_t bp_power_1s)
{
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
    const bool bp_ok = bp_power_1s >= bp_floor;

    if (!w->worn) {
        bool good = (dc_ir > dark) && (dc_ir < enter_hi) && bp_ok;
        if (good) {
            if (++w->on_streak >= NC_WEAR_ON_TICKS) {
                w->worn = true;
                w->on_streak = 0;
                w->off_streak = 0;
            }
        } else {
            w->on_streak = 0;
        }
    } else {
        bool bad = !((dc_ir > dark) && (dc_ir < stay_hi) && bp_ok);
        if (bad) {
            if (++w->off_streak >= (uint32_t)nc_knob_get(KNOB_WEAR_OFF_S)) {
                w->worn = false;
                w->on_streak = 0;
                w->off_streak = 0;
            }
        } else {
            w->off_streak = 0;
        }
    }
}
