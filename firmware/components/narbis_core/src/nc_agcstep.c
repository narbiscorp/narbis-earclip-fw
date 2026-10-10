/*
 * nc_agcstep.c — AGC step compensation. See nc_agcstep.h.
 */
#include <string.h>
#include "narbis/nc_agcstep.h"
#include "narbis/nc_types.h"

static bool scalable(int32_t r_q16)
{
    return r_q16 >= NC_AGCSTEP_R_MIN && r_q16 <= NC_AGCSTEP_R_MAX;
}

void nc_agcstep_init(nc_agcstep_t *st)
{
    memset(st, 0, sizeof(*st));
}

int32_t nc_agcstep_ratio_q16(uint32_t num, uint32_t den)
{
    if (num == 0 || den == 0) {
        return 0;
    }
    const uint64_t r = ((uint64_t)num << 16) / den;
    return (r >= NC_AGCSTEP_R_MIN && r <= NC_AGCSTEP_R_MAX) ? (int32_t)r : 0;
}

int32_t nc_agcstep_chain_q16(int32_t a_q16, int32_t b_q16)
{
    if (a_q16 <= 0 || b_q16 <= 0) {
        return 0;
    }
    const int64_t r = ((int64_t)a_q16 * b_q16) >> 16;
    return (r >= NC_AGCSTEP_R_MIN && r <= NC_AGCSTEP_R_MAX) ? (int32_t)r : 0;
}

void nc_agcstep_guard(nc_agcstep_t *st, uint64_t t_lo_us)
{
    if (!st->guard) {
        st->guard = true;
        st->guard_t_lo_us = t_lo_us;
    }
}

void nc_agcstep_arm(nc_agcstep_t *st, uint64_t t_lo_us, uint64_t t_hi_us,
                    const int32_t r_q16[2])
{
    if (st->guard && st->guard_t_lo_us < t_lo_us) {
        t_lo_us = st->guard_t_lo_us;
    }
    st->guard = false;
    if (st->armed) {
        if (t_lo_us < st->t_lo_us) st->t_lo_us = t_lo_us;
        if (t_hi_us > st->t_hi_us) st->t_hi_us = t_hi_us;
        for (int c = 0; c < 2; c++) {
            st->r_q16[c] = nc_agcstep_chain_q16(st->r_q16[c], r_q16[c]);
        }
    } else {
        st->armed = true;
        st->t_lo_us = t_lo_us;
        st->t_hi_us = t_hi_us;
        st->r_q16[0] = r_q16[0];
        st->r_q16[1] = r_q16[1];
    }
    st->seen_after = false;
}

/* Straight-line continuation of the last two values (one: repeat it). */
static int32_t extrapolate(const nc_agcstep_t *st, int32_t v1, int32_t v2)
{
    return (st->hist >= 2) ? nc_sat32(2LL * v1 - v2) : v1;
}

nc_agcstep_act_t nc_agcstep_run(nc_agcstep_t *st, nc_chan_dsp_t ch[2],
                                nc_ibi_t *ibi, int sel, nc_gate_dcstep_t *ds,
                                uint64_t t_us, const int32_t x[2],
                                nc_dsp_out_t o[2])
{
    nc_agcstep_act_t act = NC_AGCSTEP_PASS;
    if (st->armed && t_us > st->t_lo_us) {
        if (t_us <= st->t_hi_us || !st->seen_after) {
            if (t_us > st->t_hi_us) {
                st->seen_after = true;
            }
            act = NC_AGCSTEP_HOLD;
        } else {
            act = NC_AGCSTEP_APPLY;
        }
    } else if (st->guard && t_us > st->guard_t_lo_us) {
        act = NC_AGCSTEP_HOLD;
    }
    if (act == NC_AGCSTEP_HOLD && (st->held >= NC_AGCSTEP_HOLD_MAX || st->hist == 0)) {
        /* Capped (an arm that never came, or a window far too wide), or
         * no history to continue from: switch now. A guard without an arm
         * has no ratios — drop it and let the sample through. */
        act = st->armed ? NC_AGCSTEP_APPLY : NC_AGCSTEP_PASS;
        st->guard = false;
    }

    int32_t in[2] = { x[0], x[1] };
    if (act == NC_AGCSTEP_HOLD) {
        for (int c = 0; c < 2; c++) {
            in[c] = extrapolate(st, st->x1[c], st->x2[c]);
        }
    } else if (act == NC_AGCSTEP_APPLY) {
        for (int c = 0; c < 2; c++) {
            const int32_t r = st->r_q16[c];
            if (scalable(r) && st->hist > 0) {
                nc_dsp_scale(&ch[c], r);
                const int32_t hp_pred = extrapolate(st, st->hp1[c], st->hp2[c]);
                nc_dsp_rebase(&ch[c], in[c], nc_sat32(nc_mul_q16(hp_pred, r)));
                if (c == sel && ibi != NULL) {
                    nc_ibi_scale(ibi, r);
                }
            } else {
                nc_dsp_reprime(&ch[c], in[c]);
                if (c == sel && ibi != NULL) {
                    nc_ibi_restart_amp(ibi);
                }
            }
        }
    }

    for (int c = 0; c < 2; c++) {
        nc_dsp_run(&ch[c], in[c], &o[c]);
    }

    if (act == NC_AGCSTEP_APPLY) {
        if (ds != NULL) {
            const int32_t r = st->r_q16[sel];
            nc_gate_dcstep_rebase(ds, o[sel].dc, scalable(r) ? r : NC_AGCSTEP_ONE);
        }
        st->armed = false;
        st->seen_after = false;
        st->hist = 0;                 /* old-scale history is meaningless now */
    }
    for (int c = 0; c < 2; c++) {
        st->x2[c] = st->x1[c];
        st->x1[c] = in[c];
        st->hp2[c] = st->hp1[c];
        st->hp1[c] = o[c].hp;
    }
    if (st->hist < 2) {
        st->hist++;
    }
    st->held = (act == NC_AGCSTEP_HOLD) ? (uint8_t)(st->held + 1) : 0;
    return act;
}
