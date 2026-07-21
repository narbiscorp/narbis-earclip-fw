/*
 * nc_agc.c — AGC policy engine. Pure C11; knobs via nc_knob_get only.
 * Policy is locked — see nc_agc.h for the ladder specification.
 */
#include "narbis/nc_agc.h"
#include "narbis/nc_knobs.h"

/* Ascending gain-value order of the AFE4404 TIA_GAIN RF codes
 * (SBAS689: 0=500k 1=250k 2=100k 3=50k 4=25k 5=10k 6=1M 7=2M). */
static const uint8_t rf_value_order[8] = { 5, 4, 3, 2, 1, 0, 6, 7 };

int nc_agc_rf_rank(uint8_t rf_code)
{
    for (int i = 0; i < 8; i++) {
        if (rf_value_order[i] == rf_code) return i;
    }
    return -1;
}

uint8_t nc_agc_rf_code_at(int rank)
{
    if (rank < 0) rank = 0;
    if (rank > 7) rank = 7;
    return rf_value_order[rank];
}

void nc_agc_init(nc_agc_t *s)
{
    s->last_step_ms = 0;
    s->stepped_once = false;
}

static int emit(nc_agc_act_t *out, int n, nc_agc_kind_t kind,
                uint8_t chan, uint8_t led_ma, int8_t offdac, uint8_t rf)
{
    out[n].kind = kind;
    out[n].chan = chan;
    out[n].led_ma = led_ma;
    out[n].offdac = offdac;
    out[n].rf_code = rf;
    return n + 1;
}

int nc_agc_evaluate(nc_agc_t *s, const nc_agc_in_t *in, nc_agc_act_t out[4])
{
    if (nc_knob_get(KNOB_AGC_EN) == 0 || in->frozen) return 0;

    const uint32_t hold = (uint32_t)nc_knob_get(KNOB_AGC_HOLD_MS);
    if (s->stepped_once && (in->now_ms - s->last_step_ms) < hold) return 0;

    /* Band edges in counts. pct knobs are <= 100, so the products stay
     * far below 2^31 even at full scale; int64 keeps it obviously safe. */
    const int32_t target = (int32_t)(((int64_t)NC_AGC_FS_COUNTS *
                                      nc_knob_get(KNOB_AGC_TARGET_PCT)) / 100);
    const int32_t dead = (int32_t)(((int64_t)NC_AGC_FS_COUNTS *
                                    nc_knob_get(KNOB_AGC_DEADBAND_PCT)) / 100);
    const int32_t lo = target - dead;
    const int32_t hi = target + dead;
    const int32_t step = nc_knob_get(KNOB_AGC_STEP_MA);
    const bool od_en = nc_knob_get(KNOB_AGC_OFFDAC_EN) != 0;

    /* Effective per-channel LED limits: knob range intersected with the
     * hard abs-max clamps. A min knob above the effective max (operator
     * error) degrades to min == max rather than an inverted range. */
    int32_t minma[2], maxma[2];
    minma[NC_AGC_CH_IR] = nc_knob_get(KNOB_AGC_MIN_MA_IR);
    maxma[NC_AGC_CH_IR] = nc_knob_get(KNOB_AGC_MAX_MA_IR);
    if (maxma[NC_AGC_CH_IR] > NC_AGC_HARD_MAX_IR_MA)
        maxma[NC_AGC_CH_IR] = NC_AGC_HARD_MAX_IR_MA;
    minma[NC_AGC_CH_RED] = nc_knob_get(KNOB_AGC_MIN_MA_RED);
    maxma[NC_AGC_CH_RED] = nc_knob_get(KNOB_AGC_MAX_MA_RED);
    if (maxma[NC_AGC_CH_RED] > NC_AGC_HARD_MAX_RED_MA)
        maxma[NC_AGC_CH_RED] = NC_AGC_HARD_MAX_RED_MA;
    for (int c = 0; c < 2; c++) {
        if (minma[c] > maxma[c]) minma[c] = maxma[c];
    }

    int n = 0;

    /* ---- rung 3 first: shared TIA gain --------------------------- */
    /* Gain rescales BOTH channels, so when it steps, per-channel LED /
     * offdac decisions computed under the old gain are stale — the
     * gain action is emitted alone (plus offdac recenters) and the
     * ladder resumes next hold window under the new gain. */
    bool dn_req = false, up_req = true;
    for (int c = 0; c < 2; c++) {
        bool at_min = in->led_ma[c] <= minma[c];
        bool at_max = in->led_ma[c] >= maxma[c];
        bool od_dn_exh = !od_en || in->offdac[c] <= NC_AGC_OFFDAC_MIN;
        bool od_up_exh = !od_en || in->offdac[c] >= NC_AGC_OFFDAC_MAX;
        if (in->sat[c] && at_min && od_dn_exh) dn_req = true;
        if (!(!in->sat[c] && in->dc_raw[c] < lo && at_max && od_up_exh))
            up_req = false;
    }
    const int rank = nc_agc_rf_rank(in->rf_code);
    int new_rank = rank;
    if (rank >= 0) {
        if (dn_req && rank > 0) new_rank = rank - 1;          /* either */
        else if (up_req && rank < 7) new_rank = rank + 1;     /* both   */
    }
    if (new_rank != rank) {
        n = emit(out, n, NC_AGC_GAIN, 0, 0, 0, nc_agc_rf_code_at(new_rank));
        for (int c = 0; c < 2; c++) {
            if (od_en && in->offdac[c] != 0) {
                n = emit(out, n, NC_AGC_OFFDAC, (uint8_t)c, 0, 0, 0);
            }
        }
        s->last_step_ms = in->now_ms;
        s->stepped_once = true;
        return n;
    }

    /* ---- rungs 1+2: per-channel LED, then offset DAC ------------- */
    for (int c = 0; c < 2; c++) {
        /* A saturated channel's dc_raw is only a lower bound: always
         * treat saturation as "too high" regardless of the reading. */
        bool high = in->sat[c] || in->dc_raw[c] > hi;
        bool low = !in->sat[c] && in->dc_raw[c] < lo;

        if (high) {
            if (in->led_ma[c] > minma[c]) {
                int32_t nl = (int32_t)in->led_ma[c] - step;
                nl = nc_clamp_i32(nl, minma[c], maxma[c]);
                n = emit(out, n, NC_AGC_LED, (uint8_t)c, (uint8_t)nl, 0, 0);
            } else if (od_en && in->offdac[c] > NC_AGC_OFFDAC_MIN) {
                n = emit(out, n, NC_AGC_OFFDAC, (uint8_t)c, 0,
                         (int8_t)(in->offdac[c] - 1), 0);
            }
        } else if (low) {
            if (in->led_ma[c] < maxma[c]) {
                int32_t nl = (int32_t)in->led_ma[c] + step;
                nl = nc_clamp_i32(nl, minma[c], maxma[c]);
                n = emit(out, n, NC_AGC_LED, (uint8_t)c, (uint8_t)nl, 0, 0);
            } else if (od_en && in->offdac[c] < NC_AGC_OFFDAC_MAX) {
                n = emit(out, n, NC_AGC_OFFDAC, (uint8_t)c, 0,
                         (int8_t)(in->offdac[c] + 1), 0);
            }
        }
    }

    if (n > 0) {
        s->last_step_ms = in->now_ms;
        s->stepped_once = true;
    }
    return n;
}
