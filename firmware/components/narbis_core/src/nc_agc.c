/*
 * nc_agc.c — AGC policy engine. Pure C11; knobs via nc_knob_get only.
 * Policy is locked — see nc_agc.h for the ladder specification and the
 * acquire -> lock state machine wrapped around it.
 */
#include "narbis/nc_agc.h"
#include "narbis/nc_knobs.h"

/* Ascending gain-value order of the AFE4404 TIA_GAIN RF codes
 * (SBAS689: 0=500k 1=250k 2=100k 3=50k 4=25k 5=10k 6=1M 7=2M). */
static const uint8_t rf_value_order[8] = { 5, 4, 3, 2, 1, 0, 6, 7 };

/* IBI flags that disqualify a beat from counting toward a lock.
 * NOT INTERPOLATED: nc_ibi sets it whenever the parabolic sub-sample
 * peak refinement succeeds, i.e. on nearly every good beat. Counting it
 * as unclean meant real beats could never lock, nor hold off a drift
 * re-acquire (first hardware run, 2026-10-09). */
#define AGC_UNCLEAN_IBIF (NC_IBIF_GATED_CTX | NC_IBIF_AGC_SETTLING | \
                          NC_IBIF_FIRST_AFTER_GAP)

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

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */
static void enter(nc_agc_t *s, uint8_t state, uint8_t reason)
{
    s->state = state;
    s->reason = reason;
    s->changed = true;
    s->clean_beats = 0;
    s->quiet = false;
    s->sat_run = false;
    s->drift_run = false;
}

static void to_acquire(nc_agc_t *s, uint8_t reason)
{
    if (s->state == NC_AGC_ST_LOCKED && s->reacq_count < 255) {
        s->reacq_count++;
    }
    enter(s, NC_AGC_ST_ACQUIRE, reason);
}

static void to_locked(nc_agc_t *s, uint8_t reason, const nc_agc_in_t *in)
{
    enter(s, NC_AGC_ST_LOCKED, reason);
    s->locked_ms = in->now_ms;
    s->locked_this_stream = true;
    if (in->off_ear) {
        return;   /* not a wearer's settings: no restore record */
    }
    /* The settings this wearer locked at: the put-back restore target. */
    s->lock_cfg_valid = true;
    s->lock_rf_code = in->rf_code;
    for (int c = 0; c < 2; c++) {
        s->lock_led_ma[c] = in->led_ma[c];
        s->lock_offdac[c] = in->offdac[c];
    }
}

void nc_agc_init(nc_agc_t *s)
{
    s->lock_cfg_valid = false;
    nc_agc_restart(s);
}

void nc_agc_restart(nc_agc_t *s)
{
    s->last_step_ms = 0;
    s->stepped_once = false;
    s->reacq_count = 0;
    s->beat_seen = false;
    s->last_beat_ms = 0;
    s->quiet_since_ms = s->sat_since_ms = s->drift_since_ms = 0;
    s->locked_ms = 0;
    s->locked_this_stream = false;
    s->offear_run = false;
    s->offear_since_ms = 0;
    s->restore_phase = 0;
    /* lock_cfg_* kept: see RESTORE in nc_agc.h. */
    enter(s, NC_AGC_ST_ACQUIRE, NC_AGC_RSN_START);
}

void nc_agc_on_beat(nc_agc_t *s, uint16_t ibi_ms, uint8_t ibi_flags,
                    uint64_t now_ms)
{
    if (ibi_ms > 0 && (ibi_flags & AGC_UNCLEAN_IBIF) == 0) {
        if (s->clean_beats < 255) s->clean_beats++;
        s->last_beat_ms = now_ms;
        s->beat_seen = true;
    } else {
        s->clean_beats = 0;
    }
}

void nc_agc_reacquire(nc_agc_t *s, uint8_t reason)
{
    if (s->state == NC_AGC_ST_LOCKED) {
        to_acquire(s, reason);
    } else if (s->state == NC_AGC_ST_ACQUIRE) {
        s->clean_beats = 0;          /* fresh start, no edge */
        s->quiet = false;
    } else {
        return;   /* OFF: frozen or disabled wins; the unfreeze re-acquires. */
    }
    if (reason == NC_AGC_RSN_WEAR && s->lock_cfg_valid) {
        s->restore_phase = 1;        /* put back on: start from the lock */
    }
}

bool nc_agc_take_change(nc_agc_t *s)
{
    bool c = s->changed;
    s->changed = false;
    return c;
}

/* ------------------------------------------------------------------ */
/* The ladder (unchanged policy; see nc_agc.h)                         */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t lo, hi;              /* target ± deadband                 */
    int32_t lock_lo, lock_hi;    /* target ± agc_lock_band_pct         */
} agc_band_t;

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

/* Effective per-channel LED limits: knob range intersected with the
 * hard abs-max clamps. A min knob above the effective max (operator
 * error) degrades to min == max rather than an inverted range. */
static void led_limits(int32_t minma[2], int32_t maxma[2])
{
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
}

/* probe: PROBE (nc_agc.h) — LEDs only, capped at agc_probe_max_ma. */
static int ladder(nc_agc_t *s, const nc_agc_in_t *in, const agc_band_t *b,
                  nc_agc_act_t out[4], bool probe)
{
    const int32_t lo = b->lo;
    const int32_t hi = b->hi;
    const int32_t step = nc_knob_get(KNOB_AGC_STEP_MA);
    const bool od_en = !probe && nc_knob_get(KNOB_AGC_OFFDAC_EN) != 0;

    int32_t minma[2], maxma[2];
    led_limits(minma, maxma);
    if (probe) {
        const int32_t cap = nc_knob_get(KNOB_AGC_PROBE_MAX_MA);
        for (int c = 0; c < 2; c++) {
            if (maxma[c] > cap) maxma[c] = cap;
            if (minma[c] > maxma[c]) minma[c] = maxma[c];
        }
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
    if (rank >= 0 && !probe) {
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

/* RESTORE (nc_agc.h), in place of a ladder step. Phase 1 sets gain +
 * LEDs, phase 2 (next hold window, so it lands under the restored gain)
 * the offdacs; a phase with nothing to change falls through, and once
 * done the ladder takes over in the same call. */
static int restore(nc_agc_t *s, const nc_agc_in_t *in, const agc_band_t *b,
                   nc_agc_act_t out[4])
{
    int n = 0;
    if (s->restore_phase == 1) {
        s->restore_phase = 2;
        int32_t minma[2], maxma[2];
        led_limits(minma, maxma);
        if (s->lock_rf_code != in->rf_code) {
            n = emit(out, n, NC_AGC_GAIN, 0, 0, 0, s->lock_rf_code);
        }
        for (int c = 0; c < 2; c++) {
            const uint8_t l = (uint8_t)nc_clamp_i32(s->lock_led_ma[c],
                                                    minma[c], maxma[c]);
            if (l != in->led_ma[c]) {
                n = emit(out, n, NC_AGC_LED, (uint8_t)c, l, 0, 0);
            }
        }
        if (n > 0) {
            s->last_step_ms = in->now_ms;
            s->stepped_once = true;
            return n;
        }
    }
    s->restore_phase = 0;
    if (nc_knob_get(KNOB_AGC_OFFDAC_EN) != 0) {
        for (int c = 0; c < 2; c++) {
            if (s->lock_offdac[c] != in->offdac[c]) {
                n = emit(out, n, NC_AGC_OFFDAC, (uint8_t)c, 0,
                         s->lock_offdac[c], 0);
            }
        }
    }
    if (n > 0) {
        s->last_step_ms = in->now_ms;
        s->stepped_once = true;
        return n;
    }
    return ladder(s, in, b, out, false);
}

/* ------------------------------------------------------------------ */
/* Acquire -> lock                                                     */
/* ------------------------------------------------------------------ */
static bool both_within(const nc_agc_in_t *in, int32_t lo, int32_t hi)
{
    for (int c = 0; c < 2; c++) {
        if (in->sat[c] || in->dc_raw[c] < lo || in->dc_raw[c] > hi) return false;
    }
    return true;
}

/* LOCKED: track the clipping / drift runs and decide whether the signal
 * has been unusable long enough to be worth a step. */
static bool locked_wants_reacquire(nc_agc_t *s, const nc_agc_in_t *in,
                                   const agc_band_t *b)
{
    const uint64_t now = in->now_ms;

    if (in->sat[0] || in->sat[1]) {
        if (!s->sat_run) {
            s->sat_run = true;
            s->sat_since_ms = now;
        }
    } else {
        s->sat_run = false;
    }

    bool out_of_band = false;
    for (int c = 0; c < 2; c++) {
        if (!in->sat[c] &&
            (in->dc_raw[c] < b->lock_lo || in->dc_raw[c] > b->lock_hi)) {
            out_of_band = true;
        }
    }
    if (out_of_band) {
        if (!s->drift_run) {
            s->drift_run = true;
            s->drift_since_ms = now;
        }
    } else {
        s->drift_run = false;
    }

    const uint64_t min_ms = (uint64_t)nc_knob_get(KNOB_AGC_REACQ_MIN_S) * 1000u;
    if (now - s->locked_ms < min_ms) {
        return false;
    }
    if (s->sat_run &&
        now - s->sat_since_ms >= (uint64_t)nc_knob_get(KNOB_AGC_REACQ_SAT_MS)) {
        to_acquire(s, NC_AGC_RSN_SAT);
        return true;
    }
    const uint64_t drift_ms = (uint64_t)nc_knob_get(KNOB_AGC_REACQ_DRIFT_MS);
    if (s->drift_run && now - s->drift_since_ms >= drift_ms) {
        /* Beats still arriving means the signal is doing its job: a
         * level outside the band is not worth a step while it does. */
        const bool beating = s->beat_seen && now - s->last_beat_ms < drift_ms;
        if (!beating) {
            to_acquire(s, NC_AGC_RSN_DRIFT);
            return true;
        }
    }
    return false;
}

/* ACQUIRE: after this evaluation's ladder decision, decide whether the
 * level is good enough to stop touching it. `ran` is false when the
 * hold window kept the ladder from looking at all. */
static void acquire_progress(nc_agc_t *s, const nc_agc_in_t *in,
                             const agc_band_t *b, int n, bool ran)
{
    if (n > 0) {
        s->clean_beats = 0;          /* plant changed: start counting again */
        s->quiet = false;
        return;
    }
    if (s->clean_beats >= (uint8_t)nc_knob_get(KNOB_AGC_LOCK_BEATS) &&
        both_within(in, b->lock_lo, b->lock_hi)) {
        to_locked(s, NC_AGC_RSN_BEATS, in);
        return;
    }
    if (!ran) {
        return;
    }
    if (both_within(in, b->lo, b->hi)) {
        if (!s->quiet) {
            s->quiet = true;
            s->quiet_since_ms = in->now_ms;
        } else if (in->now_ms - s->quiet_since_ms >=
                   (uint64_t)nc_knob_get(KNOB_AGC_LOCK_STABLE_MS)) {
            to_locked(s, NC_AGC_RSN_STABLE, in);
        }
    } else {
        s->quiet = false;
    }
}

int nc_agc_evaluate(nc_agc_t *s, const nc_agc_in_t *in, nc_agc_act_t out[4])
{
    if (nc_knob_get(KNOB_AGC_EN) == 0 || in->frozen) {
        if (s->state != NC_AGC_ST_OFF) {
            enter(s, NC_AGC_ST_OFF, NC_AGC_RSN_MANUAL);
        }
        return 0;
    }
    if (s->state == NC_AGC_ST_OFF) {
        to_acquire(s, NC_AGC_RSN_MANUAL);   /* unfrozen / re-enabled */
    }

    const bool lock_mode = nc_knob_get(KNOB_AGC_MODE) != 0;
    if (!lock_mode && s->state == NC_AGC_ST_LOCKED) {
        to_acquire(s, NC_AGC_RSN_MODE);     /* legacy ladder resumes */
    }

    /* Band edges in counts. pct knobs are <= 100, so the products stay
     * far below 2^31 even at full scale; int64 keeps it obviously safe. */
    const int32_t target = (int32_t)(((int64_t)NC_AGC_FS_COUNTS *
                                      nc_knob_get(KNOB_AGC_TARGET_PCT)) / 100);
    const int32_t dead = (int32_t)(((int64_t)NC_AGC_FS_COUNTS *
                                    nc_knob_get(KNOB_AGC_DEADBAND_PCT)) / 100);
    const int32_t lock = (int32_t)(((int64_t)NC_AGC_FS_COUNTS *
                                    nc_knob_get(KNOB_AGC_LOCK_BAND_PCT)) / 100);
    const agc_band_t b = {
        .lo = target - dead, .hi = target + dead,
        .lock_lo = target - lock, .lock_hi = target + lock,
    };

    /* Off-ear HOLD (nc_agc.h): after a lock in this stream, an off-ear
     * clip is left exactly as it was, for at most agc_offear_hold_s. The
     * LOCKED runs restart afterwards so the hold cannot count toward a
     * SAT / DRIFT re-acquire. */
    if (in->off_ear) {
        if (!s->offear_run) {
            s->offear_run = true;
            s->offear_since_ms = in->now_ms;
        }
    } else {
        s->offear_run = false;
    }
    const uint64_t hold_ms = (uint64_t)nc_knob_get(KNOB_AGC_OFFEAR_HOLD_S) * 1000u;
    const bool hold_over = s->offear_run && in->now_ms - s->offear_since_ms >= hold_ms;
    if (lock_mode && s->locked_this_stream && s->offear_run && !hold_over) {
        s->sat_run = false;
        s->drift_run = false;
        return 0;
    }

    if (s->state == NC_AGC_ST_LOCKED && !locked_wants_reacquire(s, in, &b)) {
        return 0;                           /* locked: nothing moves */
    }

    const uint32_t hold = (uint32_t)nc_knob_get(KNOB_AGC_HOLD_MS);
    const bool ran = !s->stepped_once || (in->now_ms - s->last_step_ms) >= hold;
    /* PROBE (nc_agc.h): off-ear before this stream's first lock, or once
     * the off-ear hold is over. Never the full ladder on an empty clip. */
    const bool probe = lock_mode && in->off_ear && (!s->locked_this_stream || hold_over);
    int n = 0;
    if (ran) {
        n = s->restore_phase ? restore(s, in, &b, out)
                             : ladder(s, in, &b, out, probe);
    }

    if (lock_mode && s->state == NC_AGC_ST_ACQUIRE) {
        if (probe) {
            s->clean_beats = 0;   /* nothing locks on an off-ear clip */
            s->quiet = false;
        } else {
            acquire_progress(s, in, &b, n, ran);
        }
    }
    return n;
}
