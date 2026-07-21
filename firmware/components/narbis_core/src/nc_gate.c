/*
 * nc_gate.c — artifact gate core + accel energy + DC-step detector.
 * Pure C11; knobs via nc_knob_get only.
 */
#include <string.h>
#include "narbis/nc_gate.h"
#include "narbis/nc_knobs.h"

void nc_gate_init(nc_gate_t *g)
{
    memset(g, 0, sizeof(*g));
}

bool nc_gate_update(nc_gate_t *g, const nc_gate_in_t *in,
                    uint32_t now_ms, uint8_t *reason_out)
{
    if (nc_knob_get(KNOB_GATE_EN) == 0) {
        /* Master off: forget any pending release so re-enabling does
         * not resurrect a stale gate span. */
        g->gated = false;
        g->have_active = false;
        g->latched_mask = 0;
        if (reason_out) *reason_out = 0;
        return false;
    }

    uint8_t m = 0;
    /* gate_acc_thr knob range is 1..1e6 — always positive, u32-safe. */
    if (in->acc_energy >= (uint32_t)nc_knob_get(KNOB_GATE_ACC_THR))
        m |= NC_GATE_REASON_ACCEL;
    if (in->sat)          m |= NC_GATE_REASON_SAT;
    if (in->dc_step)      m |= NC_GATE_REASON_DC_STEP;
    if (in->amp_collapse) m |= NC_GATE_REASON_COLLAPSE;
    if (in->agc_settling) m |= NC_GATE_REASON_AGC;
    if (in->wear_off)     m |= NC_GATE_REASON_WEAR;

    if (m != 0) {                       /* attack: immediate */
        g->gated = true;
        g->have_active = true;
        g->latched_mask = m;
        g->last_active_ms = now_ms;
        if (reason_out) *reason_out = m;
        return true;
    }

    /* Release tail: hold until now >= last_active + release_ms.
     * u32 subtraction is wrap-safe across the 49-day rollover. */
    if (g->have_active &&
        (uint32_t)(now_ms - g->last_active_ms) <
            (uint32_t)nc_knob_get(KNOB_GATE_RELEASE_MS)) {
        g->gated = true;
        if (reason_out) *reason_out = g->latched_mask;
        return true;
    }

    g->gated = false;
    if (reason_out) *reason_out = 0;
    return false;
}

/* ------------------------------------------------------------------ */
/* Accel window energy                                                 */
/* ------------------------------------------------------------------ */

void nc_gate_acc_init(nc_gate_acc_t *a, uint16_t odr_hz)
{
    memset(a, 0, sizeof(*a));
    uint32_t wlen = ((uint32_t)odr_hz *
                     (uint32_t)nc_knob_get(KNOB_GATE_ACC_WIN_MS)) / 1000u;
    if (wlen < 1) wlen = 1;
    if (wlen > NC_GATE_ACC_RING_MAX) wlen = NC_GATE_ACC_RING_MAX;
    a->wlen = (uint16_t)wlen;
}

uint32_t nc_gate_accel_feed(nc_gate_acc_t *a, int16_t x, int16_t y, int16_t z)
{
    int32_t s[3] = { x, y, z };

    if (!a->primed) {
        /* Seed the DC trackers so the standing 1 g / mounting offset
         * does not read as a giant step on the first window. */
        for (int i = 0; i < 3; i++) a->lp_q4[i] = s[i] * 16;
        a->primed = true;
    }

    /* Per-axis 1-pole HP, alpha = 2^-4, 4 fractional bits of DC state.
     * >> on negatives is arithmetic (floor) — same convention as the
     * DSP chain (nc_dsp.c). h fits easily: |x| <= 32767, |lp| tracks x,
     * so |h| < 2^17. */
    int64_t e = 0;
    for (int i = 0; i < 3; i++) {
        a->lp_q4[i] += (s[i] * 16 - a->lp_q4[i]) >> 4;
        int32_t h = s[i] - (a->lp_q4[i] >> 4);
        e += (int64_t)h * h;
    }
    uint32_t e32 = (e > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)e;

    if (a->count == a->wlen) {
        a->sum -= a->ring[a->widx];
    } else {
        a->count++;
    }
    a->ring[a->widx] = e32;
    a->sum += e32;
    a->widx++;
    if (a->widx >= a->wlen) a->widx = 0;

    return (a->sum > UINT32_MAX) ? UINT32_MAX : (uint32_t)a->sum;
}

/* ------------------------------------------------------------------ */
/* DC-step detector                                                    */
/* ------------------------------------------------------------------ */

void nc_gate_dcstep_init(nc_gate_dcstep_t *d)
{
    memset(d, 0, sizeof(*d));
}

bool nc_gate_dcstep_feed(nc_gate_dcstep_t *d, int32_t dc)
{
    d->nsamp++;
    if (d->nsamp == 1) {
        d->prev_dc = dc;
        return false;
    }

    /* |ddc| < 2^22 (DC spans ±2^21), so all Q4 EMA terms fit int32. */
    int32_t ddc = dc - d->prev_dc;
    d->prev_dc = dc;
    int32_t ad = (ddc < 0) ? -ddc : ddc;

    /* thr = ema * sigma, keeping the EMA's 4 fractional bits in the
     * product (x10 knob and Q4 scale fold into one /160): truncating
     * the EMA to integer counts first would bite exactly where it
     * matters, at small quiet-signal thresholds. */
    int64_t thr = ((int64_t)d->ema_abs_q4 *
                   nc_knob_get(KNOB_STEP_SIGMA_X10)) / 160;
    if (thr < NC_GATE_DCSTEP_MIN_THR) thr = NC_GATE_DCSTEP_MIN_THR;

    bool step = (d->nsamp > NC_GATE_DCSTEP_WARMUP) && ((int64_t)ad > thr);

    /* EMA tau = 32 deltas. Flagged outliers are excluded so a single
     * step does not inflate the threshold and mask a second one. */
    if (!step) {
        d->ema_abs_q4 += (ad * 16 - d->ema_abs_q4) >> 5;
    }
    return step;
}

void nc_gate_build_input(nc_gate_in_t *in, nc_gate_dcstep_t *ds,
                         uint32_t acc_energy, int32_t dc, bool sat,
                         bool amp_collapse, bool agc_settling,
                         bool wear_off)
{
    in->acc_energy = acc_energy;
    in->sat = sat;
    in->dc_step = nc_gate_dcstep_feed(ds, dc);
    in->amp_collapse = amp_collapse;
    in->agc_settling = agc_settling;
    in->wear_off = wear_off;
}
