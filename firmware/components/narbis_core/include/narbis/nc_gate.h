/*
 * nc_gate.h — artifact gate (handoff §5.9). Pure C11.
 *
 * The gate is annotation + IBI suppression only: raw samples are never
 * dropped. Attack is immediate (any active reason gates the same call);
 * release is a countdown — after the last active reason the gate holds
 * for gate_release_ms and drops exactly when
 *     now_ms >= last_active_ms + gate_release_ms.
 * reason_mask uses NC_GATE_REASON_* (nc_types.h). While riding the
 * release tail the mask reported is the one latched from the last
 * active call; when ungated it is 0. Master gate_en knob off forces
 * ungated and clears state.
 *
 * Also owned here:
 *  - accel window-energy helper (per-axis 1-pole HP, shift-4 alpha, at
 *    the accel ODR; ring sum of hx^2+hy^2+hz^2 over gate_acc_win_ms);
 *  - DC-step detector: |ddc| > (step_sigma_x10/10) * EMA(|ddc|), where
 *    the EMA of |ddc| is a mean-abs-deviation proxy for sigma;
 *  - nc_gate_build_input, gluing raw signals into nc_gate_in_t.
 * The acc_energy >= gate_acc_thr comparison happens in nc_gate_update
 * (the input carries the raw window energy).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "narbis/nc_types.h"

typedef struct {
    uint32_t acc_energy;    /* from nc_gate_accel_feed */
    bool     sat;           /* ADC near-rail (> sat_pct, caller-side)  */
    bool     dc_step;       /* from nc_gate_dcstep_feed                */
    bool     amp_collapse;  /* beat-amplitude collapse (IBI side)      */
    bool     agc_settling;  /* inside agc_settle_ms blanking           */
    bool     wear_off;      /* !nc_wear_is_worn()                      */
} nc_gate_in_t;

typedef struct {
    bool     gated;
    bool     have_active;    /* last_active_ms/latched_mask valid      */
    uint8_t  latched_mask;
    uint32_t last_active_ms;
} nc_gate_t;

void nc_gate_init(nc_gate_t *g);

/* Returns gated; *reason_out (NULL ok) gets the active reason mask,
 * the latched mask during the release tail, 0 when ungated. */
bool nc_gate_update(nc_gate_t *g, const nc_gate_in_t *in,
                    uint32_t now_ms, uint8_t *reason_out);

/* ------------------------------------------------------------------ */
/* Accel window energy                                                 */
/* ------------------------------------------------------------------ */
/* Ring sized for the worst knob corner: 400 Hz ODR x 1000 ms window
 * = 400 samples; 512 leaves headroom and is the clamp limit. */
#define NC_GATE_ACC_RING_MAX 512

typedef struct {
    int32_t  lp_q4[3];   /* per-axis DC tracker, 4 fractional bits */
    bool     primed;     /* lp seeded from the first sample (else the
                            power-on DC offset reads as a huge step) */
    uint16_t wlen, widx, count;
    uint64_t sum;        /* running window sum of ring[]            */
    uint32_t ring[NC_GATE_ACC_RING_MAX]; /* per-sample energy, u32-sat */
} nc_gate_acc_t;

/* Window length = odr_hz * gate_acc_win_ms / 1000, clamped to
 * [1, NC_GATE_ACC_RING_MAX]. Re-init after ODR or window knob changes. */
void nc_gate_acc_init(nc_gate_acc_t *a, uint16_t odr_hz);

/* Feed one accel sample (raw counts); returns the HP energy summed
 * over the window, saturated to u32. Compare against gate_acc_thr. */
uint32_t nc_gate_accel_feed(nc_gate_acc_t *a, int16_t x, int16_t y, int16_t z);

/* ------------------------------------------------------------------ */
/* DC-step detector                                                    */
/* ------------------------------------------------------------------ */
/* No step is ever flagged for the first NC_GATE_DCSTEP_WARMUP samples
 * (the EMA has no meaningful baseline yet). */
#define NC_GATE_DCSTEP_WARMUP 50

/* Threshold floor in counts: a perfectly flat DC drives EMA(|ddc|) to
 * zero and any 1-count wiggle would then flag; sub-8-count deltas are
 * below ADC noise significance at 2^21 FS. Heuristic, knob-free. */
#define NC_GATE_DCSTEP_MIN_THR 8

typedef struct {
    int32_t  prev_dc;
    int32_t  ema_abs_q4;   /* EMA of |ddc|, 4 fractional bits */
    uint32_t nsamp;
} nc_gate_dcstep_t;

void nc_gate_dcstep_init(nc_gate_dcstep_t *d);

/* Feed the per-sample DC-tracker output; returns true on a step. */
bool nc_gate_dcstep_feed(nc_gate_dcstep_t *d, int32_t dc);

/* Assemble nc_gate_in_t from the raw signals, running the DC-step
 * detector on `dc` as part of it. */
void nc_gate_build_input(nc_gate_in_t *in, nc_gate_dcstep_t *ds,
                         uint32_t acc_energy, int32_t dc, bool sat,
                         bool amp_collapse, bool agc_settling,
                         bool wear_off);
